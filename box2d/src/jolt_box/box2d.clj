(ns jolt-box.box2d
  "Box2D v3 for jolt. Require as b2.

  A world is a handle made by make-world and released by destroy! (or
  with-world). Bodies are plain uint64 ids. Give a body a keyword name and
  you can use the name anywhere an id goes; re-evaluating a named body!
  replaces the old body, so setup forms can be re-run at the REPL.

      (b2/with-world [w {:gravity [0 9.8]}]
        (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})
        (b2/body! w :crate {:type :dynamic :shapes [{:box [0.5 0.5]}]})
        (b2/simulate w 2.0 {:sample [:crate] :every 0.25}))

  Units are meters, kilograms, seconds and radians. The default gravity
  [0 9.8] is y-down (screen style, like raylib); pass [0 -9.8] for y-up.

  Everything returned is plain data (realized vectors and maps), so it can
  cross from the sim fiber to the main thread. A world belongs to the fiber
  that made it: see guide/implementation-guide.md §5.4."
  (:require [jolt.ffi :as ffi]
            [jolt-box.common :as c]
            [jolt-box.box2d.ffi :as f]))

(def STEP
  "The fixed timestep step-fixed! and simulate use, in seconds."
  (/ 1.0 60.0))

(def SUBSTEPS
  "Box2D substeps per step. 4 is Box2D's recommended value."
  4)

;; --- world handle ------------------------------------------------------------

(defrecord World [id gravity alive registry scratch acc max-steps collect-events? events
                  sensor-events])

(def ^:private EVENT-CAP
  "The most events of one kind read back from a single step."
  512)

(def ^:private MAX-PENDING
  "The most events of one kind kept waiting for contact-events; beyond this
  they're counted in :dropped instead."
  4096)

(def ^:private no-events {:begin [] :end [] :hit [] :dropped 0})
(def ^:private no-sensor-events {:begin [] :end [] :dropped 0})

(defmethod print-method World [w ^java.io.Writer out]
  (.write out (if @(:alive w)
                (str "#b2/world{:id " (:id w)
                     " :bodies " (count (get-in @(:registry w) [:entries :body])) "}")
                (str "#b2/world{:id " (:id w) " :destroyed true}"))))

(defn- live!
  "Throws unless world w is still alive."
  [w]
  (when-not (instance? World w)
    (throw (ex-info "not a Box2D world handle" {:value w})))
  (when-not @(:alive w)
    (throw (ex-info "world destroyed" {:world (:id w)}))))

(defn- check!
  "Throws ex-info when a shim return code isn't ok."
  [rc op data]
  (when-not (zero? rc)
    (throw (ex-info (str (name op) ": invalid or stale id") (assoc data :op op))))
  rc)

(defn make-world
  "A new single-threaded Box2D world. Options:
    :gravity [gx gy]       default [0 9.8], y-down
    :max-steps n           the most steps one step-fixed! call runs, default 5
    :hit-event-threshold   approach speed (m/s) a collision needs to count as
                           a hit, default Box2D's 1.0
    :collect-events?       default true: every step's contact and sensor
                           events are kept until contact-events /
                           sensor-events is called, so none are lost when
                           step-fixed! runs several steps in one frame"
  ([] (make-world {}))
  ([{:keys [gravity max-steps hit-event-threshold collect-events?]
     :or   {gravity [0 9.8] max-steps 5 hit-event-threshold -1.0 collect-events? true}}]
   (let [[gx gy] gravity
         id (f/world-create (double gx) (double gy) (double hit-event-threshold))]
     (when (zero? id)
       (throw (ex-info "Box2D couldn't create a world (too many worlds?)" {:gravity gravity})))
     (->World id gravity (atom true) (atom c/empty-registry)
              {:state (ffi/alloc 64)                                   ; one body's numbers
               :ids   (ffi/alloc (* 8 f/MAX-CONTACTS))                 ; contact body ids
               :data  (ffi/alloc (* 8 f/CONTACT-DOUBLES f/MAX-CONTACTS))
               :ev-ids  (ffi/alloc (* 8 f/EVENT-IDS EVENT-CAP))             ; contact events
               :ev-data (ffi/alloc (* 8 f/EVENT-DOUBLES EVENT-CAP))
               :overlaps (ffi/alloc (* 8 2 f/MAX-OVERLAPS))}                ; sensor overlaps
              (atom 0.0) max-steps (boolean collect-events?) (atom no-events)
              (atom no-sensor-events)))))

(defn destroy!
  "Releases world w and everything in it. Safe to call more than once."
  [w]
  (when (compare-and-set! (:alive w) true false)
    (f/world-destroy (:id w))
    (run! ffi/free (vals (:scratch w)))
    (reset! (:registry w) c/empty-registry))
  nil)

(defmacro with-world
  "(with-world [w opts] body...): makes a world, runs body, and destroys the
  world afterwards, even if body throws. opts is optional."
  [[sym opts] & body]
  `(let [~sym (make-world ~(or opts {}))]
     (try ~@body
          (finally (destroy! ~sym)))))

;; --- stepping ----------------------------------------------------------------

(declare collect-events!)

(defn step!
  "Advances world w by dt seconds, using substeps sub-steps (default 4)."
  ([w dt] (step! w dt SUBSTEPS))
  ([w dt substeps]
   (live! w)
   (check! (f/world-step (:id w) (double dt) (int substeps)) :step! {:world (:id w)})
   (when (:collect-events? w) (collect-events! w))
   nil))

(defn step-fixed!
  "Adds frame-dt to the world's time accumulator and runs as many STEP-sized
  steps as fit, but at most :max-steps; time beyond that is dropped, so a
  pause (a breakpoint, a REPL session) doesn't make the world lurch. Returns
  the leftover fraction of a step (alpha, 0..1) for render interpolation."
  [w frame-dt]
  (live! w)
  (let [acc  (+ @(:acc w) (double frame-dt))
        ;; the epsilon stops 1/60 + 1/60 + ... rounding down to one step short
        due  (long (Math/floor (+ (/ acc STEP) 1e-9)))
        n    (min due (:max-steps w))]
    (dotimes [_ n] (step! w STEP SUBSTEPS))
    (let [left (max 0.0 (- acc (* n STEP)))
          left (if (< n due) (rem left STEP) left)]   ; capped: drop whole steps
      (reset! (:acc w) left)
      (min 1.0 (/ left STEP)))))

;; --- bodies ------------------------------------------------------------------

(def ^:private body-types {:static 0 :kinematic 1 :dynamic 2})
(def ^:private body-type-names {0 :static 1 :kinematic 2 :dynamic})

(defn- body-id
  "The body id for name-or-id, throwing on an unknown name."
  [w name-or-id]
  (live! w)
  (or (c/resolve-id @(:registry w) :body name-or-id)
      (throw (ex-info (str "no body named " name-or-id) {:name name-or-id}))))

(defn- add-shape!
  "Adds one shape spec to body id. Returns the shape id, or nil if Box2D
  rejected it."
  [id {:keys [box offset angle density friction restitution
              contact-events? hit-events? sensor? sensor-events?]
       :or   {offset [0 0] angle 0 friction 0.6 restitution 0.0}
       :as   shape}]
  (cond
    box (let [[hw hh] box
              [ox oy] offset
              ;; a sensor weighs nothing unless asked: a foot sensor
              ;; shouldn't change how heavy its body is
              density (or density (if sensor? 0.0 1.0))
              flags (bit-or (if contact-events? f/SHAPE-CONTACT-EVENTS 0)
                            (if hit-events? f/SHAPE-HIT-EVENTS 0)
                            (if sensor? f/SHAPE-SENSOR 0)
                            ;; a sensor is pointless without its events
                            (if (or sensor-events? sensor?) f/SHAPE-SENSOR-EVENTS 0))
              sid (f/body-add-box id (double hw) (double hh) (double ox) (double oy)
                                  (double angle) (double density)
                                  (double friction) (double restitution) (int flags))]
          (when-not (zero? sid) sid))
    :else (throw (ex-info "unknown shape kind (step 0 supports :box)" {:shape shape}))))

(declare destroy-body!)

(defn body!
  "Creates a body and returns its id. With a keyword name, an existing body
  of that name is destroyed first, so re-evaluating the form replaces it.

  spec keys (all optional):
    :type            :static (default), :kinematic or :dynamic
    :pos [x y]       position, default [0 0]
    :angle a         radians, default 0
    :fixed-rotation? true stops it rotating
    :awake?          false makes it start asleep (default true)
    :user-data u64   an integer to hand back later, e.g. a flecs entity id
    :shapes          [{:box [half-w half-h] :offset [x y] :angle a
                       :density d :friction f :restitution r
                       :contact-events? b :hit-events? b
                       :sensor? b :sensor-events? b}]
                     Contact (begin/end touching) and hit events are off by
                     default; they're reported when either shape asks.
                     A :sensor? shape detects overlaps without colliding,
                     and weighs nothing unless given a :density. It only
                     sees shapes with :sensor-events? true (static ones
                     included), so turn that on for whatever it should
                     notice."
  ([w spec] (body! w nil spec))
  ([w body-name {:keys [type pos angle fixed-rotation? awake? user-data shapes]
                 :or   {type :static pos [0 0] angle 0 awake? true user-data 0}
                 :as   spec}]
   (live! w)
   (when (and body-name (not (keyword? body-name)))
     (throw (ex-info "body names are keywords" {:name body-name})))
   (let [type-n (or (body-types type)
                    (throw (ex-info "unknown body :type" {:type type})))]
     (when-let [old (and body-name (c/resolve-id @(:registry w) :body body-name))]
       (destroy-body! w old))
     (let [[x y] pos
           id (f/body-create (:id w) (int type-n) (double x) (double y) (double angle)
                             (int (c/flag fixed-rotation?)) (int (c/flag awake?))
                             (long user-data))]
       (when (zero? id)
         (throw (ex-info "Box2D couldn't create the body" {:name body-name :spec spec})))
       (let [sids (loop [ss (seq shapes) acc []]
                    (if-let [s (first ss)]
                      (if-let [sid (add-shape! id s)]
                        (recur (next ss) (conj acc sid))
                        (do (f/body-destroy id)
                            (throw (ex-info "Box2D rejected a shape" {:name body-name :shape s}))))
                      acc))]
         (swap! (:registry w) c/add-entry :body id
                (cond-> {:spec spec :shapes sids} body-name (assoc :name body-name)))
         id)))))

(defn destroy-body!
  "Destroys a body (by name or id), with its shapes."
  [w name-or-id]
  (let [id (body-id w name-or-id)]
    (swap! (:registry w) c/remove-entry :body id)
    (check! (f/body-destroy id) :destroy-body! {:body name-or-id})
    nil))

(defn- read-state
  "[x y angle vx vy w] for body name-or-id."
  [w name-or-id]
  (let [id  (body-id w name-or-id)
        buf (:state (:scratch w))]
    (check! (f/body-read id buf) :read {:body name-or-id})
    (c/read-doubles buf 6)))

(defn transform
  "A body's position and rotation: {:x :y :angle}."
  [w name-or-id]
  (let [[x y a] (read-state w name-or-id)]
    {:x x :y y :angle a}))

(defn velocity
  "A body's linear and angular velocity: {:vx :vy :w}."
  [w name-or-id]
  (let [[_ _ _ vx vy av] (read-state w name-or-id)]
    {:vx vx :vy vy :w av}))

(defn- state
  "transform and velocity together, from one read."
  [w name-or-id]
  (let [[x y a vx vy av] (read-state w name-or-id)]
    {:x x :y y :angle a :vx vx :vy vy :w av}))

;; --- moving bodies -----------------------------------------------------------

(defn impulse!
  "Applies a world-space impulse [ix iy] (N·s, so a change in velocity of
  impulse / mass) at the center of mass, or at world point [px py]. Wakes the
  body."
  ([w body [ix iy]]
   (check! (f/body-apply-impulse (body-id w body) (double ix) (double iy) 0 0.0 0.0 1)
           :impulse! {:body body})
   nil)
  ([w body [ix iy] [px py]]
   (check! (f/body-apply-impulse (body-id w body) (double ix) (double iy)
                                 1 (double px) (double py) 1)
           :impulse! {:body body})
   nil))

(defn force!
  "Applies a world-space force [fx fy] (N) for the next step, at the center of
  mass or at world point [px py]. Wakes the body. Call it every step for a
  steady push."
  ([w body [fx fy]]
   (check! (f/body-apply-force (body-id w body) (double fx) (double fy) 0 0.0 0.0 1)
           :force! {:body body})
   nil)
  ([w body [fx fy] [px py]]
   (check! (f/body-apply-force (body-id w body) (double fx) (double fy)
                               1 (double px) (double py) 1)
           :force! {:body body})
   nil))

(defn set-velocity!
  "Sets a body's linear velocity [vx vy] in m/s."
  [w body [vx vy]]
  (check! (f/body-set-velocity (body-id w body) (double vx) (double vy))
          :set-velocity! {:body body})
  nil)

(defn set-angular-velocity!
  "Sets a body's angular velocity in radians per second."
  [w body av]
  (check! (f/body-set-angular-velocity (body-id w body) (double av))
          :set-angular-velocity! {:body body})
  nil)

(defn set-transform!
  "Teleports a body to [x y], keeping its angle unless one is given. Box2D
  calls this fairly expensive: fine for resets, not for every frame.
  Contacts lag a step behind: until the next step, contacts and grounded?
  still describe where the body was."
  ([w body pos] (set-transform! w body pos (:angle (transform w body))))
  ([w body [x y] angle]
   (check! (f/body-set-transform (body-id w body) (double x) (double y) (double angle))
           :set-transform! {:body body})
   nil))

(defn set-awake!
  "Wakes a body (true) or puts it to sleep (false), along with everything
  touching it."
  [w body awake?]
  (check! (f/body-set-awake (body-id w body) (int (c/flag awake?))) :set-awake! {:body body})
  nil)

(defn set-gravity-scale!
  "How strongly gravity pulls this body: 1 normal, 0 floats, -1 falls up."
  [w body scale]
  (check! (f/body-set-gravity-scale (body-id w body) (double scale))
          :set-gravity-scale! {:body body})
  nil)

(defn set-damping!
  "Sets a body's {:linear :angular} damping (either may be left out)."
  [w body {:keys [linear angular] :or {linear 0.0 angular 0.0}}]
  (check! (f/body-set-damping (body-id w body) (double linear) (double angular))
          :set-damping! {:body body})
  nil)

;; --- contacts ----------------------------------------------------------------

(def ^:private TOUCHING-SEPARATION
  "Box2D keeps speculative contacts a little before shapes touch. Closer than
  this (meters) counts as touching; it's a bit over Box2D's linear slop."
  0.01)

(defn contacts
  "The contacts Box2D is tracking on a body, as a vector of
  {:other id :other-name name :normal [nx ny] :point [x y] :points n
   :separation s :touching? bool}.
  :normal points from the other body into this one, so a body resting on a
  floor in a y-down world has a normal of about [0 -1]. Contacts can be
  speculative (shapes about to touch); :touching? says they actually are."
  [w body]
  (let [id  (body-id w body)
        {:keys [ids data]} (:scratch w)
        n   (f/body-contacts id ids data f/MAX-CONTACTS)
        reg @(:registry w)]
    (when (neg? n) (check! n :contacts {:body body}))
    (mapv (fn [i]
            (let [[nx ny pts px py sep] (c/read-doubles-at data (* i f/CONTACT-DOUBLES)
                                                           f/CONTACT-DOUBLES)
                  other (ffi/read ids :uint64 (* 8 i))]
              {:other      other
               :other-name (:name (c/entry reg :body other))
               :normal     [nx ny]
               :point      [px py]
               :points     (long pts)
               :separation sep
               :touching?  (and (pos? pts) (<= sep TOUCHING-SEPARATION))}))
          (range n))))

(defn gravity
  "The world's gravity [gx gy]."
  [w]
  (live! w)
  (:gravity w))

(defn grounded?
  "True when a body is standing on something: a touching contact whose normal
  points against gravity by at least :min-normal (default 0.7, about 45°),
  and that the body isn't moving away from faster than :max-separating-speed
  (m/s, default 0.1).

  The speed check matters right after a jump. Box2D's contact data is from
  the start of the last step, so a body that just pushed off still shows its
  floor contact for one step, while already moving away from it.

  Works for y-down and y-up worlds alike."
  ([w body] (grounded? w body {}))
  ([w body {:keys [min-normal max-separating-speed]
            :or   {min-normal 0.7 max-separating-speed 0.1}}]
   (let [[gx gy] (:gravity w)
         g       (Math/sqrt (+ (* gx gx) (* gy gy)))
         {:keys [vx vy]} (velocity w body)]
     (and (pos? g)
          (boolean
           (some (fn [{[nx ny] :normal :keys [touching? other]}]
                   (and touching?
                        (>= (/ (- (+ (* nx gx) (* ny gy))) g) min-normal)
                        ;; how fast we're leaving the other body along the normal
                        (let [{ovx :vx ovy :vy} (velocity w other)]
                          (<= (+ (* (- vx ovx) nx) (* (- vy ovy) ny))
                              max-separating-speed))))
                 (contacts w body)))))))

;; --- mass and momentum -------------------------------------------------------

(defn mass-data
  "A body's mass properties: {:mass kg :inertia kg·m² (about the center of
  mass) :center [x y] (world) :local-center [x y] (relative to the body)}."
  [w body]
  (let [id  (body-id w body)
        buf (:state (:scratch w))]
    (check! (f/body-mass-data id buf) :mass-data {:body body})
    (let [[m i lx ly wx wy] (c/read-doubles buf 6)]
      {:mass m :inertia i :center [wx wy] :local-center [lx ly]})))

(defn mass
  "A body's mass in kg (0 for static bodies)."
  [w body]
  (:mass (mass-data w body)))

(defn momentum
  "A body's linear momentum, mass × velocity: {:px :py} in kg·m/s."
  [w body]
  (let [m (mass w body)
        {:keys [vx vy]} (velocity w body)]
    {:px (* m vx) :py (* m vy)}))

;; --- shapes ------------------------------------------------------------------

(defn shapes
  "A body's shape ids, in the order its spec listed them."
  [w body]
  (:shapes (c/entry @(:registry w) :body (body-id w body))))

(defn shape-body
  "The body a shape belongs to."
  [w shape]
  (live! w)
  (let [b (f/shape-body shape)]
    (when (zero? b) (check! -1 :shape-body {:shape shape}))
    b))

(defn set-shape!
  "Changes a shape's settings while the world runs. Any of
  {:friction f :restitution r :density d :contact-events? b :hit-events? b
   :sensor-events? b}. A new density updates the body's mass."
  [w shape {:keys [friction restitution density contact-events? hit-events? sensor-events?]
            :as settings}]
  (live! w)
  (let [ok! #(check! % :set-shape! {:shape shape :settings settings})]
    (when (some? friction)        (ok! (f/shape-set-friction shape (double friction))))
    (when (some? restitution)     (ok! (f/shape-set-restitution shape (double restitution))))
    (when (some? density)         (ok! (f/shape-set-density shape (double density))))
    (when (some? contact-events?) (ok! (f/shape-enable-contact-events shape (int (c/flag contact-events?)))))
    (when (some? hit-events?)     (ok! (f/shape-enable-hit-events shape (int (c/flag hit-events?)))))
    (when (some? sensor-events?)  (ok! (f/shape-enable-sensor-events shape (int (c/flag sensor-events?)))))
    nil))

(defn user-data
  "The :user-data a body was made with (0 if none)."
  [w body]
  (f/body-user-data (body-id w body)))

;; --- contact events ----------------------------------------------------------

(defn- read-events
  "Box2D's events of one kind from the last step, as maps."
  [w kind-n kind]
  (let [{:keys [ev-ids ev-data]} (:scratch w)
        total (f/world-contact-events (:id w) (int kind-n) ev-ids ev-data EVENT-CAP)
        _     (when (neg? total) (check! total :contact-events {:world (:id w)}))
        reg   @(:registry w)
        about (fn [body]
                (let [e (c/entry reg :body body)]
                  [(:name e) (get-in e [:spec :user-data] 0)]))]
    {:dropped (max 0 (- total EVENT-CAP))
     :events
     (mapv (fn [i]
             (let [id #(ffi/read ev-ids :uint64 (* 8 (+ (* i f/EVENT-IDS) %)))
                   [nx ny px py extra] (c/read-doubles-at ev-data (* i f/EVENT-DOUBLES)
                                                          f/EVENT-DOUBLES)
                   ba (id 2) bb (id 3)
                   [na ua] (about ba)
                   [nb ub] (about bb)]
               (cond-> {:shape-a (id 0) :shape-b (id 1) :body-a ba :body-b bb
                        :name-a na :name-b nb :user-a ua :user-b ub}
                 (= kind :begin) (assoc :normal [nx ny] :point [px py] :points (long extra))
                 (= kind :hit)   (assoc :normal [nx ny] :point [px py] :speed extra))))
           (range (min total EVENT-CAP)))}))

(defn- step-events
  "All three kinds from the last step: {:begin [..] :end [..] :hit [..] :dropped n}."
  [w]
  (let [r {:begin (read-events w f/EVENTS-BEGIN :begin)
           :end   (read-events w f/EVENTS-END :end)
           :hit   (read-events w f/EVENTS-HIT :hit)}]
    {:begin   (:events (:begin r))
     :end     (:events (:end r))
     :hit     (:events (:hit r))
     :dropped (reduce + (map :dropped (vals r)))}))

(declare step-sensor-events)

(defn- add-pending
  "Adds new events (by kind, plus :dropped) to the pending ones, keeping at
  most MAX-PENDING of each kind and counting the rest as dropped."
  [pending new kinds]
  (reduce (fn [p k]
            (let [room (max 0 (- MAX-PENDING (count (get p k))))
                  evs  (get new k)]
              (-> p
                  (update k into (take room) evs)
                  (update :dropped + (max 0 (- (count evs) room))))))
          (update pending :dropped + (:dropped new))
          kinds))

(defn- any-events? [new kinds]
  (or (pos? (:dropped new)) (some #(seq (get new %)) kinds)))

(defn- collect-events!
  "Adds the last step's contact and sensor events to the ones waiting for
  contact-events and sensor-events."
  [w]
  (let [new (step-events w)]
    (when (any-events? new [:begin :end :hit])
      (swap! (:events w) add-pending new [:begin :end :hit])))
  (let [new (step-sensor-events w)]
    (when (any-events? new [:begin :end])
      (swap! (:sensor-events w) add-pending new [:begin :end]))))

(defn contact-events
  "Contact events as plain data:
    {:begin   [{:body-a :body-b :name-a :name-b :user-a :user-b :shape-a :shape-b
                :normal [nx ny] :point [x y] :points n}]   ; started touching
     :end     [{...ids, names and user data only}]        ; stopped touching
     :hit     [{...  :normal [nx ny] :point [x y] :speed s}] ; hit hard enough
     :dropped n}                                          ; lost to the caps
  :normal points from a to b. :speed is how fast they were approaching, in
  m/s. A body in an :end event may have been destroyed already (its id is
  then 0).

  With :collect-events? on (the default) this returns everything since the
  last call, over however many steps, and clears it. With it off it returns
  the last step's events. Only shapes with :contact-events? / :hit-events?
  report (either shape of a pair is enough)."
  [w]
  (live! w)
  (if (:collect-events? w)
    (let [[old _] (reset-vals! (:events w) no-events)] old)
    (step-events w)))

(defn events-for
  "The events in a contact-events kind (e.g. (:hit events)) that involve
  body (a name or id), turned around so the body is always the subject:
  each gets :other, :other-name and :other-user, and :normal (if any) points
  from the other body into this one, as in contacts."
  [w events body]
  (let [id (body-id w body)]
    (into []
          (keep (fn [{:keys [body-a body-b normal] :as e}]
                  (cond
                    (= id body-b) (assoc e :other body-a :other-name (:name-a e) :other-user (:user-a e))
                    (= id body-a) (cond-> (assoc e :other body-b :other-name (:name-b e)
                                                 :other-user (:user-b e))
                                    normal (assoc :normal (mapv - normal))))))
          events)))

;; --- sensors -----------------------------------------------------------------

(defn- read-sensor-events
  "Box2D's sensor events of one kind from the last step."
  [w kind-n]
  (let [{:keys [ev-ids]} (:scratch w)
        total (f/world-sensor-events (:id w) (int kind-n) ev-ids EVENT-CAP)
        _     (when (neg? total) (check! total :sensor-events {:world (:id w)}))
        reg   @(:registry w)
        about (fn [body]
                (let [e (c/entry reg :body body)]
                  [(:name e) (get-in e [:spec :user-data] 0)]))]
    {:dropped (max 0 (- total EVENT-CAP))
     :events
     (mapv (fn [i]
             (let [id #(ffi/read ev-ids :uint64 (* 8 (+ (* i f/EVENT-IDS) %)))
                   sb (id 2) vb (id 3)
                   [sn su] (about sb)
                   [vn vu] (about vb)]
               {:sensor-shape (id 0) :visitor-shape (id 1)
                :sensor-body sb :visitor-body vb
                :sensor-name sn :visitor-name vn
                :sensor-user su :visitor-user vu}))
           (range (min total EVENT-CAP)))}))

(defn- step-sensor-events
  "Both kinds from the last step: {:begin [..] :end [..] :dropped n}."
  [w]
  (let [b (read-sensor-events w f/EVENTS-BEGIN)
        e (read-sensor-events w f/EVENTS-END)]
    {:begin (:events b) :end (:events e) :dropped (+ (:dropped b) (:dropped e))}))

(defn sensor-events
  "Sensor events as plain data:
    {:begin [{:sensor-shape :sensor-body :sensor-name :sensor-user
              :visitor-shape :visitor-body :visitor-name :visitor-user}]
     :end   [...]
     :dropped n}
  :begin when a shape starts overlapping a sensor, :end when it stops (also
  when either is destroyed or teleported away; a destroyed body's id is 0).
  Unlike contacts, sensors are up to date as of the last step: no lag.

  Like contact-events: everything since the last call with
  :collect-events? on (the default), or the last step's events with it off."
  [w]
  (live! w)
  (if (:collect-events? w)
    (let [[old _] (reset-vals! (:sensor-events w) no-sensor-events)] old)
    (step-sensor-events w)))

(defn sensor-overlaps
  "What a sensor shape overlaps right now (as of the last step):
  [{:shape id :body id :name name :user u}]."
  [w sensor-shape]
  (live! w)
  (let [buf (:overlaps (:scratch w))
        n   (f/shape-sensor-overlaps sensor-shape buf f/MAX-OVERLAPS)
        reg @(:registry w)]
    (when (neg? n) (check! n :sensor-overlaps {:shape sensor-shape :hint "is it a :sensor? shape?"}))
    (mapv (fn [i]
            (let [body (ffi/read buf :uint64 (* 8 (inc (* 2 i))))
                  e    (c/entry reg :body body)]
              {:shape (ffi/read buf :uint64 (* 8 (* 2 i)))
               :body  body
               :name  (:name e)
               :user  (get-in e [:spec :user-data] 0)}))
          (range n))))

;; --- inspection --------------------------------------------------------------

(defn bodies
  "Every body in w: a vector of {:id :name :type}, in creation order."
  [w]
  (live! w)
  (mapv (fn [[id e]] {:id id :name (:name e) :type (get-in e [:spec :type] :static)})
        (c/entries @(:registry w) :body)))

(defn body-info
  "Everything about one body as plain data: the spec it was made from, its
  shapes, and its live state."
  [w name-or-id]
  (let [id  (body-id w name-or-id)
        e   (c/entry @(:registry w) :body id)
        buf (:state (:scratch w))]
    (check! (f/body-mass-info id buf) :body-info {:body name-or-id})
    (let [[mass inertia awake type-n] (c/read-doubles buf 4)
          st (state w id)]
      {:id        id
       :name      (:name e)
       :type      (body-type-names (long type-n))
       :spec      (:spec e)
       :shapes    (:shapes e)
       :user-data (f/body-user-data id)
       :transform (select-keys st [:x :y :angle])
       :velocity  (select-keys st [:vx :vy :w])
       :mass      mass
       :inertia   inertia
       :awake?    (== 1.0 awake)
       :contacts  (contacts w id)})))

(defn snapshot
  "The whole world as one plain map."
  [w]
  (live! w)
  {:id        (:id w)
   :gravity   (:gravity w)
   :max-steps (:max-steps w)
   :bodies    (mapv #(body-info w (:id %)) (bodies w))})

;; --- debug outlines ----------------------------------------------------------
;; Built in Clojure from the registry's shape specs and each body's transform,
;; so it can change live at the REPL. Box2D's own b2DebugDraw passes structs by
;; value to its callbacks, which jolt can't receive; if this gets slow with
;; many bodies, the shim can collect those callbacks into a buffer instead and
;; this keeps returning the same maps (guide, step 2).

(defn- rotate [[x y] c s] [(- (* c x) (* s y)) (+ (* s x) (* c y))])

(defn- box-points
  "The four corners of a :box shape spec, in world space, for a body at
  (bx, by) turned by angle ba. Counter-clockwise in y-up, so clockwise on a
  y-down screen."
  [{:keys [box offset angle] :or {offset [0 0] angle 0}} bx by ba]
  (let [[hw hh] box
        [ox oy] offset
        sc (Math/cos angle) ss (Math/sin angle)
        bc (Math/cos ba)    bs (Math/sin ba)]
    (mapv (fn [corner]
            (let [[lx ly] (rotate corner sc ss)
                  [wx wy] (rotate [(+ lx ox) (+ ly oy)] bc bs)]
              [(+ bx wx) (+ by wy)]))
          [[(- hw) (- hh)] [hw (- hh)] [hw hh] [(- hw) hh]])))

(defn- awake-and-type
  "[awake? type] for body id, from one mass-info read."
  [w id]
  (let [buf (:state (:scratch w))]
    (check! (f/body-mass-info id buf) :debug-shapes {:body id})
    (let [[_ _ awake type-n] (c/read-doubles buf 4)]
      [(== 1.0 awake) (body-type-names (long type-n))])))

(defn debug-shapes
  "Every shape in w as plain data for drawing, in world units:
  {:kind :polygon :points [[x y] ...] :center [x y]
   :body id :name name :shape id :type :static|:kinematic|:dynamic :awake? bool
   :sensor? bool}.
  Draw the points as a closed loop. No drawing happens here: the caller
  (e.g. raylib on the main thread) does that."
  [w]
  (live! w)
  (into []
        (mapcat
         (fn [[id {body-name :name :keys [spec shapes]}]]
           (let [{:keys [x y angle]} (transform w id)
                 [awake? type]       (awake-and-type w id)]
             (mapv (fn [shape-spec sid]
                     (cond-> {:body id :name body-name :shape sid
                              :type type :awake? awake? :center [x y]
                              :sensor? (boolean (:sensor? shape-spec))}
                       (:box shape-spec) (assoc :kind :polygon
                                                :points (box-points shape-spec x y angle))))
                   (:shapes spec) shapes))))
        (c/entries @(:registry w) :body)))

;; --- headless simulation -----------------------------------------------------

(defn simulate
  "Steps w for `seconds` on the fixed timestep and returns samples, so physics
  can be tried and measured at the REPL without a game. Doesn't touch the
  step-fixed! accumulator.

  opts: :sample [names-or-ids] to record, :every seconds between samples
  (default every step). Returns a vector of
  {:t seconds, <name-or-id> {:x :y :angle :vx :vy :w}, ...}, starting at t 0."
  ([w seconds] (simulate w seconds {}))
  ([w seconds {:keys [sample every] :or {sample [] every STEP}}]
   (live! w)
   (let [n     (long (Math/round (/ (double seconds) STEP)))
         k     (max 1 (long (Math/round (/ (double every) STEP))))
         snap  (fn [i] (reduce (fn [m b] (assoc m b (state w b)))
                               {:t (* i STEP)} sample))]
     (loop [i 0 acc (transient [])]
       (let [acc (if (zero? (rem i k)) (conj! acc (snap i)) acc)]
         (if (< i n)
           (do (step! w STEP SUBSTEPS) (recur (inc i) acc))
           (persistent! acc)))))))

(defn series
  "One field of one sampled body across samples: [[t v] ...]."
  [samples body field]
  (mapv (fn [s] [(:t s) (get-in s [body field])]) samples))

(defn extent
  "The smallest and largest value of a field: {:min v :max v}. With y-down,
  the highest point a body reached is the :min of :y."
  [samples body field]
  (let [vs (mapv second (series samples body field))]
    {:min (reduce min vs) :max (reduce max vs)}))

(defn period
  "The mean time between successive local maxima of a field, or nil if there
  are fewer than two. For measuring how fast something oscillates."
  [samples body field]
  (let [s     (series samples body field)
        peaks (into []
                    (comp (map (fn [[[_ a] [t b] [_ c]]] (when (and (> b a) (>= b c)) t)))
                          (remove nil?))
                    (partition 3 1 s))]
    (when (>= (count peaks) 2)
      (/ (- (peek peaks) (first peaks)) (dec (count peaks))))))

(defn settled-at
  "The time from which a body stays still (speed and spin below :speed,
  default 0.01) until the last sample, or nil if it never settles."
  ([samples body] (settled-at samples body {}))
  ([samples body {:keys [speed] :or {speed 0.01}}]
   (let [still? (fn [s] (let [{:keys [vx vy w]} (get s body)]
                          (and (< (Math/sqrt (+ (* vx vx) (* vy vy))) speed)
                               (< (Math/abs (double w)) speed))))]
     (loop [ss (rseq samples) t nil]
       (if-let [s (first ss)]
         (if (still? s) (recur (next ss) (:t s)) t)
         t)))))
