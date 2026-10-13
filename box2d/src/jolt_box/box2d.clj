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

(def ^:private STATE-DOUBLES
  "How many doubles the :state scratch buffer holds: enough for the biggest
  joint params or read (18). The shim writes straight into it, so anything
  bigger must be checked against this first (see state-buf)."
  32)

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

;; --- the native library ------------------------------------------------------

(defn- loaded-version
  "The loaded shim's JB2_VERSION, or nil when libjoltbox2d isn't loaded."
  []
  (try (long (f/version)) (catch Throwable _ nil)))

(defn available?
  "True when libjoltbox2d is loaded and its shim is the version this
  namespace expects. deps.edn declares the library :optional, so a project
  starts without the build; this says whether physics will work."
  []
  (= f/VERSION (loaded-version)))

(defn- require-native!
  "Throws, saying what to do, unless available? is true."
  []
  (let [v (loaded-version)]
    (cond
      (nil? v)
      (throw (ex-info (str "jolt-box: libjoltbox2d isn't loaded. Run `jolt native` in "
                           "jolt-box/box2d and restart: the runtime loads it at startup.")
                      {}))
      (not= v f/VERSION)
      (throw (ex-info (str "jolt-box: the built shim is version " v " and this source expects "
                           f/VERSION ". Run `jolt native` in jolt-box/box2d and restart.")
                      {:built v :expected f/VERSION})))))

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
   (require-native!)
   (let [[gx gy] gravity
         id (f/world-create (double gx) (double gy) (double hit-event-threshold))]
     (when (zero? id)
       (throw (ex-info "Box2D couldn't create a world (too many worlds?)" {:gravity gravity})))
     (->World id gravity (atom true) (atom c/empty-registry)
              {:state (ffi/alloc (* 8 STATE-DOUBLES))                  ; one body's or joint's numbers
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
  [id {:keys [box circle capsule radius offset angle density friction restitution
              contact-events? hit-events? sensor? sensor-events? group category mask]
       :or   {offset [0 0] angle 0 friction 0.6 restitution 0.0}
       :as   shape}]
  (let [[ox oy] offset
        ;; a sensor weighs nothing unless asked: a foot sensor shouldn't
        ;; change how heavy its body is
        density (double (or density (if sensor? 0.0 1.0)))
        friction (double friction)
        restitution (double restitution)
        flags (int (bit-or (if contact-events? f/SHAPE-CONTACT-EVENTS 0)
                           (if hit-events? f/SHAPE-HIT-EVENTS 0)
                           (if sensor? f/SHAPE-SENSOR 0)
                           ;; a sensor is pointless without its events
                           (if (or sensor-events? sensor?) f/SHAPE-SENSOR-EVENTS 0)))
        sid (cond
              box     (let [[hw hh] box]
                        (f/body-add-box id (double hw) (double hh) (double ox) (double oy)
                                        (double angle) density friction restitution flags))
              circle  (f/body-add-circle id (double ox) (double oy) (double circle)
                                         density friction restitution flags)
              capsule (let [[[x1 y1] [x2 y2]] capsule]
                        (f/body-add-capsule id (double (+ x1 ox)) (double (+ y1 oy))
                                            (double (+ x2 ox)) (double (+ y2 oy))
                                            (double (or radius 0)) density friction restitution
                                            flags))
              :else (throw (ex-info "unknown shape kind: use :box, :circle or :capsule"
                                    {:shape shape})))]
    (when-not (zero? sid)
      (when (or group category mask)
        ;; Box2D's defaults: category 1, every bit of the mask (-1), group 0
        (f/shape-set-filter sid (long (or category 1)) (long (or mask -1)) (int (or group 0))))
      sid)))

(declare destroy-body! attached-joints remake-joint! awake-and-type)

(defn body!
  "Creates a body and returns its id. With a keyword name, an existing body
  of that name is destroyed first, so re-evaluating the form replaces it.
  Joints attached to the old body are made again on the new one, at the same
  places on the body (and with any settings changed since).

  spec keys (all optional):
    :type            :static (default), :kinematic or :dynamic
    :pos [x y]       position, default [0 0]
    :angle a         radians, default 0
    :fixed-rotation? true stops it rotating
    :awake?          false makes it start asleep (default true)
    :enable-sleep?   false keeps it awake always (default true)
    :sleep-threshold m/s it must stay under to fall asleep (Box2D: 0.05)
    :linear-damping :angular-damping   slow it down over time (default 0)
    :gravity-scale   1 normal, 0 floats
    :user-data u64   an integer to hand back later, e.g. a flecs entity id
    :shapes          [{:box [half-w half-h] :offset [x y] :angle a
                       ;; or {:circle radius :offset [x y]}
                       ;; or {:capsule [[x1 y1] [x2 y2]] :radius r :offset [x y]}
                       :density d :friction f :restitution r
                       :contact-events? b :hit-events? b
                       :sensor? b :sensor-events? b
                       :group g :category bits :mask bits}]
                     Contact (begin/end touching) and hit events are off by
                     default; they're reported when either shape asks.
                     A :sensor? shape detects overlaps without colliding,
                     and weighs nothing unless given a :density. It only
                     sees shapes with :sensor-events? true (static ones
                     included), so turn that on for whatever it should
                     notice.
                     Collision filtering: shapes in the same negative
                     :group never collide (e.g. the parts of one character),
                     the same positive group always do. Otherwise two shapes
                     collide when each one's :category bits are in the
                     other's :mask (defaults: category 1, mask all)."
  ([w spec] (body! w nil spec))
  ([w body-name {:keys [type pos angle fixed-rotation? awake? user-data shapes]
                 :or   {type :static pos [0 0] angle 0 awake? true user-data 0}
                 :as   spec}]
   (live! w)
   (when (and body-name (not (keyword? body-name)))
     (throw (ex-info "body names are keywords" {:name body-name})))
   (let [type-n (or (body-types type)
                    (throw (ex-info "unknown body :type" {:type type})))]
     (let [old      (and body-name (c/resolve-id @(:registry w) :body body-name))
           attached (when old (attached-joints @(:registry w) old))
           _        (when old (destroy-body! w old))
           [x y]    pos
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
         (let [{:keys [linear-damping angular-damping enable-sleep? sleep-threshold gravity-scale]} spec]
           (when (or linear-damping angular-damping)
             (f/body-set-damping id (double (or linear-damping 0)) (double (or angular-damping 0))))
           (when (or (some? enable-sleep?) sleep-threshold)
             (f/body-set-sleep id (int (c/flag (if (some? enable-sleep?) enable-sleep? true)))
                               (double (or sleep-threshold -1))))
           (when gravity-scale
             (f/body-set-gravity-scale id (double gravity-scale))))
         (swap! (:registry w) c/add-entry :body id
                (cond-> {:spec spec :shapes sids} body-name (assoc :name body-name)))
         (doseq [[_ j] attached]
           (remake-joint! w j old id))
         id)))))

(defn destroy-body!
  "Destroys a body (by name or id), with its shapes and joints."
  [w name-or-id]
  (let [id (body-id w name-or-id)]
    (swap! (:registry w)
           (fn [reg]
             (reduce (fn [r [jid _]] (c/remove-entry r :joint jid))
                     (c/remove-entry reg :body id)
                     (attached-joints reg id))))
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

;; Clojure for now: one read per body. If many bodies a frame shows up in the
;; dev panel, this becomes a single shim call writing them all into one buffer
;; (guide §4.1), with the same result.
(defn read-bodies
  "Position and velocity of many bodies at once, as one flat vector:
  [x y angle vx vy w, x y angle vx vy w, ...] in the order given."
  [w names-or-ids]
  (live! w)
  (into [] (mapcat #(read-state w %)) names-or-ids))

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

;; --- joints ------------------------------------------------------------------
;; A joint's spec is given in world space (anchors, axis) at the moment it's
;; made, which is what's natural at the REPL. It's turned into the bodies'
;; local frames once, and the registry keeps that :frame, so a joint made
;; again later (its body replaced) sits at the same places on the bodies.

(defn- rotate [[x y] c s] [(- (* c x) (* s y)) (+ (* s x) (* c y))])

(defn- joint-id
  "The joint id for name-or-id, throwing on an unknown name."
  [w name-or-id]
  (live! w)
  (or (c/resolve-id @(:registry w) :joint name-or-id)
      (throw (ex-info (str "no joint named " name-or-id) {:name name-or-id}))))

(defn- joint-entry
  "The registry entry for joint id, throwing if it's unknown or stale (as an
  id the registry has forgotten is)."
  [w id op]
  (or (when (== 1 (f/joint-valid id)) (c/entry @(:registry w) :joint id))
      (check! -1 op {:joint id})))

(defn- attached-joints
  "[id entry] pairs of the registered joints attached to body id."
  [reg body]
  (filterv (fn [[_ j]] (or (= body (:a j)) (= body (:b j)))) (c/entries reg :joint)))

(defn- world->local [{:keys [x y angle]} [px py]]
  (rotate [(- px x) (- py y)] (Math/cos angle) (- (Math/sin angle))))

(defn- local->world [{:keys [x y angle]} lp]
  (let [[rx ry] (rotate lp (Math/cos angle) (Math/sin angle))]
    [(+ x rx) (+ y ry)]))

(defn- joint-frame
  "A joint spec's anchors, axis, reference angle (and for :distance, length)
  in the bodies' local frames, from where bodies a and b are now."
  [w a b {:keys [type anchor anchor-a anchor-b local-anchor-a local-anchor-b
                 axis local-axis reference-angle length]}]
  (let [ta  (transform w a)
        tb  (transform w b)
        [wa wb] (if (= type :distance)
                  ;; default: each body's own origin
                  [(or anchor-a [(:x ta) (:y ta)]) (or anchor-b [(:x tb) (:y tb)])]
                  ;; default: one shared point, b's origin
                  (let [wa (or anchor-a anchor [(:x tb) (:y tb)])]
                    [wa (or anchor-b anchor wa)]))
        [ax ay] (or axis [1 0])]
    (cond-> {:local-anchor-a  (or local-anchor-a (world->local ta wa))
             :local-anchor-b  (or local-anchor-b (world->local tb wb))
             :local-axis      (or local-axis
                                  (rotate [ax ay] (Math/cos (:angle ta)) (- (Math/sin (:angle ta)))))
             :reference-angle (or reference-angle (- (:angle tb) (:angle ta)))}
      (= type :distance)
      (assoc :length (or length (let [[x1 y1] wa [x2 y2] wb]
                                  (Math/sqrt (+ (* (- x2 x1) (- x2 x1)) (* (- y2 y1) (- y2 y1))))))))))

(defn- state-buf
  "The :state scratch buffer, after checking n doubles fit in it. The shim
  writes into it without knowing its size, so this check is all that stops
  an overrun."
  [w n]
  (when (> n STATE-DOUBLES)
    (throw (ex-info "scratch buffer too small" {:need n :have STATE-DOUBLES})))
  (:state (:scratch w)))

(defn- params-buffer
  "A scratch buffer holding the doubles xs, for a <type>-create call."
  [w xs]
  (let [xs  (vec xs)
        buf (state-buf w (count xs))]
    (dorun (map-indexed (fn [i x] (ffi/write buf :double (double x) (* 8 i))) xs))
    buf))

(defn- spring-params [spring]
  (let [{:keys [hertz damping] :or {hertz 0 damping 0}} (when (map? spring) spring)]
    [(c/flag (map? spring)) hertz damping]))

(defn- limit-params [limit]
  (let [[lo hi] (when (vector? limit) limit)]
    [(c/flag (vector? limit)) (or lo 0) (or hi 0)]))

(defn- motor-params [motor max-key]
  (let [{:keys [speed] :or {speed 0}} (when (map? motor) motor)]
    [(c/flag (map? motor)) (or (get motor max-key) 0) speed]))

(defn- prismatic-params
  [{:keys [spring target limit motor collide-connected?]}
   {[lax lay] :local-anchor-a [lbx lby] :local-anchor-b [axx axy] :local-axis :keys [reference-angle]}]
  (concat [lax lay lbx lby axx axy reference-angle (or target 0)]
          (spring-params spring) (limit-params limit) (motor-params motor :max-force)
          [(c/flag collide-connected?)]))

(defn- revolute-params
  [{:keys [spring target limit motor collide-connected?]}
   {[lax lay] :local-anchor-a [lbx lby] :local-anchor-b :keys [reference-angle]}]
  (concat [lax lay lbx lby reference-angle (or target 0)]
          (spring-params spring) (limit-params limit) (motor-params motor :max-torque)
          [(c/flag collide-connected?)]))

(defn- distance-params
  [{:keys [spring limit motor collide-connected?]}
   {[lax lay] :local-anchor-a [lbx lby] :local-anchor-b :keys [length]}]
  (concat [lax lay lbx lby length]
          (spring-params spring) (limit-params limit) (motor-params motor :max-force)
          [(c/flag collide-connected?)]))

(defn- weld-params
  [{:keys [linear-hertz angular-hertz linear-damping angular-damping collide-connected?]}
   {[lax lay] :local-anchor-a [lbx lby] :local-anchor-b :keys [reference-angle]}]
  [lax lay lbx lby reference-angle
   (or linear-hertz 0) (or angular-hertz 0) (or linear-damping 0) (or angular-damping 0)
   (c/flag collide-connected?)])

(defn- read-buf [w id f n op j]
  (let [buf (state-buf w n)]
    (check! (f id buf) op {:joint j})
    (c/read-doubles buf n)))

(defn- read-prismatic [w id j]
  (let [[tr sp mf son hz dz tgt lon lo hi mon ms mmf ax ay]
        (read-buf w id f/prismatic-read f/PRISMATIC-READ :joint j)]
    {:state    {:translation tr :speed sp :motor-force mf}
     :settings {:spring (when (== 1.0 son) {:hertz hz :damping dz})
                :target tgt
                :limit  (when (== 1.0 lon) [lo hi])
                :motor  (when (== 1.0 mon) {:max-force mmf :speed ms})
                :local-axis [ax ay]}}))

(defn- read-revolute [w id j]
  (let [[ang mt son hz dz tgt lon lo hi mon ms mmt]
        (read-buf w id f/revolute-read f/REVOLUTE-READ :joint j)
        {:keys [a b]} (c/entry @(:registry w) :joint id)]
    {:state    {:angle ang
                ;; Box2D has no getter for it: b's spin relative to a's
                :speed (- (:w (velocity w b)) (:w (velocity w a)))
                :motor-torque mt}
     :settings {:spring (when (== 1.0 son) {:hertz hz :damping dz})
                :target tgt
                :limit  (when (== 1.0 lon) [lo hi])
                :motor  (when (== 1.0 mon) {:max-torque mmt :speed ms})}}))

(defn- read-distance [w id j]
  (let [[cur mf len son hz dz lon lo hi mon ms mmf]
        (read-buf w id f/distance-read f/DISTANCE-READ :joint j)]
    {:state    {:length cur :motor-force mf}
     :settings {:length len
                :spring (when (== 1.0 son) {:hertz hz :damping dz})
                :limit  (when (== 1.0 lon) [lo hi])
                :motor  (when (== 1.0 mon) {:max-force mmf :speed ms})}}))

(defn- read-weld [w id j]
  (let [[lh ah ld ad] (read-buf w id f/weld-read f/WELD-READ :joint j)]
    {:state    {}
     :settings {:linear-hertz lh :angular-hertz ah :linear-damping ld :angular-damping ad}}))

(def ^:private joint-kinds
  "Per joint type: how to make it, change it and read it, and the key its
  motor's limit goes by."
  {:prismatic {:create f/prismatic-create :set f/prismatic-set :read read-prismatic
               :params prismatic-params :max-motor :max-force}
   :revolute  {:create f/revolute-create :set f/revolute-set :read read-revolute
               :params revolute-params :max-motor :max-torque}
   :distance  {:create f/distance-create :set f/distance-set :read read-distance
               :params distance-params :max-motor :max-force}
   :weld      {:create f/weld-create :set f/weld-set :read read-weld
               :params weld-params :max-motor nil}})

(defn- joint-kind [type spec]
  (or (joint-kinds type)
      (throw (ex-info (str "unknown joint :type (so far: " (vec (sort (keys joint-kinds))) ")")
                      {:spec spec}))))

(defn- create-joint!
  "Makes a joint between body ids a and b from its spec and local frame, and
  records it. Returns the id."
  [w joint-name a b frame {:keys [type user-data] :or {user-data 0} :as spec}]
  (let [{:keys [create params]} (joint-kind type spec)
        id (create (:id w) a b (params-buffer w (params spec frame)) (long user-data))]
    (when (zero? id)
      (throw (ex-info "Box2D couldn't create the joint" {:name joint-name :spec spec})))
    (swap! (:registry w) c/add-entry :joint id
           (cond-> {:type type :spec spec :frame frame :a a :b b}
             joint-name (assoc :name joint-name)))
    id))

(declare destroy-joint!)

(defn joint!
  "Joins two bodies and returns the joint id. With a keyword name, an
  existing joint of that name is replaced. Anchors and axes are world
  space, taken where the bodies are now. Every type takes :a, :b,
  :collide-connected? (default false) and :user-data. Springs are
  {:hertz h :damping z}; Box2D's springs lose a little each swing even at
  :damping 0.

  :prismatic  b slides along an axis fixed in a, keeping its angle.
              :anchor [x y] (or :anchor-a / :anchor-b; default b's origin)
              :axis [x y] (default [1 0]), :spring (pulls toward :target),
              :target m, :limit [lower upper] m, :motor {:max-force N :speed m/s}.
              Translation is measured along the axis from where b started.
  :revolute   a hinge: b turns about :anchor (default b's origin).
              :spring (pulls toward :target), :target radians,
              :limit [lower upper] radians (within ±0.99π),
              :motor {:max-torque N·m :speed rad/s}. Angles are measured from
              how the bodies are turned when it's made (or :reference-angle).
  :distance   keeps :anchor-a on a and :anchor-b on b (default: their
              origins) :length apart (default: as far apart as they are).
              Rigid unless it has a :spring; :limit [min max] m bounds it
              either way; :motor {:max-force N :speed m/s}.
  :weld       holds b fixed to a as they are now (anchor: :anchor, default
              b's origin). Rigid by default; :linear-hertz / :angular-hertz
              above 0 let it give (stretch / bend), with :linear-damping /
              :angular-damping. Box2D can't hold a long chain of welds
              perfectly rigid: expect some sag.
  joint-forces and break-joints! measure and snap joints under load."
  ([w spec] (joint! w nil spec))
  ([w joint-name {:keys [type a b] :as spec}]
   (live! w)
   (when (and joint-name (not (keyword? joint-name)))
     (throw (ex-info "joint names are keywords" {:name joint-name})))
   (joint-kind type spec)
   (let [a (body-id w a)
         b (body-id w b)]
     (when-let [old (and joint-name (c/resolve-id @(:registry w) :joint joint-name))]
       (destroy-joint! w old))
     (create-joint! w joint-name a b (joint-frame w a b spec) spec))))

(defn- remake-joint!
  "Makes registry entry j's joint again after body old was replaced by new."
  [w {:keys [name a b frame spec]} old new]
  (create-joint! w name (if (= a old) new a) (if (= b old) new b) frame spec))

(defn destroy-joint!
  "Destroys a joint (by name or id)."
  [w name-or-id]
  (let [id (joint-id w name-or-id)]
    (swap! (:registry w) c/remove-entry :joint id)
    (check! (f/joint-destroy id) :destroy-joint! {:joint name-or-id})
    nil))

(defn- setting-calls
  "The [selector v1 v2] calls a settings map turns into, for a joint whose
  motor limit goes by max-key."
  [{:keys [spring target length limit motor
           linear-hertz angular-hertz linear-damping angular-damping]} settings max-key]
  (cond-> []
    (some? linear-hertz)    (conj [f/SET-LINEAR-HERTZ linear-hertz 0])
    (some? angular-hertz)   (conj [f/SET-ANGULAR-HERTZ angular-hertz 0])
    (some? linear-damping)  (conj [f/SET-LINEAR-DAMPING linear-damping 0])
    (some? angular-damping) (conj [f/SET-ANGULAR-DAMPING angular-damping 0])
    (contains? settings :spring)
    (into (cons [f/SET-SPRING-ON (c/flag spring) 0]
                (when (map? spring)
                  (cond-> []
                    (:hertz spring)   (conj [f/SET-SPRING-HERTZ (:hertz spring) 0])
                    (:damping spring) (conj [f/SET-SPRING-DAMPING (:damping spring) 0])))))
    (some? target) (conj [f/SET-TARGET target 0])
    (some? length) (conj [f/SET-LENGTH length 0])
    (contains? settings :limit)
    (into (cons [f/SET-LIMIT-ON (c/flag limit) 0]
                (when (vector? limit) [[f/SET-LIMITS (first limit) (second limit)]])))
    (contains? settings :motor)
    (into (cons [f/SET-MOTOR-ON (c/flag motor) 0]
                (when (map? motor)
                  (cond-> []
                    (:speed motor)        (conj [f/SET-MOTOR-SPEED (:speed motor) 0])
                    (get motor max-key)   (conj [f/SET-MAX-MOTOR (get motor max-key) 0])))))))

(defn set-joint!
  "Changes a joint's settings while the world runs, and remembers them so a
  remade joint keeps them. Any of
    {:spring {:hertz h :damping z} or false   ; either key alone is fine
     :target t                    ; :prismatic (m), :revolute (radians)
     :length m                    ; :distance rest length
     :limit [lower upper] or false
     :motor {:max-force or :max-torque, :speed} or false
     :linear-hertz :angular-hertz :linear-damping :angular-damping  ; :weld
     :collide-connected? b
     ;; where it attaches, in the bodies' local frames (moving these at
     ;; runtime is how a joint changes shape, e.g. a neck that grows):
     :local-anchor-a [x y] :local-anchor-b [x y] :local-axis [x y]
     :reference-angle rad}"
  [w name-or-id settings]
  (let [id (joint-id w name-or-id)
        {:keys [type]} (joint-entry w id :set-joint!)
        {set-fn :set :keys [max-motor]} (joint-kinds type)]
    (doseq [[what v1 v2] (setting-calls settings settings max-motor)]
      (check! (set-fn id (int what) (double v1) (double v2)) :set-joint! {:joint name-or-id}))
    (when (some? (:collide-connected? settings))
      (check! (f/joint-set-collide-connected id (int (c/flag (:collide-connected? settings))))
              :set-joint! {:joint name-or-id}))
    (let [frame (select-keys settings [:local-anchor-a :local-anchor-b :local-axis :reference-angle])]
      (doseq [[k v] frame]
        (let [[what x y] (case k
                           :local-anchor-a  [f/FRAME-ANCHOR-A (first v) (second v)]
                           :local-anchor-b  [f/FRAME-ANCHOR-B (first v) (second v)]
                           :local-axis      [f/FRAME-AXIS-A (first v) (second v)]
                           :reference-angle [f/FRAME-REFERENCE v 0])]
          (check! (f/joint-set-frame id (int what) (double x) (double y))
                  :set-joint! {:joint name-or-id k v})))
      ;; the registry's frame is what debug-shapes, joint-info and a remade
      ;; joint use, so keep it in step
      (when (seq frame)
        (swap! (:registry w) update-in [:entries :joint id :frame] merge frame)))
    ;; remember it: merge maps (a partial :spring keeps the other key)
    (swap! (:registry w) update-in [:entries :joint id :spec]
           (fn [s] (merge-with (fn [old new] (if (and (map? old) (map? new)) (merge old new) new))
                               s (dissoc settings :local-anchor-a :local-anchor-b
                                         :local-axis :reference-angle))))
    nil))

(defn- read-joint-common [w id j]
  (let [[_ fx fy torque lin ang] (read-buf w id f/joint-read f/JOINT-DOUBLES :joint j)]
    {:force [fx fy] :torque torque :linear-separation lin :angular-separation ang}))

(defn joint-state
  "A joint's live state:
    :prismatic {:translation m :speed m/s :motor-force N}
    :revolute  {:angle rad :speed rad/s :motor-torque N·m}
    :distance  {:length m (now) :motor-force N}
  plus for every joint {:force [fx fy] :torque N·m} (what the joint is doing
  to hold the bodies together) and how far apart it's being pulled
  (:linear-separation, :angular-separation)."
  [w name-or-id]
  (let [id (joint-id w name-or-id)
        {:keys [type]} (joint-entry w id :joint-state)]
    (merge (:state ((:read (joint-kinds type)) w id name-or-id))
           (read-joint-common w id name-or-id))))

(defn- joint-anchors
  "World positions of a joint's two anchors."
  [w {:keys [a b frame]}]
  [(local->world (transform w a) (:local-anchor-a frame))
   (local->world (transform w b) (:local-anchor-b frame))])

(defn joints
  "Every joint in w: a vector of {:id :name :type :a :b}, in creation order."
  [w]
  (live! w)
  (mapv (fn [[id j]] {:id id :name (:name j) :type (:type j) :a (:a j) :b (:b j)})
        (c/entries @(:registry w) :joint)))

(defn joint-info
  "Everything about one joint as plain data: its spec, the local frame it
  was made with, the bodies (ids and names), what Box2D says its settings
  are now, its live state, and its anchors in world space."
  [w name-or-id]
  (let [id  (joint-id w name-or-id)
        {:keys [type a b] :as j} (joint-entry w id :joint-info)
        reg @(:registry w)]
    {:id       id
     :name     (:name j)
     :type     type
     :a        a
     :b        b
     :a-name   (:name (c/entry reg :body a))
     :b-name   (:name (c/entry reg :body b))
     :spec     (:spec j)
     :frame    (:frame j)
     :settings (:settings ((:read (joint-kinds type)) w id name-or-id))
     :state    (joint-state w id)
     :anchors  (joint-anchors w j)}))

;; Clojure for now: one read per joint. A wall of many welds may want this as
;; a single shim call (guide §4.1); the results would be the same.
(defn joint-forces
  "How hard each joint is working to hold its bodies together, as of the last
  step: [{:id :force N :fx :fy :torque N·m} ...] in the order given (:force
  is the magnitude of the constraint force, :fx :fy its world components)."
  [w names-or-ids]
  (live! w)
  (mapv (fn [j]
          (let [id (joint-id w j)
                {[fx fy] :force :keys [torque]} (read-joint-common w id j)]
            {:id id :force (Math/sqrt (+ (* fx fx) (* fy fy))) :fx fx :fy fy :torque torque}))
        names-or-ids))

(defn break-joints!
  "Destroys the joints among names-or-ids that are over their limit, as of
  the last step, and returns the ids of the ones it broke. The rule is either
  {:max-force N :max-torque N·m} (either may be left out) or a predicate
  given each joint-forces map ({:id :force :fx :fy :torque}), e.g. to break on
  the load above what a joint carries at rest. Joints already gone are
  skipped."
  [w names-or-ids rule]
  (let [reg   @(:registry w)
        alive (filterv #(some->> (c/resolve-id reg :joint %) (c/entry reg :joint)) names-or-ids)
        loads (joint-forces w alive)
        over? (if (fn? rule)
                rule
                (let [{:keys [max-force max-torque]} rule]
                  (fn [{:keys [force torque]}]
                    (or (and max-force (> force max-force))
                        (and max-torque (> (Math/abs (double torque)) max-torque))))))
        broken (into [] (comp (filter over?) (map :id)) loads)]
    (run! #(destroy-joint! w %) broken)
    broken))

;; --- inspection --------------------------------------------------------------

(defn awake?
  "Is a body awake? Cheaper than body-info. A sleeping body's joints report
  no force, so check this before judging joint-forces."
  [w body]
  (first (awake-and-type w (body-id w body))))

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
       :contacts  (contacts w id)
       :joints    (mapv first (attached-joints @(:registry w) id))})))

(defn snapshot
  "The whole world as one plain map."
  [w]
  (live! w)
  {:id        (:id w)
   :gravity   (:gravity w)
   :max-steps (:max-steps w)
   :bodies    (mapv #(body-info w (:id %)) (bodies w))
   :joints    (mapv #(joint-info w (:id %)) (joints w))})

;; --- debug outlines ----------------------------------------------------------
;; Built in Clojure from the registry's shape specs and each body's transform,
;; so it can change live at the REPL. Box2D's own b2DebugDraw passes structs by
;; value to its callbacks, which jolt can't receive; if this gets slow with
;; many bodies, the shim can collect those callbacks into a buffer instead and
;; this keeps returning the same maps (guide, step 2).

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
  "Every shape and joint in w as plain data for drawing, in world units.
  Shapes: {:kind ... :center [x y] :body id :name name :shape id
           :type :static|:kinematic|:dynamic :awake? bool :sensor? bool} with
    :kind :polygon  :points [[x y] ...]   (draw as a closed loop)
    :kind :circle   :at [x y] :radius r
    :kind :capsule  :points [[x1 y1] [x2 y2]] :radius r
  Joints: {:kind :segment :points [anchor-a anchor-b] :joint id :name :type}.
  No drawing happens here: the caller (e.g. raylib on the main thread) does
  that."
  [w]
  (live! w)
  (let [reg @(:registry w)
        shape-maps
        (fn [[id {body-name :name :keys [spec shapes]}]]
          (let [{:keys [x y angle] :as t} (transform w id)
                [awake? type]             (awake-and-type w id)]
            (mapv (fn [{:keys [box circle capsule radius offset] :or {offset [0 0]} :as shape-spec}
                       sid]
                    (let [at #(local->world t (mapv + offset %))]
                      (cond-> {:body id :name body-name :shape sid
                               :type type :awake? awake? :center [x y]
                               :sensor? (boolean (:sensor? shape-spec))}
                        box     (assoc :kind :polygon :points (box-points shape-spec x y angle))
                        circle  (assoc :kind :circle :at (at [0 0]) :radius circle)
                        capsule (assoc :kind :capsule :points (mapv at capsule) :radius radius))))
                  (:shapes spec) shapes)))
        joint-map
        (fn [[id j]]
          {:kind :segment :points (joint-anchors w j) :joint id :name (:name j) :type (:type j)})]
    (-> []
        (into (mapcat shape-maps) (c/entries reg :body))
        (into (map joint-map) (c/entries reg :joint)))))

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
