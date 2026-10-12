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

(defrecord World [id gravity alive registry scratch acc max-steps])

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
  "A new single-threaded Box2D world.
  Options: :gravity [gx gy] (default [0 9.8], y-down), :max-steps (the most
  steps one step-fixed! call will run, default 5)."
  ([] (make-world {}))
  ([{:keys [gravity max-steps] :or {gravity [0 9.8] max-steps 5}}]
   (let [[gx gy] gravity
         id (f/world-create (double gx) (double gy))]
     (when (zero? id)
       (throw (ex-info "Box2D couldn't create a world (too many worlds?)" {:gravity gravity})))
     (->World id gravity (atom true) (atom c/empty-registry)
              (ffi/alloc 64) (atom 0.0) max-steps))))

(defn destroy!
  "Releases world w and everything in it. Safe to call more than once."
  [w]
  (when (compare-and-set! (:alive w) true false)
    (f/world-destroy (:id w))
    (ffi/free (:scratch w))
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

(defn step!
  "Advances world w by dt seconds, using substeps sub-steps (default 4)."
  ([w dt] (step! w dt SUBSTEPS))
  ([w dt substeps]
   (live! w)
   (check! (f/world-step (:id w) (double dt) (int substeps)) :step! {:world (:id w)})
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
  [id {:keys [box offset angle density friction restitution]
       :or   {offset [0 0] angle 0 density 1.0 friction 0.6 restitution 0.0}
       :as   shape}]
  (cond
    box (let [[hw hh] box
              [ox oy] offset
              sid (f/body-add-box id (double hw) (double hh) (double ox) (double oy)
                                  (double angle) (double density)
                                  (double friction) (double restitution))]
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
                       :density d :friction f :restitution r}]"
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
        buf (:scratch w)]
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
        buf (:scratch w)]
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
       :awake?    (== 1.0 awake)})))

(defn snapshot
  "The whole world as one plain map."
  [w]
  (live! w)
  {:id        (:id w)
   :gravity   (:gravity w)
   :max-steps (:max-steps w)
   :bodies    (mapv #(body-info w (:id %)) (bodies w))})

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
