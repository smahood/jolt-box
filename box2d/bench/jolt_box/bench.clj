(ns jolt-box.bench
  "Micro-benchmarks for jolt-box.box2d, run with `jolt bench`. Each case is
  timed in batches after a warm-up; the report is the median batch, per call
  and per body. Numbers are for comparing variants on one machine, not
  absolutes."
  (:require [jolt.ffi :as ffi]
            [jolt-box.box2d :as b2]
            [jolt-box.box2d.ffi :as f]))

(def ^:private batch-ns
  "Roughly how long one timed batch runs."
  50e6)

(defn- time-batch
  "Nanoseconds per call of f over n calls."
  [f n]
  (let [t0 (System/nanoTime)]
    (dotimes [_ n] (f))
    (/ (double (- (System/nanoTime) t0)) n)))

(defn- measure
  "Median ns per call of f: warm up, size a batch to ~batch-ns, then time
  `batches` of them."
  [f & {:keys [batches] :or {batches 9}}]
  (dotimes [_ 20] (f))
  (let [once (max 1.0 (time-batch f 20))
        n    (max 1 (long (/ batch-ns once)))
        runs (sort (vec (repeatedly batches #(time-batch f n))))]
    (nth runs (quot batches 2))))

(defn- world-with-bodies
  "A zero-gravity world with n awake dynamic boxes, never sleeping, stepped
  once so every body has moved."
  [n]
  (let [w   (b2/make-world {:gravity [0 0]})
        ids (mapv (fn [i]
                    (b2/body! w {:type :dynamic :pos [(* 2.0 (mod i 100)) (* 2.0 (quot i 100))]
                                 :enable-sleep? false :shapes [{:box [0.5 0.5]}]}))
                  (range n))]
    (doseq [id ids] (b2/set-velocity! w id [1 0]))
    (b2/step! w b2/STEP)
    [w ids]))

;; --- read-bodies variants ----------------------------------------------------

(defn- per-body-reads
  "read-bodies as it was before the bulk call: one jb2_body_read and six
  ffi/reads per body. Kept as the baseline."
  [w ids]
  (let [buf (:state (:scratch w))]
    (fn []
      (into [] (mapcat (fn [id]
                         (f/body-read id buf)
                         (mapv #(ffi/read buf :double (* 8 %)) (range 6))))
            ids))))

(defn- calls-only
  "Only the per-body FFI calls: what read-bodies spends crossing into C."
  [w ids]
  (let [buf (:state (:scratch w))]
    (fn [] (doseq [id ids] (f/body-read id buf)))))

(defn- per-body-read-array
  "read-bodies' calls, with each body's 6 doubles copied out by one
  ffi/read-array instead of 6 ffi/reads."
  [w ids]
  (let [buf (:state (:scratch w))]
    (fn [] (into [] (mapcat (fn [id] (f/body-read id buf) (ffi/read-array buf :double 6))) ids))))

;; The bulk variants allocate their buffers once, outside the timing, as a
;; per-world buffer would.

(defn- bulk-call-only
  "One jb2_bodies_read for all bodies; ids already written, nothing copied out.
  Checks once, before timing, that the call reads every body and agrees with
  read-bodies, so a call that returned early can't pass for a fast one."
  [w ids]
  (let [n (count ids) idb (ffi/alloc (* 8 n)) out (ffi/alloc (* 8 6 n))]
    (ffi/write-array idb :uint64 (long-array ids))
    (let [got (f/bodies-read idb n out)]
      (when-not (and (= n got)
                     (= (b2/read-bodies w ids) (vec (ffi/read-array out :double (* 6 n)))))
        (throw (ex-info "bulk-call-only: the bulk call didn't read every body" {:bodies n :read got}))))
    (fn [] (f/bodies-read idb n out))))

(defn- bulk-raw
  "Write the ids, one bulk call, one read-array: a double-array, no vector."
  [_ ids]
  (let [n (count ids) idb (ffi/alloc (* 8 n)) out (ffi/alloc (* 8 6 n))]
    (fn []
      (ffi/write-array idb :uint64 (long-array ids))
      (f/bodies-read idb n out)
      (ffi/read-array out :double (* 6 n)))))

(def ^:private cases
  [[:per-body-reads      per-body-reads]
   [:read-bodies         (fn [w ids] #(b2/read-bodies w ids))]
   [:calls-only          calls-only]
   [:per-body-read-array per-body-read-array]
   [:bulk-call-only      bulk-call-only]
   [:bulk-raw            bulk-raw]])

(defn- fmt [x] (format "%10.1f" (double x)))

(defn read-bodies-bench
  "Times each case for each body count; prints a table and returns the rows."
  [& {:keys [counts] :or {counts [10 100 1000]}}]
  (println (format "%-22s %6s %10s %10s" "case" "bodies" "us/call" "ns/body"))
  (into []
        (mapcat (fn [n]
                  (let [[w ids] (world-with-bodies n)]
                    (try
                      (mapv (fn [[case-name make]]
                              (let [ns-per (measure (make w ids))]
                                (println (format "%-22s %6d %s %s" (name case-name) n
                                                 (fmt (/ ns-per 1e3)) (fmt (/ ns-per n))))
                                {:case case-name :bodies n
                                 :us-per-call (/ ns-per 1e3) :ns-per-body (/ ns-per n)}))
                            cases)
                      (finally (b2/destroy! w))))))
        counts))

(defn -main [& _]
  (when-not (b2/available?)
    (throw (ex-info "libjoltbox2d isn't loaded; run `jolt native` first" {})))
  (println "jolt" (or (System/getProperty "jolt.version") "(version unknown)"))
  (read-bodies-bench)
  nil)
