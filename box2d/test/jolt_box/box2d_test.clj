(ns jolt-box.box2d-test
  (:require [clojure.test :refer [deftest is testing]]
            [jolt-box.box2d :as b2]))

(defn- floor! [w]
  ;; top surface at y = 1.9 (y-down)
  (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]}))

(defn- crate! [w & {:keys [pos] :or {pos [0 0]}}]
  (b2/body! w :crate {:type :dynamic :pos pos :shapes [{:box [0.5 0.5]}]}))

(defn- thrown-ex-info? [f]
  (try (f) false
       (catch clojure.lang.ExceptionInfo _ true)))

(defn- plain?
  "True if x is only plain data: no atoms, records or pointers."
  [x]
  (cond (record? x) false
        (map? x)    (and (every? plain? (keys x)) (every? plain? (vals x)))
        (vector? x) (every? plain? x)
        :else       (or (nil? x) (number? x) (keyword? x) (string? x) (boolean? x))))

(deftest world-lifecycle
  (b2/with-world [w]
    (is (pos? (:id w)))
    (is (= [] (b2/bodies w))))
  (testing "destroy! is safe to repeat"
    (let [w (b2/make-world)]
      (b2/destroy! w)
      (is (nil? (b2/destroy! w))))))

(deftest crate-lands-on-floor
  (b2/with-world [w {:gravity [0 9.8]}]
    (floor! w)
    (crate! w)
    (let [samples (b2/simulate w 3.0 {:sample [:crate] :every 0.05})
          rest-y  (get-in (peek samples) [:crate :y])]
      (is (< (Math/abs (- rest-y 1.4)) 0.01) (str "rests on the floor, y = " rest-y))
      (is (some? (b2/settled-at samples :crate)) "comes to rest")
      (is (< (b2/settled-at samples :crate) 2.0) "within two seconds"))))

(deftest gravity-direction-is-the-callers-choice
  (b2/with-world [down {:gravity [0 9.8]}]
    (b2/with-world [up {:gravity [0 -9.8]}]
      (let [a (b2/body! down {:type :dynamic :shapes [{:box [0.1 0.1]}]})
            b (b2/body! up   {:type :dynamic :shapes [{:box [0.1 0.1]}]})]
        (testing "two worlds stepped alternately don't affect each other"
          (dotimes [_ 30] (b2/step! down b2/STEP) (b2/step! up b2/STEP))
          (is (pos? (:y (b2/transform down a))))
          (is (neg? (:y (b2/transform up b))))
          (is (< (Math/abs (+ (:y (b2/transform down a)) (:y (b2/transform up b)))) 1e-6)
              "mirror images"))))))

(deftest named-bodies-replace
  (b2/with-world [w]
    (let [old (crate! w)
          new (crate! w :pos [3 0])]
      (is (not= old new))
      (is (= 1 (count (b2/bodies w))) "re-evaluating leaves one :crate")
      (is (= new (:id (first (b2/bodies w)))))
      (is (= 3.0 (:x (b2/transform w :crate))) "the name points at the new body")
      (is (thrown-ex-info? #(b2/transform w old)) "the old id is gone"))))

(deftest unnamed-bodies
  (b2/with-world [w]
    (let [a (b2/body! w {:type :dynamic :shapes [{:box [0.1 0.1]}]})
          b (b2/body! w {:type :dynamic :shapes [{:box [0.1 0.1]}]})]
      (is (= [a b] (mapv :id (b2/bodies w))))
      (is (every? nil? (map :name (b2/bodies w)))))))

(deftest errors-throw-instead-of-crashing
  (b2/with-world [w]
    (testing "a destroyed body"
      (let [id (crate! w)]
        (b2/destroy-body! w :crate)
        (is (thrown-ex-info? #(b2/transform w id)))
        (is (thrown-ex-info? #(b2/destroy-body! w id)))))
    (testing "an unknown name"
      (is (thrown-ex-info? #(b2/transform w :nobody))))
    (testing "bad specs"
      (is (thrown-ex-info? #(b2/body! w {:type :wobbly})))
      (is (thrown-ex-info? #(b2/body! w {:shapes [{:sphere 1}]})))
      (is (thrown-ex-info? #(b2/body! w :bad {:shapes [{:box [0 1]}]})))
      (is (= [] (b2/bodies w)) "a rejected shape leaves no body behind")))
  (testing "a world used after destroy!"
    (let [w  (b2/make-world)
          id (crate! w)]
      (b2/destroy! w)
      (is (thrown-ex-info? #(b2/transform w id)))
      (is (thrown-ex-info? #(b2/step! w b2/STEP)))
      (is (thrown-ex-info? #(crate! w))))))

(deftest with-world-destroys-on-throw
  (let [held (atom nil)]
    (is (thrown-ex-info? #(b2/with-world [w]
                            (reset! held w)
                            (throw (ex-info "boom" {})))))
    (is (false? @(:alive @held)))
    (is (thrown-ex-info? #(b2/bodies @held)))))

(deftest step-fixed-caps-catch-up
  (b2/with-world [w {:gravity [0 9.8] :max-steps 5}]
    (let [b (b2/body! w {:type :dynamic :shapes [{:box [0.1 0.1]}]})]
      (testing "a 30 second frame runs only :max-steps steps"
        (let [alpha (b2/step-fixed! w 30.0)]
          (is (<= 0.0 alpha 1.0))
          (is (< (Math/abs (- (:vy (b2/velocity w b)) (* 9.8 5 b2/STEP))) 1e-4))))
      (testing "after which normal frames step normally"
        (let [vy0 (:vy (b2/velocity w b))]
          (dotimes [_ 60] (b2/step-fixed! w b2/STEP))
          (is (< (Math/abs (- (:vy (b2/velocity w b)) vy0 9.8)) 0.2)
              "one second of frames adds about one second of fall speed"))))))

(deftest inspection-is-plain-data
  (b2/with-world [w]
    (floor! w)
    (b2/body! w :crate {:type :dynamic :user-data 42 :shapes [{:box [0.5 0.5] :density 2}]})
    (let [info (b2/body-info w :crate)]
      (is (plain? info))
      (is (= :crate (:name info)))
      (is (= :dynamic (:type info)))
      (is (= 42 (:user-data info)))
      (is (= 1 (count (:shapes info))))
      (is (< (Math/abs (- (:mass info) 2.0)) 1e-4) "1 m² at density 2")
      (is (:awake? info)))
    (let [snap (b2/snapshot w)]
      (is (plain? snap))
      (is (= [:floor :crate] (mapv :name (:bodies snap)))))))

(deftest sample-helpers
  (let [samples (mapv (fn [i] (let [t (* i 0.05)]
                                {:t t :b {:y (Math/sin (* 2 Math/PI t)) :vx 0.0 :vy 0.0
                                          :w (if (< t 1.0) 1.0 0.0)}}))
                      (range 61))]
    (is (< (Math/abs (- (b2/period samples :b :y) 1.0)) 0.06) "a 1 Hz sine has a ~1 s period")
    (is (< (Math/abs (- (:max (b2/extent samples :b :y)) 1.0)) 0.01))
    (is (= 1.0 (b2/settled-at samples :b)))
    (is (nil? (b2/period (subvec samples 0 10) :b :y)))))

;; --- step 1: moving bodies and contacts ---------------------------------------

(defn- near? [a b tol] (< (Math/abs (- (double a) (double b))) tol))

(defn- settle! [w] (b2/simulate w 2.0))

(deftest contacts-on-a-resting-body
  (b2/with-world [w]
    (floor! w)
    (crate! w)
    (settle! w)
    (let [cs (b2/contacts w :crate)
          {:keys [other-name normal touching? points]} (first cs)]
      (is (= 1 (count cs)))
      (is (= :floor other-name))
      (is touching?)
      (is (= 2 points) "a box flat on a floor touches at two points")
      (is (near? (first normal) 0.0 1e-3))
      (is (near? (second normal) -1.0 1e-3) "points from the floor up into the crate (y-down)"))
    (is (near? (second (:normal (first (b2/contacts w :floor)))) 1.0 1e-3)
        "seen from the floor, the normal points down into the floor")
    (is (= 1 (count (:contacts (b2/body-info w :crate)))))))

(deftest impulse-hop-and-grounded
  (b2/with-world [w]
    (floor! w)
    (crate! w)
    (settle! w)
    (is (b2/grounded? w :crate))
    (let [m (:mass (b2/body-info w :crate))]
      (b2/impulse! w :crate [0 (* m -4.0)])     ; 4 m/s upward
      (is (near? (:vy (b2/velocity w :crate)) -4.0 1e-4) "an impulse is a change in momentum")
      (let [samples (b2/simulate w 2.0 {:sample [:crate] :every b2/STEP})
            top     (:min (b2/extent samples :crate :y))]
        ;; v²/2g = 16/19.6 ≈ 0.82 m above the 1.4 m rest height
        (is (near? (- 1.4 top) 0.816 0.03) (str "rises about 0.82 m, rose " (- 1.4 top)))
        (is (b2/grounded? w :crate) "lands again")))
    (testing "not grounded the step after pushing off"
      ;; the contact data still shows the floor for one step
      (b2/impulse! w :crate [0 -4.0])
      (b2/step! w b2/STEP)
      (is (some :touching? (b2/contacts w :crate)) "the floor contact lingers")
      (is (not (b2/grounded? w :crate)) "but it's moving away, so it isn't ground")
      (b2/simulate w 2.0))
    (testing "not grounded in mid-air"
      (b2/impulse! w :crate [0 -4.0])
      (b2/simulate w 0.2)
      (is (not (b2/grounded? w :crate))))))

(deftest grounded-follows-gravity
  (testing "y-up world"
    (b2/with-world [w {:gravity [0 -9.8]}]
      (b2/body! w :floor {:pos [0 -2] :shapes [{:box [10 0.1]}]})
      (crate! w)
      (settle! w)
      (is (near? (:y (b2/transform w :crate)) -1.4 0.01))
      (is (b2/grounded? w :crate))))
  (testing "a wall beside a falling body isn't ground"
    (b2/with-world [w]
      (b2/body! w :wall {:pos [0.6 0] :shapes [{:box [0.1 5]}]})
      (crate! w)
      (b2/set-velocity! w :crate [1 0])          ; pressed against the wall
      (b2/simulate w 0.1)
      (is (some :touching? (b2/contacts w :crate)))
      (is (not (b2/grounded? w :crate))))))

(deftest setters
  (b2/with-world [w]
    (let [b (b2/body! w {:type :dynamic :shapes [{:box [0.5 0.5]}]})]
      (testing "set-velocity! and set-angular-velocity!"
        (b2/set-velocity! w b [1 2])
        (b2/set-angular-velocity! w b 3)
        (is (= {:vx 1.0 :vy 2.0 :w 3.0} (b2/velocity w b))))
      (testing "set-transform! teleports"
        (b2/set-transform! w b [5 6] 0.5)
        (let [{:keys [x y angle]} (b2/transform w b)]
          (is (near? x 5 1e-6)) (is (near? y 6 1e-6)) (is (near? angle 0.5 1e-6)))
        (b2/set-transform! w b [7 8])
        (is (near? (:angle (b2/transform w b)) 0.5 1e-6) "keeps the angle"))
      (testing "gravity scale 0 floats"
        (b2/set-velocity! w b [0 0])
        (b2/set-angular-velocity! w b 0)
        (b2/set-gravity-scale! w b 0)
        (b2/simulate w 1.0)
        (is (near? (:y (b2/transform w b)) 8 1e-6)))
      (testing "a steady force accelerates (F = ma)"
        (let [m (:mass (b2/body-info w b))]
          (dotimes [_ 60] (b2/force! w b [(* 2 m) 0]) (b2/step! w b2/STEP))
          (is (near? (:vx (b2/velocity w b)) 2.0 0.01))))
      (testing "damping slows it"
        (b2/set-damping! w b {:linear 5.0})
        (b2/simulate w 1.0)
        (is (< (:vx (b2/velocity w b)) 0.1)))
      (testing "an off-center impulse spins it"
        (let [{:keys [x y]} (b2/transform w b)]
          (b2/set-angular-velocity! w b 0)
          (b2/impulse! w b [0 1] [(+ x 0.5) y])
          (is (pos? (:w (b2/velocity w b))))))
      (testing "set-awake!"
        (b2/set-awake! w b false)
        (is (false? (:awake? (b2/body-info w b))))
        (b2/set-awake! w b true)
        (is (:awake? (b2/body-info w b)))))))

(deftest motion-on-stale-ids-throws
  (b2/with-world [w]
    (let [b (crate! w)]
      (b2/destroy-body! w b)
      (doseq [f [#(b2/impulse! w b [0 1]) #(b2/force! w b [0 1]) #(b2/set-velocity! w b [0 0])
                 #(b2/set-transform! w b [0 0] 0) #(b2/contacts w b) #(b2/grounded? w b)]]
        (is (thrown-ex-info? f))))))

;; --- step 2: debug outlines -----------------------------------------------------

(defn- points-near? [as bs tol]
  (and (= (count as) (count bs))
       (every? true? (map (fn [[ax ay] [bx by]] (and (near? ax bx tol) (near? ay by tol))) as bs))))

(deftest debug-shapes-boxes
  (b2/with-world [w]
    (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})
    (b2/body! w :crate {:type :dynamic :pos [1 0] :shapes [{:box [0.5 0.25]}]})
    (let [shapes (b2/debug-shapes w)
          by-name (into {} (map (juxt :name identity)) shapes)]
      (is (plain? shapes))
      (is (= 2 (count shapes)))
      (is (points-near? (:points (by-name :floor)) [[-10 1.9] [10 1.9] [10 2.1] [-10 2.1]] 1e-6))
      (is (points-near? (:points (by-name :crate)) [[0.5 -0.25] [1.5 -0.25] [1.5 0.25] [0.5 0.25]] 1e-6))
      (is (= {:kind :polygon :type :static :awake? false}
             (select-keys (by-name :floor) [:kind :type :awake?])))
      (is (= {:type :dynamic :awake? true} (select-keys (by-name :crate) [:type :awake?])))
      (is (= (first (:shapes (b2/body-info w :crate))) (:shape (by-name :crate))))))
  (testing "turned bodies and offset, turned shapes"
    (b2/with-world [w]
      (let [half-pi (/ Math/PI 2)]
        (b2/body! w :b {:pos [1 1] :angle half-pi
                        :shapes [{:box [0.5 0.1] :offset [1 0]}]})
        ;; the shape's centre is 1 m along the body's x axis, which now points along +y
        (is (points-near? (:points (first (b2/debug-shapes w)))
                          [[1.1 1.5] [1.1 2.5] [0.9 2.5] [0.9 1.5]] 1e-5))
        (b2/body! w :b {:pos [0 0] :shapes [{:box [0.5 0.1] :angle half-pi}]})
        (is (points-near? (:points (first (b2/debug-shapes w)))
                          [[0.1 -0.5] [0.1 0.5] [-0.1 0.5] [-0.1 -0.5]] 1e-5)))))
  (testing "outlines follow the body and show sleep"
    (b2/with-world [w]
      (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})
      (crate! w)
      (b2/simulate w 3.0)
      (let [crate (first (filter #(= :crate (:name %)) (b2/debug-shapes w)))]
        (is (near? (second (:center crate)) 1.4 0.01))
        (is (false? (:awake? crate)) "asleep once settled")))))

;; --- step 3: contact events, mass and momentum ----------------------------------

(defn- drop-crate!
  "A crate whose bottom starts `height` m above the floor's top (1.9)."
  [w height & {:as shape-opts}]
  (b2/body! w :crate {:type :dynamic :pos [0 (- 1.4 height)] :user-data 7
                      :shapes [(merge {:box [0.5 0.5]} shape-opts)]}))

(deftest hit-events-measure-impact-speed
  (let [hit-speed (fn [height]
                    (b2/with-world [w]
                      (floor! w)
                      (drop-crate! w height :hit-events? true)
                      (b2/simulate w 2.0)
                      (:speed (first (:hit (b2/contact-events w))))))
        low  (hit-speed 0.5)
        high (hit-speed 2.0)]
    ;; v = sqrt(2gh)
    (is (near? low (Math/sqrt (* 2 9.8 0.5)) 0.2) (str "0.5 m drop hits at ~3.1 m/s, got " low))
    (is (near? high (Math/sqrt (* 2 9.8 2.0)) 0.3) (str "2 m drop hits at ~6.3 m/s, got " high))
    (is (> high low))))

(deftest begin-and-end-events
  (b2/with-world [w]
    (floor! w)
    (drop-crate! w 0.5 :contact-events? true)
    (b2/simulate w 1.5)
    (let [{:keys [begin end hit]} (b2/contact-events w)
          e (first begin)]
      (is (= 1 (count begin)) "landing starts one contact")
      (is (= [] end))
      (is (= [] hit) "hit events weren't asked for")
      (is (= #{:floor :crate} (set [(:name-a e) (:name-b e)])))
      (is (= #{0 7} (set [(:user-a e) (:user-b e)])) "user data rides along")
      (is (= 2 (:points e))))
    (is (= {:begin [] :end [] :hit [] :dropped 0} (b2/contact-events w))
        "contact-events clears what it returns")
    (b2/impulse! w :crate [0 -5])
    (b2/simulate w 0.3)
    (is (= 1 (count (:end (b2/contact-events w)))) "jumping off ends it")))

(deftest events-for-turns-events-around
  (b2/with-world [w]
    (floor! w)
    (drop-crate! w 0.5 :hit-events? true)
    (b2/simulate w 1.5)
    (let [hits (:hit (b2/contact-events w))
          [crate-view] (b2/events-for w hits :crate)
          [floor-view] (b2/events-for w hits :floor)]
      (is (= :floor (:other-name crate-view)))
      (is (= 0 (:other-user crate-view)))
      (is (near? (second (:normal crate-view)) -1.0 1e-3) "from the floor up into the crate")
      (is (= :crate (:other-name floor-view)))
      (is (= 7 (:other-user floor-view)))
      (is (near? (second (:normal floor-view)) 1.0 1e-3) "from the crate down into the floor")
      (is (= [] (b2/events-for w hits (b2/body! w {:shapes [{:box [1 1]}]})))))))

(deftest events-collect-across-steps
  (testing "step-fixed! running several steps in one call loses nothing"
    (b2/with-world [w {:max-steps 100}]
      (floor! w)
      (drop-crate! w 0.5 :hit-events? true :contact-events? true)
      (b2/step-fixed! w 1.0)                           ; 60 steps, the landing in the middle
      (let [{:keys [begin hit]} (b2/contact-events w)]
        (is (= 1 (count begin)))
        (is (= 1 (count hit))))))
  (testing "with :collect-events? false, only the last step's events"
    (b2/with-world [w {:collect-events? false :max-steps 100}]
      (floor! w)
      (drop-crate! w 0.5 :hit-events? true)
      (b2/step-fixed! w 1.0)
      (is (= [] (:hit (b2/contact-events w))) "the landing was steps ago")))
  (testing "the hit threshold"
    (b2/with-world [w {:hit-event-threshold 4.0}]
      (floor! w)
      (drop-crate! w 0.5 :hit-events? true)          ; lands at ~3.1 m/s
      (b2/simulate w 1.5)
      (is (= [] (:hit (b2/contact-events w)))))))

(deftest mass-and-momentum
  (b2/with-world [w]
    (b2/body! w :b {:type :dynamic :pos [1 2]
                    :shapes [{:box [0.5 0.5] :offset [1 0] :density 2}]})
    (let [{:keys [mass center local-center inertia]} (b2/mass-data w :b)]
      (is (near? mass 2.0 1e-4))
      (is (= 2.0 (b2/mass w :b)))
      (is (pos? inertia))
      (is (near? (first local-center) 1.0 1e-5) "the centre of mass is the shape's centre")
      (is (near? (first center) 2.0 1e-5))
      (is (near? (second center) 2.0 1e-5)))
    (b2/set-velocity! w :b [3 -1])
    (is (= {:px 6.0 :py -2.0} (b2/momentum w :b)))
    (is (= 0.0 (b2/mass w (b2/body! w {:shapes [{:box [1 1]}]}))) "static bodies have no mass")))

(deftest shape-settings
  (b2/with-world [w]
    (floor! w)
    (drop-crate! w 1.0)
    (let [[s] (b2/shapes w :crate)]
      (is (= (:id (b2/body-info w :crate)) (b2/shape-body w s)))
      (testing "density changes the mass"
        (b2/set-shape! w s {:density 3})
        (is (near? (b2/mass w :crate) 3.0 1e-4)))
      (testing "restitution makes it bounce"
        (b2/set-shape! w s {:restitution 0.8})
        (b2/set-shape! w (first (b2/shapes w :floor)) {:restitution 0.8})
        (let [samples (b2/simulate w 1.0 {:sample [:crate]})]
          (is (some #(neg? (get-in % [:crate :vy])) (drop 30 samples)) "goes back up")))
      (testing "hit events can be turned on later"
        (b2/contact-events w)
        (b2/set-shape! w s {:hit-events? true :restitution 0 :friction 0.2})
        (b2/set-transform! w :crate [0 0])
        (b2/simulate w 1.0)
        (is (= 1 (count (:hit (b2/contact-events w)))))))
    (testing "stale shapes throw"
      (let [[s] (b2/shapes w :crate)]
        (b2/destroy-body! w :crate)
        (is (thrown-ex-info? #(b2/set-shape! w s {:friction 1})))
        (is (thrown-ex-info? #(b2/shape-body w s)))))))

;; --- step 4: sensors ----------------------------------------------------------

(defn- footed-crate!
  "A crate with a thin foot sensor along its bottom edge."
  [w & {:keys [pos] :or {pos [0 0]}}]
  (b2/body! w :crate {:type :dynamic :pos pos :fixed-rotation? true
                      :shapes [{:box [0.5 0.5]}
                               {:box [0.45 0.03] :offset [0 0.5] :sensor? true}]}))

(defn- foot [w] (second (b2/shapes w :crate)))

(deftest foot-sensor-landing-and-take-off
  (b2/with-world [w]
    (b2/body! w :floor {:pos [0 2] :user-data 9 :shapes [{:box [10 0.1] :sensor-events? true}]})
    (footed-crate! w)
    (is (near? (b2/mass w :crate) 1.0 1e-4) "the sensor adds no mass")
    (b2/simulate w 0.2)
    (is (= [] (b2/sensor-overlaps w (foot w))) "nothing under it while falling")
    (b2/simulate w 1.5)
    (let [{:keys [begin end]} (b2/sensor-events w)
          e (first begin)]
      (is (= 1 (count begin)) "landing begins one overlap")
      (is (= [] end))
      (is (= {:sensor-name :crate :visitor-name :floor :visitor-user 9}
             (select-keys e [:sensor-name :visitor-name :visitor-user])))
      (is (= (foot w) (:sensor-shape e))))
    (is (= [:floor] (mapv :name (b2/sensor-overlaps w (foot w)))))
    (testing "taking off ends it, with no lag"
      (b2/impulse! w :crate [0 -5])
      (b2/step! w b2/STEP)
      (b2/step! w b2/STEP)
      (is (= [] (b2/sensor-overlaps w (foot w))))
      (is (= 1 (count (:end (b2/sensor-events w))))))
    (testing "and landing again begins it again"
      (b2/simulate w 2.0)
      (is (= 1 (count (:begin (b2/sensor-events w))))))))

(deftest sensors-only-see-shapes-that-ask
  (b2/with-world [w]
    (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})       ; no :sensor-events?
    (footed-crate! w)
    (b2/simulate w 2.0)
    (is (= [] (b2/sensor-overlaps w (foot w))))
    (is (= [] (:begin (b2/sensor-events w))))
    (testing "until it's turned on"
      (b2/set-shape! w (first (b2/shapes w :floor)) {:sensor-events? true})
      (b2/step! w b2/STEP)
      (is (= [:floor] (mapv :name (b2/sensor-overlaps w (foot w))))))))

(deftest sensors-dont-collide
  (b2/with-world [w]
    (floor! w)
    (b2/body! w :ghost {:type :dynamic :shapes [{:box [0.5 0.5] :sensor? true :density 1}]})
    (b2/simulate w 2.0)
    (is (> (:y (b2/transform w :ghost)) 5) "falls straight through the floor")
    (testing "a static sensor zone sees a body pass through it"
      (b2/body! w :zone {:pos [0 -3] :shapes [{:box [2 0.5] :sensor? true}]})
      (b2/body! w :ball {:type :dynamic :pos [0 -6] :shapes [{:box [0.2 0.2] :sensor-events? true}]})
      (b2/sensor-events w)
      (b2/simulate w 1.5)
      (let [{:keys [begin end]} (b2/sensor-events w)]
        (is (= [[:zone :ball]] (mapv (juxt :sensor-name :visitor-name) begin)))
        (is (= [[:zone :ball]] (mapv (juxt :sensor-name :visitor-name) end)))))))

(deftest sensor-odds-and-ends
  (b2/with-world [w]
    (footed-crate! w)
    (is (thrown-ex-info? #(b2/sensor-overlaps w (first (b2/shapes w :crate))))
        "overlaps of a shape that isn't a sensor")
    (is (= [false true] (mapv :sensor? (b2/debug-shapes w))))))

;; --- step 5: circles, capsules and prismatic joints -----------------------------

(deftest circles-and-capsules
  (b2/with-world [w]
    (floor! w)
    (b2/body! w :ball {:type :dynamic :pos [-2 0] :shapes [{:circle 0.25}]})
    (b2/body! w :pill {:type :dynamic :pos [2 0] :fixed-rotation? true
                       :shapes [{:capsule [[-0.5 0] [0.5 0]] :radius 0.2}]})
    (is (near? (b2/mass w :ball) (* Math/PI 0.0625) 1e-3) "a circle's mass is πr² × density")
    (b2/simulate w 2.0)
    (is (near? (:y (b2/transform w :ball)) 1.65 0.01) "rests a radius above the floor")
    (is (near? (:y (b2/transform w :pill)) 1.7 0.01))
    (let [by-name (into {} (map (juxt :name identity)) (b2/debug-shapes w))]
      (is (= :circle (:kind (by-name :ball))))
      (is (near? 0.25 (:radius (by-name :ball)) 1e-9))
      (is (= :capsule (:kind (by-name :pill))))
      (is (points-near? [[1.5 1.7] [2.5 1.7]] (:points (by-name :pill)) 0.01)))
    (is (thrown-ex-info? #(b2/body! w {:shapes [{:circle 0}]})))
    (is (thrown-ex-info? #(b2/body! w {:shapes [{:capsule [[0 0] [0 0]] :radius 0.1}]})))))

(defn- slider!
  "A static post with a 1 kg block on a vertical prismatic joint, in a world
  without gravity unless one is given."
  [w spec]
  (b2/body! w :post {:shapes [{:box [0.1 0.1]}]})
  (b2/body! w :block {:type :dynamic :pos [0 0] :shapes [{:box [0.5 0.5]}]})
  (b2/joint! w :slide (merge {:type :prismatic :a :post :b :block :axis [0 1]} spec)))

(deftest prismatic-spring-oscillates-at-its-hertz
  (b2/with-world [w {:gravity [0 0]}]
    (slider! w {:spring {:hertz 2.0 :damping 0.0} :target 0.5})
    (let [samples (b2/simulate w 3.0 {:sample [:block]})
          period  (b2/period samples :block :y)
          {:keys [min max]} (b2/extent samples :block :y)]
      (is (near? period 0.5 0.05) (str "2 Hz is a 0.5 s period, got " period))
      (is (near? min 0.0 0.02))
      ;; Box2D's soft-step spring loses ~15% a cycle at 2 Hz even undamped
      (is (< 0.85 max 1.0) "swings out to nearly twice the target")
      (is (every? #(near? (get-in % [:block :x]) 0.0 1e-6) samples) "only along the axis"))))

(deftest prismatic-limits-motor-and-settings
  (testing "limits hold"
    (b2/with-world [w {:gravity [0 0]}]
      (slider! w {:spring {:hertz 3 :damping 0} :target 2.0 :limit [-0.2 0.3]})
      (let [samples (b2/simulate w 2.0 {:sample [:block]})]
        (is (<= (:max (b2/extent samples :block :y)) 0.31))
        (is (near? (:translation (b2/joint-state w :slide)) 0.3 0.01)))))
  (testing "a motor drives it at its speed"
    (b2/with-world [w {:gravity [0 0]}]
      (slider! w {:motor {:max-force 1000 :speed 1.5}})
      (b2/simulate w 0.5)
      (is (near? (:speed (b2/joint-state w :slide)) 1.5 1e-3))
      (is (near? (:vy (b2/velocity w :block)) 1.5 1e-3))))
  (testing "set-joint! changes it live and remembers"
    (b2/with-world [w {:gravity [0 0]}]
      (slider! w {:spring {:hertz 2 :damping 0.1}})
      (b2/set-joint! w :slide {:spring {:hertz 5} :target 0.25 :limit [-1 1]})
      (let [{:keys [settings spec]} (b2/joint-info w :slide)]
        (is (near? 5.0 (get-in settings [:spring :hertz]) 1e-6))
        (is (near? 0.1 (get-in settings [:spring :damping]) 1e-6) "the damping it had")
        (is (= 0.25 (double (:target settings))))
        (is (= [-1.0 1.0] (mapv double (:limit settings))))
        (is (= {:hertz 5 :damping 0.1} (:spring spec)) "the spec remembers the merged spring"))
      (b2/simulate w 2.0)
      (is (near? (:translation (b2/joint-state w :slide)) 0.25 0.2))
      (b2/set-joint! w :slide {:spring false :limit false})
      (is (nil? (get-in (b2/joint-info w :slide) [:settings :spring])))
      (is (nil? (get-in (b2/joint-info w :slide) [:settings :limit]))))))

(deftest joint-registry
  (b2/with-world [w]
    (b2/body! w :base {:type :dynamic :pos [0 1] :shapes [{:box [0.5 0.2]}]})
    (b2/body! w :bowl {:type :dynamic :pos [0 0.4] :shapes [{:box [0.5 0.3]}]})
    (let [j (b2/joint! w :spring {:type :prismatic :a :base :b :bowl :axis [0 -1]
                                  :spring {:hertz 4 :damping 0.3}})]
      (is (= [{:id j :name :spring :type :prismatic
               :a (:id (b2/body-info w :base)) :b (:id (b2/body-info w :bowl))}]
             (b2/joints w)))
      (let [info (b2/joint-info w :spring)]
        (is (plain? info))
        (is (= [:base :bowl] [(:a-name info) (:b-name info)]))
        (is (points-near? [[0 0.4] [0 0.4]] (:anchors info) 1e-6) "anchored at :b's origin")
        (is (points-near? [[0 -0.6]] [(get-in info [:frame :local-anchor-a])] 1e-6)
            "kept in :a's own frame"))
      (is (= [j] (:joints (b2/body-info w :bowl))))
      (testing "re-evaluating a named joint replaces it"
        (let [j2 (b2/joint! w :spring {:type :prismatic :a :base :b :bowl :axis [0 -1]})]
          (is (not= j j2))
          (is (= 1 (count (b2/joints w))))
          (is (thrown-ex-info? #(b2/joint-state w j)))))
      (testing "re-evaluating a body remakes its joints on the new body"
        (b2/set-joint! w :spring {:spring {:hertz 6 :damping 0.2}})
        (let [old-bowl (:id (b2/body-info w :bowl))
              new-bowl (b2/body! w :bowl {:type :dynamic :pos [0 0.4] :shapes [{:box [0.4 0.3]}]})
              info     (b2/joint-info w :spring)]
          (is (not= old-bowl new-bowl))
          (is (= new-bowl (:b info)))
          (is (= :bowl (:b-name info)))
          (is (near? 6.0 (get-in info [:settings :spring :hertz]) 1e-6) "with the changed settings")
          (is (= [(:id info)] (:joints (b2/body-info w :bowl))))))
      (testing "destroying a body takes its joints"
        (b2/destroy-body! w :base)
        (is (= [] (b2/joints w)))
        (is (thrown-ex-info? #(b2/joint-info w :spring)))))))

(deftest joint-odds-and-ends
  (b2/with-world [w {:gravity [0 0]}]
    (slider! w {})
    (is (some #(= [:segment :slide] [(:kind %) (:name %)]) (b2/debug-shapes w)))
    (is (thrown-ex-info? #(b2/joint! w {:type :wobble :a :post :b :block})))
    (is (thrown-ex-info? #(b2/joint! w {:type :prismatic :a :post :b :nobody})))
    (is (plain? (b2/snapshot w)))
    (is (= [:slide] (mapv :name (:joints (b2/snapshot w)))))
    (b2/destroy-joint! w :slide)
    (is (thrown-ex-info? #(b2/set-joint! w :slide {:target 1})))
    (is (= [] (b2/joints w)))))

;; --- step 6: revolute and distance joints, filters, read-bodies ---------------

(defn- hinge!
  "A static post with a 1 m bar hinged at its left end, at the post."
  [w spec]
  (b2/body! w :post {:shapes [{:box [0.05 0.05]}]})
  (b2/body! w :bar {:type :dynamic :pos [0.5 0] :shapes [{:box [0.5 0.05]}]})
  (b2/joint! w :hinge (merge {:type :revolute :a :post :b :bar :anchor [0 0]} spec)))

(deftest revolute-springs-limits-and-motors
  (testing "a spring brings it back to its target"
    (b2/with-world [w {:gravity [0 0]}]
      (hinge! w {:spring {:hertz 3 :damping 0.5}})
      (b2/set-angular-velocity! w :bar 4.0)                    ; knock it
      (let [samples (b2/simulate w 3.0 {:sample [:bar]})]
        ;; ~4 rad/s against a 3 Hz spring (ω ≈ 19), damped: a small swing
        (is (< 0.05 (:max (b2/extent samples :bar :angle))) "it swung")
        (is (some? (b2/settled-at samples :bar)) "and settled")
        (is (near? (:angle (b2/joint-state w :hinge)) 0.0 0.01)))
      (b2/set-joint! w :hinge {:target 0.5})
      (b2/simulate w 3.0)
      (is (near? (:angle (b2/joint-state w :hinge)) 0.5 0.01) "and to a new target")))
  (testing "a limit stops it"
    (b2/with-world [w]                                          ; falls under gravity
      (hinge! w {:limit [-0.3 0.3]})
      (b2/simulate w 2.0)
      (is (near? (:angle (b2/joint-state w :hinge)) 0.3 0.01) "y-down: falling turns it +")
      (is (near? (:angle (b2/transform w :bar)) 0.3 0.01))))
  (testing "a motor turns it"
    (b2/with-world [w {:gravity [0 0]}]
      (hinge! w {:motor {:max-torque 100 :speed 2.0}})
      (b2/simulate w 0.5)
      (is (near? (:speed (b2/joint-state w :hinge)) 2.0 1e-3))
      (is (near? (:angle (b2/joint-state w :hinge)) 1.0 0.05))))
  (testing "it stays pinned at the anchor"
    (b2/with-world [w]
      (hinge! w {})
      (b2/simulate w 1.0)
      (let [[[ax ay] [bx by]] (:anchors (b2/joint-info w :hinge))]
        (is (near? ax bx 0.01))
        (is (near? ay by 0.01)))
      (is (nil? (get-in (b2/joint-info w :hinge) [:settings :motor])) "no motor unless asked"))))

(deftest distance-joints
  (testing "rigid: a pendulum keeps its length"
    (b2/with-world [w]
      (b2/body! w :pin {:shapes [{:box [0.05 0.05]}]})
      (b2/body! w :bob {:type :dynamic :pos [1 0] :shapes [{:circle 0.1}]})
      (b2/joint! w :rod {:type :distance :a :pin :b :bob})
      (is (near? 1.0 (get-in (b2/joint-info w :rod) [:settings :length]) 1e-6)
          "the length defaults to how far apart they are")
      (let [samples (b2/simulate w 2.0 {:sample [:bob]})]
        (is (every? (fn [{{:keys [x y]} :bob}] (near? (Math/sqrt (+ (* x x) (* y y))) 1.0 0.01))
                    samples))
        (is (> (:max (b2/extent samples :bob :y)) 0.9) "and swings down"))))
  (testing "springy, with a new length and a range"
    (b2/with-world [w {:gravity [0 0]}]
      (b2/body! w :pin {:shapes [{:box [0.05 0.05]}]})
      (b2/body! w :bob {:type :dynamic :pos [0 1] :shapes [{:circle 0.1}]})
      (b2/joint! w :leash {:type :distance :a :pin :b :bob :spring {:hertz 4 :damping 0.7}})
      (b2/set-joint! w :leash {:length 2.0})
      (b2/simulate w 2.0)
      (is (near? (:length (b2/joint-state w :leash)) 2.0 0.02) "the spring pulls it out")
      (b2/set-joint! w :leash {:limit [0.5 1.5]})
      (b2/simulate w 1.0)
      (is (<= (:length (b2/joint-state w :leash)) 1.51) "the range holds it in")
      (is (= [0.5 1.5] (mapv double (get-in (b2/joint-info w :leash) [:settings :limit]))))))
  (testing "explicit anchors"
    (b2/with-world [w {:gravity [0 0]}]
      (b2/body! w :a {:type :dynamic :pos [0 0] :shapes [{:box [0.5 0.5]}]})
      (b2/body! w :b {:type :dynamic :pos [3 0] :shapes [{:box [0.5 0.5]}]})
      (b2/joint! w :j {:type :distance :a :a :b :b :anchor-a [0.5 0] :anchor-b [2.5 0]})
      (is (near? 2.0 (get-in (b2/joint-info w :j) [:settings :length]) 1e-6))
      (is (points-near? [[0.5 0] [2.5 0]] (:anchors (b2/joint-info w :j)) 1e-6)))))

(deftest joints-of-every-type-are-remade-with-their-body
  (b2/with-world [w {:gravity [0 0]}]
    (hinge! w {:spring {:hertz 2 :damping 0.3}})
    (b2/body! w :bob {:type :dynamic :pos [2 0] :shapes [{:circle 0.1}]})
    (b2/joint! w :rod {:type :distance :a :bar :b :bob :spring {:hertz 1 :damping 0.1}})
    (b2/body! w :bar {:type :dynamic :pos [0.5 0] :shapes [{:box [0.5 0.08]}]})
    (is (= #{[:hinge :revolute] [:rod :distance]} (set (map (juxt :name :type) (b2/joints w)))))
    (is (= (:id (b2/body-info w :bar)) (:b (b2/joint-info w :hinge))))
    (is (= (:id (b2/body-info w :bar)) (:a (b2/joint-info w :rod))))
    (is (plain? (b2/snapshot w)))))

(deftest collision-filters
  (testing "the same negative group passes through itself, but not the floor"
    (b2/with-world [w]
      (floor! w)
      (b2/body! w :low {:type :dynamic :pos [0 1] :shapes [{:box [0.5 0.3] :group -1}]})
      (b2/body! w :high {:type :dynamic :pos [0 0] :shapes [{:box [0.5 0.3] :group -1}]})
      (b2/simulate w 2.0)
      (is (near? (:y (b2/transform w :low)) 1.6 0.01))
      (is (near? (:y (b2/transform w :high)) 1.6 0.01) "fell into the other one, onto the floor")))
  (testing "different groups (or none) collide"
    (b2/with-world [w]
      (floor! w)
      (b2/body! w :low {:type :dynamic :pos [0 1] :shapes [{:box [0.5 0.3] :group -1}]})
      (b2/body! w :high {:type :dynamic :pos [0 0] :shapes [{:box [0.5 0.3] :group -2}]})
      (b2/simulate w 2.0)
      (is (near? (:y (b2/transform w :high)) 1.0 0.01) "stacked")))
  (testing "a mask that leaves out the floor's category falls through it"
    (b2/with-world [w]
      (floor! w)                                               ; category 1
      (b2/body! w :ghost {:type :dynamic :shapes [{:box [0.2 0.2] :category 2 :mask 2}]})
      (b2/simulate w 1.5)
      (is (> (:y (b2/transform w :ghost)) 3)))))

(deftest read-bodies-in-one-go
  (b2/with-world [w {:gravity [0 0]}]
    (b2/body! w :a {:type :dynamic :pos [1 2] :shapes [{:box [0.1 0.1]}]})
    (b2/body! w :b {:type :dynamic :pos [3 4] :angle 0.5 :shapes [{:box [0.1 0.1]}]})
    (b2/set-velocity! w :b [5 6])
    (let [v (b2/read-bodies w [:a :b])]
      (is (= 12 (count v)))
      (is (= [1.0 2.0 0.0 0.0 0.0 0.0] (subvec v 0 6)))
      (is (near? (nth v 8) 0.5 1e-6))
      (is (= [5.0 6.0] (subvec v 9 11))))))

;; --- regressions ------------------------------------------------------------

(deftest many-joints-in-many-worlds
  ;; Joint params and reads go through a scratch buffer the shim writes into
  ;; blind. Once it was too small (8 doubles for 18), which quietly corrupted
  ;; the heap until something crashed several worlds later.
  (dotimes [_ 30]
    (b2/with-world [w]
      (slider! w {:spring {:hertz 2 :damping 0.1} :limit [-1 1] :motor {:max-force 1 :speed 0}})
      (b2/body! w :bob {:type :dynamic :pos [3 0] :shapes [{:circle 0.1}]})
      (b2/joint! w :rod {:type :distance :a :block :b :bob :spring {:hertz 1 :damping 0}
                         :limit [0.5 4] :motor {:max-force 1 :speed 0}})
      (b2/joint! w :hinge {:type :revolute :a :post :b :bob :anchor [1 0] :spring {:hertz 1 :damping 0}
                           :limit [-1 1] :motor {:max-torque 1 :speed 0}})
      (b2/simulate w 0.2)
      (dorun (map #(b2/joint-info w (:id %)) (b2/joints w)))))
  (is true "got here without the process dying"))
