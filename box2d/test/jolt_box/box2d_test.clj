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
