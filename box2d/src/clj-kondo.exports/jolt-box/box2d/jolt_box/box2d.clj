(ns jolt-box.box2d
  "clj-kondo's view of jolt-box.box2d's macros: the same binding shape, with
  the world made from opts.")

(defmacro with-world [[sym opts] & body]
  `(let [~sym ~(or opts {})]
     ~@body))
