(ns jolt-box.common
  "Helpers shared by the jolt-box engine wrappers that don't care whether the
  world is 2D or 3D: number coercion for FFI args, reading scratch buffers
  into realized vectors, and the name/id registry.

  The registry is a plain map kept in an atom on each world handle:

      {:entries {:body {id {:name :toilet :spec {...} ...}}}
       :names   {:body {:toilet id}}}

  `kind` is :body, :joint, ... Names are keywords and optional. Ids are the
  engine's integer ids. These fns are pure: they take a registry value and
  return a new one."
  (:require [jolt.ffi :as ffi]))

;; --- coercion ----------------------------------------------------------------

(defn flag
  "A truthy value as the 0/1 int a C shim takes."
  [x]
  (if x 1 0))

;; --- scratch buffers ---------------------------------------------------------

(defn read-doubles-at
  "n doubles of buffer buf starting at double index start, as a realized
  vector."
  [buf start n]
  ;; One read-array rather than n ffi/reads: the same today, and it picks up a
  ;; block copy if jolt's typed read-array gets one.
  (vec (ffi/read-array buf :double n (* 8 start))))

(defn read-doubles
  "The first n doubles of buffer buf, as a realized vector."
  [buf n]
  (read-doubles-at buf 0 n))

;; --- registry ----------------------------------------------------------------

(def empty-registry {:entries {} :names {}})

(defn resolve-id
  "The id that name-or-id refers to in registry reg, or nil for an unknown
  name. Integers pass through unchanged (the shim validates them)."
  [reg kind name-or-id]
  (if (keyword? name-or-id)
    (get-in reg [:names kind name-or-id])
    name-or-id))

(defn entry [reg kind id] (get-in reg [:entries kind id]))

(defn entries
  "Every entry of a kind, as a realized vector of [id entry] pairs, sorted by
  id so listings are stable."
  [reg kind]
  (vec (sort-by key (get-in reg [:entries kind]))))

(defn add-entry
  "Records entry e (a map, which may hold :name) under id."
  [reg kind id e]
  (cond-> (assoc-in reg [:entries kind id] e)
    (:name e) (assoc-in [:names kind (:name e)] id)))

(defn remove-entry
  "Forgets id and, if this id still owns it, its name."
  [reg kind id]
  (let [n (:name (entry reg kind id))]
    (cond-> (update-in reg [:entries kind] dissoc id)
      (and n (= id (get-in reg [:names kind n])))
      (update-in [:names kind] dissoc n))))
