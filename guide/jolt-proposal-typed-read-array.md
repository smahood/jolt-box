# Proposal: a native typed `read-array` / `write-array` for the Chez host

**Summary.** `jolt.ffi/read-array` with a numeric type (`(read-array p :double n)`)
fills the result one element at a time from Clojure, at about 125 ns per element.
A native block copy, decoded into the array's existing `flvector` / `fxvector`
backing, does the same in about 3 ns per element: **roughly 40× faster**, with
the same result. The change is small and local to the Chez host's `ffi.ss` and
the `read-array-typed` / `write-array-typed` helpers in `stdlib/jolt/ffi.clj`.
The existing Clojure loop stays as the fallback, so behaviour and the Gambit
host are unchanged.

Measured on jolt v0.8.21 (checkout `26cd9cd0`), Apple M2 Pro, macOS 15.1.

## Why it matters

Bindings to C libraries that produce bulk numeric data (physics engines,
audio, geometry, image processing) hand it back as a flat C buffer, and the
only typed way to get it into jolt is `read-array`. For those programs the
copy out is the whole cost: the C call that fills the buffer is cheap, and the
copy is not.

A concrete case from [jolt-box](https://github.com/smahood/jolt-box), a Box2D wrapper
for jolt: reading position and velocity (6 doubles) for 1000 bodies once a frame.

| | per frame |
|---|---|
| the one C call that writes all 6000 doubles into a buffer | 12 µs |
| `(ffi/read-array buf :double 6000)` to get them out | 910 µs |

That is about 5.5% of a 60 fps frame spent copying 48 KB. The
[joltc-jlt](https://github.com/noonian/joltc-jlt) binding for Jolt Physics
measured the same thing independently: its bulk transform read is 7 µs in C
and 1.0 ms once `ffi/read-array` has copied the arrays out.

## Where the time goes today

`read-array-typed` in `stdlib/jolt/ffi.clj`:

```clojure
(defn- read-array-typed [p t n off]
  (let [carrier (array-carrier t)]
    (if (nil? (:ctor carrier))
      (jolt.ffi/__read-array (+ p off) n)
      (let [width (:width carrier)
            element (:read-type carrier)
            arr ((:ctor carrier) n)]
        (dotimes [i n] (aset arr i (jolt.ffi/__read p element (+ off (* i width)))))
        arr))))
```

Each element pays for a var call into `__read` (`ffi-read*` in
`host/chez/java/ffi.ss`), which converts the type keyword (`ffi-type->chez`),
unboxes the pointer and offset (`jnum->exact`), reads with a non-constant
foreign type (`sa-foreign-ref`), and then a generic `aset`. None of that
depends on the element, but it runs once per element.

The byte form, `(read-array p n)`, is already a block copy
(`sa-foreign-bytes-ref!`, which is `memcpy`) and runs at about 1–2 ns per
byte-sized element. And a `double-array`'s backing is already a Chez
`flvector` (`na-make-backing` in `host/chez/java/natives-array.ss`), so the
typed form can be built natively the same way.

## Measurements

6000 doubles (48 KB), median of 200 calls, all returning a jolt `double-array`
with identical contents (checked with `=` against today's `read-array`):

| Approach | per double | 6000 doubles |
|---|---|---|
| `(ffi/read-array p :double n)` today | 125 ns | 751 µs |
| `ffi/read` in a Clojure loop | 144 ns | 866 µs |
| Clojure decode of `(read-array p bytes)` with `Double/longBitsToDouble` | 456 ns | 2734 µs |
| `ffi/byte-buffer` + little-endian `.getDouble` | 339 ns | 2034 µs |
| **prototype:** Scheme loop of `foreign-ref 'double` into an `flvector` | 27 ns | 162 µs |
| **prototype:** `memcpy` to a bytevector, `bytevector-ieee-double-native-ref` into an `flvector` | **2.9 ns** | **17.5 µs** |
| (for scale) the raw byte copy alone, `(read-array p bytes)` | 2 ns | 12 µs |

The workarounds available to user code (the byte copy plus decoding in
Clojure) are slower than today's typed read, so there is no fast path a
library can take without reaching into host internals.

## Proposed implementation

### 1. A native reader in `host/chez/java/ffi.ss`

Next to `ffi-read-array`. The prototype that produced the 2.9 ns figure, for
`:double`:

```scheme
;; (__read-array-typed ptr type n) -> a fresh jolt array of n elements of type
;; (the carrier's :read-type keyword), or #f for a type this host does not
;; decode natively (the caller falls back).
;; One block copy of n*width octets, then a typed decode into the array's own
;; backing: an flvector for float/double, an fxvector for the integer types.
(define (ffi-read-array-typed ptr type n)
  (let* ((p (jnum->exact ptr)) (n (jnum->exact n)))
    (define (block width)
      (let ((bv (make-bytevector (fx* n width))))
        (sa-foreign-bytes-ref! p bv (fx* n width))
        bv))
    (case (ffi-type->chez type)
      ((double)
       (let ((bv (block 8)) (fv (make-flvector n 0.0)))
         (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array fv 'double))
           (flvector-set! fv i (bytevector-ieee-double-native-ref bv (fx* i 8))))))
      ((float)
       (let ((bv (block 4)) (fv (make-flvector n 0.0)))
         (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array fv 'float))
           (flvector-set! fv i (bytevector-ieee-single-native-ref bv (fx* i 4))))))
      ((integer-32)
       (let ((bv (block 4)) (xv (make-fxvector n 0)))
         (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array xv 'int))
           (fxvector-set! xv i (bytevector-s32-native-ref bv (fx* i 4))))))
      ((integer-16)
       (let ((bv (block 2)) (xv (make-fxvector n 0)))
         (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array xv 'short))
           (fxvector-set! xv i (bytevector-s16-native-ref bv (fx* i 2))))))
      (else #f))))
(def-var! "jolt.ffi" "__read-array-typed" ffi-read-array-typed)
```

This exact function, loaded through `jolt.scheme/eval-string` on v0.8.21,
returns arrays `=` to today's `read-array` for `:double` (including ±infinity
and 1e300), `:float`, `:int32` and `:int16` (each type's minimum and maximum),
of the right class, and `#f` for `:int64`.

Points for review:

- **`:int64` is left to the fallback** in this sketch. A Chez fixnum is 61
  bits, so a `long-array` holding a full 64-bit value cannot be an
  `fxvector`; `na-make-backing` already starts such an array as a plain
  vector. A native `:int64` path would decode with
  `bytevector-s64-native-ref` and build a vector when any value is not a
  fixnum, or do the same widening `ja-set!` does.
- **The `type` argument** is the carrier's `:read-type` keyword (`:double`,
  `:float`, `:int32`, `:int16`), mapped with the existing `ffi-type->chez`, so
  `array-carrier` stays the one place that picks the element type. `#f` means
  "not handled here".
- **Large arrays.** `make-jolt-array` already pins `flvector` / `fxvector`
  backings of 8192 elements and up (`na-pin-large!`), so nothing new is needed
  there. The temporary bytevector could use `sa-make-large-bytevector` above
  `na-large-bytes`, as `na-bytes-copy` does, or be skipped by decoding straight
  from foreign memory (the 27 ns `foreign-ref` loop above, slower but with no
  temporary).
- **Float semantics.** `na-elem-of` stores a `float-array` element through
  `flsingle`; `bytevector-ieee-single-native-ref` already yields a
  single-precision value widened to a flonum, so the stored values match what
  `aset` would store. Worth a test alongside the `:float` changes in the
  Unreleased section.
- **Endianness.** `native-ref` matches the foreign memory's byte order, which
  is what `ffi/read` assumes today.

### 2. Use it from `stdlib/jolt/ffi.clj`

```clojure
(defn- read-array-typed [p t n off]
  (let [carrier (array-carrier t)]
    (if (nil? (:ctor carrier))
      (jolt.ffi/__read-array (+ p off) n)
      (or (jolt.ffi/__read-array-typed (+ p off) (:read-type carrier) n)
          (let [width (:width carrier)
                element (:read-type carrier)
                arr ((:ctor carrier) n)]
            (dotimes [i n] (aset arr i (jolt.ffi/__read p element (+ off (* i width)))))
            arr)))))
```

The Gambit host, whose `sa-foreign-bytes-ref!` raises, can define
`__read-array-typed` to return `#f` (or the stdlib can check for the var), and
keeps today's loop.

### 3. The same for `write-array`

`write-array-typed` loops `write-array-element` per element in the same way.
The mirror image is an encode into a bytevector with
`bytevector-ieee-double-native-set!` / `bytevector-s32-native-set!` and one
`sa-foreign-bytes-set!`. The masking and sign handling `write-array-element`
does for integers (`:mask`, `:signed-type`) would move into that loop. This
matters for the other direction of the same pattern: writing an array of ids
or inputs for a C call to consume.

### 4. Optional: `read-into!` for typed arrays

`read-into!` already fills an existing byte-array without allocating. A typed
version that fills an existing `double-array` would let a frame loop reuse one
array instead of allocating a new one per call. Its bounds checks can follow
`ffi-read-into!`.

## Tests

`test/chez/jolt-ffi-bytes-test.clj` covers the byte forms; the typed forms
would want, for each of `:double`, `:float`, `:int32`, `:int16` (and `:int64`
if handled):

- the native result `=` the fallback loop's result, over values that include
  negatives, NaN and ±infinity for the float types, and the integer type's
  minimum and maximum;
- a non-zero `offset`;
- `n` of 0, 1, and above 8192 (the pinning threshold);
- the returned array's class and `alength`, and `aset` / `aget` working on it
  afterwards (the backing kind is right).

## Reproducing the numbers

Self-contained, no library needed; save as `read-array-bench.clj` and run
`jolt read-array-bench.clj` on the Chez host. It defines the prototype through
`jolt.scheme/eval-string`, which reaches host internals (`make-jolt-array`,
`sa-foreign-bytes-ref!`) only to measure; that is not a suggestion for user
code.

```clojure
(require '[jolt.ffi :as ffi] '[jolt.scheme :as scm])

(def n 6000)
(def p (ffi/alloc (* 8 n)))
(dotimes [i n] (ffi/write p :double (+ 0.5 i) (* 8 i)))

(scm/eval-string "
(define (proto-read-doubles addr n)
  (let ((fv (make-flvector n 0.0)))
    (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array fv 'double))
      (flvector-set! fv i (foreign-ref 'double addr (fx* i 8))))))
(define (proto-read-doubles-bv addr n)
  (let ((bv (make-bytevector (fx* n 8))) (fv (make-flvector n 0.0)))
    (sa-foreign-bytes-ref! addr bv (fx* n 8))
    (do ((i 0 (fx+ i 1))) ((fx= i n) (make-jolt-array fv 'double))
      (flvector-set! fv i (bytevector-ieee-double-native-ref bv (fx* i 8))))))")

(def foreign-ref-loop (scm/proc "proto-read-doubles"))
(def memcpy-decode    (scm/proc "proto-read-doubles-bv"))

(defn bench [label f]
  (dotimes [_ 5] (f))
  (let [k 200 t0 (System/nanoTime)]
    (dotimes [_ k] (f))
    (let [dt (- (System/nanoTime) t0)]
      (println (format "%-30s %8.1f us  (%.1f ns/double)"
                       label (/ dt k 1e3) (/ dt k (double n)))))))

(println :same? (= (vec (memcpy-decode p n)) (vec (ffi/read-array p :double n))))
(bench "read-array :double (today)" #(ffi/read-array p :double n))
(bench "proto: foreign-ref loop"    #(foreign-ref-loop p n))
(bench "proto: memcpy + bv decode"  #(memcpy-decode p n))
(bench "raw bytes (for scale)"      #(ffi/read-array p (* 8 n)))
```

Output on the machine above:

```
:same? true
read-array :double (today)        751.2 us  (125.2 ns/double)
proto: foreign-ref loop           162.1 us  (27.0 ns/double)
proto: memcpy + bv decode          17.5 us  (2.9 ns/double)
```
