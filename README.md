# jolt-box

Box2D (and later Box3D) physics for [Jolt](https://github.com/jolt-lang/jolt),
built the same way as [ents](../ents): a flat C shim compiled together with the
engine's source into one dylib, a raw `jolt.ffi` layer, and a Clojure API
built for the REPL.

- `box2d/`: the Box2D v3.1.1 wrapper (in progress)
- `common/`: small dimension-free helpers shared by both
- `box3d/`: later

The plan, design decisions and step list are in
[guide/implementation-guide.md](guide/implementation-guide.md).

## Build and test (box2d)

Needs clang and git. There's no cmake: Box2D's sources compile straight into
the shim's dylib.

```
cd box2d
jolt native   # clone Box2D v3.1.1 into vendor/ (once) and build native/libjoltbox2d.dylib
jolt test     # headless tests
```

## Using it from a project

```clojure
;; deps.edn
:deps {jolt-box/box2d {:local/root "/Users/shaunmahood/Documents/SideProjects/jolt-box/box2d"}}
;; :jolt/native paths resolve from the directory jolt is launched in,
;; so point at the built library directly
:jolt/native [{:name "joltbox2d"
               :darwin ["/Users/shaunmahood/Documents/SideProjects/jolt-box/box2d/native/libjoltbox2d.dylib"]}]
```

```clojure
(require '[jolt-box.box2d :as b2])

(b2/with-world [w {:gravity [0 9.8]}]          ; y-down, like raylib
  (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})
  (b2/body! w :crate {:type :dynamic :shapes [{:box [0.5 0.5]}]})
  (let [samples (b2/simulate w 2.0 {:sample [:crate] :every 0.1})]
    [(b2/transform w :crate) (b2/settled-at samples :crate)]))
;=> [{:x ~0.0 :y ~1.4 :angle ~0.0} 1.0]   resting on the floor after a second
```

## API (box2d, step 0)

Meters, kilograms, seconds, radians. Anywhere a body id goes, a keyword
name works too. Re-evaluating a named `body!` replaces the old body.

| | |
|---|---|
| `(make-world {:gravity [gx gy] :max-steps 5})` | a world handle (default gravity `[0 9.8]`) |
| `(destroy! w)`, `(with-world [w opts] ...)` | release it; `with-world` does so in `finally` |
| `(step! w dt)`, `(step! w dt substeps)` | a raw step (4 substeps by default) |
| `(step-fixed! w frame-dt)` | fixed 1/60 s steps, at most `:max-steps` per call; returns alpha |
| `(body! w name? spec)` | `{:type :static\|:kinematic\|:dynamic :pos [x y] :angle a :fixed-rotation? b :awake? b :user-data n :shapes [{:box [hw hh] :offset [x y] :angle a :density d :friction f :restitution r}]}` → id |
| `(destroy-body! w body)` | |
| `(transform w body)`, `(velocity w body)` | `{:x :y :angle}`, `{:vx :vy :w}` |
| `(bodies w)`, `(body-info w body)`, `(snapshot w)` | inspection, as plain data |
| `(simulate w seconds {:sample [bodies] :every dt})` | step and sample, for the REPL and tests |
| `(series s body field)`, `(extent ...)`, `(period ...)`, `(settled-at s body)` | measuring samples |

Errors (a stale id, an unknown name, a bad spec, a destroyed world) throw
`ex-info`. They never abort the process.
