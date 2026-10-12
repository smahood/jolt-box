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

## API (box2d, steps 0–4)

Meters, kilograms, seconds, radians. Anywhere a body id goes, a keyword
name works too. Re-evaluating a named `body!` replaces the old body.

| | |
|---|---|
| `(make-world {:gravity [gx gy] :max-steps 5 :hit-event-threshold 1.0 :collect-events? true})` | a world handle (default gravity `[0 9.8]`) |
| `(destroy! w)`, `(with-world [w opts] ...)` | release it; `with-world` does so in `finally` |
| `(step! w dt)`, `(step! w dt substeps)` | a raw step (4 substeps by default) |
| `(step-fixed! w frame-dt)` | fixed 1/60 s steps, at most `:max-steps` per call; returns alpha |
| `(body! w name? spec)` | `{:type :static\|:kinematic\|:dynamic :pos [x y] :angle a :fixed-rotation? b :awake? b :user-data n :shapes [{:box [hw hh] :offset [x y] :angle a :density d :friction f :restitution r :contact-events? b :hit-events? b :sensor? b :sensor-events? b}]}` → id. A sensor only sees shapes with `:sensor-events? true` |
| `(destroy-body! w body)` | |
| `(transform w body)`, `(velocity w body)` | `{:x :y :angle}`, `{:vx :vy :w}` |
| `(impulse! w body [ix iy])`, `(impulse! w body [ix iy] [px py])` | a change in momentum, at the center or a world point |
| `(force! w body [fx fy])`, `(force! w body [fx fy] [px py])` | a push for the next step |
| `(set-velocity! w body [vx vy])`, `(set-angular-velocity! w body av)` | |
| `(set-transform! w body [x y])`, `(set-transform! w body [x y] angle)` | a teleport |
| `(set-awake! w body b)`, `(set-gravity-scale! w body s)`, `(set-damping! w body {:linear :angular})` | |
| `(contacts w body)` | `[{:other :other-name :normal [nx ny] :point [x y] :points :separation :touching?}]`, normal pointing into this body |
| `(grounded? w body)`, `(grounded? w body {:min-normal :max-separating-speed})` | standing on something, against gravity |
| `(gravity w)` | `[gx gy]` |
| `(mass w body)`, `(mass-data w body)`, `(momentum w body)` | kg; `{:mass :inertia :center [x y] :local-center [x y]}`; `{:px :py}` |
| `(shapes w body)`, `(shape-body w shape)`, `(user-data w body)` | |
| `(set-shape! w shape {:friction :restitution :density :contact-events? :hit-events? :sensor-events?})` | change a shape live (density updates the mass) |
| `(contact-events w)` | `{:begin [..] :end [..] :hit [{:body-a :body-b :name-a :name-b :user-a :user-b :normal :point :speed}] :dropped n}`, everything since the last call |
| `(events-for w events body)` | those events from one body's point of view: `:other :other-name :other-user`, normal into it |
| `(sensor-events w)` | `{:begin [{:sensor-shape :sensor-body :sensor-name :sensor-user :visitor-shape :visitor-body :visitor-name :visitor-user}] :end [..] :dropped n}`, everything since the last call |
| `(sensor-overlaps w sensor-shape)` | `[{:shape :body :name :user}]` overlapping now (no lag) |
| `(bodies w)`, `(body-info w body)`, `(snapshot w)` | inspection, as plain data |
| `(debug-shapes w)` | `[{:kind :polygon :points [[x y] ...] :center [x y] :body :name :shape :type :awake? :sensor?}]`, for drawing outlines |
| `(simulate w seconds {:sample [bodies] :every dt})` | step and sample, for the REPL and tests |
| `(series s body field)`, `(extent ...)`, `(period ...)`, `(settled-at s body)` | measuring samples |

Errors (a stale id, an unknown name, a bad spec, a destroyed world) throw
`ex-info`. They never abort the process.
