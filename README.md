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

Needs the jolt runtime 0.8.4 or later (`:jolt/min-version` in
`box2d/deps.edn`). Build it once with `jolt native` in `box2d/`, then depend
on it by path. The library's own `:jolt/native` entry resolves against
`box2d/deps.edn`, so the project needs nothing else:

```clojure
;; deps.edn
:deps {jolt-box/box2d {:local/root "/Users/shaunmahood/Documents/SideProjects/jolt-box/box2d"}}
```

The library is declared `:optional`, so a project starts without the build.
`(b2/available?)` says whether it loaded, and `make-world` throws with what
to do when it didn't (not built yet, or built from an older shim). The
runtime loads it at startup, so restart the REPL after a first
`jolt native` or a shim change. To use a build somewhere else, declare
`{:name "joltbox2d" ...}` under `:jolt/native` in the project with its own
paths; a project's declaration overlays the dependency's.

```clojure
(require '[jolt-box.box2d :as b2])

(b2/with-world [w {:gravity [0 9.8]}]          ; y-down, like raylib
  (b2/body! w :floor {:pos [0 2] :shapes [{:box [10 0.1]}]})
  (b2/body! w :crate {:type :dynamic :shapes [{:box [0.5 0.5]}]})
  (let [samples (b2/simulate w 2.0 {:sample [:crate] :every 0.1})]
    [(b2/transform w :crate) (b2/settled-at samples :crate)]))
;=> [{:x ~0.0 :y ~1.4 :angle ~0.0} 1.0]   resting on the floor after a second
```

## API (box2d, steps 0–7)

Meters, kilograms, seconds, radians. Anywhere a body id goes, a keyword
name works too. Re-evaluating a named `body!` replaces the old body.

| | |
|---|---|
| `(available?)` | true when libjoltbox2d is loaded and its shim version matches |
| `(make-world {:gravity [gx gy] :max-steps 5 :hit-event-threshold 1.0 :collect-events? true})` | a world handle (default gravity `[0 9.8]`) |
| `(destroy! w)`, `(with-world [w opts] ...)` | release it; `with-world` does so in `finally` |
| `(step! w dt)`, `(step! w dt substeps)` | a raw step (4 substeps by default) |
| `(step-fixed! w frame-dt)` | fixed 1/60 s steps, at most `:max-steps` per call; returns alpha |
| `(body! w name? spec)` | `{:type :static\|:kinematic\|:dynamic :pos [x y] :angle a :fixed-rotation? b :awake? b :enable-sleep? b :sleep-threshold :linear-damping :angular-damping :gravity-scale :user-data n :shapes [{:box [hw hh] :offset [x y] :angle a :density d :friction f :restitution r :contact-events? b :hit-events? b :sensor? b :sensor-events? b}]}` → id. Shapes are `{:box [hw hh]}`, `{:circle r}` or `{:capsule [[x1 y1] [x2 y2]] :radius r}`, and take `:group :category :mask` for collision filtering (same negative group never collides). A sensor only sees shapes with `:sensor-events? true`. Re-evaluating a named body remakes its joints |
| `(destroy-body! w body)` | |
| `(transform w body)`, `(velocity w body)` | `{:x :y :angle}`, `{:vx :vy :w}` |
| `(read-bodies w bodies)` | `[x y angle vx vy w ...]` for many bodies at once |
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
| `(joint! w name? {:type :prismatic :a body :b body :anchor [x y] :axis [x y] :spring {:hertz :damping} :target t :limit [lo hi] :motor {:max-force :speed} :collide-connected? b :user-data n})` | → joint id. Anchors and axis in world space, where the bodies are now |
| `(joint! w name? {:type :revolute :a :b :anchor [x y] :spring {..} :target rad :limit [lo hi] :motor {:max-torque :speed}})` | a hinge |
| `(joint! w name? {:type :distance :a :b :anchor-a [x y] :anchor-b [x y] :length m :spring {..} :limit [min max] :motor {:max-force :speed}})` | rigid unless it has a `:spring`; anchors default to the bodies' origins, length to how far apart they are |
| `(joint! w name? {:type :weld :a :b :anchor [x y] :linear-hertz :angular-hertz :linear-damping :angular-damping})` | holds b fixed to a; 0 hertz is rigid |
| `(set-joint! w joint {:spring {..}\|false :target t :length m :limit [lo hi]\|false :motor {..}\|false :linear-hertz .. :collide-connected? b})` | change it live (remembered if the joint is remade) |
| `(set-joint! w joint {:local-anchor-a [x y] :local-anchor-b [x y] :local-axis [x y] :reference-angle rad})` | move where it attaches, in the bodies' local frames (e.g. a neck that grows) |
| `(joint-forces w joints)` | `[{:id :force :fx :fy :torque}]` as of the last step |
| `(break-joints! w joints {:max-force :max-torque})`, `(break-joints! w joints pred)` | snaps the overloaded ones, returns their ids |
| `(joint-state w joint)` | prismatic `{:translation :speed :motor-force}`, revolute `{:angle :speed :motor-torque}`, distance `{:length :motor-force}`, plus `{:force [fx fy] :torque :linear-separation :angular-separation}` |
| `(joints w)`, `(joint-info w joint)`, `(destroy-joint! w joint)` | |
| `(awake? w body)` | cheap; a sleeping body's joints report no force |
| `(bodies w)`, `(body-info w body)`, `(snapshot w)` | inspection, as plain data |
| `(debug-shapes w)` | shapes (`:kind :polygon :points`, `:circle :at :radius`, `:capsule :points :radius`, with `:center :body :name :shape :type :awake? :sensor?`) and joints (`:kind :segment :points [a b] :joint :name`), for drawing outlines |
| `(simulate w seconds {:sample [bodies] :every dt})` | step and sample, for the REPL and tests |
| `(series s body field)`, `(extent ...)`, `(period ...)`, `(settled-at s body)` | measuring samples |

Errors (a stale id, an unknown name, a bad spec, a destroyed world) throw
`ex-info`. They never abort the process.
