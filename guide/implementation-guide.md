# jolt-box implementation guide

jolt-box wraps Erin Catto's physics engines for [Jolt](https://github.com/jolt-lang/jolt):
[Box2D](https://box2d.org) now, and [Box3D](https://github.com/erincatto/box3d) later.
It follows the [ents](../../ents) pattern: a flat C shim compiled into one dylib
together with the engine's source, a raw `jolt.ffi` layer, and a Clojure API
on top.

The first user is the skibidi-toilet game
(`kids-games/Patrick/game1-jolt`). The library grows one small piece at a
time, and each piece is added when the game needs it. raylib, ents and ebb
are the base for future projects too, so the library has to fit well with
them. It also has to work without any of them.

---

## 1. Is building it in small pieces the right call?

Yes, with one condition. Most of the API can grow feature by feature, because
each Box2D feature (a joint type, an event kind, a shape) is a few shim
functions plus a wrapper. Adding one later doesn't disturb the others.

A handful of decisions are hard to change once code depends on them. Those get
settled up front, in **Step 0** (§4), before any feature work:

- how ids and handles cross the FFI boundary
- who owns a world, and what that means on an ebb fiber
- what the data looks like going in and coming out (vectors or maps, units,
  y direction)
- how errors are reported (Box2D asserts abort the process)
- the bulk-read pattern for reading many bodies per frame
- the pinned engine version and the repo layout

Everything after Step 0 is a feature step (§5) with its own acceptance test.

---

## 2. Repo layout

One repo with two libraries and a small shared core. Each library is its own
deps root, so a project depends only on the one it uses (the same way the
game pulls raylib-jlt with `:deps/root "lib"`).

```
jolt-box/
  README.md
  guide/
    implementation-guide.md      ← this file
  common/                        ← dimension-free helpers (starts tiny)
    deps.edn
    src/jolt_box/common.clj
  box2d/
    deps.edn                     ← :jolt/native, `native` + `box2d-src` tasks, :test alias
    native/
      jb2_shim.h
      jb2_shim.c
      libjoltbox2d.dylib         ← built, gitignored
    vendor/box2d/                ← pinned checkout, gitignored
    src/jolt_box/box2d.clj       ← the API (alias b2)
    src/jolt_box/box2d/ffi.clj   ← raw defcfn layer, no logic
    test/jolt_box/box2d_test.clj
    test/jolt_box/test_runner.clj
  box3d/                         ← later (§6)
```

**What goes in `common/` at the start.** Only code the Box2D library needs
that obviously has nothing to do with 2D vs 3D:

- the callback error trap (catch inside a C callback, rethrow from the call
  that drove it; the same approach as ents)
- scratch-buffer allocation and the "read N floats into a vector" helpers
- map → number coercion helpers (`double`, `int`, bools as 0/1)

Anything else stays in `box2d/` until Box3D proves it's really shared.
Guessing the shared layer before the second library exists usually guesses
wrong.

**Consuming it from a game** (the same quirk as ents and raygui): `:jolt/native`
paths resolve from the directory jolt was launched in, so the game's
`deps.edn` points at the built dylib with an absolute path:

```clojure
:deps {jolt-box/box2d {:local/root "/Users/shaunmahood/Documents/SideProjects/jolt-box/box2d"}}
:jolt/native [{:name "joltbox2d"
               :darwin ["/Users/shaunmahood/Documents/SideProjects/jolt-box/box2d/native/libjoltbox2d.dylib"]}]
```

---

## 3. Background decisions (from the planning conversation)

- **Why a shim at all?** Box2D passes `b2Vec2`, `b2Rot`, `b2Transform` and its
  id structs **by value**, and `jolt.ffi` can't express that. Every shim
  function takes and returns only scalars and pointers.
- **Why not a per-game shim?** naval-battle has a game-specific Box3D shim
  (`native/voxel_b3.c`). It works, but none of it can be reused. A library
  gives every future project the same API and fixes FFI problems in one place.
- **Why two libraries, not one unified 2D/3D API?** The dimensions leak
  (`[x y]` vs `[x y z]`, an angle vs a quaternion, revolute axes, different
  shape sets). Box3D's API is also newer and still changing. Keeping them as
  **parallel** APIs (same names and map keys where the concept matches) keeps
  each one idiomatic, and each gets its own version pin.
- **Shim style: one flat function per def, not mirrored structs.** Mirroring
  `b2BodyDef` etc. with `ffi/layout` would expose every field with very little
  C, but it breaks silently when the struct layout changes, and it already
  has: Box2D's `main` branch has restructured the joint defs (a shared
  `b2JointDef base`) and replaced `fixedRotation` with `b2MotionLocks`. Flat
  functions are stable, and fields get added when the game needs them.

---

## 4. Step 0: foundations (no game changes)

### 4.1 Engine version and build

- **Pin Box2D `v3.1.1`** (the latest release, June 2025). Don't track `main`:
  its API has moved on (see above). Upgrading later is a deliberate step:
  bump the tag, fix the shim, run the tests.
- **No cmake.** Box2D v3.1.1 is 35 plain C17 files in `src/` with no
  dependencies. One clang call builds them together with the shim into a
  single dylib, the same way ents compiles `flecs.c`:

  ```
  clang -O2 -std=c17 -shared -fPIC \
        -Ivendor/box2d/include -Ivendor/box2d/src -Inative \
        native/jb2_shim.c vendor/box2d/src/*.c \
        -o native/libjoltbox2d.dylib
  ```

  On Apple Silicon, Box2D picks its NEON SIMD path automatically. Leave
  `B2_ENABLE_ASSERT` off (see §4.5).
- **Tasks in `box2d/deps.edn`:**
  - `box2d-src`: `git clone --depth 1 --branch v3.1.1` into `vendor/box2d`,
    skipped if it's already there. `BOX2D_HOME` overrides the location, like
    ents' `FLECS_HOME`.
  - `native`: depends on `box2d-src`. It rebuilds only when the shim or
    `deps.edn` is newer than the dylib. Pass the source files as separate
    arguments (use `fs/glob`; no shell expands `*.c` for you).
  - `test`: depends on `native`, then `jolt -M:test`.

### 4.2 Handles and ids

- **World handle = a Clojure map**, not a bare id:

  ```clojure
  {:id      <uint32 from b2StoreWorldId>
   :scratch <per-world ffi buffers>
   :cbs     <atom of foreign-callables to free on destroy>
   :err     <atom: the error a callback caught, if any>
   :acc     <atom: fixed-step time accumulator>}
  ```

  **Scratch buffers belong to the world, not to the namespace.**
  naval-battle uses global `def`'d buffers, which is fine for one world on one
  thread. Per-world buffers keep two worlds on two ebb fibers from writing
  over each other's reads.
- **Body, shape and joint ids are plain `uint64`s** (`b2StoreBodyId`,
  `b2StoreShapeId`, `b2StoreJointId`). They're numbers, so they fit in ents
  components, maps and sets with no wrapper.
- **User data** holds a `uint64` too. Box2D's `void* userData` is pointer-sized,
  so a flecs entity id fits. Contact events can then hand back entities
  directly. The shim casts in and out.

### 4.3 Data in and out

| | Convention | Why |
|---|---|---|
| Vectors in | `[x y]` | short to write in body and joint specs |
| Single reads out | `{:x :y :angle}`, `{:vx :vy :w}` | drops straight into ents components (`Position {:x :y}`) and raylib's map-style args |
| Bulk reads out | one flat realized vector (or the raw buffer) | one FFI call per frame for N bodies |
| Seqs | always realized (`mapv`, `into []`) | ebb: realizing a lazy seq takes a lock, and a fiber can't park while holding one |
| Units | meters, kilograms, seconds, radians | Box2D is tuned for 0.1–10 m objects; the **game** converts pixels |
| y direction | whatever the gravity says | see below |

**y-down works as-is, which suits raylib.** Make the world with
`:gravity [0 9.8]` and use screen-style coordinates (y grows downward). In
y-down space a positive Box2D angle turns clockwise on screen, which is the
same direction as raylib's `rotation`. So the conversion is just radians to
degrees, with no sign flip. The library makes no assumption either way.

**Pixels ↔ meters lives in the game** (e.g. `PPM 100.0`, so the toilet is about
1 m). The library never sees a pixel.

### 4.4 Ownership, threading and ebb

These match the game's existing rules for the flecs world (EBB-CHEATSHEET §0):

- **A world belongs to one fiber.** Create it inside the sim's `m/sp`, destroy
  it in that `sp`'s `finally`, and never let the handle leave. Only plain data
  read out of it crosses to the main thread.
- **Single-threaded worlds by default** (`workerCount` 1, no task callbacks).
  Box2D's multithreading wants its own task system, which would fight ebb's
  scheduler. It can be an opt-in later if ever needed.
- **Callbacks run synchronously inside `step!`**, on the calling fiber. **Never
  park (`m/?`, posting to a mailbox) inside a Box2D callback**: there's a C
  frame on the stack. The same is already true of ents system bodies.
- **No raylib calls inside the library**, ever, so it's safe to use from the
  sim fiber.

### 4.5 Errors

- Box2D asserts **abort the process**. Build without `B2_ENABLE_ASSERT`, and
  have the shim check `b2Body_IsValid` / `b2Joint_IsValid` / `b2World_IsValid`
  on every id it receives. On a bad id it returns a failure code instead of
  calling in.
- The Clojure layer turns failure codes into `ex-info` with the id and op in
  the data. This is the same contract as ents (ops on a dead entity throw).
- **Callback exceptions** are caught before they unwind through C, stored on
  the world, and rethrown from the `step!` (or query) that drove them, so the
  world stays usable. This is the ents mechanism, and it's the first thing
  that moves into `common/`.

### 4.6 Stepping

- `(b2/step! w dt substeps)` is a raw step.
- `(b2/step-fixed! w frame-dt)` runs the fixed timestep. It adds `frame-dt` to
  the world's accumulator and runs as many 1/60 s steps (4 substeps each) as
  fit. It returns the leftover fraction (`alpha`) for anyone who wants
  render interpolation.

### 4.7 Step 0 deliverable

The repo scaffold, the build tasks, and this minimal API:

```clojure
(b2/make-world {:gravity [0 9.8]})   ; → world handle
(b2/destroy! w)
(b2/with-world [w opts] ...)          ; make + destroy in finally
(b2/step! w dt substeps)
(b2/step-fixed! w frame-dt)           ; → alpha
(b2/body! w {:type :static|:dynamic|:kinematic
             :pos [x y] :angle a
             :fixed-rotation? bool
             :user-data u64
             :shapes [{:box [half-w half-h] :offset [x y]
                       :density d :friction f :restitution r}]})  ; → body id
(b2/destroy-body! w body)
(b2/transform w body)                 ; → {:x :y :angle}
(b2/velocity w body)                  ; → {:vx :vy :w}
```

**Acceptance (headless tests):**
- a dynamic box dropped onto a static floor comes to rest on top of it within
  a couple of seconds, at the expected height (±1 cm)
- `with-world` destroys the world even when the body throws
- destroying a body, then reading it, throws `ex-info` and doesn't crash
- two worlds stepped alternately don't affect each other's reads

---

## 5. Feature steps

Each step lists what the game needs, the library additions, the game
changes, and how we know it works. They're roughly in order, but steps 4–7
can be reordered depending on what's most fun to try next.

### Step 1: the smallest usable step, "physics hop"

**The smallest change that makes the library useful in the game:** the toilet
hops with Box2D gravity and an impulse on a Box2D floor, behind a dev-panel
toggle so it can be compared with the current movement.

*Library additions*
- `(b2/impulse! w body [ix iy])`: `b2Body_ApplyLinearImpulseToCenter`, wakes
  the body
- `(b2/grounded? w body min-normal)`: done in C. It walks
  `b2Body_GetContactData`, and is true if a touching contact's normal points
  up (against gravity) by at least `min-normal`. That avoids having sensors
  and events yet.
- `(b2/set-velocity! w body [vx vy])`: to stop sideways sliding on landing,
  so each hop stays one hop, as now

*Game changes*
- `game.physics` (or similar): the PPM conversion, plus making static bodies
  from `level/edges` and the walls
- the Box2D world is made and destroyed in `game.sim/sim-task` next to the
  flecs world, and stepped with `step-fixed!` in a `:pre-update` system
- the player entity gets a `Body {:id u64}` component. A system copies its
  transform into `Position` (meters → pixels; Position is the top-left of
  the hitbox, so subtract half the size)
- the hop is `impulse!` with `[±side, -up]` × mass when `grounded?`
- a dev-panel toggle picks old movement or Box2D movement

*Not in Step 1:* wall damage on the Box2D path. The toilet just bumps the
static wall. Step 2 adds that.

*Acceptance*
- headless: an impulse on a body resting on the floor lifts it, and it lands
  back with `grounded?` true; `grounded?` is false in mid-air
- in the game: the toilet hops left and right, can't hop in mid-air, and stops
  at walls and room ends; the toggle switches back to the old movement

### Step 2: contact events, user data and momentum (wall damage)

Covers the "momentum calculations" goal. Wall damage comes from how hard
something hits, not a fixed hit count.

*Library additions*
- `:user-data` reads: `(b2/user-data w body)` and `shape → body`
- `:enable-hit-events?` and `:enable-contact-events?` on shape specs
- `(b2/contact-events w)` → `{:begin [{:a ua :b ub}] :end [...] :hit [{:a ua :b ub :point [x y] :normal [x y] :speed s}]}`,
  where `ua`/`ub` are the bodies' user data (e.g. flecs entities). The shim
  copies `b2World_GetContactEvents` into a flat per-world buffer in one call
- `(b2/mass w body)`, plus the mass data (center, rotational inertia)
- `(b2/momentum w body)` → `{:px :py}`, a convenience for `m·v`

*Game changes*
- walls carry their entity id as user data
- damage = f(approach speed × player mass). A tap dents and a full-speed hop
  smashes. This replaces `:bumped` and `hp 2`, with the thresholds in
  `config` and the dev panel
- `bounce-off` becomes restitution plus an impulse, or goes away

*Acceptance*
- headless: a body dropped from a greater height reports a higher hit
  `:speed`; user data round-trips
- in the game: walls take damage from hard hits only; the old path still works
  behind the toggle

### Step 3: sensors (optional cleanup of grounding)

*Library:* `:sensor? true` on shape specs, `:enable-sensor-events?`, and
`(b2/sensor-events w)` → `{:begin [{:sensor us :visitor uv}] :end [...]}`.
*Game:* a thin foot sensor under the base replaces `grounded?`. This also
gives you "landed this frame" events for dust puffs and squash animation.
*Acceptance:* begin and end fire once per landing and per take-off.

### Step 4: spring jump (prismatic joint)

The toilet becomes a **base** and a **bowl** joined by a vertical prismatic
joint with a spring. Holding the key squashes the bowl down. Releasing lets
the spring shove the bowl up, and at the upper limit it yanks the base off
the floor, so the hop comes out of the physics.

*Library additions*
- multi-shape bodies (already allowed by `:shapes`), plus `:circle` and
  `:capsule` shapes
- `(b2/joint! w {:type :prismatic :a body-a :b body-b :anchor-a [x y] :anchor-b [x y]
                 :axis [0 -1] :spring {:hertz h :damping z} :limit [lo hi]
                 :motor {:max-force f :speed s} :target t :collide-connected? false})`
  → joint id
- runtime setters: `(b2/set-joint! w j {:spring {...} :target t :motor {...}})`
  (`b2PrismaticJoint_SetTargetTranslation`, `SetSpringHertz`, etc.)
- reads: `(b2/joint-state w j)` → `{:translation :speed}`
- `(b2/destroy-joint! w j)`

*Game changes*
- the toilet entity has two bodies. Charge = lower the target or run the motor;
  release = restore it
- the squash and stretch is drawn from `:translation`, so the art follows the
  real spring
- hertz, damping, limits and charge time go in the raygui dev panel. Optionally,
  a visual-only damped spring in plain Clojure for extra wobble on top

*Acceptance*
- headless: a sprung prismatic joint released from compression oscillates at
  about the configured hertz (measure the period ±10%); limits hold
- in the game: a longer charge gives a higher hop; the spring settings are
  live-tunable

### Step 5: neck, head weight and wobble

*Library additions*
- `:revolute` joints: `:spring`, `:limit [lo hi]` angles, `:motor`,
  `:reference-angle`; reads `{:angle :speed}`
- `:distance` joints: `:length`, `:range [min max]`, `:spring`;
  `(b2/set-joint! w j {:length l})` (`b2DistanceJoint_SetLength`)
- `(b2/set-density! w shape d)` → `b2Shape_SetDensity` +
  `b2Body_ApplyMassFromShapes`; `body!` returns shape ids when asked
  (`:return :all` → `{:body id :shapes [ids]}`)
- `(b2/read-bodies w ids)` → one flat vector `[x y angle vx vy w ...]` per
  body, in a single FFI call. Steps 5 and 6 are the first with enough bodies
  that this matters

*Game changes*
- neck = 3–5 small segment bodies joined by sprung revolute joints, plus
  distance joints that lengthen to extend the neck (or segments added at
  runtime); drawn as a smooth curve through the segment positions
- the head is a circle body; its weight is a dev-panel slider (density). A
  heavier head swings harder, shifts the centre of mass, and with Step 2 hits
  walls harder

*Acceptance*
- headless: a sprung revolute joint returns to its reference angle; a
  distance joint's length change takes effect; raising density raises `mass`
- in the game: the head wobbles after hops, the neck extends and retracts, and
  head weight changes how the toilet moves

### Step 6: deformable, breakable walls

Box2D has only rigid bodies, so walls deform by being built from pieces.

*Library additions*
- `:weld` joints with soft `:linear-hertz` / `:angular-hertz` (above 0 lets the
  wall bend and sag)
- `(b2/joint-forces w joint-ids)` → flat vector of constraint-force
  magnitudes in one call (`b2Joint_GetConstraintForce` per joint, in C)
- `(b2/break-joints! w joint-ids max-force)`: done in C. It destroys every
  joint over the threshold and returns the broken ids, avoiding a round trip
  per joint
- `:awake? false` and sleep settings on `body!`, so a wall of many pieces
  costs nothing until it's hit (naval-battle tuned these for rubble)

*Game changes*
- a wall kind gets a `:pieces` layout (grid size, weld stiffness, break force)
- loose pieces become debris bodies, alongside or replacing the chunk
  particles; maybe swap a static wall for its pieces version only on first
  hit, to keep the body count low

*Acceptance*
- headless: a welded stack holds under gravity; a hard impulse breaks the
  joints near the impact and not the far ones
- in the game: walls bend and crack where they're hit, and break apart when
  hit hard enough; frame time stays acceptable in the dev panel

### Step 7: debug draw

Box2D's `b2DebugDraw` callbacks receive `b2Transform` **by value**, so they
can't be `foreign-callable`s from jolt. The shim implements the callbacks in
C, writes primitives into a per-world buffer, and Clojure reads it once:

- `(b2/debug-shapes w)` → realized vector of plain maps:
  `{:kind :polygon :points [[x y] ...] :color 0xRRGGBB}`, `:circle`,
  `:segment`, `:point`
- **No raylib in the library.** The game, or later a tiny optional
  `jolt-box/box2d-raylib` deps root, turns those maps into raylib calls
  inside the camera, behind a dev-panel toggle

This can be done any time after Step 1. It's worth doing early if tuning
joints in Steps 4–6 gets confusing.

### Step 8: ents glue (only once it has settled in the game)

Start with the glue **in the game**. Move it into an optional
`jolt-box/box2d-ents` deps root once the shape has settled, so the core library
keeps no flecs dependency. Its expected shape:

- a `Body {:id u64}` component (and `Joint {:id u64}`)
- a step system (`:pre-update` or `:on-load`) calling `step-fixed!`
- a sync system copying `read-bodies` results into `Position` / `Velocity`
- an observer on Body removal that destroys the Box2D body. **This matters:**
  the game clears a level by deleting `:level`, and without the observer the
  bodies would outlive their entities. The simple alternative is one Box2D
  world per level, destroyed with it
- contact events turned into flecs events or components (e.g. a `Hit`, as now),
  using the user data → entity mapping from Step 2

---

## 6. Later: the Box3D library

When naval-battle moves over:

1. Pin a Box3D tag (`v0.1.0` was the first release, June 2026). Check
   whether it still needs cmake or builds with one clang call like Box2D.
2. Create `box3d/` with the same structure: `jb3_shim`, `jolt-box.box3d`
   (alias `b3`), and the same names and map keys where the concept matches
   (`body!`, `joint!`, `:spring {:hertz :damping}`, `contact-events`,
   `read-bodies`, `step-fixed!`).
3. Move into `common/` whatever turns out to be identical in both: the callback
   trap, buffers, event-buffer → map helpers, maybe the build task helpers.
4. Port naval-battle's `voxel.box3d` uses one by one, keeping its tuned
   defaults (sleep threshold, damping) as explicit options, not baked into the
   shim.

---

## 7. Conventions checklist (every step)

- [ ] shim function `jb2_<noun>_<verb>`, scalars and pointers only, validates
      every id
- [ ] declared in `jb2_shim.h` with a one-line comment on ownership and units
- [ ] raw binding in `jolt-box.box2d.ffi`, no logic
- [ ] wrapper in `jolt-box.box2d`: map in, map or realized vector out, throws
      `ex-info` on failure
- [ ] headless test for the behaviour, not just "it doesn't crash"
- [ ] no raylib, no ents, no ebb requires in the core library
- [ ] the README API list updated
