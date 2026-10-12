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
them. It also has to work without any of them. It's built to be explored and
tuned from the REPL (§4).

---

## 1. Is building it in small pieces the right call?

Yes, with one condition. Most of the API can grow feature by feature, because
each Box2D feature (a joint type, an event kind, a shape) is a few shim
functions plus a wrapper. Adding one later doesn't disturb the others.

A handful of decisions are hard to change once code depends on them. Those get
settled up front, in **Step 0** (§5), before any feature work:

- how ids and handles cross the FFI boundary
- the registry and naming scheme that make the world inspectable and
  re-evaluable from the REPL (§4)
- who owns a world, and what that means on an ebb fiber
- what the data looks like going in and coming out (vectors or maps, units,
  y direction)
- how errors are reported (Box2D asserts abort the process)
- the bulk-read pattern for reading many bodies per frame
- the pinned engine version and the repo layout

Everything after Step 0 is a feature step (§6) with its own acceptance test.

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
  box3d/                         ← later (§7)
```

**What goes in `common/` at the start.** Only code the Box2D library needs
that obviously has nothing to do with 2D vs 3D:

- the callback error trap (catch inside a C callback, rethrow from the call
  that drove it; the same approach as ents), with callbacks looked up through
  their var (§4)
- scratch-buffer allocation and the "read N floats into a vector" helpers
- map → number coercion helpers (`double`, `int`, bools as 0/1)
- the name/id registry (§4); it's the same idea in 2D and 3D

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

## 4. REPL workflow

The library should feel like ents at the REPL: re-evaluate a form and the
world updates, and anything in the world can be looked at as plain data.
Every step in §6 follows these rules.

### 4.1 Clojure first, C later

**Changing the shim means restarting jolt.** A loaded dylib can't be reliably
swapped out, so every C edit costs a REPL restart. So:

- **Write each feature in Clojure first, using the smallest possible C
  calls** (get a value, set a value, copy an event buffer). Live-edit it until
  the behaviour is right.
- **Move logic into C only when it has settled, or when the dev panel shows
  it's too slow.** Batch readers like `read-bodies` and `joint-forces` are the
  usual candidates, once there are enough bodies to matter.
- **Grow the C side in a few big batches,** not on every tweak. When a step
  needs new shim functions, add the full set of getters and setters for that
  Box2D feature at once (e.g. every `b2PrismaticJoint_*` the game might want),
  so the following REPL sessions don't need a restart.

### 4.2 A registry makes the world inspectable

Box2D v3 has no "list all bodies" call, so the world handle keeps its own
registry:

```clojure
{:bodies {id {:name :toilet :spec {...} :shapes [ids] :joints #{ids}}}
 :joints {id {:name :neck-1 :spec {...} :a id :b id}}
 :names  {:toilet id, :neck-1 id}}
```

On top of it:

- `(b2/bodies w)` and `(b2/joints w)`: everything, as a realized vector of
  `{:id :name :type ...}`
- `(b2/body-info w :toilet)`: the spec it was made from, plus its live state
  (transform, velocity, mass, awake, current contacts, attached joints)
- `(b2/joint-info w :neck-1)`: the spec plus live state (angle or
  translation, speed, constraint force)
- `(b2/snapshot w)`: the whole world as one plain map, good for `tap>`, diffs,
  or saving a scene to a file

The registry also keeps specs around, which makes live re-creation possible
(see names below).

### 4.3 Names make re-evaluation safe

Like ents' `entity!` and its systems, bodies and joints can have a name:

```clojure
(b2/body! w :toilet {:type :dynamic ...})   ; again → replaces the old :toilet
(b2/joint! w :neck-1 {:type :revolute :a :toilet :b :head ...})
(b2/transform w :toilet)
```

- **Re-evaluating a named `body!` destroys the old body and makes the new one.**
  Joints that referenced it by name are re-created against the new body, from
  their stored specs. So re-running a setup form after a tweak just works,
  with no duplicate toilets.
- **Names work anywhere an id does.** Ids still work too, and unnamed bodies
  (debris, wall pieces) don't pay for the registry lookup.
- Names live in the world's registry. A different world can reuse the same
  names.

### 4.4 Callbacks look up their function through its var

A Clojure function passed into a Box2D callback (custom filter, pre-solve,
queries) is captured when it's registered, so a later `defn` would do
nothing. The callback trap in `common/` accepts a var (`#'my-filter`) and calls
through it every time, so redefining the function takes effect immediately.
Passing a plain fn still works for one-off use.

### 4.5 Try physics without the game

```clojure
(b2/with-world [w {:gravity [0 9.8]}]
  (b2/body! w :floor {:type :static :pos [0 2] :shapes [{:box [10 0.1]}]})
  (b2/body! w :ball  {:type :dynamic :pos [0 0] :shapes [{:circle 0.2}]})
  (b2/simulate w 2.0 {:sample [:ball] :every 0.05}))
;=> [{:t 0.0 :ball {:x 0.0 :y 0.0 :angle 0.0 :vx 0.0 :vy 0.0 :w 0.0}} ...]
```

`simulate` steps the world on the fixed timestep and returns a realized
vector of samples, so a spring's period, peak height or settling time can be
checked right at the REPL. The headless tests use the same function. Small
helpers to go with it: `(b2/series samples :ball :y)` → `[[t v] ...]`,
`(b2/extent samples :ball :y)` → `{:min :max}` (with y-down, the highest point
reached is the `:min` of `:y`), `(b2/period samples :ball :y)` and
`(b2/settled-at samples :ball)`.

### 4.6 Reaching the world inside the running game

The game's world lives on the sim fiber (§5.4), and `game.sim/send!` already
runs a function there and returns the result:

```clojure
(game.sim/send! #(b2/body-info (:b2 %) :toilet))
(game.sim/send! #(b2/set-joint! (:b2 %) :jump-spring {:spring {:hertz 6}}))
```

The game should store its Box2D world handle where `send!`'s function can
reach it (e.g. a singleton component, or a field next to the flecs world in
`sim-task`).

### 4.7 Resets and pauses

- **One Box2D world per level, by default.** Resetting is "destroy the world,
  rebuild it". A messy REPL session can't leave orphaned bodies behind, and
  `delete :level` in ents doesn't need to track bodies one by one.
- **The fixed step is capped.** If you break at the REPL for 30 seconds, the
  next `step-fixed!` runs at most `:max-steps` steps (default 5) and drops the
  leftover time, so the world doesn't lurch forward.
- **Stale handles fail cleanly.** A handle held at the REPL after its world
  was destroyed throws `ex-info` ("world destroyed"), it doesn't crash.

### 4.8 See what you made

Debug outlines come right after the first game step (Step 2, §6). The first
version is pure Clojure: it walks the registry and `transform`s and returns
shapes as plain maps. It needs no C debug-draw callbacks, so it can be changed
live.

---

## 5. Step 0: foundations (no game changes) ✅ done

### 5.1 Engine version and build

- **Pin Box2D `v3.1.1`** (the latest release, June 2025). Don't track `main`:
  its API has moved on (see §3). Upgrading later is a deliberate step:
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
  `B2_ENABLE_ASSERT` off (see §5.5).
- **Tasks in `box2d/deps.edn`:**
  - `box2d-src`: `git clone --depth 1 --branch v3.1.1` into `vendor/box2d`,
    skipped if it's already there. `BOX2D_HOME` overrides the location, like
    ents' `FLECS_HOME`.
  - `native`: depends on `box2d-src`. It rebuilds only when the shim or
    `deps.edn` is newer than the dylib. Pass the source files as separate
    arguments (use `fs/glob`; no shell expands `*.c` for you).
  - `test`: depends on `native`, then `jolt -M:test`.

### 5.2 Handles and ids

- **World handle = a Clojure map**, not a bare id:

  ```clojure
  {:id        <uint32 from b2StoreWorldId>
   :gravity   [gx gy]
   :alive     <atom: false once destroyed, so stale handles throw>
   :registry  <atom: bodies, joints, names (§4.2)>
   :scratch   <per-world ffi buffers>
   :acc       <atom: fixed-step time accumulator>
   :max-steps <cap on steps per step-fixed!, default 5>}
  ```

  It's a record (`jolt-box.box2d/World`). The first step that registers a
  callback adds `:cbs` (foreign-callables to free on destroy) and `:err` (the
  error a callback caught).

  **Scratch buffers belong to the world, not to the namespace.**
  naval-battle uses global `def`'d buffers, which is fine for one world on one
  thread. Per-world buffers keep two worlds on two ebb fibers from writing
  over each other's reads.

  The handle gets a short `print-method` (`#b2/world{:id 1 :bodies 4 :joints 2}`)
  so evaluating it at the REPL doesn't dump buffers and atoms.
- **Body, shape and joint ids are plain `uint64`s** (`b2StoreBodyId`,
  `b2StoreShapeId`, `b2StoreJointId`). They're numbers, so they fit in ents
  components, maps and sets with no wrapper. **Every function that takes an id
  also accepts a registered name** (§4.3).
- **User data** holds a `uint64` too. Box2D's `void* userData` is pointer-sized,
  so a flecs entity id fits. Contact events can then hand back entities
  directly. The shim casts in and out.

### 5.3 Data in and out

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
degrees, with no sign flip. The default gravity is `[0 9.8]` to match raylib;
pass `[0 -9.8]` for y-up. Nothing else in the library cares which way y
points.

**Pixels ↔ meters lives in the game** (e.g. `PPM 100.0`, so the toilet is about
1 m). The library never sees a pixel.

### 5.4 Ownership, threading and ebb

These match the game's existing rules for the flecs world (EBB-CHEATSHEET §0):

- **A world belongs to one fiber.** Create it inside the sim's `m/sp`, destroy
  it in that `sp`'s `finally`, and never let the handle leave. Only plain data
  read out of it crosses to the main thread. The REPL reaches it through
  `send!` (§4.6).
- **Single-threaded worlds by default** (`workerCount` 1, no task callbacks).
  Box2D's multithreading wants its own task system, which would fight ebb's
  scheduler. It can be an opt-in later if ever needed.
- **Callbacks run synchronously inside `step!`**, on the calling fiber. **Never
  park (`m/?`, posting to a mailbox) inside a Box2D callback**: there's a C
  frame on the stack. The same is already true of ents system bodies.
- **No raylib calls inside the library**, ever, so it's safe to use from the
  sim fiber.

### 5.5 Errors

- Box2D asserts **abort the process**, which would kill the REPL. Build
  without `B2_ENABLE_ASSERT`, and have the shim check `b2Body_IsValid` /
  `b2Joint_IsValid` / `b2World_IsValid` on every id it receives. On a bad id it
  returns a failure code instead of calling in.
- The Clojure layer turns failure codes into `ex-info` with the id, name and op
  in the data. This is the same contract as ents (ops on a dead entity throw).
  An unknown name throws too.
- **Callback exceptions** are caught before they unwind through C, stored on
  the world, and rethrown from the `step!` (or query) that drove them, so the
  world stays usable. This is the ents mechanism, and it's the first thing
  that moves into `common/`.

### 5.6 Stepping

- `(b2/step! w dt substeps)` is a raw step.
- `(b2/step-fixed! w frame-dt)` runs the fixed timestep. It adds `frame-dt` to
  the world's accumulator and runs as many 1/60 s steps (4 substeps each) as
  fit, **up to `:max-steps`**, dropping any time beyond that (§4.7). It returns
  the leftover fraction (`alpha`) for anyone who wants render interpolation.
- `(b2/simulate w seconds opts)` is built on `step-fixed!` (§4.5).

### 5.7 Step 0 deliverable

The repo scaffold, the build tasks, and this minimal API:

```clojure
;; worlds
(b2/make-world {:gravity [0 9.8] :max-steps 5})   ; → world handle
(b2/destroy! w)
(b2/with-world [w opts] ...)                      ; make + destroy in finally

;; stepping
(b2/step! w dt substeps)
(b2/step-fixed! w frame-dt)                       ; → alpha
(b2/simulate w seconds {:sample [names-or-ids] :every dt})  ; → samples
(b2/series samples body field)  (b2/extent ...)  (b2/period ...)  (b2/settled-at samples body)

;; bodies (name optional; re-evaluating a named body replaces it)
(b2/body! w :toilet {:type :static|:dynamic|:kinematic
                     :pos [x y] :angle a
                     :fixed-rotation? bool
                     :user-data u64
                     :shapes [{:box [half-w half-h] :offset [x y]
                               :density d :friction f :restitution r}]})  ; → body id
(b2/destroy-body! w :toilet)
(b2/transform w :toilet)                          ; → {:x :y :angle}
(b2/velocity w :toilet)                           ; → {:vx :vy :w}

;; inspection
(b2/bodies w)
(b2/body-info w :toilet)
(b2/snapshot w)
```

`joints` and `joint-info` arrive with the first joint type (Step 5).

**Acceptance (headless tests):**
- a dynamic box dropped onto a static floor comes to rest on top of it within
  a couple of seconds, at the expected height (±1 cm), checked with `simulate`
- re-evaluating a named `body!` leaves exactly one body with that name
- `with-world` destroys the world even when the body throws
- destroying a body, then reading it, throws `ex-info` and doesn't crash; so
  does using a handle after `destroy!`
- `step-fixed!` with a 30-second frame runs only `:max-steps` steps
- two worlds stepped alternately don't affect each other's reads
- `body-info` and `snapshot` return plain data (no pointers or atoms)

---

## 6. Feature steps

Each step lists what the game needs, the library additions, the game
changes, and how we know it works. They're roughly in order, but steps 5–7
can be reordered depending on what's most fun to try next.

In every step, new logic starts in Clojure on top of small C calls, and moves
into C only once it has settled or proves too slow (§4.1). Where a step names
a C version, that's the eventual home, not the first draft.

### Step 1: the smallest usable step, "physics hop" ✅ done

*What changed while building it:*
- The shim got the whole body-motion batch at once (§4.1): `impulse!` and
  `force!` (center or at a point), `set-velocity!`, `set-angular-velocity!`,
  `set-transform!`, `set-awake!`, `set-gravity-scale!`, `set-damping!`, plus
  `contacts` and `gravity`.
- `grounded?` takes an options map, `{:min-normal 0.7 :max-separating-speed 0.1}`.
  The speed check was needed: Box2D's contact data is from the start of the
  last step, so right after a hop the floor contact is still there for one
  step. Without the check the game's "stop sliding on landing" killed every
  hop's sideways speed.
- The shim builds and reads rotations with exact `cosf`/`sinf`/`atan2`.
  Box2D's own `b2MakeRot` and `b2Rot_GetAngle` are fast approximations, off by
  up to ~1e-3 rad, so an angle set at the REPL didn't read back as itself.
- Static bodies report contacts too (the normal pointing into them).
- `body-info` includes `:contacts`.
- In the game, switching back from Box2D to the old movement pushes the
  toilet out of anything it overlaps first. Box2D lets bodies sink in by a
  fraction of a pixel, and the old collision code read that as a crash and
  pushed the toilet through the wall.
- The game's glue is `game.physics`. It keeps the Box2D world per flecs
  world (like ents keeps its per-world tables), so systems and
  `game.sim/send!` find it from the flecs world alone:
  `(game.sim/send! #(b2/body-info (game.physics/b2 %) :toilet))`. F6 toggles
  the movement (a key rather than a raygui control for now), and the editor
  shows which one is on.

**The smallest change that makes the library useful in the game:** the toilet
hops with Box2D gravity and an impulse on a Box2D floor, behind a dev-panel
toggle so it can be compared with the current movement.

*Library additions*
- `(b2/impulse! w body [ix iy])`: `b2Body_ApplyLinearImpulseToCenter`, wakes
  the body
- `(b2/contacts w body)` → realized vector of `{:other id :normal [x y] :touching? bool}`,
  from `b2Body_GetContactData` copied into a scratch buffer. This is the
  small C call.
- `(b2/grounded? w body min-normal)`, **in Clojure** on top of `contacts`: true
  if a touching contact's normal points up (against gravity) by at least
  `min-normal`. Easy to tweak live; it can move to C later if it ever shows up
  in the dev panel.
- `(b2/set-velocity! w body [vx vy])`: to stop sideways sliding on landing,
  so each hop stays one hop, as now

*Game changes*
- `game.physics` (or similar): the PPM conversion, plus making named static
  bodies from `level/edges` and the walls
- one Box2D world per level, made when the level is built and destroyed when
  it's cleared (§4.7). It's stepped with `step-fixed!` in a `:pre-update`
  system, and the handle is reachable from `game.sim/send!` (§4.6)
- the player entity gets a `Body {:id u64}` component, and the body is named
  `:toilet`. A system copies its transform into `Position` (meters → pixels;
  Position is the top-left of the hitbox, so subtract half the size)
- the hop is `impulse!` with `[±side, -up]` × mass when `grounded?`
- a dev-panel toggle picks old movement or Box2D movement

*Not in Step 1:* wall damage on the Box2D path. The toilet just bumps the
static wall. Step 3 adds that.

*Acceptance*
- headless: an impulse on a body resting on the floor lifts it, and it lands
  back with `grounded?` true; `grounded?` is false in mid-air (checked with
  `simulate`)
- in the game: the toilet hops left and right, can't hop in mid-air, and stops
  at walls and room ends; the toggle switches back to the old movement
- at the REPL: `(game.sim/send! #(b2/body-info (:b2 %) :toilet))` shows the
  toilet's live state

### Step 2: debug outlines ✅ done

*What changed while building it:*
- No shim changes, so no rebuild and no REPL restart. Everything is in
  Clojure: the registry's shape specs plus each body's `transform`, and one
  `mass-info` read per body for `:awake?` and `:type`.
- Each map also carries `:center` (the body's position) and `:shape` (the
  shape id), which the game uses to draw a dot at each body's centre.
- In the game, F7 toggles the outlines (static grey, moving green, asleep
  blue). The sim converts them to pixels and puts them in the snapshot only
  while they're on, so they cost nothing otherwise.
- While the hand-written movement is on, the game keeps the toilet's body on
  the drawn toilet (`game.physics/follow!`), so its outline is right in both
  modes and a switch to Box2D starts from the right place.

Seeing the shapes you just made matters most while tuning, so this comes
early.

*Library additions*
- `(b2/debug-shapes w)` → realized vector of plain maps, **built in Clojure**
  from the registry's shape specs and each body's `transform`:
  `{:kind :polygon :points [[x y] ...] :body id :name :toilet :awake? bool}`,
  plus `:circle` and `:capsule` as those shapes arrive. Joints add
  `{:kind :segment :from [x y] :to [x y]}` between their anchors.
- no raylib in the library. The game, or later a tiny optional
  `jolt-box/box2d-raylib` deps root, turns those maps into raylib calls

*Game changes*
- a dev-panel toggle draws the outlines inside the camera (meters → pixels),
  colour-coded for static, awake and asleep

*Later, in C:* Box2D's own `b2DebugDraw` callbacks receive `b2Transform` **by
value**, so they can't be `foreign-callable`s from jolt. If the body count
grows (Step 7's walls), the shim implements those callbacks in C, writes
primitives into a per-world buffer, and `debug-shapes` reads it in one call.
The output maps stay the same.

*Acceptance:* in the game, the outlines match the drawn toilet and walls, and
follow the toilet as it hops.

### Step 3: contact events, user data and momentum (wall damage) ✅ done

*What changed while building it:*
- **Events are collected after every step** (world option `:collect-events?`,
  on by default) and `contact-events` returns everything since the last call.
  Box2D only keeps the last step's events, and `step-fixed!` can run several
  steps in one frame, so reading them once a frame would lose some. Each kind
  is capped (4096 waiting, 512 per step) with a `:dropped` count.
- **Event maps carry body ids, names and user data for both sides**
  (`:body-a :name-a :user-a`, and `-b`), plus the shape ids, rather than only
  user data. `events-for` turns a list around so one body is the subject
  (`:other :other-name :other-user`, with `:normal` pointing into it, as in
  `contacts`). User data and names come from the registry, so they're still
  there in an `:end` event after a body was destroyed.
- `make-world` takes `:hit-event-threshold` (m/s, default Box2D's 1.0).
- Shape specs take `:contact-events?` / `:hit-events?` (both off by default,
  as in Box2D; either shape of a pair is enough).
- The shim batch also covers the rest of the shape settings, so Step 6's
  density slider needs no rebuild: `set-shape!` with `:friction`
  `:restitution` `:density` (updates the mass) and the two event flags, plus
  `shapes`, `shape-body`, `mass-data` (`:center` world, `:local-center`,
  `:inertia`), `mass` and `momentum` (`{:px :py}`).
- `set-transform!` documents that contacts lag a step after a teleport (the
  same lag as in Step 1).

*In the game:* a wall's damage is `floor(momentum / MOMENTUM-PER-HP)`, with
the momentum being toilet mass × approach speed from the hit event, and
`MOMENTUM-PER-HP` 2.0 kg·m/s in `config`. A full-speed hop (about 2.8)
knocks off one hitpoint and bounces back (`physics/bounce!`), so the dirt wall
still takes two hops, the same as the old movement. A gentle bump (1.6) does
nothing, and a toilet of twice the mass breaks it in one hop. The `Hit`
component got a `:damage` field; the old movement always uses 1. The value is
in config, not yet a dev-panel control.

Covers the "momentum calculations" goal. Wall damage comes from how hard
something hits, not a fixed hit count.

*Library additions*
- `:user-data` reads: `(b2/user-data w body)` and `shape → body`
- `:enable-hit-events?` and `:enable-contact-events?` on shape specs
- `(b2/contact-events w)` → `{:begin [{:a ua :b ub}] :end [...] :hit [{:a ua :b ub :point [x y] :normal [x y] :speed s}]}`,
  where `ua`/`ub` are the bodies' user data (e.g. flecs entities). The shim
  copies `b2World_GetContactEvents` into a flat per-world buffer in one call,
  since the events point into Box2D-owned arrays that only live until the
  next step
- `(b2/mass w body)`, plus the mass data (center, rotational inertia)
- `(b2/momentum w body)` → `{:px :py}`, a convenience for `m·v`, in Clojure

*Game changes*
- walls carry their entity id as user data
- damage = f(approach speed × player mass), a plain Clojure fn that's easy to
  redefine at the REPL. A tap dents and a full-speed hop smashes. This
  replaces `:bumped` and `hp 2`, with the thresholds in `config` and the dev
  panel
- `bounce-off` becomes restitution plus an impulse, or goes away

*Acceptance*
- headless: a body dropped from a greater height reports a higher hit
  `:speed`; user data round-trips
- in the game: walls take damage from hard hits only; the old path still works
  behind the toggle

### Step 4: sensors (optional cleanup of grounding) ✅ done

*What changed while building it:*
- Shape specs take `:sensor?` and `:sensor-events?`. **In Box2D v3.1 a sensor
  only sees shapes that have sensor events turned on** (static ones included),
  so the floor and walls need `:sensor-events? true`. A `:sensor?` shape turns
  its own sensor events on, and **weighs nothing unless given a `:density`**,
  so adding a foot sensor doesn't change the body's mass.
- `sensor-events` is collected per step exactly like `contact-events`
  (`{:begin :end :dropped}`, each event naming the sensor and visitor shape,
  body, name and user data). `sensor-overlaps` gives what a sensor overlaps
  now. `set-shape!` takes `:sensor-events?`. `debug-shapes` marks `:sensor?`.
- **Sensors don't lag.** Box2D updates them at the end of each step, so
  `sensor-overlaps` is right the moment a body takes off. That makes a foot
  sensor a better "standing" check than `grounded?` (which uses contacts,
  which lag a step).

*In the game:* the toilet's foot is a strip 90 px wide that pokes 1.5 px
below it. "Standing" is "the foot overlaps something". With 3 px it reported
ground about 2 px before touching down, so hops started 2 px high. A landing
(the foot's `:begin` event) only counts after a take-off, so spawning or an F6
teleport doesn't count. Each landing on the Box2D path makes a small puff of
floor dust. Sensors draw orange in the F7 outlines.

*Library:* `:sensor? true` on shape specs, `:enable-sensor-events?`, and
`(b2/sensor-events w)` → `{:begin [{:sensor us :visitor uv}] :end [...]}`.
*Game:* a thin foot sensor under the base replaces `grounded?`. This also
gives you "landed this frame" events for dust puffs and squash animation.
*Acceptance:* begin and end fire once per landing and per take-off.

### Step 5: spring jump (prismatic joint) ✅ done

*What changed while building it:*
- **Joint shim pattern.** Each joint type gets three C functions: `create`
  from a params array (so new fields don't change the signature), `set` one
  setting by selector, and `read` everything into a buffer. Plus generic
  `joint_destroy` / `bodies` / `read` (constraint force and torque,
  separations, local anchors) / `wake` / `set_collide_connected`. Revolute,
  distance and weld follow the same shape. `set` wakes the bodies, so a
  change reaches a sleeping body.
- **Joint specs are world space, kept in local frames.** `:anchor`,
  `:anchor-a`/`:anchor-b` and `:axis` are given where the bodies are now
  (default anchor: body b's origin), turned into local frames once, and kept
  in the registry as `:frame`. Re-evaluating a named body remakes its joints
  from that frame, with settings changed by `set-joint!` (which merges them
  into the stored spec). Destroying a body drops its joints from the registry.
- **Stale joint ids throw `ex-info`**, like bodies (`joint-entry` checks
  `jb2_joint_is_valid`).
- `:circle` and `:capsule` shapes, drawn by `debug-shapes` (`:kind :circle
  :at :radius`, `:kind :capsule :points :radius`); joints draw as `:kind
  :segment` between their anchors.
- **Box2D's springs lose energy each swing even at `:damping 0`**, about 15%
  per cycle at 2 Hz, from its stable soft-step solver. The period is right;
  the amplitude isn't conserved. Expect to tune by measuring (`simulate`).

*In the game (found by tuning with `simulate`):*
- The toilet is a base (40 px, 0.2 kg, with the foot sensor) and a bowl
  (68 px, 0.88 kg) on a vertical prismatic joint, `:limit [-0.3 0]`. Most of
  the weight is in the bowl so the yank at the limit wastes less speed. The
  total is still 1.08 kg, so `MOMENTUM-PER-HP` means the same.
- **13 Hz, 22 cm of squash** gives the old hop's 51 px. With a 0.2 kg base
  and damping 0.3 even 10 Hz only reached ~30 px.
- **Damping changes during a hop:** 1.0 (critically damped) on the ground
  and while charging, 0.3 from release to landing. Low damping all the time
  made it pogo 10–16 px back up after every landing; high damping all the
  time wasted the launch.
- **Sideways speed is given at take-off, not at release.** While the spring
  pushes, the base is pressed so hard into the floor that friction stopped it
  within a frame, and the joint dragged the bowl back with it.
- Holding charges for 0.25 s and launches by itself when full, so holding
  still keeps hopping. A tap charges to at least 0.6, a small hop (14 px up,
  41 px along). Hop height is roughly the square of the charge.
- The bowl, tank and head are drawn `:lift` px off the base, from the
  joint's translation, so the art follows the real spring.
- The settings are in `game.physics/spring` and can be changed live:
  `(game.sim/send! #(game.physics/tune! % {:hertz 14 :charge-time 0.2}))`.
  They're not in the editor panel yet.

The toilet becomes a **base** and a **bowl** joined by a vertical prismatic
joint with a spring. Holding the key squashes the bowl down. Releasing lets
the spring shove the bowl up, and at the upper limit it yanks the base off
the floor, so the hop comes out of the physics.

*Library additions*
- multi-shape bodies (already allowed by `:shapes`), plus `:circle` and
  `:capsule` shapes
- joints, with optional names like bodies:
  `(b2/joint! w :jump-spring {:type :prismatic :a :base :b :bowl :anchor-a [x y] :anchor-b [x y]
                              :axis [0 -1] :spring {:hertz h :damping z} :limit [lo hi]
                              :motor {:max-force f :speed s} :target t :collide-connected? false})`
  → joint id. Re-evaluating replaces it.
- runtime setters: `(b2/set-joint! w :jump-spring {:spring {...} :target t :motor {...}})`
  (`b2PrismaticJoint_SetTargetTranslation`, `SetSpringHertz`, etc.). Add the
  **whole** `b2PrismaticJoint_*` getter/setter set to the shim in this one
  batch (§4.1)
- reads: `(b2/joint-state w :jump-spring)` → `{:translation :speed}`
- `(b2/destroy-joint! w j)`, `(b2/joints w)`, `(b2/joint-info w j)`

*Game changes*
- the toilet entity has two bodies. Charge = lower the target or run the motor;
  release = restore it
- the squash and stretch is drawn from `:translation`, so the art follows the
  real spring
- hertz, damping, limits and charge time go in the raygui dev panel, and can
  be set from the REPL via `send!`. Optionally, a visual-only damped spring in
  plain Clojure for extra wobble on top
- tune first with `simulate` at the REPL (period, peak height), then in the game

*Acceptance*
- headless: a sprung prismatic joint released from compression oscillates at
  about the configured hertz (`period` of the samples, ±10%); limits hold
- in the game: a longer charge gives a higher hop; the spring settings are
  live-tunable

### Step 6: neck, head weight and wobble ✅ done

*Two library bugs found and fixed:*
- **A heap overrun since Step 5.** The per-world scratch buffer held 8
  doubles, but joint params (up to 18) and joint reads (up to 15) were
  written into it. It corrupted the heap silently and crashed the process
  several worlds later (seen as exit 133 in a tuning sweep). The buffer now
  holds 32 doubles, and `state-buf` throws `ex-info` before anything bigger
  can be written. A regression test makes many joints in many worlds.
- **Box2D's asserts were never off.** They're compiled in unless `NDEBUG` is
  defined, so the build now passes `-DNDEBUG` as Step 0 intended. The shim
  also installs an assert handler that reports on stderr, flushed, for
  anyone who builds without it to debug Box2D.

*What changed while building it:*
- `:revolute` and `:distance` joints, each with its full set of shim
  functions. Joints are now driven by a per-type table (`joint-kinds`:
  create, set, read, params, motor key), so adding a type means one entry.
  `set-joint!` takes `:target` (prismatic m, revolute radians) and `:length`
  (distance). Revolute `:speed` is computed from the bodies' spins (Box2D
  has no getter).
- **Collision filters** on shape specs: `:group` (same negative group never
  collides: the parts of one character), `:category`, `:mask`. Set with
  `b2Shape_SetFilter` after the shape is made, so the `add_*` signatures
  stayed put.
- `read-bodies`, a Clojure loop for now (§4.1).

*In the game (all found by measuring):*
- The toilet is base, bowl, a neck of a slider segment (prismatic: Up/Down
  raise it out of the bowl like a periscope, up to 80 px) and three hinged
  segments, and a circle head on a hinge. All in collision group -1.
- **Box2D sizes a hinge spring by the two bodies it joins**: stiffness =
  their effective inertia × (2π·hertz)². With feather-light segments under a
  heavy head, a fixed hertz can't hold the head up; at 0.02 kg segments and
  5 Hz the neck folded over at rest. So the neck's hertz is *derived*:
  `neck-hertz` works back from `k = stiffness × m_head × g × lever`, and the
  setting is `:neck-stiffness`, how many times stronger than gravity the neck
  is (default 5). Any head weight stays upright, and the wobble stays slow
  because the whole head swings on the neck.
- Segments are 0.08 kg (0.02 was too light for the solver), with the bowl
  lighter so the parts above the jump spring still weigh 0.88 kg.
- **Stopping and starting the hop moves the whole toilet's sideways speed,
  keeping each part's speed relative to the base** (`shift-sideways!`).
  Stopping only the base and bowl on landing whipped the neck over.
- Neck damping 0.4: a ~0.5 rad wobble after a hop that settles in about
  half a second. At 0.15 it flopped about ±1.7 rad.
- The jump spring went to 16 Hz to make up for what the neck soaks up:
  hops are 50–52 px up and 134–141 px along again.
- **Head weight is the trade-off:** at 1 kg the toilet weighs 1.83 kg, hops
  34 px instead of 52, and breaks the dirt wall in one hop instead of two.
- The editor has sliders for head weight (0.05–2 kg) and neck stiffness
  (1–8; below ~1.5 the head flops over). They reach the sim each frame as
  `:tuning`, and `physics/tune!` only applies what changed.
- Known quirk: right after some landings the toilet lifts 1–2 px for a
  few frames as the head's momentum settles into the neck's slider.
- *Later change:* the neck **curves and grows**. jolt-box got
  `jb2_joint_set_frame` (anchors, axis, reference angle) and `set-joint!`
  keys `:local-anchor-a :local-anchor-b :local-axis :reference-angle`, which
  also update the registry's `:frame` so outlines, `joint-info` and remade
  joints follow. The game's neck is a list of segments on hinges whose
  anchors move as it grows, with two modes on a dev-panel toggle: *stretch*
  (a fixed number of segments, all lengthening) and *segments* (hinges a
  fixed spacing apart; the lowest segment grows and a new one is added
  below it at each spacing, and removed again when shrinking). Lessons:
  each hinge's spring is derived from everything above it (head and
  segments) and from half a segment's inertia (Box2D sizes it by both
  bodies); a tall chain buckles below ~3× gravity; the neck only curves if
  it lags the jump (`:whip`, applied at take-off only: applying it in the
  every-frame "stop sliding" pumped the sway up); and the slow sway is only
  settled by *linear* damping on the neck and head (`:neck-drag`), not by
  the hinges' damping or angular damping.
- *Earlier change (game only):* Up/Down **grew** the neck instead of
  sliding it out of the bowl (a raised periscope neck floated above the
  bowl). It telescopes: a lower piece hinged to the bowl at the water line
  and an upper piece on a prismatic joint whose `:limit [len len]` is moved
  to the neck's length; limits held the head far more firmly than a spring
  did. The base hinge's spring is re-derived from the current length
  (`neck-hertz` takes the lever), so a 160 px neck holds a 1 kg head up. The
  neck sways straight rather than curving: a curvy neck that also grows
  needs moving joint anchors at runtime (`b2Joint_SetLocalAnchorA/B`), a
  small shim addition for later.

*Library additions*
- `:revolute` joints: `:spring`, `:limit [lo hi]` angles, `:motor`,
  `:reference-angle`; reads `{:angle :speed}`. The whole `b2RevoluteJoint_*`
  set goes into the shim in one batch
- `:distance` joints: `:length`, `:range [min max]`, `:spring`;
  `(b2/set-joint! w j {:length l})` (`b2DistanceJoint_SetLength`). The whole
  `b2DistanceJoint_*` set in one batch
- `(b2/set-density! w shape d)` → `b2Shape_SetDensity` +
  `b2Body_ApplyMassFromShapes`. Shape ids are in the registry, and
  `(b2/shapes w :head)` lists them
- `(b2/read-bodies w ids)` → one flat vector `[x y angle vx vy w ...]` per
  body. Start as a Clojure loop over `transform`/`velocity`. Move it into a
  single C call when the dev panel says so; Steps 6 and 7 are the first with
  enough bodies that it might

*Game changes*
- neck = 3–5 small segment bodies (`:neck-1` … `:neck-n`) joined by sprung
  revolute joints, plus distance joints that lengthen to extend the neck (or
  segments added at runtime); drawn as a smooth curve through the segment
  positions
- the head is a circle body; its weight is a dev-panel slider (density). A
  heavier head swings harder, shifts the centre of mass, and with Step 3 hits
  walls harder

*Acceptance*
- headless: a sprung revolute joint returns to its reference angle
  (`settled-at`); a distance joint's length change takes effect; raising
  density raises `mass`
- in the game: the head wobbles after hops, the neck extends and retracts, and
  head weight changes how the toilet moves

### Step 7: deformable, breakable walls ✅ done

*What changed while building it:*
- `:weld` joints (`:linear-hertz :angular-hertz :linear-damping
  :angular-damping`; 0 hertz is rigid), one more entry in `joint-kinds`.
- `joint-forces` returns `{:id :force :fx :fy :torque}`, and `break-joints!`
  takes either `{:max-force :max-torque}` or a predicate over those maps.
  The predicate turned out to be essential (see below).
- `awake?`, a cheap check, because **a sleeping body's joints report no
  force**: judging a sleeping wall's welds against their resting load would
  snap them all.
- Body specs take `:enable-sleep? :sleep-threshold :linear-damping
  :angular-damping :gravity-scale`.
- Everything stayed in Clojure: the sim update is ~0.8 ms beside an awake
  18-brick wall with outlines on, so no C batch readers or C debug draw yet.
- **Soft welds behave like the neck's hinges:** a weld's spring and damping
  are sized by the two light bodies it joins, so a column of soft welds
  swings slowly and stays underdamped even at damping 1. Walls use rigid
  welds.

*What measuring found (it took three designs):*
1. **Rigid welds, anchored to the floor:** the wall acts as one rigid
   lever. A hit snapped the two floor welds and the whole wall slid and
   tipped as one piece; a heavier toilet just snapped them sooner.
2. **Bricks standing on the floor, welded to each other:** better, but a
   weld holding up the bricks above it carries ~18 N standing still, as much
   as a hit adds, so a plain force limit can't tell standing from being hit.
   **Breaking on the load above each weld's resting baseline** fixed that
   (recorded after the level settles for a second). But letting snapped welds
   decide when the wall comes down was erratic: the snapping cascades (one
   hop brought it all down, or a stronger weld took five to eight), and a
   heavy head, hopping lower, hit the stiff bottom rows and did worse.
3. **What shipped: welds are the look, hitpoints decide the fall.** Welds
   snap at 16 N over their baseline, so the wall cracks and bends where it's
   hit. When it comes down is Step 3's momentum rule; at 0 hitpoints the
   rest of the welds snap and the bricks are shoved away from the toilet as
   debris, which vanishes in a puff after 3 s.

*In the game:* the dirt wall kind has `:pieces {:cols 2 :rows 9 :break-force
16.0}`: 18 bricks of 40 px with 25 welds, standing on the floor. With a
default head the first hop cracks most of the welds and bounces back, the
second brings it down; a 1 kg head brings it down in one; the toilet passes
through either way. Snapped welds shake the camera a little and throw a few
chunks. Bricks draw at their real angles (a `line-ex!` as thick as the brick,
plus its edges), and walls without `:pieces` keep the old static box. The
hand-written movement still sees a whole wall.

Box2D has only rigid bodies, so walls deform by being built from pieces.

*Library additions*
- `:weld` joints with soft `:linear-hertz` / `:angular-hertz` (above 0 lets the
  wall bend and sag). The whole `b2WeldJoint_*` set in one batch, plus
  `b2Joint_GetConstraintForce`
- `(b2/joint-forces w joint-ids)` → realized vector of constraint-force
  magnitudes. Clojure loop first; one C call once a wall has enough joints to
  matter
- `(b2/break-joints! w joint-ids max-force)` → the broken ids. Clojure first
  (so the breaking rule can be tweaked live); C once settled
- `:awake? false` and sleep settings on `body!`, so a wall of many pieces
  costs nothing until it's hit (naval-battle tuned these for rubble)
- wall pieces and their joints are unnamed, to skip the registry lookup on
  the hot path; the wall as a whole can be found through user data

*Game changes*
- a wall kind gets a `:pieces` layout (grid size, weld stiffness, break force)
- loose pieces become debris bodies, alongside or replacing the chunk
  particles; maybe swap a static wall for its pieces version only on first
  hit, to keep the body count low
- if the outlines get slow here, switch `debug-shapes` to the C version
  (Step 2)

*Acceptance*
- headless: a welded stack holds under gravity; a hard impulse breaks the
  joints near the impact and not the far ones
- in the game: walls bend and crack where they're hit, and break apart when
  hit hard enough; frame time stays acceptable in the dev panel

### Step 8: ents glue (only once it has settled in the game)

Start with the glue **in the game**. Move it into an optional
`jolt-box/box2d-ents` deps root once the shape has settled, so the core library
keeps no flecs dependency. Its expected shape:

- a `Body {:id u64}` component (and `Joint {:id u64}`)
- a step system (`:pre-update` or `:on-load`) calling `step-fixed!`
- a sync system copying `read-bodies` results into `Position` / `Velocity`
- **one Box2D world per level** (the default, §4.7), destroyed when `:level` is
  deleted, so bodies can't outlive their entities. For entities removed
  mid-level (debris, broken walls), an observer on Body removal destroys the
  body
- contact events turned into flecs events or components (e.g. a `Hit`, as now),
  using the user data → entity mapping from Step 3

---

## 7. Later: the Box3D library

When naval-battle moves over:

1. Pin a Box3D tag (`v0.1.0` was the first release, June 2026). Check
   whether it still needs cmake or builds with one clang call like Box2D.
2. Create `box3d/` with the same structure: `jb3_shim`, `jolt-box.box3d`
   (alias `b3`), and the same names and map keys where the concept matches
   (`body!`, `joint!`, `:spring {:hertz :damping}`, `contact-events`,
   `read-bodies`, `step-fixed!`, `simulate`, `body-info`, `snapshot`).
3. Move into `common/` whatever turns out to be identical in both: the callback
   trap, the registry, buffers, event-buffer → map helpers, the `simulate`
   sample helpers, maybe the build task helpers.
4. Port naval-battle's `voxel.box3d` uses one by one, keeping its tuned
   defaults (sleep threshold, damping) as explicit options, not baked into the
   shim.

---

## 8. Conventions checklist (every step)

- [ ] logic starts in Clojure on small C calls; it moves to C only once
      settled or measured slow (§4.1)
- [ ] new shim functions arrive as a full batch for that Box2D feature, so
      the next sessions don't need a restart
- [ ] shim function `jb2_<noun>_<verb>`, scalars and pointers only, validates
      every id
- [ ] declared in `jb2_shim.h` with a one-line comment on ownership and units
- [ ] raw binding in `jolt-box.box2d.ffi`, no logic
- [ ] wrapper in `jolt-box.box2d`: map in, map or realized vector out, throws
      `ex-info` on failure
- [ ] takes a name anywhere it takes an id; new state shows up in
      `body-info` / `joint-info` / `snapshot`
- [ ] callbacks take a var, so redefinition works
- [ ] headless test for the behaviour (using `simulate` where it fits), not
      just "it doesn't crash"
- [ ] no raylib, no ents, no ebb requires in the core library
- [ ] the README API list updated
