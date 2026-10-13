# jolt-box: to do later

What's left after Steps 0–7, roughly in the order to pick it up. The
implementation guide has the plan each item refers to.

## Next code work

### Ents glue (guide §6, Step 8)

The last unfinished feature step. Grow it inside the game first, then move it
into an optional `jolt-box/box2d-ents` deps root once it settles, so the core
library keeps no flecs dependency. Expected shape:

- `Body {:id u64}` and `Joint {:id u64}` components
- a step system calling `step-fixed!`
- a sync system copying body state into `Position` / `Velocity`
- one Box2D world per level, destroyed with the level; an observer on Body
  removal destroys bodies of entities removed mid-level
- contact events turned into flecs events or components, using user data
  as the entity id

The two read APIs below are best built as the sync system needs them.

### A raw `double-array` version of `read-bodies`

For render and sync loops: the same `[x y angle vx vy w ...]` as a
`double-array` instead of a vector. On jolt 0.8.21 it measured 0.91 µs per
body against 1.17 for `read-bodies` (`bulk-raw` in `jolt bench`); with the
jolt change below it would be about 0.03 µs. If jolt adds a typed
`read-into!`, a `read-bodies-into!` filling a caller's array would remove the
per-frame allocation too.

### Moved-body events

`b2World_GetBodyEvents` lists every body that moved in a step, with its
transform, user data and a `fellAsleep` flag. A sync system could read only
those, so resting bodies cost nothing. Follow the contact-event pattern: the
shim copies the events into a flat per-world buffer after every step, and
Clojure gathers them across the several steps one `step-fixed!` can run. It
has no velocity, so it complements `read-bodies` rather than replacing it.

## Only if measurements call for it

### The per-call setup in `read-bodies`

`read-bodies` is about 0.25 µs per body slower than the bare bulk call plus
copy: building the result vector, looking up names and writing the ids buffer
each call. Add a `jolt bench` case that times the name-and-ids part alone. If
it matters, a "body set" that resolves ids once and keeps them in the buffer
would remove it.

### Generate the raw FFI layer with jolt-bindgen

`jolt-box.box2d.ffi` is about 60 hand-written `defcfn`s mirroring
`jb2_shim.h`. jolt-bindgen could generate it from the header, along with a
layout test, so the bindings can't drift from the C. Worth it once keeping
them in sync by hand becomes a chore. See `../joltc-jlt` for a binding built
this way.

## Waiting on others

### Faster typed `read-array` in jolt

[jolt-proposal-typed-read-array.md](jolt-proposal-typed-read-array.md) is
written but not yet sent to the jolt project. Today copying doubles out of C
costs about 125 ns each, which is most of what `read-bodies` spends; the
proposal's prototype does it in about 3 ns. When it lands, re-run
`jolt bench`: `read-bodies` and `c/read-doubles-at` get faster with no change
here.

### Box3D (guide §7)

The parallel 3D library, once Box2D's API has settled.
