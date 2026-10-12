# jolt-box

Box2D (and later Box3D) physics for [Jolt](https://github.com/jolt-lang/jolt),
built the same way as ents: a flat C shim compiled with the engine source into
one dylib, a raw `jolt.ffi` layer, and an idiomatic Clojure API.

- `box2d/` — Box2D v3.1.1 wrapper (in progress)
- `box3d/` — later
- `common/` — small dimension-free helpers shared by both

See [guide/implementation-guide.md](guide/implementation-guide.md) for the plan.
