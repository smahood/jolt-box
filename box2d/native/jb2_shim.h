#ifndef JOLT_BOX_B2_SHIM_H
#define JOLT_BOX_B2_SHIM_H

#include <stdint.h>

/* Flat C ABI over the Box2D v3.1 pieces jolt-box needs. jolt.ffi can't pass
 * structs by value, so everything here is scalars and pointers.
 *
 * Ids cross as the integers b2Store*Id produce: worlds as uint32, bodies and
 * shapes as uint64. A valid id is never 0, so 0 means "failed".
 *
 * Every function validates the ids it is given before calling into Box2D:
 * Box2D asserts abort the process (and the REPL with it). Functions that
 * return int return JB2_OK, or JB2_BAD_ID when an id was stale or invalid.
 *
 * Units are Box2D's: meters, kilograms, seconds, radians. Doubles cross the
 * boundary; Box2D stores floats. */

#define JB2_OK 0
#define JB2_BAD_ID (-1)

/* worlds: single-threaded (workerCount 1) */
uint32_t jb2_world_create(double gx, double gy);
int jb2_world_destroy(uint32_t world);
int jb2_world_is_valid(uint32_t world);
int jb2_world_step(uint32_t world, double dt, int substeps);

/* bodies. type: 0 static, 1 kinematic, 2 dynamic (b2BodyType). user_data is
 * stored in Box2D's void* userData (a flecs entity id fits). */
uint64_t jb2_body_create(uint32_t world, int type, double x, double y, double angle,
                         int fixed_rotation, int awake, uint64_t user_data);
int jb2_body_destroy(uint64_t body); /* also destroys its shapes and joints */
int jb2_body_is_valid(uint64_t body);

/* Writes 6 doubles: x y angle vx vy w. */
int jb2_body_read(uint64_t body, double *out);

/* Writes 4 doubles: mass, rotational inertia, awake (0/1), type. */
int jb2_body_mass_info(uint64_t body, double *out);

uint64_t jb2_body_user_data(uint64_t body); /* 0 if invalid */

/* shapes. Returns the shape id, 0 if the body is invalid. Mass is updated. */
uint64_t jb2_body_add_box(uint64_t body, double half_w, double half_h, double ox, double oy,
                          double angle, double density, double friction, double restitution);

/* body motion. All return JB2_OK or JB2_BAD_ID. Impulses and forces are in
 * world space; at_point 0 applies at the center of mass, 1 at (px, py). */
int jb2_body_apply_impulse(uint64_t body, double ix, double iy, int at_point, double px, double py,
                           int wake);
int jb2_body_apply_force(uint64_t body, double fx, double fy, int at_point, double px, double py,
                         int wake);
int jb2_body_set_velocity(uint64_t body, double vx, double vy);
int jb2_body_set_angular_velocity(uint64_t body, double w);
int jb2_body_set_transform(uint64_t body, double x, double y, double angle); /* a teleport */
int jb2_body_set_awake(uint64_t body, int awake);
int jb2_body_set_gravity_scale(uint64_t body, double scale);
int jb2_body_set_damping(uint64_t body, double linear, double angular);

/* Touching contacts of a body, up to cap (at most JB2_MAX_CONTACTS). For each
 * contact i, ids_out[i] is the other body and data_out[i*6 ..] holds:
 *   nx ny     the normal, pointing from the other body into this one
 *   points    manifold point count (0..2)
 *   px py     the first point's world position (0 0 if none)
 *   sep       the smallest point separation (negative = overlapping)
 * Returns the number written, or JB2_BAD_ID. */
#define JB2_MAX_CONTACTS 64
#define JB2_CONTACT_DOUBLES 6
int jb2_body_contacts(uint64_t body, uint64_t *ids_out, double *data_out, int cap);

#endif
