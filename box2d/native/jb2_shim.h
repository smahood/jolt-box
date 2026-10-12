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

#endif
