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

/* worlds: single-threaded (workerCount 1). hit_threshold is the approach
 * speed (m/s) above which hits are reported; negative keeps Box2D's default. */
uint32_t jb2_world_create(double gx, double gy, double hit_threshold);
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

/* Writes 6 doubles: mass, rotational inertia (about the center of mass),
 * local center x y, world center x y. */
int jb2_body_mass_data(uint64_t body, double *out);

/* shapes. flags: JB2_SHAPE_* bits. Returns the shape id, 0 if the body is
 * invalid or the box degenerate. Mass is updated. */
#define JB2_SHAPE_CONTACT_EVENTS 1
#define JB2_SHAPE_HIT_EVENTS 2
#define JB2_SHAPE_SENSOR 4          /* detects overlaps, doesn't collide */
#define JB2_SHAPE_SENSOR_EVENTS 8   /* a sensor reports, or a shape can be seen by sensors */
uint64_t jb2_body_add_box(uint64_t body, double half_w, double half_h, double ox, double oy,
                          double angle, double density, double friction, double restitution,
                          int flags);
uint64_t jb2_body_add_circle(uint64_t body, double cx, double cy, double radius, double density,
                             double friction, double restitution, int flags);
uint64_t jb2_body_add_capsule(uint64_t body, double x1, double y1, double x2, double y2,
                              double radius, double density, double friction, double restitution,
                              int flags);
int jb2_shape_is_valid(uint64_t shape);
uint64_t jb2_shape_body(uint64_t shape); /* 0 if invalid */
int jb2_shape_set_friction(uint64_t shape, double friction);
int jb2_shape_set_restitution(uint64_t shape, double restitution);
int jb2_shape_set_density(uint64_t shape, double density); /* and updates the body's mass */
int jb2_shape_enable_contact_events(uint64_t shape, int enable);
int jb2_shape_enable_hit_events(uint64_t shape, int enable);
int jb2_shape_enable_sensor_events(uint64_t shape, int enable);

/* The shapes a sensor shape overlaps right now (as of the last step), up to
 * cap (at most JB2_MAX_OVERLAPS). ids_out[i*2 ..] = shape, body. Returns the
 * number written, or JB2_BAD_ID (also for a shape that isn't a sensor). */
#define JB2_MAX_OVERLAPS 64
int jb2_shape_sensor_overlaps(uint64_t shape, uint64_t *ids_out, int cap);

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

/* Contact events from the last step. kind: JB2_EVENTS_*. Copies up to cap
 * events and returns how many there were in total (which can be more than
 * cap), or JB2_BAD_ID. For event i:
 *   ids_out[i*4 ..]  shape a, shape b, body a, body b (a body is 0 if its
 *                    shape has since been destroyed, as end events can be)
 *   data_out[i*5 ..] nx ny px py extra
 *     begin: normal (from a to b), first manifold point, extra = point count
 *     end:   all 0
 *     hit:   normal (from a to b), point, extra = approach speed (m/s) */
#define JB2_EVENTS_BEGIN 0
#define JB2_EVENTS_END 1
#define JB2_EVENTS_HIT 2
#define JB2_EVENT_IDS 4
#define JB2_EVENT_DOUBLES 5
int jb2_world_contact_events(uint32_t world, int kind, uint64_t *ids_out, double *data_out, int cap);

/* Sensor events from the last step. kind: JB2_EVENTS_BEGIN or JB2_EVENTS_END.
 * Copies up to cap and returns the total (or JB2_BAD_ID).
 * ids_out[i*4 ..] = sensor shape, visitor shape, sensor body, visitor body
 * (a body is 0 if its shape has since been destroyed). */
int jb2_world_sensor_events(uint32_t world, int kind, uint64_t *ids_out, int cap);

/* --- joints ---------------------------------------------------------------
 * Joint ids cross as uint64 (b2StoreJointId). Each joint type has three
 * functions: create (from a params array, so new fields don't change the
 * signature), set (one setting at a time, by selector) and read (every
 * setting and state into an out array). Anchors and axes are in the bodies'
 * local frames, as in Box2D's defs. */

int jb2_joint_is_valid(uint64_t joint);
int jb2_joint_destroy(uint64_t joint);
int jb2_joint_wake(uint64_t joint);
int jb2_joint_set_collide_connected(uint64_t joint, int collide);

/* Writes 2 ids: body a, body b. */
int jb2_joint_bodies(uint64_t joint, uint64_t *out);

/* Writes JB2_JOINT_DOUBLES doubles:
 *   0 type  1 fx  2 fy  3 torque  4 linear sep  5 angular sep
 *   6 collide connected  7 ax  8 ay  9 bx  10 by  (local anchors) */
#define JB2_JOINT_DOUBLES 11
int jb2_joint_read(uint64_t joint, double *out);

/* prismatic: body b slides along an axis fixed in body a.
 * params (JB2_PRISMATIC_PARAMS doubles):
 *   0 1 local anchor a   2 3 local anchor b   4 5 local axis a (unit)
 *   6 reference angle    7 target translation
 *   8 spring on  9 hertz  10 damping ratio
 *   11 limit on  12 lower  13 upper
 *   14 motor on  15 max motor force  16 motor speed
 *   17 collide connected
 * Returns the joint id, 0 if a body is invalid. */
#define JB2_PRISMATIC_PARAMS 18
uint64_t jb2_prismatic_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                              const double *params, uint64_t user_data);

/* what: */
#define JB2_SET_SPRING_ON 0     /* v1 0/1 */
#define JB2_SET_SPRING_HERTZ 1
#define JB2_SET_SPRING_DAMPING 2
#define JB2_SET_TARGET 3        /* target translation (prismatic) */
#define JB2_SET_LIMIT_ON 4
#define JB2_SET_LIMITS 5        /* v1 lower, v2 upper */
#define JB2_SET_MOTOR_ON 6
#define JB2_SET_MOTOR_SPEED 7
#define JB2_SET_MAX_MOTOR 8     /* max motor force (prismatic) */
int jb2_prismatic_set(uint64_t joint, int what, double v1, double v2);

/* Writes JB2_PRISMATIC_READ doubles:
 *   0 translation  1 speed  2 motor force
 *   3 spring on  4 hertz  5 damping  6 target
 *   7 limit on  8 lower  9 upper
 *   10 motor on  11 motor speed  12 max motor force
 *   13 14 local axis a */
#define JB2_PRISMATIC_READ 15
int jb2_prismatic_read(uint64_t joint, double *out);

#endif
