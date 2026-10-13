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

/* The version of this ABI: it goes up whenever a signature, a layout or a
 * constant here changes. jolt-box.box2d/available? compares it with the
 * version the Clojure side was written against, so a stale dylib is caught
 * at load instead of misreading buffers. */
#define JB2_VERSION 1
uint32_t jb2_version(void);

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

/* jb2_body_read for count bodies in one call: out gets 6 doubles per body,
 * in the order of ids. Stops at the first invalid id and returns how many
 * were read, so a return below count means ids[return] is stale. */
int jb2_bodies_read(const uint64_t *ids, int count, double *out);

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

/* Collision filtering (b2Filter). Shapes collide when each one's category
 * is in the other's mask, unless they share a group: the same negative group
 * never collides, the same positive group always does. Defaults: category 1,
 * mask all, group 0. */
int jb2_shape_set_filter(uint64_t shape, uint64_t category, uint64_t mask, int group);

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
/* enable 0/1; threshold (m/s) < 0 keeps the current one */
int jb2_body_set_sleep(uint64_t body, int enable, double threshold);

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

/* Moves where a joint attaches while the world runs. what: JB2_FRAME_*.
 * Anchors and the axis are in the body's local frame; the axis is
 * normalized. Wakes the bodies. */
#define JB2_FRAME_ANCHOR_A 0   /* x y */
#define JB2_FRAME_ANCHOR_B 1   /* x y */
#define JB2_FRAME_AXIS_A 2     /* x y (prismatic, wheel) */
#define JB2_FRAME_REFERENCE 3  /* x = reference angle (prismatic, revolute, weld) */
int jb2_joint_set_frame(uint64_t joint, int what, double x, double y);

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
#define JB2_SET_MAX_MOTOR 8     /* max motor force (prismatic, distance) or torque (revolute) */
#define JB2_SET_LENGTH 9        /* rest length (distance) */
#define JB2_SET_LINEAR_HERTZ 10    /* weld */
#define JB2_SET_ANGULAR_HERTZ 11   /* weld */
#define JB2_SET_LINEAR_DAMPING 12  /* weld */
#define JB2_SET_ANGULAR_DAMPING 13 /* weld */
int jb2_prismatic_set(uint64_t joint, int what, double v1, double v2);

/* Writes JB2_PRISMATIC_READ doubles:
 *   0 translation  1 speed  2 motor force
 *   3 spring on  4 hertz  5 damping  6 target
 *   7 limit on  8 lower  9 upper
 *   10 motor on  11 motor speed  12 max motor force
 *   13 14 local axis a */
#define JB2_PRISMATIC_READ 15
int jb2_prismatic_read(uint64_t joint, double *out);

/* revolute: a hinge. body b turns about an anchor on body a.
 * params (JB2_REVOLUTE_PARAMS doubles):
 *   0 1 local anchor a   2 3 local anchor b   4 reference angle  5 target angle
 *   6 spring on  7 hertz  8 damping ratio
 *   9 limit on  10 lower angle  11 upper angle
 *   12 motor on  13 max motor torque  14 motor speed
 *   15 collide connected */
#define JB2_REVOLUTE_PARAMS 16
uint64_t jb2_revolute_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                             const double *params, uint64_t user_data);
/* what: JB2_SET_* (JB2_SET_TARGET is the target angle) */
int jb2_revolute_set(uint64_t joint, int what, double v1, double v2);
/* Writes JB2_REVOLUTE_READ doubles:
 *   0 angle  1 motor torque
 *   2 spring on  3 hertz  4 damping  5 target angle
 *   6 limit on  7 lower  8 upper
 *   9 motor on  10 motor speed  11 max motor torque */
#define JB2_REVOLUTE_READ 12
int jb2_revolute_read(uint64_t joint, double *out);

/* distance: keeps an anchor on each body a set length apart, rigidly or on
 * a spring, optionally within a range.
 * params (JB2_DISTANCE_PARAMS doubles):
 *   0 1 local anchor a   2 3 local anchor b   4 rest length
 *   5 spring on  6 hertz  7 damping ratio
 *   8 limit on  9 min length  10 max length
 *   11 motor on  12 max motor force  13 motor speed
 *   14 collide connected */
#define JB2_DISTANCE_PARAMS 15
uint64_t jb2_distance_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                             const double *params, uint64_t user_data);
/* what: JB2_SET_* (JB2_SET_LENGTH for the rest length, JB2_SET_LIMITS for the range) */
int jb2_distance_set(uint64_t joint, int what, double v1, double v2);
/* Writes JB2_DISTANCE_READ doubles:
 *   0 current length  1 motor force  2 rest length
 *   3 spring on  4 hertz  5 damping
 *   6 limit on  7 min  8 max
 *   9 motor on  10 motor speed  11 max motor force */
#define JB2_DISTANCE_READ 12
int jb2_distance_read(uint64_t joint, double *out);

/* weld: holds b fixed to a, rigidly (hertz 0) or springily.
 * params (JB2_WELD_PARAMS doubles):
 *   0 1 local anchor a   2 3 local anchor b   4 reference angle
 *   5 linear hertz  6 angular hertz  7 linear damping  8 angular damping
 *   9 collide connected */
#define JB2_WELD_PARAMS 10
uint64_t jb2_weld_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                         const double *params, uint64_t user_data);
/* what: JB2_SET_LINEAR_HERTZ .. JB2_SET_ANGULAR_DAMPING */
int jb2_weld_set(uint64_t joint, int what, double v1, double v2);
/* Writes JB2_WELD_READ doubles: linear hertz, angular hertz, linear damping,
 * angular damping. */
#define JB2_WELD_READ 4
int jb2_weld_read(uint64_t joint, double *out);

#endif
