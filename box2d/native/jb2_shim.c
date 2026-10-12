// SPDX-License-Identifier: MIT
// Flat scalar/pointer shim over Box2D v3.1 for jolt.ffi. See jb2_shim.h.
#include "jb2_shim.h"

#include <box2d/box2d.h>
#include <math.h>
#include <stdio.h>

/* b2MakeRot uses a fast approximation that is off by up to ~1e-3 rad, so an
 * angle set from the REPL wouldn't read back as itself. Exact is cheap here. */
static b2Rot jb2_rot(double angle)
{
	return (b2Rot){ cosf((float)angle), sinf((float)angle) };
}

/* --- worlds --------------------------------------------------------------- */

/* Box2D reports a failed assert with printf, which is lost if the process
 * then stops; say it on stderr, flushed, so it can always be seen. Release
 * builds (-DNDEBUG) compile Box2D's asserts out entirely. */
static int jb2_assert(const char *condition, const char *file, int line)
{
	fprintf(stderr, "[jolt-box] BOX2D ASSERTION: %s, %s, line %d\n", condition, file, line);
	fflush(stderr);
	return 1; /* stop, as Box2D would */
}

uint32_t jb2_world_create(double gx, double gy, double hit_threshold)
{
	b2SetAssertFcn(jb2_assert);
	b2WorldDef def = b2DefaultWorldDef();
	def.gravity = (b2Vec2){ (float)gx, (float)gy };
	if (hit_threshold >= 0.0) def.hitEventThreshold = (float)hit_threshold;
	def.workerCount = 1;
	return b2StoreWorldId(b2CreateWorld(&def));
}

int jb2_world_is_valid(uint32_t world)
{
	return world != 0 && b2World_IsValid(b2LoadWorldId(world));
}

int jb2_world_destroy(uint32_t world)
{
	if (!jb2_world_is_valid(world)) return JB2_BAD_ID;
	b2DestroyWorld(b2LoadWorldId(world));
	return JB2_OK;
}

int jb2_world_step(uint32_t world, double dt, int substeps)
{
	if (!jb2_world_is_valid(world)) return JB2_BAD_ID;
	b2World_Step(b2LoadWorldId(world), (float)dt, substeps);
	return JB2_OK;
}

/* --- bodies --------------------------------------------------------------- */

int jb2_body_is_valid(uint64_t body)
{
	return body != 0 && b2Body_IsValid(b2LoadBodyId(body));
}

uint64_t jb2_body_create(uint32_t world, int type, double x, double y, double angle,
                         int fixed_rotation, int awake, uint64_t user_data)
{
	if (!jb2_world_is_valid(world)) return 0;
	if (type < 0 || type > 2) return 0;
	b2BodyDef def = b2DefaultBodyDef();
	def.type = (b2BodyType)type;
	def.position = (b2Vec2){ (float)x, (float)y };
	def.rotation = jb2_rot(angle);
	def.fixedRotation = fixed_rotation != 0;
	def.isAwake = awake != 0;
	def.userData = (void*)(uintptr_t)user_data;
	return b2StoreBodyId(b2CreateBody(b2LoadWorldId(world), &def));
}

int jb2_body_destroy(uint64_t body)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2DestroyBody(b2LoadBodyId(body));
	return JB2_OK;
}

int jb2_body_read(uint64_t body, double *out)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2Vec2 p = b2Body_GetPosition(id);
	b2Vec2 v = b2Body_GetLinearVelocity(id);
	out[0] = p.x;
	out[1] = p.y;
	b2Rot q = b2Body_GetRotation(id);
	out[2] = atan2(q.s, q.c); /* b2Rot_GetAngle approximates atan2 too */
	out[3] = v.x;
	out[4] = v.y;
	out[5] = b2Body_GetAngularVelocity(id);
	return JB2_OK;
}

int jb2_body_mass_info(uint64_t body, double *out)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	out[0] = b2Body_GetMass(id);
	out[1] = b2Body_GetRotationalInertia(id);
	out[2] = b2Body_IsAwake(id) ? 1.0 : 0.0;
	out[3] = (double)b2Body_GetType(id);
	return JB2_OK;
}

uint64_t jb2_body_user_data(uint64_t body)
{
	if (!jb2_body_is_valid(body)) return 0;
	return (uint64_t)(uintptr_t)b2Body_GetUserData(b2LoadBodyId(body));
}

/* --- shapes --------------------------------------------------------------- */

static b2ShapeDef jb2_shape_def(double density, double friction, double restitution, int flags)
{
	b2ShapeDef sd = b2DefaultShapeDef();
	sd.density = (float)density;
	sd.material.friction = (float)friction;
	sd.material.restitution = (float)restitution;
	sd.enableContactEvents = (flags & JB2_SHAPE_CONTACT_EVENTS) != 0;
	sd.enableHitEvents = (flags & JB2_SHAPE_HIT_EVENTS) != 0;
	sd.isSensor = (flags & JB2_SHAPE_SENSOR) != 0;
	sd.enableSensorEvents = (flags & JB2_SHAPE_SENSOR_EVENTS) != 0;
	return sd;
}


uint64_t jb2_body_add_box(uint64_t body, double half_w, double half_h, double ox, double oy,
                          double angle, double density, double friction, double restitution,
                          int flags)
{
	if (!jb2_body_is_valid(body)) return 0;
	if (!(half_w > 0.0) || !(half_h > 0.0)) return 0;
	b2ShapeDef sd = jb2_shape_def(density, friction, restitution, flags);
	b2Polygon box = b2MakeOffsetBox((float)half_w, (float)half_h,
	                                (b2Vec2){ (float)ox, (float)oy }, jb2_rot(angle));
	return b2StoreShapeId(b2CreatePolygonShape(b2LoadBodyId(body), &sd, &box));
}

/* --- body motion ---------------------------------------------------------- */

int jb2_body_apply_impulse(uint64_t body, double ix, double iy, int at_point, double px, double py,
                           int wake)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2Vec2 impulse = { (float)ix, (float)iy };
	if (at_point)
		b2Body_ApplyLinearImpulse(id, impulse, (b2Vec2){ (float)px, (float)py }, wake != 0);
	else
		b2Body_ApplyLinearImpulseToCenter(id, impulse, wake != 0);
	return JB2_OK;
}

int jb2_body_apply_force(uint64_t body, double fx, double fy, int at_point, double px, double py,
                         int wake)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2Vec2 force = { (float)fx, (float)fy };
	if (at_point)
		b2Body_ApplyForce(id, force, (b2Vec2){ (float)px, (float)py }, wake != 0);
	else
		b2Body_ApplyForceToCenter(id, force, wake != 0);
	return JB2_OK;
}

int jb2_body_set_velocity(uint64_t body, double vx, double vy)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2Body_SetLinearVelocity(b2LoadBodyId(body), (b2Vec2){ (float)vx, (float)vy });
	return JB2_OK;
}

int jb2_body_set_angular_velocity(uint64_t body, double w)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2Body_SetAngularVelocity(b2LoadBodyId(body), (float)w);
	return JB2_OK;
}

int jb2_body_set_transform(uint64_t body, double x, double y, double angle)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2Body_SetTransform(b2LoadBodyId(body), (b2Vec2){ (float)x, (float)y }, jb2_rot(angle));
	return JB2_OK;
}

int jb2_body_set_awake(uint64_t body, int awake)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2Body_SetAwake(b2LoadBodyId(body), awake != 0);
	return JB2_OK;
}

int jb2_body_set_gravity_scale(uint64_t body, double scale)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2Body_SetGravityScale(b2LoadBodyId(body), (float)scale);
	return JB2_OK;
}

int jb2_body_set_damping(uint64_t body, double linear, double angular)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2Body_SetLinearDamping(id, (float)linear);
	b2Body_SetAngularDamping(id, (float)angular);
	return JB2_OK;
}

int jb2_body_set_sleep(uint64_t body, int enable, double threshold)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2Body_EnableSleep(id, enable != 0);
	if (threshold >= 0.0) b2Body_SetSleepThreshold(id, (float)threshold);
	return JB2_OK;
}

/* --- contacts ------------------------------------------------------------- */

int jb2_body_contacts(uint64_t body, uint64_t *ids_out, double *data_out, int cap)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	if (cap > JB2_MAX_CONTACTS) cap = JB2_MAX_CONTACTS;
	if (cap < 0) cap = 0;
	b2BodyId id = b2LoadBodyId(body);
	b2ContactData contacts[JB2_MAX_CONTACTS];
	int n = b2Body_GetContactData(id, contacts, cap);
	for (int i = 0; i < n; ++i)
	{
		const b2ContactData *c = &contacts[i];
		b2BodyId a = b2Shape_GetBody(c->shapeIdA);
		b2BodyId b = b2Shape_GetBody(c->shapeIdB);
		/* the manifold normal points from A to B; flip it so it points into us */
		int we_are_a = b2StoreBodyId(a) == body;
		float sign = we_are_a ? -1.0f : 1.0f;
		const b2Manifold *m = &c->manifold;
		double *d = data_out + i * JB2_CONTACT_DOUBLES;
		ids_out[i] = b2StoreBodyId(we_are_a ? b : a);
		d[0] = sign * m->normal.x;
		d[1] = sign * m->normal.y;
		d[2] = m->pointCount;
		d[3] = m->pointCount > 0 ? m->points[0].point.x : 0.0;
		d[4] = m->pointCount > 0 ? m->points[0].point.y : 0.0;
		float sep = m->pointCount > 0 ? m->points[0].separation : 0.0f;
		if (m->pointCount > 1 && m->points[1].separation < sep) sep = m->points[1].separation;
		d[5] = sep;
	}
	return n;
}

int jb2_body_mass_data(uint64_t body, double *out)
{
	if (!jb2_body_is_valid(body)) return JB2_BAD_ID;
	b2BodyId id = b2LoadBodyId(body);
	b2MassData md = b2Body_GetMassData(id);
	b2Vec2 wc = b2Body_GetWorldCenterOfMass(id);
	out[0] = md.mass;
	out[1] = md.rotationalInertia;
	out[2] = md.center.x;
	out[3] = md.center.y;
	out[4] = wc.x;
	out[5] = wc.y;
	return JB2_OK;
}

uint64_t jb2_body_add_circle(uint64_t body, double cx, double cy, double radius, double density,
                             double friction, double restitution, int flags)
{
	if (!jb2_body_is_valid(body)) return 0;
	if (!(radius > 0.0)) return 0;
	b2ShapeDef sd = jb2_shape_def(density, friction, restitution, flags);
	b2Circle circle = { { (float)cx, (float)cy }, (float)radius };
	return b2StoreShapeId(b2CreateCircleShape(b2LoadBodyId(body), &sd, &circle));
}

uint64_t jb2_body_add_capsule(uint64_t body, double x1, double y1, double x2, double y2,
                              double radius, double density, double friction, double restitution,
                              int flags)
{
	if (!jb2_body_is_valid(body)) return 0;
	if (!(radius > 0.0)) return 0;
	/* Box2D wants the two centers apart; a zero-length capsule is a circle */
	if (x1 == x2 && y1 == y2) return 0;
	b2ShapeDef sd = jb2_shape_def(density, friction, restitution, flags);
	b2Capsule capsule = { { (float)x1, (float)y1 }, { (float)x2, (float)y2 }, (float)radius };
	return b2StoreShapeId(b2CreateCapsuleShape(b2LoadBodyId(body), &sd, &capsule));
}

/* --- shape settings ------------------------------------------------------- */

int jb2_shape_is_valid(uint64_t shape)
{
	return shape != 0 && b2Shape_IsValid(b2LoadShapeId(shape));
}

uint64_t jb2_shape_body(uint64_t shape)
{
	if (!jb2_shape_is_valid(shape)) return 0;
	return b2StoreBodyId(b2Shape_GetBody(b2LoadShapeId(shape)));
}

int jb2_shape_set_friction(uint64_t shape, double friction)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_SetFriction(b2LoadShapeId(shape), (float)friction);
	return JB2_OK;
}

int jb2_shape_set_restitution(uint64_t shape, double restitution)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_SetRestitution(b2LoadShapeId(shape), (float)restitution);
	return JB2_OK;
}

int jb2_shape_set_density(uint64_t shape, double density)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_SetDensity(b2LoadShapeId(shape), (float)density, true);
	return JB2_OK;
}

int jb2_shape_enable_contact_events(uint64_t shape, int enable)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_EnableContactEvents(b2LoadShapeId(shape), enable != 0);
	return JB2_OK;
}

int jb2_shape_enable_hit_events(uint64_t shape, int enable)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_EnableHitEvents(b2LoadShapeId(shape), enable != 0);
	return JB2_OK;
}

int jb2_shape_set_filter(uint64_t shape, uint64_t category, uint64_t mask, int group)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Filter filter = { category, mask, group };
	b2Shape_SetFilter(b2LoadShapeId(shape), filter);
	return JB2_OK;
}

/* --- contact events ------------------------------------------------------- */

static void jb2_event_ids(b2ShapeId a, b2ShapeId b, uint64_t *ids)
{
	ids[0] = b2StoreShapeId(a);
	ids[1] = b2StoreShapeId(b);
	ids[2] = b2Shape_IsValid(a) ? b2StoreBodyId(b2Shape_GetBody(a)) : 0;
	ids[3] = b2Shape_IsValid(b) ? b2StoreBodyId(b2Shape_GetBody(b)) : 0;
}

int jb2_world_contact_events(uint32_t world, int kind, uint64_t *ids_out, double *data_out, int cap)
{
	if (!jb2_world_is_valid(world)) return JB2_BAD_ID;
	b2ContactEvents ev = b2World_GetContactEvents(b2LoadWorldId(world));
	int total = kind == JB2_EVENTS_BEGIN ? ev.beginCount
	          : kind == JB2_EVENTS_END   ? ev.endCount
	          : kind == JB2_EVENTS_HIT   ? ev.hitCount : 0;
	int n = total < cap ? total : cap;
	for (int i = 0; i < n; ++i)
	{
		uint64_t *ids = ids_out + i * JB2_EVENT_IDS;
		double *d = data_out + i * JB2_EVENT_DOUBLES;
		d[0] = d[1] = d[2] = d[3] = d[4] = 0.0;
		if (kind == JB2_EVENTS_BEGIN)
		{
			const b2ContactBeginTouchEvent *e = &ev.beginEvents[i];
			jb2_event_ids(e->shapeIdA, e->shapeIdB, ids);
			d[0] = e->manifold.normal.x;
			d[1] = e->manifold.normal.y;
			if (e->manifold.pointCount > 0)
			{
				d[2] = e->manifold.points[0].point.x;
				d[3] = e->manifold.points[0].point.y;
			}
			d[4] = e->manifold.pointCount;
		}
		else if (kind == JB2_EVENTS_END)
		{
			const b2ContactEndTouchEvent *e = &ev.endEvents[i];
			jb2_event_ids(e->shapeIdA, e->shapeIdB, ids);
		}
		else
		{
			const b2ContactHitEvent *e = &ev.hitEvents[i];
			jb2_event_ids(e->shapeIdA, e->shapeIdB, ids);
			d[0] = e->normal.x;
			d[1] = e->normal.y;
			d[2] = e->point.x;
			d[3] = e->point.y;
			d[4] = e->approachSpeed;
		}
	}
	return total;
}

/* --- sensors -------------------------------------------------------------- */

int jb2_shape_enable_sensor_events(uint64_t shape, int enable)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2Shape_EnableSensorEvents(b2LoadShapeId(shape), enable != 0);
	return JB2_OK;
}

int jb2_shape_sensor_overlaps(uint64_t shape, uint64_t *ids_out, int cap)
{
	if (!jb2_shape_is_valid(shape)) return JB2_BAD_ID;
	b2ShapeId id = b2LoadShapeId(shape);
	if (!b2Shape_IsSensor(id)) return JB2_BAD_ID;
	if (cap > JB2_MAX_OVERLAPS) cap = JB2_MAX_OVERLAPS;
	if (cap < 0) cap = 0;
	b2ShapeId overlaps[JB2_MAX_OVERLAPS];
	int n = b2Shape_GetSensorOverlaps(id, overlaps, cap);
	int written = 0;
	for (int i = 0; i < n; ++i)
	{
		if (!b2Shape_IsValid(overlaps[i])) continue; /* destroyed since the step */
		ids_out[written * 2] = b2StoreShapeId(overlaps[i]);
		ids_out[written * 2 + 1] = b2StoreBodyId(b2Shape_GetBody(overlaps[i]));
		++written;
	}
	return written;
}

int jb2_world_sensor_events(uint32_t world, int kind, uint64_t *ids_out, int cap)
{
	if (!jb2_world_is_valid(world)) return JB2_BAD_ID;
	b2SensorEvents ev = b2World_GetSensorEvents(b2LoadWorldId(world));
	int total = kind == JB2_EVENTS_BEGIN ? ev.beginCount : kind == JB2_EVENTS_END ? ev.endCount : 0;
	int n = total < cap ? total : cap;
	for (int i = 0; i < n; ++i)
	{
		b2ShapeId sensor = kind == JB2_EVENTS_BEGIN ? ev.beginEvents[i].sensorShapeId
		                                            : ev.endEvents[i].sensorShapeId;
		b2ShapeId visitor = kind == JB2_EVENTS_BEGIN ? ev.beginEvents[i].visitorShapeId
		                                             : ev.endEvents[i].visitorShapeId;
		jb2_event_ids(sensor, visitor, ids_out + i * JB2_EVENT_IDS);
	}
	return total;
}

/* --- joints --------------------------------------------------------------- */

int jb2_joint_is_valid(uint64_t joint)
{
	return joint != 0 && b2Joint_IsValid(b2LoadJointId(joint));
}

int jb2_joint_destroy(uint64_t joint)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2DestroyJoint(b2LoadJointId(joint));
	return JB2_OK;
}

int jb2_joint_wake(uint64_t joint)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2Joint_WakeBodies(b2LoadJointId(joint));
	return JB2_OK;
}

int jb2_joint_set_collide_connected(uint64_t joint, int collide)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2Joint_SetCollideConnected(b2LoadJointId(joint), collide != 0);
	return JB2_OK;
}

int jb2_joint_set_frame(uint64_t joint, int what, double x, double y)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	b2Vec2 v = { (float)x, (float)y };
	switch (what)
	{
		case JB2_FRAME_ANCHOR_A: b2Joint_SetLocalAnchorA(id, v); break;
		case JB2_FRAME_ANCHOR_B: b2Joint_SetLocalAnchorB(id, v); break;
		case JB2_FRAME_AXIS_A:
			if (b2Length(v) < 1e-6f) return JB2_BAD_ID; /* no direction to normalize */
			b2Joint_SetLocalAxisA(id, b2Normalize(v));
			break;
		case JB2_FRAME_REFERENCE: b2Joint_SetReferenceAngle(id, (float)x); break;
		default: return JB2_BAD_ID;
	}
	b2Joint_WakeBodies(id);
	return JB2_OK;
}

int jb2_joint_bodies(uint64_t joint, uint64_t *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	out[0] = b2StoreBodyId(b2Joint_GetBodyA(id));
	out[1] = b2StoreBodyId(b2Joint_GetBodyB(id));
	return JB2_OK;
}

int jb2_joint_read(uint64_t joint, double *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	b2Vec2 f = b2Joint_GetConstraintForce(id);
	b2Vec2 la = b2Joint_GetLocalAnchorA(id);
	b2Vec2 lb = b2Joint_GetLocalAnchorB(id);
	out[0] = (double)b2Joint_GetType(id);
	out[1] = f.x;
	out[2] = f.y;
	out[3] = b2Joint_GetConstraintTorque(id);
	out[4] = b2Joint_GetLinearSeparation(id);
	out[5] = b2Joint_GetAngularSeparation(id);
	out[6] = b2Joint_GetCollideConnected(id) ? 1.0 : 0.0;
	out[7] = la.x;
	out[8] = la.y;
	out[9] = lb.x;
	out[10] = lb.y;
	return JB2_OK;
}

uint64_t jb2_prismatic_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                              const double *p, uint64_t user_data)
{
	if (!jb2_world_is_valid(world) || !jb2_body_is_valid(body_a) || !jb2_body_is_valid(body_b))
		return 0;
	b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
	def.bodyIdA = b2LoadBodyId(body_a);
	def.bodyIdB = b2LoadBodyId(body_b);
	def.localAnchorA = (b2Vec2){ (float)p[0], (float)p[1] };
	def.localAnchorB = (b2Vec2){ (float)p[2], (float)p[3] };
	def.localAxisA = b2Normalize((b2Vec2){ (float)p[4], (float)p[5] });
	def.referenceAngle = (float)p[6];
	def.targetTranslation = (float)p[7];
	def.enableSpring = p[8] != 0.0;
	def.hertz = (float)p[9];
	def.dampingRatio = (float)p[10];
	def.enableLimit = p[11] != 0.0;
	def.lowerTranslation = (float)p[12];
	def.upperTranslation = (float)p[13];
	def.enableMotor = p[14] != 0.0;
	def.maxMotorForce = (float)p[15];
	def.motorSpeed = (float)p[16];
	def.collideConnected = p[17] != 0.0;
	def.userData = (void*)(uintptr_t)user_data;
	return b2StoreJointId(b2CreatePrismaticJoint(b2LoadWorldId(world), &def));
}

int jb2_prismatic_set(uint64_t joint, int what, double v1, double v2)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_prismaticJoint) return JB2_BAD_ID;
	switch (what)
	{
		case JB2_SET_SPRING_ON: b2PrismaticJoint_EnableSpring(id, v1 != 0.0); break;
		case JB2_SET_SPRING_HERTZ: b2PrismaticJoint_SetSpringHertz(id, (float)v1); break;
		case JB2_SET_SPRING_DAMPING: b2PrismaticJoint_SetSpringDampingRatio(id, (float)v1); break;
		case JB2_SET_TARGET: b2PrismaticJoint_SetTargetTranslation(id, (float)v1); break;
		case JB2_SET_LIMIT_ON: b2PrismaticJoint_EnableLimit(id, v1 != 0.0); break;
		case JB2_SET_LIMITS: b2PrismaticJoint_SetLimits(id, (float)v1, (float)v2); break;
		case JB2_SET_MOTOR_ON: b2PrismaticJoint_EnableMotor(id, v1 != 0.0); break;
		case JB2_SET_MOTOR_SPEED: b2PrismaticJoint_SetMotorSpeed(id, (float)v1); break;
		case JB2_SET_MAX_MOTOR: b2PrismaticJoint_SetMaxMotorForce(id, (float)v1); break;
		default: return JB2_BAD_ID;
	}
	b2Joint_WakeBodies(id); /* a sleeping body wouldn't notice the change */
	return JB2_OK;
}

int jb2_prismatic_read(uint64_t joint, double *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_prismaticJoint) return JB2_BAD_ID;
	b2Vec2 axis = b2Joint_GetLocalAxisA(id);
	out[0] = b2PrismaticJoint_GetTranslation(id);
	out[1] = b2PrismaticJoint_GetSpeed(id);
	out[2] = b2PrismaticJoint_GetMotorForce(id);
	out[3] = b2PrismaticJoint_IsSpringEnabled(id) ? 1.0 : 0.0;
	out[4] = b2PrismaticJoint_GetSpringHertz(id);
	out[5] = b2PrismaticJoint_GetSpringDampingRatio(id);
	out[6] = b2PrismaticJoint_GetTargetTranslation(id);
	out[7] = b2PrismaticJoint_IsLimitEnabled(id) ? 1.0 : 0.0;
	out[8] = b2PrismaticJoint_GetLowerLimit(id);
	out[9] = b2PrismaticJoint_GetUpperLimit(id);
	out[10] = b2PrismaticJoint_IsMotorEnabled(id) ? 1.0 : 0.0;
	out[11] = b2PrismaticJoint_GetMotorSpeed(id);
	out[12] = b2PrismaticJoint_GetMaxMotorForce(id);
	out[13] = axis.x;
	out[14] = axis.y;
	return JB2_OK;
}

uint64_t jb2_revolute_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                             const double *p, uint64_t user_data)
{
	if (!jb2_world_is_valid(world) || !jb2_body_is_valid(body_a) || !jb2_body_is_valid(body_b))
		return 0;
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	def.bodyIdA = b2LoadBodyId(body_a);
	def.bodyIdB = b2LoadBodyId(body_b);
	def.localAnchorA = (b2Vec2){ (float)p[0], (float)p[1] };
	def.localAnchorB = (b2Vec2){ (float)p[2], (float)p[3] };
	def.referenceAngle = (float)p[4];
	def.targetAngle = (float)p[5];
	def.enableSpring = p[6] != 0.0;
	def.hertz = (float)p[7];
	def.dampingRatio = (float)p[8];
	def.enableLimit = p[9] != 0.0;
	def.lowerAngle = (float)p[10];
	def.upperAngle = (float)p[11];
	def.enableMotor = p[12] != 0.0;
	def.maxMotorTorque = (float)p[13];
	def.motorSpeed = (float)p[14];
	def.collideConnected = p[15] != 0.0;
	def.userData = (void*)(uintptr_t)user_data;
	return b2StoreJointId(b2CreateRevoluteJoint(b2LoadWorldId(world), &def));
}

int jb2_revolute_set(uint64_t joint, int what, double v1, double v2)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_revoluteJoint) return JB2_BAD_ID;
	switch (what)
	{
		case JB2_SET_SPRING_ON: b2RevoluteJoint_EnableSpring(id, v1 != 0.0); break;
		case JB2_SET_SPRING_HERTZ: b2RevoluteJoint_SetSpringHertz(id, (float)v1); break;
		case JB2_SET_SPRING_DAMPING: b2RevoluteJoint_SetSpringDampingRatio(id, (float)v1); break;
		case JB2_SET_TARGET: b2RevoluteJoint_SetTargetAngle(id, (float)v1); break;
		case JB2_SET_LIMIT_ON: b2RevoluteJoint_EnableLimit(id, v1 != 0.0); break;
		case JB2_SET_LIMITS: b2RevoluteJoint_SetLimits(id, (float)v1, (float)v2); break;
		case JB2_SET_MOTOR_ON: b2RevoluteJoint_EnableMotor(id, v1 != 0.0); break;
		case JB2_SET_MOTOR_SPEED: b2RevoluteJoint_SetMotorSpeed(id, (float)v1); break;
		case JB2_SET_MAX_MOTOR: b2RevoluteJoint_SetMaxMotorTorque(id, (float)v1); break;
		default: return JB2_BAD_ID;
	}
	b2Joint_WakeBodies(id);
	return JB2_OK;
}

int jb2_revolute_read(uint64_t joint, double *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_revoluteJoint) return JB2_BAD_ID;
	out[0] = b2RevoluteJoint_GetAngle(id);
	out[1] = b2RevoluteJoint_GetMotorTorque(id);
	out[2] = b2RevoluteJoint_IsSpringEnabled(id) ? 1.0 : 0.0;
	out[3] = b2RevoluteJoint_GetSpringHertz(id);
	out[4] = b2RevoluteJoint_GetSpringDampingRatio(id);
	out[5] = b2RevoluteJoint_GetTargetAngle(id);
	out[6] = b2RevoluteJoint_IsLimitEnabled(id) ? 1.0 : 0.0;
	out[7] = b2RevoluteJoint_GetLowerLimit(id);
	out[8] = b2RevoluteJoint_GetUpperLimit(id);
	out[9] = b2RevoluteJoint_IsMotorEnabled(id) ? 1.0 : 0.0;
	out[10] = b2RevoluteJoint_GetMotorSpeed(id);
	out[11] = b2RevoluteJoint_GetMaxMotorTorque(id);
	return JB2_OK;
}

uint64_t jb2_distance_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                             const double *p, uint64_t user_data)
{
	if (!jb2_world_is_valid(world) || !jb2_body_is_valid(body_a) || !jb2_body_is_valid(body_b))
		return 0;
	b2DistanceJointDef def = b2DefaultDistanceJointDef();
	def.bodyIdA = b2LoadBodyId(body_a);
	def.bodyIdB = b2LoadBodyId(body_b);
	def.localAnchorA = (b2Vec2){ (float)p[0], (float)p[1] };
	def.localAnchorB = (b2Vec2){ (float)p[2], (float)p[3] };
	def.length = (float)p[4];
	def.enableSpring = p[5] != 0.0;
	def.hertz = (float)p[6];
	def.dampingRatio = (float)p[7];
	def.enableLimit = p[8] != 0.0;
	def.minLength = (float)p[9];
	def.maxLength = (float)p[10];
	def.enableMotor = p[11] != 0.0;
	def.maxMotorForce = (float)p[12];
	def.motorSpeed = (float)p[13];
	def.collideConnected = p[14] != 0.0;
	def.userData = (void*)(uintptr_t)user_data;
	return b2StoreJointId(b2CreateDistanceJoint(b2LoadWorldId(world), &def));
}

int jb2_distance_set(uint64_t joint, int what, double v1, double v2)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_distanceJoint) return JB2_BAD_ID;
	switch (what)
	{
		case JB2_SET_SPRING_ON: b2DistanceJoint_EnableSpring(id, v1 != 0.0); break;
		case JB2_SET_SPRING_HERTZ: b2DistanceJoint_SetSpringHertz(id, (float)v1); break;
		case JB2_SET_SPRING_DAMPING: b2DistanceJoint_SetSpringDampingRatio(id, (float)v1); break;
		case JB2_SET_LENGTH: b2DistanceJoint_SetLength(id, (float)v1); break;
		case JB2_SET_LIMIT_ON: b2DistanceJoint_EnableLimit(id, v1 != 0.0); break;
		case JB2_SET_LIMITS: b2DistanceJoint_SetLengthRange(id, (float)v1, (float)v2); break;
		case JB2_SET_MOTOR_ON: b2DistanceJoint_EnableMotor(id, v1 != 0.0); break;
		case JB2_SET_MOTOR_SPEED: b2DistanceJoint_SetMotorSpeed(id, (float)v1); break;
		case JB2_SET_MAX_MOTOR: b2DistanceJoint_SetMaxMotorForce(id, (float)v1); break;
		default: return JB2_BAD_ID;
	}
	b2Joint_WakeBodies(id);
	return JB2_OK;
}

int jb2_distance_read(uint64_t joint, double *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_distanceJoint) return JB2_BAD_ID;
	out[0] = b2DistanceJoint_GetCurrentLength(id);
	out[1] = b2DistanceJoint_GetMotorForce(id);
	out[2] = b2DistanceJoint_GetLength(id);
	out[3] = b2DistanceJoint_IsSpringEnabled(id) ? 1.0 : 0.0;
	out[4] = b2DistanceJoint_GetSpringHertz(id);
	out[5] = b2DistanceJoint_GetSpringDampingRatio(id);
	out[6] = b2DistanceJoint_IsLimitEnabled(id) ? 1.0 : 0.0;
	out[7] = b2DistanceJoint_GetMinLength(id);
	out[8] = b2DistanceJoint_GetMaxLength(id);
	out[9] = b2DistanceJoint_IsMotorEnabled(id) ? 1.0 : 0.0;
	out[10] = b2DistanceJoint_GetMotorSpeed(id);
	out[11] = b2DistanceJoint_GetMaxMotorForce(id);
	return JB2_OK;
}

uint64_t jb2_weld_create(uint32_t world, uint64_t body_a, uint64_t body_b,
                         const double *p, uint64_t user_data)
{
	if (!jb2_world_is_valid(world) || !jb2_body_is_valid(body_a) || !jb2_body_is_valid(body_b))
		return 0;
	b2WeldJointDef def = b2DefaultWeldJointDef();
	def.bodyIdA = b2LoadBodyId(body_a);
	def.bodyIdB = b2LoadBodyId(body_b);
	def.localAnchorA = (b2Vec2){ (float)p[0], (float)p[1] };
	def.localAnchorB = (b2Vec2){ (float)p[2], (float)p[3] };
	def.referenceAngle = (float)p[4];
	def.linearHertz = (float)p[5];
	def.angularHertz = (float)p[6];
	def.linearDampingRatio = (float)p[7];
	def.angularDampingRatio = (float)p[8];
	def.collideConnected = p[9] != 0.0;
	def.userData = (void*)(uintptr_t)user_data;
	return b2StoreJointId(b2CreateWeldJoint(b2LoadWorldId(world), &def));
}

int jb2_weld_set(uint64_t joint, int what, double v1, double v2)
{
	(void)v2;
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_weldJoint) return JB2_BAD_ID;
	switch (what)
	{
		case JB2_SET_LINEAR_HERTZ: b2WeldJoint_SetLinearHertz(id, (float)v1); break;
		case JB2_SET_ANGULAR_HERTZ: b2WeldJoint_SetAngularHertz(id, (float)v1); break;
		case JB2_SET_LINEAR_DAMPING: b2WeldJoint_SetLinearDampingRatio(id, (float)v1); break;
		case JB2_SET_ANGULAR_DAMPING: b2WeldJoint_SetAngularDampingRatio(id, (float)v1); break;
		default: return JB2_BAD_ID;
	}
	b2Joint_WakeBodies(id);
	return JB2_OK;
}

int jb2_weld_read(uint64_t joint, double *out)
{
	if (!jb2_joint_is_valid(joint)) return JB2_BAD_ID;
	b2JointId id = b2LoadJointId(joint);
	if (b2Joint_GetType(id) != b2_weldJoint) return JB2_BAD_ID;
	out[0] = b2WeldJoint_GetLinearHertz(id);
	out[1] = b2WeldJoint_GetAngularHertz(id);
	out[2] = b2WeldJoint_GetLinearDampingRatio(id);
	out[3] = b2WeldJoint_GetAngularDampingRatio(id);
	return JB2_OK;
}
