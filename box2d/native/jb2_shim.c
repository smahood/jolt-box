// SPDX-License-Identifier: MIT
// Flat scalar/pointer shim over Box2D v3.1 for jolt.ffi. See jb2_shim.h.
#include "jb2_shim.h"

#include <box2d/box2d.h>
#include <math.h>

/* b2MakeRot uses a fast approximation that is off by up to ~1e-3 rad, so an
 * angle set from the REPL wouldn't read back as itself. Exact is cheap here. */
static b2Rot jb2_rot(double angle)
{
	return (b2Rot){ cosf((float)angle), sinf((float)angle) };
}

/* --- worlds --------------------------------------------------------------- */

uint32_t jb2_world_create(double gx, double gy)
{
	b2WorldDef def = b2DefaultWorldDef();
	def.gravity = (b2Vec2){ (float)gx, (float)gy };
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

uint64_t jb2_body_add_box(uint64_t body, double half_w, double half_h, double ox, double oy,
                          double angle, double density, double friction, double restitution)
{
	if (!jb2_body_is_valid(body)) return 0;
	if (!(half_w > 0.0) || !(half_h > 0.0)) return 0;
	b2ShapeDef sd = b2DefaultShapeDef();
	sd.density = (float)density;
	sd.material.friction = (float)friction;
	sd.material.restitution = (float)restitution;
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
