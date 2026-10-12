// SPDX-License-Identifier: MIT
// Flat scalar/pointer shim over Box2D v3.1 for jolt.ffi. See jb2_shim.h.
#include "jb2_shim.h"

#include <box2d/box2d.h>

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
	def.rotation = b2MakeRot((float)angle);
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
	out[2] = b2Rot_GetAngle(b2Body_GetRotation(id));
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
	                                (b2Vec2){ (float)ox, (float)oy }, b2MakeRot((float)angle));
	return b2StoreShapeId(b2CreatePolygonShape(b2LoadBodyId(body), &sd, &box));
}
