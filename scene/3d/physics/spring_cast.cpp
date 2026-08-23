
#include "spring_cast.h"

#include "Jolt/Physics/Body/Body.h"

#include "core/config/engine.h"
#include "scene/3d/physics/physics_body_3d.h"
#include "scene/main/scene_tree.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/objects/jolt_body_3d.h"
#include "modules/jolt_physics/objects/jolt_object_3d.h"
#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "modules/jolt_physics/spaces/jolt_query_filter_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

SpringCast::SpringCast() {
	settings = new SpringCastSettings();
}

SpringCast::~SpringCast() {
	_destroy_constraint();
}

void SpringCast::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint()) {
				_update_debug_shape_vertices();
			}

			if (get_tree()->is_debugging_collisions_hint()) {
				_update_debug_shape();
				if (!Engine::get_singleton()->is_editor_hint()) {
					set_physics_process_internal(true);
				}
			}
		} break;

		case NOTIFICATION_POST_ENTER_TREE: {
			if (spring_cast == nullptr) {
				_build_constraint();
			}

			if (_auto_add_remove) {
				_add_constraint();
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			set_physics_process_internal(false);

			if (debug_instance.is_valid()) {
				_clear_debug_shape();
			}

			if (_auto_add_remove) {
				_remove_constraint();
			}
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			if (is_inside_tree() && debug_instance.is_valid()) {
				RenderingServer::get_singleton()->instance_set_visible(debug_instance, is_visible_in_tree());
			}
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (get_tree()->is_debugging_collisions_hint()) {
				_update_debug_shape_material();
				_update_debug_shape();
				if (is_inside_tree() && debug_instance.is_valid()) {
					RenderingServer::get_singleton()->instance_set_transform(debug_instance, get_global_transform());
				}
			}
		} break;
	}
}

PackedStringArray SpringCast::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();

	if (_get_body() == nullptr) {
		warnings.push_back("SpringCast does not have a main body, check that the path is valid");
	}

	return warnings;
}

JPH::Array<JPH::BodyID> SpringCast::_pick_collisions(const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringLength, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) const {
	JPH::Array<JPH::BodyID> results;
	if (unlikely(!pick_collisions_user.is_valid())) {
		// NOTE: generally, this callback should not be used when the user function is invalid, but
		// to be safe, the default is to just accept everything. However, the dev should ensure the
		// function is either valid or null'd, because this is just a waste of time

		WARN_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback was invalid, default is to allow all bodies (called from "%s")", get_description()));

		results.reserve(inBodyList.size());
		for (JPH::Body *body : inBodyList) {
			results.push_back(body->GetID());
		}
	} else {
		// TODO: convert types, call user function, convert result RIDs
	}

	return results;
}

PhysicsBody3D *SpringCast::_get_body() const {
	if (!body_rid.is_valid()) {
		return nullptr;
	}

	ObjectID object_id = PhysicsServer3D::get_singleton()->body_get_object_instance_id(body_rid);
	PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(ObjectDB::get_instance(object_id));
	return body;
}

JoltPhysicsDirectSpaceState3D *SpringCast::_get_direct_space() const {
	PhysicsBody3D *phys_body = _get_body();
	if (phys_body == nullptr) {
		return nullptr;
	}

	return static_cast<JoltPhysicsDirectSpaceState3D *>(phys_body->get_world_3d()->get_direct_space_state());
}

void SpringCast::_build_constraint() {
	_destroy_constraint();

	JoltPhysicsDirectSpaceState3D *jolt_direct_space = _get_direct_space();
	if (jolt_direct_space == nullptr) {
		return;
	}

	if (physics_system == nullptr) {
		physics_system = &jolt_direct_space->get_space().get_physics_system();
	}

	JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
	JoltBody3D *jolt_body = jolt_server->get_body(body_rid);

	if (jolt_body == nullptr) {
		return;
	}

	JPH::Body *body = jolt_body->get_jolt_body();
	if (body == nullptr) {
		return;
	}

	spring_cast = new JPH::SpringCastConstraint(*body, settings->settings);

	if (collision_tester == nullptr) {
		collision_tester = new JPH::SpringCastCollisionTesterCastSphere();
		collision_filter = new JoltQueryFilter3D(*jolt_direct_space, collision_mask, true, false, exclude);
		collision_tester->SetBroadPhaseLayerFilter(*collision_filter);
		collision_tester->SetObjectLayerFilter(*collision_filter);
		collision_tester->SetBodyFilter(*collision_filter);
	}

	spring_cast->SetCollisionTester(collision_tester);
	if (pick_collisions_user.is_valid()) {
		spring_cast->SetPickCollisionsFunction(&SpringCast::_forward_pick_collisions, this);
	}
}

void SpringCast::_destroy_constraint() {
	if (spring_cast != nullptr) {
		_remove_constraint();
		spring_cast = nullptr;
		collision_tester = nullptr;
		collision_filter = nullptr;
	}
}

void SpringCast::_add_constraint() {
	if (_is_constraint_added || spring_cast == nullptr) {
		return;
	}

	physics_system->AddConstraint(spring_cast);
	physics_system->AddStepListener(spring_cast);
	_is_constraint_added = true;
}

void SpringCast::_remove_constraint() {
	if (!_is_constraint_added || spring_cast == nullptr) {
		return;
	}

	physics_system->RemoveConstraint(spring_cast);
	physics_system->RemoveStepListener(spring_cast);
	_is_constraint_added = false;
}

bool SpringCast::is_enabled() const {
	return (spring_cast != nullptr) && (spring_cast->IsActive());
}

bool SpringCast::set_enabled(bool p_enabled) {
	if (spring_cast == nullptr) {
		return false;
	}

	spring_cast->SetEnabled(p_enabled);
	return true;
}

bool SpringCast::add_constraint() {
	if (_is_constraint_added) {
		return false;
	}

	_add_constraint();
	return _is_constraint_added;
}

bool SpringCast::remove_constraint() {
	if (!_is_constraint_added) {
		return false;
	}

	_remove_constraint();
	return !_is_constraint_added;
}

void SpringCast::set_main_body(const NodePath &p_body) {
	if (p_body.is_empty()) {
		// NOTE: If the path is empty, the intention is to destroy the constraint
		_destroy_constraint();
		body_rid = RID();
		main_body = NodePath();
		return;
	}

	const Node *node = get_node_or_null(p_body);
	if (unlikely(node == nullptr)) {
		const String desc = get_description();
		if (p_body.is_absolute()) {
			ERR_FAIL_MSG(vformat(R"(Cannot find node for SpringCast: "%s" (absolute path attempted from "%s").)", p_body, desc));
		}
		ERR_FAIL_MSG(vformat(R"(Cannot find node for SpringCast: "%s" (relative to "%s").)", p_body, desc));
	}

	const PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(node);
	if (unlikely(body == nullptr)) {
		const String desc = get_description();
		if (p_body.is_absolute()) {
			ERR_FAIL_MSG(vformat(R"(SpringCast requires a PhysicsBody3D, found node was not of this type: "%s" (absolute path attempted from "%s").)", p_body, desc));
		}
		ERR_FAIL_MSG(vformat(R"(SpringCast requires a PhysicsBody3D, found node was not of this type: "%s" (relative to "%s").)", p_body, desc));
	}

	// NOTE: Do nothing if the body_rid is already the same as this path, I don't want to have a
	// side-effect of rebuilding the constraint when the body is the same.
	if (body->get_rid() == body_rid) {
		return;
	}

	main_body = p_body;
	body_rid = body->get_rid();

	// Only rebuild if the constraint has already been created, otherwise this should get called later in post tree entered
	if (spring_cast != nullptr) {
		_build_constraint();
	}
}

void SpringCast::set_settings(const Ref<SpringCastSettings> &p_settings) {
	if (!p_settings.is_valid()) {
		settings = new SpringCastSettings();
		notify_property_list_changed();
	} else {
		settings = p_settings;
	}

	if (spring_cast != nullptr) {
		spring_cast->SetSettings(&settings->settings);
	}
}

void SpringCast::set_pick_collisions_function(const Callable &p_pick_function) {
	pick_collisions_user = p_pick_function;

	if (spring_cast == nullptr) {
		return;
	}

	// NOTE: setting a null has the intention of disabling the custom callback
	if (pick_collisions_user.is_null()) {
		spring_cast->SetPickCollisionsFunction(nullptr, nullptr);
	} else {
		spring_cast->SetPickCollisionsFunction(&SpringCast::_forward_pick_collisions, this);
	}
}

void SpringCast::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;

	if (collision_filter != nullptr) {
		collision_filter->collision_mask = collision_mask;
	}
}

bool SpringCast::is_colliding() const {
	return (spring_cast != nullptr) && spring_cast->HasContact();
}

int SpringCast::get_contact_body_count() const {
	if (spring_cast == nullptr) {
		// NOTE: I don't think this needs any sort of error, get_contact_body_count() = 0 may be
		// used as a way to quick-test if the spring exists and has contacts.
		return 0;
	}

	return spring_cast->GetNumContactBody();
}

PhysicsBody3D *SpringCast::get_contact_body(int p_body_index) const {
	if (spring_cast == nullptr) {
		// TODO: warn once about spring cast not existing yet
		return nullptr;
	}

	// TODO: this test kinda sucks, I think SpringCastConstraint should just return an invalid
	// value when attempting to access outside of the contact body range
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		// TODO: print error (this will be changed to an ID validity test later)
		return nullptr;
	}

	const JoltPhysicsDirectSpaceState3D *jolt_direct_space = _get_direct_space();
	if (jolt_direct_space == nullptr) {
		// TODO: print error, this should not happen
		return nullptr;
	}

	const JoltSpace3D &space = jolt_direct_space->get_space();
	const JoltObject3D *object = space.try_get_object(spring_cast->GetContactBodyID(p_body_index));
	if (object == nullptr) {
		// TODO: when there is an ID validity test added, this should error because the body
		// existed but the space failed to return it for some reason.
		return nullptr;
	}

	PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(ObjectDB::get_instance(object->get_instance_id()));
	// TODO: this should error (?) if the body is nullptr, because we expected to find a
	// PhysicsBody3D but it either no longer exists or was some other type.
	return body;
}

RID SpringCast::get_contact_body_rid(int p_body_index) const {
	if (spring_cast == nullptr) {
		// TODO: warn once about spring cast not existing yet
		return RID();
	}

	// TODO: this test kinda sucks, I think SpringCastConstraint should just return an invalid
	// value when attempting to access outside of the contact body range
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		// TODO: print error (this will be changed to an ID validity test later)
		return RID();
	}

	const JoltPhysicsDirectSpaceState3D *jolt_direct_space = _get_direct_space();
	if (jolt_direct_space == nullptr) {
		// TODO: print error, this should not happen
		return RID();
	}

	const JoltSpace3D &space = jolt_direct_space->get_space();
	const JoltObject3D *object = space.try_get_object(spring_cast->GetContactBodyID(p_body_index));
	if (object == nullptr) {
		// TODO: when there is an ID validity test added, this should error because the body
		// existed but the space failed to return it for some reason.
		return RID();
	}

	return object->get_rid();
}

Vector3 SpringCast::get_contact_normal(int p_body_index) const {
	if (spring_cast == nullptr) {
		// TODO: warn once about spring cast not existing yet
		return Vector3();
	}

	// TODO: this test kinda sucks, I think SpringCastConstraint should just return a Vec3 zero
	// value when attempting to access outside of the contact body range
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		// TODO: print error (this will be changed to an ID validity test later)
		return Vector3();
	}

	return to_godot(spring_cast->GetContactNormal(p_body_index));
}

Vector3 SpringCast::get_contact_average_point(int p_body_index) const {
	if (spring_cast == nullptr) {
		// TODO: warn once about spring cast not existing yet
		return Vector3();
	}

	// TODO: this test kinda sucks, I think SpringCastConstraint should just return a Vec3 zero
	// value when attempting to access outside of the contact body range
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		return Vector3();
	}

	return to_godot(spring_cast->GetContactPointAverage(p_body_index));
}

int SpringCast::get_contact_shape_count(int p_body_index) const {
	if (spring_cast == nullptr) {
		// TODO: print error, this should only be called when the user knows contact bodies exist
		return 0;
	}

	// TODO: SpringCastConstraint should just return -1 when outside the range
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		return 0; // TODO: if it is invalid, does this return -1? No, return 0 and print an error.
	}

	// TODO: for the error above, ensure this is checked for -1, print the error, and return 0 instead
	return spring_cast->GetNumSubShape(p_body_index);
}

int SpringCast::get_contact_shape(int p_body_index, int p_shape_index) const {
	// NOTE: for this one, return -1 when invalid because that is the typical "bad index" value
	if (spring_cast == nullptr) {
		// TODO: print error, this should only be called when the user knows contact bodies exist
		return -1;
	}

	// TODO: SpringCastConstraint should just return an invalid shape id
	if (p_body_index < 0 || p_body_index >= spring_cast->GetNumContactBody()) {
		return -1;
	}

	const JPH::SubShapeID shape = spring_cast->GetContactSubShapeID(p_body_index, p_shape_index);
	if (shape.IsEmpty()) {
		// TODO: print error here, this means the indexes were invalid
		return -1;
	}

	const JoltPhysicsDirectSpaceState3D *jolt_direct_space = _get_direct_space();
	if (jolt_direct_space == nullptr) {
		// TODO: print error, this should not happen
		return -1;
	}

	const JoltSpace3D &space = jolt_direct_space->get_space();
	const JoltShapedObject3D *object = space.try_get_shaped(spring_cast->GetContactBodyID(p_body_index));
	if (object == nullptr) {
		// TODO: when there is an ID validity test added, this should error because the body
		// existed but the space failed to return it for some reason.
		return -1;
	}

	// TODO: check if this actually returns -1 when the shape doesn't exist on the body
	// and print an error if the index is -1.
	return object->find_shape_index(shape);
}

int SpringCast::get_contact_point_count(int p_body_index, int p_shape_index) const {
	// NOTE: body index and shape index can be -1, indicating the user wants the total number of
	// contact points, the number on a specific body, or the number on a specific shape.
}

Vector3 SpringCast::get_contact_point(int p_idx, int p_body_index, int p_shape_index) const {
}

float SpringCast::get_spring_length() const {
}

bool SpringCast::has_hit_hard_point() const {
}

float SpringCast::get_spring_lamdba() const {
}

float SpringCast::get_friction_lambda() const {
}

float SpringCast::get_angular_lambda() const {
}

void SpringCast::add_exception_rid(const RID &p_rid) {
}

void SpringCast::add_exception(RequiredParam<const PhysicsBody3D> rp_node) {
}

void SpringCast::remove_exception_rid(const RID &p_rid) {
}

void SpringCast::remove_exception(RequiredParam<const PhysicsBody3D> rp_node) {
}

void SpringCast::clear_exceptions() {
}
