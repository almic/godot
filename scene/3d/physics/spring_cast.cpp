
#include "spring_cast.h"

#include "Jolt/Physics/Body/Body.h"

#include "core/config/engine.h"
#include "core/math/math_defs.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/variant/variant_utility.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/physics/physics_body_3d.h"
#include "scene/main/scene_tree.h"
#include "scene/resources/3d/convex_polygon_shape_3d.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"

#include "modules/jolt_physics/jolt_globals.h"
#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/objects/jolt_body_3d.h"
#include "modules/jolt_physics/objects/jolt_object_3d.h"
#include "modules/jolt_physics/spaces/jolt_query_filter_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

SpringCast::SpringCast() {
}

SpringCast::~SpringCast() {
	_destroy_constraint();
}

void SpringCast::_bind_methods() {
	ClassDB::bind_method(D_METHOD("is_enabled"), &SpringCast::is_enabled);
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &SpringCast::set_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");

	ClassDB::bind_method(D_METHOD("is_auto_add_enabled"), &SpringCast::is_auto_add_enabled);
	ClassDB::bind_method(D_METHOD("set_auto_add_enabled", "enabled"), &SpringCast::set_auto_add_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_add_enabled"), "set_auto_add_enabled", "is_auto_add_enabled");

	ClassDB::bind_method(D_METHOD("is_constraint_added"), &SpringCast::is_constraint_added);
	ClassDB::bind_method(D_METHOD("add_constraint"), &SpringCast::add_constraint);
	ClassDB::bind_method(D_METHOD("remove_constraint"), &SpringCast::remove_constraint);

	ClassDB::bind_method(D_METHOD("get_main_body"), &SpringCast::get_main_body);
	ClassDB::bind_method(D_METHOD("set_main_body", "body_path"), &SpringCast::set_main_body);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "main_body", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysicsBody3D"), "set_main_body", "get_main_body");

	ClassDB::bind_method(D_METHOD("get_main_body_object"), &SpringCast::get_main_body_object);

	ClassDB::bind_method(D_METHOD("get_forward_input"), &SpringCast::get_forward_input);
	ClassDB::bind_method(D_METHOD("set_forward_input", "forward_input"), &SpringCast::set_forward_input);

	ClassDB::bind_method(D_METHOD("get_forward_max_speed"), &SpringCast::get_forward_max_speed);
	ClassDB::bind_method(D_METHOD("set_forward_max_speed", "forward_max_speed"), &SpringCast::set_forward_max_speed);

	ClassDB::bind_method(D_METHOD("get_rest_offset"), &SpringCast::get_rest_offset);
	ClassDB::bind_method(D_METHOD("set_rest_offset", "rest_offset"), &SpringCast::set_rest_offset);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rest_offset", PROPERTY_HINT_RANGE, "-1,1,0.0001,or_less,or_greater,suffix:m"), "set_rest_offset", "get_rest_offset");

	ClassDB::bind_method(D_METHOD("get_settings"), &SpringCast::get_settings);
	ClassDB::bind_method(D_METHOD("set_settings", "settings"), &SpringCast::set_settings);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "settings", PROPERTY_HINT_RESOURCE_TYPE, SpringCastSettings::get_class_static()), "set_settings", "get_settings");

	ClassDB::bind_method(D_METHOD("get_pick_collisions_function"), &SpringCast::get_pick_collisions_function);
	ClassDB::bind_method(D_METHOD("set_pick_collisions_function", "pick_function"), &SpringCast::set_pick_collisions_function);
	ADD_PROPERTY(PropertyInfo(Variant::CALLABLE, "pick_collisions_function", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_pick_collisions_function", "get_pick_collisions_function");

	ClassDB::bind_method(D_METHOD("get_collision_mask"), &SpringCast::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &SpringCast::set_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ClassDB::bind_method(D_METHOD("get_collision_mask_value", "layer"), &SpringCast::get_collision_mask_value);
	ClassDB::bind_method(D_METHOD("set_collision_mask_value", "layer", "value"), &SpringCast::set_collision_mask_value);

	ClassDB::bind_method(D_METHOD("is_colliding"), &SpringCast::is_colliding);

	ClassDB::bind_method(D_METHOD("get_contact_body_count"), &SpringCast::get_contact_body_count);
	ClassDB::bind_method(D_METHOD("get_contact_body", "body_index"), &SpringCast::get_contact_body);
	ClassDB::bind_method(D_METHOD("get_contact_body_rid", "body_index"), &SpringCast::get_contact_body_rid);
	ClassDB::bind_method(D_METHOD("get_contact_normal", "body_index"), &SpringCast::get_contact_normal);
	ClassDB::bind_method(D_METHOD("get_contact_average_point", "body_index"), &SpringCast::get_contact_average_point);

	ClassDB::bind_method(D_METHOD("get_contact_shape_count", "body_index"), &SpringCast::get_contact_shape_count);
	ClassDB::bind_method(D_METHOD("get_contact_shape", "body_index", "shape_index"), &SpringCast::get_contact_shape);

	ClassDB::bind_method(D_METHOD("get_contact_point_count", "body_index", "shape_index"), &SpringCast::get_contact_point_count, DEFVAL(-1), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("get_contact_point", "point_index", "body_index", "shape_index"), &SpringCast::get_contact_point, DEFVAL(-1), DEFVAL(-1));

	ClassDB::bind_method(D_METHOD("get_contact_friction", "body_index"), &SpringCast::get_contact_friction);

	ClassDB::bind_method(D_METHOD("get_spring_length"), &SpringCast::get_spring_length);
	ClassDB::bind_method(D_METHOD("has_hit_hard_point"), &SpringCast::has_hit_hard_point);

	ClassDB::bind_method(D_METHOD("get_spring_lambda"), &SpringCast::get_spring_lambda);
	ClassDB::bind_method(D_METHOD("get_friction_lambda"), &SpringCast::get_friction_lambda);
	ClassDB::bind_method(D_METHOD("get_angular_lambda"), &SpringCast::get_angular_lambda);

	ClassDB::bind_method(D_METHOD("add_exception_rid", "rid"), &SpringCast::add_exception_rid);
	ClassDB::bind_method(D_METHOD("add_exception", "physics_body"), &SpringCast::add_exception);
	ClassDB::bind_method(D_METHOD("remove_exception_rid", "rid"), &SpringCast::remove_exception_rid);
	ClassDB::bind_method(D_METHOD("remove_exception", "physics_body"), &SpringCast::remove_exception);
	ClassDB::bind_method(D_METHOD("clear_exceptions"), &SpringCast::clear_exceptions);

	ADD_GROUP("Debug", "debug");
	ClassDB::bind_method(D_METHOD("get_debug_custom_color"), &SpringCast::get_debug_custom_color);
	ClassDB::bind_method(D_METHOD("set_debug_custom_color", "color"), &SpringCast::set_debug_custom_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "debug_custom_color"), "set_debug_custom_color", "get_debug_custom_color");
}

void SpringCastSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_position"), &SpringCastSettings::get_position);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &SpringCastSettings::set_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");

	ClassDB::bind_method(D_METHOD("is_enable_force_point"), &SpringCastSettings::is_enable_force_point);
	ClassDB::bind_method(D_METHOD("set_enable_force_point", "enable"), &SpringCastSettings::set_enable_force_point);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enable_force_point"), "set_enable_force_point", "is_enable_force_point");

	ClassDB::bind_method(D_METHOD("get_force_point"), &SpringCastSettings::get_force_point);
	ClassDB::bind_method(D_METHOD("set_force_point", "force_point"), &SpringCastSettings::set_force_point);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "force_point"), "set_force_point", "get_force_point");

	ClassDB::bind_method(D_METHOD("get_direction"), &SpringCastSettings::get_direction);
	ClassDB::bind_method(D_METHOD("set_direction", "direction"), &SpringCastSettings::set_direction);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "direction"), "set_direction", "get_direction");

	ClassDB::bind_method(D_METHOD("get_radius"), &SpringCastSettings::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &SpringCastSettings::set_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater,suffix:m"), "set_radius", "get_radius");

	ClassDB::bind_method(D_METHOD("get_max_angle"), &SpringCastSettings::get_max_angle);
	ClassDB::bind_method(D_METHOD("set_max_angle", "max_angle"), &SpringCastSettings::set_max_angle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_angle", PROPERTY_HINT_RANGE, "0,90,0.01,radians_as_degrees"), "set_max_angle", "get_max_angle");

	ClassDB::bind_method(D_METHOD("get_min_length"), &SpringCastSettings::get_min_length);
	ClassDB::bind_method(D_METHOD("set_min_length", "min_length"), &SpringCastSettings::set_min_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_length", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater,suffix:m"), "set_min_length", "get_min_length");

	ClassDB::bind_method(D_METHOD("get_max_length"), &SpringCastSettings::get_max_length);
	ClassDB::bind_method(D_METHOD("set_max_length", "max_length"), &SpringCastSettings::set_max_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_length", PROPERTY_HINT_RANGE, "0,1,0.0001,or_greater,suffix:m"), "set_max_length", "get_max_length");

	ClassDB::bind_method(D_METHOD("get_max_bodies"), &SpringCastSettings::get_max_bodies);
	ClassDB::bind_method(D_METHOD("set_max_bodies", "max_bodies"), &SpringCastSettings::set_max_bodies);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_bodies", PROPERTY_HINT_RANGE, "1,8,1,or_greater"), "set_max_bodies", "get_max_bodies");

	ClassDB::bind_method(D_METHOD("get_spring_mode"), &SpringCastSettings::get_spring_mode);
	ClassDB::bind_method(D_METHOD("set_spring_mode", "mode"), &SpringCastSettings::set_spring_mode);
	BIND_ENUM_CONSTANT(SPRING_MODE_FREQUENCY);
	BIND_ENUM_CONSTANT(SPRING_MODE_STIFFNESS);
	BIND_ENUM_CONSTANT(SPRING_MODE_MASS_NORMALIZED);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "spring_mode", PROPERTY_HINT_ENUM, "Frequency and Damping Ratio,Stiffness and Damping,Mass Normalized Stiffness and Damping"), "set_spring_mode", "get_spring_mode");

	ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &SpringCastSettings::get_spring_stiffness);
	ClassDB::bind_method(D_METHOD("set_spring_stiffness", "stiffness"), &SpringCastSettings::set_spring_stiffness);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness"), "set_spring_stiffness", "get_spring_stiffness");

	ClassDB::bind_method(D_METHOD("get_spring_damping"), &SpringCastSettings::get_spring_damping);
	ClassDB::bind_method(D_METHOD("set_spring_damping", "damping"), &SpringCastSettings::set_spring_damping);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping"), "set_spring_damping", "get_spring_damping");
}

void SpringCastSettings::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == "stiffness") {
		p_property.hint = PROPERTY_HINT_RANGE;
		switch (get_spring_mode()) {
			case SPRING_MODE_STIFFNESS: {
				p_property.hint_string = U"0,1,0.001,or_greater,hide_control,suffix:kg\u22C5m/s\u00B2 (N)";
			} break;
			case SPRING_MODE_MASS_NORMALIZED: {
				p_property.hint_string = U"0,1,0.001,or_greater,hide_control,suffix:m/s\u00B2";
			} break;
			case SPRING_MODE_FREQUENCY: {
				p_property.hint_string = "0,1,0.001,or_greater,hide_control,suffix:Hz";
			} break;
		}
	} else if (p_property.name == "damping") {
		p_property.hint = PROPERTY_HINT_RANGE;
		switch (get_spring_mode()) {
			case SPRING_MODE_STIFFNESS: {
				p_property.hint_string = U"0,1,0.001,or_greater,hide_control,suffix:kg\u22C5m/s\u00B2 (N)";
			} break;
			case SPRING_MODE_MASS_NORMALIZED: {
				p_property.hint_string = U"0,1,0.001,or_greater,hide_control,suffix:m/s\u00B2";
			} break;
			case SPRING_MODE_FREQUENCY: {
				p_property.hint_string = "0,1,0.001,or_greater,prefer_slider";
			} break;
		}
	}
}

void SpringCast::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			_settings_changed(); // Used to ensure the node is positioned from settings on load

			if (get_tree()->is_debugging_collisions_hint() && !Engine::get_singleton()->is_editor_hint()) {
				set_physics_process_internal(true);
			}
		} break;

		case NOTIFICATION_POST_ENTER_TREE: {
			// NOTE: I think this is better to avoid early transform sets messing with settings,
			// though I have no proof that transform notifications happen when entering the tree...
			if (Engine::get_singleton()->is_editor_hint()) {
				set_notify_local_transform(true);
			}

			if (spring_cast == nullptr) {
				_build_constraint();
			}

			if (_auto_add_remove) {
				_add_constraint();
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			set_notify_local_transform(false);
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

		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			if (_transform_settings_locked || !settings.is_valid()) {
				return;
			}

			PhysicsBody3D *body = _get_body();
			if (body == nullptr) {
				return;
			}

			_transform_settings_locked = true;
			Transform3D xform = body->get_global_transform().affine_inverse() * get_global_transform();
			settings->set_position(xform.origin);
			_transform_settings_locked = false;
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

JPH::Array<JPH::BodyID> SpringCast::_pick_collisions(const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringFraction, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) const {
	JPH::Array<JPH::BodyID> results;
	if (unlikely(!pick_collisions_user.is_valid())) {
		// NOTE: generally, this callback should not be used when the user function is invalid, but
		// to be safe, the default is to just accept everything. However, the dev should ensure the
		// function is either valid or null'd, because this is just a waste of time

		WARN_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback was invalid, default is to allow all bodies (called from "%s").)", get_description()));

		results.reserve(inBodyList.size());
		for (JPH::Body *body : inBodyList) {
			results.push_back(body->GetID());
		}
	} else {
		const JoltSpace3D *space = _get_space();
		if (space == nullptr) {
			// TODO: print an error
			return results;
		}

		TypedArray<RID> rid_list;
		for (const JPH::Body *body : inBodyList) {
			const JoltObject3D *object = space->try_get_object(body->GetID());
			if (object == nullptr) {
				// TODO: print error
				continue;
			}
			rid_list.push_back(object->get_rid());
		}

		PackedFloat32Array fraction_list;
		fraction_list.reserve_exact(inSpringFraction.size());
		for (float fraction : inSpringFraction) {
			fraction_list.push_back(fraction);
		}

		PackedVector3Array contact_list;
		contact_list.reserve_exact(inAverageContactPoint.size());
		for (const JPH::RVec3 &vec : inAverageContactPoint) {
			contact_list.push_back(to_godot(vec));
		}

		PackedVector3Array normal_list;
		normal_list.reserve_exact(inContactNormal.size());
		for (const JPH::Vec3 &vec : inContactNormal) {
			normal_list.push_back(to_godot(vec));
		}

		const Variant rid_list_var = rid_list;
		const Variant fraction_list_var = fraction_list;
		const Variant contact_list_var = contact_list;
		const Variant normal_list_var = normal_list;
		const Variant *args[4] = { &rid_list_var, &fraction_list_var, &contact_list_var, &normal_list_var };
		Variant result;
		Callable::CallError error;
		pick_collisions_user.callp(args, std::size(args), result, error);
		if (unlikely(error.error != Callable::CallError::CALL_OK)) {
			ERR_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback failed for "%s". It returned the following error: )", get_description(), Variant::get_callable_error_text(pick_collisions_user, args, std::size(args), error)));
			return results;
		}

		if (!result.is_array()) {
			ERR_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback did not return an array, an array result is required (called from "%s").)", get_description()));
			return results;
		}

		const Vector<RID> rid_out_list = result;
		if (rid_out_list.size() != ((Array)result).size()) {
			ERR_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback returned an array that contained non-RID elements, the result must only have RIDs (called from "%s").)", get_description()));
			// Use what was returned anyway
		}

		const JoltPhysicsServer3D &server = *JoltPhysicsServer3D::get_singleton();
		for (const RID &rid : rid_out_list) {
			if (!rid_list.has(rid)) {
				ERR_PRINT_ONCE_ED(vformat(R"(SpringCast pick_collisions_function callback returned an array that contained RIDs not present in the input list, ensure you only return from the set of given RIDs (called from "%s").)", get_description()));
				continue;
			}

			JoltBody3D *body = server.get_body(rid);
			if (body == nullptr) {
				ERR_PRINT_ED(vformat(R"(SpringCast failed to match RID %s to a physics body from the pick_collisions_function callback (called from "%s").)", rid, get_description()));
				continue;
			}

			results.push_back(body->get_jolt_id());
		}
	}

	return results;
}

PhysicsBody3D *SpringCast::_get_body() const {
	if (phys_body != nullptr && VariantUtilityFunctions::is_instance_valid(phys_body)) {
		return phys_body;
	}

	if (body_rid.is_valid()) {
		ObjectID object_id = PhysicsServer3D::get_singleton()->body_get_object_instance_id(body_rid);
		phys_body = Object::cast_to<PhysicsBody3D>(ObjectDB::get_instance(object_id));
		if (phys_body == nullptr) {
			// If RID fails, try a full update
			_update_body();
		}
	} else {
		// No cached body, no body rid, full update
		_update_body();
	}

	return phys_body;
}

// NOTE: This function is labeled const, but since the "truth" is the node path, body_rid and phys_body
// are mutable and we can update it here
void SpringCast::_update_body() const {
	body_rid = RID();
	phys_body = nullptr;

	if (main_body.is_empty() || !is_inside_tree()) {
		return;
	}

	Node *node = get_node_or_null(main_body);
	if (unlikely(node == nullptr)) {
		const String desc = get_description();
		if (main_body.is_absolute()) {
			ERR_FAIL_MSG(vformat(R"(Cannot find node for SpringCast: "%s" (absolute path attempted from "%s").)", main_body, desc));
		}
		ERR_FAIL_MSG(vformat(R"(Cannot find node for SpringCast: "%s" (relative to "%s").)", main_body, desc));
	}

	phys_body = Object::cast_to<PhysicsBody3D>(node);
	if (unlikely(phys_body == nullptr)) {
		const String desc = get_description();
		if (main_body.is_absolute()) {
			ERR_FAIL_MSG(vformat(R"(SpringCast requires a PhysicsBody3D, found node was not of this type: "%s" (absolute path attempted from "%s").)", main_body, desc));
		}
		ERR_FAIL_MSG(vformat(R"(SpringCast requires a PhysicsBody3D, found node was not of this type: "%s" (relative to "%s").)", main_body, desc));
	}

	body_rid = phys_body->get_rid();
}

JoltSpace3D *SpringCast::_get_space() const {
	const PhysicsBody3D *body = _get_body();
	if (body == nullptr) {
		return nullptr;
	}

	const RID space_rid = JoltPhysicsServer3D::get_singleton()->body_get_space(body->get_rid());
	return JoltPhysicsServer3D::get_singleton()->get_space(space_rid);
}

void SpringCast::_build_constraint() {
	_destroy_constraint();

	if (!settings.is_valid()) {
		return;
	}

	const JoltSpace3D *jolt_space = _get_space();
	if (jolt_space == nullptr) {
		// TODO: this should be an error, the space must exist by now
		return;
	}

	if (physics_system == nullptr) {
		physics_system = &jolt_space->get_physics_system();
	}

	const JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
	JoltBody3D *jolt_body = jolt_server->get_body(body_rid);

	if (jolt_body == nullptr) {
		return;
	}

	JPH::Body *body = jolt_body->get_jolt_body();
	if (body == nullptr) {
		return;
	}

	spring_cast = new JPH::SpringCastConstraint(*body, settings->settings);
	spring_cast->SetSpringRestOffset(rest_offset);

	if (collision_tester == nullptr) {
		collision_tester = spring_collision_tester_sphere();
		collision_filter = new JoltQueryFilter3D(*jolt_space, collision_mask, true, false, exclude);
		collision_tester->SetBroadPhaseLayerFilter(collision_filter);
		collision_tester->SetObjectLayerFilter(collision_filter);
		collision_tester->SetBodyFilter(collision_filter);
	}

	spring_cast->SetCollisionTester(collision_tester);
	if (pick_collisions_user.is_valid()) {
		spring_cast->SetPickCollisionsFunction(&SpringCast::_forward_pick_collisions, this);
	}

	// The value of enabled depends on spring_cast existing, so ensure the property list updates to reflect that
	notify_property_list_changed();
}

void SpringCast::_destroy_constraint() {
	_remove_constraint();

	if (spring_cast != nullptr) {
		spring_cast = nullptr;
		collision_tester = nullptr;
	}

	if (collision_filter != nullptr) {
		delete collision_filter;
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

	update_gizmos();

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

	main_body = p_body;
	const RID old_rid = body_rid;
	_update_body();

	// NOTE: Do nothing if the body_rid is already the same as this path, I don't want to have a
	// side-effect of rebuilding the constraint when the body is actually the same.
	if (body_rid == old_rid) {
		return;
	}

	// Only rebuild if the constraint has already been created, otherwise this should get called later in post tree entered
	if (spring_cast != nullptr) {
		_build_constraint();
	}
}

Vector3 SpringCast::get_forward_input() const {
	if (spring_cast == nullptr) {
		return Vector3();
	}

	return to_godot(spring_cast->GetForwardInput());
}

void SpringCast::set_forward_input(const Vector3 &p_forward_input) {
	if (spring_cast == nullptr) {
		return;
	}

	spring_cast->SetForwardInput(to_jolt(p_forward_input));
}

float SpringCast::get_forward_max_speed() const {
	if (spring_cast == nullptr) {
		return 0.0;
	}

	return spring_cast->GetForwardMaxSpeed();
}

void SpringCast::set_forward_max_speed(float p_forward_max_speed) {
	if (spring_cast == nullptr) {
		return;
	}

	spring_cast->SetForwardMaxSpeed(p_forward_max_speed);
}

void SpringCast::set_settings(const Ref<SpringCastSettings> &p_settings) {
	if (p_settings == settings) {
		return;
	}

	if (settings.is_valid()) {
		settings->disconnect_changed(callable_mp(this, &SpringCast::_settings_changed));
	}

	settings = p_settings;

	if (settings.is_valid()) {
		settings->connect_changed(callable_mp(this, &SpringCast::_settings_changed));
		if (spring_cast != nullptr) {
			spring_cast->SetSettings(&settings->settings);
		}
	}

	_settings_changed();
}

void SpringCast::_settings_changed() {
	if (!is_inside_tree()) {
		return;
	}

	const bool is_editor = Engine::get_singleton()->is_editor_hint();
	if (!_transform_settings_locked && is_editor && settings.is_valid()) {
		PhysicsBody3D *body = _get_body();
		if (body != nullptr) {
			_transform_settings_locked = true;
			set_global_transform(body->get_global_transform().translated_local(settings->get_position()));
			_transform_settings_locked = false;
		}
	}

	if (is_editor || get_tree()->is_debugging_collisions_hint()) {
		_update_debug_shape();
		update_gizmos();
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

Object *SpringCast::get_contact_body(int p_body_index) const {
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

	const JoltSpace3D *space = _get_space();
	const JoltObject3D *object = space->try_get_object(spring_cast->GetContactBodyID(p_body_index));
	if (object == nullptr) {
		// TODO: when there is an ID validity test added, this should error because the body
		// existed but the space failed to return it for some reason.
		return nullptr;
	}

	Object *body = ObjectDB::get_instance(object->get_instance_id());
	// TODO: this should error (?) if the body is nullptr, because we expected to find one.
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

	const JoltSpace3D *space = _get_space();
	const JoltObject3D *object = space->try_get_object(spring_cast->GetContactBodyID(p_body_index));
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

	const JoltSpace3D *space = _get_space();
	if (space == nullptr) {
		// TODO: print an error
		return -1;
	}

	const JoltShapedObject3D *object = space->try_get_shaped(spring_cast->GetContactBodyID(p_body_index));
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
	if (spring_cast == nullptr) {
		// NOTE: allow this to be a check for nullptr as well, no errors if spring_cast doesn't exist
		return 0;
	}

	if (p_body_index < 0) {
		return spring_cast->GetNumContactPoint();
	}

	// TODO: SpringCastConstraint should return -1 if body or shape index are invalid without asserts
	if (p_shape_index < 0) {
		return spring_cast->GetNumContactPoint(p_body_index);
	}

	return spring_cast->GetNumContactPoint(p_body_index, p_shape_index);
}

Vector3 SpringCast::get_contact_point(int p_idx, int p_body_index, int p_shape_index) const {
	// NOTE: body index and shape index can be -1, indicating the user wants to index on all
	// contact points, points on a specific body, or points on a specific shape.
	if (spring_cast == nullptr) {
		// TODO: print error, this should only be called when the user knows contact points exist
		return Vector3();
	}

	if (p_body_index < 0) {
		return to_godot(spring_cast->GetContactPoint(p_idx));
	}

	// TODO: SpringCastConstraint should return -1 if body or shape index are invalid without asserts
	if (p_shape_index < 0) {
		return to_godot(spring_cast->GetContactPoint(p_idx, p_body_index));
	}

	return to_godot(spring_cast->GetContactPoint(p_idx, p_body_index, p_shape_index));
}

Vector3 SpringCast::get_contact_friction(int p_body_index) const {
	// NOTE: body index and shape index can be -1, indicating the user wants to index on all
	// contact points, points on a specific body, or points on a specific shape.
	if (spring_cast == nullptr) {
		// TODO: print error, this should only be called when the user knows contact points exist
		return Vector3();
	}

	return to_godot(spring_cast->GetFrictionForce(p_body_index));
}

float SpringCast::get_spring_length() const {
	if (spring_cast == nullptr || Engine::get_singleton()->is_editor_hint()) {
		if (!settings.is_valid()) {
			// Treat no-spring and no-setting as a zero length, assuming devs use a fallback of
			// zero for max-length when they don't have settings (sounds reasonable to me)
			return 0.0;
		}

		// NOTE: since shorter lengths imply higher force, I think the neutral result is max_length
		// when no spring exists. I would rather not use INFINITY or NAN because those require more
		// logical tests when usually length is used as (max_length - spring_length) * force.
		return settings->get_max_length();
	}

	return spring_cast->GetSpringLength();
}

bool SpringCast::has_hit_hard_point() const {
	if (spring_cast == nullptr) {
		return false;
	}

	return spring_cast->HasHitHardPoint();
}

float SpringCast::get_spring_lambda() const {
	if (spring_cast == nullptr) {
		return 0.0;
	}

	return spring_cast->GetSpringLambda();
}

float SpringCast::get_friction_lambda() const {
	if (spring_cast == nullptr) {
		return 0.0;
	}

	return spring_cast->GetFrictionLambda();
}

float SpringCast::get_angular_lambda() const {
	if (spring_cast == nullptr) {
		return 0.0;
	}

	return spring_cast->GetFrictionAngularLambda();
}

void SpringCast::add_exception_rid(const RID &p_rid) {
	exclude.insert(p_rid);
}

void SpringCast::add_exception(RequiredParam<const PhysicsBody3D> rp_node) {
	EXTRACT_PARAM_OR_FAIL_MSG(p_node, rp_node, "The passed Node must be an instance of PhysicsBody3D.");
	exclude.insert(p_node->get_rid());
}

void SpringCast::remove_exception_rid(const RID &p_rid) {
	exclude.erase(p_rid);
}

void SpringCast::remove_exception(RequiredParam<const PhysicsBody3D> rp_node) {
	EXTRACT_PARAM_OR_FAIL_MSG(p_node, rp_node, "The passed Node must be an instance of PhysicsBody3D.");
	exclude.erase(p_node->get_rid());
}

void SpringCast::clear_exceptions() {
	exclude.clear();
}

void SpringCast::_create_debug_shape() {
	_update_debug_shape_material();

	if (!debug_instance.is_valid()) {
		debug_instance = RenderingServer::get_singleton()->instance_create();
	}

	if (debug_mesh.is_null()) {
		debug_mesh.instantiate();
	}
}

void SpringCast::_clear_debug_shape() {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	if (debug_instance.is_valid()) {
		RenderingServer::get_singleton()->free_rid(debug_instance);
		debug_instance = RID();
	}
	if (debug_mesh.is_valid()) {
		RenderingServer::get_singleton()->free_rid(debug_mesh->get_rid());
		debug_mesh = Ref<ArrayMesh>();
	}
}

void SpringCast::_update_debug_shape() {
	if (!debug_instance.is_valid()) {
		_create_debug_shape();
	}

	_update_debug_shape_vertices();

	if (!debug_instance.is_valid() || debug_mesh.is_null()) {
		return;
	}

	debug_mesh->clear_surfaces();

	PhysicsBody3D *body = _get_body();
	if (body == nullptr) {
		return;
	}

	Array a;
	a.resize(Mesh::ARRAY_MAX);

	uint32_t flags = 0;
	int surface_count = 0;

	if (!debug_shape_vertices.is_empty()) {
		a[Mesh::ARRAY_VERTEX] = debug_shape_vertices;
		debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINE_STRIP, a, Array(), Dictionary(), flags);
		debug_mesh->surface_set_material(surface_count, debug_material);
		++surface_count;
	}

	if (!debug_line_vertices.is_empty()) {
		a[Mesh::ARRAY_VERTEX] = debug_line_vertices;
		debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINE_STRIP, a, Array(), Dictionary(), flags);
		debug_mesh->surface_set_material(surface_count, debug_material);
		++surface_count;
	}

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	RenderingServer::get_singleton()->instance_set_base(debug_instance, debug_mesh->get_rid());
	if (is_inside_tree()) {
		RenderingServer::get_singleton()->instance_set_scenario(debug_instance, get_world_3d()->get_scenario());
		RenderingServer::get_singleton()->instance_set_visible(debug_instance, is_visible_in_tree());
		Vector3 offset = Vector3();
		if (settings.is_valid()) {
			offset = settings->get_position();
		}
		RenderingServer::get_singleton()->instance_set_transform(debug_instance, body->get_global_transform().translated_local(offset));
	}
}

void SpringCast::_update_debug_shape_material() {
	if (debug_material.is_null()) {
		Ref<StandardMaterial3D> material = memnew(StandardMaterial3D);
		debug_material = material;

		material->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
		material->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
		// Use double-sided rendering so that the RayCast can be seen if the camera is inside.
		material->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
		material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
	}

	Color color = debug_custom_color;
	if (color == Color(0.0, 0.0, 0.0, 0.0)) {
		// Use the default debug shape color defined in the Project Settings.
		color = get_tree()->get_debug_collisions_color();
	}

	if (is_colliding()) {
		if ((color.get_h() > 0.236 && color.get_h() < 0.417) && color.get_s() >= 0.8 && color.get_v() >= 0.8) {
			// If base color is already quite green, highlight collision with red color
			color = Color(1.0, 0, 0, color.a);
		} else {
			// Else, highlight collision with green color
			color = Color(0.0, 1.0, 0.0, color.a);
		}
	}

	Ref<StandardMaterial3D> material = static_cast<Ref<StandardMaterial3D>>(debug_material);
	material->set_albedo(color);
}

void SpringCast::_update_debug_shape_vertices() {
	debug_shape_vertices.clear();
	debug_line_vertices.clear();

	if (!settings.is_valid()) {
		return;
	}

	static const int SEGMENTS = 40; // NOTE: must be divisible by 4 to make the line strip work correctly
	static const int FIRST_QUARTER = SEGMENTS / 4;
	const float radius = settings->get_radius();
	const Vector3 offset = settings->get_direction() * get_spring_length();

	// First circle, Z plane
	for (int i = 0; i < SEGMENTS; ++i) {
		float d = (Math::TAU * i) / SEGMENTS;
		Vector2 p = Vector2(Math::cos(d), Math::sin(d)) * radius;
		debug_shape_vertices.push_back(Vector3(p.x, p.y, 0.0) + offset);
	}

	// Second circle, Y plane, first 1/4
	for (int i = 0; i < FIRST_QUARTER; ++i) {
		float d = (Math::TAU * i) / SEGMENTS;
		Vector2 p = Vector2(Math::cos(d), Math::sin(d)) * radius;
		debug_shape_vertices.push_back(Vector3(p.x, 0.0, p.y) + offset);
	}

	// Third circle, X plane
	for (int i = 0; i < SEGMENTS; ++i) {
		float d = (Math::TAU * i) / SEGMENTS;
		Vector2 p = Vector2(Math::cos(d), Math::sin(d)) * radius;
		debug_shape_vertices.push_back(Vector3(0.0, p.y, p.x) + offset);
	}

	// Finish second circle, Y plane, last 3/4
	for (int i = FIRST_QUARTER; i < SEGMENTS; ++i) {
		float d = (Math::TAU * i) / SEGMENTS;
		Vector2 p = Vector2(Math::cos(d), Math::sin(d)) * radius;
		debug_shape_vertices.push_back(Vector3(p.x, 0.0, p.y) + offset);
	}

	// Finish the strip
	debug_shape_vertices.push_back(Vector3(radius, 0.0, 0.0) + offset);

	debug_line_vertices.push_back(Vector3());
	debug_line_vertices.push_back(offset);
}
