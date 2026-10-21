#include "vehicle.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"

#include "modules/jolt_physics/jolt_globals.h"
#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "modules/jolt_physics/objects/jolt_body_3d.h"
#include "modules/jolt_physics/spaces/jolt_broad_phase_layer.h"
#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

Vehicle::~Vehicle() {
	if (vehicle != nullptr) {
		physics_system->RemoveStepListener(vehicle);
	}
}

void Vehicle::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_wheel_node_paths"), &Vehicle::get_wheel_node_paths);
	ClassDB::bind_method(D_METHOD("set_wheel_node_paths", "wheel_node_paths"), &Vehicle::set_wheel_node_paths);

	ClassDB::bind_method(D_METHOD("get_wheel_nodes"), &Vehicle::get_wheel_nodes);
	ClassDB::bind_method(D_METHOD("set_wheel_nodes", "wheel_nodes"), &Vehicle::set_wheel_nodes);

	ClassDB::bind_method(D_METHOD("get_wheel_collision_mode"), &Vehicle::get_wheel_collision_mode);
	ClassDB::bind_method(D_METHOD("set_wheel_collision_mode", "collision_mode"), &Vehicle::set_wheel_collision_mode);

	ClassDB::bind_method(D_METHOD("get_wheel_sphere_test_radius"), &Vehicle::get_wheel_sphere_test_radius);
	ClassDB::bind_method(D_METHOD("set_wheel_sphere_test_radius", "radius"), &Vehicle::set_wheel_sphere_test_radius);

	ClassDB::bind_method(D_METHOD("get_settings"), &Vehicle::get_settings);
	ClassDB::bind_method(D_METHOD("set_settings", "vehicle_settings"), &Vehicle::set_settings);

	ClassDB::bind_method(D_METHOD("get_controller"), &Vehicle::get_controller);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "wheel_nodes", PROPERTY_HINT_ARRAY_TYPE, "Node3D", PROPERTY_USAGE_NONE), "set_wheel_nodes", "get_wheel_nodes");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "wheel_paths", PROPERTY_HINT_ARRAY_TYPE, vformat("%d/%d:%s", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D")), "set_wheel_node_paths", "get_wheel_node_paths");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "wheel_collision_mode", PROPERTY_HINT_ENUM, "Raycast,Sphere,Cylinder"), "set_wheel_collision_mode", "get_wheel_collision_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_sphere_test_radius", PROPERTY_HINT_RANGE, "0.001,1.0,0.001,or_greater,suffix:m"), "set_wheel_sphere_test_radius", "get_wheel_sphere_test_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "settings", PROPERTY_HINT_RESOURCE_TYPE, "VehicleSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_settings", "get_settings");

	BIND_ENUM_CONSTANT(WHEEL_COLLISION_RAYCAST);
	BIND_ENUM_CONSTANT(WHEEL_COLLISION_SPHERE);
	BIND_ENUM_CONSTANT(WHEEL_COLLISION_CYLINDER);
}

void VehicleSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_up"), &VehicleSettings::get_up);
	ClassDB::bind_method(D_METHOD("set_up", "up"), &VehicleSettings::set_up);

	ClassDB::bind_method(D_METHOD("get_forward"), &VehicleSettings::get_forward);
	ClassDB::bind_method(D_METHOD("set_forward", "forward"), &VehicleSettings::set_forward);

	ClassDB::bind_method(D_METHOD("is_pitch_roll_limited"), &VehicleSettings::is_pitch_roll_limited);
	ClassDB::bind_method(D_METHOD("set_pitch_roll_limited", "is_limited"), &VehicleSettings::set_pitch_roll_limited);

	ClassDB::bind_method(D_METHOD("get_max_pitch_roll"), &VehicleSettings::get_max_pitch_roll);
	ClassDB::bind_method(D_METHOD("set_max_pitch_roll", "max_pitch_roll"), &VehicleSettings::set_max_pitch_roll);

	ClassDB::bind_method(D_METHOD("get_wheel_settings"), &VehicleSettings::get_wheel_settings);
	ClassDB::bind_method(D_METHOD("set_wheel_settings", "wheel_settings"), &VehicleSettings::set_wheel_settings);

	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_settings"), &VehicleSettings::get_anti_roll_bar_settings);
	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_settings", "anti_roll_bar_settings"), &VehicleSettings::set_anti_roll_bar_settings);

	ClassDB::bind_method(D_METHOD("get_controller_settings"), &VehicleSettings::get_controller_settings);
	ClassDB::bind_method(D_METHOD("set_controller_settings", "controller_settings"), &VehicleSettings::set_controller_settings);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up"), "set_up", "get_up");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "forward"), "set_forward", "get_forward");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "pitch_roll_limited"), "set_pitch_roll_limited", "is_pitch_roll_limited");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_pitch_roll", PROPERTY_HINT_RANGE, "0,89,0.1,radians_as_degrees"), "set_max_pitch_roll", "get_max_pitch_roll");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "wheels", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("WheelBaseSettings")), "set_wheel_settings", "get_wheel_settings");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "anti_roll_bars", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("AntiRollBarSettings")), "set_anti_roll_bar_settings", "get_anti_roll_bar_settings");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "controller", PROPERTY_HINT_RESOURCE_TYPE, "VehicleControllerSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_controller_settings", "get_controller_settings");
}

void VehicleSettings::_validate_property(PropertyInfo &p_property) const {
	if (!is_pitch_roll_limited() && p_property.name == "max_pitch_roll") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void Vehicle::_validate_property(PropertyInfo &p_property) const {
	if (get_wheel_collision_mode() != WHEEL_COLLISION_SPHERE && p_property.name == "wheel_sphere_test_radius") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void Vehicle::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			_setup_wheels();
			_update_wheel_transforms();
		} break;
	}
}

void Vehicle::_body_state_changed(PhysicsDirectBodyState3D *p_state) {
	if (!is_initialized) {
		init_vehicle(*this, p_state);
		init_collision_tester(*this);
		is_initialized = true;
	}

	RigidBody3D::_body_state_changed(p_state);

	_update_wheel_transforms();
}

void Vehicle::_update_wheel_transforms() {
	if (vehicle != nullptr) {
		for (int i = 0, m = MIN(wheels.size(), vehicle->GetWheels().size()); i < m; ++i) {
			Node3D *wheel = wheels[i];

			if (wheel == nullptr) {
				continue;
			}

			wheel->set_transform(to_godot(
					vehicle->GetWheelLocalTransform(
							i,
							to_jolt(Vector3(0, -1, 0)),
							to_jolt(Vector3(1, 0, 0)))));
		}
	} else if (settings.is_valid()) {
		TypedArray<WheelBaseSettings> wheel_settings = settings->get_wheel_settings();
		for (int i = 0, m = MIN(wheels.size(), wheel_settings.size()); i < m; ++i) {
			Node3D *wheel = wheels[i];

			if (wheel == nullptr) {
				continue;
			}

			Ref<WheelBaseSettings> w_settings = wheel_settings[i];
			if (w_settings.is_valid()) {
				wheel->set_transform(GetWheelLocalTransform(w_settings->get_settings()));
			}
		}
	}
}

// Duplicated from Jolt because these should honestly be static methods but they aren't
Transform3D Vehicle::GetWheelLocalTransform(const JPH::WheelSettings *inWheelSettings) {
	// Use the two vectors provided to calculate a matrix that takes us from wheel model space to X = right, Y = up, Z = forward (the space where we will rotate the wheel)
	static const JPH::Vec3 wheel_right = JPH::Vec3(0, -1, 0);
	static const JPH::Vec3 wheel_up = JPH::Vec3(1, 0, 0);
	static const JPH::Mat44 wheel_to_rotational =
			JPH::Mat44(
					JPH::Vec4(wheel_right, 0),
					JPH::Vec4(wheel_up, 0),
					JPH::Vec4(wheel_up.Cross(wheel_right), 0),
					JPH::Vec4(0, 0, 0, 1))
					.Transposed();

	// Calculate the matrix that takes us from the rotational space to vehicle local space
	JPH::Quat steer_rotation = JPH::Quat::sRotation(inWheelSettings->mSteeringAxis, 0.0f);
	JPH::Vec3 local_up = steer_rotation * inWheelSettings->mWheelUp;
	JPH::Vec3 local_forward = steer_rotation * inWheelSettings->mWheelForward;
	JPH::Vec3 local_right = local_forward.Cross(local_up).Normalized();
	local_forward = local_up.Cross(local_right).Normalized();

	JPH::Vec3 local_wheel_pos = inWheelSettings->mPosition + inWheelSettings->mSuspensionDirection * inWheelSettings->mSuspensionMinLength;
	JPH::Mat44 rotational_to_local(JPH::Vec4(local_right, 0), JPH::Vec4(local_up, 0), JPH::Vec4(local_forward, 0), JPH::Vec4(local_wheel_pos, 1));

	// Calculate transform of rotated wheel
	return to_godot(rotational_to_local * JPH::Mat44::sRotationX(0.0f) * wheel_to_rotational);
}

TypedArray<NodePath> Vehicle::get_wheel_node_paths() const {
	TypedArray<NodePath> result;
	int size = wheel_paths.size();
	result.resize(size);
	for (int i = 0; i < size; ++i) {
		result[i] = wheel_paths[i];
	}
	return result;
}

void Vehicle::set_wheel_node_paths(const TypedArray<NodePath> &p_paths) {
	wheels.clear();
	wheel_paths.clear();

	wheel_paths.resize(p_paths.size());

	int i = 0;
	for (const auto &p : p_paths) {
		wheel_paths[i++] = p;
	}

	_setup_wheels();
	_update_wheel_transforms();
}

TypedArray<Node3D> Vehicle::get_wheel_nodes() const {
	TypedArray<Node3D> result;
	int size = wheels.size();
	result.resize(size);
	for (int i = 0; i < size; ++i) {
		result[i] = wheels[i];
	}
	return result;
}

void Vehicle::set_wheel_nodes(const TypedArray<Node3D> &p_nodes) {
	wheels.clear();
	wheel_paths.clear();

	int size = p_nodes.size();

	wheels.resize(size);
	wheel_paths.resize(size);

	for (int i = 0; i < size; ++i) {
		Node3D *n = dynamic_cast<Node3D *>((Object *)p_nodes[i]);
		if (n != nullptr) {
			wheels[i] = n;
			wheel_paths[i] = get_path_to(n);
		}
	}
}

void Vehicle::set_wheel_collision_mode(WheelCollisionMode p_mode) {
	bool do_update = collision_mode != p_mode;
	collision_mode = p_mode;
	if (do_update) {
		if (vehicle != nullptr) {
			init_collision_tester(*this);
		}
		notify_property_list_changed();
	}
}

void Vehicle::set_settings(const Ref<VehicleSettings> &p_settings) {
	if (settings.is_valid()) {
		settings->disconnect_changed(callable_mp(this, &Vehicle::_update_wheel_transforms));
	}

	settings = p_settings;

	if (settings.is_valid()) {
		settings->connect_changed(callable_mp(this, &Vehicle::_update_wheel_transforms));
	}

	_update_wheel_transforms();
}

JoltBody3D *Vehicle::get_jolt_body() const {
	JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
	return jolt_server->get_body(get_rid());
}

void Vehicle::_setup_wheels() {
	if (!is_inside_tree()) {
		return;
	}

	wheels.resize(wheel_paths.size());

	int i = 0;
	for (const auto &p : wheel_paths) {
		if (p.is_empty()) {
			wheels[i++] = nullptr;
			continue;
		}

		Node *n = get_node_or_null(p);
		Node3D *n_3d = dynamic_cast<Node3D *>(n);

		wheels[i++] = n_3d;

		if (n == nullptr) {
			ERR_PRINT(vformat("Vehicle could not find node at path \"%s\"!", p));
		} else if (n_3d == nullptr) {
			ERR_PRINT("Vehicle got a wheel node whose type is not derived from Node3D!");
		}
	}
}

void Vehicle::init_vehicle(Vehicle &vehicle, PhysicsDirectBodyState3D *state) {
	JoltPhysicsDirectSpaceState3D *jolt_direct_space = (JoltPhysicsDirectSpaceState3D *)state->get_space_state().ptr();
	JoltSpace3D &jolt_space = jolt_direct_space->get_space();
	JPH::PhysicsSystem &physics_system = jolt_space.get_physics_system();

	JoltBody3D *jolt_body = vehicle.get_jolt_body();
	JPH::Body *body = jolt_body->get_jolt_body();

	vehicle.physics_system = &physics_system;
	vehicle.vehicle = new JPH::VehicleConstraint(*body, vehicle.settings->settings);
	JPH::VehicleController *jolt_controller = vehicle.vehicle->GetController();

	if (dynamic_cast<JPH::WheeledVehicleController *>(jolt_controller) != nullptr) {
		vehicle.controller = memnew(WheeledVehicleController);
	}

	if (vehicle.controller == nullptr) {
		ERR_PRINT("Vehicle has an unknown or empty VehicleControllerSettings! Please check this!");
	} else {
		vehicle.controller->set_controller(jolt_controller);

		physics_system.AddConstraint(vehicle.vehicle);
		physics_system.AddStepListener(vehicle.vehicle);
	}
}

void Vehicle::init_collision_tester(Vehicle &vehicle) {
	if (vehicle.vehicle == nullptr) {
		ERR_PRINT("Vehicle has a null constraint, unable to initialize collision tester!");
		return;
	}

	JoltBody3D *jolt_body = vehicle.get_jolt_body();
	JoltSpace3D *jolt_space = jolt_body->get_space();
	JPH::ObjectLayer layer = jolt_space->map_to_object_layer(
			JoltBroadPhaseLayer::BODY_DYNAMIC,
			jolt_body->get_collision_layer(),
			jolt_body->get_collision_mask());

	switch (vehicle.collision_mode) {
		case WHEEL_COLLISION_RAYCAST: {
			vehicle.tester = vehicle_collision_tester_ray(layer);
			break;
		}
		case WHEEL_COLLISION_SPHERE: {
			vehicle.tester = vehicle_collision_tester_sphere(layer, vehicle.sphere_test_radius);
			break;
		}
		case WHEEL_COLLISION_CYLINDER: {
			vehicle.tester = vehicle_collision_tester_cylinder(layer);
			break;
		}
		default: {
			ERR_PRINT("Vehicle got unknown wheel collision mode!");
			return;
		}
	}

	vehicle.vehicle->SetVehicleCollisionTester(vehicle.tester);
}

Vector3 VehicleSettings::get_up() const {
	return to_godot(settings.mUp);
}

void VehicleSettings::set_up(const Vector3 &p_up) {
	settings.mUp = to_jolt(p_up.normalized());
}

Vector3 VehicleSettings::get_forward() const {
	return to_godot(settings.mForward);
}

void VehicleSettings::set_forward(const Vector3 &p_forward) {
	settings.mForward = to_jolt(p_forward.normalized());
}

void VehicleSettings::set_pitch_roll_limited(bool p_is_limited) {
	limit_pitch_roll = p_is_limited;

	if (limit_pitch_roll) {
		settings.mMaxPitchRollAngle = max_pitch_roll;
	} else {
		settings.mMaxPitchRollAngle = JPH::JPH_PI;
	}

	notify_property_list_changed();
}

void VehicleSettings::set_max_pitch_roll(float p_max_pitch_roll) {
	max_pitch_roll = CLAMP(p_max_pitch_roll, 0.0, JPH::JPH_PI);

	if (is_pitch_roll_limited()) {
		settings.mMaxPitchRollAngle = max_pitch_roll;
	}
}

TypedArray<WheelBaseSettings> VehicleSettings::get_wheel_settings() const {
	TypedArray<WheelBaseSettings> result;

	for (const auto &w : wheels) {
		result.push_back(w);
	}

	return result;
}

void VehicleSettings::set_wheel_settings(const TypedArray<WheelBaseSettings> &p_wheel_settings) {
	for (const auto &w : wheels) {
		Ref<WheelBaseSettings> wheel = w;
		if (wheel.is_valid()) {
			wheel->disconnect_changed(callable_mp((Resource *)this, &VehicleSettings::emit_changed));
		}
	}

	wheels.clear();
	settings.mWheels.clear();

	for (const auto &w : p_wheel_settings) {
		Ref<WheelBaseSettings> wheel = w;
		wheels.push_back(wheel);
		if (wheel.is_valid()) {
			wheel->connect_changed(callable_mp((Resource *)this, &VehicleSettings::emit_changed));
			settings.mWheels.push_back(wheel->get_settings());
		}
	}
}

TypedArray<AntiRollBarSettings> VehicleSettings::get_anti_roll_bar_settings() const {
	TypedArray<AntiRollBarSettings> result;

	for (const auto &a : anti_roll_bars) {
		result.push_back(a);
	}

	return result;
}

void VehicleSettings::set_anti_roll_bar_settings(const TypedArray<AntiRollBarSettings> &p_anti_roll_bar_settings) {
	anti_roll_bars.clear();
	settings.mAntiRollBars.clear();

	for (const auto &a : p_anti_roll_bar_settings) {
		Ref<AntiRollBarSettings> anti_roll_bar = a;
		anti_roll_bars.push_back(anti_roll_bar);
		if (anti_roll_bar.is_valid()) {
			settings.mAntiRollBars.push_back(anti_roll_bar->settings);
		}
	}
}

void VehicleSettings::set_controller_settings(const Ref<VehicleControllerSettings> &p_controller) {
	controller = p_controller;
	if (controller.is_valid()) {
		settings.mController = controller->get_settings();
	}
}
