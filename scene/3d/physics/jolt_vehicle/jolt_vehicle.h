#pragma once

#include "jolt_vehicle_settings.h"
#include "vehicle_controller.h"

#include "scene/3d/physics/rigid_body_3d.h"

#include "modules/jolt_physics/jolt_globals.h"
#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "modules/jolt_physics/objects/jolt_body_3d.h"
#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

#include "modules/jolt_physics/spaces/jolt_broad_phase_layer.h"

#include "Jolt/Physics/PhysicsSystem.h"

class JoltVehicle : public RigidBody3D {
	GDCLASS(JoltVehicle, RigidBody3D);

	JPH::Ref<JPH::VehicleConstraint> vehicle;
	JPH::Ref<JPH::VehicleCollisionTester> tester;
	JPH::PhysicsSystem *physics_system;
	bool is_initialized = false;

	virtual void _body_state_changed(PhysicsDirectBodyState3D *p_state) override {
		if (!is_initialized) {
			init_jolt_vehicle(*this, p_state);
			init_collision_tester(*this);
			is_initialized = true;
		}

		RigidBody3D::_body_state_changed(p_state);

		_update_wheel_transforms();
	}

	void _update_wheel_transforms() {
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
	static Transform3D GetWheelLocalTransform(const JPH::WheelSettings *inWheelSettings) {
		// Use the two vectors provided to calculate a matrix that takes us from wheel model space to X = right, Y = up, Z = forward (the space where we will rotate the wheel)
		JPH::Vec3 wheel_right = JPH::Vec3(0, -1, 0);
		JPH::Vec3 wheel_up = JPH::Vec3(1, 0, 0);
		JPH::Mat44 wheel_to_rotational =
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

public:
	enum WheelCollisionMode {
		WHEEL_COLLISION_RAYCAST,
		WHEEL_COLLISION_SPHERE,
		WHEEL_COLLISION_CYLINDER,
	};

	~JoltVehicle() {
		if (vehicle != nullptr) {
			physics_system->RemoveStepListener(vehicle);
		}
	}

	TypedArray<NodePath> get_wheel_node_paths() const {
		TypedArray<NodePath> result;
		int size = wheel_paths.size();
		result.resize(size);
		for (int i = 0; i < size; ++i) {
			result[i] = wheel_paths[i];
		}
		return result;
	}

	void set_wheel_node_paths(const TypedArray<NodePath> &p_paths) {
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

	TypedArray<Node3D> get_wheel_nodes() const {
		TypedArray<Node3D> result;
		int size = wheels.size();
		result.resize(size);
		for (int i = 0; i < size; ++i) {
			result[i] = wheels[i];
		}
		return result;
	}

	void set_wheel_nodes(const TypedArray<Node3D> &p_nodes) {
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

	WheelCollisionMode get_wheel_collision_mode() const {
		return collision_mode;
	}
	void set_wheel_collision_mode(WheelCollisionMode p_mode) {
		bool do_update = collision_mode != p_mode;
		collision_mode = p_mode;
		if (do_update) {
			if (vehicle != nullptr) {
				init_collision_tester(*this);
			}
			notify_property_list_changed();
		}
	}

	real_t get_wheel_sphere_test_radius() const {
		return sphere_test_radius;
	}
	void set_wheel_sphere_test_radius(real_t p_radius) {
		sphere_test_radius = p_radius;
	}

	Ref<JoltVehicleSettings> get_settings() const {
		return settings;
	}
	void set_settings(const Ref<JoltVehicleSettings> &p_settings) {
		if (settings.is_valid()) {
			settings->disconnect_changed(callable_mp(this, &JoltVehicle::_update_wheel_transforms));
		}

		settings = p_settings;

		if (settings.is_valid()) {
			settings->connect_changed(callable_mp(this, &JoltVehicle::_update_wheel_transforms));
		}

		_update_wheel_transforms();
	}

	Ref<VehicleController> get_controller() const {
		return controller;
	}

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;
	void _notification(int p_what);

	WheelCollisionMode collision_mode = WHEEL_COLLISION_CYLINDER;
	real_t sphere_test_radius = 0.5;
	Ref<JoltVehicleSettings> settings;
	Ref<VehicleController> controller;
	LocalVector<Node3D *> wheels;
	LocalVector<NodePath> wheel_paths;

	JoltBody3D *get_jolt_body() const {
		JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
		return jolt_server->get_body(get_rid());
	}

	void _setup_wheels() {
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
				ERR_PRINT(vformat("JoltVehicle could not find node at path \"%s\"!", p));
			} else if (n_3d == nullptr) {
				ERR_PRINT("JoltVehicle got a wheel node whose type is not derived from Node3D!");
			}
		}
	}

	static void init_jolt_vehicle(JoltVehicle &jolt_vehicle, PhysicsDirectBodyState3D *state) {
		JoltPhysicsDirectSpaceState3D *jolt_direct_space = (JoltPhysicsDirectSpaceState3D *)state->get_space_state().ptr();
		JoltSpace3D &jolt_space = jolt_direct_space->get_space();
		JPH::PhysicsSystem &physics_system = jolt_space.get_physics_system();

		JoltBody3D *jolt_body = jolt_vehicle.get_jolt_body();
		JPH::Body *body = jolt_body->get_jolt_body();

		jolt_vehicle.physics_system = &physics_system;
		jolt_vehicle.vehicle = new JPH::VehicleConstraint(*body, jolt_vehicle.settings->settings);
		JPH::VehicleController *jolt_controller = jolt_vehicle.vehicle->GetController();

		if (dynamic_cast<JPH::WheeledVehicleController *>(jolt_controller) != nullptr) {
			jolt_vehicle.controller = memnew(WheeledVehicleController);
		}

		if (jolt_vehicle.controller == nullptr) {
			ERR_PRINT("JoltVehicle has an unknown or empty VehicleControllerSettings! Please check this!");
		} else {
			jolt_vehicle.controller->set_controller(jolt_controller);

			physics_system.AddConstraint(jolt_vehicle.vehicle);
			physics_system.AddStepListener(jolt_vehicle.vehicle);
		}
	}

	static void init_collision_tester(JoltVehicle &jolt_vehicle) {
		if (jolt_vehicle.vehicle == nullptr) {
			ERR_PRINT("JoltVehicle has a null constraint, unable to initialize collision tester!");
			return;
		}

		JoltBody3D *jolt_body = jolt_vehicle.get_jolt_body();
		JoltSpace3D *jolt_space = jolt_body->get_space();
		JPH::ObjectLayer layer = jolt_space->map_to_object_layer(
				JoltBroadPhaseLayer::BODY_DYNAMIC,
				jolt_body->get_collision_layer(),
				jolt_body->get_collision_mask());

		switch (jolt_vehicle.collision_mode) {
			case WHEEL_COLLISION_RAYCAST: {
				jolt_vehicle.tester = vehicle_collision_tester_ray(layer);
				break;
			}
			case WHEEL_COLLISION_SPHERE: {
				jolt_vehicle.tester = vehicle_collision_tester_sphere(layer, (float)jolt_vehicle.sphere_test_radius);
				break;
			}
			case WHEEL_COLLISION_CYLINDER: {
				jolt_vehicle.tester = vehicle_collision_tester_cylinder(layer);
				break;
			}
			default: {
				ERR_PRINT("JoltVehicle got unknown wheel collision mode!");
				return;
			}
		}

		jolt_vehicle.vehicle->SetVehicleCollisionTester(jolt_vehicle.tester);
	}
};

VARIANT_ENUM_CAST(JoltVehicle::WheelCollisionMode);
