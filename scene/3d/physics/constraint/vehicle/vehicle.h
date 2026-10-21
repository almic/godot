#pragma once

#include "anti_roll_bar_settings.h"
#include "controller.h"
#include "wheel_settings.h"

#include "core/io/resource.h"
#include "core/object/ref_counted.h"
#include "scene/3d/physics/rigid_body_3d.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

class JoltBody3D;
class VehicleSettings;
class VehicleController;

class Vehicle : public RigidBody3D {
	GDCLASS(Vehicle, RigidBody3D);

	JPH::Ref<JPH::VehicleConstraint> vehicle;
	JPH::Ref<JPH::VehicleCollisionTester> tester;
	JPH::PhysicsSystem *physics_system;
	bool is_initialized = false;

	virtual void _body_state_changed(PhysicsDirectBodyState3D *p_state) override;

	void _update_wheel_transforms();

	// Duplicated from Jolt because these should honestly be static methods but they aren't
	static Transform3D GetWheelLocalTransform(const JPH::WheelSettings *inWheelSettings);

public:
	enum WheelCollisionMode {
		WHEEL_COLLISION_RAYCAST,
		WHEEL_COLLISION_SPHERE,
		WHEEL_COLLISION_CYLINDER,
	};

	~Vehicle();

	TypedArray<NodePath> get_wheel_node_paths() const;
	void set_wheel_node_paths(const TypedArray<NodePath> &p_paths);

	TypedArray<Node3D> get_wheel_nodes() const;
	void set_wheel_nodes(const TypedArray<Node3D> &p_nodes);

	WheelCollisionMode get_wheel_collision_mode() const { return collision_mode; }
	void set_wheel_collision_mode(WheelCollisionMode p_mode);

	float get_wheel_sphere_test_radius() const { return sphere_test_radius; }
	void set_wheel_sphere_test_radius(float p_radius) { sphere_test_radius = p_radius; }

	Ref<VehicleSettings> get_settings() const { return settings; }
	void set_settings(const Ref<VehicleSettings> &p_settings);

	Ref<VehicleController> get_controller() const { return controller; }

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;
	void _notification(int p_what);

	WheelCollisionMode collision_mode = WHEEL_COLLISION_CYLINDER;
	float sphere_test_radius = 0.5;
	Ref<VehicleSettings> settings;
	Ref<VehicleController> controller;
	LocalVector<Node3D *> wheels;
	LocalVector<NodePath> wheel_paths;

	JoltBody3D *get_jolt_body() const;

	void _setup_wheels();

	static void init_vehicle(Vehicle &vehicle, PhysicsDirectBodyState3D *state);
	static void init_collision_tester(Vehicle &vehicle);
};

class VehicleSettings : public Resource {
	GDCLASS(VehicleSettings, Resource);

protected:
	JPH::VehicleConstraintSettings settings;

	friend class Vehicle;

public:
	VehicleSettings() {
		// Settings is not actually alloc'd, mark as embedded to prevent release from doing anything
		settings.SetEmbedded();
		settings.AddRef();
	}

	~VehicleSettings() {
		settings.Release();
	}

	Vector3 get_up() const;
	void set_up(const Vector3 &p_up);

	Vector3 get_forward() const;
	void set_forward(const Vector3 &p_forward);

	bool is_pitch_roll_limited() const { return limit_pitch_roll; }
	void set_pitch_roll_limited(bool p_is_limited);

	float get_max_pitch_roll() const { return max_pitch_roll; }
	void set_max_pitch_roll(float p_max_pitch_roll);

	TypedArray<WheelBaseSettings> get_wheel_settings() const;
	void set_wheel_settings(const TypedArray<WheelBaseSettings> &p_wheel_settings);

	TypedArray<AntiRollBarSettings> get_anti_roll_bar_settings() const;
	void set_anti_roll_bar_settings(const TypedArray<AntiRollBarSettings> &p_anti_roll_bar_settings);

	Ref<VehicleControllerSettings> get_controller_settings() const { return controller; }
	void set_controller_settings(const Ref<VehicleControllerSettings> &p_controller);

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	bool limit_pitch_roll = false;
	float max_pitch_roll = 0.0;
	LocalVector<Ref<WheelBaseSettings>> wheels;
	LocalVector<Ref<AntiRollBarSettings>> anti_roll_bars;
	Ref<VehicleControllerSettings> controller;
};

VARIANT_ENUM_CAST(Vehicle::WheelCollisionMode);
