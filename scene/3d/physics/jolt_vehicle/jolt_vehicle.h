#pragma once

#include "wheel_settings.h"
#include "vehicle_controller_settings.h"
#include "vehicle_differential_settings.h"
#include "vehicle_engine_settings.h"
#include "vehicle_transmission_settings.h"

#include "scene/3d/physics/rigid_body_3d.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "modules/jolt_physics/jolt_physics_server_3d.h"

#include <Jolt/Physics/PhysicsSystem.h>
#include "Jolt/Physics/Vehicle/VehicleConstraint.h"


class JoltVehicle;

class JoltVehicleSettings : Resource
{
	GDCLASS(JoltVehicleSettings, Resource);

protected:
	JPH::VehicleConstraintSettings settings;

	friend class JoltVehicle;

public:

	Vector3 get_up() const { return to_godot(settings.mUp); }
	void set_up(const Vector3 &p_up) { settings.mUp = to_jolt(p_up); }

	Vector3 get_forward() const { return to_godot(settings.mForward); }
	void set_forward(const Vector3 &p_forward) { settings.mForward = to_jolt(p_forward); }

	bool can_pitch_roll() const { return settings.mMaxPitchRollAngle == JPH_PI; }
	void set_can_pitch_roll(bool p_can_pitch_roll)
	{
		if (p_can_pitch_roll)
		{
			settings.mMaxPitchRollAngle = (float) max_pitch_roll_angle;
		}
		else
		{
			settings.mMaxPitchRollAngle = JPH_PI;
		}

		notify_property_list_changed();
	}

	real_t get_max_pitch_roll() const
	{
		if (can_pitch_roll())
		{
			return max_pitch_roll_angle;
		}
		else
		{
			return (real_t) settings.mMaxPitchRollAngle;
		}
	}
	void set_max_pitch_roll(real_t p_max_pitch_roll)
	{
		if (p_max_pitch_roll >= M_PI)
		{
			can_pitch_roll(false);
		}
		else
		{
			max_pitch_roll_angle = p_max_pitch_roll;
			settings.mMaxPitchRollAngle = (float) p_max_pitch_roll;
			notify_property_list_changed();
		}
	}

	TypedArray<WheelBaseSettings> get_wheel_settings() const
	{
		TypedArray<WheelBaseSettings> result;

		for (const auto& w : wheels)
		{
			result.push_back(w);
		}

		return result;
	}
	void set_wheel_settings(const TypedArray<WheelBaseSettings> &p_wheel_settings)
	{
		wheels.clear();
		settings.mWheels.clear();

		for (auto& w : p_wheel_settings)
		{
			Ref<WheelBaseSettings> wheel = w;
			wheels.push_back(wheel);
			settings.mWheels.push_back(wheel->get_settings());
		}
	}

	TypedArray<AntiRollBarSettings> get_anti_roll_bar_settings() const
	{
		TypedArray<AntiRollBarSettings> result;

		for (const auto& a : anti_roll_bars)
		{
			result.push_back(a);
		}

		return result;
	}
	void set_anti_roll_bar_settings(const TypedArray<AntiRollBarSettings> &p_anti_roll_bar_settings)
	{
		anti_roll_bars.clear();
		settings.mAntiRollBars.clear();

		for (auto& a : p_anti_roll_bar_settings)
		{
			Ref<AntiRollBarSettings> anti_roll_bar = a;
			anti_roll_bars.push_back(anti_roll_bar);
			settings.mAntiRollBars.push_back(anti_roll_bar->settings);
		}
	}

	Ref<VehicleControllerSettings> get_controller_settings() const { return controller; }
	void set_controller_settings(const Ref<VehicleControllerSettings> &p_controller)
	{
		controller = p_controller;
		settings.mController = controller->get_settings();
	}

protected:
	static void _bind_methods();

	real_t max_pitch_roll_angle;
	LocalVector<Ref<WheelBaseSettings>> wheels;
	LocalVector<Ref<AntiRollBarSettings>> anti_roll_bars;
	Ref<VehicleControllerSettings> controller;
};

class JoltVehicle : public RigidBody3D
{
	GDCLASS(JoltVehicle, RigidBody3D);

	JPH::Ref<JPH::VehicleConstraint> vehicle;
	JPH::PhysicsSystem* physics_system;
	bool is_initialized = false;

	virtual void _body_state_changed(PhysicsDirectBodyState3D *p_state) override
	{
		if (!is_initialized)
		{
			init_jolt_vehicle(*this, p_state);
			is_initialized = true;
		}

		RigidBody3D::_body_state_changed(p_state);
	}

public:

	~JoltVehicle()
	{
		if (vehicle != nullptr)
		{
			physics_system->RemoveStepListener(vehicle);
		}
	}

	Ref<VehicleConstraintSettings> get_settings() const { return settings; }
	void set_settings(const Ref<VehicleConstraintSettings> &p_settings) { settings = p_settings; }

	Ref<VehicleController> get_controller() const { return controller; }

protected:
	static void _bind_methods();

	Ref<VehicleConstraintSettings> settings;
	Ref<VehicleController> controller;

	static void init_jolt_vehicle(const JoltVehicle& jolt_vehicle, PhysicsDirectBodyState3D *state)
	{
		JoltPhysicsServer3D *jolt_server = JoltPhysicsServer::get_singleton();
		JoltPhysicsDirectSpaceState3D *jolt_direct_space = (JoltPhysicsDirectSpaceState3D*) state->get_space_state();
		JoltSpace3D &jolt_space = jolt_direct_space->get_space();
		JPH::PhysicsSystem &physics_system = jolt_space.get_physics_system();

		JoltBody3D *jolt_body = jolt_server->get_body(jolt_vehicle.get_rid());
		JoltWritableBody3D body = jolt_space.write_body(*jolt_body);

		jolt_vehicle.physics_system = &physics_system;
		jolt_vehicle.vehicle = new JPH::VehicleConstraint(*body, jolt_vehicle.settings->settings);
		JPH::VehicleController* jolt_controller = jolt_vehicle.vehicle->GetController();

		if (dynamic_cast<JPH::WheeledVehicleController*>(jolt_controller) != nullptr)
		{
			jolt_vehicle.controller = memnew(WheeledVehicleController);
		}

		if (jolt_vehicle.controller == nullptr)
		{
			ERR_FAIL_MSG("JoltVehicle has an unknown or empty VehicleControllerSettings! Please check this!");
		}
		else
		{
			jolt_vehicle.controller->set_controller(jolt_controller);

			physics_system->AddConstraint(jolt_vehicle.vehicle);
			physics_system->AddStepListener(jolt_vehicle.vehicle);
		}
	}
};

