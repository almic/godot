#pragma once

#include "jolt_vehicle_settings.h"
#include "vehicle_controller.h"

#include "scene/3d/physics/rigid_body_3d.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

#include "Jolt/Physics/PhysicsSystem.h"


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

	Ref<JoltVehicleSettings> get_settings() const { return settings; }
	void set_settings(const Ref<JoltVehicleSettings>& p_settings) { settings = p_settings; }

	Ref<VehicleController> get_controller() const { return controller; }

protected:
	static void _bind_methods();

	Ref<JoltVehicleSettings> settings;
	Ref<VehicleController> controller;

	static void init_jolt_vehicle(JoltVehicle& jolt_vehicle, PhysicsDirectBodyState3D *state)
	{
		JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
		JoltPhysicsDirectSpaceState3D *jolt_direct_space = (JoltPhysicsDirectSpaceState3D*) state->get_space_state();
		JoltSpace3D &jolt_space = jolt_direct_space->get_space();
		JPH::PhysicsSystem &physics_system = jolt_space.get_physics_system();

		JoltBody3D *jolt_body = jolt_server->get_body(jolt_vehicle.get_rid());
		JoltWritableBody3D body = jolt_space.write_body(*((JoltObject3D*) jolt_body));

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

			physics_system.AddConstraint(jolt_vehicle.vehicle);
			physics_system.AddStepListener(jolt_vehicle.vehicle);
		}
	}
};

