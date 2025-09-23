#pragma once

#include "jolt_vehicle_settings.h"
#include "vehicle_controller.h"

#include "scene/3d/physics/rigid_body_3d.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/objects/jolt_body_3d.h"
#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"
#include "modules/jolt_physics/spaces/jolt_broad_phase_layer.h"

#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Vehicle/VehicleCollisionTester.h"


class JoltVehicle : public RigidBody3D
{
	GDCLASS(JoltVehicle, RigidBody3D);

	JPH::Ref<JPH::VehicleConstraint> vehicle;
	JPH::Ref<JPH::VehicleCollisionTester> tester;
	JPH::PhysicsSystem* physics_system;
	bool is_initialized = false;

	virtual void _body_state_changed(PhysicsDirectBodyState3D *p_state) override
	{
		if (!is_initialized)
		{
			init_jolt_vehicle(*this, p_state);
			init_collision_tester(*this);
			is_initialized = true;
		}

		RigidBody3D::_body_state_changed(p_state);
	}

public:

	enum WheelCollisionMode
	{
		WHEEL_COLLISION_RAYCAST,
		WHEEL_COLLISION_SPHERE,
		WHEEL_COLLISION_CYLINDER,
	};

	~JoltVehicle()
	{
		if (vehicle != nullptr)
		{
			physics_system->RemoveStepListener(vehicle);
		}
	}

	WheelCollisionMode get_wheel_collision_mode() const { return collision_mode; }
	void set_wheel_collision_mode(WheelCollisionMode p_mode)
	{
		bool do_update = collision_mode != p_mode;
		collision_mode = p_mode;
		if (do_update)
		{
			if (vehicle != nullptr)
			{
				init_collision_tester(*this);
			}
			notify_property_list_changed();
		}
	}

	real_t get_wheel_sphere_test_radius() const { return sphere_test_radius; }
	void set_wheel_sphere_test_radius(real_t p_radius) { sphere_test_radius = p_radius; }

	Ref<JoltVehicleSettings> get_settings() const { return settings; }
	void set_settings(const Ref<JoltVehicleSettings>& p_settings) { settings = p_settings; }

	Ref<VehicleController> get_controller() const { return controller; }

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	WheelCollisionMode collision_mode = WHEEL_COLLISION_CYLINDER;
	real_t sphere_test_radius = 0.5;
	Ref<JoltVehicleSettings> settings;
	Ref<VehicleController> controller;

	JoltBody3D* get_jolt_body() const
	{
		JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
		return jolt_server->get_body(get_rid());
	}

	static void init_jolt_vehicle(JoltVehicle& jolt_vehicle, PhysicsDirectBodyState3D *state)
	{
		JoltPhysicsDirectSpaceState3D *jolt_direct_space = (JoltPhysicsDirectSpaceState3D*) state->get_space_state();
		JoltSpace3D &jolt_space = jolt_direct_space->get_space();
		JPH::PhysicsSystem &physics_system = jolt_space.get_physics_system();

		JoltBody3D *jolt_body = jolt_vehicle.get_jolt_body();
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

	static void init_collision_tester(JoltVehicle& jolt_vehicle)
	{
		if (jolt_vehicle.vehicle == nullptr)
		{
			ERR_FAIL_MSG("JoltVehicle has a null constraint, unable to initialize collision tester!");
			return;
		}

		JoltBody3D *jolt_body = jolt_vehicle.get_jolt_body();
		JoltSpace3D *jolt_space = jolt_body->get_space();
		JPH::ObjectLayer layer = jolt_space->map_to_object_layer(
				JoltBroadPhaseLayer::BODY_DYNAMIC,
				jolt_body->get_collision_layer(),
				jolt_body->get_collision_mask()
		);

		/*
		JPH::WheeledVehicleControllerSettings settings;
		JPH::WheeledVehicleController *test = new JPH::WheeledVehicleController(settings, *jolt_vehicle.vehicle);
		jolt_vehicle.controller->set_controller((JPH::VehicleController*) test);
		*/

		switch (jolt_vehicle.collision_mode)
		{
			case WHEEL_COLLISION_RAYCAST:
				{
					jolt_vehicle.tester = new JPH::VehicleCollisionTesterRay(layer);
					break;
				}
			case WHEEL_COLLISION_SPHERE:
				{
					// jolt_vehicle.tester = new JPH::VehicleCollisionTesterCastSphere(layer, (float) jolt_vehicle.sphere_test_radius);
					// break;
				}
			case WHEEL_COLLISION_CYLINDER:
				{
					// jolt_vehicle.tester = new JPH::VehicleCollisionTesterCastCylinder(layer);
					// break;
				}
			default:
				{
					ERR_FAIL_MSG("JoltVehicle got unknown wheel collision mode!");
					return;
				}
		}

		jolt_vehicle.vehicle->SetVehicleCollisionTester(jolt_vehicle.tester);
	}
};

VARIANT_ENUM_CAST(JoltVehicle::WheelCollisionMode);

