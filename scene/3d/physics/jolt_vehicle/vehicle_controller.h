#pragma once

#include "core/object/ref_counted.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"


class JoltVehicle;

class VehicleController : public RefCounted
{
	GDCLASS(VehicleController, Object);
public:
	virtual void set_controller(JPH::VehicleController* controller) {}
};

class WheeledVehicleController : public VehicleController
{
	GDCLASS(WheeledVehicleController, VehicleController);

protected:
	JPH::WheeledVehicleController* jolt_controller;

	friend class JoltVehicle;

public:

	virtual void set_controller(JPH::VehicleController* controller) override
	{
		jolt_controller = (JPH::WheeledVehicleController*) controller;
	}

	void set_driver_input(float p_forward, float p_right, float p_brake, float p_hand_brake)
	{
		jolt_controller->SetDriverInput(p_forward, p_right, p_brake, p_hand_brake);
	}

protected:
	static void _bind_methods();

};
