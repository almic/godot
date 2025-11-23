#pragma once

#include "core/object/ref_counted.h"
#include "core/variant/dictionary.h"
#include "core/variant/typed_array.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"

class JoltVehicle;

class VehicleController : public RefCounted {
	GDCLASS(VehicleController, RefCounted);

public:
	virtual void set_controller(JPH::VehicleController *controller) {}
};

class WheeledVehicleController : public VehicleController {
	GDCLASS(WheeledVehicleController, VehicleController);

protected:
	JPH::WheeledVehicleController *jolt_controller;

	friend class JoltVehicle;

public:
	virtual void set_controller(JPH::VehicleController *controller) override {
		jolt_controller = (JPH::WheeledVehicleController *)controller;
	}

	void set_driver_input(float p_forward, float p_right, float p_brake, float p_hand_brake) {
		jolt_controller->SetDriverInput(p_forward, p_right, p_brake, p_hand_brake);
	}

	int get_current_gear() const {
		return jolt_controller->GetTransmission().GetCurrentGear();
	}

	float get_current_rpm() const {
		return jolt_controller->GetEngine().GetCurrentRPM();
	}

	TypedArray<Dictionary> get_wheel_info() const {
		JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
		TypedArray<Dictionary> results;
		results.resize(wheels.size());
		int i = 0;
		for (JPH::Wheel *wheel_base : wheels) {
			JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheel_base);
			Dictionary d;

			d["has_contact"] = wheel->HasContact();
			d["slip_long"] = wheel->mLongitudinalSlip;
			d["slip_lat"] = wheel->mLateralSlip;
			d["steer_angle"] = wheel->GetSteerAngle();
			d["angular_velocity"] = wheel->GetAngularVelocity();
			d["rotation_angle"] = wheel->GetRotationAngle();

			results[i] = d;

			i++;
		}
		return results;
	}

	bool is_any_driven_wheel_slipping() const {
		return jolt_controller->IsAnyDrivenWheelSlipping();
	}

protected:
	static void _bind_methods();
};
