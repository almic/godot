#pragma once

#include "core/config/engine.h"
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
	virtual float get_speed_kmh() const { return 0.0f; }
	float get_speed_mph() const { return get_speed_kmh() / 1.609344; }

protected:
	static void _bind_methods();
};

class WheeledVehicleController : public VehicleController {
	GDCLASS(WheeledVehicleController, VehicleController);

protected:
	JPH::WheeledVehicleController *jolt_controller = nullptr;

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

	virtual float get_speed_kmh() const override {
		float kmh = 0.0f;

		const JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
		const int size = wheels.size();
		int on_ground = 0;

		for (int i = 0; i < size; i++) {
			JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheels[i]);
			if (!wheel->HasContact()) {
				continue;
			}

			float slip = wheel->mLongitudinalSlip;
			if (slip > 0.5f) {
				continue;
			}

			++on_ground;
			float ms = wheel->GetSettings()->mRadius * std::abs(wheel->GetAngularVelocity());
			if (slip > 0.0) {
				ms *= 1.0f - slip;
			}
			kmh += ms;
		}

		if (on_ground > 0) {
			kmh *= 3.6f;
			kmh /= on_ground;
		}

		if (kmh < 0.1f) {
			kmh = 0.0f;
		}

		return kmh;
	}

protected:
	static void _bind_methods();
};
