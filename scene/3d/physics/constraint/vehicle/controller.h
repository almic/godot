#pragma once

#include "differential_settings.h"
#include "engine_settings.h"
#include "transmission_settings.h"

#include "core/io/resource.h"
#include "core/object/ref_counted.h"
#include "core/variant/dictionary.h"
#include "core/variant/typed_array.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

class Vehicle;
class VehicleSettings;

class VehicleController : public RefCounted {
	GDCLASS(VehicleController, RefCounted);

public:
	virtual void set_controller(JPH::VehicleController *controller) {}
	virtual float get_speed_kmh() const { return 0.0f; }
	float get_speed_mph() const { return get_speed_kmh() / 1.609344; }

	virtual float get_suspension_length(int p_wheel_index) const = 0;

protected:
	static void _bind_methods();
};

class WheeledVehicleController : public VehicleController {
	GDCLASS(WheeledVehicleController, VehicleController);

protected:
	JPH::WheeledVehicleController *jolt_controller = nullptr;

	friend class Vehicle;

public:
	virtual void set_controller(JPH::VehicleController *controller) override {
		jolt_controller = (JPH::WheeledVehicleController *)controller;
		notify_property_list_changed();
	}

	virtual float get_suspension_length(int p_wheel_index) const override {
		return jolt_controller->GetConstraint().GetWheel(p_wheel_index)->GetSuspensionLength();
	}

	void set_driver_input(float p_forward, float p_right, float p_brake, float p_hand_brake) {
		jolt_controller->SetDriverInput(p_forward, p_right, p_brake, p_hand_brake);
	}

	float get_forward_input() const {
		return jolt_controller->GetForwardInput();
	}

	float get_right_input() const {
		return jolt_controller->GetRightInput();
	}

	float get_brake_input() const {
		return jolt_controller->GetBrakeInput();
	}

	float get_handbrake_input() const {
		return jolt_controller->GetHandBrakeInput();
	}

	int get_current_gear() const {
		return jolt_controller->GetTransmission().GetCurrentGear();
	}

	float get_current_rpm() const {
		return jolt_controller->GetEngine().GetCurrentRPM();
	}

	TypedArray<Dictionary> get_wheel_info() const;

	bool is_virtual_steer_enabled() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->IsVirtualSteerEnabled();
		}
		return false;
	}
	void set_virtual_steer_enabled(float p_enabled) {
		if (jolt_controller) {
			jolt_controller->SetVirtualSteerEnabled(p_enabled);
		}
	}

	float get_virtual_steer() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->GetVirtualSteer();
		}
		return 0.0f;
	}
	void set_virtual_steer(float p_virtual_steer) {
		if (jolt_controller != nullptr) {
			jolt_controller->SetVirtualSteer(p_virtual_steer);
		}
	}

	float get_virtual_steer_velocity() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->GetVirtualSteerVelocity();
		}
		return 0.0f;
	}
	void set_virtual_steer_velocity(float p_virtual_steer_velocity) {
		if (jolt_controller != nullptr) {
			jolt_controller->SetVirtualSteerVelocity(p_virtual_steer_velocity);
		}
	}

	float get_virtual_steer_feedback() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->GetVirtualSteerFeedback();
		}
		return 0.0f;
	}
	void set_virtual_steer_feedback(float p_feedback) {
		if (jolt_controller != nullptr) {
			jolt_controller->SetVirtualSteerFeedback(p_feedback);
		}
	}

	float get_virtual_steer_acceleration() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->GetVirtualSteerAcceleration();
		}
		return 0.0f;
	}
	void set_virtual_steer_acceleration(float p_acceleration) {
		if (jolt_controller != nullptr) {
			jolt_controller->SetVirtualSteerAcceleration(p_acceleration);
		}
	}

	float get_virtual_steer_max_speed() const {
		if (jolt_controller != nullptr) {
			return jolt_controller->GetVirtualSteerMaxSpeed();
		}
		return 0.0f;
	}
	void set_virtual_steer_max_speed(float p_max_speed) {
		if (jolt_controller != nullptr) {
			jolt_controller->SetVirtualSteerMaxSpeed(p_max_speed);
		}
	}

	bool is_any_driven_wheel_slipping() const {
		return jolt_controller->IsAnyDrivenWheelSlipping();
	}

	virtual float get_speed_kmh() const override;

protected:
	static void _bind_methods();
};

class VehicleControllerSettings : public Resource {
	GDCLASS(VehicleControllerSettings, Resource);

protected:
	virtual JPH::VehicleControllerSettings *get_settings() { return nullptr; }

	friend class VehicleSettings;
};

class WheeledVehicleControllerSettings : public VehicleControllerSettings {
	GDCLASS(WheeledVehicleControllerSettings, VehicleControllerSettings);

protected:
	JPH::WheeledVehicleControllerSettings settings;

	virtual JPH::VehicleControllerSettings *get_settings() override { return &settings; }

public:
	WheeledVehicleControllerSettings() {
		// Settings is not actually alloc'd, mark as embedded to prevent release from doing anything
		settings.SetEmbedded();
		settings.AddRef();
	}

	~WheeledVehicleControllerSettings() {
		settings.Release();
	}

	Ref<VehicleEngineSettings> get_engine_settings() const { return engine; }
	void set_engine_settings(const Ref<VehicleEngineSettings> &p_engine_settings);

	Ref<VehicleTransmissionSettings> get_transmission_settings() const { return transmission; }
	void set_transmission_settings(const Ref<VehicleTransmissionSettings> &p_transmission_settings);

	TypedArray<VehicleDifferentialSettings> get_differentials() const;
	void set_differentials(const TypedArray<VehicleDifferentialSettings> &p_differentials);

	bool is_open_differential() const;
	void set_open_differential(bool p_is_open);

	float get_differential_slip_ratio() const;
	void set_differential_slip_ratio(float p_differential_slip_ratio);

	bool is_virtual_steer_enabled() const { return settings.mEnableVirtualSteer; }
	void set_virtual_steer_enabled(float p_enabled) { settings.mEnableVirtualSteer = p_enabled; }

	float get_virtual_steer_feedback() const { return settings.mVirtualSteerFeedback; }
	void set_virtual_steer_feedback(float p_steer_feedback) { settings.mVirtualSteerFeedback = p_steer_feedback; }

	float get_virtual_steer_acceleration() const { return settings.mVirtualSteerAcceleration; }
	void set_virtual_steer_acceleration(float p_acceleration) { settings.mVirtualSteerAcceleration = p_acceleration; }

	float get_virtual_steer_max_speed() const { return settings.mVirtualSteerMaxSpeed; }
	void set_virtual_steer_max_speed(float p_max_speed) { settings.mVirtualSteerMaxSpeed = p_max_speed; }

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;

	Ref<VehicleEngineSettings> engine;
	Ref<VehicleTransmissionSettings> transmission;
	LocalVector<Ref<VehicleDifferentialSettings>> differentials;

	float differential_slip_ratio = settings.mDifferentialLimitedSlipRatio;

	void _apply_engine();
	void _apply_transmission();
	void _apply_differentials();
};
