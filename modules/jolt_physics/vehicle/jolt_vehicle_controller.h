#pragma once

#include "jolt_vehicle_differential_settings.h"
#include "jolt_vehicle_engine_settings.h"
#include "jolt_vehicle_transmission_settings.h"

#include "core/config/engine.h"
#include "core/io/resource.h"
#include "core/object/ref_counted.h"
#include "core/variant/dictionary.h"
#include "core/variant/typed_array.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

class JoltVehicle;
class JoltVehicleSettings;

class VehicleControllerSettings : public Resource {
	GDCLASS(VehicleControllerSettings, Resource);

public:
	virtual JPH::VehicleControllerSettings *get_settings() const = 0;
};

class WheeledVehicleControllerSettings : public VehicleControllerSettings {
	GDCLASS(WheeledVehicleControllerSettings, VehicleControllerSettings);

private:
	JPH::Ref<JPH::WheeledVehicleControllerSettings> settings;

	Ref<VehicleEngineSettings> engine;
	Ref<VehicleTransmissionSettings> transmission;
	LocalVector<Ref<VehicleDifferentialSettings>> differential_list;

	void _apply_engine();
	void _apply_transmission();
	void _apply_differential_list();

protected:
	static void _bind_methods();

public:
	virtual JPH::VehicleControllerSettings *get_settings() const override { return settings.GetPtr(); }

	TypedArray<VehicleDifferentialSettings> get_differential_list() const;
	void set_differential_list(const TypedArray<VehicleDifferentialSettings> &p_differential_list);

	real_t get_differential_slip_ratio() const;
	void set_differential_slip_ratio(real_t p_differential_slip_ratio);
	bool is_open_differential() const;

	Ref<VehicleEngineSettings> get_engine_settings() const { return engine; }
	void set_engine_settings(const Ref<VehicleEngineSettings> &p_engine_settings);

	Ref<VehicleTransmissionSettings> get_transmission_settings() const { return transmission; }
	void set_transmission_settings(const Ref<VehicleTransmissionSettings> &p_transmission_settings);
};

class VehicleController : public RefCounted {
	GDCLASS(VehicleController, RefCounted);

protected:
	static void _bind_methods();

public:
	virtual void set_controller(JPH::VehicleController *controller) = 0;

	virtual float get_speed_kmh() const = 0;
	float get_speed_mph() const { return get_speed_kmh() / 1.609344; }
};

class WheeledVehicleController : public VehicleController {
	GDCLASS(WheeledVehicleController, VehicleController);

public:
	enum WheelData {
		WHEEL_ANGULAR_VELOCITY,
		WHEEL_LATERAL_SLIP,
		WHEEL_LONGITUDINAL_SLIP,
		WHEEL_LATERAL_LAMBDA,
		WHEEL_LONGITUDITAL_LAMBDA,
		WHEEL_ROTATION_ANGLE,
		WHEEL_STEER_ANGLE,
	};

private:
	JPH::WheeledVehicleController *jolt_controller = nullptr;

protected:
	static void _bind_methods();

public:
	virtual void set_controller(JPH::VehicleController *controller) override {
		jolt_controller = (JPH::WheeledVehicleController *)controller;
	}

	virtual float get_speed_kmh() const override;

	void set_driver_input(float p_forward, float p_right, float p_brake, float p_hand_brake) {
		jolt_controller->SetDriverInput(p_forward, p_right, p_brake, p_hand_brake);
	}

	int get_current_gear() const {
		return jolt_controller->GetTransmission().GetCurrentGear();
	}

	float get_current_rpm() const {
		return jolt_controller->GetEngine().GetCurrentRPM();
	}

	float get_wheel_data(int p_index, WheelData p_data_type) const;
	bool is_wheel_contact(int p_index) const;
};

VARIANT_ENUM_CAST(WheeledVehicleController::WheelData);
