#pragma once

#include "core/io/resource.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleTransmission.h>

class WheeledVehicleControllerSettings;

class VehicleTransmissionSettings : public Resource {
	GDCLASS(VehicleTransmissionSettings, Resource);

protected:
	JPH::VehicleTransmissionSettings settings;

	friend class WheeledVehicleControllerSettings;

public:
	enum TransmissionMode {
		TRANSMISSION_MODE_AUTO,
		TRANSMISSION_MODE_MANUAL,
	};

	TransmissionMode get_transmission_mode() const { return TransmissionMode(settings.mMode); }
	void set_transmission_mode(TransmissionMode p_mode);

	PackedFloat32Array get_gear_ratios() const { return gear_ratio_list; }
	void set_gear_ratios(const PackedFloat32Array &p_gear_list);

	PackedFloat32Array get_reverse_gear_ratios() const { return reverse_gear_ratio_list; }
	void set_reverse_gear_ratios(const PackedFloat32Array &p_reverse_gear_list);

	float get_switch_time() const { return settings.mSwitchTime; }
	void set_switch_time(float p_switch_time) { settings.mSwitchTime = p_switch_time; }

	float get_clutch_release_time() const { return settings.mClutchReleaseTime; }
	void set_clutch_release_time(float p_release_time) { settings.mClutchReleaseTime = p_release_time; }

	float get_switch_latency() const { return settings.mSwitchLatency; }
	void set_switch_latency(float p_switch_latency) { settings.mSwitchLatency = p_switch_latency; }

	float get_shift_up_rpm() const { return settings.mShiftUpRPM; }
	void set_shift_up_rpm(float p_shift_up_rpm) { settings.mShiftUpRPM = p_shift_up_rpm; }

	float get_shift_down_rpm() const { return settings.mShiftDownRPM; }
	void set_shift_down_rpm(float p_shift_down_rpm) { settings.mShiftDownRPM = p_shift_down_rpm; }

	float get_clutch_strength() const { return settings.mClutchStrength; }
	void set_clutch_strength(float p_clutch_strength) { settings.mClutchStrength = p_clutch_strength; }

protected:
	PackedFloat32Array gear_ratio_list;
	PackedFloat32Array reverse_gear_ratio_list;

	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;
};

VARIANT_ENUM_CAST(VehicleTransmissionSettings::TransmissionMode);
