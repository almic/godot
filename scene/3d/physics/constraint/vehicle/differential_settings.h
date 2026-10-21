#pragma once

#include "core/io/resource.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleDifferential.h>

class WheeledVehicleControllerSettings;

class VehicleDifferentialSettings : public Resource {
	GDCLASS(VehicleDifferentialSettings, Resource);

protected:
	JPH::VehicleDifferentialSettings settings;

	friend class WheeledVehicleControllerSettings;

public:
	int get_left_wheel_index() const { return settings.mLeftWheel; }
	void set_left_wheel_index(int p_index) { settings.mLeftWheel = p_index; }

	int get_right_wheel_index() const { return settings.mRightWheel; }
	void set_right_wheel_index(int p_index) { settings.mRightWheel = p_index; }

	float get_differential_ratio() const { return settings.mDifferentialRatio; }
	void set_differential_ratio(float p_differential_ratio) { settings.mDifferentialRatio = p_differential_ratio; }

	float get_split() const { return settings.mLeftRightSplit; }
	void set_split(float p_left_right_split) { settings.mLeftRightSplit = p_left_right_split; }

	bool is_open() const;
	void set_open(bool p_is_open);

	float get_slip_ratio() const;
	void set_slip_ratio(float p_slip_ratio);

	float get_torque_ratio() const { return settings.mEngineTorqueRatio; }
	void set_torque_ratio(float p_torque_ratio) { settings.mEngineTorqueRatio = p_torque_ratio; }

protected:
	float slip_ratio = settings.mLimitedSlipRatio;

	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;
};
