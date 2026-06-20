#pragma once

#include "core/io/resource.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleDifferential.h>

class VehicleDifferentialSettings : public Resource {
	GDCLASS(VehicleDifferentialSettings, Resource);

private:
	JPH::VehicleDifferentialSettings settings;

protected:
	static void _bind_methods();

public:
	const JPH::VehicleDifferentialSettings &get_settings() const { return settings; }

	int get_left_wheel_index() const { return settings.mLeftWheel; }
	void set_left_wheel_index(int p_index) { settings.mLeftWheel = p_index; }

	int get_right_wheel_index() const { return settings.mRightWheel; }
	void set_right_wheel_index(int p_index) { settings.mRightWheel = p_index; }

	real_t get_differential_ratio() const { return (real_t)settings.mDifferentialRatio; }
	void set_differential_ratio(real_t p_differential_ratio) { settings.mDifferentialRatio = (float)p_differential_ratio; }

	real_t get_wheel_split() const { return (real_t)settings.mLeftRightSplit; }
	void set_wheel_split(real_t p_left_right_split) { settings.mLeftRightSplit = (float)p_left_right_split; }

	real_t get_slip_ratio() const;
	void set_slip_ratio(real_t p_slip_ratio);
	bool is_slip_open() const;

	real_t get_torque_ratio() const { return (real_t)settings.mEngineTorqueRatio; }
	void set_torque_ratio(real_t p_torque_ratio) { settings.mEngineTorqueRatio = (float)p_torque_ratio; }
};
