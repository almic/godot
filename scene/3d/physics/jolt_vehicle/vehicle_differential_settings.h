#pragma once

#include "core/io/resource.h"

#include <limits>
#include <cmath>

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/VehicleDifferential.h"


class WheeledVehicleControllerSettings;

class VehicleDifferentialSettings : public Resource
{
	GDCLASS(VehicleDifferentialSettings, Resource);

protected:
	JPH::VehicleDifferentialSettings settings;

	friend class WheeledVehicleControllerSettings;
public:

	int get_left_wheel_index() const { return settings.mLeftWheel; }
	void set_left_wheel_index(int p_index) { settings.mLeftWheel = p_index; }

	int get_right_wheel_index() const { return settings.mRightWheel; }
	void set_right_wheel_index(int p_index) { settings.mRightWheel = p_index; }

	real_t get_differential_ratio() const { return (real_t) settings.mDifferentialRatio; }
	void set_differential_ratio(real_t p_differential_ratio) { settings.mDifferentialRatio = (float) p_differential_ratio; }

	real_t get_split() const { return (real_t) settings.mLeftRightSplit; }
	void set_split(real_t p_left_right_split) { settings.mLeftRightSplit = (float) p_left_right_split; }

	bool is_open() const { return settings.mLimitedSlipRatio == std::numeric_limits<float>::max(); }
	void set_open(bool p_is_open)
	{
		if (p_is_open)
		{
			settings.mLimitedSlipRatio = std::numeric_limits<float>::max();
		}
		else
		{
			settings.mLimitedSlipRatio = (float) slip_ratio;
		}

		notify_property_list_changed();
	}

	real_t get_slip_ratio() const
	{
		if (is_open())
		{
			return slip_ratio;
		}
		else
		{
			return (real_t) settings.mLimitedSlipRatio;
		}
	}
	void set_slip_ratio(real_t p_slip_ratio)
	{
		if (std::isinf(p_slip_ratio) || p_slip_ratio == std::numeric_limits<real_t>::max())
		{
			set_open(true);
		}
		else
		{
			slip_ratio = p_slip_ratio;
			settings.mLimitedSlipRatio = (float) p_slip_ratio;
			notify_property_list_changed();
		}
	}

	real_t get_torque_ratio() const { return (real_t) settings.mEngineTorqueRatio; }
	void set_torque_ratio(real_t p_torque_ratio) { settings.mEngineTorqueRatio = (float) p_torque_ratio; }

protected:
	real_t slip_ratio = settings.mLimitedSlipRatio;

	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;
};

