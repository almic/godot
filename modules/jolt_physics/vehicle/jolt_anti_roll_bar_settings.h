#pragma once

#include "core/io/resource.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>

class AntiRollBarSettings : public Resource {
	GDCLASS(AntiRollBarSettings, Resource);

private:
	JPH::VehicleAntiRollBar settings;

public:
	const JPH::VehicleAntiRollBar &get_settings() const { return settings; }

	int get_left_wheel_index() const { return settings.mLeftWheel; }
	void set_left_wheel_index(int p_left_index) { settings.mLeftWheel = p_left_index; }

	int get_right_wheel_index() const { return settings.mRightWheel; }
	void set_right_wheel_index(int p_right_index) { settings.mRightWheel = p_right_index; }

	real_t get_stiffness() const { return settings.mStiffness; }
	void set_stiffness(real_t p_stiffness) { settings.mStiffness = p_stiffness; }

protected:
	static void _bind_methods();
};
