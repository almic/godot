#pragma once

#include "core/io/resource.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>

class VehicleSettings;

class AntiRollBarSettings : public Resource {
	GDCLASS(AntiRollBarSettings, Resource);

protected:
	JPH::VehicleAntiRollBar settings;

	friend class VehicleSettings;

public:
	int get_left_wheel_index() const { return settings.mLeftWheel; }
	void set_left_wheel_index(int p_left_index) { settings.mLeftWheel = p_left_index; }

	int get_right_wheel_index() const { return settings.mRightWheel; }
	void set_right_wheel_index(int p_right_index) { settings.mRightWheel = p_right_index; }

	float get_stiffness() const { return settings.mStiffness; }
	void set_stiffness(float p_stiffness) { settings.mStiffness = p_stiffness; }

protected:
	static void _bind_methods();
};
