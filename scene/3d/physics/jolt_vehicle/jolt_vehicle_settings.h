#pragma once

#include "core/io/resource.h"
#include "core/variant/typed_array.h"

#include "anti_roll_bar_settings.h"
#include "vehicle_controller_settings.h"
#include "wheel_settings.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

#include "Jolt/Physics/Vehicle/VehicleConstraint.h"


class JoltVehicle;


class JoltVehicleSettings : public Resource
{
	GDCLASS(JoltVehicleSettings, Resource);

protected:
	JPH::VehicleConstraintSettings settings;

	friend class JoltVehicle;

public:

	Vector3 get_up() const { return to_godot(settings.mUp); }
	void set_up(const Vector3 &p_up) { settings.mUp = to_jolt(p_up.normalized()); }

	Vector3 get_forward() const { return to_godot(settings.mForward); }
	void set_forward(const Vector3 &p_forward) { settings.mForward = to_jolt(p_forward.normalized()); }

	bool is_pitch_roll_limited() const { return settings.mMaxPitchRollAngle < JPH::JPH_PI; }
	void set_pitch_roll_limited(bool p_is_limited)
	{
		if (p_is_limited)
		{
			settings.mMaxPitchRollAngle = (float) max_pitch_roll_angle;
		}
		else
		{
			settings.mMaxPitchRollAngle = JPH::JPH_PI;
		}

		notify_property_list_changed();
	}

	real_t get_max_pitch_roll() const
	{
		if (is_pitch_roll_limited())
		{
			return (real_t) settings.mMaxPitchRollAngle;
		}
		else
		{
			return max_pitch_roll_angle;
		}
	}
	void set_max_pitch_roll(real_t p_max_pitch_roll)
	{
		if (p_max_pitch_roll >= M_PI)
		{
			set_pitch_roll_limited(false);
		}
		else
		{
			max_pitch_roll_angle = p_max_pitch_roll;
			settings.mMaxPitchRollAngle = (float) p_max_pitch_roll;
			notify_property_list_changed();
		}
	}

	TypedArray<WheelBaseSettings> get_wheel_settings() const
	{
		TypedArray<WheelBaseSettings> result;

		for (const auto& w : wheels)
		{
			result.push_back(w);
		}

		return result;
	}
	void set_wheel_settings(const TypedArray<WheelBaseSettings> &p_wheel_settings)
	{
		wheels.clear();
		settings.mWheels.clear();

		for (const auto& w : p_wheel_settings)
		{
			Ref<WheelBaseSettings> wheel = w;
			wheels.push_back(wheel);
			settings.mWheels.push_back(wheel->get_settings());
		}
	}

	TypedArray<AntiRollBarSettings> get_anti_roll_bar_settings() const
	{
		TypedArray<AntiRollBarSettings> result;

		for (const auto& a : anti_roll_bars)
		{
			result.push_back(a);
		}

		return result;
	}
	void set_anti_roll_bar_settings(const TypedArray<AntiRollBarSettings> &p_anti_roll_bar_settings)
	{
		anti_roll_bars.clear();
		settings.mAntiRollBars.clear();

		for (const auto& a : p_anti_roll_bar_settings)
		{
			Ref<AntiRollBarSettings> anti_roll_bar = a;
			anti_roll_bars.push_back(anti_roll_bar);
			settings.mAntiRollBars.push_back(anti_roll_bar->settings);
		}
	}

	Ref<VehicleControllerSettings> get_controller_settings() const { return controller; }
	void set_controller_settings(const Ref<VehicleControllerSettings> &p_controller)
	{
		controller = p_controller;
		settings.mController = controller->get_settings();
	}

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	real_t max_pitch_roll_angle;
	LocalVector<Ref<WheelBaseSettings>> wheels;
	LocalVector<Ref<AntiRollBarSettings>> anti_roll_bars;
	Ref<VehicleControllerSettings> controller;
};

