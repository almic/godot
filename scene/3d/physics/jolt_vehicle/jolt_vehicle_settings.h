#pragma once

#include "core/io/resource.h"
#include "core/variant/typed_array.h"

#include "anti_roll_bar_settings.h"
#include "vehicle_controller_settings.h"
#include "wheel_settings.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

#include "Jolt/Physics/Vehicle/VehicleConstraint.h"

class JoltVehicle;

class JoltVehicleSettings : public Resource {
	GDCLASS(JoltVehicleSettings, Resource);

protected:
	JPH::VehicleConstraintSettings settings;

	friend class JoltVehicle;

public:
	Vector3 get_up() const { return to_godot(settings.mUp); }
	void set_up(const Vector3 &p_up) { settings.mUp = to_jolt(p_up.normalized()); }

	Vector3 get_forward() const { return to_godot(settings.mForward); }
	void set_forward(const Vector3 &p_forward) { settings.mForward = to_jolt(p_forward.normalized()); }

	bool is_pitch_roll_limited() const { return limit_pitch_roll; }
	void set_pitch_roll_limited(bool p_is_limited) {
		limit_pitch_roll = p_is_limited;

		if (limit_pitch_roll) {
			settings.mMaxPitchRollAngle = (float)max_pitch_roll;
		} else {
			settings.mMaxPitchRollAngle = JPH::JPH_PI;
		}

		notify_property_list_changed();
	}

	real_t get_max_pitch_roll() const { return max_pitch_roll; }
	void set_max_pitch_roll(real_t p_max_pitch_roll) {
		max_pitch_roll = CLAMP(p_max_pitch_roll, 0.0, JPH::JPH_PI);

		if (is_pitch_roll_limited()) {
			settings.mMaxPitchRollAngle = max_pitch_roll;
		}
	}

	TypedArray<WheelBaseSettings> get_wheel_settings() const {
		TypedArray<WheelBaseSettings> result;

		for (const auto &w : wheels) {
			result.push_back(w);
		}

		return result;
	}
	void set_wheel_settings(const TypedArray<WheelBaseSettings> &p_wheel_settings) {
		for (const auto &w : wheels) {
			Ref<WheelBaseSettings> wheel = w;
			if (wheel.is_valid()) {
				wheel->disconnect_changed(callable_mp((Resource *)this, &JoltVehicleSettings::emit_changed));
			}
		}

		wheels.clear();
		settings.mWheels.clear();

		for (const auto &w : p_wheel_settings) {
			Ref<WheelBaseSettings> wheel = w;
			wheels.push_back(wheel);
			if (wheel.is_valid()) {
				wheel->connect_changed(callable_mp((Resource *)this, &JoltVehicleSettings::emit_changed));
				settings.mWheels.push_back(wheel->get_settings());
			}
		}
	}

	TypedArray<AntiRollBarSettings> get_anti_roll_bar_settings() const {
		TypedArray<AntiRollBarSettings> result;

		for (const auto &a : anti_roll_bars) {
			result.push_back(a);
		}

		return result;
	}
	void set_anti_roll_bar_settings(const TypedArray<AntiRollBarSettings> &p_anti_roll_bar_settings) {
		anti_roll_bars.clear();
		settings.mAntiRollBars.clear();

		for (const auto &a : p_anti_roll_bar_settings) {
			Ref<AntiRollBarSettings> anti_roll_bar = a;
			anti_roll_bars.push_back(anti_roll_bar);
			if (anti_roll_bar.is_valid()) {
				settings.mAntiRollBars.push_back(anti_roll_bar->settings);
			}
		}
	}

	Ref<VehicleControllerSettings> get_controller_settings() const { return controller; }
	void set_controller_settings(const Ref<VehicleControllerSettings> &p_controller) {
		controller = p_controller;
		if (controller.is_valid()) {
			settings.mController = controller->get_settings();
		}
	}

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	bool limit_pitch_roll = false;
	real_t max_pitch_roll = 0.0;
	LocalVector<Ref<WheelBaseSettings>> wheels;
	LocalVector<Ref<AntiRollBarSettings>> anti_roll_bars;
	Ref<VehicleControllerSettings> controller;
};
