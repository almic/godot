#pragma once

#include "core/io/resource.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "Jolt/Physics/Constraints/SpringSettings.h"
#include "Jolt/Physics/Vehicle/Wheel.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"


class JoltVehicleSettings;

class WheelBaseSettings : public Resource {
	GDCLASS(WheelBaseSettings, Resource);

protected:
	virtual JPH::WheelSettings& get_settings() = 0;
	virtual const JPH::WheelSettings& get_settings() const = 0;

	friend class JoltVehicleSettings;

public:
	enum SpringMode
	{
		SPRING_MODE_FREQUENCY,
		SPRING_MODE_STIFFNESS,
	};

	Vector3 get_position() const { return to_godot(get_settings().mPosition); }
	void set_position(const Vector3 &p_position) { get_settings().mPosition = to_jolt(p_position); }

	Vector3 get_suspension_force_point() const { return to_godot(get_settings().mSuspensionForcePoint); }
	void set_suspension_force_point(const Vector3 &p_force_point) { get_settings().mSuspensionForcePoint = to_jolt(p_force_point); }

	Vector3 get_suspension_direction() const { return to_godot(get_settings().mSuspensionDirection); }
	void set_suspension_direction(const Vector3 &p_direction) { get_settings().mSuspensionDirection = to_jolt(p_direction.normalized()); }

	Vector3 get_steering_axis() const { return to_godot(get_settings().mSteeringAxis); }
	void set_steering_axis(const Vector3 &p_axis) { get_settings().mSteeringAxis = to_jolt(p_axis.normalized()); }

	Vector3 get_up() const { return to_godot(get_settings().mWheelUp); }
	void set_up(const Vector3 &p_up) { get_settings().mWheelUp = to_jolt(p_up.normalized()); }

	Vector3 get_forward() const { return to_godot(get_settings().mWheelForward); }
	void set_forward(const Vector3 &p_forward) { get_settings().mWheelForward = to_jolt(p_forward.normalized()); }

	real_t get_suspension_max_length() const { return (real_t) get_settings().mSuspensionMaxLength; }
	void set_suspension_max_length(real_t p_max_length) { get_settings().mSuspensionMaxLength = (float) p_max_length; }

	real_t get_suspension_min_length() const { return (real_t) get_settings().mSuspensionMinLength; }
	void set_suspension_min_length(real_t p_min_length) { get_settings().mSuspensionMinLength = (float) p_min_length; }

	real_t get_suspension_preload_length() const { return (real_t) get_settings().mSuspensionPreloadLength; }
	void set_suspension_preload_length(real_t p_preload_length) { get_settings().mSuspensionPreloadLength = (real_t) p_preload_length; }

	SpringMode get_spring_mode() const { return (SpringMode) get_settings().mSuspensionSpring.mMode; }
	void set_spring_mode(SpringMode p_mode)
	{
		get_settings().mSuspensionSpring.mMode = (JPH::ESpringMode) p_mode;
		notify_property_list_changed();
	}

	real_t get_spring_frequency() const
	{
		ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, 0, "SpringMode is not based on frequency, use get_spring_stiffness() instead!");
		return (real_t) get_settings().mSuspensionSpring.mFrequency;
	}
	void set_spring_frequency(real_t p_frequency)
	{
		ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, "SpringMode is not based on frequency, change the mode to frequency, or use set_spring_stiffness() instead!");
		get_settings().mSuspensionSpring.mFrequency = (float) p_frequency;
	}

	real_t get_spring_stiffness() const
	{
		ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, 0, "SpringMode is not based on stiffness, use get_spring_frequency() instead!");
		return get_settings().mSuspensionSpring.mStiffness * 0.001;
	}
	void set_spring_stiffness(real_t p_stiffness)
	{
		ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, "SpringMode is not based on stiffness, change the mode to stiffness, or use set_spring_frequency() instead!");
		get_settings().mSuspensionSpring.mStiffness = (float) (p_stiffness * 1000.0);
	}

	real_t get_spring_damping() const { return get_settings().mSuspensionSpring.mDamping; }
	void set_spring_damping(real_t p_damping) { get_settings().mSuspensionSpring.mDamping = (float) p_damping; }

	real_t get_radius() const { return (real_t) get_settings().mRadius; }
	void set_radius(real_t p_radius) { get_settings().mRadius = (float) p_radius; }

	real_t get_width() const { return (real_t) get_settings().mWidth; }
	void set_width(real_t p_width) { get_settings().mWidth = (float) p_width; }

	bool get_enable_suspension_force_point() const { return get_settings().mEnableSuspensionForcePoint; }
	void set_enable_suspension_force_point(bool p_enable)
	{
		get_settings().mEnableSuspensionForcePoint = p_enable;
		notify_property_list_changed();
	}

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;
};

VARIANT_ENUM_CAST(WheelBaseSettings::SpringMode);

class WheelSettings : public WheelBaseSettings
{
protected:
	JPH::WheelSettingsWV settings;

	virtual JPH::WheelSettings& get_settings() override { return settings; }
	virtual const JPH::WheelSettings& get_settings() const override { return settings; }

public:

	real_t get_inertia() const { return (real_t) settings.mInertia; }
	void set_inertia(real_t p_inertia) { settings.mInertia = (float) p_inertia; }

	real_t get_angular_damping() const { return (real_t) settings.mAngularDamping; }
	void set_angular_damping(real_t p_angular_damping) { settings.mAngularDaming = (float) p_angular_damping; }

	real_t get_max_steer_angle() const { return (real_t) settings.mMaxSteerAngle; }
	void set_max_steer_angle(real_t p_max_steer_angle) { settings.mMaxSteerAngle = (float) p_max_steer_angle; }

	real_t get_max_brake_torque() const { return (real_t) settings.mMaxBrakeTorque; }
	void set_max_brake_torque(real_t p_max_brake_torque) { settings.mMaxBrakeTorque = (float) p_max_brake_torque; }

	real_t get_max_hand_brake_torque() const { return (real_t) settings.mMaxHandBrakeTorque; }
	void set_max_hand_brake_torque(real_t p_max_hand_brake_torque) { settings.mMaxHandBrakeTorque = (float) p_max_hand_brake_torque; }

	Ref<Curve> get_lateral_friction() const { return lateral_friction; }
	void set_lateral_friction(const Ref<Curve>& p_lateral_friction)
	{
		if (lateral_friction.is_valid()) {
			lateral_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
		}

		lateral_friction = p_lateral_friction;

		if (lateral_friction.is_valid() && (!is_lateral_first_set || p_lateral_friction->get_point_count() > 0)) {
			lateral_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
			_apply_lateral_friction();
		}
		else
		{
			// Reset to default
			JPH::WheelSettingsWV defaults;
			settings.mLateralFriction = defaults.mLateralFriction;

			lateral_friction = memnew(Curve);
			to_godot(settings.mLateralFriction, *lateral_friction.ptr());
			lateral_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
		}

		is_lateral_first_set = false;
	}

	Ref<Curve> get_longitudinal_friction() const { return longitudinal_friction; }
	void set_longitudinal_friction(const Ref<Curve>& p_longitudinal_friction)
	{
		if (longitudinal_friction.is_valid()) {
			longitudinal_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
		}

		longitudinal_friction = p_longitudinal_friction;

		if (longitudinal_friction.is_valid() && (!is_longitudinal_first_set || p_longitudinal_friction->get_point_count() > 0)) {
			longitudinal_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
			_apply_longitudinal_friction();
		}
		else
		{
			// Reset to default
			JPH::WheelSettingsWV defaults;
			settings.mLongitudinalFriction = defaults.mLongitudinalFriction;

			longitudinal_friction = memnew(Curve);
			to_godot(settings.mLongitudinalFriction, *longitudinal_friction.ptr());
			longitudinal_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
		}

		is_longitudinal_first_set = false;
	}

protected:
	static void _bind_methods();

	bool is_lateral_first_set = true;
	bool is_longitudinal_first_set = true;
	Ref<Curve> lateral_friction;
	Ref<Curve> longitudinal_friction;

	void _apply_lateral_friction() {
		settings.mLateralFriction = to_jolt(*lateral_friction.ptr());
	}

	void _apply_longitudinal_friction() {
		settings.mLongitudinalFriction = to_jolt(*longitudinal_friction.ptr());
	}
};

