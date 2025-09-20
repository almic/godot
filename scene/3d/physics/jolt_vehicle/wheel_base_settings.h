#pragma once

#include "core/io/resource.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "Jolt/Physics/Vehicle/Wheel.h"
#include "Jolt/Physics/Constraints/SpringSettings.h"


class WheelBaseSettings : public Resource {
	GDCLASS(WheelBaseSettings, Resource);

protected:
	JPH::WheelSettings settings;

public:
	enum SpringMode
	{
		SPRING_MODE_FREQUENCY,
		SPRING_MODE_STIFFNESS,
	};

	Vector3 get_position() const { return to_godot(settings.mPosition); }
	void set_position(const Vector3 &p_position) { settings.mPosition = to_jolt(p_position); }

	Vector3 get_suspension_force_point() const { return to_godot(settings.mSuspensionForcePoint); }
	void set_suspension_force_point(const Vector3 &p_force_point) { settings.mSuspensionForcePoint = to_jolt(p_force_point); }

	Vector3 get_suspension_direction() const { return to_godot(settings.mSuspensionDirection); }
	void set_suspension_direction(const Vector3 &p_direction) { settings.mSuspensionDirection = to_jolt(p_direction.normalized()); }

	Vector3 get_steering_axis() const { return to_godot(settings.mSteeringAxis); }
	void set_steering_axis(const Vector3 &p_axis) { settings.mSteeringAxis = to_jolt(p_axis.normalized()); }

	Vector3 get_up() const { return to_godot(settings.mWheelUp); }
	void set_up(const Vector3 &p_up) { settings.mWheelUp = to_jolt(p_up.normalized()); }

	Vector3 get_forward() const { return to_godot(settings.mWheelForward); }
	void set_forward(const Vector3 &p_forward) { settings.mWheelForward = to_jolt(p_forward.normalized()); }

	real_t get_suspension_max_length() const { return (real_t) settings.mSuspensionMaxLength; }
	void set_suspension_max_length(real_t p_max_length) { settings.mSuspensionMaxLength = (float) p_max_length; }

	real_t get_suspension_min_length() const { return (real_t) settings.mSuspensionMinLength; }
	void set_suspension_min_length(real_t p_min_length) { settings.mSuspensionMinLength = (float) p_min_length; }

	real_t get_suspension_preload_length() const { return (real_t) settings.mSuspensionPreloadLength; }
	void set_suspension_preload_length(real_t p_preload_length) { settings.mSuspensionPreloadLength = (real_t) p_preload_length; }

	SpringMode get_spring_mode() const { return (SpringMode) settings.mSuspensionSpring.mMode; }
	void set_spring_mode(SpringMode p_mode)
	{
		settings.mSuspensionSpring.mMode = (JPH::ESpringMode) p_mode;
		notify_property_list_changed();
	}

	real_t get_spring_frequency() const
	{
		ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, 0, "SpringMode is not based on frequency, use get_spring_stiffness() instead!");
		return (real_t) settings.mSuspensionSpring.mFrequency;
	}
	void set_spring_frequency(real_t p_frequency)
	{
		ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, "SpringMode is not based on frequency, change the mode to frequency, or use set_spring_stiffness() instead!");
		settings.mSuspensionSpring.mFrequency = (float) p_frequency;
	}

	real_t get_spring_stiffness() const
	{
		ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, 0, "SpringMode is not based on stiffness, use get_spring_frequency() instead!");
		return settings.mSuspensionSpring.mStiffness * 0.001;
	}
	void set_spring_stiffness(real_t p_stiffness)
	{
		ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, "SpringMode is not based on stiffness, change the mode to stiffness, or use set_spring_frequency() instead!");
		settings.mSuspensionSpring.mStiffness = (float) (p_stiffness * 1000.0);
	}

	real_t get_spring_damping() const { return settings.mSuspensionSpring.mDamping; }
	void set_spring_damping(real_t p_damping) { settings.mSuspensionSpring.mDamping = (float) p_damping; }

	real_t get_radius() const { return (real_t) settings.mRadius; }
	void set_radius(real_t p_radius) { settings.mRadius = (float) p_radius; }

	real_t get_width() const { return (real_t) settings.mWidth; }
	void set_width(real_t p_width) { settings.mWidth = (float) p_width; }

	bool get_enable_suspension_force_point() const { return settings.mEnableSuspensionForcePoint; }
	void set_enable_suspension_force_point(bool p_enable)
	{
		settings.mEnableSuspensionForcePoint = p_enable;
		notify_property_list_changed();
	}

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;
};

VARIANT_ENUM_CAST(WheelBaseSettings::SpringMode);

