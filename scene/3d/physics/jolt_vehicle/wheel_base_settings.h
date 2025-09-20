#pragma once

#include "core/io/resource.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/Wheel.h"

// class WheelSettings;

class WheelBaseSettings : public Resource {
	GDCLASS(WheelBaseSettings, Resource);

	JPH::WheelSettings settings;

	// friend class WheelSettings;

public:
	enum SpringMode
	{
		SPRING_MODE_FREQUENCY,
		SPRING_MODE_STIFFNESS,
	};

	Vector3 get_position() const;
	void set_position(const Vector3 &p_position);

	Vector3 get_suspension_force_point() const;
	void set_suspension_force_point(const Vector3 &p_force_point);

	Vector3 get_suspension_direction() const;
	void set_suspension_direction(const Vector3 &p_direction);

	Vector3 get_steering_axis() const;
	void set_steering_axis(const Vector3 &p_axis);

	Vector3 get_up() const;
	void set_up(const Vector3 &p_up);

	Vector3 get_forward() const;
	void set_forward(const Vector3 &p_forward);

	float get_suspension_max_length() const;
	void set_suspension_max_length(float p_max_length);

	float get_suspension_min_length() const;
	void set_suspension_min_length(float p_min_length);

	float get_suspension_preload_length() const;
	void set_suspension_preload_length(float p_preload_length);

	SpringMode get_spring_mode() const;
	void set_spring_mode(SpringMode p_mode);

	float get_spring_frequency() const;
	void set_spring_frequency(float p_frequency);

	float get_spring_stiffness() const;
	void set_spring_stiffness(float p_stiffness);

	float get_spring_damping() const;
	void set_spring_damping(float p_damping);

	float get_radius() const;
	void set_radius(float p_radius);

	float get_width() const;
	void set_width(float p_width);

	bool get_enable_suspension_force_point() const;
	void set_enable_suspension_force_point(bool p_enable);
protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;
};

VARIANT_ENUM_CAST(WheelBaseSettings::SpringMode);

