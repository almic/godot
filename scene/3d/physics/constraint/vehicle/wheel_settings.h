#pragma once

#include "core/io/resource.h"
#include "scene/resources/curve.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

class Vehicle;
class VehicleSettings;

class WheelBaseSettings : public Resource {
	GDCLASS(WheelBaseSettings, Resource);

protected:
	virtual JPH::WheelSettings *get_settings() { return nullptr; }
	virtual const JPH::WheelSettings *get_settings() const { return nullptr; }

	friend class Vehicle;
	friend class VehicleSettings;

public:
	enum SpringMode {
		SPRING_MODE_FREQUENCY,
		SPRING_MODE_STIFFNESS,
	};

	Vector3 get_position() const;
	void set_position(const Vector3 &p_position);

	Vector3 get_suspension_force_point() const;
	void set_suspension_force_point(const Vector3 &p_force_point);

	Vector3 get_suspension_direction() const;

	float get_suspension_forward_angle() const { return suspension_forward_angle; }
	void set_suspension_forward_angle(float p_angle);

	float get_suspension_sideways_angle() const { return suspension_sideways_angle; }
	void set_suspension_sideways_angle(float p_angle);

	Vector3 get_steering_axis() const;
	Vector3 get_up() const;
	Vector3 get_forward() const;

	float get_kingpin_angle() const { return kingpin_angle; }
	void set_kingpin_angle(float p_angle);

	float get_caster_angle() const { return caster_angle; }
	void set_caster_angle(float p_angle);

	float get_camber() const { return camber; }
	void set_camber(float p_camber);

	float get_toe() const { return toe; }
	void set_toe(float p_toe);

	float get_suspension_max_length() const { return get_settings()->mSuspensionMaxLength; }
	void set_suspension_max_length(float p_max_length) {
		get_settings()->mSuspensionMaxLength = p_max_length;
		emit_changed();
	}

	float get_suspension_min_length() const { return get_settings()->mSuspensionMinLength; }
	void set_suspension_min_length(float p_min_length) {
		get_settings()->mSuspensionMinLength = p_min_length;
		emit_changed();
	}

	float get_suspension_preload_length() const { return get_settings()->mSuspensionPreloadLength; }
	void set_suspension_preload_length(float p_preload_length) {
		get_settings()->mSuspensionPreloadLength = p_preload_length;
		emit_changed();
	}

	SpringMode get_spring_mode() const { return SpringMode(get_settings()->mSuspensionSpring.mMode); }
	void set_spring_mode(SpringMode p_mode);

	float get_spring_frequency() const;
	void set_spring_frequency(float p_frequency);

	float get_spring_stiffness() const;
	void set_spring_stiffness(float p_stiffness);

	float get_spring_damping() const { return get_settings()->mSuspensionSpring.mDamping; }
	void set_spring_damping(float p_damping) { get_settings()->mSuspensionSpring.mDamping = p_damping; }

	float get_radius() const { return get_settings()->mRadius; }
	void set_radius(float p_radius) { get_settings()->mRadius = p_radius; }

	float get_width() const { return get_settings()->mWidth; }
	void set_width(float p_width) { get_settings()->mWidth = p_width; }

	bool get_enable_suspension_force_point() const { return get_settings()->mEnableSuspensionForcePoint; }
	void set_enable_suspension_force_point(bool p_enable) {
		get_settings()->mEnableSuspensionForcePoint = p_enable;
		notify_property_list_changed();
	}

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;

	float stiffness = 0.0;
	float frequency = 0.0;

	// Wheel angle parameters
	float kingpin_angle = 0.0;
	float caster_angle = 0.0;
	float camber = 0.0;
	float toe = 0.0;

	// Suspension angle parameters
	float suspension_forward_angle = 0.0;
	float suspension_sideways_angle = 0.0;
};

class WheelSettings : public WheelBaseSettings {
	GDCLASS(WheelSettings, WheelBaseSettings);

protected:
	JPH::WheelSettingsWV settings;

	virtual JPH::WheelSettings *get_settings() override { return &settings; }
	virtual const JPH::WheelSettings *get_settings() const override { return &settings; }

public:
	WheelSettings() {
		// Settings is not actually alloc'd, mark as embedded to prevent release from doing anything
		settings.SetEmbedded();
		settings.AddRef();
	}

	~WheelSettings() {
		settings.Release();
	}

	float get_inertia() const { return settings.mInertia; }
	void set_inertia(float p_inertia) { settings.mInertia = p_inertia; }

	float get_angular_damping() const { return settings.mAngularDamping; }
	void set_angular_damping(float p_angular_damping) { settings.mAngularDamping = p_angular_damping; }

	float get_max_steer_angle() const { return settings.mMaxSteerAngle; }
	void set_max_steer_angle(float p_max_steer_angle) { settings.mMaxSteerAngle = p_max_steer_angle; }

	float get_max_brake_torque() const { return settings.mMaxBrakeTorque; }
	void set_max_brake_torque(float p_max_brake_torque) { settings.mMaxBrakeTorque = p_max_brake_torque; }

	float get_max_hand_brake_torque() const { return settings.mMaxHandBrakeTorque; }
	void set_max_hand_brake_torque(float p_max_hand_brake_torque) { settings.mMaxHandBrakeTorque = p_max_hand_brake_torque; }

	Ref<Curve> get_lateral_friction() const { return lateral_friction; }
	void set_lateral_friction(const Ref<Curve> &p_lateral_friction);

	Ref<Curve> get_longitudinal_friction() const { return longitudinal_friction; }
	void set_longitudinal_friction(const Ref<Curve> &p_longitudinal_friction);

protected:
	static void _bind_methods();

	bool is_lateral_first_set = true;
	bool is_longitudinal_first_set = true;
	Ref<Curve> lateral_friction;
	Ref<Curve> longitudinal_friction;

	void _apply_lateral_friction();
	void _apply_longitudinal_friction();
};

VARIANT_ENUM_CAST(WheelBaseSettings::SpringMode);
