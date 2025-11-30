#pragma once

#include "core/io/resource.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

#include "Jolt/Physics/Constraints/SpringSettings.h"
#include "Jolt/Physics/Vehicle/Wheel.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"

class JoltVehicle;
class JoltVehicleSettings;

class WheelBaseSettings : public Resource {
	GDCLASS(WheelBaseSettings, Resource);

protected:
	virtual JPH::WheelSettings *get_settings() { return nullptr; }
	virtual const JPH::WheelSettings *get_settings() const { return nullptr; }

	friend class JoltVehicle;
	friend class JoltVehicleSettings;

public:
	enum SpringMode {
		SPRING_MODE_FREQUENCY,
		SPRING_MODE_STIFFNESS,
	};

	Vector3 get_position() const { return to_godot(get_settings()->mPosition); }
	void set_position(const Vector3 &p_position) {
		get_settings()->mPosition = to_jolt(p_position);
		emit_changed();
	}

	Vector3 get_suspension_force_point() const { return to_godot(get_settings()->mSuspensionForcePoint); }
	void set_suspension_force_point(const Vector3 &p_force_point) { get_settings()->mSuspensionForcePoint = to_jolt(p_force_point); }

	Vector3 get_suspension_direction() const { return to_godot(get_settings()->mSuspensionDirection); }

	real_t get_suspension_forward_angle() const { return suspension_forward_angle; }
	void set_suspension_forward_angle(real_t p_angle) {
		suspension_forward_angle = p_angle;
		get_settings()->mSuspensionDirection = to_jolt(Vector3(tan(suspension_sideways_angle), -1, tan(suspension_forward_angle)).normalized());
		emit_changed();
	}

	real_t get_suspension_sideways_angle() const { return suspension_sideways_angle; }
	void set_suspension_sideways_angle(real_t p_angle) {
		suspension_sideways_angle = p_angle;
		get_settings()->mSuspensionDirection = to_jolt(Vector3(tan(suspension_sideways_angle), -1, tan(suspension_forward_angle)).normalized());
		emit_changed();
	}

	Vector3 get_steering_axis() const { return to_godot(get_settings()->mSteeringAxis); }
	Vector3 get_up() const { return to_godot(get_settings()->mWheelUp); }
	Vector3 get_forward() const { return to_godot(get_settings()->mWheelForward); }

	real_t get_kingpin_angle() const { return kingpin_angle; }
	void set_kingpin_angle(real_t p_angle) {
		kingpin_angle = p_angle;
		get_settings()->mSteeringAxis = to_jolt(Vector3(-tan(kingpin_angle), 1, -tan(caster_angle)).normalized());
		emit_changed();
	}

	real_t get_caster_angle() const { return caster_angle; }
	void set_caster_angle(real_t p_angle) {
		caster_angle = p_angle;
		get_settings()->mSteeringAxis = to_jolt(Vector3(-tan(kingpin_angle), 1, -tan(caster_angle)).normalized());
		emit_changed();
	}

	real_t get_camber() const { return camber; }
	void set_camber(real_t p_camber) {
		camber = p_camber;
		get_settings()->mWheelUp = to_jolt(Vector3(sin(camber), cos(camber), 0));
		emit_changed();
	}

	real_t get_toe() const { return toe; }
	void set_toe(real_t p_toe) {
		toe = p_toe;
		get_settings()->mWheelForward = to_jolt(Vector3(-sin(toe), 0, cos(toe)));
		emit_changed();
	}

	real_t get_suspension_max_length() const { return (real_t)get_settings()->mSuspensionMaxLength; }
	void set_suspension_max_length(real_t p_max_length) { get_settings()->mSuspensionMaxLength = (float)p_max_length; }

	real_t get_suspension_min_length() const { return (real_t)get_settings()->mSuspensionMinLength; }
	void set_suspension_min_length(real_t p_min_length) {
		get_settings()->mSuspensionMinLength = (float)p_min_length;
		emit_changed();
	}

	real_t get_suspension_preload_length() const { return (real_t)get_settings()->mSuspensionPreloadLength; }
	void set_suspension_preload_length(real_t p_preload_length) {
		get_settings()->mSuspensionPreloadLength = (real_t)p_preload_length;
		emit_changed();
	}

	SpringMode get_spring_mode() const { return (SpringMode)get_settings()->mSuspensionSpring.mMode; }
	void set_spring_mode(SpringMode p_mode) {
		frequency = get_spring_frequency();
		stiffness = get_spring_stiffness();

		get_settings()->mSuspensionSpring.mMode = (JPH::ESpringMode)p_mode;

		set_spring_frequency(frequency);
		set_spring_stiffness(stiffness);

		notify_property_list_changed();
	}

	real_t get_spring_frequency() const {
		if (get_spring_mode() == SpringMode::SPRING_MODE_FREQUENCY) {
			return (real_t)get_settings()->mSuspensionSpring.mFrequency;
		} else {
			return frequency;
		}
	}
	void set_spring_frequency(real_t p_frequency) {
		frequency = p_frequency;
		if (get_spring_mode() == SpringMode::SPRING_MODE_FREQUENCY) {
			get_settings()->mSuspensionSpring.mFrequency = (float)frequency;
		}
	}

	real_t get_spring_stiffness() const {
		if (get_spring_mode() == SpringMode::SPRING_MODE_STIFFNESS) {
			return (real_t)get_settings()->mSuspensionSpring.mStiffness * 0.001;
		} else {
			return stiffness;
		}
	}
	void set_spring_stiffness(real_t p_stiffness) {
		stiffness = p_stiffness;
		if (get_spring_mode() == SpringMode::SPRING_MODE_STIFFNESS) {
			get_settings()->mSuspensionSpring.mStiffness = (float)(stiffness * 1000.0);
		}
	}

	real_t get_spring_damping() const { return get_settings()->mSuspensionSpring.mDamping; }
	void set_spring_damping(real_t p_damping) { get_settings()->mSuspensionSpring.mDamping = (float)p_damping; }

	real_t get_radius() const { return (real_t)get_settings()->mRadius; }
	void set_radius(real_t p_radius) { get_settings()->mRadius = (float)p_radius; }

	real_t get_width() const { return (real_t)get_settings()->mWidth; }
	void set_width(real_t p_width) { get_settings()->mWidth = (float)p_width; }

	bool get_enable_suspension_force_point() const { return get_settings()->mEnableSuspensionForcePoint; }
	void set_enable_suspension_force_point(bool p_enable) {
		get_settings()->mEnableSuspensionForcePoint = p_enable;
		notify_property_list_changed();
	}

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;

	real_t stiffness = 0.0;
	real_t frequency = 0.0;

	// Wheel angle parameters
	real_t kingpin_angle = 0.0;
	real_t caster_angle = 0.0;
	real_t camber = 0.0;
	real_t toe = 0.0;

	// Suspension angle parameters
	real_t suspension_forward_angle = 0.0;
	real_t suspension_sideways_angle = 0.0;
};

VARIANT_ENUM_CAST(WheelBaseSettings::SpringMode);

class WheelSettings : public WheelBaseSettings {
	GDCLASS(WheelSettings, WheelBaseSettings);

protected:
	JPH::WheelSettingsWV settings;

	virtual JPH::WheelSettings *get_settings() override { return &settings; }
	virtual const JPH::WheelSettings *get_settings() const override { return &settings; }

public:
	WheelSettings() {
		// When VehicleConstraint takes this, it uses a Ref which will prematurely delete
		// our settings, so we add a reference here to ensure it won't release it for us
		settings.AddRef();
	}

	real_t get_inertia() const { return (real_t)settings.mInertia; }
	void set_inertia(real_t p_inertia) { settings.mInertia = (float)p_inertia; }

	real_t get_angular_damping() const { return (real_t)settings.mAngularDamping; }
	void set_angular_damping(real_t p_angular_damping) { settings.mAngularDamping = (float)p_angular_damping; }

	real_t get_max_steer_angle() const { return (real_t)settings.mMaxSteerAngle; }
	void set_max_steer_angle(real_t p_max_steer_angle) { settings.mMaxSteerAngle = (float)p_max_steer_angle; }

	real_t get_max_brake_torque() const { return (real_t)settings.mMaxBrakeTorque; }
	void set_max_brake_torque(real_t p_max_brake_torque) { settings.mMaxBrakeTorque = (float)p_max_brake_torque; }

	real_t get_max_hand_brake_torque() const { return (real_t)settings.mMaxHandBrakeTorque; }
	void set_max_hand_brake_torque(real_t p_max_hand_brake_torque) { settings.mMaxHandBrakeTorque = (float)p_max_hand_brake_torque; }

	Ref<Curve> get_lateral_friction() const { return lateral_friction; }
	void set_lateral_friction(const Ref<Curve> &p_lateral_friction) {
		if (lateral_friction.is_valid()) {
			lateral_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
		}

		lateral_friction = p_lateral_friction;

		if (lateral_friction.is_valid() && (!is_lateral_first_set || p_lateral_friction->get_point_count() > 0)) {
			lateral_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
			_apply_lateral_friction();
		} else {
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
	void set_longitudinal_friction(const Ref<Curve> &p_longitudinal_friction) {
		if (longitudinal_friction.is_valid()) {
			longitudinal_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
		}

		longitudinal_friction = p_longitudinal_friction;

		if (longitudinal_friction.is_valid() && (!is_longitudinal_first_set || p_longitudinal_friction->get_point_count() > 0)) {
			longitudinal_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
			_apply_longitudinal_friction();
		} else {
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
