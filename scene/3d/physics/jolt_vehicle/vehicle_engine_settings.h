#pragma once

#include "core/io/resource.h"
#include "scene/resources/curve.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/VehicleEngine.h"


class WheeledVehicleControllerSettings;

class VehicleEngineSettings : public Resource {
	GDCLASS(VehicleEngineSettings, Resource);

protected:
	JPH::VehicleEngineSettings settings;

	friend class WheeledVehicleControllerSettings;
public:

	real_t get_max_torque() const { return (real_t) settings.mMaxTorque; }
	void set_max_torque(real_t p_max_torque) { settings.mMaxTorque = (float) settings.mMaxTorque; }

	real_t get_min_rpm() const { return (real_t) settings.mMinRPM; }
	void set_min_rpm(real_t p_min_rpm) { settings.mMinRPM = (float) p_min_rpm; }

	real_t get_max_rpm() const { return (real_t) settings.mMaxRPM; }
	void set_max_rpm(real_t p_max_rpm) { settings.mMaxRPM = (float) p_max_rpm; }

	real_t get_inertia() const { return (real_t) settings.mInertia; }
	void set_inertia(real_t p_inertia) { settings.mInertia = (float) p_inertia; }

	real_t get_angular_damping() const { return (real_t) settings.mAngularDamping; }
	void set_angular_damping(real_t p_angular_damping) { settings.mAngularDamping = (float) p_angular_damping; }

	Ref<Curve> get_normalized_torque_curve() const { return torque_curve; }
	void set_normalized_torque_curve(const Ref<Curve>& p_torque_curve)
	{
		if (torque_curve.is_valid()) {
			torque_curve->disconnect_changed(callable_mp(this, &VehicleEngineSettings::_apply_torque_curve));
		}

		torque_curve = p_torque_curve;

		if (torque_curve.is_valid() && (!first_torque_curve_set || p_torque_curve->get_point_count() > 0)) {
			torque_curve->connect_changed(callable_mp(this, &VehicleEngineSettings::_apply_torque_curve));
			_apply_torque_curve();
		}
		else
		{
			// Reset to default
			JPH::VehicleEngineSettings defaults;
			settings.mNormalizedTorque = defaults.mNormalizedTorque;

			torque_curve = memnew(Curve);
			to_godot(settings.mNormalizedTorque, *torque_curve.ptr());
			torque_curve->connect_changed(callable_mp(this, &VehicleEngineSettings::_apply_torque_curve));
		}

		first_torque_curve_set = false;
	}

protected:
	static void _bind_methods();

	bool first_torque_curve_set = true;
	Ref<Curve> torque_curve;

	void _apply_torque_curve() {
		settings.mNormalizedTorque = to_jolt(*torque_curve.ptr());
	}
};

