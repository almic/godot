#pragma once

#include "core/io/resource.h"
#include "scene/resources/curve.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleEngine.h>

class WheeledVehicleControllerSettings;

class VehicleEngineSettings : public Resource {
	GDCLASS(VehicleEngineSettings, Resource);

protected:
	JPH::VehicleEngineSettings settings;

	friend class WheeledVehicleControllerSettings;

public:
	float get_max_torque() const { return settings.mMaxTorque; }
	void set_max_torque(float p_max_torque) { settings.mMaxTorque = p_max_torque; }

	float get_min_rpm() const { return settings.mMinRPM; }
	void set_min_rpm(float p_min_rpm) { settings.mMinRPM = p_min_rpm; }

	float get_max_rpm() const { return settings.mMaxRPM; }
	void set_max_rpm(float p_max_rpm) { settings.mMaxRPM = p_max_rpm; }

	float get_inertia() const { return settings.mInertia; }
	void set_inertia(float p_inertia) { settings.mInertia = p_inertia; }

	float get_angular_damping() const { return settings.mAngularDamping; }
	void set_angular_damping(float p_angular_damping) { settings.mAngularDamping = p_angular_damping; }

	Ref<Curve> get_normalized_torque_curve() const { return torque_curve; }
	void set_normalized_torque_curve(const Ref<Curve> &p_torque_curve);

protected:
	static void _bind_methods();

	bool first_torque_curve_set = true;
	Ref<Curve> torque_curve;

	void _apply_torque_curve();
};
