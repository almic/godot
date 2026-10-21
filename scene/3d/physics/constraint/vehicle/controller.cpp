#include "controller.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"

#include <cmath>
#include <limits>

void VehicleController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_speed_kmh"), &VehicleController::get_speed_kmh);
	ClassDB::bind_method(D_METHOD("get_speed_mph"), &VehicleController::get_speed_mph);
}

void WheeledVehicleController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_driver_input", "forward", "right", "brake", "handbrake"), &WheeledVehicleController::set_driver_input);
	ClassDB::bind_method(D_METHOD("get_current_gear"), &WheeledVehicleController::get_current_gear);
	ClassDB::bind_method(D_METHOD("get_current_rpm"), &WheeledVehicleController::get_current_rpm);
	ClassDB::bind_method(D_METHOD("get_wheel_info"), &WheeledVehicleController::get_wheel_info);
	ClassDB::bind_method(D_METHOD("is_any_driven_wheel_slipping"), &WheeledVehicleController::is_any_driven_wheel_slipping);
}

void WheeledVehicleControllerSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_engine_settings"), &WheeledVehicleControllerSettings::get_engine_settings);
	ClassDB::bind_method(D_METHOD("set_engine_settings", "engine_settings"), &WheeledVehicleControllerSettings::set_engine_settings);

	ClassDB::bind_method(D_METHOD("get_transmission_settings"), &WheeledVehicleControllerSettings::get_transmission_settings);
	ClassDB::bind_method(D_METHOD("set_transmission_settings", "transmission_settings"), &WheeledVehicleControllerSettings::set_transmission_settings);

	ClassDB::bind_method(D_METHOD("get_differentials"), &WheeledVehicleControllerSettings::get_differentials);
	ClassDB::bind_method(D_METHOD("set_differentials", "differentials"), &WheeledVehicleControllerSettings::set_differentials);

	ClassDB::bind_method(D_METHOD("is_open_differential"), &WheeledVehicleControllerSettings::is_open_differential);
	ClassDB::bind_method(D_METHOD("set_open_differential", "is_open"), &WheeledVehicleControllerSettings::set_open_differential);

	ClassDB::bind_method(D_METHOD("get_differential_slip_ratio"), &WheeledVehicleControllerSettings::get_differential_slip_ratio);
	ClassDB::bind_method(D_METHOD("set_differential_slip_ratio", "differential_slip_ratio"), &WheeledVehicleControllerSettings::set_differential_slip_ratio);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine", PROPERTY_HINT_RESOURCE_TYPE, "VehicleEngineSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_engine_settings", "get_engine_settings");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "transmission", PROPERTY_HINT_RESOURCE_TYPE, "VehicleTransmissionSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_transmission_settings", "get_transmission_settings");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "differentials", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("VehicleDifferentialSettings")), "set_differentials", "get_differentials");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "open_differential"), "set_open_differential", "is_open_differential");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "differential_slip_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_differential_slip_ratio", "get_differential_slip_ratio");
}

void WheeledVehicleControllerSettings::_validate_property(PropertyInfo &p_property) const {
	if (is_open_differential() && p_property.name == "differential_slip_ratio") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

TypedArray<Dictionary> WheeledVehicleController::get_wheel_info() const {
	JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
	TypedArray<Dictionary> results;
	results.resize(wheels.size());
	int i = 0;
	for (JPH::Wheel *wheel_base : wheels) {
		JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheel_base);
		Dictionary d;

		d["has_contact"] = wheel->HasContact();
		d["slip_long"] = wheel->mLongitudinalSlip;
		d["slip_lat"] = wheel->mLateralSlip;
		d["steer_angle"] = wheel->GetSteerAngle();
		d["angular_velocity"] = wheel->GetAngularVelocity();
		d["rotation_angle"] = wheel->GetRotationAngle();
		d["lambda_lat"] = wheel->GetLateralLambda();
		d["lambda_long"] = wheel->GetLongitudinalLambda();

		results[i] = d;

		i++;
	}
	return results;
}

float WheeledVehicleController::get_speed_kmh() const {
	float kmh = 0.0f;

	const JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
	const int size = wheels.size();
	int on_ground = 0;

	for (int i = 0; i < size; i++) {
		JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheels[i]);
		if (!wheel->HasContact()) {
			continue;
		}

		float slip = wheel->mLongitudinalSlip;
		if (slip > 0.5f) {
			continue;
		}

		++on_ground;
		float ms = wheel->GetSettings()->mRadius * std::abs(wheel->GetAngularVelocity());
		if (slip > 0.0) {
			ms *= 1.0f - slip;
		}
		kmh += ms;
	}

	if (on_ground > 0) {
		kmh *= 3.6f;
		kmh /= on_ground;
	}

	if (kmh < 0.1f) {
		kmh = 0.0f;
	}

	return kmh;
}

void WheeledVehicleControllerSettings::set_engine_settings(const Ref<VehicleEngineSettings> &p_engine_settings) {
	if (engine.is_valid()) {
		engine->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_engine));
	}

	engine = p_engine_settings;

	if (!engine.is_valid()) {
		// Reset to default
		engine = memnew(VehicleEngineSettings);
	}

	engine->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_engine));
	_apply_engine();
}

void WheeledVehicleControllerSettings::set_transmission_settings(const Ref<VehicleTransmissionSettings> &p_transmission_settings) {
	if (transmission.is_valid()) {
		transmission->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_transmission));
	}

	transmission = p_transmission_settings;

	if (!transmission.is_valid()) {
		// Reset to default
		transmission = memnew(VehicleTransmissionSettings);
	}

	transmission->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_transmission));
	_apply_transmission();
}

TypedArray<VehicleDifferentialSettings> WheeledVehicleControllerSettings::get_differentials() const {
	TypedArray<VehicleDifferentialSettings> result;
	for (const auto &d : differentials) {
		result.push_back(d);
	}
	return result;
}

void WheeledVehicleControllerSettings::set_differentials(const TypedArray<VehicleDifferentialSettings> &p_differentials) {
	for (const auto &d : differentials) {
		if (d.is_valid()) {
			d->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
		}
	}

	differentials.clear();

	for (const auto &d : p_differentials) {
		Ref<VehicleDifferentialSettings> differential = d;

		if (differential.is_valid()) {
			differential->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
		}

		differentials.push_back(differential);
	}

	_apply_differentials();
}

bool WheeledVehicleControllerSettings::is_open_differential() const {
	return settings.mDifferentialLimitedSlipRatio == std::numeric_limits<float>::max();
}

void WheeledVehicleControllerSettings::set_open_differential(bool p_is_open) {
	if (p_is_open) {
		settings.mDifferentialLimitedSlipRatio = std::numeric_limits<float>::max();
	} else {
		settings.mDifferentialLimitedSlipRatio = differential_slip_ratio;
	}

	notify_property_list_changed();
}

float WheeledVehicleControllerSettings::get_differential_slip_ratio() const {
	if (is_open_differential()) {
		return differential_slip_ratio;
	} else {
		return settings.mDifferentialLimitedSlipRatio;
	}
}

void WheeledVehicleControllerSettings::set_differential_slip_ratio(float p_differential_slip_ratio) {
	if (std::isinf(p_differential_slip_ratio) || p_differential_slip_ratio == std::numeric_limits<float>::max()) {
		set_open_differential(true);
	} else {
		differential_slip_ratio = p_differential_slip_ratio;
		settings.mDifferentialLimitedSlipRatio = p_differential_slip_ratio;
		notify_property_list_changed();
	}
}

void WheeledVehicleControllerSettings::_apply_engine() {
	settings.mEngine = engine->settings;
}

void WheeledVehicleControllerSettings::_apply_transmission() {
	settings.mTransmission = transmission->settings;
}

void WheeledVehicleControllerSettings::_apply_differentials() {
	JPH::Array<JPH::VehicleDifferentialSettings> &array = settings.mDifferentials;
	array.resize(differentials.size());

	int i = 0;
	for (const auto &d : differentials) {
		if (d.is_valid()) {
			array[i++] = d->settings;
		} else {
			ERR_PRINT("VehicleControllerSettings has an invalid setting!");
			array.resize(array.size() - 1);
		}
	}
}
