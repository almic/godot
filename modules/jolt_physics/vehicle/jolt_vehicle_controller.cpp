#include "jolt_vehicle_controller.h"

#include "core/error/error_macros.h"
#include "core/math/math_defs.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"

#include <cmath>
#include <limits>

void WheeledVehicleControllerSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_engine_settings"), &WheeledVehicleControllerSettings::get_engine_settings);
	ClassDB::bind_method(D_METHOD("set_engine_settings", "engine_settings"), &WheeledVehicleControllerSettings::set_engine_settings);

	ClassDB::bind_method(D_METHOD("get_transmission_settings"), &WheeledVehicleControllerSettings::get_transmission_settings);
	ClassDB::bind_method(D_METHOD("set_transmission_settings", "transmission_settings"), &WheeledVehicleControllerSettings::set_transmission_settings);

	ClassDB::bind_method(D_METHOD("get_differential_list"), &WheeledVehicleControllerSettings::get_differential_list);
	ClassDB::bind_method(D_METHOD("set_differential_list", "differential_list"), &WheeledVehicleControllerSettings::set_differential_list);

	ClassDB::bind_method(D_METHOD("get_differential_slip_ratio"), &WheeledVehicleControllerSettings::get_differential_slip_ratio);
	ClassDB::bind_method(D_METHOD("set_differential_slip_ratio", "differential_slip_ratio"), &WheeledVehicleControllerSettings::set_differential_slip_ratio);
	ClassDB::bind_method(D_METHOD("is_open_differential"), &WheeledVehicleControllerSettings::is_open_differential);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine", PROPERTY_HINT_RESOURCE_TYPE, "VehicleEngineSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_engine_settings", "get_engine_settings");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "transmission", PROPERTY_HINT_RESOURCE_TYPE, "VehicleTransmissionSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_transmission_settings", "get_transmission_settings");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "differential_list", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("VehicleDifferentialSettings")), "set_differential_list", "get_differential_list");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "differential_slip_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_differential_slip_ratio", "get_differential_slip_ratio");
}

void WheeledVehicleControllerSettings::_apply_engine() {
	settings->mEngine = engine->settings;
}

void WheeledVehicleControllerSettings::_apply_transmission() {
	settings->mTransmission = transmission->settings;
}

void WheeledVehicleControllerSettings::_apply_differential_list() {
	JPH::Array<JPH::VehicleDifferentialSettings> &array = settings->mDifferentials;
	array.resize(differential_list.size());

	int i = 0;
	for (const auto &d : differential_list) {
		if (d.is_valid()) {
			array[i++] = d->get_settings();
		} else {
			ERR_PRINT("WheeledVehicleControllerSettings has an invalid differential setting! This should not happen!");
			array.resize(array.size() - 1);
		}
	}
}

TypedArray<VehicleDifferentialSettings> WheeledVehicleControllerSettings::get_differential_list() const {
	TypedArray<VehicleDifferentialSettings> result;
	for (const auto &d : differential_list) {
		result.push_back(d);
	}
	return result;
}

void WheeledVehicleControllerSettings::set_differential_list(const TypedArray<VehicleDifferentialSettings> &p_differential_list) {
	for (auto &d : differential_list) {
		if (d.is_valid()) {
			d->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differential_list));
		}
	}

	differential_list.clear();

	for (auto &d : p_differential_list) {
		Ref<VehicleDifferentialSettings> differential = d;

		if (differential.is_valid()) {
			differential->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differential_list));
		}

		differential_list.push_back(differential);
	}

	_apply_differential_list();
}

real_t WheeledVehicleControllerSettings::get_differential_slip_ratio() const {
	return settings->mDifferentialLimitedSlipRatio;
}

void WheeledVehicleControllerSettings::set_differential_slip_ratio(real_t p_differential_slip_ratio) {
	if (p_differential_slip_ratio >= std::numeric_limits<float>::max()) {
		settings->mDifferentialLimitedSlipRatio = std::numeric_limits<float>::max();
	} else {
		settings->mDifferentialLimitedSlipRatio = (float)p_differential_slip_ratio;
	}
}

bool WheeledVehicleControllerSettings::is_open_differential() const {
	return settings->mDifferentialLimitedSlipRatio == std::numeric_limits<float>::max();
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

void VehicleController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_speed_kmh"), &VehicleController::get_speed_kmh);
	ClassDB::bind_method(D_METHOD("get_speed_mph"), &VehicleController::get_speed_mph);
}

void WheeledVehicleController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_driver_input", "forward", "right", "brake", "handbrake"), &WheeledVehicleController::set_driver_input);
	ClassDB::bind_method(D_METHOD("get_current_gear"), &WheeledVehicleController::get_current_gear);
	ClassDB::bind_method(D_METHOD("get_current_rpm"), &WheeledVehicleController::get_current_rpm);
	ClassDB::bind_method(D_METHOD("get_wheel_data", "index", "data_type"), &WheeledVehicleController::get_wheel_data);
	ClassDB::bind_method(D_METHOD("is_wheel_contact", "index"), &WheeledVehicleController::is_wheel_contact);

	BIND_ENUM_CONSTANT(WHEEL_ANGULAR_VELOCITY);
	BIND_ENUM_CONSTANT(WHEEL_LATERAL_SLIP);
	BIND_ENUM_CONSTANT(WHEEL_LONGITUDINAL_SLIP);
	BIND_ENUM_CONSTANT(WHEEL_LATERAL_LAMBDA);
	BIND_ENUM_CONSTANT(WHEEL_LONGITUDITAL_LAMBDA);
	BIND_ENUM_CONSTANT(WHEEL_ROTATION_ANGLE);
	BIND_ENUM_CONSTANT(WHEEL_STEER_ANGLE);
}

float WheeledVehicleController::get_speed_kmh() const {
	float kmh = 0.0f;

	const JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
	const int size = wheels.size();
	int on_ground = 0;

	for (int i = 0; i < size; i++) {
		const JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheels[i]);
		if (!wheel->HasContact()) {
			continue;
		}

		++on_ground;

		float ms = wheel->GetSettings()->mRadius * std::abs(wheel->GetAngularVelocity());
		ms *= MAX(0.0f, 1.0f - wheel->mLongitudinalSlip);
		kmh += ms;
	}

	if (on_ground > 0) {
		kmh *= 3.6f;
		kmh /= on_ground;
	}

	return kmh;
}

float WheeledVehicleController::get_wheel_data(int p_index, WheelData p_data_type) const {
	const JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
	ERR_FAIL_INDEX_V(p_index, wheels.size(), 0.0f);
	const JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheels[p_index]);

	switch (p_data_type) {
		case WHEEL_ANGULAR_VELOCITY:
			return wheel->GetAngularVelocity();
		case WHEEL_LATERAL_SLIP:
			return wheel->mLateralSlip;
		case WHEEL_LONGITUDINAL_SLIP:
			return wheel->mLongitudinalSlip;
		case WHEEL_LATERAL_LAMBDA:
			return wheel->GetLateralLambda();
		case WHEEL_LONGITUDITAL_LAMBDA:
			return wheel->GetLongitudinalLambda();
		case WHEEL_ROTATION_ANGLE:
			return wheel->GetRotationAngle();
		case WHEEL_STEER_ANGLE:
			return wheel->GetSteerAngle();
		default:
			ERR_FAIL_V_MSG(0.0f, vformat("Unknown WheelData enum value %d", p_data_type));
	}
}

bool WheeledVehicleController::is_wheel_contact(int p_index) const {
	const JPH::Wheels wheels = jolt_controller->GetConstraint().GetWheels();
	ERR_FAIL_INDEX_V(p_index, wheels.size(), 0.0f);
	const JPH::WheelWV *wheel = static_cast<JPH::WheelWV *>(wheels[p_index]);
	return wheel->HasContact();
}
