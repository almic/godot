#include "jolt_vehicle_differential_settings.h"

#include "core/object/class_db.h"

#include <limits>

void VehicleDifferentialSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_left_wheel_index"), &VehicleDifferentialSettings::get_left_wheel_index);
	ClassDB::bind_method(D_METHOD("set_left_wheel_index", "left_index"), &VehicleDifferentialSettings::set_left_wheel_index);

	ClassDB::bind_method(D_METHOD("get_right_wheel_index"), &VehicleDifferentialSettings::get_right_wheel_index);
	ClassDB::bind_method(D_METHOD("set_right_wheel_index", "right_index"), &VehicleDifferentialSettings::set_right_wheel_index);

	ClassDB::bind_method(D_METHOD("get_differential_ratio"), &VehicleDifferentialSettings::get_differential_ratio);
	ClassDB::bind_method(D_METHOD("set_differential_ratio", "differential_ratio"), &VehicleDifferentialSettings::set_differential_ratio);

	ClassDB::bind_method(D_METHOD("get_wheel_split"), &VehicleDifferentialSettings::get_wheel_split);
	ClassDB::bind_method(D_METHOD("set_wheel_split", "wheel_split"), &VehicleDifferentialSettings::set_wheel_split);

	ClassDB::bind_method(D_METHOD("get_slip_ratio"), &VehicleDifferentialSettings::get_slip_ratio);
	ClassDB::bind_method(D_METHOD("set_slip_ratio", "slip_ratio"), &VehicleDifferentialSettings::set_slip_ratio);
	ClassDB::bind_method(D_METHOD("is_slip_open"), &VehicleDifferentialSettings::is_slip_open);

	ClassDB::bind_method(D_METHOD("get_torque_ratio"), &VehicleDifferentialSettings::get_torque_ratio);
	ClassDB::bind_method(D_METHOD("set_torque_ratio", "torque_ratio"), &VehicleDifferentialSettings::set_torque_ratio);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "left_wheel", PROPERTY_HINT_RANGE, "-1,10,1,or_greater"), "set_left_wheel_index", "get_left_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "right_wheel", PROPERTY_HINT_RANGE, "-1,10,1,or_greater"), "set_right_wheel_index", "get_right_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_split", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_wheel_split", "get_wheel_split");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "differential_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_differential_ratio", "get_differential_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque_ratio", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_torque_ratio", "get_torque_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "open"), "set_open", "is_open");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slip_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_slip_ratio", "get_slip_ratio");
}

real_t VehicleDifferentialSettings::get_slip_ratio() const {
	return (real_t)settings.mLimitedSlipRatio;
}

void VehicleDifferentialSettings::set_slip_ratio(real_t p_slip_ratio) {
	if (p_slip_ratio >= std::numeric_limits<float>::max()) {
		settings.mLimitedSlipRatio = std::numeric_limits<float>::max();
	} else {
		settings.mLimitedSlipRatio = (float)p_slip_ratio;
	}
}

bool VehicleDifferentialSettings::is_slip_open() const {
	return settings.mLimitedSlipRatio == std::numeric_limits<float>::max();
}
