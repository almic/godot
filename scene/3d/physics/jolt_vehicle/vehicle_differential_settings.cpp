#include "vehicle_differential_settings.h"

void VehicleDifferentialSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_left_wheel_index"), &VehicleDifferentialSettings::get_left_wheel_index);
	ClassDB::bind_method(D_METHOD("set_left_wheel_index", "left_index"), &VehicleDifferentialSettings::set_left_wheel_index);

	ClassDB::bind_method(D_METHOD("get_right_wheel_index"), &VehicleDifferentialSettings::get_right_wheel_index);
	ClassDB::bind_method(D_METHOD("set_right_wheel_index", "right_index"), &VehicleDifferentialSettings::set_right_wheel_index);

	ClassDB::bind_method(D_METHOD("get_differential_ratio"), &VehicleDifferentialSettings::get_differential_ratio);
	ClassDB::bind_method(D_METHOD("set_differential_ratio", "differential_ratio"), &VehicleDifferentialSettings::set_differential_ratio);

	ClassDB::bind_method(D_METHOD("get_split"), &VehicleDifferentialSettings::get_split);
	ClassDB::bind_method(D_METHOD("set_split", "wheel_split"), &VehicleDifferentialSettings::set_split);

	ClassDB::bind_method(D_METHOD("is_open"), &VehicleDifferentialSettings::is_open);
	ClassDB::bind_method(D_METHOD("set_open", "is_open"), &VehicleDifferentialSettings::set_open);

	ClassDB::bind_method(D_METHOD("get_slip_ratio"), &VehicleDifferentialSettings::get_slip_ratio);
	ClassDB::bind_method(D_METHOD("set_slip_ratio", "slip_ratio"), &VehicleDifferentialSettings::set_slip_ratio);

	ClassDB::bind_method(D_METHOD("get_torque_ratio"), &VehicleDifferentialSettings::get_torque_ratio);
	ClassDB::bind_method(D_METHOD("set_torque_ratio", "torque_ratio"), &VehicleDifferentialSettings::set_torque_ratio);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "left_wheel_index", PROPERTY_HINT_RANGE, "-1,10,1,or_greater"), "set_left_wheel_index", "get_left_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "right_wheel_index", PROPERTY_HINT_RANGE, "-1,10,1,or_greater"), "set_right_wheel_index", "get_right_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "split", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_split", "get_split");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "differential_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_differential_ratio", "get_differential_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque_ratio", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_torque_ratio", "get_torque_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "open"), "set_open", "is_open");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slip_ratio", PROPERTY_HINT_RANGE, "1,10,0.001,or_less,or_greater"), "set_slip_ratio", "get_slip_ratio");
}

void VehicleDifferentialSettings::_validate_property(PropertyInfo &p_property) const
{
	if (is_open() && p_property.name == "slip_ratio")
	{
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

