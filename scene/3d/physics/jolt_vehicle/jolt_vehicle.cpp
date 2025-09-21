#include "jolt_vehicle.h"

void JoltVehicle::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_settings"), &JoltVehicle::get_settings);
	ClassDB::bind_method(D_METHOD("set_settings", "vehicle_settings"), &JoltVehicle::set_settings);

	ClassDB::bind_method(D_METHOD("get_controller"), &JoltVehicle::get_controller);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "settings", PROPERTY_HINT_RESOURCE_TYPE, "JoltVehicleSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_settings", "get_settings");
}

void JoltVehicleSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_up"), &JoltVehicleSettings::get_up);
	ClassDB::bind_method(D_METHOD("set_up", "up"), &JoltVehicleSettings::set_up);

	ClassDB::bind_method(D_METHOD("get_forward"), &JoltVehicleSettings::get_forward);
	ClassDB::bind_method(D_METHOD("set_forward", "forward"), &JoltVehicleSettings::set_forward);

	ClassDB::bind_method(D_METHOD("is_pitch_roll_limited"), &JoltVehicleSettings::is_pitch_roll_limited);
	ClassDB::bind_method(D_METHOD("set_pitch_roll_limited", "is_limited"), &JoltVehicleSettings::set_pitch_roll_limited);

	ClassDB::bind_method(D_METHOD("get_max_pitch_roll"), &JoltVehicleSettings::get_max_pitch_roll);
	ClassDB::bind_method(D_METHOD("set_max_pitch_roll", "max_pitch_roll"), &JoltVehicleSettings::set_max_pitch_roll);

	ClassDB::bind_method(D_METHOD("get_wheel_settings"), &JoltVehicleSettings::get_wheel_settings);
	ClassDB::bind_method(D_METHOD("set_wheel_settings", "wheel_settings"), &JoltVehicleSettings::set_wheel_settings);

	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_settings"), &JoltVehicleSettings::get_anti_roll_bar_settings);
	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_settings", "anti_roll_bar_settings"), &JoltVehicleSettings::set_anti_roll_bar_settings);

	ClassDB::bind_method(D_METHOD("get_controller_settings"), &JoltVehicleSettings::get_controller_settings);
	ClassDB::bind_method(D_METHOD("set_controller_settings", "controller_settings"), &JoltVehicleSettings::set_controller_settings);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up"), "set_up", "get_up");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "forward"), "set_forward", "get_forward");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "pitch_roll_limited"), "set_pitch_roll_limited", "is_pitch_roll_limited");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_pitch_roll", PROPERTY_HINT_RANGE, "0,89,0.1,radians_as_degrees"), "set_max_pitch_roll", "get_max_pitch_roll");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "wheels", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("WheelBaseSettings")), "set_wheel_settings", "get_wheel_settings");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "anti_roll_bars", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("AnitRollBarSettings")), "set_anti_roll_bar_settings", "get_anti_roll_bar_settings");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "controller", PROPERTY_HINT_RESOURCE_TYPE, "VehicleControllerSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_controller_settings", "get_controller_settings");

void JoltVehicleSettings::_validate_property(PropertyInfo &p_property) const
{
	if (!can_pitch_roll() && p_property.name == "max_pitch_roll")
	{
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

