#include "vehicle_transmission_settings.h"

void VehicleTransmissionSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_transmission_mode"), &VehicleTransmissionSettings::get_transmission_mode);
	ClassDB::bind_method(D_METHOD("set_transmission_mode", "transmission_mode"), &VehicleTransmissionSettings::set_transmission_mode);

	ClassDB::bind_method(D_METHOD("get_gear_ratios"), &VehicleTransmissionSettings::get_gear_ratios);
	ClassDB::bind_method(D_METHOD("set_gear_ratios", "gear_ratios"), &VehicleTransmissionSettings::set_gear_ratios);

	ClassDB::bind_method(D_METHOD("get_reverse_gear_ratios"), &VehicleTransmissionSettings::get_reverse_gear_ratios);
	ClassDB::bind_method(D_METHOD("set_reverse_gear_ratios", "reverse_gear_ratios"), &VehicleTransmissionSettings::set_reverse_gear_ratios);

	ClassDB::bind_method(D_METHOD("get_switch_time"), &VehicleTransmissionSettings::get_switch_time);
	ClassDB::bind_method(D_METHOD("set_switch_time", "switch_time"), &VehicleTransmissionSettings::set_switch_time);

	ClassDB::bind_method(D_METHOD("get_clutch_release_time"), &VehicleTransmissionSettings::get_clutch_release_time);
	ClassDB::bind_method(D_METHOD("set_clutch_release_time", "release_time"), &VehicleTransmissionSettings::set_clutch_release_time);

	ClassDB::bind_method(D_METHOD("get_switch_latency"), &VehicleTransmissionSettings::get_switch_latency);
	ClassDB::bind_method(D_METHOD("set_switch_latency", "switch_latency"), &VehicleTransmissionSettings::set_switch_latency);

	ClassDB::bind_method(D_METHOD("get_shift_up_rpm"), &VehicleTransmissionSettings::get_shift_up_rpm);
	ClassDB::bind_method(D_METHOD("set_shift_up_rpm", "shift_up_rpm"), &VehicleTransmissionSettings::set_shift_up_rpm);

	ClassDB::bind_method(D_METHOD("get_shift_down_rpm"), &VehicleTransmissionSettings::get_shift_down_rpm);
	ClassDB::bind_method(D_METHOD("set_shift_down_rpm", "shift_down_rpm"), &VehicleTransmissionSettings::set_shift_down_rpm);

	ClassDB::bind_method(D_METHOD("get_clutch_strength"), &VehicleTransmissionSettings::get_clutch_strength);
	ClassDB::bind_method(D_METHOD("set_clutch_strength", "clutch_strength"), &VehicleTransmissionSettings::set_clutch_strength);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "transmission_mode", PROPERTY_HINT_ENUM, "Automatic,Manual"), "set_transmission_mode", "get_transmission_mode");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "reverse_gear_ratios"), "set_reverse_gear_ratios", "get_reverse_gear_ratios");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch_strength", PROPERTY_HINT_RANGE, U"0,30,0.001,or_greater,suffix:kg\u22C5m\u00B2/s"), "set_clutch_strength", "get_clutch_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "up_rpm", PROPERTY_HINT_RANGE, "0,5000,0.1,or_greater,suffix:RPM"), "set_shift_up_rpm", "get_shift_up_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "down_rpm", PROPERTY_HINT_RANGE, "0,3000,0.1,or_greater,suffix:RPM"), "set_shift_down_rpm", "get_shift_down_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "switch_time", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:s"), "set_switch_time", "get_switch_time");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "switch_latency", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:s"), "set_switch_latency", "get_switch_latency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch_release_time", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:s"), "set_clutch_release_time", "get_clutch_release_time");

	BIND_ENUM_CONSTANT(TRANSMISSION_MODE_AUTO);
	BIND_ENUM_CONSTANT(TRANSMISSION_MODE_MANUAL);
}

void VehicleTransmissionSettings::_validate_property(PropertyInfo &p_property) const
{
	if (get_transmission_mode() == TRANSMISSION_MODE_MANUAL)
	{
		if (
			   p_property.name == "clutch_release_time"
			|| p_property.name == "shift_down_rpm"
			|| p_property.name == "shift_up_rpm"
			|| p_property.name == "switch_latency"
			|| p_property.name == "switch_time"
		)
		{
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	}
}

