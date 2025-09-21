#include "vehicle_controller_settings.h"

void WheeledVehicleControllerSettings::_bind_methods()
{
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

void WheeledVehicleControllerSettings::_validate_property(PropertyInfo &p_property) const
{
	if (is_open_differential() && p_property.name == "differential_slip_ratio")
	{
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

