#include "jolt_vehicle.h"

void JoltVehicle::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_settings"), &JoltVehicle::get_settings);
	ClassDB::bind_method(D_METHOD("set_settings", "vehicle_settings"), &JoltVehicle::set_settings);

	ClassDB::bind_method(D_METHOD("get_controller"), &JoltVehicle::get_controller);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "settings", PROPERTY_HINT_RESOURCE_TYPE, "JoltVehicleSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_settings", "get_settings");
}

