#include "jolt_vehicle.h"

void JoltVehicle::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_wheel_collision_mode"), &JoltVehicle::get_wheel_collision_mode);
	ClassDB::bind_method(D_METHOD("set_wheel_collision_mode", "collision_mode"), &JoltVehicle::set_wheel_collision_mode);

	ClassDB::bind_method(D_METHOD("get_wheel_sphere_test_radius"), &JoltVehicle::get_wheel_sphere_test_radius);
	ClassDB::bind_method(D_METHOD("set_wheel_sphere_test_radius", "radius"), &JoltVehicle::set_wheel_sphere_test_radius);

	ClassDB::bind_method(D_METHOD("get_settings"), &JoltVehicle::get_settings);
	ClassDB::bind_method(D_METHOD("set_settings", "vehicle_settings"), &JoltVehicle::set_settings);

	ClassDB::bind_method(D_METHOD("get_controller"), &JoltVehicle::get_controller);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "wheel_collision_mode", PROPERTY_HINT_ENUM, "Raycast,Sphere,Cylinder"), "set_wheel_collision_mode", "get_wheel_collision_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_sphere_test_radius", PROPERTY_HINT_RANGE, "0.001,1.0,0.001,or_greater,suffix:m"), "set_wheel_sphere_test_radius", "get_wheel_sphere_test_radius");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "settings", PROPERTY_HINT_RESOURCE_TYPE, "JoltVehicleSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_settings", "get_settings");

	BIND_ENUM_CONSTANT(WHEEL_COLLISION_RAYCAST);
	BIND_ENUM_CONSTANT(WHEEL_COLLISION_SPHERE);
	BIND_ENUM_CONSTANT(WHEEL_COLLISION_CYLINDER);
}

void JoltVehicle::_validate_property(PropertyInfo &p_property) const
{
	if (get_wheel_collision_mode() != WHEEL_COLLISION_SPHERE && p_property.name == "wheel_sphere_test_radius")
	{
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}
