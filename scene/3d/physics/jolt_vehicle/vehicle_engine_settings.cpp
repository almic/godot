#include "vehicle_engine_settings.h"


void VehicleEngineSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_max_torque"), &VehicleEngineSettings::get_max_torque);
	ClassDB::bind_method(D_METHOD("set_max_torque", "max_torque"), &VehicleEngineSettings::set_max_torque);

	ClassDB::bind_method(D_METHOD("get_max_rpm"), &VehicleEngineSettings::get_max_rpm);
	ClassDB::bind_method(D_METHOD("set_max_rpm", "max_rpm"), &VehicleEngineSettings::set_max_rpm);

	ClassDB::bind_method(D_METHOD("get_min_rpm"), &VehicleEngineSettings::get_min_rpm);
	ClassDB::bind_method(D_METHOD("set_min_rpm", "min_rpm"), &VehicleEngineSettings::set_min_rpm);

	ClassDB::bind_method(D_METHOD("get_inertia"), &VehicleEngineSettings::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &VehicleEngineSettings::set_inertia);

	ClassDB::bind_method(D_METHOD("get_angular_damping"), &VehicleEngineSettings::get_angular_damping);
	ClassDB::bind_method(D_METHOD("set_angular_damping", "angular_damping"), &VehicleEngineSettings::set_angular_damping);

	ClassDB::bind_method(D_METHOD("get_torque_curve"), &VehicleEngineSettings::get_torque_curve);
	ClassDB::bind_method(D_METHOD("set_torque_curve", "torque_curve"), &VehicleEngineSettings::set_torque_curve);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_torque", PROPERTY_HINT_RANGE, "0,5000,0.1,or_greater,suffix:Nm"), "set_max_torque", "get_max_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_rpm", PROPERTY_HINT_RANGE, "0,10000,0.1,or_greater,suffix:RPM"), "set_max_rpm", "get_max_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_rpm", PROPERTY_HINT_RANGE, "0,5000,0.1,or_greater,suffix:RPM"), "set_min_rpm", "get_min_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, U"0,1000,0.01,or_greater,exp,suffix:kg\u22C5m\u00B2"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_angular_damping", "get_angular_damping");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "torque_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_torque_curve", "get_torque_curve");
}

