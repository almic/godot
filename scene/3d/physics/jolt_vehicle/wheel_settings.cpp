#include "wheel_settings.h"

void WheelBaseSettings::_bind_methods() {

	ClassDB::bind_method(D_METHOD("get_position"), &WheelBaseSettings::get_position);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &WheelBaseSettings::set_position);

	ClassDB::bind_method(D_METHOD("get_suspension_force_point"), &WheelBaseSettings::get_suspension_force_point);
	ClassDB::bind_method(D_METHOD("set_suspension_force_point", "force_point"), &WheelBaseSettings::set_suspension_force_point);

	ClassDB::bind_method(D_METHOD("get_suspension_direction"), &WheelBaseSettings::get_suspension_direction);
	ClassDB::bind_method(D_METHOD("set_suspension_direction", "direction"), &WheelBaseSettings::set_suspension_direction);

	ClassDB::bind_method(D_METHOD("get_steering_axis"), &WheelBaseSettings::get_steering_axis);
	ClassDB::bind_method(D_METHOD("set_steering_axis", "steering_axis"), &WheelBaseSettings::set_steering_axis);

	ClassDB::bind_method(D_METHOD("get_up"), &WheelBaseSettings::get_up);
	ClassDB::bind_method(D_METHOD("set_up", "up"), &WheelBaseSettings::set_up);

	ClassDB::bind_method(D_METHOD("get_forward"), &WheelBaseSettings::get_forward);
	ClassDB::bind_method(D_METHOD("set_forward", "forward"), &WheelBaseSettings::set_forward);

	ClassDB::bind_method(D_METHOD("get_suspension_max_length"), &WheelBaseSettings::get_suspension_max_length);
	ClassDB::bind_method(D_METHOD("set_suspension_max_length", "max_length"), &WheelBaseSettings::set_suspension_max_length);

	ClassDB::bind_method(D_METHOD("get_suspension_min_length"), &WheelBaseSettings::get_suspension_min_length);
	ClassDB::bind_method(D_METHOD("set_suspension_min_length", "min_length"), &WheelBaseSettings::set_suspension_min_length);

	ClassDB::bind_method(D_METHOD("get_suspension_preload_length"), &WheelBaseSettings::get_suspension_preload_length);
	ClassDB::bind_method(D_METHOD("set_suspension_preload_length", "preload_length"), &WheelBaseSettings::set_suspension_preload_length);

	ClassDB::bind_method(D_METHOD("get_spring_mode"), &WheelBaseSettings::get_spring_mode);
	ClassDB::bind_method(D_METHOD("set_spring_mode", "mode"), &WheelBaseSettings::set_spring_mode);

	ClassDB::bind_method(D_METHOD("get_spring_frequency"), &WheelBaseSettings::get_spring_frequency);
	ClassDB::bind_method(D_METHOD("set_spring_frequency", "frequency"), &WheelBaseSettings::set_spring_frequency);

	ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &WheelBaseSettings::get_spring_stiffness);
	ClassDB::bind_method(D_METHOD("set_spring_stiffness", "stiffness"), &WheelBaseSettings::set_spring_stiffness);

	ClassDB::bind_method(D_METHOD("get_spring_damping"), &WheelBaseSettings::get_spring_damping);
	ClassDB::bind_method(D_METHOD("set_spring_damping", "damping"), &WheelBaseSettings::set_spring_damping);

	ClassDB::bind_method(D_METHOD("get_radius"), &WheelBaseSettings::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &WheelBaseSettings::set_radius);

	ClassDB::bind_method(D_METHOD("get_width"), &WheelBaseSettings::get_width);
	ClassDB::bind_method(D_METHOD("set_width", "width"), &WheelBaseSettings::set_width);

	ClassDB::bind_method(D_METHOD("get_enable_suspension_force_point"), &WheelBaseSettings::get_enable_suspension_force_point);
	ClassDB::bind_method(D_METHOD("set_enable_suspension_force_point", "enable"), &WheelBaseSettings::set_enable_suspension_force_point);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position", PROPERTY_HINT_RANGE, "-10,10,0.001,or_less,or_greater,suffix:m"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_width", "get_width");

	ADD_GROUP("Steering", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "steering_axis"), "set_steering_axis", "get_steering_axis");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "forward"), "set_forward", "get_forward");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up"), "set_up", "get_up");

	ADD_GROUP("Suspension", "suspension");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_max_length", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:m"), "set_suspension_max_length", "get_suspension_max_length"),
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_min_length", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:m"), "set_suspension_min_length", "get_suspension_min_length"),
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_preload_length", PROPERTY_HINT_RANGE, "-5,5,0.001,or_less,or_greater,suffix:m"), "set_suspension_preload_length", "get_suspension_preload_length"),
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "suspension_direction"), "set_suspension_direction", "get_suspension_direction");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "suspension_enable_force_point"), "set_enable_suspension_force_point", "get_enable_suspension_force_point");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "suspension_force_point"), "set_suspension_force_point", "get_suspension_force_point");

	ADD_SUBGROUP("Spring", "spring");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "spring_mode", PROPERTY_HINT_ENUM, "Frequency,Stiffness"), "set_spring_mode", "get_spring_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_frequency", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:Hz"), "set_spring_frequency", "get_spring_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_stiffness", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater,suffix:N/mm"), "set_spring_stiffness", "get_spring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_damping", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_spring_damping", "get_spring_damping");

	BIND_ENUM_CONSTANT(SPRING_MODE_FREQUENCY);
	BIND_ENUM_CONSTANT(SPRING_MODE_STIFFNESS);
}

void WheelBaseSettings::_validate_property(PropertyInfo &p_property) const
{
	if (!get_enable_suspension_force_point() && p_property.name == "suspension_enable_force_point") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	} else if (p_property.name == "spring_stiffness") {
		if (get_spring_mode() == SPRING_MODE_FREQUENCY) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	} else if (p_property.name == "spring_frequency") {
		if (get_spring_mode() == SPRING_MODE_STIFFNESS) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	}
}

void WheelSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_inertia"), &WheelSettings::get_inertia);
	ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &WheelSettings::set_inertia);

	ClassDB::bind_method(D_METHOD("get_angular_damping"), &WheelSettings::get_angular_damping);
	ClassDB::bind_method(D_METHOD("set_angular_damping", "angular_damping"), &WheelSettings::set_angular_damping);

	ClassDB::bind_method(D_METHOD("get_max_steer_angle"), &WheelSettings::get_max_steer_angle);
	ClassDB::bind_method(D_METHOD("set_max_steer_angle", "max_steer_angle"), &WheelSettings::set_max_steer_angle);

	ClassDB::bind_method(D_METHOD("get_max_brake_torque"), &WheelSettings::get_max_brake_torque);
	ClassDB::bind_method(D_METHOD("set_max_brake_torque", "max_brake_torque"), &WheelSettings::set_max_brake_torque);

	ClassDB::bind_method(D_METHOD("get_max_hand_brake_torque"), &WheelSettings::get_max_hand_brake_torque);
	ClassDB::bind_method(D_METHOD("set_max_hand_brake_torque", "max_hand_brake_torque"), &WheelSettings::set_max_hand_brake_torque);

	ClassDB::bind_method(D_METHOD("get_lateral_friction"), &WheelSettings::get_lateral_friction);
	ClassDB::bind_method(D_METHOD("set_lateral_friction", "lateral_friction"), &WheelSettings::set_lateral_friction);

	ClassDB::bind_method(D_METHOD("get_longitudinal_friction"), &WheelSettings::get_longitudinal_friction);
	ClassDB::bind_method(D_METHOD("set_longitudinal_friction", "longitudinal_friction"), &WheelSettings::set_longitudinal_friction);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_steer_angle", PROPERTY_HINT_RANGE, "0,90,0.01,or_greater,radians_as_degrees"), "set_max_steer_angle", "get_max_steer_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_brake_torque", PROPERTY_HINT_RANGE, "0,3000,0.1,or_greater,suffix:Nm"), "set_max_brake_torque", "get_max_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_hand_brake_torque", PROPERTY_HINT_RANGE, "0,5000,0.1,or_greater,suffix:Nm"), "set_max_hand_brake_torque", "get_max_hand_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, U"0,5,0.001,or_greater,exp,suffix:kg\u22C5m\u00B2"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_angular_damping", "get_angular_damping");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lateral_friction", PROPERTY_HINT_RESOURCE_TYPE, "Curve", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_lateral_friction", "get_lateral_friction");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "longitudinal_friction", PROPERTY_HINT_RESOURCE_TYPE, "Curve", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_longitudinal_friction", "get_longitudinal_friction");
}

