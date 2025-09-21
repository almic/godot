#include "anti_roll_bar.h"

void AntiRollBarSettings::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_left_wheel_index"), &AntiRollBarSettings::get_left_wheel_index);
	ClassDB::bind_method(D_METHOD("set_left_wheel_index", "left_index"), &AntiRollBarSettings::set_left_wheel_index);

	ClassDB::bind_method(D_METHOD("get_right_wheel_index"), &AntiRollBarSettings::get_right_wheel_index);
	ClassDB::bind_method(D_METHOD("set_right_wheel_index", "right_index"), &AntiRollBarSettings::set_right_wheel_index);

	ClassDB::bind_method(D_METHOD("get_stiffness"), &AntiRollBarSettings::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &AntiRollBarSettings::set_stiffness);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "left_wheel", PROPERTY_HINT_RANGE, "0,4,1,or_greater"), "set_left_wheel_index", "get_left_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "right_wheel", PROPERTY_HINT_RANGE, "0,4,1,or_greater"), "set_right_wheel_index", "get_right_wheel_index");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:N/mm"), "set_stiffness", "get_stiffness");
}

