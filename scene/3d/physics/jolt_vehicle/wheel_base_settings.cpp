#include "wheel_base_settings.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "Jolt/Physics/Constraints/SpringSettings.h"

Vector3 WheelBaseSettings::get_position() const { return to_godot(settings.mPosition); }
void WheelBaseSettings::set_position(const Vector3 &p_position) { settings.mPosition = to_jolt(p_position); }

Vector3 WheelBaseSettings::get_suspension_force_point() const { return to_godot(settings.mSuspensionForcePoint); }
void WheelBaseSettings::set_suspension_force_point(const Vector3 &p_force_point) { settings.mSuspensionForcePoint = to_jolt(p_force_point); }

Vector3 WheelBaseSettings::get_suspension_direction() const { return to_godot(settings.mSuspensionDirection); }
void WheelBaseSettings::set_suspension_direction(const Vector3 &p_direction) { settings.mSuspensionDirection = to_jolt(p_direction.normalized()); }

Vector3 WheelBaseSettings::get_steering_axis() const { return to_godot(settings.mSteeringAxis); }
void WheelBaseSettings::set_steering_axis(const Vector3 &p_axis) { settings.mSteeringAxis = to_jolt(p_axis.normalized()); }

Vector3 WheelBaseSettings::get_up() const { return to_godot(settings.mWheelUp); }
void WheelBaseSettings::set_up(const Vector3 &p_up) { settings.mWheelUp = to_jolt(p_up.normalized()); }

Vector3 WheelBaseSettings::get_forward() const { return to_godot(settings.mWheelForward); }
void WheelBaseSettings::set_forward(const Vector3 &p_forward) { settings.mWheelForward = to_jolt(p_forward.normalized()); }

float WheelBaseSettings::get_suspension_max_length() const { return settings.mSuspensionMaxLength; }
void WheelBaseSettings::set_suspension_max_length(float p_max_length) { settings.mSuspensionMaxLength = p_max_length; }

float WheelBaseSettings::get_suspension_min_length() const { return settings.mSuspensionMinLength; }
void WheelBaseSettings::set_suspension_min_length(float p_min_length) { settings.mSuspensionMinLength = p_min_length; }

float WheelBaseSettings::get_suspension_preload_length() const { return settings.mSuspensionPreloadLength; }
void WheelBaseSettings::set_suspension_preload_length(float p_preload_length) { settings.mSuspensionPreloadLength = p_preload_length; }

WheelBaseSettings::SpringMode WheelBaseSettings::get_spring_mode() const { return (SpringMode) settings.mSuspensionSpring.mMode; }
void WheelBaseSettings::set_spring_mode(SpringMode p_mode) {
	settings.mSuspensionSpring.mMode = (JPH::ESpringMode) p_mode;
	notify_property_list_changed();
}

float WheelBaseSettings::get_spring_frequency() const {
	ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, 0, "SpringMode is not based on frequency, use get_spring_stiffness() instead!");
	return settings.mSuspensionSpring.mFrequency;
}
void WheelBaseSettings::set_spring_frequency(float p_frequency) {
	ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_FREQUENCY, "SpringMode is not based on frequency, change the mode to frequency, or use set_spring_stiffness() instead!");
	settings.mSuspensionSpring.mFrequency = p_frequency;
}

float WheelBaseSettings::get_spring_stiffness() const {
	ERR_FAIL_COND_V_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, 0, "SpringMode is not based on stiffness, use get_spring_frequency() instead!");
	return settings.mSuspensionSpring.mStiffness * 0.001;
}
void WheelBaseSettings::set_spring_stiffness(float p_stiffness) {
	ERR_FAIL_COND_MSG(get_spring_mode() != SpringMode::SPRING_MODE_STIFFNESS, "SpringMode is not based on stiffness, change the mode to stiffness, or use set_spring_frequency() instead!");
	settings.mSuspensionSpring.mStiffness = p_stiffness * 1000.0;
}

float WheelBaseSettings::get_spring_damping() const { return settings.mSuspensionSpring.mDamping; }
void WheelBaseSettings::set_spring_damping(float p_damping) { settings.mSuspensionSpring.mDamping = p_damping; }

float WheelBaseSettings::get_radius() const { return settings.mRadius; }
void WheelBaseSettings::set_radius(float p_radius) { settings.mRadius = p_radius; }

float WheelBaseSettings::get_width() const { return settings.mWidth; }
void WheelBaseSettings::set_width(float p_width) { settings.mWidth = p_width; }

bool WheelBaseSettings::get_enable_suspension_force_point() const { return settings.mEnableSuspensionForcePoint; }
void WheelBaseSettings::set_enable_suspension_force_point(bool p_enable) {
	settings.mEnableSuspensionForcePoint = p_enable;
	notify_property_list_changed();
}

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

