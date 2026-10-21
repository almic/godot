#include "wheel_settings.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

void WheelBaseSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_position"), &WheelBaseSettings::get_position);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &WheelBaseSettings::set_position);

	ClassDB::bind_method(D_METHOD("get_suspension_force_point"), &WheelBaseSettings::get_suspension_force_point);
	ClassDB::bind_method(D_METHOD("set_suspension_force_point", "force_point"), &WheelBaseSettings::set_suspension_force_point);

	ClassDB::bind_method(D_METHOD("get_suspension_direction"), &WheelBaseSettings::get_suspension_direction);

	ClassDB::bind_method(D_METHOD("get_suspension_forward_angle"), &WheelBaseSettings::get_suspension_forward_angle);
	ClassDB::bind_method(D_METHOD("set_suspension_forward_angle", "forward_angle"), &WheelBaseSettings::set_suspension_forward_angle);

	ClassDB::bind_method(D_METHOD("get_suspension_sideways_angle"), &WheelBaseSettings::get_suspension_sideways_angle);
	ClassDB::bind_method(D_METHOD("set_suspension_sideways_angle", "sideways_angle"), &WheelBaseSettings::set_suspension_sideways_angle);

	ClassDB::bind_method(D_METHOD("get_steering_axis"), &WheelBaseSettings::get_steering_axis);

	ClassDB::bind_method(D_METHOD("get_up"), &WheelBaseSettings::get_up);

	ClassDB::bind_method(D_METHOD("get_forward"), &WheelBaseSettings::get_forward);

	ClassDB::bind_method(D_METHOD("get_kingpin_angle"), &WheelBaseSettings::get_kingpin_angle);
	ClassDB::bind_method(D_METHOD("set_kingpin_angle", "kingpin_angle"), &WheelBaseSettings::set_kingpin_angle);

	ClassDB::bind_method(D_METHOD("get_caster_angle"), &WheelBaseSettings::get_caster_angle);
	ClassDB::bind_method(D_METHOD("set_caster_angle", "caster_angle"), &WheelBaseSettings::set_caster_angle);

	ClassDB::bind_method(D_METHOD("get_camber"), &WheelBaseSettings::get_camber);
	ClassDB::bind_method(D_METHOD("set_camber", "camber"), &WheelBaseSettings::set_camber);

	ClassDB::bind_method(D_METHOD("get_toe"), &WheelBaseSettings::get_toe);
	ClassDB::bind_method(D_METHOD("set_toe", "toe"), &WheelBaseSettings::set_toe);

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

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position", PROPERTY_HINT_RANGE, "-10,10,0.0001,or_less,or_greater,suffix:m"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater,suffix:m"), "set_width", "get_width");

	ADD_GROUP("Steering", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "kingpin_angle", PROPERTY_HINT_RANGE, "-89.99,89.99,0.01,radians_as_degrees"), "set_kingpin_angle", "get_kingpin_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "caster_angle", PROPERTY_HINT_RANGE, "-89.99,89.99,0.01,radians_as_degrees"), "set_caster_angle", "get_caster_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "toe", PROPERTY_HINT_RANGE, "-5,5,0.01,radians_as_degrees,or_greater,or_less"), "set_toe", "get_toe");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber", PROPERTY_HINT_RANGE, "-5,5,0.01,radians_as_degrees,or_greater,or_less"), "set_camber", "get_camber");

	ADD_GROUP("Suspension", "suspension");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_max_length", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:m"), "set_suspension_max_length", "get_suspension_max_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_min_length", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:m"), "set_suspension_min_length", "get_suspension_min_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_preload_length", PROPERTY_HINT_RANGE, "-5,5,0.001,or_less,or_greater,suffix:m"), "set_suspension_preload_length", "get_suspension_preload_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_forward_angle", PROPERTY_HINT_RANGE, "-89.99,89.99,0.01,radians_as_degrees"), "set_suspension_forward_angle", "get_suspension_forward_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_sideways_angle", PROPERTY_HINT_RANGE, "-89.99,89.99,0.01,radians_as_degrees"), "set_suspension_sideways_angle", "get_suspension_sideways_angle");
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

void WheelSettings::_bind_methods() {
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

void WheelBaseSettings::_validate_property(PropertyInfo &p_property) const {
	if (!get_enable_suspension_force_point() && p_property.name == "suspension_force_point") {
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

Vector3 WheelBaseSettings::get_position() const {
	return to_godot(get_settings()->mPosition);
}

void WheelBaseSettings::set_position(const Vector3 &p_position) {
	get_settings()->mPosition = to_jolt(p_position);
	emit_changed();
}

Vector3 WheelBaseSettings::get_suspension_force_point() const {
	return to_godot(get_settings()->mSuspensionForcePoint);
}

void WheelBaseSettings::set_suspension_force_point(const Vector3 &p_force_point) {
	get_settings()->mSuspensionForcePoint = to_jolt(p_force_point);
}

Vector3 WheelBaseSettings::get_suspension_direction() const {
	return to_godot(get_settings()->mSuspensionDirection);
}

void WheelBaseSettings::set_suspension_forward_angle(float p_angle) {
	suspension_forward_angle = p_angle;
	get_settings()->mSuspensionDirection = to_jolt(Vector3(tan(suspension_sideways_angle), -1, tan(suspension_forward_angle)).normalized());
	emit_changed();
}

void WheelBaseSettings::set_suspension_sideways_angle(float p_angle) {
	suspension_sideways_angle = p_angle;
	get_settings()->mSuspensionDirection = to_jolt(Vector3(tan(suspension_sideways_angle), -1, tan(suspension_forward_angle)).normalized());
	emit_changed();
}

Vector3 WheelBaseSettings::get_steering_axis() const {
	return to_godot(get_settings()->mSteeringAxis);
}

Vector3 WheelBaseSettings::get_up() const {
	return to_godot(get_settings()->mWheelUp);
}

Vector3 WheelBaseSettings::get_forward() const {
	return to_godot(get_settings()->mWheelForward);
}

void WheelBaseSettings::set_kingpin_angle(float p_angle) {
	kingpin_angle = p_angle;
	get_settings()->mSteeringAxis = to_jolt(Vector3(-tan(kingpin_angle), 1, -tan(caster_angle)).normalized());
	emit_changed();
}

void WheelBaseSettings::set_caster_angle(float p_angle) {
	caster_angle = p_angle;
	get_settings()->mSteeringAxis = to_jolt(Vector3(-tan(kingpin_angle), 1, -tan(caster_angle)).normalized());
	emit_changed();
}

void WheelBaseSettings::set_camber(float p_camber) {
	camber = p_camber;
	get_settings()->mWheelUp = to_jolt(Vector3(sin(camber), cos(camber), 0));
	emit_changed();
}

void WheelBaseSettings::set_toe(float p_toe) {
	toe = p_toe;
	get_settings()->mWheelForward = to_jolt(Vector3(-sin(toe), 0, cos(toe)));
	emit_changed();
}

void WheelBaseSettings::set_spring_mode(SpringMode p_mode) {
	frequency = get_spring_frequency();
	stiffness = get_spring_stiffness();

	get_settings()->mSuspensionSpring.mMode = (JPH::ESpringMode)p_mode;

	set_spring_frequency(frequency);
	set_spring_stiffness(stiffness);

	notify_property_list_changed();
}

float WheelBaseSettings::get_spring_frequency() const {
	if (get_spring_mode() == SpringMode::SPRING_MODE_FREQUENCY) {
		return get_settings()->mSuspensionSpring.mFrequency;
	} else {
		return frequency;
	}
}

void WheelBaseSettings::set_spring_frequency(float p_frequency) {
	frequency = p_frequency;
	if (get_spring_mode() == SpringMode::SPRING_MODE_FREQUENCY) {
		get_settings()->mSuspensionSpring.mFrequency = frequency;
	}
}

float WheelBaseSettings::get_spring_stiffness() const {
	if (get_spring_mode() == SpringMode::SPRING_MODE_STIFFNESS) {
		return get_settings()->mSuspensionSpring.mStiffness * 0.001;
	} else {
		return stiffness;
	}
}

void WheelBaseSettings::set_spring_stiffness(float p_stiffness) {
	stiffness = p_stiffness;
	if (get_spring_mode() == SpringMode::SPRING_MODE_STIFFNESS) {
		get_settings()->mSuspensionSpring.mStiffness = stiffness * 1000.0;
	}
}

void WheelSettings::set_lateral_friction(const Ref<Curve> &p_lateral_friction) {
	if (lateral_friction.is_valid()) {
		lateral_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
	}

	lateral_friction = p_lateral_friction;

	if (lateral_friction.is_valid() && (!is_lateral_first_set || p_lateral_friction->get_point_count() > 0)) {
		lateral_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
		_apply_lateral_friction();
	} else {
		// Reset to default
		JPH::WheelSettingsWV defaults;
		settings.mLateralFriction = defaults.mLateralFriction;

		lateral_friction = memnew(Curve);
		to_godot(settings.mLateralFriction, *lateral_friction.ptr());
		lateral_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_lateral_friction));
	}

	is_lateral_first_set = false;
}

void WheelSettings::set_longitudinal_friction(const Ref<Curve> &p_longitudinal_friction) {
	if (longitudinal_friction.is_valid()) {
		longitudinal_friction->disconnect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
	}

	longitudinal_friction = p_longitudinal_friction;

	if (longitudinal_friction.is_valid() && (!is_longitudinal_first_set || p_longitudinal_friction->get_point_count() > 0)) {
		longitudinal_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
		_apply_longitudinal_friction();
	} else {
		// Reset to default
		JPH::WheelSettingsWV defaults;
		settings.mLongitudinalFriction = defaults.mLongitudinalFriction;

		longitudinal_friction = memnew(Curve);
		to_godot(settings.mLongitudinalFriction, *longitudinal_friction.ptr());
		longitudinal_friction->connect_changed(callable_mp(this, &WheelSettings::_apply_longitudinal_friction));
	}

	is_longitudinal_first_set = false;
}

void WheelSettings::_apply_lateral_friction() {
	settings.mLateralFriction = to_jolt(*lateral_friction.ptr());
}

void WheelSettings::_apply_longitudinal_friction() {
	settings.mLongitudinalFriction = to_jolt(*longitudinal_friction.ptr());
}
