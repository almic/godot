#pragma once

#include "core/io/resource.h"
#include "core/object/ref_counted.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/3d/shape_3d.h"
#include "scene/resources/material.h"
#include "scene/resources/mesh.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/SpringCastConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

class PhysicsBody3D;
class SpringCastSettings;
class JoltQueryFilter3D;
class JoltSpace3D;

class SpringCast : public Node3D {
	GDCLASS(SpringCast, Node3D);

	float rest_offset = 0.0;
	uint32_t collision_mask = 0;
	HashSet<RID> exclude;

	JPH::Ref<JPH::SpringCastConstraint> spring_cast;
	JPH::Ref<JPH::SpringCastCollisionTester> collision_tester;
	Ref<SpringCastSettings> settings;
	JPH::PhysicsSystem *physics_system = nullptr;
	JoltQueryFilter3D *collision_filter = nullptr;
	Callable pick_collisions_user;

	// using PickCollisionsFunction = Array<BodyID> (*)(void *inContext, const Array<Body *> &inBodyList, const Array<float> &inSpringFraction, const Array<RVec3> &inAverageContactPoint, const Array<Vec3> &inContactNormal);
	static JPH::Array<JPH::BodyID> _forward_pick_collisions(void *inContext, const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringFraction, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) {
		return static_cast<SpringCast *>(inContext)->_pick_collisions(inBodyList, inSpringFraction, inAverageContactPoint, inContactNormal);
	}

	JPH::Array<JPH::BodyID> _pick_collisions(const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringFraction, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) const;

	NodePath main_body;
	mutable RID body_rid;
	mutable PhysicsBody3D *phys_body = nullptr;

	PhysicsBody3D *_get_body() const;
	JoltSpace3D *_get_space() const;
	void _update_body() const;

	bool _transform_settings_locked = false;
	void _settings_changed();

	void _build_constraint();
	void _destroy_constraint();

	bool _auto_add_remove = true;
	bool _is_constraint_added = false;
	void _add_constraint();
	void _remove_constraint();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	SpringCast();
	~SpringCast();

	bool is_enabled() const;
	bool set_enabled(bool p_enabled);

	bool is_auto_add_enabled() const { return _auto_add_remove; }
	void set_auto_add_enabled(bool p_enabled) { _auto_add_remove = p_enabled; }

	bool is_constraint_added() const { return _is_constraint_added; }
	bool add_constraint();
	bool remove_constraint();

	NodePath get_main_body() const { return main_body; }
	void set_main_body(const NodePath &p_body);

	PhysicsBody3D *get_main_body_object() const { return _get_body(); }

	Vector3 get_forward_input() const;
	void set_forward_input(const Vector3 &p_forward_input);

	float get_forward_max_speed() const;
	void set_forward_max_speed(float p_forward_max_speed);

	float get_rest_offset() const { return rest_offset; }
	void set_rest_offset(float p_rest_offset) {
		rest_offset = p_rest_offset;
		if (spring_cast != nullptr) {
			spring_cast->SetSpringRestOffset(rest_offset);
		}
	}

	Ref<SpringCastSettings> get_settings() const { return settings; }
	void set_settings(const Ref<SpringCastSettings> &p_settings);

	Callable get_pick_collisions_function() const { return pick_collisions_user; }
	void set_pick_collisions_function(const Callable &p_pick_function);

	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_mask(uint32_t p_mask);

	bool get_collision_mask_value(int p_layer_number) const { return collision_mask & (1 << p_layer_number); }
	void set_collision_mask_value(int p_layer_number, bool p_value) {
		uint32_t new_mask = collision_mask;
		if (p_value) {
			new_mask |= (1 << p_layer_number);
		} else {
			new_mask &= (~(1 << p_layer_number));
		}
		set_collision_mask(new_mask);
	}

	bool is_colliding() const;

	int get_contact_body_count() const;
	PhysicsBody3D *get_contact_body(int p_body_index) const;
	RID get_contact_body_rid(int p_body_index) const;
	Vector3 get_contact_normal(int p_body_index) const;
	Vector3 get_contact_average_point(int p_body_index) const;

	int get_contact_shape_count(int p_body_index) const;
	int get_contact_shape(int p_body_index, int p_shape_index) const;

	int get_contact_point_count(int p_body_index = -1, int p_shape_index = -1) const;
	Vector3 get_contact_point(int p_idx, int p_body_index = -1, int p_shape_index = -1) const;

	Vector3 get_contact_friction(int p_body_index) const;

	float get_spring_length() const;
	bool has_hit_hard_point() const;

	float get_spring_lambda() const;
	float get_friction_lambda() const;
	float get_angular_lambda() const;

	void add_exception_rid(const RID &p_rid);
	void add_exception(RequiredParam<const PhysicsBody3D> rp_node);
	void remove_exception_rid(const RID &p_rid);
	void remove_exception(RequiredParam<const PhysicsBody3D> rp_node);
	void clear_exceptions();

	const Color &get_debug_custom_color() const { return debug_custom_color; }
	void set_debug_custom_color(const Color &p_color) {
		debug_custom_color = p_color;
		if (debug_material.is_valid()) {
			_update_debug_shape_material();
		}
	}

	Ref<StandardMaterial3D> get_debug_material() {
		_update_debug_shape_material();
		return debug_material;
	}

	// NOTE: only for the editor gizmo, don't expose this to user
	Ref<ArrayMesh> get_debug_mesh() const { return debug_mesh; }

	virtual PackedStringArray get_configuration_warnings() const override;

private:
	Ref<Material> debug_material;
	Color debug_custom_color = Color(0.0, 0.0, 0.0, 0.0);
	Vector<Vector3> debug_shape_vertices;
	Vector<Vector3> debug_line_vertices;

	RID debug_instance;
	Ref<ArrayMesh> debug_mesh;

	void _create_debug_shape();
	void _clear_debug_shape();
	void _update_debug_shape();
	void _update_debug_shape_material();
	void _update_debug_shape_vertices();
};

class SpringCastSettings : public Resource {
	GDCLASS(SpringCastSettings, Resource);

protected:
	friend class SpringCast;
	JPH::SpringCastConstraintSettings settings;

	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

public:
	enum SpringMode {
		SPRING_MODE_FREQUENCY = int(JPH::ESpringMode::FrequencyAndDamping),
		SPRING_MODE_STIFFNESS = int(JPH::ESpringMode::StiffnessAndDamping),
		SPRING_MODE_MASS_NORMALIZED = int(JPH::ESpringMode::MassNormalizedStiffnessAndDamping),
	};

	SpringCastSettings() {
		// Settings is not actually alloc'd, mark as embedded to prevent release from doing anything
		settings.SetEmbedded();
		settings.AddRef();
	}

	~SpringCastSettings() {
		settings.Release();
	}

	Vector3 get_position() const { return to_godot(settings.mPosition); }
	void set_position(Vector3 p_position) {
		settings.mPosition = to_jolt(p_position);
		emit_changed();
	}

	bool is_enable_force_point() const { return settings.mEnableForcePoint; }
	void set_enable_force_point(bool p_enable) {
		settings.mEnableForcePoint = p_enable;
		emit_changed();
	}

	Vector3 get_force_point() const { return to_godot(settings.mForcePoint); }
	void set_force_point(Vector3 p_force_point) {
		settings.mForcePoint = to_jolt(p_force_point);
		emit_changed();
	}

	Vector3 get_direction() const { return to_godot(settings.mDirection); }
	void set_direction(Vector3 p_direction) {
		settings.mDirection = to_jolt(p_direction);
		emit_changed();
	}

	float get_radius() const { return settings.mRadius; }
	void set_radius(float p_radius) {
		settings.mRadius = p_radius;
		emit_changed();
	}

	float get_max_angle() const { return settings.mMaxAngle; }
	void set_max_angle(float p_max_angle) {
		settings.mMaxAngle = p_max_angle;
		emit_changed();
	}

	float get_min_length() const { return settings.mMinLength; }
	void set_min_length(float p_min_length) {
		settings.mMinLength = p_min_length;
		emit_changed();
	}

	float get_max_length() const { return settings.mMaxLength; }
	void set_max_length(float p_max_length) {
		settings.mMaxLength = p_max_length;
		emit_changed();
	}

	int get_max_bodies() const { return settings.mMaxBodyCount; }
	void set_max_bodies(uint p_max_bodies) {
		settings.mMaxBodyCount = p_max_bodies;
		emit_changed();
	}

	SpringMode get_spring_mode() const { return SpringMode(settings.mSpringSettings.mMode); }
	void set_spring_mode(SpringMode p_mode) {
		settings.mSpringSettings.mMode = JPH::ESpringMode(p_mode);
		notify_property_list_changed();
		emit_changed();
	}

	float get_spring_stiffness() const {
		if (settings.mSpringSettings.mMode == JPH::ESpringMode::FrequencyAndDamping) {
			return settings.mSpringSettings.mFrequency;
		}
		return settings.mSpringSettings.mStiffness;
	}
	void set_spring_stiffness(float p_stiffness) {
		if (settings.mSpringSettings.mMode == JPH::ESpringMode::FrequencyAndDamping) {
			settings.mSpringSettings.mFrequency = p_stiffness;
		} else {
			settings.mSpringSettings.mStiffness = p_stiffness;
		}
		emit_changed();
	}

	float get_spring_damping() const { return settings.mSpringSettings.mDamping; }
	void set_spring_damping(float p_damping) {
		settings.mSpringSettings.mDamping = p_damping;
		emit_changed();
	}
};

VARIANT_ENUM_CAST(SpringCastSettings::SpringMode);
