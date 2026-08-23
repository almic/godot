#pragma once

#include "core/io/resource.h"
#include "core/object/ref_counted.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/3d/shape_3d.h"
#include "scene/resources/material.h"

#include "modules/jolt_physics/misc/jolt_type_conversions.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/SpringCastConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

class PhysicsBody3D;
class SpringCastSettings;
class JoltQueryFilter3D;
class JoltPhysicsDirectSpaceState3D;

class SpringCast : public Node3D {
	GDCLASS(SpringCast, Node3D);

	uint32_t collision_mask = 0;
	HashSet<RID> exclude;

	Ref<Material> debug_material;
	Color debug_shape_custom_color = Color(0.0, 0.0, 0.0);
	Vector<Vector3> debug_shape_vertices;
	Vector<Vector3> debug_line_vertices;

	void _create_debug_shape();
	void _update_debug_shape();
	void _update_debug_shape_material();
	void _update_debug_shape_vertices();
	void _clear_debug_shape();

	RID debug_instance;
	Ref<ArrayMesh> debug_mesh;

	JPH::Ref<JPH::SpringCastConstraint> spring_cast;
	JPH::Ref<JPH::SpringCastCollisionTester> collision_tester;
	Ref<SpringCastSettings> settings;
	JPH::PhysicsSystem *physics_system;
	Ref<JoltQueryFilter3D> collision_filter;
	Callable pick_collisions_user;

	// using PickCollisionsFunction = Array<BodyID> (*)(void *inContext, const Array<Body *> &inBodyList, const Array<float> &inSpringLength, const Array<RVec3> &inAverageContactPoint, const Array<Vec3> &inContactNormal);
	static JPH::Array<JPH::BodyID> _forward_pick_collisions(void *inContext, const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringLength, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) {
		return static_cast<SpringCast *>(inContext)->_pick_collisions(inBodyList, inSpringLength, inAverageContactPoint, inContactNormal);
	}

	JPH::Array<JPH::BodyID> _pick_collisions(const JPH::Array<JPH::Body *> &inBodyList, const JPH::Array<float> &inSpringLength, const JPH::Array<JPH::RVec3> &inAverageContactPoint, const JPH::Array<JPH::Vec3> &inContactNormal) const;

	NodePath main_body;
	mutable RID body_rid;

	PhysicsBody3D *_get_body() const;
	JoltPhysicsDirectSpaceState3D *_get_direct_space() const;

	void _build_constraint();
	void _destroy_constraint();

	bool _auto_add_remove = true;
	bool _is_constraint_added = false;
	void _add_constraint();
	void _remove_constraint();

protected:
	void _notification(int p_what);
	void _update_shapecast_state();
	void _shape_changed();
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

	float get_spring_length() const;
	bool has_hit_hard_point() const;

	float get_spring_lamdba() const;
	float get_friction_lambda() const;
	float get_angular_lambda() const;

	void add_exception_rid(const RID &p_rid);
	void add_exception(RequiredParam<const PhysicsBody3D> rp_node);
	void remove_exception_rid(const RID &p_rid);
	void remove_exception(RequiredParam<const PhysicsBody3D> rp_node);
	void clear_exceptions();

	const Color &get_debug_shape_custom_color() const;
	void set_debug_shape_custom_color(const Color &p_color);

	const Vector<Vector3> &get_debug_shape_vertices() const;
	const Vector<Vector3> &get_debug_line_vertices() const;

	Ref<StandardMaterial3D> get_debug_material();

	virtual PackedStringArray get_configuration_warnings() const override;
};

class SpringCastSettings : public Resource {
protected:
	friend class SpringCast;
	JPH::SpringCastConstraintSettings settings;

public:
	enum SpringMode {
		SPRING_MODE_FREQUENCY = int(JPH::ESpringMode::FrequencyAndDamping),
		SPRING_MODE_STIFFNESS = int(JPH::ESpringMode::StiffnessAndDamping),
		SPRING_MODE_MASS_NORMALIZED = int(JPH::ESpringMode::MassNormalizedStiffnessAndDamping),
	};

	Vector3 get_position() const { return to_godot(settings.mPosition); }
	void set_position(Vector3 p_position) { settings.mPosition = to_jolt(p_position); }

	Vector3 get_force_point() const { return to_godot(settings.mForcePoint); }
	void set_force_point(Vector3 p_force_point) { settings.mForcePoint = to_jolt(p_force_point); }

	Vector3 get_direction() const { return to_godot(settings.mDirection); }
	void set_direction(Vector3 p_direction) { settings.mDirection = to_jolt(p_direction); }

	bool is_enable_force_point() const { return settings.mEnableForcePoint; }
	void set_enable_force_point(bool p_enable) { settings.mEnableForcePoint = p_enable; }

	float get_radius() const { return settings.mRadius; }
	void set_radius(float p_radius) { settings.mRadius = p_radius; }

	float get_max_angle() const { return settings.mMaxAngle; }
	void set_max_angle(float p_max_angle) { settings.mMaxAngle = p_max_angle; }

	float get_min_length() const { return settings.mMinLength; }
	void set_min_length(float p_min_length) { settings.mMinLength = p_min_length; }

	float get_max_length() const { return settings.mMaxLength; }
	void set_max_length(float p_max_length) { settings.mMaxLength = p_max_length; }

	int get_max_bodies() const { return settings.mMaxBodyCount; }
	void set_max_bodies(uint p_max_bodies) { settings.mMaxBodyCount = p_max_bodies; }

	SpringMode get_spring_mode() const { return SpringMode(settings.mSpringSettings.mMode); }
	void set_spring_mode(SpringMode p_mode) { settings.mSpringSettings.mMode = JPH::ESpringMode(p_mode); }

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
	}

	float get_spring_damping() const { return settings.mSpringSettings.mDamping; }
	void set_spring_damping(float p_damping) { settings.mSpringSettings.mDamping = p_damping; }
};

VARIANT_ENUM_CAST(SpringCastSettings::SpringMode);
