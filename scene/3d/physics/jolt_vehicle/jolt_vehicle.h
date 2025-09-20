#pragma once

#include "scene/3d/physics/rigid_body_3d.h"


class JoltVehicle : public RigidBody3D {
	GDCLASS(JoltVehicle, RigidBody3D);



	static void _bind_methods();

	virtual void _body_state_changed(PhysicsDirectBodyState3D *p_state) override;
}

