#pragma once

#include "editor/scene/3d/node_3d_editor_gizmos.h"

class SpringCastGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(SpringCastGizmoPlugin, EditorNode3DGizmoPlugin);

public:
	SpringCastGizmoPlugin();

	bool has_gizmo(Node3D *p_spatial) override;

	String get_gizmo_name() const override { return "SpringCast"; }

	int get_priority() const override { return -1; }

	void redraw(EditorNode3DGizmo *p_gizmo) override;
};
