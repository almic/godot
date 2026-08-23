#include "spring_cast_gizmo_plugin.h"

#include "scene/3d/physics/spring_cast.h"
#include "scene/main/scene_tree.h"

SpringCastGizmoPlugin::SpringCastGizmoPlugin() {
	const Color gizmo_color = SceneTree::get_singleton()->get_debug_collisions_color();
	create_material("shape_material", gizmo_color);
	const float gizmo_value = gizmo_color.get_v();
	const Color gizmo_color_disabled = Color(gizmo_value, gizmo_value, gizmo_value, 0.65);
	create_material("shape_material_disabled", gizmo_color_disabled);
}

bool SpringCastGizmoPlugin::has_gizmo(Node3D *p_spatial) {
	return Object::cast_to<SpringCast>(p_spatial) != nullptr;
}

void SpringCastGizmoPlugin::redraw(EditorNode3DGizmo *p_gizmo) {
	SpringCast *spring_cast = Object::cast_to<SpringCast>(p_gizmo->get_node_3d());

	p_gizmo->clear();

	const Ref<StandardMaterial3D> material = spring_cast->is_enabled() ? spring_cast->get_debug_material() : get_material("shape_material_disabled");

	const Ref<ArrayMesh> mesh = spring_cast->get_debug_mesh();
	if (mesh.is_valid()) {
		p_gizmo->add_mesh(mesh, material);
	}
}
