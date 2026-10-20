extends Node3D
class_name Visualizer3D

## Renderizador 3D procedural de geometría original/deformada, malla, numeración de nodos,
## apoyos, cargas puntuales/distribuidas, reacciones y mapas de colores (desplazamientos y esfuerzos).

signal node_selected(node_id: int)
signal element_selected(element_id: int)

enum ColorMapMode {
	VON_MISES_STRESS = 0,
	DISPLACEMENT_MAG = 1,
	AXIAL_STRESS = 2
}

var show_original: bool = true
var show_deformed: bool = true
var show_node_labels: bool = true
var show_loads: bool = true
var show_reactions: bool = true
var deform_scale: float = 100.0
var colormap_mode: int = ColorMapMode.VON_MISES_STRESS

var selected_node_id: int = -1
var selected_element_id: int = -1

var _grid_root: Node3D
var _scene_root: Node3D


func _ready() -> void:
	_grid_root = Node3D.new()
	_grid_root.name = "ReferenceGrid"
	add_child(_grid_root)

	_scene_root = Node3D.new()
	_scene_root.name = "StructuralEntities"
	add_child(_scene_root)

	_build_reference_grid()


func render_structure(model: Dictionary, result: Dictionary) -> Dictionary:
	if _scene_root == null:
		_ready()
	for child in _scene_root.get_children():
		_scene_root.remove_child(child)
		child.queue_free()

	var nodes: Array = model.get("nodes", [])
	var elements: Array = model.get("elements", [])
	var bcs: Array = model.get("boundary_conditions", [])
	var nloads: Array = model.get("nodal_loads", [])
	var dloads: Array = model.get("distributed_loads", [])

	var has_result := bool(result.get("success", false))
	var disp_map: Dictionary = result.get("nodal_displacements", {}) if has_result else {}
	var reac_map: Dictionary = result.get("nodal_reactions", {}) if has_result else {}
	var eres_map: Dictionary = result.get("element_results", {}) if has_result else {}

	var node_pos_orig := {}
	var node_pos_def := {}
	var node_disp_mag := {}
	var node_dict_map := {}

	var max_u := float(result.get("max_displacement", 0.0)) if has_result else 0.0
	var max_vm := float(result.get("max_von_mises", 0.0)) if has_result else 0.0
	var min_ax := float(result.get("min_axial_stress", 0.0)) if has_result else 0.0
	var max_ax := float(result.get("max_axial_stress", 0.0)) if has_result else 0.0

	for n in nodes:
		var nid := int(n["id"])
		node_dict_map[nid] = n
		var p0 := Vector3(float(n["x"]), float(n["y"]), float(n["z"]))
		node_pos_orig[nid] = p0
		var u_arr: Array = disp_map.get(str(nid), disp_map.get(nid, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
		var u_vec := Vector3(float(u_arr[0]), float(u_arr[1]), float(u_arr[2]))
		node_pos_def[nid] = p0 + u_vec * deform_scale
		node_disp_mag[nid] = u_vec.length()

	# Rango de la escala de colores activa
	var c_min := 0.0
	var c_max := 1.0
	var unit_str := "MPa"
	if colormap_mode == ColorMapMode.VON_MISES_STRESS:
		c_min = 0.0
		c_max = maxf(max_vm / 1e6, 0.001)
		unit_str = "MPa"
	elif colormap_mode == ColorMapMode.DISPLACEMENT_MAG:
		c_min = 0.0
		c_max = maxf(max_u * 1000.0, 0.0001)
		unit_str = "mm"
	elif colormap_mode == ColorMapMode.AXIAL_STRESS:
		c_min = min_ax / 1e6
		c_max = max_ax / 1e6
		if absf(c_max - c_min) < 1e-4:
			c_max = c_min + 1.0
		unit_str = "MPa"

	# 1. Elementos (geometría original y deformada)
	for e in elements:
		var eid := int(e["id"])
		var etype := String(e["type"])
		var enodes: Array = e["nodes"]
		var er: Dictionary = eres_map.get(str(eid), eres_map.get(eid, {}))
		var is_sel_elem := (eid == selected_element_id)

		var elem_color := Color(0.25, 0.72, 0.95)
		if has_result and not er.is_empty():
			var val := 0.0
			if colormap_mode == ColorMapMode.VON_MISES_STRESS:
				val = float(er.get("von_mises_stress", 0.0)) / 1e6
			elif colormap_mode == ColorMapMode.AXIAL_STRESS:
				val = float(er.get("axial_stress", 0.0)) / 1e6
			else:
				var sum_u := 0.0
				for nid_v in enodes:
					sum_u += float(node_disp_mag.get(int(nid_v), 0.0)) * 1000.0
				val = sum_u / float(max(enodes.size(), 1))
			elem_color = sample_scientific_colormap((val - c_min) / maxf(c_max - c_min, 1e-9))

		if is_sel_elem:
			elem_color = elem_color.lightened(0.35)

		if etype == "TRUSS_3D" or etype == "FRAME_3D":
			var n1 := int(enodes[0])
			var n2 := int(enodes[1])
			var p1_0: Vector3 = node_pos_orig[n1]
			var p2_0: Vector3 = node_pos_orig[n2]
			var radius := 0.032 if etype == "TRUSS_3D" else 0.048

			if show_original:
				var orig_col := Color(0.65, 0.72, 0.82, 0.32) if (show_deformed and has_result) else Color(0.38, 0.65, 0.92, 0.95)
				_create_bar_cylinder(p1_0, p2_0, radius * 0.75, orig_col, eid, not (show_deformed and has_result))

			if show_deformed and has_result:
				var p1_d: Vector3 = node_pos_def[n1]
				var p2_d: Vector3 = node_pos_def[n2]
				_create_bar_cylinder(p1_d, p2_d, radius, elem_color, eid, true)
				if show_node_labels:
					_create_label_3d(0.5 * (p1_d + p2_d) + Vector3(0, 0.09, 0), "E%d" % eid, Color(0.82, 0.92, 1.0), 14)
			elif show_node_labels:
				_create_label_3d(0.5 * (p1_0 + p2_0) + Vector3(0, 0.09, 0), "E%d" % eid, Color(0.82, 0.92, 1.0), 14)

		elif etype == "TRIANGLE_3D" and enodes.size() == 3:
			var pts0: Array[Vector3] = [node_pos_orig[int(enodes[0])], node_pos_orig[int(enodes[1])], node_pos_orig[int(enodes[2])]]
			var ptsd: Array[Vector3] = [node_pos_def[int(enodes[0])], node_pos_def[int(enodes[1])], node_pos_def[int(enodes[2])]]
			if show_original:
				_create_triangle_mesh(pts0, Color(0.6, 0.7, 0.8, 0.28), true)
			if show_deformed and has_result:
				var col_a := elem_color
				col_a.a = 0.82
				_create_triangle_mesh(ptsd, col_a, true)

		elif etype == "TETRA_4" and enodes.size() == 4:
			var pts0: Array[Vector3] = [
				node_pos_orig[int(enodes[0])], node_pos_orig[int(enodes[1])],
				node_pos_orig[int(enodes[2])], node_pos_orig[int(enodes[3])]
			]
			var ptsd: Array[Vector3] = [
				node_pos_def[int(enodes[0])], node_pos_def[int(enodes[1])],
				node_pos_def[int(enodes[2])], node_pos_def[int(enodes[3])]
			]
			if show_original:
				_create_tetra_mesh(pts0, Color(0.6, 0.7, 0.85, 0.18))
			if show_deformed and has_result:
				var col_t := elem_color
				col_t.a = 0.55
				_create_tetra_mesh(ptsd, col_t)

	# 2. Nodos y numeración
	for n in nodes:
		var nid := int(n["id"])
		var p_draw: Vector3 = node_pos_def[nid] if (show_deformed and has_result) else node_pos_orig[nid]
		var n_col := Color(1.0, 0.85, 0.2) if nid == selected_node_id else Color(0.92, 0.96, 1.0)
		var n_rad := 0.075 if nid == selected_node_id else 0.055
		_create_node_sphere(p_draw, n_rad, n_col, nid)

		if show_node_labels:
			var lbl_txt := "N%d" % nid
			if has_result:
				var umm := float(node_disp_mag[nid]) * 1000.0
				lbl_txt = "N%d (%.2f mm)" % [nid, umm]
			_create_label_3d(p_draw + Vector3(0.0, 0.13, 0.0), lbl_txt, Color(1.0, 0.96, 0.78), 16)

	# 3. Condiciones de apoyo (iconos 3D en posición original)
	for bc in bcs:
		var nid := int(bc["node_id"])
		if not node_pos_orig.has(nid):
			continue
		_create_support_symbol(node_pos_orig[nid], bc)

	# 4. Cargas puntuales y distribuidas
	if show_loads:
		for nl in nloads:
			var nid := int(nl["node_id"])
			if not node_pos_orig.has(nid):
				continue
			var f_vec := Vector3(float(nl.get("fx", 0.0)), float(nl.get("fy", 0.0)), float(nl.get("fz", 0.0)))
			if f_vec.length() > 1e-6:
				var p_target: Vector3 = node_pos_def[nid] if (show_deformed and has_result) else node_pos_orig[nid]
				_create_vector_arrow(
					p_target,
					f_vec.normalized(),
					0.85,
					Color(1.0, 0.32, 0.22),
					"F=%.1f kN" % (f_vec.length() / 1000.0),
					true
				)

		var elem_lookup := {}
		for e in elements:
			elem_lookup[int(e["id"])] = e
		for dl in dloads:
			var eid := int(dl["element_id"])
			if not elem_lookup.has(eid):
				continue
			var e: Dictionary = elem_lookup[eid]
			var enodes: Array = e["nodes"]
			if enodes.size() != 2:
				continue
			var q_vec := Vector3(float(dl.get("qx", 0.0)), float(dl.get("qy", 0.0)), float(dl.get("qz", 0.0)))
			if q_vec.length() <= 1e-6:
				continue
			var pa: Vector3 = node_pos_def[int(enodes[0])] if (show_deformed and has_result) else node_pos_orig[int(enodes[0])]
			var pb: Vector3 = node_pos_def[int(enodes[1])] if (show_deformed and has_result) else node_pos_orig[int(enodes[1])]
			for step in range(5):
				var t := float(step) / 4.0
				var pt := pa.lerp(pb, t)
				var lbl := ("q=%.1f kN/m" % (q_vec.length() / 1000.0)) if step == 2 else ""
				_create_vector_arrow(pt, q_vec.normalized(), 0.55, Color(1.0, 0.58, 0.15), lbl, true)

	# 5. Reacciones en los apoyos
	if show_reactions and has_result:
		for bc in bcs:
			var nid := int(bc["node_id"])
			if not node_pos_orig.has(nid):
				continue
			var r_arr: Array = reac_map.get(str(nid), reac_map.get(nid, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
			var r_vec := Vector3(float(r_arr[0]), float(r_arr[1]), float(r_arr[2]))
			if r_vec.length() > 1.0:
				_create_vector_arrow(
					node_pos_orig[nid],
					r_vec.normalized(),
					0.8,
					Color(0.18, 0.92, 0.52),
					"R=%.1f kN" % (r_vec.length() / 1000.0),
					false
				)

	return {"min": c_min, "max": c_max, "unit": unit_str}


static func sample_scientific_colormap(t_raw: float) -> Color:
	var t := clampf(t_raw, 0.0, 1.0)
	if t < 0.25:
		return Color(0.12, 0.35, 0.95).lerp(Color(0.05, 0.82, 0.92), t / 0.25)
	elif t < 0.5:
		return Color(0.05, 0.82, 0.92).lerp(Color(0.18, 0.86, 0.36), (t - 0.25) / 0.25)
	elif t < 0.75:
		return Color(0.18, 0.86, 0.36).lerp(Color(0.98, 0.82, 0.15), (t - 0.5) / 0.25)
	else:
		return Color(0.98, 0.82, 0.15).lerp(Color(0.95, 0.18, 0.18), (t - 0.75) / 0.25)


func _create_node_sphere(pos: Vector3, radius: float, color: Color, node_id: int) -> void:
	var mesh_inst := MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = radius
	sphere.height = radius * 2.0
	mesh_inst.mesh = sphere
	mesh_inst.material_override = _make_material(color)
	mesh_inst.position = pos

	var body := StaticBody3D.new()
	body.set_meta("node_id", node_id)
	var col := CollisionShape3D.new()
	var shape := SphereShape3D.new()
	shape.radius = radius * 1.4
	col.shape = shape
	body.add_child(col)
	mesh_inst.add_child(body)
	_scene_root.add_child(mesh_inst)


func _create_bar_cylinder(p1: Vector3, p2: Vector3, radius: float, color: Color, elem_id: int, pickable: bool) -> void:
	var dist := p1.distance_to(p2)
	if dist <= 1e-6:
		return
	var mesh_inst := MeshInstance3D.new()
	var cyl := CylinderMesh.new()
	cyl.top_radius = radius
	cyl.bottom_radius = radius
	cyl.height = dist
	mesh_inst.mesh = cyl
	mesh_inst.material_override = _make_material(color)

	var mid := 0.5 * (p1 + p2)
	var y_axis := (p2 - p1) / dist
	var ref := Vector3.FORWARD if absf(y_axis.dot(Vector3.UP)) > 0.98 else Vector3.UP
	var x_axis := y_axis.cross(ref).normalized()
	var z_axis := x_axis.cross(y_axis).normalized()
	mesh_inst.transform = Transform3D(Basis(x_axis, y_axis, z_axis), mid)

	if pickable:
		var body := StaticBody3D.new()
		body.set_meta("element_id", elem_id)
		var col := CollisionShape3D.new()
		var shape := CylinderShape3D.new()
		shape.radius = radius * 1.5
		shape.height = dist
		col.shape = shape
		body.add_child(col)
		mesh_inst.add_child(body)

	_scene_root.add_child(mesh_inst)


func _create_triangle_mesh(pts: Array[Vector3], color: Color, draw_edges: bool) -> void:
	var im := ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	var normal := (pts[1] - pts[0]).cross(pts[2] - pts[0]).normalized()
	im.surface_set_normal(normal)
	im.surface_add_vertex(pts[0])
	im.surface_add_vertex(pts[1])
	im.surface_add_vertex(pts[2])
	im.surface_end()

	var mi := MeshInstance3D.new()
	mi.mesh = im
	var mat := _make_material(color)
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mi.material_override = mat
	_scene_root.add_child(mi)

	if draw_edges:
		_create_polyline([pts[0], pts[1], pts[2], pts[0]], color.lightened(0.3))


func _create_tetra_mesh(pts: Array[Vector3], color: Color) -> void:
	var faces := [
		[pts[0], pts[1], pts[2]],
		[pts[0], pts[1], pts[3]],
		[pts[1], pts[2], pts[3]],
		[pts[0], pts[2], pts[3]]
	]
	for f in faces:
		var f_typed: Array[Vector3] = [f[0], f[1], f[2]]
		_create_triangle_mesh(f_typed, color, true)


func _create_support_symbol(pos: Vector3, bc: Dictionary) -> void:
	var ux := bool(bc.get("ux", false))
	var uy := bool(bc.get("uy", false))
	var uz := bool(bc.get("uz", false))
	var rx := bool(bc.get("rx", false))
	var ry := bool(bc.get("ry", false))
	var rz := bool(bc.get("rz", false))

	var mi := MeshInstance3D.new()
	if ux and uy and uz and rx and ry and rz:
		# Empotramiento perfecto: cubo magenta
		var box := BoxMesh.new()
		box.size = Vector3(0.18, 0.18, 0.18)
		mi.mesh = box
		mi.position = pos - Vector3(0, 0.10, 0)
		mi.material_override = _make_material(Color(0.86, 0.28, 0.78, 0.9))
	elif ux and uy and uz:
		# Articulación fija: cono/pirámide naranja
		var cone := CylinderMesh.new()
		cone.top_radius = 0.01
		cone.bottom_radius = 0.13
		cone.height = 0.18
		mi.mesh = cone
		mi.position = pos - Vector3(0, 0.11, 0)
		mi.material_override = _make_material(Color(0.95, 0.55, 0.15, 0.9))
	else:
		# Apoyo tipo rodillo: esfera achatada cian/verde
		var cyl := CylinderMesh.new()
		cyl.top_radius = 0.11
		cyl.bottom_radius = 0.11
		cyl.height = 0.08
		mi.mesh = cyl
		mi.position = pos - Vector3(0, 0.09, 0)
		mi.material_override = _make_material(Color(0.2, 0.85, 0.72, 0.9))
	_scene_root.add_child(mi)


func _create_vector_arrow(
	target_pos: Vector3,
	dir: Vector3,
	length: float,
	color: Color,
	label_text: String,
	point_towards_target: bool
) -> void:
	var n_dir := dir.normalized()
	var start_pos := (target_pos - n_dir * length) if point_towards_target else target_pos
	var tip_pos := target_pos if point_towards_target else (target_pos + n_dir * length)
	var shaft_end := tip_pos - n_dir * (length * 0.25)

	_create_bar_cylinder(start_pos, shaft_end, 0.018, color, -1, false)

	var head := MeshInstance3D.new()
	var cone := CylinderMesh.new()
	cone.top_radius = 0.0
	cone.bottom_radius = 0.055
	cone.height = length * 0.25
	head.mesh = cone
	head.material_override = _make_material(color)

	var ref := Vector3.FORWARD if absf(n_dir.dot(Vector3.UP)) > 0.98 else Vector3.UP
	var x_axis := n_dir.cross(ref).normalized()
	var z_axis := x_axis.cross(n_dir).normalized()
	head.transform = Transform3D(Basis(x_axis, n_dir, z_axis), tip_pos - n_dir * (length * 0.125))
	_scene_root.add_child(head)

	if label_text != "":
		var lbl_pos := start_pos + Vector3(0, 0.1, 0) if point_towards_target else (tip_pos + Vector3(0, 0.1, 0))
		_create_label_3d(lbl_pos, label_text, color.lightened(0.25), 15)


func _create_label_3d(pos: Vector3, text: String, color: Color, font_size: int = 16) -> void:
	var lbl := Label3D.new()
	lbl.text = text
	lbl.modulate = color
	lbl.font_size = font_size
	lbl.pixel_size = 0.0045
	lbl.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lbl.no_depth_test = true
	lbl.outline_size = 6
	lbl.outline_modulate = Color(0.03, 0.05, 0.09, 0.95)
	lbl.position = pos
	_scene_root.add_child(lbl)


func _create_polyline(points: Array[Vector3], color: Color, parent: Node3D = null) -> void:
	if points.size() < 2:
		return
	var im := ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in points:
		im.surface_add_vertex(p)
	im.surface_end()

	var mi := MeshInstance3D.new()
	mi.mesh = im
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = color
	mi.material_override = mat
	(parent if parent != null else _scene_root).add_child(mi)


func _make_material(color: Color) -> StandardMaterial3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.35
	mat.metallic = 0.15
	if color.a < 0.99:
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return mat


func _build_reference_grid() -> void:
	var im := ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_LINES)
	for i in range(-8, 9):
		var c := float(i) * 1.0
		im.surface_add_vertex(Vector3(c, 0.0, -8.0))
		im.surface_add_vertex(Vector3(c, 0.0, 8.0))
		im.surface_add_vertex(Vector3(-8.0, 0.0, c))
		im.surface_add_vertex(Vector3(8.0, 0.0, c))
	im.surface_end()

	var mi := MeshInstance3D.new()
	mi.mesh = im
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(0.22, 0.29, 0.38, 0.45)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mi.material_override = mat
	_grid_root.add_child(mi)

	# Ejes coordenados X (rojo), Y (verde), Z (azul)
	_create_polyline([Vector3.ZERO, Vector3(1.5, 0, 0)], Color(0.95, 0.25, 0.25), _grid_root)
	_create_polyline([Vector3.ZERO, Vector3(0, 1.5, 0)], Color(0.25, 0.95, 0.35), _grid_root)
	_create_polyline([Vector3.ZERO, Vector3(0, 0, 1.5)], Color(0.30, 0.55, 0.98), _grid_root)
