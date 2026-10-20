extends Node3D

## Controlador Principal de Structura 3D (Godot 4).
## Integra el Editor Geométrico 3D, el Visor Estructural 3D, la Cámara Orbital,
## el Solucionador Numérico (Python/NumPy + GDScript Nativo) y el Exportador de Informes.

var current_model: Dictionary = {}
var current_result: Dictionary = {}

var camera: OrbitCamera3D
var visualizer: Visualizer3D

# Controles UI
var benchmark_selector: OptionButton
var backend_selector: OptionButton
var colormap_selector: OptionButton
var deform_slider: HSlider
var deform_badge_label: Label
var chk_original: CheckBox
var chk_deformed: CheckBox
var chk_labels: CheckBox
var chk_loads: CheckBox
var chk_reactions: CheckBox

# Controles Editor Geométrico
var lbl_selection_info: Label
var spn_node_x: SpinBox
var spn_node_y: SpinBox
var spn_node_z: SpinBox
var opt_elem_type: OptionButton
var spn_elem_n1: SpinBox
var spn_elem_n2: SpinBox
var spn_mat_e: SpinBox
var spn_mat_nu: SpinBox
var spn_sec_b: SpinBox
var spn_sec_h: SpinBox
var spn_bc_node: SpinBox
var opt_bc_preset: OptionButton
var chk_bc_ux: CheckBox
var chk_bc_uy: CheckBox
var chk_bc_uz: CheckBox
var chk_bc_rx: CheckBox
var chk_bc_ry: CheckBox
var chk_bc_rz: CheckBox
var spn_load_node: SpinBox
var spn_load_fx: SpinBox
var spn_load_fy: SpinBox
var spn_load_fz: SpinBox
var spn_dload_elem: SpinBox
var spn_dload_qy: SpinBox

# Controles Panel de Resultados
var legend_min_label: Label
var legend_mid_label: Label
var legend_max_label: Label
var results_richtext: RichTextLabel
var status_bar_label: Label

const BENCHMARK_FILES := [
	{"title": "01 — Barra Axial (Solución Exacta u = PL/EA)", "file": "res://benchmarks/01_axial_bar.structura.json"},
	{"title": "02 — Celosía Espacial 3D (Trípode 3 Barras)", "file": "res://benchmarks/02_space_truss_3d.structura.json"},
	{"title": "03 — Viga en Voladizo (Carga Puntual)", "file": "res://benchmarks/03_cantilever_beam.structura.json"},
	{"title": "04 — Viga Biapoyada (Carga Distribuida q)", "file": "res://benchmarks/04_simply_supported_beam.structura.json"},
	{"title": "05 — Pórtico Espacial 3D (Gravedad + Empuje)", "file": "res://benchmarks/05_space_portal_frame.structura.json"},
	{"title": "06 — Placa Triangular CST 3D (Patch Test)", "file": "res://benchmarks/06_triangle_plate_cst.structura.json"},
	{"title": "07 — Sólido Tetraédrico TETRA_4 (Patch Test 3D)", "file": "res://benchmarks/07_tetra4_solid_prism.structura.json"},
	{"title": "08 — Prueba Fallo: Apoyos Insuficientes (Mecanismo)", "file": "res://benchmarks/08_insufficient_supports.structura.json"},
	{"title": "09 — Prueba Fallo: Mecanismo Interno Colineal", "file": "res://benchmarks/09_collinear_mechanism.structura.json"}
]


func _ready() -> void:
	_setup_3d_environment()
	_setup_user_interface()
	# Cargar por defecto la primera estructura espacial de barras 3D (o barra axial)
	_load_benchmark_by_index(1)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT and mb.pressed:
			_handle_3d_pick(mb.position)


func _handle_3d_pick(screen_pos: Vector2) -> void:
	if camera == null:
		return
	var from := camera.project_ray_origin(screen_pos)
	var to := from + camera.project_ray_normal(screen_pos) * 200.0
	var space_state := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(from, to)
	var hit := space_state.intersect_ray(query)
	if not hit.is_empty() and hit.has("collider"):
		var collider: Object = hit["collider"]
		if collider.has_meta("node_id"):
			var nid := int(collider.get_meta("node_id"))
			visualizer.selected_node_id = nid
			visualizer.selected_element_id = -1
			_populate_inspector_for_node(nid)
			_refresh_3d_view()
		elif collider.has_meta("element_id"):
			var eid := int(collider.get_meta("element_id"))
			visualizer.selected_element_id = eid
			visualizer.selected_node_id = -1
			_populate_inspector_for_element(eid)
			_refresh_3d_view()


func _setup_3d_environment() -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.06, 0.09, 0.14)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.65, 0.72, 0.82)
	env.ambient_light_energy = 0.75

	var world_env := WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

	var dir_light := DirectionalLight3D.new()
	dir_light.rotation_degrees = Vector3(-48.0, -35.0, 0.0)
	dir_light.light_energy = 1.15
	dir_light.shadow_enabled = true
	add_child(dir_light)

	camera = OrbitCamera3D.new()
	add_child(camera)

	visualizer = Visualizer3D.new()
	add_child(visualizer)


func _setup_user_interface() -> void:
	var canvas := CanvasLayer.new()
	add_child(canvas)

	# 1. Barra superior
	var top_panel := PanelContainer.new()
	top_panel.set_anchors_preset(Control.PRESET_TOP_WIDE)
	top_panel.custom_minimum_size = Vector2(0, 84)
	canvas.add_child(top_panel)

	var top_vbox := VBoxContainer.new()
	top_panel.add_child(top_vbox)

	var row1 := HBoxContainer.new()
	row1.add_theme_constant_override("separation", 10)
	top_vbox.add_child(row1)

	var title_lbl := Label.new()
	title_lbl.text = "  STRUCTURA 3D"
	title_lbl.add_theme_color_override("font_color", Color(0.38, 0.82, 1.0))
	row1.add_child(title_lbl)

	benchmark_selector = OptionButton.new()
	for b in BENCHMARK_FILES:
		benchmark_selector.add_item(b["title"])
	benchmark_selector.item_selected.connect(_load_benchmark_by_index)
	row1.add_child(benchmark_selector)

	backend_selector = OptionButton.new()
	backend_selector.add_item("Motor Numérico: Python (NumPy/SciPy) + Fallback GDScript")
	backend_selector.add_item("Motor Numérico: GDScript Nativo (100% Interno)")
	row1.add_child(backend_selector)

	var btn_solve := Button.new()
	btn_solve.text = "Resolver Sistema (K·u = F)"
	btn_solve.pressed.connect(_run_solver)
	row1.add_child(btn_solve)

	var btn_export := Button.new()
	btn_export.text = "Exportar Informe (.md / .html)"
	btn_export.pressed.connect(_on_export_report_pressed)
	row1.add_child(btn_export)

	var row2 := HBoxContainer.new()
	row2.add_theme_constant_override("separation", 12)
	top_vbox.add_child(row2)

	deform_badge_label = Label.new()
	deform_badge_label.text = "  Escala Visual de Deformación: 100.0x"
	deform_badge_label.add_theme_color_override("font_color", Color(1.0, 0.85, 0.25))
	row2.add_child(deform_badge_label)

	deform_slider = HSlider.new()
	deform_slider.min_value = 1.0
	deform_slider.max_value = 2500.0
	deform_slider.step = 1.0
	deform_slider.value = 100.0
	deform_slider.custom_minimum_size = Vector2(180, 24)
	deform_slider.value_changed.connect(_on_deform_scale_changed)
	row2.add_child(deform_slider)

	var btn_auto_scale := Button.new()
	btn_auto_scale.text = "Auto-Escala"
	btn_auto_scale.pressed.connect(_on_auto_scale_pressed)
	row2.add_child(btn_auto_scale)

	colormap_selector = OptionButton.new()
	colormap_selector.add_item("Mapa: Tensión Von Mises [MPa]")
	colormap_selector.add_item("Mapa: Desplazamiento ||u|| [mm]")
	colormap_selector.add_item("Mapa: Tensión Axial [MPa]")
	colormap_selector.item_selected.connect(_on_display_option_changed)
	row2.add_child(colormap_selector)

	chk_original = CheckBox.new()
	chk_original.text = "Geometría Original"
	chk_original.button_pressed = true
	chk_original.toggled.connect(_on_display_option_changed)
	row2.add_child(chk_original)

	chk_deformed = CheckBox.new()
	chk_deformed.text = "Deformada"
	chk_deformed.button_pressed = true
	chk_deformed.toggled.connect(_on_display_option_changed)
	row2.add_child(chk_deformed)

	chk_labels = CheckBox.new()
	chk_labels.text = "Nodos/Malla"
	chk_labels.button_pressed = true
	chk_labels.toggled.connect(_on_display_option_changed)
	row2.add_child(chk_labels)

	chk_loads = CheckBox.new()
	chk_loads.text = "Cargas"
	chk_loads.button_pressed = true
	chk_loads.toggled.connect(_on_display_option_changed)
	row2.add_child(chk_loads)

	chk_reactions = CheckBox.new()
	chk_reactions.text = "Reacciones"
	chk_reactions.button_pressed = true
	chk_reactions.toggled.connect(_on_display_option_changed)
	row2.add_child(chk_reactions)

	# 2. Panel Izquierdo — Editor Geométrico 3D
	var left_panel := PanelContainer.new()
	left_panel.anchor_left = 0.0
	left_panel.anchor_right = 0.0
	left_panel.anchor_top = 0.0
	left_panel.anchor_bottom = 1.0
	left_panel.offset_left = 10
	left_panel.offset_right = 365
	left_panel.offset_top = 92
	left_panel.offset_bottom = -50
	canvas.add_child(left_panel)

	var left_scroll := ScrollContainer.new()
	left_panel.add_child(left_scroll)

	var left_vbox := VBoxContainer.new()
	left_vbox.custom_minimum_size = Vector2(335, 0)
	left_vbox.add_theme_constant_override("separation", 6)
	left_scroll.add_child(left_vbox)

	_add_section_header(left_vbox, "EDITOR GEOMÉTRICO 3D")
	lbl_selection_info = Label.new()
	lbl_selection_info.text = "Clic izq. en nodo/barra 3D para seleccionar."
	lbl_selection_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	left_vbox.add_child(lbl_selection_info)

	# Nodos
	_add_section_header(left_vbox, "1. Nodos (Coordenadas [m])")
	var h_nx := _make_spin_row(left_vbox, "X [m]:", -100.0, 100.0, 0.1, 0.0)
	var h_ny := _make_spin_row(left_vbox, "Y [m]:", -100.0, 100.0, 0.1, 2.0)
	var h_nz := _make_spin_row(left_vbox, "Z [m]:", -100.0, 100.0, 0.1, 0.0)
	spn_node_x = h_nx
	spn_node_y = h_ny
	spn_node_z = h_nz

	var node_btns := HBoxContainer.new()
	left_vbox.add_child(node_btns)
	var btn_add_node := Button.new()
	btn_add_node.text = "+ Añadir Nodo"
	btn_add_node.pressed.connect(_on_add_node_pressed)
	node_btns.add_child(btn_add_node)

	var btn_upd_node := Button.new()
	btn_upd_node.text = "Mover Selec."
	btn_upd_node.pressed.connect(_on_update_selected_node_pressed)
	node_btns.add_child(btn_upd_node)

	var btn_del_node := Button.new()
	btn_del_node.text = "Borrar Nodo"
	btn_del_node.pressed.connect(_on_delete_selected_node_pressed)
	node_btns.add_child(btn_del_node)

	# Elementos y Secciones
	_add_section_header(left_vbox, "2. Conectar Barra / Viga y Sección")
	opt_elem_type = OptionButton.new()
	opt_elem_type.add_item("TRUSS_3D (Barra Articulada)")
	opt_elem_type.add_item("FRAME_3D (Viga-Columna 6 GDL)")
	left_vbox.add_child(opt_elem_type)

	spn_elem_n1 = _make_spin_row(left_vbox, "Nodo Inicio ID:", 1, 999, 1, 1)
	spn_elem_n2 = _make_spin_row(left_vbox, "Nodo Fin ID:", 1, 999, 1, 2)
	spn_mat_e = _make_spin_row(left_vbox, "Young E [GPa]:", 1.0, 1000.0, 5.0, 210.0)
	spn_mat_nu = _make_spin_row(left_vbox, "Poisson nu [-]:", 0.0, 0.49, 0.01, 0.30)
	spn_sec_b = _make_spin_row(left_vbox, "Ancho b [m]:", 0.01, 2.0, 0.01, 0.15)
	spn_sec_h = _make_spin_row(left_vbox, "Canto h [m]:", 0.01, 2.0, 0.01, 0.30)

	var elem_btns := HBoxContainer.new()
	left_vbox.add_child(elem_btns)
	var btn_add_elem := Button.new()
	btn_add_elem.text = "+ Conectar Barra"
	btn_add_elem.pressed.connect(_on_add_element_pressed)
	elem_btns.add_child(btn_add_elem)

	var btn_upd_prop := Button.new()
	btn_upd_prop.text = "Aplicar Mat/Sección"
	btn_upd_prop.pressed.connect(_on_update_material_section_pressed)
	elem_btns.add_child(btn_upd_prop)

	var btn_del_elem := Button.new()
	btn_del_elem.text = "Borrar Elem."
	btn_del_elem.pressed.connect(_on_delete_selected_element_pressed)
	elem_btns.add_child(btn_del_elem)

	# Condiciones de Apoyo
	_add_section_header(left_vbox, "3. Condiciones de Apoyo (Restricciones)")
	spn_bc_node = _make_spin_row(left_vbox, "Nodo Apoyo ID:", 1, 999, 1, 1)
	opt_bc_preset = OptionButton.new()
	opt_bc_preset.add_item("PINNED_3D (Articulado ux=uy=uz=0)")
	opt_bc_preset.add_item("FIXED_3D (Empotrado 6 GDL)")
	opt_bc_preset.add_item("ROLLER_X (Libre en X)")
	opt_bc_preset.add_item("ROLLER_Y (Libre en Y)")
	opt_bc_preset.add_item("PINNED_2D_XY")
	opt_bc_preset.add_item("ROLLER_2D_XY")
	opt_bc_preset.add_item("CUSTOM (Casillas manuales)")
	opt_bc_preset.add_item("FREE (Eliminar apoyo)")
	left_vbox.add_child(opt_bc_preset)

	var bc_checks := HBoxContainer.new()
	left_vbox.add_child(bc_checks)
	chk_bc_ux = CheckBox.new(); chk_bc_ux.text = "ux"; chk_bc_ux.button_pressed = true; bc_checks.add_child(chk_bc_ux)
	chk_bc_uy = CheckBox.new(); chk_bc_uy.text = "uy"; chk_bc_uy.button_pressed = true; bc_checks.add_child(chk_bc_uy)
	chk_bc_uz = CheckBox.new(); chk_bc_uz.text = "uz"; chk_bc_uz.button_pressed = true; bc_checks.add_child(chk_bc_uz)
	chk_bc_rx = CheckBox.new(); chk_bc_rx.text = "rx"; bc_checks.add_child(chk_bc_rx)
	chk_bc_ry = CheckBox.new(); chk_bc_ry.text = "ry"; bc_checks.add_child(chk_bc_ry)
	chk_bc_rz = CheckBox.new(); chk_bc_rz.text = "rz"; bc_checks.add_child(chk_bc_rz)

	var btn_set_bc := Button.new()
	btn_set_bc.text = "Asignar Apoyo al Nodo"
	btn_set_bc.pressed.connect(_on_assign_bc_pressed)
	left_vbox.add_child(btn_set_bc)

	# Cargas Puntuales y Distribuidas
	_add_section_header(left_vbox, "4. Cargas Puntuales y Distribuidas")
	spn_load_node = _make_spin_row(left_vbox, "Nodo Carga ID:", 1, 999, 1, 2)
	spn_load_fx = _make_spin_row(left_vbox, "Fx [kN]:", -5000.0, 5000.0, 5.0, 0.0)
	spn_load_fy = _make_spin_row(left_vbox, "Fy [kN]:", -5000.0, 5000.0, 5.0, -50.0)
	spn_load_fz = _make_spin_row(left_vbox, "Fz [kN]:", -5000.0, 5000.0, 5.0, 0.0)

	var btn_set_load := Button.new()
	btn_set_load.text = "Asignar Carga Puntual Nodal"
	btn_set_load.pressed.connect(_on_assign_nodal_load_pressed)
	left_vbox.add_child(btn_set_load)

	spn_dload_elem = _make_spin_row(left_vbox, "Elemento Viga ID:", 1, 999, 1, 1)
	spn_dload_qy = _make_spin_row(left_vbox, "Carga qy [kN/m]:", -1000.0, 1000.0, 1.0, -15.0)
	var btn_set_dload := Button.new()
	btn_set_dload.text = "Asignar Carga Distribuida"
	btn_set_dload.pressed.connect(_on_assign_distributed_load_pressed)
	left_vbox.add_child(btn_set_dload)

	# 3. Panel Derecho — Resultados, Escala Cromática y Restricciones Normativas
	var right_panel := PanelContainer.new()
	right_panel.anchor_left = 1.0
	right_panel.anchor_right = 1.0
	right_panel.anchor_top = 0.0
	right_panel.anchor_bottom = 1.0
	right_panel.offset_left = -430
	right_panel.offset_right = -10
	right_panel.offset_top = 92
	right_panel.offset_bottom = -50
	canvas.add_child(right_panel)

	var right_vbox := VBoxContainer.new()
	right_vbox.add_theme_constant_override("separation", 6)
	right_panel.add_child(right_vbox)

	_add_section_header(right_vbox, "ESCALA CROMÁTICA Y RESULTADOS")

	var grad_bar := ColorRect.new()
	grad_bar.custom_minimum_size = Vector2(390, 16)
	grad_bar.color = Color(0.2, 0.65, 0.95)
	right_vbox.add_child(grad_bar)

	var grad_labels := HBoxContainer.new()
	right_vbox.add_child(grad_labels)
	legend_min_label = Label.new(); legend_min_label.text = "Min: 0.0"; grad_labels.add_child(legend_min_label)
	var spacer1 := Control.new(); spacer1.size_flags_horizontal = Control.SIZE_EXPAND_FILL; grad_labels.add_child(spacer1)
	legend_mid_label = Label.new(); legend_mid_label.text = "Med: 0.5"; grad_labels.add_child(legend_mid_label)
	var spacer2 := Control.new(); spacer2.size_flags_horizontal = Control.SIZE_EXPAND_FILL; grad_labels.add_child(spacer2)
	legend_max_label = Label.new(); legend_max_label.text = "Max: 1.0"; grad_labels.add_child(legend_max_label)

	results_richtext = RichTextLabel.new()
	results_richtext.bbcode_enabled = true
	results_richtext.selection_enabled = true
	results_richtext.size_flags_vertical = Control.SIZE_EXPAND_FILL
	right_vbox.add_child(results_richtext)

	# 4. Barra inferior de estado y aviso de no certificación
	var bottom_panel := PanelContainer.new()
	bottom_panel.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	bottom_panel.custom_minimum_size = Vector2(0, 42)
	canvas.add_child(bottom_panel)

	status_bar_label = Label.new()
	status_bar_label.text = "  AVISO TÉCNICO: Resultados de uso académico/conceptual. NO APTOS PARA CONSTRUCCIÓN NI CERTIFICACIÓN ESTRUCTURAL."
	status_bar_label.add_theme_color_override("font_color", Color(1.0, 0.78, 0.32))
	bottom_panel.add_child(status_bar_label)


func _load_benchmark_by_index(index: int) -> void:
	if index < 0 or index >= BENCHMARK_FILES.size():
		return
	benchmark_selector.select(index)
	var path: String = BENCHMARK_FILES[index]["file"]
	if FileAccess.file_exists(path):
		var raw := FileAccess.get_file_as_string(path)
		var parsed = JSON.parse_string(raw)
		if typeof(parsed) == TYPE_DICTIONARY:
			current_model = parsed
			visualizer.selected_node_id = -1
			visualizer.selected_element_id = -1
			_run_solver()
			_on_auto_scale_pressed()
			_frame_camera_to_model()
			return


func _run_solver() -> void:
	var mode := backend_selector.selected if backend_selector != null else 0
	current_result = SolverBridge.solve(current_model, mode)
	_refresh_3d_view()
	_update_results_panel()


func _refresh_3d_view() -> void:
	if visualizer == null:
		return
	visualizer.show_original = chk_original.button_pressed
	visualizer.show_deformed = chk_deformed.button_pressed
	visualizer.show_node_labels = chk_labels.button_pressed
	visualizer.show_loads = chk_loads.button_pressed
	visualizer.show_reactions = chk_reactions.button_pressed
	visualizer.deform_scale = deform_slider.value
	visualizer.colormap_mode = colormap_selector.selected

	var legend_info := visualizer.render_structure(current_model, current_result)
	var cmin := float(legend_info.get("min", 0.0))
	var cmax := float(legend_info.get("max", 1.0))
	var cmid := 0.5 * (cmin + cmax)
	var unit := String(legend_info.get("unit", "MPa"))
	legend_min_label.text = "Azul: %.3f %s" % [cmin, unit]
	legend_mid_label.text = "Verde: %.3f %s" % [cmid, unit]
	legend_max_label.text = "Rojo: %.3f %s" % [cmax, unit]


func _update_results_panel() -> void:
	if results_richtext == null:
		return
	var ok := bool(current_result.get("success", false))
	var diag: Dictionary = current_result.get("diagnostics", {})
	var mname := String(current_model.get("name", "Modelo"))

	var txt := "[b][color=#61dafb]%s[/color][/b]\n" % mname
	txt += "%s\n\n" % String(current_model.get("description", ""))

	if not ok:
		txt += "[b][color=#ff5555]ESTADO: INESTABLE / CONDICIÓN INVÁLIDA[/color][/b]\n"
		var errs: Array = diag.get("errors", [])
		for err in errs:
			txt += "[color=#ff8888]• %s[/color]\n" % String(err)
		txt += "\n[color=#ffcc66]Revise las condiciones de apoyo, coacciones en 3D o articulaciones colineales.[/color]\n"
		results_richtext.text = txt
		return

	var max_u_mm := float(current_result.get("max_displacement", 0.0)) * 1000.0
	var max_vm_mpa := float(current_result.get("max_von_mises", 0.0)) / 1e6
	var kappa := float(diag.get("condition_number", 1.0))
	var f_res := float(diag.get("force_residual_norm", 0.0))
	var m_res := float(diag.get("moment_residual_norm", 0.0))

	txt += "[b][color=#4ade80]ESTADO: RESUELTO (EQUILIBRIO VERIFICADO)[/color][/b]\n"
	txt += "• GDL libres / restringidos: [b]%d / %d[/b]\n" % [int(diag.get("free_dofs", 0)), int(diag.get("constrained_dofs", 0))]
	txt += "• Condicionamiento κ(K): [b]%s[/b]\n" % String.num_scientific(kappa)
	txt += "• Residuo ΣF / ΣM: [b]%s N / %s N·m[/b]\n" % [String.num_scientific(f_res), String.num_scientific(m_res)]
	txt += "• Escala visual activa: [b][color=#fde047]%.1fx[/color][/b]\n" % deform_slider.value
	txt += "• Desplazamiento máx ||u||: [b]%.4f mm[/b]\n" % max_u_mm
	txt += "• Tensión Von Mises máx: [b]%.3f MPa[/b]\n\n" % max_vm_mpa

	txt += "[b][color=#93c5fd]DESPLAZAMIENTOS NODALES [mm]:[/color][/b]\n"
	var disp_map: Dictionary = current_result.get("nodal_displacements", {})
	for n in current_model.get("nodes", []):
		var nid := int(n["id"])
		var u: Array = disp_map.get(str(nid), disp_map.get(nid, [0, 0, 0, 0, 0, 0]))
		txt += "  N%d: ux=%.3f, uy=%.3f, uz=%.3f mm\n" % [nid, float(u[0]) * 1000.0, float(u[1]) * 1000.0, float(u[2]) * 1000.0]

	txt += "\n[b][color=#86efac]REACCIONES EN APOYOS [kN / kN·m]:[/color][/b]\n"
	var reac_map: Dictionary = current_result.get("nodal_reactions", {})
	for bc in current_model.get("boundary_conditions", []):
		var nid := int(bc["node_id"])
		var r: Array = reac_map.get(str(nid), reac_map.get(nid, [0, 0, 0, 0, 0, 0]))
		txt += "  N%d: R=(%.2f, %.2f, %.2f) kN | M=(%.2f, %.2f, %.2f) kN·m\n" % [
			nid,
			float(r[0]) / 1000.0, float(r[1]) / 1000.0, float(r[2]) / 1000.0,
			float(r[3]) / 1000.0, float(r[4]) / 1000.0, float(r[5]) / 1000.0
		]

	txt += "\n[b][color=#fca5a5]ESFUERZOS Y TENSIONES POR ELEMENTO:[/color][/b]\n"
	var eres_map: Dictionary = current_result.get("element_results", {})
	for e in current_model.get("elements", []):
		var eid := int(e["id"])
		var er: Dictionary = eres_map.get(str(eid), eres_map.get(eid, {}))
		if er.is_empty():
			continue
		var n_kn := float(er.get("axial_force", 0.0)) / 1000.0
		var s_ax := float(er.get("axial_stress", 0.0)) / 1e6
		var s_vm := float(er.get("von_mises_stress", 0.0)) / 1e6
		var mz_knm := float(er.get("moment_z", 0.0)) / 1000.0
		txt += "  E%d (%s): N=%.2f kN | Mz=%.2f kN·m | σ_x=%.2f MPa | σ_VM=%.2f MPa\n" % [
			eid, String(e["type"]), n_kn, mz_knm, s_ax, s_vm
		]

	txt += "\n[color=#fbbf24][i]Nota: Modelo elástico lineal infinitesimal. No contempla pandeo, plastificación, fatiga ni fractura. Prohibido su uso para construcción o certificación.[/i][/color]"
	results_richtext.text = txt


func _populate_inspector_for_node(nid: int) -> void:
	for n in current_model.get("nodes", []):
		if int(n["id"]) == nid:
			spn_node_x.value = float(n["x"])
			spn_node_y.value = float(n["y"])
			spn_node_z.value = float(n["z"])
			spn_bc_node.value = nid
			spn_load_node.value = nid
			lbl_selection_info.text = "Seleccionado: Nodo N%d (%.2f, %.2f, %.2f) m" % [nid, float(n["x"]), float(n["y"]), float(n["z"])]
			return


func _populate_inspector_for_element(eid: int) -> void:
	for e in current_model.get("elements", []):
		if int(e["id"]) == eid:
			var enodes: Array = e["nodes"]
			if enodes.size() >= 2:
				spn_elem_n1.value = int(enodes[0])
				spn_elem_n2.value = int(enodes[1])
			spn_dload_elem.value = eid
			lbl_selection_info.text = "Seleccionado: Elemento E%d (%s) Nodos %s" % [eid, String(e["type"]), str(enodes)]
			return


func _on_add_node_pressed() -> void:
	var nodes: Array = current_model.get("nodes", [])
	var max_id := 0
	for n in nodes:
		max_id = max(max_id, int(n["id"]))
	var new_id := max_id + 1
	nodes.append({
		"id": new_id,
		"x": spn_node_x.value,
		"y": spn_node_y.value,
		"z": spn_node_z.value,
		"label": "N%d" % new_id
	})
	current_model["nodes"] = nodes
	visualizer.selected_node_id = new_id
	lbl_selection_info.text = "Creado Nodo N%d." % new_id
	_run_solver()


func _on_update_selected_node_pressed() -> void:
	var sel := visualizer.selected_node_id
	if sel <= 0:
		return
	for n in current_model.get("nodes", []):
		if int(n["id"]) == sel:
			n["x"] = spn_node_x.value
			n["y"] = spn_node_y.value
			n["z"] = spn_node_z.value
			break
	_run_solver()


func _on_delete_selected_node_pressed() -> void:
	var sel := visualizer.selected_node_id
	if sel <= 0:
		return
	var new_nodes := []
	for n in current_model.get("nodes", []):
		if int(n["id"]) != sel:
			new_nodes.append(n)
	var new_elems := []
	for e in current_model.get("elements", []):
		if not (sel in e["nodes"]):
			new_elems.append(e)
	current_model["nodes"] = new_nodes
	current_model["elements"] = new_elems
	visualizer.selected_node_id = -1
	_run_solver()


func _on_add_element_pressed() -> void:
	var n1 := int(spn_elem_n1.value)
	var n2 := int(spn_elem_n2.value)
	if n1 == n2:
		lbl_selection_info.text = "Error: Los nodos de inicio y fin deben ser distintos."
		return
	var elems: Array = current_model.get("elements", [])
	var max_id := 0
	for e in elems:
		max_id = max(max_id, int(e["id"]))
	var new_id := max_id + 1
	var etype := "TRUSS_3D" if opt_elem_type.selected == 0 else "FRAME_3D"
	elems.append({
		"id": new_id,
		"type": etype,
		"nodes": [n1, n2],
		"material_id": 1,
		"section_id": 1,
		"roll_angle": 0.0,
		"label": "E%d" % new_id
	})
	current_model["elements"] = elems
	_run_solver()


func _on_update_material_section_pressed() -> void:
	var b := spn_sec_b.value
	var h := spn_sec_h.value
	var area := b * h
	var iz := b * pow(h, 3) / 12.0
	var iy := h * pow(b, 3) / 12.0
	var j_t := iz + iy

	current_model["materials"] = [{
		"id": 1,
		"name": "Material Editado",
		"E": spn_mat_e.value * 1e9,
		"nu": spn_mat_nu.value,
		"rho": 7850.0,
		"yield_strength": 275e6
	}]
	current_model["sections"] = [{
		"id": 1,
		"name": "Rect %.0fx%.0f mm" % [b * 1000.0, h * 1000.0],
		"shape": "RECTANGULAR",
		"A": area,
		"Iy": iy,
		"Iz": iz,
		"J": j_t,
		"thickness": b,
		"dimensions": {"b": b, "h": h}
	}]
	_run_solver()


func _on_delete_selected_element_pressed() -> void:
	var sel := visualizer.selected_element_id
	if sel <= 0:
		return
	var new_elems := []
	for e in current_model.get("elements", []):
		if int(e["id"]) != sel:
			new_elems.append(e)
	current_model["elements"] = new_elems
	visualizer.selected_element_id = -1
	_run_solver()


func _on_assign_bc_pressed() -> void:
	var nid := int(spn_bc_node.value)
	var bcs: Array = []
	for bc in current_model.get("boundary_conditions", []):
		if int(bc["node_id"]) != nid:
			bcs.append(bc)

	var p_idx := opt_bc_preset.selected
	if p_idx == 7: # FREE
		current_model["boundary_conditions"] = bcs
		_run_solver()
		return

	var ux := chk_bc_ux.button_pressed
	var uy := chk_bc_uy.button_pressed
	var uz := chk_bc_uz.button_pressed
	var rx := chk_bc_rx.button_pressed
	var ry := chk_bc_ry.button_pressed
	var rz := chk_bc_rz.button_pressed

	if p_idx == 0: # PINNED_3D
		ux = true; uy = true; uz = true; rx = false; ry = false; rz = false
	elif p_idx == 1: # FIXED_3D
		ux = true; uy = true; uz = true; rx = true; ry = true; rz = true
	elif p_idx == 2: # ROLLER_X
		ux = false; uy = true; uz = true; rx = false; ry = false; rz = false
	elif p_idx == 3: # ROLLER_Y
		ux = true; uy = false; uz = true; rx = false; ry = false; rz = false
	elif p_idx == 4: # PINNED_2D_XY
		ux = true; uy = true; uz = true; rx = true; ry = true; rz = false
	elif p_idx == 5: # ROLLER_2D_XY
		ux = false; uy = true; uz = true; rx = true; ry = true; rz = false

	bcs.append({
		"node_id": nid,
		"ux": ux, "uy": uy, "uz": uz,
		"rx": rx, "ry": ry, "rz": rz,
		"prescribed_ux": 0.0, "prescribed_uy": 0.0, "prescribed_uz": 0.0,
		"prescribed_rx": 0.0, "prescribed_ry": 0.0, "prescribed_rz": 0.0,
		"preset": "CUSTOM"
	})
	current_model["boundary_conditions"] = bcs
	_run_solver()


func _on_assign_nodal_load_pressed() -> void:
	var nid := int(spn_load_node.value)
	var nloads: Array = []
	for nl in current_model.get("nodal_loads", []):
		if int(nl["node_id"]) != nid:
			nloads.append(nl)
	var fx := spn_load_fx.value * 1000.0
	var fy := spn_load_fy.value * 1000.0
	var fz := spn_load_fz.value * 1000.0
	if absf(fx) > 1e-3 or absf(fy) > 1e-3 or absf(fz) > 1e-3:
		nloads.append({
			"node_id": nid,
			"fx": fx, "fy": fy, "fz": fz,
			"mx": 0.0, "my": 0.0, "mz": 0.0,
			"label": "Carga N%d" % nid
		})
	current_model["nodal_loads"] = nloads
	_run_solver()


func _on_assign_distributed_load_pressed() -> void:
	var eid := int(spn_dload_elem.value)
	var dloads: Array = []
	for dl in current_model.get("distributed_loads", []):
		if int(dl["element_id"]) != eid:
			dloads.append(dl)
	var qy := spn_dload_qy.value * 1000.0
	if absf(qy) > 1e-3:
		dloads.append({
			"element_id": eid,
			"qx": 0.0, "qy": qy, "qz": 0.0,
			"coordinate_system": "GLOBAL",
			"label": "q_E%d" % eid
		})
	current_model["distributed_loads"] = dloads
	_run_solver()


func _on_deform_scale_changed(val: float) -> void:
	deform_badge_label.text = "  Escala Visual de Deformación: %.1fx" % val
	_refresh_3d_view()
	_update_results_panel()


func _on_auto_scale_pressed() -> void:
	var rec := float(current_result.get("recommended_visual_scale", 100.0))
	deform_slider.value = clampf(rec, deform_slider.min_value, deform_slider.max_value)
	deform_badge_label.text = "  Escala Visual de Deformación: %.1fx" % deform_slider.value
	_refresh_3d_view()


func _on_display_option_changed(_arg = null) -> void:
	_refresh_3d_view()


func _on_export_report_pressed() -> void:
	var rep_dir := SolverBridge.export_model_reports(current_model)
	status_bar_label.text = "  Informe exportado en: %s | AVISO: NO APTO PARA CONSTRUCCIÓN NI CERTIFICACIÓN." % rep_dir


func _frame_camera_to_model() -> void:
	var nodes: Array = current_model.get("nodes", [])
	if nodes.is_empty() or camera == null:
		return
	var min_v := Vector3(1e9, 1e9, 1e9)
	var max_v := Vector3(-1e9, -1e9, -1e9)
	for n in nodes:
		var p := Vector3(float(n["x"]), float(n["y"]), float(n["z"]))
		min_v = min_v.min(p)
		max_v = max_v.max(p)
	var center := 0.5 * (min_v + max_v)
	var span := maxf(min_v.distance_to(max_v), 2.0)
	camera.reset_view(center, span * 1.85)


func _add_section_header(parent: Control, text: String) -> void:
	var lbl := Label.new()
	lbl.text = text
	lbl.add_theme_color_override("font_color", Color(0.45, 0.82, 1.0))
	parent.add_child(lbl)


func _make_spin_row(parent: Control, label_text: String, min_v: float, max_v: float, step_v: float, def_v: float) -> SpinBox:
	var hb := HBoxContainer.new()
	parent.add_child(hb)
	var lbl := Label.new()
	lbl.text = label_text
	lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hb.add_child(lbl)
	var spn := SpinBox.new()
	spn.min_value = min_v
	spn.max_value = max_v
	spn.step = step_v
	spn.value = def_v
	spn.custom_minimum_size = Vector2(130, 0)
	hb.add_child(spn)
	return spn
