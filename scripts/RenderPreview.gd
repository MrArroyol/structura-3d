extends SceneTree

const OrbitCamera3DScript = preload("res://scripts/OrbitCamera3D.gd")
const Visualizer3DScript = preload("res://scripts/Visualizer3D.gd")

func _initialize() -> void:
	var root_node := Node3D.new()
	get_root().add_child(root_node)

	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.06, 0.09, 0.14)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.72, 0.78, 0.88)
	env.ambient_light_energy = 0.85

	var world_env := WorldEnvironment.new()
	world_env.environment = env
	root_node.add_child(world_env)

	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-45.0, -35.0, 0.0)
	light.light_energy = 1.25
	root_node.add_child(light)

	var cam: Camera3D = OrbitCamera3DScript.new()
	root_node.add_child(cam)
	cam.make_current()

	var vis: Node3D = Visualizer3DScript.new()
	root_node.add_child(vis)

	await process_frame
	await process_frame

	var proj_dir := ProjectSettings.globalize_path("res://")
	var out_dir := proj_dir.path_join("reports/renders")
	DirAccess.make_dir_recursive_absolute(out_dir)

	var cases := [
		{"key": "02_space_truss_3d", "scale": 450.0, "center": Vector3(0.0, 0.9, 0.0), "dist": 5.2},
		{"key": "03_cantilever_beam", "scale": 80.0, "center": Vector3(2.0, -0.2, 0.0), "dist": 5.5},
		{"key": "05_space_portal_frame", "scale": 600.0, "center": Vector3(2.5, 1.75, 2.0), "dist": 9.5},
		{"key": "07_tetra4_solid_prism", "scale": 200.0, "center": Vector3(0.5, 0.5, 0.5), "dist": 3.4}
	]

	for c in cases:
		var key: String = c["key"]
		var m_path := "res://benchmarks/%s.structura.json" % key
		var r_path := "res://reports/%s.result.json" % key
		if not FileAccess.file_exists(m_path) or not FileAccess.file_exists(r_path):
			continue
		var model: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(m_path))
		var result: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(r_path))

		vis.deform_scale = float(c["scale"])
		vis.render_structure(model, result)
		cam.reset_view(c["center"], float(c["dist"]))

		await process_frame
		await process_frame
		await RenderingServer.frame_post_draw

		var img := get_root().get_texture().get_image()
		if img != null:
			var save_path := out_dir.path_join("%s_3d.png" % key)
			img.save_png(save_path)
			print("Render 3D guardado en: ", save_path)

	# Captura adicional de la interfaz interactiva completa (Main.tscn)
	root_node.queue_free()
	await process_frame
	var main_packed := load("res://scenes/Main.tscn") as PackedScene
	if main_packed != null:
		var main_inst := main_packed.instantiate()
		get_root().add_child(main_inst)
		await process_frame
		await process_frame
		await RenderingServer.frame_post_draw
		var ui_img := get_root().get_texture().get_image()
		if ui_img != null:
			var ui_path := out_dir.path_join("00_structura3d_main_ui.png")
			ui_img.save_png(ui_path)
			print("Render UI principal guardado en: ", ui_path)

	quit(0)
