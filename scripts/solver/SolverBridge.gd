extends RefCounted
class_name SolverBridge

## Puente de resolución numérica para Structura 3D en Godot 4.
## Conecta la interfaz 3D con el módulo numérico local en Python/NumPy (`solver/cli.py`)
## y dispone del motor nativo en GDScript (`NativeSolver.gd`) como respaldo inmediato.

enum BackendMode {
	PYTHON_LOCAL_WITH_FALLBACK = 0,
	GDSCRIPT_NATIVE_ONLY = 1
}


static func solve(model: Dictionary, mode: int = BackendMode.PYTHON_LOCAL_WITH_FALLBACK) -> Dictionary:
	if mode == BackendMode.GDSCRIPT_NATIVE_ONLY:
		var ns = load("res://scripts/solver/NativeSolver.gd")
		if ns != null:
			return ns.solve_model(model)

	# Intentar ejecutar el motor numérico local en Python (NumPy/SciPy) para máxima precisión y soporte TETRA_4/CST
	var proj_dir := ProjectSettings.globalize_path("res://")
	var tmp_in := proj_dir.path_join("reports/_active_model.structura.json")
	var tmp_out := proj_dir.path_join("reports/_active_result.result.json")

	DirAccess.make_dir_recursive_absolute(proj_dir.path_join("reports"))
	var f_in := FileAccess.open(tmp_in, FileAccess.WRITE)
	if f_in != null:
		f_in.store_string(JSON.stringify(model, "  "))
		f_in.close()

		var output := []
		var args := PackedStringArray(["-m", "solver.cli", "--input", tmp_in, "--output", tmp_out])
		var exit_code := OS.execute("py", args, output, true)
		if exit_code != 0 and exit_code != 2:
			exit_code = OS.execute("python", args, output, true)

		if (exit_code == 0 or exit_code == 2) and FileAccess.file_exists(tmp_out):
			var raw_res := FileAccess.get_file_as_string(tmp_out)
			var parsed = JSON.parse_string(raw_res)
			if typeof(parsed) == TYPE_DICTIONARY:
				return parsed

	# Respaldo inmediato al motor nativo GDScript
	var ns_fb = load("res://scripts/solver/NativeSolver.gd")
	if ns_fb != null:
		return ns_fb.solve_model(model)
	return {"success": false, "diagnostics": {"errors": ["No se pudo ejecutar el motor de cálculo."]}}


static func export_model_reports(model: Dictionary) -> String:
	var proj_dir := ProjectSettings.globalize_path("res://")
	var rep_dir := proj_dir.path_join("reports")
	DirAccess.make_dir_recursive_absolute(rep_dir)

	var tmp_in := rep_dir.path_join("exported_model.structura.json")
	var f_in := FileAccess.open(tmp_in, FileAccess.WRITE)
	if f_in != null:
		f_in.store_string(JSON.stringify(model, "  "))
		f_in.close()

	var output := []
	var args := PackedStringArray(["-m", "solver.cli", "--input", tmp_in, "--report-dir", rep_dir])
	var exit_code := OS.execute("py", args, output, true)
	if exit_code != 0 and exit_code != 2:
		OS.execute("python", args, output, true)

	return rep_dir
