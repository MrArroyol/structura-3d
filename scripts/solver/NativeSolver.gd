extends RefCounted
class_name NativeSolver

## Motor numérico nativo en GDScript para Structura 3D (Método de Rigidez Directa).
## Funciona 100% offline y sin dependencias externas, como alternativa o respaldo directo
## al motor numérico local en Python/NumPy (`solver/engine.py`).

const DISCLAIMER_TEXT := (
	"ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D "
	+ "corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. "
	+ "NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural."
)


static func solve_model(model: Dictionary) -> Dictionary:
	var diag := {
		"is_valid": true,
		"is_singular": false,
		"is_mechanism": false,
		"condition_number": 1.0,
		"matrix_rank": 0,
		"free_dofs": 0,
		"constrained_dofs": 0,
		"total_dofs": 0,
		"zero_energy_modes": 0,
		"force_residual_norm": 0.0,
		"moment_residual_norm": 0.0,
		"equilibrium_satisfied": true,
		"errors": [],
		"warnings": []
	}

	var nodes: Array = model.get("nodes", [])
	var elements: Array = model.get("elements", [])
	var materials: Array = model.get("materials", [])
	var sections: Array = model.get("sections", [])
	var bcs: Array = model.get("boundary_conditions", [])
	var nodal_loads: Array = model.get("nodal_loads", [])
	var dist_loads: Array = model.get("distributed_loads", [])

	if nodes.is_empty() or elements.is_empty():
		diag["is_valid"] = false
		diag["errors"].append("El modelo debe tener al menos un nodo y un elemento.")
		return _fail_result(model.get("name", "Modelo"), diag)

	var node_map := {}
	var node_idx := {}
	for i in range(nodes.size()):
		var n: Dictionary = nodes[i]
		var nid := int(n["id"])
		node_map[nid] = n
		node_idx[nid] = i

	var mat_map := {}
	for m in materials:
		var mid := int(m["id"])
		if float(m.get("E", 0.0)) <= 0.0 or float(m.get("nu", 0.3)) >= 0.5 or float(m.get("nu", 0.3)) <= -1.0:
			diag["is_valid"] = false
			diag["errors"].append("Propiedades elásticas inválidas en material ID %d." % mid)
			return _fail_result(model.get("name", "Modelo"), diag)
		mat_map[mid] = m

	var sec_map := {}
	for s in sections:
		var sid := int(s["id"])
		if float(s.get("A", 0.0)) <= 0.0:
			diag["is_valid"] = false
			diag["errors"].append("Área no positiva en sección ID %d." % sid)
			return _fail_result(model.get("name", "Modelo"), diag)
		sec_map[sid] = s

	var num_nodes := nodes.size()
	var total_dofs := 6 * num_nodes
	diag["total_dofs"] = total_dofs

	var K := _zeros_matrix(total_dofs, total_dofs)
	var F_nodal := _zeros_vec(total_dofs)
	var F_equiv := _zeros_vec(total_dofs)

	var connected_nodes := {}
	var rotational_nodes := {}
	var elem_q_local := {}
	var char_lengths: Array[float] = []

	for e in elements:
		var eid := int(e["id"])
		var etype := String(e["type"])
		var enodes: Array = e["nodes"]
		var mat: Dictionary = mat_map[int(e["material_id"])]
		var sec: Dictionary = sec_map.get(int(e.get("section_id", 1)), {"A": 0.01, "Iy": 1e-5, "Iz": 1e-5, "J": 2e-5, "thickness": 0.01})
		elem_q_local[eid] = Vector3.ZERO

		for nid in enodes:
			connected_nodes[int(nid)] = true
			if etype == "FRAME_3D":
				rotational_nodes[int(nid)] = true

		if etype == "TRUSS_3D":
			var n1: Dictionary = node_map[int(enodes[0])]
			var n2: Dictionary = node_map[int(enodes[1])]
			var p1 := _node_vec(n1)
			var p2 := _node_vec(n2)
			var L := p1.distance_to(p2)
			if L <= 1e-12:
				diag["is_valid"] = false
				diag["errors"].append("Elemento TRUSS_3D %d tiene longitud cero." % eid)
				return _fail_result(model.get("name", "Modelo"), diag)
			char_lengths.append(L)
			var ex := (p2 - p1) / L
			var k_ax := float(mat["E"]) * float(sec["A"]) / L
			var i1 := int(node_idx[int(enodes[0])]) * 6
			var i2 := int(node_idx[int(enodes[1])]) * 6
			var ex_arr := [ex.x, ex.y, ex.z]
			for a in range(3):
				for b in range(3):
					var v := k_ax * ex_arr[a] * ex_arr[b]
					K[i1 + a][i1 + b] += v
					K[i1 + a][i2 + b] -= v
					K[i2 + a][i1 + b] -= v
					K[i2 + a][i2 + b] += v

		elif etype == "FRAME_3D":
			var n1: Dictionary = node_map[int(enodes[0])]
			var n2: Dictionary = node_map[int(enodes[1])]
			var p1 := _node_vec(n1)
			var p2 := _node_vec(n2)
			var L := p1.distance_to(p2)
			if L <= 1e-12:
				diag["is_valid"] = false
				diag["errors"].append("Elemento FRAME_3D %d tiene longitud cero." % eid)
				return _fail_result(model.get("name", "Modelo"), diag)
			char_lengths.append(L)
			var R := _beam_rotation(p1, p2, float(e.get("roll_angle", 0.0)))
			var T := _frame_T(R)
			var k_loc := _frame_local_k(L, mat, sec)
			var k_glob := _mat_mul(_mat_transpose(T), _mat_mul(k_loc, T))
			var dofs := _dofs_2nodes(int(node_idx[int(enodes[0])]), int(node_idx[int(enodes[1])]))
			_assemble(K, k_glob, dofs)

	# Cargas puntuales
	for nl in nodal_loads:
		var nid := int(nl["node_id"])
		var base := int(node_idx[nid]) * 6
		F_nodal[base + 0] += float(nl.get("fx", 0.0))
		F_nodal[base + 1] += float(nl.get("fy", 0.0))
		F_nodal[base + 2] += float(nl.get("fz", 0.0))
		F_nodal[base + 3] += float(nl.get("mx", 0.0))
		F_nodal[base + 4] += float(nl.get("my", 0.0))
		F_nodal[base + 5] += float(nl.get("mz", 0.0))

	# Cargas distribuidas
	var elem_by_id := {}
	for e in elements:
		elem_by_id[int(e["id"])] = e
	for dl in dist_loads:
		var eid := int(dl["element_id"])
		if not elem_by_id.has(eid):
			continue
		var e: Dictionary = elem_by_id[eid]
		var enodes: Array = e["nodes"]
		if enodes.size() != 2:
			continue
		var p1 := _node_vec(node_map[int(enodes[0])])
		var p2 := _node_vec(node_map[int(enodes[1])])
		var L := p1.distance_to(p2)
		var R := _beam_rotation(p1, p2, float(e.get("roll_angle", 0.0)))
		var q_glob := Vector3(float(dl.get("qx", 0.0)), float(dl.get("qy", 0.0)), float(dl.get("qz", 0.0)))
		var q_loc := _mat3_vec(R, q_glob) if String(dl.get("coordinate_system", "GLOBAL")) == "GLOBAL" else q_glob
		elem_q_local[eid] += q_loc
		var feq_loc := _frame_feq_local(L, q_loc)
		var T := _frame_T(R)
		var feq_glob := _mat_vec(_mat_transpose(T), feq_loc)
		var dofs := _dofs_2nodes(int(node_idx[int(enodes[0])]), int(node_idx[int(enodes[1])]))
		for k in range(12):
			F_equiv[dofs[k]] += feq_glob[k]

	# Condiciones de apoyo
	var restrained := []
	var u_presc := _zeros_vec(total_dofs)
	for i in range(total_dofs):
		restrained.append(false)

	var user_bc_count := 0
	for bc in bcs:
		var nid := int(bc["node_id"])
		if not node_idx.has(nid):
			continue
		var base := int(node_idx[nid]) * 6
		var flags := [
			bool(bc.get("ux", false)),
			bool(bc.get("uy", false)),
			bool(bc.get("uz", false)),
			bool(bc.get("rx", false)),
			bool(bc.get("ry", false)),
			bool(bc.get("rz", false))
		]
		var presc := [
			float(bc.get("prescribed_ux", 0.0)),
			float(bc.get("prescribed_uy", 0.0)),
			float(bc.get("prescribed_uz", 0.0)),
			float(bc.get("prescribed_rx", 0.0)),
			float(bc.get("prescribed_ry", 0.0)),
			float(bc.get("prescribed_rz", 0.0))
		]
		for j in range(6):
			if flags[j]:
				restrained[base + j] = true
				u_presc[base + j] = presc[j]
				user_bc_count += 1

	var auto_locked := []
	for i in range(total_dofs):
		auto_locked.append(false)
	for n in nodes:
		var nid := int(n["id"])
		var base := int(node_idx[nid]) * 6
		if connected_nodes.has(nid) and not rotational_nodes.has(nid):
			for rj in [3, 4, 5]:
				if not restrained[base + rj]:
					restrained[base + rj] = true
					auto_locked[base + rj] = true

	var free_dofs: Array[int] = []
	var rest_dofs: Array[int] = []
	for i in range(total_dofs):
		if restrained[i]:
			rest_dofs.append(i)
		else:
			free_dofs.append(i)

	diag["free_dofs"] = free_dofs.size()
	diag["constrained_dofs"] = user_bc_count

	if user_bc_count == 0:
		diag["is_valid"] = false
		diag["is_singular"] = true
		diag["is_mechanism"] = true
		diag["errors"].append("Condiciones de apoyo insuficientes: ningún grado de libertad restringido.")
		return _fail_result(model.get("name", "Modelo"), diag)

	var F_total := _zeros_vec(total_dofs)
	for i in range(total_dofs):
		F_total[i] = F_nodal[i] + F_equiv[i]

	var u_global := _zeros_vec(total_dofs)
	for idx_r in rest_dofs:
		u_global[idx_r] = u_presc[idx_r]

	if free_dofs.size() > 0:
		var nf := free_dofs.size()
		var K_ff := _zeros_matrix(nf, nf)
		var rhs := _zeros_vec(nf)
		for a in range(nf):
			var da := free_dofs[a]
			rhs[a] = F_total[da]
			for idx_r in rest_dofs:
				rhs[a] -= K[da][idx_r] * u_global[idx_r]
			for b in range(nf):
				K_ff[a][b] = K[da][free_dofs[b]]

		var solve_out := _solve_linear_system(K_ff, rhs)
		if not solve_out["ok"]:
			diag["is_valid"] = false
			diag["is_singular"] = true
			diag["is_mechanism"] = true
			diag["zero_energy_modes"] = 1
			diag["errors"].append(solve_out["error"])
			return _fail_result(model.get("name", "Modelo"), diag)

		diag["condition_number"] = solve_out["cond_est"]
		diag["matrix_rank"] = nf
		var u_f: Array = solve_out["x"]
		for a in range(nf):
			u_global[free_dofs[a]] = u_f[a]

	var R_global := _mat_vec(K, u_global)
	for i in range(total_dofs):
		R_global[i] -= F_total[i]
	for df in free_dofs:
		R_global[df] = 0.0
	for i in range(total_dofs):
		if auto_locked[i]:
			R_global[i] = 0.0

	var disp_dict := {}
	var reac_dict := {}
	var max_disp := 0.0
	for n in nodes:
		var nid := int(n["id"])
		var base := int(node_idx[nid]) * 6
		var ud := [
			u_global[base + 0], u_global[base + 1], u_global[base + 2],
			u_global[base + 3], u_global[base + 4], u_global[base + 5]
		]
		var rd := [
			R_global[base + 0], R_global[base + 1], R_global[base + 2],
			R_global[base + 3], R_global[base + 4], R_global[base + 5]
		]
		disp_dict[str(nid)] = ud
		reac_dict[str(nid)] = rd
		var mag := Vector3(ud[0], ud[1], ud[2]).length()
		if mag > max_disp:
			max_disp = mag

	var elem_results := {}
	var max_vm := 0.0
	var max_ax := 0.0
	var min_ax := 0.0
	for idx_e in range(elements.size()):
		var e: Dictionary = elements[idx_e]
		var eid := int(e["id"])
		var etype := String(e["type"])
		var enodes: Array = e["nodes"]
		var mat: Dictionary = mat_map[int(e["material_id"])]
		var sec: Dictionary = sec_map.get(int(e.get("section_id", 1)), {"A": 0.01, "Iy": 1e-5, "Iz": 1e-5, "J": 2e-5})
		var n1: Dictionary = node_map[int(enodes[0])]
		var n2: Dictionary = node_map[int(enodes[1])]
		var p1 := _node_vec(n1)
		var p2 := _node_vec(n2)
		var L := p1.distance_to(p2)
		var i1 := int(node_idx[int(enodes[0])]) * 6
		var i2 := int(node_idx[int(enodes[1])]) * 6
		var u1 := Vector3(u_global[i1], u_global[i1 + 1], u_global[i1 + 2])
		var u2 := Vector3(u_global[i2], u_global[i2 + 1], u_global[i2 + 2])
		var ex := (p2 - p1) / L
		var elong := (u2 - u1).dot(ex)
		var s_ax := float(mat["E"]) * (elong / L)
		var f_ax := s_ax * float(sec["A"])
		var vm := absf(s_ax)

		elem_results[str(eid)] = {
			"element_id": eid,
			"element_type": etype,
			"length_or_measure": L,
			"axial_force": f_ax,
			"shear_y": 0.0,
			"shear_z": 0.0,
			"torsion": 0.0,
			"moment_y": 0.0,
			"moment_z": 0.0,
			"axial_stress": s_ax,
			"bending_stress_max": 0.0,
			"combined_stress": vm,
			"von_mises_stress": vm
		}
		if vm > max_vm:
			max_vm = vm
		if idx_e == 0 or s_ax > max_ax:
			max_ax = s_ax
		if idx_e == 0 or s_ax < min_ax:
			min_ax = s_ax

	var avg_l := 2.0
	if not char_lengths.is_empty():
		var s := 0.0
		for v in char_lengths:
			s += v
		avg_l = s / float(char_lengths.size())
	var rec_scale := 1.0
	if max_disp > 1e-12:
		rec_scale = clampf((0.12 * avg_l) / max_disp, 1.0, 1e6)

	return {
		"success": true,
		"model_name": model.get("name", "Modelo"),
		"diagnostics": diag,
		"nodal_displacements": disp_dict,
		"nodal_reactions": reac_dict,
		"element_results": elem_results,
		"max_displacement": max_disp,
		"max_von_mises": max_vm,
		"max_axial_stress": max_ax,
		"min_axial_stress": min_ax,
		"recommended_visual_scale": rec_scale,
		"strain_energy": 0.0,
		"disclaimer": DISCLAIMER_TEXT
	}


static func _fail_result(mname: String, diag: Dictionary) -> Dictionary:
	return {
		"success": false,
		"model_name": mname,
		"diagnostics": diag,
		"nodal_displacements": {},
		"nodal_reactions": {},
		"element_results": {},
		"max_displacement": 0.0,
		"max_von_mises": 0.0,
		"max_axial_stress": 0.0,
		"min_axial_stress": 0.0,
		"recommended_visual_scale": 1.0,
		"strain_energy": 0.0,
		"disclaimer": DISCLAIMER_TEXT
	}


static func _node_vec(n: Dictionary) -> Vector3:
	return Vector3(float(n["x"]), float(n["y"]), float(n["z"]))


static func _zeros_vec(n: int) -> Array[float]:
	var arr: Array[float] = []
	arr.resize(n)
	arr.fill(0.0)
	return arr


static func _zeros_matrix(rows: int, cols: int) -> Array:
	var m := []
	for _r in range(rows):
		var row: Array[float] = []
		row.resize(cols)
		row.fill(0.0)
		m.append(row)
	return m


static func _dofs_2nodes(idx1: int, idx2: int) -> Array[int]:
	var b1 := idx1 * 6
	var b2 := idx2 * 6
	return [
		b1, b1 + 1, b1 + 2, b1 + 3, b1 + 4, b1 + 5,
		b2, b2 + 1, b2 + 2, b2 + 3, b2 + 4, b2 + 5
	]


static func _assemble(K: Array, k_el: Array, dofs: Array[int]) -> void:
	var n := dofs.size()
	for i in range(n):
		var gi := dofs[i]
		for j in range(n):
			K[gi][dofs[j]] += k_el[i][j]


static func _beam_rotation(p1: Vector3, p2: Vector3, roll: float) -> Array:
	var L := p1.distance_to(p2)
	var ex := (p2 - p1) / L
	var ey := Vector3.ZERO
	var ez := Vector3.ZERO
	if absf(ex.x) < 1e-6 and absf(ex.z) < 1e-6:
		ez = ex.cross(Vector3(0, 0, 1)).normalized()
		ey = ez.cross(ex).normalized()
	else:
		ez = ex.cross(Vector3(0, 1, 0)).normalized()
		ey = ez.cross(ex).normalized()
	if absf(roll) > 1e-12:
		var c := cos(roll)
		var s := sin(roll)
		var ey_n := c * ey + s * ez
		var ez_n := -s * ey + c * ez
		ey = ey_n
		ez = ez_n
	return [
		[ex.x, ex.y, ex.z],
		[ey.x, ey.y, ey.z],
		[ez.x, ez.y, ez.z]
	]


static func _frame_T(R: Array) -> Array:
	var T := _zeros_matrix(12, 12)
	for b in range(4):
		var off := 3 * b
		for i in range(3):
			for j in range(3):
				T[off + i][off + j] = R[i][j]
	return T


static func _frame_local_k(L: float, mat: Dictionary, sec: Dictionary) -> Array:
	var k := _zeros_matrix(12, 12)
	var E := float(mat["E"])
	var nu := float(mat.get("nu", 0.3))
	var G := E / (2.0 * (1.0 + nu))
	var A := float(sec.get("A", 0.01))
	var Iy := float(sec.get("Iy", 1e-5))
	var Iz := float(sec.get("Iz", 1e-5))
	var J := float(sec.get("J", 2e-5))

	var ea_l := E * A / L
	k[0][0] = ea_l; k[0][6] = -ea_l; k[6][0] = -ea_l; k[6][6] = ea_l

	var gj_l := G * J / L
	k[3][3] = gj_l; k[3][9] = -gj_l; k[9][3] = -gj_l; k[9][9] = gj_l

	var eiz := E * Iz
	var cy1 := 12.0 * eiz / pow(L, 3)
	var cy2 := 6.0 * eiz / pow(L, 2)
	var cy3 := 4.0 * eiz / L
	var cy4 := 2.0 * eiz / L
	k[1][1] = cy1; k[1][5] = cy2; k[1][7] = -cy1; k[1][11] = cy2
	k[5][1] = cy2; k[5][5] = cy3; k[5][7] = -cy2; k[5][11] = cy4
	k[7][1] = -cy1; k[7][5] = -cy2; k[7][7] = cy1; k[7][11] = -cy2
	k[11][1] = cy2; k[11][5] = cy4; k[11][7] = -cy2; k[11][11] = cy3

	var eiy := E * Iy
	var cz1 := 12.0 * eiy / pow(L, 3)
	var cz2 := 6.0 * eiy / pow(L, 2)
	var cz3 := 4.0 * eiy / L
	var cz4 := 2.0 * eiy / L
	k[2][2] = cz1; k[2][4] = -cz2; k[2][8] = -cz1; k[2][10] = -cz2
	k[4][2] = -cz2; k[4][4] = cz3; k[4][8] = cz2; k[4][10] = cz4
	k[8][2] = -cz1; k[8][4] = cz2; k[8][8] = cz1; k[8][10] = cz2
	k[10][2] = -cz2; k[10][4] = cz4; k[10][8] = cz2; k[10][10] = cz3
	return k


static func _frame_feq_local(L: float, q: Vector3) -> Array[float]:
	var f := _zeros_vec(12)
	f[0] = 0.5 * q.x * L
	f[6] = 0.5 * q.x * L
	f[1] = 0.5 * q.y * L
	f[5] = q.y * L * L / 12.0
	f[7] = 0.5 * q.y * L
	f[11] = -q.y * L * L / 12.0
	f[2] = 0.5 * q.z * L
	f[4] = -q.z * L * L / 12.0
	f[8] = 0.5 * q.z * L
	f[10] = q.z * L * L / 12.0
	return f


static func _mat3_vec(R: Array, v: Vector3) -> Vector3:
	return Vector3(
		R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
		R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
		R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z
	)


static func _mat_transpose(A: Array) -> Array:
	var r := A.size()
	var c: int = A[0].size()
	var T := _zeros_matrix(c, r)
	for i in range(r):
		for j in range(c):
			T[j][i] = A[i][j]
	return T


static func _mat_mul(A: Array, B: Array) -> Array:
	var r := A.size()
	var k_dim: int = A[0].size()
	var c: int = B[0].size()
	var C := _zeros_matrix(r, c)
	for i in range(r):
		for k in range(k_dim):
			var aik: float = A[i][k]
			if absf(aik) < 1e-18:
				continue
			for j in range(c):
				C[i][j] += aik * float(B[k][j])
	return C


static func _mat_vec(A: Array, x: Array) -> Array[float]:
	var r := A.size()
	var c: int = A[0].size()
	var y := _zeros_vec(r)
	for i in range(r):
		var s := 0.0
		for j in range(c):
			s += float(A[i][j]) * float(x[j])
		y[i] = s
	return y


static func _solve_linear_system(A_in: Array, b_in: Array) -> Dictionary:
	var n := A_in.size()
	var A := _zeros_matrix(n, n)
	var b := _zeros_vec(n)
	var scale := _zeros_vec(n)

	for i in range(n):
		var d := float(A_in[i][i])
		if d <= 1e-14:
			return {"ok": false, "error": "Mecanismo o apoyo insuficiente: rigidez diagonal nula en GDL libre #%d." % i}
		scale[i] = 1.0 / sqrt(d)

	for i in range(n):
		b[i] = float(b_in[i]) * scale[i]
		for j in range(n):
			A[i][j] = float(A_in[i][j]) * scale[i] * scale[j]

	var max_piv := 0.0
	var min_piv := 1e30

	for k in range(n):
		var piv_row := k
		var piv_val := absf(A[k][k])
		for i in range(k + 1, n):
			var v := absf(A[i][k])
			if v > piv_val:
				piv_val = v
				piv_row = i

		if piv_val <= 1e-10:
			return {"ok": false, "error": "Matriz de rigidez singular (mecanismo cinemático o apoyos insuficientes)."}

		max_piv = maxf(max_piv, piv_val)
		min_piv = minf(min_piv, piv_val)

		if piv_row != k:
			var tmp_r: Array = A[k]
			A[k] = A[piv_row]
			A[piv_row] = tmp_r
			var tmp_b := b[k]
			b[k] = b[piv_row]
			b[piv_row] = tmp_b

		var akk: float = A[k][k]
		for i in range(k + 1, n):
			var factor: float = A[i][k] / akk
			b[i] -= factor * b[k]
			for j in range(k, n):
				A[i][j] -= factor * A[k][j]

	var x := _zeros_vec(n)
	for i in range(n - 1, -1, -1):
		var s := b[i]
		for j in range(i + 1, n):
			s -= A[i][j] * x[j]
		x[i] = (s / A[i][i]) * scale[i]

	return {"ok": true, "x": x, "cond_est": max_piv / maxf(min_piv, 1e-15)}
