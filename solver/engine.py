"""
Motor de Resolución Numérica por el Método de Rigidez Directa (DSM) — Structura 3D.

Resuelve el sistema lineal K · u = F para estructuras de barras 3D (TRUSS_3D),
pórticos y vigas espaciales (FRAME_3D), membranas triangulares (TRIANGLE_3D)
y sólidos tetraédricos (TETRA_4).

Incluye:
- Validación previa de topología, geometría, materiales y condiciones de contorno.
- Detección de mecanismos, hipostaticidad, modos de energía nula y mal condicionamiento.
- Tratamiento exacto de restricciones homogéneas y desplazamientos prescritos no nulos.
- Verificación automática de equilibrio global de fuerzas y momentos en 3D.
"""

from __future__ import annotations

import math
from typing import Dict, List, Set, Tuple
import numpy as np

from .model import (
    StructuralModel,
    Node3D,
    Material,
    CrossSection,
    Element,
    ElementType,
    SolverDiagnostics,
    SolverResult,
    ElementResult,
)
from .elements import (
    beam_rotation_matrix_3d,
    frame3d_transformation_matrix,
    truss3d_stiffness_global,
    truss3d_recover_results,
    frame3d_stiffness_global,
    frame3d_equivalent_nodal_loads_local,
    frame3d_recover_results,
    triangle3d_stiffness_global,
    triangle3d_recover_results,
    tetra4_stiffness_global,
    tetra4_recover_results,
)


def solve_structure(model: StructuralModel) -> SolverResult:
    """
    Ejecuta el análisis estático lineal completo sobre `model` y devuelve `SolverResult`.
    Nunca lanza excepciones no controladas por singularidad o datos inválidos; en su lugar,
    registra diagnósticos detallados en `result.diagnostics`.
    """
    diag = SolverDiagnostics()

    # 1. Validaciones básicas de existencia
    if not model.nodes:
        diag.is_valid = False
        diag.errors.append("El modelo no contiene nodos.")
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    if not model.elements:
        diag.is_valid = False
        diag.errors.append("El modelo no contiene elementos estructurales.")
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    # Indexar entidades y verificar duplicados
    node_map: Dict[int, Node3D] = {}
    node_index: Dict[int, int] = {}
    for idx, n in enumerate(model.nodes):
        if n.id in node_map:
            diag.is_valid = False
            diag.errors.append(f"ID de nodo duplicado: {n.id}.")
        if not (math.isfinite(n.x) and math.isfinite(n.y) and math.isfinite(n.z)):
            diag.is_valid = False
            diag.errors.append(f"El nodo {n.id} tiene coordenadas no finitas (NaN/Inf).")
        node_map[n.id] = n
        node_index[n.id] = idx

    mat_map: Dict[int, Material] = {}
    for m in model.materials:
        if m.E <= 0.0 or not math.isfinite(m.E):
            diag.is_valid = False
            diag.errors.append(f"Material '{m.name}' (id={m.id}) tiene módulo de Young inválido E={m.E}.")
        if m.nu <= -1.0 or m.nu >= 0.5 or not math.isfinite(m.nu):
            diag.is_valid = False
            diag.errors.append(
                f"Material '{m.name}' (id={m.id}) tiene coeficiente de Poisson inválido nu={m.nu} (debe estar en (-1, 0.5))."
            )
        if not (math.isfinite(m.rho) and math.isfinite(m.yield_strength)):
            diag.is_valid = False
            diag.errors.append(f"Material '{m.name}' (id={m.id}) contiene propiedades físicas no finitas (NaN/Inf).")
        mat_map[m.id] = m

    sec_map: Dict[int, CrossSection] = {}
    for s in model.sections:
        if s.A <= 0.0 or not math.isfinite(s.A):
            diag.is_valid = False
            diag.errors.append(f"Sección '{s.name}' (id={s.id}) tiene área no positiva o no finita A={s.A}.")
        if not (math.isfinite(s.Iy) and math.isfinite(s.Iz) and math.isfinite(s.J) and math.isfinite(s.thickness)):
            diag.is_valid = False
            diag.errors.append(f"Sección '{s.name}' (id={s.id}) contiene propiedades geométricas no finitas (NaN/Inf).")
        sec_map[s.id] = s

    if not diag.is_valid:
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    # 2. Verificar conectividad de elementos y nodos activos / rotacionales
    connected_nodes: Set[int] = set()
    rotational_nodes: Set[int] = set()

    for elem in model.elements:
        if not math.isfinite(elem.roll_angle):
            diag.is_valid = False
            diag.errors.append(f"Elemento {elem.id} tiene ángulo de giro (roll_angle) no finito: {elem.roll_angle}.")
        if elem.material_id not in mat_map:
            diag.is_valid = False
            diag.errors.append(f"Elemento {elem.id} referencia un material inexistente (id={elem.material_id}).")
        if elem.type in (ElementType.TRUSS_3D.value, ElementType.FRAME_3D.value, ElementType.TRIANGLE_3D.value):
            if elem.section_id not in sec_map:
                diag.is_valid = False
                diag.errors.append(f"Elemento {elem.id} referencia una sección inexistente (id={elem.section_id}).")
            else:
                sec = sec_map[elem.section_id]
                if elem.type == ElementType.FRAME_3D.value and (sec.Iy <= 0.0 or sec.Iz <= 0.0 or sec.J <= 0.0):
                    diag.is_valid = False
                    diag.errors.append(
                        f"Sección '{sec.name}' (id={sec.id}) usada en viga FRAME_3D (elem {elem.id}) requiere Iy, Iz, J > 0."
                    )
                if elem.type == ElementType.TRIANGLE_3D.value and sec.thickness <= 0.0:
                    diag.is_valid = False
                    diag.errors.append(
                        f"Sección '{sec.name}' (id={sec.id}) usada en membrana TRIANGLE_3D (elem {elem.id}) requiere thickness > 0 (recibido {sec.thickness})."
                    )

        expected_n = {
            ElementType.TRUSS_3D.value: 2,
            ElementType.FRAME_3D.value: 2,
            ElementType.TRIANGLE_3D.value: 3,
            ElementType.TETRA_4.value: 4,
        }.get(elem.type, -1)

        if expected_n == -1:
            diag.is_valid = False
            diag.errors.append(f"Elemento {elem.id} tiene un tipo desconocido '{elem.type}'.")
            continue

        if len(elem.nodes) != expected_n:
            diag.is_valid = False
            diag.errors.append(
                f"Elemento {elem.id} ({elem.type}) requiere {expected_n} nodos, pero tiene {len(elem.nodes)}."
            )
            continue

        if len(set(elem.nodes)) != len(elem.nodes):
            diag.is_valid = False
            diag.errors.append(f"Elemento {elem.id} conecta un mismo nodo consigo mismo: {elem.nodes}.")
            continue

        for nid in elem.nodes:
            if nid not in node_map:
                diag.is_valid = False
                diag.errors.append(f"Elemento {elem.id} referencia un nodo inexistente (id={nid}).")
            else:
                connected_nodes.add(nid)
                if elem.type == ElementType.FRAME_3D.value:
                    rotational_nodes.add(nid)

    if not diag.is_valid:
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    # 3. Dimensionamiento del sistema global (6 GDL por nodo: ux, uy, uz, rx, ry, rz)
    num_nodes = len(model.nodes)
    total_dofs = 6 * num_nodes
    diag.total_dofs = total_dofs

    K_global = np.zeros((total_dofs, total_dofs), dtype=float)
    F_nodal = np.zeros(total_dofs, dtype=float)
    F_equiv = np.zeros(total_dofs, dtype=float)

    # Acumulador de cargas distribuidas por elemento en coordenadas locales (para FRAME_3D / TRUSS_3D)
    elem_q_local: Dict[int, np.ndarray] = {e.id: np.zeros(3, dtype=float) for e in model.elements}
    elem_map: Dict[int, Element] = {e.id: e for e in model.elements}

    # 4. Ensamblaje de matrices de rigidez elementales
    characteristic_lengths: List[float] = []

    try:
        for elem in model.elements:
            mat = mat_map[elem.material_id]
            if elem.type == ElementType.TRUSS_3D.value:
                sec = sec_map[elem.section_id]
                n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
                k_el, L, _ = truss3d_stiffness_global(n1, n2, mat, sec)
                characteristic_lengths.append(L)
                dofs = _node_dofs_6(node_index[n1.id]) + _node_dofs_6(node_index[n2.id])
                _assemble_into(K_global, k_el, dofs)

            elif elem.type == ElementType.FRAME_3D.value:
                sec = sec_map[elem.section_id]
                n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
                k_el, _, _, L = frame3d_stiffness_global(n1, n2, mat, sec, roll_angle=elem.roll_angle)
                characteristic_lengths.append(L)
                dofs = _node_dofs_6(node_index[n1.id]) + _node_dofs_6(node_index[n2.id])
                _assemble_into(K_global, k_el, dofs)

            elif elem.type == ElementType.TRIANGLE_3D.value:
                sec = sec_map[elem.section_id]
                n1, n2, n3 = (
                    node_map[elem.nodes[0]],
                    node_map[elem.nodes[1]],
                    node_map[elem.nodes[2]],
                )
                k_el_9, _, _, _, area = triangle3d_stiffness_global(n1, n2, n3, mat, sec)
                characteristic_lengths.append(math.sqrt(area))
                dofs_9 = (
                    _node_dofs_3_trans(node_index[n1.id])
                    + _node_dofs_3_trans(node_index[n2.id])
                    + _node_dofs_3_trans(node_index[n3.id])
                )
                _assemble_into(K_global, k_el_9, dofs_9)

            elif elem.type == ElementType.TETRA_4.value:
                n1, n2, n3, n4 = (
                    node_map[elem.nodes[0]],
                    node_map[elem.nodes[1]],
                    node_map[elem.nodes[2]],
                    node_map[elem.nodes[3]],
                )
                k_el_12, _, _, vol = tetra4_stiffness_global(n1, n2, n3, n4, mat)
                characteristic_lengths.append(vol ** (1.0 / 3.0))
                dofs_12 = (
                    _node_dofs_3_trans(node_index[n1.id])
                    + _node_dofs_3_trans(node_index[n2.id])
                    + _node_dofs_3_trans(node_index[n3.id])
                    + _node_dofs_3_trans(node_index[n4.id])
                )
                _assemble_into(K_global, k_el_12, dofs_12)

    except ValueError as exc:
        diag.is_valid = False
        diag.errors.append(str(exc))
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    # 5. Ensamblaje de fuerzas nodales puntuales
    moment_loaded_nodes: Set[int] = set()
    loaded_nodes: Set[int] = set()
    for nl in model.nodal_loads:
        if nl.node_id not in node_index:
            diag.is_valid = False
            diag.errors.append(f"Carga nodal referencia un nodo inexistente (id={nl.node_id}).")
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        base = 6 * node_index[nl.node_id]
        vec = nl.vector()
        if not all(math.isfinite(v) for v in vec):
            diag.is_valid = False
            diag.errors.append(f"Carga nodal en nodo {nl.node_id} contiene valores NaN o Inf.")
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        for j in range(6):
            F_nodal[base + j] += vec[j]
        if any(abs(v) > 1e-12 for v in vec):
            loaded_nodes.add(nl.node_id)
        if abs(nl.mx) > 1e-12 or abs(nl.my) > 1e-12 or abs(nl.mz) > 1e-12:
            moment_loaded_nodes.add(nl.node_id)

    # 6. Ensamblaje de cargas distribuidas sobre barras y vigas
    for dl in model.distributed_loads:
        if dl.element_id not in elem_map:
            diag.is_valid = False
            diag.errors.append(f"Carga distribuida referencia un elemento inexistente (id={dl.element_id}).")
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        if not (math.isfinite(dl.qx) and math.isfinite(dl.qy) and math.isfinite(dl.qz)):
            diag.is_valid = False
            diag.errors.append(
                f"Carga distribuida en elemento {dl.element_id} contiene valores no finitos (NaN/Inf)."
            )
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        elem = elem_map[dl.element_id]
        if elem.type not in (ElementType.FRAME_3D.value, ElementType.TRUSS_3D.value):
            diag.warnings.append(
                f"Carga distribuida en elemento {elem.id} ({elem.type}) ignorada (sólo aplicable a barras/vigas)."
            )
            continue

        n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
        p1 = np.array(n1.coords(), dtype=float)
        p2 = np.array(n2.coords(), dtype=float)
        R, L = beam_rotation_matrix_3d(p1, p2, roll_angle=elem.roll_angle)
        q_vec = np.array([dl.qx, dl.qy, dl.qz], dtype=float)
        if dl.coordinate_system.upper() == "GLOBAL":
            q_local = R @ q_vec
        else:
            q_local = q_vec

        elem_q_local[elem.id] += q_local

        if elem.type == ElementType.FRAME_3D.value:
            T = frame3d_transformation_matrix(R)
            feq_local = frame3d_equivalent_nodal_loads_local(L, q_local)
            feq_global = T.T @ feq_local
            dofs = _node_dofs_6(node_index[n1.id]) + _node_dofs_6(node_index[n2.id])
            for local_i, global_dof in enumerate(dofs):
                F_equiv[global_dof] += feq_global[local_i]
        else:
            # En TRUSS_3D las cargas distribuidas se reparten mitad y mitad en los nudos extremos
            q_global = (R.T @ q_local) if dl.coordinate_system.upper() != "GLOBAL" else q_vec
            f_half = 0.5 * L * q_global
            b1 = 6 * node_index[n1.id]
            b2 = 6 * node_index[n2.id]
            F_equiv[b1 : b1 + 3] += f_half
            F_equiv[b2 : b2 + 3] += f_half

    # 7. Tratamiento de condiciones de apoyo, nodos huérfanos y GDL rotacionales inactivos
    restrained_mask = np.zeros(total_dofs, dtype=bool)
    user_restrained_mask = np.zeros(total_dofs, dtype=bool)
    u_prescribed = np.zeros(total_dofs, dtype=float)

    for bc in model.boundary_conditions:
        if bc.node_id not in node_index:
            diag.is_valid = False
            diag.errors.append(f"Condición de apoyo referencia un nodo inexistente (id={bc.node_id}).")
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        base = 6 * node_index[bc.node_id]
        flags = bc.flags()
        presc = bc.prescribed_values()
        if not all(math.isfinite(v) for v in presc):
            diag.is_valid = False
            diag.errors.append(
                f"Condición de apoyo en nodo {bc.node_id} contiene desplazamientos prescritos no finitos (NaN/Inf)."
            )
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        for j in range(6):
            if flags[j]:
                if user_restrained_mask[base + j] and abs(u_prescribed[base + j] - presc[j]) > 1e-12:
                    diag.is_valid = False
                    diag.errors.append(
                        f"Condiciones de apoyo incompatibles en nodo {bc.node_id} (GDL {j}): "
                        f"desplazamientos prescritos contradictorios ({u_prescribed[base + j]} vs {presc[j]})."
                    )
                    return SolverResult(success=False, model_name=model.name, diagnostics=diag)
                restrained_mask[base + j] = True
                user_restrained_mask[base + j] = True
                u_prescribed[base + j] = presc[j]

    user_constrained_count = int(np.sum(user_restrained_mask))

    # Política explícita para nodos huérfanos (desconectados de todo elemento) y GDL rotacionales inactivos:
    # - Si un nodo huérfano tiene cargas aplicadas o desplazamientos prescritos no nulos, se rechaza el modelo
    #   por constituir un nudo desconectado solicitado (mecanismo/condición mal definida).
    # - Si un nodo huérfano es puramente pasivo (sin cargas ni desplazamientos impuestos), se emite una
    #   advertencia explícita en `diag.warnings` y se excluyen (auto-bloquean) sus 6 GDL inactivos.
    auto_locked_dofs = np.zeros(total_dofs, dtype=bool)
    for n in model.nodes:
        idx = node_index[n.id]
        base = 6 * idx
        if n.id not in connected_nodes:
            has_nonzero_presc = any(
                user_restrained_mask[base + j] and abs(u_prescribed[base + j]) > 1e-12 for j in range(6)
            )
            if n.id in loaded_nodes or has_nonzero_presc:
                diag.is_valid = False
                diag.is_singular = True
                diag.is_mechanism = True
                diag.errors.append(
                    f"El nodo {n.id} está desconectado de la malla (nodo huérfano) pero tiene cargas "
                    "o desplazamientos prescritos aplicados (mecanismo cinemático)."
                )
                return SolverResult(success=False, model_name=model.name, diagnostics=diag)

            diag.warnings.append(
                f"El nodo {n.id} está desconectado (nodo huérfano inactivo sin cargas); "
                "sus 6 GDL han sido excluidos del sistema activo."
            )
            for j in range(6):
                if not restrained_mask[base + j]:
                    restrained_mask[base + j] = True
                    auto_locked_dofs[base + j] = True
            continue

        if n.id not in rotational_nodes and n.id not in moment_loaded_nodes:
            for rot_j in (3, 4, 5):
                if not restrained_mask[base + rot_j]:
                    restrained_mask[base + rot_j] = True
                    auto_locked_dofs[base + rot_j] = True

    free_dofs = np.where(~restrained_mask)[0]
    rest_dofs = np.where(restrained_mask)[0]

    diag.free_dofs = int(len(free_dofs))
    diag.constrained_dofs = int(user_constrained_count)

    if user_constrained_count == 0:
        diag.is_valid = False
        diag.is_singular = True
        diag.is_mechanism = True
        diag.errors.append("Condiciones de apoyo insuficientes: la estructura no tiene ningún apoyo definido.")
        return SolverResult(success=False, model_name=model.name, diagnostics=diag)

    if len(free_dofs) == 0:
        diag.warnings.append("Todos los grados de libertad activos están restringidos.")

    # 8. Partición del sistema lineal y diagnóstico espectral de estabilidad (K_ff)
    F_total = F_nodal + F_equiv
    u_global = np.zeros(total_dofs, dtype=float)
    u_global[rest_dofs] = u_prescribed[rest_dofs]

    if len(free_dofs) > 0:
        K_ff = K_global[np.ix_(free_dofs, free_dofs)]
        K_fr = K_global[np.ix_(free_dofs, rest_dofs)]
        rhs_f = F_total[free_dofs] - K_fr @ u_global[rest_dofs]

        # Comprobar diagonal nula en K_ff (nudo libre sin rigidez en alguna dirección)
        diag_kff = np.diag(K_ff)
        zero_diag_indices = np.where(diag_kff <= 1e-14)[0]
        if len(zero_diag_indices) > 0:
            diag.is_valid = False
            diag.is_singular = True
            diag.is_mechanism = True
            diag.zero_energy_modes = int(len(zero_diag_indices))
            bad_dof = int(free_dofs[zero_diag_indices[0]])
            bad_node_id = model.nodes[bad_dof // 6].id
            dof_names = ["ux", "uy", "uz", "rx", "ry", "rz"]
            diag.errors.append(
                f"Mecanismo o condición de apoyo insuficiente detectada: rigidez nula en nodo {bad_node_id} "
                f"(GDL '{dof_names[bad_dof % 6]}'). Verifique apoyos en 3D o articulaciones colineales."
            )
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)

        # Escalado simétrico de Jacobi D^(-1/2) K_ff D^(-1/2) para evaluar rango y condicionamiento
        # sin sesgo numérico entre rigideces traslacionales (EA/L) y flexionales (EI/L)
        scale_inv = 1.0 / np.sqrt(diag_kff)
        K_scaled = (K_ff * scale_inv[:, None]) * scale_inv[None, :]

        # Valores singulares / autovalores de la matriz escalada
        eigvals = np.linalg.eigvalsh(K_scaled)
        max_eig = float(np.max(np.abs(eigvals))) if len(eigvals) > 0 else 0.0
        min_eig = float(np.min(eigvals)) if len(eigvals) > 0 else 0.0

        rank_tol = 1e-10 * max(1.0, max_eig)
        zero_modes = int(np.sum(eigvals <= rank_tol))
        diag.zero_energy_modes = zero_modes
        diag.matrix_rank = int(len(free_dofs) - zero_modes)

        if zero_modes > 0 or min_eig <= rank_tol:
            diag.is_valid = False
            diag.is_singular = True
            diag.is_mechanism = True
            diag.condition_number = float("inf")
            diag.errors.append(
                f"Sistema estructural inestable (mecanismo cinemático o apoyos insuficientes): "
                f"se detectaron {zero_modes} modo(s) de energía nula en K_ff "
                f"(rango={diag.matrix_rank}/{len(free_dofs)})."
            )
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)

        cond_scaled = max_eig / min_eig
        diag.condition_number = float(cond_scaled)
        if cond_scaled > 1e12:
            diag.is_valid = False
            diag.is_singular = True
            diag.errors.append(
                f"Matriz de rigidez severamente mal condicionada (kappa = {cond_scaled:.3e} > 1e12). "
                "Posible cuasi-mecanismo o disparidad extrema de rigideces."
            )
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)
        elif cond_scaled > 1e8:
            diag.warnings.append(
                f"Número de condición elevado en la matriz escalada (kappa = {cond_scaled:.3e})."
            )

        # Resolución del sistema lineal escalado para máxima precisión numérica
        rhs_scaled = rhs_f * scale_inv
        u_scaled = np.linalg.solve(K_scaled, rhs_scaled)
        u_f = u_scaled * scale_inv

        if not np.all(np.isfinite(u_f)):
            diag.is_valid = False
            diag.errors.append("La solución numérica produjo desplazamientos no finitos (NaN/Inf).")
            return SolverResult(success=False, model_name=model.name, diagnostics=diag)

        u_global[free_dofs] = u_f

    # 9. Cálculo de reacciones en los apoyos: R = K_global * u_global - F_total
    R_global = K_global @ u_global - F_total
    # Limpiar ruido numérico en GDL libres y GDL rotacionales auto-bloqueados
    R_global[free_dofs] = 0.0
    R_global[auto_locked_dofs] = 0.0

    # 10. Verificación rigurosa de equilibrio estático global (Suma de Fuerzas = 0 y Suma de Momentos = 0)
    f_res, m_res, eq_ok = _verify_global_equilibrium(model, node_map, node_index, R_global)
    diag.force_residual_norm = f_res
    diag.moment_residual_norm = m_res
    diag.equilibrium_satisfied = eq_ok
    if not eq_ok:
        diag.warnings.append(
            f"Residuo de equilibrio global superior a la tolerancia: |dF|={f_res:.3e} N, |dM|={m_res:.3e} N·m."
        )

    # 11. Recuperación de esfuerzos internos y tensiones por elemento
    element_results: Dict[int, ElementResult] = {}
    max_vm = 0.0
    max_axial = 0.0
    min_axial = 0.0

    for idx_e, elem in enumerate(model.elements):
        mat = mat_map[elem.material_id]
        if elem.type == ElementType.TRUSS_3D.value:
            sec = sec_map[elem.section_id]
            n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
            dofs = _node_dofs_6(node_index[n1.id]) + _node_dofs_6(node_index[n2.id])
            res = truss3d_recover_results(elem.id, n1, n2, mat, sec, u_global[dofs])

        elif elem.type == ElementType.FRAME_3D.value:
            sec = sec_map[elem.section_id]
            n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
            dofs = _node_dofs_6(node_index[n1.id]) + _node_dofs_6(node_index[n2.id])
            res = frame3d_recover_results(
                elem.id,
                n1,
                n2,
                mat,
                sec,
                elem.roll_angle,
                u_global[dofs],
                elem_q_local[elem.id],
            )

        elif elem.type == ElementType.TRIANGLE_3D.value:
            sec = sec_map[elem.section_id]
            n1, n2, n3 = (
                node_map[elem.nodes[0]],
                node_map[elem.nodes[1]],
                node_map[elem.nodes[2]],
            )
            dofs_9 = (
                _node_dofs_3_trans(node_index[n1.id])
                + _node_dofs_3_trans(node_index[n2.id])
                + _node_dofs_3_trans(node_index[n3.id])
            )
            res = triangle3d_recover_results(elem.id, n1, n2, n3, mat, sec, u_global[dofs_9])

        elif elem.type == ElementType.TETRA_4.value:
            n1, n2, n3, n4 = (
                node_map[elem.nodes[0]],
                node_map[elem.nodes[1]],
                node_map[elem.nodes[2]],
                node_map[elem.nodes[3]],
            )
            dofs_12 = (
                _node_dofs_3_trans(node_index[n1.id])
                + _node_dofs_3_trans(node_index[n2.id])
                + _node_dofs_3_trans(node_index[n3.id])
                + _node_dofs_3_trans(node_index[n4.id])
            )
            res = tetra4_recover_results(elem.id, n1, n2, n3, n4, mat, u_global[dofs_12])
        else:
            continue

        element_results[elem.id] = res
        max_vm = max(max_vm, res.von_mises_stress)
        if idx_e == 0:
            max_axial = res.axial_stress
            min_axial = res.axial_stress
        else:
            max_axial = max(max_axial, res.axial_stress)
            min_axial = min(min_axial, res.axial_stress)

    # 12. Empaquetar desplazamientos y reacciones nodales y calcular escala visual recomendada
    nodal_displacements: Dict[int, List[float]] = {}
    nodal_reactions: Dict[int, List[float]] = {}
    max_disp = 0.0

    for n in model.nodes:
        base = 6 * node_index[n.id]
        u_node = u_global[base : base + 6]
        r_node = R_global[base : base + 6]
        nodal_displacements[n.id] = [float(x) for x in u_node]
        nodal_reactions[n.id] = [float(x) for x in r_node]
        disp_mag = float(np.linalg.norm(u_node[0:3]))
        max_disp = max(max_disp, disp_mag)

    avg_len = float(np.mean(characteristic_lengths)) if characteristic_lengths else 1.0
    if max_disp > 1e-12:
        # Escala visual para que la deformación máxima aparente sea ~12% de la longitud característica
        rec_scale = round((0.12 * avg_len) / max_disp, 3)
        rec_scale = max(1.0, min(rec_scale, 1e6))
    else:
        rec_scale = 1.0

    strain_energy = float(0.5 * (u_global @ (K_global @ u_global)))

    return SolverResult(
        success=True,
        model_name=model.name,
        diagnostics=diag,
        nodal_displacements=nodal_displacements,
        nodal_reactions=nodal_reactions,
        element_results=element_results,
        max_displacement=max_disp,
        max_von_mises=max_vm,
        max_axial_stress=max_axial,
        min_axial_stress=min_axial,
        recommended_visual_scale=rec_scale,
        strain_energy=strain_energy,
    )


def _node_dofs_6(idx: int) -> List[int]:
    base = 6 * idx
    return [base, base + 1, base + 2, base + 3, base + 4, base + 5]


def _node_dofs_3_trans(idx: int) -> List[int]:
    base = 6 * idx
    return [base, base + 1, base + 2]


def _assemble_into(K_global: np.ndarray, k_elem: np.ndarray, dofs: List[int]) -> None:
    idx_grid = np.ix_(dofs, dofs)
    K_global[idx_grid] += k_elem


def _verify_global_equilibrium(
    model: StructuralModel,
    node_map: Dict[int, Node3D],
    node_index: Dict[int, int],
    R_global: np.ndarray,
) -> Tuple[float, float, bool]:
    """
    Comprueba que la resultante de fuerzas externas + reacciones sea nula
    y que el momento resultante respecto al origen (0,0,0) sea nulo.
    """
    total_force = np.zeros(3, dtype=float)
    total_moment = np.zeros(3, dtype=float)
    ref_force_scale = 1.0

    # Fuerzas y momentos nodales externos
    for nl in model.nodal_loads:
        n = node_map[nl.node_id]
        r = np.array(n.coords(), dtype=float)
        f = np.array([nl.fx, nl.fy, nl.fz], dtype=float)
        m = np.array([nl.mx, nl.my, nl.mz], dtype=float)
        total_force += f
        total_moment += np.cross(r, f) + m
        ref_force_scale = max(ref_force_scale, float(np.linalg.norm(f)), float(np.linalg.norm(m)))

    # Cargas distribuidas externas
    elem_map = {e.id: e for e in model.elements}
    for dl in model.distributed_loads:
        elem = elem_map.get(dl.element_id)
        if not elem or elem.type not in (ElementType.FRAME_3D.value, ElementType.TRUSS_3D.value):
            continue
        n1, n2 = node_map[elem.nodes[0]], node_map[elem.nodes[1]]
        p1 = np.array(n1.coords(), dtype=float)
        p2 = np.array(n2.coords(), dtype=float)
        R, L = beam_rotation_matrix_3d(p1, p2, roll_angle=elem.roll_angle)
        q_vec = np.array([dl.qx, dl.qy, dl.qz], dtype=float)
        q_global = q_vec if dl.coordinate_system.upper() == "GLOBAL" else (R.T @ q_vec)
        f_res = q_global * L
        midpoint = 0.5 * (p1 + p2)
        total_force += f_res
        total_moment += np.cross(midpoint, f_res)
        ref_force_scale = max(ref_force_scale, float(np.linalg.norm(f_res)))

    # Reacciones en los apoyos
    for n in model.nodes:
        base = 6 * node_index[n.id]
        r_pos = np.array(n.coords(), dtype=float)
        r_f = R_global[base : base + 3]
        r_m = R_global[base + 3 : base + 6]
        total_force += r_f
        total_moment += np.cross(r_pos, r_f) + r_m
        ref_force_scale = max(ref_force_scale, float(np.linalg.norm(r_f)), float(np.linalg.norm(r_m)))

    force_norm = float(np.linalg.norm(total_force))
    moment_norm = float(np.linalg.norm(total_moment))
    tol = 1e-6 * ref_force_scale
    return force_norm, moment_norm, (force_norm <= tol and moment_norm <= tol)
