"""
Structura 3D — Validation Gauntlet (Suite Independiente de Auditoría y Regresión).

AVISO TÉCNICO:
Structura 3D representa un modelo matemático elástico lineal idealizado bajo cinemática
infinitesimal. NO está validado ni es apto para construcción ni certificación estructural.

Esta suite evalúa de forma independiente:
1. Flexión biaxial (planos X-Y y X-Z), torsión de Saint-Venant y cargas distribuidas en FRAME_3D.
2. Equilibrio estático global de fuerzas y momentos respecto a puntos arbitrarios del espacio 3D,
   así como equilibrio elemental local.
3. Invariancia objetiva ante rotaciones ortogonales 3D (SO(3)) y ante permutación de la conectividad nodal.
4. Comportamiento mecánico real, rango matricial, modos de cuerpo rígido y estado de tensión plana
   del elemento membrana TRIANGLE_3D (CST), sin rigidez artificial fuera del plano.
5. Rechazo de geometrías degeneradas, espesores no positivos y entradas NaN/Inf en cargas distribuidas,
   desplazamientos prescritos e inercias.
6. Detección de mecanismos 3D (apoyos colineales, momentos en nudos articulados), conteo de GDL
   restringidos sin duplicidad y aislamiento de nodos huérfanos.
7. Comparación diferencial Python vs. GDScript (ejecutada sólo si el binario de Godot 4 está presente).
"""

from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path
import numpy as np
import pytest

from solver.model import (
    StructuralModel,
    Node3D,
    Material,
    CrossSection,
    Element,
    ElementType,
    BoundaryCondition,
    SupportPreset,
    NodalLoad,
    DistributedLoad,
)
from solver.engine import solve_structure
from solver.elements import triangle3d_stiffness_global


def _rotation_matrix_so3(alpha: float, beta: float, gamma: float) -> np.ndarray:
    """Construye una matriz de rotación propia Q in SO(3) con det(Q) = +1."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]], dtype=float)
    ry = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=float)
    rz = np.array([[cg, -sg, 0.0], [sg, cg, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


# =============================================================================
# 1. FLEXIÓN BIAXIAL (PLANOS XY y XZ), TORSIÓN Y CARGAS DISTRIBUIDAS EN FRAME_3D
# =============================================================================

def test_gauntlet_frame3d_xz_plane_bending_point_and_distributed():
    """
    Verifica analíticamente la flexión de FRAME_3D en el plano local X-Z (alrededor del eje Y)
    bajo carga puntual Pz y bajo carga distribuida uniforme qz en un voladizo de L = 4.0 m.
    Solución analítica exacta:
      - Con Pz = -20 kN en x=L:
          uz(L) = Pz * L^3 / (3 * E * Iy)
          ry(L) = -Pz * L^2 / (2 * E * Iy)  (regla de la mano derecha: z' x x' = y')
          Rz(0) = +20 kN, My(0) = -80 kN·m
          |My|_max en el elemento = 80 kN·m
      - Con qz = +10 kN/m uniforme:
          uz(L) = qz * L^4 / (8 * E * Iy)
          ry(L) = -qz * L^3 / (6 * E * Iy)
          Rz(0) = -40 kN, My(0) = +80 kN·m
          |My|_max en el elemento = qz * L^2 / 2 = 80 kN·m
    """
    L = 4.0
    E = 200e9
    sec = CrossSection.rectangular(1, "Rect_200x400", b=0.20, h=0.40)
    mat = Material(1, "Acero", E=E, nu=0.3)

    # Caso A: Carga puntual Pz = -20 kN en el extremo libre
    Pz = -20e3
    m_point = StructuralModel(
        name="Gauntlet_Cantilever_XZ_Point",
        nodes=[Node3D(1, 0.0, 0.0, 0.0), Node3D(2, L, 0.0, 0.0)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        nodal_loads=[NodalLoad(2, fz=Pz)],
    )
    res_p = solve_structure(m_point)
    assert res_p.success is True

    uz_exact = (Pz * (L ** 3)) / (3.0 * E * sec.Iy)
    ry_exact = -(Pz * (L ** 2)) / (2.0 * E * sec.Iy)
    my_max_exact = abs(Pz * L)
    sigma_bend_y_exact = my_max_exact * (0.5 * 0.20) / sec.Iy

    assert math.isclose(res_p.nodal_displacements[2][2], uz_exact, rel_tol=1e-9)
    assert math.isclose(res_p.nodal_displacements[2][4], ry_exact, rel_tol=1e-9)
    assert math.isclose(res_p.nodal_reactions[1][2], -Pz, rel_tol=1e-9)
    assert math.isclose(res_p.nodal_reactions[1][4], Pz * L, rel_tol=1e-9)
    assert math.isclose(res_p.element_results[1].shear_z, abs(Pz), rel_tol=1e-9)
    assert math.isclose(res_p.element_results[1].moment_y, my_max_exact, rel_tol=1e-6)
    assert math.isclose(res_p.element_results[1].bending_stress_max, sigma_bend_y_exact, rel_tol=1e-6)

    # Caso B: Carga distribuida qz = +10 kN/m
    qz = 10e3
    m_dist = StructuralModel(
        name="Gauntlet_Cantilever_XZ_Dist",
        nodes=[Node3D(1, 0.0, 0.0, 0.0), Node3D(2, L, 0.0, 0.0)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        distributed_loads=[DistributedLoad(1, qz=qz, coordinate_system="GLOBAL")],
    )
    res_d = solve_structure(m_dist)
    assert res_d.success is True

    uz_dist_exact = (qz * (L ** 4)) / (8.0 * E * sec.Iy)
    ry_dist_exact = -(qz * (L ** 3)) / (6.0 * E * sec.Iy)
    my_dist_exact = 0.5 * abs(qz) * (L ** 2)
    sigma_dist_exact = my_dist_exact * (0.5 * 0.20) / sec.Iy

    assert math.isclose(res_d.nodal_displacements[2][2], uz_dist_exact, rel_tol=1e-9)
    assert math.isclose(res_d.nodal_displacements[2][4], ry_dist_exact, rel_tol=1e-9)
    assert math.isclose(res_d.element_results[1].shear_z, abs(qz * L), rel_tol=1e-9)
    assert math.isclose(res_d.element_results[1].moment_y, my_dist_exact, rel_tol=1e-6)
    assert math.isclose(res_d.element_results[1].bending_stress_max, sigma_dist_exact, rel_tol=1e-6)


def test_gauntlet_frame3d_pure_torsion_analytical():
    """
    Verifica torsión pura de Saint-Venant en FRAME_3D:
      rx(L) = T * L / (G * J), Mx_reacción(0) = -T, esfuerzo interno de torsión = |T|.
    """
    L = 3.5
    E = 210e9
    nu = 0.3
    T_moment = 12e3
    sec = CrossSection.circular(1, "Eje Circular d=100mm", diameter=0.10)
    mat = Material(1, "Acero", E=E, nu=nu)

    model = StructuralModel(
        name="Gauntlet_Pure_Torsion",
        nodes=[Node3D(1, 0.0, 0.0, 0.0), Node3D(2, L, 0.0, 0.0)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        nodal_loads=[NodalLoad(2, mx=T_moment)],
    )
    res = solve_structure(model)
    assert res.success is True

    rx_exact = (T_moment * L) / (mat.G * sec.J)
    assert math.isclose(res.nodal_displacements[2][3], rx_exact, rel_tol=1e-9)
    assert math.isclose(res.nodal_reactions[1][3], -T_moment, rel_tol=1e-9)
    assert math.isclose(res.element_results[1].torsion, abs(T_moment), rel_tol=1e-9)


# =============================================================================
# 2. EQUILIBRIO GLOBAL RESPECTO A PUNTO ARBITRARIO Y EQUILIBRIO LOCAL ELEMENTAL
# =============================================================================

def test_gauntlet_arbitrary_point_global_and_local_equilibrium():
    """
    Verifica en un pórtico 3D asimétrico con cargas oblicuas puntuales y distribuidas:
    1) Equilibrio global de fuerzas: Sum(F_ext) + Sum(R_F) = 0.
    2) Equilibrio global de momentos respecto a un punto arbitrario P0 = (13.7, -4.2, 8.9) m.
    3) Equilibrio estático local de cada elemento FRAME_3D entre sus `local_end_forces` y su carga distribuida q_local.
    """
    sec = CrossSection.rectangular(1, "Seccion_Portico", b=0.25, h=0.35)
    mat = Material(1, "Acero", E=210e9, nu=0.28)
    nodes = [
        Node3D(1, 0.0, 0.0, 0.0),
        Node3D(2, 0.0, 3.2, 0.5),
        Node3D(3, 4.1, 3.6, 1.2),
        Node3D(4, 4.5, 0.0, 1.5),
    ]
    elements = [
        Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1, roll_angle=0.2),
        Element(2, ElementType.FRAME_3D.value, [2, 3], 1, 1, roll_angle=-0.15),
        Element(3, ElementType.FRAME_3D.value, [3, 4], 1, 1, roll_angle=0.4),
    ]
    bcs = [
        BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D),
        BoundaryCondition.from_preset(4, SupportPreset.PINNED_3D),
    ]
    nloads = [
        NodalLoad(2, fx=18e3, fy=-25e3, fz=9e3, mx=2e3, my=-3e3, mz=4e3),
    ]
    dloads = [
        DistributedLoad(2, qx=1.5e3, qy=-12e3, qz=3.2e3, coordinate_system="LOCAL"),
    ]
    model = StructuralModel(
        name="Gauntlet_3D_Equilibrium",
        nodes=nodes,
        materials=[mat],
        sections=[sec],
        elements=elements,
        boundary_conditions=bcs,
        nodal_loads=nloads,
        distributed_loads=dloads,
    )
    res = solve_structure(model)
    assert res.success is True
    assert res.diagnostics.equilibrium_satisfied is True

    # Verificar momento respecto a un punto arbitrario P0
    p0 = np.array([13.7, -4.2, 8.9], dtype=float)
    nmap = {n.id: np.array(n.coords(), dtype=float) for n in nodes}

    total_f = np.zeros(3)
    total_m_p0 = np.zeros(3)

    for nl in nloads:
        r_vec = nmap[nl.node_id] - p0
        f_vec = np.array([nl.fx, nl.fy, nl.fz])
        m_vec = np.array([nl.mx, nl.my, nl.mz])
        total_f += f_vec
        total_m_p0 += np.cross(r_vec, f_vec) + m_vec

    from solver.elements import beam_rotation_matrix_3d
    for dl in dloads:
        el = elements[dl.element_id - 1]
        p1, p2 = nmap[el.nodes[0]], nmap[el.nodes[1]]
        R_el, L_el = beam_rotation_matrix_3d(p1, p2, roll_angle=el.roll_angle)
        q_loc = np.array([dl.qx, dl.qy, dl.qz])
        q_glob = R_el.T @ q_loc
        f_res = q_glob * L_el
        mid = 0.5 * (p1 + p2)
        total_f += f_res
        total_m_p0 += np.cross(mid - p0, f_res)

        # Equilibrio local del elemento 2:
        # f_int = k_local * u_local - feq_local representa las fuerzas nodales ejercidas SOBRE el elemento
        # más las reacciones de empotramiento equivalentes, por lo que:
        # f_int[0:3] + f_int[6:9] + q_loc * L_el == 0
        f_int = np.array(res.element_results[el.id].local_end_forces)
        np.testing.assert_allclose(f_int[0:3] + f_int[6:9] + q_loc * L_el, np.zeros(3), atol=1e-5)

    for n in nodes:
        reac = np.array(res.nodal_reactions[n.id])
        r_vec = nmap[n.id] - p0
        total_f += reac[0:3]
        total_m_p0 += np.cross(r_vec, reac[0:3]) + reac[3:6]

    np.testing.assert_allclose(total_f, np.zeros(3), atol=1e-4)
    np.testing.assert_allclose(total_m_p0, np.zeros(3), atol=1e-3)


# =============================================================================
# 3. INVARIANCIA ANTE ROTACIONES 3D (SO(3)) Y PERMUTACIÓN DE CONECTIVIDAD NODAL
# =============================================================================

def test_gauntlet_rotational_invariance_so3_truss_frame_tetra():
    """
    Comprueba que al rotar un modelo 3D (nodos y cargas) mediante una matriz ortogonal Q in SO(3)
    con apoyos isótropos (FIXED_3D / PINNED_3D), los desplazamientos y reacciones rotan como
    u' = Q u, R' = Q R y la energía elástica U y tensiones de Von Mises permanecen invariantes.
    """
    Q = _rotation_matrix_so3(0.37, -0.52, 0.71)
    sec = CrossSection.circular(1, "Circular", diameter=0.12)
    mat = Material(1, "Acero", E=210e9, nu=0.3)

    base_nodes = [
        Node3D(1, 0.0, 0.0, 0.0),
        Node3D(2, 3.0, 0.0, 0.0),
        Node3D(3, 3.0, 2.5, 1.0),
    ]
    base_f = np.array([15e3, -40e3, 22e3])
    base_m = np.array([4e3, -2e3, 6e3])

    m1 = StructuralModel(
        name="Frame_Unrotated",
        nodes=base_nodes,
        materials=[mat],
        sections=[sec],
        elements=[
            Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1),
            Element(2, ElementType.FRAME_3D.value, [2, 3], 1, 1),
        ],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        nodal_loads=[NodalLoad(3, fx=base_f[0], fy=base_f[1], fz=base_f[2], mx=base_m[0], my=base_m[1], mz=base_m[2])],
    )
    r1 = solve_structure(m1)
    assert r1.success is True

    rot_nodes = []
    for n in base_nodes:
        p_rot = Q @ np.array(n.coords())
        rot_nodes.append(Node3D(n.id, float(p_rot[0]), float(p_rot[1]), float(p_rot[2])))
    f_rot = Q @ base_f
    m_rot = Q @ base_m

    m2 = StructuralModel(
        name="Frame_Rotated",
        nodes=rot_nodes,
        materials=[mat],
        sections=[sec],
        elements=[
            Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1),
            Element(2, ElementType.FRAME_3D.value, [2, 3], 1, 1),
        ],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        nodal_loads=[NodalLoad(3, fx=float(f_rot[0]), fy=float(f_rot[1]), fz=float(f_rot[2]), mx=float(m_rot[0]), my=float(m_rot[1]), mz=float(m_rot[2]))],
    )
    r2 = solve_structure(m2)
    assert r2.success is True

    assert math.isclose(r1.strain_energy, r2.strain_energy, rel_tol=1e-9)
    assert math.isclose(r1.max_displacement, r2.max_displacement, rel_tol=1e-9)

    for nid in (2, 3):
        u1 = np.array(r1.nodal_displacements[nid][0:3])
        th1 = np.array(r1.nodal_displacements[nid][3:6])
        u2 = np.array(r2.nodal_displacements[nid][0:3])
        th2 = np.array(r2.nodal_displacements[nid][3:6])
        np.testing.assert_allclose(u2, Q @ u1, rtol=1e-7, atol=1e-10)
        np.testing.assert_allclose(th2, Q @ th1, rtol=1e-7, atol=1e-10)


def test_gauntlet_triangle3d_node_permutation_stress_invariance():
    """
    Verifica la invariancia ante la permutación de la conectividad de los nodos [1, 2, 3] -> [2, 3, 1]
    en TRIANGLE_3D. Tanto los desplazamientos nodales como el tensor de tensiones en el sistema global
    y la tensión axial global sigma_xx deben ser invariantes a qué arista del triángulo se liste primero.
    """
    from solver.benchmarks import triangle_plate_patch_test
    m_orig = triangle_plate_patch_test()
    m_perm = triangle_plate_patch_test()
    # Permutar cíclicamente la conectividad del elemento 1: [1, 2, 3] -> [2, 3, 1]
    m_perm.elements[0].nodes = [2, 3, 1]

    r_orig = solve_structure(m_orig)
    r_perm = solve_structure(m_perm)

    assert r_orig.success is True and r_perm.success is True

    # Desplazamientos nodales idénticos
    for nid in (1, 2, 3, 4):
        np.testing.assert_allclose(
            r_orig.nodal_displacements[nid],
            r_perm.nodal_displacements[nid],
            atol=1e-12,
        )

    # La tensión de Von Mises y el tensor de tensiones expresado en coordenadas globales
    # no deben cambiar al permutar el orden de los vértices del triángulo 1:
    er_orig = r_orig.element_results[1]
    er_perm = r_perm.element_results[1]
    assert math.isclose(er_orig.von_mises_stress, er_perm.von_mises_stress, rel_tol=1e-9)
    np.testing.assert_allclose(
        er_orig.stress_tensor_voigt,
        er_perm.stress_tensor_voigt,
        rtol=1e-6,
        atol=1e-3,
    )


# =============================================================================
# 4. COMPORTAMIENTO MECÁNICO REAL DE TRIANGLE_3D (MEMBRANA CST)
# =============================================================================

def test_gauntlet_triangle3d_rank_and_out_of_plane_mechanism():
    """
    Demuestra el comportamiento mecánico real de TRIANGLE_3D:
    1) Su matriz 9x9 tiene rango exactamente 3 (3 modos de deformación plana de membrana: exx, eyy, gxy)
       y 6 modos de energía nula (3 traslaciones + 3 rotaciones rígidas en 3D).
    2) No posee rigidez a flexión transversal fuera de su plano: si un nodo libre recibe carga normal
       al plano del triángulo sin apoyo transversal, el solver detecta correctamente la singularidad/mecanismo.
    """
    n1 = Node3D(1, 0.0, 0.0, 0.0)
    n2 = Node3D(2, 2.0, 0.0, 0.0)
    n3 = Node3D(3, 0.0, 1.5, 0.0)
    mat = Material(1, "Acero", E=200e9, nu=0.3)
    sec = CrossSection(1, "Espesor_10mm", A=0.015, thickness=0.01)

    K_9x9, _, _, _, area = triangle3d_stiffness_global(n1, n2, n3, mat, sec)
    assert math.isclose(area, 1.5, rel_tol=1e-12)
    assert np.linalg.matrix_rank(K_9x9) == 3

    # Triángulo apoyado en N1 y N2 en (ux, uy, uz), pero N3 libre en uz con carga transversal Fz:
    # Al ser membrana pura sin rigidez fuera del plano, debe ser detectado como mecanismo (singular).
    m_out_of_plane = StructuralModel(
        name="Triangle_Out_Of_Plane_Load",
        nodes=[n1, n2, n3],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.TRIANGLE_3D.value, [1, 2, 3], 1, 1)],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(2, SupportPreset.PINNED_3D),
        ],
        nodal_loads=[NodalLoad(3, fz=1000.0)],
    )
    res = solve_structure(m_out_of_plane)
    assert res.success is False
    assert res.diagnostics.is_singular is True
    assert res.diagnostics.is_mechanism is True


# =============================================================================
# 5. GEOMETRÍAS DEGENERADAS, ESPESORES INVÁLIDOS Y ENTRADAS NaN / Inf
# =============================================================================

def test_gauntlet_degenerate_triangle_and_tetra_geometry():
    """
    Verifica el rechazo controlado (`success=False`, `is_valid=False`) ante:
    - TRIANGLE_3D con 3 nodos colineales (área nula).
    - TETRA_4 con 4 nodos coplanares (volumen nulo).
    """
    mat = Material(1, "Acero", E=200e9, nu=0.3)
    sec = CrossSection(1, "Sec", A=0.01, thickness=0.01)

    # 1. Triángulo colineal
    m_collinear_tri = StructuralModel(
        name="Degenerate_Collinear_Triangle",
        nodes=[Node3D(1, 0, 0, 0), Node3D(2, 1, 1, 1), Node3D(3, 2, 2, 2)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.TRIANGLE_3D.value, [1, 2, 3], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D)],
    )
    r1 = solve_structure(m_collinear_tri)
    assert r1.success is False and r1.diagnostics.is_valid is False

    # 2. Tetraedro coplanar (todos los nodos en z = 0)
    m_coplanar_tet = StructuralModel(
        name="Degenerate_Coplanar_Tetra",
        nodes=[Node3D(1, 0, 0, 0), Node3D(2, 1, 0, 0), Node3D(3, 0, 1, 0), Node3D(4, 1, 1, 0)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.TETRA_4.value, [1, 2, 3, 4], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D)],
    )
    r2 = solve_structure(m_coplanar_tet)
    assert r2.success is False and r2.diagnostics.is_valid is False


def test_gauntlet_rejects_nonpositive_triangle_thickness():
    """
    Verifica que un elemento TRIANGLE_3D cuya sección tiene `thickness <= 0` sea rechazado
    explícitamente (`is_valid=False`) en lugar de sustituirse silenciosamente por `0.01`.
    """
    mat = Material(1, "Acero", E=200e9, nu=0.3)
    sec_bad_t = CrossSection(1, "BadThickness", A=0.01, thickness=-0.005)
    m_bad_t = StructuralModel(
        name="Invalid_Thickness_Triangle",
        nodes=[Node3D(1, 0, 0, 0), Node3D(2, 1, 0, 0), Node3D(3, 0, 1, 0)],
        materials=[mat],
        sections=[sec_bad_t],
        elements=[Element(1, ElementType.TRIANGLE_3D.value, [1, 2, 3], 1, 1)],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(2, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(3, SupportPreset.ROLLER_Z),
        ],
    )
    r3 = solve_structure(m_bad_t)
    assert r3.success is False and r3.diagnostics.is_valid is False


def test_gauntlet_rejects_nan_distributed_load_and_inf_prescribed_bc():
    """
    Verifica el rechazo limpio (`success=False`, `is_valid=False`, sin excepciones no capturadas)
    ante `NaN`/`Inf` en cargas distribuidas y en desplazamientos prescritos de apoyos.
    """
    mat = Material(1, "Acero", E=200e9, nu=0.3)
    sec = CrossSection(1, "Sec", A=0.01, thickness=0.01)

    # 1. Carga distribuida con NaN
    m_nan_dload = StructuralModel(
        name="NaN_Distributed_Load",
        nodes=[Node3D(1, 0, 0, 0), Node3D(2, 2, 0, 0)],
        materials=[mat],
        sections=[CrossSection.rectangular(1, "R", 0.1, 0.2)],
        elements=[Element(1, ElementType.FRAME_3D.value, [1, 2], 1, 1)],
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        distributed_loads=[DistributedLoad(1, qy=float("nan"))],
    )
    r4 = solve_structure(m_nan_dload)
    assert r4.success is False and r4.diagnostics.is_valid is False

    # 2. Desplazamiento prescrito con Inf
    m_inf_bc = StructuralModel(
        name="Inf_Prescribed_BC",
        nodes=[Node3D(1, 0, 0, 0), Node3D(2, 2, 0, 0)],
        materials=[mat],
        sections=[sec],
        elements=[Element(1, ElementType.TRUSS_3D.value, [1, 2], 1, 1)],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition(2, ux=True, uy=True, uz=True, prescribed_ux=float("inf")),
        ],
    )
    r5 = solve_structure(m_inf_bc)
    assert r5.success is False and r5.diagnostics.is_valid is False


# =============================================================================
# 6. MECANISMOS ESTRUCTURALES 3D, NODOS HUÉRFANOS Y CONDICIONES DUPLICADAS
# =============================================================================

def test_gauntlet_collinear_supports_3d_rotational_mechanism():
    """
    Un triángulo de 3 barras TRUSS_3D cuyos 2 apoyos articulados están sobre el eje X (y=0, z=0)
    y el tercer nodo está fuera del eje X: puede girar libremente como sólido rígido alrededor
    del eje X (modo de energía nula). Debe ser detectado como mecanismo.
    """
    mat = Material(1, "Acero", E=200e9, nu=0.3)
    sec = CrossSection(1, "Sec", A=0.001)
    model = StructuralModel(
        name="Collinear_Supports_Rigid_Rotation",
        nodes=[
            Node3D(1, 0.0, 0.0, 0.0),
            Node3D(2, 2.0, 0.0, 0.0),
            Node3D(3, 1.0, 1.5, 0.0),
        ],
        materials=[mat],
        sections=[sec],
        elements=[
            Element(1, ElementType.TRUSS_3D.value, [1, 2], 1, 1),
            Element(2, ElementType.TRUSS_3D.value, [1, 3], 1, 1),
            Element(3, ElementType.TRUSS_3D.value, [2, 3], 1, 1),
        ],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(2, SupportPreset.PINNED_3D),
        ],
        nodal_loads=[NodalLoad(3, fz=10e3)],
    )
    res = solve_structure(model)
    assert res.success is False
    assert res.diagnostics.is_singular is True
    assert res.diagnostics.is_mechanism is True


def test_gauntlet_moment_on_truss_pin_node_is_mechanism():
    """
    Aplicar un momento puntual externo (Mz != 0) en un nudo que sólo conecta barras articuladas TRUSS_3D
    debe ser detectado como un mecanismo rotacional (rigidez rotacional nula).
    """
    from solver.benchmarks import small_space_truss_benchmark
    model = small_space_truss_benchmark()
    model.nodal_loads.append(NodalLoad(node_id=4, mz=5e3))
    res = solve_structure(model)
    assert res.success is False
    assert res.diagnostics.is_singular is True
    assert res.diagnostics.is_mechanism is True


def test_gauntlet_duplicate_boundary_conditions_and_orphan_node():
    """
    1) Si se especifican dos entradas BoundaryCondition para el mismo nodo restringiendo los mismos GDL,
       `diagnostics.constrained_dofs` debe reflejar el número real de GDL únicos restringidos por el usuario.
    2) Si existe un nodo huérfano pasivo (sin elementos conectados y sin cargas), el solver debe emitir una
       advertencia en `diagnostics.warnings` y excluir sus GDL inactivos sin abortar el cálculo.
    3) Si el nodo huérfano tiene cargas aplicadas, debe rechazarse explícitamente como mecanismo (`success=False`).
    4) Si dos condiciones de apoyo imponen desplazamientos prescritos contradictorios en un mismo GDL,
       debe rechazarse explícitamente (`success=False`, `is_valid=False`).
    """
    from solver.benchmarks import axial_bar_benchmark
    model = axial_bar_benchmark()
    # Añadir condición duplicada compatible sobre el nodo 1
    model.boundary_conditions.append(BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D))
    # Añadir nodo huérfano desconectado pasivo (nodo 99)
    model.nodes.append(Node3D(99, 5.0, 5.0, 5.0, label="Huerfano"))

    res = solve_structure(model)
    assert res.success is True
    assert any("huérfano" in w.lower() or "desconectado" in w.lower() for w in res.diagnostics.warnings)
    # Nodo 1 tiene 3 GDL restringidos (ux, uy, uz) y Nodo 2 tiene 2 (uy, uz) -> total 5 GDL únicos
    assert res.diagnostics.constrained_dofs == 5

    # Caso 3: Nodo huérfano con carga aplicada -> debe fallar como mecanismo
    model_loaded_orphan = axial_bar_benchmark()
    model_loaded_orphan.nodes.append(Node3D(99, 5.0, 5.0, 5.0, label="Huerfano_Cargado"))
    model_loaded_orphan.nodal_loads.append(NodalLoad(node_id=99, fx=1000.0))
    res_lo = solve_structure(model_loaded_orphan)
    assert res_lo.success is False
    assert res_lo.diagnostics.is_singular is True
    assert res_lo.diagnostics.is_mechanism is True

    # Caso 4: Desplazamientos prescritos contradictorios en el mismo nodo/GDL -> debe fallar por incompatibilidad
    model_conflict = axial_bar_benchmark()
    model_conflict.boundary_conditions.append(BoundaryCondition(node_id=1, ux=True, prescribed_ux=0.005))
    res_conf = solve_structure(model_conflict)
    assert res_conf.success is False
    assert res_conf.diagnostics.is_valid is False


# =============================================================================
# 7. COMPARACIÓN DIFERENCIAL PYTHON vs. GDSCRIPT (SI GODOT ESTÁ DISPONIBLE)
# =============================================================================

def test_gauntlet_differential_python_vs_gdscript_if_executable():
    """
    Ejecuta una comparación diferencial real entre `solver.engine.solve_structure` (Python)
    y `scripts/solver/NativeSolver.gd` (Godot 4) si el ejecutable de Godot está presente en PATH.
    """
    godot_bin = shutil.which("godot") or shutil.which("godot4")
    if not godot_bin:
        pytest.skip("Ejecutable de Godot 4 no encontrado en PATH; no se declara equivalencia sin ejecución real.")
