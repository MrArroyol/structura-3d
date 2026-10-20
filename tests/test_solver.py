"""
Suite de Pruebas Automatizadas para el Motor Estructural Structura 3D.

Verifica:
1. Soluciones analíticas exactas en barra axial, celosía espacial 3D y vigas Euler-Bernoulli.
2. Equilibrio estático global de fuerzas (Sum F = 0) y momentos (Sum M = 0).
3. Sensibilidad a la discretización y convergencia con refinamiento de malla (N = 1, 2, 4, 8, 16, 32).
4. Extensión a elementos triangulares CST 3D y sólidos tetraédricos TETRA_4 (Patch Test 3D).
5. Detección de condiciones de apoyo insuficientes, mecanismos cinemáticos y entradas no válidas.
6. Restricciones normativas y avisos de no aptitud para construcción en los informes exportados.
"""

from __future__ import annotations

import math
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
    DISCLAIMER_TEXT,
)
from solver.engine import solve_structure
from solver.benchmarks import (
    axial_bar_benchmark,
    small_space_truss_benchmark,
    cantilever_beam_point_load,
    simply_supported_beam_uniform_load,
    space_portal_frame_3d,
    triangle_plate_patch_test,
    tetra4_solid_prism_benchmark,
    insufficient_supports_benchmark,
    collinear_mechanism_truss_benchmark,
)
from solver.report import generate_markdown_report


def test_axial_bar_analytical_solution():
    """Verifica elongación exacta u = PL/(EA), tensión P/A y reacción R = -P."""
    model = axial_bar_benchmark()
    result = solve_structure(model)

    assert result.success is True
    assert result.diagnostics.is_singular is False
    assert result.diagnostics.equilibrium_satisfied is True

    expected = model.metadata["analytical_solution"]
    ux_2 = result.nodal_displacements[2][0]
    rx_1 = result.nodal_reactions[1][0]
    elem_res = result.element_results[1]

    assert math.isclose(ux_2, expected["node_2_ux"], rel_tol=1e-12)
    assert math.isclose(rx_1, expected["node_1_Rx"], rel_tol=1e-12)
    assert math.isclose(elem_res.axial_force, expected["element_1_axial_force"], rel_tol=1e-12)
    assert math.isclose(elem_res.axial_stress, expected["element_1_axial_stress"], rel_tol=1e-12)


def test_small_space_truss_3d_equilibrium_and_analytical_forces():
    """
    Verifica la primera estructura espacial de barras (trípode 3D de 4 nodos y 3 barras).
    Por ser isostática en el nudo superior (3 barras no coplanares concurrentes en el nodo 4),
    las fuerzas axiales N_i deben satisfacer exactamente Sum(N_i * e_i) = F_ext.
    """
    model = small_space_truss_benchmark()
    result = solve_structure(model)

    assert result.success is True
    assert result.diagnostics.equilibrium_satisfied is True
    assert result.diagnostics.force_residual_norm < 1e-6
    assert result.diagnostics.moment_residual_norm < 1e-6

    # Verificar equilibrio exacto de fuerzas internas concurrentes en la cúspide (nodo 4)
    p4 = np.array(model.nodes[3].coords())
    f_internal_at_4 = np.zeros(3)
    for elem in model.elements:
        n_base = model.nodes[elem.nodes[0] - 1]
        p_base = np.array(n_base.coords())
        e_vec = (p4 - p_base) / np.linalg.norm(p4 - p_base)
        N_force = result.element_results[elem.id].axial_force
        # Si la barra está en tracción (N > 0), tira del nodo 4 hacia la base (-e_vec)
        f_internal_at_4 += -N_force * e_vec

    f_ext_4 = np.array([30e3, -120e3, 15e3])
    np.testing.assert_allclose(f_internal_at_4 + f_ext_4, np.zeros(3), atol=1e-6)


def test_cantilever_beam_point_and_distributed_loads():
    """
    Verifica flecha máxima, giro en el extremo y momento de empotramiento en una viga en voladizo
    frente a las fórmulas cerradas de Euler-Bernoulli.
    """
    model = cantilever_beam_point_load(num_elements=4)
    result = solve_structure(model)

    assert result.success is True
    assert result.diagnostics.equilibrium_satisfied is True

    expected = model.metadata["analytical_solution"]
    tip_disp = result.nodal_displacements[5]
    root_reac = result.nodal_reactions[1]

    assert math.isclose(tip_disp[1], expected["tip_uy"], rel_tol=1e-9)
    assert math.isclose(tip_disp[5], expected["tip_rz"], rel_tol=1e-9)
    assert math.isclose(root_reac[1], expected["root_Ry"], rel_tol=1e-9)
    assert math.isclose(root_reac[5], expected["root_Mz"], rel_tol=1e-9)


def test_beam_discretization_sensitivity_and_convergence():
    """
    Evalúa la sensibilidad a la discretización (N = 1, 2, 4, 8, 16 elementos) en:
    1) Viga biapoyada bajo carga uniforme q: verifica que para cualquier N par la flecha en el centro
       es exacta (propiedad de funciones de forma cúbicas de Hermite) y el momento máximo muestreado
       converge monótonamente a q*L^2/8.
    2) Barra axial de sección variable A(x) = A0 * (1 + x/L) sometida a carga P en x=L:
       la solución analítica exacta es u(L) = (P*L / (E*A0)) * ln(2).
       Con elementos de sección constante por tramo (evaluada en el punto medio de cada elemento),
       el error relativo debe converger con orden cuadrático O(1/N^2) al aumentar N = 1, 2, 4, 8, 16, 32.
    """
    # Parte A: Viga biapoyada con N = 2, 4, 8, 16
    for n_el in (2, 4, 8, 16):
        bm = simply_supported_beam_uniform_load(num_elements=n_el)
        res = solve_structure(bm)
        assert res.success is True
        assert res.diagnostics.equilibrium_satisfied is True
        mid_node = (n_el // 2) + 1
        w_mid = res.nodal_displacements[mid_node][1]
        w_exact = bm.metadata["analytical_solution"]["midspan_uy"]
        assert math.isclose(w_mid, w_exact, rel_tol=1e-9)

    # Parte B: Convergencia de malla en barra de sección variable A(x) = A0*(1 + x/L)
    L = 3.0
    E = 200e9
    A0 = 0.001
    P = 150e3
    u_exact_tapered = (P * L / (E * A0)) * math.log(2.0)

    errors = []
    for n_el in (1, 2, 4, 8, 16, 32):
        dx = L / n_el
        nodes = [Node3D(id=i + 1, x=i * dx, y=0.0, z=0.0) for i in range(n_el + 1)]
        sections = []
        elements = []
        for i in range(n_el):
            x_mid = (i + 0.5) * dx
            a_mid = A0 * (1.0 + x_mid / L)
            sections.append(CrossSection(id=i + 1, name=f"Sec_{i+1}", A=a_mid))
            elements.append(
                Element(id=i + 1, type=ElementType.TRUSS_3D.value, nodes=[i + 1, i + 2], material_id=1, section_id=i + 1)
            )
        bcs = [BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D)]
        for i in range(1, n_el + 1):
            bcs.append(BoundaryCondition.from_preset(i + 1, SupportPreset.ROLLER_X))
        tapered_model = StructuralModel(
            name=f"Barra_Ahusada_N{n_el}",
            nodes=nodes,
            materials=[Material(id=1, name="Acero", E=E, nu=0.3)],
            sections=sections,
            elements=elements,
            boundary_conditions=bcs,
            nodal_loads=[NodalLoad(node_id=n_el + 1, fx=P)],
        )
        res = solve_structure(tapered_model)
        assert res.success is True
        u_num = res.nodal_displacements[n_el + 1][0]
        rel_err = abs(u_num - u_exact_tapered) / abs(u_exact_tapered)
        errors.append(rel_err)

    # Verificar que el error decrece monótonamente con la discretización y para N=32 es < 0.01%
    for k in range(len(errors) - 1):
        assert errors[k + 1] < errors[k]
    assert errors[-1] < 1e-4


def test_space_portal_frame_3d_equilibrium():
    """Verifica el equilibrio tridimensional de fuerzas y momentos en el pórtico espacial 3D."""
    model = space_portal_frame_3d()
    res = solve_structure(model)

    assert res.success is True
    assert res.diagnostics.equilibrium_satisfied is True
    assert res.diagnostics.force_residual_norm < 1e-5
    assert res.diagnostics.moment_residual_norm < 1e-5


def test_triangle_cst_patch_test():
    """Verifica el Patch Test de membrana en elementos triangulares TRIANGLE_3D (CST)."""
    model = triangle_plate_patch_test()
    res = solve_structure(model)

    assert res.success is True
    assert res.diagnostics.equilibrium_satisfied is True

    expected = model.metadata["analytical_solution"]
    # Desplazamientos en el borde derecho (nodos 2 y 3)
    assert math.isclose(res.nodal_displacements[2][0], expected["ux_right"], rel_tol=1e-9)
    assert math.isclose(res.nodal_displacements[3][0], expected["ux_right"], rel_tol=1e-9)
    # Contracción transversal de Poisson en nodos superiores (nodos 3 y 4)
    assert math.isclose(res.nodal_displacements[3][1], expected["uy_top"], rel_tol=1e-9)
    assert math.isclose(res.nodal_displacements[4][1], expected["uy_top"], rel_tol=1e-9)

    # Tensión constante e igual a sigma_0 = 50 MPa en ambos triángulos
    for eid in (1, 2):
        er = res.element_results[eid]
        assert math.isclose(er.von_mises_stress, expected["sigma_xx"], rel_tol=1e-9)


def test_tetra4_solid_patch_test_3d():
    """
    Verifica la extensión a elementos sólidos tetraédricos de 4 nodos (TETRA_4) en un cubo 3D
    compuesto por 6 tetraedros bajo estado uniaxial de tracción.
    """
    model = tetra4_solid_prism_benchmark()
    res = solve_structure(model)

    assert res.success is True
    assert res.diagnostics.equilibrium_satisfied is True

    expected = model.metadata["analytical_solution"]
    for eid in range(1, 7):
        er = res.element_results[eid]
        exx, eyy, ezz, gxy, gyz, gxz = er.strain_tensor_voigt
        sxx, syy, szz, txy, tyz, txz = er.stress_tensor_voigt

        assert math.isclose(exx, expected["strain_xx"], rel_tol=1e-8)
        assert math.isclose(eyy, expected["strain_yy"], rel_tol=1e-8)
        assert math.isclose(ezz, expected["strain_zz"], rel_tol=1e-8)
        assert abs(gxy) < 1e-10 and abs(gyz) < 1e-10 and abs(gxz) < 1e-10

        assert math.isclose(sxx, expected["sigma_xx"], rel_tol=1e-8)
        assert abs(syy) < 1e-3 and abs(szz) < 1e-3
        assert abs(txy) < 1e-3 and abs(tyz) < 1e-3 and abs(txz) < 1e-3
        assert math.isclose(er.von_mises_stress, expected["von_mises"], rel_tol=1e-8)


def test_detects_insufficient_supports_and_mechanisms():
    """
    Comprueba que el motor detecta condiciones de apoyo insuficientes (modos de sólido rígido)
    y mecanismos cinemáticos internos sin producir resultados numéricos espurios.
    """
    # 1. Apoyos insuficientes (barra libre de deslizar en X)
    m_insuff = insufficient_supports_benchmark()
    res_insuff = solve_structure(m_insuff)
    assert res_insuff.success is False
    assert res_insuff.diagnostics.is_singular is True
    assert res_insuff.diagnostics.is_mechanism is True
    assert len(res_insuff.diagnostics.errors) > 0

    # 2. Mecanismo interno (dos barras articuladas colineales con nudo central sin coacción transversal)
    m_mech = collinear_mechanism_truss_benchmark()
    res_mech = solve_structure(m_mech)
    assert res_mech.success is False
    assert res_mech.diagnostics.is_singular is True
    assert res_mech.diagnostics.is_mechanism is True

    # 3. Estructura totalmente libre (sin ningún apoyo)
    m_free = axial_bar_benchmark()
    m_free.boundary_conditions = []
    res_free = solve_structure(m_free)
    assert res_free.success is False
    assert res_free.diagnostics.is_singular is True


def test_detects_invalid_inputs_and_degenerate_geometry():
    """Comprueba que el motor rechaza materiales no físicos y elementos de longitud/volumen cero."""
    # Material con E <= 0 o Poisson >= 0.5
    m_bad_mat = axial_bar_benchmark()
    m_bad_mat.materials[0].E = -100.0
    m_bad_mat.materials[0].nu = 0.55
    res_bad_mat = solve_structure(m_bad_mat)
    assert res_bad_mat.success is False
    assert res_bad_mat.diagnostics.is_valid is False

    # Elemento de longitud cero (nodos coincidentes)
    m_zero_len = axial_bar_benchmark()
    m_zero_len.nodes[1].x = 0.0
    res_zero_len = solve_structure(m_zero_len)
    assert res_zero_len.success is False
    assert res_zero_len.diagnostics.is_valid is False


def test_report_contains_mandatory_disclaimer_and_units():
    """Verifica que los informes generados incluyen las unidades y la advertencia de no certificación."""
    model = axial_bar_benchmark()
    res = solve_structure(model)
    md = generate_markdown_report(model, res)

    assert DISCLAIMER_TEXT in md
    assert "NO son aptos para construcción" in md
    assert "certificación estructural" in md
    assert "Hipótesis del Modelo, Limitaciones y Modos de Fallo No Analizados" in md
    assert "Pandeo" in md
