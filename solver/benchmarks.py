"""
Casos de Referencia (Benchmarks) con Solución Analítica Conocida — Structura 3D.

Incluye:
1. Barra axial empotrada (TRUSS_3D) con solución exacta u = PL/(EA).
2. Celosía espacial pequeña de 3 barras / trípode 3D (TRUSS_3D) — Primera estructura de barras.
3. Viga en voladizo con carga puntual y carga distribuida (FRAME_3D) parametrizada en N elementos.
4. Viga biapoyada bajo carga distribuida uniforme (FRAME_3D) parametrizada en N elementos.
5. Pórtico espacial 3D de 1 nivel (FRAME_3D) bajo carga gravitatoria y sismo/viento lateral.
6. Placa en tensión plana con elementos triangulares CST (TRIANGLE_3D).
7. Prisma sólido 3D discretizado en tetraedros (TETRA_4) — Patch Test 3D exacto.
8. Casos patológicos de prueba: apoyos insuficientes y mecanismo interno colineal.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Any, List

from .model import (
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


def axial_bar_benchmark() -> StructuralModel:
    """
    Caso 1: Barra prismática sometida a carga axial P = 100 kN.
    L = 2.0 m, E = 200 GPa, A = 0.001 m² (10 cm²).
    Solución analítica:
      u_x(L) = P*L / (E*A) = 100e3 * 2.0 / (200e9 * 1e-3) = 0.001 m (1.0 mm)
      sigma_x = P / A = 100 MPa
      R_x(nodo 1) = -100 kN
    """
    L = 2.0
    E = 200e9
    A = 1e-3
    P = 100e3

    u_exact = (P * L) / (E * A)
    sigma_exact = P / A

    return StructuralModel(
        name="01_Barra_Axial_Referencia",
        description="Barra sometida a tracción uniaxial pura (solución analítica exacta u = PL/EA).",
        nodes=[
            Node3D(id=1, x=0.0, y=0.0, z=0.0, label="Apoyo"),
            Node3D(id=2, x=L, y=0.0, z=0.0, label="Extremo Cargado"),
        ],
        materials=[Material(id=1, name="Acero Estructural S275", E=E, nu=0.3, rho=7850.0, yield_strength=275e6)],
        sections=[CrossSection(id=1, name="Barra Maciza 10 cm²", shape="CIRCULAR", A=A, Iy=1e-6, Iz=1e-6, J=2e-6)],
        elements=[
            Element(id=1, type=ElementType.TRUSS_3D.value, nodes=[1, 2], material_id=1, section_id=1, label="Barra 1")
        ],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(2, SupportPreset.ROLLER_X),
        ],
        nodal_loads=[NodalLoad(node_id=2, fx=P, fy=0.0, fz=0.0, label="P = 100 kN")],
        metadata={
            "analytical_solution": {
                "node_2_ux": u_exact,
                "element_1_axial_force": P,
                "element_1_axial_stress": sigma_exact,
                "node_1_Rx": -P,
            }
        },
    )


def small_space_truss_benchmark() -> StructuralModel:
    """
    Caso 2 (Primera tarea): Pequeña estructura de barras 3D (Trípode espacial simétrico de 3 barras).
    Nodos base articulados en:
      N1 = (-1.5, 0, -1.0)
      N2 = ( 1.5, 0, -1.0)
      N3 = ( 0.0, 0,  1.5)
    Vértice superior:
      N4 = ( 0.0, 2.0, 0.0)
    Sometido a carga vertical Fy = -120 kN y fuerza horizontal Fx = 30 kN en el nodo 4.
    Por equilibrio estático de nudo en N4 (sistema 3x3 determinado en 3D):
      Sum(F_i * e_i) + F_ext = 0
    """
    E = 210e9
    sec = CrossSection.circular(1, "Tubo Circular d=50mm", diameter=0.05)

    return StructuralModel(
        name="02_Celosia_Espacial_Tripode_3D",
        description="Estructura espacial de 3 barras articuladas (trípode 3D) sometida a carga combinada en su cúspide.",
        nodes=[
            Node3D(id=1, x=-1.5, y=0.0, z=-1.0, label="Base Izq"),
            Node3D(id=2, x=1.5, y=0.0, z=-1.0, label="Base Der"),
            Node3D(id=3, x=0.0, y=0.0, z=1.5, label="Base Frontal"),
            Node3D(id=4, x=0.0, y=2.0, z=0.0, label="Cúspide"),
        ],
        materials=[Material(id=1, name="Acero S355", E=E, nu=0.3, rho=7850.0, yield_strength=355e6)],
        sections=[sec],
        elements=[
            Element(id=1, type=ElementType.TRUSS_3D.value, nodes=[1, 4], material_id=1, section_id=1, label="Diagonal 1"),
            Element(id=2, type=ElementType.TRUSS_3D.value, nodes=[2, 4], material_id=1, section_id=1, label="Diagonal 2"),
            Element(id=3, type=ElementType.TRUSS_3D.value, nodes=[3, 4], material_id=1, section_id=1, label="Diagonal 3"),
        ],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(2, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(3, SupportPreset.PINNED_3D),
        ],
        nodal_loads=[
            NodalLoad(node_id=4, fx=30e3, fy=-120e3, fz=15e3, label="Carga Cúspide"),
        ],
        metadata={
            "analytical_equilibrium": {
                "sum_Rx": -30e3,
                "sum_Ry": 120e3,
                "sum_Rz": -15e3,
            }
        },
    )


def cantilever_beam_point_load(num_elements: int = 4) -> StructuralModel:
    """
    Caso 3: Viga en voladizo de longitud L = 4.0 m sometida a carga puntual P = -20 kN en el extremo libre.
    Sección rectangular b = 0.15 m, h = 0.30 m, E = 200 GPa.
    Solución analítica Euler-Bernoulli:
      uy(L) = P * L^3 / (3 * E * Iz)
      rz(L) = P * L^2 / (2 * E * Iz)
      Ry(0) = -P = +20 kN
      Mz(0) = -P * L = +80 kN·m
    """
    L = 4.0
    E = 200e9
    P = -20e3
    sec = CrossSection.rectangular(1, "Rect 150x300 mm", b=0.15, h=0.30)

    nodes: List[Node3D] = []
    elements: List[Element] = []
    dx = L / num_elements
    for i in range(num_elements + 1):
        nodes.append(Node3D(id=i + 1, x=i * dx, y=0.0, z=0.0, label=f"N{i+1}"))
    for i in range(num_elements):
        elements.append(
            Element(
                id=i + 1,
                type=ElementType.FRAME_3D.value,
                nodes=[i + 1, i + 2],
                material_id=1,
                section_id=1,
                label=f"Tramo {i+1}",
            )
        )

    uy_exact = (P * (L ** 3)) / (3.0 * E * sec.Iz)
    rz_exact = (P * (L ** 2)) / (2.0 * E * sec.Iz)

    return StructuralModel(
        name=f"03_Viga_Voladizo_N{num_elements}",
        description=f"Viga en voladizo Euler-Bernoulli (L=4m, P=-20kN) discretizada en {num_elements} elementos.",
        nodes=nodes,
        materials=[Material(id=1, name="Acero Estructural", E=E, nu=0.3)],
        sections=[sec],
        elements=elements,
        boundary_conditions=[BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D)],
        nodal_loads=[NodalLoad(node_id=num_elements + 1, fy=P, label="P = -20 kN")],
        metadata={
            "analytical_solution": {
                "tip_uy": uy_exact,
                "tip_rz": rz_exact,
                "root_Ry": -P,
                "root_Mz": -P * L,
                "max_moment_z": abs(P * L),
            }
        },
    )


def simply_supported_beam_uniform_load(num_elements: int = 4) -> StructuralModel:
    """
    Caso 4: Viga biapoyada de longitud L = 6.0 m sometida a carga distribuida uniforme qy = -15 kN/m.
    Solución analítica:
      w_mid = 5 * qy * L^4 / (384 * E * Iz)
      Ry(0) = Ry(L) = -qy * L / 2 = +45 kN
      Mz_max(L/2) = |qy| * L^2 / 8 = 67.5 kN·m
    """
    L = 6.0
    E = 210e9
    qy = -15e3
    sec = CrossSection.rectangular(1, "Viga 200x400 mm", b=0.20, h=0.40)

    nodes: List[Node3D] = []
    elements: List[Element] = []
    dloads: List[DistributedLoad] = []
    dx = L / num_elements

    for i in range(num_elements + 1):
        nodes.append(Node3D(id=i + 1, x=i * dx, y=0.0, z=0.0, label=f"N{i+1}"))
    for i in range(num_elements):
        eid = i + 1
        elements.append(
            Element(id=eid, type=ElementType.FRAME_3D.value, nodes=[i + 1, i + 2], material_id=1, section_id=1)
        )
        dloads.append(DistributedLoad(element_id=eid, qx=0.0, qy=qy, qz=0.0, coordinate_system="GLOBAL"))

    w_mid_exact = (5.0 * qy * (L ** 4)) / (384.0 * E * sec.Iz)
    m_max_exact = (abs(qy) * (L ** 2)) / 8.0
    r_exact = -0.5 * qy * L

    return StructuralModel(
        name=f"04_Viga_Biapoyada_Carga_Distribuida_N{num_elements}",
        description=f"Viga simplemente apoyada (L=6m) bajo carga uniforme q=-15 kN/m ({num_elements} elementos).",
        nodes=nodes,
        materials=[Material(id=1, name="Acero S275", E=E, nu=0.3)],
        sections=[sec],
        elements=elements,
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_2D_XY),
            BoundaryCondition.from_preset(num_elements + 1, SupportPreset.ROLLER_2D_XY),
        ],
        distributed_loads=dloads,
        metadata={
            "analytical_solution": {
                "midspan_uy": w_mid_exact,
                "max_moment_z": m_max_exact,
                "reaction_Ry_each": r_exact,
            }
        },
    )


def space_portal_frame_3d() -> StructuralModel:
    """
    Caso 5: Pórtico espacial 3D de una planta (4 columnas empotradas en la base + 4 vigas de cubierta)
    sometido a cargas distribuidas verticales en las vigas y empujes horizontales de viento/sismo en X y Z.
    Dimensiones: planta de 5.0 m x 4.0 m, altura H = 3.5 m.
    """
    Lx, Lz, H = 5.0, 4.0, 3.5
    E = 210e9
    sec_col = CrossSection.rectangular(1, "Columna 300x300 mm", b=0.30, h=0.30)
    sec_beam = CrossSection.rectangular(2, "Viga 200x400 mm", b=0.20, h=0.40)

    nodes = [
        # Base (y = 0)
        Node3D(id=1, x=0.0, y=0.0, z=0.0, label="Base B1"),
        Node3D(id=2, x=Lx, y=0.0, z=0.0, label="Base B2"),
        Node3D(id=3, x=Lx, y=0.0, z=Lz, label="Base B3"),
        Node3D(id=4, x=0.0, y=0.0, z=Lz, label="Base B4"),
        # Cubierta (y = H)
        Node3D(id=5, x=0.0, y=H, z=0.0, label="Nudo C1"),
        Node3D(id=6, x=Lx, y=H, z=0.0, label="Nudo C2"),
        Node3D(id=7, x=Lx, y=H, z=Lz, label="Nudo C3"),
        Node3D(id=8, x=0.0, y=H, z=Lz, label="Nudo C4"),
    ]

    elements = [
        # 4 Columnas
        Element(id=1, type=ElementType.FRAME_3D.value, nodes=[1, 5], material_id=1, section_id=1, label="Col 1"),
        Element(id=2, type=ElementType.FRAME_3D.value, nodes=[2, 6], material_id=1, section_id=1, label="Col 2"),
        Element(id=3, type=ElementType.FRAME_3D.value, nodes=[3, 7], material_id=1, section_id=1, label="Col 3"),
        Element(id=4, type=ElementType.FRAME_3D.value, nodes=[4, 8], material_id=1, section_id=1, label="Col 4"),
        # 4 Vigas perimetrales
        Element(id=5, type=ElementType.FRAME_3D.value, nodes=[5, 6], material_id=1, section_id=2, label="Viga Norte"),
        Element(id=6, type=ElementType.FRAME_3D.value, nodes=[6, 7], material_id=1, section_id=2, label="Viga Este"),
        Element(id=7, type=ElementType.FRAME_3D.value, nodes=[8, 7], material_id=1, section_id=2, label="Viga Sur"),
        Element(id=8, type=ElementType.FRAME_3D.value, nodes=[5, 8], material_id=1, section_id=2, label="Viga Oeste"),
    ]

    bcs = [
        BoundaryCondition.from_preset(1, SupportPreset.FIXED_3D),
        BoundaryCondition.from_preset(2, SupportPreset.FIXED_3D),
        BoundaryCondition.from_preset(3, SupportPreset.FIXED_3D),
        BoundaryCondition.from_preset(4, SupportPreset.FIXED_3D),
    ]

    nloads = [
        NodalLoad(node_id=5, fx=25e3, fy=0.0, fz=10e3, label="Empuje Lateral C1"),
        NodalLoad(node_id=8, fx=25e3, fy=0.0, fz=10e3, label="Empuje Lateral C4"),
    ]

    dloads = [
        DistributedLoad(element_id=5, qy=-12e3, coordinate_system="GLOBAL", label="Carga Cubierta Norte"),
        DistributedLoad(element_id=7, qy=-12e3, coordinate_system="GLOBAL", label="Carga Cubierta Sur"),
    ]

    return StructuralModel(
        name="05_Portico_Espacial_3D",
        description="Pórtico espacial 3D de 4 columnas y 4 vigas sometido a cargas gravitatorias y empuje lateral.",
        nodes=nodes,
        materials=[Material(id=1, name="Acero S355", E=E, nu=0.3)],
        sections=[sec_col, sec_beam],
        elements=elements,
        boundary_conditions=bcs,
        nodal_loads=nloads,
        distributed_loads=dloads,
    )


def triangle_plate_patch_test() -> StructuralModel:
    """
    Caso 6: Placa cuadrada Lx = 2.0 m, Ly = 1.0 m, espesor t = 0.02 m, discretizada en 2 elementos TRIANGLE_3D (CST)
    bajo tracción uniaxial uniforme sigma_xx = 50 MPa.
    Fuerza total en el borde derecho x = 2.0 m: F_total = sigma_xx * (Ly * t) = 50e6 * (1.0 * 0.02) = 1,000,000 N (500 kN en cada nodo derecho).
    Solución analítica exacta:
      ux(x=2) = (sigma_xx / E) * Lx = (50e6 / 200e9) * 2.0 = 0.0005 m (0.5 mm)
      uy(y=1) = -nu * (sigma_xx / E) * Ly = -0.3 * 2.5e-4 * 1.0 = -7.5e-5 m
    """
    Lx, Ly, t = 2.0, 1.0, 0.02
    E, nu = 200e9, 0.3
    sigma_0 = 50e6
    f_edge = sigma_0 * Ly * t

    return StructuralModel(
        name="06_Placa_Membrana_CST_3D",
        description="Placa rectangular discretizada en elementos triangulares CST bajo tracción uniaxial (Patch Test).",
        nodes=[
            Node3D(id=1, x=0.0, y=0.0, z=0.0, label="Inf-Izq"),
            Node3D(id=2, x=Lx, y=0.0, z=0.0, label="Inf-Der"),
            Node3D(id=3, x=Lx, y=Ly, z=0.0, label="Sup-Der"),
            Node3D(id=4, x=0.0, y=Ly, z=0.0, label="Sup-Izq"),
        ],
        materials=[Material(id=1, name="Acero Placa", E=E, nu=nu)],
        sections=[CrossSection(id=1, name="Espesor 20mm", A=Ly * t, thickness=t)],
        elements=[
            Element(id=1, type=ElementType.TRIANGLE_3D.value, nodes=[1, 2, 3], material_id=1, section_id=1, label="CST 1"),
            Element(id=2, type=ElementType.TRIANGLE_3D.value, nodes=[1, 3, 4], material_id=1, section_id=1, label="CST 2"),
        ],
        boundary_conditions=[
            # Nodo 1 fijo en X, Y, Z; Nodo 4 fijo en X, Z (libre en Y para contracción de Poisson); Nodos 2 y 3 fijos en Z
            BoundaryCondition(node_id=1, ux=True, uy=True, uz=True),
            BoundaryCondition(node_id=4, ux=True, uy=False, uz=True),
            BoundaryCondition(node_id=2, ux=False, uy=False, uz=True),
            BoundaryCondition(node_id=3, ux=False, uy=False, uz=True),
        ],
        nodal_loads=[
            NodalLoad(node_id=2, fx=0.5 * f_edge),
            NodalLoad(node_id=3, fx=0.5 * f_edge),
        ],
        metadata={
            "analytical_solution": {
                "ux_right": (sigma_0 / E) * Lx,
                "uy_top": -nu * (sigma_0 / E) * Ly,
                "sigma_xx": sigma_0,
            }
        },
    )


def tetra4_solid_prism_benchmark() -> StructuralModel:
    """
    Caso 7: Cubo / Prisma sólido 3D (Lx = 1.0 m, Ly = 1.0 m, Lz = 1.0 m) descompuesto en 6 tetraedros TETRA_4
    que comparten la diagonal principal (nodo 1 -> nodo 7).
    Se aplica un desplazamiento axial prescrito ux = 0.001 m en la cara x = 1.0 m (o cargas equivalentes),
    dejando libre la contracción transversal de Poisson en Y y Z con coacciones mínimas isostáticas en x = 0.
    Solución analítica exacta en cada uno de los 6 tetraedros (Patch Test 3D):
      exx = 0.001, eyy = ezz = -nu * 0.001
      sigma_xx = E * exx = 200 MPa, sigma_yy = sigma_zz = 0
      von_mises = 200 MPa
    """
    E, nu = 200e9, 0.3
    u0 = 0.001  # 1 mm en Lx = 1 m

    nodes = [
        Node3D(id=1, x=0.0, y=0.0, z=0.0, label="000"),
        Node3D(id=2, x=1.0, y=0.0, z=0.0, label="100"),
        Node3D(id=3, x=1.0, y=1.0, z=0.0, label="110"),
        Node3D(id=4, x=0.0, y=1.0, z=0.0, label="010"),
        Node3D(id=5, x=0.0, y=0.0, z=1.0, label="001"),
        Node3D(id=6, x=1.0, y=0.0, z=1.0, label="101"),
        Node3D(id=7, x=1.0, y=1.0, z=1.0, label="111"),
        Node3D(id=8, x=0.0, y=1.0, z=1.0, label="011"),
    ]

    # Descomposición de Kuhn de un cubo en 6 tetraedros de volumen 1/6 m³ cada uno
    tets = [
        [1, 2, 3, 7],
        [1, 3, 4, 7],
        [1, 4, 8, 7],
        [1, 8, 5, 7],
        [1, 5, 6, 7],
        [1, 6, 2, 7],
    ]
    elements = [
        Element(id=i + 1, type=ElementType.TETRA_4.value, nodes=tet, material_id=1, section_id=1, label=f"Tet {i+1}")
        for i, tet in enumerate(tets)
    ]

    # Condiciones de contorno para tracción uniaxial pura sin restricción transversal:
    # Cara x=0 (nodos 1, 4, 5, 8) tiene ux=0; además:
    #   nodo 1: uy=0, uz=0 (fija traslación rígida Y, Z)
    #   nodo 4 (0,1,0): uz=0 (fija giro alrededor de X y Y)
    #   nodo 5 (0,0,1): uy=0 (fija giro alrededor de Z)
    # Cara x=1 (nodos 2, 3, 6, 7) tiene ux prescrito = u0
    bcs = [
        BoundaryCondition(node_id=1, ux=True, uy=True, uz=True),
        BoundaryCondition(node_id=4, ux=True, uy=False, uz=True),
        BoundaryCondition(node_id=5, ux=True, uy=True, uz=False),
        BoundaryCondition(node_id=8, ux=True, uy=False, uz=False),
        BoundaryCondition(node_id=2, ux=True, prescribed_ux=u0),
        BoundaryCondition(node_id=3, ux=True, prescribed_ux=u0),
        BoundaryCondition(node_id=6, ux=True, prescribed_ux=u0),
        BoundaryCondition(node_id=7, ux=True, prescribed_ux=u0),
    ]

    return StructuralModel(
        name="07_Solido_Tetraedrico_PatchTest_3D",
        description="Cubo unitario discretizado en 6 elementos sólidos TETRA_4 bajo elongación axial prescrita.",
        nodes=nodes,
        materials=[Material(id=1, name="Acero Isotrópico 3D", E=E, nu=nu)],
        sections=[CrossSection(id=1, name="Sólido 3D", A=1.0)],
        elements=elements,
        boundary_conditions=bcs,
        metadata={
            "analytical_solution": {
                "strain_xx": u0,
                "strain_yy": -nu * u0,
                "strain_zz": -nu * u0,
                "sigma_xx": E * u0,
                "von_mises": E * u0,
            }
        },
    )


def insufficient_supports_benchmark() -> StructuralModel:
    """
    Caso 8 (Prueba negativa): Barra con un único apoyo tipo rodillo que permite traslación libre en X.
    Debe ser detectado como sistema singular / mecanismo de sólido rígido.
    """
    return StructuralModel(
        name="08_Caso_Invalido_Apoyos_Insuficientes",
        description="Estructura hipostática con apoyo insuficiente en dirección X (mecanismo).",
        nodes=[
            Node3D(id=1, x=0.0, y=0.0, z=0.0),
            Node3D(id=2, x=2.0, y=0.0, z=0.0),
        ],
        materials=[Material(id=1, name="Acero", E=200e9, nu=0.3)],
        sections=[CrossSection(id=1, name="Sección", A=0.001)],
        elements=[Element(id=1, type=ElementType.TRUSS_3D.value, nodes=[1, 2], material_id=1, section_id=1)],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.ROLLER_X),
            BoundaryCondition.from_preset(2, SupportPreset.ROLLER_X),
        ],
        nodal_loads=[NodalLoad(node_id=2, fx=10e3)],
    )


def collinear_mechanism_truss_benchmark() -> StructuralModel:
    """
    Caso 9 (Prueba negativa): Dos barras TRUSS_3D colineales cuyos extremos están articulados,
    pero el nudo central (nodo 2) no tiene coacción transversal en Y ni Z.
    Debe ser detectado inmediatamente como un mecanismo interno (rigidez transversal nula).
    """
    return StructuralModel(
        name="09_Caso_Invalido_Mecanismo_Colineal",
        description="Dos barras articuladas colineales con nudo central inestable (mecanismo cinemático).",
        nodes=[
            Node3D(id=1, x=0.0, y=0.0, z=0.0),
            Node3D(id=2, x=2.0, y=0.0, z=0.0),
            Node3D(id=3, x=4.0, y=0.0, z=0.0),
        ],
        materials=[Material(id=1, name="Acero", E=200e9, nu=0.3)],
        sections=[CrossSection(id=1, name="Sección", A=0.001)],
        elements=[
            Element(id=1, type=ElementType.TRUSS_3D.value, nodes=[1, 2], material_id=1, section_id=1),
            Element(id=2, type=ElementType.TRUSS_3D.value, nodes=[2, 3], material_id=1, section_id=1),
        ],
        boundary_conditions=[
            BoundaryCondition.from_preset(1, SupportPreset.PINNED_3D),
            BoundaryCondition.from_preset(3, SupportPreset.PINNED_3D),
        ],
        nodal_loads=[NodalLoad(node_id=2, fy=-5e3)],
    )


def get_all_reference_models() -> Dict[str, StructuralModel]:
    return {
        "01_axial_bar": axial_bar_benchmark(),
        "02_space_truss_3d": small_space_truss_benchmark(),
        "03_cantilever_beam": cantilever_beam_point_load(num_elements=4),
        "04_simply_supported_beam": simply_supported_beam_uniform_load(num_elements=4),
        "05_space_portal_frame": space_portal_frame_3d(),
        "06_triangle_plate_cst": triangle_plate_patch_test(),
        "07_tetra4_solid_prism": tetra4_solid_prism_benchmark(),
        "08_insufficient_supports": insufficient_supports_benchmark(),
        "09_collinear_mechanism": collinear_mechanism_truss_benchmark(),
    }


def export_all_benchmarks(output_dir: str | Path) -> List[Path]:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths: List[Path] = []
    for key, model in get_all_reference_models().items():
        file_path = out / f"{key}.structura.json"
        model.save(file_path)
        paths.append(file_path)
    return paths
