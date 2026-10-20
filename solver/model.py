"""
Modelo de datos y formato de malla (.structura.json) para Structura 3D.

Define las entidades fundamentales del modelo estructural:
- Unidades físicas explícitas (SI por defecto: m, N, Pa, kg/m³).
- Nodos 3D (x, y, z) y condiciones de contorno / apoyos (6 GDL por nodo: ux, uy, uz, rx, ry, rz).
- Materiales isotrópicos elásticos lineales (E, nu, G, rho, fy_referencia).
- Secciones transversales paramétricas o directas (A, Iy, Iz, J, wy, wz).
- Elementos estructurales y finitos:
    * TRUSS_3D: Barra espacial articulada (2 nodos, sólo rigidez axial EA/L).
    * FRAME_3D: Viga-columna espacial de Euler-Bernoulli (2 nodos, 6 GDL/nodo).
    * TRIANGLE_3D (CST): Elemento finito triangular de deformación constante (3 nodos, espesor t).
    * TETRA_4: Elemento finito sólido tetraédrico lineal (4 nodos, 3 GDL/nodo).
- Cargas estáticas:
    * Cargas puntuales nodales (fx, fy, fz, mx, my, mz).
    * Cargas distribuidas uniformes sobre barras/vigas (qx, qy, qz en sistema global o local).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any


DISCLAIMER_TEXT = (
    "ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D "
    "corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. "
    "NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural."
)


class ElementType(str, Enum):
    TRUSS_3D = "TRUSS_3D"         # Barra articulada 3D (2 nodos, esfuerzos axiales)
    FRAME_3D = "FRAME_3D"         # Viga-columna 3D Euler-Bernoulli (2 nodos, 6 GDL/nodo)
    TRIANGLE_3D = "TRIANGLE_3D"   # Membrana triangular CST en 3D (3 nodos, 3 GDL/nodo)
    TETRA_4 = "TETRA_4"           # Sólido tetraédrico lineal 3D (4 nodos, 3 GDL/nodo)


class SupportPreset(str, Enum):
    FREE = "FREE"                 # Libre en los 6 GDL
    PINNED_3D = "PINNED_3D"       # Articulación fija 3D (ux=uy=uz=0, giros libres)
    FIXED_3D = "FIXED_3D"         # Empotramiento perfecto 3D (ux=uy=uz=rx=ry=rz=0)
    ROLLER_X = "ROLLER_X"         # Apoyo deslizante (restringe uy, uz; libre en ux)
    ROLLER_Y = "ROLLER_Y"         # Apoyo deslizante (restringe ux, uz; libre en uy)
    ROLLER_Z = "ROLLER_Z"         # Apoyo deslizante sobre plano XY (restringe uz)
    PINNED_2D_XY = "PINNED_2D_XY" # Articulado en plano XY (ux=uy=uz=rx=ry=0, rz libre)
    ROLLER_2D_XY = "ROLLER_2D_XY" # Rodillo en plano XY con apoyo vertical Y (uy=uz=rx=ry=0)
    CUSTOM = "CUSTOM"


@dataclass
class UnitSystem:
    length: str = "m"
    force: str = "N"
    stress: str = "Pa"
    moment: str = "N·m"
    density: str = "kg/m³"
    angle: str = "rad"


@dataclass
class Node3D:
    id: int
    x: float
    y: float
    z: float
    label: str = ""

    def coords(self) -> Tuple[float, float, float]:
        return (float(self.x), float(self.y), float(self.z))


@dataclass
class Material:
    id: int
    name: str
    E: float               # Módulo de elasticidad de Young [Pa]
    nu: float = 0.3        # Coeficiente de Poisson [-] (0 <= nu < 0.5)
    rho: float = 7850.0    # Densidad [kg/m³]
    yield_strength: float = 250e6  # Límite elástico sólo informativo [Pa]

    @property
    def G(self) -> float:
        """Módulo de cortante isotrópico G = E / (2 * (1 + nu))."""
        return self.E / (2.0 * (1.0 + self.nu))


@dataclass
class CrossSection:
    id: int
    name: str
    shape: str = "GENERIC"  # GENERIC, RECTANGULAR, CIRCULAR, TUBE, I_SECTION
    A: float = 0.01         # Área de la sección transversal [m²]
    Iy: float = 1e-5        # Segundo momento de área respecto al eje local y [m⁴]
    Iz: float = 1e-5        # Segundo momento de área respecto al eje local z [m⁴]
    J: float = 2e-5         # Constante torsional de Saint-Venant [m⁴]
    thickness: float = 0.01 # Espesor para elementos TRIANGLE_3D [m]
    dimensions: Dict[str, float] = field(default_factory=dict)

    @classmethod
    def rectangular(cls, sec_id: int, name: str, b: float, h: float) -> "CrossSection":
        """Crea una sección rectangular maciza de ancho b (eje z local) y canto h (eje y local)."""
        area = b * h
        iz = (b * (h ** 3)) / 12.0
        iy = (h * (b ** 3)) / 12.0
        a_long = max(b, h)
        b_short = min(b, h)
        beta = 1.0 / 3.0 - 0.21 * (b_short / a_long) * (1.0 - (b_short ** 4) / (12.0 * (a_long ** 4)))
        j_torsion = beta * a_long * (b_short ** 3)
        return cls(
            id=sec_id,
            name=name,
            shape="RECTANGULAR",
            A=area,
            Iy=iy,
            Iz=iz,
            J=j_torsion,
            dimensions={"b": b, "h": h},
        )

    @classmethod
    def circular(cls, sec_id: int, name: str, diameter: float) -> "CrossSection":
        """Crea una sección circular maciza de diámetro d."""
        r = diameter / 2.0
        area = math.pi * (r ** 2)
        inertia = (math.pi * (diameter ** 4)) / 64.0
        j_torsion = (math.pi * (diameter ** 4)) / 32.0
        return cls(
            id=sec_id,
            name=name,
            shape="CIRCULAR",
            A=area,
            Iy=inertia,
            Iz=inertia,
            J=j_torsion,
            dimensions={"d": diameter},
        )


@dataclass
class Element:
    id: int
    type: str              # Valor de ElementType (TRUSS_3D, FRAME_3D, TRIANGLE_3D, TETRA_4)
    nodes: List[int]       # IDs de nodos conectados (2 para barra/viga, 3 para triángulo, 4 para tetraedro)
    material_id: int
    section_id: int = 1
    roll_angle: float = 0.0  # Ángulo de giro del eje local alrededor del eje longitudinal [rad]
    label: str = ""


@dataclass
class BoundaryCondition:
    node_id: int
    ux: bool = False
    uy: bool = False
    uz: bool = False
    rx: bool = False
    ry: bool = False
    rz: bool = False
    prescribed_ux: float = 0.0
    prescribed_uy: float = 0.0
    prescribed_uz: float = 0.0
    prescribed_rx: float = 0.0
    prescribed_ry: float = 0.0
    prescribed_rz: float = 0.0
    preset: str = SupportPreset.CUSTOM.value

    @classmethod
    def from_preset(cls, node_id: int, preset: SupportPreset) -> "BoundaryCondition":
        if preset == SupportPreset.FIXED_3D:
            return cls(node_id=node_id, ux=True, uy=True, uz=True, rx=True, ry=True, rz=True, preset=preset.value)
        if preset == SupportPreset.PINNED_3D:
            return cls(node_id=node_id, ux=True, uy=True, uz=True, rx=False, ry=False, rz=False, preset=preset.value)
        if preset == SupportPreset.ROLLER_X:
            return cls(node_id=node_id, ux=False, uy=True, uz=True, rx=False, ry=False, rz=False, preset=preset.value)
        if preset == SupportPreset.ROLLER_Y:
            return cls(node_id=node_id, ux=True, uy=False, uz=True, rx=False, ry=False, rz=False, preset=preset.value)
        if preset == SupportPreset.ROLLER_Z:
            return cls(node_id=node_id, ux=False, uy=False, uz=True, rx=False, ry=False, rz=False, preset=preset.value)
        if preset == SupportPreset.PINNED_2D_XY:
            return cls(node_id=node_id, ux=True, uy=True, uz=True, rx=True, ry=True, rz=False, preset=preset.value)
        if preset == SupportPreset.ROLLER_2D_XY:
            return cls(node_id=node_id, ux=False, uy=True, uz=True, rx=True, ry=True, rz=False, preset=preset.value)
        return cls(node_id=node_id, preset=SupportPreset.FREE.value)

    def flags(self) -> List[bool]:
        return [self.ux, self.uy, self.uz, self.rx, self.ry, self.rz]

    def prescribed_values(self) -> List[float]:
        return [
            self.prescribed_ux,
            self.prescribed_uy,
            self.prescribed_uz,
            self.prescribed_rx,
            self.prescribed_ry,
            self.prescribed_rz,
        ]


@dataclass
class NodalLoad:
    node_id: int
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    label: str = ""

    def vector(self) -> List[float]:
        return [self.fx, self.fy, self.fz, self.mx, self.my, self.mz]


@dataclass
class DistributedLoad:
    element_id: int
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    coordinate_system: str = "GLOBAL"  # "GLOBAL" o "LOCAL"
    label: str = ""


@dataclass
class StructuralModel:
    name: str
    description: str = ""
    units: UnitSystem = field(default_factory=UnitSystem)
    nodes: List[Node3D] = field(default_factory=list)
    materials: List[Material] = field(default_factory=list)
    sections: List[CrossSection] = field(default_factory=list)
    elements: List[Element] = field(default_factory=list)
    boundary_conditions: List[BoundaryCondition] = field(default_factory=list)
    nodal_loads: List[NodalLoad] = field(default_factory=list)
    distributed_loads: List[DistributedLoad] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["schema_version"] = "1.0.0"
        data["disclaimer"] = DISCLAIMER_TEXT
        return data

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    def save(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(self.to_json(indent=2), encoding="utf-8")

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "StructuralModel":
        units_data = data.get("units", {})
        units = UnitSystem(**units_data) if isinstance(units_data, dict) else UnitSystem()
        nodes = [
            Node3D(**{**n, "id": int(n["id"])})
            for n in data.get("nodes", [])
        ]
        materials = [
            Material(**{**m, "id": int(m["id"])})
            for m in data.get("materials", [])
        ]
        sections = [
            CrossSection(**{**s, "id": int(s["id"])})
            for s in data.get("sections", [])
        ]
        elements = [
            Element(
                **{
                    **e,
                    "id": int(e["id"]),
                    "nodes": [int(nid) for nid in e.get("nodes", [])],
                    "material_id": int(e["material_id"]),
                    "section_id": int(e.get("section_id", 1)),
                }
            )
            for e in data.get("elements", [])
        ]
        bcs = [
            BoundaryCondition(**{**bc, "node_id": int(bc["node_id"])})
            for bc in data.get("boundary_conditions", [])
        ]
        nloads = [
            NodalLoad(**{**nl, "node_id": int(nl["node_id"])})
            for nl in data.get("nodal_loads", [])
        ]
        dloads = [
            DistributedLoad(**{**dl, "element_id": int(dl["element_id"])})
            for dl in data.get("distributed_loads", [])
        ]
        return cls(
            name=data.get("name", "Modelo sin título"),
            description=data.get("description", ""),
            units=units,
            nodes=nodes,
            materials=materials,
            sections=sections,
            elements=elements,
            boundary_conditions=bcs,
            nodal_loads=nloads,
            distributed_loads=dloads,
            metadata=data.get("metadata", {}),
        )

    @classmethod
    def load(cls, path: str | Path) -> "StructuralModel":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(raw)


@dataclass
class ElementResult:
    element_id: int
    element_type: str
    length_or_measure: float          # Longitud [m], área [m²] o volumen [m³]
    axial_force: float = 0.0          # N [N] (+ tracción, - compresión)
    shear_y: float = 0.0              # Vy máximo en magnitud [N]
    shear_z: float = 0.0              # Vz máximo en magnitud [N]
    torsion: float = 0.0              # Mx [N·m]
    moment_y: float = 0.0             # My máximo en magnitud [N·m]
    moment_z: float = 0.0             # Mz máximo en magnitud [N·m]
    axial_stress: float = 0.0         # sigma_x = N / A [Pa]
    bending_stress_max: float = 0.0   # sigma_flexión máxima estimada [Pa]
    combined_stress: float = 0.0      # |sigma_axial| + |sigma_flexión| [Pa]
    von_mises_stress: float = 0.0     # Tensión equivalente de Von Mises [Pa]
    strain_tensor_voigt: List[float] = field(default_factory=list)  # [exx, eyy, ezz, gxy, gyz, gxz]
    stress_tensor_voigt: List[float] = field(default_factory=list)  # [sxx, syy, szz, txy, tyz, txz]
    local_end_forces: List[float] = field(default_factory=list)


@dataclass
class SolverDiagnostics:
    is_valid: bool = True
    is_singular: bool = False
    is_mechanism: bool = False
    condition_number: float = 1.0
    matrix_rank: int = 0
    free_dofs: int = 0
    constrained_dofs: int = 0
    total_dofs: int = 0
    zero_energy_modes: int = 0
    force_residual_norm: float = 0.0
    moment_residual_norm: float = 0.0
    equilibrium_satisfied: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


@dataclass
class SolverResult:
    success: bool
    model_name: str
    diagnostics: SolverDiagnostics
    nodal_displacements: Dict[int, List[float]] = field(default_factory=dict)  # node_id -> [ux, uy, uz, rx, ry, rz]
    nodal_reactions: Dict[int, List[float]] = field(default_factory=dict)      # node_id -> [Rx, Ry, Rz, Mx, My, Mz]
    element_results: Dict[int, ElementResult] = field(default_factory=dict)    # elem_id -> ElementResult
    max_displacement: float = 0.0
    max_von_mises: float = 0.0
    max_axial_stress: float = 0.0
    min_axial_stress: float = 0.0
    recommended_visual_scale: float = 1.0
    strain_energy: float = 0.0
    disclaimer: str = DISCLAIMER_TEXT

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)
