"""
Formulaciones de Elementos Estructurales y Finitos 3D para Structura 3D.

Contiene las rutinas de ensamblaje local, transformación de coordenadas y recuperación
de esfuerzos internos y tensiones para:
1. TRUSS_3D: Barra espacial articulada (2 nodos, rigidez axial EA/L).
2. FRAME_3D: Viga-columna espacial de Euler-Bernoulli (2 nodos, 12 GDL).
3. TRIANGLE_3D: Elemento finito triangular de deformación constante CST en 3D (3 nodos).
4. TETRA_4: Elemento finito sólido tetraédrico lineal de 4 nodos (12 GDL traslacionales).
"""

from __future__ import annotations

import math
from typing import Tuple, List
import numpy as np

from .model import Node3D, Material, CrossSection, ElementResult


def compute_von_mises_3d(stress_voigt: np.ndarray) -> float:
    """
    Calcula la tensión equivalente de Von Mises a partir del vector de Cauchy en notación de Voigt:
    stress_voigt = [sigma_xx, sigma_yy, sigma_zz, tau_xy, tau_yz, tau_xz].
    """
    sxx, syy, szz, txy, tyz, txz = stress_voigt
    term_normal = 0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
    term_shear = 3.0 * (txy ** 2 + tyz ** 2 + txz ** 2)
    return float(math.sqrt(max(0.0, term_normal + term_shear)))


def beam_rotation_matrix_3d(
    p1: np.ndarray, p2: np.ndarray, roll_angle: float = 0.0
) -> Tuple[np.ndarray, float]:
    """
    Construye la matriz de cosenos directores R (3x3) cuyas filas son los vectores
    unitarios de los ejes locales (x', y', z') expresados en coordenadas globales (X, Y, Z),
    y devuelve (R, L).
    """
    delta = p2 - p1
    length = float(np.linalg.norm(delta))
    if length <= 1e-12:
        raise ValueError("Elemento de longitud nula o casi nula (nodos coincidentes).")

    ex = delta / length
    # Eje global Y = (0, 1, 0) como referencia por defecto; si la barra es casi vertical, usar Z = (0, 0, 1)
    if abs(ex[0]) < 1e-6 and abs(ex[2]) < 1e-6:
        # Barra vertical paralela al eje Y global
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        ez = np.cross(ex, ref)
        ez /= np.linalg.norm(ez)
        ey = np.cross(ez, ex)
    else:
        up = np.array([0.0, 1.0, 0.0], dtype=float)
        ez = np.cross(ex, up)
        ez /= np.linalg.norm(ez)
        ey = np.cross(ez, ex)
        ey /= np.linalg.norm(ey)

    if abs(roll_angle) > 1e-12:
        c = math.cos(roll_angle)
        s = math.sin(roll_angle)
        ey_new = c * ey + s * ez
        ez_new = -s * ey + c * ez
        ey, ez = ey_new, ez_new

    R = np.vstack([ex, ey, ez])
    return R, length


def frame3d_transformation_matrix(R: np.ndarray) -> np.ndarray:
    """Construye la matriz de transformación 12x12 diagonal por bloques con R (3x3)."""
    T = np.zeros((12, 12), dtype=float)
    for i in range(4):
        T[3 * i : 3 * (i + 1), 3 * i : 3 * (i + 1)] = R
    return T


# =============================================================================
# 1. ELEMENTO DE BARRA ARTICULADA ESPACIAL (TRUSS_3D)
# =============================================================================

def truss3d_stiffness_global(
    n1: Node3D, n2: Node3D, mat: Material, sec: CrossSection
) -> Tuple[np.ndarray, float, np.ndarray]:
    """
    Devuelve la matriz de rigidez global 12x12 (con ceros en los GDL rotacionales),
    la longitud L y el vector unitario director ex (3,) de la barra articulada 3D.
    """
    p1 = np.array(n1.coords(), dtype=float)
    p2 = np.array(n2.coords(), dtype=float)
    delta = p2 - p1
    L = float(np.linalg.norm(delta))
    if L <= 1e-12:
        raise ValueError(f"Barra TRUSS_3D entre nodos {n1.id} y {n2.id} tiene longitud nula (L=0).")

    ex = delta / L
    k_val = (mat.E * sec.A) / L
    outer = np.outer(ex, ex)  # 3x3

    K_global = np.zeros((12, 12), dtype=float)
    # GDL traslacionales del nodo 1: [0, 1, 2], nodo 2: [6, 7, 8]
    K_global[0:3, 0:3] = k_val * outer
    K_global[0:3, 6:9] = -k_val * outer
    K_global[6:9, 0:3] = -k_val * outer
    K_global[6:9, 6:9] = k_val * outer

    return K_global, L, ex


def truss3d_recover_results(
    elem_id: int,
    n1: Node3D,
    n2: Node3D,
    mat: Material,
    sec: CrossSection,
    u_elem_global: np.ndarray,
) -> ElementResult:
    """Recupera fuerza axial N, deformación unitaria y tensión normal en una barra TRUSS_3D."""
    p1 = np.array(n1.coords(), dtype=float)
    p2 = np.array(n2.coords(), dtype=float)
    delta = p2 - p1
    L = float(np.linalg.norm(delta))
    ex = delta / L

    u1 = u_elem_global[0:3]
    u2 = u_elem_global[6:9]
    elongation = float(np.dot(ex, u2 - u1))
    strain_xx = elongation / L
    axial_stress = mat.E * strain_xx
    axial_force = axial_stress * sec.A

    strain_voigt = [strain_xx, -mat.nu * strain_xx, -mat.nu * strain_xx, 0.0, 0.0, 0.0]
    stress_voigt = [axial_stress, 0.0, 0.0, 0.0, 0.0, 0.0]

    return ElementResult(
        element_id=elem_id,
        element_type="TRUSS_3D",
        length_or_measure=L,
        axial_force=axial_force,
        axial_stress=axial_stress,
        combined_stress=abs(axial_stress),
        von_mises_stress=abs(axial_stress),
        strain_tensor_voigt=strain_voigt,
        stress_tensor_voigt=stress_voigt,
        local_end_forces=[-axial_force, 0.0, 0.0, 0.0, 0.0, 0.0, axial_force, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


# =============================================================================
# 2. ELEMENTO DE VIGA-COLUMNA ESPACIAL EULER-BERNOULLI (FRAME_3D)
# =============================================================================

def frame3d_local_stiffness(
    L: float, E: float, G: float, A: float, Iy: float, Iz: float, J: float
) -> np.ndarray:
    """Construye la matriz de rigidez local 12x12 de una viga Euler-Bernoulli 3D."""
    k = np.zeros((12, 12), dtype=float)

    # Rigidez axial (u_x1 = 0, u_x2 = 6)
    ea_l = E * A / L
    k[0, 0] = ea_l
    k[0, 6] = -ea_l
    k[6, 0] = -ea_l
    k[6, 6] = ea_l

    # Rigidez torsional (rx1 = 3, rx2 = 9)
    gj_l = G * J / L
    k[3, 3] = gj_l
    k[3, 9] = -gj_l
    k[9, 3] = -gj_l
    k[9, 9] = gj_l

    # Flexión en el plano local x'-y' (alrededor del eje z': uy1=1, rz1=5, uy2=7, rz2=11)
    eiz = E * Iz
    c_y1 = 12.0 * eiz / (L ** 3)
    c_y2 = 6.0 * eiz / (L ** 2)
    c_y3 = 4.0 * eiz / L
    c_y4 = 2.0 * eiz / L

    k[1, 1] = c_y1
    k[1, 5] = c_y2
    k[1, 7] = -c_y1
    k[1, 11] = c_y2

    k[5, 1] = c_y2
    k[5, 5] = c_y3
    k[5, 7] = -c_y2
    k[5, 11] = c_y4

    k[7, 1] = -c_y1
    k[7, 5] = -c_y2
    k[7, 7] = c_y1
    k[7, 11] = -c_y2

    k[11, 1] = c_y2
    k[11, 5] = c_y4
    k[11, 7] = -c_y2
    k[11, 11] = c_y3

    # Flexión en el plano local x'-z' (alrededor del eje y': uz1=2, ry1=4, uz2=8, ry2=10)
    eiy = E * Iy
    c_z1 = 12.0 * eiy / (L ** 3)
    c_z2 = 6.0 * eiy / (L ** 2)
    c_z3 = 4.0 * eiy / L
    c_z4 = 2.0 * eiy / L

    k[2, 2] = c_z1
    k[2, 4] = -c_z2
    k[2, 8] = -c_z1
    k[2, 10] = -c_z2

    k[4, 2] = -c_z2
    k[4, 4] = c_z3
    k[4, 8] = c_z2
    k[4, 10] = c_z4

    k[8, 2] = -c_z1
    k[8, 4] = c_z2
    k[8, 8] = c_z1
    k[8, 10] = c_z2

    k[10, 2] = -c_z2
    k[10, 4] = c_z4
    k[10, 8] = c_z2
    k[10, 10] = c_z3

    return k


def frame3d_equivalent_nodal_loads_local(
    L: float, q_local: np.ndarray
) -> np.ndarray:
    """
    Calcula el vector de fuerzas nodales equivalentes (12,) en coordenadas locales
    para una carga distribuida uniforme q_local = [qx', qy', qz'] [N/m].
    """
    qx, qy, qz = float(q_local[0]), float(q_local[1]), float(q_local[2])
    feq = np.zeros(12, dtype=float)

    # Carga axial distribuida uniforme qx'
    feq[0] = 0.5 * qx * L
    feq[6] = 0.5 * qx * L

    # Carga transversal qy' en plano x'-y'
    feq[1] = 0.5 * qy * L
    feq[5] = (qy * (L ** 2)) / 12.0
    feq[7] = 0.5 * qy * L
    feq[11] = -(qy * (L ** 2)) / 12.0

    # Carga transversal qz' en plano x'-z'
    feq[2] = 0.5 * qz * L
    feq[4] = -(qz * (L ** 2)) / 12.0
    feq[8] = 0.5 * qz * L
    feq[10] = (qz * (L ** 2)) / 12.0

    return feq


def frame3d_stiffness_global(
    n1: Node3D, n2: Node3D, mat: Material, sec: CrossSection, roll_angle: float = 0.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Devuelve (K_global, k_local, T, L) para un elemento FRAME_3D.
    """
    p1 = np.array(n1.coords(), dtype=float)
    p2 = np.array(n2.coords(), dtype=float)
    R, L = beam_rotation_matrix_3d(p1, p2, roll_angle=roll_angle)
    T = frame3d_transformation_matrix(R)
    k_local = frame3d_local_stiffness(L, mat.E, mat.G, sec.A, sec.Iy, sec.Iz, sec.J)
    K_global = T.T @ k_local @ T
    return K_global, k_local, T, L


def frame3d_recover_results(
    elem_id: int,
    n1: Node3D,
    n2: Node3D,
    mat: Material,
    sec: CrossSection,
    roll_angle: float,
    u_elem_global: np.ndarray,
    q_local_total: np.ndarray,
) -> ElementResult:
    """
    Recupera esfuerzos internos, momentos flectores máximos (incluyendo el interior del vano
    bajo carga distribuida) y tensiones normales/combinadas para un elemento FRAME_3D.
    """
    _, k_local, T, L = frame3d_stiffness_global(n1, n2, mat, sec, roll_angle=roll_angle)
    u_local = T @ u_elem_global
    feq_local = frame3d_equivalent_nodal_loads_local(L, q_local_total)

    # Fuerzas internas en los nudos extremos en coordenadas locales: f_int = k_local * u_local - feq_local
    f_int = k_local @ u_local - feq_local

    # Convención de esfuerzos de sección:
    # En extremo 1 (x=0): N(0) = -f_int[0], Vy(0) = f_int[1], Vz(0) = f_int[2], Mx(0) = -f_int[3], My(0) = -f_int[4], Mz(0) = -f_int[5]
    # En extremo 2 (x=L): N(L) = f_int[6],  Vy(L) = -f_int[7], Vz(L) = -f_int[8], Mx(L) = f_int[9], My(L) = f_int[10], Mz(L) = f_int[11]
    N_1 = -float(f_int[0])
    N_2 = float(f_int[6])
    axial_force = N_2 if abs(N_2) >= abs(N_1) else N_1

    Vy_1 = float(f_int[1])
    Vy_2 = -float(f_int[7])
    shear_y = max(abs(Vy_1), abs(Vy_2))

    Vz_1 = float(f_int[2])
    Vz_2 = -float(f_int[8])
    shear_z = max(abs(Vz_1), abs(Vz_2))

    torsion = max(abs(float(f_int[3])), abs(float(f_int[9])))

    # Muestreo del momento flector Mz(x) y My(x) a lo largo de la viga para capturar máximos en el vano
    qy = float(q_local_total[1])
    qz = float(q_local_total[2])
    Mz_0 = -float(f_int[5])
    My_0 = -float(f_int[4])

    xs = np.linspace(0.0, L, 41)
    # Mz(x) = Mz(0) + Vy(0)*x + 0.5*qy*x^2
    Mz_vals = Mz_0 + Vy_1 * xs + 0.5 * qy * (xs ** 2)
    # My(x) = My(0) - Vz(0)*x - 0.5*qz*x^2  (regla de la mano derecha: z' x x' = +y')
    My_vals = My_0 - Vz_1 * xs - 0.5 * qz * (xs ** 2)

    moment_z = float(np.max(np.abs(Mz_vals)))
    moment_y = float(np.max(np.abs(My_vals)))

    axial_stress = axial_force / sec.A if sec.A > 0 else 0.0

    # Radio de fibra extrema estimado a partir de las dimensiones o de sqrt(I/A)
    h_y = sec.dimensions.get("h", 2.0 * math.sqrt(max(sec.Iz, 1e-16) / max(sec.A, 1e-12)) * math.sqrt(3.0))
    b_z = sec.dimensions.get("b", 2.0 * math.sqrt(max(sec.Iy, 1e-16) / max(sec.A, 1e-12)) * math.sqrt(3.0))
    if "d" in sec.dimensions:
        h_y = sec.dimensions["d"]
        b_z = sec.dimensions["d"]

    c_y = 0.5 * h_y
    c_z = 0.5 * b_z
    sigma_bend_z = (moment_z * c_y / sec.Iz) if sec.Iz > 0 else 0.0
    sigma_bend_y = (moment_y * c_z / sec.Iy) if sec.Iy > 0 else 0.0
    bending_stress_max = sigma_bend_z + sigma_bend_y

    combined_stress = abs(axial_stress) + bending_stress_max
    # Tensión tangencial torsional aproximada
    r_eq = max(c_y, c_z)
    tau_torsion = (torsion * r_eq / sec.J) if sec.J > 0 else 0.0
    von_mises = math.sqrt(combined_stress ** 2 + 3.0 * (tau_torsion ** 2))

    strain_xx = axial_stress / mat.E
    return ElementResult(
        element_id=elem_id,
        element_type="FRAME_3D",
        length_or_measure=L,
        axial_force=axial_force,
        shear_y=shear_y,
        shear_z=shear_z,
        torsion=torsion,
        moment_y=moment_y,
        moment_z=moment_z,
        axial_stress=axial_stress,
        bending_stress_max=bending_stress_max,
        combined_stress=combined_stress,
        von_mises_stress=von_mises,
        strain_tensor_voigt=[strain_xx, -mat.nu * strain_xx, -mat.nu * strain_xx, 0.0, 0.0, 0.0],
        stress_tensor_voigt=[combined_stress, 0.0, 0.0, tau_torsion, 0.0, 0.0],
        local_end_forces=[float(x) for x in f_int],
    )


# =============================================================================
# 3. ELEMENTO FINITO TRIANGULAR DE DEFORMACIÓN CONSTANTE EN 3D (TRIANGLE_3D / CST)
# =============================================================================

def triangle3d_local_frame(
    n1: Node3D, n2: Node3D, n3: Node3D
) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Construye la base ortonormal local (ex, ey, ez) en el plano del triángulo (n1, n2, n3),
    las coordenadas 2D locales [(x1,y1), (x2,y2), (x3,y3)] y el área A del triángulo.
    """
    p1 = np.array(n1.coords(), dtype=float)
    p2 = np.array(n2.coords(), dtype=float)
    p3 = np.array(n3.coords(), dtype=float)

    v12 = p2 - p1
    v13 = p3 - p1
    normal = np.cross(v12, v13)
    norm_n = float(np.linalg.norm(normal))
    area = 0.5 * norm_n
    if area <= 1e-12:
        raise ValueError(
            f"Elemento TRIANGLE_3D ({n1.id}, {n2.id}, {n3.id}) es degenerado (área nula o nodos colineales)."
        )

    ex = v12 / np.linalg.norm(v12)
    ez = normal / norm_n
    ey = np.cross(ez, ex)
    R = np.vstack([ex, ey, ez])  # 3x3

    coords_2d = np.array(
        [
            [0.0, 0.0],
            [float(np.dot(v12, ex)), float(np.dot(v12, ey))],
            [float(np.dot(v13, ex)), float(np.dot(v13, ey))],
        ],
        dtype=float,
    )
    return R, coords_2d, area


def triangle3d_stiffness_global(
    n1: Node3D, n2: Node3D, n3: Node3D, mat: Material, sec: CrossSection
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Calcula la matriz de rigidez global 9x9 (en los 3 GDL traslacionales de cada uno de los 3 nodos)
    para un elemento triangular de membrana CST (Constant Strain Triangle) en tensión plana.
    Su matriz de rigidez 9x9 posee rango exactamente 3 (sin rigidez transversal fuera de su plano
    ni rigidez a flexión de placa).
    """
    if not math.isfinite(sec.thickness) or sec.thickness <= 0.0:
        raise ValueError(
            f"Sección '{sec.name}' (id={sec.id}) usada en TRIANGLE_3D tiene espesor inválido thickness={sec.thickness} (debe ser > 0 y finito)."
        )

    R, coords_2d, area = triangle3d_local_frame(n1, n2, n3)
    x1, y1 = coords_2d[0]
    x2, y2 = coords_2d[1]
    x3, y3 = coords_2d[2]

    b1 = y2 - y3
    b2 = y3 - y1
    b3 = y1 - y2
    c1 = x3 - x2
    c2 = x1 - x3
    c3 = x2 - x1

    B_2d = (1.0 / (2.0 * area)) * np.array(
        [
            [b1, 0.0, b2, 0.0, b3, 0.0],
            [0.0, c1, 0.0, c2, 0.0, c3],
            [c1, b1, c2, b2, c3, b3],
        ],
        dtype=float,
    )

    E = mat.E
    nu = mat.nu
    D_plane_stress = (E / (1.0 - nu ** 2)) * np.array(
        [
            [1.0, nu, 0.0],
            [nu, 1.0, 0.0],
            [0.0, 0.0, 0.5 * (1.0 - nu)],
        ],
        dtype=float,
    )

    t = float(sec.thickness)
    k_local_6x6 = t * area * (B_2d.T @ D_plane_stress @ B_2d)

    # Proyección exacta de los 6 GDL de membrana en el plano local (u_x', u_y' por nodo)
    # a los 9 GDL traslacionales globales 3D. No se añade rigidez transversal ni rotacional artificial;
    # la respuesta fuera del plano requiere apoyos coplanares o una formulación de placa/lámina.
    T_proj = np.zeros((6, 9), dtype=float)
    for i in range(3):
        T_proj[2 * i : 2 * i + 2, 3 * i : 3 * i + 3] = R[0:2, :]

    K_global_9x9 = T_proj.T @ k_local_6x6 @ T_proj
    return K_global_9x9, B_2d, D_plane_stress, T_proj, area


def triangle3d_recover_results(
    elem_id: int,
    n1: Node3D,
    n2: Node3D,
    n3: Node3D,
    mat: Material,
    sec: CrossSection,
    u_trans_9: np.ndarray,
) -> ElementResult:
    """
    Recupera deformaciones, tensiones de Cauchy y tensión equivalente de Von Mises en TRIANGLE_3D.
    Los tensores de deformación y tensión se transforman desde la base local del triángulo (x', y', z')
    hacia el sistema de coordenadas global (X, Y, Z) mediante T_glob = R^T * T_loc * R, garantizando
    invariancia total ante permutaciones de la conectividad nodal del elemento.
    Orden de Voigt global:
      strain_tensor_voigt = [exx, eyy, ezz, gxy, gyz, gxz]
      stress_tensor_voigt = [sxx, syy, szz, txy, tyz, txz]
    """
    R, _, _ = triangle3d_local_frame(n1, n2, n3)
    _, B_2d, D_ps, T_proj, area = triangle3d_stiffness_global(n1, n2, n3, mat, sec)
    u_local_6 = T_proj @ u_trans_9
    strain_2d = B_2d @ u_local_6  # [ex'x', ey'y', gx'y']
    stress_2d = D_ps @ strain_2d  # [sx'x', sy'y', tx'y']

    sxx_l, syy_l, txy_l = float(stress_2d[0]), float(stress_2d[1]), float(stress_2d[2])
    exx_l, eyy_l, gxy_l = float(strain_2d[0]), float(strain_2d[1]), float(strain_2d[2])
    ezz_l = -(mat.nu / mat.E) * (sxx_l + syy_l)

    sigma_local_3x3 = np.array(
        [
            [sxx_l, txy_l, 0.0],
            [txy_l, syy_l, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    eps_local_3x3 = np.array(
        [
            [exx_l, 0.5 * gxy_l, 0.0],
            [0.5 * gxy_l, eyy_l, 0.0],
            [0.0, 0.0, ezz_l],
        ],
        dtype=float,
    )

    sigma_global_3x3 = R.T @ sigma_local_3x3 @ R
    eps_global_3x3 = R.T @ eps_local_3x3 @ R

    stress_voigt_global = np.array(
        [
            sigma_global_3x3[0, 0],
            sigma_global_3x3[1, 1],
            sigma_global_3x3[2, 2],
            sigma_global_3x3[0, 1],
            sigma_global_3x3[1, 2],
            sigma_global_3x3[0, 2],
        ],
        dtype=float,
    )
    strain_voigt_global = [
        float(eps_global_3x3[0, 0]),
        float(eps_global_3x3[1, 1]),
        float(eps_global_3x3[2, 2]),
        float(2.0 * eps_global_3x3[0, 1]),
        float(2.0 * eps_global_3x3[1, 2]),
        float(2.0 * eps_global_3x3[0, 2]),
    ]

    vm = compute_von_mises_3d(stress_voigt_global)
    sxx_glob = float(stress_voigt_global[0])

    return ElementResult(
        element_id=elem_id,
        element_type="TRIANGLE_3D",
        length_or_measure=area,
        axial_stress=sxx_glob,
        combined_stress=vm,
        von_mises_stress=vm,
        strain_tensor_voigt=strain_voigt_global,
        stress_tensor_voigt=[float(x) for x in stress_voigt_global],
    )


# =============================================================================
# 4. ELEMENTO FINITO SÓLIDO TETRAÉDRICO LINEAL DE 4 NODOS (TETRA_4)
# =============================================================================

def isotropic_3d_constitutive_matrix(E: float, nu: float) -> np.ndarray:
    """
    Construye la matriz constitutiva elástica lineal isotrópica 3D D (6x6)
    en orden de Voigt: [exx, eyy, ezz, gxy, gyz, gxz].
    """
    if nu <= -1.0 or nu >= 0.5:
        raise ValueError(f"Coeficiente de Poisson inválido para elasticidad 3D (nu={nu}). Debe cumplir -1 < nu < 0.5.")
    factor = E / ((1.0 + nu) * (1.0 - 2.0 * nu))
    c1 = 1.0 - nu
    c2 = 0.5 * (1.0 - 2.0 * nu)
    D = factor * np.array(
        [
            [c1, nu, nu, 0.0, 0.0, 0.0],
            [nu, c1, nu, 0.0, 0.0, 0.0],
            [nu, nu, c1, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, c2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, c2, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, c2],
        ],
        dtype=float,
    )
    return D


def tetra4_stiffness_global(
    n1: Node3D, n2: Node3D, n3: Node3D, n4: Node3D, mat: Material
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Calcula la matriz de rigidez 12x12 (3 GDL traslacionales por cada uno de los 4 nodos),
    la matriz deformación-desplazamiento B (6x12), la matriz constitutiva D (6x6) y el volumen V.
    """
    coords = np.array([n1.coords(), n2.coords(), n3.coords(), n4.coords()], dtype=float)
    # Matriz jacobiana afín 4x4: [1, x_i, y_i, z_i]
    M = np.ones((4, 4), dtype=float)
    M[:, 1:4] = coords
    det_M = float(np.linalg.det(M))
    volume = abs(det_M) / 6.0
    if volume <= 1e-14:
        raise ValueError(
            f"Elemento sólido TETRA_4 ({n1.id}, {n2.id}, {n3.id}, {n4.id}) es degenerado (volumen casi nulo V={volume:.3e})."
        )

    # Gradientes de las funciones de forma N_i(x, y, z) = a_i + b_i*x + c_i*y + d_i*z
    # La inversa de M tiene en sus filas 1, 2, 3 los coeficientes [b_i, c_i, d_i]
    inv_M = np.linalg.inv(M)
    b = inv_M[1, :]  # dN_i / dx
    c = inv_M[2, :]  # dN_i / dy
    d = inv_M[3, :]  # dN_i / dz

    B = np.zeros((6, 12), dtype=float)
    for i in range(4):
        col = 3 * i
        bi, ci, di = b[i], c[i], d[i]
        # exx = du/dx
        B[0, col] = bi
        # eyy = dv/dy
        B[1, col + 1] = ci
        # ezz = dw/dz
        B[2, col + 2] = di
        # gxy = du/dy + dv/dx
        B[3, col] = ci
        B[3, col + 1] = bi
        # gyz = dv/dz + dw/dy
        B[4, col + 1] = di
        B[4, col + 2] = ci
        # gxz = du/dz + dw/dx
        B[5, col] = di
        B[5, col + 2] = bi

    D = isotropic_3d_constitutive_matrix(mat.E, mat.nu)
    K_elem = volume * (B.T @ D @ B)
    return K_elem, B, D, volume


def tetra4_recover_results(
    elem_id: int,
    n1: Node3D,
    n2: Node3D,
    n3: Node3D,
    n4: Node3D,
    mat: Material,
    u_trans_12: np.ndarray,
) -> ElementResult:
    """Recupera el tensor de deformaciones, tensor de tensiones de Cauchy y tensión de Von Mises en TETRA_4."""
    _, B, D, volume = tetra4_stiffness_global(n1, n2, n3, n4, mat)
    strain = B @ u_trans_12
    stress = D @ strain
    vm = compute_von_mises_3d(stress)
    sxx = float(stress[0])

    return ElementResult(
        element_id=elem_id,
        element_type="TETRA_4",
        length_or_measure=volume,
        axial_stress=sxx,
        combined_stress=vm,
        von_mises_stress=vm,
        strain_tensor_voigt=[float(x) for x in strain],
        stress_tensor_voigt=[float(x) for x in stress],
    )
