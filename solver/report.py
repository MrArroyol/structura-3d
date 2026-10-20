"""
Generador de Informes Técnicos Estructurales (Markdown, HTML y JSON) — Structura 3D.

Exporta informes completos de análisis con:
- Aviso explícito de prohibición de uso para construcción o certificación estructural.
- Hipótesis del modelo numérico, limitaciones y modos de fallo no contemplados.
- Resumen del modelo geométrico, malla, materiales y secciones con unidades SI.
- Diagnósticos numéricos (rango, número de condición, equilibrio global de fuerzas y momentos).
- Tablas detalladas de desplazamientos nodales, reacciones en los apoyos y esfuerzos elementales.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Tuple

from .model import StructuralModel, SolverResult, DISCLAIMER_TEXT


def generate_markdown_report(model: StructuralModel, result: SolverResult) -> str:
    """Genera un informe técnico en formato Markdown con tablas numéricas y unidades explícitas."""
    u = model.units
    diag = result.diagnostics
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    lines = [
        f"# Informe Técnico de Análisis Estructural — {model.name}",
        "",
        f"**Fecha de generación:** {now_str}  ",
        f"**Estado del cálculo:** {'EXITOSO (Convergente y Estable)' if result.success else 'FALLIDO / INESTABLE'}",
        "",
        "> **AVISO IMPORTANTE (RESTRICCIÓN DE USO):**  ",
        f"> {DISCLAIMER_TEXT}",
        "",
        "---",
        "",
        "## 1. Descripción del Modelo y Sistema de Unidades",
        "",
        f"- **Descripción:** {model.description or 'Sin descripción adicional.'}",
        f"- **Nodos totales:** {len(model.nodes)}",
        f"- **Elementos totales:** {len(model.elements)}",
        f"- **Grados de libertad (GDL):** {diag.total_dofs} totales ({diag.free_dofs} activos libres, {diag.constrained_dofs} coaccionados por apoyos)",
        f"- **Sistema de unidades:** Longitud `{u.length}`, Fuerza `{u.force}`, Tensión `{u.stress}`, Momento `{u.moment}`, Giro `{u.angle}`",
        "",
        "## 2. Diagnóstico Numérico y Verificación de Equilibrio",
        "",
        "| Parámetro de Diagnóstico | Valor | Estado |",
        "| :--- | :--- | :--- |",
        f"| Validez topológica y constitutiva | `{diag.is_valid}` | {'OK' if diag.is_valid else 'ERROR'} |",
        f"| Singularidad / Mecanismo cinemático | `{diag.is_singular}` / `{diag.is_mechanism}` | {'ESTABLE' if not diag.is_singular else 'INESTABLE'} |",
        f"| Número de condición escalado ($\\kappa$) | `{diag.condition_number:.4e}` | {'Óptimo' if diag.condition_number < 1e8 else 'Elevado'} |",
        f"| Modos de energía nula detectados | `{diag.zero_energy_modes}` | {'0 (Correcto)' if diag.zero_energy_modes == 0 else 'Mecanismo'} |",
        f"| Residuo global de fuerzas $\\|\\sum \\mathbf{{F}} + \\sum \\mathbf{{R}}\\|$ | `{diag.force_residual_norm:.4e} {u.force}` | {'Equilibrio Exacto' if diag.equilibrium_satisfied else 'Desviación'} |",
        f"| Residuo global de momentos $\\|\\sum \\mathbf{{M}}_0\\|$ | `{diag.moment_residual_norm:.4e} {u.moment}` | {'Equilibrio Exacto' if diag.equilibrium_satisfied else 'Desviación'} |",
        f"| Energía de deformación elástica ($U$) | `{result.strain_energy:.6e} J` | Calculado |",
        f"| Escala visual de deformación recomendada | `{result.recommended_visual_scale:.2f}x` | Identificada |",
        "",
    ]

    if diag.errors:
        lines.append("### Errores Detectados")
        for err in diag.errors:
            lines.append(f"- **ERROR:** {err}")
        lines.append("")

    if diag.warnings:
        lines.append("### Advertencias del Solucionador")
        for warn in diag.warnings:
            lines.append(f"- **ADVERTENCIA:** {warn}")
        lines.append("")

    if result.success:
        lines.extend([
            "## 3. Resumen de Valores Extremos",
            "",
            f"- **Desplazamiento nodal máximo ($\\|\\mathbf{{u}}\\|_{{\\max}}$):** `{result.max_displacement:.6e} {u.length}` (`{result.max_displacement * 1000.0:.4f} mm`)",
            f"- **Tensión equivalente de Von Mises máxima ($\\sigma_{{vm,\\max}}$):** `{result.max_von_mises:.6e} {u.stress}` (`{result.max_von_mises / 1e6:.3f} MPa`)",
            f"- **Tensión axial máxima de tracción ($\\sigma_{{x,\\max}}$):** `{result.max_axial_stress:.6e} {u.stress}` (`{result.max_axial_stress / 1e6:.3f} MPa`)",
            f"- **Tensión axial mínima / compresión ($\\sigma_{{x,\\min}}$):** `{result.min_axial_stress:.6e} {u.stress}` (`{result.min_axial_stress / 1e6:.3f} MPa`)",
            "",
            "## 4. Desplazamientos Nodales Calculados",
            "",
            f"| Nodo ID | Coord $(x, y, z)$ [{u.length}] | $u_x$ [{u.length}] | $u_y$ [{u.length}] | $u_z$ [{u.length}] | $\\theta_x$ [{u.angle}] | $\\theta_y$ [{u.angle}] | $\\theta_z$ [{u.angle}] | $\\|\\mathbf{{u}}\\|$ [{u.length}] |",
            "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        ])

        for n in model.nodes:
            disp = result.nodal_displacements.get(n.id, [0.0] * 6)
            umag = (disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2) ** 0.5
            lines.append(
                f"| **{n.id}** ({n.label or '-'}) | ({n.x:.3f}, {n.y:.3f}, {n.z:.3f}) | "
                f"`{disp[0]:.5e}` | `{disp[1]:.5e}` | `{disp[2]:.5e}` | "
                f"`{disp[3]:.5e}` | `{disp[4]:.5e}` | `{disp[5]:.5e}` | `{umag:.5e}` |"
            )

        lines.extend([
            "",
            "## 5. Reacciones en los Apoyos",
            "",
            f"| Nodo ID | $R_x$ [{u.force}] | $R_y$ [{u.force}] | $R_z$ [{u.force}] | $M_x$ [{u.moment}] | $M_y$ [{u.moment}] | $M_z$ [{u.moment}] |",
            "| :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        ])

        bc_node_ids = {bc.node_id for bc in model.boundary_conditions}
        for nid in sorted(bc_node_ids):
            reac = result.nodal_reactions.get(nid, [0.0] * 6)
            lines.append(
                f"| **{nid}** | `{reac[0]:.4f}` | `{reac[1]:.4f}` | `{reac[2]:.4f}` | "
                f"`{reac[3]:.4f}` | `{reac[4]:.4f}` | `{reac[5]:.4f}` |"
            )

        lines.extend([
            "",
            "## 6. Esfuerzos Internos y Tensiones por Elemento",
            "",
            f"| Elem ID | Tipo | Conectividad | Medida ($L/A/V$) | Axial $N$ [{u.force}] | Cortante $\\max(V_y, V_z)$ [{u.force}] | Momento $\\max(M_y, M_z)$ [{u.moment}] | $\\sigma_{{axial}}$ [MPa] | $\\sigma_{{VonMises}}$ [MPa] |",
            "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        ])

        for elem in model.elements:
            er = result.element_results.get(elem.id)
            if not er:
                continue
            v_max = max(abs(er.shear_y), abs(er.shear_z))
            m_max = max(abs(er.moment_y), abs(er.moment_z))
            lines.append(
                f"| **{elem.id}** | `{elem.type}` | {elem.nodes} | `{er.length_or_measure:.4f}` | "
                f"`{er.axial_force:.3f}` | `{v_max:.3f}` | `{m_max:.3f}` | "
                f"`{er.axial_stress / 1e6:.3f}` | `{er.von_mises_stress / 1e6:.3f}` |"
            )

    lines.extend([
        "",
        "## 7. Hipótesis del Modelo, Limitaciones y Modos de Fallo No Analizados",
        "",
        "1. **Cinemática Lineal Infinitesimal:** Se asumen pequeños desplazamientos y pequeñas deformaciones ($\\|\\nabla \\mathbf{u}\\| \\ll 1$). El equilibrio se plantea sobre la configuración indeformada original (teoría de primer orden). No se consideran efectos $P-\\Delta$ ni grandes rotaciones.",
        "2. **Material Elástico Lineal, Homogéneo e Isotrópico:** Se aplica la ley de Hooke generalizada sin plastificación, endurecimiento, fluencia viscosa (creep), daño ni fisuración.",
        "3. **Formulaciones de Elementos:**",
        "   - `TRUSS_3D`: Barras biarticuladas con rigidez exclusivamente axial ($EA/L$).",
        "   - `FRAME_3D`: Vigas de Euler-Bernoulli sin deformación por cortante transversal de Timoshenko ni alabeo torsional restringido (Vlasov).",
        "   - `TRIANGLE_3D` y `TETRA_4`: Elementos finitos lineales de deformación constante (pueden presentar bloqueo por cortante o rigidez artificial en flexión dominante con mallas gruesas).",
        "4. **Modos de Fallo NO Analizados:**",
        "   - Pandeo global por compresión (Euler), pandeo lateral-torsional ni abolladura local de chapas.",
        "   - Plastificación de secciones, rótulas plásticas ni redistribución no lineal de esfuerzos.",
        "   - Fallo frágil, fractura, fatiga cíclica, impacto dinámico o resonancia sísmica.",
        "   - Deslizamiento o rotura de uniones (soldaduras, pernos, placas base) ni interacción suelo-estructura.",
        "",
        "---",
        f"*{DISCLAIMER_TEXT}*",
        "",
    ])

    return "\n".join(lines)


def generate_html_report(model: StructuralModel, result: SolverResult) -> str:
    """Genera una versión HTML autocontenida del informe técnico."""
    md = generate_markdown_report(model, result)
    escaped = (
        md.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )
    return f"""<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <title>Informe Estructural — {model.name}</title>
  <style>
    body {{
      font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
      background: #0f172a;
      color: #e2e8f0;
      margin: 2rem auto;
      max-width: 1080px;
      padding: 0 1.5rem;
      line-height: 1.55;
    }}
    .banner {{
      background: #451a03;
      border-left: 5px solid #f59e0b;
      padding: 1rem 1.25rem;
      border-radius: 6px;
      margin-bottom: 1.5rem;
      color: #fef3c7;
      font-weight: 600;
    }}
    pre {{
      background: #1e293b;
      border: 1px solid #334155;
      border-radius: 8px;
      padding: 1.25rem;
      overflow-x: auto;
      white-space: pre-wrap;
      font-family: ui-monospace, SFMono-Regular, Consolas, monospace;
      font-size: 0.9rem;
    }}
  </style>
</head>
<body>
  <div class="banner">{DISCLAIMER_TEXT}</div>
  <pre>{escaped}</pre>
</body>
</html>
"""


def export_reports(
    model: StructuralModel, result: SolverResult, output_dir: str | Path, base_name: str = ""
) -> Tuple[Path, Path, Path]:
    """Exporta los informes en Markdown (.md), HTML (.html) y resultados completos (.result.json)."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    slug = base_name or model.name.lower().replace(" ", "_")

    md_path = out / f"{slug}_report.md"
    html_path = out / f"{slug}_report.html"
    json_path = out / f"{slug}.result.json"

    md_path.write_text(generate_markdown_report(model, result), encoding="utf-8")
    html_path.write_text(generate_html_report(model, result), encoding="utf-8")
    json_path.write_text(result.to_json(indent=2), encoding="utf-8")

    return md_path, html_path, json_path
