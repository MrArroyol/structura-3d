"""
Interfaz de Línea de Comandos (CLI) y Puente Local para Godot 4 — Structura 3D.

Permite:
- Resolver un archivo `.structura.json` y exportar `.result.json` y reportes `.md`/`.html`.
- Generar todos los casos de referencia en `benchmarks/` y sus informes en `reports/`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .model import StructuralModel
from .engine import solve_structure
from .benchmarks import get_all_reference_models, export_all_benchmarks
from .report import export_reports, generate_markdown_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="structura-solver",
        description="Motor numérico local de análisis estructural 3D (Structura 3D).",
    )
    parser.add_argument(
        "--input",
        "-i",
        type=str,
        help="Ruta al archivo de entrada (.structura.json) a resolver.",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        help="Ruta al archivo JSON de salida (.result.json).",
    )
    parser.add_argument(
        "--report-dir",
        "-r",
        type=str,
        help="Directorio donde exportar los informes técnicos (.md, .html, .result.json).",
    )
    parser.add_argument(
        "--export-benchmarks",
        type=str,
        help="Exporta todos los modelos de referencia al directorio indicado.",
    )
    parser.add_argument(
        "--run-all-benchmarks",
        action="store_true",
        help="Resuelve todos los casos de referencia y muestra un resumen de verificación.",
    )

    args = parser.parse_args(argv)

    if args.export_benchmarks:
        paths = export_all_benchmarks(args.export_benchmarks)
        print(f"Exportados {len(paths)} modelos de referencia en '{args.export_benchmarks}'.")

    if args.run_all_benchmarks:
        models = get_all_reference_models()
        bench_dir = Path(args.export_benchmarks or "benchmarks")
        rep_dir = Path(args.report_dir or "reports")
        export_all_benchmarks(bench_dir)
        print("=" * 84)
        print("VERIFICACIÓN DE CASOS DE REFERENCIA — STRUCTURA 3D")
        print("=" * 84)
        for key, model in models.items():
            res = solve_structure(model)
            export_reports(model, res, rep_dir, base_name=key)
            status = "OK" if res.success else "FALLO CONTROLADO (ESPERADO)"
            print(
                f"[{status:27s}] {key:26s} | u_max={res.max_displacement:.4e} m | "
                f"VM_max={res.max_von_mises/1e6:.2f} MPa | eq={res.diagnostics.equilibrium_satisfied}"
            )
        print("=" * 84)
        return 0

    if args.input:
        model = StructuralModel.load(args.input)
        result = solve_structure(model)
        if args.output:
            out_path = Path(args.output)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(result.to_json(indent=2), encoding="utf-8")
        if args.report_dir:
            export_reports(model, result, args.report_dir)
        if not args.output and not args.report_dir:
            print(generate_markdown_report(model, result))
        return 0 if result.success else 2

    if not args.export_benchmarks:
        parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
