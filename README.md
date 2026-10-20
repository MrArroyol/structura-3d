# Structura 3D — Entorno Interactivo de Análisis Estructural en 3D

> **ADVERTENCIA TÉCNICA:** Los resultados numéricos obtenidos mediante **Structura 3D** tienen fines exclusivamente académicos, educativos y conceptuales. **NO son aptos para construcción ni constituyen certificación estructural.**

---

## Estructura del Proyecto

- [`project.godot`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/project.godot): Proyecto principal de **Godot 4** con interfaz 3D interactiva.
- [`scenes/Main.tscn`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scenes/Main.tscn): Escena 3D principal.
- `scripts/`:
  - [`scripts/MainController.gd`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scripts/MainController.gd): Editor geométrico 3D interactivo, selector de casos de referencia, control de escala visual de deformación, inspector de resultados y exportador de informes.
  - [`scripts/Visualizer3D.gd`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scripts/Visualizer3D.gd): Renderizado 3D de geometría original y deformada, numeración de nodos/elementos, apoyos, flechas de cargas/reacciones y mapas de color (Von Mises, desplazamientos y tensión axial).
  - [`scripts/OrbitCamera3D.gd`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scripts/OrbitCamera3D.gd): Cámara orbital 3D (clic derecho/central para orbitar, `Shift` + arrastrar para desplazar, rueda para zoom, `R` para centrar).
  - [`scripts/solver/SolverBridge.gd`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scripts/solver/SolverBridge.gd) y [`scripts/solver/NativeSolver.gd`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/scripts/solver/NativeSolver.gd): Puente al motor numérico local en Python (`NumPy`/`SciPy`) y solucionador nativo en GDScript (100% sin IA en ejecución).
- `solver/`:
  - [`solver/model.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/model.py): Modelo de datos estructural y esquema `.structura.json`.
  - [`solver/elements.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/elements.py): Formulaciones matriciales 3D (`TRUSS_3D`, `FRAME_3D` Euler-Bernoulli, `TRIANGLE_3D` CST y `TETRA_4` sólido tetraédrico).
  - [`solver/engine.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/engine.py): Solucionador por el Método de Rigidez Directa ($\mathbf{K}\cdot\mathbf{u}=\mathbf{F}$), detección de singularidad/mecanismos y verificación de equilibrio de fuerzas y momentos.
  - [`solver/benchmarks.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/benchmarks.py): 9 casos de referencia analíticos y de detección de fallos.
  - [`solver/report.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/report.py): Exportador de informes técnicos (`.md`, `.html`, `.result.json`).
  - [`solver/cli.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/solver/cli.py): Interfaz de línea de comandos del motor numérico.
- [`tests/test_solver.py`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/tests/test_solver.py): Suite de pruebas automatizadas con `pytest`.
- `benchmarks/`: Archivos de malla `.structura.json` de todos los casos de referencia.
- `reports/`: Informes técnicos generados en Markdown, HTML y JSON.
- [`docs/DOCUMENTACION_TECNICA.md`](file:///c:/Users/arroy/OneDrive/Documentos/proyecto/docs/DOCUMENTACION_TECNICA.md): Documentación matemática, hipótesis, limitaciones y modos de fallo no analizados.

---

## Ejecución Rápida

### 1. Ejecutar Pruebas Automatizadas
```powershell
py -m pytest -v
```

### 2. Resolver Todos los Casos de Referencia y Generar Informes
```powershell
py -m solver.cli --run-all-benchmarks
```

### 3. Resolver una Malla Específica desde CLI
```powershell
py -m solver.cli --input benchmarks/02_space_truss_3d.structura.json --report-dir reports
```

### 4. Abrir el Entorno 3D Interactivo en Godot 4
Abra el archivo `project.godot` desde **Godot Engine 4.x** y presione **F5 (Run Project)**.
