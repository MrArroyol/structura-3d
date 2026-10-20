# Documentación Técnica — Structura 3D

> [!CAUTION]
> **RESTRICCIÓN DE USO Y AVISO DE NO CERTIFICACIÓN:**  
> Los resultados generados por **Structura 3D** corresponden a simulaciones numéricas idealizadas con fines exclusivamente educativos, académicos y de exploración conceptual. **NO son aptos para construcción, diseño estructural ejecutivo ni constituyen certificación o aval estructural.**

---

## 1. Arquitectura General del Sistema

**Structura 3D** opera de manera **100% determinista y local (sin modelos de IA durante la ejecución)** mediante una arquitectura desacoplada en dos capas:

1. **Entorno Interactivo 3D en Godot 4 (`project.godot`, `scenes/Main.tscn`, `scripts/`):**
   - **Editor Geométrico 3D (`MainController.gd`):** Creación y edición interactiva de nodos 3D, conectividad de barras y elementos finitos, asignación de materiales isotrópicos, secciones transversales paramétricas, condiciones de apoyo y cargas (puntuales y distribuidas).
   - **Visor Estructural 3D (`Visualizer3D.gd` y `OrbitCamera3D.gd`):** Renderizado en tiempo real de la geometría original (alambre/semitransparente) y deformada, numeración de nodos y elementos (`Label3D`), vectores 3D de cargas y reacciones en apoyos, mapa cromático continuo de desplazamientos y tensiones, y control explícito de la **escala visual de deformación**.
   - **Puente de Cálculo (`SolverBridge.gd`) y Solucionador Nativo (`NativeSolver.gd`):** Permite ejecutar el módulo numérico local en Python (`NumPy`/`SciPy`) para máxima velocidad y diagnóstico espectral, o utilizar el motor nativo en GDScript sin dependencias externas.

2. **Motor Numérico Local y Generador de Informes (`solver/`):**
   - `solver/model.py`: Modelo de datos tipado y serialización del formato de malla `.structura.json`.
   - `solver/elements.py`: Formulaciones matriciales de elementos `TRUSS_3D`, `FRAME_3D`, `TRIANGLE_3D` (CST) y `TETRA_4`.
   - `solver/engine.py`: Ensamblaje por el Método de Rigidez Directa (DSM), partición de grados de libertad, precondicionamiento simétrico de Jacobi, detección de mecanismos/singularidades, resolución de $\mathbf{K}\cdot\mathbf{u}=\mathbf{F}$ y verificación de equilibrio estático global.
   - `solver/benchmarks.py`: Conjunto de 9 problemas de referencia con soluciones analíticas exactas y casos de fallo controlado.
   - `solver/report.py`: Exportación de informes técnicos en Markdown (`.md`), HTML (`.html`) y JSON (`.result.json`).

---

## 2. Modelo de Datos y Formato de Mallas (`.structura.json`)

El formato `.structura.json` almacena de forma explícita las unidades físicas y todas las entidades topológicas, mecánicas y de contorno:

- **`units`**: Sistema Internacional por defecto (`m`, `N`, `Pa`, `N·m`, `kg/m³`, `rad`).
- **`nodes`**: Lista de nudos con `id` entero único y coordenadas tridimensionales `(x, y, z)` en metros.
- **`materials`**: Propiedades elásticas lineales isotrópicas:
  - Módulo de Young $E > 0$ [Pa]
  - Coeficiente de Poisson $-1 < \nu < 0.5$ [-]
  - Módulo de cortante derivado $G = \frac{E}{2(1+\nu)}$ [Pa]
- **`sections`**: Propiedades geométricas de sección transversal:
  - Área $A$ [$\text{m}^2$], momentos de inercia $I_y, I_z$ [$\text{m}^4$], constante torsional de Saint-Venant $J$ [$\text{m}^4$] y espesor $t$ [$\text{m}$] para elementos planos.
- **`elements`**: Conectividad nodal ordenada según el tipo de elemento:
  - `TRUSS_3D`: 2 nodos $[n_1, n_2]$
  - `FRAME_3D`: 2 nodos $[n_1, n_2]$ + ángulo de giro axial `roll_angle` $\psi$ [rad]
  - `TRIANGLE_3D`: 3 nodos $[n_1, n_2, n_3]$ (orientación antihoraria define la normal local $+z'$)
  - `TETRA_4`: 4 nodos $[n_1, n_2, n_3, n_4]$ (volumen $V = \frac{1}{6}|\det \mathbf{J}| > 0$)
- **`boundary_conditions`**: Coacciones booleanas por cada uno de los 6 GDL del nodo $(u_x, u_y, u_z, \theta_x, \theta_y, \theta_z)$ y valores de desplazamiento/giro prescrito $(u_{x0}, \dots, \theta_{z0})$.
- **`nodal_loads`** y **`distributed_loads`**: Cargas puntuales nodales $(F_x, F_y, F_z, M_x, M_y, M_z)$ y cargas distribuidas uniformes por unidad de longitud $(q_x, q_y, q_z)$ en coordenadas globales o locales.

---

## 3. Formulación Matemática del Motor de Cálculo

### 3.1 Método de Rigidez Directa y Partición de Restricciones

Cada nodo $i \in \{1, \dots, N_{nodos}\}$ posee hasta 6 grados de libertad globales:

$$\mathbf{u}_i = \begin{bmatrix} u_{x,i} & u_{y,i} & u_{z,i} & \theta_{x,i} & \theta_{y,i} & \theta_{z,i} \end{bmatrix}^T$$

Tras ensamblar las matrices de rigidez elementales $\mathbf{K}^{(e)}$, el vector de fuerzas puntuales nodales $\mathbf{F}_{nodal}$ y el vector de fuerzas nodales equivalentes por cargas distribuidas $\mathbf{F}_{eq}$, el sistema global de equilibrio se particiona en grados de libertad **libres ($f$)** y **restringidos ($r$)**:

$$\begin{bmatrix} \mathbf{K}_{ff} & \mathbf{K}_{fr} \\ \mathbf{K}_{rf} & \mathbf{K}_{rr} \end{bmatrix} \begin{bmatrix} \mathbf{u}_f \\ \mathbf{u}_r \end{bmatrix} = \begin{bmatrix} \mathbf{F}_{nodal,f} + \mathbf{F}_{eq,f} \\ \mathbf{F}_{nodal,r} + \mathbf{F}_{eq,r} + \mathbf{R}_r \end{bmatrix}$$

Donde $\mathbf{u}_r$ contiene los desplazamientos prescritos en los apoyos ($\mathbf{u}_r = \mathbf{0}$ en apoyos fijos). El sub-sistema reducido para los desplazamientos incógnita $\mathbf{u}_f$ es:

$$\mathbf{K}_{ff} \mathbf{u}_f = (\mathbf{F}_{nodal,f} + \mathbf{F}_{eq,f}) - \mathbf{K}_{fr} \mathbf{u}_r$$

Una vez obtenido $\mathbf{u}_f$, las **reacciones exactas en los apoyos** se recuperan mediante:

$$\mathbf{R} = \mathbf{K}\mathbf{u} - (\mathbf{F}_{nodal} + \mathbf{F}_{eq})$$

### 3.2 Precondicionamiento de Jacobi y Detección de Singularidad / Mecanismos

Dado que las rigideces traslacionales ($EA/L \sim 10^8\text{ N/m}$) y flexionales ($EI/L \sim 10^6\text{ N}\cdot\text{m/rad}$) difieren en órdenes de magnitud y unidades físicas, el motor aplica un escalado simétrico diagonal de Jacobi antes de evaluar el espectro y resolver el sistema:

$$\tilde{\mathbf{K}}_{ff} = \mathbf{D}^{-1/2} \mathbf{K}_{ff} \mathbf{D}^{-1/2}, \quad \text{con } D_{ii} = (K_{ff})_{ii}$$

1. Si algún elemento diagonal $(K_{ff})_{ii} \le 10^{-14}$, existe un grado de libertad activo sin rigidez conexa (mecanismo local o nudo desarticulado).
2. Se calculan los autovalores reales $\lambda_1 \le \lambda_2 \le \dots \le \lambda_m$ de $\tilde{\mathbf{K}}_{ff}$:
   - El número de autovalores con $\lambda_k \le 10^{-10} \lambda_{\max}$ determina exactamente el número de **modos de energía nula (mecanismos cinemáticos internos o movimientos de sólido rígido por apoyos insuficientes)**.
   - El número de condición escalado $\kappa(\tilde{\mathbf{K}}_{ff}) = \frac{\lambda_{\max}}{\lambda_{\min}}$ detecta cuasi-mecanismos cuando $\kappa > 10^{12}$.

### 3.3 Evaluación de la Extensión a Elementos Triangulares (`TRIANGLE_3D`) y Sólidos Tetraédricos (`TETRA_4`)

- **`TRIANGLE_3D` (Membrana CST en 3D):** Proyecta los 3 nodos del triángulo sobre un sistema ortonormal local coplanar $(\hat{\mathbf{e}}_x', \hat{\mathbf{e}}_y', \hat{\mathbf{e}}_z')$ e integra el estado de **membrana en tensión plana** con matriz de deformación constante $\mathbf{B}_{2D}$ ($3 \times 6$):
  $$\mathbf{k}_{local} = t \, A \, \mathbf{B}_{2D}^T \mathbf{D}_{ps} \mathbf{B}_{2D}, \qquad \mathbf{K}_{global,9\times 9} = \mathbf{T}_{proj}^T \mathbf{k}_{local} \mathbf{T}_{proj}$$
  - **Comportamiento mecánico real y rango:** La matriz $\mathbf{K}_{global,9\times 9}$ tiene **rango exactamente 3** y **6 modos de energía nula** en 3D. **No posee rigidez transversal fuera de su plano ni rigidez a flexión de placa**, y no introduce rigidez artificial (*drilling/out-of-plane*) que oculte mecanismos. Si se requiere analizar flexión transversal de placas o láminas, debe emplearse una formulación específica de placa/cascarón (Kirchhoff, DKT o Mindlin-Reissner).
  - **Convención global de tensores (invariancia ante permutación nodal):** Las deformaciones y tensiones planas calculadas en la base local $(\hat{\mathbf{e}}_x', \hat{\mathbf{e}}_y', \hat{\mathbf{e}}_z')$ se transforman de vuelta al sistema de coordenadas global $(X, Y, Z)$ mediante $\boldsymbol{\sigma}_{glob} = \mathbf{R}^T \boldsymbol{\sigma}_{loc} \mathbf{R}$ y $\boldsymbol{\varepsilon}_{glob} = \mathbf{R}^T \boldsymbol{\varepsilon}_{loc} \mathbf{R}$, exportándose en orden de Voigt global:
    $$\text{stress\_tensor\_voigt} = [\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{xz}], \quad \text{strain\_tensor\_voigt} = [\varepsilon_{xx}, \varepsilon_{yy}, \varepsilon_{zz}, \gamma_{xy}, \gamma_{yz}, \gamma_{xz}]$$
- **`TETRA_4` (Tetraedro Sólido Lineal de 4 Nodos):** Utiliza funciones de forma lineales $N_i(x,y,z) = a_i + b_i x + c_i y + d_i z$ ($i=1\dots 4$) obtenidas invirtiendo la matriz jacobiana afín $4 \times 4$. Su matriz de rigidez elemental ($12 \times 12$) es exacta con un único punto de Gauss:
  $$\mathbf{K}^{(e)} = V \, \mathbf{B}^T \mathbf{D}_{3D} \mathbf{B}, \quad V = \frac{1}{6} |\det \mathbf{M}|$$
  - **Evaluación crítica:** Los elementos lineales de deformación constante (`TRIANGLE_3D` y `TETRA_4`) superan de forma exacta el *Patch Test* de tracción/compresión y cortante uniforme (error $< 10^{-12}$) y garantizan convergencia monótona al refinar la malla, pero presentan rigidez artificial (*shear locking*) en problemas dominados por flexión esbelta si se emplea una malla gruesa, y no deben utilizarse con $\nu \to 0.5$ por bloqueo volumétrico (*volumetric locking*). Para vigas y pórticos esbeltos debe preferirse el elemento `FRAME_3D`.

### 3.4 Política de Validación, Condiciones de Apoyo y Nodos Huérfanos
- **Rechazo de entradas no finitas (`NaN`/`Inf`) y espesores no positivos:** Se rechaza previo al ensamblaje cualquier modelo con `NaN` o `Inf` en coordenadas, propiedades de material o sección, ángulos `roll_angle`, cargas puntuales, cargas distribuidas o desplazamientos prescritos, así como secciones con `thickness <= 0` en elementos `TRIANGLE_3D`.
- **Conteo único y compatibilidad de apoyos:** Cada grado de libertad restringido por el usuario se contabiliza una sola vez en `constrained_dofs`. Si dos condiciones de frontera imponen desplazamientos prescritos distintos sobre el mismo GDL, el modelo se rechaza por incompatibilidad cinemática.
- **Política para nodos huérfanos (desconectados):** Si un nodo sin elementos conectados recibe cargas externas o desplazamientos prescritos no nulos, el solucionador falla explícitamente reportando un mecanismo cinemático (`is_valid=False`, `is_singular=True`, `is_mechanism=True`). Si el nodo huérfano es puramente pasivo (sin cargas ni desplazamientos prescritos), se registra una advertencia explícita en `diagnostics.warnings` y se excluyen sus 6 GDL inactivos.

---

## 4. Limitaciones del Modelo, Hipótesis de Material y Modos de Fallo NO Analizados

1. **Hipótesis de Pequeños Desplazamientos (Cinemática Lineal):**
   - La matriz de rigidez se evalúa únicamente en la geometría original indeformada.
   - **No se analizan** efectos geométricos de segundo orden ($P-\Delta$, $P-\delta$), rigidización por tensión (*stress stiffening*) ni inestabilidad por salto (*snap-through*).
2. **Hipótesis Constitutiva Elástica Lineal Isotrópica:**
   - Se asume proporcionalidad indefinida entre tensión y deformación ($\boldsymbol{\sigma} = \mathbf{D}\boldsymbol{\varepsilon}$).
   - **No se analizan** plastificación del material, formación de rótulas plásticas, endurecimiento por deformación, fisuración del hormigón, fluencia (*creep*) ni anisotropía/ortotropía.
3. **Modos de Fallo Estructural NO Contemplados:**
   - **Pandeo:** No se verifica pandeo flexional de Euler en barras comprimidas, pandeo por torsión o flexo-torsión, pandeo lateral-torsional de vigas ni abolladura local de almas/alas.
   - **Deformación por Cortante en Vigas:** El elemento `FRAME_3D` sigue la hipótesis de Euler-Bernoulli (secciones planas permanecen normales al eje deformado), por lo que subestima desplazamientos en vigas de gran canto ($L/h < 5$) donde domina la teoría de Timoshenko.
   - **Flexión Transversal de Placas/Láminas:** El elemento `TRIANGLE_3D` modela exclusivamente membrana en tensión plana (CST) y carece de rigidez a flexión fuera de su plano.
   - **Fatiga, Fractura y Dinámica:** No se evalúa propagación de grietas, fatiga por cargas cíclicas, impacto, vibraciones libres/forzadas ni respuesta sísmica modal/temporal.
   - **Uniones y Concentradores de Tensión:** Los nudos se modelan como puntos matemáticos ideales (perfectamente rígidos en `FRAME_3D` o perfectamente articulados en `TRUSS_3D`) sin modelar soldaduras, tornillos, placas base ni plastificación local.
