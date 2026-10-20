# Informe Técnico de Análisis Estructural — 04_Viga_Biapoyada_Carga_Distribuida_N4

**Fecha de generación:** 2026-10-20 10:26:22  
**Estado del cálculo:** EXITOSO (Convergente y Estable)

> **AVISO IMPORTANTE (RESTRICCIÓN DE USO):**  
> ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural.

---

## 1. Descripción del Modelo y Sistema de Unidades

- **Descripción:** Viga simplemente apoyada (L=6m) bajo carga uniforme q=-15 kN/m (4 elementos).
- **Nodos totales:** 5
- **Elementos totales:** 4
- **Grados de libertad (GDL):** 30 totales (21 activos libres, 9 coaccionados por apoyos)
- **Sistema de unidades:** Longitud `m`, Fuerza `N`, Tensión `Pa`, Momento `N·m`, Giro `rad`

## 2. Diagnóstico Numérico y Verificación de Equilibrio

| Parámetro de Diagnóstico | Valor | Estado |
| :--- | :--- | :--- |
| Validez topológica y constitutiva | `True` | OK |
| Singularidad / Mecanismo cinemático | `False` / `False` | ESTABLE |
| Número de condición escalado ($\kappa$) | `1.5131e+02` | Óptimo |
| Modos de energía nula detectados | `0` | 0 (Correcto) |
| Residuo global de fuerzas $\|\sum \mathbf{F} + \sum \mathbf{R}\|$ | `2.3283e-10 N` | Equilibrio Exacto |
| Residuo global de momentos $\|\sum \mathbf{M}_0\|$ | `1.0477e-09 N·m` | Equilibrio Exacto |
| Energía de deformación elástica ($U$) | `3.252345e+01 J` | Calculado |
| Escala visual de deformación recomendada | `159.29x` | Identificada |

## 3. Resumen de Valores Extremos

- **Desplazamiento nodal máximo ($\|\mathbf{u}\|_{\max}$):** `1.130022e-03 m` (`1.1300 mm`)
- **Tensión equivalente de Von Mises máxima ($\sigma_{vm,\max}$):** `1.265625e+07 Pa` (`12.656 MPa`)
- **Tensión axial máxima de tracción ($\sigma_{x,\max}$):** `0.000000e+00 Pa` (`0.000 MPa`)
- **Tensión axial mínima / compresión ($\sigma_{x,\min}$):** `0.000000e+00 Pa` (`0.000 MPa`)

## 4. Desplazamientos Nodales Calculados

| Nodo ID | Coord $(x, y, z)$ [m] | $u_x$ [m] | $u_y$ [m] | $u_z$ [m] | $\theta_x$ [rad] | $\theta_y$ [rad] | $\theta_z$ [rad] | $\|\mathbf{u}\|$ [m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** (N1) | (0.000, 0.000, 0.000) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `-6.02679e-04` | `0.00000e+00` |
| **2** (N2) | (1.500, 0.000, 0.000) | `0.00000e+00` | `-8.05141e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `-4.14342e-04` | `8.05141e-04` |
| **3** (N3) | (3.000, 0.000, 0.000) | `0.00000e+00` | `-1.13002e-03` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `3.14070e-20` | `1.13002e-03` |
| **4** (N4) | (4.500, 0.000, 0.000) | `-0.00000e+00` | `-8.05141e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `4.14342e-04` | `8.05141e-04` |
| **5** (N5) | (6.000, 0.000, 0.000) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `6.02679e-04` | `0.00000e+00` |

## 5. Reacciones en los Apoyos

| Nodo ID | $R_x$ [N] | $R_y$ [N] | $R_z$ [N] | $M_x$ [N·m] | $M_y$ [N·m] | $M_z$ [N·m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `0.0000` | `45000.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **5** | `0.0000` | `45000.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |

## 6. Esfuerzos Internos y Tensiones por Elemento

| Elem ID | Tipo | Conectividad | Medida ($L/A/V$) | Axial $N$ [N] | Cortante $\max(V_y, V_z)$ [N] | Momento $\max(M_y, M_z)$ [N·m] | $\sigma_{axial}$ [MPa] | $\sigma_{VonMises}$ [MPa] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `FRAME_3D` | [1, 2] | `1.5000` | `0.000` | `45000.000` | `50625.000` | `0.000` | `9.492` |
| **2** | `FRAME_3D` | [2, 3] | `1.5000` | `0.000` | `22500.000` | `67500.000` | `0.000` | `12.656` |
| **3** | `FRAME_3D` | [3, 4] | `1.5000` | `0.000` | `22500.000` | `67500.000` | `0.000` | `12.656` |
| **4** | `FRAME_3D` | [4, 5] | `1.5000` | `0.000` | `45000.000` | `50625.000` | `0.000` | `9.492` |

## 7. Hipótesis del Modelo, Limitaciones y Modos de Fallo No Analizados

1. **Cinemática Lineal Infinitesimal:** Se asumen pequeños desplazamientos y pequeñas deformaciones ($\|\nabla \mathbf{u}\| \ll 1$). El equilibrio se plantea sobre la configuración indeformada original (teoría de primer orden). No se consideran efectos $P-\Delta$ ni grandes rotaciones.
2. **Material Elástico Lineal, Homogéneo e Isotrópico:** Se aplica la ley de Hooke generalizada sin plastificación, endurecimiento, fluencia viscosa (creep), daño ni fisuración.
3. **Formulaciones de Elementos:**
   - `TRUSS_3D`: Barras biarticuladas con rigidez exclusivamente axial ($EA/L$).
   - `FRAME_3D`: Vigas de Euler-Bernoulli sin deformación por cortante transversal de Timoshenko ni alabeo torsional restringido (Vlasov).
   - `TRIANGLE_3D` y `TETRA_4`: Elementos finitos lineales de deformación constante (pueden presentar bloqueo por cortante o rigidez artificial en flexión dominante con mallas gruesas).
4. **Modos de Fallo NO Analizados:**
   - Pandeo global por compresión (Euler), pandeo lateral-torsional ni abolladura local de chapas.
   - Plastificación de secciones, rótulas plásticas ni redistribución no lineal de esfuerzos.
   - Fallo frágil, fractura, fatiga cíclica, impacto dinámico o resonancia sísmica.
   - Deslizamiento o rotura de uniones (soldaduras, pernos, placas base) ni interacción suelo-estructura.

---
*ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural.*
