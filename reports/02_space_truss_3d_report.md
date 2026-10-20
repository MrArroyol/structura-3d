# Informe Técnico de Análisis Estructural — 02_Celosia_Espacial_Tripode_3D

**Fecha de generación:** 2026-10-20 10:26:22  
**Estado del cálculo:** EXITOSO (Convergente y Estable)

> **AVISO IMPORTANTE (RESTRICCIÓN DE USO):**  
> ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural.

---

## 1. Descripción del Modelo y Sistema de Unidades

- **Descripción:** Estructura espacial de 3 barras articuladas (trípode 3D) sometida a carga combinada en su cúspide.
- **Nodos totales:** 4
- **Elementos totales:** 3
- **Grados de libertad (GDL):** 24 totales (3 activos libres, 9 coaccionados por apoyos)
- **Sistema de unidades:** Longitud `m`, Fuerza `N`, Tensión `Pa`, Momento `N·m`, Giro `rad`

## 2. Diagnóstico Numérico y Verificación de Equilibrio

| Parámetro de Diagnóstico | Valor | Estado |
| :--- | :--- | :--- |
| Validez topológica y constitutiva | `True` | OK |
| Singularidad / Mecanismo cinemático | `False` / `False` | ESTABLE |
| Número de condición escalado ($\kappa$) | `1.0658e+00` | Óptimo |
| Modos de energía nula detectados | `0` | 0 (Correcto) |
| Residuo global de fuerzas $\|\sum \mathbf{F} + \sum \mathbf{R}\|$ | `2.3009e-11 N` | Equilibrio Exacto |
| Residuo global de momentos $\|\sum \mathbf{M}_0\|$ | `3.2539e-11 N·m` | Equilibrio Exacto |
| Energía de deformación elástica ($U$) | `3.243886e+01 J` | Calculado |
| Escala visual de deformación recomendada | `555.22x` | Identificada |

## 3. Resumen de Valores Extremos

- **Desplazamiento nodal máximo ($\|\mathbf{u}\|_{\max}$):** `5.680710e-04 m` (`0.5681 mm`)
- **Tensión equivalente de Von Mises máxima ($\sigma_{vm,\max}$):** `3.819719e+07 Pa` (`38.197 MPa`)
- **Tensión axial máxima de tracción ($\sigma_{x,\max}$):** `-6.856605e+06 Pa` (`-6.857 MPa`)
- **Tensión axial mínima / compresión ($\sigma_{x,\min}$):** `-3.819719e+07 Pa` (`-38.197 MPa`)

## 4. Desplazamientos Nodales Calculados

| Nodo ID | Coord $(x, y, z)$ [m] | $u_x$ [m] | $u_y$ [m] | $u_z$ [m] | $\theta_x$ [rad] | $\theta_y$ [rad] | $\theta_z$ [rad] | $\|\mathbf{u}\|$ [m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** (Base Izq) | (-1.500, 0.000, -1.000) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` |
| **2** (Base Der) | (1.500, 0.000, -1.000) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` |
| **3** (Base Frontal) | (0.000, 0.000, 1.500) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` |
| **4** (Cúspide) | (0.000, 2.000, 0.000) | `3.15621e-04` | `-4.40409e-04` | `1.70669e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `5.68071e-04` |

## 5. Reacciones en los Apoyos

| Nodo ID | $R_x$ [N] | $R_y$ [N] | $R_z$ [N] | $M_x$ [N·m] | $M_y$ [N·m] | $M_z$ [N·m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `7500.0000` | `10000.0000` | `5000.0000` | `0.0000` | `0.0000` | `0.0000` |
| **2** | `-37500.0000` | `50000.0000` | `25000.0000` | `0.0000` | `0.0000` | `0.0000` |
| **3** | `0.0000` | `60000.0000` | `-45000.0000` | `0.0000` | `0.0000` | `0.0000` |

## 6. Esfuerzos Internos y Tensiones por Elemento

| Elem ID | Tipo | Conectividad | Medida ($L/A/V$) | Axial $N$ [N] | Cortante $\max(V_y, V_z)$ [N] | Momento $\max(M_y, M_z)$ [N·m] | $\sigma_{axial}$ [MPa] | $\sigma_{VonMises}$ [MPa] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `TRUSS_3D` | [1, 4] | `2.6926` | `-13462.912` | `0.000` | `0.000` | `-6.857` | `6.857` |
| **2** | `TRUSS_3D` | [2, 4] | `2.6926` | `-67314.560` | `0.000` | `0.000` | `-34.283` | `34.283` |
| **3** | `TRUSS_3D` | [3, 4] | `2.5000` | `-75000.000` | `0.000` | `0.000` | `-38.197` | `38.197` |

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
