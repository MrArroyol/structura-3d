# Informe Técnico de Análisis Estructural — 07_Solido_Tetraedrico_PatchTest_3D

**Fecha de generación:** 2026-10-20 10:26:22  
**Estado del cálculo:** EXITOSO (Convergente y Estable)

> **AVISO IMPORTANTE (RESTRICCIÓN DE USO):**  
> ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural.

---

## 1. Descripción del Modelo y Sistema de Unidades

- **Descripción:** Cubo unitario discretizado en 6 elementos sólidos TETRA_4 bajo elongación axial prescrita.
- **Nodos totales:** 8
- **Elementos totales:** 6
- **Grados de libertad (GDL):** 48 totales (12 activos libres, 12 coaccionados por apoyos)
- **Sistema de unidades:** Longitud `m`, Fuerza `N`, Tensión `Pa`, Momento `N·m`, Giro `rad`

## 2. Diagnóstico Numérico y Verificación de Equilibrio

| Parámetro de Diagnóstico | Valor | Estado |
| :--- | :--- | :--- |
| Validez topológica y constitutiva | `True` | OK |
| Singularidad / Mecanismo cinemático | `False` / `False` | ESTABLE |
| Número de condición escalado ($\kappa$) | `1.9595e+01` | Óptimo |
| Modos de energía nula detectados | `0` | 0 (Correcto) |
| Residuo global de fuerzas $\|\sum \mathbf{F} + \sum \mathbf{R}\|$ | `1.6893e-08 N` | Equilibrio Exacto |
| Residuo global de momentos $\|\sum \mathbf{M}_0\|$ | `8.7861e-09 N·m` | Equilibrio Exacto |
| Energía de deformación elástica ($U$) | `1.000000e+05 J` | Calculado |
| Escala visual de deformación recomendada | `60.79x` | Identificada |

## 3. Resumen de Valores Extremos

- **Desplazamiento nodal máximo ($\|\mathbf{u}\|_{\max}$):** `1.086278e-03 m` (`1.0863 mm`)
- **Tensión equivalente de Von Mises máxima ($\sigma_{vm,\max}$):** `2.000000e+08 Pa` (`200.000 MPa`)
- **Tensión axial máxima de tracción ($\sigma_{x,\max}$):** `2.000000e+08 Pa` (`200.000 MPa`)
- **Tensión axial mínima / compresión ($\sigma_{x,\min}$):** `2.000000e+08 Pa` (`200.000 MPa`)

## 4. Desplazamientos Nodales Calculados

| Nodo ID | Coord $(x, y, z)$ [m] | $u_x$ [m] | $u_y$ [m] | $u_z$ [m] | $\theta_x$ [rad] | $\theta_y$ [rad] | $\theta_z$ [rad] | $\|\mathbf{u}\|$ [m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** (000) | (0.000, 0.000, 0.000) | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` |
| **2** (100) | (1.000, 0.000, 0.000) | `1.00000e-03` | `1.72297e-19` | `6.76883e-20` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `1.00000e-03` |
| **3** (110) | (1.000, 1.000, 0.000) | `1.00000e-03` | `-3.00000e-04` | `1.29924e-19` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `1.04403e-03` |
| **4** (010) | (0.000, 1.000, 0.000) | `0.00000e+00` | `-3.00000e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `3.00000e-04` |
| **5** (001) | (0.000, 0.000, 1.000) | `0.00000e+00` | `0.00000e+00` | `-3.00000e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `3.00000e-04` |
| **6** (101) | (1.000, 0.000, 1.000) | `1.00000e-03` | `2.66934e-19` | `-3.00000e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `1.04403e-03` |
| **7** (111) | (1.000, 1.000, 1.000) | `1.00000e-03` | `-3.00000e-04` | `-3.00000e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `1.08628e-03` |
| **8** (011) | (0.000, 1.000, 1.000) | `0.00000e+00` | `-3.00000e-04` | `-3.00000e-04` | `0.00000e+00` | `0.00000e+00` | `0.00000e+00` | `4.24264e-04` |

## 5. Reacciones en los Apoyos

| Nodo ID | $R_x$ [N] | $R_y$ [N] | $R_z$ [N] | $M_x$ [N·m] | $M_y$ [N·m] | $M_z$ [N·m] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `-66666666.6667` | `-0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **2** | `66666666.6667` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **3** | `33333333.3333` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **4** | `-33333333.3333` | `0.0000` | `-0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **5** | `-33333333.3333` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **6** | `33333333.3333` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **7** | `66666666.6667` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |
| **8** | `-66666666.6667` | `0.0000` | `0.0000` | `0.0000` | `0.0000` | `0.0000` |

## 6. Esfuerzos Internos y Tensiones por Elemento

| Elem ID | Tipo | Conectividad | Medida ($L/A/V$) | Axial $N$ [N] | Cortante $\max(V_y, V_z)$ [N] | Momento $\max(M_y, M_z)$ [N·m] | $\sigma_{axial}$ [MPa] | $\sigma_{VonMises}$ [MPa] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | `TETRA_4` | [1, 2, 3, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |
| **2** | `TETRA_4` | [1, 3, 4, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |
| **3** | `TETRA_4` | [1, 4, 8, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |
| **4** | `TETRA_4` | [1, 8, 5, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |
| **5** | `TETRA_4` | [1, 5, 6, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |
| **6** | `TETRA_4` | [1, 6, 2, 7] | `0.1667` | `0.000` | `0.000` | `0.000` | `200.000` | `200.000` |

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
