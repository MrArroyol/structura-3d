# Informe Técnico de Análisis Estructural — 09_Caso_Invalido_Mecanismo_Colineal

**Fecha de generación:** 2026-10-20 10:26:22  
**Estado del cálculo:** FALLIDO / INESTABLE

> **AVISO IMPORTANTE (RESTRICCIÓN DE USO):**  
> ADVERTENCIA TÉCNICA Y LEGAL: Los resultados numéricos obtenidos mediante Structura 3D corresponden a un modelo idealizado bajo hipótesis de elasticidad lineal e infinitesimal. NO son aptos para construcción, diseño ejecutivo ni constituyen certificación estructural.

---

## 1. Descripción del Modelo y Sistema de Unidades

- **Descripción:** Dos barras articuladas colineales con nudo central inestable (mecanismo cinemático).
- **Nodos totales:** 3
- **Elementos totales:** 2
- **Grados de libertad (GDL):** 18 totales (3 activos libres, 6 coaccionados por apoyos)
- **Sistema de unidades:** Longitud `m`, Fuerza `N`, Tensión `Pa`, Momento `N·m`, Giro `rad`

## 2. Diagnóstico Numérico y Verificación de Equilibrio

| Parámetro de Diagnóstico | Valor | Estado |
| :--- | :--- | :--- |
| Validez topológica y constitutiva | `False` | ERROR |
| Singularidad / Mecanismo cinemático | `True` / `True` | INESTABLE |
| Número de condición escalado ($\kappa$) | `1.0000e+00` | Óptimo |
| Modos de energía nula detectados | `2` | Mecanismo |
| Residuo global de fuerzas $\|\sum \mathbf{F} + \sum \mathbf{R}\|$ | `0.0000e+00 N` | Equilibrio Exacto |
| Residuo global de momentos $\|\sum \mathbf{M}_0\|$ | `0.0000e+00 N·m` | Equilibrio Exacto |
| Energía de deformación elástica ($U$) | `0.000000e+00 J` | Calculado |
| Escala visual de deformación recomendada | `1.00x` | Identificada |

### Errores Detectados
- **ERROR:** Mecanismo o condición de apoyo insuficiente detectada: rigidez nula en nodo 2 (GDL 'uy'). Verifique apoyos en 3D o articulaciones colineales.


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
