# Capa 4b — Detección de víctimas

Este documento describe el nodo `victim_detector` (paquete `usar_detection`) y justifica sus
parámetros (`usar_detection/config/detector.yaml`).

## 1. Principio

En el MVP el signo de vida es la **forma humana por color**: las víctimas simuladas son modelos
humanoides de un naranja distintivo (RGB 1.0, 0.4, 0.0) y los escombros son grises. El detector
segmenta ese color en la imagen RGB, usa la imagen de profundidad para obtener el punto 3D, lo
transforma al frame `map` con la pose estimada por SLAM y confirma una víctima cuando la ha visto
varias veces en el mismo lugar.

El detector usa **solo** la cámara del robot y su propia estimación de pose. Nunca lee el ground
truth ni las poses de Gazebo.

```
/camera/image ───────┐
/camera/depth_image ─┼─► sincronización ─► detectores de signos de vida ─► puntos 3D (cámara)
/camera/camera_info ─┘         (5 Hz)        (ColorShapeDetector, ...)           │
                                                                TF map ← cámara  ▼
                     /victims/markers ◄── mapa de evidencia: acumulación, grupos, confirmación
                     /victims/detections ◄─┘                  └─► log CSV (results/)
```

## 2. Diseño extensible: signos de vida

La detección se separa en una interfaz común (`sign_of_life.py`):

- `Observation`: imagen RGB, profundidad, intrínsecos de la cámara e instante.
- `Detection`: tipo de signo de vida, punto 3D en el frame del sensor, píxel y confianza.
- `SignOfLifeDetector`: clase base abstracta con el método `detect(observación) → detecciones`.
- `@register_detector` y `create_detector(nombre, parámetros)`: registro por nombre.

El nodo instancia los detectores listados en el parámetro `detectors` y pasa a cada uno los
parámetros con el prefijo de su nombre (por ejemplo `color_shape.hsv_lower`). Para añadir otro
signo de vida (calor con una cámara térmica, sonido, CO₂) basta con escribir una subclase,
registrarla y añadir su nombre a `detectors`. El seguimiento, la publicación y el log no cambian.

## 3. Detector por color (`ColorShapeDetector`)

1. **Segmentación HSV.** La imagen se convierte al espacio HSV, que separa el tono del brillo y es
   más robusto a la iluminación que RGB. Se umbralizan el tono, la saturación y el brillo.
2. **Limpieza morfológica:** apertura (elimina píxeles sueltos) y cierre (rellena huecos) con un
   elemento elíptico.
3. **Blobs:** componentes conexas de la máscara; se descartan las de área menor que
   `min_blob_area`.
4. **Puntos 3D:** para cada píxel del blob con profundidad válida se calcula su punto 3D con el
   modelo de cámara *pinhole* (x = (u − cₓ)·z / fₓ, y = (v − c_y)·z / f_y).
5. **Separación en grupos:** los puntos se agrupan en el plano del suelo (lateral × profundidad)
   con una rejilla de `cluster_cell`. Dos víctimas que en la imagen forman un único blob, porque
   una está detrás de otra o casi pegadas, dan grupos separados. Cada grupo es una detección con
   sus puntos 3D y su punto representativo, la mediana.

**Calibración de los umbrales.** Medidos en imágenes de la cámara simulada frente a una víctima:
el naranja tiene H ≈ 20 (escala de OpenCV, 0–180), S ≥ 248 y V entre 170 y 255. El resto de la
escena (escombros, suelo, muros, cielo) tiene S ≤ 30. Los umbrales H ∈ [10, 30], S ≥ 150 y V ≥ 60
dejan un margen amplio sin riesgo de confundir los escombros.

## 4. Mapa de evidencia (`VictimTracker`)

- **Transformación:** los puntos 3D de las detecciones se llevan al frame `map` con la TF del
  instante de la imagen. Si esa TF aún no llegó (la imagen suele adelantarse unos milisegundos a
  la odometría), se usa la más reciente siempre que no difiera más de `tf_tolerance`.
- **Acumulación:** cada punto, proyectado al suelo, suma evidencia en una rejilla de
  `evidence_cell`. Cada celda guarda cuántos puntos recibió y en cuántas capturas distintas se
  observó.
- **Grupos:** se toman las celdas observadas en al menos `min_cell_frames` capturas, lo que filtra
  ruido de un solo instante, y se unen las que están a menos de `join_distance`.
- **Confirmación:** un grupo observado en al menos `min_observations` capturas distintas es una
  víctima confirmada. Su posición es el centroide del grupo ponderado por la evidencia.
- **Numeración estable:** cada grupo confirmado conserva el número de la víctima confirmada más
  cercana (a menos de `match_distance`); los nuevos reciben el siguiente número.
- **Confianza:** 0.5 al confirmarse y 1.0 al duplicar las observaciones necesarias.

**Por qué un mapa de evidencia.** Las pruebas descartaron dos alternativas:

| Alternativa | Problema observado en la semilla 42 |
|---|---|
| Un punto por blob (mediana) asociado a la pista más cercana | Dos víctimas alineadas con la cámara (`victim_02` delante de `victim_00`) formaban un único blob, y su mediana caía en un punto intermedio vacío: un falso positivo, y errores de 0.6 m en las dos víctimas reales. Dos víctimas casi pegadas (`victim_01` y `victim_03`) se veían como una. |
| Un punto por grupo 3D, asociado con D = 1 m | Según el punto de vista, una misma víctima se partía en grupos (piernas, torso) cuyas medianas de la cara visible quedaban a más de 1 m de la pista: 2 falsos positivos. No hay un D que funcione, porque `victim_01` y `victim_03` están a solo 1.47 m. |

Con el mapa de evidencia, las partes de una víctima vistas desde distintos ángulos se acumulan en
el mismo grupo y su centroide se acerca al centro del cuerpo. Dos víctimas solo se unen si entre
sus cuerpos queda menos de `join_distance` de suelo.

## 5. Parámetros

| Parámetro | Valor | Justificación |
|---|---|---|
| `processing_rate` | 5 Hz | La cámara publica a 15 Hz; procesar 5 imágenes/s basta a la velocidad del robot (≤ 0.4 m/s) y reduce la carga de CPU. |
| `color_shape.hsv_lower` / `hsv_upper` | [10, 150, 60] / [30, 255, 255] | Ver sección 3. |
| `color_shape.morph_kernel` | 5 px | Elimina ruido de pocos píxeles sin borrar extremidades lejanas. |
| `color_shape.min_blob_area` | 150 px | ≈ 0.05 % de la imagen; descarta reflejos y víctimas apenas visibles. |
| `color_shape.max_depth` | 6 m | A mayor distancia el blob es pequeño y la posición poco precisa. |
| `color_shape.min_valid_depth_fraction` | 0.3 | Exige profundidad válida en al menos el 30 % del blob. |
| `color_shape.cluster_cell` | 0.15 m | Separa objetos con más de ~15 cm de suelo libre entre ellos; los huecos dentro de una víctima son menores. |
| `evidence_cell` | 0.1 m | Resolución del mapa de evidencia; mucho menor que una víctima (1.7 × 0.6 m). |
| `min_cell_frames` | 2 | Una celda vista en una sola captura se considera ruido. |
| `join_distance` | 0.2 m | Une las partes de una víctima (los huecos entre extremidades son de ≈ 6 cm) sin unir víctimas separadas por más de 20 cm de suelo. |
| `min_observations` | 5 | ≈ 1 s de observación a 5 Hz. |
| `match_distance` | 1.0 m | El centroide de una víctima se desplaza poco al añadir evidencia; conserva su número. |
| `tf_tolerance` | 0.1 s | A 0.35 m/s y 1 rad/s, el error por desfase es < 4 cm y < 6°. |

## 6. Interfaces

| Interfaz | Tipo | Descripción |
|---|---|---|
| `/victims/markers` | `visualization_msgs/MarkerArray` | Esfera roja y etiqueta "Víctima k (confianza)" por víctima confirmada; esferas grises pequeñas para las pistas tentativas. |
| `/victims/detections` | `geometry_msgs/PoseArray` (frame `map`) | Posiciones de las víctimas confirmadas (entrada del evaluador de la Fase 6). |
| `/victims/debug_image` | `sensor_msgs/Image` | Imagen con la segmentación superpuesta y la distancia de cada blob (solo si hay suscriptores). |
| `results/victims_seed<s>_<fecha>.csv` | CSV | `event` (`confirmed` al confirmarse, `final` al cerrar), tiempo simulado, número de víctima, posición estimada, observaciones y confianza. |

## 7. Verificación

- **Pruebas unitarias** (`test/test_detection.py`), con imágenes sintéticas: registro e interfaz,
  proyección *pinhole*, detección y punto 3D de un blob, escena sin víctimas, blobs pequeños, dos
  víctimas separadas, profundidad inválida o lejana, robustez de la mediana a píxeles de fondo,
  separación de un blob con dos víctimas a distinta profundidad, víctima vista en diagonal, y en
  el mapa de evidencia: confirmación tras N capturas, partes vistas por separado unidas en una
  víctima centrada, víctimas cercanas separadas, ruido de una captura descartado, numeración
  estable y confianza.
- **Prueba de extremo a extremo** en la semilla 42 (sección 8).

## 8. Resultados

Prueba de extremo a extremo en la semilla 42 (`exploration.launch.py`, 15 min, sin interfaz
gráfica). Las víctimas confirmadas se compararon **fuera del robot** con el ground truth, con un
umbral de asociación de 1 m. Es la misma métrica que implementará la Fase 6.

| Métrica | Resultado |
|---|---|
| Víctimas detectadas | **5 / 5** (las 5 confirmadas a los 11 min; 3 en los primeros 2 min) |
| Falsos positivos | **0** |
| Error de localización (estimada vs real) | 0.14, 0.22, 0.22, 0.28 y 0.37 m (media 0.25 m) |
| Imágenes procesadas | 3311 (≈ 3.7 por segundo de tiempo simulado), 4 sin TF disponible |
| Cobertura de la exploración en la misma prueba | 97.8 % |

El error de localización se debe sobre todo a que la cámara solo ve la cara visible del cuerpo;
disminuye cuanto más variados son los puntos de vista.

**Limitaciones.**
- El detector por color solo funciona porque las víctimas simuladas tienen un color que no aparece
  en el resto del escenario.
- La simulación no es determinista: en las pruebas con versiones anteriores del seguimiento los
  resultados variaron entre ejecuciones, y esta configuración se probó en una ejecución completa.
  La Fase 6 permitirá repetir la prueba con varias semillas.
