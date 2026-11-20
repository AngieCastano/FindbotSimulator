# Fase 8 — Signo de vida "forma humana" por geometría 3D

El MVP detectaba las víctimas por su **color** naranja. Esta fase añade un detector de **forma
humana** que no usa el color: reconoce la geometría de un cuerpo acostado en la nube de puntos de
la cámara de profundidad. Para comprobarlo se añadió una variante del escenario con víctimas
"cubiertas de polvo" y escombros bajos que se parecen a una víctima.

## 1. Variante del escenario `dusty`

`usar_worldgen/config/dusty_params.yaml`:

| Parámetro | Valor | Efecto |
|---|---|---|
| `victim_appearance` | `dusty` | Víctimas gris parduzco (RGB 0.52, 0.49, 0.45), dentro de la gama de los escombros: el detector de color no las ve. |
| `n_low_debris` | 10 | Escombros bajos que se confunden geométricamente con una víctima. |
| `low_debris_height_range` | 0.08–0.28 m | Alturas parecidas a la de una víctima (0.22 m). |

Tipos de escombro bajo (huella en planta):

| Tipo | Largo | Ancho | Por qué confunde |
|---|---|---|---|
| Tablón (`plank`) | 1.2–2.2 m | 0.12–0.25 m | Largo como una persona. |
| Losa (`slab`) | 0.9–1.5 m | 0.9–1.5 m | Baja y extensa. |
| Bloque (`block`) | 0.25–0.5 m | 0.25–0.5 m | Bajo, como una parte del cuerpo. |

**Mismo escenario que la variante por defecto.** Los escombros bajos se sortean con un flujo
aleatorio propio después de colocar el resto. Si dejan una víctima inalcanzable, se vuelven a
sortear solo ellos. Así, `seed42` y `seed42_dusty` tienen los mismos escombros altos y víctimas en
las mismas posiciones, y la comparación entre variantes es justa. Con `n_low_debris: 0` no se
consume ningún número aleatorio extra: los mundos por defecto son idénticos byte a byte a los de
versiones anteriores (verificado con las semillas 1–5 y 42).

**Uso.** `world_params:=dusty` en cualquier launch; el mundo se guarda en
`worlds/seed<N>_dusty/` y los resultados como `seed<N>_dusty_<fecha>.*`.
También `WORLD_PARAMS=dusty scripts/run_batch.sh ...`.

## 2. Detector `human_shape`

La cámara está a 0.12 m del suelo y mira al frente: ve las víctimas (0.22 m de alto) de lado,
como un muro bajo. Por cada imagen de profundidad (`human_shape_detector.py`):

1. **Puntos 3D en la base del robot.** Cada píxel (1 de cada 4) se proyecta con el modelo
   *pinhole* y se lleva al frame `base_footprint` (z = altura sobre el suelo), con la TF fija
   cámara → base.
2. **Suelo.** Se descartan los puntos con z ≤ 0.04 m.
3. **Escombros altos.** Las columnas del suelo (rejilla de 0.1 m) con algún punto por encima de
   0.32 m pertenecen a un objeto alto; sus puntos bajos (la base del escombro) se descartan.
4. **Altura verificable.** Muy cerca de la cámara no se ve si un objeto sigue hacia arriba: con
   un campo de visión vertical de ±27°, a 0.6 m solo se ve hasta 0.43 m de altura. Se descartan
   los puntos donde la altura visible es menor que 0.45 m.
5. **Grupos.** Los puntos bajos restantes se agrupan en el plano del suelo (rejilla de 0.15 m);
   cada grupo con al menos 40 puntos y diagonal ≤ 2.4 m es una detección.

**Silueta en el mapa de evidencia.** Una sola imagen ve una cara del objeto; la forma completa se
comprueba en el mapa de evidencia del detector, que acumula las vistas desde distintos ángulos.
Para cada grupo se calcula (`tracker.py`):

- largo y ancho a lo largo de su eje principal (análisis de componentes principales),
- altura máxima,
- **perfil de alturas**: la altura máxima de cada tercio del largo; la relación entre el tercio
  más alto y el más bajo. Una persona acostada tiene cabeza y torso (≈ 0.2 m) más altos que las
  piernas (≈ 0.14 m): relación ≈ 1.5. Un tablón, una losa o un bloque tienen altura uniforme:
  relación ≈ 1.0.

Un grupo solo se confirma como víctima si tiene `min_observations` capturas **y** su geometría
cumple el filtro. Si después deja de cumplir la **altura o el perfil**, se retira (evento
`retracted` en el log). Si solo supera el largo o el ancho máximos, la víctima **se mantiene** en
su última posición: ocurre cuando la evidencia de dos víctimas cercanas (o de una víctima y un
escombro bajo vecino) se acaba uniendo en un solo grupo, y eso no contradice que haya una víctima.

| Filtro | Valor | Justificación |
|---|---|---|
| Largo | 1.2–2.4 m | Víctima de 1.7 m; la evidencia acumulada se alarga ≈ 0.2–0.3 m por el error de pose de SLAM. |
| Ancho | ≤ 1.1 m | Víctima de 0.6 m medida hasta 0.93 m (mismo ensanchamiento). |
| Altura máxima | 0.15–0.30 m | Víctima de 0.22 m; descarta escombros muy bajos. |
| Perfil (relación) | ≥ 1.25 | Víctimas medidas: 1.54–1.57; escombros bajos: 1.00–1.03. |

## 3. Varios detectores a la vez

Cada detector acumula evidencia en **su propio** mapa (con su propio filtro de forma: el de color
no tiene). Un registro común (`VictimRegistry`) une las víctimas confirmadas por distintos
detectores a menos de `merge_distance` (1 m) y les da una numeración única. Cada víctima indica
qué detectores la confirmaron: la etiqueta en RViz es, por ejemplo, "Víctima 2 (100 %,
color+forma)", y el log CSV tiene la columna `sources`. Es un anticipo de la fusión de
evidencias de la Fase 13.

## 4. Filtro con el mapa del propio robot

En una misión naranja aparecieron 2 falsos positivos del detector de color **detrás del muro
oeste** (x = −11.2 y −13.0, fuera del escenario). El robot dio varios giros de recuperación junto
a la víctima 2, SLAM perdió la pose un instante y la víctima se proyectó al otro lado del muro; el
mapa final quedó bien porque SLAM se recuperó, pero la evidencia ya estaba acumulada.

Ahora el detector descarta los puntos de evidencia que caen en celdas **desconocidas** o fuera
del `/map` del robot (`require_known_map`). Usa solo el mapa que construye el propio robot, no el
ground truth. Las víctimas reales caen siempre en celdas conocidas: el LiDAR (alcance 12 m) cubre
de sobra el alcance útil de la cámara (5–6 m).

## 5. Cambio en la exploración: escape de último recurso

En la primera misión `dusty` el robot quedó encajado 14 min entre dos escombros altos, con un
obstáculo a 0.26 m de su centro, dentro del polígono del `collision_monitor` (0.27 m a los lados).
Con un obstáculo ya dentro del polígono, el modo *approach* frena todo movimiento, también
alejarse, y las 51 maniobras de escape no movieron el robot. No depende de la Fase 8: es la misma
disposición de la semilla 42 naranja.

Si `escape_bypass_after` (3) escapes seguidos no mueven el robot más de 3 cm, el siguiente escape
es de **último recurso**: avance o retroceso recto, sin girar, hacia el lado de menor costo, a
0.05 m/s y como máximo 0.15 m, publicado directamente en `/cmd_vel` (sin `collision_monitor`).
Alejarse en línea recta de un obstáculo que aún no toca la carcasa no puede chocar con él.

## 6. Resultados

### Detector, fuera de línea (semilla 42 `dusty`, 1660 imágenes de una misión de 15 min)

| Objeto | Largo [m] | Ancho [m] | Altura [m] | Perfil | Resultado |
|---|---|---|---|---|---|
| victim_00 | 1.80 | 0.89 | 0.22 | 1.54 | confirmada |
| victim_01 | 1.94 | 0.93 | 0.22 | 1.57 | confirmada |
| victim_02 | 1.84 | 0.80 | 0.22 | 1.57 | confirmada |
| victim_03 | 1.89 | 0.86 | 0.22 | 1.57 | confirmada |
| victim_04 | 2.03 | 0.87 | 0.22 | 1.56 | confirmada |
| 6 losas | 0.60–1.74 | 0.20–1.63 | 0.10–0.23 | 1.00–1.01 | rechazadas |
| 1 tablón | 2.16 | 0.49 | 0.17 | 1.01 | rechazado |
| 3 bloques | 0.58–0.68 | 0.34–0.61 | 0.09–0.12 | 1.00 | rechazados |

Las 5 víctimas confirmadas y ninguno de los 10 escombros bajos: el perfil de alturas separa por
completo los dos grupos (1.54–1.57 frente a 1.00–1.01). Error de posición (centroide de la
evidencia frente al centro real): 0.24–0.42 m. Con el primer filtro de ancho (≤ 0.8 m) las
víctimas se confirmaban y luego se retiraban, al ensancharse la evidencia; de ahí los límites
actuales.

### Misiones completas (semilla 42, 15 min, configuración final)

| Variante | Víctimas | Por forma | Por color | Falsos positivos | Error medio | Cobertura |
|---|---|---|---|---|---|---|
| `dusty` | **5 / 5** | 5 | 0 | **0** | 0.35 m | 100 % |
| naranja | **5 / 5** | 4 | 4 | **0** | 0.30 m | 99.5 % |

- Con víctimas `dusty` el detector de color no ve nada y la forma encuentra las 5. Ninguno de los
  10 escombros bajos se confirmó.
- En la variante naranja, 3 víctimas las confirmaron los dos detectores ("color+forma"), una solo
  la forma y otra solo el color. En un caso el color confirmó dos partes de la misma víctima y el
  registro común las unió al confirmarla también la forma (evento `retracted` de la duplicada):
  el caso de la "víctima partida en dos" del MVP.
- El escape de último recurso se usó 2 veces en la misión `dusty`; las dos veces el robot siguió
  explorando.

**Misión de verificación `seed42_dusty_20261120_183012` (antes de mantener las víctimas unidas): 1 / 5.** Las
víctimas V1 y V3 (a ≈ 1.3 m una de otra) se confirmaron por forma a los 437 s y 562 s y se
retiraron **a la vez** a los 711 s: su evidencia se unió en un grupo más largo que 2.4 m. Además,
el robot pasó ≈ 250 s atascado en la misma zona que en la misión anterior (110–360 s) y no llegó
al oeste del escenario. Tras el cambio (mantener las víctimas cuando el grupo solo crece de
tamaño) se repitió la misión (ver «Repetición tras el cambio»).

**Repetición tras el cambio** (semilla 42 `dusty`, 15 min): **5 / 5**, 0 falsos positivos,
error medio 0.22 m, cobertura 97.7 %, recorrido 134 m. La víctima 3 (junto a un escombro alto)
se confirmó, retiró y volvió a confirmar dos veces entre 322 s y 463 s, con el mismo número: su
perfil oscila cerca del umbral mientras se acumulan las primeras vistas.

Pruebas anteriores con versiones intermedias (no repetibles, como referencia): con el filtro de
ancho ≤ 0.8 m, 0/5 al final (víctimas confirmadas y retiradas); sin el filtro del mapa, una misión
naranja con 5/5 y 2 falsos positivos detrás del muro; sin el escape de último recurso, una misión
`dusty` bloqueada 14 min (cobertura 31 %).

## 7. Limitaciones

- La silueta depende de que las víctimas simuladas sean humanoides geométricos con cabeza y torso
  más altos que las piernas. Una víctima real parcialmente enterrada podría no mostrar ese perfil.
- El detector necesita ver el objeto desde más de 0.6 m y desde varios ángulos para medir su
  forma; una víctima vista una sola vez y de lejos puede quedar sin confirmar.
- Los escombros bajos están diseñados para engañar al tamaño, no al perfil: un objeto bajo con
  un escalón de altura (por ejemplo, dos tablones apilados en parte) podría confirmarse.
