# Investigación: deriva del mapa de SLAM (entre la Fase 11 y la Fase 12)

En la Fase 11 dos misiones con sonido mostraron el mapa de SLAM degradado (paredes duplicadas,
obstáculos corridos; ver `fase11_sonido.md`, sección 6.3). Antes de la Fase 12 se investigó si
las paradas de escucha (u otro comportamiento del explorador) causaban esa deriva.

## 1. Nueva métrica: error de localización del robot

El evaluador (`usar_metrics/evaluator.py`) compara cada segundo:

- **SLAM:** la TF `map → base_footprint` (lo que el robot cree que es su pose);
- **odometría de ruedas:** `/odom` (DiffDrive);
- **pose real:** `/sim/ground_truth/odom` (OdometryPublisher de Gazebo; solo simulación).

Las tres se llevan al marco del mundo con la pose de aparición y se emparejan por tiempo
(tolerancia 0.1 s). Salidas: `<semilla>_pose.csv`, `<semilla>_pose_error.png` y el bloque
`robot_localization` del JSON (media, máximo y final del error de posición y de orientación).

El evaluador ya leía el ground truth y no publica nada; la pose real se añade a la excepción
y el test `test_evaluator_only_reads_real_pose` comprueba que de `/sim/*` solo lee ese tópico.
Por eso `sim.launch.py` ahora siempre lanza el puente de la pose real en simulación.

## 2. Experimento

6 misiones de 900 s en el escenario `dusty`, semillas 1, 2 y 3, sin y con sonido
(`results/drift_eval/`).

| Misión | Víctimas | Cobertura | Error SLAM medio / máx. | Error odometría medio / máx. | Escapes (de último recurso) | Retrocesos | Paradas |
|---|---|---|---|---|---|---|---|
| s1 sin sonido | 3/5 | 98.7 % | 0.15 / 0.27 m | 0.41 / 2.77 m | 16 (2) | 5 | 0 |
| s2 sin sonido | 3/5 | 98.2 % | 0.09 / 0.16 m | 0.39 / 0.65 m | 4 (0) | 5 | 0 |
| s3 sin sonido | 4/5 | 99.5 % | 0.10 / 0.21 m | 0.40 / 2.15 m | 16 (2) | 4 | 0 |
| s1 con sonido | 3/5 | 99.5 % | 0.17 / 0.49 m | 0.00 / 0.02 m | 5 (0) | 2 | 18 |
| s2 con sonido | 5/5 | 96.6 % | 0.06 / 0.12 m | 0.00 / 0.02 m | 4 (0) | 4 | 17 |
| s3 con sonido | 5/5 | 99.8 % | 0.07 / 0.16 m | 0.00 / 0.02 m | 4 (0) | 3 | 17 |

Falsos positivos: 0 en las 6 misiones.

## 3. Hallazgos

1. **SLAM no se degradó en ninguna de las 6 misiones:** el error máximo fue 0.49 m y el medio
   está entre 0.06 y 0.17 m. No se repitieron las paredes duplicadas de la Fase 11.
2. **Las paradas de escucha no empeoran la localización:** las dos misiones con menor error de
   SLAM (s2 y s3 con sonido, 0.06–0.07 m) tuvieron 17 paradas cada una.
3. **La odometría de ruedas deriva por patinaje y SLAM la corrige.** El error de `/odom` crece
   a saltos justo después de los escapes "de último recurso" (s1 sin sonido: escapes a los 601 y
   661 s → +1.9 m de error entre 667 y 760 s; s3: escapes a los 703 y 773 s → +1.4 m entre 768
   y 900 s). El robot empuja los escombros, las ruedas giran sin avanzar y DiffDrive integra un
   movimiento que no ocurrió. Aun con 2.8 m de error en `/odom`, SLAM mantuvo la pose a
   ≤ 0.27 m. En las misiones sin escapes de último recurso la odometría casi no deriva
   (≤ 0.02 m: el modelo de Gazebo no añade ruido a la odometría, solo patinaje real).
4. **La deriva lenta de SLAM (s1 con sonido, 0.05 → 0.49 m entre 250 y 850 s)** es acumulación
   del emparejamiento de barridos en una zona sin cierres de lazo; no coincide con paradas,
   escapes ni patinaje (la odometría estuvo exacta toda la misión). Se ve en el mapa como
   obstáculos de la mitad superior corridos ~0.3 m.
5. La diferencia en escapes de último recurso entre las series (2 por misión sin sonido, 0 con
   sonido) no se puede atribuir al sonido con 3 semillas; los modelos del robot con y sin
   micrófonos difieren en 20 g y un visual (se comprobó con `xacro` + `gz sdf`).

## 4. Conclusión

No hay una causa ligada al sonido ni a las paradas: la degradación vista en la Fase 11 fue
ocasional y no se reprodujo. Se mantiene el diseño actual y se pasa a la Fase 12. Queda la
métrica de error de localización como parte permanente de la evaluación, que además sirve
para reportar en el informe que la odometría sola no basta en escombros (patinaje) y SLAM sí.

Mejora posible, no aplicada (no es necesaria para el MVP): bajar el empuje de los escapes de
último recurso o cortarlos al detectar patinaje (diferencia entre `/odom` y el avance que ve el
LiDAR).

## 5. Ruido de los encoders en la odometría (aplicado)

Estas 6 misiones mostraron que la odometría de Gazebo no tenía ruido (solo patinaje), aunque la
Capa 2 lo pide. Se añadió `usar_sim_sensors/odom_noise_sim`:

- DiffDrive publica la odometría ideal en `/sim/wheel_odom` (su TF ya no se puentea a ROS).
- El nodo suma ruido gaussiano a cada incremento (avance Δs, giro Δθ) con varianza proporcional
  a lo recorrido, `σ_s² = k_ss·|Δs| + k_st·|Δθ|`, `σ_θ² = k_ts·|Δs| + k_tt·|Δθ|`, integra la
  pose y publica `/odom` (con covarianzas) y la TF `odom → base_footprint`. Parámetros en
  `usar_sim_sensors/config/odom_noise_sim.yaml`; ruido reproducible con la semilla del mundo.
- Solo lee la odometría de las ruedas, nunca la pose real (lo comprueba
  `test_wheel_odometry_does_not_use_real_pose`). En el robot real `/odom` lo publica el ESP32.

Misión de verificación (semilla 1, sin sonido, `results/odom_noise/`):

| | Sin ruido (sección 2) | Con ruido |
|---|---|---|
| Error odometría medio / máx. | 0.41 / 2.77 m (solo patinaje, a saltos) | 0.63 / 1.49 m (deriva continua) |
| Error de rumbo de la odometría, máx. | 14.5° | 9.6° |
| Error SLAM medio / máx. / final | 0.15 / 0.27 / 0.25 m | 0.14 / 0.42 / 0.05 m |
| Víctimas / falsos positivos / cobertura | 3/5, 0, 98.7 % | 3/5, 0, 98.6 % |

La odometría ahora deriva de forma continua, como unos encoders reales, y SLAM la sigue
corrigiendo: el error de SLAM sube despacio hasta 0.42 m y un cierre de lazo a los 795 s lo
devuelve a 0.05 m. La misión no empeoró.
