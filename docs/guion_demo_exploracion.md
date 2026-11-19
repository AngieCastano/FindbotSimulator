# Guion de demostración: exploración autónoma en 5 minutos

**Uso:** leer mientras corre la prueba corta.

```bash
ros2 launch usar_bringup exploration.launch.py seed:=42 quick:=true
```

Las indicaciones entre corchetes, **[EN PANTALLA]**, dicen qué mostrar en cada momento. Los
tiempos son aproximados: la simulación no es idéntica en cada ejecución, así que conviene seguir lo
que se ve y no el reloj.

---

## 1. Presentación (antes de lanzar, ~1 min)

Este proyecto es un sistema robótico autónomo, simulado, para buscar víctimas entre escombros
después de un sismo. Se desarrolló con la metodología CDIO: concebir, diseñar, implementar y
operar.

La idea es sencilla. Después de un derrumbe, entrar es peligroso y no hay planos de cómo quedó el
lugar. El robot tiene que entrar sin conocer nada, construir su propio mapa mientras avanza y
recorrer todo el espacio al que pueda llegar.

En esta demostración vamos a ver la parte de **exploración**: el robot recorre solo un escenario
de escombros durante cinco minutos, sin que nadie lo maneje.

---

## 2. Lo que se ve al arrancar (0:00 – 0:30)

**[EN PANTALLA]** Se abren dos ventanas: Gazebo a la izquierda y RViz a la derecha.

La ventana de **Gazebo** es el mundo simulado, con su física: gravedad, fricción y colisiones.
Vemos un área de 20 por 20 metros rodeada de muros, con 40 escombros, que son las cajas y
cilindros grises, y 5 víctimas, las figuras humanas naranjas acostadas en el suelo.

Este escenario no se dibujó a mano. Lo genera un programa a partir de un número, la **semilla**,
que aquí es 42. Con la misma semilla siempre sale el mismo escenario, así que los experimentos son
reproducibles. Con otra semilla sale un escenario distinto. El generador comprueba además que los
escombros no se superpongan, que haya espacio libre alrededor del robot y que todas las víctimas
sean alcanzables.

La ventana de **RViz** muestra lo que el robot sabe del mundo, que no es lo mismo que el mundo
real. Al principio casi todo es desconocido.

**Regla importante del diseño:** el robot **no tiene acceso** a dónde están los escombros ni las
víctimas. Solo decide con lo que perciben sus sensores. Esa información real se guarda aparte y se
usa únicamente para evaluar al robot al final, como haría un juez externo.

---

## 3. El robot y sus sentidos (0:30 – 1:00)

**[EN PANTALLA]** Acercar la vista al robot azul en Gazebo.

El robot es un vehículo de tracción diferencial: dos ruedas motrices y una rueda loca. Gira
variando la velocidad de cada rueda, como un tanque. Tiene tres fuentes de información:

1. **Odometría:** cuenta cuánto giró cada rueda para estimar cuánto avanzó.
2. **LiDAR:** un láser que gira 360 grados y mide la distancia a los obstáculos a 18 centímetros
   del suelo. **[EN PANTALLA]** Los puntos rojos en RViz.
3. **Cámara de profundidad:** mira al frente y, además de la imagen, mide la distancia de cada
   píxel. Es necesaria porque los brazos y las piernas de una víctima acostada llegan a 10 y 14
   centímetros de alto: el láser pasa por encima y no las ve.

---

## 4. El robot construye su mapa (1:00 – 2:00)

**[EN PANTALLA]** En RViz, el mapa en gris: claro = libre, negro = obstáculo, sin color =
desconocido.

El robot ya empezó a moverse solo. Mientras avanza dibuja el mapa. Esta técnica se llama **SLAM**,
localización y mapeo simultáneos. Usamos el paquete estándar *slam_toolbox* de ROS 2.

El problema que resuelve SLAM es este: para dibujar el mapa hay que saber dónde está el robot, y
para saber dónde está el robot hace falta el mapa. La odometría sola acumula error, porque las
ruedas patinan. SLAM compara cada nuevo barrido del láser con el mapa construido y corrige la
posición. En nuestras pruebas, el error de posición de SLAM se mantuvo por debajo de unos 10
centímetros tras recorrer 45 metros.

---

## 5. Cómo decide a dónde ir: las fronteras (2:00 – 3:00)

**[EN PANTALLA]** En RViz, las líneas **cian** en el borde entre lo gris y lo desconocido, los
puntos **amarillos** y el punto **verde**.

Esta es la parte central del proyecto. ¿Cómo decide el robot a dónde ir si no conoce el lugar?

Usamos la **exploración por fronteras**, propuesta por Yamauchi en 1997. Una **frontera** es el
borde entre lo que el robot ya sabe que está libre y lo que todavía no conoce. Son las líneas
cian. Si el robot va a una frontera, seguro descubre algo nuevo. Si repite esto hasta que no
quedan fronteras, ha explorado todo lo que podía alcanzar.

Cada pocos segundos el programa hace esto:

1. **Encuentra las fronteras** en el mapa y las agrupa en tramos.
2. Para cada tramo busca un **punto de llegada seguro**, con suficiente distancia a los obstáculos
   para que el robot no quede encajado. Son los puntos amarillos.
3. **Califica cada candidato** con una fórmula:

   **J = α · C − β · I**

   - **C** es el **costo**: cuántos metros tendría que recorrer el robot por el camino real, no en
     línea recta.
   - **I** es la **información**: cuánta zona desconocida hay alrededor de ese punto.
   - **α** y **β** son pesos que se ajustan en un archivo de configuración. Más α hace al robot
     "ahorrador" y prefiere lo cercano. Más β lo hace "curioso" y prefiere las zonas grandes sin
     explorar.

4. **Elige el de menor J**, el punto verde: el mejor equilibrio entre poco recorrido y mucha
   información nueva.

Si una meta falla, por ejemplo porque no hay paso, la anota en una **lista negra** (puntos rojos)
para no insistir, y elige otra.

---

## 6. Cómo llega sin chocar: la navegación (3:00 – 4:00)

**[EN PANTALLA]** La línea **verde** de la ruta y las zonas de colores alrededor de los escombros
(costmaps).

Una vez elegida la meta, la navegación la hace **Nav2**, el sistema de navegación estándar de
ROS 2. Funciona en capas:

- **Mapa de costos:** alrededor de cada obstáculo hay una zona de "peligro" que decrece con la
  distancia. Son los colores en RViz. El robot prefiere pasar por el centro de los pasillos.
- **Planificador:** calcula la ruta más corta y segura sobre ese mapa de costos. Es la línea verde.
- **Controlador:** hace que el robot siga la ruta, frenando en las curvas y cerca de los
  obstáculos. Usamos *Regulated Pure Pursuit*, un algoritmo geométrico que persigue un punto
  situado un poco más adelante sobre la ruta.
- **Filtro de seguridad:** antes de que cualquier orden llegue a las ruedas, un filtro simula
  durante un segundo hacia adelante el movimiento pedido, con la forma real del robot. Si ese
  movimiento lo llevaría a chocar, reduce la velocidad hasta detenerlo. Es la última barrera, y
  también protege al robot cuando se le maneja con el teclado.

Algo que se ve a menudo: el robot **gira sobre sí mismo**. Lo hace cuando la ruta empieza en otra
dirección. Y a veces **se detiene unos segundos** cerca de los escombros: el controlador es
conservador y prefiere parar antes que arriesgar un choque. Si no logra avanzar, el explorador
cambia de meta.

---

## 7. Final de los 5 minutos (4:00 – 5:00)

**[EN PANTALLA]** Mostrar el mapa en RViz y compararlo con la vista de Gazebo.

Al cumplirse el tiempo, el robot se detiene y en la terminal aparece el mensaje
**"Exploración terminada"**, con el número de metas enviadas y fallidas.

Si comparamos el mapa que construyó el robot con el escenario real, se ve que reconoce las mismas
formas: los muros, los escombros y los torsos de las víctimas.

Para medirlo usamos la **cobertura**: qué porcentaje del área alcanzable quedó explorada. Este
cálculo lo hace un programa aparte, el único que conoce el escenario real. A los 5 minutos el
robot suele haber explorado entre un tercio y dos tercios del área, según la ejecución. En el
recorrido completo de 15 minutos llegó al 96 %, sin que el robot se subiera a ninguna víctima ni
chocara con ningún obstáculo.

---

## 8. Cómo se hizo el programa (cierre, ~2 min)

Todo está construido sobre **ROS 2 Jazzy**, el sistema estándar para programar robots, y el
simulador **Gazebo Harmonic**, dentro de un contenedor **Docker**. Así cualquiera puede reproducir
el entorno exacto en su computador. Los programas propios están escritos en **Python**.

El sistema se organizó en **capas**, siguiendo el diseño del informe, y se construyó por **fases**.
Cada fase solo empezó cuando la anterior compilaba, pasaba sus pruebas y se había verificado
visualmente:

| Fase | Qué se construyó |
|---|---|
| 0 | Entorno de trabajo: contenedor con ROS 2 y Gazebo. |
| 1 | **Generador de escenarios:** crea el mundo a partir de la semilla y guarda aparte la información real para la evaluación. |
| 2 | **El robot:** su modelo físico, ruedas, LiDAR y cámara, y la conexión entre Gazebo y ROS 2. |
| 3 | **Mapeo y navegación:** SLAM y Nav2 configurados para este robot y este tipo de escenario. |
| 4 | **Exploración:** el programa de fronteras que acabamos de ver. |

Siempre preferimos usar **herramientas estándar y probadas**, como slam_toolbox y Nav2, en lugar
de reinventarlas. Así el trabajo propio se concentra en lo específico del problema: generar los
escenarios, decidir a dónde explorar y lidiar con los obstáculos bajos.

Las pruebas obligaron a corregir varios problemas reales, y eso es parte del aprendizaje del
proyecto:

- **El mapa se deformaba.** La causa estaba en cómo el simulador calculaba el contacto de las
  ruedas con el suelo: la odometría se equivocaba unos 45 grados por vuelta. Se corrigió la forma
  de contacto de las ruedas.
- **El robot se subía a las víctimas.** El láser no ve las piernas, y además sus lecturas borraban
  lo que había detectado la cámara. Se separó la información de cada sensor y se añadió el filtro
  de seguridad, con "memoria" de los obstáculos vistos.
- **El robot quedaba atrapado** en huecos estrechos. Se añadieron metas con holgura y una
  maniobra de escape.

Todos los parámetros están en archivos de configuración y documentados con su justificación, de
modo que los resultados se pueden reproducir y discutir.

**Próximos pasos:** detectar y ubicar las víctimas con la cámara (Fase 5), calcular las métricas
automáticamente (Fase 6) y repetir los experimentos con muchas semillas para evaluar el sistema de
forma estadística.
