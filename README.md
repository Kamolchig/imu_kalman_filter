# Filtro de Kalman para Datos de IMU en ROS

Este repositorio contiene un nodo de ROS escrito en Python que implementa un filtro de Kalman unidimensional para filtrar datos de una Unidad de Medición Inercial (IMU). El nodo se suscribe a los datos de orientación en forma de ángulos de Euler (roll, pitch y yaw) y publica los valores filtrados correspondientes. El objetivo es reducir el ruido en las mediciones y obtener estimaciones más precisas de la orientación.

## Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Modelo Matemático del Filtro de Kalman](#modelo-matemático-del-filtro-de-kalman)
- [Cómo Funciona el Código](#cómo-funciona-el-código)
- [Uso](#uso)
  - [Requisitos Previos](#requisitos-previos)
  - [Ejecución del Nodo](#ejecución-del-nodo)
- [Ajuste de Parámetros](#ajuste-de-parámetros)
- [Código](#código)
- [Consideraciones Adicionales](#consideraciones-adicionales)
- [Licencia](#licencia)

## Descripción General

El nodo implementa un filtro de Kalman para cada uno de los ángulos de orientación obtenidos de una IMU:

- **Roll**
- **Pitch**
- **Yaw**

Al aplicar el filtro de Kalman, se mejora la precisión de las mediciones al reducir el impacto del ruido inherente en los sensores de la IMU.

## Arquitectura de la Prueba

El siguiente diagrama describe cómo se integró el filtro de Kalman en el sistema ROS:



![Descripción](/ARQ.png)


Sensor (IMU) → Nodo de Kalman → Tópico filtrado → Nodo de control de navegación

Sensor (IMU): Genera datos de orientación (simulados o reales).

Nodo de Kalman: Aplica el filtro de Kalman a las lecturas de orientación.

Tópico filtrado: Publica los datos procesados en un nuevo tópico.

Nodo navegación: Utiliza los datos filtrados para navegación precisa.

NODO DE COMPARACIÓN: Comparación de señales.

## Modelo Matemático del Filtro de Kalman

El filtro de Kalman es un algoritmo recursivo que estima el estado de un sistema dinámico a partir de mediciones con ruido. Para un sistema unidimensional, el filtro de Kalman se compone de dos pasos principales:

### 1. Predicción

- **Estado Predicho (`x_prior`):**

  ```
  x_prior = x_posterior
  ```

- **Error de Estimación Predicho (`P_prior`):**

  ```
  P_prior = P_posterior + Q
  ```

  Donde:

  - `P_posterior`: Error de estimación posterior del paso anterior.
  - `Q`: Varianza del proceso (representa la incertidumbre del modelo del sistema).

### 2. Actualización (Corrección)

- **Ganancia de Kalman (`K`):**

  ```
  K = P_prior / (P_prior + R)
  ```

  Donde:

  - `R`: Varianza de medición (representa la incertidumbre de las mediciones).

- **Estado Posterior (`x_posterior`):**

  ```
  x_posterior = x_prior + K * (z - x_prior)
  ```

  Donde:

  - `z`: Medición actual.

- **Error de Estimación Posterior (`P_posterior`):**

  ```
  P_posterior = (1 - K) * P_prior
  ```

## Cómo Funciona el Código

El código realiza las siguientes tareas:

1. **Inicialización del Nodo ROS**: Se crea un nodo llamado `imu_kalman_filter`.

2. **Creación de Instancias del Filtro de Kalman**: Se instancian tres objetos de la clase `KalmanFilter`, uno para cada ángulo (roll, pitch y yaw).

3. **Configuración de Publishers y Subscribers**:

   - **Subscribers**: Se suscribe a los tópicos `/imu_euler/roll`, `/imu_euler/pitch` y `/imu_euler/yaw` para recibir los datos de orientación.
   - **Publishers**: Publica los datos filtrados en los tópicos `/imu_filtered/roll`, `/imu_filtered/pitch` y `/imu_filtered/yaw`.

4. **Procesamiento de Datos**:

   - Cada vez que se recibe un nuevo dato de orientación, se llama al método `update` del filtro de Kalman correspondiente para filtrar el dato.
   - Se publican los valores filtrados en los tópicos respectivos.

## Uso

### Requisitos Previos

- **ROS (Robot Operating System)** instalado y configurado.
- **Python 3**.
- Paquetes de ROS:

  - `rospy`
  - `std_msgs.msg.Float32`

### Ejecución del Nodo

1. Clona este repositorio en tu espacio de trabajo de ROS:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tu_usuario/imu_kalman_filter.git
   ```

2. Compila tu espacio de trabajo:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Fuente el espacio de trabajo:

   ```bash
   source devel/setup.bash
   ```

4. Ejecuta el nodo:

   ```bash
   rosrun imu_kalman_filter imu_kalman_filter.py
   ```

## Ajuste de Parámetros

Los valores de la varianza del proceso (`process_variance`) y la varianza de medición (`measurement_variance`) son críticos para el rendimiento del filtro:

- **Varianza del Proceso (`Q`)**:

  - Representa la incertidumbre en el modelo del sistema.
  - Un valor más alto indica que se espera que el sistema sea más dinámico o impredecible.

- **Varianza de Medición (`R`)**:

  - Representa el ruido en las mediciones.
  - Un valor más alto indica mediciones más ruidosas.
 
## Comparación de Señales

A continuación, se presenta una gráfica que muestra la comparación entre las señales sin filtrar y filtradas aplicando el filtro de Kalman:

![Descripción](/image.png)


En la gráfica se observa cómo la señal filtrada (en verde) sigue de cerca la señal original (en azul), reduciendo significativamente el ruido presente en la señal sin filtrar (en rojo).

## Conclusiones

La aplicación del filtro de Kalman mejora notablemente la calidad de las mediciones de la IMU, reduciendo el ruido y proporcionando estimaciones más precisas de los ángulos de orientación. Al comparar las señales antes y después de filtrar, se aprecia que el filtro es efectivo y mejora la estabilidad y precisión de los datos, lo cual es beneficioso para aplicaciones que dependen de mediciones confiables de orientación.

## Licencia

Este proyecto está bajo la Licencia MIT - consulta el archivo [LICENSE](LICENSE) para más detalles.
