# Sumo1 – Robot Minisumo ESP32

> Firmware para robot de minisumo competitivo basado en ESP32-WROOM-32, desarrollado con PlatformIO + ESP-IDF y FreeRTOS.

---

## Español

- [Descripción](#descripción)
- [Hardware](#hardware)
- [Arquitectura de software](#arquitectura-de-software)
- [Tareas FreeRTOS](#tareas-freertos)
- [Modos de operación – Botón](#modos-de-operación--botón)
- [Configuración y ajuste fino](#configuración-y-ajuste-fino)

---

### Descripción

El robot gira buscando al oponente con 3 sensores de distancia infrarrojo (GP2Y0E03). Al detectarlo, embiste a velocidad máxima corrigiendo la dirección según el sensor que lo detectó. Dos sensores de piso detectan el borde blanco del dohyo y ejecutan una maniobra de escape. Todo el comportamiento se configura sin tocar electrónica: parámetros de velocidad, filtros y tipo de dohyo se ajustan por código o por botón.

---

### Hardware

| Componente | Descripción |
|---|---|
| MCU | ESP32-WROOM-32 |
| Driver de motores | TB6612FNG |
| Sensores de distancia | 3× GP2Y0E03 (analógico, hasta 35 cm) |
| Sensores de piso | 2× sensor digital (GPIO34, GPIO35) |
| Speaker | Buzzer pasivo vía LEDC |
| LEDs de feedback | 2× LED (GPIO13, GPIO14) |
| Botón de modo | GPIO5 (pull-up interno, activo en LOW) |

#### Mapa de pines

| Señal | GPIO |
|---|---|
| Motor izq. PWM | 26 |
| Motor der. PWM | 25 |
| Motor ENABLE (STBY) | 23 |
| Motor izq. IN1 / IN2 | 22 / 21 |
| Motor der. IN1 / IN2 | 18 / 19 |
| Sensor distancia – Frente | 33 (ADC1_CH5) |
| Sensor distancia – Izq. 45° | 4 (ADC2_CH0) |
| Sensor distancia – Der. 45° | 32 (ADC1_CH4) |
| Sensor piso 1 / 2 | 34 / 35 |
| Speaker | 27 |
| LED 1 / LED 2 | 13 / 14 |
| Botón modo | 5 |

---

### Arquitectura de software

```
┌─────────────────────────────────────────────────────────┐
│                        app_main                         │
│   Init HW → Crea colas/mutex → Lanza 4 tareas FreeRTOS  │
└───────────────┬─────────────────────────────────────────┘
                │
    ┌───────────┼───────────────────┐
    ▼           ▼                   ▼
vTaskSensors  vTaskBrain        vTaskMotors    vTaskFeedback
(prioridad 4) (prioridad 2)    (prioridad 3)  (prioridad 1)
    │               │                │              │
    │  g_sensor_data│  g_motor_queue │  motors_drive│  LEDs + speaker
    └───────────────┘                └──────────────┘
```

Los datos fluyen en una sola dirección: **Sensores → Brain → Motores**. El mutex `g_sensor_mutex` protege la lectura/escritura de `g_sensor_data`. La cola de motores usa `xQueueOverwrite` (profundidad 1) para que siempre llegue el comando más reciente.

---

### Tareas FreeRTOS

#### `vTaskSensors` — Prioridad 4 (la más alta)
- Corre cada **10 ms** usando `vTaskDelayUntil` (periodo exacto, no deriva).
- Lee los 3 sensores GP2Y0E03 via ADC con oversampling × 4 y actualiza el filtro de media móvil.
- Lee los 2 sensores de piso.
- Publica todo en `g_sensor_data` protegido por mutex.

#### `vTaskBrain` — Prioridad 2
- Ejecuta la máquina de estados de combate o el modo de test activo.
- Lee `g_sensor_data` via mutex y envía comandos a `g_motor_queue`.
- Detecta cambios de modo y habilita/deshabilita los motores en consecuencia.

#### `vTaskMotors` — Prioridad 3
- Espera comandos en `g_motor_queue` con timeout de **20 ms**.
- Si no llega comando en 20 ms y el robot está en combate, aplica freno de seguridad (watchdog).
- Convierte `motor_cmd_t` en señales PWM y dirección para el TB6612FNG.

#### `vTaskFeedback` — Prioridad 1 (la más baja)
- Espera comandos en `g_feedback_queue` (profundidad 8).
- Controla LEDs y genera tonos por el speaker vía LEDC.
- Cada nota bloquea la tarea el tiempo de duración, por eso tiene la prioridad más baja.

---

### Modos de operación – Botón

El botón (GPIO5) acepta pulsaciones múltiples con una ventana de **400 ms** entre presses. Cada press emite un beep corto. Al soltar, suena una confirmación con N beeps igual al número de presses.

| Presses | Modo | Descripción |
|:---:|---|---|
| 1 | **COMBAT** | Inicia el combate. Motores habilitados. |
| 2 | **TEST DISTANCIA** | Imprime por serial los valores raw y cm de los 3 sensores cada 500 ms. |
| 3 | **TEST LÍNEA** | Imprime por serial el estado de los 2 sensores de piso cada 100 ms. |
| 4 | **TEST MOTORES** | Secuencia automática: prueba cada motor por separado en ambas direcciones, 2 s por fase. |
| 5 | **CALIBRAR SENSORES** | Calibración interactiva por serial de los 3 sensores GP2Y0E03. Guarda en NVS (persiste entre reinicios). |
| 6 | **CAMBIAR TIPO DOHYO** | Alterna entre dohyo negro/borde blanco ↔ dohyo blanco/borde negro. Guarda en NVS. Confirma con beeps: grave = negro, agudo = blanco. |
| 7 | **TEST OPONENTE** | Detecta al oponente hasta 40 cm y parpadea LEDs + emite tono: grave izq. / medio frente / agudo der. Motores apagados. |

#### Indicador visual al encender
Tras el jingle de inicio, los LEDs muestran el tipo de dohyo guardado durante 2 segundos:
- **LED1** encendido = dohyo negro con borde blanco (estándar)
- **LED2** encendido = dohyo blanco con borde negro (invertido)

---

### Máquina de estados de combate

```
        ┌─────────┐
  boot  │  IDLE   │
───────►│         │
        └────┬────┘
             │ (automático al entrar a COMBAT)
        ┌────▼────┐
   ┌───►│ SEARCH  │◄──────────────────────┐
   │    │ gira    │                       │
   │    └────┬────┘                       │
   │         │ enemigo < 30 cm            │ enemigo perdido
   │    ┌────▼────┐                       │
   │    │ ATTACK  ├───────────────────────┘
   │    │ embiste │
   │    └────┬────┘
   │         │ borde detectado (en cualquier estado)
   │    ┌────▼────┐
   └────┤  EVADE  │ retrocede 300 ms → gira 200 ms
        └─────────┘
```

---

### Configuración y ajuste fino

#### Comportamiento de combate
> **Archivo:** `src/tasks/task_brain.c`

```c
#define ENEMY_DETECT_CM   30.0f  // Distancia (cm) a la que comienza el ataque
#define ENEMY_LOST_CM     35.0f  // Distancia (cm) a la que vuelve a buscar
#define SPEED_ATTACK     100.0f  // Velocidad de ataque: 0–100%
#define SPEED_SEARCH      40.0f  // Velocidad de rotación en búsqueda: 0–100%
#define EVADE_BACKUP_MS    300   // Tiempo de retroceso al detectar borde (ms)
#define EVADE_TURN_MS      200   // Tiempo de giro tras retroceso (ms)
```

> **Nota:** `ENEMY_LOST_CM` debe ser mayor que `ENEMY_DETECT_CM` para crear histéresis y evitar que el robot oscile entre ATTACK y SEARCH.

---

#### Filtro de los sensores de distancia
> **Archivo:** `src/drivers/gp2y0e03_driver.h`

```c
#define GP2Y_MA_WINDOW  4   // Ventana del filtro de media móvil (muestras)
                            // Lag total = GP2Y_MA_WINDOW × 10 ms
                            // 4 → 40 ms lag (recomendado para combate)
                            // 8 → 80 ms lag (más suave, más lento)
```

> Cada muestra ya tiene oversampling × 4 interno (4 lecturas ADC promediadas). Bajar la ventana a 3–4 mejora la respuesta al girar; subirla a 6–8 reduce el ruido en posición estática.

---

#### Periodo de muestreo de sensores
> **Archivo:** `src/tasks/task_sensors.c`

```c
#define SENSOR_PERIOD_MS  10   // Periodo de la tarea de sensores (ms)
                               // No bajar de 5 ms (límite del ADC legacy)
```

---

#### Rango de detección en modo de prueba
> **Archivo:** `src/tasks/task_brain.c`

```c
#define TEST_DETECT_CM  40.0f  // Rango de detección en modo 7 (TEST OPONENTE)
                               // Puede ser mayor que ENEMY_DETECT_CM para facilitar pruebas
```

---

#### Inversión de motores
> **Archivo:** `src/hal/hardware_map.h`

```c
#define MOTOR_LEFT_INVERTED   1   // 1 = motor izquierdo físicamente invertido
#define MOTOR_RIGHT_INVERTED  0   // 1 = motor derecho físicamente invertido
```

> Cambiar a `1` si un motor gira al revés respecto a lo esperado. La inversión se aplica en el driver antes de cualquier lógica de dirección.

---

#### Tipo de dohyo por defecto
> **Archivo:** `src/main.c`

```c
volatile dohyo_type_t g_dohyo_type = DOHYO_BLACK_RING;
// DOHYO_BLACK_RING = superficie negra, borde blanco (estándar)
// DOHYO_WHITE_RING = superficie blanca, borde negro (invertido)
```

> El valor se sobreescribe con lo que esté guardado en NVS al arrancar. Para forzar un tipo independientemente de la NVS, borrar la NVS o cambiar el valor inicial aquí.

---

#### Frecuencia y resolución PWM de motores
> **Archivo:** `src/hal/hardware_map.h`

```c
#define MOTOR_PWM_FREQ_HZ   20000     // Frecuencia PWM (Hz) – inaudible al oído humano
#define MOTOR_PWM_RES_HZ    10000000  // Reloj del timer MCPWM (Hz)
                                      // Ticks por periodo = RES / FREQ = 500
```

---

---

## English

- [Description](#description)
- [Hardware](#hardware-1)
- [Software Architecture](#software-architecture)
- [FreeRTOS Tasks](#freertos-tasks)
- [Operating Modes – Button](#operating-modes--button)
- [Configuration & Tuning](#configuration--tuning)

---

### Description

The robot spins searching for an opponent using 3 infrared distance sensors (GP2Y0E03). When detected, it charges at full speed, steering toward the sensor that triggered. Two floor sensors detect the white border of the dohyo and execute an evasion maneuver. All behavior is configurable without touching electronics: speed, filters, and dohyo type are adjusted in code or via button.

---

### Hardware

| Component | Description |
|---|---|
| MCU | ESP32-WROOM-32 |
| Motor driver | TB6612FNG |
| Distance sensors | 3× GP2Y0E03 (analog, up to 35 cm) |
| Floor sensors | 2× digital sensor (GPIO34, GPIO35) |
| Speaker | Passive buzzer via LEDC |
| Feedback LEDs | 2× LED (GPIO13, GPIO14) |
| Mode button | GPIO5 (internal pull-up, active LOW) |

#### Pin Map

| Signal | GPIO |
|---|---|
| Left motor PWM | 26 |
| Right motor PWM | 25 |
| Motor ENABLE (STBY) | 23 |
| Left motor IN1 / IN2 | 22 / 21 |
| Right motor IN1 / IN2 | 18 / 19 |
| Distance sensor – Front | 33 (ADC1_CH5) |
| Distance sensor – Left 45° | 4 (ADC2_CH0) |
| Distance sensor – Right 45° | 32 (ADC1_CH4) |
| Floor sensor 1 / 2 | 34 / 35 |
| Speaker | 27 |
| LED 1 / LED 2 | 13 / 14 |
| Mode button | 5 |

---

### Software Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        app_main                         │
│   Init HW → Create queues/mutex → Launch 4 FreeRTOS tasks│
└───────────────┬─────────────────────────────────────────┘
                │
    ┌───────────┼───────────────────┐
    ▼           ▼                   ▼
vTaskSensors  vTaskBrain        vTaskMotors    vTaskFeedback
(priority 4)  (priority 2)     (priority 3)   (priority 1)
    │               │                │              │
    │  g_sensor_data│  g_motor_queue │  motors_drive│  LEDs + speaker
    └───────────────┘                └──────────────┘
```

Data flows in one direction: **Sensors → Brain → Motors**. The `g_sensor_mutex` mutex protects `g_sensor_data` reads/writes. The motor queue uses `xQueueOverwrite` (depth 1) so only the latest command ever reaches the motors.

---

### FreeRTOS Tasks

#### `vTaskSensors` — Priority 4 (highest)
- Runs every **10 ms** using `vTaskDelayUntil` (exact period, no drift).
- Reads 3 GP2Y0E03 sensors via ADC with ×4 oversampling and updates moving-average filter.
- Reads 2 floor sensors.
- Publishes all data to `g_sensor_data` protected by mutex.

#### `vTaskBrain` — Priority 2
- Runs the combat state machine or the active test mode.
- Reads `g_sensor_data` via mutex and sends commands to `g_motor_queue`.
- Detects mode changes and enables/disables motors accordingly.

#### `vTaskMotors` — Priority 3
- Waits for commands on `g_motor_queue` with a **20 ms** timeout.
- If no command arrives within 20 ms during combat, applies safety brake (watchdog).
- Translates `motor_cmd_t` into PWM and direction signals for the TB6612FNG.

#### `vTaskFeedback` — Priority 1 (lowest)
- Waits for commands on `g_feedback_queue` (depth 8).
- Controls LEDs and generates tones via LEDC.
- Each note blocks the task for its duration — that's why it has the lowest priority.

---

### Operating Modes – Button

The button (GPIO5) accepts multiple presses with a **400 ms** window between presses. Each press emits a short beep. On release, a confirmation plays N beeps matching the press count.

| Presses | Mode | Description |
|:---:|---|---|
| 1 | **COMBAT** | Starts combat. Motors enabled. |
| 2 | **TEST DISTANCE** | Prints raw and cm values of all 3 sensors to serial every 500 ms. |
| 3 | **TEST LINE** | Prints the state of both floor sensors to serial every 100 ms. |
| 4 | **TEST MOTORS** | Automatic sequence: tests each motor individually in both directions, 2 s per phase. |
| 5 | **CALIBRATE SENSORS** | Interactive serial calibration of the 3 GP2Y0E03 sensors. Saves to NVS (survives resets). |
| 6 | **TOGGLE DOHYO TYPE** | Switches between black ring/white border ↔ white ring/black border. Saves to NVS. Confirmed with beeps: low pitch = black, high pitch = white. |
| 7 | **TEST OPPONENT** | Detects opponent up to 40 cm and blinks LEDs + emits tone: low left / mid front / high right. Motors off. |

#### Visual indicator at boot
After the startup jingle, LEDs show the saved dohyo type for 2 seconds:
- **LED1** on = black dohyo with white border (standard)
- **LED2** on = white dohyo with black border (inverted)

---

### Combat State Machine

```
        ┌─────────┐
  boot  │  IDLE   │
───────►│         │
        └────┬────┘
             │ (automatic on entering COMBAT)
        ┌────▼────┐
   ┌───►│ SEARCH  │◄──────────────────────┐
   │    │ spins   │                       │
   │    └────┬────┘                       │
   │         │ enemy < 30 cm             │ enemy lost
   │    ┌────▼────┐                       │
   │    │ ATTACK  ├───────────────────────┘
   │    │ charges │
   │    └────┬────┘
   │         │ border detected (from any state)
   │    ┌────▼────┐
   └────┤  EVADE  │ back up 300 ms → turn 200 ms
        └─────────┘
```

---

### Configuration & Tuning

#### Combat behavior
> **File:** `src/tasks/task_brain.c`

```c
#define ENEMY_DETECT_CM   30.0f  // Distance (cm) at which attack begins
#define ENEMY_LOST_CM     35.0f  // Distance (cm) at which robot returns to search
#define SPEED_ATTACK     100.0f  // Attack speed: 0–100%
#define SPEED_SEARCH      40.0f  // Rotation speed while searching: 0–100%
#define EVADE_BACKUP_MS    300   // Backup duration when border detected (ms)
#define EVADE_TURN_MS      200   // Turn duration after backing up (ms)
```

> **Note:** `ENEMY_LOST_CM` must be greater than `ENEMY_DETECT_CM` to create hysteresis and prevent oscillation between ATTACK and SEARCH.

---

#### Distance sensor filter
> **File:** `src/drivers/gp2y0e03_driver.h`

```c
#define GP2Y_MA_WINDOW  4   // Moving-average filter window (samples)
                            // Total lag = GP2Y_MA_WINDOW × 10 ms
                            // 4 → 40 ms lag (recommended for combat)
                            // 8 → 80 ms lag (smoother, slower response)
```

> Each sample already has ×4 internal oversampling (4 ADC reads averaged). Lower the window to 3–4 for faster response while spinning; raise to 6–8 for less noise in static position.

---

#### Sensor sampling period
> **File:** `src/tasks/task_sensors.c`

```c
#define SENSOR_PERIOD_MS  10   // Sensor task period (ms)
                               // Do not go below 5 ms (legacy ADC limit)
```

---

#### Test mode detection range
> **File:** `src/tasks/task_brain.c`

```c
#define TEST_DETECT_CM  40.0f  // Detection range in mode 7 (TEST OPPONENT)
                               // Can be larger than ENEMY_DETECT_CM for easier testing
```

---

#### Motor inversion
> **File:** `src/hal/hardware_map.h`

```c
#define MOTOR_LEFT_INVERTED   1   // 1 = left motor physically mounted in reverse
#define MOTOR_RIGHT_INVERTED  0   // 1 = right motor physically mounted in reverse
```

> Set to `1` if a motor spins in the wrong direction. Inversion is applied in the driver before any direction logic.

---

#### Default dohyo type
> **File:** `src/main.c`

```c
volatile dohyo_type_t g_dohyo_type = DOHYO_BLACK_RING;
// DOHYO_BLACK_RING = black surface, white border (standard)
// DOHYO_WHITE_RING = white surface, black border (inverted)
```

> This value is overridden by whatever is saved in NVS at boot. To force a type regardless of NVS, erase NVS or change the initial value here.

---

#### Motor PWM frequency and resolution
> **File:** `src/hal/hardware_map.h`

```c
#define MOTOR_PWM_FREQ_HZ   20000     // PWM frequency (Hz) – inaudible to humans
#define MOTOR_PWM_RES_HZ    10000000  // MCPWM timer clock (Hz)
                                      // Ticks per period = RES / FREQ = 500
```
