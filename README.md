
# Instrucciones de Construcción y Programación

## 1. Clonar el repositorio (si aplica)
```bash
git clone git@github.com:jodridez/libopencm3-examples.git
cd libopencm3-examples
git checkout Proyecto
```

## 1.2 Inicializar y actualizar submódulos
```bash
git submodule init
git submodule update
```

## 2. Compilar libopencm3 (solo la primera vez)
```bash
cd libopencm3
make
```

## 3. Compilar el proyecto principal (Pong)
```bash
cd ..
cd examples/stm32/f4/stm32f429i-discovery/Pong
make
```

## 4. Cargar en la placa mediante OpenOCD
```bash
make flash
```
## 5. Conectar a la consola serial (opcional)
```bash
screen /dev/ttyACM0 115200
```

## Si se quiere cargar usar el programa de prueba de los sensores:
## 1. Compilar el programa para probar los sensores (HY-SRF05)
```bash
cd ../../../../..
cd examples/stm32/f4/stm32f429i-discovery/HY-SRF05
make
```

## 2. Cargar en la placa mediante OpenOCD
```bash
make flash
```

## 3. Conectar a la consola serial
```bash
screen /dev/ttyACM0 115200
```





# Pong con Control Gestual – Instrucciones de Uso
Controlado mediante dos sensores ultrasónicos HY-SR05/HCSR05 y un botón USER (PA0)  
Plataforma: **STM32F429I-DISC1**

---

## 📌 1. Conexiones de Hardware

### **Sensores Ultrasonido (versión PD4–PD7)**  
| Jugador | Trigger | Echo | Puerto MCU |
|--------|---------|------|------------|
| Player 1 | TRIG1 | ECHO1 | PD4 / PD5 |
| Player 2 | TRIG2 | ECHO2 | PD6 / PD7 |

**Advertencia:** Necesaria division de tension a 3.3 V en la salida echo del sensor

**Recomendación:** Mantener los sensores separados al menos 15–20 cm para evitar interferencias.

---

## 📌 2. Botón USER (Reinicio / Inicio)
- El botón **USER** en la placa (PA0) inicia el juego después del *warmup*.
- Durante *Game Over*, una nueva pulsación reinicia el juego.

---

## 📌 3. Flujo de Inicio del Sistema

1. Encender el STM32F429I-DISC1.  
2. El firmware ejecuta una fase de *warmup* del timer (≈1 s o 60 s según configuración).  
   - Se muestra una **barra de progreso** en el LCD.  
   - Esto estabiliza el temporizador para medir pulsos de 10 µs correctamente.
3. Al finalizar, aparecerá en pantalla:  
   **“BOTON USER para iniciar”**
4. Presionar **USER** para iniciar el juego.

---

## 📌 4. Cómo Controlar las Paletas

Cada jugador mueve su paleta acercando o alejando la mano frente a su sensor:

- **Movimiento vertical proporcional a la distancia detectada.**
- Se aplica un **filtro de suavizado (EMA)** para evitar jitter.
- Rango recomendado:  
  **5 cm – 20 cm** desde el sensor.

Si el sensor no detecta, la paleta mantiene la última posición válida.

---

## 📌 5. Reglas del Juego

- El primer jugador en alcanzar **5 puntos** gana.
- Al anotar un punto, la pelota regresa al centro.
- Al finalizar la partida, se muestra el mensaje:
  - **PLAYER 1 WINS**  
  - **PLAYER 2 WINS**
- Para iniciar una nueva partida, presiona el **botón USER**.

---

## 📌 6. Parámetros Editables en el Código

| Función | Parámetro | Efecto |
|--------|-----------|--------|
| Suavizado | `FILTER_ALPHA` | Reduce jitter (0 = muy suave, 1 = crudo) |
| Rango del sensor | `MIN_SENSOR_DISTANCE_MM / MAX_SENSOR_DISTANCE_MM` | Ajusta la zona útil |
| Velocidad de la pelota | `BALL_SPEED_X / BALL_SPEED_Y` | Dificultad |
| Puntos para ganar | `WINNING_SCORE` | Objetivo del juego |

---

## 📌 7. Requisitos de Librerías Externas

El proyecto depende de:  
- libopencm3  
- Drivers de LCD (lcd-spi.h)  
- Librería de gráficos (gfx.h)  
- SDRAM + consola serie  
- clock.c / console.c / sdram.c preinstalados en la plantilla de la placa Discovery

---

## 📌 8. Consejos de Uso

- Evita colocar la mano demasiado cerca del sensor (< 3 cm).  
- El HY-SR05 puede fallar si detecta superficies brillantes o pequeñas.  
- Asegura que los sensores apunten hacia el pecho o mano del jugador.


