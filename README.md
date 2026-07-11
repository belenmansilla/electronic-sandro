# 🤖 Sandro - Robot Minisumo Autónomo

[cite_start]Sandro es un robot minisumo autónomo diseñado y construido desde cero para competencias de robótica[cite: 3, 5]. [cite_start]Este repositorio contiene el código fuente, la lógica de control y la documentación técnica de la versión 1.5 del proyecto[cite: 4, 8].

## 📋 Especificaciones Técnicas

[cite_start]El hardware de Sandro está diseñado en una placa PCB artesanal de simple faz (FR4) [cite: 105][cite_start], fabricada con el método de cloruro férrico, y va montada sobre un chasis impreso en 3D[cite: 8, 105].

* [cite_start]**Microcontrolador:** ESP32-DEVKIT (30 pines) operando a 3.3V[cite: 42].
* [cite_start]**Tracción:** Motores DC N30 con caja reductora (500 RPM) configurados en tracción diferencial[cite: 12, 14].
* [cite_start]**Driver de Motores:** Puente H TB6612FNG[cite: 92].
* [cite_start]**Alimentación:** Batería LiPo CNHL MiniStar 3S (11.1V, 850 mAh, 70C)[cite: 21].
* [cite_start]**Regulador de Tensión:** Módulo Step-Down HW-613 ajustado a 3.3V fijos[cite: 27].
* [cite_start]**Sensores de Oponente:** 3 pares de emisores/receptores IR discretos (LED IR + 2N2222A) montados en el chasis[cite: 97].
* [cite_start]**Sensores de Línea:** 2 módulos QTR para detección del borde del dohyo[cite: 100].
* [cite_start]**Módulo de Largada/Debug:** Receptor infrarrojo VS1838B (38 kHz)[cite: 103].
* [cite_start]**Chasis:** PLA Rojo (FDM) equipado con una cuña frontal de hoja metálica[cite: 8, 12].

## 🧠 Lógica de Software y Estados

El cerebro de Sandro está programado en C++ utilizando el framework de Arduino. [cite_start]El flujo de ejecución está diseñado para ser seguro en pista, basándose en una **Máquina de Estados** gestionada por un pulsador físico (`KEY2`) [cite: 44] y comandos externos.

### Modos de Operación
1.  **Modo Desarmado (Seguridad):** Al energizar la placa con el switch general, el robot bloquea los motores por software. Un LED rojo indica que el sistema está energizado pero en espera.
2.  **Modo Armado (Combate/Prueba):** Al presionar el pulsador táctil de la placa, el sistema se arma (LED verde) y comienza a escuchar el entorno.
3.  **Telemetría Inalámbrica:** El ESP32 transmite datos de depuración en tiempo real mediante **Bluetooth Serial** (`Sandro_BT`), permitiendo monitorear variables a distancia y aislando la PC de los picos de corriente (Stall) del puente H.

### Control de Movimiento (Fase de Pruebas IR)
Actualmente, el firmware cuenta con un sistema de mapeo IR que permite probar la cinemática aislada usando un control remoto estándar para no requerir conexión física por USB:
* `Vol+` / `Vol-`: Avance y retroceso continuo.
* `CH+` / `CH-`: Giros tipo tanque de 90° con retoma de avance automático.
* `Play`: Freno de emergencia (Stop).

## 🛠️ Estructura del Repositorio

* `/src`: Archivos fuente (`.cpp`, `.h` o `.ino`) con la lógica principal del ESP32.
* `/docs`: Documentación técnica, esquemáticos en PDF y lista de materiales.
* `/hardware`: Archivos Gerber o de diseño (EasyEDA) para la fabricación de la PCB.

## 🚀 Próximos Pasos
* [ ] Integración de lecturas de los módulos QTR para evitar la caída del dohyo.
* [ ] Programación de la rutina autónoma de búsqueda de oponente mediante el arreglo de transistores Darlington.
* [ ] Calibración del algoritmo de empuje a máxima potencia PWM.

---
*Proyecto de robótica amateur desarrollado por Belén Mansilla.*