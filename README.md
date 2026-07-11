# 🤖 Sandro - Robot Minisumo Autónomo

Sandro es un robot minisumo autónomo diseñado y construido desde cero para competencias de robótica. Este repositorio contiene el código fuente, la lógica de control y la documentación técnica de la versión 1.5 del proyecto.

## 📋 Especificaciones Técnicas

El hardware de Sandro está diseñado en una placa PCB artesanal de simple faz (FR4), fabricada con el método de cloruro férrico, y va montada sobre un chasis impreso en 3D.

* **Microcontrolador:** ESP32-DEVKIT (30 pines) operando a 3.3V.
* **Tracción:** Motores DC N30 con caja reductora (500 RPM) configurados en tracción diferencial.
* **Driver de Motores:** Puente H TB6612FNG.
* **Alimentación:** Batería LiPo CNHL MiniStar 3S (11.1V, 850 mAh, 70C).
* **Regulador de Tensión:** Módulo Step-Down HW-613 ajustado a 3.3V fijos.
* **Sensores de Oponente:** 3 pares de emisores/receptores IR discretos (LED IR + 2N2222A) montados en el chasis.
* **Sensores de Línea:** 2 módulos QTR para detección del borde del dohyo.
* **Módulo de Largada/Debug:** Receptor infrarrojo VS1838B (38 kHz).
* **Chasis:** PLA Rojo (FDM) equipado con una cuña frontal de hoja metálica.

## 🧠 Lógica de Software y Estados

El cerebro de Sandro está programado en C++ utilizando el framework de Arduino. El flujo de ejecución está diseñado para ser seguro en pista, basándose en una **Máquina de Estados** gestionada por un pulsador físico (`KEY2`) y comandos externos.