# SmartComfort-PMV

**SmartComfort-PMV** es un sistema inteligente de control y monitoreo ambiental desarrollado con Arduino. Su objetivo principal es medir y gestionar las condiciones ambientales de un espacio cerrado para mantener el confort térmico óptimo según el **índice PMV (Predicted Mean Vote)**.

---

## Funcionalidades

- **Lectura de sensores ambientales:**  
  - Temperatura (DHT11 y NTC)  
  - Humedad relativa  
  - Presencia (sensor IR)  
  - Control de actuadores (relevo y servomotor)  

- **Cálculo de PMV:**  
  Evalúa el confort térmico del ambiente y determina si es alto, bajo o aceptable, activando mecanismos de alerta o ajuste.

- **Sistema de seguridad y control de acceso:**  
  - Lectura de tarjetas RFID  
  - Introducción de códigos por teclado  
  - Bloqueo del sistema ante intentos fallidos  
  - Alarmas visuales (LEDs) y sonoras (buzzer)

- **Interfaz de usuario:**  
  - Pantalla LCD para mostrar temperatura, humedad, PMV y mensajes de estado  
  - Retroalimentación visual mediante LEDs  

- **Manejo de eventos asíncronos:**  
  - Temporizadores para actualización de sensores  
  - Debounce en sensores de presencia  
  - Control de tareas concurrentes sin bloquear el loop principal

---

## Tecnologías y librerías

- Arduino C++  
- Librerías utilizadas:  
  - `DHT.h` para sensores de temperatura y humedad  
  - `Servo.h` para control de servomotor  
  - `MFRC522.h` para lectura de RFID  
  - `LiquidCrystal.h` para pantalla LCD  
  - `AsyncTaskLib.h` para manejo de tareas asíncronas  
  - `StateMachineLib.h` para la máquina de estados  

---

## Cómo usar

1. Conectar los sensores y actuadores según la asignación de pines del código.  
2. Subir el proyecto a la placa Arduino.  
3. Abrir el monitor serie para ver mensajes de depuración.  
4. Escanear tarjetas RFID o ingresar códigos para entrar al modo de configuración.  
5. Observar los cambios de PMV y las alertas en la pantalla y los LEDs.

---

## Repositorio

Este repositorio contiene:  

- Código fuente completo (`.cpp`, `.h`)  
- Archivos de proyecto de Visual Studio (`.sln`, `.vcxproj`)  
- Ejemplo de configuración de tarjetas y parámetros PMV  

---

## Contribuciones

Puedes sugerir mejoras en el cálculo de PMV, agregar más sensores o refinar la interfaz de usuario.  

---

**Autor:** Santiago Solarte  
**Proyecto de ingeniería de sistemas**

