#include <LiquidCrystal.h>       // Librería para controlar pantalla LCD
#include <Keypad.h>              // Librería para teclado matricial
#include "DHT.h"                 // Librería del sensor DHT (humedad y temperatura)
#include <ezButton.h>            // Librería para manejar botones físicos
#include <millisDelay.h>         // Librería para temporizaciones sin usar delay()
#include <StateMachine.h>        // Librería para implementar la máquina de estados

// -------------------- CONFIGURACIÓN HARDWARE -------------------- //

// Pines del sensor DHT
#define DHTPIN 9
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Sensor NTC (temperatura ambiente adicional)
const int pinNTC = A0;

// Pines de control de relés (ventilador y servo)
const int pinVentilador = 2;
const int pinCalefactor = 3;

// LCD: pines RS, E, D4, D5, D6, D7
LiquidCrystal lcd(13, 12, 11, 10, 8, 7);

// Definición del teclado matricial (4x4)
const byte filas = 4;
const byte columnas = 4;
char teclas[filas][columnas] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte pinesFilas[filas] = {A1, A2, A3, A4};
byte pinesColumnas[columnas] = {A5, 6, 5, 4};
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, filas, columnas);

// -------------------- VARIABLES DE CONTROL -------------------- //

enum Input {
  desconocido, // Sin entrada o estado indefinido
  dht,         // Sensor DHT disponible
  ntc,         // Sensor NTC disponible
  pmv,         // Índice PMV (Predicted Mean Vote)
  tiempo       // Variable de control por tiempo
};

Input input = desconocido;
StateMachine stateMachine; // Máquina de estados principal

// -------------------- VARIABLES DE SENSORES -------------------- //

float temperaturaDHT, humedadDHT, temperaturaNTC;
float temperaturaMedia, temperaturaIdeal;
float pmv_actual;

// Botón de bloqueo del sistema
ezButton botonBloqueo(1);

// Temporizador sin delay()
millisDelay temporizador;

// Variables para control de flujo
bool sistemaBloqueado = false;
bool sistemaPausado = false;

// -------------------- PROTOTIPOS DE FUNCIONES -------------------- //
void mostrarMensaje(String mensaje);
Input leerEntrada();
float calcularPMV(float temperatura, float humedad);
void setupStateMachine();
void mostrarPMV(float pmv);
void controlarClima(float pmv);
void controlManual();


// ============================================================================
//                                SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();

  pinMode(pinVentilador, OUTPUT);
  pinMode(pinCalefactor, OUTPUT);
  digitalWrite(pinVentilador, LOW);
  digitalWrite(pinCalefactor, LOW);

  botonBloqueo.setDebounceTime(50);

  temperaturaIdeal = 25.0f; // Valor por defecto (modificable por usuario)
  setupStateMachine();      // Configurar máquina de estados
  mostrarMensaje("Bienvenido al Sistema");
  delay(2000);
  mostrarMensaje("Inicializando...");
  delay(2000);
  lcd.clear();
  temporizador.start(2000);
}


// ============================================================================
//                                LOOP PRINCIPAL
// ============================================================================
void loop() {
  botonBloqueo.loop();

  // Si se presiona el botón de bloqueo:
  if (botonBloqueo.isPressed()) {
    sistemaBloqueado = !sistemaBloqueado; // Cambiar estado del sistema

    if (sistemaBloqueado) {
      mostrarMensaje("Sistema BLOQUEADO");
      digitalWrite(pinVentilador, LOW);
      digitalWrite(pinCalefactor, LOW);
    } else {
      mostrarMensaje("Sistema DESBLOQUEADO");
    }
  }

  // Si está bloqueado, no hace nada más
  if (sistemaBloqueado) return;

  // Leer entrada de sensores o teclado
  input = leerEntrada();
  stateMachine.Update();
}


// ============================================================================
//                            FUNCIÓN LEER ENTRADA
// ============================================================================
Input leerEntrada() {
  // Leer temperaturas y humedad
  temperaturaDHT = dht.readTemperature();
  humedadDHT = dht.readHumidity();
  int valorNTC = analogRead(pinNTC);
  temperaturaNTC = map(valorNTC, 0, 1023, -20, 50); // Aprox. en °C
  temperaturaMedia = (temperaturaDHT + temperaturaNTC) / 2;

  pmv_actual = calcularPMV(temperaturaMedia, humedadDHT);

  // --- Estados de máquina de estados según variable actual --- //
  State* estadoActual = stateMachine.GetState();

  // Estado: pmv_alto → volver a monitoreo si vuelve al rango
  if (estadoActual->GetName() == "pmv_alto" && pmv_actual <= 1.0f) {
    mostrarMensaje("Regreso al confort");
    digitalWrite(pinVentilador, LOW);
    delay(1000);
    return Input::pmv;
  }

  // Estado: Monitor → evaluar cambios
  if (estadoActual->GetName() == "Monitor") {
    if (pmv_actual > 1.0f) {
      mostrarMensaje("Temperatura alta");
      return Input::pmv;
    }
    if (pmv_actual < -1.0f) {
      mostrarMensaje("Temperatura baja");
      return Input::pmv;
    }
  }

  // Lectura de teclado para control manual
  char tecla = teclado.getKey();
  if (tecla) {
    if (tecla == '*') {
      sistemaPausado = !sistemaPausado;
      mostrarMensaje(sistemaPausado ? "Sistema pausado" : "Sistema activo");
    } else if (tecla == '#') {
      temperaturaIdeal = temperaturaMedia;
      mostrarMensaje("Nueva T ideal: " + String(temperaturaIdeal) + "C");
    }
  }

  // Si hay DHT disponible
  if (!isnan(temperaturaDHT) && !isnan(humedadDHT)) {
    mostrarPMV(pmv_actual);
    controlarClima(pmv_actual);
    return Input::dht;
  }

  // Si no hay DHT, pero sí NTC
  if (!isnan(temperaturaNTC)) {
    return Input::ntc;
  }

  return Input::desconocido;
}


// ============================================================================
//                           CÁLCULO DEL ÍNDICE PMV
// ============================================================================
float calcularPMV(float temperatura, float humedad) {
  // Modelo simplificado del índice PMV
  const float Tn = 22.0f; // temperatura neutra
  const float Hn = 50.0f; // humedad neutra
  float pmv = 0.3f * (temperatura - Tn) + 0.01f * (humedad - Hn);
  return pmv;
}


// ============================================================================
//                     CONFIGURACIÓN DE MÁQUINA DE ESTADOS
// ============================================================================
void setupStateMachine() {
  State* Monitor   = stateMachine.AddState("Monitor", []() {
    mostrarMensaje("Monitoreando...");
  });
  State* pmv_alto  = stateMachine.AddState("pmv_alto", []() {
    mostrarMensaje("Activando ventilador");
    digitalWrite(pinVentilador, HIGH);
    digitalWrite(pinCalefactor, LOW);
  });
  State* pmv_bajo  = stateMachine.AddState("pmv_bajo", []() {
    mostrarMensaje("Activando calefactor");
    digitalWrite(pinVentilador, LOW);
    digitalWrite(pinCalefactor, HIGH);
  });

  // --- Transiciones --- //
  stateMachine.AddTransition(Monitor, pmv_alto, []() {
    return input == pmv && pmv_actual > 1.0f;
  });
  stateMachine.AddTransition(Monitor, pmv_bajo, []() {
    return input == pmv && pmv_actual < -1.0f;
  });
  stateMachine.AddTransition(pmv_alto, Monitor, []() {
    return pmv_actual <= 1.0f;
  });
  stateMachine.AddTransition(pmv_bajo, Monitor, []() {
    return input == tiempo || pmv_actual >= -1.0f;
  });
}


// ============================================================================
//                     FUNCIÓN DE CONTROL AUTOMÁTICO DEL CLIMA
// ============================================================================
void controlarClima(float pmv) {
  if (sistemaPausado) return; // No actuar si está pausado

  if (pmv > 1.0f) { // Demasiado calor
    digitalWrite(pinVentilador, HIGH);
    digitalWrite(pinCalefactor, LOW);
    mostrarMensaje("Ventilador ON");
  } 
  else if (pmv < -1.0f) { // Demasiado frío
    digitalWrite(pinVentilador, LOW);
    digitalWrite(pinCalefactor, HIGH);
    mostrarMensaje("Calefactor ON");
  } 
  else { // Confort térmico
    digitalWrite(pinVentilador, LOW);
    digitalWrite(pinCalefactor, LOW);
    mostrarMensaje("Confort térmico");
  }
}


// ============================================================================
//                     FUNCIÓN DE CONTROL MANUAL (por teclado)
// ============================================================================
void controlManual() {
  char tecla = teclado.getKey();
  if (tecla) {
    if (tecla == 'A') {
      digitalWrite(pinVentilador, HIGH);
      mostrarMensaje("Ventilador manual ON");
    } else if (tecla == 'B') {
      digitalWrite(pinCalefactor, HIGH);
      mostrarMensaje("Calefactor manual ON");
    } else if (tecla == 'C') {
      digitalWrite(pinVentilador, LOW);
      digitalWrite(pinCalefactor, LOW);
      mostrarMensaje("Apagado manual");
    }
  }
}


// ============================================================================
//                        FUNCIÓN PARA MOSTRAR MENSAJES
// ============================================================================
void mostrarMensaje(String mensaje) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(mensaje);
  Serial.println(mensaje);
}


// ============================================================================
//                      FUNCIÓN PARA MOSTRAR EL PMV EN PANTALLA
// ============================================================================
void mostrarPMV(float pmv) {
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperaturaMedia, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("PMV: ");
  lcd.print(pmv, 2);
  lcd.print("  ");
}
