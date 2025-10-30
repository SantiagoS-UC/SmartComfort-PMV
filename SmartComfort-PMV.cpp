#include "StateMachineLib.h"
#include <AsyncTaskLib.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>

#define LED_GREEN 28
#define LED_RED 27
#define LED_BLUE 29
#define BUTTON_PIN 49
#define DHTPIN 22 
#define DHTTYPE DHT11
#define RELAY_PIN 25
#define SERVO_PIN 26
#define RST_PIN 9
#define SS_PIN 53
#define BUZZER_PIN 7
#define IR_SENSOR 23

DHT dht(DHTPIN, DHTTYPE);
Servo servo;
MFRC522 mfrc522(SS_PIN, RST_PIN);

#define analogPin A0
#define beta 3950.0
#define resistance 10.0
#define R0 10.0
#define T0 298.15

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
	{ '1', '2', '3', 'A' },
{ '4', '5', '6', 'B' },
	{ '7', '8', '9', 'C' },
{ '*', '0', '#', 'D' }
};

byte tarjetaUID[4] = {0x43, 0x89, 0x4F, 0x2E};
byte llaveroUID[4] = {0x56, 0x34, 0xDA, 0x73};
byte rowPins[ROWS] = { 30, 31, 32, 33 };
byte colPins[COLS] = { 34, 35, 36, 37 };

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
MFRC522::MIFARE_Key key;

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

String clave_store = "1234";
String inputKey = "";
float pmv_actual = 0.0;
int intentos_temp_alta = 0;
float temperatura_actual = 0.0;
bool pmv_alto_debe_salir = false;

// Variables para manejo del sensor IR y debounce (corrección)
unsigned long ultimo_ir_detectado = 0;
const unsigned long IR_DEBOUNCE_TIME = 500; // 500 ms debounce razonable
bool ir_armed = true; // arma para detectar sólo una vez hasta que libere

enum State { inicio,
	Config,
	Bloqueado,
	Alarma,
	Monitor,
	pmv_alto,
	pmv_bajo };
enum Input { tiempo,
	boton,
	Unknown,
	pmv,
	temperatura,
	keypadInput,
	keypadBlock,
	alarmaTemp,
	sensorIR};

StateMachine stateMachine(7, 13);
Input input;

AsyncTask taskConfig(5000, true, []() {
	input = tiempo;
});
AsyncTask taskMonitor(7000, true, []() {
	input = tiempo;
});
AsyncTask taskpmv_alto(5000, true, []() {
	input = tiempo;
});
AsyncTask taskpmv_bajo(3000, true, []() {
	input = tiempo;
});
AsyncTask taskLEDBLUEON(300, false, []() {
	digitalWrite(LED_BLUE, HIGH);
});
AsyncTask taskLEDBLUEOFF(400, false, []() {
	digitalWrite(LED_BLUE, LOW);
});
AsyncTask taskLEDGREENON(200, false, []() {
	digitalWrite(LED_GREEN, HIGH);
});
AsyncTask taskLEDGREENOFF(300, false, []() {
	digitalWrite(LED_GREEN, LOW);
});
AsyncTask taskLEDREDON(500, false, []() {
	digitalWrite(LED_RED, HIGH);
});
AsyncTask taskLEDREDOFF(500, false, []() {
	digitalWrite(LED_RED, LOW);
});
AsyncTask taskSHORTLEDREDON(100, false, []() {
	digitalWrite(LED_RED, HIGH);
});
AsyncTask taskSHORTLEDREDOFF(500, false, []() {
	digitalWrite(LED_RED, LOW);
});
AsyncTask taskBuzzer(500, true, []() {
	static bool buzzerState = false;
	buzzerState = !buzzerState;
	digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
});

int readInput();
void setupStateMachine();
void leerDatosRFID();
void autenticarBloque(byte bloque);
String leerBloque(byte bloque);
bool compararUID(byte *uid, byte *referencia);
String recibirCodigo();
String leerStringEEPROM(int direccion);
bool estaVacioEEPROM(int direccion);
void actualizarDisplayMonitor();

struct PMVResult {
	float pmv;
};

static inline float saturation_vapor_pressure_kPa(float T) {
	return 0.6105f * expf((17.27f * T) / (T + 237.3f));
}

PMVResult computePMV(float Ta, float Tr, float RH, float met, float clo, float va) {
	if (isnan(Ta) || isnan(Tr) || isnan(RH)) {
		return { 0.0f };
	}
	
	if (Ta < -10.0f || Ta > 50.0f) Ta = 25.0f;
	if (Tr < -10.0f || Tr > 50.0f) Tr = Ta;
	if (RH < 0.0f) RH = 0.0f;
	if (RH > 100.0f) RH = 100.0f;
	if (va < 0.0f) va = 0.1f;
	
	const float M = met * 58.15f;
	const float W = 0.0f;
	
	float p_sat = saturation_vapor_pressure_kPa(Ta);
	float p_a = (RH / 100.0f) * p_sat * 1000.0f;
	
	float f_cl = (clo <= 0.078f) ? (1.05f + 0.1f * clo) : (1.0f + 0.2f * clo);
	float I_cl = clo * 0.155f;
	float T_cl = Ta + 0.1f;
	
	for (int i = 0; i < 200; i++) {
		float h_c = 12.1f * sqrtf(max(0.0001f, va));
		float delta = fabsf(T_cl - Ta);
		float h_c2 = 2.38f * powf(delta, 0.25f);
		if (h_c2 > h_c) h_c = h_c2;
		
		float tclK = T_cl + 273.15f;
		float trK = Tr + 273.15f;
		float rad = 3.96e-8f * f_cl * (powf(tclK, 4.0f) - powf(trK, 4.0f));
		
		float T_new = 35.7f - 0.028f * (M - W) - I_cl * (rad + f_cl * h_c * (T_cl - Ta));
		
		if (fabsf(T_new - T_cl) < 1e-4f) {
			T_cl = T_new;
			break;
		}
		T_cl = T_new;
	}
	
	float h_c = 12.1f * sqrtf(max(0.0001f, va));
	float delta = fabsf(T_cl - Ta);
	float h_c2 = 2.38f * powf(delta, 0.25f);
	if (h_c2 > h_c) h_c = h_c2;
	
	float tclK = T_cl + 273.15f;
	float trK = Tr + 273.15f;
	float rad = 3.96e-8f * f_cl * (powf(tclK, 4.0f) - powf(trK, 4.0f));
	
	float PMV_factor = 0.303f * expf(-0.036f * M) + 0.028f;
	float PMV_balance = (M - W) 
		- 3.05e-3f * (5733.0f - 6.99f * (M - W) - p_a) 
		- 0.42f * ((M - W) - 58.15f)
		- 1.7e-5f * M * (5867.0f - p_a) 
		- 0.0014f * M * (34.0f - Ta) 
		- rad 
		- f_cl * h_c * (T_cl - Ta);
	
	float pmv = PMV_factor * PMV_balance;
	
	if (pmv > 3.0f) pmv = 3.0f;
	if (pmv < -3.0f) pmv = -3.0f;
	
	return { pmv };
}

float readNTCTemperature() {
	int adc = analogRead(analogPin);
	float Vout = adc * (5.0f / 1023.0f);
	float Rntc = (resistance * Vout) / (5.0f - Vout);
	float T_kelvin = 1.0f / ((1.0f / T0) + (1.0f / beta) * log(Rntc / R0));
	return T_kelvin - 273.15f;
}

void setup() {
	Serial.begin(9600);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	pinMode(LED_BLUE, OUTPUT);
	pinMode(LED_RED, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(IR_SENSOR, INPUT);
	digitalWrite(RELAY_PIN, LOW);
	digitalWrite(BUZZER_PIN, LOW);
	servo.attach(SERVO_PIN);
	servo.write(0);
	SPI.begin();
	mfrc522.PCD_Init();
	lcd.begin(16, 2);
	dht.begin();
	Serial.println("Starting State Machine...");
	setupStateMachine();
	Serial.println("State Machine Started");
	stateMachine.SetState(inicio, false, true);
	for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
}

void loop() {
	// readInput devuelve el Input detectado (o Unknown)
	Input newInput = static_cast<Input>(readInput());
	
	// Solo actualizamos si hay evento válido
	if (newInput != Unknown) {
		input = newInput;
	}
	
	// Actualiza la máquina
	stateMachine.Update();
	
	// Si hubo cambio de estado, limpiamos input para evitar doble procesado
	static State prevState = inicio;
	State currentState = stateMachine.GetState();
	if (currentState != prevState) {
		Serial.print("CAMBIO DE ESTADO: ");
		Serial.print(prevState);
		Serial.print(" -> ");
		Serial.println(currentState);
		prevState = currentState;
		input = Unknown;
	}
	
	// Actualizamos tareas asíncronas (mantén orden similar al original)
	taskConfig.Update();
	taskMonitor.Update();
	taskpmv_alto.Update();
	taskpmv_bajo.Update();
	taskLEDBLUEON.Update(taskLEDBLUEOFF);
	taskLEDBLUEOFF.Update(taskLEDBLUEON);
	taskLEDGREENON.Update(taskLEDGREENOFF);
	taskLEDGREENOFF.Update(taskLEDGREENON);
	taskLEDREDON.Update(taskLEDREDOFF);
	taskLEDREDOFF.Update(taskLEDREDON);
	taskSHORTLEDREDON.Update(taskSHORTLEDREDOFF);
	taskSHORTLEDREDOFF.Update(taskSHORTLEDREDON);
	taskBuzzer.Update();
	
	// Pequeño delay no crítico para estabilidad del loop (10 ms)
	delay(10);
}

void setupStateMachine() {
	// Transiciones desde INICIO
	stateMachine.AddTransition(inicio, Config, []() {
		return input == keypadInput;
	});
	stateMachine.AddTransition(inicio, Bloqueado, []() {
		return input == keypadBlock;
	});
	
	// Transiciones desde BLOQUEADO
	stateMachine.AddTransition(Bloqueado, inicio, []() {
		return input == boton;
	});
	stateMachine.AddTransition(Bloqueado, inicio, []() {
		return input == keypadInput;
	});
	
	// Transiciones entre CONFIG y MONITOR
	stateMachine.AddTransition(Config, Monitor, []() {
		return input == tiempo;
	});
	stateMachine.AddTransition(Monitor, Config, []() {
		return input == tiempo;
	});
	
	// Transiciones desde MONITOR a PMV
	stateMachine.AddTransition(Monitor, pmv_alto, []() {
		return input == pmv && pmv_actual > 1.0f;
	});
	stateMachine.AddTransition(Monitor, pmv_bajo, []() {
		return input == pmv && pmv_actual < -1.0f;
	});
	
	// Transiciones desde PMV_ALTO
	stateMachine.AddTransition(pmv_alto, Alarma, []() {
		// Solo iremos a Alarma si se llegó por condición de alerta (se usa input alarmaTemp)
		return input == alarmaTemp && intentos_temp_alta >= 3;
	});
	
	// --- CORRECCIÓN ---
	// Salida de pmv_alto a Monitor basada en pmv_actual (no depender únicamente de 'input')
	stateMachine.AddTransition(pmv_alto, Monitor, []() {
		return pmv_actual <= 1.00f;
	});
	
	// Transiciones desde PMV_BAJO
	stateMachine.AddTransition(pmv_bajo, Monitor, []() {
		return input == tiempo || pmv_actual >= -1.00f;
	});
	
	// Transiciones desde ALARMA
	stateMachine.AddTransition(Alarma, inicio, []() {
		return input == sensorIR;
	});
	
	stateMachine.AddTransition(Alarma, inicio, []() {
		return input == keypadInput;
	});
	
	// Configurar callbacks de entrada a estados
	stateMachine.SetOnEntering(inicio, enteringInicio);
	stateMachine.SetOnEntering(Config, enteringConfig);
	stateMachine.SetOnEntering(Bloqueado, enteringBloqueado);
	stateMachine.SetOnEntering(Alarma, enteringAlarma);
	stateMachine.SetOnEntering(Monitor, enteringMonitor);
	stateMachine.SetOnEntering(pmv_alto, enteringPMVALTO);
	stateMachine.SetOnEntering(pmv_bajo, enteringPMVBAJO);
	
	// Configurar callbacks de salida de estados
	stateMachine.SetOnLeaving(inicio, leavingInicio);
	stateMachine.SetOnLeaving(Config, leavingConfig);
	stateMachine.SetOnLeaving(Bloqueado, leavingBloqueado);
	stateMachine.SetOnLeaving(Alarma, leavingAlarma);
	stateMachine.SetOnLeaving(Monitor, leavingMonitor);
	stateMachine.SetOnLeaving(pmv_alto, leavingPmvAlto);
	stateMachine.SetOnLeaving(pmv_bajo, leavingPmvBajo);
}

void leerDatosRFID() {
	if (!mfrc522.PICC_IsNewCardPresent()) return;
	if (!mfrc522.PICC_ReadCardSerial()) return;
	byte bloqueNombre = 4;
	byte bloqueTemp = 5;
	
	Serial.print(F("\nUID detectado: "));
	for (byte i = 0; i < mfrc522.uid.size; i++) {
		Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
		Serial.print(mfrc522.uid.uidByte[i], HEX);
	}
	Serial.println();
	
	if (compararUID(mfrc522.uid.uidByte, tarjetaUID)) {
		String nombre = leerBloque(bloqueNombre);
		String temp = leerBloque(bloqueTemp);
		Serial.print(F("Bienvenido "));
		Serial.println(nombre);
		Serial.print(F("Temperatura preferida: "));
		Serial.println(temp);
		taskConfig.Start();
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(nombre);
		lcd.setCursor(0, 1);
		lcd.print("Temp pref:");
		lcd.print(temp);
		
	} 
	else if (compararUID(mfrc522.uid.uidByte, llaveroUID)) {
		String nombre = leerBloque(bloqueNombre);
		String temp = leerBloque(bloqueTemp);
		Serial.print(F("Bienvenido "));
		Serial.println(nombre);
		Serial.print(F("Temperatura preferida: "));
		Serial.println(temp);
		taskConfig.Start();
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(nombre);
		lcd.setCursor(0, 1);
		lcd.print("Temp pref:");
		lcd.print(temp);
	} 
	else {
		Serial.println(F("UID desconocido ? no registrado"));
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Tarjeta no");
		lcd.setCursor(0, 1);
		lcd.print("reconocida");
	}
	mfrc522.PICC_HaltA();
	mfrc522.PCD_StopCrypto1();
	delay(2000);
}

void autenticarBloque(byte bloque) {
	byte sectorTrailer = bloque - (bloque % 4) + 3;
	MFRC522::StatusCode status;
	status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
															sectorTrailer, &key, &(mfrc522.uid));
	if (status != MFRC522::STATUS_OK) {
		Serial.print(F("Error de autenticación en bloque "));
		Serial.print(bloque);
		Serial.print(F(": "));
		Serial.println(mfrc522.GetStatusCodeName(status));
	}
}

String leerBloque(byte bloque) {
	autenticarBloque(bloque);
	byte buffer[18];
	byte size = sizeof(buffer);
	MFRC522::StatusCode status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(bloque, buffer, &size);
	
	if (status != MFRC522::STATUS_OK) {
		Serial.print(F("Error leyendo bloque "));
		Serial.println(bloque);
		return "";
	}
	
	String data = "";
	for (int i = 0; i < 16; i++) {
		if (buffer[i] == 0) break;
		data += (char)buffer[i];
	}
	return data;
}

bool compararUID(byte *uid, byte *referencia) {
	for (byte i = 0; i < 4; i++) {
		if (uid[i] != referencia[i]) return false;
	}
	return true;
}

String recibirCodigo() {
	lcd.setCursor(0, 1);
	lcd.print("Clave: []     ");
	lcd.setCursor(7, 1);
	String result = "";
	unsigned long startTime = millis();
	
	while (result.length() < 4 && (millis() - startTime) < 15000) {
		char key = keypad.getKey();
		if (key) {
			if (key >= '0' && key <= '9') {
				lcd.print('*');
				result += key;
			} else if (key == '*' && result.length() > 0) {
				result = result.substring(0, result.length() - 1);
				lcd.setCursor(7, 1);
				for (int i = 0; i < result.length(); i++) lcd.print('*');
				for (int i = result.length(); i < 4; i++) lcd.print(' ');
				lcd.setCursor(7 + result.length(), 1);
			}
		}
		delay(50);
	}
	
	if (result.length() < 4) return "";
	return result;
}

// -------------------------------------------------------------
// readInput (con correcciones para pmv_alto y Alarma)
// -------------------------------------------------------------
int readInput() {
	State currentState = stateMachine.GetState();
	char key = keypad.getKey();
	
	// Estado Alarma - DEBOUNCE / REARM (CORRECCIÓN)
	if (currentState == Alarma) {
		int presencia = digitalRead(IR_SENSOR);
		unsigned long now = millis();
		
		// Si presencia (LOW) y estamos armados y pasó debounce -> desencadenar una vez
		if (presencia == LOW && ir_armed && (now - ultimo_ir_detectado > IR_DEBOUNCE_TIME)) {
			Serial.println("=== PRESENCIA DETECTADA - TRANSICION A INICIO ===");
			ultimo_ir_detectado = now;
			ir_armed = false;            // desarma hasta que vuelva HIGH
			// Apagar todo lo relacionado a ALARMA (buzzer/leds) ya aquí por seguridad
			digitalWrite(LED_RED, LOW);
			digitalWrite(BUZZER_PIN, LOW);
			taskBuzzer.Stop();
			//taskSHORTLEDREDON.Stop();
			//taskSHORTLEDREDOFF.Stop();
			return Input::sensorIR;
		}
		
		// Si el sensor vuelve a HIGH, rearmamos la detección futura
		if (presencia == HIGH && !ir_armed) {
			ir_armed = true;
			Serial.println("=== SENSOR IR LIBERADO - REARMADO ===");
		}
		
		// también aceptar '#' para salir
		if (key == '#') {
			Serial.println("=== TECLA # PRESIONADA - TRANSICION A INICIO ===");
			// Apagar sirena/leds
			digitalWrite(LED_RED, LOW);
			digitalWrite(BUZZER_PIN, LOW);
			taskBuzzer.Stop();
			taskSHORTLEDREDON.Stop();
			taskSHORTLEDREDOFF.Stop();
			return Input::keypadInput;
		}
		
		return Input::Unknown;
	}
	
	// Estado inicio
	if (currentState == inicio) {
		String codigo = recibirCodigo();
		if (codigo.length() == 4) {
			if (codigo == clave_store)
				return Input::keypadInput;
			else
				return Input::keypadBlock;
		}
	}
	
	// Estado Bloqueado
	if (currentState == Bloqueado) {
		
		if (key == '*') {
			Serial.println("Tecla '*' presionada - Volviendo a INICIO");
			return Input::keypadInput;
		}
	}
	
	// Estado Config
	if (currentState == Config) {
		leerDatosRFID();  
		if (input == tiempo) {
			return Input::tiempo;
		}
	}
	
	// Estado pmv_alto - SOLUCIÓN CORREGIDA: recalcula y decide salida
	if (currentState == pmv_alto) {
		if (input == tiempo) {
			Serial.println(">>> Timer pmv_alto disparado - Leyendo sensores");
			
			// Leer sensores
			float Ta = dht.readTemperature();
			float RH = dht.readHumidity();
			float Tr = readNTCTemperature();
			
			if (isnan(Ta) || isnan(RH)) {
				Serial.println("ERROR: Lecturas NaN - Reintentando");
				input = Unknown;
				taskpmv_alto.Start();
				return Input::Unknown;
			}
			
			temperatura_actual = Ta;
			PMVResult res = computePMV(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
			pmv_actual = res.pmv;
			
			Serial.print("PMV_ALTO -> T:");
			Serial.print(Ta);
			Serial.print("C | RH:");
			Serial.print(RH);
			Serial.print("% | PMV:");
			Serial.println(pmv_actual);
			
			// Limpiar input inmediatamente después de leer
			input = Unknown;
			
			// Si PMV se normalizó, detener timer y marcar salida a Monitor
			if (pmv_actual <= 1.00f) { // margen por precisión
				Serial.println("PMV NORMALIZADO - PREPARANDO SALIDA A MONITOR");
				taskpmv_alto.Stop();
				intentos_temp_alta = 0;
				pmv_alto_debe_salir = true; // flag: en next loop se hará la transición
				return Input::Unknown;      // la transición se hace por condición en stateMachine
			}
			
			// Si sigue alto, contar solo si temp>22
			if (temperatura_actual >= 21.0f) {
				intentos_temp_alta++;
				Serial.print("Intento ");
				Serial.print(intentos_temp_alta);
				Serial.println("/3 - PMV continua alto");
				
				if (intentos_temp_alta >= 3) {
					Serial.println("ALARMA: 3 intentos agotados - TRANSICION A ALARMA");
					taskpmv_alto.Stop();
					return Input::alarmaTemp;
				}
			} else {
				Serial.println("Temperatura <=22 -> reseteo contador");
				intentos_temp_alta = 0;
			}
			
			// Reiniciar timer para otro ciclo
			taskpmv_alto.Start();
			return Input::Unknown;
		}
		
		// Si pmv_alto_debe_salir fue marcado (pmv normalizado), devolver Unknown y
		// la transición pmv_alto->Monitor depende de pmv_actual <= 0.22 en setupStateMachine.
		return Input::Unknown;
	}
	
	// Estado pmv_bajo
	if (currentState == pmv_bajo) {
		if (input == tiempo) {
			taskpmv_bajo.Start();
			return Input::tiempo;
		}
		
		float Ta = dht.readTemperature();
		float RH = dht.readHumidity();
		float Tr = readNTCTemperature();
		temperatura_actual = Ta;
		
		if (!isnan(Ta) && !isnan(RH)) {
			PMVResult res = computePMV(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
			pmv_actual = res.pmv;
			
			if (pmv_actual >= -1.0f) {
				Serial.print("PMV normalizado en estado BAJO: ");
				Serial.println(pmv_actual);
				return Input::tiempo;
			}
		}
	}
	
	// Estado Monitor
	if (currentState == Monitor) {
		if (input == tiempo) {
			return Input::tiempo;
		}
		
		float Ta = dht.readTemperature();
		float RH = dht.readHumidity();
		float Tr = readNTCTemperature();
		temperatura_actual = Ta;
		
		if (!isnan(Ta) && !isnan(RH)) {
			PMVResult res = computePMV(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
			pmv_actual = res.pmv;
			
			Serial.print("Monitor - Temp: ");
			Serial.print(Ta);
			Serial.print("C, PMV: ");
			Serial.println(pmv_actual);
			
			// Detectar PMV alto o bajo para hacer transición
			if (pmv_actual > 1.0f) {
				return Input::pmv;
			}
			if (pmv_actual < -1.0f) {
				return Input::pmv;
			}
		}
	}
	
	int boton = digitalRead(BUTTON_PIN);
	if (boton == LOW) {
		delay(50);
		if (digitalRead(BUTTON_PIN) == LOW) {
			return Input::boton;
		}
	}
	
	return Input::Unknown;
}

String leerStringEEPROM(int direccion) {
	char caracter;
	String data = "";
	int i = 0;
	while (true) {
		caracter = EEPROM.read(direccion + i);
		if (caracter == '\0' || caracter == 0xFF) break;
		data += caracter;
		i++;
	}
	return data;
}

bool estaVacioEEPROM(int direccion) {
	byte valor = EEPROM.read(direccion);
	return (valor == 0xFF || valor == '\0');
}

void leavingInicio() {
	Serial.println("Leaving INICIO");
	inputKey = "";
	intentos_temp_alta = 0;
}

void leavingConfig() {
	Serial.println("Leaving CONFIG");
	
	taskConfig.Stop();
	input = Unknown;
}

void leavingBloqueado() {
	Serial.println("Leaving BLOQUEADO");
	taskLEDREDON.Stop();
	taskLEDREDOFF.Stop();
	digitalWrite(LED_RED, LOW);
}

void leavingAlarma() {
	Serial.println("Leaving ALARMA");
	intentos_temp_alta = 0;
	digitalWrite(LED_RED, LOW);
	taskSHORTLEDREDON.Stop();
	taskSHORTLEDREDOFF.Stop();
	taskBuzzer.Stop();
	digitalWrite(BUZZER_PIN, LOW);
	
	// Resetear estado del sensor IR al salir de alarma
	ir_armed = true;
	ultimo_ir_detectado = 0;
}

void leavingMonitor() {
	Serial.println("Leaving MONITOR");
	taskMonitor.Stop();
	input = Unknown;
}

void leavingPmvAlto() {
	Serial.println("Leaving PMV_ALTO");
	digitalWrite(RELAY_PIN, LOW);
	digitalWrite(LED_RED, LOW);
	taskpmv_alto.Stop();
	taskLEDBLUEON.Stop();
	//taskSHORTLEDREDON.Stop();
	//taskSHORTLEDREDOFF.Stop();
	
	// Solo resetear contador si NO vamos a Alarma
	if (input != alarmaTemp) {
		intentos_temp_alta = 0;
		Serial.println("Contador de temperatura alta reseteado");
	}
	
	// Reset flags
	pmv_alto_debe_salir = false;
	input = Unknown;
}

void leavingPmvBajo() {
	Serial.println("Leaving PMV_BAJO");
	servo.write(0);
	digitalWrite(LED_GREEN, LOW);
	taskpmv_bajo.Stop();
	taskLEDGREENON.Stop();
	taskLEDGREENOFF.Stop();
	input = Unknown;
}

void enteringInicio() {
	Serial.println("-> Estado: INICIO");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Sistema Listo");
	lcd.setCursor(0, 1);
	lcd.print("Ingrese clave:");
}

void enteringConfig() {
	Serial.println("-> Estado: CONFIG");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Modo CONFIG");
	lcd.setCursor(0, 1);
	lcd.print("Escanee tarjeta");
	// digitalWrite(LED_BLUE, HIGH);
}

void enteringBloqueado() {
	taskLEDREDON.Start();
	Serial.println("-> Estado: BLOQUEADO");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("BLOQUEADO");
	lcd.setCursor(0, 1);
	lcd.print("Clave incorrecta , presione *");
}

void enteringAlarma() {
	taskBuzzer.Start();
	taskSHORTLEDREDON.Start();
	Serial.println("-> Estado: ALARMA");
	Serial.println("*** ALARMA ACTIVADA ***");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("*** ALARMA ***");
	lcd.setCursor(0, 1);
	lcd.print("Presione OFF");
	
	// Resetear banderas del sensor IR al entrar
	ir_armed = true;
	ultimo_ir_detectado = 0;
}

void enteringMonitor() {
	taskMonitor.Start();
	Serial.println("-> Estado: MONITOR");
	
	// Resetear bandera de salida
	pmv_alto_debe_salir = false;
	
	// Leer sensores al entrar
	float Ta = dht.readTemperature();
	float RH = dht.readHumidity();
	float Tr = readNTCTemperature();
	if (!isnan(Ta) && !isnan(RH)) {
		temperatura_actual = Ta;
		PMVResult res = computePMV(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
		pmv_actual = res.pmv;
	}
	
	lcd.setCursor(0, 0);
	lcd.print("T:");
	lcd.print(temperatura_actual, 1);
	lcd.print("C Tr:");
	lcd.print(Tr, 1);
	lcd.print("C");
	
	lcd.setCursor(0, 1);
	lcd.print("H:");
	lcd.print(RH, 0);
	lcd.print("% PMV:");
	lcd.print(pmv_actual, 2);
	
	/*lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Monitor:");*/
	//actualizarDisplayMonitor();
	
	Serial.print("Temperatura: ");
	Serial.println(temperatura_actual);
	Serial.print("PMV: ");
	Serial.println(pmv_actual);
	
}

void actualizarDisplayMonitor() {
	lcd.setCursor(0, 1);
	lcd.print("T:");
	lcd.print(temperatura_actual, 1);
	lcd.print("C PMV:");
	lcd.print(pmv_actual, 2);
	lcd.print("  ");
}

void enteringPMVALTO() {
	taskpmv_alto.Start();
	digitalWrite(RELAY_PIN, HIGH);
	taskLEDBLUEON.Start();
	Serial.println("-> Estado: PMV_ALTO");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("PMV ALTO ");
	lcd.print(pmv_actual, 1);
	lcd.setCursor(0, 1);
}

void enteringPMVBAJO() {
	taskpmv_bajo.Start();
	taskLEDGREENON.Start();
	servo.write(90);
	Serial.println("-> Estado: PMV_BAJO");
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("PMV BAJO");
	lcd.setCursor(0, 1);
	lcd.print("Calentando...");
}
