/* =====================================================================
   ETAPA 4 — Se agrega el RC522 por SPI + control de acceso

   ===================================================================== */

#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <MFRC522.h>
#include <math.h>


/* =====================================================================
   1. CONFIGURACIÓN
   ===================================================================== */

// ---------- Bus I2C ----------
#define PIN_SDA            21
#define PIN_SCL            22
#define FREC_I2C_HZ        100000UL

#define DIR_MAX30102       0x57
#define DIR_MPU6050        0x68

// ---------- Bus 1-Wire ----------
#define PIN_ONEWIRE         4
#define RESOLUCION_DS18B20  12
#define TIEMPO_CONVERSION_DS18B20  750UL

// ---------- Bus SPI (VSPI) / RC522 ----------
// SCK=18, MOSI=23, MISO=19 los toma la librería SPI por defecto en ESP32.
#define PIN_RC522_SS        5
#define PIN_RC522_RST       27

// ---------- Actuadores ----------
#define PIN_LED_VERDE       25
#define PIN_LED_ROJO        26
#define PIN_BUZZER          33

#define T_INDICACION_MS     800    // cuánto quedan encendidos los LEDs
#define T_ALARMA_MS         400    // cuánto suena el buzzer en acceso denegado

// ---------- Períodos de muestreo, en milisegundos ----------
#define PERIODO_MAX30102     10
#define PERIODO_MPU6050      10
#define PERIODO_DIAGNOSTICO 1000
#define PERIODO_RC522       100    // cada cuánto se pregunta por una tarjeta nueva

// ---------- Política de reintento ----------
#define UMBRAL_FALLOS         5

// ---------- Rangos de validez ----------
#define IR_MINIMO_CONTACTO  50000UL
#define IR_MAXIMO           262143UL
#define ACEL_MAXIMA_G         16.0
#define ACEL_REPOSO_MIN_G      0.5
#define TEMP_MINIMA_C          20.0
#define TEMP_MAXIMA_C           45.0

#define SERIAL_BAUDIOS       115200


/* =====================================================================
   2. LISTA DE TAGS AUTORIZADOS

   Cada UID es el identificador único de una tarjeta/llavero RFID.
   Se guardan como texto hexadecimal en mayúsculas, sin espacios.

   CÓMO OBTENER EL UID DE UN TAG REAL:
   Cargar este mismo sketch, acercar el tag al lector y mirar la
   consola: el UID leído se imprime ahí (ver leerTagRFID()) incluso
   antes de estar en esta lista, marcado como DENEGADO. Copiarlo desde
   ahí y pegarlo aquí abajo.
   ===================================================================== */

const char* TAGS_AUTORIZADOS[] = {
  "A3B21C04",   // reemplazar por el UID real del primer tag del kit
  "8F1E9D22"    // reemplazar por el UID real del segundo tag del kit
};
const uint8_t CANTIDAD_TAGS_AUTORIZADOS =
    sizeof(TAGS_AUTORIZADOS) / sizeof(TAGS_AUTORIZADOS[0]);


/* =====================================================================
   3. MODELO DE DATOS
   ===================================================================== */

enum EstadoModulo {
  NO_INICIALIZADO,
  LECTURA_OK,
  ERROR_BUS,
  FUERA_DE_RANGO,
  SIN_DATO
};

struct Medicion {
  float        valor;
  const char*  unidad;
  uint32_t     marca_tiempo;
  const char*  origen;
  bool         valida;
  EstadoModulo estado;
};

const char* nombreEstado(EstadoModulo e) {
  switch (e) {
    case NO_INICIALIZADO: return "NO_INIT";
    case LECTURA_OK:       return "OK";
    case ERROR_BUS:       return "ERROR_BUS";
    case FUERA_DE_RANGO:  return "FUERA_RANGO";
    case SIN_DATO:        return "SIN_DATO";
    default:              return "?";
  }
}


/* =====================================================================
   4. VALIDACIÓN
   ===================================================================== */

EstadoModulo validarMAX30102(uint32_t ir) {
  if (ir == 0)                 return ERROR_BUS;
  if (ir >= IR_MAXIMO)         return FUERA_DE_RANGO;
  if (ir < IR_MINIMO_CONTACTO) return FUERA_DE_RANGO;
  return LECTURA_OK;
}

EstadoModulo validarMPU6050(float ax, float ay, float az) {
  float magnitud = sqrt(ax*ax + ay*ay + az*az);
  if (ax == 0.0 && ay == 0.0 && az == 0.0) return ERROR_BUS;
  if (magnitud > ACEL_MAXIMA_G)            return FUERA_DE_RANGO;
  if (magnitud < ACEL_REPOSO_MIN_G)        return FUERA_DE_RANGO;
  return LECTURA_OK;
}

EstadoModulo validarDS18B20(float temperatura) {
  if (temperatura <= -126.0 || fabs(temperatura - 85.0) < 0.01) return ERROR_BUS;
  if (temperatura < TEMP_MINIMA_C || temperatura > TEMP_MAXIMA_C) return FUERA_DE_RANGO;
  return LECTURA_OK;
}


/* =====================================================================
   5. ESTADO GLOBAL
   ===================================================================== */

MAX30105          max30102;
Adafruit_MPU6050  mpu;

OneWire            busOneWire(PIN_ONEWIRE);
DallasTemperature  ds18b20(&busOneWire);
DeviceAddress      direccionDS18B20;

MFRC522            rc522(PIN_RC522_SS, PIN_RC522_RST);

Medicion medIR   = {0, "cuentas", 0, "MAX30102", false, NO_INICIALIZADO};
Medicion medAcel = {0, "g",       0, "MPU6050",  false, NO_INICIALIZADO};
Medicion medTemp = {0, "C",       0, "DS18B20",  false, NO_INICIALIZADO};

uint8_t fallosMax  = 0;
uint8_t fallosMpu  = 0;
uint8_t fallosTemp = 0;

uint32_t tMax  = 0;
uint32_t tMpu  = 0;
uint32_t tTemp = 0;
uint32_t tDiag = 0;
uint32_t tRfid = 0;

char     ultimoUID[21] = "ninguno";   // últimos 20 caracteres + terminador
EstadoModulo estadoRFID = NO_INICIALIZADO;


/* =====================================================================
   6. MÓDULOS DE LAS ETAPAS ANTERIORES (sin cambios)
   ===================================================================== */

bool inicializarMAX30102() {
  if (!max30102.begin(Wire, I2C_SPEED_STANDARD)) { medIR.estado = ERROR_BUS; return false; }
  max30102.setup(0x1F, 4, 2, 100, 411, 4096);
  medIR.estado = SIN_DATO;
  return true;
}

void actualizarMAX30102() {
  max30102.check();
  if (!max30102.available()) {
    fallosMax++;
    if (fallosMax >= UMBRAL_FALLOS) medIR.estado = ERROR_BUS;
    return;
  }
  uint32_t ir = max30102.getFIFOIR();
  max30102.nextSample();
  EstadoModulo estado = validarMAX30102(ir);
  medIR.valor = (float)ir; medIR.marca_tiempo = millis();
  medIR.estado = estado; medIR.valida = (estado == LECTURA_OK);
  fallosMax = (estado == ERROR_BUS) ? fallosMax + 1 : 0;
}

bool inicializarMPU6050() {
  if (!mpu.begin(DIR_MPU6050, &Wire)) { medAcel.estado = ERROR_BUS; return false; }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  medAcel.estado = SIN_DATO;
  return true;
}

void actualizarMPU6050() {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    fallosMpu++;
    if (fallosMpu >= UMBRAL_FALLOS) medAcel.estado = ERROR_BUS;
    return;
  }
  const float G = 9.80665;
  float ax = a.acceleration.x / G, ay = a.acceleration.y / G, az = a.acceleration.z / G;
  EstadoModulo estado = validarMPU6050(ax, ay, az);
  medAcel.valor = sqrt(ax*ax + ay*ay + az*az); medAcel.marca_tiempo = millis();
  medAcel.estado = estado; medAcel.valida = (estado == LECTURA_OK);
  fallosMpu = (estado == ERROR_BUS) ? fallosMpu + 1 : 0;
}

enum EstadoDS18B20 { DS_REPOSO, DS_CONVIRTIENDO };
EstadoDS18B20 estadoDS = DS_REPOSO;
uint32_t tSolicitudDS = 0;

bool inicializarDS18B20() {
  ds18b20.begin();
  if (ds18b20.getDeviceCount() == 0) { medTemp.estado = ERROR_BUS; return false; }
  if (!ds18b20.getAddress(direccionDS18B20, 0)) { medTemp.estado = ERROR_BUS; return false; }
  ds18b20.setResolution(direccionDS18B20, RESOLUCION_DS18B20);
  ds18b20.setWaitForConversion(false);
  medTemp.estado = SIN_DATO;
  return true;
}

void actualizarDS18B20() {
  uint32_t ahora = millis();
  switch (estadoDS) {
    case DS_REPOSO:
      ds18b20.requestTemperaturesByAddress(direccionDS18B20);
      tSolicitudDS = ahora;
      estadoDS = DS_CONVIRTIENDO;
      break;
    case DS_CONVIRTIENDO:
      if (ahora - tSolicitudDS >= TIEMPO_CONVERSION_DS18B20) {
        float t = ds18b20.getTempC(direccionDS18B20);
        EstadoModulo estado = validarDS18B20(t);
        medTemp.valor = t; medTemp.marca_tiempo = ahora;
        medTemp.estado = estado; medTemp.valida = (estado == LECTURA_OK);
        fallosTemp = (estado == ERROR_BUS) ? fallosTemp + 1 : 0;
        if (fallosTemp >= UMBRAL_FALLOS) medTemp.estado = ERROR_BUS;
        estadoDS = DS_REPOSO;
      }
      break;
  }
}


/* =====================================================================
   7. MÓDULO RC522 — módulo nuevo de esta etapa

   Tres partes: leer el UID, decidir si está autorizado, y accionar
   LEDs/buzzer sin bloquear.
   ===================================================================== */

bool inicializarRC522() {
  SPI.begin();               // SCK=18, MISO=19, MOSI=23 por defecto en ESP32
  rc522.PCD_Init();

  // Verifica que el lector responda con una versión de firmware
  // válida. 0x00 y 0xFF son las lecturas típicas de "no hay lector"
  // o "cableado incorrecto" (por ejemplo RST sin conectar).
  byte version = rc522.PCD_ReadRegister(MFRC522::VersionReg);
  if (version == 0x00 || version == 0xFF) {
    estadoRFID = ERROR_BUS;
    return false;
  }

  estadoRFID = SIN_DATO;
  return true;
}

// Convierte el UID binario que entrega la librería a texto
// hexadecimal en mayúsculas, para poder compararlo e imprimirlo.
void uidATexto(MFRC522::Uid *uid, char *destino, size_t tam) {
  destino[0] = '\0';
  char byteTexto[3];
  for (byte i = 0; i < uid->size; i++) {
    snprintf(byteTexto, sizeof(byteTexto), "%02X", uid->uidByte[i]);
    strncat(destino, byteTexto, tam - strlen(destino) - 1);
  }
}

bool tagAutorizado(const char *uidTexto) {
  for (uint8_t i = 0; i < CANTIDAD_TAGS_AUTORIZADOS; i++) {
    if (strcmp(uidTexto, TAGS_AUTORIZADOS[i]) == 0) return true;
  }
  return false;
}

// Prototipos de los actuadores, definidos en el bloque 8.
void concederAcceso();
void denegarAcceso();

void actualizarRC522() {

  // PICC_IsNewCardPresent() y PICC_ReadCardSerial() devuelven de
  // inmediato si no hay tarjeta: no bloquean. Solo cuando hay una
  // tarjeta nueva se entra al bloque de abajo.
  if (!rc522.PICC_IsNewCardPresent()) return;
  if (!rc522.PICC_ReadCardSerial())   return;

  uidATexto(&rc522.uid, ultimoUID, sizeof(ultimoUID));

  if (tagAutorizado(ultimoUID)) {
    estadoRFID = LECTURA_OK;
    concederAcceso();
  } else {
    estadoRFID = LECTURA_OK;   // el lector funcionó bien; el tag es el que no está autorizado
    denegarAcceso();
  }

  Serial.print("RFID UID leido: ");
  Serial.println(ultimoUID);

  // Libera la tarjeta para poder detectar la siguiente.
  rc522.PICC_HaltA();
  rc522.PCD_StopCrypto1();
}


/* =====================================================================
   8. ACTUADORES — LEDs y buzzer, sin delay()

   Mismo patrón que todo el resto del firmware: se guarda cuándo se
   encendió cada salida y se apaga comparando contra millis(), nunca
   con una espera bloqueante. Un delay() aquí congelaría al MAX30102
   igual que lo haría en cualquier otro módulo.
   ===================================================================== */

bool     ledVerdeEncendido = false;
uint32_t tLedVerde = 0;

bool     ledRojoEncendido = false;
uint32_t tLedRojo = 0;

bool     buzzerEncendido = false;
uint32_t tBuzzer = 0;

void concederAcceso() {
  digitalWrite(PIN_LED_VERDE, HIGH);
  ledVerdeEncendido = true;
  tLedVerde = millis();
}

void denegarAcceso() {
  digitalWrite(PIN_LED_ROJO, HIGH);
  ledRojoEncendido = true;
  tLedRojo = millis();

  digitalWrite(PIN_BUZZER, HIGH);
  buzzerEncendido = true;
  tBuzzer = millis();
}

// Se llama en cada vuelta del loop: apaga cada salida cuando se
// cumple su tiempo, sin detener nada mientras tanto.
void actualizarActuadores() {
  uint32_t ahora = millis();

  if (ledVerdeEncendido && (ahora - tLedVerde >= T_INDICACION_MS)) {
    digitalWrite(PIN_LED_VERDE, LOW);
    ledVerdeEncendido = false;
  }

  if (ledRojoEncendido && (ahora - tLedRojo >= T_INDICACION_MS)) {
    digitalWrite(PIN_LED_ROJO, LOW);
    ledRojoEncendido = false;
  }

  if (buzzerEncendido && (ahora - tBuzzer >= T_ALARMA_MS)) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerEncendido = false;
  }
}


/* =====================================================================
   9. DIAGNÓSTICO
   ===================================================================== */

void imprimirDiagnostico() {
  Serial.print("[t="); Serial.print(millis()); Serial.print(" ms] ");

  Serial.print("MAX30102: "); Serial.print(nombreEstado(medIR.estado));
  Serial.print("  IR="); Serial.print(medIR.valor, 0);

  Serial.print("  | MPU6050: "); Serial.print(nombreEstado(medAcel.estado));
  Serial.print("  |a|="); Serial.print(medAcel.valor, 2); Serial.print(" g");

  Serial.print("  | DS18B20: "); Serial.print(nombreEstado(medTemp.estado));
  Serial.print("  T="); Serial.print(medTemp.valor, 1); Serial.print(" C");

  Serial.print("  | RFID: "); Serial.print(nombreEstado(estadoRFID));
  Serial.print("  ultimo="); Serial.println(ultimoUID);
}


/* =====================================================================
   10. SETUP
   ===================================================================== */

void setup() {
  Serial.begin(SERIAL_BAUDIOS);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO,  OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_ROJO,  LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);

  Serial.println();
  Serial.println("=== ETAPA 4: + RC522 por SPI y control de acceso ===");

  Serial.print("MAX30102: "); Serial.println(inicializarMAX30102() ? "inicializado" : "NO responde");
  Serial.print("MPU6050 : "); Serial.println(inicializarMPU6050()  ? "inicializado" : "NO responde");
  Serial.print("DS18B20 : "); Serial.println(inicializarDS18B20()  ? "inicializado" : "NO responde");
  Serial.print("RC522   : "); Serial.println(inicializarRC522()    ? "inicializado" : "NO responde");

  Serial.println("-----------------------------------------------------");
  Serial.println("Acerca un tag para ver su UID en consola.");
}


/* =====================================================================
   11. LOOP — planificador no bloqueante
   ===================================================================== */

void loop() {
  uint32_t ahora = millis();

  if (ahora - tMax >= PERIODO_MAX30102) {
    tMax = ahora;
    actualizarMAX30102();
  }

  if (ahora - tMpu >= PERIODO_MPU6050) {
    tMpu = ahora;
    actualizarMPU6050();
  }

  actualizarDS18B20();   // su propia máquina de estados regula el tiempo

  if (ahora - tRfid >= PERIODO_RC522) {
    tRfid = ahora;
    actualizarRC522();
  }

  actualizarActuadores();   // debe revisarse en cada vuelta, no con período fijo

  if (ahora - tDiag >= PERIODO_DIAGNOSTICO) {
    tDiag = ahora;
    imprimirDiagnostico();
  }
}
