/* =====================================================================
   ETAPA 5 — Migración a FreeRTOS

   Proyecto 6: Monitor Inalámbrico de Seguridad e Intervención
               Preventiva en Unidades de Internación Psiquiátrica

   Misma funcionalidad que la etapa 4 (4 sensores + control de acceso),
   pero cada módulo corre en su propia tarea de FreeRTOS, fijada a un
   núcleo del ESP32:

     Núcleo 1 (adquisición rápida): tareaI2C      prioridad 3, 10 ms
     Núcleo 0 (servicios):          tareaDS18B20  prioridad 2, 1000 ms
                                    tareaRFID     prioridad 2, 50 ms
                                    tareaDiagnostico prioridad 1, 1000 ms

   No usa delay(). Las esperas se hacen con vTaskDelay / vTaskDelayUntil,
   que suspenden SOLO a la tarea que las llama.

   Librerías: las mismas de la etapa 4.
   Placa: ESP32 Dev Module
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

// ---------- Buses ----------
#define PIN_SDA             21
#define PIN_SCL             22
#define FREC_I2C_HZ         100000UL
#define DIR_MPU6050         0x68

#define PIN_ONEWIRE          4
#define RESOLUCION_DS18B20  12
#define TIEMPO_CONVERSION_DS18B20  750   // ms a 12 bits

#define PIN_RC522_SS         5
#define PIN_RC522_RST       27

// ---------- Actuadores ----------
#define PIN_LED_VERDE       25
#define PIN_LED_ROJO        26
#define PIN_BUZZER          33
#define T_INDICACION_MS    800
#define T_ALARMA_MS        400

// ---------- FreeRTOS: núcleos, prioridades, pila y períodos ----------
#define NUCLEO_ADQUISICION   1
#define NUCLEO_SERVICIOS     0

#define PRIO_I2C             3
#define PRIO_DS18B20         2
#define PRIO_RFID            2
#define PRIO_DIAGNOSTICO     1

#define PILA_TAREA        4096   // bytes

#define PERIODO_I2C_MS          10
#define PERIODO_DS18B20_MS    1000
#define PERIODO_RFID_MS         50
#define PERIODO_DIAGNOSTICO_MS 1000

// ---------- Política de fallos ----------
#define UMBRAL_FALLOS            5
// El MAX30102 entrega muestras a menor ritmo que el sondeo (ver
// max30102.setup). 25 ciclos de 10 ms = 250 ms sin ninguna muestra
// nueva se considera falla del sensor.
#define UMBRAL_SIN_MUESTRAS     25

// ---------- Rangos de validez ----------
#define IR_MINIMO_CONTACTO  50000UL
#define IR_MAXIMO           262143UL
#define ACEL_MAXIMA_G         16.0
#define ACEL_REPOSO_MIN_G      0.5
#define TEMP_MINIMA_C          20.0
#define TEMP_MAXIMA_C          45.0

#define SERIAL_BAUDIOS      115200


/* =====================================================================
   2. TAGS AUTORIZADOS
   Reemplazar por los UID reales leídos en el laboratorio.
   ===================================================================== */

const char* TAGS_AUTORIZADOS[] = {
  "A3B21C04",
  "8F1E9D22"
};
const uint8_t CANTIDAD_TAGS_AUTORIZADOS =
    sizeof(TAGS_AUTORIZADOS) / sizeof(TAGS_AUTORIZADOS[0]);


/* =====================================================================
   3. MODELO DE DATOS (igual que la etapa 4)
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
    case LECTURA_OK:      return "OK";
    case ERROR_BUS:       return "ERROR_BUS";
    case FUERA_DE_RANGO:  return "FUERA_RANGO";
    case SIN_DATO:        return "SIN_DATO";
    default:              return "?";
  }
}

// Arma una medición con su contexto completo.
Medicion crearMedicion(float valor, const char* unidad,
                       const char* origen, EstadoModulo estado) {
  Medicion m;
  m.valor        = valor;
  m.unidad       = unidad;
  m.marca_tiempo = (uint32_t)millis();
  m.origen       = origen;
  m.valida       = (estado == LECTURA_OK);
  m.estado       = estado;
  return m;
}


/* =====================================================================
   4. VALIDACIÓN (igual que la etapa 4)
   ===================================================================== */

EstadoModulo validarMAX30102(uint32_t ir) {
  if (ir == 0)                 return ERROR_BUS;
  if (ir >= IR_MAXIMO)         return FUERA_DE_RANGO;
  if (ir < IR_MINIMO_CONTACTO) return FUERA_DE_RANGO;
  return LECTURA_OK;
}

EstadoModulo validarMPU6050(float ax, float ay, float az) {
  float magnitud = sqrtf(ax * ax + ay * ay + az * az);
  if (ax == 0.0f && ay == 0.0f && az == 0.0f) return ERROR_BUS;
  if (magnitud > ACEL_MAXIMA_G)               return FUERA_DE_RANGO;
  if (magnitud < ACEL_REPOSO_MIN_G)           return FUERA_DE_RANGO;
  return LECTURA_OK;
}

EstadoModulo validarDS18B20(float temperatura) {
  if (temperatura <= -126.0f || fabsf(temperatura - 85.0f) < 0.01f) return ERROR_BUS;
  if (temperatura < TEMP_MINIMA_C || temperatura > TEMP_MAXIMA_C)   return FUERA_DE_RANGO;
  return LECTURA_OK;
}


/* =====================================================================
   5. ESTADO GLOBAL Y DATOS COMPARTIDOS ENTRE TAREAS
   ===================================================================== */

MAX30105          max30102;
Adafruit_MPU6050  mpu;
OneWire           busOneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&busOneWire);
DeviceAddress     direccionDS18B20;
MFRC522           rc522(PIN_RC522_SS, PIN_RC522_RST);

// Datos que escriben las tareas de adquisición y lee el diagnóstico.
// Como las tareas corren en núcleos distintos, TODO acceso a estas
// variables pasa por el mutex.
Medicion     medIR, medAcel, medTemp;
EstadoModulo estadoRFID   = NO_INICIALIZADO;
char         ultimoUID[21] = "ninguno";
const char*  ultimoAcceso  = "-";

SemaphoreHandle_t mutexDatos;

// Núcleo en que quedó corriendo cada tarea (evidencia para la demo).
volatile int nucleoI2C = -1, nucleoDS = -1, nucleoRFID = -1, nucleoDiag = -1;

// Guarda una medición nueva. Si el mutex está ocupado más de 5 ms,
// se descarta esta actualización en vez de bloquear la adquisición.
void guardar(Medicion &destino, const Medicion &nueva) {
  if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
    destino = nueva;
    xSemaphoreGive(mutexDatos);
  }
}

void marcarError(Medicion &destino) {
  if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
    destino.estado = ERROR_BUS;
    destino.valida = false;
    xSemaphoreGive(mutexDatos);
  }
}


/* =====================================================================
   6. INICIALIZACIÓN DE MÓDULOS (se ejecuta antes de crear las tareas)
   ===================================================================== */

bool inicializarMAX30102() {
  if (!max30102.begin(Wire, I2C_SPEED_STANDARD)) return false;
  max30102.setup(0x1F, 4, 2, 100, 411, 4096);
  return true;
}

bool inicializarMPU6050() {
  if (!mpu.begin(DIR_MPU6050, &Wire)) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  return true;
}

bool inicializarDS18B20() {
  ds18b20.begin();
  if (ds18b20.getDeviceCount() == 0)            return false;
  if (!ds18b20.getAddress(direccionDS18B20, 0)) return false;
  ds18b20.setResolution(direccionDS18B20, RESOLUCION_DS18B20);
  ds18b20.setWaitForConversion(false);   // la espera la maneja la tarea
  return true;
}

bool inicializarRC522() {
  SPI.begin();
  rc522.PCD_Init();
  byte version = rc522.PCD_ReadRegister(MFRC522::VersionReg);
  return !(version == 0x00 || version == 0xFF);
}


/* =====================================================================
   7. RFID Y ACTUADORES (solo los usa tareaRFID)
   ===================================================================== */

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

bool     ledVerdeEncendido = false, ledRojoEncendido = false, buzzerEncendido = false;
uint32_t tLedVerde = 0, tLedRojo = 0, tBuzzer = 0;

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

// Apaga cada salida cuando se cumple su tiempo, sin esperar.
void actualizarActuadores() {
  uint32_t ahora = millis();
  if (ledVerdeEncendido && (ahora - tLedVerde >= T_INDICACION_MS)) {
    digitalWrite(PIN_LED_VERDE, LOW);  ledVerdeEncendido = false;
  }
  if (ledRojoEncendido && (ahora - tLedRojo >= T_INDICACION_MS)) {
    digitalWrite(PIN_LED_ROJO, LOW);   ledRojoEncendido = false;
  }
  if (buzzerEncendido && (ahora - tBuzzer >= T_ALARMA_MS)) {
    digitalWrite(PIN_BUZZER, LOW);     buzzerEncendido = false;
  }
}


/* =====================================================================
   8. TAREAS DE FREERTOS
   Cada tarea es un ciclo infinito. vTaskDelayUntil la suspende hasta
   su próximo período exacto, sin acumular desfase, y mientras duerme
   el procesador queda libre para las demás tareas.
   ===================================================================== */

// ---- Núcleo 1: los dos sensores I2C en una sola tarea ----
// Comparten SDA/SCL: al estar en la misma tarea, nunca intentan usar
// el bus al mismo tiempo y no hace falta un mutex para el bus.
void tareaI2C(void *parametro) {
  nucleoI2C = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();
  uint16_t ciclosSinMuestra = 0;
  uint8_t  fallosMpu = 0;
  const float G = 9.80665f;

  for (;;) {
    // --- MAX30102: se vacía la FIFO y se conserva la muestra más reciente
    max30102.check();
    bool hayMuestra = false;
    uint32_t ir = 0;
    while (max30102.available()) {
      ir = max30102.getFIFOIR();
      max30102.nextSample();
      hayMuestra = true;
    }

    if (hayMuestra) {
      ciclosSinMuestra = 0;
      guardar(medIR, crearMedicion((float)ir, "cuentas", "MAX30102",
                                   validarMAX30102(ir)));
    } else if (ciclosSinMuestra < UMBRAL_SIN_MUESTRAS) {
      ciclosSinMuestra++;
      if (ciclosSinMuestra == UMBRAL_SIN_MUESTRAS) marcarError(medIR);
    }

    // --- MPU6050
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      fallosMpu = 0;
      float ax = a.acceleration.x / G;
      float ay = a.acceleration.y / G;
      float az = a.acceleration.z / G;
      guardar(medAcel, crearMedicion(sqrtf(ax * ax + ay * ay + az * az), "g",
                                     "MPU6050", validarMPU6050(ax, ay, az)));
    } else if (++fallosMpu >= UMBRAL_FALLOS) {
      fallosMpu = UMBRAL_FALLOS;
      marcarError(medAcel);
    }

    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_I2C_MS));
  }
}

// ---- Núcleo 0: temperatura ----
// La conversión tarda 750 ms. vTaskDelay duerme SOLO esta tarea:
// el resto del sistema sigue funcionando. Esto reemplaza la máquina
// de estados de la etapa 4.
void tareaDS18B20(void *parametro) {
  nucleoDS = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    ds18b20.requestTemperaturesByAddress(direccionDS18B20);
    vTaskDelay(pdMS_TO_TICKS(TIEMPO_CONVERSION_DS18B20));

    float t = ds18b20.getTempC(direccionDS18B20);
    guardar(medTemp, crearMedicion(t, "C", "DS18B20", validarDS18B20(t)));

    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_DS18B20_MS));
  }
}

// ---- Núcleo 0: control de acceso y actuadores ----
void tareaRFID(void *parametro) {
  nucleoRFID = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();

  for (;;) {
    // Ambas funciones responden de inmediato si no hay tarjeta.
    if (rc522.PICC_IsNewCardPresent() && rc522.PICC_ReadCardSerial()) {
      char uid[21];
      uidATexto(&rc522.uid, uid, sizeof(uid));
      bool autorizado = tagAutorizado(uid);

      if (autorizado) concederAcceso();
      else            denegarAcceso();

      if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
        strncpy(ultimoUID, uid, sizeof(ultimoUID) - 1);
        ultimoUID[sizeof(ultimoUID) - 1] = '\0';
        ultimoAcceso = autorizado ? "CONCEDIDO" : "DENEGADO";
        estadoRFID   = LECTURA_OK;
        xSemaphoreGive(mutexDatos);
      }

      rc522.PICC_HaltA();
      rc522.PCD_StopCrypto1();
    }

    actualizarActuadores();
    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_RFID_MS));
  }
}

// ---- Núcleo 0: diagnóstico por consola ----
// Copia los datos bajo el mutex y los imprime FUERA del mutex, para no
// retener la llave mientras el puerto serie escribe (que es lento).
void tareaDiagnostico(void *parametro) {
  nucleoDiag = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();
  bool primeraVez = true;

  for (;;) {
    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_DIAGNOSTICO_MS));

    if (primeraVez) {
      primeraVez = false;
      Serial.printf("Tareas -> I2C: nucleo %d | DS18B20: nucleo %d | "
                    "RFID: nucleo %d | Diagnostico: nucleo %d\n",
                    nucleoI2C, nucleoDS, nucleoRFID, nucleoDiag);
    }

    Medicion ir, acel, temp;
    EstadoModulo eRfid;
    char uid[21];
    const char* acceso;

    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    ir    = medIR;
    acel  = medAcel;
    temp  = medTemp;
    eRfid = estadoRFID;
    strncpy(uid, ultimoUID, sizeof(uid));
    acceso = ultimoAcceso;
    xSemaphoreGive(mutexDatos);

    Serial.printf("[t=%lu ms] MAX30102: %s IR=%.0f | MPU6050: %s |a|=%.2f g | "
                  "DS18B20: %s T=%.1f C | RFID: %s ultimo=%s (%s)\n",
                  (unsigned long)millis(),
                  nombreEstado(ir.estado),   ir.valor,
                  nombreEstado(acel.estado), acel.valor,
                  nombreEstado(temp.estado), temp.valor,
                  nombreEstado(eRfid), uid, acceso);
  }
}


/* =====================================================================
   9. SETUP: inicializa, luego crea las tareas
   ===================================================================== */

void setup() {
  Serial.begin(SERIAL_BAUDIOS);

  pinMode(PIN_LED_VERDE, OUTPUT);  digitalWrite(PIN_LED_VERDE, LOW);
  pinMode(PIN_LED_ROJO,  OUTPUT);  digitalWrite(PIN_LED_ROJO,  LOW);
  pinMode(PIN_BUZZER,    OUTPUT);  digitalWrite(PIN_BUZZER,    LOW);

  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);

  Serial.println();
  Serial.println("=== ETAPA 5: FreeRTOS, tareas en ambos nucleos ===");

  // Se inicializa todo ANTES de crear las tareas: en este punto solo
  // corre setup(), así que no hay riesgo de accesos simultáneos.
  bool okMax  = inicializarMAX30102();
  bool okMpu  = inicializarMPU6050();
  bool okDs   = inicializarDS18B20();
  bool okRfid = inicializarRC522();

  medIR   = crearMedicion(0, "cuentas", "MAX30102", okMax ? SIN_DATO : ERROR_BUS);
  medAcel = crearMedicion(0, "g",       "MPU6050",  okMpu ? SIN_DATO : ERROR_BUS);
  medTemp = crearMedicion(0, "C",       "DS18B20",  okDs  ? SIN_DATO : ERROR_BUS);
  estadoRFID = okRfid ? SIN_DATO : ERROR_BUS;

  Serial.printf("MAX30102: %s | MPU6050: %s | DS18B20: %s | RC522: %s\n",
                okMax ? "OK" : "NO responde", okMpu ? "OK" : "NO responde",
                okDs  ? "OK" : "NO responde", okRfid ? "OK" : "NO responde");

  mutexDatos = xSemaphoreCreateMutex();

  //                       función           nombre        pila        parám. prioridad         handle  núcleo
  xTaskCreatePinnedToCore(tareaI2C,         "I2C",        PILA_TAREA, NULL,  PRIO_I2C,         NULL,   NUCLEO_ADQUISICION);
  xTaskCreatePinnedToCore(tareaDS18B20,     "DS18B20",    PILA_TAREA, NULL,  PRIO_DS18B20,     NULL,   NUCLEO_SERVICIOS);
  xTaskCreatePinnedToCore(tareaRFID,        "RFID",       PILA_TAREA, NULL,  PRIO_RFID,        NULL,   NUCLEO_SERVICIOS);
  xTaskCreatePinnedToCore(tareaDiagnostico, "Diagnostico",PILA_TAREA, NULL,  PRIO_DIAGNOSTICO, NULL,   NUCLEO_SERVICIOS);

  Serial.println("Tareas creadas. Acerca un tag para ver su UID.");
}


/* =====================================================================
   10. LOOP
   Todo el trabajo vive en las tareas. El loop de Arduino (que también
   es una tarea de FreeRTOS) se elimina a sí mismo para no ocupar CPU.
   ===================================================================== */

void loop() {
  vTaskDelete(NULL);
}
