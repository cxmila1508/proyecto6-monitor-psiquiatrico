/* MÓDULO 6 — Nodo IoT: transmisión cifrada por Wi-Fi (Hito 2)*/
#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <MFRC522.h>
#include <math.h>
#include <stdarg.h>
#include <sys/time.h>
#include <atomic>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "mbedtls/md.h"
#include "mbedtls/aes.h"
#include "mbedtls/base64.h"
#include "secrets.h"
#include "ca_cert.h"


/*1. CONFIGURACIÓN*/

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
#define NUCLEO_SERVICIOS     0   //el Wi-Fi del ESP32 corre en el núcleo 0
#define PRIO_I2C             3
#define PRIO_DS18B20         2
#define PRIO_RFID            2
#define PRIO_DIAGNOSTICO     1
#define PRIO_TRANSMISION     1   
#define PILA_TAREA        4096   
#define PILA_TRANSMISION 12288   
#define PERIODO_I2C_MS          10
#define PERIODO_DS18B20_MS    1000
#define PERIODO_RFID_MS         50
#define PERIODO_DIAGNOSTICO_MS 1000   //es el período de muestreo hacia el servidor
#define UMBRAL_FALLOS            5
#define UMBRAL_SIN_MUESTRAS     25
#define IR_MINIMO_CONTACTO  50000UL
#define IR_MAXIMO           262143UL
#define ACEL_MAXIMA_G         16.0
#define ACEL_REPOSO_MIN_G      0.5
#define TEMP_MINIMA_C          20.0
#define TEMP_MAXIMA_C          45.0
#define SERIAL_BAUDIOS      115200

// ---------- Red ----------
#define RUTA_API               "/api/v1/lotes"
#define LARGO_COLA_MUESTRAS     120   //2 min de respaldo si cae la red
#define LARGO_COLA_EVENTOS       20
#define MAX_MUESTRAS_POR_LOTE    10
#define MAX_EVENTOS_POR_LOTE      5
#define TAM_LOTE_JSON          4096   
#define TAM_SOBRE              6144   
#define LARGO_SAL                16   
//Demostración en consola: cada cuántos lotes se imprime el recorrido
#define DEMO_CRIPTO_CADA          5
#define T_ESPERA_WIFI_MS      15000   //tiempo máximo esperando asociarse
#define T_REINTENTO_MS         3000   //pausa tras un envío fallido
#define TIMEOUT_HTTP_MS        5000
#define EPOCH_VALIDO     1700000000L  //antes de esto, el reloj no está sincronizado

const char* TAGS_AUTORIZADOS[] = {
  "AC1A2D49",
  "239D991A"
};
const uint8_t CANTIDAD_TAGS_AUTORIZADOS =
    sizeof(TAGS_AUTORIZADOS) / sizeof(TAGS_AUTORIZADOS[0]);


/*2. MODELO DE DATOS*/

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
struct Muestra {
  uint32_t     secuencia;
  uint32_t     t_ms;        //millis() del nodo
  int64_t      epoch_ms;    //hora absoluta (NTP); 0 si aún no se sincroniza
  float        ir;
  EstadoModulo eIr;
  float        ax, ay, az, acel;
  EstadoModulo eAcel;
  float        temp;
  EstadoModulo eTemp;
  EstadoModulo eRfid;
};

//Cada lectura de tag es un evento: se envía aparte para no perderlo entre dos muestras.
struct EventoAcceso {
  uint32_t t_ms;
  int64_t  epoch_ms;
  char     uid[21];
  bool     autorizado;
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


/*3. ESTADO COMPARTIDO*/

MAX30105          max30102;
Adafruit_MPU6050  mpu;
OneWire           busOneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&busOneWire);
DeviceAddress     direccionDS18B20;
MFRC522           rc522(PIN_RC522_SS, PIN_RC522_RST);
Medicion     medIR, medAcel, medTemp;
EstadoModulo estadoRFID   = NO_INICIALIZADO;
char         ultimoUID[21] = "ninguno";
const char*  ultimoAcceso  = "-";
float        acelX = 0, acelY = 0, acelZ = 0;   // componentes de la aceleración, en g
SemaphoreHandle_t mutexDatos;
volatile int nucleoI2C = -1, nucleoDS = -1, nucleoRFID = -1, nucleoDiag = -1, nucleoTx = -1;

//Colas entre productores  y la tarea de transmisión.
QueueHandle_t colaMuestras;
QueueHandle_t colaEventos;

uint32_t idArranque = 0;
uint32_t siguienteSecuencia = 0;

//Contadores de la red. Se escriben desde una tarea y se leen desde otra, por eso son atómicos: cada incremento ocurre completo, sin interferencias.
std::atomic<uint32_t> lotesEnviados{0};
std::atomic<uint32_t> lotesFallidos{0};
std::atomic<uint32_t> muestrasDescartadas{0};   // por cola llena
std::atomic<uint32_t> eventosDescartados{0};
volatile uint32_t ultimoRttMs        = 0;
volatile int      ultimoCodigoHttp   = 0;
const char* volatile estadoRed       = "INICIO";

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
//Hora absoluta en ms, o 0 si el NTP todavía no respondió
int64_t epochMs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec < EPOCH_VALIDO) return 0;
  return (int64_t)tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}
/*4. INICIALIZACIÓN DE SENSORES*/

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


/*5. RFID Y ACTUADORES LOCALES*/

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

/*6. CRIPTOGRAFÍA*/

void bytesAHex(const uint8_t *bytes, size_t n, char *destino) {
  static const char HEX_DIG[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    destino[2 * i]     = HEX_DIG[bytes[i] >> 4];
    destino[2 * i + 1] = HEX_DIG[bytes[i] & 0x0F];
  }
  destino[2 * n] = '\0';
}
// HMAC-SHA256
bool hmacSha256(const char *clave, const uint8_t *mensaje, size_t largo, char *hex65) {
  uint8_t digest[32];
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (mbedtls_md_hmac(info, (const uint8_t *)clave, strlen(clave),
                      mensaje, largo, digest) != 0) return false;
  bytesAHex(digest, sizeof(digest), hex65);
  return true;
}
//Seudónimo del tag: primeros 16 hex de SHA-256(sal + UID). El servidor puede reconocer al mismo operario sin que el UID real viaje por la red.
void seudonimoUID(const char *uid, char *hex17) {
  uint8_t digest[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const uint8_t *)SAL_UID, strlen(SAL_UID));
  mbedtls_md_update(&ctx, (const uint8_t *)uid, strlen(uid));
  mbedtls_md_finish(&ctx, digest);
  mbedtls_md_free(&ctx);
  bytesAHex(digest, 8, hex17);
}

// La clave AES viene en hexadecimal desde secrets.h (32 caracteres = 16 bytes)
bool hexABytes(const char *hex, uint8_t *destino, size_t n) {
  if (strlen(hex) != 2 * n) return false;
  for (size_t i = 0; i < n; i++) {
    char par[3] = { hex[2 * i], hex[2 * i + 1], '\0' };
    char *fin;
    destino[i] = (uint8_t)strtoul(par, &fin, 16);
    if (*fin != '\0') return false;
  }
  return true;
}

bool cifrarAesCtr(const uint8_t clave[16], const uint8_t sal[LARGO_SAL],
                  const uint8_t *claro, size_t largo,
                  uint8_t *cifrado) {
  mbedtls_aes_context aes;
  uint8_t contador[16], bloque[16];
  size_t desfase = 0;
  memcpy(contador, sal, 16);          // el contador parte en la sal
  mbedtls_aes_init(&aes);
  bool ok = mbedtls_aes_setkey_enc(&aes, clave, 128) == 0 &&
            mbedtls_aes_crypt_ctr(&aes, largo, &desfase, contador, bloque,
                                  claro, cifrado) == 0;
  mbedtls_aes_free(&aes);
  return ok;
}

// Arma el sobre que viaja por la red
size_t armarSobre(const uint8_t claveAes[16], const char *claro, size_t largo,
                  char *sobre, size_t tam, char *salHex33) {
  static uint8_t cifrado[TAM_LOTE_JSON];
  uint8_t sal[LARGO_SAL];
  esp_fill_random(sal, sizeof(sal));   // RNG por hardware (con el Wi-Fi activo es verdaderamente aleatorio)
  bytesAHex(sal, sizeof(sal), salHex33);
  if (largo > sizeof(cifrado)) return 0;
  if (!cifrarAesCtr(claveAes, sal, (const uint8_t *)claro, largo, cifrado)) return 0;

  int n = snprintf(sobre, tam, "{\"v\":1,\"dispositivo\":\"%s\",\"sal\":\"%s\",\"datos\":\"",
                   DISPOSITIVO_ID, salHex33);
  if (n < 0 || (size_t)n >= tam) return 0;
  size_t escritos = 0;
  if (mbedtls_base64_encode((unsigned char *)sobre + n, tam - n, &escritos,
                            cifrado, largo) != 0) return 0;
  size_t pos = n + escritos;
  if (pos + 3 > tam) return 0;
  memcpy(sobre + pos, "\"}", 3);       // incluye el '\0'
  return pos + 2;
}


/*7. TAREAS DE ADQUISICIÓN */

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
      if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
        acelX = ax;  acelY = ay;  acelZ = az;
        xSemaphoreGive(mutexDatos);
      }
    } else if (++fallosMpu >= UMBRAL_FALLOS) {
      fallosMpu = UMBRAL_FALLOS;
      marcarError(medAcel);
    }
    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_I2C_MS));
  }
}

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

void tareaRFID(void *parametro) {
  nucleoRFID = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();
  for (;;) {
    if (rc522.PICC_IsNewCardPresent() && rc522.PICC_ReadCardSerial()) {
      char uid[21];
      uidATexto(&rc522.uid, uid, sizeof(uid));
      bool autorizado = tagAutorizado(uid);

      //La alarma local se activa aquí mismo, sin esperar a la red.
      if (autorizado) concederAcceso();
      else            denegarAcceso();
      if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
        strncpy(ultimoUID, uid, sizeof(ultimoUID) - 1);
        ultimoUID[sizeof(ultimoUID) - 1] = '\0';
        ultimoAcceso = autorizado ? "CONCEDIDO" : "DENEGADO";
        estadoRFID   = LECTURA_OK;
        xSemaphoreGive(mutexDatos);
      }
.
      EventoAcceso ev;
      ev.t_ms       = millis();
      ev.epoch_ms   = epochMs();
      ev.autorizado = autorizado;
      strncpy(ev.uid, uid, sizeof(ev.uid));
      if (xQueueSend(colaEventos, &ev, 0) != pdTRUE) eventosDescartados++;
      rc522.PICC_HaltA();
      rc522.PCD_StopCrypto1();
    }

    actualizarActuadores();
    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_RFID_MS));
  }
}



void encolarMuestra(const Muestra &m) {
  if (xQueueSend(colaMuestras, &m, 0) == pdTRUE) return;
  Muestra descartada;
  xQueueReceive(colaMuestras, &descartada, 0);
  muestrasDescartadas++;
  xQueueSend(colaMuestras, &m, 0);
}
void tareaDiagnostico(void *parametro) {
  nucleoDiag = xPortGetCoreID();
  TickType_t ultimoDespertar = xTaskGetTickCount();
  bool primeraVez = true;

  for (;;) {
    vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_DIAGNOSTICO_MS));

    if (primeraVez) {
      primeraVez = false;
      Serial.printf("Tareas -> I2C: nucleo %d | DS18B20: nucleo %d | "
                    "RFID: nucleo %d | Diagnostico: nucleo %d | Transmision: nucleo %d\n",
                    nucleoI2C, nucleoDS, nucleoRFID, nucleoDiag, nucleoTx);
    }

    Medicion ir, acel, temp;
    float x, y, z;
    EstadoModulo eRfid;
    char uid[21];
    const char* acceso;

    // Copia rápida bajo mutex; el formateo y el envío se hacen fuera.
    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    ir    = medIR;
    acel  = medAcel;
    x = acelX;  y = acelY;  z = acelZ;
    temp  = medTemp;
    eRfid = estadoRFID;
    strncpy(uid, ultimoUID, sizeof(uid));
    acceso = ultimoAcceso;
    xSemaphoreGive(mutexDatos);

    Muestra m;
    m.secuencia = siguienteSecuencia++;
    m.t_ms      = millis();
    m.epoch_ms  = epochMs();
    m.ir   = ir.valor;    m.eIr   = ir.estado;
    m.ax   = x;  m.ay = y;  m.az = z;
    m.acel = acel.valor;  m.eAcel = acel.estado;
    m.temp = temp.valor;  m.eTemp = temp.estado;
    m.eRfid = eRfid;
    encolarMuestra(m);

    Serial.printf("[t=%lu ms] MAX30102: %s IR=%.0f | MPU6050: %s x=%.2f y=%.2f z=%.2f |a|=%.2f g | "
                  "DS18B20: %s T=%.1f C | RFID: %s ultimo=%s (%s)\n",
                  (unsigned long)millis(),
                  nombreEstado(ir.estado),   ir.valor,
                  nombreEstado(acel.estado), x, y, z, acel.valor,
                  nombreEstado(temp.estado), temp.valor,
                  nombreEstado(eRfid), uid, acceso);
    Serial.printf("           RED: %s | cola=%u | lotes ok=%lu fallidos=%lu | "
                  "HTTP=%d RTT=%lu ms | descartadas=%lu\n",
                  estadoRed, (unsigned)uxQueueMessagesWaiting(colaMuestras),
                  (unsigned long)lotesEnviados, (unsigned long)lotesFallidos,
                  ultimoCodigoHttp, (unsigned long)ultimoRttMs,
                  (unsigned long)muestrasDescartadas);
  }
}


/* 9. TRANSMISIÓN */

bool agregarJson(char *buf, size_t tam, size_t &pos, const char *fmt, ...) {
  if (pos >= tam) return false;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf + pos, tam - pos, fmt, args);
  va_end(args);
  if (n < 0 || (size_t)n >= tam - pos) return false;
  pos += n;
  return true;
}

float finito(float v) { return isfinite(v) ? v : 0.0f; }

size_t armarLote(char *buf, size_t tam,
                 const Muestra *muestras, uint8_t nM,
                 const EventoAcceso *eventos, uint8_t nE) {
  size_t pos = 0;
  bool ok = agregarJson(buf, tam, pos,
      "{\"dispositivo\":\"%s\",\"arranque\":\"%08lx\",\"enviado_epoch_ms\":%lld,"
      "\"descartadas\":%lu,\"muestras\":[",
      DISPOSITIVO_ID, (unsigned long)idArranque, (long long)epochMs(),
      (unsigned long)muestrasDescartadas);

  for (uint8_t i = 0; ok && i < nM; i++) {
    const Muestra &m = muestras[i];
    ok = agregarJson(buf, tam, pos,
        "%s{\"seq\":%lu,\"t_ms\":%lu,\"epoch_ms\":%lld,"
        "\"ir\":{\"v\":%.0f,\"e\":\"%s\"},"
        "\"acel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"mag\":%.3f,\"e\":\"%s\"},"
        "\"temp\":{\"v\":%.2f,\"e\":\"%s\"},\"rfid\":\"%s\"}",
        i ? "," : "", (unsigned long)m.secuencia, (unsigned long)m.t_ms, (long long)m.epoch_ms,
        finito(m.ir), nombreEstado(m.eIr),
        finito(m.ax), finito(m.ay), finito(m.az), finito(m.acel), nombreEstado(m.eAcel),
        finito(m.temp), nombreEstado(m.eTemp), nombreEstado(m.eRfid));
  }

  ok = ok && agregarJson(buf, tam, pos, "],\"eventos\":[");
  for (uint8_t i = 0; ok && i < nE; i++) {
    char seudonimo[17];
    seudonimoUID(eventos[i].uid, seudonimo);
    ok = agregarJson(buf, tam, pos,
        "%s{\"t_ms\":%lu,\"epoch_ms\":%lld,\"uid_hash\":\"%s\",\"acceso\":\"%s\"}",
        i ? "," : "", (unsigned long)eventos[i].t_ms, (long long)eventos[i].epoch_ms,
        seudonimo, eventos[i].autorizado ? "CONCEDIDO" : "DENEGADO");
  }
  ok = ok && agregarJson(buf, tam, pos, "]}");
  return ok ? pos : 0;
}

//Asocia al AP sin bloquear al resto del sistema: solo esta tarea espera, y lo hace cediendo la CPU con vTaskDelay.
bool conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  estadoRed = "CONECTANDO_WIFI";
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_CLAVE);
  uint32_t inicio = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - inicio > T_ESPERA_WIFI_MS) {
      estadoRed = "SIN_WIFI";
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  Serial.printf("Wi-Fi conectado: IP %s, RSSI %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  //NTP en segundo plano: el reloj se ajusta solo cuando responda.
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  return true;
}

void tareaTransmision(void *parametro) {
  nucleoTx = xPortGetCoreID();

  static WiFiClientSecure clienteTLS;
  static HTTPClient       http;
  static char             lote[TAM_LOTE_JSON];   
  static char             sobre[TAM_SOBRE];      
  clienteTLS.setCACert(CA_CERT);
  http.setReuse(true);

  static uint8_t claveAes[16];
  if (!hexABytes(CLAVE_AES, claveAes, sizeof(claveAes))) {
    Serial.println("ERROR: CLAVE_AES en secrets.h debe tener 32 caracteres hex. Transmision detenida.");
    estadoRed = "SIN_CLAVE_AES";
    vTaskDelete(NULL);   //la adquisición y la alarma local siguen funcionando
  }

  static Muestra      pendM[MAX_MUESTRAS_POR_LOTE];
  static EventoAcceso pendE[MAX_EVENTOS_POR_LOTE];
  uint8_t nM = 0, nE = 0;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          //menos latencia; el nodo está enchufado
  WiFi.setAutoReconnect(true);

  for (;;) {
    if (!conectarWiFi()) {
      vTaskDelay(pdMS_TO_TICKS(T_REINTENTO_MS));
      continue;
    }
    //Espera la próxima muestra sin consumir CPU (máx. 2 s).
    if (nM == 0 && nE == 0) {
      Muestra tmp;
      if (xQueuePeek(colaMuestras, &tmp, pdMS_TO_TICKS(2000)) != pdTRUE &&
          uxQueueMessagesWaiting(colaEventos) == 0) continue;
    }
    while (nM < MAX_MUESTRAS_POR_LOTE && xQueueReceive(colaMuestras, &pendM[nM], 0) == pdTRUE) nM++;
    while (nE < MAX_EVENTOS_POR_LOTE  && xQueueReceive(colaEventos,  &pendE[nE], 0) == pdTRUE) nE++;
    if (nM == 0 && nE == 0) continue;


    size_t largoClaro = armarLote(lote, sizeof(lote), pendM, nM, pendE, nE);
    char salHex[2 * LARGO_SAL + 1];
    size_t largo = largoClaro ? armarSobre(claveAes, lote, largoClaro, sobre, sizeof(sobre), salHex) : 0;
    char firma[65];
    if (largo == 0 || !hmacSha256(CLAVE_HMAC, (const uint8_t *)sobre, largo, firma)) {
      Serial.println("ERROR: el lote no cabe en el buffer; se descarta");
      nM = nE = 0;
      continue;
    }

#if DEMO_CRIPTO_CADA > 0
    if (lotesEnviados % DEMO_CRIPTO_CADA == 0) {
      Serial.println("---------------- DEMO CRIPTO (lo que sale del ESP32) ----------------");
      Serial.printf("[1] Texto claro (%u B): %.110s...\n", (unsigned)largoClaro, lote);
      Serial.printf("[2] Sal aleatoria     : %s\n", salHex);
      Serial.printf("[3] En el aire (%u B) : %.110s...\n", (unsigned)largo, sobre);
      Serial.printf("[4] HMAC-SHA256       : %s\n", firma);
      Serial.println("--------------------------------------------------------------------");
    }
#endif

    estadoRed = "ENVIANDO";
    uint32_t t0 = millis();
    int codigo = -1;
    if (http.begin(clienteTLS, SERVIDOR_HOST, SERVIDOR_PUERTO, RUTA_API, true)) {
      http.setConnectTimeout(TIMEOUT_HTTP_MS);
      http.setTimeout(TIMEOUT_HTTP_MS);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-Dispositivo", DISPOSITIVO_ID);
      http.addHeader("X-Firma", firma);
      codigo = http.POST((uint8_t *)sobre, largo);
      http.end();
    }
    ultimoRttMs      = millis() - t0;
    ultimoCodigoHttp = codigo;

    if (codigo == 200) {
      lotesEnviados++;
      estadoRed = "OK";
      nM = nE = 0;
    } else if (codigo == 400) {
      //El servidor dice que el lote está mal formado: reenviarlo no sirve.
      lotesFallidos++;
      estadoRed = "LOTE_RECHAZADO";
      nM = nE = 0;
    } else {
      lotesFallidos++;
      if (codigo == 401)   estadoRed = "FIRMA_RECHAZADA";
      else if (codigo < 0) estadoRed = "SIN_SERVIDOR";
      else                 estadoRed = "ERROR_HTTP";
      char error[100];
      if (clienteTLS.lastError(error, sizeof(error)) != 0)
        Serial.printf("TLS: %s\n", error);
      clienteTLS.stop();   // la próxima vez parte con un handshake limpio
      vTaskDelay(pdMS_TO_TICKS(T_REINTENTO_MS));
    }
  }
}


/* 10. ARRANQUE*/

void setup() {
  Serial.begin(SERIAL_BAUDIOS);
  pinMode(PIN_LED_VERDE, OUTPUT);  digitalWrite(PIN_LED_VERDE, LOW);
  pinMode(PIN_LED_ROJO,  OUTPUT);  digitalWrite(PIN_LED_ROJO,  LOW);
  pinMode(PIN_BUZZER,    OUTPUT);  digitalWrite(PIN_BUZZER,    LOW);
  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);

  Serial.println();
  Serial.println("=== ETAPA 6: Nodo IoT, transmision TLS en el nucleo 0 ===");

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

  mutexDatos   = xSemaphoreCreateMutex();
  colaMuestras = xQueueCreate(LARGO_COLA_MUESTRAS, sizeof(Muestra));
  colaEventos  = xQueueCreate(LARGO_COLA_EVENTOS,  sizeof(EventoAcceso));
  idArranque   = esp_random();

  Serial.printf("Dispositivo %s, arranque %08lx, servidor https://%s:%d%s\n",
                DISPOSITIVO_ID, (unsigned long)idArranque,
                SERVIDOR_HOST, SERVIDOR_PUERTO, RUTA_API);

  xTaskCreatePinnedToCore(tareaI2C,         "I2C",        PILA_TAREA,       NULL,  PRIO_I2C,         NULL,   NUCLEO_ADQUISICION);
  xTaskCreatePinnedToCore(tareaDS18B20,     "DS18B20",    PILA_TAREA,       NULL,  PRIO_DS18B20,     NULL,   NUCLEO_SERVICIOS);
  xTaskCreatePinnedToCore(tareaRFID,        "RFID",       PILA_TAREA,       NULL,  PRIO_RFID,        NULL,   NUCLEO_SERVICIOS);
  xTaskCreatePinnedToCore(tareaDiagnostico, "Diagnostico",PILA_TAREA,       NULL,  PRIO_DIAGNOSTICO, NULL,   NUCLEO_SERVICIOS);
  xTaskCreatePinnedToCore(tareaTransmision, "Transmision",PILA_TRANSMISION, NULL,  PRIO_TRANSMISION, NULL,   NUCLEO_SERVICIOS);

  Serial.println("Tareas creadas. Acerca un tag para ver su UID.");
}

void loop() {
  vTaskDelete(NULL);
}
