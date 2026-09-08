/* =====================================================================
   ETAPA 3 — Se agrega el DS18B20 en el bus 1-Wire

   Proyecto 6: Monitor Inalámbrico de Seguridad e Intervención
               Preventiva en Unidades de Internación Psiquiátrica

   Punto de partida: la etapa 2 ya funcionaba (MAX30102 + MPU6050 en
   I2C). Esta etapa solo AGREGA el DS18B20. Si algo de I2C deja de
   funcionar al integrar esto, el problema NO es del DS18B20: revisar
   primero alimentación y GND comunes.

   Librerías nuevas para esta etapa (Herramientas -> Administrar
   bibliotecas):
     - OneWire (Paul Stoffregen)
     - DallasTemperature (Miles Burton)

   Placa: ESP32 Dev Module
   ===================================================================== */

#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>


/* =====================================================================
   1. CONFIGURACIÓN
   ===================================================================== */

// ---------- Bus I2C ----------
#define PIN_SDA            21
#define PIN_SCL            22
#define FREC_I2C_HZ        100000UL

#define DIR_MAX30102       0x57
#define DIR_MPU6050        0x68      // 0x69 si AD0 va a VCC

// ---------- Bus 1-Wire ----------
#define PIN_ONEWIRE         4
#define RESOLUCION_DS18B20  12       // bits. A más bits, más lenta la conversión.

/* Tiempo de conversión según la resolución (datasheet DS18B20):
     9 bits  ->  94 ms
    10 bits  -> 188 ms
    11 bits  -> 375 ms
    12 bits  -> 750 ms
   A 12 bits es donde el bloqueo duele más: por eso es el caso de
   ejemplo del pseudocódigo. */
#define TIEMPO_CONVERSION_DS18B20  750UL

// ---------- Períodos de muestreo, en milisegundos ----------
#define PERIODO_MAX30102     10      // 100 Hz
#define PERIODO_MPU6050      10      // 100 Hz
#define PERIODO_DS18B20     1000     // cada cuánto se INICIA un ciclo de medición
#define PERIODO_DIAGNOSTICO 1000

// ---------- Política de reintento ----------
#define UMBRAL_FALLOS         5

// ---------- Rangos de validez ----------
#define IR_MINIMO_CONTACTO  50000UL
#define IR_MAXIMO           262143UL

#define ACEL_MAXIMA_G         16.0
#define ACEL_REPOSO_MIN_G      0.5

#define TEMP_MINIMA_C          20.0  // rango plausible para piel/periferia
#define TEMP_MAXIMA_C           45.0

#define SERIAL_BAUDIOS       115200


/* =====================================================================
   2. MODELO DE DATOS
   ===================================================================== */

enum EstadoModulo {
  NO_INICIALIZADO,
  LECTURA_OK,        // renombrado desde "OK": choca con una macro del core ESP32
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
   3. VALIDACIÓN
   ===================================================================== */

EstadoModulo validarMAX30102(uint32_t ir) {
  if (ir == 0)              return ERROR_BUS;
  if (ir >= IR_MAXIMO)      return FUERA_DE_RANGO;
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

  // Filtro 1: códigos de error de la librería DallasTemperature.
  // -127.0 = el sensor no respondió al bus.
  //   85.0 = valor de reset: se leyó antes de que la conversión
  //          terminara realmente (o el sensor recién se energizó).
  if (temperatura <= -126.0 || fabs(temperatura - 85.0) < 0.01) {
    return ERROR_BUS;
  }

  // Filtro 2: rango plausible para temperatura periférica de un
  // paciente medida sobre la piel (no es temperatura core/rectal).
  if (temperatura < TEMP_MINIMA_C || temperatura > TEMP_MAXIMA_C) {
    return FUERA_DE_RANGO;
  }

  return LECTURA_OK;
}


/* =====================================================================
   4. ESTADO GLOBAL
   ===================================================================== */

MAX30105         max30102;
Adafruit_MPU6050 mpu;

OneWire           busOneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&busOneWire);
DeviceAddress     direccionDS18B20;   // ROM code de 8 bytes, se lee en inicializarDS18B20()

Medicion medIR    = {0, "cuentas", 0, "MAX30102", false, NO_INICIALIZADO};
Medicion medAcel  = {0, "g",       0, "MPU6050",  false, NO_INICIALIZADO};
Medicion medTemp  = {0, "C",       0, "DS18B20",  false, NO_INICIALIZADO};

uint8_t fallosMax  = 0;
uint8_t fallosMpu  = 0;
uint8_t fallosTemp = 0;

uint32_t tMax  = 0;
uint32_t tMpu  = 0;
uint32_t tDiag = 0;


/* =====================================================================
   5. MÓDULO MAX30102  (sin cambios respecto a la etapa 2)
   ===================================================================== */

bool inicializarMAX30102() {
  if (!max30102.begin(Wire, I2C_SPEED_STANDARD)) {
    medIR.estado = ERROR_BUS;
    return false;
  }
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
  medIR.valor        = (float)ir;
  medIR.marca_tiempo = millis();
  medIR.estado       = estado;
  medIR.valida       = (estado == LECTURA_OK);

  fallosMax = (estado == ERROR_BUS) ? fallosMax + 1 : 0;
}


/* =====================================================================
   6. MÓDULO MPU6050  (sin cambios respecto a la etapa 2)
   ===================================================================== */

bool inicializarMPU6050() {
  if (!mpu.begin(DIR_MPU6050, &Wire)) {
    medAcel.estado = ERROR_BUS;
    return false;
  }
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
  float ax = a.acceleration.x / G;
  float ay = a.acceleration.y / G;
  float az = a.acceleration.z / G;

  EstadoModulo estado = validarMPU6050(ax, ay, az);
  medAcel.valor        = sqrt(ax*ax + ay*ay + az*az);
  medAcel.marca_tiempo = millis();
  medAcel.estado       = estado;
  medAcel.valida       = (estado == LECTURA_OK);

  fallosMpu = (estado == ERROR_BUS) ? fallosMpu + 1 : 0;
}


/* =====================================================================
   7. MÓDULO DS18B20 — máquina de estados no bloqueante

   Este es el módulo nuevo de la etapa 3, y el único que necesita una
   máquina de estados explícita en vez de una simple lectura directa.

   Por qué: pedir una conversión de temperatura y esperar el resultado
   toma hasta 750 ms a 12 bits. Si el firmware esperara ahí, el
   MAX30102 perdería ~75 muestras cada vez. La solución es partir la
   operación en tres pasos que se ejecutan en llamadas SEPARADAS al
   loop(), dejando pasar el tiempo de conversión sin bloquear nada.
   ===================================================================== */

enum EstadoDS18B20 { DS_REPOSO, DS_CONVIRTIENDO };

EstadoDS18B20 estadoDS  = DS_REPOSO;
uint32_t      tSolicitud = 0;

bool inicializarDS18B20() {
  ds18b20.begin();

  if (ds18b20.getDeviceCount() == 0) {
    medTemp.estado = ERROR_BUS;
    return false;
  }

  if (!ds18b20.getAddress(direccionDS18B20, 0)) {
    medTemp.estado = ERROR_BUS;
    return false;
  }

  ds18b20.setResolution(direccionDS18B20, RESOLUCION_DS18B20);

  // Clave: se desactiva la espera bloqueante interna de la librería.
  // Sin esto, requestTemperatures() se comporta como un delay(750).
  ds18b20.setWaitForConversion(false);

  medTemp.estado = SIN_DATO;
  return true;
}

void actualizarDS18B20() {
  uint32_t ahora = millis();

  switch (estadoDS) {

    case DS_REPOSO:
      // Se pide la conversión y se sigue de largo, sin esperar.
      ds18b20.requestTemperaturesByAddress(direccionDS18B20);
      tSolicitud = ahora;
      estadoDS = DS_CONVIRTIENDO;
      break;

    case DS_CONVIRTIENDO:
      // Mientras no se cumpla el tiempo de conversión, no se hace
      // nada: el resto del firmware sigue corriendo con normalidad.
      if (ahora - tSolicitud >= TIEMPO_CONVERSION_DS18B20) {

        float temperatura = ds18b20.getTempC(direccionDS18B20);
        EstadoModulo estado = validarDS18B20(temperatura);

        medTemp.valor        = temperatura;
        medTemp.marca_tiempo = ahora;
        medTemp.estado       = estado;
        medTemp.valida       = (estado == LECTURA_OK);

        fallosTemp = (estado == ERROR_BUS) ? fallosTemp + 1 : 0;
        if (fallosTemp >= UMBRAL_FALLOS) medTemp.estado = ERROR_BUS;

        estadoDS = DS_REPOSO;   // listo para el siguiente ciclo
      }
      break;
  }
}


/* =====================================================================
   8. DIAGNÓSTICO
   ===================================================================== */

void imprimirDiagnostico() {
  Serial.print("[t=");
  Serial.print(millis());
  Serial.print(" ms] MAX30102: ");
  Serial.print(nombreEstado(medIR.estado));
  Serial.print("  IR=");
  Serial.print(medIR.valor, 0);

  Serial.print("  | MPU6050: ");
  Serial.print(nombreEstado(medAcel.estado));
  Serial.print("  |a|=");
  Serial.print(medAcel.valor, 2);
  Serial.print(" g");

  Serial.print("  | DS18B20: ");
  Serial.print(nombreEstado(medTemp.estado));
  Serial.print("  T=");
  Serial.print(medTemp.valor, 1);
  Serial.println(" C");
}


/* =====================================================================
   9. SETUP
   ===================================================================== */

void setup() {
  Serial.begin(SERIAL_BAUDIOS);

  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);

  Serial.println();
  Serial.println("=== ETAPA 3: + DS18B20 en 1-Wire ===");

  Serial.print("MAX30102: ");
  Serial.println(inicializarMAX30102() ? "inicializado" : "NO responde");

  Serial.print("MPU6050 : ");
  Serial.println(inicializarMPU6050() ? "inicializado" : "NO responde");

  Serial.print("DS18B20 : ");
  Serial.println(inicializarDS18B20() ? "inicializado" : "NO responde");

  Serial.println("-------------------------------------");
}


/* =====================================================================
   10. LOOP — planificador no bloqueante

   El DS18B20 se suma a la misma estructura de la etapa 2: se compara
   millis() contra la última marca de tiempo del módulo, sin delay().
   Su período (PERIODO_DS18B20) marca cada cuánto se INICIA un nuevo
   ciclo de conversión, no cuánto dura la lectura.
   ===================================================================== */

uint32_t tTemp = 0;   // nueva marca de tiempo para este módulo

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

  // El DS18B20 se actualiza en cada vuelta del loop, sin período de
  // entrada: la propia máquina de estados regula su temporización
  // interna (ver DS_CONVIRTIENDO). Llamarla seguido no cuesta nada,
  // porque en DS_CONVIRTIENDO solo compara millis() y sale.
  actualizarDS18B20();

  if (ahora - tDiag >= PERIODO_DIAGNOSTICO) {
    tDiag = ahora;
    imprimirDiagnostico();
  }
}
