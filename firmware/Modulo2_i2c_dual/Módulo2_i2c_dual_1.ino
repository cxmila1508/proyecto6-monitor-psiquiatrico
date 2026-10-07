/*Módulo2 — Bus I2C compartido: MAX30102 + MPU6050

   Proyecto 6: Monitor Inalámbrico de Seguridad e Intervención
               Preventiva en Unidades de Internación Psiquiátrica

   Objetivo: los dos sensores I2C leyendo de forma simultánea, sin
   funciones bloqueantes, y reportando su estado por separado*/

#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>


/*1. CONFIGURACIÓN PARA EL HARDWARE*/
// ---------- Bus I2C ----------
#define PIN_SDA            21
#define PIN_SCL            22
#define FREC_I2C_HZ        100000UL
#define DIR_MAX30102       0x57
#define DIR_MPU6050        0x68     
// ---------- Períodos de muestreo, en milisegundos ----------
#define PERIODO_MAX30102    10       //100 Hz
#define PERIODO_MPU6050     10       //100 Hz
#define PERIODO_DIAGNOSTICO 1000
// ---------- Política de reintento ----------
#define UMBRAL_FALLOS        5       //fallos seguidos antes de marcar ERROR
// ---------- Rangos de validez ----------
#define IR_MINIMO_CONTACTO  50000UL  //por debajo: no hay dedo sobre el sensor
#define IR_MAXIMO           262143UL //saturación del ADC de 18 bits
#define ACEL_MAXIMA_G        16.0    //fondo de escala del MPU6050
#define ACEL_REPOSO_MIN_G     0.5    //magnitud mínima creíble (gravedad)
#define SERIAL_BAUDIOS      115200


/*2. MODELO DE DATOS*/
enum EstadoModulo {
  NO_INICIALIZADO,   //todavía no se intentó abrir el dispositivo
  LECTURA_OK,        //última operación exitosa
  ERROR_BUS,         //el dispositivo no responde
  FUERA_DE_RANGO,    //responde, pero el valor no es plausible
  SIN_DATO           //inicializado, aún sin primera medición
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
    case LECTURA_OK:              return "OK";
    case ERROR_BUS:       return "ERROR_BUS";
    case FUERA_DE_RANGO:  return "FUERA_RANGO";
    case SIN_DATO:        return "SIN_DATO";
    default:              return "?";
  }
}
/* 3. VALIDACIÓN*/

EstadoModulo validarMAX30102(uint32_t ir) {

  //Caso 1: el sensor no responde en absoluto. Cuando el bus falla o la FIFO no entrega nada, la librería devuelve consistentemente 0. Un dedo real sobre el sensor nunca produce una lectura de exactamente cero.
  if (ir == 0) {
    return ERROR_BUS;
  }
  //Caso 2: el ADC llegó al tope de su rango (18 bits = 262143). El sensor está funcionando, pero saturado: problema de
  //configuración o de contacto excesivo, no una falla de bus.
  if (ir >= IR_MAXIMO) {
    return FUERA_DE_RANGO;
  }
  //Caso 3: el sensor responde, pero por debajo del umbral de contacto. No hay un dedo apoyado; el dispositivo funciona bien.
  if (ir < IR_MINIMO_CONTACTO) {
    return FUERA_DE_RANGO;
  }
  return LECTURA_OK;
}

EstadoModulo validarMPU6050(float ax, float ay, float az) {
  float magnitud = sqrt(ax*ax + ay*ay + az*az);

  //Caso 1: las tres componentes en cero exacto. Un MPU6050 real, incluso perfectamente quieto, siempre mide la aceleración de la gravedad (~1 g) en alguna combinación de ejes. 0.0 exacto en los tres solo ocurre si el bus no está respondiendo.
  if (ax == 0.0 && ay == 0.0 && az == 0.0) {
    return ERROR_BUS;
  }
  //Caso 2: la magnitud supera el fondo de escala configurado.  No es fisiológicamente plausible: más probable un golpe al
  //sensor o un error de lectura que movimiento real del paciente.
  if (magnitud > ACEL_MAXIMA_G) {
    return FUERA_DE_RANGO;
  }
  //Caso 3: magnitud sospechosamente baja, más probable ruido del
  //sensor que una medición real, incluso en reposo total.
  if (magnitud < ACEL_REPOSO_MIN_G) {
    return FUERA_DE_RANGO;
  }
  return LECTURA_OK;
}

EstadoModulo validarDS18B20(float temperatura) {
  if (temperatura <= -126.0 || fabs(temperatura - 85.0) < 0.01) {
    return ERROR_BUS;
  }
  if (temperatura < 20.0 || temperatura > 45.0) {
    return FUERA_DE_RANGO;
  }
  return LECTURA_OK;
}

/*4. ESTADO GLOBAL*/

MAX30105         max30102;
Adafruit_MPU6050 mpu;
Medicion medIR   = {0, "cuentas", 0, "MAX30102", false, NO_INICIALIZADO};
Medicion medAcel = {0, "g",       0, "MPU6050",  false, NO_INICIALIZADO};
uint8_t fallosMax = 0;
uint8_t fallosMpu = 0;
uint32_t tMax  = 0;
uint32_t tMpu  = 0;
uint32_t tDiag = 0;


/*5. MÓDULO MAX30102*/
bool inicializarMAX30102() {
  if (!max30102.begin(Wire, I2C_SPEED_STANDARD)) {
    medIR.estado = ERROR_BUS;
    return false;
  }
  //Parámetros
  max30102.setup(0x1F, 4, 2, 100, 411, 4096);
  medIR.estado = SIN_DATO;
  return true;
}
void actualizarMAX30102() {
  //check() consulta la FIFO sin bloquear. getIR() en cambio espera
  //internamente hasta 250 ms, lo que congelaría al MPU6050.
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

  if (estado == ERROR_BUS) fallosMax++;
  else                     fallosMax = 0;
}


/*6. MÓDULO MPU6050*/
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
  if (estado == ERROR_BUS) fallosMpu++;
  else                     fallosMpu = 0;
}


/*7. DIAGNÓSTICO*/

void imprimirDiagnostico() {
  Serial.print("[t=");
  Serial.print(millis());
  Serial.print(" ms] MAX30102: ");
  Serial.print(nombreEstado(medIR.estado));
  Serial.print("  IR=");
  Serial.print(medIR.valor, 0);
  Serial.print("   | MPU6050: ");
  Serial.print(nombreEstado(medAcel.estado));
  Serial.print("  |a|=");
  Serial.print(medAcel.valor, 2);
  Serial.println(" g");
}


/* 8. SETUP*/

void setup() {
  Serial.begin(SERIAL_BAUDIOS);
  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);
  Serial.println();
  Serial.println("=== ETAPA 2: bus I2C compartido ===");
  //Que un sensor falle no puede detener al sistema completo.
  Serial.print("MAX30102: ");
  Serial.println(inicializarMAX30102() ? "inicializado" : "NO responde");
  Serial.print("MPU6050 : ");
  Serial.println(inicializarMPU6050() ? "inicializado" : "NO responde");
  Serial.println("-----------------------------------");
}

/*9. LOOP — planificador no bloqueante*/

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
  if (ahora - tDiag >= PERIODO_DIAGNOSTICO) {
    tDiag = ahora;
    imprimirDiagnostico();
  }
}
