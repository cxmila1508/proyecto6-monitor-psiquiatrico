/* =====================================================================
   ETAPA 1 - Escaner I2C / validacion fisica del MAX30102
   Proyecto 6: Monitor Inalambrico de Seguridad e Intervencion
               Preventiva en Unidades de Internacion Psiquiatrica

   Objetivo de esta etapa: comprobar que el MAX30102 esta electricamente
   presente en el bus I2C ANTES de instalar cualquier libreria de sensor.

   Herramienta de diagnostico. NO es firmware de proyecto: no lo cuentes
   como arquitectura propia en la rubrica. Reescribe estos comentarios
   con tus palabras y asegurate de poder explicar cada linea.
   ===================================================================== */

#include <Wire.h>

// ---------- Configuracion de hardware ----------
static const uint8_t  PIN_SDA        = 21;
static const uint8_t  PIN_SCL        = 22;
static const uint32_t FREC_I2C_HZ    = 100000UL;  // 100 kHz: margen amplio de pull-up

// Direcciones esperadas del kit (7 bits)
static const uint8_t DIR_MAX30102    = 0x57;
static const uint8_t DIR_MPU6050     = 0x68;      // 0x69 si AD0 va a VCC

// Registro de identificacion del MAX30102. Valor esperado 0x15.
// VERIFICAR contra el datasheet antes de citarlo en el informe.
static const uint8_t REG_PART_ID     = 0xFF;
static const uint8_t PART_ID_ESPERADO = 0x15;

// ---------- Temporizacion no bloqueante ----------
// Sin delay(): el lazo queda libre para que en etapas siguientes
// convivan otras tareas (requisito de la Fase 2 / FreeRTOS).
static const uint32_t PERIODO_ESCANEO_MS = 2000;
static uint32_t t_ultimo_escaneo = 0;

// ---------- Prototipos ----------
void escanearBus();
bool leerRegistro(uint8_t direccion, uint8_t registro, uint8_t &valor);
void reportarDiagnostico(uint8_t dispositivos, bool maxPresente);

void setup() {
  Serial.begin(115200);

  // Wire.begin(SDA, SCL, frecuencia) es la sobrecarga propia del core ESP32.
  Wire.begin(PIN_SDA, PIN_SCL, FREC_I2C_HZ);

  Serial.println();
  Serial.println(F("=== ETAPA 1: escaner I2C ==="));
  Serial.print(F("SDA=GPIO"));  Serial.print(PIN_SDA);
  Serial.print(F("  SCL=GPIO")); Serial.print(PIN_SCL);
  Serial.print(F("  f=")); Serial.print(FREC_I2C_HZ / 1000); Serial.println(F(" kHz"));
}

void loop() {
  uint32_t ahora = millis();

  // Comparacion por resta: sobrevive al desbordamiento de millis()
  // (~49.7 dias) porque la aritmetica es sin signo.
  if (ahora - t_ultimo_escaneo >= PERIODO_ESCANEO_MS) {
    t_ultimo_escaneo = ahora;
    escanearBus();
  }

  // Aqui NO hay delay(): el resto del lazo queda disponible.
}

/* Recorre el rango de direcciones validas de 7 bits (0x08..0x77).
   Las direcciones 0x00-0x07 y 0x78-0x7F estan reservadas por la
   especificacion I2C y no deben sondearse. */
void escanearBus() {
  uint8_t dispositivos = 0;
  bool maxPresente = false;

  Serial.println(F("\n--- escaneando ---"));

  for (uint8_t dir = 0x08; dir <= 0x77; dir++) {
    Wire.beginTransmission(dir);
    uint8_t err = Wire.endTransmission();   // 0 = el esclavo respondio ACK

    if (err == 0) {
      dispositivos++;
      Serial.print(F("  0x"));
      if (dir < 0x10) Serial.print('0');
      Serial.print(dir, HEX);

      if (dir == DIR_MAX30102) { Serial.print(F("  <- MAX30102")); maxPresente = true; }
      if (dir == DIR_MPU6050)  { Serial.print(F("  <- MPU6050"));  }
      Serial.println();
    }
    else if (err == 4) {
      Serial.print(F("  0x")); Serial.print(dir, HEX);
      Serial.println(F("  error desconocido en el bus"));
    }
  }

  // Un ACK solo prueba que algo contesta en esa direccion.
  // Leer el Part ID confirma la identidad del integrado.
  if (maxPresente) {
    uint8_t partId = 0;
    if (leerRegistro(DIR_MAX30102, REG_PART_ID, partId)) {
      Serial.print(F("  PART_ID = 0x"));
      Serial.print(partId, HEX);
      Serial.println(partId == PART_ID_ESPERADO ? F("  OK") : F("  INESPERADO"));
    } else {
      Serial.println(F("  ACK en 0x57 pero la lectura de registro fallo"));
    }
  }

  reportarDiagnostico(dispositivos, maxPresente);
}

/* Lectura de un registro de 8 bits: escritura del puntero de registro
   sin STOP (repeated start) y luego lectura de un byte.
   Devuelve true solo si la transaccion completa fue exitosa. */
bool leerRegistro(uint8_t direccion, uint8_t registro, uint8_t &valor) {
  Wire.beginTransmission(direccion);
  Wire.write(registro);
  if (Wire.endTransmission(false) != 0) return false;   // false = repeated start

  if (Wire.requestFrom(direccion, (uint8_t)1) != 1) return false;
  valor = Wire.read();
  return true;
}

/* Traduce el resultado a la siguiente accion fisica a revisar.
   Sigue la tabla sintoma -> causa de la Leccion 2. */
void reportarDiagnostico(uint8_t dispositivos, bool maxPresente) {
  Serial.print(F("Dispositivos detectados: "));
  Serial.println(dispositivos);

  if (dispositivos == 0) {
    Serial.println(F("REVISAR: 1) VIN del modulo a 3.3 V"));
    Serial.println(F("         2) GND comun ESP32-modulo (continuidad < 1 ohm)"));
    Serial.println(F("         3) SDA y SCL en reposo deben medir ~3.3 V"));
    Serial.println(F("         4) SDA/SCL no invertidos"));
  } else if (!maxPresente) {
    Serial.println(F("Hay trafico en el bus pero 0x57 no responde:"));
    Serial.println(F("revisar alimentacion del MAX30102 y su regulador."));
  } else {
    Serial.println(F("ETAPA 1 SUPERADA -> hacer commit antes de agregar el MPU6050."));
  }
}
