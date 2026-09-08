# Diseño del firmware — Nodo de adquisición ESP32

Proyecto 6 · Monitor Inalámbrico de Seguridad e Intervención Preventiva
en Unidades de Internación Psiquiátrica

Este documento define la arquitectura **antes** de escribir el código.
El firmware de `firmware/` debe ser una transcripción de lo que sigue.

---

## 1. Principio de diseño

Tres reglas que condicionan todas las decisiones posteriores:

1. **Separar adquisición de comunicación.** En la Fase 2 se agregará Wi-Fi
   con TLS, cuyo handshake congela el microcontrolador entre 2 y 4 segundos.
   Si la adquisición estuviera acoplada a la comunicación, el MAX30102
   perdería muestras. Todo lo que se diseñe hoy debe seguir funcionando
   cuando esa capa se agregue encima.

2. **Ningún módulo puede detener al sistema.** Que el MAX30102 falle no
   significa que el firmware deba detenerse. El sistema debe poder reportar
   `MAX30102: ERROR / MPU6050: OK / DS18B20: OK / RFID: OK`.

3. **Sin funciones bloqueantes.** Cero `delay()`. Toda espera se implementa
   como máquina de estados con marcas de tiempo.

---

## 2. Modelo de datos

Una medición no es un número suelto. Debe llevar su contexto desde el
momento de la adquisición, porque después tendrá que empaquetarse en tramas,
viajar por la red, almacenarse en SQLite3 y graficarse en el dashboard.
Diseñarlo ahora evita rehacer el firmware en la Fase 2.

```
ENUMERACIÓN EstadoModulo:
    NO_INICIALIZADO     // aún no se intentó abrir el dispositivo
    OK                  // última operación exitosa
    ERROR_BUS           // el dispositivo no responde
    FUERA_DE_RANGO      // responde, pero el valor no es fisiológicamente válido
    SIN_DATO            // inicializado, aún sin primera medición

ESTRUCTURA Medicion:
    valor           : real
    unidad          : texto           // "C", "bpm", "g", "rad/s"
    marca_tiempo    : entero          // ms desde el arranque
    origen          : texto           // "DS18B20", "MAX30102", ...
    valida          : booleano
    estado          : EstadoModulo
```

> **Decisión a discutir (1):** la marca de tiempo será `millis()` mientras no
> haya red. Cuando llegue el Wi-Fi habrá que sincronizar con NTP y convertir
> a tiempo absoluto. Definir desde ya si el campo guardará milisegundos
> relativos o si se agregará un segundo campo de tiempo absoluto.

---

## 3. Contrato de módulo

Cada sensor se implementa como un módulo que responde a las mismas tres
operaciones. Uniformar la interfaz permite sustituir un sensor o cambiar su
frecuencia sin tocar el resto del firmware.

```
INTERFAZ ModuloSensor:

    inicializar() → booleano
        // Abre el dispositivo y verifica que responda.
        // Devuelve verdadero solo si quedó operativo.

    actualizar() → nada
        // NO BLOQUEANTE. Avanza la máquina de estados del sensor.
        // Puede no producir una medición nueva en esta llamada.

    consultar() → Medicion
        // Devuelve la última medición válida conocida y su estado.
```

---

## 4. Planificación temporal

Cada módulo tiene su propio período. Ningún período se implementa con
`delay()`: se compara `millis()` contra la última ejecución.

| Módulo | Período | Justificación |
|---|---|---|
| MAX30102 | 10 ms | Vaciar la FIFO antes de que desborde |
| MPU6050 | 10 ms | La detección de agitación exige alta frecuencia |
| DS18B20 | 1000 ms | La temperatura corporal cambia lentamente |
| RC522 | 100 ms | Latencia imperceptible al presentar un tag |
| Diagnóstico | 1000 ms | Reporte por consola |

> **Decisión a discutir (2):** el MPU6050 a 100 Hz es lo mínimo para
> distinguir agitación psicomotriz de movimiento normal. Verificar en los
> papers cuál es la frecuencia de muestreo reportada en la literatura y
> justificar el valor elegido en el informe con esa referencia.

### El caso del DS18B20

Es el ejemplo canónico del problema de bloqueo. La conversión de temperatura
a 12 bits tarda unos 750 ms. La forma ingenua es pedir la conversión y
esperar — lo que congelaría el MAX30102 durante casi un segundo y arruinaría
la señal de pulso.

Solución: máquina de tres estados.

```
MÁQUINA DE ESTADOS DS18B20:

    ESTADO REPOSO:
        si (ahora - ultima_conversion >= PERIODO):
            solicitar_conversion()        // sin esperar el resultado
            t_solicitud ← ahora
            ir a ESTADO CONVIRTIENDO

    ESTADO CONVIRTIENDO:
        si (ahora - t_solicitud >= TIEMPO_CONVERSION):
            ir a ESTADO LEER
        // mientras tanto el lazo principal sigue corriendo

    ESTADO LEER:
        valor ← leer_temperatura()
        validar(valor)
        ultima_conversion ← ahora
        ir a ESTADO REPOSO
```

---

## 5. Validación de mediciones

Un valor recibido no es un valor válido. Cada módulo aplica dos filtros:

```
FUNCIÓN validar(medicion) → booleano:

    // Filtro 1: códigos de error conocidos del dispositivo
    si medicion.valor está en CODIGOS_ERROR[medicion.origen]:
        medicion.estado ← ERROR_BUS
        devolver falso

    // Filtro 2: rango fisiológicamente plausible
    si medicion.valor fuera de RANGO_VALIDO[medicion.origen]:
        medicion.estado ← FUERA_DE_RANGO
        devolver falso

    medicion.estado ← OK
    devolver verdadero
```

| Origen | Códigos de error | Rango válido |
|---|---|---|
| DS18B20 | 85.0 (reset), −127.0 (sin respuesta) | 20 – 45 °C |
| MAX30102 | FIFO vacía | señal IR sobre umbral de contacto |
| MPU6050 | lectura constante | ±16 g, ±2000 °/s |

> Los rangos de la tabla son un punto de partida. Ajustarlos con los valores
> del datasheet y con lo que reporte la literatura clínica.

---

## 6. Política de reintento

Un módulo que falla no se abandona ni bloquea el sistema.

```
si actualizar() falla:
    fallos_consecutivos ← fallos_consecutivos + 1
    si fallos_consecutivos >= UMBRAL_FALLOS:
        estado ← ERROR_BUS
        // no se detiene: se reintenta inicializar cada INTERVALO_REINTENTO
si actualizar() tiene éxito:
    fallos_consecutivos ← 0
    estado ← OK
```

---

## 7. Programa principal

```
INICIO

    configurar_puerto_serie(115200)
    configurar_buses(I2C: GPIO 21/22, 1-Wire: GPIO 4, SPI: VSPI)
    configurar_salidas(LED_VERDE: 25, LED_ROJO: 26, BUZZER: 33)

    // La inicialización registra el resultado, no aborta
    PARA CADA modulo EN [max30102, mpu6050, ds18b20, rc522]:
        modulo.estado ← modulo.inicializar() ? OK : ERROR_BUS

    reportar_estado_inicial()

    BUCLE INFINITO:

        ahora ← millis()

        PARA CADA modulo EN lista_modulos:
            si (ahora - modulo.ultima_ejecucion >= modulo.periodo):
                modulo.ultima_ejecucion ← ahora
                modulo.actualizar()

        si (ahora - t_ultimo_diagnostico >= PERIODO_DIAGNOSTICO):
            t_ultimo_diagnostico ← ahora
            imprimir_diagnostico()

        // Sin delay(). El lazo queda libre.
        // Aquí se insertarán en la Fase 2: empaquetarDatos(),
        // transmitirDatos(), almacenarLocalmente()

FIN
```

### Control de acceso

```
FUNCIÓN al_detectar_tag(uid):
    si uid está en LISTA_AUTORIZADOS:
        acceso ← CONCEDIDO
        encender(LED_VERDE) durante T_INDICACION
    sino:
        acceso ← DENEGADO
        encender(LED_ROJO) durante T_INDICACION
        pulsar(BUZZER) durante T_ALARMA
```

La temporización de LEDs y buzzer también es no bloqueante: se registra el
instante de encendido y el lazo principal los apaga cuando corresponde. Un
`delay()` aquí congelaría la adquisición del MAX30102 — y es exactamente el
criterio de puntaje 1 de la rúbrica ("el proceso bloquea la lectura de los
demás sensores").

---

## 8. Diagrama de flujo (nodos a dibujar)

Estructura para `docs/diagrama_flujo.png`:

```
[INICIO]
    ↓
[Configurar buses y salidas]
    ↓
[Inicializar módulos] → registra OK / ERROR por módulo, no aborta
    ↓
[Reportar estado inicial]
    ↓
┌─→ [Leer millis()]
│       ↓
│   <¿Venció el período del módulo N?> ──No──┐
│       ↓ Sí                                  │
│   [modulo.actualizar()]                     │
│       ↓                                     │
│   <¿Lectura válida?> ──No──► [Marcar estado, contar fallo]
│       ↓ Sí                                  │
│   [Guardar Medición con contexto]           │
│       ↓                                     │
│   ◄─────────────────────────────────────────┘
│       ↓
│   <¿Venció el período de diagnóstico?> ──No──┐
│       ↓ Sí                                    │
│   [Imprimir estado de los 4 módulos]          │
│       ↓                                       │
└───────◄───────────────────────────────────────┘
```

Herramienta sugerida: draw.io o Mermaid. Exportar en PNG a 300 dpi para
que sea legible impreso.

---

## 9. Salida diagnóstica objetivo

```
[t=001234 ms] MAX30102: OK     IR=12345        | MPU6050: OK  ax=0.02 g
              DS18B20 : OK     T=21.4 C        | RFID   : OK  ultimo=A3B21C04

[t=002234 ms] MAX30102: ERROR  sin respuesta   | MPU6050: OK  ax=0.05 g
              DS18B20 : OK     T=21.5 C        | RFID   : OK  ultimo=A3B21C04
```

La segunda línea es la evidencia clave de la defensa: un módulo caído se
reporta como tal y el resto del sistema continúa operando.

---

## 10. Respuestas a las preguntas de defensa

| Pregunta | Dónde está la respuesta |
|---|---|
| ¿Qué buses usan? | README, mapa de pines |
| ¿Por qué comparten I²C? | Direcciones distintas (0x57 / 0x68), salidas open-drain |
| ¿Qué pasa si un sensor desaparece? | Sección 6, política de reintento |
| ¿Cómo distinguen medición válida de error? | Sección 5, doble filtro |
| ¿Qué sobrevive al agregar Wi-Fi? | Secciones 1 y 7: adquisición y comunicación desacopladas |
