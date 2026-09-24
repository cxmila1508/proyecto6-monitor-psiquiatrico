# Proyecto 6 — Monitor Inalámbrico de Seguridad e Intervención Preventiva en Unidades de Internación Psiquiátrica

**Laboratorio de Electromedicina III · 2026-S2**
Escuela de Ingeniería Civil Biomédica, Facultad de Ingeniería, Universidad de Valparaíso

**Integrantes:**
- Camila Díaz — [cxmila1508]
- Gabriela Garfe — [Gabriela2005-yuki]

---

## Problema clínico

Las crisis de agitación psicomotriz en pacientes de internación psiquiátrica
conllevan riesgo de autolesión cuando los protocolos de contención se retrasan.
Este nodo captura constantes vitales e interpreta patrones de aceleración de
alta frecuencia para detectar crisis físicas, transmitiéndolos a un servidor
central que exige al personal registrar la medida aplicada antes de permitir
silenciar la alarma.

## Arquitectura del sistema

```
[MAX30102]  ─┐
[MPU6050]   ─┼─► [ESP32] ─► [Wi-Fi seguro] ─► [Flask + SQLite3] ─► [Dashboard web]
[DS18B20]   ─┤       │                                                    │
[RC522]     ─┘       └────────── alarma local (LED / buzzer) ◄────────────┘
```

| Fase | Alcance | Estado |
|---|---|---|
| 1 | Nodo de adquisición ESP32 (este repositorio) | En curso |
| 2 | Canal inalámbrico seguro con FreeRTOS | Pendiente |
| 3 | Backend Flask + SQLite3 | Pendiente |
| 4 | Frontend WebSockets | Pendiente |

---

## Mapa de pines

Todo el sistema opera a **3.3 V**. El RC522 no tolera 5 V.

| Módulo | Bus | Señal | GPIO | Observación |
|---|---|---|---|---|
| MAX30102 | I²C | SDA / SCL | 21 / 22 | Dirección 0x57 |
| MPU6050 | I²C | SDA / SCL | 21 / 22 | Dirección 0x68 · **AD0 → GND** |
| DS18B20 | 1-Wire | DQ | 4 | Pull-up 4.7 kΩ obligatoria |
| RC522 | SPI (VSPI) | SCK | 18 | |
| RC522 | SPI | MOSI | 23 | |
| RC522 | SPI | MISO | 19 | |
| RC522 | SPI | SS (CS) | 5 | |
| RC522 | SPI | RST | 27 | |
| LED verde | GPIO | — | 25 | Acceso concedido |
| LED rojo | GPIO | — | 26 | Acceso denegado |
| Buzzer | GPIO | — | 33 | Alarma local |

### Nota sobre las pull-ups del bus I²C

Las resistencias externas de 4.7 kΩ son **condicionales**. Los breakouts
comerciales suelen integrar sus propias pull-ups; instalarlas en paralelo
reduce la resistencia equivalente por debajo del margen seguro.

Procedimiento: medir la resistencia entre SDA y 3V3 con el circuito **sin
alimentar**. Si el valor está entre 1 kΩ y 10 kΩ, los módulos ya las traen y
no se agregan externas.

Límites de diseño calculados (ver informe, sección Cálculos de diseño):
- R mínima ≈ 967 Ω, con V_DD = 3.3 V, V_OL = 0.4 V, I_OL = 3 mA
- R máxima ≈ 11.8 kΩ a 100 kHz, con C_bus ≈ 100 pF y t_r ≤ 1000 ns

La pull-up del DS18B20 sí es obligatoria: el sensor no integra ninguna.

---

## Estructura del repositorio

```
├─ docs/            diagramas, pseudocódigo, evidencia fotográfica
├─ firmware/        una carpeta por etapa de integración incremental
└─ informe/         documento de la entrega
```

---

## Metodología: integración incremental

Ningún módulo se incorpora sobre un sistema que no esté previamente estable.
Cada etapa cierra con un commit funcional que deja un estado recuperable.

| Etapa | Alcance | Criterio de cierre | Estado |
|---|---|---|---|
| 1 | MAX30102 solo | Responde en 0x57 y PART_ID correcto | Compila · pendiente validación en lab |
| 2 | + MPU6050 | Ambas direcciones presentes, ninguna se pierde | Compila · pendiente validación en lab |
| 3 | + DS18B20 | Temperatura coherente (ni 85 °C ni −127 °C) | Compila · pendiente validación en lab |
| 4 | + RC522 | UID legible sin degradar los sensores previos | Compila · pendiente validación en lab |
| 5 | Migración a FreeRTOS | Tareas distribuidas en ambos núcleos sin pérdida de muestras | Compila · pendiente validación en lab |

---

## Entorno de compilación

- **Placa:** ESP32 Dev Module (paquete `esp32` de Espressif Systems)
- **Velocidad del monitor serie:** 115200 baudios

Librerías utilizadas:

| Librería | Autor | Uso |
|---|---|---|
| SparkFun MAX3010x Pulse and Proximity Sensor Library | SparkFun | MAX30102 |
| Adafruit MPU6050 + Adafruit Unified Sensor | Adafruit | MPU6050 |
| OneWire | P. Stoffregen | Bus 1-Wire |
| DallasTemperature | M. Burton | DS18B20 |
| MFRC522 | GithubCommunity | RC522 |

Registrar la versión exacta de cada librería una vez instaladas: la
reproducibilidad del build forma parte de la documentación técnica.

---

## Flujo de trabajo Git

La rama `main` está protegida: no se acepta push directo.

```bash
git checkout -b etapa2-mpu6050
# trabajar, probar en hardware
git add firmware/etapa2_i2c_dual
git commit -m "etapa2: bus I2C compartido, ambas direcciones estables"
git push -u origin etapa2-mpu6050
# abrir Pull Request hacia main desde la web
```

Convención de mensajes: `etapaN: qué quedó funcionando` o
`docs: qué se documentó`. Un commit por avance real, nunca cargas masivas.

---

## Bitácora

| Fecha | Etapa | Resultado | Responsable |
|---|---|---|---|
| 08/09/2026 | Estructura | Repositorio, README y pseudocódigo inicial | Camila |
| 08/09/2026 | 1 y 2 | Escáner I²C y bus compartido MAX30102 + MPU6050 compilando | Camila |
| 08/09/2026 | 3 | DS18B20 con máquina de estados no bloqueante, compilando | Camila |
| 23/09/2026 | 4 | RC522 por SPI y actuadores sin delay, compilando | Camila |
| 23/09/2026 | Configuración | .gitignore para excluir compilados y credenciales | Camila |
| 23/09/2026 | 5 | Migración a FreeRTOS: 4 tareas fijadas a ambos núcleos, datos protegidos con mutex, compilando | Camila |
