# LAMQ — prototipo sobre connectedhomeip

Primera versión del firmware de LAMQ (lámpara Matter con monitor de calidad del aire), desarrollada
entre diciembre de 2023 y febrero de 2024 para el Trabajo Fin de Máster.

> **Este repositorio es el prototipo y ya no se mantiene.** La versión final del firmware está en
> [LAMQ_FinaL_ESP32](https://github.com/joseayebenes/LAMQ_FinaL_ESP32). Esa versión usa esp-matter,
> corrige los errores de los drivers y del modelo de datos, compila de forma reproducible y está
> documentada.

## Qué contiene

Parte del ejemplo `lighting-app` de ESP32 de [connectedhomeip](https://github.com/project-chip/connectedhomeip)
y el modelo de datos se define con ZAP (`lamq-app.zap` → `lamq-app.matter`):

| Endpoint | Tipo de dispositivo | Uso |
| --- | --- | --- |
| 0 | Root Node | — |
| 1 | Dimmable Light (con Color Control) | Anillo de 12 LED WS2812 en GPIO26 (`main/matter/LEDWidget.cpp`) |
| 2 | Air Quality Sensor | AHT20 y AGS10 por I2C (GPIO19/18), PMS5003 por UART2 (GPIO16/17) y MH‑Z14A por UART1 (GPIO15/2) |

- `main/tasks/SensorTask.cpp` lee los sensores cada 20 s y actualiza los clusters mediante
  `main/matter/air-quality-sensor-manager.*`, que usa las clases `Instance` de connectedhomeip.
- `main/sensors/` tiene los drivers de los cuatro sensores.

## Compilación

Los submódulos apuntan a *snapshots* de desarrollo de febrero de 2024:

| Submódulo | Commit | Fecha |
| --- | --- | --- |
| `third_party/esp-idf` | `4d90eed` (`master`) | 13/02/2024 |
| `third_party/connectedhomeip` | `2f2c4f1` (`master`) | 16/02/2024 |

El `dependencies.lock` se generó con ESP-IDF 5.1.1.

```bash
git submodule update --init third_party/esp-idf third_party/connectedhomeip
git -C third_party/esp-idf submodule update --init --recursive
cd third_party/connectedhomeip && ./scripts/checkout_submodules.py --platform esp32 --shallow && cd ../..

./third_party/esp-idf/install.sh esp32
source third_party/esp-idf/export.sh
source third_party/connectedhomeip/scripts/activate.sh

idf.py set-target esp32
idf.py build flash monitor
```

Para más detalles, consulta la guía de ESP32 de connectedhomeip
(`third_party/connectedhomeip/docs/guides/esp32/`).

## Problemas conocidos

Están corregidos en LAMQ_FinaL_ESP32 y no se van a corregir aquí:

- Los drivers ignoran los errores de lectura. Si un sensor no responde se publican valores antiguos o
  a cero, y no se comprueban los CRC del AHT20 ni del AGS10.
- El modelo de datos ZAP declara clusters que el firmware no implementa: CO, NO₂, O₃, formaldehído y
  radón.
- Los umbrales del índice de calidad del aire son distintos de los de la versión final.

Limpieza de 2026: se han quitado del repositorio `managed_components/` (se descargan según
`dependencies.lock`) y la copia de seguridad `lamq-app.zap~`. También se han corregido estos errores:
- Las unidades de TVOC (ppb) y PM (µg/m³).
- La escritura sobrante de un CO₂ fijo de 500 ppm en cada ciclo.
- La doble inicialización del gestor de calidad del aire.
- La conversión de la temperatura, que tenía comportamiento indefinido con valores negativos.
- El nombre del `.elf` en la configuración de OTA cifrada.
