# ESP32-2432S028R CO2 Dashboard

Full-colour indoor air-quality display for the **ESP32-2432S028R**
("Cheap Yellow Display") and a **Sensirion SCD40 or SCD41** sensor.

<img width="1600" height="1028" alt="Photo" src="https://github.com/user-attachments/assets/c737540a-e915-470b-839a-d51b911a8628" />

## Display

- Large, colour-coded CO2 reading
- Touch-selectable CO2 graph: 10 minutes, 1 hour, 6 hours, or 12 hours
- Smaller temperature and humidity cards
- Green: below 800 ppm
- Amber: 800-1199 ppm
- Red: 1200 ppm and above

The display is configured for the ESP32-2432S028R's built-in 2.8-inch
320x240 ILI9341 panel and XPT2046 touch controller. No separate display or
touch wiring is required.

## Touch control

Tap anywhere inside the **CO2 TREND** panel to cycle through:

`10 min` -> `1 hour` -> `6 hours` -> `12 hours`

The selected range is remembered after a restart.

## Persistent history

- CO2 is sampled every 10 seconds.
- The 10-minute view uses the latest 60 readings.
- Longer views use one-minute averages.
- Up to 12 hours of one-minute history is stored in ESP32 NVS.
- History is saved every 10 minutes to reduce flash wear.

After an unexpected power loss, up to the most recent 10 minutes of unsaved
history may be lost. Previously saved history and the selected graph range are
restored on startup.

## Sensor wiring

| SCD40/SCD41 | ESP32-2432S028R |
|-------------|-----------------|
| VIN         | 3V3             |
| GND         | GND             |
| SDA         | GPIO 27         |
| SCL         | GPIO 22         |

Use a breakout board that includes the required I2C pull-up resistors. The
pins can be changed at the top of `src/main.cpp` if your board exposes a
different connector.

## Build and upload

1. Open this folder in VS Code with the PlatformIO extension.
2. Connect the ESP32-2432S028R by USB.
3. Run **PlatformIO: Upload**.
4. Optionally open the serial monitor at 115200 baud.

Command-line equivalent:

```text
pio run
pio run -t upload
pio device monitor
```

The dashboard requests a new reading every 10 seconds. The first displayed
reading normally appears about 10 seconds after startup.
