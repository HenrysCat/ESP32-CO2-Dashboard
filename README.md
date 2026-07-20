# ESP32-2432S028R CO2 Dashboard

Full-colour indoor air-quality display for the **ESP32-2432S028R**
("Cheap Yellow Display") and a **Sensirion SCD40, SCD41, or SCD30** sensor.

No PlatformIO or build step required - flash it straight from your browser:
**[Web flasher](https://henryscat.github.io/)**

<img width="1600" height="1028" alt="Photo" src="https://github.com/HenrysCat/ESP32-CO2-Dashboard/blob/main/Web%20UI%20Screenshots/co2-dashboard.jpg" />

## Display

- Large, colour-coded CO2, temperature, and humidity readings
- Touch-selectable CO2 graph: 10 minutes, 1 hour, 6 hours, or 12 hours
- Swipe-selectable SD history graph: 24 hours, 7 days, or 30 days
- Smaller temperature and humidity cards
- The date/time in the top-right corner is joined by the device's Wi-Fi IP
  address whenever it's connected, and left blank otherwise

### Reading colours

Each reading is colour-coded against its own thresholds:

| | CO2 | Temperature | Humidity |
| --- | --- | --- | --- |
| Green | below 800 ppm | - | - |
| Cyan | - | below 18°C | below 40% |
| Amber | 800-1199 ppm | 18-24.9°C | 40-59.9% |
| Red | 1200 ppm and above | 25°C and above | 60% and above |

The display is configured for the ESP32-2432S028R's built-in 2.8-inch
320x240 ILI9341 panel and XPT2046 touch controller. No separate display or
touch wiring is required.

## Touch control

Tap anywhere inside the **CO2 TREND** panel to cycle through:

`10 min` -> `1 hour` -> `6 hours` -> `12 hours`

The selected range is remembered after a restart.

Short-press the **BOOT** button on the dashboard to cycle the main CYD display
between:

`CO2` -> `Temperature` -> `Humidity`

The large reading, small trend panel, and long-term SD history screen follow
the selected metric. When temperature or humidity is promoted to the large
panel, CO2 moves into that metric's previous bottom-card position. The
selected metric is remembered after a restart.

Swipe left anywhere on the dashboard to open the long-term SD history screen.
Tap that screen to cycle through:

`24 hours` -> `7 days` -> `30 days`

Swipe right to return to the dashboard. The long-term graph reads only the
required tail of the active monthly CSV and condenses it to the screen width,
so its RAM usage does not grow with the size of the log file.

Hold the **BOOT** button for about one second while viewing the long-term
screen to open the calibration screen, titled for whichever sensor is active
(for example **SCD4x CALIBRATION** or **SCD30 CALIBRATION**).
This screen provides:

- Automatic self-calibration on/off
- Temperature offset, read from the sensor
- Forced recalibration references of 400, 420, or 450 ppm

Altitude compensation is fixed at zero. The controls are operated with the
board's **BOOT** button:

- Short press: move the highlighted selection
- Hold for about one second: activate the highlighted selection
- Holding on temperature offset adds 0.5 C; after 10.0 C it wraps to 0.0 C
- Select **EXIT** and hold to return to the long-term screen

Swipe right or activate **EXIT** to leave the calibration screen. Do not hold
BOOT while powering on or resetting the board, because GPIO 0 also selects the
ESP32 firmware-download mode during startup.

Forced recalibration is guarded by two long presses. Before using it,
place the sensor in stable air at the selected reference concentration for at
least three minutes. Performing forced recalibration in unsuitable or changing
air can make subsequent CO2 readings less accurate. Swipe right to leave the
calibration screen.

## Persistent history

- CO2 is sampled every 10 seconds.
- CO2 readings from the first 30 seconds after sensor startup or a measurement
  restart are hidden and excluded from trend and CSV history.
- Temperature and humidity readings remain hidden and excluded for three
  minutes, preventing longer thermal and humidity settling transients from
  entering the display, web UI, recent trends, or long-term history.
- The 10-minute view uses the latest 60 readings.
- Longer views use one-minute averages.
- Up to 12 hours of one-minute history is stored in ESP32 NVS.
- History is saved every 10 minutes to reduce flash wear.
- One-minute CO2, temperature, and humidity averages are appended to
  month-specific CSV files on the onboard microSD card.

After an unexpected power loss, up to the most recent 10 minutes of unsaved
history may be lost. Previously saved history and the selected graph range are
restored on startup.

### SD card log

Insert a FAT32-formatted microSD card before startup. Once network time is
available, the firmware creates one file per calendar month:

```text
co2-2026-01.csv
co2-2026-02.csv
co2-2026-03.csv
```

Each file uses these columns:

```text
boot_id,uptime_seconds,co2_ppm,temperature_c,humidity_percent,local_timestamp
```

Each monthly log is append-only and can be opened directly in spreadsheet,
charting, or database-import tools. At the beginning of a new month, the
firmware automatically creates the next file and writes its header. Each row
is closed and committed after it is written, limiting loss during a power
failure to the current one-minute average. If the card is missing or a write
fails, the firmware retries once a minute without interrupting the dashboard.

`local_timestamp` is supplied by network time when Wi-Fi is connected. Until
the first NTP synchronization completes, readings are stored in
`co2-unsynced.csv` with `unsynced` timestamps because their calendar month is
not yet known.
`uptime_seconds` and `boot_id` remain available to identify and order readings
when network time is unavailable.

Existing `co2log.csv` files are left unchanged as legacy data.

The SD slot uses GPIO 5 (CS), 18 (SCK), 19 (MISO), and 23 (MOSI). Software SPI
is used because the LCD and touch controller already occupy both ESP32
hardware SPI controllers.

## Wi-Fi and timezone setup

On first boot, or whenever the saved network cannot be reached, the dashboard
creates an open Wi-Fi access point named:

```text
CO2-Dashboard-Setup
```

Connect a phone or computer to that network. The captive portal should open
automatically; otherwise browse to `http://192.168.4.1`. Select the local Wi-Fi
network, enter its password, and set the timezone before saving.
The non-blocking setup portal remains active until Wi-Fi is configured, while
the dashboard and SD logging continue operating.

The timezone field accepts a POSIX timezone rule. The default is for the UK:

```text
GMT0BST,M3.5.0/1,M10.5.0
```

Other examples:

| Location | POSIX timezone rule |
|----------|---------------------|
| UTC | `UTC0` |
| US Eastern | `EST5EDT,M3.2.0,M11.1.0` |
| US Central | `CST6CDT,M3.2.0,M11.1.0` |
| US Mountain | `MST7MDT,M3.2.0,M11.1.0` |
| US Pacific | `PST8PDT,M3.2.0,M11.1.0` |
| Central Europe | `CET-1CEST,M3.5.0,M10.5.0/3` |

Wi-Fi credentials are stored by the ESP32 Wi-Fi stack and the timezone is
stored in NVS. The board reconnects automatically after subsequent restarts.
A full flash erase removes both, so the setup access point will appear again.

## Local web dashboard

Once connected to Wi-Fi, the ESP32 serves a responsive dashboard on the local
network. Open either:

```text
http://co2-dashboard.local/
http://<device-ip-address>/
```

The current IP address and web dashboard URL are printed to the serial monitor
after Wi-Fi connects. The page shows the live CO2, temperature, and humidity
readings. Select any of the three metric cards to move that reading into the
large display and change the trend chart. Available trend ranges are 10
minutes, 1 hour, 6 hours, 12 hours, 1 day, and 1 week. Longer ranges are
aggregated from the current and previous monthly CSV files. The page updates
automatically and does not require an internet connection.
Trend charts include sparse time grid lines. Short ranges use local clock
times, 6-hour/12-hour/1-day ranges use whole-hour labels, and 1-week ranges
use date/time labels.

A monthly history section lists the month files found on the SD card. Select a
month and switch between CO2, temperature, and humidity charts. Each view also
shows the minimum, average, and maximum value for that metric. The ESP32
aggregates each month into 600 chart points while scanning the CSV, keeping
memory use fixed even as files grow.
Monthly charts show sparse date/time grid lines from the first reading to the
latest reading in the selected file.

A gear button at the top of the page opens a **CO2 sensor** selector -
Auto-detect, SCD4x (SCD40/SCD41), or SCD30 - alongside sensor settings for
automatic self-calibration, temperature offset, and guarded forced
recalibration. Auto-detect uses whichever sensor answers on the I2C bus at
startup. Changing a sensor setting briefly pauses measurements while the
value is written and persisted by the active sensor.

### Display settings

The same settings panel also has a display section:

- **Dashboard colour theme** - Midnight blue, Black/red, Slate green, or
  Violet dusk. This only restyles this web page and is saved per browser, so
  different visitors can each pick their own without affecting anyone else.
- **Device colour theme** - Navy, Midnight Blue, Slate Green, Violet Dusk,
  Mahogany, Carbon Red, Black Red, Mauve, or Custom. Restyles the physical
  CYD screen and is saved on the device.
- **Custom theme colours** - shown only when Custom is selected. Five
  6-digit hex fields (no `#`) let you set the screen's background, panel
  fill, panel border/graph gridlines, primary text, and muted/label text
  independently. The fields default-fill with whichever theme is currently
  active, so switching to Custom starts from a real theme instead of blank
  values, and switching to a preset and back re-fills them from that preset
  rather than keeping a stale edit. CO2/temperature/humidity threshold
  colours (green/amber/red/cyan) are fixed and not affected by any theme.
- **Screen colour order** (BGR/RGB) - corrects red and blue appearing swapped
  on differently-wired panels.
- **Rotate display 90 degrees** - corrects panels that boot into a cropped
  portrait image because of a differently wired panel variant.
- **Flip display 180 degrees** - for mounting the board with the cable exit
  on either side.

Changing a device display setting fully redraws the physical screen so it
doesn't stay garbled after the colours change.

The `.local` address requires mDNS support, which is built into current
Windows, macOS, iOS, and most Linux and Android installations. Use the printed
IP address if the hostname is not resolved by a particular device.

## Sensor wiring

| SCD4x (SCD40/SCD41) or SCD30 | ESP32-2432S028R |
|-------------------------------|-----------------|
| VIN                           | 3V3             |
| GND                           | GND             |
| SDA                           | GPIO 27         |
| SCL                           | GPIO 22         |

Both sensor families share the same I2C wiring; the firmware tells them apart
by I2C address, either automatically or via the **CO2 sensor** selector in the
web dashboard's settings. Use a breakout board that includes the required I2C
pull-up resistors. The pins can be changed at the top of `src/main.cpp` if
your board exposes a different connector.

### Sensor power

On the tested board, powering the CYD through its **Micro-USB port** provides
stable operation with the sensor connected to the onboard 3.3 V pin. Powering
the same board through its **USB-C port** caused the LCD backlight to flicker
during sensor measurements. This is likely due to additional voltage drop in
that input path, the USB cable, or the connector.

Prefer Micro-USB for normal operation and do not power both USB ports at the
same time.

The SCD4x can draw approximately 205 mA in short measurement pulses; the
SCD30's power draw was not separately measured, but the same precautions
apply. If the backlight still flickers:

- Try a shorter, higher-quality Micro-USB cable and a reliable 5 V supply.
- Add a 470 uF electrolytic capacitor and a 100 nF ceramic capacitor directly
  across the sensor's VIN and GND.
- As a fallback, power the sensor from a separate regulated 3.3 V supply rated
  for at least 300 mA, with its ground connected to CYD ground.

Some SCD4x breakout boards accept 5 V and include both a regulator and I2C
level shifting. Only power a breakout from the CYD 5 V pin when its
manufacturer explicitly states that both features are present. Do not allow
SDA or SCL to be pulled up to 5 V.

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

The dashboard requests a new reading every 10 seconds. The first displayed reading normally appears about 30 seconds after startup for CO2 and 3 minutes for temperature and humidity, this is allow the sensors to settle down and not log false readings.
