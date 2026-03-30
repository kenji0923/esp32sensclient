# esp32sensclient

ESP32 sensor client firmware for SHT40, DPS310, and battery measurement.

This firmware:

- receives configuration from `esp32senserv` over ESP-NOW
- reads SHT40 temperature / humidity
- reads DPS310 pressure / temperature
- reads battery voltage through ADC
- reports measurement data and retry statistics back to the server

Main firmware file:

- `main/main.c`

## Sensors

- SHT40 at I2C address `0x44`
- DPS310 at I2C address `0x77`
- battery ADC on `ADC_CHANNEL_2`

## Current Config Fields

The client expects this config from the server:

- `sleep_sec`
- `work_delay_ms`
- `batt_avg`
- `sht_avg`
- `sht_read_wait_time_ms`
- `dps_osr`
- `dps_avg`
- `dps_read_wait_time_ms`
- `config_hash`

Meaning:

- `sleep_sec`: measurement interval in seconds
- `work_delay_ms`: delay after I2C initialization before sensor access
- `batt_avg`: number of ADC samples for battery averaging
- `sht_avg`: number of valid SHT40 readings required
- `sht_read_wait_time_ms`: delay between SHT40 command and read
- `dps_osr`: DPS310 oversampling code
- `dps_avg`: number of valid DPS310 readings required
- `dps_read_wait_time_ms`: delay used between DPS310 command and read steps

## Sleep Behavior

- If `sleep_sec != 0`, the client enters deep sleep between measurement cycles.
- If `sleep_sec == 0`, the client stays awake and runs the measurement loop every second.

This matters for firmware updates:

- after a measurement, the device normally goes to deep sleep
- if it is sleeping most of the time, flashing can be inconvenient or fail because the device does not stay awake long enough
- for maintenance or reflashing, set `sleep_sec` to `0`
- with `sleep_sec == 0`, the device stays awake, repeats measurements every second, and does not enter deep sleep

## Retry Logic

Both sensor loops now use valid-sample targets rather than fixed-attempt averaging.

### SHT40

- target valid samples: `sht_avg`
- maximum trials: `3 * sht_avg`

The client keeps trying until either:

- it collects `sht_avg` valid readings, or
- it reaches `3 * sht_avg` attempts

### DPS310

- target valid samples: `dps_avg`
- maximum trials: `3 * dps_avg`

The client keeps trying until either:

- it collects `dps_avg` valid readings, or
- it reaches `3 * dps_avg` attempts

## Reported Data

The client sends these fields to the server:

- `temperature`
- `humidity`
- `pressure`
- `v_batt`
- `dps_temp`
- `temphumid_validcount`
- `temphumid_trial`
- `pressure_validcount`
- `pressure_trial`
- `timestamp`
- `config_hash`

The `*_validcount` and `*_trial` fields are useful for diagnosing startup or sensor-read reliability.

## DPS310 Oversampling Mapping

`dps_osr` uses this mapping:

- `0 = x1`
- `1 = x2`
- `2 = x4`
- `3 = x8`
- `4 = x16`
- `5 = x32`
- `6 = x64`
- `7 = x128`

## Build

From this directory:

```bash
cd /home/kshu/work/development/esp32sensclient
idf.py build
```

## Flash

Typical ESP-IDF flow:

```bash
idf.py flash
```

There is also a local helper script in:

- `flash.sh`

if you want to use your existing local flashing workflow.

## Compatibility Note

This client shares binary struct layouts with the server and web parser. If you change `config_data_t` or `sensor_data_t`, also update:

- `esp32senserv/main/main.c`
- `esp32senserv/web_server.py`

and rebuild / reflash as needed.
