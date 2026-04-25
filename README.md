# Vibration Sensor (STM32H755ZI + V2S200DZ)

Firmware and PC tools to acquire vibration data from the Syntiant V2S200DZ sensor using an STM32H755ZI (Nucleo-144), then analyze it on a PC.

- DFSDM captures 1‑bit PDM at 3.2 MHz and converts to 23‑bit PCM at 12.5 kHz
- USB CDC streams framed data to the PC every 100 ms
- PC utilities record CSV and perform FFT/Welch PSD/spectrogram analysis

## Features

- DFSDM + DMA double-buffered acquisition
- USB CDC data streaming (virtual COM)
- Python analysis pipeline (no SciPy required)
- Practical bandwidth (sensor-limited): ~20 Hz – 1 kHz

## Data framing

Each frame (every 100 ms) consists of 32-bit integers:

- SOF: 0x55555555
- Timestamp (ms since MCU startup, uint32)
- 1250 PCM samples (23-bit signed, sign-extended to 32-bit)
- EOF: 0xAAAAAAAA

Notes:
- 1250 samples/100 ms → 12.5 kHz sample rate.
- If you change SOF/EOF, pick patterns that cannot appear from DFSDM output when viewed as bytes.

## Repository structure

```
VibrationSensor-V2S200DZ/
├── README.md                     # This file
├── README_draft.md               # Original draft documentation
├── analyze_vibration.py          # Python frequency analysis tool
├── read_cdc.c                    # C program for USB CDC data acquisition
└── vibration_dfsdm/              # STM32 firmware project
		├── vibration_dfsdm.ioc       # STM32CubeMX configuration
		├── .project, .mxproject      # STM32CubeIDE metadata
		├── CM4/                      # Cortex-M4 core files (unused in this project)
		├── CM7/                      # Cortex-M7 core (main application)
		│   ├── Core/                 # App sources/headers/startup
		│   ├── USB_DEVICE/           # USB device (CDC) app/target
		│   └── STM32H755ZITX_*       # Linker scripts
		├── Common/                   # Dual-core boot support
		├── Drivers/                  # HAL, CMSIS, BSP
		└── Middlewares/ST/STM32_USB_Device_Library/
```

## Build and flash (firmware)

Requirements:
- STM32CubeIDE / STM32CubeMX toolchain
- NUCLEO-H755ZI board and USB cable

Steps:
1. Open `vibration_dfsdm/vibration_dfsdm.ioc` in STM32CubeIDE and build the CM7 target.
2. Flash the board with ST-Link.
3. Connect the board’s USB OTG (not the ST-Link interface) to the PC.

## Capture on PC (Linux/WSL)

Compile and run the USB CDC reader:

```bash
gcc read_cdc.c -O3 -o read_cdc
timeout 5 ./read_cdc -p /dev/ttyACM0 -o data.csv
```

The CSV has two columns: `timestamp_us, data`.

## Analyze on PC

Install Python dependencies:

```bash
python -m pip install --upgrade pip
pip install numpy pandas matplotlib
```

Run the analysis (required arguments shown):

```bash
python analyze_vibration.py \
	--input data.csv \
	--fs 12500 \
	--calib 0.00000212 \
	--outdir ./analysis
```

Outputs in `--outdir`:
- `psd_welch.csv` — frequency vs PSD
- `psd_welch.png` — PSD plot (Welch)
- `amplitude_spectrum.png` — single-sided amplitude spectrum
- `spectrogram.png` — PSD spectrogram

Argument notes:
- `--input`: CSV from `read_cdc.c`
- `--fs`: sampling rate in Hz (default pipeline: 12500)
- `--calib`: counts→g (example provided)
- `--outdir`: output directory for plots/CSV

## Calibration note

Approximate acceleration from counts (example scaling):

accel[g] ≈ pcm × (17.7828 / 2^23)

Adjust to your calibrated chain.

## Troubleshooting

- No data on /dev/ttyACM0: ensure OTG port is used and firmware is running.
- Frame interval/size warnings: may indicate dropped/partial USB packets.
- If you change sample rate or SOF/EOF, update both firmware and PC tools.

## References

1. V2S200DZ Datasheet — https://static1.squarespace.com/static/6488b0b8150a045d2d112999/t/67521f2adb4a8e0c75ecfd53/1733435181145/V2S200DZ.pdf
2. AN4990 — https://www.st.com/resource/en/application_note/an4990-getting-started-with-sigmadelta-digital-interface-on-applicable-stm32-microcontrollers-stmicroelectronics.pdf
3. RM0399 — https://www.st.com/resource/en/reference_manual/rm0399-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
4. STM32 USB Wiki — https://wiki.st.com/stm32mcu/wiki/Category:USB
5. WSL USB passthrough — https://learn.microsoft.com/en-us/windows/wsl/connect-usb


## ESP32 Arduino example (I2S PDM -> WAV on SD)

This repository now includes `esp32_v2s200dz_sd_wav.ino`, an Arduino sketch for ESP32 that:

- reads two V2S200DZ sensors as a stereo PDM source over I2S
- uses `GPIO26` as the sensor clock and `GPIO27` as data in
- writes stereo 16-bit PCM into `/vibration.wav` on an SD card
- patches the WAV header at the end so the file is directly playable/processable

Wiring used by the sketch:

- Sensor 1 CLK -> ESP32 GPIO26
- Sensor 2 CLK -> ESP32 GPIO26
- Sensor 1 DT  -> ESP32 GPIO27
- Sensor 2 DT  -> ESP32 GPIO27
- Sensor 1 SEL/LR -> GND (Left channel)
- Sensor 2 SEL/LR -> 3.3V (Right channel)
- SD CS      -> ESP32 GPIO5
- SD SPI pins (default): MOSI=23, MISO=19, SCK=18

Usage:

1. Open `esp32_v2s200dz_sd_wav.ino` in Arduino IDE (ESP32 core).
2. Select your ESP32 board.
3. Insert/formatted FAT32 SD card.
4. Flash and open serial monitor at 115200.
5. After capture completes, read `/vibration.wav` from SD card.

If your ESP32 core version prefers the newer I2S API (`i2s_std` / `i2s_pdm`), adapt the initialization section accordingly.
