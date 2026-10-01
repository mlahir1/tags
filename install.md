# ESP32-C6 Time Recorder — Installation & Setup Guide

This guide sets up the complete project on macOS:

1. Arduino IDE + ESP32-C6 firmware
2. Required Arduino libraries
3. Display, SD card, microphone, and button hardware
4. Firmware upload and serial monitor
5. Python/Bleak BLE receiver
6. First recording and file-download test
7. Troubleshooting

The project contains two separate programs:

- **ESP32 firmware:** `Arduino/ESP32C6_TimeRecorder/ESP32C6_TimeRecorder.ino`
- **Mac BLE receiver:** `client/receiver.py`

Do not try to run `receiver.py` from Arduino IDE. Arduino IDE is used only for the ESP32 firmware.

---

## 1. Recommended folder layout

After extracting the ZIP, the important files are:

```text
esp32c6_time_recorder_refactored/
├── Arduino/
│   └── ESP32C6_TimeRecorder/
│       ├── ESP32C6_TimeRecorder.ino
│       ├── App.cpp
│       ├── App.h
│       ├── AppConfig.h
│       ├── AudioRecorder.cpp
│       ├── AudioRecorder.h
│       ├── BleFileManager.cpp
│       ├── BleFileManager.h
│       ├── ButtonController.cpp
│       ├── ButtonController.h
│       ├── DisplayUI.cpp
│       ├── DisplayUI.h
│       ├── Hardware.cpp
│       ├── Hardware.h
│       ├── TimeService.cpp
│       ├── TimeService.h
│       └── lv_conf.h
├── client/
│   ├── receiver.py
│   └── requirements.txt
├── docs/
│   └── install.md
└── legacy/
    └── original_monolith.cpp
```

The Arduino folder is the firmware you open in Arduino IDE.

---

# 2. macOS prerequisites

## 2.1 Install Arduino IDE

Install a current Arduino IDE release for macOS from the official Arduino website.

After installation, launch Arduino IDE once before continuing.

## 2.2 Install Python 3

The BLE receiver requires Python 3.

Check whether Python is already installed:

```bash
python3 --version
```

If that command does not work, install a current Python 3 release for macOS.

Verify:

```bash
python3 --version
python3 -m pip --version
```

---

# 3. Install ESP32 board support

In Arduino IDE:

1. Open **Arduino IDE**.
2. Open **Arduino IDE → Settings** on macOS.
3. Find **Additional boards manager URLs**.
4. Add the Espressif ESP32 package URL supplied by Espressif for Arduino.
5. Save the settings.
6. Open **Tools → Board → Boards Manager**.
7. Search for **esp32**.
8. Install **esp32 by Espressif Systems**.

After installation, select an ESP32-C6 board matching your hardware.

The original project was developed around an `esp32-c6-devkitc-1` target. If your physical board has a different ESP32-C6 board definition, select the corresponding board in Arduino IDE.

---

# 4. Install Arduino libraries

The firmware uses these external Arduino libraries:

## Required

### GFX Library for Arduino

Install **GFX Library for Arduino** using:

**Sketch → Include Library → Manage Libraries**

Search for:

```text
GFX Library for Arduino
```

Install it.

### LVGL

Install the **LVGL** library using the Library Manager.

This project was prepared for **LVGL 9.x**.

Search for:

```text
lvgl
```

Install a compatible 9.x release.

The project includes its own `lv_conf.h` inside the Arduino sketch folder.

## ESP32 libraries supplied by the ESP32 board package

These headers are provided by the Espressif ESP32 Arduino core and should not normally be installed separately:

```text
Arduino.h
BLEDevice.h
BLEServer.h
BLEUtils.h
BLE2902.h
FS.h
SD.h
SPI.h
driver/i2s_std.h
```

---

# 5. Open the Arduino project

In Arduino IDE choose:

**File → Open**

Open this file:

```text
Arduino/ESP32C6_TimeRecorder/ESP32C6_TimeRecorder.ino
```

Important: open the `.ino` file inside the `ESP32C6_TimeRecorder` folder. Do not open one of the `.cpp` files as the main sketch.

Arduino IDE will compile the other `.cpp` files in the same sketch folder automatically.

---

# 6. Select the ESP32-C6 board

Connect the ESP32-C6 to your Mac with a USB data cable.

In Arduino IDE:

**Tools → Board → esp32 → ESP32C6 Dev Module**

or select the exact ESP32-C6 board definition that matches your hardware.

Then select:

**Tools → Port**

and choose the serial port belonging to the ESP32-C6.

If no port appears, see the troubleshooting section below.

---

# 7. Verify the firmware configuration

Open:

```text
Arduino/ESP32C6_TimeRecorder/AppConfig.h
```

The current project configuration is:

| Function | Pin |
|---|---:|
| I2S BCLK | 18 |
| I2S WS/LRCLK | 19 |
| I2S microphone data | 20 |
| SD MISO | 5 |
| SD MOSI | 6 |
| SD SCLK | 7 |
| SD CS | 4 |
| TFT MOSI | 6 |
| TFT SCLK | 7 |
| TFT CS | 14 |
| TFT DC | 15 |
| TFT RST | 21 |
| TFT backlight | 22 |
| Button | 1 |

The display configuration is:

```text
Width:  172
Height: 320
```

The audio configuration is:

```text
Sample rate: 16000 Hz
Channels:    1 (mono)
```

**Do not change these pins unless your actual hardware is wired differently.**

The SD card and TFT share the SPI MOSI/SCLK pins but have separate chip-select pins.

---

# 8. Compile the firmware

In Arduino IDE select:

**Sketch → Verify/Compile**

The first compilation can take longer because the ESP32 core and libraries are being compiled.

If compilation succeeds, continue to uploading.

If compilation fails, check:

1. The ESP32 board package is installed.
2. An ESP32-C6 board is selected.
3. GFX Library for Arduino is installed.
4. LVGL 9.x is installed.
5. You opened `ESP32C6_TimeRecorder.ino`.
6. `lv_conf.h` is still inside the sketch folder.

---

# 9. Upload the firmware

Connect the ESP32-C6 to the Mac and select the correct port.

Choose:

**Sketch → Upload**

If the board does not enter download mode automatically, use the board's BOOT/RESET procedure appropriate for your ESP32-C6 hardware.

A successful upload normally ends with a message similar to:

```text
Hard resetting via RTS pin...
```

The exact message varies with the ESP32 board and Arduino IDE version.

---

# 10. Open the serial monitor

Open:

**Tools → Serial Monitor**

Set the baud rate to:

```text
115200
```

The firmware was configured for a 115200-baud serial monitor.

Reset the ESP32-C6 if necessary and watch the startup messages.

---

# 11. Hardware setup

The firmware expects these hardware functions:

- I2S microphone
- microSD card
- SPI TFT display
- push button
- ESP32-C6

Make sure the ESP32 and peripherals share a suitable ground.

## SD card

The firmware uses:

```text
MISO = 5
MOSI = 6
SCLK = 7
CS   = 4
```

The SD card should be usable by the ESP32 SD library.

## TFT

The TFT uses:

```text
MOSI = 6
SCLK = 7
CS   = 14
DC   = 15
RST  = 21
BL   = 22
```

## Button

The button is assigned to GPIO 1.

The firmware uses a short press for message navigation and a long press for recording control, preserving the behavior of the original monolithic firmware.

## I2S microphone

The microphone uses:

```text
BCLK = 18
WS   = 19
DIN  = 20
```

The recorder captures:

```text
16 kHz
mono
```

and stores the recording using the project's IMA/DVI ADPCM WAV implementation.

---

# 12. BLE device setup

The ESP32 advertises as:

```text
ESP32C6-TimeDevice
```

The firmware uses these BLE UUIDs:

```text
Service:
4fafc201-1fb5-459e-8fcc-c5c9c331914b

Time:
beb5483e-36e1-4688-b7f5-ea07361b26a8

File control:
beb5483e-36e1-4688-b7f5-ea07361b26a9

File data:
beb5483e-36e1-4688-b7f5-ea07361b26aa
```

The Mac receiver uses the same UUIDs.

---

# 13. Install the Mac BLE receiver

Open Terminal and change to the project's `client` directory:

```bash
cd /path/to/esp32c6_time_recorder_refactored/client
```

Create a Python virtual environment:

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Upgrade pip:

```bash
python3 -m pip install --upgrade pip
```

Install the receiver dependency:

```bash
python3 -m pip install -r requirements.txt
```

The project's requirements file installs:

```text
bleak>=0.22
```

---

# 14. macOS Bluetooth permission

The first time the Python receiver accesses Bluetooth, macOS may request permission.

Allow Terminal or the application running Python to use Bluetooth.

If Bluetooth access was previously denied, check:

**System Settings → Privacy & Security → Bluetooth**

and allow the application that is running the receiver.

Also make sure Bluetooth is enabled on the Mac.

---

# 15. Run the receiver

With the virtual environment active:

```bash
python3 receiver.py
```

The receiver searches for:

```text
ESP32C6-TimeDevice
```

It then:

1. Connects to the ESP32.
2. Enables BLE notifications.
3. Synchronizes the ESP32 time.
4. Requests the WAV file list.
5. Downloads each file.
6. Uses sequence numbers and ACKs during transfer.
7. Waits for the EOF marker.
8. Verifies the received file size.
9. Performs basic RIFF/WAVE validation.
10. Saves the WAV locally.
11. Requests deletion from the ESP32 only after validation succeeds.

The receiver intentionally does **not** delete an SD-card recording when validation fails.

---

# 16. First complete test

Use this order for the first test:

### Step 1 — Upload firmware

Upload `ESP32C6_TimeRecorder.ino` from Arduino IDE.

### Step 2 — Confirm startup

Open Serial Monitor at 115200 baud.

### Step 3 — Check SD card

Make sure the SD card is detected by the firmware.

### Step 4 — Check display

Confirm that the TFT initializes and displays the application UI.

### Step 5 — Record a short test

Use the button according to the firmware's recording controls.

### Step 6 — Stop recording

Allow the recorder to finalize the WAV file.

### Step 7 — Run the Mac receiver

From another Terminal window:

```bash
cd /path/to/esp32c6_time_recorder_refactored/client
source .venv/bin/activate
python3 receiver.py
```

### Step 8 — Confirm download

The receiver should print the discovered WAV files and download them.

Downloaded files are saved using the same relative path supplied by the ESP32.

---

# 17. BLE transfer protocol used by the project

The receiver and firmware communicate using the existing command protocol.

## Commands sent to the ESP32

```text
LIST
GET:<path>
ACK:<sequence>
ACKDONE
DEL:<path>
```

Time synchronization uses:

```text
YYYY-MM-DD HH:MM:SS
```

## File-data packets

Each data packet contains:

```text
4-byte little-endian sequence number
+ file payload
```

The firmware uses a maximum data notification size of 244 bytes, consisting of a 4-byte sequence number and up to 240 bytes of payload.

EOF is:

```text
FF EE EE FF
```

The receiver verifies the sequence number and requests retransmission when packets are missing or arrive out of order.

---

# 18. Troubleshooting

## Arduino IDE cannot find the ESP32-C6

Check:

- ESP32 by Espressif Systems is installed in Boards Manager.
- The selected board is an ESP32-C6 board.
- The USB cable supports data.
- The board is powered.
- The correct USB port is selected.

Try disconnecting and reconnecting the board, then reopen **Tools → Port**.

## Upload fails with a timeout

Try:

1. Hold the BOOT button.
2. Start the upload.
3. Release BOOT when the upload begins.

The exact sequence depends on the ESP32-C6 development board.

If necessary, press RESET and retry.

## Compile error: `Arduino_GFX_Library.h` not found

Install:

```text
GFX Library for Arduino
```

from Arduino Library Manager.

## Compile error: `lvgl.h` not found

Install LVGL 9.x using Arduino Library Manager.

## LVGL configuration errors

Make sure this file exists next to the `.ino` file:

```text
Arduino/ESP32C6_TimeRecorder/lv_conf.h
```

Do not remove it.

## BLE receiver cannot find the ESP32

Confirm:

- ESP32 firmware is running.
- ESP32 is advertising as `ESP32C6-TimeDevice`.
- Mac Bluetooth is enabled.
- Terminal/Python has Bluetooth permission.
- The receiver is using the project's `receiver.py`.

Run:

```bash
python3 receiver.py
```

again after resetting the ESP32.

## Receiver says the device was not found

Move the Mac closer to the ESP32 and make sure another BLE application is not holding the connection.

Restart Bluetooth or reset the ESP32 if necessary.

## Receiver downloads 0 bytes

The receiver resets its transfer buffer before each requested file and waits for EOF/ACKDONE before saving the file.

If a transfer still produces 0 bytes, check the ESP32 serial output for:

- file-open errors
- SD-card errors
- BLE notification errors
- transfer/retransmission errors

Do not delete the recording manually until the transfer problem is understood.

## Receiver reports a size mismatch

The receiver compares the received byte count with the file size reported by the ESP32. If they differ, it does not delete the ESP32 copy.

Check BLE connection quality and retransmission messages first.

## Receiver reports invalid RIFF/WAVE

The downloaded file must begin with a valid RIFF/WAVE header. If validation fails, the receiver leaves the original recording on the ESP32.

Check the recording finalization and SD-card write process.

## SD card is not detected

Check:

```text
MISO = GPIO 5
MOSI = GPIO 6
SCLK = GPIO 7
CS   = GPIO 4
```

Also verify the SD card wiring, power, and formatting.

## Display does not work

Check the TFT wiring and these configured pins:

```text
MOSI = GPIO 6
SCLK = GPIO 7
CS   = GPIO 14
DC   = GPIO 15
RST  = GPIO 21
BL   = GPIO 22
```

The exact TFT controller and wiring must match the display configuration used by the original hardware.

---

# 19. Updating the firmware configuration

The main hardware/application settings are centralized in:

```text
Arduino/ESP32C6_TimeRecorder/AppConfig.h
```

Change hardware-specific settings there rather than scattering pin definitions through the source files.

Important settings include:

```text
I2S_BCLK
I2S_WS
I2S_DIN
SD_MISO
SD_MOSI
SD_SCLK
SD_CS
TFT_MOSI
TFT_SCLK
TFT_CS
TFT_DC
TFT_RST
TFT_BL
BUTTON_PIN
SCREEN_WIDTH
SCREEN_HEIGHT
SAMPLE_RATE
```

The BLE UUIDs and device name are also centralized there.

If you change a BLE UUID or device name, update both the ESP32 firmware and `client/receiver.py` so they continue to communicate.

---

# 20. Keeping the original firmware

The original monolithic source is preserved at:

```text
legacy/original_monolith.cpp
```

This is provided for comparison/reference. The Arduino IDE should use the refactored files under:

```text
Arduino/ESP32C6_TimeRecorder/
```

Do not copy the legacy source into the Arduino sketch folder unless you intentionally want to compile it as part of the sketch.

---

# 21. Recommended development workflow

For firmware changes:

1. Edit the appropriate module in `Arduino/ESP32C6_TimeRecorder/`.
2. Keep hardware constants in `AppConfig.h`.
3. Compile with Arduino IDE.
4. Upload to the ESP32-C6.
5. Test using Serial Monitor.
6. Test recording and SD storage.
7. Test BLE transfer using `client/receiver.py`.

For receiver changes:

1. Activate the Python virtual environment.
2. Edit `client/receiver.py`.
3. Run:

```bash
python3 receiver.py
```

---

# 22. Quick-start checklist

Use this checklist after installing everything:

- [ ] Arduino IDE installed
- [ ] ESP32 by Espressif Systems installed
- [ ] ESP32-C6 board selected
- [ ] Correct USB port selected
- [ ] GFX Library for Arduino installed
- [ ] LVGL 9.x installed
- [ ] `lv_conf.h` present
- [ ] `ESP32C6_TimeRecorder.ino` opens successfully
- [ ] Firmware compiles
- [ ] Firmware uploads
- [ ] Serial Monitor set to 115200
- [ ] Microphone wired correctly
- [ ] SD card wired correctly
- [ ] TFT wired correctly
- [ ] Button wired to GPIO 1
- [ ] ESP32 advertises as `ESP32C6-TimeDevice`
- [ ] Python 3 installed
- [ ] Bluetooth enabled on Mac
- [ ] Bluetooth permission granted
- [ ] Python virtual environment created
- [ ] Bleak installed
- [ ] `receiver.py` runs
- [ ] Test WAV recording created
- [ ] WAV downloads successfully
- [ ] Downloaded WAV passes validation

---

# 23. Important note about toolchain verification

This project package is organized for Arduino IDE and includes the required source files and configuration. The development environment used to assemble the package does not contain the ESP32 Arduino toolchain or the physical ESP32-C6 hardware, so a real hardware compile/upload/test cannot be claimed from this package build alone.

The first hardware verification should therefore be performed in Arduino IDE on the target ESP32-C6 board.
