#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s_std.h"
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <time.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define I2S_BCLK  18
#define I2S_WS    19
#define I2S_DIN   20

#define SD_MISO   5
#define SD_MOSI   6
#define SD_SCLK   7
#define SD_CS     4

#define TFT_MOSI  6
#define TFT_SCLK  7
#define TFT_CS    14
#define TFT_DC    15
#define TFT_RST   21
#define TFT_BL    22

#define BUTTON_PIN 1

#define SCREEN_WIDTH  172
#define SCREEN_HEIGHT 320

// ============================================================
// RECORDING SETTINGS
// ============================================================
#define SAMPLE_RATE 16000
#define CHANNELS 1

// I2S microphone delivers 32-bit samples.
#define I2S_BUFFER_SAMPLES 256

// ============================================================
// IMA ADPCM SETTINGS
// ============================================================
//
// Each ADPCM block:
//
//   2 bytes = predictor / first PCM sample
//   1 byte  = step-table index + reserved byte
//   252 bytes = 504 x 4-bit ADPCM samples
//
// Total = 256 bytes.
//
// Therefore:
//
//   256-byte block = 505 decoded samples
//
// At 16 kHz this gives approximately 31.56 ms/block.
//
#define ADPCM_BLOCK_SIZE       256
#define ADPCM_SAMPLES_PER_BLOCK 505
#define ADPCM_FORMAT_CODE      0x0011
#define ADPCM_BITS_PER_SAMPLE  4

// ============================================================
// BLE SETTINGS
// ============================================================
//
// 247-byte ATT MTU allows 244-byte notification payloads.
//
// Packet:
//
//   bytes 0..3 = sequence number
//   bytes 4..243 = file data
//
#define BLE_DATA_PACKET_SIZE   244
#define BLE_DATA_PAYLOAD_SIZE  240

#define BLE_TX_WINDOW 8

#define BLE_TRANSFER_RETRY_MS       500
#define BLE_RETRANSMIT_DELAY_MS     2
#define BLE_PACKET_DELAY_MS         3

// ============================================================
// I2S
// ============================================================
i2s_chan_handle_t rx_handle = NULL;

// Forward declaration so setup() always sees it.
bool setupI2S();

// ============================================================
// RECORDING STATE
// ============================================================
File recordingFile;

bool is_actively_recording = false;

uint32_t audioDataBytes = 0;
uint32_t totalSamplesRecorded = 0;

String current_wav_filepath = "";
String recording_start_timestamp = "";

// ============================================================
// ADPCM STATE
// ============================================================
static const int16_t IMA_INDEX_TABLE[16] = {
    -1, -1, -1, -1,
     2,  4,  6,  8,
    -1, -1, -1, -1,
     2,  4,  6,  8
};

static const int16_t IMA_STEP_TABLE[89] = {
     7,     8,     9,    10,    11,    12,    13,    14,
    16,    17,    19,    21,    23,    25,    28,    31,
    34,    37,    41,    45,    50,    55,    60,    66,
    73,    80,    88,    97,   107,   118,   130,   143,
   157,   173,   190,   209,   230,   253,   279,   307,
   337,   371,   408,   449,   494,   544,   598,   658,
   724,   796,   876,   963,  1060,  1166,  1282,  1411,
  1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
 32767
};

int16_t adpcmPredictor = 0;
int adpcmIndex = 0;

// Current encoded ADPCM block.
uint8_t adpcmBlock[ADPCM_BLOCK_SIZE];
size_t adpcmBlockSamples = 0;
size_t adpcmBlockBytes = 0;

// ============================================================
// BLE UUIDs
// ============================================================
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TIME_CHAR_UUID         "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define FILE_CTRL_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define FILE_DATA_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"

#define DEVICE_NAME "ESP32C6-TimeDevice"

bool deviceConnected = false;

bool timeReceived = false;
time_t receivedTime = 0;
unsigned long timeReceivedMillis = 0;

BLECharacteristic *pFileCtrlCharacteristic = nullptr;
BLECharacteristic *pFileDatCharacteristic = nullptr;

// ============================================================
// BLE FILE TRANSFER
// ============================================================
const uint8_t FILE_EOF_MARKER[4] = {
    0xFF, 0xEE, 0xEE, 0xFF
};

volatile bool fileTransferActive = false;

volatile uint32_t transferAckSeq = 0;
volatile bool transferAckChanged = false;

volatile bool transferRetransmitRequested = false;
volatile uint32_t transferRetransmitSeq = 0;

volatile bool transferDoneAckReceived = false;

File transferFile;
String transferFilePath = "";

uint32_t transferFileSize = 0;

uint32_t transferBaseSeq = 0;
uint32_t transferNextSeq = 0;
uint32_t transferNextOffset = 0;

bool transferReadDone = false;
bool transferEofSent = false;

unsigned long transferLastAckMillis = 0;

// ============================================================
// GENERAL CONSTANTS
// ============================================================
#define DRAW_BUF_SIZE (SCREEN_WIDTH * 32)

const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_DELAY = 700;

const uint32_t SD_SPI_FREQUENCY = 4000000;

const unsigned long RECORDING_BLINK_DELAY = 500;

SPIClass sharedSPI(FSPI);

Arduino_DataBus *bus =
    new Arduino_HWSPI(
        TFT_DC,
        TFT_CS,
        TFT_SCLK,
        TFT_MOSI,
        GFX_NOT_DEFINED,
        &sharedSPI,
        true
    );

Arduino_GFX *gfx =
    new Arduino_ST7789(
        bus,
        TFT_RST,
        0,
        true,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        34,
        0,
        34,
        0
    );

static lv_color_t draw_buf[DRAW_BUF_SIZE];

static lv_obj_t *label = nullptr;
static lv_obj_t *recording_dot = nullptr;

// ============================================================
// MESSAGE / FOLDER SETTINGS
// ============================================================
const char *messages[] = {
    "Daily Log",
    "Actions",
    "To Dos",
    "Meeting"
};

const int TOTAL_MESSAGES =
    sizeof(messages) / sizeof(messages[0]);

int current_msg_index = 0;

// ============================================================
// DEVICE STATE
// ============================================================
enum DeviceState {
    STATE_SCROLLING,
    STATE_RECORDING
};

DeviceState device_state = STATE_SCROLLING;

// ============================================================
// BUTTON STATE
// ============================================================
bool last_button_reading = HIGH;
bool stable_button_state = HIGH;

unsigned long last_debounce_time = 0;
unsigned long button_press_start = 0;

bool long_press_handled = false;

// ============================================================
// RECORDING INDICATOR
// ============================================================
bool recording_dot_visible = false;
unsigned long last_recording_blink = 0;

// ============================================================
// TIME
// ============================================================
bool getCustomLocalTime(struct tm *timeinfo) {

    if (!timeReceived) {
        return false;
    }

    unsigned long elapsedSeconds =
        (millis() - timeReceivedMillis) / 1000;

    time_t currentEpoch =
        receivedTime + elapsedSeconds;

    localtime_r(&currentEpoch, timeinfo);

    return true;
}

String getTimestamp() {

    struct tm timeinfo;

    if (!getCustomLocalTime(&timeinfo)) {
        return "";
    }

    char timestamp[32];

    strftime(
        timestamp,
        sizeof(timestamp),
        "%Y%m%d_%H%M%S",
        &timeinfo
    );

    return String(timestamp);
}

// ============================================================
// LITTLE-ENDIAN HELPERS
// ============================================================
void writeLE16(File &file, uint16_t value) {

    file.write((uint8_t)(value & 0xFF));
    file.write((uint8_t)((value >> 8) & 0xFF));
}

void writeLE32(File &file, uint32_t value) {

    file.write((uint8_t)(value & 0xFF));
    file.write((uint8_t)((value >> 8) & 0xFF));
    file.write((uint8_t)((value >> 16) & 0xFF));
    file.write((uint8_t)((value >> 24) & 0xFF));
}

// ============================================================
// WAV HEADER
//
// IMA/DVI ADPCM WAV:
//
// RIFF
//   fmt  (20 bytes)
//   fact (4 bytes)
//   data
//
// fmt:
//
//   wFormatTag      = 0x11
//   nChannels       = 1
//   nSamplesPerSec  = 16000
//   nAvgBytesPerSec = sample_rate * block_align / samples_per_block
//   nBlockAlign     = 256
//   wBitsPerSample  = 4
//   cbSize          = 2
//   wSamplesPerBlock= 505
//   wNumCoef        = 7
//
// ============================================================
void writeWavHeader(
    File &file,
    uint32_t dataSize,
    uint32_t sampleCount
) {

    const uint16_t blockAlign =
        ADPCM_BLOCK_SIZE;

    const uint32_t avgBytesPerSec =
        (
            SAMPLE_RATE *
            blockAlign
        ) /
        ADPCM_SAMPLES_PER_BLOCK;

    const uint32_t riffSize =
        52 + dataSize;

    file.seek(0);

    // --------------------------------------------------------
    // RIFF
    // --------------------------------------------------------
    file.write(
        (const uint8_t *)"RIFF",
        4
    );

    writeLE32(
        file,
        riffSize
    );

    file.write(
        (const uint8_t *)"WAVE",
        4
    );

    // --------------------------------------------------------
    // fmt
    // --------------------------------------------------------
    file.write(
        (const uint8_t *)"fmt ",
        4
    );

    writeLE32(file, 20);

    // IMA ADPCM
    writeLE16(
        file,
        ADPCM_FORMAT_CODE
    );

    writeLE16(
        file,
        CHANNELS
    );

    writeLE32(
        file,
        SAMPLE_RATE
    );

    writeLE32(
        file,
        avgBytesPerSec
    );

    writeLE16(
        file,
        blockAlign
    );

    writeLE16(
        file,
        ADPCM_BITS_PER_SAMPLE
    );

    // Extra format bytes.
    writeLE16(file, 2);

    // Samples per ADPCM block.
    writeLE16(
        file,
        ADPCM_SAMPLES_PER_BLOCK
    );

    // Number of coefficients.
    writeLE16(file, 7);

    // Standard IMA ADPCM coefficient table.
    const int16_t coefficients[7][2] = {
        {256,   0},
        {512, -256},
        {  0,   0},
        {192,  64},
        {460, -208},
        {392, -232},
        {488, -256}
    };

    for (int i = 0; i < 7; i++) {
        writeLE16(
            file,
            coefficients[i][0]
        );

        writeLE16(
            file,
            coefficients[i][1]
        );
    }

    // --------------------------------------------------------
    // fact
    // --------------------------------------------------------
    file.write(
        (const uint8_t *)"fact",
        4
    );

    writeLE32(file, 4);

    writeLE32(
        file,
        sampleCount
    );

    // --------------------------------------------------------
    // data
    // --------------------------------------------------------
    file.write(
        (const uint8_t *)"data",
        4
    );

    writeLE32(
        file,
        dataSize
    );
}

// ============================================================
// I2S INITIALIZATION
// ============================================================
bool setupI2S() {

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_MASTER
        );

    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = I2S_BUFFER_SAMPLES;

    esp_err_t result =
        i2s_new_channel(
            &chan_cfg,
            NULL,
            &rx_handle
        );

    if (result != ESP_OK) {

        Serial.printf(
            "[I2S] Failed to create channel: %d\n",
            result
        );

        rx_handle = NULL;

        return false;
    }

    i2s_std_config_t std_cfg = {

        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(
                SAMPLE_RATE
            ),

        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT,
                I2S_SLOT_MODE_MONO
            ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,

            .bclk =
                (gpio_num_t)I2S_BCLK,

            .ws =
                (gpio_num_t)I2S_WS,

            .dout =
                I2S_GPIO_UNUSED,

            .din =
                (gpio_num_t)I2S_DIN,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    std_cfg.slot_cfg.slot_mask =
        I2S_STD_SLOT_LEFT;

    result =
        i2s_channel_init_std_mode(
            rx_handle,
            &std_cfg
        );

    if (result != ESP_OK) {

        Serial.printf(
            "[I2S] Failed to initialize standard mode: %d\n",
            result
        );

        i2s_del_channel(rx_handle);
        rx_handle = NULL;

        return false;
    }

    result =
        i2s_channel_enable(
            rx_handle
        );

    if (result != ESP_OK) {

        Serial.printf(
            "[I2S] Failed to enable channel: %d\n",
            result
        );

        i2s_del_channel(rx_handle);
        rx_handle = NULL;

        return false;
    }

    Serial.println(
        "[I2S] Initialized."
    );

    return true;
}

// ============================================================
// ADPCM RESET
// ============================================================
void resetADPCM() {

    adpcmPredictor = 0;
    adpcmIndex = 0;

    adpcmBlockSamples = 0;
    adpcmBlockBytes = 0;

    memset(
        adpcmBlock,
        0,
        sizeof(adpcmBlock)
    );
}

// ============================================================
// CLAMP
// ============================================================
int16_t clamp16(int32_t value) {

    if (value > 32767) {
        return 32767;
    }

    if (value < -32768) {
        return -32768;
    }

    return (int16_t)value;
}

// ============================================================
// ENCODE ONE SAMPLE
// ============================================================
uint8_t encodeIMA4(
    int16_t sample
) {

    int32_t diff =
        (int32_t)sample -
        (int32_t)adpcmPredictor;

    int32_t step =
        IMA_STEP_TABLE[adpcmIndex];

    uint8_t code = 0;

    if (diff < 0) {
        code |= 8;
        diff = -diff;
    }

    int32_t tempStep = step;

    if (diff >= tempStep) {
        code |= 4;
        diff -= tempStep;
    }

    tempStep >>= 1;

    if (diff >= tempStep) {
        code |= 2;
        diff -= tempStep;
    }

    tempStep >>= 1;

    if (diff >= tempStep) {
        code |= 1;
    }

    // Reconstruct delta.
    int32_t delta =
        step >> 3;

    if (code & 4) {
        delta += step;
    }

    if (code & 2) {
        delta += step >> 1;
    }

    if (code & 1) {
        delta += step >> 2;
    }

    if (code & 8) {
        adpcmPredictor -= delta;
    } else {
        adpcmPredictor += delta;
    }

    adpcmPredictor =
        clamp16(adpcmPredictor);

    adpcmIndex +=
        IMA_INDEX_TABLE[code & 0x0F];

    if (adpcmIndex < 0) {
        adpcmIndex = 0;
    }

    if (adpcmIndex > 88) {
        adpcmIndex = 88;
    }

    return code & 0x0F;
}

// ============================================================
// WRITE COMPLETED ADPCM BLOCK
// ============================================================
bool writeADPCMBlock() {

    if (!recordingFile) {
        return false;
    }

    if (adpcmBlockSamples == 0) {
        return true;
    }

    // --------------------------------------------------------
    // A complete block is always 256 bytes.
    //
    // For a short final block, pad with encoded samples
    // derived from the last predictor.
    // --------------------------------------------------------
    while (
        adpcmBlockSamples <
        ADPCM_SAMPLES_PER_BLOCK
    ) {

        uint8_t nibble =
            encodeIMA4(
                adpcmPredictor
            );

        size_t nibbleIndex =
            adpcmBlockSamples - 1;

        size_t byteIndex =
            4 + (nibbleIndex / 2);

        if (
            (nibbleIndex & 1) == 0
        ) {
            adpcmBlock[byteIndex] =
                nibble;
        } else {
            adpcmBlock[byteIndex] |=
                nibble << 4;
        }

        adpcmBlockSamples++;
    }

    size_t written =
        recordingFile.write(
            adpcmBlock,
            ADPCM_BLOCK_SIZE
        );

    if (
        written !=
        ADPCM_BLOCK_SIZE
    ) {
        Serial.println(
            "[REC] ERROR: ADPCM block write failed."
        );

        return false;
    }

    audioDataBytes +=
        ADPCM_BLOCK_SIZE;

    adpcmBlockSamples = 0;
    adpcmBlockBytes = 0;

    memset(
        adpcmBlock,
        0,
        sizeof(adpcmBlock)
    );

    return true;
}

// ============================================================
// ADD ONE PCM SAMPLE TO ADPCM ENCODER
// ============================================================
bool addPCMToADPCM(
    int16_t sample
) {

    // --------------------------------------------------------
    // First sample is stored directly as predictor.
    // --------------------------------------------------------
    if (adpcmBlockSamples == 0) {

        adpcmPredictor =
            sample;

        adpcmIndex = 0;

        adpcmBlock[0] =
            (uint8_t)(
                adpcmPredictor &
                0xFF
            );

        adpcmBlock[1] =
            (uint8_t)(
                (
                    adpcmPredictor >>
                    8
                ) &
                0xFF
            );

        adpcmBlock[2] =
            (uint8_t)adpcmIndex;

        adpcmBlock[3] =
            0;

        adpcmBlockSamples = 1;

        return true;
    }

    uint8_t nibble =
        encodeIMA4(sample);

    size_t nibbleIndex =
        adpcmBlockSamples - 1;

    size_t byteIndex =
        4 + (nibbleIndex / 2);

    if (
        (nibbleIndex & 1) == 0
    ) {

        adpcmBlock[byteIndex] =
            nibble;

    } else {

        adpcmBlock[byteIndex] |=
            nibble << 4;
    }

    adpcmBlockSamples++;

    if (
        adpcmBlockSamples >=
        ADPCM_SAMPLES_PER_BLOCK
    ) {

        return writeADPCMBlock();
    }

    return true;
}

// ============================================================
// BLE CONTROL NOTIFICATION
// ============================================================
void sendControlNotification(
    const String &message
) {

    if (
        !deviceConnected ||
        pFileCtrlCharacteristic == nullptr
    ) {
        return;
    }

    pFileCtrlCharacteristic->setValue(
        message.c_str()
    );

    pFileCtrlCharacteristic->notify();
}

// ============================================================
// CONTROL CHUNK
// ============================================================
void sendControlChunk(
    const String &chunk
) {

    if (
        !deviceConnected ||
        pFileCtrlCharacteristic == nullptr
    ) {
        return;
    }

    pFileCtrlCharacteristic->setValue(
        chunk.c_str()
    );

    pFileCtrlCharacteristic->notify();

    delay(5);
}

// ============================================================
// IGNORE MACOS METADATA FILES
// ============================================================
bool isValidRecordingFilename(
    const String &filename
) {

    if (
        filename.startsWith("._")
    ) {
        return false;
    }

    if (
        filename == "." ||
        filename == ".."
    ) {
        return false;
    }

    if (
        filename.endsWith(".wav") ||
        filename.endsWith(".WAV")
    ) {
        return true;
    }

    return false;
}

// ============================================================
// LIST FILES
// ============================================================
void sendFileList() {

    sendControlNotification(
        "LIST_BEGIN"
    );

    for (
        int i = 0;
        i < TOTAL_MESSAGES;
        i++
    ) {

        String folderPath =
            "/" +
            String(messages[i]);

        File dir =
            SD.open(folderPath);

        if (
            !dir ||
            !dir.isDirectory()
        ) {

            if (dir) {
                dir.close();
            }

            continue;
        }

        File file =
            dir.openNextFile();

        while (file) {

            if (!file.isDirectory()) {

                String filename =
                    String(file.name());

                // ------------------------------------------------
                // IMPORTANT:
                // Ignore macOS AppleDouble files such as:
                //
                // ._20260929_....wav
                // ------------------------------------------------
                if (
                    isValidRecordingFilename(
                        filename
                    )
                ) {

                    String entry =
                        String(messages[i]) +
                        "/" +
                        filename +
                        "\n";

                    while (
                        entry.length() > 0
                    ) {

                        size_t chunkLength =
                            entry.length();

                        if (
                            chunkLength >
                            20
                        ) {
                            chunkLength = 20;
                        }

                        String chunk =
                            entry.substring(
                                0,
                                chunkLength
                            );

                        sendControlChunk(
                            chunk
                        );

                        entry =
                            entry.substring(
                                chunkLength
                            );
                    }
                }
            }

            file.close();

            file =
                dir.openNextFile();
        }

        dir.close();
    }

    sendControlNotification(
        "LIST_END"
    );
}

// ============================================================
// RESET FILE TRANSFER
// ============================================================
void resetFileTransferState() {

    if (transferFile) {
        transferFile.close();
    }

    transferFilePath = "";

    transferFileSize = 0;

    transferBaseSeq = 0;
    transferNextSeq = 0;
    transferNextOffset = 0;

    transferReadDone = false;
    transferEofSent = false;

    transferAckSeq = 0;
    transferAckChanged = false;

    transferRetransmitRequested = false;
    transferRetransmitSeq = 0;

    transferDoneAckReceived = false;

    transferLastAckMillis =
        millis();

    fileTransferActive = false;
}

// ============================================================
// START FILE TRANSFER
// ============================================================
bool startFileTransfer(
    const String &targetFile
) {

    if (fileTransferActive) {

        sendControlNotification(
            "ERR_BUSY"
        );

        return false;
    }

    String path =
        targetFile;

    if (
        !path.startsWith("/")
    ) {
        path =
            "/" + path;
    }

    // Do not allow macOS metadata files.
    String basename =
        path.substring(
            path.lastIndexOf('/') + 1
        );

    if (
        basename.startsWith("._")
    ) {

        sendControlNotification(
            "ERR_METADATA_FILE"
        );

        return false;
    }

    if (
        !SD.exists(path)
    ) {

        sendControlNotification(
            "ERR_NOT_FOUND"
        );

        return false;
    }

    File f =
        SD.open(
            path,
            FILE_READ
        );

    if (!f) {

        sendControlNotification(
            "ERR_OPEN"
        );

        return false;
    }

    resetFileTransferState();

    transferFile = f;
    transferFilePath = path;
    transferFileSize = transferFile.size();

    transferBaseSeq = 0;
    transferNextSeq = 0;
    transferNextOffset = 0;

    transferReadDone =
        (
            transferFileSize == 0
        );

    transferEofSent = false;

    transferDoneAckReceived = false;

    transferLastAckMillis =
        millis();

    fileTransferActive = true;

    Serial.printf(
        "[BLE] Starting transfer: %s (%lu bytes)\n",
        transferFilePath.c_str(),
        (unsigned long)transferFileSize
    );

    sendControlNotification(
        "START:" +
        String(transferFileSize)
    );

    return true;
}

// ============================================================
// BUILD DATA PACKET
// ============================================================
void buildDataPacket(
    uint32_t sequence,
    const uint8_t *payload,
    size_t payloadLength,
    uint8_t *packet,
    size_t *packetLength
) {

    packet[0] =
        (uint8_t)(
            sequence &
            0xFF
        );

    packet[1] =
        (uint8_t)(
            (sequence >> 8) &
            0xFF
        );

    packet[2] =
        (uint8_t)(
            (sequence >> 16) &
            0xFF
        );

    packet[3] =
        (uint8_t)(
            (sequence >> 24) &
            0xFF
        );

    memcpy(
        packet + 4,
        payload,
        payloadLength
    );

    *packetLength =
        4 + payloadLength;
}

// ============================================================
// SEND ONE DATA PACKET
// ============================================================
bool sendDataPacket(
    uint32_t sequence,
    uint32_t fileOffset
) {

    if (
        !transferFile ||
        pFileDatCharacteristic == nullptr
    ) {
        return false;
    }

    if (
        fileOffset >=
        transferFileSize
    ) {
        return false;
    }

    if (
        !transferFile.seek(
            fileOffset
        )
    ) {
        return false;
    }

    uint8_t payload[
        BLE_DATA_PAYLOAD_SIZE
    ];

    size_t bytesToRead =
        transferFileSize -
        fileOffset;

    if (
        bytesToRead >
        BLE_DATA_PAYLOAD_SIZE
    ) {
        bytesToRead =
            BLE_DATA_PAYLOAD_SIZE;
    }

    size_t bytesRead =
        transferFile.read(
            payload,
            bytesToRead
        );

    if (
        bytesRead == 0
    ) {
        return false;
    }

    uint8_t packet[
        BLE_DATA_PACKET_SIZE
    ];

    size_t packetLength = 0;

    buildDataPacket(
        sequence,
        payload,
        bytesRead,
        packet,
        &packetLength
    );

    pFileDatCharacteristic->setValue(
        packet,
        packetLength
    );

    pFileDatCharacteristic->notify();

    return true;
}

// ============================================================
// RETRANSMIT WINDOW
// ============================================================
void retransmitWindow(
    uint32_t startSequence
) {

    if (!fileTransferActive) {
        return;
    }

    if (
        startSequence >=
        transferNextSeq
    ) {
        return;
    }

    Serial.printf(
        "[BLE] Retransmitting seq %lu-%lu\n",
        (unsigned long)startSequence,
        (unsigned long)(
            transferNextSeq - 1
        )
    );

    for (
        uint32_t seq =
            startSequence;

        seq <
        transferNextSeq;

        seq++
    ) {

        uint32_t offset =
            seq *
            BLE_DATA_PAYLOAD_SIZE;

        if (
            offset >=
            transferFileSize
        ) {
            break;
        }

        if (
            !sendDataPacket(
                seq,
                offset
            )
        ) {
            break;
        }

        delay(
            BLE_RETRANSMIT_DELAY_MS
        );
    }
}

// ============================================================
// SEND EOF
// ============================================================
void sendFileEOF() {

    if (
        transferEofSent ||
        !fileTransferActive ||
        pFileDatCharacteristic == nullptr
    ) {
        return;
    }

    pFileDatCharacteristic->setValue(
        FILE_EOF_MARKER,
        sizeof(FILE_EOF_MARKER)
    );

    pFileDatCharacteristic->notify();

    transferEofSent = true;

    Serial.println(
        "[BLE] EOF sent."
    );
}

// ============================================================
// FINISH TRANSFER
// ============================================================
void finishFileTransfer() {

    if (transferFile) {
        transferFile.close();
    }

    Serial.printf(
        "[BLE] Transfer finished: %s (%lu bytes)\n",
        transferFilePath.c_str(),
        (unsigned long)transferFileSize
    );

    sendControlNotification(
        "DONE:" +
        String(transferFileSize)
    );

    transferFilePath = "";

    transferFileSize = 0;

    transferBaseSeq = 0;
    transferNextSeq = 0;
    transferNextOffset = 0;

    transferReadDone = false;
    transferEofSent = false;

    transferAckChanged = false;
    transferRetransmitRequested = false;

    transferDoneAckReceived = false;

    fileTransferActive = false;
}

// ============================================================
// PROCESS FILE TRANSFER
// ============================================================
void processFileTransfer() {

    if (!fileTransferActive) {
        return;
    }

    if (!deviceConnected) {

        Serial.println(
            "[BLE] Transfer cancelled: disconnected."
        );

        resetFileTransferState();

        return;
    }

    // --------------------------------------------------------
    // Explicit retransmission request.
    // --------------------------------------------------------
    if (
        transferRetransmitRequested
    ) {

        transferRetransmitRequested =
            false;

        uint32_t retransmitSeq =
            transferRetransmitSeq;

        retransmitWindow(
            retransmitSeq
        );

        transferLastAckMillis =
            millis();

        return;
    }

    // --------------------------------------------------------
    // Everything sent and ACKed.
    // --------------------------------------------------------
    if (
        transferReadDone &&
        transferBaseSeq ==
            transferNextSeq
    ) {

        if (!transferEofSent) {
            sendFileEOF();
        }

        return;
    }

    // --------------------------------------------------------
    // Fill TX window.
    // --------------------------------------------------------
    while (
        !transferReadDone &&
        (
            transferNextSeq -
            transferBaseSeq
        ) <
        BLE_TX_WINDOW
    ) {

        if (
            transferNextOffset >=
            transferFileSize
        ) {

            transferReadDone =
                true;

            break;
        }

        bool ok =
            sendDataPacket(
                transferNextSeq,
                transferNextOffset
            );

        if (!ok) {

            Serial.println(
                "[BLE] Data send failed."
            );

            return;
        }

        uint32_t remaining =
            transferFileSize -
            transferNextOffset;

        uint32_t bytesThisPacket =
            remaining;

        if (
            bytesThisPacket >
            BLE_DATA_PAYLOAD_SIZE
        ) {
            bytesThisPacket =
                BLE_DATA_PAYLOAD_SIZE;
        }

        transferNextOffset +=
            bytesThisPacket;

        transferNextSeq++;

        if (
            bytesThisPacket <
            BLE_DATA_PAYLOAD_SIZE
        ) {

            transferReadDone =
                true;

            break;
        }

        delay(
            BLE_PACKET_DELAY_MS
        );
    }

    // --------------------------------------------------------
    // ACK timeout.
    //
    // IMPORTANT:
    // This does NOT print every packet.
    // Only timeout/retransmission events are logged.
    // --------------------------------------------------------
    if (
        transferBaseSeq <
        transferNextSeq
    ) {

        if (
            millis() -
            transferLastAckMillis >
            BLE_TRANSFER_RETRY_MS
        ) {

            Serial.printf(
                "[BLE] ACK timeout; retrying from seq %lu\n",
                (unsigned long)
                    transferBaseSeq
            );

            retransmitWindow(
                transferBaseSeq
            );

            transferLastAckMillis =
                millis();
        }
    }
}

// ============================================================
// BLE SERVER CALLBACKS
// ============================================================
class MyServerCallbacks
    : public BLEServerCallbacks {

    void onConnect(
        BLEServer *pServer
    ) override {

        deviceConnected =
            true;

        Serial.println(
            "[BLE] Device connected."
        );
    }

    void onDisconnect(
        BLEServer *pServer
    ) override {

        deviceConnected =
            false;

        Serial.println(
            "[BLE] Device disconnected."
        );

        if (fileTransferActive) {
            resetFileTransferState();
        }

        pServer->startAdvertising();
    }
};

// ============================================================
// TIME CALLBACK
// ============================================================
class TimeCallbacks
    : public BLECharacteristicCallbacks {

    void onWrite(
        BLECharacteristic *pCharacteristic
    ) override {

        String timeData =
            pCharacteristic->getValue();

        if (
            timeData.length() == 0
        ) {
            return;
        }

        int year;
        int month;
        int day;
        int hour;
        int minute;
        int second;

        if (
            sscanf(
                timeData.c_str(),
                "%d-%d-%d %d:%d:%d",
                &year,
                &month,
                &day,
                &hour,
                &minute,
                &second
            ) == 6
        ) {

            struct tm timeinfo;

            memset(
                &timeinfo,
                0,
                sizeof(timeinfo)
            );

            timeinfo.tm_year =
                year - 1900;

            timeinfo.tm_mon =
                month - 1;

            timeinfo.tm_mday =
                day;

            timeinfo.tm_hour =
                hour;

            timeinfo.tm_min =
                minute;

            timeinfo.tm_sec =
                second;

            receivedTime =
                mktime(&timeinfo);

            timeReceivedMillis =
                millis();

            timeReceived =
                true;

            Serial.println(
                "[TIME] Synchronized."
            );
        }
    }
};

// ============================================================
// FILE CONTROL CALLBACK
// ============================================================
class FileCtrlCallbacks
    : public BLECharacteristicCallbacks {

    void onWrite(
        BLECharacteristic *pCharacteristic
    ) override {

        String cmd =
            pCharacteristic->getValue();

        cmd.trim();

        if (
            cmd.length() == 0
        ) {
            return;
        }

        // ----------------------------------------------------
        // Do not log every command except useful state changes.
        // ACK traffic is intentionally silent.
        // ----------------------------------------------------

        // ----------------------------------------------------
        // LIST
        // ----------------------------------------------------
        if (
            cmd == "LIST"
        ) {

            if (fileTransferActive) {

                sendControlNotification(
                    "ERR_BUSY"
                );

                return;
            }

            Serial.println(
                "[BLE] LIST requested."
            );

            sendFileList();

            return;
        }

        // ----------------------------------------------------
        // GET
        // ----------------------------------------------------
        if (
            cmd.startsWith("GET:")
        ) {

            String targetFile =
                cmd.substring(4);

            targetFile.trim();

            startFileTransfer(
                targetFile
            );

            return;
        }

        // ----------------------------------------------------
        // ACK
        // ----------------------------------------------------
        if (
            cmd.startsWith("ACK:")
        ) {

            if (!fileTransferActive) {
                return;
            }

            String ackString =
                cmd.substring(4);

            uint32_t ack =
                strtoul(
                    ackString.c_str(),
                    nullptr,
                    10
                );

            if (
                ack >
                transferNextSeq
            ) {

                Serial.printf(
                    "[BLE] Invalid ACK %lu.\n",
                    (unsigned long)ack
                );

                return;
            }

            if (
                ack <
                transferBaseSeq
            ) {
                return;
            }

            if (
                ack >
                transferBaseSeq
            ) {

                transferBaseSeq =
                    ack;

                transferAckSeq =
                    ack;

                transferAckChanged =
                    true;

                transferLastAckMillis =
                    millis();

                // ------------------------------------------------
                // Do NOT print every ACK.
                // ------------------------------------------------
            }

            if (
                ack <
                transferNextSeq
            ) {

                transferRetransmitSeq =
                    ack;

                transferRetransmitRequested =
                    true;

                transferLastAckMillis =
                    millis();
            }

            return;
        }

        // ----------------------------------------------------
        // ACKDONE
        // ----------------------------------------------------
        if (
            cmd == "ACKDONE"
        ) {

            if (
                fileTransferActive &&
                transferEofSent
            ) {

                transferDoneAckReceived =
                    true;

                Serial.println(
                    "[BLE] Client acknowledged completed transfer."
                );

                finishFileTransfer();
            }

            return;
        }

        // ----------------------------------------------------
        // DELETE
        // ----------------------------------------------------
        if (
            cmd.startsWith("DEL:")
        ) {

            if (fileTransferActive) {

                sendControlNotification(
                    "ERR_BUSY"
                );

                return;
            }

            String targetFile =
                cmd.substring(4);

            targetFile.trim();

            if (
                !targetFile.startsWith("/")
            ) {
                targetFile =
                    "/" + targetFile;
            }

            String basename =
                targetFile.substring(
                    targetFile.lastIndexOf('/') + 1
                );

            if (
                basename.startsWith("._")
            ) {

                sendControlNotification(
                    "ERR_METADATA_FILE"
                );

                return;
            }

            if (
                SD.exists(
                    targetFile
                )
            ) {

                if (
                    SD.remove(
                        targetFile
                    )
                ) {

                    Serial.printf(
                        "[SD] Deleted: %s\n",
                        targetFile.c_str()
                    );

                    sendControlNotification(
                        "OK_DELETED"
                    );

                } else {

                    sendControlNotification(
                        "ERR_DELETE"
                    );
                }

            } else {

                sendControlNotification(
                    "ERR_NOT_FOUND"
                );
            }

            return;
        }

        // ----------------------------------------------------
        // Unknown command
        // ----------------------------------------------------
        Serial.printf(
            "[BLE] Unknown command: %s\n",
            cmd.c_str()
        );

        sendControlNotification(
            "ERR_UNKNOWN"
        );
    }
};

// ============================================================
// UI
// ============================================================
void displayFlush(
    lv_display_t *display,
    const lv_area_t *area,
    uint8_t *px_map
) {

    uint32_t width =
        area->x2 -
        area->x1 +
        1;

    uint32_t height =
        area->y2 -
        area->y1 +
        1;

    gfx->draw16bitRGBBitmap(
        area->x1,
        area->y1,
        reinterpret_cast<uint16_t *>(px_map),
        width,
        height
    );

    lv_disp_flush_ready(display);
}

void centerLabel() {

    if (label) {

        lv_obj_align(
            label,
            LV_ALIGN_CENTER,
            0,
            0
        );
    }
}

void updateLabel() {

    lv_label_set_text(
        label,
        messages[current_msg_index]
    );

    centerLabel();

    lv_obj_invalidate(label);
}

void nextMessage() {

    if (
        device_state !=
        STATE_SCROLLING
    ) {
        return;
    }

    current_msg_index =
        (
            current_msg_index + 1
        ) %
        TOTAL_MESSAGES;

    Serial.printf(
        "[BUTTON] Short press -> message %d: %s\n",
        current_msg_index,
        messages[current_msg_index]
    );

    updateLabel();
}

// ============================================================
// RECORDING DOT
// ============================================================
void createRecordingDot() {

    if (
        recording_dot != nullptr
    ) {
        return;
    }

    recording_dot =
        lv_obj_create(
            lv_scr_act()
        );

    lv_obj_set_size(
        recording_dot,
        14,
        14
    );

    lv_obj_set_style_radius(
        recording_dot,
        LV_RADIUS_CIRCLE,
        0
    );

    lv_obj_set_style_bg_color(
        recording_dot,
        lv_color_hex(0xFF8C00),
        0
    );

    lv_obj_set_style_border_width(
        recording_dot,
        0,
        0
    );

    lv_obj_set_style_pad_all(
        recording_dot,
        0,
        0
    );

    lv_obj_align(
        recording_dot,
        LV_ALIGN_TOP_RIGHT,
        -10,
        10
    );
}

void showRecordingDot() {

    if (recording_dot) {

        lv_obj_clear_flag(
            recording_dot,
            LV_OBJ_FLAG_HIDDEN
        );
    }
}

void hideRecordingDot() {

    if (recording_dot) {

        lv_obj_add_flag(
            recording_dot,
            LV_OBJ_FLAG_HIDDEN
        );
    }
}

void updateRecordingIndicator() {

    if (
        device_state !=
        STATE_RECORDING
    ) {
        return;
    }

    unsigned long now =
        millis();

    if (
        now -
        last_recording_blink <
        RECORDING_BLINK_DELAY
    ) {
        return;
    }

    last_recording_blink =
        now;

    recording_dot_visible =
        !recording_dot_visible;

    if (
        recording_dot_visible
    ) {
        showRecordingDot();
    } else {
        hideRecordingDot();
    }
}

// ============================================================
// START RECORDING
// ============================================================
void startRecording() {

    String start_ts =
        getTimestamp();

    if (
        start_ts.length() == 0
    ) {

        start_ts =
            "unknown_start";
    }

    recording_start_timestamp =
        start_ts;

    String folderPath =
        "/" +
        String(
            messages[current_msg_index]
        );

    if (
        !SD.exists(
            folderPath
        )
    ) {

        SD.mkdir(
            folderPath
        );
    }

    current_wav_filepath =
        folderPath +
        "/" +
        recording_start_timestamp +
        "_to_recording.wav";

    if (
        SD.exists(
            current_wav_filepath
        )
    ) {

        SD.remove(
            current_wav_filepath
        );
    }

    recordingFile =
        SD.open(
            current_wav_filepath,
            FILE_WRITE
        );

    if (!recordingFile) {

        Serial.println(
            "[REC] ERROR: Failed to create file."
        );

        is_actively_recording =
            false;

        return;
    }

    // Reserve enough space for the header.
    uint8_t emptyHeader[60] = {
        0
    };

    recordingFile.write(
        emptyHeader,
        sizeof(emptyHeader)
    );

    audioDataBytes = 0;
    totalSamplesRecorded = 0;

    resetADPCM();

    is_actively_recording =
        true;

    Serial.printf(
        "[REC] START: %s\n",
        current_wav_filepath.c_str()
    );
}

// ============================================================
// POLL RECORDING DATA
// ============================================================
void pollRecordingData() {

    if (
        !is_actively_recording ||
        !recordingFile ||
        rx_handle == NULL
    ) {
        return;
    }

    int32_t i2sBuffer[
        I2S_BUFFER_SAMPLES
    ];

    size_t bytesRead = 0;

    esp_err_t result =
        i2s_channel_read(
            rx_handle,
            i2sBuffer,
            sizeof(i2sBuffer),
            &bytesRead,
            0
        );

    if (
        result != ESP_OK ||
        bytesRead == 0
    ) {
        return;
    }

    size_t samplesRead =
        bytesRead /
        sizeof(int32_t);

    for (
        size_t i = 0;
        i < samplesRead;
        i++
    ) {

        // ----------------------------------------------------
        // Convert 32-bit I2S microphone sample to 16-bit PCM.
        // Keep the same scaling used by the original program.
        // ----------------------------------------------------
        int32_t sample =
            i2sBuffer[i] >> 13;

        sample =
            clamp16(sample);

        if (
            !addPCMToADPCM(
                (int16_t)sample
            )
        ) {
            Serial.println(
                "[REC] ERROR writing ADPCM block."
            );

            return;
        }

        totalSamplesRecorded++;
    }
}

// ============================================================
// STOP RECORDING
// ============================================================
void stopRecording() {

    if (
        !is_actively_recording
    ) {
        return;
    }

    is_actively_recording =
        false;

    String stop_timestamp =
        getTimestamp();

    if (
        stop_timestamp.length() == 0
    ) {

        stop_timestamp =
            "unknown_end";
    }

    // --------------------------------------------------------
    // Write/pad final ADPCM block.
    // --------------------------------------------------------
    if (
        adpcmBlockSamples > 0
    ) {

        if (
            !writeADPCMBlock()
        ) {

            Serial.println(
                "[REC] ERROR finalizing ADPCM block."
            );
        }
    }

    // --------------------------------------------------------
    // Rewrite valid WAV header.
    // --------------------------------------------------------
    if (recordingFile) {

        recordingFile.flush();

        writeWavHeader(
            recordingFile,
            audioDataBytes,
            totalSamplesRecorded
        );

        recordingFile.flush();

        recordingFile.close();
    }

    String finalPath =
        "/" +
        String(
            messages[current_msg_index]
        ) +
        "/" +
        recording_start_timestamp +
        "_to_" +
        stop_timestamp +
        ".wav";

    if (
        SD.exists(
            current_wav_filepath
        )
    ) {

        if (
            SD.rename(
                current_wav_filepath,
                finalPath
            )
        ) {

            Serial.printf(
                "[REC] SAVED: %s | samples=%lu | ADPCM bytes=%lu\n",
                finalPath.c_str(),
                (unsigned long)
                    totalSamplesRecorded,
                (unsigned long)
                    audioDataBytes
            );

        } else {

            Serial.println(
                "[REC] ERROR: Failed to rename WAV."
            );
        }
    }

    current_wav_filepath = "";

    recording_start_timestamp = "";

    resetADPCM();
}

// ============================================================
// RECORDING MODE
// ============================================================
void startRecordingMode() {

    if (
        device_state ==
        STATE_RECORDING
    ) {
        return;
    }

    Serial.println(
        "[BUTTON] Long press -> START recording."
    );

    device_state =
        STATE_RECORDING;

    recording_dot_visible =
        true;

    last_recording_blink =
        millis();

    createRecordingDot();

    showRecordingDot();

    startRecording();

    if (
        !is_actively_recording
    ) {

        device_state =
            STATE_SCROLLING;

        hideRecordingDot();

        Serial.println(
            "[REC] Recording failed to start."
        );
    }
}

void stopRecordingMode() {

    if (
        device_state !=
        STATE_RECORDING
    ) {
        return;
    }

    Serial.println(
        "[BUTTON] Long press -> STOP recording."
    );

    stopRecording();

    device_state =
        STATE_SCROLLING;

    hideRecordingDot();
}

void toggleRecordingMode() {

    if (
        device_state ==
        STATE_RECORDING
    ) {

        stopRecordingMode();

    } else {

        startRecordingMode();
    }
}

// ============================================================
// BUTTON
// ============================================================
void handleButton() {

    bool current_reading =
        digitalRead(
            BUTTON_PIN
        );

    if (
        current_reading !=
        last_button_reading
    ) {

        last_debounce_time =
            millis();
    }

    if (
        millis() -
        last_debounce_time >
        DEBOUNCE_DELAY
    ) {

        if (
            current_reading !=
            stable_button_state
        ) {

            stable_button_state =
                current_reading;

            // ------------------------------------------------
            // BUTTON DOWN
            // ------------------------------------------------
            if (
                stable_button_state ==
                LOW
            ) {

                button_press_start =
                    millis();

                long_press_handled =
                    false;

                Serial.printf(
                    "[BUTTON] PRESS DOWN | state=%s\n",
                    device_state ==
                        STATE_RECORDING
                        ? "RECORDING"
                        : "SCROLLING"
                );

            // ------------------------------------------------
            // BUTTON UP
            // ------------------------------------------------
            } else {

                unsigned long pressDuration =
                    millis() -
                    button_press_start;

                Serial.printf(
                    "[BUTTON] RELEASE | duration=%lu ms\n",
                    pressDuration
                );

                // A short press changes the message.
                if (
                    !long_press_handled &&
                    device_state ==
                        STATE_SCROLLING
                ) {

                    nextMessage();
                }
            }
        }

        // ----------------------------------------------------
        // LONG PRESS
        // ----------------------------------------------------
        if (
            stable_button_state ==
            LOW &&
            !long_press_handled
        ) {

            if (
                millis() -
                button_press_start >=
                LONG_PRESS_DELAY
            ) {

                long_press_handled =
                    true;

                Serial.printf(
                    "[BUTTON] LONG PRESS | duration=%lu ms\n",
                    millis() -
                        button_press_start
                );

                toggleRecordingMode();
            }
        }
    }

    last_button_reading =
        current_reading;
}

// ============================================================
// SETUP
// ============================================================
void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println(
        "========================================"
    );
    Serial.println(
        "ESP32-C6 4-bit IMA ADPCM Recorder"
    );
    Serial.println(
        "========================================"
    );

    // --------------------------------------------------------
    // GPIO
    // --------------------------------------------------------
    pinMode(
        BUTTON_PIN,
        INPUT_PULLUP
    );

    pinMode(
        TFT_BL,
        OUTPUT
    );

    digitalWrite(
        TFT_BL,
        HIGH
    );

    pinMode(
        TFT_CS,
        OUTPUT
    );

    digitalWrite(
        TFT_CS,
        HIGH
    );

    pinMode(
        SD_CS,
        OUTPUT
    );

    digitalWrite(
        SD_CS,
        HIGH
    );

    // --------------------------------------------------------
    // Shared SPI
    // --------------------------------------------------------
    sharedSPI.begin(
        SD_SCLK,
        SD_MISO,
        SD_MOSI,
        -1
    );

    // --------------------------------------------------------
    // Display
    // --------------------------------------------------------
    gfx->begin();

    lv_init();

    lv_display_t *display =
        lv_display_create(
            SCREEN_WIDTH,
            SCREEN_HEIGHT
        );

    lv_display_set_buffers(
        display,
        draw_buf,
        nullptr,
        sizeof(draw_buf),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    lv_display_set_flush_cb(
        display,
        displayFlush
    );

    lv_obj_set_style_bg_color(
        lv_scr_act(),
        lv_color_hex(0x000000),
        LV_PART_MAIN
    );

    label =
        lv_label_create(
            lv_scr_act()
        );

    lv_label_set_text(
        label,
        messages[current_msg_index]
    );

    lv_obj_set_style_text_color(
        label,
        lv_color_hex(0x00FF00),
        LV_PART_MAIN
    );

    lv_obj_set_style_text_font(
        label,
        &lv_font_montserrat_20,
        LV_PART_MAIN
    );

    centerLabel();

    // --------------------------------------------------------
    // SD
    // --------------------------------------------------------
    if (
        SD.begin(
            SD_CS,
            sharedSPI,
            SD_SPI_FREQUENCY
        )
    ) {

        Serial.println(
            "[SD] Initialized."
        );

    } else {

        Serial.println(
            "[SD] Initialization FAILED."
        );
    }

    // --------------------------------------------------------
    // I2S
    // --------------------------------------------------------
    if (
        !setupI2S()
    ) {

        Serial.println(
            "[I2S] Initialization FAILED."
        );

    } else {

        Serial.println(
            "[I2S] Ready."
        );
    }

    // --------------------------------------------------------
    // BLE
    // --------------------------------------------------------
    BLEDevice::init(
        DEVICE_NAME
    );

    // Request support for the large notification packets.
    // The central should negotiate the MTU as well.
    BLEDevice::setMTU(247);

    BLEServer *pServer =
        BLEDevice::createServer();

    pServer->setCallbacks(
        new MyServerCallbacks()
    );

    BLEService *pService =
        pServer->createService(
            SERVICE_UUID
        );

    // --------------------------------------------------------
    // TIME CHARACTERISTIC
    // --------------------------------------------------------
    BLECharacteristic *pTimeChar =
        pService->createCharacteristic(
            TIME_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );

    pTimeChar->setCallbacks(
        new TimeCallbacks()
    );

    // --------------------------------------------------------
    // FILE CONTROL
    // --------------------------------------------------------
    pFileCtrlCharacteristic =
        pService->createCharacteristic(
            FILE_CTRL_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pFileCtrlCharacteristic->addDescriptor(
        new BLE2902()
    );

    pFileCtrlCharacteristic->setCallbacks(
        new FileCtrlCallbacks()
    );

    // --------------------------------------------------------
    // FILE DATA
    // --------------------------------------------------------
    pFileDatCharacteristic =
        pService->createCharacteristic(
            FILE_DATA_CHAR_UUID,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pFileDatCharacteristic->addDescriptor(
        new BLE2902()
    );

    // --------------------------------------------------------
    // BLE START
    // --------------------------------------------------------
    pService->start();

    BLEDevice::getAdvertising()
        ->addServiceUUID(
            SERVICE_UUID
        );

    BLEDevice::startAdvertising();

    Serial.println(
        "[BLE] Advertising."
    );

    Serial.println(
        "Setup complete."
    );
}

// ============================================================
// LOOP
// ============================================================
void loop() {

    handleButton();

    updateRecordingIndicator();

    lv_tick_inc(5);

    lv_timer_handler();

    if (
        device_state ==
        STATE_RECORDING
    ) {

        pollRecordingData();
    }

    // File transfer is deliberately handled outside
    // the BLE callback.
    processFileTransfer();

    delay(2);
}
