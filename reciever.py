import asyncio
import os
import struct
from datetime import datetime

from bleak import BleakClient, BleakScanner


# ============================================================
# BLE UUIDS
# ============================================================
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

TIME_CHAR_UUID = (
    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
)

FILE_CTRL_UUID = (
    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
)

FILE_DATA_UUID = (
    "beb5483e-36e1-4688-b7f5-ea07361b26aa"
)

DEVICE_NAME = "ESP32C6-TimeDevice"


# ============================================================
# TRANSFER SETTINGS
# ============================================================
DATA_HEADER_SIZE = 4
DATA_PAYLOAD_SIZE = 16

ACK_EVERY = 4

TRANSFER_TIMEOUT = 180.0


# ============================================================
# GLOBAL STATE
# ============================================================
file_list_data = ""
file_list_complete = None
list_collecting = False

file_data_buffer = bytearray()

expected_sequence = 0
expected_file_size = None

transfer_start_received = False
transfer_eof_received = False

file_download_complete = None

ack_event = None
pending_ack = None

ble_client = None


# ============================================================
# CONTROL NOTIFICATION HANDLER
# ============================================================
def ctrl_notification_handler(sender, data):
    global file_list_data
    global list_collecting
    global expected_file_size
    global transfer_start_received

    decoded = data.decode(
        "utf-8",
        errors="ignore",
    )

    # --------------------------------------------------------
    # LIST BEGIN
    # --------------------------------------------------------
    if decoded == "LIST_BEGIN":
        file_list_data = ""
        list_collecting = True
        return

    # --------------------------------------------------------
    # LIST END
    # --------------------------------------------------------
    if decoded == "LIST_END":
        list_collecting = False

        if file_list_complete:
            file_list_complete.set()

        return

    # --------------------------------------------------------
    # FILE LIST DATA
    # --------------------------------------------------------
    if list_collecting:
        file_list_data += decoded
        return

    # --------------------------------------------------------
    # TRANSFER START
    # --------------------------------------------------------
    if decoded.startswith("START:"):
        try:
            expected_file_size = int(
                decoded[6:].strip()
            )

            transfer_start_received = True

            print(
                f"[BLE] ESP32 reports file size: "
                f"{expected_file_size} bytes"
            )

        except ValueError:
            print(
                f"[BLE] Invalid START message: {decoded}"
            )

        return

    # --------------------------------------------------------
    # TRANSFER DONE
    # --------------------------------------------------------
    if decoded.startswith("DONE:"):
        print(
            f"[BLE] ESP32 confirmed transfer completion: "
            f"{decoded}"
        )
        return

    # --------------------------------------------------------
    # DELETE RESPONSE
    # --------------------------------------------------------
    if decoded in (
        "OK_DELETED",
        "ERR_NOT_FOUND",
        "ERR_DELETE",
        "ERR_BUSY",
    ):
        print(
            f"[BLE Ctrl] Response: {decoded}"
        )
        return

    # --------------------------------------------------------
    # OTHER ERRORS
    # --------------------------------------------------------
    if decoded.startswith("ERR_"):
        print(
            f"[BLE Ctrl] Error: {decoded}"
        )


# ============================================================
# REQUEST CUMULATIVE ACK
# ============================================================
def request_ack(sequence):
    global pending_ack

    if (
        pending_ack is None
        or sequence > pending_ack
    ):
        pending_ack = sequence

    if ack_event:
        ack_event.set()


# ============================================================
# DATA NOTIFICATION HANDLER
# ============================================================
def data_notification_handler(sender, data):
    global file_data_buffer
    global expected_sequence
    global transfer_eof_received

    # --------------------------------------------------------
    # EOF MARKER
    # --------------------------------------------------------
    if (
        len(data) == 4
        and bytes(data) == b"\xff\xee\xee\xff"
    ):
        print(
            "[BLE DATA] EOF received."
        )

        transfer_eof_received = True

        if ble_client:
            asyncio.create_task(
                acknowledge_done()
            )

        return

    # --------------------------------------------------------
    # Validate minimum packet size
    # --------------------------------------------------------
    if len(data) < DATA_HEADER_SIZE:
        print(
            f"[BLE DATA] Invalid packet length: "
            f"{len(data)}"
        )

        request_ack(expected_sequence)
        return

    # --------------------------------------------------------
    # Extract sequence number
    # --------------------------------------------------------
    sequence = struct.unpack_from(
        "<I",
        data,
        0,
    )[0]

    payload = bytes(
        data[DATA_HEADER_SIZE:]
    )

    # --------------------------------------------------------
    # EXPECTED PACKET
    # --------------------------------------------------------
    if sequence == expected_sequence:

        file_data_buffer.extend(
            payload
        )

        expected_sequence += 1

        # ACK periodically.
        if (
            expected_sequence % ACK_EVERY == 0
        ):
            request_ack(
                expected_sequence
            )

        # If this is the final short packet,
        # acknowledge immediately.
        if (
            len(payload) < DATA_PAYLOAD_SIZE
        ):
            request_ack(
                expected_sequence
            )

        return

    # --------------------------------------------------------
    # DUPLICATE PACKET
    # --------------------------------------------------------
    if sequence < expected_sequence:

        request_ack(
            expected_sequence
        )

        return

    # --------------------------------------------------------
    # PACKET ARRIVED OUT OF ORDER
    # --------------------------------------------------------
    print(
        f"[BLE DATA] Missing packet: "
        f"expected {expected_sequence}, "
        f"received {sequence}"
    )

    # Ask ESP32 to retransmit starting with the missing packet.
    request_ack(
        expected_sequence
    )


# ============================================================
# ACK WORKER
# ============================================================
async def ack_worker():
    global pending_ack

    while True:

        await ack_event.wait()

        while True:

            ack = pending_ack
            pending_ack = None

            if ack is None:
                break

            try:

                await ble_client.write_gatt_char(
                    FILE_CTRL_UUID,
                    f"ACK:{ack}".encode("utf-8"),
                    response=True,
                )

            except Exception as exc:

                print(
                    f"[BLE ACK] Failed to send ACK "
                    f"{ack}: {exc}"
                )

                await asyncio.sleep(0.1)

        ack_event.clear()


# ============================================================
# ACKNOWLEDGE EOF
# ============================================================
async def acknowledge_done():
    global file_download_complete

    try:

        await ble_client.write_gatt_char(
            FILE_CTRL_UUID,
            b"ACKDONE",
            response=True,
        )

        print(
            "[BLE] Sent ACKDONE."
        )

        if file_download_complete:
            file_download_complete.set()

    except Exception as exc:

        print(
            f"[BLE] Failed to send ACKDONE: {exc}"
        )


# ============================================================
# MAIN
# ============================================================
async def main():

    global file_list_data
    global file_list_complete

    global file_data_buffer

    global file_download_complete

    global ack_event
    global pending_ack

    global expected_sequence
    global expected_file_size

    global transfer_start_received
    global transfer_eof_received

    global ble_client

    # --------------------------------------------------------
    # Initialize events inside running event loop.
    # --------------------------------------------------------
    file_list_complete = asyncio.Event()
    file_download_complete = asyncio.Event()
    ack_event = asyncio.Event()

    pending_ack = None

    # --------------------------------------------------------
    # Find ESP32
    # --------------------------------------------------------
    print(
        f"Searching for BLE device: {DEVICE_NAME}..."
    )

    device = await BleakScanner.find_device_by_filter(
        lambda d, ad:
            d.name is not None
            and DEVICE_NAME in d.name,
        timeout=10.0,
    )

    if not device:
        print(
            f"Error: Device '{DEVICE_NAME}' not found."
        )
        return

    print(
        f"Found {DEVICE_NAME} "
        f"(ID: {device.address})."
    )

    print(
        "Connecting..."
    )

    # --------------------------------------------------------
    # Connect
    # --------------------------------------------------------
    async with BleakClient(device) as client:

        ble_client = client

        print(
            f"Connected: {client.is_connected}"
        )

        print(
            f"Negotiated MTU: {client.mtu_size}"
        )

        # ----------------------------------------------------
        # Start ACK worker
        # ----------------------------------------------------
        ack_task = asyncio.create_task(
            ack_worker()
        )

        try:

            # ------------------------------------------------
            # Enable notifications
            # ------------------------------------------------
            await client.start_notify(
                FILE_CTRL_UUID,
                ctrl_notification_handler,
            )

            await client.start_notify(
                FILE_DATA_UUID,
                data_notification_handler,
            )

            # ------------------------------------------------
            # Time synchronization
            # ------------------------------------------------
            current_time = datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S"
            )

            await client.write_gatt_char(
                TIME_CHAR_UUID,
                current_time.encode("utf-8"),
                response=True,
            )

            print(
                f"Synced time to ESP32-C6: "
                f"{current_time}"
            )

            await asyncio.sleep(0.5)

            # ------------------------------------------------
            # Request file list
            # ------------------------------------------------
            print(
                "Requesting WAV file list..."
            )

            file_list_data = ""

            file_list_complete.clear()

            await client.write_gatt_char(
                FILE_CTRL_UUID,
                b"LIST",
                response=True,
            )

            try:

                await asyncio.wait_for(
                    file_list_complete.wait(),
                    timeout=10.0,
                )

            except asyncio.TimeoutError:

                print(
                    "Timeout waiting for file list."
                )

                return

            # ------------------------------------------------
            # Parse file list
            # ------------------------------------------------
            files = [
                f.strip()
                for f in file_list_data.split("\n")
                if f.strip()
            ]

            # Remove duplicates while preserving order.
            files = list(
                dict.fromkeys(files)
            )

            if not files:

                print(
                    "No WAV recordings found."
                )

                return

            print(
                f"Found {len(files)} WAV file(s):"
            )

            for filepath in files:
                print(
                    f"  {filepath}"
                )

            # ------------------------------------------------
            # Download files
            # ------------------------------------------------
            for filepath in files:

                print()
                print(
                    "=" * 60
                )

                print(
                    f"Downloading: {filepath}"
                )

                # --------------------------------------------
                # RESET TRANSFER STATE
                #
                # IMPORTANT:
                # file_data_buffer is GLOBAL.
                # This is the fix for your 0-byte problem.
                # --------------------------------------------
                file_data_buffer.clear()

                expected_sequence = 0

                expected_file_size = None

                transfer_start_received = False

                transfer_eof_received = False

                file_download_complete.clear()

                pending_ack = None

                # --------------------------------------------
                # Request file
                # --------------------------------------------
                try:

                    await client.write_gatt_char(
                        FILE_CTRL_UUID,
                        f"GET:{filepath}".encode(
                            "utf-8"
                        ),
                        response=True,
                    )

                except Exception as exc:

                    print(
                        f"Failed to request "
                        f"{filepath}: {exc}"
                    )

                    continue

                # --------------------------------------------
                # Wait for EOF/ACKDONE
                # --------------------------------------------
                try:

                    await asyncio.wait_for(
                        file_download_complete.wait(),
                        timeout=TRANSFER_TIMEOUT,
                    )

                except asyncio.TimeoutError:

                    print(
                        f"TIMEOUT downloading "
                        f"{filepath}"
                    )

                    print(
                        f"Received so far: "
                        f"{len(file_data_buffer)} bytes"
                    )

                    print(
                        f"Expected packets: "
                        f"{expected_sequence}"
                    )

                    continue

                # --------------------------------------------
                # Verify EOF
                # --------------------------------------------
                if not transfer_eof_received:

                    print(
                        f"ERROR: transfer for "
                        f"{filepath} completed "
                        f"without EOF."
                    )

                    continue

                # --------------------------------------------
                # Verify file size
                # --------------------------------------------
                received_size = len(
                    file_data_buffer
                )

                print(
                    f"Received: {received_size} bytes"
                )

                if expected_file_size is not None:

                    print(
                        f"Expected: "
                        f"{expected_file_size} bytes"
                    )

                    if (
                        received_size
                        != expected_file_size
                    ):

                        print(
                            f"ERROR: size mismatch "
                            f"for {filepath}"
                        )

                        print(
                            "File will NOT be "
                            "deleted from SD card."
                        )

                        continue

                # --------------------------------------------
                # Basic WAV validation
                # --------------------------------------------
                if received_size < 44:

                    print(
                        f"ERROR: {filepath} is "
                        f"too small to be a WAV."
                    )

                    continue

                if (
                    file_data_buffer[0:4] != b"RIFF"
                    or
                    file_data_buffer[8:12] != b"WAVE"
                ):

                    print(
                        f"ERROR: {filepath} does "
                        f"not contain a valid "
                        f"RIFF/WAVE header."
                    )

                    print(
                        "File will NOT be deleted."
                    )

                    continue

                # --------------------------------------------
                # Save locally
                # --------------------------------------------
                local_path = os.path.join(
                    ".",
                    filepath,
                )

                local_directory = os.path.dirname(
                    local_path
                )

                if local_directory:
                    os.makedirs(
                        local_directory,
                        exist_ok=True,
                    )

                with open(
                    local_path,
                    "wb",
                ) as f:

                    f.write(
                        file_data_buffer
                    )

                print(
                    f"Successfully saved: "
                    f"{local_path}"
                )

                print(
                    f"File size: "
                    f"{received_size} bytes"
                )

                # --------------------------------------------
                # Delete from SD card only after verification
                # --------------------------------------------
                print(
                    f"Deleting from SD card: "
                    f"{filepath}"
                )

                try:

                    await client.write_gatt_char(
                        FILE_CTRL_UUID,
                        f"DEL:{filepath}".encode(
                            "utf-8"
                        ),
                        response=True,
                    )

                except Exception as exc:

                    print(
                        f"Delete request failed: "
                        f"{exc}"
                    )

                    continue

                await asyncio.sleep(0.75)

            print()
            print(
                "=" * 60
            )

            print(
                "All file synchronization completed."
            )

        finally:

            ack_task.cancel()

            try:
                await ack_task
            except asyncio.CancelledError:
                pass

            ble_client = None


# ============================================================
# ENTRY POINT
# ============================================================
if __name__ == "__main__":

    try:

        asyncio.run(main())

    except KeyboardInterrupt:

        print(
            "\nScript stopped by user."
        )

    except Exception as exc:

        print(
            f"\nFatal error: {exc}"
        )

