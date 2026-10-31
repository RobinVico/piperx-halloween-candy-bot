"""Windows CAN settings for the candleLight USB-CAN adapter (VID 1D50 PID 606F, WinUSB driver).

Uses the `candle` python-can interface (python-can-candle), which talks to WinUSB directly.
The libusb-based `gs_usb` interface fails on this composite device with "Access denied".
"""

CAN_INTERFACE = "candle"
CAN_CHANNEL = 0  # channel 0 of the first candleLight adapter; must be an int, not "0"
CAN_BITRATE = 1_000_000  # PiPER / PiPER X


def list_adapters() -> list[str]:
    import candle_api

    return [f"{d.manufacturer} {d.product} SN={d.serial_number} ch={len(d)}" for d in candle_api.list_device()]
