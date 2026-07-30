# python-can-usb8dev

A **pure-Python [python-can](https://python-can.readthedocs.io/) backend** for the
[8devices USB2CAN / Korlan](https://www.8devices.com/products/usb2can_korlan)
adapter (VID `0483`, PID `1234`).

It reimplements the vendor `usb2can.dll` (CANAL API) entirely in Python, speaking
the device's native `usb_8dev` USB protocol directly over libusb — **no vendor
DLL** and **no python-can `usb2can`/CANAL interface**. The protocol matches the
mainline Linux kernel driver `drivers/net/can/usb/usb_8dev.c` (used as
documentation, not copied). Works on **Windows, macOS, and Linux**.

## Install

```bash
pip install python-can-usb8dev
```

This pulls in `python-can`, `pyusb`, and `libusb-package` (which vendors the
libusb-1.0 native library as a wheel, so nothing native needs to be installed
separately).

## Use

It registers itself as the python-can interface **`usb8dev`**, so it behaves like
any built-in backend:

```python
import can

print(can.detect_available_configs(interfaces=["usb8dev"]))   # find adapters

with can.Bus(interface="usb8dev", channel="ED000001", bitrate=250000) as bus:
    for msg in bus:
        print(msg)
```

Works with python-can's CLIs too:

```bash
can_logger -i usb8dev -c ED000001 -b 250000 -f capture.asc
can_viewer -i usb8dev -c ED000001 -b 250000
```

### Options

| kwarg | default | meaning |
|-------|---------|---------|
| `channel` / `serial` | first found | device serial number |
| `bitrate` | `250000` | CAN bitrate (bit/s) |
| `listen_only` | `False` | silent / listen-only (real `SILENT` mode bit) |
| `loopback` | `False` | internal loopback, no bus needed (real `LOOPBACK` bit) |
| `one_shot` | `False` | disable automatic retransmission |
| `status_frames` | `True` | surface bus error/status frames as error `Message`s |
| `sample_point` | `0.875` | target sample point for bit-timing |

### Standalone recorder

The package also installs a small recorder / test harness, handy to confirm an
adapter works on its own:

```bash
usb8dev-record --list                      # list connected adapters
usb8dev-record -b 250000                    # print frames live
usb8dev-record -b 250000 -o capture.asc     # + log to file (.asc/.blf/.csv/.log)
usb8dev-record -b 250000 --loopback         # self-test with no bus attached
```

(Equivalently `python -m can_usb8dev …`.)

## Driver setup

- **Windows** — the adapter must be bound to a libusb-compatible driver. The
  [8devices WinUSB package](https://www.8devices.com/media/products/usb2can_korlan/downloads/usb2can_winusb.msi)
  binds WinUSB, which libusb drives; otherwise use [Zadig](https://zadig.akeo.ie/)
  to bind WinUSB or libusbK.
- **macOS** — no driver setup: no kernel driver claims this vendor device, so
  libusb talks to it directly.
- **Linux** — the mainline `usb_8dev` SocketCAN driver is the native path
  (`ip link set can0 up type can bitrate 250000`); this package is mainly useful
  on platforms without SocketCAN, but works on Linux too via libusb (detach the
  kernel driver first if it has claimed the device).

## How it works

- Bit timing is solved from the 32 MHz device clock at an 87.5% sample point
  (e.g. 250k → brp=8, tseg1=13, tseg2=2, sjw=2).
- Four bulk endpoints: data RX/TX and command RX/TX. A 16-byte command message
  (start `0x11` / end `0x22`, reply `opt1==0` == success) carries OPEN/CLOSE with
  timing + mode flags; CAN frames use fixed 21-byte (RX) / 16-byte (TX) framed
  messages with start `0x55` / end `0xAA`.

## License

MIT
