"""
Pure-Python driver for the 8devices USB2CAN / Korlan (VID 0483, PID 1234).

Reimplements the vendor ``usb2can.dll`` (CANAL API) entirely in Python by
speaking the device's native USB protocol directly over libusb (via pyusb).
No proprietary DLL, and no python-can ``usb2can`` interface.

The protocol is the one implemented by the mainline Linux kernel driver
``drivers/net/can/usb/usb_8dev.c`` (GPL) — used here as protocol documentation,
not copied. The device exposes four bulk endpoints (data RX/TX, command RX/TX),
a 16-byte command message with a start/end magic and an ``opt1``-is-status reply
convention, and fixed-size framed CAN messages on the data endpoints.

Exposed as a python-can :class:`~can.BusABC` subclass and registered as the
``usb8dev`` interface, so it plugs into the whole python-can ecosystem::

    import can
    with can.Bus(interface="usb8dev", channel=None, bitrate=250000) as bus:
        for msg in bus:
            print(msg)

Runtime deps: ``pyusb`` plus a libusb backend. ``libusb-package`` (a pip wheel
that vendors libusb-1.0 for win/mac/linux) is used automatically when present so
no native library needs to be installed or fetched separately.

Windows note: the adapter must be bound to a libusb-compatible driver (WinUSB or
libusbK). The 8devices WinUSB package binds WinUSB, which libusb can drive; if it
is bound to something else, use Zadig to switch it to WinUSB/libusbK.
macOS note: no kernel driver claims this vendor device, so libusb talks to it
directly — nothing extra to install beyond the pip wheels.
"""

import logging
import struct
import time
from collections import deque
from typing import List, Optional

import usb.core
import usb.util
from can import BusABC, BusState, CanProtocol, Message
from can.exceptions import CanInitializationError, CanOperationError

log = logging.getLogger("can.usb8dev")

# -- USB identity & endpoints ------------------------------------------------

USB_VID = 0x0483
USB_PID = 0x1234

# Endpoint numbers from the device (host perspective: RX = IN, TX = OUT).
EP_DATA_RX = 0x81  # IN  ep 1 — CAN frames from the bus
EP_DATA_TX = 0x02  # OUT ep 2 — CAN frames to the bus
EP_CMD_RX = 0x83   # IN  ep 3 — command replies
EP_CMD_TX = 0x04   # OUT ep 4 — commands

# -- Command protocol --------------------------------------------------------

CMD_START = 0x11
CMD_END = 0x22
CMD_SUCCESS = 0  # reply opt1 == 0 means success
CMD_MSG_LEN = 16

# Command opcodes (1-based, matching the device enum).
CMD_RESET = 1
CMD_OPEN = 2
CMD_CLOSE = 3
CMD_SET_SPEED = 4
CMD_GET_SOFTW_HARDW_VER = 12

BAUD_MANUAL = 0x09  # opt1 for OPEN: manual bit-timing follows in data[]

# Open-command mode flags (data[5:9], big-endian u32).
FLAG_SILENT = 0x01       # listen-only
FLAG_LOOPBACK = 0x02
FLAG_ONE_SHOT = 0x04     # disable automatic retransmission
FLAG_STATUS_FRAME = 0x08  # emit bus error/status frames

# -- Data frames -------------------------------------------------------------

DATA_START = 0x55
DATA_END = 0xAA

TYPE_CAN_FRAME = 0
TYPE_ERROR_FRAME = 3

MSG_EXTID = 0x01
MSG_RTR = 0x02
MSG_ERR = 0x04

RX_MSG_LEN = 21  # begin,type,flags,id(4),dlc,data(8),timestamp(4),end
TX_MSG_LEN = 16  # begin,flags,id(4),dlc,data(8),end
RX_BUFFER_SIZE = 64

# -- Bit timing --------------------------------------------------------------

ABP_CLOCK = 32_000_000  # 32 MHz CAN clock
TSEG1_MIN, TSEG1_MAX = 1, 16
TSEG2_MIN, TSEG2_MAX = 1, 8
SJW_MAX = 4
BRP_MIN, BRP_MAX = 1, 1024


def solve_bit_timing(bitrate: int, sample_point: float = 0.875):
    """Find (brp, tseg1, tseg2, sjw) for a bitrate on the 32 MHz clock.

    tseg1 is the combined ``prop_seg + phase_seg1`` the device expects.
    Prefers exact bitrate, the sample point closest to ``sample_point``, and
    the finest resolution (smallest BRP) as a tie-break — all within the
    device's timing limits.
    """
    best = None  # (sp_error, brp, tseg1, tseg2)
    for brp in range(BRP_MIN, BRP_MAX + 1):
        ntq = ABP_CLOCK / (brp * bitrate)
        if abs(ntq - round(ntq)) > 1e-9:
            continue  # bitrate must divide the clock exactly for this BRP
        ntq = round(ntq)
        if ntq < 1 + TSEG1_MIN + TSEG2_MIN:
            continue
        tseg1 = round(sample_point * ntq) - 1  # sample point = (1 + tseg1)/ntq
        tseg1 = max(TSEG1_MIN, min(TSEG1_MAX, tseg1))
        tseg2 = ntq - 1 - tseg1
        if not (TSEG2_MIN <= tseg2 <= TSEG2_MAX):
            continue
        sp_error = abs((1 + tseg1) / ntq - sample_point)
        cand = (sp_error, brp, tseg1, tseg2)
        if best is None or cand < best:
            best = cand
    if best is None:
        raise CanInitializationError(
            f"no valid bit timing for {bitrate} bit/s on a {ABP_CLOCK} Hz clock"
        )
    _, brp, tseg1, tseg2 = best
    sjw = min(SJW_MAX, tseg2)
    log.debug("bitrate %d -> brp=%d tseg1=%d tseg2=%d sjw=%d", bitrate, brp, tseg1, tseg2, sjw)
    return brp, tseg1, tseg2, sjw


# -- Device discovery --------------------------------------------------------

def _find(find_all=False):
    """usb.core.find() wrapper that prefers the vendored libusb backend."""
    kwargs = dict(find_all=find_all, idVendor=USB_VID, idProduct=USB_PID)
    try:
        import libusb_package

        return libusb_package.find(**kwargs)
    except ImportError:
        return usb.core.find(**kwargs)


def _serial_of(dev) -> Optional[str]:
    try:
        return usb.util.get_string(dev, dev.iSerialNumber)
    except (usb.core.USBError, ValueError):
        return None


# -- Bus ---------------------------------------------------------------------

class Usb8DevBus(BusABC):
    """python-can Bus for the 8devices USB2CAN/Korlan, in pure Python."""

    def __init__(
        self,
        channel: Optional[str] = None,
        bitrate: int = 250000,
        *,
        serial: Optional[str] = None,
        listen_only: bool = False,
        loopback: bool = False,
        one_shot: bool = False,
        status_frames: bool = True,
        sample_point: float = 0.875,
        state: BusState = BusState.ACTIVE,
        **kwargs,
    ):
        """
        :param channel: device serial number; ``None`` picks the first found.
        :param serial: alias for ``channel`` (python-can/usb2can convention).
        :param listen_only: silent / listen-only mode (no ACK, no TX).
        :param state: python-can convention — ``BusState.PASSIVE`` selects
            listen-only (equivalent to ``listen_only=True``); ``BusState.ACTIVE``
            (the default) is normal operation.
        :param loopback: hardware internal loopback — a self-test mode with no
            bus attached. This is NOT python-can's receive-own-messages echo.
        :param one_shot: disable automatic retransmission.
        :param status_frames: surface bus error/status frames as error Messages.
        """
        listen_only = listen_only or state == BusState.PASSIVE
        self._want = serial or channel
        self._dev = self._acquire()
        if self._dev is None:
            raise CanInitializationError(
                "no USB2CAN device found"
                + (f" with serial {self._want!r}" if self._want else "")
            )

        self._serial = _serial_of(self._dev)
        self.channel_info = f"USB2CAN {self._serial or '(unknown serial)'}"
        self._can_protocol = CanProtocol.CAN_20
        self._state = BusState.PASSIVE if listen_only else BusState.ACTIVE
        self._rx = deque()  # parsed Messages not yet handed to _recv_internal
        self._status_frames = status_frames

        self._configure()

        # Recover from a previous unclean exit: a stale CLOSE just errors, which
        # we ignore. Then OPEN with our timing + mode flags.
        self._send_cmd(CMD_CLOSE, check=False)
        self._open(bitrate, sample_point, listen_only, loopback, one_shot)

        super().__init__(channel, **kwargs)

    @property
    def state(self) -> BusState:
        # BusABC's default getter is hard-coded to ACTIVE; report the real state.
        return self._state

    # -- device acquisition / configuration --

    def _acquire(self):
        """Return the first matching device (by serial if requested), or None."""
        for dev in (_find(find_all=True) or []):
            if self._want is None or _serial_of(dev) == self._want:
                return dev
        return None

    def _configure(self):
        """Set configuration + claim, recovering from a stuck device.

        On macOS/libusb (and after an unclean previous session) ``set_configuration``
        can fail with a generic USBError even though the device is fine. A USB
        reset clears it; the device re-enumerates, so we re-acquire and retry.
        """
        last = None
        for _ in range(3):
            try:
                self._dev.set_configuration()
                self._dev.get_active_configuration()
                break
            except usb.core.USBError as e:
                last = e
                try:
                    self._dev.reset()
                except usb.core.USBError:
                    pass
                usb.util.dispose_resources(self._dev)
                # reset re-enumerates the device; wait for it to reappear.
                dev = None
                for _ in range(20):
                    time.sleep(0.2)
                    dev = self._acquire()
                    if dev is not None:
                        break
                if dev is not None:
                    self._dev = dev
        else:
            raise CanInitializationError(f"could not configure device: {last}")

        # Linux may have the kernel usb_8dev driver attached; detach it. Harmless
        # (and absent) on Windows/macOS.
        try:
            if self._dev.is_kernel_driver_active(0):
                self._dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            usb.util.claim_interface(self._dev, 0)
        except usb.core.USBError:
            pass

    # -- command channel --

    def _send_cmd(self, command: int, opt1: int = 0, opt2: int = 0,
                  data: bytes = b"", check: bool = True) -> Optional[bytes]:
        msg = bytearray(CMD_MSG_LEN)
        msg[0] = CMD_START
        msg[1] = 0  # channel
        msg[2] = command
        msg[3] = opt1
        msg[4] = opt2
        msg[5:5 + len(data)] = data
        msg[15] = CMD_END
        try:
            self._dev.write(EP_CMD_TX, bytes(msg), timeout=1000)
            reply = self._dev.read(EP_CMD_RX, CMD_MSG_LEN, timeout=1000)
        except usb.core.USBError as e:
            if check:
                raise CanInitializationError(f"command {command} failed: {e}") from e
            return None
        reply = bytes(reply)
        if check and (len(reply) != CMD_MSG_LEN or reply[0] != CMD_START
                      or reply[15] != CMD_END or reply[3] != CMD_SUCCESS):
            raise CanInitializationError(
                f"command {command} rejected (reply={reply.hex()})"
            )
        return reply

    def _open(self, bitrate, sample_point, listen_only, loopback, one_shot):
        brp, tseg1, tseg2, sjw = solve_bit_timing(bitrate, sample_point)
        flags = FLAG_STATUS_FRAME if self._status_frames else 0
        if listen_only:
            flags |= FLAG_SILENT
        if loopback:
            flags |= FLAG_LOOPBACK
        if one_shot:
            flags |= FLAG_ONE_SHOT
        # data: tseg1, tseg2, sjw, brp(be16), flags(be32)
        payload = struct.pack(">BBBHI", tseg1, tseg2, sjw, brp, flags)
        self._send_cmd(CMD_OPEN, opt1=BAUD_MANUAL, data=payload)

    # -- receive --

    def _recv_internal(self, timeout):
        if not self._rx:
            timeout_ms = 0 if timeout is None else max(1, int(timeout * 1000))
            try:
                buf = self._dev.read(EP_DATA_RX, RX_BUFFER_SIZE, timeout=timeout_ms)
            except usb.core.USBError as e:
                # libusb reports a read timeout via errno 110 (ETIMEDOUT); pyusb
                # also raises the USBTimeoutError subclass. Both mean "no frame".
                if isinstance(e, usb.core.USBTimeoutError) or e.errno in (110, 60):
                    return None, False
                raise CanOperationError(f"USB read failed: {e}") from e
            self._parse(bytes(buf))
        if self._rx:
            return self._rx.popleft(), False
        return None, False

    def _parse(self, buf: bytes):
        pos = 0
        n = len(buf)
        while pos + RX_MSG_LEN <= n:
            frame = buf[pos:pos + RX_MSG_LEN]
            pos += RX_MSG_LEN
            if frame[0] != DATA_START or frame[-1] != DATA_END:
                log.warning("dropping malformed USB frame: %s", frame.hex())
                continue
            ftype = frame[1]
            flags = frame[2]
            can_id = struct.unpack(">I", frame[3:7])[0]
            extended = bool(flags & MSG_EXTID)
            can_id &= 0x1FFFFFFF if extended else 0x7FF
            dlc = frame[7] & 0x0F
            data = bytes(frame[8:8 + dlc])
            # layout: begin[0] type[1] flags[2] id[3:7] dlc[7] data[8:16]
            #         timestamp[16:20] end[20]
            ts = struct.unpack(">I", frame[16:20])[0] / 1000.0  # device ms -> s

            if ftype == TYPE_ERROR_FRAME and flags == MSG_ERR:
                if self._status_frames:
                    self._rx.append(Message(
                        timestamp=ts, is_error_frame=True,
                        arbitration_id=can_id, dlc=dlc, data=data,
                        channel=self._serial, is_rx=True,
                    ))
                continue
            if ftype != TYPE_CAN_FRAME:
                continue
            self._rx.append(Message(
                timestamp=ts,
                arbitration_id=can_id,
                is_extended_id=extended,
                is_remote_frame=bool(flags & MSG_RTR),
                dlc=dlc,
                data=data,
                channel=self._serial,
                is_rx=True,
            ))

    # -- transmit --

    def send(self, msg: Message, timeout: Optional[float] = None):
        flags = 0
        if msg.is_extended_id:
            flags |= MSG_EXTID
        if msg.is_remote_frame:
            flags |= MSG_RTR
        dlc = min(msg.dlc, 8)
        data = bytes(msg.data[:dlc]).ljust(8, b"\x00")
        frame = struct.pack(">BBIB8sB", DATA_START, flags,
                            msg.arbitration_id & 0x1FFFFFFF, dlc, data, DATA_END)
        timeout_ms = 0 if timeout is None else max(1, int(timeout * 1000))
        try:
            self._dev.write(EP_DATA_TX, frame, timeout=timeout_ms)
        except usb.core.USBError as e:
            raise CanOperationError(f"USB write failed: {e}") from e

    # -- lifecycle --

    def shutdown(self):
        super().shutdown()
        try:
            self._send_cmd(CMD_CLOSE, check=False)
        finally:
            usb.util.dispose_resources(self._dev)

    @staticmethod
    def _detect_available_configs() -> List[dict]:
        # Never raise from discovery — callers (Wireshark, can_viewer) enumerate
        # at startup and a missing backend must not crash them. __init__ still
        # surfaces a real error when actually opening a device.
        try:
            devices = _find(find_all=True) or []
        except usb.core.NoBackendError:
            log.warning("no libusb backend available")
            return []
        return [{"interface": "usb8dev", "channel": _serial_of(d)} for d in devices]
