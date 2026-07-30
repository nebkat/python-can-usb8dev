"""Protocol-level unit tests — no hardware, no libusb backend required.

They exercise the pure encode/decode logic by constructing a bus via
``__new__`` (bypassing the USB open) and driving it with synthetic frames and a
fake device that records/returns bytes.
"""

import struct

import can
import pytest
from can.exceptions import CanInitializationError

from can_usb8dev import Usb8DevBus
from can_usb8dev import bus as m

# -- bit timing --------------------------------------------------------------

@pytest.mark.parametrize("bitrate,expected", [
    (1000000, (2, 13, 2, 2)),
    (500000, (4, 13, 2, 2)),
    (250000, (8, 13, 2, 2)),   # ISOBUS / J1939
    (125000, (16, 13, 2, 2)),
])
def test_bit_timing_standard_rates(bitrate, expected):
    assert m.solve_bit_timing(bitrate) == expected


def test_bit_timing_sample_point():
    brp, tseg1, tseg2, _ = m.solve_bit_timing(250000)
    ntq = 1 + tseg1 + tseg2
    assert (1 + tseg1) / ntq == pytest.approx(0.875)
    assert m.ABP_CLOCK / (brp * ntq) == 250000


def test_bit_timing_impossible_rate_raises():
    with pytest.raises(CanInitializationError):
        m.solve_bit_timing(83333)


# -- helpers -----------------------------------------------------------------

def make_rx(ftype, flags, can_id, dlc, data, ts_ms):
    """Build a 21-byte device RX frame."""
    payload = bytes(data).ljust(8, b"\x00")
    return struct.pack(">BBBIB8sIB", m.DATA_START, ftype, flags, can_id, dlc,
                       payload, ts_ms, m.DATA_END)


class FakeDev:
    """Minimal usb.core.Device stand-in: records writes, returns canned reads."""

    def __init__(self, cmd_reply=None):
        self.writes = []
        # default: a CMD success reply (begin, .., opt1=0, .., end)
        r = bytearray(m.CMD_MSG_LEN)
        r[0] = m.CMD_START
        r[3] = m.CMD_SUCCESS
        r[15] = m.CMD_END
        self._cmd_reply = cmd_reply if cmd_reply is not None else bytes(r)

    def write(self, ep, data, timeout=None):
        self.writes.append((ep, bytes(data)))

    def read(self, ep, size, timeout=None):
        return self._cmd_reply


def bare_bus(status_frames=True):
    b = Usb8DevBus.__new__(Usb8DevBus)
    from collections import deque
    b._rx = deque()
    b._serial = "TEST"
    b._status_frames = status_frames
    b._dev = FakeDev()
    return b


# -- RX parsing --------------------------------------------------------------

def test_parse_standard_frame():
    b = bare_bus()
    b._parse(make_rx(m.TYPE_CAN_FRAME, 0x00, 0x123, 3, b"\x11\x22\x33", 100))
    msg = b._rx.popleft()
    assert not msg.is_extended_id and not msg.is_error_frame
    assert msg.arbitration_id == 0x123
    assert msg.dlc == 3
    assert bytes(msg.data) == b"\x11\x22\x33"
    assert msg.timestamp == pytest.approx(0.1)  # 100 ms -> 0.1 s (offset regression)


def test_parse_extended_frame():
    b = bare_bus()
    b._parse(make_rx(m.TYPE_CAN_FRAME, m.MSG_EXTID, 0x18FEF100, 8, bytes(range(8)), 0))
    msg = b._rx.popleft()
    assert msg.is_extended_id
    assert msg.arbitration_id == 0x18FEF100
    assert bytes(msg.data) == bytes(range(8))


def test_parse_error_frame():
    b = bare_bus()
    b._parse(make_rx(m.TYPE_ERROR_FRAME, m.MSG_ERR, 0, 0, b"", 0))
    msg = b._rx.popleft()
    assert msg.is_error_frame


def test_error_frame_suppressed_when_disabled():
    b = bare_bus(status_frames=False)
    b._parse(make_rx(m.TYPE_ERROR_FRAME, m.MSG_ERR, 0, 0, b"", 0))
    assert not b._rx


def test_parse_multiple_frames_in_one_buffer():
    b = bare_bus()
    buf = (make_rx(m.TYPE_CAN_FRAME, 0, 0x100, 1, b"\x01", 0)
           + make_rx(m.TYPE_CAN_FRAME, 0, 0x200, 1, b"\x02", 0))
    b._parse(buf)
    assert [msg.arbitration_id for msg in b._rx] == [0x100, 0x200]


def test_parse_drops_malformed():
    b = bare_bus()
    bad = bytearray(make_rx(m.TYPE_CAN_FRAME, 0, 0x100, 1, b"\x01", 0))
    bad[0] = 0x00  # corrupt start magic
    b._parse(bytes(bad))
    assert not b._rx


# -- TX encoding -------------------------------------------------------------

def test_send_frame_encoding():
    b = bare_bus()
    b.send(can.Message(arbitration_id=0x18EAFFFE, is_extended_id=True, data=b"\x00\xEE\x00"))
    ep, frame = b._dev.writes[0]
    assert ep == m.EP_DATA_TX
    assert len(frame) == m.TX_MSG_LEN == 16
    assert frame[0] == m.DATA_START and frame[-1] == m.DATA_END
    assert frame[1] & m.MSG_EXTID
    assert struct.unpack(">I", frame[2:6])[0] == 0x18EAFFFE
    assert frame[6] == 3  # dlc
    assert frame[7:10] == b"\x00\xEE\x00"


# -- OPEN command payload ----------------------------------------------------

def test_open_command_payload():
    b = bare_bus()
    b._open(250000, 0.875, listen_only=False, loopback=False, one_shot=False)
    ep, cmd = b._dev.writes[-1]
    assert ep == m.EP_CMD_TX
    assert cmd[0] == m.CMD_START and cmd[15] == m.CMD_END
    assert cmd[2] == m.CMD_OPEN
    assert cmd[3] == m.BAUD_MANUAL
    # data[0:3]=tseg1,tseg2,sjw ; data[3:5]=brp be16 ; data[5:9]=flags be32
    tseg1, tseg2, sjw = cmd[5], cmd[6], cmd[7]
    brp = struct.unpack(">H", cmd[8:10])[0]
    flags = struct.unpack(">I", cmd[10:14])[0]
    assert (brp, tseg1, tseg2, sjw) == (8, 13, 2, 2)
    assert flags == m.FLAG_STATUS_FRAME


def test_open_mode_flags():
    b = bare_bus()
    b._open(250000, 0.875, listen_only=True, loopback=True, one_shot=True)
    _, cmd = b._dev.writes[-1]
    flags = struct.unpack(">I", cmd[10:14])[0]
    assert flags == (m.FLAG_STATUS_FRAME | m.FLAG_SILENT | m.FLAG_LOOPBACK | m.FLAG_ONE_SHOT)
