"""
MO-NAV-EH Ground Station Receiver
===================================
Handles the actual FC radio protocol:

  Frame format (both directions):
    [0xAA] [SEQ:u8] [ID:u8] [LEN:u8] [DATA:LEN bytes] [CHECKSUM:u8]
    CHECKSUM = SEQ ^ ID ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]

  FC -> GC packet IDs:
    1  Positioning  36 B  3xf64 pos (m, ECEF-delta) + 3xf32 vel (m/s)
    2  Orientation  36 B  3xf32 MRP + 3xf32 ang_rate (rad/s) + 3xf32 mag (norm)
    3  Sensors      36 B  3xf32 accel (m/s^2) + 3xf32 accel_bias + 3xf32 gyro_bias
    4  Weather       0 B  stub - no payload yet
    5  Uncertainty  28 B  6xf32 pos/vel std-devs + 1xf32 heading_err (rad)
    6  Controls     10 B  f32 time + 6xu8 flags

  GC -> FC packet IDs:
    2  Data Request  5 B  [drogue_override, main_override, mode, fire_drogue, fire_main]
    3  ACK           1 B  [0x01 = good checksum / 0x00 = bad, please resend]
"""

import struct
import logging
import threading
import time
import serial

from rocket_state import RocketState

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
#  PROTOCOL CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────

START_BYTE   = 0xAA
MAX_PKT_SIZE = 64

# FC -> GC IDs
PKT_POSITIONING  = 1
PKT_ORIENTATION  = 2
PKT_SENSORS      = 3
PKT_WEATHER      = 4
PKT_UNCERTAINTY  = 5
PKT_CONTROLS     = 6

# GC -> FC IDs
PKT_GC_DATA_REQ  = 2
PKT_GC_ACK       = 3

# ─────────────────────────────────────────────────────────────────────────────
#  FRAME BUILDER  (GC -> FC direction)
# ─────────────────────────────────────────────────────────────────────────────

def _checksum(seq, pid, length, data):
    cs = seq ^ pid ^ length
    for b in data:
        cs ^= b
    return cs & 0xFF

def build_frame(seq, pid, data: bytes) -> bytes:
    length = len(data)
    cs = _checksum(seq, pid, length, data)
    return bytes([START_BYTE, seq & 0xFF, pid & 0xFF, length & 0xFF]) + data + bytes([cs])

def build_ack(seq, checksum_ok: bool) -> bytes:
    """GC -> FC ACK packet (ID=3)."""
    return build_frame(seq, PKT_GC_ACK, bytes([0x01 if checksum_ok else 0x00]))

def build_data_request(seq,
                       drogue_override=1, main_override=1,
                       mode=0, fire_drogue=0, fire_main=0) -> bytes:
    """GC -> FC data request (ID=2) - tells FC to start sending telemetry."""
    return build_frame(seq, PKT_GC_DATA_REQ,
                       bytes([drogue_override, main_override,
                              mode, fire_drogue, fire_main]))

# ─────────────────────────────────────────────────────────────────────────────
#  FRAME PARSER  (state machine - mirrors the FC's recieve() function)
# ─────────────────────────────────────────────────────────────────────────────

class FrameParser:
    """Feed bytes one at a time via push(byte).
    Returns (seq, pid, data_bytes) on a valid complete frame, None otherwise.
    """
    WAIT_START  = 0
    READ_SEQ    = 1
    READ_ID     = 2
    READ_LEN    = 3
    READ_DATA   = 4
    READ_CHKSUM = 5

    def __init__(self):
        self._state    = self.WAIT_START
        self._seq      = 0
        self._pid      = 0
        self._length   = 0
        self._data     = bytearray()
        self._checksum = 0
        self._last_rx_seq = None

    def push(self, byte: int):
        if self._state == self.WAIT_START:
            if byte == START_BYTE:
                self._checksum = 0
                self._state = self.READ_SEQ

        elif self._state == self.READ_SEQ:
            self._seq = byte
            self._checksum ^= byte
            self._state = self.READ_ID

        elif self._state == self.READ_ID:
            self._pid = byte
            self._checksum ^= byte
            self._state = self.READ_LEN

        elif self._state == self.READ_LEN:
            if byte > MAX_PKT_SIZE:
                logger.warning(f"Packet length {byte} exceeds max - discarding")
                self._state = self.WAIT_START
                return None
            self._length = byte
            self._checksum ^= byte
            self._data = bytearray()
            self._state = self.READ_DATA if byte > 0 else self.READ_CHKSUM

        elif self._state == self.READ_DATA:
            self._data.append(byte)
            self._checksum ^= byte
            if len(self._data) >= self._length:
                self._state = self.READ_CHKSUM

        elif self._state == self.READ_CHKSUM:
            self._state = self.WAIT_START
            if byte == self._checksum:
                return (self._seq, self._pid, bytes(self._data))
            else:
                logger.warning(
                    f"Checksum mismatch on pkt ID={self._pid}: "
                    f"got {byte:#04x}, expected {self._checksum:#04x}"
                )
                return None

        return None

# ─────────────────────────────────────────────────────────────────────────────
#  PACKET DECODERS  (FC -> GC)
# ─────────────────────────────────────────────────────────────────────────────

def decode_positioning(data: bytes, state: RocketState):
    """Packet ID 1 - 3xf64 pos + 3xf32 vel."""
    if len(data) < 36:
        return
    px, py, pz   = struct.unpack_from('<ddd', data, 0)
    vx, vy, vz   = struct.unpack_from('<fff', data, 24)
    state.pos[:] = [px, py, pz]
    state.vel[:] = [vx, vy, vz]

def decode_orientation(data: bytes, state: RocketState):
    """Packet ID 2 - 3xf32 MRP + 3xf32 ang_rate + 3xf32 mag."""
    if len(data) < 36:
        return
    mx, my, mz        = struct.unpack_from('<fff', data, 0)
    rx, ry, rz        = struct.unpack_from('<fff', data, 12)
    mgx, mgy, mgz     = struct.unpack_from('<fff', data, 24)
    state.mrp[:]      = [mx, my, mz]
    state.ang_rate[:] = [rx, ry, rz]
    state.mag[:]      = [mgx, mgy, mgz]

def decode_sensors(data: bytes, state: RocketState):
    """Packet ID 3 - 3xf32 accel + 3xf32 accel_bias + 3xf32 gyro_bias."""
    if len(data) < 36:
        return
    ax, ay, az          = struct.unpack_from('<fff', data, 0)
    bax, bay, baz       = struct.unpack_from('<fff', data, 12)
    bgx, bgy, bgz       = struct.unpack_from('<fff', data, 24)
    state.accel[:]      = [ax, ay, az]
    state.accel_bias[:] = [bax, bay, baz]
    state.gyro_bias[:]  = [bgx, bgy, bgz]

def decode_weather(data: bytes, state: RocketState):
    """Packet ID 4 - stub, no payload yet."""
    pass

def decode_uncertainty(data: bytes, state: RocketState):
    """Packet ID 5 - 6xf32 pos/vel std-devs + 1xf32 heading_err."""
    if len(data) < 28:
        return
    spx, spy, spz    = struct.unpack_from('<fff', data, 0)
    svx, svy, svz    = struct.unpack_from('<fff', data, 12)
    ang_err,         = struct.unpack_from('<f',   data, 24)
    state.pos_std[:] = [spx, spy, spz]
    state.vel_std[:] = [svx, svy, svz]
    state.heading_err = ang_err

def decode_controls(data: bytes, state: RocketState):
    """Packet ID 6 - f32 time + 6xu8 flags."""
    if len(data) < 10:
        return
    t,                    = struct.unpack_from('<f', data, 0)
    flags                 = list(data[4:10])
    state.time            = t
    state.drogue_deploy   = bool(flags[0])
    state.main_deploy     = bool(flags[1])
    state.drogue_desired  = bool(flags[2])
    state.main_desired    = bool(flags[3])
    state.drogue_override = bool(flags[4])
    state.main_override   = bool(flags[5])

_DECODERS = {
    PKT_POSITIONING: decode_positioning,
    PKT_ORIENTATION: decode_orientation,
    PKT_SENSORS:     decode_sensors,
    PKT_WEATHER:     decode_weather,
    PKT_UNCERTAINTY: decode_uncertainty,
    PKT_CONTROLS:    decode_controls,
}

# ─────────────────────────────────────────────────────────────────────────────
#  RECEIVER
# ─────────────────────────────────────────────────────────────────────────────

class E32Receiver:
    """
    Continuously reads bytes from the serial port, assembles frames,
    decodes them into RocketState, and sends the required ACKs back.
    """

    def __init__(self, port='/dev/ttyACM0', baud=115200,
                 request_interval_s=0.5):
        self._port     = port
        self._baud     = baud
        self._interval = request_interval_s

        self._ser      = None
        self._parser   = FrameParser()
        self._state    = RocketState()
        self._lock     = threading.Lock()

        self._gc_seq   = 0
        self._running  = False
        self._thread   = None

        self._connect()

    # ── Connection ────────────────────────────────────────────────────────────

    def _connect(self):
        """Try to open the serial port. Closes any existing connection first."""
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
            logger.info(f"Opened {self._port} @ {self._baud} baud")
        except Exception as e:
            logger.warning(f"Could not open {self._port}: {e} — retrying...")
            self._ser = None

    def _reconnect(self):
        """Called after an I/O error. Waits briefly then tries to reopen."""
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        time.sleep(2.0)   # give the 32u4 time to finish its USB reset
        self._connect()

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self):
        """Start the background receive thread."""
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("Receiver thread started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.close()

    def get_state(self) -> RocketState:
        """Return a copy of the latest rocket state (thread-safe)."""
        with self._lock:
            return self._state.copy()

    def request_data(self):
        """Send a Data Request to the FC, triggering a 6-packet burst."""
        self._send_data_request()

    # ── Internal: serial write helpers ───────────────────────────────────────

    def _next_seq(self):
        self._gc_seq = (self._gc_seq + 1) & 0xFF
        return self._gc_seq

    def _write(self, frame: bytes):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(frame)
            except OSError as e:
                logger.warning(f"Serial write error: {e} — reconnecting")
                self._reconnect()

    def _send_ack(self, seq: int, ok: bool):
        self._write(build_ack(seq, ok))

    def _send_data_request(self):
        seq = self._next_seq()
        self._write(build_data_request(seq))
        logger.debug(f"Data request sent (seq={seq})")

    # ── Internal: background thread ───────────────────────────────────────────

    def _run(self):
        last_request = 0.0

        while self._running:
            now = time.time()
            if now - last_request >= self._interval:
                self._send_data_request()
                last_request = now
            if not self._ser or not self._ser.is_open:
                logger.info("Port not open — attempting reconnect...")
                self._reconnect()
                last_request = 0.0  # force a data request after reconnect
                continue

            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except OSError as e:
                logger.warning(f"Serial read error: {e} — reconnecting")
                self._reconnect()
                last_request = 0.0
                continue

            for byte in chunk:
                result = self._parser.push(byte)
                if result is None:
                    continue
                seq, pid, data = result
                self._handle_packet(seq, pid, data)

    def _handle_packet(self, seq: int, pid: int, data: bytes):
        """Decode a valid frame and update state."""
        if seq == self._last_rx_seq:
            logger.debug(f"Duplicate seq={seq} ignored")
            self._send_ack(seq, True)
            return
        self._last_rx_seq = seq
        
        decoder = _DECODERS.get(pid)
        if decoder is None:
            logger.warning(f"Unknown packet ID={pid}, ignoring")
            self._send_ack(seq, True)  # ACK anyway so FC keeps going
            return

        with self._lock:
            decoder(data, self._state)

        self._send_ack(seq, True)
        logger.debug(f"Received & ACK'd packet ID={pid} (seq={seq})")

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def close(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
