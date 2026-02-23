import socket
import struct
from typing import Tuple

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

OP_RESET = 1
OP_STEP = 2
OP_GET_FRAME = 3
OP_GET_RAM = 4
OP_PING = 5
OP_GET_FRAME_GRAY_80X84 = 6

RESP_OK = 101
RESP_STEP = 102
RESP_FRAME = 103
RESP_RAM = 104
RESP_FRAME_GRAY_80X84 = 105
RESP_ERROR = 255


class RustNesUdsClient:
    def __init__(self, socket_path: str):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(socket_path)

    def close(self) -> None:
        self.sock.close()

    def _send(self, payload: bytes) -> bytes:
        self.sock.sendall(struct.pack("<I", len(payload)) + payload)
        hdr = self._recv_exact(4)
        (length,) = struct.unpack("<I", hdr)
        return self._recv_exact(length)

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("socket closed")
            buf.extend(chunk)
        return bytes(buf)

    def ping(self) -> None:
        resp = self._send(bytes([OP_PING]))
        self._expect_ok(resp)

    def reset(self) -> None:
        resp = self._send(bytes([OP_RESET]))
        self._expect_ok(resp)

    def step(self, action: int, frame_skip: int = 1) -> dict:
        resp = self._send(bytes([OP_STEP, action & 0xFF, frame_skip & 0xFF]))
        if not resp:
            raise RuntimeError("empty response")
        if resp[0] == RESP_ERROR:
            raise RuntimeError(resp[1:].decode("utf-8", errors="replace"))
        if resp[0] != RESP_STEP:
            raise RuntimeError(f"unexpected step response opcode {resp[0]}")
        reward, = struct.unpack_from("<f", resp, 1)
        done = bool(resp[5])
        frame_no, = struct.unpack_from("<Q", resp, 6)
        frames_advanced, = struct.unpack_from("<I", resp, 14)
        return {
            "reward": reward,
            "done": done,
            "frame_no": frame_no,
            "frames_advanced": frames_advanced,
        }

    def get_frame(self):
        resp = self._send(bytes([OP_GET_FRAME]))
        if resp[0] == RESP_ERROR:
            raise RuntimeError(resp[1:].decode("utf-8", errors="replace"))
        if resp[0] != RESP_FRAME:
            raise RuntimeError(f"unexpected frame response opcode {resp[0]}")
        width, height = struct.unpack_from("<HH", resp, 1)
        channels = resp[5]
        frame = resp[6:]
        if np is None:
            return (width, height, channels, frame)
        arr = np.frombuffer(frame, dtype=np.uint8)
        return arr.reshape((height, width, channels))

    def get_ram(self, start: int = 0, length: int = 2048) -> bytes:
        resp = self._send(bytes([OP_GET_RAM]) + struct.pack("<HH", start & 0xFFFF, length & 0xFFFF))
        if resp[0] == RESP_ERROR:
            raise RuntimeError(resp[1:].decode("utf-8", errors="replace"))
        if resp[0] != RESP_RAM:
            raise RuntimeError(f"unexpected ram response opcode {resp[0]}")
        (actual_len,) = struct.unpack_from("<H", resp, 1)
        return resp[3:3 + actual_len]

    def get_frame_gray_80x84(self):
        resp = self._send(bytes([OP_GET_FRAME_GRAY_80X84]))
        if resp[0] == RESP_ERROR:
            raise RuntimeError(resp[1:].decode("utf-8", errors="replace"))
        if resp[0] != RESP_FRAME_GRAY_80X84:
            raise RuntimeError(f"unexpected small frame response opcode {resp[0]}")
        height, width = struct.unpack_from("<HH", resp, 1)
        frame = resp[5:]
        if np is None:
            return (height, width, frame)
        arr = np.frombuffer(frame, dtype=np.uint8)
        return arr.reshape((height, width))

    @staticmethod
    def _expect_ok(resp: bytes) -> None:
        if not resp:
            raise RuntimeError("empty response")
        if resp[0] == RESP_ERROR:
            raise RuntimeError(resp[1:].decode("utf-8", errors="replace"))
        if resp[0] != RESP_OK:
            raise RuntimeError(f"unexpected opcode {resp[0]}")


ACTION_SET = {
    "NOOP": 0x00,
    "A": 0x01,
    "B": 0x02,
    "START": 0x08,
    "LEFT": 0x40,
    "RIGHT": 0x80,
    "LEFT_A": 0x41,
    "RIGHT_A": 0x81,
}
