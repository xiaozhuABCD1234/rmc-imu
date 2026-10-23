import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import (
    FRAME_BYTES,
    PAYLOAD_BYTES,
    VERSION_IMU,
    VERSION_SYNC,
    FrameParser,
    ImuFrame,
    SyncFrame,
    crc16_ccitt_false,
)


def build_frame(version: int, payload: bytes) -> bytes:
    frame = bytearray(FRAME_BYTES)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = version
    frame[3] = PAYLOAD_BYTES
    frame[4:29] = payload
    crc = crc16_ccitt_false(bytes(frame[2:29]))
    struct.pack_into("<H", frame, 29, crc)
    return bytes(frame)


class ProtocolTest(unittest.TestCase):
    def test_sync_frame(self) -> None:
        payload = bytearray(PAYLOAD_BYTES)
        struct.pack_into("<I", payload, 0, 42)
        struct.pack_into("<Q", payload, 4, 1234567890123)
        struct.pack_into("<Q", payload, 12, 0)
        payload[20] = 0
        frame = build_frame(VERSION_SYNC, bytes(payload))
        parser = FrameParser()
        frames = parser.feed(b"xx" + frame[:10])
        self.assertEqual(frames, [])
        frames = parser.feed(frame[10:])
        self.assertEqual(len(frames), 1)
        parsed = frames[0]
        self.assertIsInstance(parsed, SyncFrame)
        self.assertEqual(parsed.seq, 42)
        self.assertEqual(parsed.trigger_ns, 1234567890123)
        self.assertEqual(parsed.strobe_ns, 0)
        self.assertEqual(parsed.flags, 0)

    def test_imu_frame(self) -> None:
        payload = bytearray(PAYLOAD_BYTES)
        struct.pack_into("<Q", payload, 0, 999)
        struct.pack_into("<3h", payload, 8, 1, -2, 3)
        struct.pack_into("<3h", payload, 14, 4, -5, 6)
        struct.pack_into("<h", payload, 20, 160)
        payload[22] = 0
        payload[23] = 1
        payload[24] = 3
        frame = build_frame(VERSION_IMU, bytes(payload))
        parser = FrameParser()
        frames = parser.feed(frame)
        self.assertEqual(len(frames), 1)
        parsed = frames[0]
        self.assertIsInstance(parsed, ImuFrame)
        self.assertEqual(parsed.timestamp_ns, 999)
        self.assertEqual(parsed.gyro_raw, (1, -2, 3))
        self.assertEqual(parsed.accel_raw, (4, -5, 6))
        self.assertEqual(parsed.temperature_code, 160)
        self.assertEqual(parsed.gyro_range, 0)
        self.assertEqual(parsed.accel_range, 1)
        self.assertEqual(parsed.flags, 3)

    def test_bad_crc_is_rejected(self) -> None:
        payload = bytearray(PAYLOAD_BYTES)
        struct.pack_into("<I", payload, 0, 1)
        frame = bytearray(build_frame(VERSION_SYNC, bytes(payload)))
        frame[10] ^= 0xFF
        parser = FrameParser()
        frames = parser.feed(bytes(frame))
        self.assertEqual(frames, [])
        self.assertGreaterEqual(parser.bad_frames, 1)

    def test_resync_after_garbage(self) -> None:
        payload = bytearray(PAYLOAD_BYTES)
        struct.pack_into("<I", payload, 0, 7)
        frame = build_frame(VERSION_SYNC, bytes(payload))
        parser = FrameParser()
        frames = parser.feed(b"\x00\x11\x22" + frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 7)


if __name__ == "__main__":
    unittest.main()
