import struct
from dataclasses import dataclass
from typing import List, Tuple, Union

SYNC = b"\xaa\x55"
FRAME_BYTES = 31
PAYLOAD_BYTES = 25
VERSION_IMU = 0x01
VERSION_SYNC = 0x02


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class ImuFrame:
    timestamp_ns: int
    gyro_raw: Tuple[int, int, int]
    accel_raw: Tuple[int, int, int]
    temperature_code: int
    gyro_range: int
    accel_range: int
    flags: int


@dataclass(frozen=True)
class SyncFrame:
    seq: int
    trigger_ns: int
    strobe_ns: int
    flags: int


Frame = Union[ImuFrame, SyncFrame]


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0
        self.resyncs = 0

    def feed(self, data: bytes) -> List[Frame]:
        self.buffer.extend(data)
        frames: List[Frame] = []

        while True:
            index = self.buffer.find(SYNC)
            if index < 0:
                if len(self.buffer) > 1:
                    del self.buffer[: len(self.buffer) - 1]
                break
            if index > 0:
                del self.buffer[:index]
                self.resyncs += 1
            if len(self.buffer) < FRAME_BYTES:
                break

            frame = bytes(self.buffer[:FRAME_BYTES])
            version = frame[2]
            length = frame[3]
            expected_crc = struct.unpack_from("<H", frame, FRAME_BYTES - 2)[0]
            valid = (
                length == PAYLOAD_BYTES
                and version in (VERSION_IMU, VERSION_SYNC)
                and crc16_ccitt_false(frame[2 : FRAME_BYTES - 2]) == expected_crc
            )
            if not valid:
                self.bad_frames += 1
                del self.buffer[:1]
                continue

            del self.buffer[:FRAME_BYTES]
            if version == VERSION_SYNC:
                frames.append(
                    SyncFrame(
                        seq=struct.unpack_from("<I", frame, 4)[0],
                        trigger_ns=struct.unpack_from("<Q", frame, 8)[0],
                        strobe_ns=struct.unpack_from("<Q", frame, 16)[0],
                        flags=frame[24],
                    )
                )
            else:
                gyro = struct.unpack_from("<3h", frame, 12)
                accel = struct.unpack_from("<3h", frame, 18)
                frames.append(
                    ImuFrame(
                        timestamp_ns=struct.unpack_from("<Q", frame, 4)[0],
                        gyro_raw=(gyro[0], gyro[1], gyro[2]),
                        accel_raw=(accel[0], accel[1], accel[2]),
                        temperature_code=struct.unpack_from("<h", frame, 24)[0],
                        gyro_range=frame[26],
                        accel_range=frame[27],
                        flags=frame[28],
                    )
                )
        return frames
