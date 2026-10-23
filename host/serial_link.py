import threading
import time
from collections import deque
from typing import Optional

from protocol import FrameParser, ImuFrame, SyncFrame


class SyncBuffer:
    def __init__(self, depth: int = 512) -> None:
        self._depth = depth
        self._events = deque()
        self._by_seq = {}
        self._condition = threading.Condition()
        self._closed = False

    def add(self, event: SyncFrame) -> None:
        with self._condition:
            self._events.append(event)
            self._by_seq[event.seq] = event
            while len(self._events) > self._depth:
                old = self._events.popleft()
                self._by_seq.pop(old.seq, None)
            self._condition.notify_all()

    def latest(self) -> Optional[SyncFrame]:
        with self._condition:
            return self._events[-1] if self._events else None

    def wait_latest(self, timeout: float) -> Optional[SyncFrame]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._events and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._events[-1] if self._events else None

    def wait_for_seq(self, seq: int, timeout: float) -> Optional[SyncFrame]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while seq not in self._by_seq and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._by_seq.get(seq)

    def pop_oldest(self) -> Optional[SyncFrame]:
        with self._condition:
            if not self._events:
                return None
            event = self._events.popleft()
            self._by_seq.pop(event.seq, None)
            return event

    def wait_oldest(self, timeout: float) -> Optional[SyncFrame]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._events and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            if not self._events:
                return None
            event = self._events.popleft()
            self._by_seq.pop(event.seq, None)
            return event

    def clear(self) -> None:
        with self._condition:
            self._events.clear()
            self._by_seq.clear()

    def drop_through(self, seq: int) -> None:
        with self._condition:
            while self._events and self._events[0].seq <= seq:
                old = self._events.popleft()
                self._by_seq.pop(old.seq, None)

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def __len__(self) -> int:
        with self._condition:
            return len(self._events)


class SerialLink:
    def __init__(self, port: str, baud: int, sync_buffer: SyncBuffer) -> None:
        self.port = port
        self.baud = baud
        self.sync_buffer = sync_buffer
        self.error = ""
        self.sync_frames = 0
        self.imu_frames = 0
        self.bad_frames = 0
        self.resyncs = 0
        self._stop = threading.Event()
        self._thread = None
        self._serial = None

    def start(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required: pip install pyserial") from exc

        self._serial = serial.Serial(
            self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
        self._serial.reset_input_buffer()
        self._thread = threading.Thread(target=self._run, name="rmc-serial", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        parser = FrameParser()
        try:
            while not self._stop.is_set():
                data = self._serial.read(self._serial.in_waiting or 1)
                if not data:
                    continue
                for frame in parser.feed(data):
                    if isinstance(frame, SyncFrame):
                        self.sync_buffer.add(frame)
                        self.sync_frames += 1
                    elif isinstance(frame, ImuFrame):
                        self.imu_frames += 1
                self.bad_frames = parser.bad_frames
                self.resyncs = parser.resyncs
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.sync_buffer.close()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
