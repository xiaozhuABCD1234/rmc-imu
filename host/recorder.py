import argparse
import csv
import ctypes
import json
import os
import queue
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from mvs_loader import load_mvs
from serial_link import SerialLink, SyncBuffer

PIXEL_RGB8 = 35127316
PIXEL_BGR8 = 35127317
PIXEL_MONO8 = 17301505
PIXEL_BAYER_RG8 = 17301513
PIXEL_BAYER_BG8 = 17301515
PIXEL_BAYER_GR8 = 17301512
PIXEL_BAYER_GB8 = 17301514


@dataclass
class CapturedFrame:
    data: bytes
    width: int
    height: int
    pixel_type: int
    frame_num: int
    trigger_index: int
    dev_ts: int
    host_ts: int
    lost_packet: int
    recv_ns: int


@dataclass
class Stats:
    captured: int = 0
    saved: int = 0
    dropped_queue: int = 0
    missing_sync: int = 0
    exact_pairs: int = 0
    gap_frames: int = 0
    gap_triggers: int = 0
    lost_packets: int = 0
    decode_failures: int = 0
    dt_values: List[int] = field(default_factory=list)
    started_ns: int = field(default_factory=time.time_ns)


class SyncMatcher:
    def __init__(self, buffer: Optional[SyncBuffer]) -> None:
        self.buffer = buffer
        self.offset: Optional[int] = None
        self.fifo_started = False

    def match(self, frame: CapturedFrame, timeout: float):
        if self.buffer is None:
            return None, False

        index = frame.trigger_index
        if index > 0:
            if self.offset is None:
                latest = self.buffer.wait_latest(timeout)
                if latest is None:
                    return None, False
                self.offset = index - latest.seq
            seq = index - self.offset
            event = self.buffer.wait_for_seq(seq, timeout)
            if event is not None:
                return event, True
            return self.buffer.latest(), False

        if not self.fifo_started:
            event = self.buffer.wait_latest(timeout)
            if event is not None:
                self.buffer.drop_through(event.seq)
                self.fifo_started = True
            return event, event is not None

        event = self.buffer.wait_oldest(timeout)
        return event, event is not None


class MvsCamera:
    def __init__(
        self,
        module,
        camera_index: int,
        camera_serial: str,
        exposure_us: float,
        gain_db: float,
        pixel_format: str,
    ) -> None:
        self.mod = module
        self.camera_index = camera_index
        self.camera_serial = camera_serial
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self.pixel_format = pixel_format
        self.cam = None
        self.callback = None
        self.queue = None
        self.stats = None
        self.info = ""
        self._initialized = False
        self._grabbing = False

    def _set_enum(self, key: str, value: str, required: bool = False) -> bool:
        ret = self.cam.MV_CC_SetEnumValueByString(key, value)
        if ret != 0:
            if required:
                raise RuntimeError(f"{key}={value} failed: 0x{ret:x}")
            print(f"warning: {key}={value} failed: 0x{ret:x}")
            return False
        return True

    def _set_bool(self, key: str, value: bool, required: bool = False) -> bool:
        ret = self.cam.MV_CC_SetBoolValue(key, value)
        if ret != 0:
            if required:
                raise RuntimeError(f"{key}={value} failed: 0x{ret:x}")
            print(f"warning: {key}={value} failed: 0x{ret:x}")
            return False
        return True

    def _set_int(self, key: str, value: int, required: bool = False) -> bool:
        ret = self.cam.MV_CC_SetIntValue(key, value)
        if ret != 0:
            if required:
                raise RuntimeError(f"{key}={value} failed: 0x{ret:x}")
            print(f"warning: {key}={value} failed: 0x{ret:x}")
            return False
        return True

    def _set_float(self, key: str, value: float, required: bool = False) -> bool:
        ret = self.cam.MV_CC_SetFloatValue(key, value)
        if ret != 0:
            if required:
                raise RuntimeError(f"{key}={value} failed: 0x{ret:x}")
            print(f"warning: {key}={value} failed: 0x{ret:x}")
            return False
        return True

    def open(self) -> None:
        initialize = getattr(self.mod.MvCamera, "MV_CC_Initialize", None)
        if initialize is not None:
            initialize()
            self._initialized = True

        devices = self.mod.MV_CC_DEVICE_INFO_LIST()
        ret = self.mod.MvCamera.MV_CC_EnumDevices(
            self.mod.MV_GIGE_DEVICE | self.mod.MV_USB_DEVICE, devices
        )
        if ret != 0:
            raise RuntimeError(f"MV_CC_EnumDevices failed: 0x{ret:x}")
        if devices.nDeviceNum == 0:
            raise RuntimeError("no MVS camera found")

        selected = None
        selected_model = ""
        selected_serial = ""
        usb_index = 0
        for i in range(devices.nDeviceNum):
            info = ctypes.cast(
                devices.pDeviceInfo[i], ctypes.POINTER(self.mod.MV_CC_DEVICE_INFO)
            ).contents
            if info.nTLayerType != self.mod.MV_USB_DEVICE:
                continue
            usb = info.SpecialInfo.stUsb3VInfo
            model = "".join(chr(c) for c in usb.chModelName if c)
            serial = "".join(chr(c) for c in usb.chSerialNumber if c)
            print(f"USB camera [{usb_index}]: {model} serial {serial}")
            if self.camera_serial:
                if serial == self.camera_serial and selected is None:
                    selected = info
                    selected_model = model
                    selected_serial = serial
            elif usb_index == self.camera_index and selected is None:
                selected = info
                selected_model = model
                selected_serial = serial
            usb_index += 1

        if selected is None:
            raise RuntimeError("requested camera not found")

        self.cam = self.mod.MvCamera()
        ret = self.cam.MV_CC_CreateHandle(selected)
        if ret != 0:
            raise RuntimeError(f"MV_CC_CreateHandle failed: 0x{ret:x}")
        ret = self.cam.MV_CC_OpenDevice(
            getattr(self.mod, "MV_ACCESS_Exclusive", 1), 0
        )
        if ret != 0:
            raise RuntimeError(f"MV_CC_OpenDevice failed: 0x{ret:x}")

        self.info = f"{selected_model} {selected_serial}"

    def configure(self) -> None:
        self._set_enum("TriggerMode", "Off", required=True)
        self._set_enum("AcquisitionMode", "Continuous")
        if not self._set_enum("TriggerSelector", "FrameBurstStart"):
            self._set_enum("TriggerSelector", "FrameStart")
        self._set_enum("TriggerSource", "Line0", required=True)
        self._set_enum("TriggerActivation", "RisingEdge", required=True)
        self._set_float("TriggerDelay", 0.0)
        self._set_int("AcquisitionBurstFrameCount", 1)
        self._set_bool("TriggerCacheEnable", False)
        self._set_float("LineDebouncerTime", 0.0)
        self._set_enum("ExposureAuto", "Off", required=True)
        self._set_enum("GainAuto", "Off", required=True)
        self._set_enum("BalanceWhiteAuto", "Off")
        self._set_float("ExposureTime", float(self.exposure_us), required=True)
        self._set_float("Gain", float(self.gain_db))
        self._set_bool("AcquisitionFrameRateEnable", False)
        if self.pixel_format:
            self._set_enum("PixelFormat", self.pixel_format)
        self._set_enum("TriggerMode", "On", required=True)

    def start(self, frame_queue: queue.Queue, stats: Stats) -> None:
        self.queue = frame_queue
        self.stats = stats
        callback_type = ctypes.WINFUNCTYPE if os.name == "nt" else ctypes.CFUNCTYPE
        prototype = callback_type(
            None,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(self.mod.MV_FRAME_OUT_INFO_EX),
            ctypes.c_void_p,
        )
        self.callback = prototype(self._on_image)
        ret = self.cam.MV_CC_RegisterImageCallBackEx(self.callback, None)
        if ret != 0:
            raise RuntimeError(f"MV_CC_RegisterImageCallBackEx failed: 0x{ret:x}")
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError(f"MV_CC_StartGrabbing failed: 0x{ret:x}")
        self._grabbing = True

    def _on_image(self, data_ptr, frame_info_ptr, user_ptr) -> None:
        info = ctypes.cast(
            frame_info_ptr, ctypes.POINTER(self.mod.MV_FRAME_OUT_INFO_EX)
        ).contents
        length = int(info.nFrameLen)
        if length <= 0:
            return
        data = ctypes.string_at(data_ptr, length)
        dev_ts = (int(getattr(info, "nDevTimeStampHigh", 0)) << 32) | int(
            getattr(info, "nDevTimeStampLow", 0)
        )
        frame = CapturedFrame(
            data=data,
            width=int(info.nWidth),
            height=int(info.nHeight),
            pixel_type=int(info.enPixelType),
            frame_num=int(info.nFrameNum),
            trigger_index=int(getattr(info, "nTriggerIndex", 0)),
            dev_ts=dev_ts,
            host_ts=int(getattr(info, "nHostTimeStamp", 0)),
            lost_packet=int(getattr(info, "nLostPacket", 0)),
            recv_ns=time.time_ns(),
        )
        try:
            self.queue.put_nowait(frame)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(frame)
            except queue.Full:
                pass
            if self.stats is not None:
                self.stats.dropped_queue += 1

    def stop(self) -> None:
        if self.cam is not None:
            if self._grabbing:
                self.cam.MV_CC_StopGrabbing()
                self._grabbing = False
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.cam = None
        if self._initialized:
            finalize = getattr(self.mod.MvCamera, "MV_CC_Finalize", None)
            if finalize is not None:
                finalize()
            self._initialized = False


def decode_frame(frame: CapturedFrame) -> Optional[np.ndarray]:
    width = frame.width
    height = frame.height
    pixel = frame.pixel_type

    if pixel == PIXEL_RGB8:
        data = np.frombuffer(frame.data, dtype=np.uint8, count=width * height * 3)
        image = data.reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if pixel == PIXEL_BGR8:
        data = np.frombuffer(frame.data, dtype=np.uint8, count=width * height * 3)
        return data.reshape((height, width, 3))
    if pixel == PIXEL_MONO8:
        data = np.frombuffer(frame.data, dtype=np.uint8, count=width * height)
        gray = data.reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if pixel in (
        PIXEL_BAYER_RG8,
        PIXEL_BAYER_BG8,
        PIXEL_BAYER_GR8,
        PIXEL_BAYER_GB8,
    ):
        data = np.frombuffer(frame.data, dtype=np.uint8, count=width * height)
        bayer = data.reshape((height, width))
        code = {
            PIXEL_BAYER_RG8: cv2.COLOR_BAYER_RG2BGR,
            PIXEL_BAYER_BG8: cv2.COLOR_BAYER_BG2BGR,
            PIXEL_BAYER_GR8: cv2.COLOR_BAYER_GR2BGR,
            PIXEL_BAYER_GB8: cv2.COLOR_BAYER_GB2BGR,
        }[pixel]
        return cv2.cvtColor(bayer, code)
    return None


def draw_overlay(
    image: np.ndarray, lines: List[str], status_color, enabled: bool
) -> np.ndarray:
    if not enabled:
        return image
    scale = max(0.55, min(1.0, image.shape[1] / 1800.0))
    thickness = max(1, int(round(scale * 2)))
    line_height = int(round(32 * scale))
    top = 8
    bottom = top + line_height * len(lines) + 8
    cv2.rectangle(image, (0, 0), (image.shape[1], bottom), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        color = status_color if i == 0 else (255, 255, 255)
        y = top + line_height * (i + 1) - 6
        cv2.putText(
            image,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
    return image


def capture_stats(
    stats: Stats,
    serial: Optional[SerialLink],
    camera: Optional[MvsCamera],
    args,
    session_dir: Path,
) -> None:
    dt_ms = [value / 1e6 for value in stats.dt_values]
    summary = {
        "session": str(session_dir),
        "camera": camera.info if camera is not None else "",
        "serial_port": args.port,
        "baud": args.baud,
        "trigger_hz": args.fps,
        "frames_captured": stats.captured,
        "frames_saved": stats.saved,
        "frames_dropped_queue": stats.dropped_queue,
        "sync_missing": stats.missing_sync,
        "exact_pairs": stats.exact_pairs,
        "frame_index_gaps": stats.gap_frames,
        "trigger_index_gaps": stats.gap_triggers,
        "lost_packets": stats.lost_packets,
        "decode_failures": stats.decode_failures,
        "dt_samples": len(stats.dt_values),
        "dt_min_ms": min(dt_ms) if dt_ms else None,
        "dt_median_ms": statistics.median(dt_ms) if dt_ms else None,
        "dt_max_ms": max(dt_ms) if dt_ms else None,
        "serial_error": serial.error if serial is not None else "",
        "serial_bad_frames": serial.bad_frames if serial is not None else 0,
        "serial_resyncs": serial.resyncs if serial is not None else 0,
        "serial_sync_frames": serial.sync_frames if serial is not None else 0,
    }
    (session_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


def writer_loop(
    args,
    session_dir: Path,
    frame_queue: queue.Queue,
    matcher: SyncMatcher,
    stats: Stats,
    stop_event: threading.Event,
) -> None:
    expected_ns = 1e9 / args.fps
    tolerance_ns = max(1_000_000, expected_ns * 0.01)
    prev_trigger_ns = None
    prev_dev_ts = None
    prev_frame_num = None
    prev_trigger_index = None

    csv_path = session_dir / "sync.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "image",
                "frame_num",
                "trigger_index",
                "dev_ts",
                "host_ts",
                "recv_ns",
                "mcu_seq",
                "trigger_ns",
                "strobe_ns",
                "sync_exact",
                "dt_ns",
                "cam_dt_ticks",
                "lost_packet",
                "status",
            ]
        )

        while not stop_event.is_set() or not frame_queue.empty():
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            stats.captured += 1
            event, exact = matcher.match(frame, args.sync_timeout)
            trigger_ns = event.trigger_ns if event is not None else None
            dt_ns = (
                trigger_ns - prev_trigger_ns
                if trigger_ns is not None and prev_trigger_ns is not None
                else None
            )
            cam_dt_ticks = (
                frame.dev_ts - prev_dev_ts
                if frame.dev_ts > 0 and prev_dev_ts not in (None, 0)
                else None
            )
            frame_gap = (
                frame.frame_num - prev_frame_num
                if prev_frame_num is not None
                else None
            )
            trigger_gap = (
                frame.trigger_index - prev_trigger_index
                if prev_trigger_index is not None
                and frame.trigger_index > 0
                and prev_trigger_index > 0
                else None
            )

            if event is None:
                stats.missing_sync += 1
            if exact:
                stats.exact_pairs += 1
            if frame_gap is not None and frame_gap != 1:
                stats.gap_frames += 1
            if trigger_gap is not None and trigger_gap != 1:
                stats.gap_triggers += 1
            if frame.lost_packet > 0:
                stats.lost_packets += 1
            if dt_ns is not None:
                stats.dt_values.append(dt_ns)
                if len(stats.dt_values) > 20000:
                    del stats.dt_values[:10000]

            sync_ok = (
                event is not None
                and exact
                and (dt_ns is None or abs(dt_ns - expected_ns) <= tolerance_ns)
                and (frame_gap is None or frame_gap == 1)
                and (trigger_gap is None or trigger_gap == 1)
                and frame.lost_packet == 0
            )

            try:
                image = decode_frame(frame)
            except Exception as exc:
                print(f"warning: decode frame {frame.frame_num} failed: {exc}")
                image = None
            if image is None:
                stats.decode_failures += 1
                print(
                    f"warning: unsupported pixel type 0x{frame.pixel_type:x} "
                    f"frame {frame.frame_num}"
                )
                image_path = None
                image_name = ""
            else:
                if event is None:
                    status = "NO MCU" if args.no_serial else "SYNC LOST"
                    status_color = (0, 165, 255) if args.no_serial else (0, 0, 255)
                elif sync_ok:
                    status = "SYNC OK"
                    status_color = (0, 255, 0)
                else:
                    status = "SYNC LOST"
                    status_color = (0, 0, 255)

                mcu_text = (
                    f"MCU {trigger_ns / 1e9:.6f} s"
                    if trigger_ns is not None
                    else "MCU N/A"
                )
                dt_text = (
                    f"dt {dt_ns / 1e6:.3f} ms" if dt_ns is not None else "dt --"
                )
                cam_dt_text = (
                    f"cam_dt {cam_dt_ticks} ticks"
                    if cam_dt_ticks is not None
                    else "cam_dt --"
                )
                seq_text = (
                    f"#{event.seq:08d}"
                    if event is not None
                    else f"#{frame.trigger_index:08d}"
                )
                pc_text = datetime.fromtimestamp(frame.recv_ns / 1e9).strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )[:-3]
                lines = [
                    f"{seq_text}  {mcu_text}  {dt_text}  {status}",
                    f"CAM trig {frame.trigger_index} frame {frame.frame_num} "
                    f"dev {frame.dev_ts}  {cam_dt_text}",
                    f"PC {pc_text}  lost {frame.lost_packet} "
                    f"qdrop {stats.dropped_queue}",
                ]
                image = draw_overlay(image, lines, status_color, not args.no_overlay)

                trigger_part = str(trigger_ns) if trigger_ns is not None else "nosync"
                suffix = args.format
                image_name = (
                    f"img_{frame.frame_num:06d}_trig{frame.trigger_index:06d}_"
                    f"{trigger_part}.{suffix}"
                )
                image_path = session_dir / image_name
                if suffix == "jpg":
                    cv2.imwrite(
                        str(image_path),
                        image,
                        [cv2.IMWRITE_JPEG_QUALITY, args.quality],
                    )
                else:
                    cv2.imwrite(
                        str(image_path), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]
                    )

            writer.writerow(
                [
                    image_name,
                    frame.frame_num,
                    frame.trigger_index,
                    frame.dev_ts,
                    frame.host_ts,
                    frame.recv_ns,
                    event.seq if event is not None else "",
                    trigger_ns if trigger_ns is not None else "",
                    event.strobe_ns if event is not None else "",
                    int(exact),
                    dt_ns if dt_ns is not None else "",
                    cam_dt_ticks if cam_dt_ticks is not None else "",
                    frame.lost_packet,
                    "SYNC_OK"
                    if sync_ok
                    else ("NO_MCU" if args.no_serial else "SYNC_LOST"),
                ]
            )
            handle.flush()
            stats.saved += 1

            prev_trigger_ns = trigger_ns if trigger_ns is not None else prev_trigger_ns
            prev_dev_ts = frame.dev_ts if frame.dev_ts > 0 else prev_dev_ts
            prev_frame_num = frame.frame_num
            prev_trigger_index = frame.trigger_index if frame.trigger_index > 0 else prev_trigger_index

            if stats.saved % 25 == 0:
                print(
                    f"saved {stats.saved}, captured {stats.captured}, "
                    f"missing_sync {stats.missing_sync}, "
                    f"queue_drop {stats.dropped_queue}"
                )


def parse_args() -> argparse.Namespace:
    default_port = "COM3" if os.name == "nt" else "/dev/ttyUSB0"
    default_out = Path(__file__).resolve().parents[1] / "photo"
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", type=Path, default=default_out)
    parser.add_argument("--port", default=default_port)
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--no-serial", action="store_true")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-serial", default="")
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--exposure-us", type=float, default=5000.0)
    parser.add_argument("--gain-db", type=float, default=0.0)
    parser.add_argument("--pixel-format", default="RGB8")
    parser.add_argument("--format", choices=("jpg", "png"), default="jpg")
    parser.add_argument("--quality", type=int, default=95)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--queue-depth", type=int, default=16)
    parser.add_argument("--sync-timeout", type=float, default=0.5)
    parser.add_argument("--flat", action="store_true")
    parser.add_argument("--no-overlay", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    outdir = args.outdir.expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    session_dir = (
        outdir
        if args.flat
        else outdir / f"session_{datetime.now():%Y%m%d_%H%M%S}"
    )
    session_dir.mkdir(parents=True, exist_ok=True)

    serial = None
    sync_buffer = None
    camera = None
    writer = None
    writer_stop = threading.Event()
    stats = Stats()
    frame_queue = queue.Queue(maxsize=args.queue_depth)

    if not args.no_serial:
        sync_buffer = SyncBuffer()
        serial = SerialLink(args.port, args.baud, sync_buffer)
        try:
            serial.start()
        except Exception as exc:
            print(f"serial error: {exc}", file=sys.stderr)
            return 2

    try:
        module = load_mvs()
        camera = MvsCamera(
            module,
            args.camera_index,
            args.camera_serial,
            args.exposure_us,
            args.gain_db,
            args.pixel_format,
        )
        camera.open()
        camera.configure()
        if sync_buffer is not None:
            sync_buffer.clear()

        matcher = SyncMatcher(sync_buffer)
        writer = threading.Thread(
            target=writer_loop,
            args=(args, session_dir, frame_queue, matcher, stats, writer_stop),
            name="rmc-writer",
            daemon=True,
        )
        writer.start()
        camera.start(frame_queue, stats)

        started = time.monotonic()
        print(f"capturing to {session_dir}")
        while True:
            if args.duration > 0 and time.monotonic() - started >= args.duration:
                break
            if args.max_frames > 0 and stats.saved >= args.max_frames:
                break
            if serial is not None and serial.error:
                print(f"serial error: {serial.error}", file=sys.stderr)
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                print(f"camera stop error: {exc}", file=sys.stderr)
        writer_stop.set()
        if writer is not None:
            writer.join(timeout=5.0)
        if serial is not None:
            serial.stop()
        capture_stats(stats, serial, camera, args, session_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
