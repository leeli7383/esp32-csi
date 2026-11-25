"""Capture CSI1 binary frames from native USB Serial/JTAG or a UART bridge."""

import argparse
import json
import struct
import sys
import time
import zlib
from pathlib import Path

MAGIC = b"CSI1"
HEADER = struct.Struct("<4sBBHHbBIQ6s6sIII")
MAX_CSI_LEN = 1024


class Decoder:
    """Incremental decoder; ignores boot logs and recovers after corrupt frames."""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                discard = max(0, len(self.buffer) - len(MAGIC) + 1)
                self.discarded_bytes += discard
                del self.buffer[:discard]
                break
            self.discarded_bytes += start
            del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            fields = HEADER.unpack_from(self.buffer)
            (_, version, flags, header_len, length, rssi, channel, sequence,
             timestamp, local, peer, usb_drops, motion_drops, oversize_drops) = fields
            if version != 1 or flags & ~1 or header_len != HEADER.size or not 0 < length <= MAX_CSI_LEN:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            total = header_len + length + 4
            if len(self.buffer) < total:
                break
            expected = struct.unpack_from("<I", self.buffer, total - 4)[0]
            if zlib.crc32(self.buffer[:total - 4]) != expected:
                self.bad_crc += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            raw = bytes(self.buffer[header_len:total - 4])
            frames.append({
                "sequence": sequence,
                "timestamp_us": timestamp,
                "local_mac": local.hex(":"),
                "peer_mac": peer.hex(":"),
                "rssi": rssi,
                "channel": channel,
                "first_word_invalid": bool(flags & 1),
                "usb_drops": usb_drops,
                "motion_drops": motion_drops,
                "oversize_drops": oversize_drops,
                "length": length,
                # Preserve every signed byte, including an invalid first word.
                "csi": list(struct.unpack(f"<{length}b", raw)),
            })
            del self.buffer[:total]
        return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Native USB or UART bridge port, e.g. COM6")
    source.add_argument("--input", type=Path, help="Decode an existing binary capture")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", type=Path, default=Path("csi.jsonl"))
    parser.add_argument("--raw", type=Path, help="Also save the exact serial byte stream")
    parser.add_argument("--seconds", type=float, default=0, help="Capture duration; 0 means until Ctrl+C")
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds must be nonnegative")
    paths = [p.resolve() for p in (args.input, args.output, args.raw) if p]
    if len(paths) != len(set(paths)):
        parser.error("input, output and raw files must be different")

    decoder = Decoder()
    count = 0
    source_stream = None
    raw_file = None
    latest = None
    try:
        if args.port:
            try:
                import serial
            except ImportError:
                parser.error("Install pyserial first: python -m pip install pyserial")
            # Avoid requesting an ESP reset through the bridge's DTR/RTS lines.
            source_stream = serial.Serial(port=None, baudrate=args.baud, timeout=0.2, write_timeout=1,
                                          rtscts=False, dsrdtr=False)
            source_stream.dtr = False
            source_stream.rts = False
            source_stream.port = args.port
            source_stream.open()
            # XCOM may have selected TEXT mode. Return to binary at a frame
            # boundary; older firmware ignores these bytes safely.
            source_stream.write(b"\nBINARY\nSTATUS\n")
        else:
            source_stream = args.input.open("rb")
        raw_file = args.raw.open("wb") if args.raw else None
        started = time.monotonic()
        report_at = started + 1
        with args.output.open("w", encoding="utf-8") as output:
            try:
                while not args.seconds or time.monotonic() - started < args.seconds:
                    data = source_stream.read(4096)
                    if not data and args.input:
                        break
                    if raw_file:
                        raw_file.write(data)
                    for frame in decoder.feed(data):
                        output.write(json.dumps(frame, separators=(",", ":")) + "\n")
                        count += 1
                        latest = frame
                    if time.monotonic() >= report_at:
                        output.flush()
                        if raw_file:
                            raw_file.flush()
                        print(f"frames={count} bad_crc={decoder.bad_crc} "
                              f"usb_drops={latest['usb_drops'] if latest else '-'} "
                              f"motion_drops={latest['motion_drops'] if latest else '-'}", file=sys.stderr)
                        report_at = time.monotonic() + 1
            except KeyboardInterrupt:
                pass
    except (OSError, ValueError) as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if source_stream:
            source_stream.close()
        if raw_file:
            raw_file.close()
    print(f"Saved {count} frames to {args.output}; bad_crc={decoder.bad_crc}, "
          f"discarded_bytes={decoder.discarded_bytes}, trailing_bytes={len(decoder.buffer)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
