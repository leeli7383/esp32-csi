import importlib.util
from pathlib import Path
import struct
import unittest
import zlib

spec = importlib.util.spec_from_file_location("capture_csi", Path(__file__).parents[1] / "tools/capture_csi.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def packet(data=bytes(range(256)), sequence=42, flags=1):
    header = capture.HEADER.pack(b"CSI1", 1, flags, 48, len(data), -63, 11,
                                 sequence, 1234567890123, bytes.fromhex("020000000001"),
                                 bytes.fromhex("020000000002"), 3, 4, 5)
    body = header + data
    return body + struct.pack("<I", zlib.crc32(body))


class DecoderTests(unittest.TestCase):
    def test_every_raw_byte_and_metadata(self):
        frame = capture.Decoder().feed(packet())[0]
        self.assertEqual(bytes(x & 255 for x in frame["csi"]), bytes(range(256)))
        self.assertEqual(frame["timestamp_us"], 1234567890123)
        self.assertEqual(frame["rssi"], -63)
        self.assertEqual(frame["local_mac"], "02:00:00:00:00:01")
        self.assertEqual(frame["peer_mac"], "02:00:00:00:00:02")
        self.assertEqual([frame[k] for k in ("usb_drops", "motion_drops", "oversize_drops")], [3, 4, 5])
        self.assertTrue(frame["first_word_invalid"])

    def test_every_split_boundary(self):
        wire = packet()
        for split in range(len(wire) + 1):
            decoder = capture.Decoder()
            frames = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
            self.assertEqual(len(frames), 1)

    def test_boot_noise_and_bytewise_input(self):
        decoder = capture.Decoder()
        frames = []
        for value in b"ESP-ROM boot log\r\nCSI" + packet(b"CSI1\x00\x80\xff"):
            frames.extend(decoder.feed(bytes([value])))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["csi"][-3:], [0, -128, -1])

    def test_corruption_recovers_next_frame(self):
        broken = bytearray(packet())
        broken[70] ^= 1
        decoder = capture.Decoder()
        frames = decoder.feed(broken + packet(sequence=43))
        self.assertEqual([f["sequence"] for f in frames], [43])
        self.assertGreater(decoder.bad_crc, 0)

    def test_truncated_write_recovers(self):
        decoder = capture.Decoder()
        frames = decoder.feed(packet()[:100] + packet(sequence=43) + packet(sequence=44))
        self.assertEqual([f["sequence"] for f in frames], [43, 44])

    def test_invalid_length_and_version(self):
        for offset, value in ((4, 9), (6, 47), (8, 0), (8, 1025)):
            broken = bytearray(packet())
            if offset == 4:
                broken[offset] = value
            else:
                struct.pack_into("<H", broken, offset, value)
            frames = capture.Decoder().feed(broken + packet())
            self.assertEqual(len(frames), 1)

    def test_maximum_size_and_sequence_wrap(self):
        decoder = capture.Decoder()
        frames = decoder.feed(packet(bytes(range(256)) * 4, 0xffffffff, 0) + packet(sequence=0))
        self.assertEqual([f["sequence"] for f in frames], [0xffffffff, 0])
        self.assertEqual(frames[0]["length"], 1024)
        self.assertFalse(frames[0]["first_word_invalid"])

    def test_noise_memory_is_bounded(self):
        decoder = capture.Decoder()
        self.assertEqual(decoder.feed(b"noise" * 100000), [])
        self.assertLessEqual(len(decoder.buffer), 3)

    def test_periodic_diagnostics_between_binary_frames(self):
        status = b"\r\n#CSI_STATUS transport=USB_SERIAL_JTAG mode=BINARY stage=running error=0x0\r\n"
        wire = status + packet(sequence=1) + status + packet(sequence=2) + status
        decoder = capture.Decoder()
        frames = []
        for value in wire:
            frames.extend(decoder.feed(bytes([value])))
        self.assertEqual([f["sequence"] for f in frames], [1, 2])
        self.assertEqual(decoder.bad_crc, 0)

    def test_text_to_binary_transition(self):
        text = b"CSI_DATA,seq=1,len=3,data=[-128,0,127]\r\n"
        frames = capture.Decoder().feed(text + packet(b"\x80\x00\x7f", sequence=2))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["csi"], [-128, 0, 127])


if __name__ == "__main__":
    unittest.main()
