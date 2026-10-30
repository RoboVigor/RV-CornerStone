import dataclasses
import struct
import unittest
from gimbal_tuning.protocol import (Frame, Message, StreamDecoder, crc16, Safety, Parameters,
                                    Telemetry, Status, Trial, Result, CounterUnwrapper)


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_and_hello_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x29b1)
        raw = Frame(Message.HELLO).encode()
        self.assertEqual(raw[:18].hex(), "a55a01010000000000000000000000000000")
        self.assertEqual(len(raw), 20)
        self.assertEqual(StreamDecoder().feed(raw), [Frame(Message.HELLO)])

    def test_explicit_sizes_and_roundtrip(self):
        for cls, length in [(Safety, 140), (Parameters, 24), (Telemetry, 124), (Status, 48), (Trial, 20), (Result, 28)]:
            zeros = bytes(length)
            self.assertEqual(cls.unpack(zeros).pack(), zeros)
            with self.assertRaises(ValueError):
                cls.unpack(zeros + b"\0")
        raw = Frame(Message.TELEMETRY, 0x12345678, 5, 9, bytes(124)).encode()
        self.assertEqual(raw[6:10], b"\x78\x56\x34\x12")
        self.assertEqual(StreamDecoder().feed(raw)[0].payload, bytes(124))

    def test_crc_version_length_resync(self):
        valid = Frame(Message.GET_STATUS, 3, 4, 5).encode()
        corrupt = bytearray(valid)
        corrupt[-1] ^= 1
        version = bytearray(valid)
        version[2] = 2
        length = bytearray(valid)
        length[4:6] = struct.pack("<H", 257)
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(b"junk" + corrupt + version + length + valid), [Frame(Message.GET_STATUS, 3, 4, 5)])
        self.assertGreaterEqual(decoder.rejected, 3)
        self.assertLessEqual(len(decoder.buffer), 276)

    def test_fragmentation_and_timeout(self):
        raw = Frame(Message.GET_CONFIG, 10, 20, 1, bytes(256)).encode()
        decoder = StreamDecoder()
        result = []
        for i, byte in enumerate(raw):
            result += decoder.feed(bytes([byte]), i / 10)
        self.assertEqual(len(result), 1)
        decoder.feed(raw[:25], 30)
        self.assertEqual(decoder.feed(b"", 130), [])
        self.assertGreaterEqual(decoder.rejected, 1)
        self.assertEqual(len(decoder.feed(Frame(Message.HELLO).encode(), 131)), 1)
        with self.assertRaises(ValueError):
            decoder.feed(b"", 120)

    def test_bounded_buffer_large_noise_and_truncated_length(self):
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(b"x" * 20000), [])
        self.assertLessEqual(len(decoder.buffer), 276)
        raw = bytearray(Frame(Message.HELLO).encode())
        raw[4:6] = struct.pack("<H", 100)
        self.assertEqual(decoder.feed(raw), [])
        decoder.feed(b"", 101)
        self.assertEqual(decoder.feed(Frame(Message.HELLO).encode(), 102), [Frame(Message.HELLO)])
        with self.assertRaises(ValueError):
            Frame(1, payload=bytes(257)).encode()
        with self.assertRaises(ValueError):
            Frame(1, boot_id=True).encode()

    def test_wrap_and_reorder(self):
        counter = CounterUnwrapper()
        self.assertEqual(counter.add(0xfffffffe), 0xfffffffe)
        self.assertEqual(counter.add(1), 0x100000001)
        for stale in (1, 0, 0xffffffff):
            with self.assertRaises(ValueError):
                counter.add(stale)


if __name__ == "__main__":
    unittest.main()
