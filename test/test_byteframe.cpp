/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * Unit tests for ByteFrame.
 *
 * Each test group covers one feature area. Tests use Unity (available via PlatformIO).
 *
 * Coverage:
 * - CRC16: check value of the CCITT-FALSE variant ("123456789" -> 0x29B1), empty input.
 * - getMaxEncodedSize: constexpr values, including the COBS block boundaries.
 * - encode: byte-exact frames (known full vector, zero handling, block boundary at 254 bytes of
 *   data), structural properties (single trailing delimiter, no interior zeros), empty payload and
 *   undersized output buffer rejected, output never exceeds getMaxEncodedSize().
 * - Decoder: initial state, byte-by-byte and chunked feeding, multiple frames per chunk, payloads
 *   containing zeros, payload at exactly MaxPayload, oversized frames (overflow counter, recovery
 *   afterwards), corrupted frames (CRC counter), malformed frames (truncated COBS block, frame too
 *   short), idle delimiters ignored, resynchronization when joining a stream mid-frame, reset(),
 *   frame availability cleared by the next feed.
 * - Round-trip: every payload size from 1 to MaxPayload.
 * - CRC policies: standard check values (CRC-8/SMBUS, CRC-16/CCITT-FALSE, CRC-32/ISO-HDLC) and a
 *   round-trip with NoCrc/Crc8/Crc16/Crc32 through both the one-shot decode() and the Decoder.
 * - decode(): one-shot round-trip, optional trailing delimiter, capacity rejection, CRC rejection.
 * - Stress (deterministic xorshift32 PRNG): random payloads fed in random chunk sizes, random
 *   garbage input (no crash, invariants hold), single-byte corruption of valid frames.
 */

#ifdef ARDUINO
#  include <Arduino.h>
#endif

#include <unity.h>

#include <ByteFrame.h>
using namespace ByteFrame;

/* ---------------------------------------------------------------------------------------------- */
/*                                            Helpers                                             */
/* ---------------------------------------------------------------------------------------------- */

// Feeds a whole buffer byte by byte and returns true if exactly one valid frame came out,
// completed by the very last byte
template <size_t MaxPayload, class Crc>
bool feedAll(Decoder<MaxPayload, Crc>& decoder, const uint8_t* data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    const bool frame = decoder.feed(data[i]);
    if (frame) return i == size - 1;
  }
  return false;
}

// Computes a CRC over a buffer using a policy's incremental steps
template <class Crc>
typename Crc::value_type crcOf(const uint8_t* data, const size_t size) {
  typename Crc::value_type crc = Crc::init();
  for (size_t i = 0; i < size; i++)
    crc = Crc::update(crc, data[i]);
  return Crc::finalize(crc);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                       getMaxEncodedSize                                        */
/* ---------------------------------------------------------------------------------------------- */

void test_get_max_encoded_size() {
  // data = payload + 2 (CRC); frame = data + ceil(data / 254) (COBS) + 1 (delimiter)
  constexpr size_t size_1   = getMaxEncodedSize(1);   // 3 + 1 + 1
  constexpr size_t size_10  = getMaxEncodedSize(10);  // 12 + 1 + 1
  constexpr size_t size_252 = getMaxEncodedSize(252); // 254 + 1 + 1
  constexpr size_t size_253 = getMaxEncodedSize(253); // 255 + 2 + 1
  constexpr size_t size_300 = getMaxEncodedSize(300); // 302 + 2 + 1

  TEST_ASSERT_EQUAL(5, size_1);
  TEST_ASSERT_EQUAL(14, size_10);
  TEST_ASSERT_EQUAL(256, size_252);
  TEST_ASSERT_EQUAL(258, size_253);
  TEST_ASSERT_EQUAL(305, size_300);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                            Encoder                                             */
/* ---------------------------------------------------------------------------------------------- */

void test_encode_known_vector() {
  // crc16("123456789") == 0x29B1, so the full frame is known byte by byte:
  // data = "123456789" + [0xB1, 0x29] (CRC little-endian), no zeros -> single COBS block
  const uint8_t payload[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  const uint8_t expected[13] =
    {0x0C, '1', '2', '3', '4', '5', '6', '7', '8', '9', 0xB1, 0x29, 0x00};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, sizeof(expected));
}

void test_encode_payload_with_zeros() {
  // data = [0x11, 0x00, 0x22, crc_lo, crc_hi]: the zero splits the data in two COBS blocks
  const uint8_t payload[3] = {0x11, 0x00, 0x22};
  const uint16_t crc       = crcOf<Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo     = uint8_t(crc & 0xFF);
  const uint8_t crc_hi     = uint8_t(crc >> 8);

  // The expected layout below assumes the CRC bytes are not zero for this payload
  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  const uint8_t expected[7] = {0x02, 0x11, 0x04, 0x22, crc_lo, crc_hi, 0x00};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(7, written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, 7);
}

void test_encode_leading_zero() {
  // data = [0x00, crc_lo, crc_hi]: a leading zero produces an empty first block (code 0x01)
  const uint8_t payload[1] = {0x00};
  const uint16_t crc       = crcOf<Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo     = uint8_t(crc & 0xFF);
  const uint8_t crc_hi     = uint8_t(crc >> 8);

  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  const uint8_t expected[5] = {0x01, 0x03, crc_lo, crc_hi, 0x00};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(5, written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, 5);
}

void test_encode_block_boundary() {
  // 252 payload bytes + 2 CRC bytes = exactly one full COBS block (254 data bytes, code 0xFF).
  // The canonical encoding adds no trailing code byte: frame = 0xFF + 254 bytes + delimiter.
  uint8_t payload[252];
  for (size_t i = 0; i < sizeof(payload); i++) {
    payload[i] = uint8_t(1 + (i % 255)); // never zero
  }

  const uint16_t crc   = crcOf<Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo = uint8_t(crc & 0xFF);
  const uint8_t crc_hi = uint8_t(crc >> 8);

  // The expected layout below assumes the CRC bytes are not zero for this payload
  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(256, written); // 1 (code) + 254 (data) + 1 (delimiter)
  TEST_ASSERT_EQUAL_HEX8(0xFF, frame[0]);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, frame + 1, sizeof(payload));
  TEST_ASSERT_EQUAL_HEX8(crc_lo, frame[253]);
  TEST_ASSERT_EQUAL_HEX8(crc_hi, frame[254]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[255]);

  // 253 payload bytes -> 255 data bytes: a second block appears after the full one
  uint8_t payload2[253];
  for (size_t i = 0; i < sizeof(payload2); i++) {
    payload2[i] = uint8_t(1 + (i % 255));
  }

  uint8_t frame2[getMaxEncodedSize(sizeof(payload2))] = {};
  const size_t written2 = encode(payload2, sizeof(payload2), frame2, sizeof(frame2));

  TEST_ASSERT_TRUE(written2 > 0);
  TEST_ASSERT_TRUE(written2 <= getMaxEncodedSize(sizeof(payload2)));
  TEST_ASSERT_EQUAL_HEX8(0xFF, frame2[0]);
}

void test_encode_structural_properties() {
  // Whatever the payload, a frame ends with exactly one delimiter and contains no other zero
  const uint8_t payload[6] = {0x00, 0xFF, 0x00, 0x00, 0x01, 0x00};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_TRUE(written > 0);
  TEST_ASSERT_TRUE(written <= getMaxEncodedSize(sizeof(payload)));
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[written - 1]);
  for (size_t i = 0; i < written - 1; i++) {
    TEST_ASSERT_NOT_EQUAL(0x00, frame[i]);
  }
}

void test_encode_rejects_invalid_input() {
  uint8_t frame[16]        = {};
  const uint8_t payload[4] = {1, 2, 3, 4};

  // Empty payload
  TEST_ASSERT_EQUAL(0, encode(payload, 0, frame, sizeof(frame)));

  // Exact fit works, one byte less does not (and returns 0)
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);
  TEST_ASSERT_EQUAL(written, encode(payload, sizeof(payload), frame, written));
  TEST_ASSERT_EQUAL(0, encode(payload, sizeof(payload), frame, written - 1));
  TEST_ASSERT_EQUAL(0, encode(payload, sizeof(payload), frame, 0));
}

/* ---------------------------------------------------------------------------------------------- */
/*                                       Encoder (streaming)                                      */
/* ---------------------------------------------------------------------------------------------- */

void test_encoder_streaming_matches_one_shot() {
  // Fed in uneven chunks (as if from a header + body), the Encoder must produce the exact same
  // bytes as the one-shot encode() and decode back to the original
  const uint8_t payload[9] = {0x11, 0x00, 0x22, 0x00, 0xAB, 0xCD, 0x00, 0xEE, 0xFF};

  uint8_t one_shot[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t one_shot_size = encode(payload, sizeof(payload), one_shot, sizeof(one_shot));
  TEST_ASSERT_TRUE(one_shot_size > 0);

  uint8_t streamed[getMaxEncodedSize(sizeof(payload))] = {};
  Encoder<> encoder(streamed, sizeof(streamed));
  encoder.feed(payload, 3);     // first chunk
  encoder.feed(payload + 3, 4); // second chunk
  encoder.feed(payload + 7, 2); // third chunk
  const size_t streamed_size = encoder.finalize();

  TEST_ASSERT_EQUAL(one_shot_size, streamed_size);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(one_shot, streamed, one_shot_size);

  Decoder<32> decoder;
  TEST_ASSERT_TRUE(feedAll(decoder, streamed, streamed_size));
  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_encoder_streaming_block_boundary() {
  // 252 payload + 2 CRC = one full 254-byte COBS block. Feeding split across the boundary must
  // still match the one-shot output exactly (no spurious trailing code byte)
  uint8_t payload[252];
  for (size_t i = 0; i < sizeof(payload); i++) {
    payload[i] = uint8_t(1 + (i % 255)); // never zero
  }

  uint8_t one_shot[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t one_shot_size = encode(payload, sizeof(payload), one_shot, sizeof(one_shot));

  uint8_t streamed[getMaxEncodedSize(sizeof(payload))] = {};
  Encoder<> encoder(streamed, sizeof(streamed));
  encoder.feed(payload, 100);
  encoder.feed(payload + 100, sizeof(payload) - 100);
  const size_t streamed_size = encoder.finalize();

  TEST_ASSERT_EQUAL(one_shot_size, streamed_size);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(one_shot, streamed, one_shot_size);
}

void test_encoder_reuse_and_limits() {
  const uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  Encoder<> encoder(frame, sizeof(frame));

  // finalize() with nothing fed yields no frame
  TEST_ASSERT_EQUAL(0, encoder.finalize());

  // restart() lets the same buffer encode a fresh frame, repeatably
  encoder.restart();
  encoder.feed(payload, sizeof(payload));
  const size_t size1 = encoder.finalize();
  TEST_ASSERT_TRUE(size1 > 0);
  TEST_ASSERT_TRUE(encoder.isOk());

  encoder.restart();
  encoder.feed(payload, sizeof(payload));
  const size_t size2 = encoder.finalize();
  TEST_ASSERT_EQUAL(size1, size2);

  // A capacity one byte too small overflows: finalize() returns 0 and isOk() turns false
  uint8_t tight[getMaxEncodedSize(sizeof(payload))] = {};
  Encoder<> small(tight, size1 - 1);
  small.feed(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(0, small.finalize());
  TEST_ASSERT_FALSE(small.isOk());
}

/* ---------------------------------------------------------------------------------------------- */
/*                                            Decoder                                             */
/* ---------------------------------------------------------------------------------------------- */

void test_decoder_initial_state() {
  Decoder<32> decoder;

  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(0, decoder.getPayloadSize());
  TEST_ASSERT_EQUAL(32, decoder.getMaxPayloadSize());
  TEST_ASSERT_EQUAL(0, decoder.getStats().frames_ok);
  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_decoder_byte_feed_round_trip() {
  const uint8_t payload[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  Decoder<32> decoder;

  // The frame becomes available exactly at the delimiter, not before
  for (size_t i = 0; i < written - 1; i++) {
    TEST_ASSERT_FALSE(decoder.feed(frame[i]));
    TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  }
  TEST_ASSERT_TRUE(decoder.feed(frame[written - 1]));
  TEST_ASSERT_TRUE(decoder.isFrameAvailable());

  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));

  // The next feed clears the availability flag
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x55)));
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(0, decoder.getPayloadSize());

  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_decoder_chunk_feed_multiple_frames() {
  const uint8_t payload1[3] = {0x01, 0x02, 0x03};
  const uint8_t payload2[4] = {0xAA, 0x00, 0xBB, 0x00};

  // Two frames back to back in a single buffer, as they would arrive from a stream
  uint8_t stream[getMaxEncodedSize(sizeof(payload1)) + getMaxEncodedSize(sizeof(payload2))] = {};

  const size_t size1 = encode(payload1, sizeof(payload1), stream, sizeof(stream));
  TEST_ASSERT_TRUE(size1 > 0);
  const size_t size2 = encode(payload2, sizeof(payload2), stream + size1, sizeof(stream) - size1);
  TEST_ASSERT_TRUE(size2 > 0);
  const size_t total = size1 + size2;

  Decoder<32> decoder;

  // First call consumes up to and including the delimiter of the first frame
  size_t consumed = decoder.feed(stream, total);
  TEST_ASSERT_EQUAL(size1, consumed);
  TEST_ASSERT_TRUE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(sizeof(payload1), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload1, decoder.getPayload(), sizeof(payload1));

  // Second call consumes the rest and yields the second frame
  consumed += decoder.feed(stream + consumed, total - consumed);
  TEST_ASSERT_EQUAL(total, consumed);
  TEST_ASSERT_TRUE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(sizeof(payload2), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload2, decoder.getPayload(), sizeof(payload2));
}

void test_decoder_max_payload() {
  Decoder<16> decoder;

  // A payload of exactly MaxPayload bytes is accepted
  uint8_t payload[17];
  for (size_t i = 0; i < sizeof(payload); i++) {
    payload[i] = uint8_t(i + 1);
  }

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  size_t written                                    = encode(payload, 16, frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(16, decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), 16);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);

  // One byte more overflows: no frame, counted once, parser recovers afterwards
  written = encode(payload, 17, frame, sizeof(frame));
  TEST_ASSERT_FALSE(feedAll(decoder, frame, written));
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(1, decoder.getStats().overflows);

  // A valid frame right after the oversized one decodes normally
  written = encode(payload, 16, frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(16, decoder.getPayloadSize());
  TEST_ASSERT_EQUAL(1, decoder.getStats().overflows);
}

void test_decoder_crc_error() {
  const uint8_t payload[4] = {0x10, 0x20, 0x30, 0x40};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // Corrupt one payload byte inside the frame, keeping it non-zero so the COBS structure (and
  // therefore the frame boundary) is preserved: the CRC must catch it
  frame[2] = (frame[2] == 0x7F) ? 0x7E : 0x7F;

  Decoder<32> decoder;
  TEST_ASSERT_FALSE(feedAll(decoder, frame, written));
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(1, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);

  // The decoder keeps working after the error
  const size_t written2 = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written2));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_decoder_malformed_frames() {
  Decoder<32> decoder;

  // Code byte announces 4 data bytes but the delimiter arrives mid-block
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x05)));
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x11)));
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x22)));
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x00)));
  TEST_ASSERT_EQUAL(1, decoder.getStats().malformed);

  // A frame decoding to fewer than 3 bytes (1 payload + 2 CRC) is too short
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x02)));
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x41)));
  TEST_ASSERT_FALSE(decoder.feed(uint8_t(0x00)));
  TEST_ASSERT_EQUAL(2, decoder.getStats().malformed);

  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_decoder_idle_delimiters_ignored() {
  Decoder<32> decoder;

  // Idle delimiters (empty frames) are not data and not errors
  for (int i = 0; i < 10; i++) {
    TEST_ASSERT_FALSE(decoder.feed(DELIMITER));
  }

  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_decoder_resync_mid_stream() {
  const uint8_t payload[6] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  Decoder<32> decoder;

  // Join the stream in the middle of a frame: the partial frame is dropped (one error of some
  // kind), and the next complete frame decodes normally
  for (size_t i = written / 2; i < written; i++) {
    decoder.feed(frame[i]);
  }
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());

  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_decoder_reset() {
  const uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  Decoder<32> decoder;

  // Feed half a frame, reset, then feed a complete frame: it must decode cleanly with no errors
  for (size_t i = 0; i < written / 2; i++) {
    decoder.feed(frame[i]);
  }
  decoder.reset();

  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                           Round-trip                                           */
/* ---------------------------------------------------------------------------------------------- */

void test_round_trip_all_sizes() {
  constexpr size_t MAX_PAYLOAD = 64;
  Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (size_t size = 1; size <= MAX_PAYLOAD; size++) {
    // Pattern with zeros sprinkled in to exercise the COBS paths
    for (size_t i = 0; i < size; i++) {
      payload[i] = (i % 3 == 0) ? 0x00 : uint8_t(i);
    }

    const size_t written = encode(payload, size, frame, sizeof(frame));
    TEST_ASSERT_TRUE(written > 0);
    TEST_ASSERT_TRUE(written <= getMaxEncodedSize(size));

    TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
    TEST_ASSERT_EQUAL(size, decoder.getPayloadSize());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), size);
  }

  TEST_ASSERT_EQUAL(MAX_PAYLOAD, decoder.getStats().frames_ok);
  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                     Decode & CRC policies                                      */
/* ---------------------------------------------------------------------------------------------- */

void test_crc_policies_check_values() {
  // Standard CRC check values over the ASCII string "123456789"
  const uint8_t data[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX8(0xF4, crcOf<Crc8Smbus>(data, sizeof(data)));
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crcOf<Crc16CcittFalse>(data, sizeof(data)));
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926, crcOf<Crc32IsoHdlc>(data, sizeof(data)));
}

void test_decode_one_shot() {
  const uint8_t payload[7] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0x00};

  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // Round-trips: only the payload is written out, so the buffer is just the payload size
  uint8_t out[sizeof(payload)] = {};
  TEST_ASSERT_EQUAL(sizeof(payload), decode(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // The trailing delimiter is optional: decoding the frame without it yields the same payload
  TEST_ASSERT_EQUAL(sizeof(payload), decode(frame, written - 1, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // A buffer too small for the payload is rejected
  TEST_ASSERT_EQUAL(0, decode(frame, written, out, sizeof(out) - 1));

  // Corrupting a payload byte (kept non-zero) is caught by the CRC
  frame[1] = (frame[1] == 0x7F) ? 0x7E : 0x7F;
  TEST_ASSERT_EQUAL(0, decode(frame, written, out, sizeof(out)));
}

// Encodes a fixed payload and decodes it back through both decode paths with a given CRC policy
template <class Crc>
void roundTripWithCrc() {
  const uint8_t payload[7] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0x00};

  uint8_t frame[getMaxEncodedSize<Crc>(sizeof(payload))] = {};
  const size_t written = encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // One-shot decode
  uint8_t out[sizeof(payload)] = {};
  TEST_ASSERT_EQUAL(sizeof(payload), decode<Crc>(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // Streaming decoder with the same policy
  Decoder<32, Crc> decoder;
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_crc_variants_round_trip() {
  roundTripWithCrc<NoCrc>();
  roundTripWithCrc<Crc8Smbus>();
  roundTripWithCrc<Crc16CcittFalse>();
  roundTripWithCrc<Crc32IsoHdlc>();

  // The CRC width drives the encoded size: a wider CRC yields a larger frame
  const size_t no_crc = getMaxEncodedSize<NoCrc>(8);
  const size_t crc8   = getMaxEncodedSize<Crc8Smbus>(8);
  const size_t crc16  = getMaxEncodedSize<Crc16CcittFalse>(8);
  const size_t crc32  = getMaxEncodedSize<Crc32IsoHdlc>(8);
  TEST_ASSERT_TRUE(no_crc < crc8);
  TEST_ASSERT_TRUE(crc8 < crc16);
  TEST_ASSERT_TRUE(crc16 < crc32);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                  All paths across CRC policies                                 */
/* ---------------------------------------------------------------------------------------------- */

/**
 * Encoder, Decoder, encode(), decode() and getMaxEncodedSize() are templates: gcov tracks every CRC
 * instantiation separately, so a branch exercised only for the default Crc16CcittFalse still counts
 * as uncovered for NoCrc, Crc8Smbus and Crc32IsoHdlc. The templated helpers below drive every error
 * and edge path and are instantiated once per policy so each instantiation reaches full coverage.
 */

// Encode a payload that crosses a full COBS block and contains zeros into every output-buffer
// capacity from 0 to the worst case. The full capacity exercises the happy path; each smaller
// capacity trips a different overflow branch (literal byte, encoded zero, full-block successor, CRC
// bytes, trailing delimiter). Every frame that is produced must round-trip back to the payload.
template <class Crc>
void exerciseEncoderOverflow() {
  constexpr size_t N = 300; // > 254 so the payload crosses a full COBS block boundary
  uint8_t payload[N];
  for (size_t i = 0; i < N; i++) {
    payload[i] = uint8_t(1 + (i % 254)); // never zero
  }
  payload[260] = 0x00; // zeros past the first block boundary (encoded-zero overflow paths)
  payload[280] = 0x00;

  uint8_t frame[getMaxEncodedSize<Crc>(N)] = {};
  uint8_t out[N]                           = {};
  bool produced                            = false;
  bool overflowed                          = false;

  for (size_t cap = 0; cap <= sizeof(frame); cap++) {
    const size_t written = encode<Crc>(payload, N, frame, cap);
    if (written == 0) {
      overflowed = true;
      continue;
    }
    produced = true;
    TEST_ASSERT_TRUE(written <= getMaxEncodedSize<Crc>(N));
    TEST_ASSERT_EQUAL(N, decode<Crc>(frame, written, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, N);
  }

  TEST_ASSERT_TRUE(produced);
  TEST_ASSERT_TRUE(overflowed);

  // finalize() on a fresh encoder with nothing fed yields no frame (the "empty" guard)
  Encoder<Crc> empty(frame, sizeof(frame));
  TEST_ASSERT_EQUAL(0, empty.finalize());
}

// A payload whose data (payload + CRC) is exactly 254 bytes encodes to a single full COBS block
// (code 0xFF) with no trailing code byte: this exercises the block-boundary close at finalize().
template <class Crc>
void exerciseEncoderFullBlock() {
  constexpr size_t payload_size                       = 254 - Crc::SIZE; // data == 254 == one block
  uint8_t payload[payload_size]                       = {};
  uint8_t frame[getMaxEncodedSize<Crc>(payload_size)] = {};

  size_t written = 0;
  for (unsigned seed = 0; seed < 256; seed++) {
    for (size_t i = 0; i < payload_size; i++) {
      payload[i] = uint8_t(1 + ((i + seed) % 254));
    }
    written = encode<Crc>(payload, payload_size, frame, sizeof(frame));
    // 1 code byte + 254 data bytes + delimiter == 256: reached only when the last data byte (the
    // top CRC byte) is non-zero, so the block closes "full" instead of via an encoded zero.
    if (written == 256 && frame[0] == 0xFF) break;
  }

  TEST_ASSERT_EQUAL(256, written);
  TEST_ASSERT_EQUAL_HEX8(0xFF, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(DELIMITER, frame[written - 1]);

  uint8_t out[payload_size] = {};
  TEST_ASSERT_EQUAL(payload_size, decode<Crc>(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, payload_size);
}

// Drive every Decoder path: idle delimiter, happy frame, mid-block and too-short malformed frames,
// overflow on a literal byte and on an implicit inter-block zero, recovery, and a CRC mismatch.
// Templated on MaxPayload as well, because each Decoder<MaxPayload, Crc> is a distinct gcov
// instantiation that must be exercised in full.
template <size_t MaxPayload, class Crc>
void exerciseDecoderPaths() {
  Decoder<MaxPayload, Crc> decoder;

  // getPayloadSize() before any frame is available -> 0
  TEST_ASSERT_EQUAL(0, decoder.getPayloadSize());

  // Idle delimiter: an empty frame is ignored, not an error
  TEST_ASSERT_FALSE(decoder.feed(DELIMITER));

  // Happy path
  const uint8_t payload[8] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0xCD, 0x00};
  uint8_t frame[getMaxEncodedSize<Crc>(sizeof(payload))] = {};
  size_t written = encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
  TEST_ASSERT_EQUAL(1, decoder.getStats().frames_ok);

  // Malformed: the delimiter arrives in the middle of a block
  decoder.feed(uint8_t(0x05)); // announces 4 data bytes
  decoder.feed(uint8_t(0x11));
  decoder.feed(uint8_t(0x22));
  TEST_ASSERT_FALSE(decoder.feed(DELIMITER));
  TEST_ASSERT_EQUAL(1, decoder.getStats().malformed);

  // Malformed: a frame decoding to fewer than 1 + Crc::SIZE bytes is too short (only meaningful
  // when the policy carries a CRC; with NoCrc a single decoded byte is already a valid frame)
  if (Crc::SIZE > 0) {
    decoder.feed(uint8_t(0x02)); // announces 1 data byte
    decoder.feed(uint8_t(0x41));
    TEST_ASSERT_FALSE(decoder.feed(DELIMITER));
    TEST_ASSERT_EQUAL(2, decoder.getStats().malformed);
  }

  // Overflow on a literal byte: a run of non-zero bytes longer than the buffer
  uint8_t big[MaxPayload + 8];
  for (size_t i = 0; i < sizeof(big); i++) {
    big[i] = uint8_t(1 + (i % 254));
  }
  uint8_t bigframe[getMaxEncodedSize<Crc>(sizeof(big))] = {};
  written = encode<Crc>(big, sizeof(big), bigframe, sizeof(bigframe));
  TEST_ASSERT_FALSE(feedAll(decoder, bigframe, written));
  TEST_ASSERT_EQUAL(1, decoder.getStats().overflows);

  // The decoder recovers after the overflow
  written = encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));

  // Overflow exactly on an implicit inter-block zero: a zero placed right at the buffer limit, so
  // the buffer fills on the literal run and the next push is the implied zero between blocks
  uint8_t big_zero[MaxPayload + 8];
  for (size_t i = 0; i < sizeof(big_zero); i++) {
    big_zero[i] = uint8_t(1 + (i % 254));
  }
  big_zero[MaxPayload + Crc::SIZE]                            = 0x00;
  uint8_t bigzframe[getMaxEncodedSize<Crc>(sizeof(big_zero))] = {};
  written               = encode<Crc>(big_zero, sizeof(big_zero), bigzframe, sizeof(bigzframe));
  const uint32_t before = decoder.getStats().overflows;
  TEST_ASSERT_FALSE(feedAll(decoder, bigzframe, written));
  TEST_ASSERT_EQUAL(before + 1, decoder.getStats().overflows);

  // CRC mismatch: corrupt a payload byte, keeping it non-zero so the COBS structure survives (only
  // a policy that carries a CRC can detect this)
  if (Crc::SIZE > 0) {
    written                   = encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
    frame[1]                  = (frame[1] == 0x7F) ? 0x7E : 0x7F;
    const uint32_t crc_before = decoder.getStats().crc_errors;
    TEST_ASSERT_FALSE(feedAll(decoder, frame, written));
    TEST_ASSERT_EQUAL(crc_before + 1, decoder.getStats().crc_errors);
  }
}

// Drive every one-shot decode() path: happy round-trip, optional trailing delimiter, capacity
// rejection, truncated COBS, interior zero, too-short frame and CRC mismatch.
template <class Crc>
void exerciseDecodePaths() {
  const uint8_t payload[8] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0xCD, 0x00};
  uint8_t frame[getMaxEncodedSize<Crc>(sizeof(payload))] = {};
  const size_t written         = encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
  uint8_t out[sizeof(payload)] = {};

  // Happy path
  TEST_ASSERT_EQUAL(sizeof(payload), decode<Crc>(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // The trailing delimiter is optional: without it the first/second pass loops exit on frame_size
  TEST_ASSERT_EQUAL(sizeof(payload), decode<Crc>(frame, written - 1, out, sizeof(out)));

  // Output buffer too small for the payload
  TEST_ASSERT_EQUAL(0, decode<Crc>(frame, written, out, sizeof(out) - 1));

  // Truncated COBS block: the code byte announces more bytes than the frame provides
  const uint8_t truncated[3] = {0x05, 0x11, 0x22};
  TEST_ASSERT_EQUAL(0, decode<Crc>(truncated, sizeof(truncated), out, sizeof(out)));

  // Interior zero: a zero may not appear inside a COBS block
  const uint8_t interior_zero[3] = {0x03, 0x11, 0x00};
  TEST_ASSERT_EQUAL(0, decode<Crc>(interior_zero, sizeof(interior_zero), out, sizeof(out)));

  // Too short: not even one payload byte plus the CRC. With a CRC, a 1-byte decode is too short;
  // with NoCrc only an empty frame (zero decoded bytes) is too short.
  if (Crc::SIZE > 0) {
    const uint8_t too_short[3] = {0x02, 0x41, 0x00}; // decodes to 1 byte < 1 + Crc::SIZE
    TEST_ASSERT_EQUAL(0, decode<Crc>(too_short, sizeof(too_short), out, sizeof(out)));
  } else {
    const uint8_t empty_frame[1] = {0x00}; // decodes to 0 bytes < 1
    TEST_ASSERT_EQUAL(0, decode<Crc>(empty_frame, sizeof(empty_frame), out, sizeof(out)));
  }

  // CRC mismatch (only a policy that carries a CRC can detect this)
  if (Crc::SIZE > 0) {
    uint8_t corrupted[getMaxEncodedSize<Crc>(sizeof(payload))] = {};
    for (size_t i = 0; i < written; i++) {
      corrupted[i] = frame[i];
    }
    corrupted[1] = (corrupted[1] == 0x7F) ? 0x7E : 0x7F;
    TEST_ASSERT_EQUAL(0, decode<Crc>(corrupted, written, out, sizeof(out)));
  }
}

// Run every path helper for one CRC policy. The Decoder paths run for each MaxPayload the suite
// instantiates (16, 32, 64), since gcov tracks every Decoder<MaxPayload, Crc> separately.
template <class Crc>
void exerciseAllPaths() {
  exerciseEncoderOverflow<Crc>();
  exerciseEncoderFullBlock<Crc>();
  exerciseDecodePaths<Crc>();
  exerciseDecoderPaths<16, Crc>();
  exerciseDecoderPaths<32, Crc>();
  exerciseDecoderPaths<64, Crc>();
}

void test_all_paths_no_crc() { exerciseAllPaths<NoCrc>(); }
void test_all_paths_crc8() { exerciseAllPaths<Crc8Smbus>(); }
void test_all_paths_crc16() { exerciseAllPaths<Crc16CcittFalse>(); }
void test_all_paths_crc32() { exerciseAllPaths<Crc32IsoHdlc>(); }

/* ---------------------------------------------------------------------------------------------- */
/*                                             Stress                                             */
/* ---------------------------------------------------------------------------------------------- */

constexpr uint32_t STRESS_ITERATIONS = 10000;

// Deterministic PRNG (xorshift32): failures are reproducible from the seed
static uint32_t stress_state = 1;

uint32_t nextRandom() {
  stress_state ^= stress_state << 13;
  stress_state ^= stress_state >> 17;
  stress_state ^= stress_state << 5;
  return stress_state;
}

void test_stress_round_trip_random_chunks() {
  stress_state = 0xC0FFEE11;

  constexpr size_t MAX_PAYLOAD = 64;
  Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    const size_t size = 1 + (nextRandom() % MAX_PAYLOAD);
    for (size_t j = 0; j < size; j++) {
      payload[j] = uint8_t(nextRandom());
    }

    const size_t written = encode(payload, size, frame, sizeof(frame));
    TEST_ASSERT_TRUE(written > 0);

    // Feed the frame in random-sized chunks, as a stream would deliver it
    size_t consumed  = 0;
    bool frame_found = false;
    while (consumed < written) {
      const size_t chunk = 1 + (nextRandom() % (written - consumed));
      consumed += decoder.feed(frame + consumed, chunk);
      frame_found = decoder.isFrameAvailable();
    }

    TEST_ASSERT_TRUE(frame_found);
    TEST_ASSERT_EQUAL(size, decoder.getPayloadSize());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), size);
  }

  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_stress_garbage_input() {
  stress_state = 0xA5A5A5A5;

  Decoder<32> decoder;

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    // Random bytes, delimiters included. Random garbage virtually never carries a valid CRC; the
    // property under test is "no crash and invariants hold", not "rejects everything"
    if (decoder.feed(uint8_t(nextRandom()))) {
      TEST_ASSERT_TRUE(decoder.getPayloadSize() >= 1);
      TEST_ASSERT_TRUE(decoder.getPayloadSize() <= decoder.getMaxPayloadSize());
    }
  }

  // After the garbage, a valid frame still decodes
  const uint8_t payload[3]                          = {0x01, 0x02, 0x03};
  uint8_t frame[getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = encode(payload, sizeof(payload), frame, sizeof(frame));

  decoder.feed(DELIMITER); // close any partial garbage frame first
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_stress_corruption() {
  stress_state = 0xDEADBEEF;

  constexpr size_t MAX_PAYLOAD = 32;
  Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    const size_t size = 1 + (nextRandom() % MAX_PAYLOAD);
    for (size_t j = 0; j < size; j++) {
      payload[j] = uint8_t(nextRandom());
    }

    const size_t written = encode(payload, size, frame, sizeof(frame));
    TEST_ASSERT_TRUE(written > 0);

    // Corrupt one random byte of the encoded frame (delimiter excluded), forcing a real change.
    // The corruption may garble the COBS structure or the content; either way the decoder must
    // never deliver a payload different from the original one
    const size_t corrupt_pos = nextRandom() % (written - 1);
    uint8_t corrupted        = frame[corrupt_pos];
    while (corrupted == frame[corrupt_pos]) {
      corrupted = uint8_t(nextRandom());
    }
    const uint8_t original = frame[corrupt_pos];
    frame[corrupt_pos]     = corrupted;

    size_t consumed = 0;
    while (consumed < written) {
      consumed += decoder.feed(frame + consumed, written - consumed);
      if (decoder.isFrameAvailable()) {
        // Only acceptable outcome: the corruption canceled out and the payload is intact
        TEST_ASSERT_EQUAL(size, decoder.getPayloadSize());
        TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), size);
      }
    }

    // Restore the frame and close any partial state so iterations stay independent
    frame[corrupt_pos] = original;
    decoder.feed(DELIMITER);
  }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                             Runners                                            */
/* ---------------------------------------------------------------------------------------------- */

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

int runUnityTests(void) {
  UNITY_BEGIN();

  // Size helper
  RUN_TEST(test_get_max_encoded_size);

  // Encoder
  RUN_TEST(test_encode_known_vector);
  RUN_TEST(test_encode_payload_with_zeros);
  RUN_TEST(test_encode_leading_zero);
  RUN_TEST(test_encode_block_boundary);
  RUN_TEST(test_encode_structural_properties);
  RUN_TEST(test_encode_rejects_invalid_input);

  // Encoder (streaming)
  RUN_TEST(test_encoder_streaming_matches_one_shot);
  RUN_TEST(test_encoder_streaming_block_boundary);
  RUN_TEST(test_encoder_reuse_and_limits);

  // Decoder
  RUN_TEST(test_decoder_initial_state);
  RUN_TEST(test_decoder_byte_feed_round_trip);
  RUN_TEST(test_decoder_chunk_feed_multiple_frames);
  RUN_TEST(test_decoder_max_payload);
  RUN_TEST(test_decoder_crc_error);
  RUN_TEST(test_decoder_malformed_frames);
  RUN_TEST(test_decoder_idle_delimiters_ignored);
  RUN_TEST(test_decoder_resync_mid_stream);
  RUN_TEST(test_decoder_reset);

  // Round-trip
  RUN_TEST(test_round_trip_all_sizes);

  // Decode & CRC policies
  RUN_TEST(test_crc_policies_check_values);
  RUN_TEST(test_decode_one_shot);
  RUN_TEST(test_crc_variants_round_trip);

  // All paths across CRC policies
  RUN_TEST(test_all_paths_no_crc);
  RUN_TEST(test_all_paths_crc8);
  RUN_TEST(test_all_paths_crc16);
  RUN_TEST(test_all_paths_crc32);

  // Stress
  RUN_TEST(test_stress_round_trip_random_chunks);
  RUN_TEST(test_stress_garbage_input);
  RUN_TEST(test_stress_corruption);

  return UNITY_END();
}

// For native
int main(void) { return runUnityTests(); }

// For Arduino framework
#ifdef ARDUINO
void setup() {
  delay(2000);
  runUnityTests();
}
void loop() {}
#endif
