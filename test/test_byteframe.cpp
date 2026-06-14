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

#include <Arduino.h>
#include <unity.h>

#include <ByteFrame.h>

/* ---------------------------------------------------------------------------------------------- */
/*                                            Helpers                                             */
/* ---------------------------------------------------------------------------------------------- */

// Feeds a whole buffer byte by byte and returns true if exactly one valid frame came out,
// completed by the very last byte
template <size_t MaxPayload, class Crc>
bool feedAll(ByteFrame::Decoder<MaxPayload, Crc>& decoder, const uint8_t* data, const size_t size) {
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
  constexpr size_t size_1   = ByteFrame::getMaxEncodedSize(1);   // 3 + 1 + 1
  constexpr size_t size_10  = ByteFrame::getMaxEncodedSize(10);  // 12 + 1 + 1
  constexpr size_t size_252 = ByteFrame::getMaxEncodedSize(252); // 254 + 1 + 1
  constexpr size_t size_253 = ByteFrame::getMaxEncodedSize(253); // 255 + 2 + 1
  constexpr size_t size_300 = ByteFrame::getMaxEncodedSize(300); // 302 + 2 + 1

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

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, sizeof(expected));
}

void test_encode_payload_with_zeros() {
  // data = [0x11, 0x00, 0x22, crc_lo, crc_hi]: the zero splits the data in two COBS blocks
  const uint8_t payload[3] = {0x11, 0x00, 0x22};
  const uint16_t crc       = crcOf<ByteFrame::Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo     = uint8_t(crc & 0xFF);
  const uint8_t crc_hi     = uint8_t(crc >> 8);

  // The expected layout below assumes the CRC bytes are not zero for this payload
  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  const uint8_t expected[7] = {0x02, 0x11, 0x04, 0x22, crc_lo, crc_hi, 0x00};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_EQUAL(7, written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, 7);
}

void test_encode_leading_zero() {
  // data = [0x00, crc_lo, crc_hi]: a leading zero produces an empty first block (code 0x01)
  const uint8_t payload[1] = {0x00};
  const uint16_t crc       = crcOf<ByteFrame::Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo     = uint8_t(crc & 0xFF);
  const uint8_t crc_hi     = uint8_t(crc >> 8);

  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  const uint8_t expected[5] = {0x01, 0x03, crc_lo, crc_hi, 0x00};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

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

  const uint16_t crc   = crcOf<ByteFrame::Crc16CcittFalse>(payload, sizeof(payload));
  const uint8_t crc_lo = uint8_t(crc & 0xFF);
  const uint8_t crc_hi = uint8_t(crc >> 8);

  // The expected layout below assumes the CRC bytes are not zero for this payload
  TEST_ASSERT_NOT_EQUAL(0x00, crc_lo);
  TEST_ASSERT_NOT_EQUAL(0x00, crc_hi);

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

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

  uint8_t frame2[ByteFrame::getMaxEncodedSize(sizeof(payload2))] = {};
  const size_t written2 = ByteFrame::encode(payload2, sizeof(payload2), frame2, sizeof(frame2));

  TEST_ASSERT_TRUE(written2 > 0);
  TEST_ASSERT_TRUE(written2 <= ByteFrame::getMaxEncodedSize(sizeof(payload2)));
  TEST_ASSERT_EQUAL_HEX8(0xFF, frame2[0]);
}

void test_encode_structural_properties() {
  // Whatever the payload, a frame ends with exactly one delimiter and contains no other zero
  const uint8_t payload[6] = {0x00, 0xFF, 0x00, 0x00, 0x01, 0x00};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

  TEST_ASSERT_TRUE(written > 0);
  TEST_ASSERT_TRUE(written <= ByteFrame::getMaxEncodedSize(sizeof(payload)));
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[written - 1]);
  for (size_t i = 0; i < written - 1; i++) {
    TEST_ASSERT_NOT_EQUAL(0x00, frame[i]);
  }
}

void test_encode_rejects_invalid_input() {
  uint8_t frame[16]        = {};
  const uint8_t payload[4] = {1, 2, 3, 4};

  // Empty payload
  TEST_ASSERT_EQUAL(0, ByteFrame::encode(payload, 0, frame, sizeof(frame)));

  // Exact fit works, one byte less does not (and returns 0)
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);
  TEST_ASSERT_EQUAL(written, ByteFrame::encode(payload, sizeof(payload), frame, written));
  TEST_ASSERT_EQUAL(0, ByteFrame::encode(payload, sizeof(payload), frame, written - 1));
  TEST_ASSERT_EQUAL(0, ByteFrame::encode(payload, sizeof(payload), frame, 0));
}

/* ---------------------------------------------------------------------------------------------- */
/*                                       Encoder (streaming)                                      */
/* ---------------------------------------------------------------------------------------------- */

void test_encoder_streaming_matches_one_shot() {
  // Fed in uneven chunks (as if from a header + body), the Encoder must produce the exact same
  // bytes as the one-shot encode() and decode back to the original
  const uint8_t payload[9] = {0x11, 0x00, 0x22, 0x00, 0xAB, 0xCD, 0x00, 0xEE, 0xFF};

  uint8_t one_shot[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t one_shot_size =
    ByteFrame::encode(payload, sizeof(payload), one_shot, sizeof(one_shot));
  TEST_ASSERT_TRUE(one_shot_size > 0);

  uint8_t streamed[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  ByteFrame::Encoder<> encoder(streamed, sizeof(streamed));
  encoder.feed(payload, 3);     // first chunk
  encoder.feed(payload + 3, 4); // second chunk
  encoder.feed(payload + 7, 2); // third chunk
  const size_t streamed_size = encoder.finalize();

  TEST_ASSERT_EQUAL(one_shot_size, streamed_size);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(one_shot, streamed, one_shot_size);

  ByteFrame::Decoder<32> decoder;
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

  uint8_t one_shot[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t one_shot_size =
    ByteFrame::encode(payload, sizeof(payload), one_shot, sizeof(one_shot));

  uint8_t streamed[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  ByteFrame::Encoder<> encoder(streamed, sizeof(streamed));
  encoder.feed(payload, 100);
  encoder.feed(payload + 100, sizeof(payload) - 100);
  const size_t streamed_size = encoder.finalize();

  TEST_ASSERT_EQUAL(one_shot_size, streamed_size);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(one_shot, streamed, one_shot_size);
}

void test_encoder_reuse_and_limits() {
  const uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  ByteFrame::Encoder<> encoder(frame, sizeof(frame));

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
  uint8_t tight[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  ByteFrame::Encoder<> small(tight, size1 - 1);
  small.feed(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(0, small.finalize());
  TEST_ASSERT_FALSE(small.isOk());
}

/* ---------------------------------------------------------------------------------------------- */
/*                                            Decoder                                             */
/* ---------------------------------------------------------------------------------------------- */

void test_decoder_initial_state() {
  ByteFrame::Decoder<32> decoder;

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

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  ByteFrame::Decoder<32> decoder;

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
  uint8_t stream[ByteFrame::getMaxEncodedSize(sizeof(payload1)) +
                 ByteFrame::getMaxEncodedSize(sizeof(payload2))] = {};

  const size_t size1 = ByteFrame::encode(payload1, sizeof(payload1), stream, sizeof(stream));
  TEST_ASSERT_TRUE(size1 > 0);
  const size_t size2 =
    ByteFrame::encode(payload2, sizeof(payload2), stream + size1, sizeof(stream) - size1);
  TEST_ASSERT_TRUE(size2 > 0);
  const size_t total = size1 + size2;

  ByteFrame::Decoder<32> decoder;

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
  ByteFrame::Decoder<16> decoder;

  // A payload of exactly MaxPayload bytes is accepted
  uint8_t payload[17];
  for (size_t i = 0; i < sizeof(payload); i++) {
    payload[i] = uint8_t(i + 1);
  }

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  size_t written = ByteFrame::encode(payload, 16, frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(16, decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), 16);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);

  // One byte more overflows: no frame, counted once, parser recovers afterwards
  written = ByteFrame::encode(payload, 17, frame, sizeof(frame));
  TEST_ASSERT_FALSE(feedAll(decoder, frame, written));
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(1, decoder.getStats().overflows);

  // A valid frame right after the oversized one decodes normally
  written = ByteFrame::encode(payload, 16, frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(16, decoder.getPayloadSize());
  TEST_ASSERT_EQUAL(1, decoder.getStats().overflows);
}

void test_decoder_crc_error() {
  const uint8_t payload[4] = {0x10, 0x20, 0x30, 0x40};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // Corrupt one payload byte inside the frame, keeping it non-zero so the COBS structure (and
  // therefore the frame boundary) is preserved: the CRC must catch it
  frame[2] = (frame[2] == 0x7F) ? 0x7E : 0x7F;

  ByteFrame::Decoder<32> decoder;
  TEST_ASSERT_FALSE(feedAll(decoder, frame, written));
  TEST_ASSERT_FALSE(decoder.isFrameAvailable());
  TEST_ASSERT_EQUAL(1, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);

  // The decoder keeps working after the error
  const size_t written2 = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written2));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_decoder_malformed_frames() {
  ByteFrame::Decoder<32> decoder;

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
  ByteFrame::Decoder<32> decoder;

  // Idle delimiters (empty frames) are not data and not errors
  for (int i = 0; i < 10; i++) {
    TEST_ASSERT_FALSE(decoder.feed(ByteFrame::DELIMITER));
  }

  TEST_ASSERT_EQUAL(0, decoder.getStats().crc_errors);
  TEST_ASSERT_EQUAL(0, decoder.getStats().malformed);
  TEST_ASSERT_EQUAL(0, decoder.getStats().overflows);
}

void test_decoder_resync_mid_stream() {
  const uint8_t payload[6] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  ByteFrame::Decoder<32> decoder;

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

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  ByteFrame::Decoder<32> decoder;

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
  ByteFrame::Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (size_t size = 1; size <= MAX_PAYLOAD; size++) {
    // Pattern with zeros sprinkled in to exercise the COBS paths
    for (size_t i = 0; i < size; i++) {
      payload[i] = (i % 3 == 0) ? 0x00 : uint8_t(i);
    }

    const size_t written = ByteFrame::encode(payload, size, frame, sizeof(frame));
    TEST_ASSERT_TRUE(written > 0);
    TEST_ASSERT_TRUE(written <= ByteFrame::getMaxEncodedSize(size));

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
  TEST_ASSERT_EQUAL_HEX8(0xF4, crcOf<ByteFrame::Crc8Smbus>(data, sizeof(data)));
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crcOf<ByteFrame::Crc16CcittFalse>(data, sizeof(data)));
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926, crcOf<ByteFrame::Crc32IsoHdlc>(data, sizeof(data)));
}

void test_decode_one_shot() {
  const uint8_t payload[7] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0x00};

  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // Round-trips: only the payload is written out, so the buffer is just the payload size
  uint8_t out[sizeof(payload)] = {};
  TEST_ASSERT_EQUAL(sizeof(payload), ByteFrame::decode(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // The trailing delimiter is optional: decoding the frame without it yields the same payload
  TEST_ASSERT_EQUAL(sizeof(payload), ByteFrame::decode(frame, written - 1, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // A buffer too small for the payload is rejected
  TEST_ASSERT_EQUAL(0, ByteFrame::decode(frame, written, out, sizeof(out) - 1));

  // Corrupting a payload byte (kept non-zero) is caught by the CRC
  frame[1] = (frame[1] == 0x7F) ? 0x7E : 0x7F;
  TEST_ASSERT_EQUAL(0, ByteFrame::decode(frame, written, out, sizeof(out)));
}

// Encodes a fixed payload and decodes it back through both decode paths with a given CRC policy
template <class Crc>
void roundTripWithCrc() {
  const uint8_t payload[7] = {0x11, 0x00, 0x22, 0xFF, 0x00, 0xAB, 0x00};

  uint8_t frame[ByteFrame::getMaxEncodedSize<Crc>(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode<Crc>(payload, sizeof(payload), frame, sizeof(frame));
  TEST_ASSERT_TRUE(written > 0);

  // One-shot decode
  uint8_t out[sizeof(payload)] = {};
  TEST_ASSERT_EQUAL(sizeof(payload), ByteFrame::decode<Crc>(frame, written, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, out, sizeof(payload));

  // Streaming decoder with the same policy
  ByteFrame::Decoder<32, Crc> decoder;
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL(sizeof(payload), decoder.getPayloadSize());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_crc_variants_round_trip() {
  roundTripWithCrc<ByteFrame::NoCrc>();
  roundTripWithCrc<ByteFrame::Crc8Smbus>();
  roundTripWithCrc<ByteFrame::Crc16CcittFalse>();
  roundTripWithCrc<ByteFrame::Crc32IsoHdlc>();

  // The CRC width drives the encoded size: a wider CRC yields a larger frame
  const size_t no_crc = ByteFrame::getMaxEncodedSize<ByteFrame::NoCrc>(8);
  const size_t crc8   = ByteFrame::getMaxEncodedSize<ByteFrame::Crc8Smbus>(8);
  const size_t crc16  = ByteFrame::getMaxEncodedSize<ByteFrame::Crc16CcittFalse>(8);
  const size_t crc32  = ByteFrame::getMaxEncodedSize<ByteFrame::Crc32IsoHdlc>(8);
  TEST_ASSERT_TRUE(no_crc < crc8);
  TEST_ASSERT_TRUE(crc8 < crc16);
  TEST_ASSERT_TRUE(crc16 < crc32);
}

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
  ByteFrame::Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    const size_t size = 1 + (nextRandom() % MAX_PAYLOAD);
    for (size_t j = 0; j < size; j++) {
      payload[j] = uint8_t(nextRandom());
    }

    const size_t written = ByteFrame::encode(payload, size, frame, sizeof(frame));
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

  ByteFrame::Decoder<32> decoder;

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    // Random bytes, delimiters included. Random garbage virtually never carries a valid CRC; the
    // property under test is "no crash and invariants hold", not "rejects everything"
    if (decoder.feed(uint8_t(nextRandom()))) {
      TEST_ASSERT_TRUE(decoder.getPayloadSize() >= 1);
      TEST_ASSERT_TRUE(decoder.getPayloadSize() <= decoder.getMaxPayloadSize());
    }
  }

  // After the garbage, a valid frame still decodes
  const uint8_t payload[3]                                     = {0x01, 0x02, 0x03};
  uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(payload))] = {};
  const size_t written = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

  decoder.feed(ByteFrame::DELIMITER); // close any partial garbage frame first
  TEST_ASSERT_TRUE(feedAll(decoder, frame, written));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, decoder.getPayload(), sizeof(payload));
}

void test_stress_corruption() {
  stress_state = 0xDEADBEEF;

  constexpr size_t MAX_PAYLOAD = 32;
  ByteFrame::Decoder<MAX_PAYLOAD> decoder;

  uint8_t payload[MAX_PAYLOAD];
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  for (uint32_t i = 0; i < STRESS_ITERATIONS; i++) {
    const size_t size = 1 + (nextRandom() % MAX_PAYLOAD);
    for (size_t j = 0; j < size; j++) {
      payload[j] = uint8_t(nextRandom());
    }

    const size_t written = ByteFrame::encode(payload, size, frame, sizeof(frame));
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
    decoder.feed(ByteFrame::DELIMITER);
  }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                          setup / loop                                          */
/* ---------------------------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(2000);

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

  // Stress
  RUN_TEST(test_stress_round_trip_random_chunks);
  RUN_TEST(test_stress_garbage_input);
  RUN_TEST(test_stress_corruption);

  UNITY_END();
}

void loop() {}
