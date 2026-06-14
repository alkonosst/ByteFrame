/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * StreamingEncoder Example Overview:
 * - Builds a single frame from several non-contiguous sources (a type byte, a length byte and a
 *   body) with the streaming Encoder, so no intermediate buffer is needed to concatenate them
 *   first.
 * - Shows both feed() overloads: one byte at a time and a whole chunk.
 * - Proves the streamed frame is byte-identical to the same payload passed to one-shot encode().
 * - Detects a buffer overflow with isOk()/finalize() (nothing partial is sent), then reuses the
 * same buffer for a second frame with restart().
 */

#include <Arduino.h>

#include <ByteFrame.h>

constexpr uint8_t MSG_TYPE = 0x07;
constexpr size_t MAX_BODY  = 32;

// Frame buffer sized at compile time for the worst case:
// 2 header bytes + body + CRC + COBS overhead
uint8_t frame[ByteFrame::getMaxEncodedSize(2 + MAX_BODY)] = {};

void printHex(const uint8_t* data, const size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("------------------------------------");
  Serial.println("ByteFrame - StreamingEncoder Example");
  Serial.println("------------------------------------");

  const char* body       = "temp=23.5";
  const size_t body_size = strlen(body);

  // Assemble the frame from three sources, in order, straight into the output buffer
  ByteFrame::Encoder<> encoder(frame, sizeof(frame));              // <> selects the default CRC
  encoder.feed(MSG_TYPE);                                          // 1) a single type byte
  encoder.feed(uint8_t(body_size));                                // 2) a single length byte
  encoder.feed(reinterpret_cast<const uint8_t*>(body), body_size); // 3) the body chunk

  const size_t frame_size = encoder.finalize();
  if (frame_size == 0) {
    Serial.println("Encoding failed: frame buffer too small");
    return;
  }

  Serial.print("Streamed frame (");
  Serial.print(frame_size);
  Serial.println(" bytes):");
  printHex(frame, frame_size);
  Serial.println();

  // The same payload built contiguously and passed to encode() must produce the exact same bytes
  uint8_t whole[2 + MAX_BODY] = {};
  whole[0]                    = MSG_TYPE;
  whole[1]                    = uint8_t(body_size);
  memcpy(whole + 2, body, body_size);

  uint8_t frame2[sizeof(frame)] = {};
  const size_t frame2_size      = ByteFrame::encode(whole, 2 + body_size, frame2, sizeof(frame2));
  const bool identical = (frame2_size == frame_size) && (memcmp(frame, frame2, frame_size) == 0);

  Serial.print("Identical to one-shot encode(): ");
  Serial.println(identical ? "yes" : "no");

  // Decode it back to confirm the payload survived the round trip
  uint8_t out[2 + MAX_BODY] = {};
  const size_t out_size     = ByteFrame::decode(frame, frame_size, out, sizeof(out));
  Serial.print("Decoded payload (");
  Serial.print(out_size);
  Serial.println(" bytes): type, length, body");
  printHex(out, out_size);
  Serial.println();

  // Overflow: a buffer too small to hold the frame. finalize() returns 0 and isOk() turns false,
  // so a partial frame is never mistaken for a complete one
  uint8_t tiny[4] = {};
  ByteFrame::Encoder<> tiny_encoder(tiny, sizeof(tiny));
  tiny_encoder.feed(reinterpret_cast<const uint8_t*>(body), body_size);
  const size_t tiny_size = tiny_encoder.finalize();

  Serial.print("Into a 4-byte buffer -> finalize(): ");
  Serial.print(tiny_size);
  Serial.print(", isOk(): ");
  Serial.println(tiny_encoder.isOk() ? "yes" : "no");

  // restart() rewinds the encoder so the same buffer can frame the next message
  encoder.restart();
  const uint8_t ping[] = {'p', 'i', 'n', 'g'};
  encoder.feed(ping, sizeof(ping));
  const size_t ping_size = encoder.finalize();

  Serial.print("After restart(), next frame (");
  Serial.print(ping_size);
  Serial.println(" bytes):");
  printHex(frame, ping_size);
}

void loop() {}
