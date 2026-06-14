/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * OneShotDecode Example Overview:
 * - Encodes a payload, then decodes the whole frame from a buffer with decode() - the one-shot
 *   counterpart to encode(), for when a frame is already complete in memory rather than arriving as
 *   a stream.
 * - Only the payload is written out (the CRC is validated and discarded), so the output buffer just
 *   needs MAX_PAYLOAD bytes.
 * - Shows the trailing delimiter is optional, and that decode() returns 0 on a CRC error or when
 *   the payload does not fit the output buffer.
 */

#include <Arduino.h>

#include <ByteFrame.h>

constexpr size_t MAX_PAYLOAD = 32;

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

  Serial.println("---------------------------------");
  Serial.println("ByteFrame - OneShotDecode Example");
  Serial.println("---------------------------------");

  const uint8_t payload[] = {0xDE, 0xAD, 0x00, 0xBE, 0xEF}; // a zero in the middle is fine

  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};
  const size_t frame_size = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));

  // The output buffer only has to hold the payload, never the CRC
  uint8_t out[MAX_PAYLOAD] = {};

  // 1) Normal decode of the complete frame (delimiter included)
  size_t out_size = ByteFrame::decode(frame, frame_size, out, sizeof(out));
  Serial.print("1) Decoded payload (");
  Serial.print(out_size);
  Serial.println(" bytes):");
  printHex(out, out_size);
  Serial.println();

  // 2) The trailing 0x00 delimiter is optional: decode stops at the first 0x00 or at the end
  out_size = ByteFrame::decode(frame, frame_size - 1, out, sizeof(out));
  Serial.print("2) Without the trailing delimiter: ");
  Serial.print(out_size);
  Serial.println(" bytes decoded");

  // 3) CRC error: flipping a payload byte makes the stored CRC no longer match
  uint8_t corrupted[sizeof(frame)] = {};
  memcpy(corrupted, frame, frame_size);
  corrupted[1] += 1; // a data byte; stays non-zero so COBS still parses
  out_size = ByteFrame::decode(corrupted, frame_size, out, sizeof(out));
  Serial.print("3) Corrupted frame: ");
  Serial.print(out_size);
  Serial.println(" bytes (0 = rejected by CRC)");

  // 4) Insufficient capacity: the payload (5 bytes) does not fit a 4-byte buffer
  out_size = ByteFrame::decode(frame, frame_size, out, sizeof(payload) - 1);
  Serial.print("4) Into a too-small buffer: ");
  Serial.print(out_size);
  Serial.println(" bytes (0 = rejected, nothing written)");
}

void loop() {}
