/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * Basic Example Overview:
 * - Sizes the transmit buffer at compile time with getMaxEncodedSize().
 * - Encodes a payload into a self-delimited frame with encode() (COBS + CRC16 + delimiter) and
 *   prints the bytes.
 * - Feeds the frame to a Decoder as a stream would deliver it and recovers the payload.
 * - Corrupts one byte of the frame to show the CRC rejecting it and the error counters at work.
 */

#include <Arduino.h>

#include <ByteFrame.h>

// Maximum payload this link accepts; both the TX buffer and the Decoder derive from it
constexpr size_t MAX_PAYLOAD = 32;

ByteFrame::Decoder<MAX_PAYLOAD> decoder;

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

  Serial.println("-------------------------");
  Serial.println("ByteFrame - Basic Example");
  Serial.println("-------------------------");

  // Any bytes work as payload: a BytePack message, a string, a raw struct...
  const uint8_t payload[] = {'H', 'i', ' ', 0x00, 0x42, 0xFF}; // zeros are fine: COBS handles them

  // TX buffer sized at compile time for the worst case (CRC + COBS overhead + delimiter)
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  const size_t frame_size = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  if (frame_size == 0) {
    Serial.println("Encoding failed: frame buffer too small");
    return;
  }

  Serial.print("Payload (");
  Serial.print(sizeof(payload));
  Serial.println(" bytes):");
  printHex(payload, sizeof(payload));
  Serial.println();

  Serial.print("Encoded frame (");
  Serial.print(frame_size);
  Serial.println(" bytes, ends with the 0x00 delimiter):");
  printHex(frame, frame_size);
  Serial.println();

  // Feed the frame byte by byte, as it would arrive from a UART
  for (size_t i = 0; i < frame_size; i++) {
    if (decoder.feed(frame[i])) {
      Serial.print("Decoded payload (");
      Serial.print(decoder.getPayloadSize());
      Serial.println(" bytes):");
      printHex(decoder.getPayload(), decoder.getPayloadSize());
      Serial.println();
    }
  }

  // Corrupt one byte: the CRC rejects the frame and the counter registers it
  frame[2] ^= 0x55;

  for (size_t i = 0; i < frame_size; i++) {
    decoder.feed(frame[i]);
  }

  const auto& stats = decoder.getStats();

  Serial.println("After feeding a corrupted copy of the frame:");
  Serial.print("- Frame available: ");
  Serial.println(decoder.isFrameAvailable() ? "yes" : "no");
  Serial.print("- Frames OK: ");
  Serial.println(stats.frames_ok);
  Serial.print("- CRC errors: ");
  Serial.println(stats.crc_errors);
  Serial.print("- Malformed errors: ");
  Serial.println(stats.malformed);
  Serial.print("- Overflow errors: ");
  Serial.println(stats.overflows);
}

void loop() {}
