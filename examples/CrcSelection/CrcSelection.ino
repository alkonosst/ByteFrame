/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * CrcSelection Example Overview:
 * - Encodes the same payload with each built-in CRC policy (NoCrc, Crc8Smbus, Crc16CcittFalse,
 *   Crc32IsoHdlc) and prints how the frame grows with the CRC width.
 * - Round-trips each frame through decode() with the matching policy.
 * - Shows the policy is part of the contract: decoding a frame with a different CRC rejects it, so
 *   both endpoints must agree on the same policy.
 */

#include <Arduino.h>

#include <ByteFrame.h>

// Largest payload these buffers accept; every policy is sized against it at compile time
constexpr size_t MAX_PAYLOAD = 16;

// Encode the payload with policy Crc, decode it back with the same policy, and report the result.
// The buffer size adapts to the policy through getMaxEncodedSize<Crc>().
template <class Crc>
void showPolicy(const char* name, const uint8_t* payload, const size_t payload_size) {
  uint8_t frame[ByteFrame::getMaxEncodedSize<Crc>(MAX_PAYLOAD)] = {};
  const size_t frame_size = ByteFrame::encode<Crc>(payload, payload_size, frame, sizeof(frame));

  uint8_t out[MAX_PAYLOAD] = {};
  const size_t out_size    = ByteFrame::decode<Crc>(frame, frame_size, out, sizeof(out));

  Serial.print("- ");
  Serial.print(name);
  Serial.print(": CRC ");
  Serial.print(size_t(Crc::SIZE));
  Serial.print(" byte(s), frame ");
  Serial.print(frame_size);
  Serial.print(" bytes, round-trip ");
  Serial.println(out_size == payload_size ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("--------------------------------");
  Serial.println("ByteFrame - CrcSelection Example");
  Serial.println("--------------------------------");

  // Zeros and 0xFF are fine in the payload: COBS handles them regardless of the CRC
  const uint8_t payload[] = {0x10, 0x00, 0x20, 0xFF, 0x30};

  Serial.print("Payload: ");
  Serial.print(sizeof(payload));
  Serial.println(" bytes. Same payload through every policy:");

  showPolicy<ByteFrame::NoCrc>("NoCrc", payload, sizeof(payload));
  showPolicy<ByteFrame::Crc8Smbus>("Crc8Smbus", payload, sizeof(payload));
  showPolicy<ByteFrame::Crc16CcittFalse>("Crc16CcittFalse", payload, sizeof(payload));
  showPolicy<ByteFrame::Crc32IsoHdlc>("Crc32IsoHdlc", payload, sizeof(payload));
  Serial.println();

  // The CRC is part of the contract. Encode with one policy, decode with another: the CRC bytes do
  // not line up, so the frame is rejected (returns 0)
  using TxCrc = ByteFrame::Crc16CcittFalse;

  uint8_t frame[ByteFrame::getMaxEncodedSize<TxCrc>(MAX_PAYLOAD)] = {};
  const size_t frame_size =
    ByteFrame::encode<TxCrc>(payload, sizeof(payload), frame, sizeof(frame));

  uint8_t out[MAX_PAYLOAD] = {};
  const size_t matched     = ByteFrame::decode<TxCrc>(frame, frame_size, out, sizeof(out));
  const size_t mismatch =
    ByteFrame::decode<ByteFrame::Crc32IsoHdlc>(frame, frame_size, out, sizeof(out));

  Serial.println("Frame encoded with Crc16CcittFalse:");
  Serial.print("- decoded with the matching policy: ");
  Serial.print(matched);
  Serial.println(" bytes");
  Serial.print("- decoded with Crc32IsoHdlc: ");
  Serial.print(mismatch);
  Serial.println(" bytes (0 = rejected)");
}

void loop() {}
