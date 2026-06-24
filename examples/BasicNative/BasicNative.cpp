/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * Basic Native Example Overview:
 * - Mirrors the Basic Arduino example, swapping Serial for printf. Useful for native builds.
 * - Build and run locally:
 *   PowerShell: $env:EXAMPLE="examples/BasicNative"; pio run -e native-example -t exec
 *   bash/WSL  : export EXAMPLE="examples/BasicNative"; pio run -e native-example -t exec
 */

#include <cstdio>

#include <ByteFrame.h>

// Maximum payload this link accepts; both the TX buffer and the Decoder derive from it
constexpr size_t MAX_PAYLOAD = 32;

ByteFrame::Decoder<MAX_PAYLOAD> decoder;

static void printHex(const uint8_t* data, const size_t len) {
  for (size_t i = 0; i < len; ++i)
    printf("%02X ", data[i]);
  printf("\n");
}

int main() {
  printf("--------------------------------\n");
  printf("ByteFrame - Basic Native Example\n");
  printf("--------------------------------\n");

  // Any bytes work as payload: a BytePack message, a string, a raw struct...
  const uint8_t payload[] = {'H', 'i', ' ', 0x00, 0x42, 0xFF}; // zeros are fine: COBS handles them

  // TX buffer sized at compile time for the worst case (CRC + COBS overhead + delimiter)
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  const size_t frame_size = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  if (frame_size == 0) {
    printf("Encoding failed: frame buffer too small\n");
    return 1;
  }

  printf("Payload (%zu bytes):\n", sizeof(payload));
  printHex(payload, sizeof(payload));
  printf("\n");

  printf("Encoded frame (%zu bytes, ends with the 0x00 delimiter):\n", frame_size);
  printHex(frame, frame_size);
  printf("\n");

  // Feed the frame byte by byte, as it would arrive from a UART
  for (size_t i = 0; i < frame_size; i++) {
    if (decoder.feed(frame[i])) {
      printf("Decoded payload (%zu bytes):\n", decoder.getPayloadSize());
      printHex(decoder.getPayload(), decoder.getPayloadSize());
      printf("\n");
    }
  }

  // Corrupt one byte: the CRC rejects the frame and the counter registers it
  frame[2] ^= 0x55;

  for (size_t i = 0; i < frame_size; i++) {
    decoder.feed(frame[i]);
  }

  const auto& stats = decoder.getStats();

  printf("After feeding a corrupted copy of the frame:\n");
  printf("- Frame available: %s\n", decoder.isFrameAvailable() ? "yes" : "no");
  printf("- Frames OK: %u\n", stats.frames_ok);
  printf("- CRC errors: %u\n", stats.crc_errors);
  printf("- Malformed errors: %u\n", stats.malformed);
  printf("- Overflow errors: %u\n", stats.overflows);

  return 0;
}
