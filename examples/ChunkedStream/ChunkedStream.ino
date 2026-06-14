/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * ChunkedStream Example Overview:
 * - Packs several frames back to back into one buffer, the way bytes pile up in a UART/DMA/TCP
 *   receive buffer, and corrupts one of them.
 * - Drains the buffer with the chunked feed(data, size) overload, which stops after each complete
 *   frame so none is lost when several share a chunk.
 * - Shows automatic resynchronization: the corrupted frame is dropped and counted, the frames after
 *   it still decode. Prints the per-cause Stats at the end.
 */

#include <Arduino.h>

#include <ByteFrame.h>

constexpr size_t MAX_PAYLOAD = 32;

ByteFrame::Decoder<MAX_PAYLOAD> decoder;

uint8_t stream[256] = {};
size_t stream_len   = 0;

void appendFrame(const uint8_t* payload, const size_t payload_size) {
  stream_len +=
    ByteFrame::encode(payload, payload_size, stream + stream_len, sizeof(stream) - stream_len);
}

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
  Serial.println("ByteFrame - ChunkedStream Example");
  Serial.println("---------------------------------");

  // Three frames concatenated into one receive buffer, exactly as they would arrive on the wire
  const uint8_t a[] = {'A', 'A', 'A'};
  const uint8_t b[] = {'B', 0x00, 'B'};
  const uint8_t c[] = {'C', 'C', 'C', 'C'};

  appendFrame(a, sizeof(a));
  const size_t b_start = stream_len;
  appendFrame(b, sizeof(b));
  appendFrame(c, sizeof(c));

  // Corrupt a data byte inside the middle frame. Keeping it non-zero avoids creating a false 0x00
  // delimiter, so only frame B's CRC fails - the decoder still resynchronizes for frame C
  stream[b_start + 1] += 1;
  if (stream[b_start + 1] == ByteFrame::DELIMITER) stream[b_start + 1] = 1;

  Serial.print("Receive buffer holds 3 frames (");
  Serial.print(stream_len);
  Serial.println(" bytes), the middle one corrupted:");
  printHex(stream, stream_len);
  Serial.println();

  // Drain the whole buffer: feed() returns after each complete frame, so the loop keeps going until
  // every byte is consumed
  size_t consumed = 0;
  while (consumed < stream_len) {
    consumed += decoder.feed(stream + consumed, stream_len - consumed);
    if (decoder.isFrameAvailable()) {
      Serial.print("Decoded frame: ");
      printHex(decoder.getPayload(), decoder.getPayloadSize());
    }
  }
  Serial.println();

  const auto& stats = decoder.getStats();
  Serial.println("Stats after draining the buffer:");
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
