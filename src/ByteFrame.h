/**
 * SPDX-FileCopyrightText: 2026 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ByteFrame {

// MARK: Constants

/**
 * @brief Byte that terminates every frame on the wire (0x00). COBS encoding guarantees it never
 * appears inside a frame, so it is always safe to resynchronize by searching for it.
 */
constexpr uint8_t DELIMITER = 0x00;

/** @brief Maximum number of data bytes a single COBS block can carry (code byte 0xFF). */
constexpr size_t COBS_MAX_BLOCK = 254;

// MARK: CRC policies

/**
 * @brief CRC policy used by Encoder, Decoder, encode(), decode() and getMaxEncodedSize() to select
 * the integrity check. Every policy is a type exposing its byte width (`SIZE`), a `value_type`, and
 * the three steps of an incremental CRC: `init()`, `update()` and `finalize()`. To support a CRC
 * that is not provided here, define your own type with the same shape. `Crc16CcittFalse` is the
 * default everywhere.
 */

/** @brief No integrity check: the frame carries only the COBS-encoded payload (SIZE == 0). */
struct NoCrc {
  typedef uint8_t value_type;
  static const size_t SIZE = 0;
  static value_type init() { return 0; }
  static value_type update(const value_type crc, const uint8_t) { return crc; }
  static value_type finalize(const value_type crc) { return crc; }
};

/** @brief CRC-8/SMBUS. Check value: crc("123456789") == 0xF4. */
struct Crc8Smbus {
  typedef uint8_t value_type;
  static const size_t SIZE = 1;
  static value_type init() { return 0x00; }
  // Polynomial 0x07, no reflection, no final XOR
  static value_type update(value_type crc, const uint8_t byte) {
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? value_type((crc << 1) ^ 0x07) : value_type(crc << 1);
    }
    return crc;
  }
  static value_type finalize(const value_type crc) { return crc; }
};

/** @brief CRC-16/CCITT-FALSE (the default). Check value: crc("123456789") == 0x29B1. */
struct Crc16CcittFalse {
  typedef uint16_t value_type;
  static const size_t SIZE = 2;
  static value_type init() { return 0xFFFF; }
  // Polynomial 0x1021, no reflection, no final XOR
  static value_type update(value_type crc, const uint8_t byte) {
    crc ^= value_type(byte) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? value_type((crc << 1) ^ 0x1021) : value_type(crc << 1);
    }
    return crc;
  }
  static value_type finalize(const value_type crc) { return crc; }
};

/** @brief CRC-32/ISO-HDLC (zlib/Ethernet). Check value: crc("123456789") == 0xCBF43926. */
struct Crc32IsoHdlc {
  typedef uint32_t value_type;
  static const size_t SIZE = 4;
  static value_type init() { return 0xFFFFFFFFu; }
  // Reflected polynomial 0xEDB88320, input and output reflected, final XOR 0xFFFFFFFF
  static value_type update(value_type crc, const uint8_t byte) {
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return crc;
  }
  static value_type finalize(const value_type crc) { return crc ^ 0xFFFFFFFFu; }
};

// MARK: Encoder

/**
 * @brief Streaming frame encoder that writes COBS and CRC directly into a caller-provided buffer.
 *
 * Build a frame incrementally from several sources - for example a header followed by a body -
 * without an intermediate buffer to concatenate them first. Feed the payload with any number of
 * `feed()` calls, then `finalize()` to append the CRC and the trailing delimiter. Nothing is
 * allocated; the output goes straight into the buffer passed to the constructor, which can be sized
 * with `getMaxEncodedSize()`. For a single contiguous payload, prefer the `encode()` helper.
 *
 * Example:
 * ```cpp
 * uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(Header) + MAX_BODY)] = {};
 * ByteFrame::Encoder<> encoder(frame, sizeof(frame));
 *
 * encoder.feed(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
 * encoder.feed(body, body_size);
 *
 * const size_t frame_size = encoder.finalize();
 * if (frame_size > 0) {
 *   send(frame, frame_size);
 * }
 * ```
 *
 * @tparam Crc CRC policy appended by finalize(). Defaults to Crc16CcittFalse.
 */
template <class Crc = Crc16CcittFalse>
class Encoder {
  typedef typename Crc::value_type value_type;

  public:
  /**
   * @brief Bind the encoder to a destination buffer and start a new frame.
   * @param frame Destination buffer for the encoded frame.
   * @param frame_size Capacity of the destination buffer in bytes.
   */
  Encoder(uint8_t* frame, const size_t frame_size)
      : _frame(frame)
      , _frame_size(frame_size) {
    restart();
  }

  /** @brief Discard any progress and begin a new frame in the same buffer. */
  void restart() {
    _pos        = 1; // slot 0 is reserved for the code byte of the first block
    _code_pos   = 0;
    _code       = 1;
    _crc        = Crc::init();
    _block_full = false;
    _empty      = true;
    _error      = (_frame_size == 0);
  }

  /**
   * @brief Feed a single payload byte.
   * @param byte Payload byte to append.
   * @return `true` if the encoder is still healthy, `false` once the buffer has overflowed.
   */
  bool feed(const uint8_t byte) {
    _empty = false;
    _crc   = Crc::update(_crc, byte);
    push(byte);
    return !_error;
  }

  /**
   * @brief Feed a chunk of payload bytes.
   * @param data Payload bytes to append.
   * @param size Number of bytes in the chunk.
   * @return `true` if the encoder is still healthy, `false` once the buffer has overflowed.
   */
  bool feed(const uint8_t* data, const size_t size) {
    for (size_t i = 0; i < size; i++)
      feed(data[i]);
    return !_error;
  }

  /**
   * @brief Append the CRC, close the COBS encoding and write the trailing delimiter.
   * @return `size_t` Frame size in bytes (delimiter included), or 0 if no payload was fed or the
   * frame did not fit.
   */
  size_t finalize() {
    if (_empty) return 0;

    // Finalize the running CRC, then append it little-endian; the CRC bytes are not part of the CRC
    _crc = Crc::finalize(_crc);
    for (size_t i = 0; i != Crc::SIZE; i++) {
      push(uint8_t(_crc & 0xFF));
      _crc = value_type(_crc >> 8);
    }
    if (_error) return 0;

    // Write the code byte of the open block; a block closed at 0xFF already wrote its own
    if (!_block_full) _frame[_code_pos] = _code;

    if (_pos >= _frame_size) {
      _error = true;
      return 0;
    }
    _frame[_pos++] = DELIMITER;
    return _pos;
  }

  /**
   * @brief Check whether the encoder is still healthy (no buffer overflow yet).
   * @return `true` if healthy, `false` after an overflow.
   */
  bool isOk() const { return !_error; }

  private:
  // Append one COBS-encoded data byte, opening and closing blocks as needed.
  void push(const uint8_t byte) {
    if (_error) return;

    // A full (0xFF) block was closed without reserving a successor: do it now that more data came
    if (_block_full) {
      if (_pos >= _frame_size) {
        _error = true;
        return;
      }
      _code_pos   = _pos++;
      _code       = 1;
      _block_full = false;
    }

    if (byte == 0x00) {
      // The zero is represented by the current block's code byte; open a fresh block after it
      _frame[_code_pos] = _code;
      if (_pos >= _frame_size) {
        _error = true;
        return;
      }
      _code_pos = _pos++;
      _code     = 1;
      return;
    }

    if (_pos >= _frame_size) {
      _error = true;
      return;
    }
    _frame[_pos++] = byte;

    // A block carries at most 254 data bytes. Close it but defer reserving the successor, so a
    // frame ending right here needs no extra code byte (keeps the output within getMaxEncodedSize)
    if (++_code == 0xFF) {
      _frame[_code_pos] = _code;
      _code             = 1;
      _block_full       = true;
    }
  }

  uint8_t* _frame;
  size_t _frame_size;
  size_t _pos      = 1;
  size_t _code_pos = 0;
  uint8_t _code    = 1;
  value_type _crc  = Crc::init();
  bool _block_full = false;
  bool _empty      = true;
  bool _error      = false;
};

// MARK: Decoder

/**
 * @brief Incremental frame decoder with a fixed internal buffer and no dynamic allocation.
 *
 * Feed it raw stream bytes (from UART, a radio, a socket...) one at a time or in chunks; it decodes
 * COBS on the fly, detects frame boundaries and validates the CRC. When a complete valid frame
 * arrives, the decoded payload is exposed through `getPayload()` / `getPayloadSize()` and remains
 * valid until the next call to `feed()` or `reset()`.
 *
 * Invalid input never produces a frame: oversized frames, malformed COBS data and CRC mismatches
 * are silently dropped and counted in per-cause diagnostic counters. The decoder resynchronizes on
 * every delimiter, so it recovers from corruption or from joining a stream mid-frame.
 *
 * Example:
 * ```cpp
 * ByteFrame::Decoder<64> decoder; // accepts payloads up to 64 bytes
 *
 * void loop() {
 *   while (Serial.available()) {
 *     if (decoder.feed(uint8_t(Serial.read()))) {
 *       handlePayload(decoder.getPayload(), decoder.getPayloadSize());
 *     }
 *   }
 * }
 * ```
 *
 * @tparam MaxPayload Maximum accepted payload size in bytes (sizes the internal buffer).
 * @tparam Crc CRC policy validated on each frame; must match the encoder. Defaults to
 * Crc16CcittFalse.
 */
template <size_t MaxPayload, class Crc = Crc16CcittFalse>
class Decoder {
  static_assert(MaxPayload >= 1, "ByteFrame: Decoder MaxPayload must be at least 1.");

  typedef typename Crc::value_type crc_type;

  public:
  /**
   * @brief Diagnostic counters describing the health of the link. All cumulative since construction
   * (or the last `resetStats()`).
   */
  struct Stats {
    uint32_t frames_ok  = 0; // complete, CRC-valid frames decoded
    uint32_t crc_errors = 0; // frames dropped due to a CRC mismatch (corrupted content)
    uint32_t malformed = 0; // frames dropped due to invalid structure (truncated COBS or too short)
    uint32_t overflows = 0; // frames dropped because the payload exceeded MaxPayload
  };

  /**
   * @brief Feed a single stream byte to the decoder.
   * @param byte Raw byte from the stream.
   * @return `true` if this byte completed a valid frame (read it with `getPayload()` before the
   * next call to `feed()`), `false` otherwise.
   */
  bool feed(const uint8_t byte) {
    _frame_available = false;
    processByte(byte);
    return _frame_available;
  }

  /**
   * @brief Feed a chunk of stream bytes to the decoder, stopping after the first complete frame.
   *
   * Call it in a loop to drain a chunk that may contain multiple frames:
   * ```cpp
   * size_t consumed = 0;
   * while (consumed < size) {
   *   consumed += decoder.feed(data + consumed, size - consumed);
   *   if (decoder.isFrameAvailable()) {
   *     handlePayload(decoder.getPayload(), decoder.getPayloadSize());
   *   }
   * }
   * ```
   *
   * @param data Raw bytes from the stream.
   * @param size Number of bytes available.
   * @return `size_t` Number of bytes consumed: `size` if no frame completed, or the position right
   * after the delimiter that completed a frame (check `isFrameAvailable()`).
   */
  size_t feed(const uint8_t* data, const size_t size) {
    _frame_available = false;
    size_t consumed  = 0;
    while (consumed < size) {
      processByte(data[consumed++]);
      if (_frame_available) break;
    }
    return consumed;
  }

  /**
   * @brief Check if a complete valid frame is available from the last `feed()` call.
   * @return `true` if a decoded payload is available, `false` otherwise.
   */
  bool isFrameAvailable() const { return _frame_available; }

  /**
   * @brief Get a read-only pointer to the decoded payload of the last complete frame.
   * @return `const uint8_t*` Pointer to the payload bytes. Valid only while `isFrameAvailable()`
   * is `true`; the content is overwritten by subsequent `feed()` calls.
   */
  const uint8_t* getPayload() const { return _buffer; }

  /**
   * @brief Get the size of the decoded payload of the last complete frame.
   * @return `size_t` Payload size in bytes, or 0 if no frame is available.
   */
  size_t getPayloadSize() const { return _frame_available ? _payload_size : 0; }

  /**
   * @brief Get the maximum accepted payload size (the template parameter `MaxPayload`).
   * @return `constexpr size_t` Maximum payload size in bytes.
   */
  static constexpr size_t getMaxPayloadSize() { return MaxPayload; }

  /**
   * @brief Discard any partial frame and pending payload, returning the parser to a clean state.
   * The diagnostic counters are preserved.
   */
  void reset() {
    _frame_available = false;
    _payload_size    = 0;
    _decoded_size    = 0;
    _block_remaining = 0;
    _append_zero     = false;
    _discarding      = false;
  }

  /**
   * @brief Get the diagnostic counters of the decoder.
   * @return `const Stats&` Reference to the live counters.
   */
  const Stats& getStats() const { return _stats; }

  /** @brief Reset all diagnostic counters to zero. The parse state is left untouched. */
  void resetStats() { _stats = Stats{}; }

  private:
  uint8_t _buffer[MaxPayload + Crc::SIZE] = {};
  size_t _payload_size                    = 0;     // payload size of the last complete frame
  size_t _decoded_size                    = 0;     // decoded bytes of the frame being parsed
  size_t _block_remaining                 = 0;     // data bytes left in the current COBS block
  bool _append_zero                       = false; // append an implicit zero before the next block
  bool _discarding                        = false; // drop bytes until the next delimiter
  bool _frame_available                   = false;
  Stats _stats;

  void processByte(const uint8_t byte) {
    if (byte == DELIMITER) {
      endFrame();
      return;
    }

    if (_discarding) return;

    if (_block_remaining == 0) {
      // Code byte: starts a new block. The previous block (if not full) encoded a zero.
      if (_append_zero && !push(0x00)) return;
      _block_remaining = size_t(byte) - 1;
      _append_zero     = (byte != 0xFF);
    } else {
      if (push(byte)) _block_remaining--;
    }
  }

  bool push(const uint8_t byte) {
    if (_decoded_size >= sizeof(_buffer)) {
      _stats.overflows++;
      _discarding = true;
      return false;
    }
    _buffer[_decoded_size++] = byte;
    return true;
  }

  void endFrame() {
    const bool discarding = _discarding;
    const bool mid_block  = (_block_remaining != 0);
    const size_t decoded  = _decoded_size;

    // The delimiter always resynchronizes the parser, whatever came before it
    _decoded_size    = 0;
    _block_remaining = 0;
    _append_zero     = false;
    _discarding      = false;

    if (discarding) return;   // already counted when the overflow happened
    if (decoded == 0) return; // empty frame (idle delimiters): ignored, not an error

    if (mid_block || decoded < 1 + Crc::SIZE) {
      _stats.malformed++;
      return;
    }

    const size_t payload_size = decoded - Crc::SIZE;

    crc_type received = 0;
    for (size_t i = 0; i != Crc::SIZE; i++)
      received = crc_type(received | (crc_type(_buffer[payload_size + i]) << (8 * i)));

    crc_type crc = Crc::init();
    for (size_t i = 0; i < payload_size; i++)
      crc = Crc::update(crc, _buffer[i]);

    if (Crc::finalize(crc) != received) {
      _stats.crc_errors++;
      return;
    }

    _payload_size    = payload_size;
    _frame_available = true;
    _stats.frames_ok++;
  }
};

// MARK: Helpers

/**
 * @brief Get the worst-case encoded frame size for a given payload size, computed at compile time.
 *
 * Accounts for the CRC, the COBS overhead (1 byte per 254 bytes of data, rounded up) and the
 * trailing delimiter. Use it to size transmit buffers exactly:
 *
 * ```cpp
 * uint8_t tx_buffer[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};
 * ```
 *
 * @tparam Crc CRC policy whose width is included in the total. Defaults to Crc16CcittFalse.
 * @param payload_size Payload size in bytes.
 * @return `constexpr size_t` Worst-case frame size in bytes (CRC, COBS overhead and delimiter
 * included).
 */
template <class Crc = Crc16CcittFalse>
constexpr size_t getMaxEncodedSize(const size_t payload_size) {
  return payload_size + Crc::SIZE +
         (payload_size + Crc::SIZE + COBS_MAX_BLOCK - 1) / COBS_MAX_BLOCK + 1;
}

/**
 * @brief Encode a payload into a self-delimited frame: `COBS(payload + CRC) + delimiter`.
 *
 * One-shot convenience over `Encoder` for a single contiguous payload. The CRC (CRC-16/CCITT-FALSE
 * by default) is computed over the payload and appended little-endian before COBS encoding, so the
 * resulting frame contains no `0x00` byte except the final delimiter. The output buffer is
 * caller-provided; nothing is allocated.
 *
 * Example:
 * ```cpp
 * uint8_t frame[ByteFrame::getMaxEncodedSize(8)] = {};
 * const uint8_t payload[3] = {0x11, 0x22, 0x33};
 *
 * const size_t frame_size = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
 * if (frame_size > 0) {
 *   send(frame, frame_size); // safe to concatenate with other frames on the same stream
 * }
 * ```
 *
 * @tparam Crc CRC policy appended to the payload. Defaults to Crc16CcittFalse.
 * @param payload Payload bytes to frame.
 * @param payload_size Payload size in bytes (must be at least 1).
 * @param frame Destination buffer for the encoded frame.
 * @param frame_size Capacity of the destination buffer in bytes.
 * @return `size_t` Frame size in bytes (delimiter included), or 0 if the payload is empty or the
 * frame did not fit.
 */
template <class Crc = Crc16CcittFalse>
size_t encode(const uint8_t* payload, const size_t payload_size, uint8_t* frame,
  const size_t frame_size) {
  Encoder<Crc> encoder(frame, frame_size);
  encoder.feed(payload, payload_size);
  return encoder.finalize();
}

/**
 * @brief Decode a single self-delimited frame back into its payload, validating the CRC.
 *
 * One-shot counterpart to `encode()` for when a whole frame already sits in a buffer (as opposed to
 * the streaming `Decoder`). The frame is accepted exactly as `encode()` produced it: COBS decoding
 * stops at the first `0x00` delimiter, so a trailing delimiter is optional. Only the payload is
 * written out - the CRC is used to validate the frame and then discarded - so the output buffer
 * just needs to hold the payload (`MaxPayload` bytes). Nothing is allocated.
 *
 * Example:
 * ```cpp
 * uint8_t payload[MAX_PAYLOAD] = {};
 * const size_t payload_size = ByteFrame::decode(frame, frame_size, payload, sizeof(payload));
 * if (payload_size > 0) {
 *   handlePayload(payload, payload_size);
 * }
 * ```
 *
 * @tparam Crc CRC policy; must match the one used to encode the frame. Defaults to Crc16CcittFalse.
 * @param frame Encoded frame bytes (with or without the trailing delimiter).
 * @param frame_size Number of frame bytes available.
 * @param payload Destination buffer for the decoded payload.
 * @param payload_capacity Capacity of the destination buffer in bytes.
 * @return `size_t` Decoded payload size in bytes, or 0 on any failure (malformed COBS, CRC
 * mismatch, empty frame or insufficient capacity).
 */
template <class Crc = Crc16CcittFalse>
size_t decode(const uint8_t* frame, const size_t frame_size, uint8_t* payload,
  const size_t payload_capacity) {
  typedef typename Crc::value_type crc_type;

  // First pass: COBS-decode just to measure the decoded length (payload + CRC) and reject
  // structurally invalid input
  size_t total     = 0;
  bool append_zero = false; // a zero is implied between COBS blocks (but not at the frame end)
  for (size_t in = 0; in < frame_size;) {
    const uint8_t code = frame[in++];
    if (code == DELIMITER) break; // the delimiter closes the frame
    if (append_zero) total++;
    const size_t block = size_t(code) - 1;
    for (size_t i = 0; i < block; i++) {
      if (in >= frame_size) return 0;         // truncated COBS block
      if (frame[in++] == DELIMITER) return 0; // a zero may not appear inside a block
      total++;
    }
    append_zero = (code != 0xFF);
  }

  if (total < Crc::SIZE + 1) return 0; // need at least one payload byte plus the CRC
  const size_t payload_size = total - Crc::SIZE;
  if (payload_size > payload_capacity) return 0; // the payload does not fit

  // Second pass: COBS-decode again, writing only the payload and rebuilding the received CRC from
  // the trailing bytes, so the CRC never lands in the caller's buffer. The frame is already known
  // to be well formed, so no bounds or interior-zero checks are needed here.
  crc_type crc      = Crc::init();
  crc_type received = 0;
  size_t produced   = 0;

  append_zero = false;
  for (size_t in = 0; in < frame_size;) {
    const uint8_t code = frame[in++];
    if (code == DELIMITER) break;
    const size_t block = size_t(code) - 1;

    // One emit site: i == 0 is the implicit inter-block zero (only when append_zero), i >= 1 the
    // literal block bytes
    for (size_t i = append_zero ? 0 : 1; i <= block; i++) {
      const uint8_t byte = (i == 0) ? uint8_t(0x00) : frame[in++];
      if (produced < payload_size) {
        payload[produced] = byte;
        crc               = Crc::update(crc, byte);
      } else {
        received = crc_type(received | (crc_type(byte) << (8 * (produced - payload_size))));
      }
      produced++;
    }
    append_zero = (code != 0xFF);
  }

  if (Crc::finalize(crc) != received) return 0;

  return payload_size;
}

} // namespace ByteFrame
