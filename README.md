<h1 align="center">
  <a><img src=".img/logo.svg" alt="Logo" width="350"></a>
  <br>
  ByteFrame
</h1>

<p align="center">
  <b>Header-only frame delimiting library for C++11 with zero dynamic memory allocation.</b>
</p>

<p align="center">
  <a href="https://www.ardu-badge.com/ByteFrame">
    <img src="https://www.ardu-badge.com/badge/ByteFrame.svg?" alt="Arduino Library Badge">
  </a>
  <a href="https://registry.platformio.org/libraries/alkonosst/ByteFrame">
    <img src="https://badges.registry.platformio.org/packages/alkonosst/library/ByteFrame.svg" alt="PlatformIO Registry">
  </a>
  <br><br>
  <a href="https://codecov.io/github/alkonosst/ByteFrame">
    <img src="https://img.shields.io/codecov/c/github/alkonosst/ByteFrame?style=for-the-badge&logo=codecov&logoColor=white&labelColor=F01F7A" alt="Coverage">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/license-MIT-blue.svg?style=for-the-badge&color=blue" alt="License">
  </a>
  <br><br>
  <a href="https://ko-fi.com/alkonosst">
    <img src="https://ko-fi.com/img/githubbutton_sm.svg" alt="Ko-fi">
  </a>
</p>

---

# Table of contents <!-- omit in toc -->

- [Description](#description)
- [Key Features](#key-features)
- [Quick Example](#quick-example)
- [Installation](#installation)
  - [PlatformIO](#platformio)
  - [Arduino IDE](#arduino-ide)
  - [CMake](#cmake)
- [Usage](#usage)
  - [Including the library](#including-the-library)
  - [Namespace](#namespace)
  - [Frame Format](#frame-format)
  - [Encoding Frames](#encoding-frames)
  - [Choosing the CRC](#choosing-the-crc)
  - [Compile-Time Size Budgets](#compile-time-size-budgets)
  - [Decoding a Stream](#decoding-a-stream)
  - [Feeding in Chunks](#feeding-in-chunks)
  - [Decoding a Single Frame](#decoding-a-single-frame)
  - [Statistics](#statistics)
  - [Using with BytePack](#using-with-bytepack)
- [Release Status](#release-status)
- [License](#license)

---

# Description

**ByteFrame** is a header-only C++11 embedded/native library that delimits packets over raw byte streams (UART, RS-485, radios, raw TCP...). It answers the question a stream cannot: _where does each packet start and end, and did it arrive intact?_

Each frame is the payload plus a selectable CRC, encoded with [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) and terminated by a single `0x00` delimiter. COBS guarantees the delimiter never appears inside a frame, so the decoder can always resynchronize after corruption or after joining a stream mid-frame. The payload is just bytes: ByteFrame does not care if it is a [BytePack](https://github.com/alkonosst/BytePack) message, a protobuf, or a raw struct.

All buffers are caller-provided or statically sized; there is no dynamic memory allocation.

# Key Features

- **Header-only** - A single `#include <ByteFrame.h>`; no source files, no dependencies.
- **Zero dynamic allocation** - The encoder writes into a caller-provided buffer; the decoder uses a fixed internal buffer sized by a template parameter.
- **Self-delimiting frames** - COBS encoding plus a `0x00` delimiter: frames can be concatenated freely and boundaries are always recoverable.
- **Bounded, predictable overhead** - COBS adds at most 1 byte per 254 bytes of data; the worst case is known at compile time via `getMaxEncodedSize()`.
- **Selectable integrity check** - A CRC over the payload travels inside every frame; corrupted frames are dropped, never delivered. Pick `NoCrc`, `Crc8Smbus`, `Crc16CcittFalse` (default) or `Crc32IsoHdlc`, or plug in your own.
- **Incremental decoder** - Feed bytes one at a time or in chunks, straight from `Serial.read()`, an ISR ring buffer or a DMA block; no intermediate buffer needed.
- **Automatic resynchronization** - Every delimiter resets the parser: corruption, truncated frames and mid-stream joins cost at most one dropped frame.
- **Per-cause diagnostics** - Separate counters for CRC mismatches, malformed frames and oversized frames make link problems visible.
- **Payload-agnostic** - Frames carry raw bytes; pairs naturally with [BytePack](https://github.com/alkonosst/BytePack) for type-safe payloads (see [Using with BytePack](#using-with-bytepack)).

# Quick Example

```cpp
#include <Arduino.h>

#include <ByteFrame.h>

constexpr size_t MAX_PAYLOAD = 32;

ByteFrame::Decoder<MAX_PAYLOAD> decoder;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // the link both devices share

  // Frame buffer sized at compile time: payload + CRC + COBS overhead + delimiter
  uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

  // Encode and send (e.g. on the transmitting device)
  const uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  const size_t frame_size  = ByteFrame::encode(payload, sizeof(payload), frame, sizeof(frame));
  if (frame_size > 0) {
    Serial1.write(frame, frame_size);
  }
}

void loop() {
  // Receive and decode (e.g. on the receiving device)
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (decoder.feed(b)) {
      Serial.print("Got a frame of ");
      Serial.print(decoder.getPayloadSize());
      Serial.println(" bytes");
    }
  }
}
```

# Installation

## PlatformIO

Add to your `platformio.ini`:

```ini
[env:your_env]
; Most recent changes
lib_deps =
  https://github.com/alkonosst/ByteFrame.git

; Pinned release (recommended for production)
lib_deps =
  https://github.com/alkonosst/ByteFrame.git#vx.y.z
```

## Arduino IDE

1. Open Arduino IDE.
2. Go to **Sketch > Manage Libraries...**
3. Search for **"ByteFrame"**.
4. Click **Install**.

## CMake

For desktop C++ projects, pull the library with `FetchContent` and link the `alkonosst::ByteFrame`
target:

```cmake
include(FetchContent)
FetchContent_Declare(
  ByteFrame
  GIT_REPOSITORY https://github.com/alkonosst/ByteFrame.git
  GIT_TAG        vx.y.z # pin a release tag (recommended), or a branch/commit
)
FetchContent_MakeAvailable(ByteFrame)

target_link_libraries(your_app PRIVATE alkonosst::ByteFrame)
```

# Usage

## Including the library

A single header includes everything:

```cpp
#include <ByteFrame.h>
```

## Namespace

All public types live in the `ByteFrame` namespace:

```cpp
using namespace ByteFrame;
```

## Frame Format

On the wire, a frame is:

```
COBS( [payload bytes][CRC little-endian] ) + [0x00 delimiter]
```

- The **CRC** is computed over the payload and appended little-endian. It is selectable per endpoint (see [Choosing the CRC](#choosing-the-crc)); the default is **CRC-16/CCITT-FALSE** (polynomial `0x1021`, initial value `0xFFFF`). With `NoCrc` the frame carries only the payload.
- **COBS** (Consistent Overhead Byte Stuffing) re-encodes payload + CRC so that no `0x00` byte remains, at a cost of at most 1 extra byte per 254 bytes of data.
- A single **`0x00` delimiter** terminates the frame. Since it cannot appear inside the encoded data, scanning for it always finds a frame boundary.

The format is easy to implement on non-C++ peers (Python scripts, PC tools, etc.): COBS and the standard CRCs are widely available in every language.

> [!NOTE]
> The payload must be at least 1 byte. Empty frames (consecutive delimiters) are ignored by the decoder, so idle `0x00` bytes can be used as a keep-alive or line flush without side effects.

## Encoding Frames

`encode()` builds the complete frame into a caller-provided buffer and returns its size, or `0` if the payload is empty or the frame did not fit:

```cpp
uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

const size_t frame_size = ByteFrame::encode(payload, payload_size, frame, sizeof(frame));
if (frame_size == 0) {
  // Frame buffer too small (or empty payload): nothing partial was written
}

link.write(frame, frame_size); // frames can be sent back to back on the same stream
```

When the payload is not contiguous - a header followed by a body, or several fields from different places - the streaming `Encoder` writes COBS and CRC directly into the output buffer as you `feed()` it, with no intermediate buffer to concatenate the parts first. Call `finalize()` to append the CRC and the delimiter; it returns the frame size, or `0` on overflow:

```cpp
uint8_t frame[ByteFrame::getMaxEncodedSize(sizeof(Header) + MAX_BODY)] = {};
ByteFrame::Encoder<> encoder(frame, sizeof(frame)); // <> selects the default CRC

encoder.feed(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
encoder.feed(body, body_size);

const size_t frame_size = encoder.finalize();
if (frame_size > 0) {
  link.write(frame, frame_size);
}

encoder.restart(); // reuse the same buffer for the next frame
```

The `Encoder` is bound to its output buffer at construction (`Encoder<Crc> encoder(frame, frame_size)`); its methods are:

| Method             | Description                                                                     |
| ------------------ | ------------------------------------------------------------------------------- |
| `feed(byte)`       | Feed one payload byte; returns `true` while the encoder is healthy.             |
| `feed(data, size)` | Feed a chunk of payload bytes; returns `true` while the encoder is healthy.     |
| `finalize()`       | Append the CRC and delimiter; returns the frame size, or `0` on empty/overflow. |
| `restart()`        | Discard progress and start a new frame in the same buffer.                      |
| `isOk()`           | `true` while the encoder is still healthy (no buffer overflow yet).             |

`encode()` is just a one-shot wrapper over `Encoder` for a single contiguous payload, so both
produce byte-identical frames.

## Choosing the CRC

The integrity check is a template parameter shared by `Encoder`, `Decoder`, `encode()`, `decode()` and `getMaxEncodedSize()`. It defaults to `Crc16CcittFalse`, so the common case needs no type argument. Both endpoints must agree on the same policy.

| Policy            | Width   | Standard / check value of `"123456789"` |
| ----------------- | ------- | --------------------------------------- |
| `NoCrc`           | 0 bytes | no integrity check                      |
| `Crc8Smbus`       | 1 byte  | CRC-8/SMBUS, `0xF4`                     |
| `Crc16CcittFalse` | 2 bytes | CRC-16/CCITT-FALSE, `0x29B1` (default)  |
| `Crc32IsoHdlc`    | 4 bytes | CRC-32/ISO-HDLC (zlib), `0xCBF43926`    |

```cpp
// Default CRC (CRC-16/CCITT-FALSE): no type argument anywhere
ByteFrame::Decoder<64> decoder;
ByteFrame::encode(payload, payload_size, frame, sizeof(frame));

// A heavier CRC for a noisier link: pass the same policy everywhere
using Crc = ByteFrame::Crc32IsoHdlc;
uint8_t frame[ByteFrame::getMaxEncodedSize<Crc>(MAX_PAYLOAD)] = {};
ByteFrame::Decoder<64, Crc> decoder32;
ByteFrame::encode<Crc>(payload, payload_size, frame, sizeof(frame));
```

To use a CRC that is not provided, define a type with the same shape (`value_type`, `SIZE`, `init()`, `update()`, `finalize()`) and pass it as the policy.

## Compile-Time Size Budgets

`getMaxEncodedSize()` is `constexpr` and returns the worst-case frame size for a given payload size, accounting for the CRC, the COBS overhead and the delimiter. Use it to size transmit buffers exactly and to enforce transport budgets at compile time:

```cpp
// Buffer sized exactly for the worst case
uint8_t frame[ByteFrame::getMaxEncodedSize(MAX_PAYLOAD)] = {};

// If the frame outgrows the link MTU, the firmware stops compiling
static_assert(ByteFrame::getMaxEncodedSize(MAX_PAYLOAD) <= LINK_MTU,
  "Frame does not fit in the link MTU");
```

## Decoding a Stream

`Decoder<MaxPayload>` is an incremental parser with a fixed internal buffer: feed it raw stream bytes and it reports when a complete, CRC-valid frame arrived. `MaxPayload` is the largest payload it accepts; bigger frames are dropped and counted:

```cpp
ByteFrame::Decoder<64> decoder;

void loop() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (decoder.feed(b)) {
      handlePayload(decoder.getPayload(), decoder.getPayloadSize());
    }
  }
}
```

The decoded payload is valid until the next call to `feed()` or `reset()`: consume it (or copy it out) before feeding more bytes.

Invalid input never produces a frame and never breaks the parser: every `0x00` delimiter resynchronizes it, so corruption, truncated frames or joining a stream mid-frame cost at most one dropped frame.

| Method                | Description                                                                         |
| --------------------- | ----------------------------------------------------------------------------------- |
| `feed(byte)`          | Feed one byte; returns `true` when it completes a valid frame.                      |
| `feed(data, size)`    | Feed a chunk; returns bytes consumed (see [Feeding in Chunks](#feeding-in-chunks)). |
| `isFrameAvailable()`  | `true` if the last `feed()` produced a complete valid frame.                        |
| `getPayload()`        | Read-only pointer to the decoded payload.                                           |
| `getPayloadSize()`    | Decoded payload size, or `0` if no frame is available.                              |
| `getMaxPayloadSize()` | The `MaxPayload` template parameter (`constexpr`).                                  |
| `reset()`             | Discard any partial frame; counters are preserved.                                  |
| `getStats()`          | Diagnostic counters by cause (see [Statistics](#statistics)).                       |
| `resetStats()`        | Zero the counters; the parse state is left untouched.                               |

## Feeding in Chunks

When bytes arrive in blocks (chunked `Serial.read()`, DMA buffers, TCP segments), the chunked `feed()` overload consumes bytes until the first complete frame and returns how many it took, so no frame is lost when several share a chunk:

```cpp
uint8_t chunk[128];
const size_t received = link.read(chunk, sizeof(chunk));

size_t consumed = 0;
while (consumed < received) {
  consumed += decoder.feed(chunk + consumed, received - consumed);
  if (decoder.isFrameAvailable()) {
    handlePayload(decoder.getPayload(), decoder.getPayloadSize());
  }
}
```

## Decoding a Single Frame

When a whole frame already sits in a buffer (not arriving as a stream), `decode()` is the one-shot counterpart to `encode()`: it COBS-decodes the frame, verifies the CRC and writes the payload into a caller-provided buffer, returning the payload size or `0` on failure. Only the payload is written out (the CRC is validated and discarded), so the buffer just needs to hold the payload. COBS decoding stops at the first `0x00`, so the trailing delimiter is optional.

```cpp
uint8_t payload[MAX_PAYLOAD] = {};

const size_t payload_size = ByteFrame::decode(frame, frame_size, payload, sizeof(payload));
if (payload_size > 0) {
  handlePayload(payload, payload_size);
}
```

`decode()` takes the same CRC policy as `encode()` (see [Choosing the CRC](#choosing-the-crc)). The streaming `Decoder` remains the right tool when frames arrive incrementally or when you want the per-cause [statistics](#statistics).

## Statistics

`getStats()` returns a `Stats` struct that tracks decoded frames and drops by cause, which makes link problems diagnosable without a logic analyzer:

```cpp
const auto& stats = decoder.getStats();
stats.frames_ok;  // complete, CRC-valid frames decoded
stats.crc_errors; // frames with corrupted content (CRC mismatch)
stats.malformed;  // truncated/invalid COBS data, frames too short
stats.overflows;  // frames whose payload exceeds MaxPayload

decoder.resetStats(); // zero the counters (the parse state is left untouched)
```

A healthy link keeps the error counts at zero while `frames_ok` climbs. Rising CRC errors point to electrical noise or baud rate mismatch; malformed errors to framing/sync issues; overflows to a `MaxPayload` smaller than what the peer sends. Joining an already-active stream typically counts a single error for the partial first frame: that is expected.

## Using with BytePack

ByteFrame is payload-agnostic, but it pairs naturally with [BytePack](https://github.com/alkonosst/BytePack): BytePack gives the payload a type-safe layout, ByteFrame moves it safely over a stream. The compile-time size helpers of both libraries compose, so every buffer is sized exactly:

```cpp
#include <ByteFrame.h>
#include <BytePack.h>

// Define a message as a BytePack struct with Header fields for dispatching
struct Telemetry {
  static constexpr uint8_t ID      = 0x01;
  static constexpr uint8_t VERSION = 1;

  uint32_t uptime_ms  = 0;
  int16_t temperature = 0;

  template <typename Archive>
  constexpr void io(Archive& ar) {
    ar(uptime_ms, temperature);
  }
};

// Worst-case frame for this message, fully computed at compile time
constexpr size_t MAX_PAYLOAD = BytePack::getMaxPackedSizeWithHeader<Telemetry>();
constexpr size_t MAX_FRAME   = ByteFrame::getMaxEncodedSize(MAX_PAYLOAD);

// Transmit: serialize, then frame
void sendTelemetry(const Telemetry& msg) {
  uint8_t payload[MAX_PAYLOAD] = {};
  uint8_t frame[MAX_FRAME]     = {};

  const size_t payload_size = BytePack::serializeWithHeader(msg, payload, sizeof(payload));
  const size_t frame_size   = ByteFrame::encode(payload, payload_size, frame, sizeof(frame));
  if (frame_size > 0) {
    Serial1.write(frame, frame_size);
  }
}

// Receive: deframe, then dispatch
ByteFrame::Decoder<MAX_PAYLOAD> decoder;

void loop() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (decoder.feed(b)) {
      BytePack::dispatch<Telemetry>(decoder.getPayload(), decoder.getPayloadSize(),
        BytePack::Overloaded{
          [](const Telemetry& msg) { /* handle it */ },
        });
    }
  }
}
```

# Release Status

This project is in active development. Until reaching version **v1.0.0**, consider it **beta software**. APIs may change in future releases, and some features may be incomplete or unstable. Please report any issues on the [GitHub Issues](https://github.com/alkonosst/ByteFrame/issues) page.

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
