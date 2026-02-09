# lepton_rs

![Crates.io](https://img.shields.io/crates/v/lepton_rs)
![Docs.rs](https://docs.rs/lepton_rs/badge.svg)

`lepton_rs` is a Rust library providing a device driver for the Lepton thermal camera over I2C (CCI) + SPI (VoSPI).

## Features

- Interface with Lepton thermal cameras
- Backward-compatible `read_frame()` API (legacy 60x164 packet stream)
- Robust Lepton 3.x/3.5 VoSPI capture path (`read_frame_robust` / `read_frame_robust_into`)
- Optional packet CRC validation and bounded auto-resync
- Packet/segment sequencing validation with diagnostics counters
- Optional capture timestamp/tick injection via `read_frame_robust_into_with_ticks`

## Lepton 3.5 robust acquisition notes

Lepton 3.x/3.5 sends frames as 4 segments × 60 lines over VoSPI (160x120 total).
The robust API:


`RobustCaptureConfig` currently models telemetry-disabled Lepton 3.x/3.5 packets by default
(164-byte packets with 4-byte headers and 160-byte payload). If your firmware output
format differs, adjust `packet_size_bytes`, `lines_per_segment`, and `segments_per_frame`
accordingly.

- Rejects discard packets
- Validates line ordering and segment progression
- Optionally validates packet CRC
- Applies bounded retries/resync
- Returns metadata + diagnostics for capture health

```rust
use lepton_rs::lepton::Lepton;
use lepton_rs::vospi::RobustCaptureConfig;

// after building Lepton::new(...)
let mut cfg = RobustCaptureConfig::default();
cfg.enable_crc = true;
cfg.max_frame_retries = 4;
lepton.set_robust_config(cfg);

let frame = lepton.read_frame_with_meta()?;
assert!(frame.meta.valid);
// frame.pixels is 4 * 60 * 160 = 38400 payload bytes (header stripped)
```

## Migration snippet

```rust
// old
let legacy = lepton.read_frame()?;

// new robust path
let robust = lepton.read_frame_robust()?;
let pixels = robust.pixels;
let meta = robust.meta;
```


## Legacy vs robust output format

- `read_frame()` returns legacy raw VoSPI packets (60 x 164 bytes).
- `read_frame_robust()` returns payload-only image bytes for Lepton 3.x/3.5 (4 x 60 x 160).
- `read_frame_robust_into(&mut [u8])` avoids per-frame allocation and leaves `capture_ticks` as `0`.
- `read_frame_robust_into_with_ticks(&mut [u8], now_ticks)` captures into a caller buffer and stamps metadata with your monotonic tick source.

## Shared SPI bus guidance (Lepton + other sensors)

Lepton VoSPI is timing sensitive. When sharing SPI (for example with MAX31865):

- Hold your SPI lock/mutex for a full frame capture whenever possible.
- Prefer a continuous capture task/thread for Lepton.
- Poll slower peripherals between Lepton captures.
- Use `read_frame_robust_locked` with your external lock/mutex pattern.

Example shape:

```rust
// pseudo-code with a user-owned mutex around Lepton access
let frame = lepton.read_frame_robust_locked(|cam| cam.read_frame_robust())?;
```

For example usage, see: ![esp_ir](https://github.com/KennethPrice288/esp_ir)

Documentation: docs.rs.
Contributions are welcome.
License: MIT.
