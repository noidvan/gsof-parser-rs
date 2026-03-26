# gsof-parser-rs

A `no_std`-compatible Rust parser for Trimble GSOF binary data, based on
Trimble's reference implementation (`gsofParser.c`, 2005–2011).

GSOF data is carried inside **Trimcomm** packets (framed with `0x02`/`0x03`
delimiters). Large messages are split across multiple packets and must be
reassembled before parsing individual GSOF record types.

## Crate layout

| Module | Responsibility |
|---|---|
| `trimcomm` | Packet framing — `FrameParser` state machine + optional `read_packet` wrapper |
| `reassembly` | Multi-page message accumulation into a 2 048-byte fixed buffer |
| `gsof` | Per-record-type decoders (position, velocity, SV info, attitude, …) |

## Feature flags

| Feature | Default | Effect |
|---|---|---|
| `std` | yes | Enables `thiserror` for `Display`/`Error` impls, adds `FrameError::Io`, `From<io::Error>`, and the blocking `read_packet` wrapper |

Build without `std` for bare-metal targets:

```toml
[dependencies]
gsof_parser = { version = "0.1", default-features = false }
```

---

## Embedded usage — Embassy + UART (serial-first)

The typical Pixhawk/autopilot wiring runs the Trimble receiver over a UART.
Embassy's async UART driver delivers bytes as they arrive from DMA; the
`FrameParser` state machine consumes them one at a time with no blocking and
no heap allocation.

```toml
# Cargo.toml (embedded target, e.g. STM32H7)
[dependencies]
gsof_parser    = { version = "0.1", default-features = false }
embassy-stm32  = { version = "0.1", features = ["stm32h743zi", "time-driver-any"] }
embassy-executor = { version = "0.6", features = ["arch-cortex-m"] }
embedded-io-async = "0.6"
```

```rust
use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embedded_io_async::Read;

use gsof_parser::trimcomm::{FrameParser, FrameError, TYPE_GSOF};
use gsof_parser::reassembly::Reassembler;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::task]
async fn gsof_task(mut uart: Uart<'static, peripherals::USART3, peripherals::DMA1_CH0, peripherals::DMA1_CH1>) {
    let mut parser     = FrameParser::new();
    let mut reassembler = Reassembler::new();
    let mut buf        = [0u8; 1];

    loop {
        // Yield to the executor until one byte arrives from DMA.
        if uart.read(&mut buf).await.is_err() {
            // Transport error — reset framer and keep going.
            parser = FrameParser::new();
            continue;
        }

        match parser.push(buf[0]) {
            // Frame still in progress; fetch the next byte.
            None => {}

            // Framing error (bad ETX or checksum) — parser already reset.
            Some(Err(e)) => {
                defmt::warn!("Trimcomm framing error: {:?}", defmt::Debug2Format(&e));
            }

            Some(Ok(pkt)) => {
                if pkt.packet_type != TYPE_GSOF {
                    continue;
                }

                match reassembler.push(&pkt.data) {
                    Err(e) => defmt::warn!("Reassembly error: {:?}", defmt::Debug2Format(&e)),
                    Ok(push) => {
                        if let Some(Ok(records)) = push.records {
                            for record in &records {
                                // Hand off to your application logic:
                                handle_gsof_record(record);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let uart = Uart::new(
        p.USART3,
        p.PD9,  // RX
        p.PD8,  // TX
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        Config::default(),
    ).unwrap();

    spawner.spawn(gsof_task(uart)).unwrap();
}
```

### Why byte-at-a-time?

`FrameParser` advances a state machine on each call to `push`. The
`FrameState::ReadingData` arm is the hot path; it runs once per data byte and
does a single `heapless::Vec::push` (infallible — capacity 255 matches the
protocol's u8 `LENGTH` field) plus an integer compare. The parser never blocks
and never allocates, which means:

- It can be called directly from a UART ISR or DMA-complete callback if needed.
- The `await` on `uart.read` releases the executor between bytes, keeping other
  tasks schedulable during long GPS updates.
- TCP and UDP paths (see below) use the identical `push` loop.

---

## Embedded usage — Embassy + TCP (Ethernet)

Some installations route the Trimble receiver over a TCP connection (e.g.
through a network bridge). The framing logic is unchanged; only the byte
source differs.

```rust
use embassy_net::tcp::TcpSocket;
use embedded_io_async::Read;

use gsof_parser::trimcomm::{FrameParser, TYPE_GSOF};
use gsof_parser::reassembly::Reassembler;

#[embassy_executor::task]
async fn gsof_tcp_task(mut socket: TcpSocket<'static>) {
    let mut parser      = FrameParser::new();
    let mut reassembler = Reassembler::new();
    let mut buf         = [0u8; 64]; // read in small chunks for efficiency

    'outer: loop {
        let n = match socket.read(&mut buf).await {
            Ok(0) | Err(_) => break 'outer, // EOF or error — reconnect upstream
            Ok(n) => n,
        };

        // Feed all received bytes through the same state machine.
        for &byte in &buf[..n] {
            match parser.push(byte) {
                None => {}
                Some(Err(e)) => {
                    defmt::warn!("Framing error: {:?}", defmt::Debug2Format(&e));
                }
                Some(Ok(pkt)) if pkt.packet_type == TYPE_GSOF => {
                    if let Ok(push) = reassembler.push(&pkt.data) {
                        if let Some(Ok(records)) = push.records {
                            for record in &records {
                                handle_gsof_record(record);
                            }
                        }
                    }
                }
                Some(Ok(_)) => {} // Non-GSOF packet type — ignore
            }
        }
    }
}
```

TCP delivers data in segments rather than byte-by-byte, so reading a small
buffer and iterating over it amortises the `async` overhead without
complicating the framing logic. The `FrameParser` is agnostic to delivery
granularity — it only cares about byte order.

### UDP note

UDP datagrams may be silently dropped or reordered. If the deployment uses
UDP (e.g. a local network bridge that multicast-forwards GSOF), the framing
layer handles misaligned streams by discarding bytes until the next STX.
However, GSOF multi-page reassembly (`Reassembler`) assumes in-order delivery
of pages within a message; dropped datagrams can leave `Reassembler` in a
partial state until the next page-0 packet resets it.

---

## std / desktop usage

The `read_packet` convenience function drives `FrameParser` from any
`io::Read` source — a serial port, a binary file, or a TCP stream:

```rust
use std::io::BufReader;
use gsof_parser::trimcomm::{read_packet, FrameError, TYPE_GSOF};
use gsof_parser::reassembly::Reassembler;

fn main() {
    let stdin = std::io::stdin();
    let mut reader = BufReader::new(stdin.lock());
    let mut reassembler = Reassembler::new();

    loop {
        match read_packet(&mut reader, |b| eprintln!("Skipping 0x{b:02X}")) {
            Ok(None) => break,                          // clean EOF
            Err(FrameError::Io(e))
                if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(e) => eprintln!("Framing error: {e}"),
            Ok(Some(pkt)) if pkt.packet_type == TYPE_GSOF => {
                if let Ok(push) = reassembler.push(&pkt.data) {
                    if let Some(Ok(records)) = push.records {
                        for rec in &records { println!("{rec}"); }
                    }
                }
            }
            Ok(Some(_)) => {} // non-GSOF packet
        }
    }
}
```

`read_packet` wraps `FrameParser` with `EINTR` retry and EOF detection.
The `skipped` callback receives each byte discarded before the first STX,
reproducing the "Skipping %02X" diagnostic from the original C tool.

---

## Memory budget

All allocations are static / stack-resident:

| Structure | Size |
|---|---|
| `FrameParser` | 1 (state) + 3 (stat/type/len) + 255 (data buf) ≈ **260 B** |
| `Reassembler` | 2 048 (payload buf) + 8 (header + len) ≈ **2 056 B** |
| `heapless::Vec<GsofRecord, 32>` | bounded by the largest `GsofRecord` variant × 32 |

No heap allocator required. The 2 048-byte reassembly buffer matches the
static array in the original C code (`unsigned char gsofData[2048]`).

---

## Testing

### Unit tests

Hand-built byte sequences for each GSOF record type:

```sh
cargo test --test gsof_tests
```

### Snapshot (golden record) tests

End-to-end tests that feed real receiver pcap captures through the full
pipeline (`FrameParser` → `Reassembler` → `parse_gsof_payload` → `Display`)
and compare against checked-in `.expected` files.

The fixture data comes from the
[trimble_driver_ros](https://github.com/trimble-navigation/trimble_driver_ros)
test suite — 30 pcap captures from five Trimble receivers (APPlus20, APPlus60,
APX-18, SPS986, LVX) covering types 1–3, 6–12, 15–16, 27, 33–35, 37–38,
40–41, 49–50, 52, including multi-message and multi-page reassembly cases.

#### Prerequisites

- **tshark** (part of Wireshark): used to extract TCP payloads from pcaps.
- **git-lfs**: the pcap files in trimble_driver_ros are stored with Git LFS.

```sh
# Debian/Ubuntu
sudo apt install tshark git-lfs
```

#### Pulling the fixture data

The pcaps live in a separate repository. Clone it (or ensure your existing
checkout has LFS objects pulled):

```sh
# If you already have the repo:
cd /path/to/trimble_driver_ros
git lfs install
git lfs pull

# If cloning fresh:
git lfs install
git clone https://github.com/trimble-navigation/trimble_driver_ros.git
```

Then extract raw Trimcomm byte streams from the pcaps:

```sh
# From the gsof-parser-rs root:
./tests/extract_fixtures.sh /path/to/trimble_driver_ros/trimble_driver/test/data
```

This produces `tests/fixtures/*.bin` files (one per pcap).

#### Running the snapshot tests

```sh
cargo test --test snapshot_tests
```

Each test reads a `.bin` fixture, runs it through the full parser, and
compares the output line-by-line against the corresponding `.expected` file.
On failure, a diff is printed showing exactly which lines changed.

#### Updating expected output after intentional changes

After changing `Display` formatting or fixing a parser bug, regenerate the
golden files:

```sh
GSOF_BLESS=1 cargo test --test snapshot_tests
```

Review the updated `.expected` files with `git diff` before committing.
