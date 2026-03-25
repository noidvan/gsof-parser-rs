/// gsof_parser — Rust rewrite of Trimble's gsofParser.c
///
/// Reads raw Trimcomm/GSOF binary from stdin, a TCP connection, or a serial
/// port, and prints decoded GSOF records to stdout.
///
/// Usage:
///   gsof_parser                              # stdin (pipe or redirect)
///   gsof_parser --tcp <host:port>            # TCP connection
///   gsof_parser --serial <device>            # serial @ 115200 baud
///   gsof_parser --serial <device> --baud <N> # serial @ custom baud
///
/// Exit codes:
///   0  — clean EOF / connection closed
///   1  — bad arguments or transport open failure
mod gsof;
mod reassembly;
mod trimcomm;

use reassembly::Reassembler;
use std::io::{self, BufReader};
use std::net::TcpStream;
use trimcomm::{read_packet, FrameError, TYPE_GSOF};

// ---------------------------------------------------------------------------
// Core processing loop — generic over any io::Read source
// ---------------------------------------------------------------------------

fn run(reader: &mut impl io::Read) {
    let mut reassembler = Reassembler::new();

    loop {
        match read_packet(reader, |b| eprintln!("  Skipping 0x{b:02X}")) {
            Ok(None) | Err(FrameError::Io(..)) => {
                eprintln!("END OF FILE");
                std::process::exit(0);
            }
            Err(e) => {
                // Log framing errors and keep going — mirrors the original C
                // code's note: "not designed to handle corrupted data".
                eprintln!("Framing error: {e}");
            }
            Ok(Some(pkt)) => {
                println!(
                    "STX:02h  Stat:{:02X}h  Type:{:02X}h  Len:{}  CS:??h  ETX:03h",
                    pkt.stat, pkt.packet_type, pkt.data.len(),
                );

                if pkt.packet_type != TYPE_GSOF {
                    continue;
                }

                match reassembler.push(&pkt.data) {
                    Err(e) => eprintln!("Reassembly error: {e}"),
                    Ok(push) => {
                        println!(
                            "  GSOF packet: Trans#:{}  Page:{} MaxPage:{}",
                            push.header.transmission_number,
                            push.header.page_index,
                            push.header.max_page_index,
                        );
                        if let Some(result) = push.records {
                            match result {
                                Err(e) => eprintln!("Parse error: {e}"),
                                Ok(records) => {
                                    println!("\nGSOF Records");
                                    for rec in &records {
                                        println!("{rec}\n");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Argument parsing and source dispatch
// ---------------------------------------------------------------------------

fn usage() -> ! {
    eprintln!("Usage:");
    eprintln!("  gsof_parser                              # stdin");
    eprintln!("  gsof_parser --tcp <host:port>            # TCP");
    eprintln!("  gsof_parser --serial <device>            # serial @ 115200 baud");
    eprintln!("  gsof_parser --serial <device> --baud <N> # serial @ custom baud");
    std::process::exit(1);
}

fn die(msg: impl std::fmt::Display) -> ! {
    eprintln!("Error: {msg}");
    std::process::exit(1);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str>   = args.iter().map(String::as_str).collect();

    eprintln!("GSOF Parser (Rust)");

    match args.as_slice() {
        // ── stdin ──────────────────────────────────────────────────────────
        [] => {
            eprintln!("Source: stdin");
            run(&mut BufReader::new(io::stdin().lock()));
        }

        // ── TCP ────────────────────────────────────────────────────────────
        ["--tcp", addr] => {
            eprintln!("Source: TCP {addr}");
            let stream = TcpStream::connect(addr)
                .unwrap_or_else(|e| die(format_args!("TCP connect to {addr} failed: {e}")));
            run(&mut BufReader::new(stream));
        }

        // ── serial (default baud) ──────────────────────────────────────────
        ["--serial", device] => open_serial(device, 115_200),

        // ── serial (explicit baud) ─────────────────────────────────────────
        ["--serial", device, "--baud", baud] => {
            let baud: u32 = baud.parse()
                .unwrap_or_else(|_| die(format_args!("Invalid baud rate: {baud}")));
            open_serial(device, baud);
        }

        _ => usage(),
    }
}

// ---------------------------------------------------------------------------
// Serial helper — compiled only when the `serial` feature is enabled
// ---------------------------------------------------------------------------

#[cfg(feature = "serial")]
fn open_serial(device: &str, baud: u32) {
    use std::time::Duration;
    eprintln!("Source: serial {device} @ {baud} baud");
    let port = serialport::new(device, baud)
        .timeout(Duration::from_secs(10))
        .open()
        .unwrap_or_else(|e| die(format_args!("Failed to open {device}: {e}")));
    run(&mut BufReader::new(port));
}

#[cfg(not(feature = "serial"))]
fn open_serial(device: &str, _baud: u32) -> ! {
    die(format_args!(
        "Serial support not compiled in (rebuild with --features serial). Device: {device}"
    ))
}
