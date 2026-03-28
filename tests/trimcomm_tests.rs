/// Integration tests for the Trimcomm frame parser.
use gsof_parser::trimcomm::{FrameError, FrameParser, ETX, STX};

/// Compute checksum as sum of STAT+TYPE+LENGTH+DATA, mod 256.
fn checksum(stat: u8, ptype: u8, data: &[u8]) -> u8 {
    let len = data.len() as u8;
    let mut sum: u8 = stat;
    sum = sum.wrapping_add(ptype);
    sum = sum.wrapping_add(len);
    for &b in data {
        sum = sum.wrapping_add(b);
    }
    sum
}

/// Build a valid Trimcomm packet as raw bytes.
fn build_packet(stat: u8, ptype: u8, data: &[u8]) -> Vec<u8> {
    let len = data.len() as u8;
    let csum = checksum(stat, ptype, data);
    let mut pkt = Vec::new();
    pkt.push(STX);
    pkt.push(stat);
    pkt.push(ptype);
    pkt.push(len);
    pkt.extend_from_slice(data);
    pkt.push(csum);
    pkt.push(ETX);
    pkt
}

// ---------------------------------------------------------------------------
// valid_packet_parses
// ---------------------------------------------------------------------------

#[test]
fn valid_packet_parses() {
    let data = [0xAA, 0xBB, 0xCC];
    let raw = build_packet(0x01, 0x40, &data);

    let mut parser = FrameParser::new();
    let mut result = None;
    for &byte in &raw {
        if let Some(r) = parser.push(byte) {
            result = Some(r);
        }
    }

    let pkt = result.expect("should have produced a result").expect("should be Ok");
    assert_eq!(pkt.stat, 0x01);
    assert_eq!(pkt.packet_type, 0x40);
    assert_eq!(pkt.data.as_slice(), &[0xAA, 0xBB, 0xCC]);
}

// ---------------------------------------------------------------------------
// bad_etx_returns_error
// ---------------------------------------------------------------------------

#[test]
fn bad_etx_returns_error() {
    let data = [0x11, 0x22];
    let mut raw = build_packet(0x00, 0x40, &data);
    // Corrupt the ETX byte (last byte)
    let last = raw.len() - 1;
    raw[last] = 0xFF;

    let mut parser = FrameParser::new();
    let mut result = None;
    for &byte in &raw {
        if let Some(r) = parser.push(byte) {
            result = Some(r);
        }
    }

    let err = result.expect("should have produced a result").unwrap_err();
    assert!(matches!(err, FrameError::BadEtx(0xFF)));
}

// ---------------------------------------------------------------------------
// bad_checksum_returns_error
// ---------------------------------------------------------------------------

#[test]
fn bad_checksum_returns_error() {
    let data = [0x11, 0x22];
    let mut raw = build_packet(0x00, 0x40, &data);
    // Corrupt the checksum byte (second-to-last)
    let csum_idx = raw.len() - 2;
    raw[csum_idx] = raw[csum_idx].wrapping_add(1);

    let mut parser = FrameParser::new();
    let mut result = None;
    for &byte in &raw {
        if let Some(r) = parser.push(byte) {
            result = Some(r);
        }
    }

    let err = result.expect("should have produced a result").unwrap_err();
    assert!(matches!(err, FrameError::BadChecksum { .. }));
}

// ---------------------------------------------------------------------------
// sync_recovery_after_garbage
// ---------------------------------------------------------------------------

#[test]
fn sync_recovery_after_garbage() {
    let data = [0xDE, 0xAD];
    let valid = build_packet(0x05, 0x40, &data);

    let mut stream = Vec::new();
    // Feed 10 garbage bytes first
    stream.extend_from_slice(&[0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8, 0xF7, 0xF6]);
    stream.extend_from_slice(&valid);

    let mut parser = FrameParser::new();
    let mut result = None;
    for &byte in &stream {
        if let Some(r) = parser.push(byte) {
            result = Some(r);
        }
    }

    let pkt = result.expect("should have produced a result").expect("should recover and parse");
    assert_eq!(pkt.stat, 0x05);
    assert_eq!(pkt.packet_type, 0x40);
    assert_eq!(pkt.data.as_slice(), &[0xDE, 0xAD]);
}
