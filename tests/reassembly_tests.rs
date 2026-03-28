/// Integration tests for the GSOF multi-packet reassembler.
use gsof_parser::gsof::GsofRecord;
use gsof_parser::reassembly::{Reassembler, ReassemblyError, GSOF_BUF_MAX};

fn be_u32(v: u32) -> [u8; 4] {
    v.to_be_bytes()
}
fn be_u16(v: u16) -> [u8; 2] {
    v.to_be_bytes()
}

/// Build a GSOF 0x40 payload with paging header + GSOF sub-records.
/// transmission_number, page_index, max_page_index are the 3-byte header.
fn build_paged_payload(
    tx_num: u8,
    page_idx: u8,
    max_page_idx: u8,
    gsof_data: &[u8],
) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(tx_num);
    payload.push(page_idx);
    payload.push(max_page_idx);
    payload.extend_from_slice(gsof_data);
    payload
}

/// Build a simple Type 1 (PositionTime) GSOF sub-record as type+length+data.
fn build_position_time_record() -> Vec<u8> {
    let mut rec = Vec::new();
    rec.push(1u8); // type
    rec.push(10u8); // length
    rec.extend_from_slice(&be_u32(1_000_000)); // ms
    rec.extend_from_slice(&be_u16(2300)); // week
    rec.push(8); // num_svs
    rec.push(0x47); // flags1
    rec.push(0x00); // flags2
    rec.push(1); // init_number
    rec
}

// ---------------------------------------------------------------------------
// single_page_message
// ---------------------------------------------------------------------------

#[test]
fn single_page_message() {
    let gsof_data = build_position_time_record();
    let payload = build_paged_payload(0, 0, 0, &gsof_data);

    let mut reassembler = Reassembler::new();
    let result = reassembler.push(&payload).unwrap();

    assert_eq!(result.header.page_index, 0);
    assert_eq!(result.header.max_page_index, 0);

    let records = result.records.expect("should have records on single page");
    let records = records.expect("parsing should succeed");
    assert_eq!(records.len(), 1);
    assert!(matches!(records[0], GsofRecord::PositionTime(_)));
}

// ---------------------------------------------------------------------------
// multi_page_message
// ---------------------------------------------------------------------------

#[test]
fn multi_page_message() {
    let gsof_data = build_position_time_record();

    // Split the data across two pages: first 6 bytes on page 0, rest on page 1
    let split = 6;
    let page0_data = &gsof_data[..split];
    let page1_data = &gsof_data[split..];

    let payload0 = build_paged_payload(42, 0, 1, page0_data);
    let payload1 = build_paged_payload(42, 1, 1, page1_data);

    let mut reassembler = Reassembler::new();

    // First page: no records yet
    let result0 = reassembler.push(&payload0).unwrap();
    assert!(result0.records.is_none());

    // Second page: records returned
    let result1 = reassembler.push(&payload1).unwrap();
    let records = result1.records.expect("should have records after last page");
    let records = records.expect("parsing should succeed");
    assert_eq!(records.len(), 1);
    assert!(matches!(records[0], GsofRecord::PositionTime(_)));
}

// ---------------------------------------------------------------------------
// buffer_overflow
// ---------------------------------------------------------------------------

#[test]
fn buffer_overflow() {
    // Create a payload that would exceed GSOF_BUF_MAX
    let big_data = vec![0u8; GSOF_BUF_MAX + 1];
    let payload = build_paged_payload(1, 0, 0, &big_data);

    let mut reassembler = Reassembler::new();
    let result = reassembler.push(&payload);
    assert!(result.is_err());
    match result {
        Err(ReassemblyError::BufferOverflow { .. }) => {} // expected
        other => panic!("expected BufferOverflow, got {:?}", other.is_ok()),
    }
}

// ---------------------------------------------------------------------------
// page_restart
// ---------------------------------------------------------------------------

#[test]
fn page_restart() {
    let gsof_data = build_position_time_record();

    // Start a multi-page sequence (tx_num=10, page 0 of 1)
    let partial_payload = build_paged_payload(10, 0, 1, &gsof_data[..6]);

    let mut reassembler = Reassembler::new();
    let result0 = reassembler.push(&partial_payload).unwrap();
    assert!(result0.records.is_none());

    // Now a new single-page message arrives (different tx_num=20), causing reset
    let fresh = build_paged_payload(20, 0, 0, &gsof_data);
    let result1 = reassembler.push(&fresh).unwrap();

    let records = result1.records.expect("should have records");
    let records = records.expect("parsing should succeed");
    assert_eq!(records.len(), 1);
    assert!(matches!(records[0], GsofRecord::PositionTime(_)));
}
