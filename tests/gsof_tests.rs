/// Integration and unit tests for the GSOF parser.
///
/// Every test uses known-good byte sequences built by hand from the
/// GSOF ICD, so the tests are self-contained and require no real
/// receiver data.  The original C code had no tests at all.
use gsof_parser::gsof::{
    parse_gsof_payload, parse_gsof_record, AllDetailedSvInfo, AttitudeInfo, BriefSvInfo,
    GsofRecord, LatLonHeight, LbandStatus, ParseError, PdopInfo, PositionTime, SvDetailedInfo,
    UtcTime, Velocity,
};

// ---------------------------------------------------------------------------
// Helper: build big-endian byte sequences inline
// ---------------------------------------------------------------------------

fn be_u16(v: u16) -> [u8; 2] {
    v.to_be_bytes()
}
fn be_i16(v: i16) -> [u8; 2] {
    v.to_be_bytes()
}
fn be_u32(v: u32) -> [u8; 4] {
    v.to_be_bytes()
}
fn be_f32(v: f32) -> [u8; 4] {
    v.to_be_bytes()
}
fn be_f64(v: f64) -> [u8; 8] {
    v.to_be_bytes()
}

// ---------------------------------------------------------------------------
// Type 1 — PositionTime
// ---------------------------------------------------------------------------

#[test]
fn position_time_round_trip() {
    // ms=386400000 (week offset), week=2300, svs=9, f1=0x47, f2=0x00, init=3
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(386_400_000));
    buf.extend_from_slice(&be_u16(2300));
    buf.push(9); // num_svs
    buf.push(0x47); // flags1
    buf.push(0x00); // flags2
    buf.push(3); // init_number

    let rec = PositionTime::parse(&buf).unwrap();
    assert_eq!(rec.milliseconds, 386_400_000);
    assert_eq!(rec.week_number, 2300);
    assert_eq!(rec.num_svs, 9);
    assert_eq!(rec.flags1, 0x47);
    assert_eq!(rec.flags2, 0x00);
    assert_eq!(rec.init_number, 3);
}

#[test]
fn position_time_too_short_returns_error() {
    let buf = [0u8; 5]; // needs 10
    let err = PositionTime::parse(&buf).unwrap_err();
    assert!(matches!(
        err,
        ParseError::UnexpectedEof { record_type: 1, .. }
    ));
}

// ---------------------------------------------------------------------------
// Type 2 — LatLonHeight
// ---------------------------------------------------------------------------

#[test]
fn lat_lon_height_round_trip() {
    const PI: f64 = core::f64::consts::PI;
    let lat_rad: f64 = 43.7_f64.to_radians();
    let lon_rad: f64 = (-79.4_f64).to_radians();
    let h: f64 = 174.5;

    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f64(lat_rad));
    buf.extend_from_slice(&be_f64(lon_rad));
    buf.extend_from_slice(&be_f64(h));

    let rec = LatLonHeight::parse(&buf).unwrap();
    // Tolerate floating-point round-trip to 7 decimal places (≈11 mm)
    assert!((rec.lat_deg - 43.7_f64).abs() < 1e-7, "lat={}", rec.lat_deg);
    assert!(
        (rec.lon_deg - (-79.4_f64)).abs() < 1e-7,
        "lon={}",
        rec.lon_deg
    );
    assert!((rec.height_m - h).abs() < 1e-9);
    let _ = PI; // used indirectly via to_radians
}

#[test]
fn lat_lon_height_too_short_returns_error() {
    let buf = [0u8; 20]; // needs 24
    assert!(matches!(
        LatLonHeight::parse(&buf).unwrap_err(),
        ParseError::UnexpectedEof { record_type: 2, .. }
    ));
}

// ---------------------------------------------------------------------------
// Type 8 — Velocity
// ---------------------------------------------------------------------------

#[test]
fn velocity_round_trip() {
    let heading_rad: f32 = 1.5_f32; // ~85.9°
    let mut buf = Vec::new();
    buf.push(0x03); // flags
    buf.extend_from_slice(&be_f32(12.5_f32)); // velocity m/s
    buf.extend_from_slice(&be_f32(heading_rad));
    buf.extend_from_slice(&be_f32(-0.1_f32)); // vertical

    let rec = Velocity::parse(&buf).unwrap();
    assert_eq!(rec.flags, 0x03);
    assert!((rec.velocity_ms - 12.5_f32).abs() < 1e-5);
    let expected_deg = heading_rad * (180.0 / core::f32::consts::PI);
    assert!((rec.heading_deg - expected_deg).abs() < 1e-4);
    assert!((rec.vertical_ms - (-0.1_f32)).abs() < 1e-5);
}

// ---------------------------------------------------------------------------
// Type 9 — PdopInfo
// ---------------------------------------------------------------------------

#[test]
fn pdop_round_trip() {
    let mut buf = Vec::new();
    for v in [1.8_f32, 1.1_f32, 1.4_f32, 0.9_f32] {
        buf.extend_from_slice(&be_f32(v));
    }
    let rec = PdopInfo::parse(&buf).unwrap();
    assert!((rec.pdop - 1.8_f32).abs() < 1e-6);
    assert!((rec.hdop - 1.1_f32).abs() < 1e-6);
    assert!((rec.vdop - 1.4_f32).abs() < 1e-6);
    assert!((rec.tdop - 0.9_f32).abs() < 1e-6);
}

// ---------------------------------------------------------------------------
// Type 13 — BriefSvInfo
// ---------------------------------------------------------------------------

#[test]
fn brief_sv_info_three_svs() {
    let buf = [
        3, // count
        5, 0xC0, 0x00, // sv 1
        7, 0xC0, 0x00, // sv 2
        9, 0x40, 0x01, // sv 3
    ];
    let rec = BriefSvInfo::parse(&buf).unwrap();
    assert_eq!(rec.svs.len(), 3);
    assert_eq!(rec.svs[2].prn, 9);
    assert_eq!(rec.svs[2].flags2, 0x01);
}

#[test]
fn brief_sv_info_count_overflow_error() {
    // count=10 but only 6 bytes follow — 10×3 = 30 > 6
    let buf = [10u8, 1, 2, 3, 4, 5, 6];
    assert!(matches!(
        BriefSvInfo::parse(&buf).unwrap_err(),
        ParseError::OverflowingCount {
            record_type: 13,
            ..
        }
    ));
}

// ---------------------------------------------------------------------------
// Type 14 — SvDetailedInfo
// ---------------------------------------------------------------------------

#[test]
fn sv_detailed_info_one_sv() {
    let mut buf = vec![1u8]; // count = 1
    buf.push(12); // prn
    buf.push(0xC0); // flags1
    buf.push(0x00); // flags2
    buf.push(45); // elevation
    buf.extend_from_slice(&be_u16(135)); // azimuth
    buf.push(148); // l1_snr  → 37.0 dB-Hz
    buf.push(120); // l2_snr  → 30.0 dB-Hz

    let rec = SvDetailedInfo::parse(&buf).unwrap();
    assert_eq!(rec.svs.len(), 1);
    assert_eq!(rec.svs[0].prn, 12);
    assert_eq!(rec.svs[0].elevation_deg, 45);
    assert_eq!(rec.svs[0].azimuth_deg, 135);
    assert!((rec.svs[0].l1_snr as f64 / 4.0 - 37.0).abs() < 0.01);
}

// ---------------------------------------------------------------------------
// Type 16 — UtcTime
// ---------------------------------------------------------------------------

#[test]
fn utc_time_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(432_000_000)); // ms
    buf.extend_from_slice(&be_u16(2300)); // week
    buf.extend_from_slice(&be_i16(18)); // UTC offset
    buf.push(0x0F); // flags

    let rec = UtcTime::parse(&buf).unwrap();
    assert_eq!(rec.milliseconds, 432_000_000);
    assert_eq!(rec.week_number, 2300);
    assert_eq!(rec.utc_offset_s, 18);
    assert_eq!(rec.flags, 0x0F);
}

// ---------------------------------------------------------------------------
// Type 27 — AttitudeInfo (base record, no extended variance)
// ---------------------------------------------------------------------------

#[test]
fn attitude_info_base_record() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(432_000_000)); // ms (÷1000 → 432000.0 s)
    buf.push(0x03); // flags
    buf.push(6); // num_svs
    buf.push(1); // mode
    buf.push(0); // reserved

    let pitch_rad: f64 = 0.1_f64;
    let yaw_rad: f64 = 1.2_f64;
    let roll_rad: f64 = -0.05_f64;
    buf.extend_from_slice(&be_f64(pitch_rad));
    buf.extend_from_slice(&be_f64(yaw_rad));
    buf.extend_from_slice(&be_f64(roll_rad));
    buf.extend_from_slice(&be_f64(1.23)); // range
    buf.extend_from_slice(&be_u16(15)); // pdop ×10 = 1.5

    // Ensure no extended fields — length is exactly 42
    assert_eq!(buf.len(), 42);

    let rec = AttitudeInfo::parse(&buf).unwrap();
    assert_eq!(rec.num_svs, 6);
    assert!(rec.variance.is_none());
    let tol = 1e-10;
    assert!((rec.pitch_deg - pitch_rad.to_degrees()).abs() < tol);
    assert!((rec.yaw_deg - yaw_rad.to_degrees()).abs() < tol);
    assert!((rec.pdop - 1.5).abs() < 1e-9);
}

#[test]
fn attitude_info_extended_variance() {
    // Build the base 42-byte record first
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(0));
    buf.push(0);
    buf.push(4);
    buf.push(0);
    buf.push(0); // flags/svs/mode/reserved
    for _ in 0..4 {
        buf.extend_from_slice(&be_f64(0.0));
    } // pitch/yaw/roll/range
    buf.extend_from_slice(&be_u16(10)); // pdop
    assert_eq!(buf.len(), 42);

    // Append variance block (7 × f32 = 28 bytes)
    for v in [
        1e-4_f32, 2e-4_f32, 3e-4_f32, 1e-5_f32, 2e-5_f32, 3e-5_f32, 0.01_f32,
    ] {
        buf.extend_from_slice(&be_f32(v));
    }
    assert_eq!(buf.len(), 70);

    let rec = AttitudeInfo::parse(&buf).unwrap();
    let var = rec.variance.expect("should have variance block");
    assert!((var.pitch_var_rad2 - 1e-4_f32).abs() < 1e-8);
    assert!((var.range_var_m2 - 0.01_f32).abs() < 1e-7);
}

// ---------------------------------------------------------------------------
// Type 40 — LbandStatus (base, no measured-freq extension)
// ---------------------------------------------------------------------------

#[test]
fn lband_status_base() {
    let mut buf = Vec::new();
    buf.extend_from_slice(b"OMNI\0"); // name (5 bytes)
    buf.extend_from_slice(&be_f32(1539.83e6_f32)); // freq
    buf.extend_from_slice(&be_u16(600)); // bit_rate
    buf.extend_from_slice(&be_f32(38.5_f32)); // snr
    buf.push(1);
    buf.push(2);
    buf.push(1);
    buf.push(0);
    buf.push(0);
    buf.extend_from_slice(&be_f32(0.04_f32)); // horiz_prec_thresh
    buf.extend_from_slice(&be_f32(0.08_f32)); // vert_prec_thresh
    buf.push(0); // nmea_encryption
    buf.extend_from_slice(&be_f32(1.1_f32)); // iq_ratio
    buf.extend_from_slice(&be_f32(1e-6_f32)); // est_ber
    for v in [1000u32, 2u32, 0u32, 5000u32, 1u32, 0u32] {
        buf.extend_from_slice(&be_u32(v));
    }
    // 61 bytes total (no extension)
    assert_eq!(buf.len(), 61);

    let rec = LbandStatus::parse(&buf).unwrap();
    assert_eq!(rec.name_str(), "OMNI\0");
    assert_eq!(rec.bit_rate_bps, 600);
    assert!(rec.measured_freq.is_none());
}

// ---------------------------------------------------------------------------
// Type 34 — AllDetailedSvInfo (multi-constellation)
// ---------------------------------------------------------------------------

#[test]
fn all_detailed_sv_info_mixed_constellation() {
    let mut buf = vec![2u8]; // 2 SVs

    // SV 1: GPS PRN 5
    buf.push(5); // prn
    buf.push(0); // system = GPS
    buf.push(0xC0);
    buf.push(0x00); // flags
    buf.push(60); // elevation
    buf.extend_from_slice(&be_u16(200)); // azimuth
    buf.push(160);
    buf.push(140);
    buf.push(100); // snr[3]

    // SV 2: GALILEO PRN 2
    buf.push(2); // prn
    buf.push(3); // system = GALILEO
    buf.push(0xC0);
    buf.push(0x00);
    buf.push(35);
    buf.extend_from_slice(&be_u16(90));
    buf.push(120);
    buf.push(0);
    buf.push(100);

    let rec = AllDetailedSvInfo::parse(&buf).unwrap();
    assert_eq!(rec.svs.len(), 2);
    assert_eq!(rec.svs[0].system, 0); // GPS
    assert_eq!(rec.svs[1].system, 3); // GALILEO
    assert_eq!(rec.svs[1].prn, 2);
}

// ---------------------------------------------------------------------------
// parse_gsof_payload — walking a concatenated payload
// ---------------------------------------------------------------------------

#[test]
fn parse_payload_two_records_sequential() {
    // Build a payload with a Type-1 record followed by a Type-9 record.
    let mut pt_data = Vec::new();
    pt_data.extend_from_slice(&be_u32(1_000_000));
    pt_data.extend_from_slice(&be_u16(2300));
    pt_data.push(8);
    pt_data.push(0x47);
    pt_data.push(0x00);
    pt_data.push(1);

    let mut pdop_data = Vec::new();
    for v in [2.1_f32, 1.5_f32, 1.7_f32, 1.0_f32] {
        pdop_data.extend_from_slice(&be_f32(v));
    }

    let mut payload = Vec::new();
    payload.push(1u8); // type
    payload.push(pt_data.len() as u8); // length
    payload.extend_from_slice(&pt_data);
    payload.push(9u8);
    payload.push(pdop_data.len() as u8);
    payload.extend_from_slice(&pdop_data);

    let records = parse_gsof_payload(&payload).unwrap();
    assert_eq!(records.len(), 2);
    assert!(matches!(records[0], GsofRecord::PositionTime(_)));
    assert!(matches!(records[1], GsofRecord::PdopInfo(_)));
}

#[test]
fn parse_payload_truncated_header_returns_error() {
    // Only one byte — too short for even the (type, length) header pair.
    let payload = [42u8];
    assert!(parse_gsof_payload(&payload).is_err());
}

#[test]
fn parse_payload_length_extends_past_end_returns_error() {
    // type=1, length=20, but only 5 data bytes follow.
    let payload = [1u8, 20, 0, 0, 0, 0, 0];
    assert!(parse_gsof_payload(&payload).is_err());
}

#[test]
fn parse_record_unknown_type_returns_unknown_variant() {
    let data = [0xAA, 0xBB, 0xCC];
    let rec = parse_gsof_record(0xFE, &data);
    assert!(matches!(
        rec,
        GsofRecord::Unknown {
            gsof_type: 0xFE,
            ..
        }
    ));
}

// ---------------------------------------------------------------------------
// Display smoke test — just ensure it doesn't panic
// ---------------------------------------------------------------------------

#[test]
fn display_position_time_does_not_panic() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(123_456));
    buf.extend_from_slice(&be_u16(2200));
    buf.push(12);
    buf.push(0x47);
    buf.push(0x00);
    buf.push(0);
    let rec = PositionTime::parse(&buf).unwrap();
    let s = format!("{rec}");
    assert!(s.contains("PositionTime"));
    assert!(s.contains("123456"));
}
