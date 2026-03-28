/// Integration and unit tests for the GSOF parser.
///
/// Every test uses known-good byte sequences built by hand from the
/// GSOF ICD, so the tests are self-contained and require no real
/// receiver data.  The original C code had no tests at all.
use gsof_parser::gsof::{
    parse_gsof_payload, parse_gsof_record, AllBriefSvInfo, AllDetailedSvInfo, AttitudeInfo,
    BasePositionQuality, BatteryMemoryInfo, BriefSvInfo, ClockInfo, DmiRawData, Ecef, EcefDelta,
    GsofRecord, InsFullNav, InsRmsInfo, InsVnavFullNav, InsVnavRmsInfo, LatLonHeight, LbandStatus,
    ParseError, PdopInfo, PositionSigmaInfo, PositionTime, PositionTypeInfo, PositionVcvInfo,
    ReceivedBaseInfo, ReceiverSerialNumber, SvDetailedInfo, TangentPlaneDelta, UtcTime, Velocity,
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
fn be_i32(v: i32) -> [u8; 4] {
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

// ---------------------------------------------------------------------------
// Type 10 — ClockInfo
// ---------------------------------------------------------------------------

#[test]
fn clock_info_round_trip() {
    let mut buf = Vec::new();
    buf.push(0x03); // flags
    buf.extend_from_slice(&be_f64(1.234)); // clock offset ms
    buf.extend_from_slice(&be_f64(-0.567)); // freq offset ppm

    let rec = ClockInfo::parse(&buf).unwrap();
    assert_eq!(rec.clock_flags, 0x03);
    assert!((rec.clock_offset_ms - 1.234).abs() < 1e-10);
    assert!((rec.freq_offset_ppm - (-0.567)).abs() < 1e-10);
}

#[test]
fn clock_info_too_short() {
    let buf = [0u8; 10]; // needs 17
    assert!(matches!(
        ClockInfo::parse(&buf).unwrap_err(),
        ParseError::UnexpectedEof { record_type: 10, .. }
    ));
}

// ---------------------------------------------------------------------------
// Type 11 — PositionVcvInfo
// ---------------------------------------------------------------------------

#[test]
fn position_vcv_info_round_trip() {
    let mut buf = Vec::new();
    for v in [0.5_f32, 1.0, 0.1, 0.2, 2.0, 0.3, 3.0, 0.8] {
        buf.extend_from_slice(&be_f32(v));
    }
    buf.extend_from_slice(&be_u16(42));

    let rec = PositionVcvInfo::parse(&buf).unwrap();
    assert!((rec.position_rms - 0.5).abs() < 1e-6);
    assert!((rec.vcv_xx - 1.0).abs() < 1e-6);
    assert!((rec.vcv_zz - 3.0).abs() < 1e-6);
    assert_eq!(rec.num_epochs, 42);
}

// ---------------------------------------------------------------------------
// Type 12 — PositionSigmaInfo
// ---------------------------------------------------------------------------

#[test]
fn position_sigma_info_round_trip() {
    let mut buf = Vec::new();
    for v in [0.5_f32, 0.1, 0.2, 0.01, 0.3, 0.4, 0.15, 45.0, 0.9] {
        buf.extend_from_slice(&be_f32(v));
    }
    buf.extend_from_slice(&be_u16(100));

    let rec = PositionSigmaInfo::parse(&buf).unwrap();
    assert!((rec.position_rms - 0.5).abs() < 1e-6);
    assert!((rec.sigma_east - 0.1).abs() < 1e-6);
    assert!((rec.orientation_deg - 45.0).abs() < 1e-6);
    assert_eq!(rec.num_epochs, 100);
}

// ---------------------------------------------------------------------------
// Type 15 — ReceiverSerialNumber
// ---------------------------------------------------------------------------

#[test]
fn receiver_serial_number_round_trip() {
    let buf = be_i32(123456);
    let rec = ReceiverSerialNumber::parse(&buf).unwrap();
    assert_eq!(rec.serial_number, 123456);
}

#[test]
fn receiver_serial_number_negative() {
    let buf = be_i32(-1);
    let rec = ReceiverSerialNumber::parse(&buf).unwrap();
    assert_eq!(rec.serial_number, -1);
}

// ---------------------------------------------------------------------------
// Type 35 — ReceivedBaseInfo
// ---------------------------------------------------------------------------

#[test]
fn received_base_info_round_trip() {
    let mut buf = Vec::new();
    buf.push(0x08); // flags
    buf.extend_from_slice(b"BASE0001"); // name (8 bytes)
    buf.extend_from_slice(&be_u16(42)); // base_id
    let lat_rad = 43.7_f64.to_radians();
    let lon_rad = (-79.4_f64).to_radians();
    buf.extend_from_slice(&be_f64(lat_rad));
    buf.extend_from_slice(&be_f64(lon_rad));
    buf.extend_from_slice(&be_f64(174.5)); // height

    let rec = ReceivedBaseInfo::parse(&buf).unwrap();
    assert_eq!(rec.flags, 0x08);
    assert_eq!(rec.name_str(), "BASE0001");
    assert_eq!(rec.base_id, 42);
    assert!((rec.lat_deg - 43.7).abs() < 1e-7);
    assert!((rec.lon_deg - (-79.4)).abs() < 1e-7);
    assert!((rec.height_m - 174.5).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// Type 37 — BatteryMemoryInfo
// ---------------------------------------------------------------------------

#[test]
fn battery_memory_info_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(85)); // battery capacity
    buf.extend_from_slice(&be_f64(3600.0)); // remaining time

    let rec = BatteryMemoryInfo::parse(&buf).unwrap();
    assert_eq!(rec.battery_capacity, 85);
    assert!((rec.remaining_time - 3600.0).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// Type 38 — PositionTypeInfo (variable length)
// ---------------------------------------------------------------------------

#[test]
fn position_type_info_minimal() {
    // Only error_scale (4 bytes)
    let buf = be_f32(1.5);
    let rec = PositionTypeInfo::parse(&buf).unwrap();
    assert!((rec.error_scale - 1.5).abs() < 1e-6);
    assert!(rec.ext_440.is_none());
    assert!(rec.ext_482.is_none());
    assert!(rec.ext_490.is_none());
    assert!(rec.position_fix_type.is_none());
}

#[test]
fn position_type_info_with_440() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f32(1.5)); // error_scale
    buf.push(0x03); // solution_flags
    buf.push(0); // rtk_condition
    buf.extend_from_slice(&be_f32(2.5)); // correction_age
    buf.push(0x40); // network_flags
    buf.push(0x01); // network_flags2
    assert_eq!(buf.len(), 12);

    let rec = PositionTypeInfo::parse(&buf).unwrap();
    let ext = rec.ext_440.unwrap();
    assert_eq!(ext.solution_flags, 0x03);
    assert!((ext.correction_age - 2.5).abs() < 1e-6);
    assert!(rec.ext_482.is_none());
}

#[test]
fn position_type_info_full() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f32(1.5)); // error_scale (4)
    buf.push(0x03);
    buf.push(0);
    buf.extend_from_slice(&be_f32(2.5));
    buf.push(0x40);
    buf.push(0x01); // ext_440 (+8 = 12)
    buf.push(1); // frame_flag
    buf.extend_from_slice(&be_i16(2020)); // itrf_epoch
    buf.push(5); // tectonic_plate
    buf.extend_from_slice(&be_i32(120)); // rtx_ram (+8 = 20)
    buf.push(1); // pole_wobble_status
    buf.extend_from_slice(&be_f32(0.05)); // pole_wobble_distance (+5 = 25)
    buf.push(3); // position_fix_type (+1 = 26)

    let rec = PositionTypeInfo::parse(&buf).unwrap();
    assert!(rec.ext_440.is_some());
    let e482 = rec.ext_482.unwrap();
    assert_eq!(e482.itrf_epoch, 2020);
    assert_eq!(e482.rtx_ram_sub_minutes_left, 120);
    let e490 = rec.ext_490.unwrap();
    assert_eq!(e490.pole_wobble_status, 1);
    assert_eq!(rec.position_fix_type, Some(3));
}

// ---------------------------------------------------------------------------
// Type 41 — BasePositionQuality
// ---------------------------------------------------------------------------

#[test]
fn base_position_quality_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u32(432_000_000)); // gps_time_ms
    buf.extend_from_slice(&be_u16(2300)); // gps_week
    let lat_rad = 43.7_f64.to_radians();
    let lon_rad = (-79.4_f64).to_radians();
    buf.extend_from_slice(&be_f64(lat_rad));
    buf.extend_from_slice(&be_f64(lon_rad));
    buf.extend_from_slice(&be_f64(174.5));
    buf.push(3); // quality

    let rec = BasePositionQuality::parse(&buf).unwrap();
    assert_eq!(rec.gps_time_ms, 432_000_000);
    assert_eq!(rec.gps_week, 2300);
    assert!((rec.lat_deg - 43.7).abs() < 1e-7);
    assert_eq!(rec.quality, 3);
}

// ---------------------------------------------------------------------------
// Type 49 — InsFullNav
// ---------------------------------------------------------------------------

#[test]
fn ins_full_nav_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300)); // week
    buf.extend_from_slice(&be_u32(432_000_000)); // time_ms
    buf.push(4); // imu_alignment
    buf.push(2); // gps_quality
    buf.extend_from_slice(&be_f64(43.7)); // lat
    buf.extend_from_slice(&be_f64(-79.4)); // lon
    buf.extend_from_slice(&be_f64(174.5)); // alt
    buf.extend_from_slice(&be_f32(1.0)); // north_vel
    buf.extend_from_slice(&be_f32(2.0)); // east_vel
    buf.extend_from_slice(&be_f32(-0.5)); // down_vel
    buf.extend_from_slice(&be_f32(2.5)); // total_speed
    buf.extend_from_slice(&be_f64(0.1)); // roll
    buf.extend_from_slice(&be_f64(0.2)); // pitch
    buf.extend_from_slice(&be_f64(90.0)); // heading
    buf.extend_from_slice(&be_f64(89.5)); // track
    buf.extend_from_slice(&be_f32(0.01)); // ang_rate_x
    buf.extend_from_slice(&be_f32(0.02)); // ang_rate_y
    buf.extend_from_slice(&be_f32(0.03)); // ang_rate_z
    buf.extend_from_slice(&be_f32(0.1)); // accel_x
    buf.extend_from_slice(&be_f32(0.2)); // accel_y
    buf.extend_from_slice(&be_f32(-9.8)); // accel_z
    assert_eq!(buf.len(), 104);

    let rec = InsFullNav::parse(&buf).unwrap();
    assert_eq!(rec.gps_week, 2300);
    assert_eq!(rec.gps_time_ms, 432_000_000);
    assert_eq!(rec.imu_alignment_status, 4);
    assert_eq!(rec.gps_quality, 2);
    assert!((rec.lat_deg - 43.7).abs() < 1e-10);
    assert!((rec.lon_deg - (-79.4)).abs() < 1e-10);
    assert!((rec.heading_deg - 90.0).abs() < 1e-10);
    assert!((rec.accel_z - (-9.8_f32)).abs() < 1e-5);
}

#[test]
fn ins_full_nav_too_short() {
    let buf = [0u8; 50]; // needs 104
    assert!(matches!(
        InsFullNav::parse(&buf).unwrap_err(),
        ParseError::UnexpectedEof { record_type: 49, .. }
    ));
}

// ---------------------------------------------------------------------------
// Type 50 — InsRmsInfo
// ---------------------------------------------------------------------------

#[test]
fn ins_rms_info_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(432_000_000));
    buf.push(4); // imu
    buf.push(2); // gps
    for v in [0.1_f32, 0.2, 0.3, 0.01, 0.02, 0.03, 0.5, 0.4, 1.0] {
        buf.extend_from_slice(&be_f32(v));
    }
    assert_eq!(buf.len(), 44);

    let rec = InsRmsInfo::parse(&buf).unwrap();
    assert_eq!(rec.gps_week, 2300);
    assert!((rec.north_pos_rms - 0.1).abs() < 1e-6);
    assert!((rec.heading_rms_deg - 1.0).abs() < 1e-6);
}

// ---------------------------------------------------------------------------
// Type 52 — DmiRawData
// ---------------------------------------------------------------------------

#[test]
fn dmi_raw_data_two_measurements() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300)); // week
    buf.extend_from_slice(&be_u32(100_000)); // time_ms
    buf.push(2); // count

    // Measurement 1
    buf.extend_from_slice(&be_u16(10)); // time_offset
    buf.extend_from_slice(&be_u32(5000)); // abs_dist
    buf.extend_from_slice(&be_i32(-100)); // ud_dist

    // Measurement 2
    buf.extend_from_slice(&be_u16(20));
    buf.extend_from_slice(&be_u32(5100));
    buf.extend_from_slice(&be_i32(200));

    let rec = DmiRawData::parse(&buf).unwrap();
    assert_eq!(rec.gps_week, 2300);
    assert_eq!(rec.measurements.len(), 2);
    assert_eq!(rec.measurements[0].abs_dist_count, 5000);
    assert_eq!(rec.measurements[0].ud_dist_count, -100);
    assert_eq!(rec.measurements[1].ud_dist_count, 200);
}

#[test]
fn dmi_raw_data_count_overflow() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(100_000));
    buf.push(20); // count=20, but no measurement data
    assert!(matches!(
        DmiRawData::parse(&buf).unwrap_err(),
        ParseError::OverflowingCount { record_type: 52, .. }
    ));
}

// ---------------------------------------------------------------------------
// Type 63 — InsVnavFullNav
// ---------------------------------------------------------------------------

#[test]
fn ins_vnav_full_nav_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(432_000_000));
    buf.push(4);
    buf.push(2);
    buf.extend_from_slice(&be_f64(43.7)); // lat
    buf.extend_from_slice(&be_f64(-79.4)); // lon
    buf.extend_from_slice(&be_f64(174.5)); // alt
    for _ in 0..4 {
        buf.extend_from_slice(&be_f32(1.0));
    } // vel + speed
    for _ in 0..4 {
        buf.extend_from_slice(&be_f64(0.1));
    } // attitude + track
    for _ in 0..6 {
        buf.extend_from_slice(&be_f32(0.01));
    } // ang_rate + accel
    buf.extend_from_slice(&be_f64(0.05)); // heave
    assert_eq!(buf.len(), 112);

    let rec = InsVnavFullNav::parse(&buf).unwrap();
    assert_eq!(rec.gps_week, 2300);
    assert!((rec.heave_m - 0.05).abs() < 1e-10);
}

// ---------------------------------------------------------------------------
// Type 64 — InsVnavRmsInfo
// ---------------------------------------------------------------------------

#[test]
fn ins_vnav_rms_info_round_trip() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(432_000_000));
    buf.push(4);
    buf.push(2);
    for v in [0.1_f32, 0.2, 0.3, 0.01, 0.02, 0.03, 0.5, 0.4, 1.0, 0.05] {
        buf.extend_from_slice(&be_f32(v));
    }
    assert_eq!(buf.len(), 48);

    let rec = InsVnavRmsInfo::parse(&buf).unwrap();
    assert_eq!(rec.gps_week, 2300);
    assert!((rec.heave_rms_m - 0.05).abs() < 1e-6);
}

// ---------------------------------------------------------------------------
// Dispatcher — new types route correctly
// ---------------------------------------------------------------------------

#[test]
fn dispatch_ins_full_nav() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(0));
    buf.push(0);
    buf.push(0);
    for _ in 0..3 { buf.extend_from_slice(&be_f64(0.0)); }
    for _ in 0..4 { buf.extend_from_slice(&be_f32(0.0)); }
    for _ in 0..4 { buf.extend_from_slice(&be_f64(0.0)); }
    for _ in 0..6 { buf.extend_from_slice(&be_f32(0.0)); }
    assert_eq!(buf.len(), 104);

    let rec = parse_gsof_record(49, &buf);
    assert!(matches!(rec, GsofRecord::InsFullNav(_)));
    // Ensure Display doesn't panic
    let _ = format!("{rec}");
}

#[test]
fn dispatch_position_type_info() {
    let buf = be_f32(1.0);
    let rec = parse_gsof_record(38, &buf);
    assert!(matches!(rec, GsofRecord::PositionTypeInfo(_)));
}

#[test]
fn dispatch_dmi_raw_data_empty() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_u16(2300));
    buf.extend_from_slice(&be_u32(0));
    buf.push(0); // zero measurements
    let rec = parse_gsof_record(52, &buf);
    assert!(matches!(rec, GsofRecord::DmiRawData(_)));
}

// ---------------------------------------------------------------------------
// Type 3 — ECEF round trip
// ---------------------------------------------------------------------------

#[test]
fn ecef_round_trip() {
    let x: f64 = 918_000.123;
    let y: f64 = -4_346_000.456;
    let z: f64 = 4_562_000.789;
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f64(x));
    buf.extend_from_slice(&be_f64(y));
    buf.extend_from_slice(&be_f64(z));

    let rec = Ecef::parse(&buf).unwrap();
    assert!((rec.x_m - x).abs() < 1e-9);
    assert!((rec.y_m - y).abs() < 1e-9);
    assert!((rec.z_m - z).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// Type 6 — ECEF Delta round trip
// ---------------------------------------------------------------------------

#[test]
fn ecef_delta_round_trip() {
    let dx: f64 = 0.123;
    let dy: f64 = -0.456;
    let dz: f64 = 0.789;
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f64(dx));
    buf.extend_from_slice(&be_f64(dy));
    buf.extend_from_slice(&be_f64(dz));

    let rec = EcefDelta::parse(&buf).unwrap();
    assert!((rec.dx_m - dx).abs() < 1e-9);
    assert!((rec.dy_m - dy).abs() < 1e-9);
    assert!((rec.dz_m - dz).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// Type 7 — Tangent Plane Delta round trip
// ---------------------------------------------------------------------------

#[test]
fn tangent_plane_delta_round_trip() {
    let e: f64 = 1.5;
    let n: f64 = -2.3;
    let u: f64 = 0.7;
    let mut buf = Vec::new();
    buf.extend_from_slice(&be_f64(e));
    buf.extend_from_slice(&be_f64(n));
    buf.extend_from_slice(&be_f64(u));

    let rec = TangentPlaneDelta::parse(&buf).unwrap();
    assert!((rec.east_m - e).abs() < 1e-9);
    assert!((rec.north_m - n).abs() < 1e-9);
    assert!((rec.up_m - u).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// Type 33 — AllBriefSvInfo with two SVs
// ---------------------------------------------------------------------------

#[test]
fn all_brief_sv_info_two_svs() {
    let mut buf = Vec::new();
    buf.push(2); // count

    // SV 1: GPS PRN 5
    buf.push(5);  // prn
    buf.push(0);  // system = GPS
    buf.push(0x07); // flags1 = above horizon + assigned + tracked
    buf.push(0x00); // flags2

    // SV 2: GALILEO PRN 12
    buf.push(12); // prn
    buf.push(3);  // system = GALILEO
    buf.push(0x05); // flags1 = above horizon + tracked
    buf.push(0x01); // flags2

    let rec = AllBriefSvInfo::parse(&buf).unwrap();
    assert_eq!(rec.svs.len(), 2);
    assert_eq!(rec.svs[0].prn, 5);
    assert_eq!(rec.svs[0].system, 0);
    assert!(rec.svs[0].is_above_horizon());
    assert!(rec.svs[0].is_assigned_to_channel());
    assert!(rec.svs[0].is_tracked());
    assert_eq!(rec.svs[1].prn, 12);
    assert_eq!(rec.svs[1].system, 3);
    assert!(rec.svs[1].is_above_horizon());
    assert!(!rec.svs[1].is_assigned_to_channel());
    assert!(rec.svs[1].is_tracked());
}

// ---------------------------------------------------------------------------
// Type 8 — Velocity with local_heading
// ---------------------------------------------------------------------------

#[test]
fn velocity_with_local_heading() {
    let heading_rad: f32 = 1.5_f32;
    let local_heading_rad: f32 = 0.5_f32;
    let mut buf = Vec::new();
    buf.push(0x07); // flags
    buf.extend_from_slice(&be_f32(10.0_f32));
    buf.extend_from_slice(&be_f32(heading_rad));
    buf.extend_from_slice(&be_f32(-0.2_f32));
    buf.extend_from_slice(&be_f32(local_heading_rad));
    assert_eq!(buf.len(), 17);

    let rec = Velocity::parse(&buf).unwrap();
    let lh = rec.local_heading.expect("local_heading should be Some");
    let expected_deg = local_heading_rad * (180.0 / core::f32::consts::PI);
    assert!((lh - expected_deg).abs() < 1e-4);
}

#[test]
fn velocity_without_local_heading() {
    let heading_rad: f32 = 1.5_f32;
    let mut buf = Vec::new();
    buf.push(0x03); // flags
    buf.extend_from_slice(&be_f32(12.5_f32));
    buf.extend_from_slice(&be_f32(heading_rad));
    buf.extend_from_slice(&be_f32(-0.1_f32));
    assert_eq!(buf.len(), 13);

    let rec = Velocity::parse(&buf).unwrap();
    assert!(rec.local_heading.is_none());
}
