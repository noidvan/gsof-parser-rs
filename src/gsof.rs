/// gsof.rs — GSOF record type definitions and decoders.
///
/// All structs implement `Display` for human-readable output matching the
/// original gsofParser.c listing format.
///
/// Design notes
/// ============
/// * `parse_*` functions accept a `&[u8]` slice bounded to exactly the bytes
///   declared by the record's `length` field.  They return `Result<_, ParseError>`
///   rather than panicking or silently truncating.
/// * Endian conversion is handled by the `Reader` helper, which advances an
///   internal cursor.  No raw pointer arithmetic.
/// * The original code assumed little-endian host; we simply call
///   `from_be_bytes` for the big-endian wire format instead of reversing bytes
///   manually.
/// * `f32`/`f64` values are reinterpreted via `from_be_bytes` — guaranteed
///   well-defined in Rust (IEEE-754 layout identical to C on both endiannesses).
use core::fmt;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum ParseError {
    /// The slice was shorter than the number of bytes we tried to read.
    #[cfg_attr(feature="std", error("GSOF type {record_type}: unexpected EOF — needed {needed} bytes, only {available} available"))]
    UnexpectedEof {
        record_type: u8,
        needed: usize,
        available: usize,
    },
    /// A variable-length record contained more items than the remaining bytes
    /// could possibly hold.
    #[cfg_attr(feature="std", error("GSOF type {record_type}: SV count {count} × {bytes_per_item} B/item exceeds {available} remaining bytes"))]
    OverflowingCount {
        record_type: u8,
        count: usize,
        bytes_per_item: usize,
        available: usize,
    },
}

#[cfg(not(feature = "std"))]
impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParseError::UnexpectedEof { record_type, needed, available } => write!(
                f,
                "GSOF type {record_type}: unexpected EOF — needed {needed} bytes, only {available} available"
            ),
            ParseError::OverflowingCount { record_type, count, bytes_per_item, available } => write!(
                f,
                "GSOF type {record_type}: SV count {count} × {bytes_per_item} B/item exceeds {available} remaining bytes"
            ),
        }
    }
}

// ---------------------------------------------------------------------------
// Cursor-based byte reader
// ---------------------------------------------------------------------------

/// Thin cursor over a byte slice.  Every read advances the position and
/// returns a `Result` — no unchecked indexing.
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
    record_type: u8, // carried along for error messages
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8], record_type: u8) -> Self {
        Self {
            buf,
            pos: 0,
            record_type,
        }
    }

    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], ParseError> {
        if self.remaining() < N {
            return Err(ParseError::UnexpectedEof {
                record_type: self.record_type,
                needed: N,
                available: self.remaining(),
            });
        }
        let arr: [u8; N] = self.buf[self.pos..self.pos + N].try_into().unwrap();
        self.pos += N;
        Ok(arr)
    }

    pub fn u8(&mut self) -> Result<u8, ParseError> {
        Ok(u8::from_be_bytes(self.take::<1>()?))
    }

    pub fn u16(&mut self) -> Result<u16, ParseError> {
        Ok(u16::from_be_bytes(self.take::<2>()?))
    }

    pub fn i16(&mut self) -> Result<i16, ParseError> {
        Ok(i16::from_be_bytes(self.take::<2>()?))
    }

    pub fn u32(&mut self) -> Result<u32, ParseError> {
        Ok(u32::from_be_bytes(self.take::<4>()?))
    }

    pub fn f32(&mut self) -> Result<f32, ParseError> {
        Ok(f32::from_be_bytes(self.take::<4>()?))
    }

    pub fn f64(&mut self) -> Result<f64, ParseError> {
        Ok(f64::from_be_bytes(self.take::<8>()?))
    }

    /// Skip `n` bytes (e.g. reserved fields).
    pub fn skip(&mut self, n: usize) -> Result<(), ParseError> {
        if self.remaining() < n {
            return Err(ParseError::UnexpectedEof {
                record_type: self.record_type,
                needed: n,
                available: self.remaining(),
            });
        }
        self.pos += n;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// GNSS constellation helper
// ---------------------------------------------------------------------------

fn constellation_name(system: u8) -> &'static str {
    match system {
        0 => "GPS",
        1 => "SBAS",
        2 => "GLONASS",
        3 => "GALILEO",
        4 => "QZSS",
        5 => "BEIDOU",
        _ => "RESERVED",
    }
}

const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;

// ---------------------------------------------------------------------------
// GSOF record types
// ---------------------------------------------------------------------------

/// Encompasses every decoded GSOF subtype record.
#[derive(Debug)]
pub enum GsofRecord {
    PositionTime(PositionTime),           // type 1
    LatLonHeight(LatLonHeight),           // type 2
    Ecef(Ecef),                           // type 3
    LocalDatum(LocalDatum),               // type 4
    EcefDelta(EcefDelta),                 // type 6
    TangentPlaneDelta(TangentPlaneDelta), // type 7
    Velocity(Velocity),                   // type 8
    PdopInfo(PdopInfo),                   // type 9
    BriefSvInfo(BriefSvInfo),             // type 13
    SvDetailedInfo(SvDetailedInfo),       // type 14
    UtcTime(UtcTime),                     // type 16
    AttitudeInfo(AttitudeInfo),           // type 27
    AllBriefSvInfo(AllBriefSvInfo),       // type 33
    AllDetailedSvInfo(AllDetailedSvInfo), // type 34
    LbandStatus(LbandStatus),             // type 40
    Unknown {
        gsof_type: u8,
        data: heapless::Vec<u8, 255>,
    },
}

impl fmt::Display for GsofRecord {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GsofRecord::PositionTime(r) => write!(f, "{r}"),
            GsofRecord::LatLonHeight(r) => write!(f, "{r}"),
            GsofRecord::Ecef(r) => write!(f, "{r}"),
            GsofRecord::LocalDatum(r) => write!(f, "{r}"),
            GsofRecord::EcefDelta(r) => write!(f, "{r}"),
            GsofRecord::TangentPlaneDelta(r) => write!(f, "{r}"),
            GsofRecord::Velocity(r) => write!(f, "{r}"),
            GsofRecord::PdopInfo(r) => write!(f, "{r}"),
            GsofRecord::BriefSvInfo(r) => write!(f, "{r}"),
            GsofRecord::SvDetailedInfo(r) => write!(f, "{r}"),
            GsofRecord::UtcTime(r) => write!(f, "{r}"),
            GsofRecord::AttitudeInfo(r) => write!(f, "{r}"),
            GsofRecord::AllBriefSvInfo(r) => write!(f, "{r}"),
            GsofRecord::AllDetailedSvInfo(r) => write!(f, "{r}"),
            GsofRecord::LbandStatus(r) => write!(f, "{r}"),
            GsofRecord::Unknown { gsof_type, data } => {
                write!(f, "  GsofType:{gsof_type}  len:{}\n  ", data.len())?;
                for (i, b) in data.iter().enumerate() {
                    if i > 0 && i % 16 == 0 {
                        write!(f, "\n  ")?;
                    }
                    write!(f, "{b:02X} ")?;
                }
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Type 1 — Position/Time
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct PositionTime {
    pub milliseconds: u32,
    pub week_number: u16,
    pub num_svs: u8,
    pub flags1: u8,
    pub flags2: u8,
    pub init_number: u8,
}

impl PositionTime {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 1);
        Ok(Self {
            milliseconds: r.u32()?,
            week_number: r.u16()?,
            num_svs: r.u8()?,
            flags1: r.u8()?,
            flags2: r.u8()?,
            init_number: r.u8()?,
        })
    }
}

impl fmt::Display for PositionTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:1 - PositionTime  len:{}", 10)?;
        write!(
            f,
            "  Milliseconds:{}  Week:{}  #Svs:{} flags:{:02X}:{:02X} init:{}",
            self.milliseconds,
            self.week_number,
            self.num_svs,
            self.flags1,
            self.flags2,
            self.init_number
        )
    }
}

// ---------------------------------------------------------------------------
// Type 2 — Lat/Lon/Height
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct LatLonHeight {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub height_m: f64,
}

impl LatLonHeight {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 2);
        Ok(Self {
            lat_deg: r.f64()? * RAD_TO_DEG,
            lon_deg: r.f64()? * RAD_TO_DEG,
            height_m: r.f64()?,
        })
    }
}

impl fmt::Display for LatLonHeight {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:2 - LatLongHeight   len:{}", 24)?;
        write!(
            f,
            "  Lat:{:.7} Lon:{:.7} Height:{:.3}",
            self.lat_deg, self.lon_deg, self.height_m
        )
    }
}

// ---------------------------------------------------------------------------
// Type 3 — ECEF
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct Ecef {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
}

impl Ecef {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 3);
        Ok(Self {
            x_m: r.f64()?,
            y_m: r.f64()?,
            z_m: r.f64()?,
        })
    }
}

impl fmt::Display for Ecef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:3 - ECEF   len:{}", 24)?;
        write!(f, "  X:{:.3} Y:{:.3} Z:{:.3}", self.x_m, self.y_m, self.z_m)
    }
}

// ---------------------------------------------------------------------------
// Type 4 — Local Datum
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct LocalDatum {
    pub id: [u8; 8],
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub height_m: f64,
}

impl LocalDatum {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 4);
        let id = r.take::<8>()?;
        Ok(Self {
            id,
            lat_deg: r.f64()? * RAD_TO_DEG,
            lon_deg: r.f64()? * RAD_TO_DEG,
            height_m: r.f64()?,
        })
    }

    pub fn id_str(&self) -> &str {
        core::str::from_utf8(&self.id).unwrap_or("????????")
    }
}

impl fmt::Display for LocalDatum {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:4 - Local Datum Position  !!!!!UNTESTED!!!!!!!"
        )?;
        write!(
            f,
            "  Id:{} Lat:{:.7} Lon:{:.7} Height:{:.3}",
            self.id_str(),
            self.lat_deg,
            self.lon_deg,
            self.height_m
        )
    }
}

// ---------------------------------------------------------------------------
// Type 6 — ECEF Delta
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct EcefDelta {
    pub dx_m: f64,
    pub dy_m: f64,
    pub dz_m: f64,
}

impl EcefDelta {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 6);
        Ok(Self {
            dx_m: r.f64()?,
            dy_m: r.f64()?,
            dz_m: r.f64()?,
        })
    }
}

impl fmt::Display for EcefDelta {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:6 - ECEF Delta  len:{}", 24)?;
        write!(
            f,
            "  X:{:.3} Y:{:.3} Z:{:.3}",
            self.dx_m, self.dy_m, self.dz_m
        )
    }
}

// ---------------------------------------------------------------------------
// Type 7 — Tangent Plane Delta
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct TangentPlaneDelta {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
}

impl TangentPlaneDelta {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 7);
        Ok(Self {
            east_m: r.f64()?,
            north_m: r.f64()?,
            up_m: r.f64()?,
        })
    }
}

impl fmt::Display for TangentPlaneDelta {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:7 - Tangent Plane Delta  len:{}", 24)?;
        write!(
            f,
            "  East:{:.3} North:{:.3} Up:{:.3}",
            self.east_m, self.north_m, self.up_m
        )
    }
}

// ---------------------------------------------------------------------------
// Type 8 — Velocity
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct Velocity {
    pub flags: u8,
    pub velocity_ms: f32,
    pub heading_deg: f32,
    pub vertical_ms: f32,
}

impl Velocity {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 8);
        Ok(Self {
            flags: r.u8()?,
            velocity_ms: r.f32()?,
            heading_deg: r.f32()? * (180.0 / core::f32::consts::PI),
            vertical_ms: r.f32()?,
        })
    }
}

impl fmt::Display for Velocity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:8 - Velocity Data  len:{}", 13)?;
        write!(
            f,
            "  Flags:{:02X}  velocity:{:.3}  heading:{:.3}  vertical:{:.3}",
            self.flags, self.velocity_ms, self.heading_deg, self.vertical_ms
        )
    }
}

// ---------------------------------------------------------------------------
// Type 9 — PDOP
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct PdopInfo {
    pub pdop: f32,
    pub hdop: f32,
    pub vdop: f32,
    pub tdop: f32,
}

impl PdopInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 9);
        Ok(Self {
            pdop: r.f32()?,
            hdop: r.f32()?,
            vdop: r.f32()?,
            tdop: r.f32()?,
        })
    }
}

impl fmt::Display for PdopInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:9 - PDOP Info   len:{}", 16)?;
        write!(
            f,
            "  PDOP:{:.1}  HDOP:{:.1}  VDOP:{:.1}  TDOP:{:.1}",
            self.pdop, self.hdop, self.vdop, self.tdop
        )
    }
}

// ---------------------------------------------------------------------------
// Type 13 — Brief SV Info (GPS-only, older format)
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct BriefSv {
    pub prn: u8,
    pub flags1: u8,
    pub flags2: u8,
}

#[derive(Debug)]
pub struct BriefSvInfo {
    pub svs: heapless::Vec<BriefSv, 32>,
}

impl BriefSvInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 13);
        let count = r.u8()? as usize;
        let bytes_needed = count * 3;
        if bytes_needed > r.remaining() {
            return Err(ParseError::OverflowingCount {
                record_type: 13,
                count,
                bytes_per_item: 3,
                available: r.remaining(),
            });
        }
        let mut svs: heapless::Vec<BriefSv, 32> = heapless::Vec::new();
        for _ in 0..count {
            svs.push(BriefSv {
                prn: r.u8()?,
                flags1: r.u8()?,
                flags2: r.u8()?,
            })
            .map_err(|_| ParseError::OverflowingCount {
                record_type: 13,
                count,
                bytes_per_item: 3,
                available: svs.capacity(),
            })?;
        }
        Ok(Self { svs })
    }
}

impl fmt::Display for BriefSvInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:13 - SV Brief Info   len:{}",
            1 + self.svs.len() * 3
        )?;
        writeln!(f, "  SvCount:{}", self.svs.len())?;
        for sv in &self.svs {
            write!(
                f,
                "  Prn:{:<2}  flags:{:02X}:{:02X}",
                sv.prn, sv.flags1, sv.flags2
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 14 — SV Detailed Info (GPS-only, older format)
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct SvDetail {
    pub prn: u8,
    pub flags1: u8,
    pub flags2: u8,
    pub elevation_deg: u8,
    pub azimuth_deg: u16,
    pub l1_snr: u8,
    pub l2_snr: u8,
}

#[derive(Debug)]
pub struct SvDetailedInfo {
    pub svs: heapless::Vec<SvDetail, 32>,
}

impl SvDetailedInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 14);
        let count = r.u8()? as usize;
        let bytes_needed = count * 8;
        if bytes_needed > r.remaining() {
            return Err(ParseError::OverflowingCount {
                record_type: 14,
                count,
                bytes_per_item: 8,
                available: r.remaining(),
            });
        }
        let mut svs: heapless::Vec<SvDetail, 32> = heapless::Vec::new();
        for _ in 0..count {
            svs.push(SvDetail {
                prn: r.u8()?,
                flags1: r.u8()?,
                flags2: r.u8()?,
                elevation_deg: r.u8()?,
                azimuth_deg: r.u16()?,
                l1_snr: r.u8()?,
                l2_snr: r.u8()?,
            })
            .map_err(|_| ParseError::OverflowingCount {
                record_type: 14,
                count,
                bytes_per_item: 8,
                available: svs.capacity(),
            })?;
        }
        Ok(Self { svs })
    }
}

impl fmt::Display for SvDetailedInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:14 - SV Detailed Info   len:{}",
            1 + self.svs.len() * 8
        )?;
        writeln!(f, "  SvCount:{}", self.svs.len())?;
        for sv in &self.svs {
            writeln!(
                f,
                "   Prn:{:<2}  flags:{:02X}:{:02X} elv:{:<2} azm:{:<3}  L1snr:{:<5.2} L2snr:{:<5.2}",
                sv.prn,
                sv.flags1,
                sv.flags2,
                sv.elevation_deg,
                sv.azimuth_deg,
                sv.l1_snr as f64 / 4.0,
                sv.l2_snr as f64 / 4.0,
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 16 — UTC Time
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct UtcTime {
    pub milliseconds: u32,
    pub week_number: u16,
    pub utc_offset_s: i16,
    pub flags: u8,
}

impl UtcTime {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 16);
        Ok(Self {
            milliseconds: r.u32()?,
            week_number: r.u16()?,
            utc_offset_s: r.i16()?,
            flags: r.u8()?,
        })
    }
}

impl fmt::Display for UtcTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:16 - UTC Time Info   len:{}", 9)?;
        write!(
            f,
            "  ms:{}  week:{}  utcOff:{}  flags:{:02x}",
            self.milliseconds, self.week_number, self.utc_offset_s, self.flags
        )
    }
}

// ---------------------------------------------------------------------------
// Type 27 — Attitude Info
// ---------------------------------------------------------------------------

/// Optional variance/covariance extension (present when record length > 42).
#[derive(Debug)]
pub struct AttitudeVariance {
    pub pitch_var_rad2: f32,
    pub yaw_var_rad2: f32,
    pub roll_var_rad2: f32,
    pub pitch_yaw_covar_rad2: f32,
    pub pitch_roll_covar_rad2: f32,
    pub yaw_roll_covar_rad2: f32,
    pub range_var_m2: f32,
}

#[derive(Debug)]
pub struct AttitudeInfo {
    pub gps_time_s: f64,
    pub flags: u8,
    pub num_svs: u8,
    pub mode: u8,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
    pub roll_deg: f64,
    pub range_m: f64,
    pub pdop: f64,
    pub variance: Option<AttitudeVariance>,
}

impl AttitudeInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 27);
        let gps_time_s = r.u32()? as f64 / 1000.0;
        let flags = r.u8()?;
        let num_svs = r.u8()?;
        let mode = r.u8()?;
        r.skip(1)?; // reserved
        let pitch_deg = r.f64()? * RAD_TO_DEG;
        let yaw_deg = r.f64()? * RAD_TO_DEG;
        let roll_deg = r.f64()? * RAD_TO_DEG;
        let range_m = r.f64()?;
        let pdop = r.u16()? as f64 / 10.0;

        // Extended record: present when original length > 42
        let variance = if data.len() > 42 {
            Some(AttitudeVariance {
                pitch_var_rad2: r.f32()?,
                yaw_var_rad2: r.f32()?,
                roll_var_rad2: r.f32()?,
                pitch_yaw_covar_rad2: r.f32()?,
                pitch_roll_covar_rad2: r.f32()?,
                yaw_roll_covar_rad2: r.f32()?,
                range_var_m2: r.f32()?,
            })
        } else {
            None
        };

        Ok(Self {
            gps_time_s,
            flags,
            num_svs,
            mode,
            pitch_deg,
            yaw_deg,
            roll_deg,
            range_m,
            pdop,
            variance,
        })
    }
}

impl fmt::Display for AttitudeInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:27 - AttitudeInfo  len:{}",
            if self.variance.is_some() { 70 } else { 42 }
        )?;
        writeln!(
            f,
            "  Time:{:.3} flags:{:02X} nSVs:{} mode:{}",
            self.gps_time_s, self.flags, self.num_svs, self.mode
        )?;
        write!(
            f,
            "  pitch:{:.3} yaw:{:.3} roll:{:.3} range:{:.3} pdop:{:.1}",
            self.pitch_deg, self.yaw_deg, self.roll_deg, self.range_m, self.pdop
        )?;
        if let Some(v) = &self.variance {
            write!(
                f,
                "\n  variance (radians^2) pitch:{:.4e} yaw:{:.4e} roll:{:.4e}\
                 \n  covariance (radians^2) pitch-yaw:{:.4e} pitch-roll:{:.4e} yaw-roll:{:.4e}\
                 \n  variance (m^2) range: {:.4e}",
                v.pitch_var_rad2,
                v.yaw_var_rad2,
                v.roll_var_rad2,
                v.pitch_yaw_covar_rad2,
                v.pitch_roll_covar_rad2,
                v.yaw_roll_covar_rad2,
                v.range_var_m2
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 33 — All Brief SV Info (multi-constellation)
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct AllBriefSv {
    pub prn: u8,
    pub system: u8,
    pub flags1: u8,
    pub flags2: u8,
}

#[derive(Debug)]
pub struct AllBriefSvInfo {
    pub svs: heapless::Vec<AllBriefSv, 64>,
}

impl AllBriefSvInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 33);
        let count = r.u8()? as usize;
        let bytes_needed = count * 4;
        if bytes_needed > r.remaining() {
            return Err(ParseError::OverflowingCount {
                record_type: 33,
                count,
                bytes_per_item: 4,
                available: r.remaining(),
            });
        }
        let mut svs: heapless::Vec<AllBriefSv, 64> = heapless::Vec::new();
        for _ in 0..count {
            svs.push(AllBriefSv {
                prn: r.u8()?,
                system: r.u8()?,
                flags1: r.u8()?,
                flags2: r.u8()?,
            })
            .map_err(|_| ParseError::OverflowingCount {
                record_type: 33,
                count,
                bytes_per_item: 4,
                available: svs.capacity(),
            })?;
        }
        Ok(Self { svs })
    }
}

impl fmt::Display for AllBriefSvInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:33 - All SV Brief Info   len:{}",
            1 + self.svs.len() * 4
        )?;
        writeln!(f, "  SvCount:{}", self.svs.len())?;
        for sv in &self.svs {
            writeln!(
                f,
                "  {} SV:{:<2}  flags:{:02X}:{:02X}",
                constellation_name(sv.system),
                sv.prn,
                sv.flags1,
                sv.flags2
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 34 — All Detailed SV Info (multi-constellation)
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct AllDetailedSv {
    pub prn: u8,
    pub system: u8,
    pub flags1: u8,
    pub flags2: u8,
    pub elevation_deg: u8,
    pub azimuth_deg: u16,
    pub snr: [u8; 3], // L1/E1, L2/N/A, L5/E5/G1P
}

#[derive(Debug)]
pub struct AllDetailedSvInfo {
    pub svs: heapless::Vec<AllDetailedSv, 64>,
}

impl AllDetailedSvInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 34);
        let count = r.u8()? as usize;
        // each SV: prn(1) + system(1) + flags1(1) + flags2(1) + elv(1) + az(2) + snr[3](3) = 10 bytes
        let bytes_needed = count * 10;
        if bytes_needed > r.remaining() {
            return Err(ParseError::OverflowingCount {
                record_type: 34,
                count,
                bytes_per_item: 10,
                available: r.remaining(),
            });
        }
        let mut svs: heapless::Vec<AllDetailedSv, 64> = heapless::Vec::new();
        for _ in 0..count {
            svs.push(AllDetailedSv {
                prn: r.u8()?,
                system: r.u8()?,
                flags1: r.u8()?,
                flags2: r.u8()?,
                elevation_deg: r.u8()?,
                azimuth_deg: r.u16()?,
                snr: [r.u8()?, r.u8()?, r.u8()?],
            })
            .map_err(|_| ParseError::OverflowingCount {
                record_type: 34,
                count,
                bytes_per_item: 10,
                available: svs.capacity(),
            })?;
        }
        Ok(Self { svs })
    }
}

impl fmt::Display for AllDetailedSvInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:34 - All Detailed SV Info   len:{}",
            1 + self.svs.len() * 10
        )?;
        writeln!(f, "  SvCount:{}", self.svs.len())?;
        for sv in &self.svs {
            let sys = sv.system;
            let (l1_label, l2_label, l5_label) = match sys {
                3 => ("E1 ", "N/A", "E5 "),
                2 => ("L1 ", "L2 ", "G1P"),
                _ => ("L1 ", "L2 ", "L5 "),
            };
            writeln!(
                f,
                "     {} SV:{:<2}  flags:{:02X}:{:02X}\n     El:{:2}  Az:{:3}\n     SNR {} {:5.2}\n     SNR {} {:5.2}\n     SNR {} {:5.2}",
                constellation_name(sys),
                sv.prn,
                sv.flags1,
                sv.flags2,
                sv.elevation_deg,
                sv.azimuth_deg,
                l1_label, sv.snr[0] as f32 / 4.0,
                l2_label, sv.snr[1] as f32 / 4.0,
                l5_label, sv.snr[2] as f32 / 4.0,
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 40 — L-band Status
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub struct LbandStatus {
    pub name: [u8; 5],
    pub freq_hz: f32,
    pub bit_rate_bps: u16,
    pub snr: f32,
    pub hp_xp_subscribed_engine: u8,
    pub hp_xp_library_mode: u8,
    pub vbs_library_mode: u8,
    pub beam_mode: u8,
    pub omnistar_motion: u8,
    pub horiz_prec_thresh: f32,
    pub vert_prec_thresh: f32,
    pub nmea_encryption: u8,
    pub iq_ratio: f32,
    pub est_ber: f32,
    pub total_uw: u32,
    pub total_bad_uw: u32,
    pub total_bad_uw_bits: u32,
    pub total_viterbi: u32,
    pub total_bad_viterbi: u32,
    pub total_bad_messages: u32,
    /// Only present when record length > 61.
    pub measured_freq: Option<(u8, f64)>,
}

impl LbandStatus {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 40);
        let name = r.take::<5>()?;
        let freq_hz = r.f32()?;
        let bit_rate_bps = r.u16()?;
        let snr = r.f32()?;
        let hp_xp_subscribed_engine = r.u8()?;
        let hp_xp_library_mode = r.u8()?;
        let vbs_library_mode = r.u8()?;
        let beam_mode = r.u8()?;
        let omnistar_motion = r.u8()?;
        let horiz_prec_thresh = r.f32()?;
        let vert_prec_thresh = r.f32()?;
        let nmea_encryption = r.u8()?;
        let iq_ratio = r.f32()?;
        let est_ber = r.f32()?;
        let total_uw = r.u32()?;
        let total_bad_uw = r.u32()?;
        let total_bad_uw_bits = r.u32()?;
        let total_viterbi = r.u32()?;
        let total_bad_viterbi = r.u32()?;
        let total_bad_messages = r.u32()?;
        let measured_freq = if data.len() > 61 {
            Some((r.u8()?, r.f64()?))
        } else {
            None
        };
        Ok(Self {
            name,
            freq_hz,
            bit_rate_bps,
            snr,
            hp_xp_subscribed_engine,
            hp_xp_library_mode,
            vbs_library_mode,
            beam_mode,
            omnistar_motion,
            horiz_prec_thresh,
            vert_prec_thresh,
            nmea_encryption,
            iq_ratio,
            est_ber,
            total_uw,
            total_bad_uw,
            total_bad_uw_bits,
            total_viterbi,
            total_bad_viterbi,
            total_bad_messages,
            measured_freq,
        })
    }

    pub fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name).unwrap_or("?????")
    }
}

impl fmt::Display for LbandStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:40 - LBAND status  len:{}",
            if self.measured_freq.is_some() { 70 } else { 61 }
        )?;
        writeln!(
            f,
            "  Name:{}  Freq:{}  bit rate:{}  SNR:{}",
            self.name_str(),
            self.freq_hz,
            self.bit_rate_bps,
            self.snr
        )?;
        writeln!(
            f,
            "  HP/XP engine:{}  HP/XP mode:{}  VBS mode:{}",
            self.hp_xp_subscribed_engine, self.hp_xp_library_mode, self.vbs_library_mode
        )?;
        writeln!(
            f,
            "  Beam mode:{}  Omnistar Motion:{}",
            self.beam_mode, self.omnistar_motion
        )?;
        writeln!(
            f,
            "  Horiz prec. thresh.:{}  Vert prec. thresh.:{}",
            self.horiz_prec_thresh, self.vert_prec_thresh
        )?;
        writeln!(
            f,
            "  NMEA encryp.:{}  I/Q ratio:{}  Estimated BER:{}",
            self.nmea_encryption, self.iq_ratio, self.est_ber
        )?;
        writeln!(
            f,
            "  Total unique words(UW):{}  Bad UW:{}  Bad UW bits:{}",
            self.total_uw, self.total_bad_uw, self.total_bad_uw_bits
        )?;
        write!(
            f,
            "  Total Viterbi:{}  Corrected Viterbi:{}  Bad messages:{}",
            self.total_viterbi, self.total_bad_viterbi, self.total_bad_messages
        )?;
        if let Some((valid, freq)) = self.measured_freq {
            write!(f, "\n  Meas freq valid?:{}  Meas freq:{:.3}", valid, freq)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Top-level GSOF record dispatcher
// ---------------------------------------------------------------------------

fn unknown(gsof_type: u8, data: &[u8]) -> GsofRecord {
    GsofRecord::Unknown {
        gsof_type,
        data: heapless::Vec::from_slice(data).unwrap_or_default(),
    }
}

/// Parse a single GSOF record from `(type_byte, data_slice)`.
/// The data slice must already be bounded to exactly `length` bytes.
pub fn parse_gsof_record(gsof_type: u8, data: &[u8]) -> GsofRecord {
    match gsof_type {
        1 => PositionTime::parse(data)
            .map(GsofRecord::PositionTime)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        2 => LatLonHeight::parse(data)
            .map(GsofRecord::LatLonHeight)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        3 => Ecef::parse(data)
            .map(GsofRecord::Ecef)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        4 => LocalDatum::parse(data)
            .map(GsofRecord::LocalDatum)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        6 => EcefDelta::parse(data)
            .map(GsofRecord::EcefDelta)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        7 => TangentPlaneDelta::parse(data)
            .map(GsofRecord::TangentPlaneDelta)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        8 => Velocity::parse(data)
            .map(GsofRecord::Velocity)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        9 => PdopInfo::parse(data)
            .map(GsofRecord::PdopInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        13 => BriefSvInfo::parse(data)
            .map(GsofRecord::BriefSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        14 => SvDetailedInfo::parse(data)
            .map(GsofRecord::SvDetailedInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        16 => UtcTime::parse(data)
            .map(GsofRecord::UtcTime)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        27 => AttitudeInfo::parse(data)
            .map(GsofRecord::AttitudeInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        33 => AllBriefSvInfo::parse(data)
            .map(GsofRecord::AllBriefSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        34 => AllDetailedSvInfo::parse(data)
            .map(GsofRecord::AllDetailedSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        40 => LbandStatus::parse(data)
            .map(GsofRecord::LbandStatus)
            .unwrap_or_else(|_| unknown(gsof_type, data)),
        _ => unknown(gsof_type, data),
    }
}

/// Walk a complete GSOF payload buffer and return all records.
/// Returns an error if the type+length header extends past the buffer.
pub fn parse_gsof_payload(payload: &[u8]) -> Result<heapless::Vec<GsofRecord, 32>, ParseError> {
    let mut records: heapless::Vec<GsofRecord, 32> = heapless::Vec::new();
    let mut pos = 0;

    while pos < payload.len() {
        // Each sub-record: 1-byte type, 1-byte length, then `length` data bytes.
        if pos + 2 > payload.len() {
            return Err(ParseError::UnexpectedEof {
                record_type: 0xFF,
                needed: 2,
                available: payload.len() - pos,
            });
        }
        let gsof_type = payload[pos];
        let length = payload[pos + 1] as usize;
        pos += 2;

        if pos + length > payload.len() {
            return Err(ParseError::UnexpectedEof {
                record_type: gsof_type,
                needed: length,
                available: payload.len() - pos,
            });
        }

        let data = &payload[pos..pos + length];
        records
            .push(parse_gsof_record(gsof_type, data))
            .map_err(|_| ParseError::OverflowingCount {
                record_type: 0xFF,
                count: records.len() + 1,
                bytes_per_item: 1,
                available: records.capacity(),
            })?;
        pos += length;
    }

    Ok(records)
}
