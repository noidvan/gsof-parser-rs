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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    pub fn i32(&mut self) -> Result<i32, ParseError> {
        Ok(i32::from_be_bytes(self.take::<4>()?))
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
// Enumerations
// ---------------------------------------------------------------------------

/// Velocity measurement source (bit 1 of Velocity flags).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VelocitySource {
    Doppler,
    ConsecutiveMeasurements,
}

/// IMU alignment status for INS records (#49, #50, #63, #64).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImuAlignmentStatus {
    GpsOnly,
    CoarseLeveling,
    Degraded,
    Aligned,
    FullNav,
    Unknown(u8),
}

impl From<u8> for ImuAlignmentStatus {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::GpsOnly,
            1 => Self::CoarseLeveling,
            2 => Self::Degraded,
            3 => Self::Aligned,
            4 => Self::FullNav,
            other => Self::Unknown(other),
        }
    }
}

/// GNSS quality indicator for INS records (#49, #50, #63, #64).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GnssQuality {
    FixNotAvailable,
    GnssSpsMode,
    DifferentialGpsSps,
    GpsPpsMode,
    FixedRtkMode,
    FloatRtk,
    DirectGeoreferencingMode,
    Unknown(u8),
}

impl From<u8> for GnssQuality {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::FixNotAvailable,
            1 => Self::GnssSpsMode,
            2 => Self::DifferentialGpsSps,
            3 => Self::GpsPpsMode,
            4 => Self::FixedRtkMode,
            5 => Self::FloatRtk,
            6 => Self::DirectGeoreferencingMode,
            other => Self::Unknown(other),
        }
    }
}

/// Satellite system type for multi-constellation records (#33, #34).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SatelliteType {
    Gps,
    Sbas,
    Glonass,
    Galileo,
    Qzss,
    Beidou,
    Irnss,
    Omnistar,
    Unknown(u8),
}

impl From<u8> for SatelliteType {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Gps,
            1 => Self::Sbas,
            2 => Self::Glonass,
            3 => Self::Galileo,
            4 => Self::Qzss,
            5 => Self::Beidou,
            6 => Self::Irnss,
            10 => Self::Omnistar,
            other => Self::Unknown(other),
        }
    }
}

/// Position fix type from trimble position_fix.h (#38).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PositionFix {
    None,
    Autonomous,
    AutonomousProp,
    Sbas,
    SbasProp,
    Dgnss,
    DgnssProp,
    RtkFloatSync,
    RtkFloatProp,
    RtkFixedSync,
    RtkFixedProp,
    OmniHp,
    OmniXp,
    DitheredRtk,
    OmniVbs,
    BeaconDgnss,
    OmniHpXp,
    OmniHpG2,
    OmniG2,
    RtxSync,
    RtxProp,
    OmniMs,
    OmniL1Only,
    InsAutonomous,
    InsSbas,
    InsDgnss,
    InsRtxCode,
    InsRtxCarrier,
    InsOmniPrecise,
    InsRtk,
    InsDeadReckoning,
    RtxCode,
    RtxFastSync,
    RtxFastProp,
    OmniG3,
    OmniG4,
    XfillX,
    RtxLiteProp,
    RtxLiteSync,
    RtxLiteL1Prop,
    RtxLiteL1Sync,
    RtxFieldPointProp,
    RtxFieldPointSync,
    OmniG2Plus,
    OmniG4Plus,
    TitanAutonomous,
    TitanSbas,
    TitanDgnss,
    SlasDgnss,
    Unknown(u8),
}

impl From<u8> for PositionFix {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::None,
            1 => Self::Autonomous,
            2 => Self::AutonomousProp,
            3 => Self::Sbas,
            4 => Self::SbasProp,
            5 => Self::Dgnss,
            6 => Self::DgnssProp,
            7 => Self::RtkFloatSync,
            8 => Self::RtkFloatProp,
            9 => Self::RtkFixedSync,
            10 => Self::RtkFixedProp,
            11 => Self::OmniHp,
            12 => Self::OmniXp,
            13 => Self::DitheredRtk,
            14 => Self::OmniVbs,
            15 => Self::BeaconDgnss,
            16 => Self::OmniHpXp,
            17 => Self::OmniHpG2,
            18 => Self::OmniG2,
            19 => Self::RtxSync,
            20 => Self::RtxProp,
            21 => Self::OmniMs,
            22 => Self::OmniL1Only,
            23 => Self::InsAutonomous,
            24 => Self::InsSbas,
            25 => Self::InsDgnss,
            26 => Self::InsRtxCode,
            27 => Self::InsRtxCarrier,
            28 => Self::InsOmniPrecise,
            29 => Self::InsRtk,
            30 => Self::InsDeadReckoning,
            31 => Self::RtxCode,
            32 => Self::RtxFastSync,
            33 => Self::RtxFastProp,
            34 => Self::OmniG3,
            35 => Self::OmniG4,
            36 => Self::XfillX,
            37 => Self::RtxLiteProp,
            38 => Self::RtxLiteSync,
            39 => Self::RtxLiteL1Prop,
            40 => Self::RtxLiteL1Sync,
            41 => Self::RtxFieldPointProp,
            42 => Self::RtxFieldPointSync,
            43 => Self::OmniG2Plus,
            44 => Self::OmniG4Plus,
            45 => Self::TitanAutonomous,
            46 => Self::TitanSbas,
            47 => Self::TitanDgnss,
            48 => Self::SlasDgnss,
            other => Self::Unknown(other),
        }
    }
}

/// Solution integrity status derived from bits 2-3 of solution_flags (#38).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SolutionIntegrity {
    NotChecking,
    CheckingInitialization,
    InitializationPassed,
    InitializationFailed,
    Unknown(u8),
}

impl SolutionIntegrity {
    pub fn from_solution_flags(flags: u8) -> Self {
        let val = (flags >> 2) & 0x03;
        match val {
            0 => Self::NotChecking,
            1 => Self::CheckingInitialization,
            2 => Self::InitializationPassed,
            3 => Self::InitializationFailed,
            other => Self::Unknown(other),
        }
    }
}

impl From<u8> for SolutionIntegrity {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::NotChecking,
            1 => Self::CheckingInitialization,
            2 => Self::InitializationPassed,
            3 => Self::InitializationFailed,
            other => Self::Unknown(other),
        }
    }
}

/// RTK condition code (#38).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtkCondition {
    NewPositionComputed,
    UnableToObtainSyncedPair,
    InsufficientDoubleDiffMeasurements,
    ReferencePositionUnavailable,
    FailedIntegerVerification,
    SolutionRmsOverLimit,
    PdopOrRdopExceedsMask,
    Unknown(u8),
}

impl From<u8> for RtkCondition {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::NewPositionComputed,
            1 => Self::UnableToObtainSyncedPair,
            2 => Self::InsufficientDoubleDiffMeasurements,
            3 => Self::ReferencePositionUnavailable,
            4 => Self::FailedIntegerVerification,
            5 => Self::SolutionRmsOverLimit,
            6 => Self::PdopOrRdopExceedsMask,
            other => Self::Unknown(other),
        }
    }
}

/// RTCM correction status derived from bits 1-2 of network_flags (#38).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcmStatus {
    NotAvailableOrUnknown,
    CollectingMessages,
    CollectionComplete,
    Working,
    Unknown(u8),
}

impl RtcmStatus {
    pub fn from_network_flags(flags: u8) -> Self {
        let val = (flags >> 1) & 0x03;
        match val {
            0 => Self::NotAvailableOrUnknown,
            1 => Self::CollectingMessages,
            2 => Self::CollectionComplete,
            3 => Self::Working,
            other => Self::Unknown(other),
        }
    }
}

impl From<u8> for RtcmStatus {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::NotAvailableOrUnknown,
            1 => Self::CollectingMessages,
            2 => Self::CollectionComplete,
            3 => Self::Working,
            other => Self::Unknown(other),
        }
    }
}

/// Base position quality indicator (#41).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaseQuality {
    NotAvailableOrInvalid,
    Autonomous,
    DifferentialSbasOrOmnistarVbs,
    RtkFixed,
    OmnistarXpHpOrRtkFloat,
    Unknown(u8),
}

impl From<u8> for BaseQuality {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::NotAvailableOrInvalid,
            1 => Self::Autonomous,
            2 => Self::DifferentialSbasOrOmnistarVbs,
            3 => Self::RtkFixed,
            4 => Self::OmnistarXpHpOrRtkFloat,
            other => Self::Unknown(other),
        }
    }
}

/// OmniSTAR HP/XP subscribed engine (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HpXpEngine {
    None,
    Hp,
    Xp,
    G2,
    Unknown(u8),
}

impl From<u8> for HpXpEngine {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::None,
            1 => Self::Hp,
            2 => Self::Xp,
            3 => Self::G2,
            other => Self::Unknown(other),
        }
    }
}

/// OmniSTAR HP/XP library mode (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HpXpLibraryMode {
    Idle,
    Searching,
    Converging,
    Converged,
    Degraded,
    Unknown(u8),
}

impl From<u8> for HpXpLibraryMode {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Searching,
            2 => Self::Converging,
            3 => Self::Converged,
            4 => Self::Degraded,
            other => Self::Unknown(other),
        }
    }
}

/// OmniSTAR VBS library mode (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VbsLibraryMode {
    Idle,
    Searching,
    Converging,
    Converged,
    Unknown(u8),
}

impl From<u8> for VbsLibraryMode {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Searching,
            2 => Self::Converging,
            3 => Self::Converged,
            other => Self::Unknown(other),
        }
    }
}

/// OmniSTAR beam mode (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BeamMode {
    Manual,
    Auto,
    Unknown(u8),
}

impl From<u8> for BeamMode {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Manual,
            1 => Self::Auto,
            other => Self::Unknown(other),
        }
    }
}

/// OmniSTAR motion state (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotionState {
    Stationary,
    Moving,
    Unknown(u8),
}

impl From<u8> for MotionState {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Stationary,
            1 => Self::Moving,
            other => Self::Unknown(other),
        }
    }
}

/// NMEA encryption state (#40).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NmeaEncryptionState {
    Disabled,
    Enabled,
    Unknown(u8),
}

impl From<u8> for NmeaEncryptionState {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::Disabled,
            1 => Self::Enabled,
            other => Self::Unknown(other),
        }
    }
}

// ---------------------------------------------------------------------------
// GSOF record types
// ---------------------------------------------------------------------------

/// Encompasses every decoded GSOF subtype record.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::large_enum_variant)]
pub enum GsofRecord {
    PositionTime(PositionTime),                 // type 1
    LatLonHeight(LatLonHeight),                 // type 2
    Ecef(Ecef),                                 // type 3
    LocalDatum(LocalDatum),                     // type 4
    EcefDelta(EcefDelta),                       // type 6
    TangentPlaneDelta(TangentPlaneDelta),       // type 7
    Velocity(Velocity),                         // type 8
    PdopInfo(PdopInfo),                         // type 9
    ClockInfo(ClockInfo),                       // type 10
    PositionVcvInfo(PositionVcvInfo),           // type 11
    PositionSigmaInfo(PositionSigmaInfo),       // type 12
    BriefSvInfo(BriefSvInfo),                   // type 13
    SvDetailedInfo(SvDetailedInfo),             // type 14
    ReceiverSerialNumber(ReceiverSerialNumber), // type 15
    UtcTime(UtcTime),                           // type 16
    AttitudeInfo(AttitudeInfo),                 // type 27
    AllBriefSvInfo(AllBriefSvInfo),             // type 33
    AllDetailedSvInfo(AllDetailedSvInfo),       // type 34
    ReceivedBaseInfo(ReceivedBaseInfo),         // type 35
    BatteryMemoryInfo(BatteryMemoryInfo),       // type 37
    PositionTypeInfo(PositionTypeInfo),         // type 38
    LbandStatus(LbandStatus),                   // type 40
    BasePositionQuality(BasePositionQuality),   // type 41
    InsFullNav(InsFullNav),                     // type 49
    InsRmsInfo(InsRmsInfo),                     // type 50
    DmiRawData(DmiRawData),                     // type 52
    InsVnavFullNav(InsVnavFullNav),             // type 63
    InsVnavRmsInfo(InsVnavRmsInfo),             // type 64
    Unknown {
        gsof_type: u8,
        length: u8,
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
            GsofRecord::ClockInfo(r) => write!(f, "{r}"),
            GsofRecord::PositionVcvInfo(r) => write!(f, "{r}"),
            GsofRecord::PositionSigmaInfo(r) => write!(f, "{r}"),
            GsofRecord::BriefSvInfo(r) => write!(f, "{r}"),
            GsofRecord::SvDetailedInfo(r) => write!(f, "{r}"),
            GsofRecord::ReceiverSerialNumber(r) => write!(f, "{r}"),
            GsofRecord::UtcTime(r) => write!(f, "{r}"),
            GsofRecord::AttitudeInfo(r) => write!(f, "{r}"),
            GsofRecord::AllBriefSvInfo(r) => write!(f, "{r}"),
            GsofRecord::AllDetailedSvInfo(r) => write!(f, "{r}"),
            GsofRecord::ReceivedBaseInfo(r) => write!(f, "{r}"),
            GsofRecord::BatteryMemoryInfo(r) => write!(f, "{r}"),
            GsofRecord::PositionTypeInfo(r) => write!(f, "{r}"),
            GsofRecord::LbandStatus(r) => write!(f, "{r}"),
            GsofRecord::BasePositionQuality(r) => write!(f, "{r}"),
            GsofRecord::InsFullNav(r) => write!(f, "{r}"),
            GsofRecord::InsRmsInfo(r) => write!(f, "{r}"),
            GsofRecord::DmiRawData(r) => write!(f, "{r}"),
            GsofRecord::InsVnavFullNav(r) => write!(f, "{r}"),
            GsofRecord::InsVnavRmsInfo(r) => write!(f, "{r}"),
            GsofRecord::Unknown { gsof_type, length } => {
                write!(f, "  GsofType:{gsof_type}  len:{length}")
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Type 1 — Position/Time
// ---------------------------------------------------------------------------

/// GSOF Record Type 1 — Position Time Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | u32 | GPS time of week \[ms\] |
/// | 4 | 2 | u16 | GPS week number |
/// | 6 | 1 | u8 | Number of SVs used |
/// | 7 | 1 | u8 | Position flags 1 |
/// | 8 | 1 | u8 | Position flags 2 |
/// | 9 | 1 | u8 | Initialization number |
///
/// flags1 bits:
///   0: new position, 1: clock fix only, 2: horizontal coords computed,
///   3: height computed, 5: least squares position, 7: L1 pseudo-range used
///
/// flags2 bits:
///   0: differential solution, 1: differential position is phase,
///   2: differential position has fixed integers, 3: OmniSTAR solution,
///   4: position constrained (static), 5: network RTK solution,
///   6: RTK location, 7: beacon DGPS
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionTime {
    /// GPS time of week \[ms\].
    pub milliseconds: u32,
    /// GPS week number.
    pub week_number: u16,
    /// Number of SVs used in solution.
    pub num_svs: u8,
    /// Position flags byte 1.
    pub flags1: u8,
    /// Position flags byte 2.
    pub flags2: u8,
    /// Initialization counter.
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

    /// flags1 bit 0: new position computed.
    pub fn is_new_pos(&self) -> bool { self.flags1 & (1 << 0) != 0 }
    /// flags1 bit 1: clock fix only (no position).
    pub fn is_clock_fix(&self) -> bool { self.flags1 & (1 << 1) != 0 }
    /// flags1 bit 2: horizontal coordinates computed.
    pub fn is_h_coordinates_computed(&self) -> bool { self.flags1 & (1 << 2) != 0 }
    /// flags1 bit 3: height computed.
    pub fn is_height_computed(&self) -> bool { self.flags1 & (1 << 3) != 0 }
    /// flags1 bit 5: least squares position.
    pub fn is_least_squares(&self) -> bool { self.flags1 & (1 << 5) != 0 }
    /// flags1 bit 7: L1 pseudo-range used.
    pub fn is_l1_pseudo_range_used(&self) -> bool { self.flags1 & (1 << 7) != 0 }
    /// flags2 bit 0: differential solution.
    pub fn is_diff_soln(&self) -> bool { self.flags2 & (1 << 0) != 0 }
    /// flags2 bit 1: differential position is in phase.
    pub fn is_diff_pos_in_phase(&self) -> bool { self.flags2 & (1 << 1) != 0 }
    /// flags2 bit 2: differential position has fixed integers.
    pub fn is_diff_pos_fixed_int(&self) -> bool { self.flags2 & (1 << 2) != 0 }
    /// flags2 bit 3: OmniSTAR solution.
    pub fn is_omnistar_soln(&self) -> bool { self.flags2 & (1 << 3) != 0 }
    /// flags2 bit 4: position constrained (static).
    pub fn is_static_constraint(&self) -> bool { self.flags2 & (1 << 4) != 0 }
    /// flags2 bit 5: network RTK solution.
    pub fn is_network_rtk_soln(&self) -> bool { self.flags2 & (1 << 5) != 0 }
    /// flags2 bit 6: RTK location.
    pub fn is_rtk_location(&self) -> bool { self.flags2 & (1 << 6) != 0 }
    /// flags2 bit 7: beacon DGPS.
    pub fn is_beacon_dgps(&self) -> bool { self.flags2 & (1 << 7) != 0 }
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

/// GSOF Record Type 2 — Latitude, Longitude, Height
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 8 | f64 | Latitude \[rad\] |
/// | 8 | 8 | f64 | Longitude \[rad\] |
/// | 16 | 8 | f64 | Height above ellipsoid \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LatLonHeight {
    /// Latitude \[deg\] (converted from radians on parse).
    pub lat_deg: f64,
    /// Longitude \[deg\] (converted from radians on parse).
    pub lon_deg: f64,
    /// Height above ellipsoid \[m\].
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

/// GSOF Record Type 3 — ECEF Position
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 8 | f64 | X coordinate \[m\] |
/// | 8 | 8 | f64 | Y coordinate \[m\] |
/// | 16 | 8 | f64 | Z coordinate \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ecef {
    /// ECEF X \[m\].
    pub x_m: f64,
    /// ECEF Y \[m\].
    pub y_m: f64,
    /// ECEF Z \[m\].
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

/// GSOF Record Type 4 — Local Datum Position
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 8 | \[u8;8\] | Datum ID string |
/// | 8 | 8 | f64 | Latitude \[rad\] |
/// | 16 | 8 | f64 | Longitude \[rad\] |
/// | 24 | 8 | f64 | Height above ellipsoid \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LocalDatum {
    /// Datum ID string (8 ASCII bytes).
    pub id: [u8; 8],
    /// Latitude \[deg\] (converted from radians on parse).
    pub lat_deg: f64,
    /// Longitude \[deg\] (converted from radians on parse).
    pub lon_deg: f64,
    /// Height above ellipsoid \[m\].
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

/// GSOF Record Type 6 — ECEF Delta
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 8 | f64 | Delta X \[m\] |
/// | 8 | 8 | f64 | Delta Y \[m\] |
/// | 16 | 8 | f64 | Delta Z \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EcefDelta {
    /// ECEF delta X \[m\].
    pub dx_m: f64,
    /// ECEF delta Y \[m\].
    pub dy_m: f64,
    /// ECEF delta Z \[m\].
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

/// GSOF Record Type 7 — Tangent Plane Delta
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 8 | f64 | East \[m\] |
/// | 8 | 8 | f64 | North \[m\] |
/// | 16 | 8 | f64 | Up \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TangentPlaneDelta {
    /// East component \[m\].
    pub east_m: f64,
    /// North component \[m\].
    pub north_m: f64,
    /// Up component \[m\].
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

/// GSOF Record Type 8 — Velocity Data
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | Velocity flags |
/// | 1 | 4 | f32 | Horizontal velocity \[m/s\] |
/// | 5 | 4 | f32 | Heading \[rad\] |
/// | 9 | 4 | f32 | Vertical velocity \[m/s\] |
/// | 13 | 4 | f32 | Local heading \[rad\] (optional, present when len >= 17) |
///
/// flags bits:
///   0: velocity data valid, 1: velocity source (0=Doppler, 1=consecutive),
///   2: heading data valid
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Velocity {
    /// Velocity flags byte.
    pub flags: u8,
    /// Horizontal velocity \[m/s\].
    pub velocity_ms: f32,
    /// Heading \[deg\] (converted from radians on parse).
    pub heading_deg: f32,
    /// Vertical velocity \[m/s\].
    pub vertical_ms: f32,
    /// Local heading \[deg\] (optional, converted from radians on parse).
    pub local_heading: Option<f32>,
}

impl Velocity {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 8);
        let flags = r.u8()?;
        let velocity_ms = r.f32()?;
        let heading_deg = r.f32()? * (180.0 / core::f32::consts::PI);
        let vertical_ms = r.f32()?;
        let local_heading = if r.remaining() >= 4 {
            Some(r.f32()? * (180.0 / core::f32::consts::PI))
        } else {
            None
        };
        Ok(Self {
            flags,
            velocity_ms,
            heading_deg,
            vertical_ms,
            local_heading,
        })
    }

    /// flags bit 0: velocity data is valid.
    pub fn is_vel_data_valid(&self) -> bool { self.flags & (1 << 0) != 0 }
    /// flags bit 1: velocity source.
    pub fn velocity_source(&self) -> VelocitySource {
        if self.flags & (1 << 1) != 0 {
            VelocitySource::ConsecutiveMeasurements
        } else {
            VelocitySource::Doppler
        }
    }
    /// flags bit 2: heading data is valid.
    pub fn is_heading_data_valid(&self) -> bool { self.flags & (1 << 2) != 0 }
}

impl fmt::Display for Velocity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let len = if self.local_heading.is_some() { 17 } else { 13 };
        writeln!(f, "  GsofType:8 - Velocity Data  len:{}", len)?;
        write!(
            f,
            "  Flags:{:02X}  velocity:{:.3}  heading:{:.3}  vertical:{:.3}",
            self.flags, self.velocity_ms, self.heading_deg, self.vertical_ms
        )?;
        if let Some(lh) = self.local_heading {
            write!(f, "  local_heading:{:.3}", lh)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 9 — PDOP
// ---------------------------------------------------------------------------

/// GSOF Record Type 9 — PDOP Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | f32 | PDOP |
/// | 4 | 4 | f32 | HDOP |
/// | 8 | 4 | f32 | VDOP |
/// | 12 | 4 | f32 | TDOP |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdopInfo {
    /// Position dilution of precision.
    pub pdop: f32,
    /// Horizontal dilution of precision.
    pub hdop: f32,
    /// Vertical dilution of precision.
    pub vdop: f32,
    /// Time dilution of precision.
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
// Type 10 — Clock Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 10 — Clock Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | Clock flags |
/// | 1 | 8 | f64 | Clock offset \[ms\] |
/// | 9 | 8 | f64 | Frequency offset \[ppm\] |
///
/// clock_flags bits:
///   0: clock offset valid, 1: frequency offset valid,
///   2: receiver in anywhere fix mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockInfo {
    /// Clock flags byte.
    pub clock_flags: u8,
    /// Clock offset \[ms\].
    pub clock_offset_ms: f64,
    /// Frequency offset \[ppm\].
    pub freq_offset_ppm: f64,
}

impl ClockInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 10);
        Ok(Self {
            clock_flags: r.u8()?,
            clock_offset_ms: r.f64()?,
            freq_offset_ppm: r.f64()?,
        })
    }

    /// clock_flags bit 0: clock offset is valid.
    pub fn is_clock_offset_valid(&self) -> bool { self.clock_flags & (1 << 0) != 0 }
    /// clock_flags bit 1: frequency offset is valid.
    pub fn is_freq_offset_valid(&self) -> bool { self.clock_flags & (1 << 1) != 0 }
    /// clock_flags bit 2: receiver is in anywhere fix mode.
    pub fn is_receiver_in_anywhere_fix_mode(&self) -> bool { self.clock_flags & (1 << 2) != 0 }
}

impl fmt::Display for ClockInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:10 - Clock Info  len:{}", 17)?;
        write!(
            f,
            "  Flags:{:02X}  ClockOffset:{:.6} ms  FreqOffset:{:.6} ppm",
            self.clock_flags, self.clock_offset_ms, self.freq_offset_ppm
        )
    }
}

// ---------------------------------------------------------------------------
// Type 11 — Position VCV Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 11 — Position Variance-Covariance Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | f32 | Position RMS \[m\] |
/// | 4 | 4 | f32 | VCV XX \[m^2\] |
/// | 8 | 4 | f32 | VCV XY \[m^2\] |
/// | 12 | 4 | f32 | VCV XZ \[m^2\] |
/// | 16 | 4 | f32 | VCV YY \[m^2\] |
/// | 20 | 4 | f32 | VCV YZ \[m^2\] |
/// | 24 | 4 | f32 | VCV ZZ \[m^2\] |
/// | 28 | 4 | f32 | Unit variance |
/// | 32 | 2 | u16 | Number of epochs |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionVcvInfo {
    /// Position RMS \[m\].
    pub position_rms: f32,
    /// Variance-covariance XX \[m^2\].
    pub vcv_xx: f32,
    /// Variance-covariance XY \[m^2\].
    pub vcv_xy: f32,
    /// Variance-covariance XZ \[m^2\].
    pub vcv_xz: f32,
    /// Variance-covariance YY \[m^2\].
    pub vcv_yy: f32,
    /// Variance-covariance YZ \[m^2\].
    pub vcv_yz: f32,
    /// Variance-covariance ZZ \[m^2\].
    pub vcv_zz: f32,
    /// Unit variance (a-posteriori).
    pub unit_variance: f32,
    /// Number of epochs in solution.
    pub num_epochs: u16,
}

impl PositionVcvInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 11);
        Ok(Self {
            position_rms: r.f32()?,
            vcv_xx: r.f32()?,
            vcv_xy: r.f32()?,
            vcv_xz: r.f32()?,
            vcv_yy: r.f32()?,
            vcv_yz: r.f32()?,
            vcv_zz: r.f32()?,
            unit_variance: r.f32()?,
            num_epochs: r.u16()?,
        })
    }
}

impl fmt::Display for PositionVcvInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:11 - Position VCV Info  len:{}", 34)?;
        writeln!(
            f,
            "  RMS:{:.4}  UnitVar:{:.4}  Epochs:{}",
            self.position_rms, self.unit_variance, self.num_epochs
        )?;
        write!(
            f,
            "  VCV xx:{:.4e} xy:{:.4e} xz:{:.4e} yy:{:.4e} yz:{:.4e} zz:{:.4e}",
            self.vcv_xx, self.vcv_xy, self.vcv_xz, self.vcv_yy, self.vcv_yz, self.vcv_zz
        )
    }
}

// ---------------------------------------------------------------------------
// Type 12 — Position Sigma Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 12 — Position Sigma Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | f32 | Position RMS \[m\] |
/// | 4 | 4 | f32 | Sigma East \[m\] |
/// | 8 | 4 | f32 | Sigma North \[m\] |
/// | 12 | 4 | f32 | Covariance East-North \[m^2\] |
/// | 16 | 4 | f32 | Sigma Up \[m\] |
/// | 20 | 4 | f32 | Semi-major axis \[m\] |
/// | 24 | 4 | f32 | Semi-minor axis \[m\] |
/// | 28 | 4 | f32 | Orientation \[deg\] |
/// | 32 | 4 | f32 | Unit variance |
/// | 36 | 2 | u16 | Number of epochs |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionSigmaInfo {
    /// Position RMS \[m\].
    pub position_rms: f32,
    /// Sigma East \[m\].
    pub sigma_east: f32,
    /// Sigma North \[m\].
    pub sigma_north: f32,
    /// Covariance East-North \[m^2\].
    pub covar_east_north: f32,
    /// Sigma Up \[m\].
    pub sigma_up: f32,
    /// Error ellipse semi-major axis \[m\].
    pub semi_major_axis: f32,
    /// Error ellipse semi-minor axis \[m\].
    pub semi_minor_axis: f32,
    /// Error ellipse orientation \[deg\].
    pub orientation_deg: f32,
    /// Unit variance (a-posteriori).
    pub unit_variance: f32,
    /// Number of epochs in solution.
    pub num_epochs: u16,
}

impl PositionSigmaInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 12);
        Ok(Self {
            position_rms: r.f32()?,
            sigma_east: r.f32()?,
            sigma_north: r.f32()?,
            covar_east_north: r.f32()?,
            sigma_up: r.f32()?,
            semi_major_axis: r.f32()?,
            semi_minor_axis: r.f32()?,
            orientation_deg: r.f32()?,
            unit_variance: r.f32()?,
            num_epochs: r.u16()?,
        })
    }
}

impl fmt::Display for PositionSigmaInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:12 - Position Sigma Info  len:{}", 38)?;
        writeln!(
            f,
            "  RMS:{:.4}  E:{:.4}  N:{:.4}  CovarEN:{:.4}  Up:{:.4}",
            self.position_rms,
            self.sigma_east,
            self.sigma_north,
            self.covar_east_north,
            self.sigma_up
        )?;
        write!(
            f,
            "  SemiMajor:{:.4}  SemiMinor:{:.4}  Orient:{:.1}  UnitVar:{:.4}  Epochs:{}",
            self.semi_major_axis,
            self.semi_minor_axis,
            self.orientation_deg,
            self.unit_variance,
            self.num_epochs
        )
    }
}

// ---------------------------------------------------------------------------
// Type 13 — Brief SV Info (GPS-only, older format)
// ---------------------------------------------------------------------------

/// Individual SV entry in Brief SV Info (Type 13).
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | PRN |
/// | 1 | 1 | u8 | flags1 |
/// | 2 | 1 | u8 | flags2 |
///
/// flags1 bits:
///   0: above horizon, 1: assigned to channel, 2: tracked
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BriefSv {
    /// Satellite PRN.
    pub prn: u8,
    /// SV flags byte 1.
    pub flags1: u8,
    /// SV flags byte 2.
    pub flags2: u8,
}

impl BriefSv {
    /// flags1 bit 0: SV is above the horizon.
    pub fn is_above_horizon(&self) -> bool { self.flags1 & (1 << 0) != 0 }
    /// flags1 bit 1: SV is assigned to a channel.
    pub fn is_assigned_to_channel(&self) -> bool { self.flags1 & (1 << 1) != 0 }
    /// flags1 bit 2: SV is being tracked.
    pub fn is_tracked(&self) -> bool { self.flags1 & (1 << 2) != 0 }
}

/// GSOF Record Type 13 — Brief SV Info (GPS-only)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | SV count |
/// | 1 | 3*N | ... | Per-SV data (3 bytes each) |
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BriefSvInfo {
    /// Collection of brief SV entries.
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

/// Individual SV entry in Detailed SV Info (Type 14).
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | PRN |
/// | 1 | 1 | u8 | flags1 |
/// | 2 | 1 | u8 | flags2 |
/// | 3 | 1 | u8 | Elevation \[deg\] |
/// | 4 | 2 | u16 | Azimuth \[deg\] |
/// | 6 | 1 | u8 | L1 SNR \[dB-Hz * 4\] |
/// | 7 | 1 | u8 | L2 SNR \[dB-Hz * 4\] |
///
/// flags1 bits:
///   0: above horizon, 1: assigned to channel, 2: tracked single freq,
///   3: tracked dual freq, 4: reported at base L1, 5: reported at base L2,
///   6: used in position, 7: used in RTK
///
/// flags2 bits:
///   0: tracking P-code L1, 1: tracking P-code L2
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SvDetail {
    /// Satellite PRN.
    pub prn: u8,
    /// SV flags byte 1.
    pub flags1: u8,
    /// SV flags byte 2.
    pub flags2: u8,
    /// Elevation angle \[deg\].
    pub elevation_deg: u8,
    /// Azimuth angle \[deg\].
    pub azimuth_deg: u16,
    /// L1 signal-to-noise ratio \[dB-Hz * 4\].
    pub l1_snr: u8,
    /// L2 signal-to-noise ratio \[dB-Hz * 4\].
    pub l2_snr: u8,
}

impl SvDetail {
    /// flags1 bit 0: SV is above the horizon.
    pub fn is_above_horizon(&self) -> bool { self.flags1 & (1 << 0) != 0 }
    /// flags1 bit 1: SV is assigned to a channel.
    pub fn is_assigned_to_channel(&self) -> bool { self.flags1 & (1 << 1) != 0 }
    /// flags1 bit 2: SV is tracked on single frequency.
    pub fn is_tracked_single_freq(&self) -> bool { self.flags1 & (1 << 2) != 0 }
    /// flags1 bit 3: SV is tracked on dual frequency.
    pub fn is_tracked_dual_freq(&self) -> bool { self.flags1 & (1 << 3) != 0 }
    /// flags1 bit 4: SV is reported at base station L1.
    pub fn is_reported_at_base_l1(&self) -> bool { self.flags1 & (1 << 4) != 0 }
    /// flags1 bit 5: SV is reported at base station L2.
    pub fn is_reported_at_base_l2(&self) -> bool { self.flags1 & (1 << 5) != 0 }
    /// flags1 bit 6: SV is used in position solution.
    pub fn is_used_in_position(&self) -> bool { self.flags1 & (1 << 6) != 0 }
    /// flags1 bit 7: SV is used in RTK solution.
    pub fn is_used_in_rtk(&self) -> bool { self.flags1 & (1 << 7) != 0 }
    /// flags2 bit 0: tracking P-code on L1.
    pub fn is_tracking_p_code_l1(&self) -> bool { self.flags2 & (1 << 0) != 0 }
    /// flags2 bit 1: tracking P-code on L2.
    pub fn is_tracking_p_code_l2(&self) -> bool { self.flags2 & (1 << 1) != 0 }
}

/// GSOF Record Type 14 — SV Detailed Info (GPS-only)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | SV count |
/// | 1 | 8*N | ... | Per-SV data (8 bytes each) |
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SvDetailedInfo {
    /// Collection of detailed SV entries.
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
// Type 15 — Receiver Serial Number
// ---------------------------------------------------------------------------

/// GSOF Record Type 15 — Receiver Serial Number
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | i32 | Serial number |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceiverSerialNumber {
    /// Receiver serial number.
    pub serial_number: i32,
}

impl ReceiverSerialNumber {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 15);
        Ok(Self {
            serial_number: r.i32()?,
        })
    }
}

impl fmt::Display for ReceiverSerialNumber {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:15 - Receiver Serial Number  len:{}", 4)?;
        write!(f, "  Serial:{}", self.serial_number)
    }
}

// ---------------------------------------------------------------------------
// Type 16 — UTC Time
// ---------------------------------------------------------------------------

/// GSOF Record Type 16 — UTC Time Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | u32 | GPS time of week \[ms\] |
/// | 4 | 2 | u16 | GPS week number |
/// | 6 | 2 | i16 | UTC offset \[s\] |
/// | 8 | 1 | u8 | Time flags |
///
/// flags bits:
///   0: time information valid, 1: UTC offset valid
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UtcTime {
    /// GPS time of week \[ms\].
    pub milliseconds: u32,
    /// GPS week number.
    pub week_number: u16,
    /// UTC offset from GPS time \[s\].
    pub utc_offset_s: i16,
    /// Time flags byte.
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

    /// flags bit 0: time information is valid.
    pub fn is_time_info_valid(&self) -> bool { self.flags & (1 << 0) != 0 }
    /// flags bit 1: UTC offset is valid.
    pub fn is_utc_offset_valid(&self) -> bool { self.flags & (1 << 1) != 0 }
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
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 42 | 4 | f32 | Pitch variance \[rad^2\] |
/// | 46 | 4 | f32 | Yaw variance \[rad^2\] |
/// | 50 | 4 | f32 | Roll variance \[rad^2\] |
/// | 54 | 4 | f32 | Pitch-Yaw covariance \[rad^2\] |
/// | 58 | 4 | f32 | Pitch-Roll covariance \[rad^2\] |
/// | 62 | 4 | f32 | Yaw-Roll covariance \[rad^2\] |
/// | 66 | 4 | f32 | Range variance \[m^2\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AttitudeVariance {
    /// Pitch variance \[rad^2\].
    pub pitch_var_rad2: f32,
    /// Yaw variance \[rad^2\].
    pub yaw_var_rad2: f32,
    /// Roll variance \[rad^2\].
    pub roll_var_rad2: f32,
    /// Pitch-Yaw covariance \[rad^2\].
    pub pitch_yaw_covar_rad2: f32,
    /// Pitch-Roll covariance \[rad^2\].
    pub pitch_roll_covar_rad2: f32,
    /// Yaw-Roll covariance \[rad^2\].
    pub yaw_roll_covar_rad2: f32,
    /// Range variance \[m^2\].
    pub range_var_m2: f32,
}

/// GSOF Record Type 27 — Attitude Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | u32 | GPS time \[ms\] |
/// | 4 | 1 | u8 | Attitude flags |
/// | 5 | 1 | u8 | Number of SVs |
/// | 6 | 1 | u8 | Computation mode |
/// | 7 | 1 | u8 | Reserved |
/// | 8 | 8 | f64 | Pitch \[rad\] |
/// | 16 | 8 | f64 | Yaw \[rad\] |
/// | 24 | 8 | f64 | Roll \[rad\] |
/// | 32 | 8 | f64 | Master-slave range \[m\] |
/// | 40 | 2 | u16 | PDOP * 10 |
/// | 42-69 | 28 | 7xf32 | Variance (optional) |
///
/// flags bits:
///   0: calibrated, 1: pitch valid, 2: yaw valid, 3: roll valid,
///   4: scalar valid, 5: diag valid, 6: slave static, 7: error stats valid
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AttitudeInfo {
    /// GPS time of week \[s\] (from ms / 1000.0).
    pub gps_time_s: f64,
    /// Attitude flags byte.
    pub flags: u8,
    /// Number of SVs used.
    pub num_svs: u8,
    /// Computation mode.
    pub mode: u8,
    /// Pitch \[deg\] (converted from radians on parse).
    pub pitch_deg: f64,
    /// Yaw \[deg\] (converted from radians on parse).
    pub yaw_deg: f64,
    /// Roll \[deg\] (converted from radians on parse).
    pub roll_deg: f64,
    /// Master-slave range \[m\].
    pub range_m: f64,
    /// Position dilution of precision.
    pub pdop: f64,
    /// Variance/covariance extension (present when record length > 42).
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

    /// flags bit 0: attitude is calibrated.
    pub fn is_calibrated(&self) -> bool { self.flags & (1 << 0) != 0 }
    /// flags bit 1: pitch value is valid.
    pub fn is_pitch_valid(&self) -> bool { self.flags & (1 << 1) != 0 }
    /// flags bit 2: yaw value is valid.
    pub fn is_yaw_valid(&self) -> bool { self.flags & (1 << 2) != 0 }
    /// flags bit 3: roll value is valid.
    pub fn is_roll_valid(&self) -> bool { self.flags & (1 << 3) != 0 }
    /// flags bit 4: scalar value is valid.
    pub fn is_scalar_valid(&self) -> bool { self.flags & (1 << 4) != 0 }
    /// flags bit 5: diagnostic is valid.
    pub fn is_diag_valid(&self) -> bool { self.flags & (1 << 5) != 0 }
    /// flags bit 6: slave antenna is static.
    pub fn is_slave_static(&self) -> bool { self.flags & (1 << 6) != 0 }
    /// flags bit 7: error statistics are valid.
    pub fn is_err_stats_valid(&self) -> bool { self.flags & (1 << 7) != 0 }
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

/// Individual SV entry in All Brief SV Info (Type 33).
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | PRN |
/// | 1 | 1 | u8 | Satellite system |
/// | 2 | 1 | u8 | flags1 |
/// | 3 | 1 | u8 | flags2 |
///
/// flags1 bits:
///   0: above horizon, 1: assigned to channel, 2: tracked
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllBriefSv {
    /// Satellite PRN.
    pub prn: u8,
    /// Satellite system identifier.
    pub system: u8,
    /// SV flags byte 1.
    pub flags1: u8,
    /// SV flags byte 2.
    pub flags2: u8,
}

impl AllBriefSv {
    /// flags1 bit 0: SV is above the horizon.
    pub fn is_above_horizon(&self) -> bool { self.flags1 & (1 << 0) != 0 }
    /// flags1 bit 1: SV is assigned to a channel.
    pub fn is_assigned_to_channel(&self) -> bool { self.flags1 & (1 << 1) != 0 }
    /// flags1 bit 2: SV is being tracked.
    pub fn is_tracked(&self) -> bool { self.flags1 & (1 << 2) != 0 }
    /// Satellite system type enum.
    pub fn satellite_type(&self) -> SatelliteType { SatelliteType::from(self.system) }
}

/// GSOF Record Type 33 — All Brief SV Info (multi-constellation)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | SV count |
/// | 1 | 4*N | ... | Per-SV data (4 bytes each) |
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllBriefSvInfo {
    /// Collection of brief SV entries.
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

/// Individual SV entry in All Detailed SV Info (Type 34).
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | PRN |
/// | 1 | 1 | u8 | Satellite system |
/// | 2 | 1 | u8 | flags1 |
/// | 3 | 1 | u8 | flags2 |
/// | 4 | 1 | u8 | Elevation \[deg\] |
/// | 5 | 2 | u16 | Azimuth \[deg\] |
/// | 7 | 3 | \[u8;3\] | SNR (L1/E1, L2/N/A, L5/E5/G1P) \[dB-Hz * 4\] |
///
/// flags1 bits:
///   0: above horizon, 1: assigned to channel, 2: tracked single freq,
///   3: tracked dual freq, 4: reported at base L1, 5: reported at base L2,
///   6: used in position, 7: used in RTK
///
/// flags2 bits:
///   0: tracking P-code L1, 1: tracking P-code L2
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllDetailedSv {
    /// Satellite PRN.
    pub prn: u8,
    /// Satellite system identifier.
    pub system: u8,
    /// SV flags byte 1.
    pub flags1: u8,
    /// SV flags byte 2.
    pub flags2: u8,
    /// Elevation angle \[deg\].
    pub elevation_deg: u8,
    /// Azimuth angle \[deg\].
    pub azimuth_deg: u16,
    /// SNR values \[dB-Hz * 4\]: L1/E1, L2/N/A, L5/E5/G1P.
    pub snr: [u8; 3],
}

impl AllDetailedSv {
    /// flags1 bit 0: SV is above the horizon.
    pub fn is_above_horizon(&self) -> bool { self.flags1 & (1 << 0) != 0 }
    /// flags1 bit 1: SV is assigned to a channel.
    pub fn is_assigned_to_channel(&self) -> bool { self.flags1 & (1 << 1) != 0 }
    /// flags1 bit 2: SV is tracked on single frequency.
    pub fn is_tracked_single_freq(&self) -> bool { self.flags1 & (1 << 2) != 0 }
    /// flags1 bit 3: SV is tracked on dual frequency.
    pub fn is_tracked_dual_freq(&self) -> bool { self.flags1 & (1 << 3) != 0 }
    /// flags1 bit 4: SV is reported at base station L1.
    pub fn is_reported_at_base_l1(&self) -> bool { self.flags1 & (1 << 4) != 0 }
    /// flags1 bit 5: SV is reported at base station L2.
    pub fn is_reported_at_base_l2(&self) -> bool { self.flags1 & (1 << 5) != 0 }
    /// flags1 bit 6: SV is used in position solution.
    pub fn is_used_in_position(&self) -> bool { self.flags1 & (1 << 6) != 0 }
    /// flags1 bit 7: SV is used in RTK solution.
    pub fn is_used_in_rtk(&self) -> bool { self.flags1 & (1 << 7) != 0 }
    /// flags2 bit 0: tracking P-code on L1.
    pub fn is_tracking_p_code_l1(&self) -> bool { self.flags2 & (1 << 0) != 0 }
    /// flags2 bit 1: tracking P-code on L2.
    pub fn is_tracking_p_code_l2(&self) -> bool { self.flags2 & (1 << 1) != 0 }
    /// Satellite system type enum.
    pub fn satellite_type(&self) -> SatelliteType { SatelliteType::from(self.system) }
}

/// GSOF Record Type 34 — All Detailed SV Info (multi-constellation)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | SV count |
/// | 1 | 10*N | ... | Per-SV data (10 bytes each) |
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllDetailedSvInfo {
    /// Collection of detailed SV entries.
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
// Type 35 — Received Base Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 35 — Received Base Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 1 | u8 | Flags |
/// | 1 | 8 | \[u8;8\] | Base name (ASCII) |
/// | 9 | 2 | u16 | Base station ID |
/// | 11 | 8 | f64 | Latitude \[rad\] |
/// | 19 | 8 | f64 | Longitude \[rad\] |
/// | 27 | 8 | f64 | Height above ellipsoid \[m\] |
///
/// flags:
///   bits 0-2: version number, bit 3: base info valid
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedBaseInfo {
    /// Flags byte.
    pub flags: u8,
    /// Base station name (8 ASCII bytes).
    pub name: [u8; 8],
    /// Base station ID.
    pub base_id: u16,
    /// Latitude \[deg\] (converted from radians on parse).
    pub lat_deg: f64,
    /// Longitude \[deg\] (converted from radians on parse).
    pub lon_deg: f64,
    /// Height above ellipsoid \[m\].
    pub height_m: f64,
}

impl ReceivedBaseInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 35);
        let flags = r.u8()?;
        let name = r.take::<8>()?;
        Ok(Self {
            flags,
            name,
            base_id: r.u16()?,
            lat_deg: r.f64()? * RAD_TO_DEG,
            lon_deg: r.f64()? * RAD_TO_DEG,
            height_m: r.f64()?,
        })
    }

    pub fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name).unwrap_or("????????")
    }

    /// Version number from bits 0-2 of flags.
    pub fn version_number(&self) -> u8 { self.flags & 0x07 }
    /// flags bit 3: base information is valid.
    pub fn is_base_info_valid(&self) -> bool { self.flags & (1 << 3) != 0 }
}

impl fmt::Display for ReceivedBaseInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:35 - Received Base Info  len:{}", 35)?;
        write!(
            f,
            "  Flags:{:02X}  Name:{}  Id:{}  Lat:{:.7}  Lon:{:.7}  Height:{:.3}",
            self.flags,
            self.name_str(),
            self.base_id,
            self.lat_deg,
            self.lon_deg,
            self.height_m
        )
    }
}

// ---------------------------------------------------------------------------
// Type 37 — Battery/Memory Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 37 — Battery/Memory Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | Battery capacity \[%\] |
/// | 2 | 8 | f64 | Remaining time \[s\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryMemoryInfo {
    /// Battery capacity \[%\].
    pub battery_capacity: u16,
    /// Remaining battery time \[s\].
    pub remaining_time: f64,
}

impl BatteryMemoryInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 37);
        Ok(Self {
            battery_capacity: r.u16()?,
            remaining_time: r.f64()?,
        })
    }
}

impl fmt::Display for BatteryMemoryInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:37 - Battery/Memory Info  len:{}", 10)?;
        write!(
            f,
            "  BatteryCap:{}  RemainingTime:{:.1}",
            self.battery_capacity, self.remaining_time
        )
    }
}

// ---------------------------------------------------------------------------
// Type 38 — Position Type Info
// ---------------------------------------------------------------------------

/// Firmware >=4.40 extension fields for PositionTypeInfo.
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 4 | 1 | u8 | Solution flags |
/// | 5 | 1 | u8 | RTK condition code |
/// | 6 | 4 | f32 | Correction age \[s\] |
/// | 10 | 1 | u8 | Network flags |
/// | 11 | 1 | u8 | Network flags 2 |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionTypeExt440 {
    /// Solution flags byte.
    pub solution_flags: u8,
    /// RTK condition code.
    pub rtk_condition: u8,
    /// Correction age \[s\].
    pub correction_age: f32,
    /// Network flags byte 1.
    pub network_flags: u8,
    /// Network flags byte 2.
    pub network_flags2: u8,
}

/// Firmware >=4.82 extension fields for PositionTypeInfo.
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 12 | 1 | u8 | Frame flag |
/// | 13 | 2 | i16 | ITRF epoch |
/// | 15 | 1 | u8 | Tectonic plate |
/// | 16 | 4 | i32 | RTX RAM subscription minutes left |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionTypeExt482 {
    /// Reference frame flag.
    pub frame_flag: u8,
    /// ITRF epoch.
    pub itrf_epoch: i16,
    /// Tectonic plate identifier.
    pub tectonic_plate: u8,
    /// RTX RAM subscription minutes remaining.
    pub rtx_ram_sub_minutes_left: i32,
}

/// Firmware >=4.90 extension fields for PositionTypeInfo.
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 20 | 1 | u8 | Pole wobble status |
/// | 21 | 4 | f32 | Pole wobble distance \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionTypeExt490 {
    /// Pole wobble correction status.
    pub pole_wobble_status: u8,
    /// Pole wobble distance \[m\].
    pub pole_wobble_distance: f32,
}

/// GSOF Record Type 38 — Position Type Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | f32 | Error scale |
/// | 4+ | var | ... | Firmware extensions (see ext structs) |
/// | 25 | 1 | u8 | Position fix type (fw >=4.94) |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PositionTypeInfo {
    /// Error scale factor.
    pub error_scale: f32,
    /// Firmware >=4.40 extension.
    pub ext_440: Option<PositionTypeExt440>,
    /// Firmware >=4.82 extension.
    pub ext_482: Option<PositionTypeExt482>,
    /// Firmware >=4.90 extension.
    pub ext_490: Option<PositionTypeExt490>,
    /// Position fix type (firmware >=4.94).
    pub position_fix_type: Option<u8>,
}

impl PositionTypeInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 38);
        let error_scale = r.f32()?;

        // fw 4.40 fields (8 bytes: 1+1+4+1+1)
        let ext_440 = if data.len() > 4 {
            Some(PositionTypeExt440 {
                solution_flags: r.u8()?,
                rtk_condition: r.u8()?,
                correction_age: r.f32()?,
                network_flags: r.u8()?,
                network_flags2: r.u8()?,
            })
        } else {
            None
        };

        // fw 4.82 fields (8 bytes: 1+2+1+4)
        let ext_482 = if data.len() > 12 {
            Some(PositionTypeExt482 {
                frame_flag: r.u8()?,
                itrf_epoch: r.i16()?,
                tectonic_plate: r.u8()?,
                rtx_ram_sub_minutes_left: r.i32()?,
            })
        } else {
            None
        };

        // fw 4.90 fields (5 bytes: 1+4)
        let ext_490 = if data.len() > 20 {
            Some(PositionTypeExt490 {
                pole_wobble_status: r.u8()?,
                pole_wobble_distance: r.f32()?,
            })
        } else {
            None
        };

        // fw 4.94 field (1 byte)
        let position_fix_type = if data.len() > 25 { Some(r.u8()?) } else { None };

        Ok(Self {
            error_scale,
            ext_440,
            ext_482,
            ext_490,
            position_fix_type,
        })
    }

    /// solution_flags bit 0: wide area solution.
    pub fn is_wide_area_solution(&self) -> bool {
        self.ext_440.map_or(false, |e| e.solution_flags & (1 << 0) != 0)
    }
    /// solution_flags bit 1: RTK fix solution.
    pub fn is_rtk_fix_solution(&self) -> bool {
        self.ext_440.map_or(false, |e| e.solution_flags & (1 << 1) != 0)
    }
    /// Solution integrity from bits 2-3 of solution_flags.
    pub fn solution_integrity(&self) -> SolutionIntegrity {
        self.ext_440.map_or(SolutionIntegrity::NotChecking, |e| {
            SolutionIntegrity::from_solution_flags(e.solution_flags)
        })
    }
    /// RTK condition code as enum.
    pub fn rtk_condition(&self) -> RtkCondition {
        self.ext_440.map_or(RtkCondition::NewPositionComputed, |e| {
            RtkCondition::from(e.rtk_condition)
        })
    }
    /// network_flags bit 0: new physical base station.
    pub fn is_new_physical_base_station(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 0) != 0)
    }
    /// RTCM status from bits 1-2 of network_flags.
    pub fn rtcm_status(&self) -> RtcmStatus {
        self.ext_440.map_or(RtcmStatus::NotAvailableOrUnknown, |e| {
            RtcmStatus::from_network_flags(e.network_flags)
        })
    }
    /// network_flags bit 3: geofence triggered.
    pub fn is_geofence_triggered(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 3) != 0)
    }
    /// network_flags bit 4: RTK range limit exceeded.
    pub fn is_rtk_range_limit_exceeded(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 4) != 0)
    }
    /// network_flags bit 5: xFill position.
    pub fn is_xfill_position(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 5) != 0)
    }
    /// network_flags bit 6: RTX position.
    pub fn is_rtx_position(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 6) != 0)
    }
    /// network_flags bit 7: RTX or xFill link down.
    pub fn is_rtx_or_xfill_link_down(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags & (1 << 7) != 0)
    }
    /// network_flags2 bit 0: xFill ready.
    pub fn is_xfill_ready(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags2 & (1 << 0) != 0)
    }
    /// network_flags2 bit 1: RTX solution in rain.
    pub fn is_rtx_solution_rain(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags2 & (1 << 1) != 0)
    }
    /// network_flags2 bit 2: xFill RTX offset good.
    pub fn is_xfill_rtx_offset_good(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags2 & (1 << 2) != 0)
    }
    /// network_flags2 bit 3: CMRxE received.
    pub fn is_cmrxe_received(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags2 & (1 << 3) != 0)
    }
    /// network_flags2 bit 4: RTX in wet area.
    pub fn is_rtx_in_wet_area(&self) -> bool {
        self.ext_440.map_or(false, |e| e.network_flags2 & (1 << 4) != 0)
    }
    /// Position fix type as enum.
    pub fn position_fix_type(&self) -> PositionFix {
        self.position_fix_type.map_or(PositionFix::None, PositionFix::from)
    }
}

impl fmt::Display for PositionTypeInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:38 - Position Type Info")?;
        write!(f, "  ErrorScale:{:.4}", self.error_scale)?;
        if let Some(e) = &self.ext_440 {
            write!(
                f,
                "\n  SolFlags:{:02X}  RTKcond:{}  CorrAge:{:.1}  NetFlags:{:02X} {:02X}",
                e.solution_flags,
                e.rtk_condition,
                e.correction_age,
                e.network_flags,
                e.network_flags2
            )?;
        }
        if let Some(e) = &self.ext_482 {
            write!(
                f,
                "\n  FrameFlag:{}  ITRF:{}  TecPlate:{}  RTXminLeft:{}",
                e.frame_flag, e.itrf_epoch, e.tectonic_plate, e.rtx_ram_sub_minutes_left
            )?;
        }
        if let Some(e) = &self.ext_490 {
            write!(
                f,
                "\n  PoleWobbleStat:{}  PoleWobbleDist:{:.3}",
                e.pole_wobble_status, e.pole_wobble_distance
            )?;
        }
        if let Some(ft) = self.position_fix_type {
            write!(f, "\n  PosFixType:{}", ft)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 40 — L-band Status
// ---------------------------------------------------------------------------

/// GSOF Record Type 40 — L-band Status (OmniSTAR)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 5 | \[u8;5\] | Satellite name (ASCII) |
/// | 5 | 4 | f32 | Frequency \[Hz\] |
/// | 9 | 2 | u16 | Bit rate \[bps\] |
/// | 11 | 4 | f32 | Signal-to-noise ratio \[dB\] |
/// | 15 | 1 | u8 | HP/XP subscribed engine |
/// | 16 | 1 | u8 | HP/XP library mode |
/// | 17 | 1 | u8 | VBS library mode |
/// | 18 | 1 | u8 | Beam mode (0=manual, 1=auto) |
/// | 19 | 1 | u8 | OmniSTAR motion state |
/// | 20 | 4 | f32 | Horizontal precision threshold |
/// | 24 | 4 | f32 | Vertical precision threshold |
/// | 28 | 1 | u8 | NMEA encryption (0=disabled, 1=enabled) |
/// | 29 | 4 | f32 | I/Q ratio |
/// | 33 | 4 | f32 | Estimated BER |
/// | 37 | 4 | u32 | Total unique words |
/// | 41 | 4 | u32 | Total bad unique words |
/// | 45 | 4 | u32 | Total bad unique word bits |
/// | 49 | 4 | u32 | Total Viterbi corrections |
/// | 53 | 4 | u32 | Total bad Viterbi corrections |
/// | 57 | 4 | u32 | Total bad messages |
/// | 61 | 1 | u8 | Measured freq valid (optional) |
/// | 62 | 8 | f64 | Measured frequency \[Hz\] (optional) |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LbandStatus {
    /// Satellite name (5 ASCII bytes).
    pub name: [u8; 5],
    /// Frequency \[Hz\].
    pub freq_hz: f32,
    /// Bit rate \[bps\].
    pub bit_rate_bps: u16,
    /// Signal-to-noise ratio \[dB\].
    pub snr: f32,
    /// HP/XP subscribed engine.
    pub hp_xp_subscribed_engine: u8,
    /// HP/XP library mode.
    pub hp_xp_library_mode: u8,
    /// VBS library mode.
    pub vbs_library_mode: u8,
    /// Beam mode (0=manual, 1=auto).
    pub beam_mode: u8,
    /// OmniSTAR motion state.
    pub omnistar_motion: u8,
    /// Horizontal precision threshold.
    pub horiz_prec_thresh: f32,
    /// Vertical precision threshold.
    pub vert_prec_thresh: f32,
    /// NMEA encryption state (0=disabled, 1=enabled).
    pub nmea_encryption: u8,
    /// I/Q ratio.
    pub iq_ratio: f32,
    /// Estimated bit error rate.
    pub est_ber: f32,
    /// Total unique words received.
    pub total_uw: u32,
    /// Total bad unique words.
    pub total_bad_uw: u32,
    /// Total bad unique word bits.
    pub total_bad_uw_bits: u32,
    /// Total Viterbi corrections.
    pub total_viterbi: u32,
    /// Total bad Viterbi corrections.
    pub total_bad_viterbi: u32,
    /// Total bad messages.
    pub total_bad_messages: u32,
    /// Measured frequency (valid_flag, freq_hz), present when record length > 61.
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

    /// HP/XP subscribed engine as enum.
    pub fn hp_xp_engine(&self) -> HpXpEngine { HpXpEngine::from(self.hp_xp_subscribed_engine) }
    /// HP/XP library mode as enum.
    pub fn hp_xp_library_mode(&self) -> HpXpLibraryMode { HpXpLibraryMode::from(self.hp_xp_library_mode) }
    /// VBS library mode as enum.
    pub fn vbs_library_mode(&self) -> VbsLibraryMode { VbsLibraryMode::from(self.vbs_library_mode) }
    /// Beam mode as enum.
    pub fn beam_mode(&self) -> BeamMode { BeamMode::from(self.beam_mode) }
    /// Motion state as enum.
    pub fn motion_state(&self) -> MotionState { MotionState::from(self.omnistar_motion) }
    /// NMEA encryption state as enum.
    pub fn nmea_encryption_state(&self) -> NmeaEncryptionState { NmeaEncryptionState::from(self.nmea_encryption) }
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
// Type 41 — Base Position and Quality
// ---------------------------------------------------------------------------

/// GSOF Record Type 41 — Base Position and Quality
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 4 | u32 | GPS time of week \[ms\] |
/// | 4 | 2 | u16 | GPS week number |
/// | 6 | 8 | f64 | Latitude \[rad\] |
/// | 14 | 8 | f64 | Longitude \[rad\] |
/// | 22 | 8 | f64 | Height above ellipsoid \[m\] |
/// | 30 | 1 | u8 | Quality indicator |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BasePositionQuality {
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// GPS week number.
    pub gps_week: u16,
    /// Latitude \[deg\] (converted from radians on parse).
    pub lat_deg: f64,
    /// Longitude \[deg\] (converted from radians on parse).
    pub lon_deg: f64,
    /// Height above ellipsoid \[m\].
    pub height_m: f64,
    /// Quality indicator.
    pub quality: u8,
}

impl BasePositionQuality {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 41);
        Ok(Self {
            gps_time_ms: r.u32()?,
            gps_week: r.u16()?,
            lat_deg: r.f64()? * RAD_TO_DEG,
            lon_deg: r.f64()? * RAD_TO_DEG,
            height_m: r.f64()?,
            quality: r.u8()?,
        })
    }

    /// Base quality indicator as enum.
    pub fn base_quality(&self) -> BaseQuality { BaseQuality::from(self.quality) }
}

impl fmt::Display for BasePositionQuality {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:41 - Base Position Quality  len:{}", 31)?;
        write!(
            f,
            "  Week:{}  Time:{}ms  Lat:{:.7}  Lon:{:.7}  Height:{:.3}  Quality:{}",
            self.gps_week,
            self.gps_time_ms,
            self.lat_deg,
            self.lon_deg,
            self.height_m,
            self.quality
        )
    }
}

// ---------------------------------------------------------------------------
// Type 49 — INS Full Navigation Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 49 — INS Full Navigation Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | GPS week number |
/// | 2 | 4 | u32 | GPS time of week \[ms\] |
/// | 6 | 1 | u8 | IMU alignment status |
/// | 7 | 1 | u8 | GNSS quality indicator |
/// | 8 | 8 | f64 | Latitude \[deg\] |
/// | 16 | 8 | f64 | Longitude \[deg\] |
/// | 24 | 8 | f64 | Altitude \[m\] |
/// | 32 | 4 | f32 | North velocity \[m/s\] |
/// | 36 | 4 | f32 | East velocity \[m/s\] |
/// | 40 | 4 | f32 | Down velocity \[m/s\] |
/// | 44 | 4 | f32 | Total speed \[m/s\] |
/// | 48 | 8 | f64 | Roll \[deg\] |
/// | 56 | 8 | f64 | Pitch \[deg\] |
/// | 64 | 8 | f64 | Heading \[deg\] |
/// | 72 | 8 | f64 | Track angle \[deg\] |
/// | 80 | 4 | f32 | Angular rate X \[rad/s\] |
/// | 84 | 4 | f32 | Angular rate Y \[rad/s\] |
/// | 88 | 4 | f32 | Angular rate Z \[rad/s\] |
/// | 92 | 4 | f32 | Acceleration X \[m/s^2\] |
/// | 96 | 4 | f32 | Acceleration Y \[m/s^2\] |
/// | 100 | 4 | f32 | Acceleration Z \[m/s^2\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InsFullNav {
    /// GPS week number.
    pub gps_week: u16,
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// IMU alignment status.
    pub imu_alignment_status: u8,
    /// GNSS quality indicator.
    pub gps_quality: u8,
    /// Latitude \[deg\].
    pub lat_deg: f64,
    /// Longitude \[deg\].
    pub lon_deg: f64,
    /// Altitude \[m\].
    pub alt_m: f64,
    /// North velocity \[m/s\].
    pub north_vel_ms: f32,
    /// East velocity \[m/s\].
    pub east_vel_ms: f32,
    /// Down velocity \[m/s\].
    pub down_vel_ms: f32,
    /// Total speed \[m/s\].
    pub total_speed_ms: f32,
    /// Roll \[deg\].
    pub roll_deg: f64,
    /// Pitch \[deg\].
    pub pitch_deg: f64,
    /// Heading \[deg\].
    pub heading_deg: f64,
    /// Track angle \[deg\].
    pub track_angle_deg: f64,
    /// Angular rate X \[rad/s\].
    pub angular_rate_x: f32,
    /// Angular rate Y \[rad/s\].
    pub angular_rate_y: f32,
    /// Angular rate Z \[rad/s\].
    pub angular_rate_z: f32,
    /// Acceleration X \[m/s^2\].
    pub accel_x: f32,
    /// Acceleration Y \[m/s^2\].
    pub accel_y: f32,
    /// Acceleration Z \[m/s^2\].
    pub accel_z: f32,
}

impl InsFullNav {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 49);
        Ok(Self {
            gps_week: r.u16()?,
            gps_time_ms: r.u32()?,
            imu_alignment_status: r.u8()?,
            gps_quality: r.u8()?,
            lat_deg: r.f64()?,
            lon_deg: r.f64()?,
            alt_m: r.f64()?,
            north_vel_ms: r.f32()?,
            east_vel_ms: r.f32()?,
            down_vel_ms: r.f32()?,
            total_speed_ms: r.f32()?,
            roll_deg: r.f64()?,
            pitch_deg: r.f64()?,
            heading_deg: r.f64()?,
            track_angle_deg: r.f64()?,
            angular_rate_x: r.f32()?,
            angular_rate_y: r.f32()?,
            angular_rate_z: r.f32()?,
            accel_x: r.f32()?,
            accel_y: r.f32()?,
            accel_z: r.f32()?,
        })
    }

    /// IMU alignment status as enum.
    pub fn alignment_status(&self) -> ImuAlignmentStatus { ImuAlignmentStatus::from(self.imu_alignment_status) }
    /// GNSS quality as enum.
    pub fn gnss_quality(&self) -> GnssQuality { GnssQuality::from(self.gps_quality) }
}

impl fmt::Display for InsFullNav {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:49 - INS Full Nav  len:{}", 104)?;
        writeln!(
            f,
            "  Week:{}  Time:{}ms  IMU:{}  GPS:{}",
            self.gps_week, self.gps_time_ms, self.imu_alignment_status, self.gps_quality
        )?;
        writeln!(
            f,
            "  Lat:{:.7}  Lon:{:.7}  Alt:{:.3}",
            self.lat_deg, self.lon_deg, self.alt_m
        )?;
        writeln!(
            f,
            "  Vel N:{:.3} E:{:.3} D:{:.3}  Speed:{:.3}",
            self.north_vel_ms, self.east_vel_ms, self.down_vel_ms, self.total_speed_ms
        )?;
        writeln!(
            f,
            "  Roll:{:.3}  Pitch:{:.3}  Heading:{:.3}  Track:{:.3}",
            self.roll_deg, self.pitch_deg, self.heading_deg, self.track_angle_deg
        )?;
        writeln!(
            f,
            "  AngRate X:{:.3} Y:{:.3} Z:{:.3}",
            self.angular_rate_x, self.angular_rate_y, self.angular_rate_z
        )?;
        write!(
            f,
            "  Accel X:{:.3} Y:{:.3} Z:{:.3}",
            self.accel_x, self.accel_y, self.accel_z
        )
    }
}

// ---------------------------------------------------------------------------
// Type 50 — INS RMS Info
// ---------------------------------------------------------------------------

/// GSOF Record Type 50 — INS RMS Information
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | GPS week number |
/// | 2 | 4 | u32 | GPS time of week \[ms\] |
/// | 6 | 1 | u8 | IMU alignment status |
/// | 7 | 1 | u8 | GNSS quality indicator |
/// | 8 | 4 | f32 | North position RMS \[m\] |
/// | 12 | 4 | f32 | East position RMS \[m\] |
/// | 16 | 4 | f32 | Down position RMS \[m\] |
/// | 20 | 4 | f32 | North velocity RMS \[m/s\] |
/// | 24 | 4 | f32 | East velocity RMS \[m/s\] |
/// | 28 | 4 | f32 | Down velocity RMS \[m/s\] |
/// | 32 | 4 | f32 | Roll RMS \[deg\] |
/// | 36 | 4 | f32 | Pitch RMS \[deg\] |
/// | 40 | 4 | f32 | Heading RMS \[deg\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InsRmsInfo {
    /// GPS week number.
    pub gps_week: u16,
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// IMU alignment status.
    pub imu_alignment_status: u8,
    /// GNSS quality indicator.
    pub gps_quality: u8,
    /// North position RMS \[m\].
    pub north_pos_rms: f32,
    /// East position RMS \[m\].
    pub east_pos_rms: f32,
    /// Down position RMS \[m\].
    pub down_pos_rms: f32,
    /// North velocity RMS \[m/s\].
    pub north_vel_rms: f32,
    /// East velocity RMS \[m/s\].
    pub east_vel_rms: f32,
    /// Down velocity RMS \[m/s\].
    pub down_vel_rms: f32,
    /// Roll RMS \[deg\].
    pub roll_rms_deg: f32,
    /// Pitch RMS \[deg\].
    pub pitch_rms_deg: f32,
    /// Heading RMS \[deg\].
    pub heading_rms_deg: f32,
}

impl InsRmsInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 50);
        Ok(Self {
            gps_week: r.u16()?,
            gps_time_ms: r.u32()?,
            imu_alignment_status: r.u8()?,
            gps_quality: r.u8()?,
            north_pos_rms: r.f32()?,
            east_pos_rms: r.f32()?,
            down_pos_rms: r.f32()?,
            north_vel_rms: r.f32()?,
            east_vel_rms: r.f32()?,
            down_vel_rms: r.f32()?,
            roll_rms_deg: r.f32()?,
            pitch_rms_deg: r.f32()?,
            heading_rms_deg: r.f32()?,
        })
    }

    /// IMU alignment status as enum.
    pub fn alignment_status(&self) -> ImuAlignmentStatus { ImuAlignmentStatus::from(self.imu_alignment_status) }
    /// GNSS quality as enum.
    pub fn gnss_quality(&self) -> GnssQuality { GnssQuality::from(self.gps_quality) }
}

impl fmt::Display for InsRmsInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:50 - INS RMS Info  len:{}", 44)?;
        writeln!(
            f,
            "  Week:{}  Time:{}ms  IMU:{}  GPS:{}",
            self.gps_week, self.gps_time_ms, self.imu_alignment_status, self.gps_quality
        )?;
        writeln!(
            f,
            "  PosRMS N:{:.4} E:{:.4} D:{:.4}",
            self.north_pos_rms, self.east_pos_rms, self.down_pos_rms
        )?;
        writeln!(
            f,
            "  VelRMS N:{:.4} E:{:.4} D:{:.4}",
            self.north_vel_rms, self.east_vel_rms, self.down_vel_rms
        )?;
        write!(
            f,
            "  AttRMS Roll:{:.3} Pitch:{:.3} Hdg:{:.3}",
            self.roll_rms_deg, self.pitch_rms_deg, self.heading_rms_deg
        )
    }
}

// ---------------------------------------------------------------------------
// Type 52 — DMI Raw Data
// ---------------------------------------------------------------------------

/// Individual DMI measurement entry (Type 52).
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | Time offset \[ms\] |
/// | 2 | 4 | u32 | Absolute distance count |
/// | 6 | 4 | i32 | Up/Down distance count |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmiMeasurement {
    /// Time offset from GPS time \[ms\].
    pub time_offset_ms: u16,
    /// Absolute distance count.
    pub abs_dist_count: u32,
    /// Up/Down distance count.
    pub ud_dist_count: i32,
}

/// GSOF Record Type 52 — DMI Raw Data
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | GPS week number |
/// | 2 | 4 | u32 | GPS time of week \[ms\] |
/// | 6 | 1 | u8 | Measurement count |
/// | 7 | 10*N | ... | Per-measurement data (10 bytes each) |
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmiRawData {
    /// GPS week number.
    pub gps_week: u16,
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// Collection of DMI measurements.
    pub measurements: heapless::Vec<DmiMeasurement, 16>,
}

impl DmiRawData {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 52);
        let gps_week = r.u16()?;
        let gps_time_ms = r.u32()?;
        let count = r.u8()? as usize;

        if count * 10 > r.remaining() {
            return Err(ParseError::OverflowingCount {
                record_type: 52,
                count,
                bytes_per_item: 10,
                available: r.remaining(),
            });
        }

        let mut measurements = heapless::Vec::new();
        for _ in 0..count {
            measurements
                .push(DmiMeasurement {
                    time_offset_ms: r.u16()?,
                    abs_dist_count: r.u32()?,
                    ud_dist_count: r.i32()?,
                })
                .map_err(|_| ParseError::OverflowingCount {
                    record_type: 52,
                    count,
                    bytes_per_item: 10,
                    available: 16,
                })?;
        }

        Ok(Self {
            gps_week,
            gps_time_ms,
            measurements,
        })
    }
}

impl fmt::Display for DmiRawData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  GsofType:52 - DMI Raw Data  len:{}",
            7 + self.measurements.len() * 10
        )?;
        write!(
            f,
            "  Week:{}  Time:{}ms  Meas:{}",
            self.gps_week,
            self.gps_time_ms,
            self.measurements.len()
        )?;
        for (i, m) in self.measurements.iter().enumerate() {
            write!(
                f,
                "\n  [{}] Offset:{}ms  AbsDist:{}  UdDist:{}",
                i, m.time_offset_ms, m.abs_dist_count, m.ud_dist_count
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Type 63 — INS VNAV Full Navigation Info (Krypton)
// ---------------------------------------------------------------------------

/// GSOF Record Type 63 — INS VNAV Full Navigation Information (Krypton)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | GPS week number |
/// | 2 | 4 | u32 | GPS time of week \[ms\] |
/// | 6 | 1 | u8 | IMU alignment status |
/// | 7 | 1 | u8 | GNSS quality indicator |
/// | 8 | 8 | f64 | Latitude \[deg\] |
/// | 16 | 8 | f64 | Longitude \[deg\] |
/// | 24 | 8 | f64 | Altitude \[m\] |
/// | 32 | 4 | f32 | North velocity \[m/s\] |
/// | 36 | 4 | f32 | East velocity \[m/s\] |
/// | 40 | 4 | f32 | Down velocity \[m/s\] |
/// | 44 | 4 | f32 | Total speed \[m/s\] |
/// | 48 | 8 | f64 | Roll \[deg\] |
/// | 56 | 8 | f64 | Pitch \[deg\] |
/// | 64 | 8 | f64 | Heading \[deg\] |
/// | 72 | 8 | f64 | Track angle \[deg\] |
/// | 80 | 4 | f32 | Angular rate X \[rad/s\] |
/// | 84 | 4 | f32 | Angular rate Y \[rad/s\] |
/// | 88 | 4 | f32 | Angular rate Z \[rad/s\] |
/// | 92 | 4 | f32 | Acceleration X \[m/s^2\] |
/// | 96 | 4 | f32 | Acceleration Y \[m/s^2\] |
/// | 100 | 4 | f32 | Acceleration Z \[m/s^2\] |
/// | 104 | 8 | f64 | Heave \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InsVnavFullNav {
    /// GPS week number.
    pub gps_week: u16,
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// IMU alignment status.
    pub imu_alignment_status: u8,
    /// GNSS quality indicator.
    pub gps_quality: u8,
    /// Latitude \[deg\].
    pub lat_deg: f64,
    /// Longitude \[deg\].
    pub lon_deg: f64,
    /// Altitude \[m\].
    pub alt_m: f64,
    /// North velocity \[m/s\].
    pub north_vel_ms: f32,
    /// East velocity \[m/s\].
    pub east_vel_ms: f32,
    /// Down velocity \[m/s\].
    pub down_vel_ms: f32,
    /// Total speed \[m/s\].
    pub total_speed_ms: f32,
    /// Roll \[deg\].
    pub roll_deg: f64,
    /// Pitch \[deg\].
    pub pitch_deg: f64,
    /// Heading \[deg\].
    pub heading_deg: f64,
    /// Track angle \[deg\].
    pub track_angle_deg: f64,
    /// Angular rate X \[rad/s\].
    pub angular_rate_x: f32,
    /// Angular rate Y \[rad/s\].
    pub angular_rate_y: f32,
    /// Angular rate Z \[rad/s\].
    pub angular_rate_z: f32,
    /// Acceleration X \[m/s^2\].
    pub accel_x: f32,
    /// Acceleration Y \[m/s^2\].
    pub accel_y: f32,
    /// Acceleration Z \[m/s^2\].
    pub accel_z: f32,
    /// Heave \[m\].
    pub heave_m: f64,
}

impl InsVnavFullNav {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 63);
        Ok(Self {
            gps_week: r.u16()?,
            gps_time_ms: r.u32()?,
            imu_alignment_status: r.u8()?,
            gps_quality: r.u8()?,
            lat_deg: r.f64()?,
            lon_deg: r.f64()?,
            alt_m: r.f64()?,
            north_vel_ms: r.f32()?,
            east_vel_ms: r.f32()?,
            down_vel_ms: r.f32()?,
            total_speed_ms: r.f32()?,
            roll_deg: r.f64()?,
            pitch_deg: r.f64()?,
            heading_deg: r.f64()?,
            track_angle_deg: r.f64()?,
            angular_rate_x: r.f32()?,
            angular_rate_y: r.f32()?,
            angular_rate_z: r.f32()?,
            accel_x: r.f32()?,
            accel_y: r.f32()?,
            accel_z: r.f32()?,
            heave_m: r.f64()?,
        })
    }

    /// IMU alignment status as enum.
    pub fn alignment_status(&self) -> ImuAlignmentStatus { ImuAlignmentStatus::from(self.imu_alignment_status) }
    /// GNSS quality as enum.
    pub fn gnss_quality(&self) -> GnssQuality { GnssQuality::from(self.gps_quality) }
}

impl fmt::Display for InsVnavFullNav {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:63 - INS VNAV Full Nav  len:{}", 112)?;
        writeln!(
            f,
            "  Week:{}  Time:{}ms  IMU:{}  GPS:{}",
            self.gps_week, self.gps_time_ms, self.imu_alignment_status, self.gps_quality
        )?;
        writeln!(
            f,
            "  Lat:{:.7}  Lon:{:.7}  Alt:{:.3}",
            self.lat_deg, self.lon_deg, self.alt_m
        )?;
        writeln!(
            f,
            "  Vel N:{:.3} E:{:.3} D:{:.3}  Speed:{:.3}",
            self.north_vel_ms, self.east_vel_ms, self.down_vel_ms, self.total_speed_ms
        )?;
        writeln!(
            f,
            "  Roll:{:.3}  Pitch:{:.3}  Heading:{:.3}  Track:{:.3}",
            self.roll_deg, self.pitch_deg, self.heading_deg, self.track_angle_deg
        )?;
        writeln!(
            f,
            "  AngRate X:{:.3} Y:{:.3} Z:{:.3}",
            self.angular_rate_x, self.angular_rate_y, self.angular_rate_z
        )?;
        writeln!(
            f,
            "  Accel X:{:.3} Y:{:.3} Z:{:.3}",
            self.accel_x, self.accel_y, self.accel_z
        )?;
        write!(f, "  Heave:{:.3}", self.heave_m)
    }
}

// ---------------------------------------------------------------------------
// Type 64 — INS VNAV RMS Info (Krypton)
// ---------------------------------------------------------------------------

/// GSOF Record Type 64 — INS VNAV RMS Information (Krypton)
///
/// | Offset | Size | Type | Field |
/// |--------|------|------|-------|
/// | 0 | 2 | u16 | GPS week number |
/// | 2 | 4 | u32 | GPS time of week \[ms\] |
/// | 6 | 1 | u8 | IMU alignment status |
/// | 7 | 1 | u8 | GNSS quality indicator |
/// | 8 | 4 | f32 | North position RMS \[m\] |
/// | 12 | 4 | f32 | East position RMS \[m\] |
/// | 16 | 4 | f32 | Down position RMS \[m\] |
/// | 20 | 4 | f32 | North velocity RMS \[m/s\] |
/// | 24 | 4 | f32 | East velocity RMS \[m/s\] |
/// | 28 | 4 | f32 | Down velocity RMS \[m/s\] |
/// | 32 | 4 | f32 | Roll RMS \[deg\] |
/// | 36 | 4 | f32 | Pitch RMS \[deg\] |
/// | 40 | 4 | f32 | Heading RMS \[deg\] |
/// | 44 | 4 | f32 | Heave RMS \[m\] |
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InsVnavRmsInfo {
    /// GPS week number.
    pub gps_week: u16,
    /// GPS time of week \[ms\].
    pub gps_time_ms: u32,
    /// IMU alignment status.
    pub imu_alignment_status: u8,
    /// GNSS quality indicator.
    pub gps_quality: u8,
    /// North position RMS \[m\].
    pub north_pos_rms: f32,
    /// East position RMS \[m\].
    pub east_pos_rms: f32,
    /// Down position RMS \[m\].
    pub down_pos_rms: f32,
    /// North velocity RMS \[m/s\].
    pub north_vel_rms: f32,
    /// East velocity RMS \[m/s\].
    pub east_vel_rms: f32,
    /// Down velocity RMS \[m/s\].
    pub down_vel_rms: f32,
    /// Roll RMS \[deg\].
    pub roll_rms_deg: f32,
    /// Pitch RMS \[deg\].
    pub pitch_rms_deg: f32,
    /// Heading RMS \[deg\].
    pub heading_rms_deg: f32,
    /// Heave RMS \[m\].
    pub heave_rms_m: f32,
}

impl InsVnavRmsInfo {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        let mut r = Reader::new(data, 64);
        Ok(Self {
            gps_week: r.u16()?,
            gps_time_ms: r.u32()?,
            imu_alignment_status: r.u8()?,
            gps_quality: r.u8()?,
            north_pos_rms: r.f32()?,
            east_pos_rms: r.f32()?,
            down_pos_rms: r.f32()?,
            north_vel_rms: r.f32()?,
            east_vel_rms: r.f32()?,
            down_vel_rms: r.f32()?,
            roll_rms_deg: r.f32()?,
            pitch_rms_deg: r.f32()?,
            heading_rms_deg: r.f32()?,
            heave_rms_m: r.f32()?,
        })
    }

    /// IMU alignment status as enum.
    pub fn alignment_status(&self) -> ImuAlignmentStatus { ImuAlignmentStatus::from(self.imu_alignment_status) }
    /// GNSS quality as enum.
    pub fn gnss_quality(&self) -> GnssQuality { GnssQuality::from(self.gps_quality) }
}

impl fmt::Display for InsVnavRmsInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  GsofType:64 - INS VNAV RMS Info  len:{}", 48)?;
        writeln!(
            f,
            "  Week:{}  Time:{}ms  IMU:{}  GPS:{}",
            self.gps_week, self.gps_time_ms, self.imu_alignment_status, self.gps_quality
        )?;
        writeln!(
            f,
            "  PosRMS N:{:.4} E:{:.4} D:{:.4}",
            self.north_pos_rms, self.east_pos_rms, self.down_pos_rms
        )?;
        writeln!(
            f,
            "  VelRMS N:{:.4} E:{:.4} D:{:.4}",
            self.north_vel_rms, self.east_vel_rms, self.down_vel_rms
        )?;
        write!(
            f,
            "  AttRMS Roll:{:.3} Pitch:{:.3} Hdg:{:.3}  HeaveRMS:{:.4}",
            self.roll_rms_deg, self.pitch_rms_deg, self.heading_rms_deg, self.heave_rms_m
        )
    }
}

// ---------------------------------------------------------------------------
// Top-level GSOF record dispatcher
// ---------------------------------------------------------------------------

fn unknown(gsof_type: u8, length: u8) -> GsofRecord {
    GsofRecord::Unknown {
        gsof_type,
        length,
    }
}

/// Parse a single GSOF record from `(type_byte, data_slice)`.
/// The data slice must already be bounded to exactly `length` bytes.
pub fn parse_gsof_record(gsof_type: u8, data: &[u8]) -> GsofRecord {
    match gsof_type {
        1 => PositionTime::parse(data)
            .map(GsofRecord::PositionTime)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        2 => LatLonHeight::parse(data)
            .map(GsofRecord::LatLonHeight)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        3 => Ecef::parse(data)
            .map(GsofRecord::Ecef)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        4 => LocalDatum::parse(data)
            .map(GsofRecord::LocalDatum)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        6 => EcefDelta::parse(data)
            .map(GsofRecord::EcefDelta)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        7 => TangentPlaneDelta::parse(data)
            .map(GsofRecord::TangentPlaneDelta)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        8 => Velocity::parse(data)
            .map(GsofRecord::Velocity)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        9 => PdopInfo::parse(data)
            .map(GsofRecord::PdopInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        10 => ClockInfo::parse(data)
            .map(GsofRecord::ClockInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        11 => PositionVcvInfo::parse(data)
            .map(GsofRecord::PositionVcvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        12 => PositionSigmaInfo::parse(data)
            .map(GsofRecord::PositionSigmaInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        13 => BriefSvInfo::parse(data)
            .map(GsofRecord::BriefSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        14 => SvDetailedInfo::parse(data)
            .map(GsofRecord::SvDetailedInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        15 => ReceiverSerialNumber::parse(data)
            .map(GsofRecord::ReceiverSerialNumber)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        16 => UtcTime::parse(data)
            .map(GsofRecord::UtcTime)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        27 => AttitudeInfo::parse(data)
            .map(GsofRecord::AttitudeInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        33 => AllBriefSvInfo::parse(data)
            .map(GsofRecord::AllBriefSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        34 => AllDetailedSvInfo::parse(data)
            .map(GsofRecord::AllDetailedSvInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        35 => ReceivedBaseInfo::parse(data)
            .map(GsofRecord::ReceivedBaseInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        37 => BatteryMemoryInfo::parse(data)
            .map(GsofRecord::BatteryMemoryInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        38 => PositionTypeInfo::parse(data)
            .map(GsofRecord::PositionTypeInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        40 => LbandStatus::parse(data)
            .map(GsofRecord::LbandStatus)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        41 => BasePositionQuality::parse(data)
            .map(GsofRecord::BasePositionQuality)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        49 => InsFullNav::parse(data)
            .map(GsofRecord::InsFullNav)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        50 => InsRmsInfo::parse(data)
            .map(GsofRecord::InsRmsInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        52 => DmiRawData::parse(data)
            .map(GsofRecord::DmiRawData)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        63 => InsVnavFullNav::parse(data)
            .map(GsofRecord::InsVnavFullNav)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        64 => InsVnavRmsInfo::parse(data)
            .map(GsofRecord::InsVnavRmsInfo)
            .unwrap_or_else(|_| unknown(gsof_type, data.len() as u8)),
        _ => unknown(gsof_type, data.len() as u8),
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
