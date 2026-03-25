/// trimcomm.rs — Trimble Trimcomm packet framing.
///
/// Wire format:
///   STX  (0x02)   1 byte
///   STAT          1 byte
///   TYPE          1 byte
///   LENGTH        1 byte  — number of DATA bytes
///   DATA          LENGTH bytes
///   CHECKSUM      1 byte  — sum of STAT..DATA[last], mod 256
///   ETX  (0x03)   1 byte
///
/// A GSOF packet has TYPE == 0x40.
///
/// # Embedded usage
///
/// [`FrameParser`] is a push-based state machine: call [`FrameParser::push`]
/// once per received byte from any source (UART DMA callback, RTOS queue,
/// TCP/UDP receive handler).  
///
/// # std usage
///
/// [`read_packet`] drives `FrameParser` from an `io::Read` source (serial
/// port, file, TCP stream) and is available when the `std` feature is on.
pub const STX: u8 = 0x02;
pub const ETX: u8 = 0x03;
pub const TYPE_GSOF: u8 = 0x40;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum FrameError {
    /// ETX byte was wrong; may indicate stream misalignment.
    BadEtx(u8),
    /// Checksum mismatch: got vs expected.
    BadChecksum { got: u8, expected: u8 },
    /// I/O error from the blocking wrapper (std only).
    #[cfg(feature = "std")]
    Io(std::io::Error),
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FrameError::BadEtx(b) => write!(f, "expected ETX 0x03, got 0x{b:02X}"),
            FrameError::BadChecksum { got, expected } => {
                write!(
                    f,
                    "checksum mismatch: got 0x{got:02X}, expected 0x{expected:02X}"
                )
            }
            #[cfg(feature = "std")]
            FrameError::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for FrameError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for FrameError {
    fn from(e: std::io::Error) -> Self {
        FrameError::Io(e)
    }
}

// ---------------------------------------------------------------------------
// Packet type
// ---------------------------------------------------------------------------

/// A validated Trimcomm packet.
#[derive(Debug)]
pub struct TrimcommPacket {
    pub stat: u8,
    pub packet_type: u8,
    /// Payload bytes; capacity 255 matches the u8 LENGTH field maximum.
    pub data: heapless::Vec<u8, 255>,
}

// ---------------------------------------------------------------------------
// FrameParser — push-based state machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
enum FrameState {
    WaitingForStx,
    ReadingStat,
    ReadingType,
    ReadingLength,
    ReadingData { expected: u8, received: u8 },
    ReadingChecksum,
    ReadingEtx { checksum: u8 },
}

/// Push-based, no_std Trimcomm frame parser.
///
/// Feed bytes one at a time via [`FrameParser::push`]:
/// - Returns `None` while a frame is still in progress.
/// - Returns `Some(Ok(packet))` when a complete, validated frame arrives.
/// - Returns `Some(Err(e))` on a framing error; the parser resets to sync
///   state automatically so the next STX starts a fresh frame.
///
/// [`FrameParser::is_syncing`] returns `true` while no STX has been seen,
/// which lets callers identify and log discarded pre-sync bytes.
pub struct FrameParser {
    state: FrameState,
    stat: u8,
    packet_type: u8,
    length: u8,
    data: heapless::Vec<u8, 255>,
}

impl FrameParser {
    pub fn new() -> Self {
        Self {
            state: FrameState::WaitingForStx,
            stat: 0,
            packet_type: 0,
            length: 0,
            data: heapless::Vec::new(),
        }
    }

    /// `true` while waiting for a STX byte — not yet inside any frame.
    /// Useful for callers that want to count or log discarded bytes.
    pub fn is_syncing(&self) -> bool {
        matches!(self.state, FrameState::WaitingForStx)
    }

    /// Feed one byte into the parser.
    pub fn push(&mut self, byte: u8) -> Option<Result<TrimcommPacket, FrameError>> {
        match self.state {
            FrameState::WaitingForStx => {
                if byte == STX {
                    self.state = FrameState::ReadingStat;
                }
                None
            }

            FrameState::ReadingStat => {
                self.stat = byte;
                self.state = FrameState::ReadingType;
                None
            }

            FrameState::ReadingType => {
                self.packet_type = byte;
                self.state = FrameState::ReadingLength;
                None
            }

            FrameState::ReadingLength => {
                self.length = byte;
                self.data.clear();
                self.state = if byte == 0 {
                    FrameState::ReadingChecksum
                } else {
                    FrameState::ReadingData {
                        expected: byte,
                        received: 0,
                    }
                };
                None
            }

            FrameState::ReadingData { expected, received } => {
                // Capacity == 255 == max u8, so push is infallible here.
                let _ = self.data.push(byte);
                let received = received + 1;
                self.state = if received == expected {
                    FrameState::ReadingChecksum
                } else {
                    FrameState::ReadingData { expected, received }
                };
                None
            }

            FrameState::ReadingChecksum => {
                self.state = FrameState::ReadingEtx { checksum: byte };
                None
            }

            FrameState::ReadingEtx { checksum } => {
                // Always reset to sync state regardless of outcome.
                self.state = FrameState::WaitingForStx;

                if byte != ETX {
                    return Some(Err(FrameError::BadEtx(byte)));
                }

                let expected_cs: u8 = self
                    .stat
                    .wrapping_add(self.packet_type)
                    .wrapping_add(self.length)
                    .wrapping_add(self.data.iter().fold(0u8, |a, &b| a.wrapping_add(b)));

                if checksum != expected_cs {
                    return Some(Err(FrameError::BadChecksum {
                        got: checksum,
                        expected: expected_cs,
                    }));
                }

                // Swap data out so we don't copy the payload bytes.
                let mut data = heapless::Vec::new();
                core::mem::swap(&mut self.data, &mut data);
                Some(Ok(TrimcommPacket {
                    stat: self.stat,
                    packet_type: self.packet_type,
                    data,
                }))
            }
        }
    }
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Blocking wrapper (std only)
// ---------------------------------------------------------------------------

/// Drive [`FrameParser`] from a blocking [`std::io::Read`] source.
///
/// Bytes discarded before the first STX are reported via the `skipped`
/// callback, matching the "Skipping %02X" output of the original C code.
/// Returns `Ok(None)` on clean EOF before any complete frame.
#[cfg(feature = "std")]
pub fn read_packet(
    r: &mut impl std::io::Read,
    mut skipped: impl FnMut(u8),
) -> Result<Option<TrimcommPacket>, FrameError> {
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 1];
    loop {
        match r.read(&mut buf) {
            Ok(0) => return Ok(None),
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(FrameError::Io(e)),
        }
        let was_syncing = parser.is_syncing();
        match parser.push(buf[0]) {
            None => {
                // If still in WaitingForStx after a non-STX byte, it was discarded.
                if was_syncing && parser.is_syncing() {
                    skipped(buf[0]);
                }
            }
            Some(result) => return result.map(Some),
        }
    }
}
