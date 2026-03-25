/// reassembly.rs — GSOF multi-packet reassembly.
///
/// A single logical GSOF "message" may be spread across several Trimcomm
/// 0x40 packets (pages).  Each 0x40 payload begins with three bytes:
///
///   Transmission Number  (1 byte) — groups pages of the same message
///   Page Index           (1 byte) — 0-based page counter
///   Max Page Index       (1 byte) — last page index in this message
///
/// Rules:
///   * Page Index == 0 → start of a new message; reset the accumulator.
///   * Page Index == Max Page Index → last page; message is complete.
///   * The data bytes (after the three header bytes) are concatenated.
///
/// The buffer is bounded to 2 048 bytes, matching the static array in the
/// original C code.  Exceeding it produces `ReassemblyError::BufferOverflow`.
use crate::gsof::{parse_gsof_payload, GsofRecord, ParseError};

/// Maximum accumulated GSOF payload size — identical to the original C code.
pub const GSOF_BUF_MAX: usize = 2048;

#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum ReassemblyError {
    /// Packet header was fewer than 3 bytes.
    #[cfg_attr(
        feature = "std",
        error("GSOF 0x40 packet has fewer than 3 header bytes")
    )]
    ShortHeader,
    #[cfg_attr(
        feature = "std",
        error("GSOF payload overflow: attempted {attempted} bytes, max {GSOF_BUF_MAX}")
    )]
    BufferOverflow { attempted: usize },

    #[cfg_attr(feature = "std", error("GSOF parse error: {0}"))]
    Parse(ParseError),
}

#[cfg(not(feature = "std"))]
impl core::fmt::Display for ReassemblyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ReassemblyError::ShortHeader => {
                write!(f, "GSOF 0x40 packet has fewer than 3 header bytes")
            }
            ReassemblyError::BufferOverflow { attempted } => write!(
                f,
                "GSOF payload overflow: attempted {attempted} bytes, max {GSOF_BUF_MAX}"
            ),
            ReassemblyError::Parse(e) => write!(f, "GSOF parse error: {e}"),
        }
    }
}

/// Stateful GSOF reassembler.  Feed it the `data` field of every Trimcomm
/// 0x40 packet in arrival order.  When `push()` returns `Ok(Some(_))` you
/// have a complete, parsed message.
pub struct Reassembler {
    buf: [u8; GSOF_BUF_MAX],
    buf_len: usize,
}

/// The paged header present at the start of every Trimcomm 0x40 payload.
#[derive(Debug, Clone, Copy)]
pub struct PageHeader {
    pub transmission_number: u8,
    pub page_index: u8,
    pub max_page_index: u8,
}

pub struct PushResult {
    pub header: PageHeader,
    /// Non-empty only when the message is complete (`page == max_page`).
    pub records: Option<Result<heapless::Vec<GsofRecord, 32>, ReassemblyError>>,
}

impl Reassembler {
    pub fn new() -> Self {
        Self {
            buf: [0u8; GSOF_BUF_MAX],
            buf_len: 0,
        }
    }

    /// Process one Trimcomm 0x40 payload (the `data` field of a `TrimcommPacket`
    /// with `packet_type == TYPE_GSOF`).
    pub fn push(&mut self, payload: &[u8]) -> Result<PushResult, ReassemblyError> {
        if payload.len() < 3 {
            return Err(ReassemblyError::ShortHeader);
        }

        let header = PageHeader {
            transmission_number: payload[0],
            page_index: payload[1],
            max_page_index: payload[2],
        };
        let data = &payload[3..];

        // First page of a new message → reset.
        if header.page_index == 0 {
            self.buf_len = 0;
        }

        // Bounds check before extending.
        let new_len = self.buf_len + data.len();
        if new_len > GSOF_BUF_MAX {
            return Err(ReassemblyError::BufferOverflow { attempted: new_len });
        }
        self.buf[self.buf_len..new_len].copy_from_slice(data);
        self.buf_len = new_len;

        // Last page → parse and return.
        let records = if header.page_index == header.max_page_index {
            Some(parse_gsof_payload(&self.buf[..self.buf_len]).map_err(ReassemblyError::Parse))
        } else {
            None
        };

        Ok(PushResult { header, records })
    }
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}
