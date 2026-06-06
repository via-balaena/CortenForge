//! The logged-sample type and its CSV wire format — the single source of truth
//! for the firmware → S3-analysis data contract.

use core::fmt::{self, Write};

use crate::MAX_HALL;

/// One logged sample streamed over USB serial.
///
/// CSV layout (one line, `\n`-terminated), produced by [`write_csv`](Self::write_csv):
///
/// ```text
/// t_us,drive,h0,h1,...,h{n_hall-1},ax,ay,az
/// ```
///
/// `t_us` is microseconds since acquisition start; `drive` is the commanded
/// drive scaled to `i16` (`-32767..=32767` ↔ `-1..=1`); `h*` are raw Hall ADC
/// counts (one per p-bit); `ax/ay/az` are the base accelerometer ADC counts (the
/// measured bath). The matching header comes from [`csv_header`](Self::csv_header).
pub struct LogRecord {
    /// Microseconds since acquisition start. `u64` so it never wraps — a `u32`
    /// microsecond counter rolls over at ~71.6 min and the cold-calibration /
    /// rare-escape regime needs multi-hour captures.
    pub t_us: u64,
    /// Commanded drive, scaled to `i16` (`-1..=1` ↔ `-32767..=32767`).
    pub drive: i16,
    /// Hall-sensor readings (raw ADC counts), one per p-bit.
    pub hall: [i16; MAX_HALL],
    /// Number of active Hall channels (`<= MAX_HALL`).
    pub n_hall: u8,
    /// Base accelerometer X/Y/Z (raw ADC counts).
    pub accel: [i16; 3],
}

impl LogRecord {
    /// Write the CSV line for this record into `buf`, returning the number of
    /// bytes written, or `None` if `buf` is too small.
    pub fn write_csv(&self, buf: &mut [u8]) -> Option<usize> {
        let mut w = ByteWriter { buf, pos: 0 };
        write!(w, "{},{}", self.t_us, self.drive).ok()?;
        for h in self.hall.iter().take(self.n_hall as usize) {
            write!(w, ",{h}").ok()?;
        }
        writeln!(w, ",{},{},{}", self.accel[0], self.accel[1], self.accel[2]).ok()?;
        Some(w.pos)
    }

    /// Write the CSV header line for `n_hall` channels into `buf`, returning the
    /// number of bytes written, or `None` if `buf` is too small.
    pub fn csv_header(n_hall: u8, buf: &mut [u8]) -> Option<usize> {
        let mut w = ByteWriter { buf, pos: 0 };
        write!(w, "t_us,drive").ok()?;
        for i in 0..n_hall {
            write!(w, ",h{i}").ok()?;
        }
        writeln!(w, ",ax,ay,az").ok()?;
        Some(w.pos)
    }
}

/// `core::fmt::Write` adapter over a fixed byte buffer; signals overflow by
/// returning `Err`, which the `.ok()?` call sites turn into `None`.
struct ByteWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Write for ByteWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let end = self.pos + bytes.len();
        if end > self.buf.len() {
            return Err(fmt::Error);
        }
        self.buf[self.pos..end].copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn csv_round_trips() {
        let rec = LogRecord {
            t_us: 123_456,
            drive: -2000,
            hall: [10, -20, 30, 0, 0, 0, 0, 0],
            n_hall: 3,
            accel: [100, -200, 300],
        };
        let mut buf = [0u8; 128];
        let n = rec.write_csv(&mut buf).unwrap();
        let line = core::str::from_utf8(&buf[..n]).unwrap();
        assert_eq!(line, "123456,-2000,10,-20,30,100,-200,300\n");

        // Parse it back the way the S3 tool will, and check the fields survive.
        let mut fields = line.trim_end().split(',');
        assert_eq!(fields.next().unwrap().parse::<u32>().unwrap(), 123_456);
        assert_eq!(fields.next().unwrap().parse::<i16>().unwrap(), -2000);
        assert_eq!(fields.next().unwrap().parse::<i16>().unwrap(), 10);
        assert_eq!(fields.next().unwrap().parse::<i16>().unwrap(), -20);
        assert_eq!(fields.next().unwrap().parse::<i16>().unwrap(), 30);
        assert_eq!(fields.next().unwrap().parse::<i16>().unwrap(), 100);
        // ax consumed above; ay, az remain.
        assert_eq!(fields.clone().count(), 2);
    }

    #[test]
    fn header_matches_channel_count() {
        let mut buf = [0u8; 64];
        let n = LogRecord::csv_header(3, &mut buf).unwrap();
        let line = core::str::from_utf8(&buf[..n]).unwrap();
        assert_eq!(line, "t_us,drive,h0,h1,h2,ax,ay,az\n");
    }

    #[test]
    fn overflow_returns_none() {
        let rec = LogRecord {
            t_us: 1,
            drive: 1,
            hall: [0; MAX_HALL],
            n_hall: 8,
            accel: [0, 0, 0],
        };
        let mut tiny = [0u8; 4];
        assert!(rec.write_csv(&mut tiny).is_none());
    }
}
