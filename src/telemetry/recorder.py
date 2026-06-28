import os
import struct
import time
from machine import Pin, SPI

import sdcard

_SD_MOUNT    = "/sd"
_LOG_DIR     = _SD_MOUNT + "/flights"
_SECTOR      = 512
_FILL_CHUNK  = b'\x00' * _SECTOR
_RECORD_FMT  = "<I20fHHHHHH"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)


class SdSink:
    """Output backend that writes binary telemetry records to a file on a mounted SD card.

    Owns the full SD lifecycle: mount on init, open run directory on
    ``init_session()``, unmount on ``close()``.  Two-phase design lets callers
    mount early (fail-fast) and create the run directory later.
    """

    def __init__(self, sck, mosi, miso, cs, preallocate_bytes=0):
        """Mount SD card and validate it is accessible.

        Raises OSError immediately if the card is missing or unreadable,
        giving the operator a clear signal before motors are armed.
        """
        cs_pin = Pin(cs, Pin.OUT, value=1)
        spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
                  sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        time.sleep_ms(250)
        self._sd = sdcard.SDCard(spi, cs_pin, baudrate=25_000_000)
        self._vfs = os.VfsFat(self._sd)
        os.mount(self._vfs, _SD_MOUNT)

        self._preallocate_bytes = preallocate_bytes
        self._write_buf = bytearray(_SECTOR) if preallocate_bytes > 0 else None
        self._buf_pos  = 0  # bytes used in _write_buf
        self._file_pos = 0  # bytes written to file (full sectors only)
        self._run_dir = None
        self._f = None

    def init_session(self, dt):
        """Create a timestamped run directory, copy config and open log file.

        dt: (year, month, day, weekday, hour, minute, second) from PCF8523.datetime().
        Call once when the recording session actually starts (after arming).
        """
        try:
            os.mkdir(_LOG_DIR)
        except OSError:
            pass  # already exists
        self._run_dir = "{}/{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
            _LOG_DIR, dt[0], dt[1], dt[2], dt[4], dt[5], dt[6]
        )
        os.mkdir(self._run_dir)

        with open("config.json", "rb") as src:
            cfg_bytes = src.read()
        with open(self._run_dir + "/config.json", "wb") as dst:
            dst.write(cfg_bytes)

        with open("specification.json", "rb") as src:
            crit_bytes = src.read()
        with open(self._run_dir + "/specification.json", "wb") as dst:
            dst.write(crit_bytes)

        if self._preallocate_bytes > 0:
            tmp_path = self._run_dir + "/log.tmp"
            t0 = time.ticks_ms()
            with open(tmp_path, "wb") as f:
                remaining = self._preallocate_bytes
                while remaining > 0:
                    n = min(512, remaining)
                    f.write(_FILL_CHUNK[:n])
                    remaining -= n
            print("prealloc: {}ms for {} bytes ({} KB/s)".format(
                time.ticks_diff(time.ticks_ms(), t0),
                self._preallocate_bytes,
                self._preallocate_bytes // max(1, time.ticks_diff(time.ticks_ms(), t0)),
            ))
            self._f = open(tmp_path, "r+b")
            self._f.seek(0)
        else:
            self._f = open(self._run_dir + "/log.bin", "wb")

    @property
    def path(self):
        """Return the run directory path (useful for diagnostics)."""
        return self._run_dir

    def write_bytes(self, data):
        """Append a binary record to the log file."""
        if self._preallocate_bytes > 0:
            if self._file_pos + self._buf_pos + len(data) > self._preallocate_bytes:
                raise OSError("telemetry overflow: preallocate_bytes exceeded")
            di = 0
            while di < len(data):
                take = min(_SECTOR - self._buf_pos, len(data) - di)
                self._write_buf[self._buf_pos:self._buf_pos + take] = data[di:di + take]
                self._buf_pos += take
                di += take
                if self._buf_pos == _SECTOR:
                    self._f.write(self._write_buf)
                    self._file_pos += _SECTOR
                    self._buf_pos = 0
        else:
            self._f.write(data)

    def flush(self):
        """Flush FatFs sector buffer to SD card.  Note: in prealloc mode the
        Python-level write buffer (_write_buf) is NOT flushed here; call
        close() for full durability."""
        self._f.flush()

    def write_crash_log(self, exc):
        """Write exception traceback to crash.log in the session folder. No-op if no session."""
        if self._run_dir is None:
            return
        import sys
        try:
            with open(self._run_dir + "/crash.log", "w") as f:
                sys.print_exception(exc, f)
        except Exception:
            pass

    def _finalize_log(self, actual_bytes):
        """Copy actual_bytes from log.tmp to log.bin, then remove log.tmp."""
        tmp_path = self._run_dir + "/log.tmp"
        log_path = self._run_dir + "/log.bin"
        remaining = actual_bytes
        with open(tmp_path, 'rb') as src, open(log_path, 'wb') as dst:
            while remaining > 0:
                n = min(_SECTOR, remaining)
                dst.write(src.read(n))
                remaining -= n
        os.remove(tmp_path)

    def close(self):
        """Flush and close the log file, then unmount the SD card."""
        if self._f:
            actual_bytes = self._file_pos
            if self._write_buf is not None and self._buf_pos > 0:
                self._f.write(memoryview(self._write_buf)[:self._buf_pos])
                actual_bytes += self._buf_pos
            self._f.flush()
            self._f.close()
            self._f = None
            if self._preallocate_bytes > 0:
                try:
                    self._finalize_log(actual_bytes)
                except Exception as e:
                    self.write_crash_log(e)  # log.tmp stays on SD; crash.log records why
        os.umount(_SD_MOUNT)


class TelemetryRecorder:
    """Facade that decimates and writes binary telemetry records, delegating I/O to a sink.

    Records are packed via struct.pack_into into a pre-allocated bytearray with zero
    heap allocation per record. pull_flights.py decodes log.bin -> log.csv on the PC.
    Record format: _RECORD_FMT = "<I20fHHHHHH" (96 bytes per record).
    """

    def __init__(self, sample_every, sink):
        self._sample_every = sample_every
        self._sink = sink
        self._counter = 0
        self._max_dt_ms = 0
        self._pack_buf = bytearray(_RECORD_SIZE)

    def begin_session(self):
        """Reset counters. Call when entering STABILIZING state."""
        self._counter = 0
        self._max_dt_ms = 0

    def record(self, t_ms, dt_ms, enc_roll,
               iqr, iqi, iqj, iqk, grv_acc, grv_lag_ms,
               gyro_x, gyro_acc, gyro_lag_ms,
               ang_err, ang_p, ang_i, ang_d, rate_sp,
               rate_err, rate_p, rate_i, rate_d, pid_out, m1, m2, m3, m4):
        """Pack and emit a binary record every sample_every-th call. Others are silently dropped."""
        if dt_ms > self._max_dt_ms:
            self._max_dt_ms = dt_ms
        self._counter += 1
        if self._counter < self._sample_every:
            return
        self._counter = 0
        max_dt = self._max_dt_ms
        self._max_dt_ms = 0

        struct.pack_into(_RECORD_FMT, self._pack_buf, 0,
            t_ms, enc_roll,
            iqr, iqi, iqj, iqk, grv_acc, grv_lag_ms,
            gyro_x, gyro_acc, gyro_lag_ms,
            ang_err, ang_p, ang_i, ang_d, rate_sp,
            rate_err, rate_p, rate_i, rate_d, pid_out,
            m1, m2, m3, m4, dt_ms, max_dt)
        self._sink.write_bytes(self._pack_buf)

    def write_crash_log(self, exc):
        """Write exception traceback to the session folder via the sink."""
        self._sink.write_crash_log(exc)

    def end_session(self):
        """Flush and close the sink. Call after motors are stopped. SD errors are swallowed."""
        try:
            self._sink.close()
        except Exception:
            pass
