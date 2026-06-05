"""
pull_flights.py — move new flight runs from Pico SD card to test_runs/flights/

Run from project root:
  python pipelines/flight-runner/scripts/pull_flights.py [--erase]

Pulls all new runs from the SD card and deletes them from the card after
a successful byte-exact transfer. No confirmation prompt.

  --erase  Also delete failed/ghost runs that remain on the SD card
                     after a failed transfer (e.g. empty folders, missing files).

Pico must be connected on COM7. mpremote sends Ctrl+C before running any
script — do not run this during a live stabilisation session.
"""

import base64
import os
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

# Mirror of recorder.py _RECORD_FMT — must stay in sync if the Pico format changes.
_LOG_RECORD_FMT  = "<I8f11fHHHH"
_LOG_RECORD_SIZE = struct.calcsize(_LOG_RECORD_FMT)
_LOG_CSV_HEADER  = (
    "T_MS,ENC_QR,ENC_QI,ENC_QJ,ENC_QK,IMU_QR,IMU_QI,IMU_QJ,IMU_QK,"
    "GYRO_X,ANG_ERR,ANG_P,ANG_I,ANG_D,RATE_SP,RATE_ERR,RATE_P,RATE_I,"
    "RATE_D,PID_OUT,M1,M2,DT_MS,MAX_DT_MS"
)

PYTHON = sys.executable  # use same interpreter that's running this script

# -- Configuration -------------------------------------------------------------
COM_PORT   = "COM7"
REMOTE_DIR = "/sd/flights"
LOCAL_DIR  = Path("test_runs/flights")

# -- Pico-side SD mount boilerplate --------------------------------------------
_SD_MOUNT = """\
import os, time
from machine import SPI, Pin
import sdcard
_cs  = Pin(17, Pin.OUT, value=1)
_spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
           sck=Pin(18), mosi=Pin(19), miso=Pin(16))
time.sleep_ms(250)
_sd  = sdcard.SDCard(_spi, _cs)
os.mount(os.VfsFat(_sd), '/sd')
"""

# -- Pico-side scripts ---------------------------------------------------------

_LIST_SCRIPT = _SD_MOUNT + """\
try:
    for name in sorted(os.listdir('/sd/flights')):
        print(name)
finally:
    os.umount('/sd')
"""


def _transfer_script(flight_ids):
    return _SD_MOUNT + f"""\
import ubinascii
try:
    for fid in {flight_ids!r}:
        for fname in ('config.json', 'specification.json', 'log.bin'):
            path = '/sd/flights/' + fid + '/' + fname
            try:
                size = os.stat(path)[6]
                print('BEGIN_FILE ' + fid + '/' + fname + ' ' + str(size))
                with open(path, 'rb') as f:
                    while True:
                        chunk = f.read(192)
                        if not chunk:
                            break
                        print(ubinascii.b2a_base64(chunk).decode(), end='')
                print('END_FILE ' + fid + '/' + fname)
            except OSError as e:
                print('PICO_ERROR ' + fid + '/' + fname + ' ' + str(e))
finally:
    os.umount('/sd')
"""


def _decode_log_bin(raw):
    """Decode a binary log.bin into a CSV string."""
    n = len(raw) // _LOG_RECORD_SIZE
    lines = [_LOG_CSV_HEADER]
    for i in range(n):
        (t_ms, eqr, eqi, eqj, eqk, iqr, iqi, iqj, iqk,
         gyro_x, ang_err, ang_p, ang_i, ang_d, rate_sp,
         rate_err, rate_p, rate_i, rate_d, pid_out,
         m1, m2, dt_ms, max_dt) = struct.unpack_from(_LOG_RECORD_FMT, raw, i * _LOG_RECORD_SIZE)
        lines.append(
            f"{t_ms},{eqr:.5f},{eqi:.5f},{eqj:.5f},{eqk:.5f},"
            f"{iqr:.5f},{iqi:.5f},{iqj:.5f},{iqk:.5f},"
            f"{gyro_x:.2f},{ang_err:.2f},{ang_p:.2f},{ang_i:.2f},{ang_d:.2f},"
            f"{rate_sp:.2f},{rate_err:.2f},{rate_p:.2f},{rate_i:.2f},{rate_d:.2f},"
            f"{pid_out:.2f},{m1},{m2},{dt_ms},{max_dt}"
        )
    return "\n".join(lines) + "\n"


def _delete_script(flight_ids):
    return _SD_MOUNT + f"""\
try:
    for fid in {flight_ids!r}:
        base = '/sd/flights/' + fid
        try:
            for fname in os.listdir(base):
                try:
                    os.remove(base + '/' + fname)
                except OSError:
                    pass
        except OSError:
            pass
        try:
            os.rmdir(base)
            print('DELETED ' + fid)
        except OSError as e:
            print('DELETE_FAILED ' + fid + ' ' + str(e))
finally:
    os.umount('/sd')
"""


# -- mpremote runner -----------------------------------------------------------

def _run_on_pico(code, timeout=120):
    """Write code to a temp file and run it on the Pico via mpremote run."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False,
                                     encoding='utf-8') as tf:
        tf.write(code)
        tf_path = tf.name
    try:
        result = subprocess.run(
            [PYTHON, '-m', 'mpremote', 'connect', COM_PORT, 'run', tf_path],
            capture_output=True, text=True, timeout=timeout,
        )
    finally:
        os.unlink(tf_path)
    if result.returncode != 0:
        sys.exit(f"mpremote failed:\n{result.stderr.strip()}")
    return result.stdout


# -- Transfer output parser ----------------------------------------------------

def _parse_transfer(output):
    """Parse base64-encoded structured output from the transfer script.

    Returns (files, expected_sizes) where files maps 'fid/fname' -> bytes.
    """
    files = {}
    expected_sizes = {}
    current_key = None
    b64_lines = []

    for line in output.splitlines():
        if line.startswith('BEGIN_FILE '):
            _, key, size = line.split(' ', 2)
            current_key = key
            expected_sizes[key] = int(size)
            b64_lines = []
        elif line.startswith('END_FILE '):
            if current_key:
                files[current_key] = base64.b64decode(''.join(b64_lines))
            current_key = None
            b64_lines = []
        elif line.startswith('PICO_ERROR '):
            print(f"  {line}")
        elif current_key is not None:
            b64_lines.append(line.strip())

    return files, expected_sizes


# -- Core operations -----------------------------------------------------------

def list_remote():
    out = _run_on_pico(_LIST_SCRIPT)
    return sorted(line.strip() for line in out.splitlines() if line.strip())


def list_local():
    if not LOCAL_DIR.exists():
        return set()
    return {p.name for p in LOCAL_DIR.iterdir() if p.is_dir()}


def fetch(new_ids, transfer_timeout=120):
    """Transfer files from SD to local. Returns (ok_ids, failed_ids)."""
    print(f"\nTransferring {len(new_ids)} run(s)...")
    output = _run_on_pico(_transfer_script(new_ids), timeout=transfer_timeout)
    files, expected = _parse_transfer(output)

    ok, failed = [], []
    for fid in new_ids:
        success = True
        for fname in ('config.json', 'specification.json', 'log.bin'):
            key = f"{fid}/{fname}"
            data = files.get(key)
            if data is None:
                print(f"  MISSING  {key}")
                success = False
                continue
            if len(data) != expected.get(key, -1):
                print(f"  MISMATCH {key}: expected {expected[key]}B got {len(data)}B")
                success = False
                continue
            dest = LOCAL_DIR / fid
            dest.mkdir(parents=True, exist_ok=True)
            if fname == 'log.bin':
                (dest / 'log.csv').write_text(_decode_log_bin(data), encoding='utf-8')
            else:
                (dest / fname).write_bytes(data)
        if success:
            ok.append(fid)
            print(f"  OK   {fid}")
        else:
            failed.append(fid)
            print(f"  FAIL {fid}")

    return ok, failed


def delete_from_sd(ok_ids):
    print(f"\nDeleting {len(ok_ids)} run(s) from SD card...")
    out = _run_on_pico(_delete_script(ok_ids))
    for line in out.splitlines():
        if line.startswith('DELETED '):
            print(f"  deleted {line[8:]}")
        elif line.startswith('DELETE_FAILED '):
            print(f"  WARNING: {line}")


# -- Main ----------------------------------------------------------------------

def main():
    erase_leftovers = '--erase' in sys.argv

    print(f"Connecting to Pico on {COM_PORT} -- listing SD card...")
    remote = list_remote()
    local  = list_local()
    new_ids = sorted(set(remote) - local)

    print(f"  SD card : {len(remote)} run(s)  |  local: {len(local)} run(s)")

    if not new_ids:
        print("Nothing new to pull.")
        return

    print(f"\nNew ({len(new_ids)}):")
    for fid in new_ids:
        print(f"  {fid}")

    transfer_timeout = max(120, len(new_ids) * 30)  # 30s per run; SD now truncated to actual data
    ok_ids, failed_ids = fetch(new_ids, transfer_timeout=transfer_timeout)

    if ok_ids:
        delete_from_sd(ok_ids)

    print(f"\nDone: {len(ok_ids)} pulled, {len(failed_ids)} failed.")
    if failed_ids:
        if erase_leftovers:
            print(f"\nErasing {len(failed_ids)} leftover(s) from SD card...")
            delete_from_sd(failed_ids)
        else:
            print("Failed runs remain on SD card:")
            for fid in failed_ids:
                print(f"  {fid}")
            print("  (re-run with --erase to delete them)")


if __name__ == '__main__':
    main()
