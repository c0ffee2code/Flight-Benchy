"""Verify sign relationship between GYRO_X, ENC_ROLL derivative, and IMU quaternion roll.

Also estimate GRV lag vs encoder, and test which feedforward sign better
reconstructs the true (encoder) angle:
    A (current code): imu_roll + gyro_x * lead
    B (flipped):      imu_roll - gyro_x * lead
"""
import csv
import math
import sys

PATH = r"C:\Important\Intellij Projects\Flight Benchy\test_runs\flights\2026-06-07_13-48-31\log.csv"
LEAD_S = 0.012

t, enc, gx, qr, qi = [], [], [], [], []
with open(PATH) as f:
    for row in csv.DictReader(f):
        t.append(int(row["T_MS"]) / 1000.0)
        enc.append(float(row["ENC_ROLL"]))
        gx.append(float(row["GYRO_X"]))
        qr.append(float(row["IMU_QR"]))
        qi.append(float(row["IMU_QI"]))

n = len(t)
imu_roll = [math.degrees(2.0 * math.atan2(qi[k], qr[k])) for k in range(n)]

# encoder angular rate (central difference)
denc = [0.0] * n
for k in range(1, n - 1):
    dt = t[k + 1] - t[k - 1]
    denc[k] = (enc[k + 1] - enc[k - 1]) / dt if dt > 0 else 0.0

def corr(a, b):
    m = len(a)
    ma = sum(a) / m
    mb = sum(b) / m
    num = sum((a[k] - ma) * (b[k] - mb) for k in range(m))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((x - mb) ** 2 for x in b))
    return num / (da * db) if da > 0 and db > 0 else float("nan")

print(f"rows: {n}, span: {t[-1]-t[0]:.1f}s")
print(f"corr(IMU_roll, ENC_ROLL)        = {corr(imu_roll, enc):+.4f}")
print(f"corr(GYRO_X, d(ENC_ROLL)/dt)    = {corr(gx[1:-1], denc[1:-1]):+.4f}")

# RMS error of feedforward variants vs encoder (encoder ~ true angle, near-zero latency)
err_raw = [imu_roll[k] - enc[k] for k in range(n)]
err_a = [imu_roll[k] + gx[k] * LEAD_S - enc[k] for k in range(n)]
err_b = [imu_roll[k] - gx[k] * LEAD_S - enc[k] for k in range(n)]

def rms(e):
    return math.sqrt(sum(x * x for x in e) / len(e))

print(f"RMS(imu_roll - enc)                    = {rms(err_raw):.4f} deg")
print(f"RMS(imu_roll + gyro*lead - enc)  [code] = {rms(err_a):.4f} deg")
print(f"RMS(imu_roll - gyro*lead - enc) [flip]  = {rms(err_b):.4f} deg")

# estimate GRV lag: shift imu_roll forward k samples, find best match to encoder
# sample interval:
dts = [(t[k+1] - t[k]) for k in range(n - 1)]
dt_med = sorted(dts)[len(dts) // 2]
print(f"median record interval: {dt_med*1000:.1f} ms")
best = None
for shift in range(0, 8):
    # imu_roll[k] corresponds to enc[k - shift] if GRV lags by shift samples
    e = [imu_roll[k] - enc[k - shift] for k in range(shift, n)]
    r = rms(e)
    tag = ""
    if best is None or r < best[1]:
        best = (shift, r)
    print(f"  lag {shift} samples ({shift*dt_med*1000:.0f} ms): RMS = {r:.4f}")
print(f"best lag ~ {best[0]*dt_med*1000:.0f} ms (RMS {best[1]:.4f})")