"""Read the two course runs from the robot's flash over ST-Link (OpenOCD from PlatformIO) and print them.

    python3 tools/read_run.py                 # both slots: summary, 400 ms trace and the fast log
    python3 tools/read_run.py --csv data/x    # also write x_run1.csv and x_run2.csv (the fast logs)
    python3 tools/analyze.py                  # read, compare and plot

Flash layout (src/run_record.cpp): two slots of 6 KB. 0x0800D000 holds run 1 (no random block; the detection reference
when it finished cleanly), 0x0800E800 holds run 2. A slot is: 8-byte header (magic, run number) + Trace at 0..511, then up
to 352 16-byte fast-log records. Wheel speeds are stored in 8 mm/s units. Byte 8 of a record is the 90-degree right ToF
range (5 mm units) in logs with magic 0xC0DE0006/7, the steering angle (10 mrad) in the older 0xC0DE0004/5 logs. Both
are read; the CSV always has the 12 old columns, then tofS (right 90, mm; 1200 for old logs), and alpha_mrad is 0 for
new logs.
"""
import argparse, os, struct, subprocess, tempfile

OPENOCD = os.path.expanduser("~/.platformio/packages/tool-openocd")
SLOTS = (0x0800D000, 0x0800E800)
SLOT_BYTES = 6 * 1024
TRACE_BYTES = 512
REC = 16
MAGIC = 0xC0DE0006
REF_MAGIC = 0xC0DE0007  # run 1 that serves as the reference
OLD_MAGIC = 0xC0DE0004  # byte 8 = steering angle instead of the right-90 ToF
OLD_REF_MAGIC = 0xC0DE0005
SPEED_UNIT = 8  # mm/s
COLS = "ms x y th_mrad alpha_mrad tgtL tgtR measL measR tofR tofF tofL tofS".split()


def dump(addr, size):
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        path = f.name
    cmd = [OPENOCD + "/bin/openocd", "-s", OPENOCD + "/openocd/scripts", "-f", "interface/stlink.cfg",
           "-f", "target/stm32f1x.cfg", "-c", "init", "-c", f"dump_image {path} {addr:#x} {size}", "-c", "shutdown"]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    data = open(path, "rb").read()
    os.unlink(path)
    return data


def parse_slot(d):
    magic, seq = struct.unpack_from("<2I", d, 0)
    valid = magic in (MAGIC, REF_MAGIC, OLD_MAGIC, OLD_REF_MAGIC)
    old = magic in (OLD_MAGIC, OLD_REF_MAGIC)
    t = {}
    if valid:
        o = 8
        t["samples"], t["replans"], t["unmapped"], t["doneAtMs"], t["stackFree"] = struct.unpack_from("<5I", d, o)
        t["state"], t["fault"], t["lastFault"], t["resetFlags"], t["extras"] = struct.unpack_from("<5B", d, o + 20)
        t["extraX"] = struct.unpack_from("<8h", d, o + 26)
        t["extraY"] = struct.unpack_from("<8h", d, o + 42)
        t["reference"] = magic in (REF_MAGIC, OLD_REF_MAGIC)
        t["x"] = struct.unpack_from("<48h", d, o + 58)
        t["y"] = struct.unpack_from("<48h", d, o + 154)
        t["th"] = struct.unpack_from("<48h", d, o + 250)
    rows = []
    for i in range((SLOT_BYTES - TRACE_BYTES) // REC):
        b = d[TRACE_BYTES + i * REC: TRACE_BYTES + (i + 1) * REC]
        if b[0:2] == b"\xff\xff":
            break
        ms, x, y, th, al, tl, tr, ml, mr, r, f, l = struct.unpack("<Hhhhb4b3B", b)
        if old:
            alpha, side = al * 10, 1200
        else:
            alpha, side = 0, b[8] * 5
        rows.append((ms, x, y, th, alpha, tl * SPEED_UNIT, tr * SPEED_UNIT, ml * SPEED_UNIT, mr * SPEED_UNIT, r * 5, f * 5, l * 5, side))
    return valid, seq, t, rows


def show(name, valid, seq, t, rows, csv_prefix):
    kind = " (reference)" if valid and t["reference"] else ""
    print(f"=== {name}{kind}: " + (f"#{seq}" if valid else "no finished run (empty, or the run never ended)") + f", {len(rows)} log records")
    if valid:
        print(f"  {t['doneAtMs']} ms, replans {t['replans']}, unmapped {t['unmapped']}, state {t['state']} fault {t['fault']} "
              f"lastFault {t['lastFault']}")
        ex = [(t['extraX'][i], t['extraY'][i]) for i in range(min(t['extras'], 8))]
        if ex:
            print("  unmapped blocks found at", ex)
        for i in range(min(t["samples"], 48)):
            print(f"  {i:2d} ({t['x'][i]:5d},{t['y'][i]:5d}) th {t['th'][i] / 1000:6.3f}")
    if csv_prefix and rows:
        path = f"{csv_prefix}_{name}.csv"
        with open(path, "w") as f:
            f.write(",".join(COLS) + "\n")
            for r in rows:
                f.write(",".join(map(str, r)) + "\n")
        print("  wrote", path)
    elif rows and not csv_prefix:
        print(" ".join(f"{c:>8}" for c in COLS))
        for r in rows[::3]:
            print(" ".join(f"{v:8d}" for v in r))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", metavar="PREFIX")
    a = ap.parse_args()
    for name, addr in zip(("run1", "run2"), SLOTS):
        show(name, *parse_slot(dump(addr, SLOT_BYTES)), a.csv)
