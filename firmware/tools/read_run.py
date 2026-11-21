"""Read the last two course runs from the robot's flash over ST-Link (OpenOCD from PlatformIO) and print them.

    python3 tools/read_run.py                 # both slots, oldest first: summary, 400 ms trace and the fast log
    python3 tools/read_run.py --csv run       # also write run_<n>.csv for each slot's fast log (n = run number)
    python3 tools/read_run.py --last          # only the newest run

Flash layout (src/course.cpp): two slots of 5 KB at 0x0800D800 and 0x0800EC00; each run goes into the older slot.
A slot is: 8-byte header (magic, run number) + CourseTrace at 0..511, then 16-byte fast-log records.
"""
import argparse, os, struct, subprocess, tempfile

OPENOCD = os.path.expanduser("~/.platformio/packages/tool-openocd")
SLOTS = (0x0800D800, 0x0800EC00)
SLOT_BYTES = 5 * 1024
TRACE_BYTES = 512
REC = 16
MAGIC = 0xC0DE0002
COLS = "ms x y th_mrad alpha_mrad tgtL tgtR measL measR tofR tofF tofL".split()


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
    valid = magic == MAGIC
    t = {}
    if valid:
        o = 8
        t["samples"], t["replans"], t["unmapped"], t["doneAtMs"], t["stackFree"] = struct.unpack_from("<5I", d, o)
        t["state"], t["fault"], t["lastFault"], t["resetFlags"], t["extras"] = struct.unpack_from("<5B", d, o + 20)
        t["extraX"] = struct.unpack_from("<8h", d, o + 26)
        t["extraY"] = struct.unpack_from("<8h", d, o + 42)
        t["x"] = struct.unpack_from("<48h", d, o + 58)
        t["y"] = struct.unpack_from("<48h", d, o + 154)
        t["th"] = struct.unpack_from("<48h", d, o + 250)
    rows = []
    for i in range((SLOT_BYTES - TRACE_BYTES) // REC):
        b = d[TRACE_BYTES + i * REC: TRACE_BYTES + (i + 1) * REC]
        if b[0:2] == b"\xff\xff":
            break
        ms, x, y, th, al, tl, tr, ml, mr, r, f, l = struct.unpack("<Hhhhb4b3B", b)
        rows.append((ms, x, y, th, al * 10, tl * 5, tr * 5, ml * 5, mr * 5, r * 5, f * 5, l * 5))
    return valid, seq, t, rows


def show(name, valid, seq, t, rows, csv_prefix):
    print(f"=== slot {name}: " + (f"run #{seq}" if valid else "no finished run (empty, or the run never ended)") + f", {len(rows)} log records")
    if valid:
        print(f"  {t['doneAtMs']} ms, replans {t['replans']}, unmapped {t['unmapped']}, state {t['state']} fault {t['fault']} "
              f"lastFault {t['lastFault']}")
        ex = [(t['extraX'][i], t['extraY'][i]) for i in range(min(t['extras'], 8))]
        if ex:
            print("  unmapped blocks found at", ex)
        for i in range(min(t["samples"], 48)):
            print(f"  {i:2d} ({t['x'][i]:5d},{t['y'][i]:5d}) th {t['th'][i] / 1000:6.3f}")
    if csv_prefix and rows:
        path = f"{csv_prefix}_{seq if valid else name}.csv"
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
    ap.add_argument("--last", action="store_true")
    a = ap.parse_args()
    slots = [parse_slot(dump(addr, SLOT_BYTES)) for addr in SLOTS]
    order = sorted(range(2), key=lambda i: slots[i][1] if slots[i][0] else 0)
    if a.last:
        order = order[-1:]
    for i in order:
        show("AB"[i], *slots[i], a.csv)
