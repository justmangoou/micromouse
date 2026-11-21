"""Read and analyse the motor step test (env:test_step) over ST-Link.

    python3 tools/read_step.py            # needs the built ELF in .pio/build/test_step for the address of gStep
Prints, per duty level and wheel: steady speed, speed per unit duty, the time to 63% of the steady speed after the step
on, and the time constant of the decay after the step off.
"""
import os, struct, subprocess, sys, tempfile

OPENOCD = os.path.expanduser("~/.platformio/packages/tool-openocd")
NM = os.path.expanduser("~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm")
ELF = os.path.join(os.path.dirname(__file__), "..", ".pio", "build", "test_step_floor", "firmware.elf")
MM_PER_COUNT = 2 * 3.14159265 * 16.5 / 270  # config.h: wheel radius 16.5 mm, 270 counts per revolution
LEVELS, N = 7, 1680
SIZE = 26 + N * 2 + N * 2


def address():
    out = subprocess.run([NM, ELF], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        if line.endswith(" gStep"):
            return int(line.split()[0], 16)
    sys.exit("gStep not found in the ELF")


def dump(addr, size):
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        path = f.name
    cmd = [OPENOCD + "/bin/openocd", "-s", OPENOCD + "/openocd/scripts", "-f", "interface/stlink.cfg",
           "-f", "target/stm32f1x.cfg", "-c", "init", "-c", f"dump_image {path} {addr:#x} {size}", "-c", "shutdown"]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    d = open(path, "rb").read()
    os.unlink(path)
    return d


def main():
    d = dump(address(), SIZE)
    state, n, used = struct.unpack_from("<3I", d, 0)
    levels = struct.unpack_from(f"<{LEVELS}h", d, 12)
    ms = struct.unpack_from(f"<{N}H", d, 26)
    dL = struct.unpack_from(f"<{N}b", d, 26 + 2 * N)
    dR = struct.unpack_from(f"<{N}b", d, 26 + 3 * N)
    print(f"state {state} (3 = done), {n} samples")
    if state != 3 or n < used * 240:
        sys.exit("the test has not finished")
    # speed in mm/s from counts per sample interval, lightly smoothed over 3 samples
    def speeds(c):
        v = []
        for i in range(N):
            dt = (ms[i] - (ms[i - 1] if i else 0)) / 1000.0
            v.append(c[i] * MM_PER_COUNT / dt if dt > 0 else 0.0)
        return [sum(v[max(0, i - 1):i + 2]) / len(v[max(0, i - 1):i + 2]) for i in range(N)]
    for name, c in (("L", dL), ("R", dR)):
        v = speeds(c)
        print(f"\nwheel {name}: level  duty  steady mm/s  mm/s per duty  t63% ms   decay tau ms   start ms")
        per = 1200 // 5
        for k, lv in enumerate(levels[:used]):
            a = k * per
            on = v[a:a + 140]  # 0.7 s on            # 0.7 s on
            off = v[a + 140:a + 240]     # 0.5 s off
            steady = sum(on[-30:]) / 30  # last 150 ms of the on phase
            # first sample above 5% of steady
            start = next((i for i, x in enumerate(on) if steady > 20 and x > 0.05 * steady), None)
            t63 = next((i for i, x in enumerate(on) if steady > 20 and x >= 0.63 * steady), None)
            decay = None
            if steady > 20:
                decay = next((i for i, x in enumerate(off) if x <= 0.37 * steady), None)
            f = lambda i: "  --" if i is None else f"{i * 5:4d}"
            print(f"          {k}  {lv / 1000:4.1f}  {steady:10.0f}  {steady / (lv / 1000):13.0f}  {f(t63)}   {f(decay)}        {f(start)}")


if __name__ == "__main__":
    main()
