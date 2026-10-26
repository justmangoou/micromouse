"""Host console for the robot's comms link (lib/comms): shows Log text and telemetry, sends commands.

    python tools/console.py /dev/tty.HC-05-DevB --baud 9600 [--csv run.csv]
    python tools/console.py --selftest

Type start, stop or ping and Enter; quit or Ctrl-C to leave. With the manual build (env:manual), start arms the
robot and "drive L R" sets the wheel speeds in mm/s (forward positive, e.g. "drive 100 100", "drive -80 80" turns
left). The console resends the last drive 20 times a second; the robot stops its wheels when the packets stop, and
"drive 0 0" or "stop" stops them. Needs pyserial (PlatformIO's Python has it:
~/.platformio/penv/bin/python). Mirrors lib/comms/include/comms.h; change both together.
"""
import argparse
import csv
import struct
import sys
import threading
import time

TELEMETRY, LOG, ACK = 0x01, 0x02, 0x03
START, STOP, PING, DRIVE = 0x10, 0x11, 0x12, 0x13
COMMANDS = {"start": START, "stop": STOP, "ping": PING}
NAMES = {TELEMETRY: "telemetry", LOG: "log", ACK: "ack", START: "start", STOP: "stop", PING: "ping", DRIVE: "drive"}
MAX_PAYLOAD = 64
DRIVE_PERIOD_S = 0.05  # robot stops after cfg::MANUAL_TIMEOUT_MS (300 ms) without a Drive packet

# reliability::State and reliability::Fault, in declaration order.
STATES = ["Ready", "Running", "Recovering", "Done", "Fault"]
FAULTS = ["None", "Sensor", "Timing", "CommandExpired", "NoRoute", "Pose", "Configuration", "Timeout"]

# ms; x, y (mm); th (mrad); v, target L/R (mm/s); 4 ranges (mm); valid mask, state, fault
TELEMETRY_FORMAT = struct.Struct("<I6h4H3B")
TELEMETRY_FIELDS = ["ms", "x", "y", "th", "v", "target_l", "target_r",
                    "range0", "range1", "range2", "range3", "valid", "state", "fault"]

GOLDEN_PING = bytes([0x05, 0x12, 0x07, 0xF9, 0x08, 0x00])  # Ping, seq 7; also checked in test/test_comms


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out += bytes([len(block) + 1]) + block
            block = bytearray()
        else:
            block.append(b)
            if len(block) == 254:
                out += b"\xff" + block
                block = bytearray()
    return bytes(out + bytes([len(block) + 1]) + block)


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            return None
        out += data[i + 1:i + code]
        i += code
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode(ptype, seq, payload=b""):
    body = bytes([ptype, seq & 0xFF]) + payload
    return cobs_encode(body + struct.pack("<H", crc16(body))) + b"\x00"


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        """Yields (type, seq, payload) for each valid frame completed by data."""
        for b in data:
            if b != 0:
                self.buffer.append(b)
                continue
            frame, self.buffer = bytes(self.buffer), bytearray()
            if not frame:
                continue
            body = cobs_decode(frame)
            if (body is None or len(body) < 4 or len(body) - 4 > MAX_PAYLOAD
                    or crc16(body[:-2]) != struct.unpack("<H", body[-2:])[0]):
                self.errors += 1
                continue
            yield body[0], body[1], body[2:-2]


def pack_drive(left, right):
    clamp = lambda v: max(-32768, min(32767, int(round(v))))
    return struct.pack("<hh", clamp(left), clamp(right))


def unpack_telemetry(payload):
    if len(payload) != TELEMETRY_FORMAT.size:
        return None
    return dict(zip(TELEMETRY_FIELDS, TELEMETRY_FORMAT.unpack(payload)))


def describe(t):
    state = STATES[t["state"]] if t["state"] < len(STATES) else t["state"]
    fault = FAULTS[t["fault"]] if t["fault"] < len(FAULTS) else t["fault"]
    ranges = " ".join("%4d%s" % (t["range%d" % i], "" if t["valid"] >> i & 1 else "?") for i in range(4))
    return "%8.2fs %-10s %-14s x%6d y%6d th%6.3f v%5d tgt%5d%5d  tof %s" % (
        t["ms"] / 1000, state, fault, t["x"], t["y"], t["th"] / 1000, t["v"], t["target_l"], t["target_r"], ranges)


def selftest():
    assert crc16(b"123456789") == 0x29B1
    assert encode(PING, 7) == GOLDEN_PING
    payload = bytes(i if i % 3 else 0 for i in range(MAX_PAYLOAD))
    assert list(Decoder().feed(b"junk\x00" + encode(LOG, 200, payload))) == [(LOG, 200, payload)]
    assert TELEMETRY_FORMAT.size == 27  # comms::TELEMETRY_SIZE
    assert pack_drive(-250.4, 99999) == bytes([0x06, 0xFF, 0xFF, 0x7F])  # test_comms: -250, saturated to 32767
    print("selftest ok")


def run(port, baud, csv_path):
    import serial  # pyserial

    link = serial.Serial(port, baud, timeout=0.1)
    writer = None
    if csv_path:
        csv_file = open(csv_path, "w", newline="")
        writer = csv.DictWriter(csv_file, TELEMETRY_FIELDS)
        writer.writeheader()
    decoder = Decoder()
    stopping = threading.Event()

    def reader():
        line = ""
        while not stopping.is_set():
            for ptype, seq, payload in decoder.feed(link.read(256)):
                if ptype == LOG:
                    line += payload.decode("utf-8", "replace")
                    while "\n" in line:
                        text, line = line.split("\n", 1)
                        print("log  " + text.rstrip("\r"))
                elif ptype == TELEMETRY and (t := unpack_telemetry(payload)):
                    print("tel  " + describe(t))
                    if writer:
                        writer.writerow(t)
                elif ptype == ACK and len(payload) == 2:
                    print("ack  %s seq %d" % (NAMES.get(payload[0], hex(payload[0])), payload[1]))
                else:
                    print("?    type 0x%02x seq %d, %d bytes" % (ptype, seq, len(payload)))

    seq = 0
    drive = None  # (left, right) while a drive is active
    lock = threading.Lock()

    def send(ptype, payload=b""):
        nonlocal seq
        with lock:
            link.write(encode(ptype, seq, payload))
            seq = (seq + 1) & 0xFF

    def driver():
        while not stopping.is_set():
            if drive:
                send(DRIVE, pack_drive(*drive))
            time.sleep(DRIVE_PERIOD_S)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    drive_thread = threading.Thread(target=driver, daemon=True)
    drive_thread.start()
    print("connected to %s at %d baud; commands: %s, drive L R, quit" % (port, baud, ", ".join(COMMANDS)))
    try:
        for command in sys.stdin:
            command = command.strip().lower()
            if command in ("quit", "exit"):
                break
            words = command.split()
            if command in COMMANDS:
                if command == "stop":
                    drive = None
                send(COMMANDS[command])
            elif len(words) == 3 and words[0] == "drive":
                try:
                    drive = (float(words[1]), float(words[2]))
                except ValueError:
                    print("usage: drive LEFT RIGHT (mm/s)")
                    continue
                if drive == (0, 0):
                    send(DRIVE, pack_drive(0, 0))
                    drive = None
            elif command:
                print("unknown command %r" % command)
    except KeyboardInterrupt:
        pass
    drive = None
    try:
        send(DRIVE, pack_drive(0, 0))  # leave with the wheels stopped; the timeout covers a dead link
    except Exception:
        pass
    stopping.set()
    thread.join(timeout=1)
    if decoder.errors:
        print("%d corrupt frames dropped" % decoder.errors)
    if writer:
        csv_file.close()
    link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", nargs="?")
    parser.add_argument("--baud", type=int, default=9600, help="cfg::UART1_BAUD_RATE (default 9600)")
    parser.add_argument("--csv", help="also write every telemetry packet to this CSV file")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        selftest()
    elif args.port:
        run(args.port, args.baud, args.csv)
    else:
        parser.error("port required")


if __name__ == "__main__":
    main()
