# Comms

Packet protocol between the robot and a host over any byte stream, normally an
HC-05 in slave data mode. Pure C++, fixed storage, no HAL: host-tested in
`test/test_comms`. The USART1 transport is firmware glue in
[remote.h](../../include/remote.h); the host side is
[tools/console.py](../../tools/console.py). Change all three together.

## Frame

```
COBS( type u8 | seq u8 | payload 0..64 B | CRC-16/CCITT-FALSE u16 LE ) 0x00
```

COBS removes every `0x00` from the body, so `0x00` always ends a frame. After
noise, a dropped byte, or plain text, the receiver drops one frame and resyncs
at the next delimiter. The CRC covers type, seq and payload. `seq` counts per
sender and wraps at 256. All multi-byte fields are little-endian.

| Type | Dir | Payload |
|---|---|---|
| `0x01` Telemetry | robot → host | 27 B, below |
| `0x02` Log | robot → host | text; `printf` output, split at 64 B |
| `0x03` Ack | robot → host | command type u8, command seq u8 |
| `0x10` Start | host → robot | none; same as pressing the button |
| `0x11` Stop | host → robot | none; ends the run, motors off |
| `0x12` Ping | host → robot | none |
| `0x13` Drive | host → robot | left, right wheel speed i16 each, mm/s, forward positive |

Drive is for manual mode (below) and is ignored by the other modes. It is
streamed, so it gets no Ack; telemetry shows its effect. Each Start, Stop and Ping gets an Ack, which means the command was received,
not that it changed anything. Telemetry shows the effect: Start does nothing
unless the robot is Ready, and Stop ends a navigation run for good
(`Supervisor::done()`), so reset the board to run again. In manual mode Stop
only disarms.

Telemetry (`comms::Telemetry`, sent every `cfg::TELEMETRY_PERIOD_MS`, 100 ms):

| Field | Type | Unit |
|---|---|---|
| ms | u32 | HAL tick |
| x, y | i16 | mm |
| th | i16 | mrad |
| v | i16 | mm/s, forward |
| target L, R | i16 | mm/s, wheel targets this step |
| range ×4 | u16 | mm, clamped ToF in sensor order |
| valid | u8 | bit i: range i is fresh, valid data |
| state, fault | u8 | `reliability::State`, `reliability::Fault` |

`pack()` saturates out-of-range values and sends non-finite ones as 0.

## Manual driving

`pio run -e manual -t upload` builds a mode with no navigation: the host sets the
wheel speeds. Same health rules as the other modes, except the ToF sensors are
optional. Wheels off the ground for the first try.

```
> start              # arm (or press the button); Ready blinks, Running is solid
> drive 100 100      # both wheels forward, mm/s
> drive -80 80       # turn left in place
> drive 0 0          # stop the wheels
> stop               # disarm; Ready again after 0.5 s standing still
```

The console resends the last `drive` 20 times a second. The robot clamps each
wheel to `cfg::MANUAL_MAX_MM_S` (300) and stops the wheels when no Drive packet has
arrived for `cfg::MANUAL_TIMEOUT_MS` (300 ms), so a closed console, a dropped
Bluetooth link or a dead host all stop the robot. A Drive sent before `start` is
discarded. A sensor, timing or pose fault disables the motors and stays latched
until reset, same as the other modes.

## Bandwidth

One telemetry frame is about 32 bytes on the wire. At 9600 baud (the HC-05
default, ~960 B/s) 10 Hz telemetry uses a third of the link. Logs share the
rest. The firmware's TX buffer is 1 KB. A frame that does not fit is dropped
whole and counted in `gRemote.txDropped`; nothing blocks the control loop.
The firmware (`cfg::UART1_BAUD_RATE`) and the console default to 9600, the
HC-05 factory setting. For more headroom, set the module to 115200
(`AT+UART=115200,0,0`), set the constant to match, and pass `--baud 115200`.

## Using it

```
python tools/console.py /dev/tty.HC-05-DevB --baud 9600 --csv run.csv
> start
> stop
```

`pio run -e comms_test -t upload` builds a stationary link check. It sends a log
line and a telemetry packet once a second and acknowledges commands. Start
lights the status LED and Stop clears it. Wheels never move.

`tof_test` builds with `-DREMOTE_PLAIN_TEXT`, so its `printf` stays plain text
for `pio device monitor`. Every other build carries packets only, so read it
with the console, not a serial terminal.

## HC-05 wiring and setup

`LAYOUT.JPG` assigns USART1 TX to PA9 and RX to PA10. Connect PA9 to module RX,
PA10 to module TX, and share ground. Module power must match your breakout's
specification. KEY/EN and STATE are not wired, so the firmware cannot detect a
Bluetooth connection. A phone or computer initiates the connection, and the MCU
only sends and receives UART bytes.

Configure the module once using an external UART adapter and its KEY/EN
procedure: `AT+ROLE=0` selects slave mode, and `AT+UART=9600,0,0` selects 9600
baud, 8N1. Check each response for OK, then leave AT mode and restart in data
mode. AT baud and KEY behaviour vary by firmware; follow your module's
instructions. See the [ITead HC-05 datasheet](https://www.rajguruelectronics.com/Product/705/hc-05%20Bluetooth%20module.pdf).

A successful UART write does not confirm Bluetooth delivery. Use Ping/Ack to
check the round trip.
