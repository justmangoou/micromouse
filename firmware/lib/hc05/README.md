# HC-05

STM32 HAL UART driver, fixed storage, no allocation. `LAYOUT.JPG` assigns
USART1 TX to PA9 and RX to PA10. Connect PA9 to module RX, PA10 to module TX,
and share ground. Module power must match your breakout's specification.
The layout does not assign KEY/EN or STATE pins; manage AT-mode entry on the
module itself. The library does not assume those pins are wired.

`boardInit()` configures USART1 and exposes `bluetooth`. Set
`cfg::UART1_BAUD_RATE` in `include/config.h` to match the module. The existing
460800 console baud is preserved by default. `printf` uses this same UART,
so logs will also go to Bluetooth when it is connected. Set PlatformIO's
`monitor_speed` to match if using a wired monitor.

```cpp
#include "board.h"
#include "hc05.h"

// After boardInit(), from the main loop:
if (!bluetooth.write("hello\r\n")) {
    // Inspect bluetooth.lastStatus() / lastError().
}
if (auto byte = bluetooth.readByte()) {
    // Handle an incoming byte. Empty means no byte or a UART error.
}
```

For AT configuration, enter AT mode using the module's KEY/EN procedure and
configure the host UART to the module's AT baud (commonly 38400):

```cpp
if (bluetooth.init(38400) && bluetooth.sendAt("AT+UART?")) {
    // Read the response with readByte(timeoutMs); check for OK / ERROR.
}
```

`init()` changes only the MCU UART, not the HC-05 configuration or its mode.
`sendAt()` appends CRLF and accepts commands of at most 126 bytes without line
breaks; success reports transmission, not an OK response. Restore the host
baud after leaving AT mode. Default baud and KEY behavior vary by firmware;
use your module's settings. See the [ITead HC-05 datasheet](https://www.rajguruelectronics.com/Product/705/hc-05%20Bluetooth%20module.pdf).

All operations are polled. Writes may block for the constructor's timeout
(100 ms by default); readByte defaults to a zero timeout. Poll frequently or
add interrupt/DMA buffering before receiving sustained traffic. Avoid calls
from interrupts and concurrent writes through printf during AT exchanges.
