# I2C bus

Shared STM32 HAL register read/write helpers for the IMU and ToF sensors.

Flow: configure HAL I2C → create `I2cBus` → read/write device registers.
Use 7-bit device addresses and call from the main loop only; transfers are blocking.

Failed operations return `false` or `std::nullopt`; inspect `lastStatus()` / `lastError()` for details.