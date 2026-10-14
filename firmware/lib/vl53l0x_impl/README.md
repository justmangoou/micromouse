# vl53l0x_impl

The STM32 side of ST's [VL53L0X API](../vl53l0x_api/README.md): it implements the I2C functions the API expects
(`VL53L0X_WrByte`, `VL53L0X_ReadMulti`, ... declared in `vl53l0x_platform.h`) on top of [`I2cBus`](../i2c_bus/include/i2c_bus.h).

## Use

```cpp
I2cBus bus(&hi2c1);
VL53L0X_SetBus(&bus);   // once, before any other VL53L0X_* call; the bus must outlive the sensors
```

## Notes

- One bus only: the API's device handle has no bus field, so the bus is set once for all sensors.
- Device addresses are in ST's 8-bit form (`VL53L0X_Dev_t::I2cDevAddr`, 0x52 at power-on).
- A failed transfer returns `VL53L0X_ERROR_CONTROL_INTERFACE`.
- `VL53L0X_PollingDelay` waits 2 ms (`HAL_Delay`).
- Blocking, main loop only, like `I2cBus`.
