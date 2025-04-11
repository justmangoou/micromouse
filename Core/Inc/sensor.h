/**
 * @file   sensor.h
 * @brief  Defines functions and structures for I2C communication, sensors (MPU6050, VL53L0X), and motor control.
 */
#ifndef SENSOR_H
#define SENSOR_H

#include "stdint.h"
#include "stdbool.h"

#define I2C_WRITE 0x00
#define I2C_READ  0x01

static const uint16_t I2C_TIMEOUT = 100;
//---------------------------------------------------------------------
// Motor Encoder
//---------------------------------------------------------------------
typedef struct {
    int32_t position;
    int32_t velocity;
} MotorEncoderData;

/**
 * @brief Initializes the motor encoder.
 * @return True if initialization was successful, false otherwise.
 */
bool MotorEncoder_Init(void);

/**
 * @brief Reads the current position from the motor encoder.
 * @param data Pointer to MotorEncoderData structure to store the data.
 */
void MotorEncoder_ReadPosition(MotorEncoderData *data);

/**
 * @brief Reads the current velocity from the motor encoder.
 * @param data Pointer to MotorEncoderData structure to store the data.
 */
void MotorEncoder_ReadVelocity(MotorEncoderData *data);

//---------------------------------------------------------------------
// MPU6050 Sensor
//---------------------------------------------------------------------
const uint8_t MPU6050_DEF_ADDR = 0x68 << 1;

enum MPU6050_REG_ADDR {
    MPU6050_REG_WHO_AM_I = 0x75,
    MPU6050_REG_PWR_MGMT_1 = 0x6B,
    MPU6050_REG_SMPLRT_DIV = 0x19,

    MPU6050_REG_ACCEL_CONFIG = 0x1C,
    MPU6050_REG_ACCEL_ZOUT_H = 0x3F,
    MPU6050_REG_GYRO_CONFIG = 0x1B,
    MPU6050_REG_GYRO_ZOUT_H = 0x47,
};

/**
 * @brief Initializes the MPU6050 sensor.
 * @return True if initialization was successful, false otherwise.
 */
bool MPU6050_Init(void);

/**
 * @brief Reads the Z-axis angle from the MPU6050 sensor.
 * @return The Z-axis angle in degrees.
 */
int16_t MPU6050_ReadGyroZ(void);

/**
 * @brief Calculates the Z-axis angle based on the gyroscope data.
 * @return The Z-axis angle in degrees.
 */
float MPU6050_GetZAngle(void);

//-----------------------------------------------------
// VL53L0X Sensor
//-----------------------------------------------------
#define VL53L0X_L_XSHUT_GPIO GPIOB
#define VL53L0X_L_XSHUT_PIN GPIO_PIN_10
#define VL53L0X_R_XSHUT_GPIO GPIOB
#define VL53L0X_R_XSHUT_PIN GPIO_PIN_11
#define VL53L0X_F_XSHUT_GPIO GPIOB
#define VL53L0X_F_XSHUT_PIN GPIO_PIN_12

const uint8_t VL53L0X_DEF_ADDR = 0x29 << 1;
const uint8_t VL53L0X_L_ADDR   = 0x30 << 1;
const uint8_t VL53L0X_R_ADDR   = 0x31 << 1;
const uint8_t VL53L0X_F_ADDR   = 0x32 << 1;

const uint8_t VL53L0X_L_OFFSET = -40;
const uint8_t VL53L0X_R_OFFSET = -40;
const uint8_t VL53L0X_F_OFFSET = -40;

enum VL53L0X_REG_ADDR {
    VL53L0X_REG_SYSRANGE_START = 0x00,

    VL53L0X_REG_SYSTEM_THRESH_HIGH = 0x0C,
    VL53L0X_REG_SYSTEM_THRESH_LOW = 0x0E,
    VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG = 0x01,
    VL53L0X_REG_SYSTEM_RANGE_CONFIG = 0x09,
    VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

    // GPIO & I2C
    VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
    VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR = 0x0B,

    VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

    VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS = 0x8A,
    VL53L0X_REG_I2C_MODE = 0x88,

    // RESULT
    VL53L0X_REG_RESULT_INTERRUPT_STATUS = 0x13,
    VL53L0X_REG_RESULT_RANGE_STATUS = 0x14,

    VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
    VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
    VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
    VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
    VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

    // ALGO
    VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

    // CHECK LIMIT
    VL53L0X_REG_MSRC_CONFIG_CONTROL = 0x60,
    VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR = 0x27,
    VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
    VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
    VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

    VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
    VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
    VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,
    VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
    VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
    VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

    // INTERNAL TUNING
    VL53L0X_REG_INTERNAL_TUNING_1 = 0x91,
    VL53L0X_REG_INTERNAL_TUNING_2 = 0xFF,

    // OTHERS
    VL53L0X_REG_SYSTEM_HISTOGRAM_BIN = 0x81,
    VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
    VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

    VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
    VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

    VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

    VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
    VL53L0X_REG_IDENTIFICATION_MODEL_ID = 0xC0,
    VL53L0X_REG_IDENTIFICATION_REVISION_ID = 0xC2,

    VL53L0X_REG_OSC_CALIBRATE_VAL = 0xF8,

    VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

    VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

    VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

    VL53L0X_REG_ALGO_PHASECAL_LIM = 0x30,
    VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

typedef struct {
    uint16_t RawDistance; //uncorrected distance  [mm],   uint16_t
    uint16_t SignalCnt;   //Signal  Counting Rate [mcps], uint16_t, fixpoint9.7
    uint16_t AmbientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
    uint16_t SpadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
    uint8_t  RangeStatus; //Ranging status (0-15)
} VL53L0XData;

/**
 * @brief Enumeration for VCSEL (vertical cavity surface emitting laser) pulse period types.
 */
typedef enum {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} VcselPeriodType;

/**
 * TCC: Target CentreCheck
 * MSRC: Minimum Signal Rate Check
 * DSS: Dynamic Spad Selection
 */
typedef struct {
    uint8_t Tcc, Msrc, Dss, PreRange, FinalRange;
} SequenceStepEnables;

typedef struct {
    uint16_t PreRangeVcselPeriodPclks, FinalRangeVcselPeriodPclks;

    uint16_t MsrcDssTccMclks, PreRangeMclks, FinalRangeMclks;
    uint32_t MsrcDssTccUs, PreRangeUs, FinalRangeUs;
} SequenceStepTimeouts;

/**
 * @brief Sets a new I2C address for the VL53L0X sensor.
 * @param new_addr The new I2C address to set.
 */
void VL53L0X_SetAddr(uint8_t new_addr);

/**
 * @brief Gets the current I2C address of the VL53L0X sensor.
 * @return The current I2C address of the sensor.
 */
uint8_t VL53L0X_GetAddr();

/**
 * @brief Performs reference calibration for the VL53L0X sensor.
 * @return True if calibration was successful, false otherwise.
 */
bool VL53L0X_PerformRefCalibration(void);

/**
 * @brief Initializes the VL53L0X sensor.
 * @param io_2v8 If true, sets the sensor to use 2.8V I/O mode.
 * @return True if initialization was successful, false otherwise.
 */
bool VL53L0X_Init(bool io_2v8);

/**
 * @brief Initializes all VL53L0X sensors.
 * @param io_2v8 see VL53L0X_Init
 * @return True if initialization was successful, false otherwise.
 */
bool VL53L0X_InitAll(bool io_2v8);

/**
 * @brief Sets the return signal rate limit to the given value in units of MCPS (mega counts per second).
 *
 * This is the minimum amplitude of the signal reflected from the target and received by the sensor
 * necessary for it to report a valid reading. Setting a lower limit increases the potential range
 * of the sensor but also increases the likelihood of getting an inaccurate reading because of
 * reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS
 * by default.
 *
 * @param limit_mcps The desired signal rate limit in MCPS.
 * @return True if the requested limit was valid, false otherwise.
 */
bool VL53L0X_SetSignalRateLimit(float limit_mcps);

/**
 * @brief Gets the current return signal rate limit.
 * @return Signal rate limit in MCPS.
 */
float VL53L0X_GetSignalRateLimit(void);

/**
 * @brief Sets the measurement timing budget in microseconds.
 *
 * The timing budget is the time allowed for one measurement. A longer timing budget allows for more accurate measurements.
 * Increasing the budget by a factor of N decreases the range measurement standard deviation by a factor of sqrt(N).
 * Defaults to about 33 milliseconds; the minimum is 20 ms.
 *
 * @param budget_us Timing budget in microseconds.
 * @return True if the timing budget was set successfully, false otherwise.
 */
bool VL53L0X_SetMeasurementTimingBudget(uint32_t budget_us);

/**
 * @brief Returns the current measurement timing budget in microseconds.
 *
 * @return The current measurement timing budget in microseconds.
 */
uint32_t VL53L0X_GetMeasurementTimingBudget(void);

/**
 * @brief Sets the VCSEL pulse period for the given period type.
 *
 * Longer periods increase the potential range of the sensor. Valid values are (even numbers only):
 * Pre: 12 to 18 (initialized to 14 by default)
 * Final: 8 to 14 (initialized to 10 by default)
 *
 * @param type The VCSEL period type (VcselPeriodPreRange or VcselPeriodFinalRange).
 * @param period_pclks The desired pulse period in PCLKs.
 * @return True if the requested period was valid, false otherwise.
 */
bool VL53L0X_SetVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks);

/**
 * @brief Returns the current VCSEL pulse period for the given period type.
 *
 * @param type The VCSEL period type (VcselPeriodPreRange or VcselPeriodFinalRange).
 * @return The current VCSEL pulse period in PCLKs.
 */
uint8_t VL53L0X_GetVcselPulsePeriod(VcselPeriodType type);

/**
 * @brief Starts continuous ranging measurements.
 *
 * If the argument period_ms is 0, continuous back-to-back mode is used
 * (the sensor takes measurements as often as possible). If it is nonzero,
 * continuous timed mode is used, with the specified inter-measurement period
 * in milliseconds determining how often the sensor takes a measurement.
 *
 * @param period_ms Inter-measurement period in milliseconds.
 */
void VL53L0X_StartContinuous(uint32_t period_ms);

/**
 * @brief Stops continuous ranging measurements.
 */
void VL53L0X_StopContinuous(void);

/**
 * @brief Reads a range measurement in millimeters during continuous mode.
 * @param extraStats Pointer to VL53L0XData structure to store additional measurement data.
 * @return Range measurement in millimeters.
 */
uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0XData *extraStats);

/**
 * @brief Performs a single-shot ranging measurement.
 * @param extraStats Pointer to VL53L0XData structure to store additional measurement data.
 * @return Range measurement in millimeters.
 */
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0XData *extraStats);

/**
 * @brief Sets a timeout period in milliseconds after which read operations will abort
 * if the sensor is not ready. A value of 0 disables the timeout.
 *
 * @param timeout Timeout period in milliseconds.
 */
void VL53L0X_SetTimeout(uint16_t timeout);

/**
 * @brief Gets the current timeout period setting.
 * @return Timeout period in milliseconds.
 */
uint16_t VL53L0X_GetTimeout(void);

/**
 * @brief Checks if a read timeout has occurred.
 * @return True if a timeout occurred, false otherwise.
 */
bool VL53L0X_TimeoutOccurred(void);

#endif //SENSOR_H
