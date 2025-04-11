/**
 * @file sensor_vl53l0x.c
 * @brief VL53L0X sensor implementation.
 * @author https://github.com/Squieler/VL53L0X---STM32-HAL
 */
#include "i2c.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "sensor.h"

uint8_t g_i2cAddr = VL53L0X_DEF_ADDR;
uint16_t g_ioTimeout = 0;  // no timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t g_measTimBudUs;

/* INTERNAL FUNCTION DECLARATIONS ------------------------------------*/
static void write(uint32_t reg, uint8_t *data, uint8_t len);
static void write_8bit(uint32_t reg, uint8_t data);
static void write_16bit(uint32_t reg, uint16_t data);
static void write_32bit(uint32_t reg, uint32_t data);
static void read(uint32_t reg, uint8_t *data, uint8_t len);
static uint8_t read_8bit(uint32_t reg);
static uint16_t read_16bit(uint32_t reg);

static bool get_spad_info(uint8_t *count, bool *type_is_aperture);
static void get_sequence_step_enables(SequenceStepEnables *enables);
static void get_sequence_step_timeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);
static void load_tuning_settings(void);
static bool perform_single_ref_calibration(uint8_t vhv_init_byte);
static bool perform_ref_calibration(void);

static void internal_init(bool io_2v8, uint8_t addr);

static void start_timeout(void);
static bool check_timeout_expired(void);
static uint16_t decode_timeout(uint16_t reg_val);
static uint16_t encode_timeout(uint16_t timeout_mclks);
static bool timeout_occurred(void);
static uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

static uint8_t decode_vcsel_period(uint8_t reg_val);
static uint8_t encode_vcsel_period(uint8_t period_pclks);
static uint32_t calc_macro_period(uint8_t vcsel_period_pclks);

/* PUBLIC FUNCTIONS --------------------------------------------------*/
void VL53L0X_SetAddr(const uint8_t new_addr) {
    write_8bit(VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, (new_addr >> 1) & 0x7F);
    g_i2cAddr = new_addr;
}

uint8_t VL53L0X_GetAddr(void) {
    return g_i2cAddr;
}

bool VL53L0X_Init(const bool io_2v8) {
    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io_2v8)
        write_8bit(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            read_8bit(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0

    // "Set I2C standard mode"
    write_8bit(0x88, 0x00);

    write_8bit(0x80, 0x01);
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);
    g_stopVariable = read_8bit(0x91);
    write_8bit(0x00, 0x01);
    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    write_8bit(VL53L0X_REG_MSRC_CONFIG_CONTROL, read_8bit(VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    VL53L0X_SetSignalRateLimit(0.25F);

    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(&spad_count, &spad_type_is_aperture)) return false;

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    read(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    write_8bit(0xFF, 0x01);
    write_8bit(VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_8bit(VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_8bit(0xFF, 0x00);
    write_8bit(VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    const uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            spads_enabled++;
    }

    write(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    load_tuning_settings();

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin
    write_8bit(VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_8bit(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, read_8bit(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    write_8bit(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    // -- VL53L0X_SetGpioConfig() end

    g_measTimBudUs = VL53L0X_GetMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable()
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // "Recalculate timing budget"
    VL53L0X_SetMeasurementTimingBudget(g_measTimBudUs);

    // VL53L0X_StaticInit() end

    if (!perform_ref_calibration()) return false;

    return true;
}

bool VL53L0X_InitAll(const bool io_2v8) {
    HAL_GPIO_WritePin(VL53L0X_L_XSHUT_GPIO, VL53L0X_L_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L0X_R_XSHUT_GPIO, VL53L0X_R_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L0X_F_XSHUT_GPIO, VL53L0X_F_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(VL53L0X_L_XSHUT_GPIO, VL53L0X_L_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    internal_init(io_2v8, VL53L0X_L_ADDR);

    HAL_GPIO_WritePin(VL53L0X_R_XSHUT_GPIO, VL53L0X_R_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    internal_init(io_2v8, VL53L0X_L_ADDR);

    HAL_GPIO_WritePin(VL53L0X_F_XSHUT_GPIO, VL53L0X_F_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    internal_init(io_2v8, VL53L0X_L_ADDR);

    return true;
}

bool VL53L0X_SetSignalRateLimit(const float limit_mcps) {
    if (limit_mcps < 0 || limit_mcps > 511.99) return false;

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    write_16bit(VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_mcps * (1 << 7));
    return true;
}

float VL53L0X_GetSignalRateLimit(void) {
    return (float) read_16bit(VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X_SetMeasurementTimingBudget(const uint32_t budget_us) {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead      = 1320; // note that this is different than the value in get_
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) return false;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(&enables);
    get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.Tcc)
        used_budget_us += (timeouts.MsrcDssTccUs + TccOverhead);

    if (enables.Dss)
        used_budget_us += 2 * (timeouts.MsrcDssTccUs + DssOverhead);
    else if (enables.Msrc)
        used_budget_us += (timeouts.MsrcDssTccUs + MsrcOverhead);

    if (enables.PreRange)
        used_budget_us += (timeouts.PreRangeUs + PreRangeOverhead);

    if (enables.FinalRange) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise, the remaining time will be applied to
        // the final range."
        if (used_budget_us > budget_us)
            // "Requested timeout too big."
            return false;

        const uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            timeout_microseconds_to_mclks(final_range_timeout_us, timeouts.FinalRangeVcselPeriodPclks);

        if (enables.PreRange)
            final_range_timeout_mclks += timeouts.PreRangeMclks;

        write_16bit(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encode_timeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        g_measTimBudUs = budget_us; // store for internal reuse
    }

    return true;
}

uint32_t VL53L0X_GetMeasurementTimingBudget(void) {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead     = 1910; // note that this is different than the value in set_
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(&enables);
    get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.Tcc)
        budget_us += (timeouts.MsrcDssTccUs + TccOverhead);

    if (enables.Dss)
        budget_us += 2 * (timeouts.MsrcDssTccUs + DssOverhead);
    else if (enables.Msrc)
        budget_us += (timeouts.MsrcDssTccUs + MsrcOverhead);

    if (enables.PreRange)
        budget_us += (timeouts.PreRangeUs + PreRangeOverhead);
    if (enables.FinalRange)
        budget_us += (timeouts.PreRangeUs + FinalRangeOverhead);

    g_measTimBudUs = budget_us; // store for internal reuse
    return budget_us;
}

bool VL53L0X_SetVcselPulsePeriod(const VcselPeriodType type, const uint8_t period_pclks) {
    const uint8_t vcsel_period_reg = encode_vcsel_period(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    get_sequence_step_enables(&enables);
    get_sequence_step_timeouts(&enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type == VcselPeriodPreRange) {
        // "Set phase check limits"
        switch (period_pclks) {
            case 12:
                write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                break;

            case 14:
                write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                break;

            case 16:
                write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                break;

            case 18:
                write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                break;

            default:
                // invalid period
                return false;
        }
        write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        write_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        const uint16_t new_pre_range_timeout_mclks =
            timeout_microseconds_to_mclks(timeouts.PreRangeUs, period_pclks);

        write_16bit(VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
              encode_timeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        const uint16_t new_msrc_timeout_mclks =
            timeout_microseconds_to_mclks(timeouts.MsrcDssTccUs, period_pclks);

        write_8bit(VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                   (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                write_8bit(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                write_8bit(0xFF, 0x01);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x30);
                write_8bit(0xFF, 0x00);
                break;

            case 10:
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                write_8bit(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                write_8bit(0xFF, 0x01);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                write_8bit(0xFF, 0x00);
                break;

            case 12:
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                write_8bit(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                write_8bit(0xFF, 0x01);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                write_8bit(0xFF, 0x00);
                break;

            case 14:
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                write_8bit(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                write_8bit(0xFF, 0x01);
                write_8bit(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                write_8bit(0xFF, 0x00);
                break;

            default:
                // invalid period
                return false;
        }

        // apply new VCSEL period
        write_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
            timeout_microseconds_to_mclks(timeouts.FinalRangeUs, period_pclks);

        if (enables.PreRange)
            new_final_range_timeout_mclks += timeouts.PreRangeMclks;

        write_16bit(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
              encode_timeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
    } else
        return false; // invalid type

    // "Finally, the timing budget must be re-applied"

    VL53L0X_SetMeasurementTimingBudget(g_measTimBudUs);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    const uint8_t sequence_config = read_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    perform_single_ref_calibration(0x0);
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return true;
}

uint8_t VL53L0X_GetVcselPulsePeriod(const VcselPeriodType type) {
    if (type == VcselPeriodPreRange)
        return decode_vcsel_period(read_8bit(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
    if (type == VcselPeriodFinalRange)
        return decode_vcsel_period(read_8bit(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    return 255;
}

// Based on VL53L0X_StartMeasurement()
void Vl53L0X_StartContinuous(uint32_t period_ms) {
    write_8bit(0x80, 0x01);
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);
    write_8bit(0x91, g_stopVariable);
    write_8bit(0x00, 0x01);
    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x00);

    if (period_ms != 0) {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        const uint16_t osc_calibrate_val = read_8bit(VL53L0X_REG_OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0)
            period_ms *= osc_calibrate_val;

        write_32bit(VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        write_8bit(VL53L0X_REG_SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else
        // continuous back-to-back mode
        write_8bit(VL53L0X_REG_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
}

// Based on VL53L0X_StopMeasurement()
void VL53L0X_StopContinuous(void) {
    write_8bit(VL53L0X_REG_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);
    write_8bit(0x91, 0x00);
    write_8bit(0x00, 0x01);
    write_8bit(0xFF, 0x00);
}

uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0XData *extraStats) {
    uint16_t temp;

    start_timeout();

    while ((read_8bit(VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        if (check_timeout_expired()) return 65535;

    if (extraStats == 0) {
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        temp = read_16bit(VL53L0X_REG_RESULT_RANGE_STATUS + 10);
    } else {
        // Register map starting at 0x14
        //     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
        //    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
        //   0: Ranging status, uint8_t
        //   1: ???
        // 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
        //   4: 0 ?
        //   5: ???
        // 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
        // 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
        // A,B: uncorrected distance [mm], uint16_t
        uint8_t buffer[12];
        read(0x14, buffer, 12);

        extraStats->RangeStatus =  buffer[0x00]>>3;
        extraStats->SpadCnt     = (buffer[0x02]<<8) | buffer[0x03];
        extraStats->SignalCnt   = (buffer[0x06]<<8) | buffer[0x07];
        extraStats->AmbientCnt  = (buffer[0x08]<<8) | buffer[0x09];
        temp                    = (buffer[0x0A]<<8) | buffer[0x0B];
        extraStats->RawDistance = temp;
    }

    write_8bit(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return temp;
}

uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0XData *extraStats) {
    write_8bit(0x80, 0x01);
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);
    write_8bit(0x91, g_stopVariable);
    write_8bit(0x00, 0x01);
    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x00);
    write_8bit(VL53L0X_REG_SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    start_timeout();
    while (read_8bit(VL53L0X_REG_SYSRANGE_START) & 0x01)
        if (check_timeout_expired()) return 65535;

    return VL53L0X_ReadRangeContinuousMillimeters(extraStats);
}

void VL53L0X_SetTimeout(const uint16_t timeout){
    g_ioTimeout = timeout;
}

uint16_t VL53L0X_GetTimeout(void){
    return g_ioTimeout;
}

/* INTERNAL FUNCTIONS ------------------------------------------------*/
static void write(const uint32_t reg, uint8_t *data, const uint8_t len) {
    HAL_I2C_Mem_Write(&hi2c1, g_i2cAddr | I2C_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
}

static void write_8bit(const uint32_t reg, uint8_t data) {
    write(reg, &data, 1);
}

static void write_16bit(const uint32_t reg, uint16_t data) {
    write(reg, (uint8_t *)&data, 2);
}

static void write_32bit(const uint32_t reg, uint32_t data) {
    write(reg, (uint8_t *)&data, 4);
}

static void read(const uint32_t reg, uint8_t *data, const uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c1, g_i2cAddr | I2C_READ, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
}

static uint8_t read_8bit(const uint32_t reg) {
    uint8_t value;
    read(reg, &value, 1);
    return value;
}

static uint16_t read_16bit(const uint32_t reg) {
    uint16_t value;
    read(reg, (uint8_t *)&value, 2);
    return value;
}

/**
 * @brief Get reference SPAD (single photon avalanche diode) count and type
 * Based on VL53L0X_get_info_from_device() but only gets reference SPAD count and type
 */
bool get_spad_info(uint8_t *count, bool *type_is_aperture) {
    write_8bit(0x80, 0x01);
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);

    write_8bit(0xFF, 0x06);
    write_8bit(0x83, read_8bit(0x83) | 0x04);
    write_8bit(0xFF, 0x07);
    write_8bit(0x81, 0x01);

    write_8bit(0x80, 0x01);

    write_8bit(0x94, 0x6b);
    write_8bit(0x83, 0x00);

    start_timeout();
    while (read_8bit(0x83) == 0x00) {
        if (check_timeout_expired()) return false;
    }

    write_8bit(0x83, 0x01);
    const uint8_t tmp = read_8bit(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    write_8bit(0x81, 0x00);
    write_8bit(0xFF, 0x06);
    write_8bit(0x83, read_8bit(0x83)  & ~0x04);
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x01);

    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x00);

    return true;
}

/**
 * @brief Get sequence step enables
 * Based on VL53L0X_GetSequenceStepEnables()
 */
void get_sequence_step_enables(SequenceStepEnables * enables) {
    const uint8_t sequence_config = read_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);

    enables->Tcc          = (sequence_config >> 4) & 0x1;
    enables->Dss          = (sequence_config >> 3) & 0x1;
    enables->Msrc         = (sequence_config >> 2) & 0x1;
    enables->PreRange     = (sequence_config >> 6) & 0x1;
    enables->FinalRange   = (sequence_config >> 7) & 0x1;
}

/**
 * @brief Get sequence step timeouts
 * Based on get_sequence_step_timeout(),
 * but gets all timeouts instead of just the requested one, and also stores
 * intermediate values
 */
void get_sequence_step_timeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts) {
    timeouts->PreRangeVcselPeriodPclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodPreRange);

    timeouts->MsrcDssTccMclks = read_8bit(VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->MsrcDssTccUs =
        timeout_mclks_to_microseconds(timeouts->MsrcDssTccMclks, timeouts->PreRangeVcselPeriodPclks);

    timeouts->PreRangeMclks =
        decode_timeout(read_16bit(VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->PreRangeUs =
        timeout_mclks_to_microseconds(timeouts->PreRangeMclks, timeouts->PreRangeVcselPeriodPclks);

    timeouts->FinalRangeVcselPeriodPclks = VL53L0X_GetVcselPulsePeriod(VcselPeriodFinalRange);
    timeouts->FinalRangeMclks =
        decode_timeout(read_16bit(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->PreRange)
        timeouts->FinalRangeMclks -= timeouts->PreRangeMclks;

    timeouts->FinalRangeUs =
        timeout_mclks_to_microseconds(timeouts->FinalRangeMclks, timeouts->FinalRangeVcselPeriodPclks);
}

/**
 * @brief Load tuning settings
 * Based on VL53L0X_load_tuning_settings(), and
 * DefaultTuningSettings from vl53l0x_tuning.h
 */
static void load_tuning_settings(void) {
    write_8bit(0xFF, 0x01);
    write_8bit(0x00, 0x00);

    write_8bit(0xFF, 0x00);
    write_8bit(0x09, 0x00);
    write_8bit(0x10, 0x00);
    write_8bit(0x11, 0x00);

    write_8bit(0x24, 0x01);
    write_8bit(0x25, 0xFF);
    write_8bit(0x75, 0x00);

    write_8bit(0xFF, 0x01);
    write_8bit(0x4E, 0x2C);
    write_8bit(0x48, 0x00);
    write_8bit(0x30, 0x20);

    write_8bit(0xFF, 0x00);
    write_8bit(0x30, 0x09);
    write_8bit(0x54, 0x00);
    write_8bit(0x31, 0x04);
    write_8bit(0x32, 0x03);
    write_8bit(0x40, 0x83);
    write_8bit(0x46, 0x25);
    write_8bit(0x60, 0x00);
    write_8bit(0x27, 0x00);
    write_8bit(0x50, 0x06);
    write_8bit(0x51, 0x00);
    write_8bit(0x52, 0x96);
    write_8bit(0x56, 0x08);
    write_8bit(0x57, 0x30);
    write_8bit(0x61, 0x00);
    write_8bit(0x62, 0x00);
    write_8bit(0x64, 0x00);
    write_8bit(0x65, 0x00);
    write_8bit(0x66, 0xA0);

    write_8bit(0xFF, 0x01);
    write_8bit(0x22, 0x32);
    write_8bit(0x47, 0x14);
    write_8bit(0x49, 0xFF);
    write_8bit(0x4A, 0x00);

    write_8bit(0xFF, 0x00);
    write_8bit(0x7A, 0x0A);
    write_8bit(0x7B, 0x00);
    write_8bit(0x78, 0x21);

    write_8bit(0xFF, 0x01);
    write_8bit(0x23, 0x34);
    write_8bit(0x42, 0x00);
    write_8bit(0x44, 0xFF);
    write_8bit(0x45, 0x26);
    write_8bit(0x46, 0x05);
    write_8bit(0x40, 0x40);
    write_8bit(0x0E, 0x06);
    write_8bit(0x20, 0x1A);
    write_8bit(0x43, 0x40);

    write_8bit(0xFF, 0x00);
    write_8bit(0x34, 0x03);
    write_8bit(0x35, 0x44);

    write_8bit(0xFF, 0x01);
    write_8bit(0x31, 0x04);
    write_8bit(0x4B, 0x09);
    write_8bit(0x4C, 0x05);
    write_8bit(0x4D, 0x04);

    write_8bit(0xFF, 0x00);
    write_8bit(0x44, 0x00);
    write_8bit(0x45, 0x20);
    write_8bit(0x47, 0x08);
    write_8bit(0x48, 0x28);
    write_8bit(0x67, 0x00);
    write_8bit(0x70, 0x04);
    write_8bit(0x71, 0x01);
    write_8bit(0x72, 0xFE);
    write_8bit(0x76, 0x00);
    write_8bit(0x77, 0x00);

    write_8bit(0xFF, 0x01);
    write_8bit(0x0D, 0x01);

    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x01);
    write_8bit(0x01, 0xF8);

    write_8bit(0xFF, 0x01);
    write_8bit(0x8E, 0x01);
    write_8bit(0x00, 0x01);
    write_8bit(0xFF, 0x00);
    write_8bit(0x80, 0x00);
}

/**
 * Based on VL53L0X_perform_single_ref_calibration()
 */
static bool perform_single_ref_calibration(const uint8_t vhv_init_byte) {
    write_8bit(VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    start_timeout();
    while ((read_8bit(VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (check_timeout_expired()) return false;
    }

    write_8bit(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_8bit(VL53L0X_REG_SYSRANGE_START, 0x00);

    return true;
}

/**
 * Based on VL53L0X_PerformRefCalibration()
 */
static bool perform_ref_calibration(void) {
    // -- VL53L0X_perform_vhv_calibration() begin
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(0x40)) return false;
    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(0x00)) return false;
    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    write_8bit(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    return true;
}

static void internal_init(const bool io_2v8, const uint8_t addr) {
    VL53L0X_SetAddr(addr);
    VL53L0X_Init(io_2v8);
    VL53L0X_SetSignalRateLimit(20);
    VL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 10);
    VL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
    VL53L0X_SetMeasurementTimingBudget(300 * 1000UL);
}

/**
 * @brief Record the current time to check an upcoming timeout against
 */
static void start_timeout(void) {
    g_timeoutStartMs = HAL_GetTick();
}

/**
 * @brief Check if timeout is enabled (set to nonzero value) and has expired
 * @return True if timeout has expired, false otherwise
 */
static bool check_timeout_expired(void) {
    return g_isTimeout = (g_ioTimeout > 0 && ((uint16_t)HAL_GetTick() - g_timeoutStartMs) > g_ioTimeout);
}

/**
 * @brief Decode sequence step timeout in MCLKs from register value
 * Note: the original function returned an uint32_t, but the return value is
 * always stored in an uint16_t.
 * Based on VL53L0X_decode_timeout()
 */
uint16_t decode_timeout(const uint16_t reg_val) {
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
           (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/**
 * @brief Encode sequence step timeout register value from timeout in MCLKs
 * Based on VL53L0X_encode_timeout()
 * Note: the original function took an uint16_t, but the argument passed to it
 * is always an uint16_t.
 */
uint16_t encode_timeout(const uint16_t timeout_mclks) {
    // format: "(LSByte * 2^MSByte) + 1"
    if (timeout_mclks > 0) {
        uint32_t ls_byte = timeout_mclks - 1;
        uint16_t ms_byte = 0;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return ms_byte << 8 | ls_byte & 0xFF;
    }

    return 0;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool timeout_occurred(void) {
    const bool tmp = g_isTimeout;
    g_isTimeout = false;
    return tmp;
}

/**
 * @brief Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs.
 * Based on VL53L0X_calc_timeout_us()
 */
uint32_t timeout_mclks_to_microseconds(const uint16_t timeout_period_mclks, const uint8_t vcsel_period_pclks) {
    const uint32_t macro_period_ns = calc_macro_period(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

/**
 * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs.
 * Based on VL53L0X_calc_timeout_mclks()
 */
uint32_t timeout_microseconds_to_mclks(const uint32_t timeout_period_us, const uint8_t vcsel_period_pclks) {
    const uint32_t macro_period_ns = calc_macro_period(vcsel_period_pclks);
    return ((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns;
}

/**
 * @brief Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
 * from register value.
 * Based on VL53L0X_decode_vcsel_period()
 *
 * @param reg_val The register value to decode.
 * @return The decoded VCSEL pulse period in PCLKs.
 */
static uint8_t decode_vcsel_period(const uint8_t reg_val) {
    return (reg_val + 1) << 1;
}

/**
 * @brief Encode VCSEL pulse period register value from period in PCLKs.
 * Based on VL53L0X_encode_vcsel_period()
 * @param period_pclks The VCSEL pulse period in PCLKs.
 * @return The encoded register value.
 */
static uint8_t encode_vcsel_period(const uint8_t period_pclks) {
    return (period_pclks >> 1) - 1;
}

/**
 * @brief Calculate macro period in nanoseconds from VCSEL period in PCLKs.
 * Based on VL53L0X_calc_macro_period_ps()
 *
 * @param vcsel_period_pclks The VCSEL period in PCLKs.
 * @return The calculated macro period in nanoseconds.
 */
static uint32_t calc_macro_period(const uint8_t vcsel_period_pclks) {
    return ((uint32_t)2304 * (uint32_t)vcsel_period_pclks * 1655 + 500) / 1000;
}
