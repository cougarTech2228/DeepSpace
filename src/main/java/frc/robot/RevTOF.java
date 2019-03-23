package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class RevTOF {

	// The Arduino two-wire interface uses a 7-bit number for the address,
	// and sets the last bit correctly based on reads and writes
	public static final int ADDRESS_DEFAULT = 0b0101001;

	public static final int REG_ADDR_SYSRANGE_START = 0x00;

	public static final int REG_ADDR_SYSTEM_THRESH_HIGH = 0x0C;
	public static final int REG_ADDR_SYSTEM_THRESH_LOW = 0x0E;
	public static final int REG_ADDR_SYSTEM_SEQUENCE_CONFIG = 0x01;
	public static final int REG_ADDR_SYSTEM_RANGE_CONFIG = 0x09;
	public static final int REG_ADDR_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
	public static final int REG_ADDR_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
	public static final int REG_ADDR_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
	public static final int REG_ADDR_SYSTEM_INTERRUPT_CLEAR = 0x0B;
	public static final int REG_ADDR_RESULT_INTERRUPT_STATUS = 0x13;
	public static final int REG_ADDR_RESULT_RANGE_STATUS = 0x14;
	public static final int REG_ADDR_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
	public static final int REG_ADDR_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
	public static final int REG_ADDR_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
	public static final int REG_ADDR_RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
	public static final int REG_ADDR_RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
	public static final int REG_ADDR_ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
	public static final int REG_ADDR_I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
	public static final int REG_ADDR_MSRC_CONFIG_CONTROL = 0x60;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_MIN_SNR = 0x27;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
	public static final int REG_ADDR_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;

	public static final int REG_ADDR_FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
	public static final int REG_ADDR_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
	public static final int REG_ADDR_SYSTEM_HISTOGRAM_BIN = 0x81;
	public static final int REG_ADDR_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
	public static final int REG_ADDR_HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
	public static final int REG_ADDR_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
	public static final int REG_ADDR_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
	public static final int REG_ADDR_MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
	public static final int REG_ADDR_SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
	public static final int REG_ADDR_IDENTIFICATION_MODEL_ID = 0xC0;
	public static final int REG_ADDR_IDENTIFICATION_REVISION_ID = 0xC2;
	public static final int REG_ADDR_OSC_CALIBRATE_VAL = 0xF8;
	public static final int REG_ADDR_GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
	public static final int REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
	public static final int REG_ADDR_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
	public static final int REG_ADDR_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
	public static final int REG_ADDR_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
	public static final int REG_ADDR_POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
	public static final int REG_ADDR_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
	public static final int REG_ADDR_ALGO_PHASECAL_LIM = 0x30;
	public static final int REG_ADDR_ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;

	public enum vcselPeriodType {
		VcselPeriodPreRange, VcselPeriodFinalRange
	};

	public int last_status; // status of last I2C transmission

	private I2C i2cPort;

	private int address;
	private int io_timeout;
	private boolean did_timeout;
	private long timeout_start_ms;

	private int stop_variable;
	private int measurement_timing_budget_us;
	private long startTime;

	public RevTOF() {
		i2cPort = new I2C(I2C.Port.kOnboard, 0x54);
		startTime = System.currentTimeMillis();
	}

	public boolean init(boolean io_2v8) {
		if (io_2v8) {
			writeReg(REG_ADDR_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
					readReg(REG_ADDR_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
		}

		// "Set I2C standard mode"
		writeReg(0x88, 0x00);

		writeReg(0x80, 0x01);
		writeReg(0xFF, 0x01);
		writeReg(0x00, 0x00);
		stop_variable = readReg(0x91);
		writeReg(0x00, 0x01);
		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x00);

		// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit
		// checks
		writeReg(REG_ADDR_MSRC_CONFIG_CONTROL, readReg(REG_ADDR_MSRC_CONFIG_CONTROL) | 0x12);

		// set final range signal rate limit to 0.25 MCPS (million counts per second)
		setSignalRateLimit(0.25);

		writeReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG, 0xFF);

		// VL53L0X_DataInit() end

		// VL53L0X_StaticInit() begin

		int spad_count = 0;
		boolean spad_type_is_aperture = false;
		// if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

		// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		// the API, but the same data seems to be more easily readable from
		// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
		byte ref_spad_map[];
		ref_spad_map = readMulti(REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6);

		// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

		writeReg(0xFF, 0x01);
		writeReg(REG_ADDR_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		writeReg(REG_ADDR_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		writeReg(0xFF, 0x00);
		writeReg(REG_ADDR_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

		int first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
		int spads_enabled = 0;

		for (int i = 0; i < 48; i++) {
			if (i < first_spad_to_enable || spads_enabled == spad_count) {
				// This bit is lower than the first one that should be enabled, or
				// (reference_spad_count) bits have already been enabled, so zero this bit
				ref_spad_map[i / 8] &= ~(1 << (i % 8));
			} else if (((ref_spad_map[i / 8] >> (i % 8)) & 0x1) == 1) {
				spads_enabled++;
			}
		}

		writeMulti(REG_ADDR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

		// -- VL53L0X_set_reference_spads() end

		// -- VL53L0X_load_tuning_settings() begin
		// DefaultTuningSettings from vl53l0x_tuning.h

		writeReg(0xFF, 0x01);
		writeReg(0x00, 0x00);

		writeReg(0xFF, 0x00);
		writeReg(0x09, 0x00);
		writeReg(0x10, 0x00);
		writeReg(0x11, 0x00);

		writeReg(0x24, 0x01);
		writeReg(0x25, 0xFF);
		writeReg(0x75, 0x00);

		writeReg(0xFF, 0x01);
		writeReg(0x4E, 0x2C);
		writeReg(0x48, 0x00);
		writeReg(0x30, 0x20);

		writeReg(0xFF, 0x00);
		writeReg(0x30, 0x09);
		writeReg(0x54, 0x00);
		writeReg(0x31, 0x04);
		writeReg(0x32, 0x03);
		writeReg(0x40, 0x83);
		writeReg(0x46, 0x25);
		writeReg(0x60, 0x00);
		writeReg(0x27, 0x00);
		writeReg(0x50, 0x06);
		writeReg(0x51, 0x00);
		writeReg(0x52, 0x96);
		writeReg(0x56, 0x08);
		writeReg(0x57, 0x30);
		writeReg(0x61, 0x00);
		writeReg(0x62, 0x00);
		writeReg(0x64, 0x00);
		writeReg(0x65, 0x00);
		writeReg(0x66, 0xA0);

		writeReg(0xFF, 0x01);
		writeReg(0x22, 0x32);
		writeReg(0x47, 0x14);
		writeReg(0x49, 0xFF);
		writeReg(0x4A, 0x00);

		writeReg(0xFF, 0x00);
		writeReg(0x7A, 0x0A);
		writeReg(0x7B, 0x00);
		writeReg(0x78, 0x21);

		writeReg(0xFF, 0x01);
		writeReg(0x23, 0x34);
		writeReg(0x42, 0x00);
		writeReg(0x44, 0xFF);
		writeReg(0x45, 0x26);
		writeReg(0x46, 0x05);
		writeReg(0x40, 0x40);
		writeReg(0x0E, 0x06);
		writeReg(0x20, 0x1A);
		writeReg(0x43, 0x40);

		writeReg(0xFF, 0x00);
		writeReg(0x34, 0x03);
		writeReg(0x35, 0x44);

		writeReg(0xFF, 0x01);
		writeReg(0x31, 0x04);
		writeReg(0x4B, 0x09);
		writeReg(0x4C, 0x05);
		writeReg(0x4D, 0x04);

		writeReg(0xFF, 0x00);
		writeReg(0x44, 0x00);
		writeReg(0x45, 0x20);
		writeReg(0x47, 0x08);
		writeReg(0x48, 0x28);
		writeReg(0x67, 0x00);
		writeReg(0x70, 0x04);
		writeReg(0x71, 0x01);
		writeReg(0x72, 0xFE);
		writeReg(0x76, 0x00);
		writeReg(0x77, 0x00);

		writeReg(0xFF, 0x01);
		writeReg(0x0D, 0x01);

		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x01);
		writeReg(0x01, 0xF8);

		writeReg(0xFF, 0x01);
		writeReg(0x8E, 0x01);
		writeReg(0x00, 0x01);
		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x00);

		// -- VL53L0X_load_tuning_settings() end

		// "Set interrupt config to new sample ready"
		// -- VL53L0X_SetGpioConfig() begin

		writeReg(REG_ADDR_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		writeReg(REG_ADDR_GPIO_HV_MUX_ACTIVE_HIGH, readReg(REG_ADDR_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
		writeReg(REG_ADDR_SYSTEM_INTERRUPT_CLEAR, 0x01);

		// -- VL53L0X_SetGpioConfig() end

		measurement_timing_budget_us = getMeasurementTimingBudget();

		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin

		writeReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG, 0xE8);

		// -- VL53L0X_SetSequenceStepEnable() end

		// "Recalculate timing budget"
		setMeasurementTimingBudget(measurement_timing_budget_us);

		// VL53L0X_StaticInit() end

		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

		// -- VL53L0X_perform_vhv_calibration() begin

		/*writeReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (!performSingleRefCalibration((byte) 0x40)) {
			return false;
		}

		// -- VL53L0X_perform_vhv_calibration() end

		// -- VL53L0X_perform_phase_calibration() begin

		writeReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (!performSingleRefCalibration((byte) 0x00)) {
			return false;
		}*/

		// -- VL53L0X_perform_phase_calibration() end

		// "restore the previous Sequence Config"
		writeReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG, 0xE8);

		// VL53L0X_PerformRefCalibration() end

		return true;
	}

	// Set the return signal rate limit check value in units of MCPS (mega counts
	// per second). "This represents the amplitude of the signal reflected from the
	// target and detected by the device"; setting this limit presumably determines
	// the minimum measurement necessary for the sensor to report a valid reading.
	// Setting a lower limit increases the potential range of the sensor but also
	// seems to increase the likelihood of getting an inaccurate reading because of
	// unwanted reflections from objects other than the intended target.
	// Defaults to 0.25 MCPS as initialized by the ST API and this library.
	public boolean setSignalRateLimit(double limit_Mcps) {
		if (limit_Mcps < 0 || limit_Mcps > 511.99) {
			return false;
		}

		// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
		writeReg16Bit(REG_ADDR_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int) (limit_Mcps * (1 << 7)));
		return true;
	}

	public void setAddress(int new_addr) {
		writeReg(REG_ADDR_I2C_SLAVE_DEVICE_ADDRESS, (int) (new_addr & 0x7F));
		address = new_addr;
	}

	public int getAddress() {
		return address;
	}

	public void writeReg(int reg, int value) {
		i2cPort.write(reg, value);
	}

	public void writeReg16Bit(int reg, int value) {
		i2cPort.write(reg, value);
	}

	public void writeReg32Bit(int reg, int value) {
		i2cPort.write(reg, value);
	}

	public int readReg(int reg) {
		byte buffer[] = new byte[1];

		i2cPort.write(reg, 0);
		i2cPort.read(reg, 1, buffer);

		return buffer[0];
	}

	public int readReg16Bit(int reg) {
		byte buffer[] = new byte[2];

		i2cPort.write(reg, 0);
		i2cPort.read(reg, 2, buffer);

		int sensorData = 0;

		sensorData = (int) (Byte.toUnsignedInt(buffer[0]) << 8);
		sensorData = sensorData | (int) (Byte.toUnsignedInt(buffer[1]));

		return sensorData;
	}

	public int readReg32Bit(int reg) {
		byte buffer[] = new byte[4];

		i2cPort.write(reg, 0);
		i2cPort.read(reg, 4, buffer);

		int sensorData = 0;

		sensorData = (int) (Byte.toUnsignedInt(buffer[0]) << 24);
		sensorData = sensorData | (int) (Byte.toUnsignedInt(buffer[1]) << 16);
		sensorData = sensorData | (int) (Byte.toUnsignedInt(buffer[2]) << 8);
		sensorData = sensorData | (int) (Byte.toUnsignedInt(buffer[3]));

		return sensorData;
	}

	public void writeMulti(int reg, byte src[], int count) {

		for (int loopIndex = 0; loopIndex < count; loopIndex++) {
			i2cPort.write(reg, src[loopIndex]);
		}
	}

	public byte[] readMulti(int reg, int count) {

		byte buffer[] = new byte[count];
		i2cPort.read(reg, count, buffer);
		return buffer;
	}

	private void startTimeout() {

		timeout_start_ms = System.currentTimeMillis();
	}

	private boolean checkTimeoutExpired() {

		return (io_timeout > 0 && (System.currentTimeMillis() - timeout_start_ms) > io_timeout);
	}

	public int readRangeContinuousMillimeters() {
		startTimeout();
		while ((readReg(REG_ADDR_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
			if (checkTimeoutExpired()) {
				did_timeout = true;
				return 65535;
			}
		}

		// assumptions: Linearity Corrective Gain is 1000 (default);
		// fractional ranging is not enabled
		int range = readReg16Bit(REG_ADDR_RESULT_RANGE_STATUS + 10);

		writeReg(REG_ADDR_SYSTEM_INTERRUPT_CLEAR, 0x01);

		return range;
	}

	public void setTimeout(short timeout) {
		io_timeout = timeout;
	}

	public void startContinuous(int period_ms) { // had '= 0' but gave an error

		writeReg(0x80, 0x01);
		writeReg(0xFF, 0x01);
		writeReg(0x00, 0x00);
		writeReg(0x91, stop_variable);
		writeReg(0x00, 0x01);
		writeReg(0xFF, 0x00);
		writeReg(0x80, 0x00);

		if (period_ms != 0) {
			// continuous timed mode

			// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

			int osc_calibrate_val = readReg16Bit(REG_ADDR_OSC_CALIBRATE_VAL);

			if (osc_calibrate_val != 0) {
				period_ms *= osc_calibrate_val;
			}

			writeReg32Bit(REG_ADDR_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

			// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

			writeReg(REG_ADDR_SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
		} else {
			// continuous back-to-back mode
			writeReg(REG_ADDR_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		}
	}

	public boolean timeoutOccurred() {
		boolean tmp = did_timeout;
		did_timeout = false;
		return tmp;
	}

	public int getMeasurementTimingBudget() {

		SequenceStepEnables enables = new SequenceStepEnables();
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		int StartOverhead = 1910; // note that this is different than the value in set_
		int EndOverhead = 960;
		int MsrcOverhead = 660;
		int TccOverhead = 590;
		int DssOverhead = 690;
		int PreRangeOverhead = 660;
		int FinalRangeOverhead = 550;

		// "Start and end overhead times always present"
		int budget_us = StartOverhead + EndOverhead;

		enables = getSequenceStepEnables();
		timeouts = getSequenceStepTimeouts(enables);

		if (enables.tcc) {
			budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
		}

		if (enables.dss) {
			budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
		} else if (enables.msrc) {
			budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}

		if (enables.pre_range) {
			budget_us += (timeouts.pre_range_us + PreRangeOverhead);
		}

		if (enables.final_range) {
			budget_us += (timeouts.final_range_us + FinalRangeOverhead);
		}

		measurement_timing_budget_us = budget_us; // store for internal reuse
		return budget_us;
	}

	// Set the measurement timing budget in microseconds, which is the time allowed
	// for one measurement; the ST API and this library take care of splitting the
	// timing budget among the sub-steps in the ranging sequence. A longer timing
	// budget allows for more accurate measurements. Increasing the budget by a
	// factor of N decreases the range measurement standard deviation by a factor of
	// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
	// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
	public boolean setMeasurementTimingBudget(int budget_us) {
		SequenceStepEnables enables = new SequenceStepEnables();
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		int StartOverhead = 1320; // note that this is different than the value in get_
		int EndOverhead = 960;
		int MsrcOverhead = 660;
		int TccOverhead = 590;
		int DssOverhead = 690;
		int PreRangeOverhead = 660;
		int FinalRangeOverhead = 550;

		int MinTimingBudget = 20000;

		enables = getSequenceStepEnables();
		timeouts = getSequenceStepTimeouts(enables);

		if (budget_us < MinTimingBudget) {
			return false;
		}

		int used_budget_us = StartOverhead + EndOverhead;

		if (enables.tcc) {
			used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
		}

		if (enables.dss) {
			used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
		} else if (enables.msrc) {
			used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}

		if (enables.pre_range) {
			used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
		}

		if (enables.final_range) {
			used_budget_us += FinalRangeOverhead;

			// "Note that the final range timeout is determined by the timing
			// budget and the sum of all other timeouts within the sequence.
			// If there is no room for the final range timeout, then an error
			// will be set. Otherwise the remaining time will be applied to
			// the final range."

			if (used_budget_us > budget_us) {
				// "Requested timeout too big."
				return false;
			}

			int final_range_timeout_us = budget_us - used_budget_us;

			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

			// "For the final range timeout, the pre-range timeout
			// must be added. To do this both final and pre-range
			// timeouts must be expressed in macro periods MClks
			// because they have different vcsel periods."

			int final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us,
					timeouts.final_range_vcsel_period_pclks);

			if (enables.pre_range) {
				final_range_timeout_mclks += timeouts.pre_range_mclks;
			}

			writeReg16Bit(REG_ADDR_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

			// set_sequence_step_timeout() end

			measurement_timing_budget_us = budget_us; // store for internal reuse
		}
		return true;
	}

	private boolean performSingleRefCalibration(byte vhv_init_byte) {
		writeReg(REG_ADDR_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

		startTimeout();
		while ((readReg(REG_ADDR_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
			if (checkTimeoutExpired()) {
				return false;
			}
		}

		writeReg(REG_ADDR_SYSTEM_INTERRUPT_CLEAR, 0x01);

		writeReg(REG_ADDR_SYSRANGE_START, 0x00);

		return true;
	}

	public class SequenceStepEnables {
		boolean tcc, msrc, dss, pre_range, final_range;
	};

	private SequenceStepEnables getSequenceStepEnables() {

		SequenceStepEnables enables  = new SequenceStepEnables();

		int sequence_config = readReg(REG_ADDR_SYSTEM_SEQUENCE_CONFIG);

		enables.tcc = (((sequence_config >> 4) & 0x1) == 1);
		enables.dss = (((sequence_config >> 3) & 0x1) == 1);
		enables.msrc = (((sequence_config >> 2) & 0x1) == 1);
		enables.pre_range = (((sequence_config >> 6) & 0x1) == 1);
		enables.final_range = (((sequence_config >> 7) & 0x1) == 1);

		return enables;
	}

	// Get sequence step timeouts
	// based on get_sequence_step_timeout(),
	// but gets all timeouts instead of just the requested one, and also stores
	// intermediate values
	private SequenceStepTimeouts getSequenceStepTimeouts(SequenceStepEnables enables) {
		SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

		timeouts.pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);

		timeouts.msrc_dss_tcc_mclks = readReg(REG_ADDR_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
		timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks,
				timeouts.pre_range_vcsel_period_pclks);

		timeouts.pre_range_mclks = decodeTimeout(readReg16Bit(REG_ADDR_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
		timeouts.pre_range_us = timeoutMclksToMicroseconds(timeouts.pre_range_mclks,
				timeouts.pre_range_vcsel_period_pclks);

		timeouts.final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);

		timeouts.final_range_mclks = decodeTimeout(readReg16Bit(REG_ADDR_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

		if (enables.pre_range) {
			timeouts.final_range_mclks -= timeouts.pre_range_mclks;
		}

		timeouts.final_range_us = timeoutMclksToMicroseconds(timeouts.final_range_mclks,
				timeouts.final_range_vcsel_period_pclks);

		return timeouts;
	}

	
	public void setTimeout(int timeout) {
		io_timeout = timeout;
	}
	
	public class SequenceStepTimeouts {

		int pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
		int msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
		int msrc_dss_tcc_us, pre_range_us, final_range_us;
	}

	// Convert sequence step timeout from microseconds to MCLKs with given VCSEL
	// period in PCLKs
	// based on VL53L0X_calc_timeout_mclks()
	private int timeoutMicrosecondsToMclks(int timeout_period_us, int vcsel_period_pclks) {
		int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

		return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
	}

	// Encode sequence step timeout register value from timeout in MCLKs
	// based on VL53L0X_encode_timeout()
	// Note: the original function took a uint16_t, but the argument passed to it
	// is always a uint16_t.
	private int encodeTimeout(int timeout_mclks) {
		// format: "(LSByte * 2^MSByte) + 1"

		int ls_byte = 0;
		int ms_byte = 0;

		if (timeout_mclks > 0) {
			ls_byte = timeout_mclks - 1;

			while ((ls_byte & 0xFFFFFF00) > 0) {
				ls_byte >>= 1;
				ms_byte++;
			}

			return (ms_byte << 8) | (ls_byte & 0xFF);
		} else {
			return 0;
		}
	}

	private static int timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks) {
		int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

		return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
	}

	private static int calcMacroPeriod(int vcsel_period_pclks) {
		return ((((int) 2304 * ((vcsel_period_pclks) * 1655) + 500)) / 1000);
	}

	// Get the VCSEL pulse period in PCLKs for the given period type.
	// based on VL53L0X_get_vcsel_pulse_period()
	private int getVcselPulsePeriod(vcselPeriodType type) {
		if (type == vcselPeriodType.VcselPeriodPreRange) {
			return decodeVcselPeriod(readReg(REG_ADDR_PRE_RANGE_CONFIG_VCSEL_PERIOD));
		} else if (type == vcselPeriodType.VcselPeriodFinalRange) {
			return decodeVcselPeriod(readReg(REG_ADDR_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
		} else {
			return 255;
		}
	}

	private int decodeVcselPeriod(int reg_val) {
		return ((((reg_val) + 1) << 1));
	}

	// Decode sequence step timeout in MCLKs from register value
	// based on VL53L0X_decode_timeout()
	// Note: the original function returned a uint32_t, but the return value is
	// always stored in a uint16_t.
	private int decodeTimeout(int reg_val) {
		// format: "(LSByte * 2^MSByte) + 1"
		return (int) ((reg_val & 0x00FF) << (int) ((reg_val & 0xFF00) >> 8)) + 1;
	}
}
