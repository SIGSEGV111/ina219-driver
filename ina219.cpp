#include "ina219.hpp"
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <endian.h>

namespace ina219
{
	bool DEBUG = false;

	static const uint8_t REG_CONFIG = 0x00;
	static const uint8_t REG_SHUNTVOLTAGE = 0x01;
	static const uint8_t REG_BUSVOLTAGE = 0x02;
	static const uint8_t REG_POWER = 0x03;
	static const uint8_t REG_CURRENT = 0x04;
	static const uint8_t REG_CALIBRATION = 0x05;

	void TINA219::WriteRegister(const uint8_t register_address, const uint16_t value, const bool verify)
	{
		if(DEBUG) fprintf(stderr, "DEBUG: write-register: reg=%02hhx, value=0x%04hx\n", register_address, value);
		const uint16_t value_be = htobe16(value);
		uint8_t buffer[3];
		memcpy(buffer + 0, &register_address, 1);
		memcpy(buffer + 1, &value_be, 2);

		if(write(this->fd_i2cbus, buffer, 3) != 3) throw "failed to write to register";

		if(verify)
		{
			usleep(4);
			uint16_t check = 0;
			if(read(this->fd_i2cbus, &check, 2) != 2) throw "failed to read register after write";
			if(DEBUG) fprintf(stderr, "DEBUG: verify-register: reg=%02hhx, wanted-value=0x%04hx, actual-value=0x%04hx\n", register_address, value, be16toh(check));
			if(check != value_be) throw "register write verify failed";
		}
	}

	uint16_t TINA219::ReadRegister(const uint8_t register_address)
	{
		uint16_t value = 0;
		if(write(this->fd_i2cbus, &register_address, 1) != 1) throw "failed to set the register pointer for a register read operation";
		if(read(this->fd_i2cbus, &value, 2) != 2) throw "failed to read register value";
		value = be16toh(value);
		if(DEBUG) fprintf(stderr, "DEBUG: read-register: reg=%02hhx, value=0x%04hx\n", register_address, value);
		return value;
	}

	void TINA219::Reset()
	{
		SYSERR(ioctl(this->fd_i2cbus, I2C_SLAVE, this->address));

		WriteRegister(REG_CONFIG, 0xffff, false);
		usleep(1000);
		const uint16_t config_reg = ReadRegister(REG_CONFIG);
		const uint16_t cal_reg = ReadRegister(REG_CALIBRATION);
		if(config_reg != 0x399f || cal_reg != 0) throw "chip reset failed";
	}

	static uint8_t SampleToResAvg(const uint8_t ns)
	{
		if(ns ==   0) throw "n_sample must at least be 1";
		if(ns ==   1) return 0b0011;
		if(ns ==   2) return 0b1001;
		if(ns <=   4) return 0b1010;
		if(ns <=   8) return 0b1011;
		if(ns <=  16) return 0b1100;
		if(ns <=  32) return 0b1101;
		if(ns <=  64) return 0b1110;
		if(ns <= 128) return 0b1111;
		throw "n_sample must be <= 128";
	}

	void TINA219::Calibrate(const float max_voltage, const float max_current_amps, const float r_shunt_ohm, const uint8_t n_sample_voltage, const uint8_t n_sample_current)
	{
		ARGERR(max_voltage > 32);
		const float max_shunt_voltage = max_current_amps * r_shunt_ohm;
		if(DEBUG) fprintf(stderr, "DEBUG: max_voltage = %f, max_current_amps = %f, r_shunt_ohm = %f,  max_shunt_voltage = %f\n", max_voltage, max_current_amps, r_shunt_ohm, max_shunt_voltage);
		if(max_shunt_voltage > 0.32f) throw "shunt resistor has a too high ohm value for the desired target current (shunt voltage will exeed 320mV)";

		Reset();

		struct config_t
		{
			uint16_t
				mode : 3,
				sadc : 4,
				badc : 4,
				pg : 2,
				brng : 1,
				__unused0 : 1,
				rst : 1;
		};

		config_t config;

		config.mode = 0b111;
		config.sadc = SampleToResAvg(n_sample_current);
		config.badc = SampleToResAvg(n_sample_voltage);

		if(max_shunt_voltage <= 0.04)
			config.pg = 0b00;
		else if(max_shunt_voltage <= 0.08)
			config.pg = 0b01;
		else if(max_shunt_voltage <= 0.16)
			config.pg = 0b10;
		else
			config.pg = 0b11;

		config.brng = max_voltage > 16;

		if(DEBUG) fprintf(stderr, "DEBUG: mode = %hu, sadc = %hu, badc = %hu, pg = %hu, brng = %hu\n", config.mode, config.sadc, config.badc, config.pg, config.brng);

		config.__unused0 = 0;
		config.rst = 0;

		WriteRegister(REG_CONFIG, *(uint16_t*)&config);

		this->current_lsb = max_current_amps / 32768.0f;
		const uint16_t cal_reg = (uint16_t)(0.04096f / (this->current_lsb * r_shunt_ohm)) & ~1;
		if(DEBUG) fprintf(stderr, "DEBUG: current_lsb = %lf, cal_reg = %hu\n", this->current_lsb, cal_reg);
		WriteRegister(REG_CALIBRATION, cal_reg);

		usleep(150000);	// wait long enough for initial conversion to complete
	}

	void TINA219::Refresh()
	{
		SYSERR(ioctl(this->fd_i2cbus, I2C_SLAVE, this->address));

		uint16_t voltage_reg = ReadRegister(REG_BUSVOLTAGE);
//		if((voltage_reg & 1) == 0) throw "chip or logic error, no data available";	// is never set in continous mode apparently
//		if((voltage_reg & 2) == 2) throw "math overflow on current or power calculation";	// is apparently ALWAYS set... pretty useless
		voltage_reg >>= 3;

		const int16_t current_reg = ReadRegister(REG_CURRENT);
		this->voltage = voltage_reg * 0.004;
		this->current = this->current_lsb * current_reg;
	}

	TINA219::TINA219(const int fd_i2cbus, const uint8_t address) : fd_i2cbus(fd_i2cbus), address(address), current_lsb(0), voltage(0), current(0)
	{
		if(DEBUG) fprintf(stderr, "DEBUG: fd_i2cbus = %d, address = 0x%hhx (%hhu)\n", fd_i2cbus, address, address);

		ARGERR(fd_i2cbus < 0);
		ARGERR(address > 127);

		Reset();
	}

	TINA219::TINA219(const int fd_i2cbus, const uint8_t address, const float max_voltage, const float max_current_amps, const float r_shunt_ohm, const uint8_t n_sample_voltage, const uint8_t n_sample_current) : fd_i2cbus(fd_i2cbus), address(address), current_lsb(0), voltage(0), current(0)
	{
		if(DEBUG) fprintf(stderr, "DEBUG: fd_i2cbus = %d, address = 0x%hhx (%hhu), max_current_amps = %f, r_shunt_ohm = %f\n", fd_i2cbus, address, address, max_current_amps, r_shunt_ohm);

		ARGERR(fd_i2cbus < 0);
		ARGERR(address > 127);

		Calibrate(max_voltage, max_current_amps, r_shunt_ohm, n_sample_voltage, n_sample_current);
	}
}
