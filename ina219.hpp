#pragma once

#include <stdint.h>

#define SYSERR(expr) (([&](){ const auto r = ((expr)); if( (long)r == -1L ) { perror(#expr); throw #expr; } else return r; })())
#define ARGERR(expr) do { if(expr) throw #expr; } while(false)

namespace ina219
{
	extern bool DEBUG;

	class TINA219
	{
		protected:
			const int fd_i2cbus;
			const uint8_t address;
			float current_lsb;
			float voltage;
			float current;

			void WriteRegister(const uint8_t register_address, const uint16_t value, const bool verify = true);
			uint16_t ReadRegister(const uint8_t register_address);

		public:
			inline float Voltage() const { return this->voltage; }
			inline float Current() const { return this->current; }
			inline float Power() const   { return Voltage() * Current(); }

			void Refresh();
			void Reset();
			void Calibrate(const float max_voltage, const float max_current_amps, const float r_shunt_ohm, const uint8_t n_sample_voltage, const uint8_t n_sample_current);

			TINA219(const int fd_i2cbus, const uint8_t address);
			TINA219(const int fd_i2cbus, const uint8_t address, const float max_voltage, const float max_current_amps, const float r_shunt_ohm, const uint8_t n_sample_voltage, const uint8_t n_sample_current);
	};
}
