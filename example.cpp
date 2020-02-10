#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ina219.hpp"

int main()
{
	try
	{
		using namespace ina219;
		DEBUG = false;

		const int fd_i2cbus = SYSERR(open("/dev/i2c-1", O_RDWR|O_CLOEXEC|O_NOCTTY|O_SYNC));
		TINA219 sensor(fd_i2cbus, 0x40, 24, 1.5, 0.1, 128, 128);

		while(true)
		{
			sensor.Refresh();
			printf("voltage = %g [V], current = %g [A], power = %g [W]\n\n", sensor.Voltage(), sensor.Current(), sensor.Power());
			usleep(200000);
		}

		return 0;
	}
	catch(const char* errmsg)
	{
		fprintf(stderr, "ERROR: %s\n", errmsg);
		return 1;
	}
	return 2;
}
