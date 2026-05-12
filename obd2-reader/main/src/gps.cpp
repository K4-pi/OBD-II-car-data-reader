#include "../include/gps.h"
#include "uart.h"

#include <cstdlib>
#include <vector>
#include <cstring>

static float geo_to_float(char *str) // TODO: fix, wrong numbers
{
    float val = std::atof(str);
	int deg = (int)val / 100;
	float min = val - deg * 100;

	return deg + min / 60.0f;
}

void gps_gngll(char *gps_buffer, void *self)
{
	char *gngll = strstr(gps_buffer, "$GNGLL,");

	if (gngll)
	{
	    char *end = strchr(gngll, '\n');
		if (end) *end = '\0';

		char *s_ptr = strtok(gngll, ",");

		std::vector<char *> params;

		while (s_ptr)
		{
		    params.push_back(s_ptr);
                s_ptr = strtok(NULL, ",");
		}

		if (params.size() > 3)
		{
			float lat = geo_to_float(params.at(1));
			float lng = geo_to_float(params.at(3));

			((Uart *)self)->printf("GPS=%f,%f\n", lat, lng);
		}
	}
}
