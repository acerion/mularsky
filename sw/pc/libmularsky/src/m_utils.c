#define _XOPEN_SOURCE /* strptime() */

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <linux/limits.h> /* PATH_MAX */

#include "m_utils.h"


char abs_path[PATH_MAX];



int m_get_parent_dir_name(char * buffer, size_t size)
{
	if (NULL == getcwd(abs_path, sizeof (abs_path))) {
		fprintf(stderr, "[EE] %s:%d: getcwd() failed\n", __FUNCTION__, __LINE__);
		return -1;
	}

	size_t pos = strlen(abs_path);
	if (pos == 0 || pos == 1) {
		return 0;
	}
	--pos; /* Convert length to position. */

	if (abs_path[pos] == '/') {
		pos--;
	}

	while (abs_path[pos] != '/') {
		pos--;
	}
	pos++;

	pos += 3; /* For "XX_" number in front of date. */

	if (strlen(abs_path + pos) > (size - 1)) {
		fprintf(stderr, "[EE] %s:%d: not enough place to store '%s'\n", __FUNCTION__, __LINE__, abs_path + pos);
		return -1;
	}

	return snprintf(buffer, size, "%s", abs_path + pos);
}




time_t m_nmea_gps_time_to_timestamp(const char * day_string, int nmea_ts, int ts_offset)
{
	/* "2017-04-30 133940" */
	nmea_ts += (ts_offset * 10000); /* Plus N hours for conversion from GPS's UTC to local time zone. */
	static char buffer[sizeof ("2017-04-30 133940") + 1] = { 0 };
	sprintf(buffer, "%s %d", day_string, nmea_ts);

	struct tm tm = { 0 };
	if (NULL == strptime(buffer, "%Y_%m_%d %H%M%S", &tm)) {
		fprintf(stderr, "[EE][%s] failed at strptime (%s)\n", __FUNCTION__, buffer);
		return (time_t) -1;
	}

	time_t time = mktime(&tm);
	if (time == -1) {
		fprintf(stderr, "[EE][%s] failed at mktime\n", __FUNCTION__);
		return (time_t) -1;
	}

	return time;
}
