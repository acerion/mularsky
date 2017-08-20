#include <time.h>
#include <stdio.h>
#include <string.h>
#include "m_utils.h"


struct m_route_t {
	time_t start;
	time_t stop;
	char id[32];
};


#define NROUTES 15
struct m_route_t routes[10] = {{ 0 }};
int timestamp_shift = 0;


int main(void)
{
	FILE * config = fopen("config.txt", "r");
	if (!config) {
		fprintf(stderr, "[EE] can't open config file\n");
		return -1;
	}

	char line_buffer[128];
	int n_routes = 0;
	while (0 != fgets(line_buffer, sizeof (line_buffer), config)) {
		int r;
		r = sscanf(line_buffer, "route_%[A-Z],%ld,%ld", routes[n_routes].id, &routes[n_routes].start, &routes[n_routes].stop);
		if (r == 3) {
			n_routes++;
			if (n_routes == NROUTES) {
				fprintf(stderr, "[EE] reached limit of routes\n");
				return -1;
			}
			continue;
		}

		int tmp;
		r = sscanf(line_buffer, "ts_shift,%d", &tmp);
		if (r == 1) {
			timestamp_shift = tmp;
		}
	}
	fclose(config);
	config = NULL;


	/* Validate config. */
	for (int i = 0; i < n_routes; i++) {
		if (routes[i].stop <= routes[i].start) {
			fprintf(stderr, "[EE] route '%s': invalid start/stop: %lu, %lu\n", routes[i].id, routes[i].start, routes[i].stop);
			return -1;
		}
	}



	fprintf(stderr, "[II] time stamp shift = %d\n", timestamp_shift);

	char parent_dir[128];
	m_get_parent_dir_name(parent_dir, sizeof (parent_dir));
	fprintf(stderr, "[II] parent dir is '%s'\n", parent_dir);
	parent_dir[strlen("2017_04_30")] = '\0';



	/* Read and split. */



	/* First process nmea.txt file. Go over it as many times, as there are routes. */
	for (int i = 0; i < n_routes; i++) {

		fprintf(stderr, "[II] carving out NMEA route '%s', from %lu to %lu\n", routes[i].id, routes[i].start, routes[i].stop);

		FILE * file_in = fopen("nmea.txt", "r");
		if (!file_in) {
			fprintf(stderr, "[EE] can't open input nmea file\n");
			return -1;
		}


		char filename[64];
		snprintf(filename, sizeof (filename), "nmea_%s.txt", routes[i].id);
		FILE * file_out = fopen(filename, "w+");
		if (!file_out) {
			fprintf(stderr, "[EE] can't open output nmea file '%s'\n", filename);
			fclose(file_in);
			return -1;
		}


		int forward = 0;
		while (0 != fgets(line_buffer, sizeof (line_buffer), file_in)) {
			int nmea_timestamp;
			int tmp;
			if (2 == sscanf(line_buffer, "$GPRMC,%d.%d", &nmea_timestamp, &tmp)
			    || 2 == sscanf(line_buffer, "$GPGGA,%d.%d", &nmea_timestamp, &tmp)) {


				time_t timestamp = m_nmea_gps_time_to_timestamp(parent_dir, nmea_timestamp, timestamp_shift);

				if (timestamp >= routes[i].start && timestamp <= routes[i].stop) {
					forward = 1;
				} else {
					forward = 0;
				}

				//fprintf(stderr, "[II] found NMEA timestamp %d in '%s'\n", nmea_timestamp, line_buffer);
			}

			if (forward) {
				fprintf(file_out, "%s", line_buffer);
			}
		}

		fclose(file_in);
		file_in = NULL;

		fclose(file_out);
		file_out = NULL;
	}


	return 0;
}
