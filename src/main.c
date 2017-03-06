#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>


#include "m_bme280.h"
#include "m_bno055.h"
#include "m_i2c.h"



bool cancel_treads;
extern int pressure_sensor_fd;
extern int imu_sensor_fd;

static pthread_t pressure_thread;
static pthread_t imu_thread;

static bool run_pressure = false;
static bool run_imu = true;




void m_sighandler(int sig)
{
	fprintf(stderr, "\n");
	cancel_treads = true;
	sleep(2);
	exit(EXIT_SUCCESS);
}




void m_atexit(void)
{
	if (pressure_sensor_fd) {
		fprintf(stderr, "closing pressure sensor file\n");
		close(pressure_sensor_fd);
		pressure_sensor_fd = 0;
	}

	if (imu_sensor_fd) {
		fprintf(stderr, "closing imu sensor file\n");
		close(imu_sensor_fd);
		imu_sensor_fd = 0;
	}

	sleep(1);

	return;
}




int main(int argc, char ** argv)
{
	atexit(m_atexit);
	signal(SIGINT, m_sighandler);
	signal(SIGTERM, m_sighandler);

	char * dir_path = NULL;
	if (argc == 2) {
		fprintf(stderr, "%s: checking path %s\n", argv[0], argv[1]);
		if (0 != access(argv[1], X_OK | W_OK)) {
			exit(EXIT_FAILURE);
		}
		dir_path = argv[1];
	}

	if (run_pressure) {
		if (-1 == pressure_prepare(dir_path)) {
			exit(EXIT_FAILURE);
		}
		errno = 0;
		int rv = pthread_create(&pressure_thread, NULL, pressure_thread_fn, NULL);
		fprintf(stderr, "pressure thread created: %d / %s\n", rv, strerror(errno));
	}

	if (run_imu) {
		if (-1 == imu_prepare(dir_path)) {
			exit(EXIT_FAILURE);
		}
		errno = 0;
		int rv = pthread_create(&imu_thread, NULL, imu_thread_fn, NULL);
		fprintf(stderr, "imu thread created: %d / %s\n", rv, strerror(errno));
	}



	if (run_pressure) {
		errno = 0;
		int rv = pthread_join(pressure_thread, NULL);
		fprintf(stderr, "pressure thread joined: %d / %s\n", rv, strerror(errno));
	}

	if (run_imu) {
		errno = 0;
		int rv = pthread_join(imu_thread, NULL);
		fprintf(stderr, "imu thread joined: %d / %s\n", rv, strerror(errno));
	}


        sleep(1);


	exit(EXIT_SUCCESS);
}
