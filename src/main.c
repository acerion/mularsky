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
#include "m_i2c.h"



bool cancel_treads;
extern int pressure_sensor_fd;

static pthread_t pressure_thread;




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

	sleep(1);

	return;
}




int main(int argc, char ** argv)
{
	atexit(m_atexit);
	signal(SIGINT, m_sighandler);

	if (-1 == pressure_prepare()) {
		exit(EXIT_FAILURE);
	}
	errno = 0;
        int rv = pthread_create(&pressure_thread, NULL, pressure_thread_fn, NULL);
        fprintf(stderr, "pressure thread created: %d / %s\n", rv, strerror(errno));


        errno = 0;
        rv = pthread_join(pressure_thread, NULL);
        fprintf(stderr, "pressure thread joined: %d / %s\n", rv, strerror(errno));

        sleep(1);


	exit(EXIT_SUCCESS);
}
