#define _BSD_SOURCE /* usleep() */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <time.h>

#include <wiringPi.h>

#include "m_bme280.h"
#include "m_bno055.h"
#include "m_i2c.h"
#include "m_misc.h"


time_t global_time;


#define BEGINNING_OF_2017   1483232401 /* [s] since epoch. */

int imu_led_time = BLINK_NOK;
int pressure_led_time = BLINK_NOK;
int gps_led_time = BLINK_NOK;
const int space_led_time = 500;
const int off_led_time = 1000;


bool cancel_treads;
extern int pressure_sensor_fd;
extern int imu_sensor_fd;

static pthread_t pressure_thread;
static pthread_t imu_thread;

static bool run_pressure = false;
static bool run_imu = true;



/* WiringPi numbering method. */
#define G_GPIO_LED       29
#define G_GPIO_BUTTON    28
//#define G_GPIO_GPS_PPS    7


static FILE * button_out_fd;
static const char * data_filename = "button.txt";


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
	wiringPiSetup();
	pinMode(G_GPIO_LED, OUTPUT);
	pinMode(G_GPIO_BUTTON, INPUT);


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


	if (dir_path == NULL) {
		button_out_fd = stderr;
	} else {
		char buffer[64] = { 0 };
		snprintf(buffer, sizeof (buffer), "%s/%s", dir_path, data_filename);
		button_out_fd = fopen(buffer, "w");
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



	int n_button_pressed = 0;
	for (;;) {

		if (global_time > BEGINNING_OF_2017) {
			gps_led_time = BLINK_OK;
		}

		int button_state = digitalRead(G_GPIO_BUTTON);
		if (button_state == LOW) {
			n_button_pressed++;
		} else {
			n_button_pressed = 0;
		}
		button_state = !button_state;
		fprintf(button_out_fd, "button: state = %d, button counter = %d\n", button_state, n_button_pressed);


		digitalWrite(G_GPIO_LED, button_state * HIGH);
		usleep(1000 * gps_led_time);

		digitalWrite(G_GPIO_LED, LOW);
		usleep(1000 * space_led_time);

		digitalWrite(G_GPIO_LED, button_state * HIGH);
		usleep(1000 * pressure_led_time);

		digitalWrite(G_GPIO_LED, LOW);
		usleep(1000 * space_led_time);

		digitalWrite(G_GPIO_LED, button_state * HIGH);
		usleep(1000 * imu_led_time);

		digitalWrite(G_GPIO_LED, LOW);
		usleep(1000 * off_led_time);
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
