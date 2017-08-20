#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

#include <wiringPi.h>

#include "m_misc.h"


#define PRESS_LIMIT 15


int main(int argc, char ** argv)
{
	wiringPiSetup();
	pinMode(G_GPIO_BUTTON, INPUT);
        pinMode(G_GPIO_LED, OUTPUT);


	int n_button_pressed = 0;
	for (;;) {
		int button_state = digitalRead(G_GPIO_BUTTON);
		if (button_state == LOW) {
			n_button_pressed++;
		} else {
			n_button_pressed = 0;
		}
		button_state = !button_state;
		fprintf(stderr, "button: state = %d, button counter = %d\n", button_state, n_button_pressed);

		sleep(1);

		if (n_button_pressed == PRESS_LIMIT) {
			break;
		}
	}

	system("sync");
	system("sudo systemctl stop mularsky");
	system("sudo killall -9 mularsky");
	system("sync");
	digitalWrite(G_GPIO_LED, HIGH);
	sleep(20);
	system("sudo poweroff");

	return 0;
}

