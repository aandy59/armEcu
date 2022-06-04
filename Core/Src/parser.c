#include "parser.h"
#include "string.h"




uint32_t* parseString(char* bufer_for_processingRX, char delimeter, char terminator) {
	uint16_t str_length = strlen(bufer_for_processingRX);
	uint8_t del_counter = 0;
	static uint32_t package[5];

	for (int i = 0; i < str_length; i++) {
		if (bufer_for_processingRX[i] == delimeter) {
			del_counter++;
		}
		else if (bufer_for_processingRX[i] == terminator) {
			break;
		}
	}
	switch (del_counter) {
	case 0:
		sscanf(bufer_for_processingRX, "%d;", &package[0]);
		break;
	case 1:
		sscanf(bufer_for_processingRX, "%d,%d;", &package[0], &package[1]);
		break;
	case 2:
		sscanf(bufer_for_processingRX, "%d,%d,%d;", &package[0], &package[1], &package[2]);
		break;
	case 3:
		sscanf(bufer_for_processingRX, "%d,%d,%d,%d;", &package[0], &package[1], &package[2], &package[3]);
		break;
	case 4:
		sscanf(bufer_for_processingRX, "%d,%d,%d,%d,%d;", &package[0], &package[1], &package[2], &package[3], &package[4]);
		break;
	}
	return package;
}


