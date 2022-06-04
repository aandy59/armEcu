#ifndef RHHDRIVER_H
#define RHHDRIVER_H

#include "stdint.h"
#include "stdbool.h"


struct state{
	//uint16_t currentPosition;
	bool currentDir;
	bool powerState;
};


struct state getCurrentState();
void stepRHH(bool dir, uint16_t number, uint16_t speed);

#endif /* __RHHDRIVER_H */
