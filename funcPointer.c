#include "funcPointer.h"

bool callBack(uint8_t data) {
	return true;
}

bool (*p_func)(uint8_t) = NULL;

void funcInit(void) {
	p_func = callBack;

	(*p_func)(1);	//p_func(1);
}


