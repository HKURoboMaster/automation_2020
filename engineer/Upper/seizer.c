#include "seizer.h"
#include "upper.h"
#include "engg_gpio.h"

/* VARIABLES: SEIZER - related */
Seizer seizer;
extern struct upper_info upper;
extern upper_ctrl upper_controller;
/* END of VARIABLES: SEIZER - related */

/* FUNCTIONS: SEIZER - related */
void seizer_seize(void);
void seizer_release(void);

int32_t seizer_execute(struct upper_info* upperinf, upper_ctrl* upperctrl) {
	if (upperinf == NULL || upperctrl == NULL)
		return -RM_INVAL;
	
	if (seizer.request == RELEASE)
		seizer_release();
	else if (seizer.request == SEIZE)
		seizer_seize();
	
	return RM_OK;
}

void seizer_seize_cd(void) {
	if (seizer.state != SEIZER_SEIZED) {
		osDelay(SEIZER_SEIZE_MS);
		seizer.state = SEIZER_SEIZED;
	}
}

void seizer_seize(void) {
	pneum_1head_LOW(&(seizer.cylinderX1));
}

void seizer_release_cd(void) {
	if (seizer.state != SEIZER_RELEASED) {
		osDelay(SEIZER_RELEASE_MS);
		seizer.state = SEIZER_RELEASED;
	}
}

void seizer_release(void) {
	pneum_1head_HIGH(&(seizer.cylinderX1));
}

void seizer_state_change(int state) {
	seizer.request = state;
}

int return_seizer_state(void) {
	return seizer.state;
}
/* END of FUNCTIONS: SEIZER - related */

/* RTOS: SEIZER - related */
void seizer_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
	
	seizer.cylinderX1 = init_pneum_1head(SEIZE_GPIO_Port, SEIZE_Pin);
	seizer.request = RELEASE;
	seizer.state = RELEASE;
	
	for(;;) {
		seizer_execute(&upper, &upper_controller);

    osDelayUntil(&period, 2);
	}
}
/* END of RTOS: FLIPPER - related */

