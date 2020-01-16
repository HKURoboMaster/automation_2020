
#ifndef __PLIER_TASK_H__
#define __PLIER_TASK_H__

#ifdef PLIER_TASK_H_GLOBAL
#define PLIER_TASK_H_EXTERN
#else
#define PLIER_TASK_H_EXTERN extern
#endif

#define PLIER_PERIOD 5
#define BACK_CENTER_TIME 10000
#define PLIER_OFFSET 0.0f

void plier_task(void const *argument);
void plier_init_state_reset(void);

#endif // __PLIER_TASK_H__
