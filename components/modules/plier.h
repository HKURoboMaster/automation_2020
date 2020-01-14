
#ifndef __PLIER_H__
#define __PLIER_H__

#ifdef PLIER_H_GLOBAL
#define PLIER_H_EXTERN
#else
#define PLIER_H_EXTERN extern
#endif

#include "motor.h"
#include "pid_controller.h"

#define PLIER_ANGLE_MAX 180.0f       /////  waiting for adjusting
#define PLIER_ANGLE_MIN 0.0f         /////  waiting for adjusting
#define PLIER_MOTOR_POSITIVE_DIR 1.0 ///// motor direction

#define CALIED_FLAG 0x55

enum motor_name
{
    PLIER_MOTOR_INDEX_L = 0,
    PLIER_MOTOR_INDEX_R,
    PLIER_MOTOR_NUMBER,
};

enum step
{
    STEP_1 = 1,
    STEP_2,
    STEP_3,
    STEP_4,
    STEP_5,
    STEP_6,
};

struct plier
{
    struct object parent;
    enum motor_name motor_name;
    struct motor_device motor[2];
    struct cascade cascade;
    struct cascade_feedback cascade_fdb[2];
    struct controller ctrl;
    float angle_speed; //degree per ms
    float ecd_angle;
    float last_ecd_angle;
    float target_angle;
    float ecd_center;
    enum step step;
};

typedef struct plier *plier_t;

int32_t plier_cascade_register(struct plier *plier, char *name, enum device_can can);
int32_t plier_set_offset(struct plier *plier, float ecd_angle);
int32_t plier_motor_enable(struct plier *plier);
int32_t plier_motor_disable(struct plier *plier);
plier_t plier_find(const char *name);
static int32_t plier_ecd_input_convert(struct controller *ctrl, void *input);
int32_t plier_set_angle(struct plier *plier, float left_plier);
static int16_t plier_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);
int32_t calc_ecd_angle_speed(struct plier *plier, uint32_t time);
int32_t plier_execute(struct plier *plier);

#endif // __PLIER_H__
