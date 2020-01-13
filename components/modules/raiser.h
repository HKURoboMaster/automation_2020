#include "motor.h"
#include "pid_controller.h"

#define RAISER_MOTOR_RADIUS 30 ////waiting for testing
#define RAISER_ANGLE_MAX 40.0
#define RAISER_ANGLE_MIN -40.0

enum motor_name
{
     RAISER_MOTOR_INDEX_L = 0,
     RAISER_MOTOR_INDEX_R,
     RAISER_MOTOR_NUMBER,
};

struct raiser
{
    struct object parent;
    enum motor_name motor_name;
    struct motor_device motor[2];
    struct cascade cascade;
    struct cascade_feedback cascade_fdb[2];
    struct controller ctrl;
    float raiser_angle;
    float raiser_target_angle;
    float raiser_ecd_center[RAISER_MOTOR_NUMBER];
};

typedef struct raiser *raiser_t;