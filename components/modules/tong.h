#include "motor.h"
#include "pid_controller.h"

#define TONG_ANGLE_MAX      180.0f  /////  waiting for adjusting
#define TONG_ANGLE_MIN      0.0f  /////  waiting for adjusting
#define TONG_MOTOR_POSITIVE_DIR 1.0f ///// motor direction
enum motor_name
{
     TONG_MOTOR_INDEX_L = 0,
     TONG_MOTOR_INDEX_R,
     TONG_MOTOR_NUMBER,
};

struct tong
{
    struct object parent;
    enum motor_name motor_name;
    struct motor_device motor[2];
    struct cascade cascade;
    struct cascade_feedback cascade_fdb;
    struct controller ctrl;
    float tong_angle;
    float tong_target_angle;
    float tong_ecd_center;
    int8_t step;
};

typedef struct tong *tong_t;

int32_t tong_cascade_register(struct tong *tong, char *name, enum device_can can);
int32_t tong_motor_enable(struct tong *tong);
tong_t tong_find(const char *name);
static int32_t tong_ecd_input_convert(struct controller *ctrl, void *input);
int32_t tong_set_angle(struct tong *tong, float left_tong);
static int16_t tong_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);
int32_t tong_set_offset(struct tong *tong, uint16_t tong_encode_center);
int32_t tong_execute(struct tong *tong);