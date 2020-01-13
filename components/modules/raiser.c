#include "raiser.h"

int32_t raiser_cascade_register(struct raiser *raiser, char *name, enum device_can can)
{
    char motor_name[2][OBJECT_NAME_MAX_LEN]={0};
    uint8_t name_len;
    uint32_t err;

    if (raiser==NULL)
        return -RM_INVAL;
    if (raiser_find(name) != NULL)
        return -RM_EXISTED;

    object_init(raiser, Object_Class_Raiser, name);

    name_len=strlen(name);

    if (name_len > OBJECT_NAME_MAX_LEN / 2)
    {
        name_len = OBJECT_NAME_MAX_LEN / 2;
    }
    for (int i = 0; i < 2; i++)
    {
    memcpy(&motor_name[i], name, name_len);
    raiser->motor[i].can_periph = can;
    //--------------------------------
    raiser->motor[i].can_id = 0x205 + i;//// waiting for motor
    //--------------------------------
    }

    memcpy(&motor_name[RAISER_MOTOR_INDEX_L][name_len], "_raiser_L\0", 8);
    memcpy(&motor_name[RAISER_MOTOR_INDEX_R][name_len], "_raiser_R\0", 8);

    for (int i = 0; i < 2; i++)
    {
        err = motor_device_register(&(raiser->motor[i]), motor_name[i], 0);
        if (err != RM_OK)
          goto end;
    }

    //Only control the left motor using cascade pid.
    raiser->ctrl.convert_feedback = raiser_ecd_input_convert;
    pid_struct_init(&(raiser->cascade.outer), 500, 600, 15, 0, 0); 
    pid_struct_init(&(raiser ->cascade.inter), 30000, 3000, 240, 0, 0);


    end:
        object_detach(raiser);

        return err;
    return RM_OK;

  
}

int32_t raiser_motor_enable(struct raiser *raiser)
{
  if (raiser == NULL)
    return -RM_INVAL;
      controller_enable(&(raiser->ctrl));

  return RM_OK;
}

raiser_t raiser_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Raiser);

  return (raiser_t)object;
}

static int32_t raiser_ecd_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  raiser_t data = (raiser_t)input;
  cascade_fdb->outer_fdb = data->raiser_angle ;
  cascade_fdb->inter_fdb = -data->raiser_angle ;
  return RM_OK;
}

int32_t raiser_set_angle(struct raiser *raiser, float left_raiser)
{
    if (raiser==NULL)
        return -RM_INVAL;
    else
    {
        VAL_LIMIT(left_raiser, RAISER_ANGLE_MIN, RAISER_ANGLE_MAX);
        raiser->raiser_target_angle=left_raiser;
    }

    return -RM_OK;
}

static int16_t raiser_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}

int32_t raiser_execute(struct raiser *raiser)
{
    float motor_out;
    struct motor_data *pdata;
    struct controller *ctrl;
    float raiser_ecd_angle;
    

    if (raiser==NULL)
        return -RM_INVAL;
        
    raiser_ecd_angle=raiser->raiser_target_angle;
    ctrl=&(raiser->ctrl);
    controller_set_input(ctrl,raiser_ecd_angle);

    pdata = motor_device_get_data(&(raiser->motor[RAISER_MOTOR_INDEX_L]));
    raiser->raiser_angle = raiser_get_ecd_angle(pdata->ecd, raiser->raiser_ecd_center[RAISER_MOTOR_INDEX_L]) / ENCODER_ANGLE_RATIO;
    controller_execute(&(raiser->ctrl), (void *)raiser);
    controller_get_output(&(raiser ->ctrl), &motor_out);
    motor_device_set_current(&(raiser->motor[RAISER_MOTOR_INDEX_L]), (int16_t)motor_out);

    pdata = motor_device_get_data(&(raiser->motor[RAISER_MOTOR_INDEX_R]));
    motor_device_set_current(&(raiser->motor[RAISER_MOTOR_INDEX_R]), (int16_t)motor_out);
    
}