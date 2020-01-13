#include "tong.h"

int32_t tong_cascade_register(struct tong *tong, char *name, enum device_can can)
{
    char motor_name[2][OBJECT_NAME_MAX_LEN]={0};
    uint8_t name_len;
    uint32_t err;

    if (tong==NULL)
        return -RM_INVAL;
    if (tong_find(name) != NULL)
        return -RM_EXISTED;

    object_init(tong, Object_Class_Tong, name);

    name_len=strlen(name);

    if (name_len > OBJECT_NAME_MAX_LEN / 2)
    {
        name_len = OBJECT_NAME_MAX_LEN / 2;
    }
    for (int i = 0; i < 2; i++)
    {
    memcpy(&motor_name[i], name, name_len);
    tong->motor[i].can_periph = can;
    //--------------------------------
    tong->motor[i].can_id = 0x205 + i;//// waiting for motor
    //--------------------------------
    }

    memcpy(&motor_name[TONG_MOTOR_INDEX_L][name_len], "_TONG_L\0", 8);
    memcpy(&motor_name[TONG_MOTOR_INDEX_R][name_len], "_TONG_R\0", 8);

    for (int i = 0; i < 2; i++)
    {
        err = motor_device_register(&(tong->motor[i]), motor_name[i], 0);
        if (err != RM_OK)
          goto end;
    }

    //Only control the left motor using cascade pid.
    tong->ctrl.convert_feedback = tong_ecd_input_convert;
    pid_struct_init(&(tong->cascade.outer), 500, 600, 15, 0, 0); 
    pid_struct_init(&(tong ->cascade.inter), 30000, 3000, 240, 0, 0);


    end:
        object_detach(tong);

        return err;
    return RM_OK;

  
}

int32_t tong_motor_enable(struct tong *tong)
{
  if (tong == NULL)
    return -RM_INVAL;
      controller_enable(&(tong->ctrl));

  return RM_OK;
}

tong_t tong_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Tong);

  return (tong_t)object;
}

static int32_t tong_ecd_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  tong_t data = (tong_t)input;
  cascade_fdb->outer_fdb = data->tong_angle ;
  cascade_fdb->inter_fdb = -data->tong_angle ;
  return RM_OK;
}

int32_t tong_set_angle(struct tong *tong, float left_tong)
{
    if (tong==NULL)
        return -RM_INVAL;
    else
    {
        VAL_LIMIT(left_tong, TONG_ANGLE_MIN, TONG_ANGLE_MAX);
        tong->tong_target_angle=left_tong;
    }

    return -RM_OK;
}

static int16_t tong_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
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

int32_t tong_set_offset(struct tong *tong, uint16_t tong_encode_center)
{
  if (tong==NULL)
    return -RM_INVAL;
  else
  {
    tong->tong_ecd_center = tong_encode_center;
  }
  return -RM_OK;
}

int32_t tong_execute(struct tong *tong)
{
    float motor_out;
    struct motor_data *pdata;
    struct controller *ctrl;
    float tong_ecd_angle;
    

    if (tong==NULL)
        return -RM_INVAL;
        
    tong_ecd_angle=tong->tong_target_angle;
    ctrl=&(tong->ctrl);
    controller_set_input(ctrl,tong_ecd_angle);

    pdata = motor_device_get_data(&(tong->motor[TONG_MOTOR_INDEX_L]));
    tong->tong_angle = TONG_MOTOR_POSITIVE_DIR * tong_get_ecd_angle(pdata->ecd, tong->tong_ecd_center) / ENCODER_ANGLE_RATIO;
    controller_execute(&(tong->ctrl), (void *)tong);
    controller_get_output(&(tong ->ctrl), &motor_out);
    motor_device_set_current(&(tong->motor[TONG_MOTOR_INDEX_L]), (int16_t)TONG_MOTOR_POSITIVE_DIR * motor_out);

    pdata = motor_device_get_data(&(tong->motor[TONG_MOTOR_INDEX_R]));
    motor_device_set_current(&(tong->motor[TONG_MOTOR_INDEX_R]), -(int16_t)TONG_MOTOR_POSITIVE_DIR * motor_out);
  
  return RM_OK;
    
}