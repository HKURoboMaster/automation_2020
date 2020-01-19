#include "plier.h"
#include "ramp.h"

float ecd_speed_js; //test 1
float ecd_angle_js;
float last_ecd_angle_js;
float target_angle_js;
float ecd_center_js;
float motor_out_js;
float cascade_outer_fdb_js;
float cascade_inter_fdb_js;
float controller_input_js;
float controller_output_js;
float cascade_inter_out_js;
float cascade_outer_out_js;
float ramp_angle_js;
int8_t err_js;
int8_t controller_ex_js;
int16_t center_offset_js;

int32_t plier_cascade_register(struct plier *plier, const char *name, enum device_can can)
{
  char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  uint32_t err;

  if (plier == NULL)
    return -RM_INVAL;
  if (plier_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(plier->parent), Object_Class_Plier, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }
  for (int i = 0; i < 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    plier->motor[i].can_periph = can;
    //--------------------------------
    plier->motor[i].can_id = 0x208 + i; // waiting for motor // test 1
    //--------------------------------
  }

  memcpy(&motor_name[PLIER_MOTOR_INDEX_L][name_len], "_PLIER_L\0", 9);
  memcpy(&motor_name[PLIER_MOTOR_INDEX_R][name_len], "_PLIER_R\0", 9);

  for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(plier->motor[i]), motor_name[i], 0);
    if (err != RM_OK)
      goto end;
  }

  //Only control the left motor using cascade pid.
  plier->ctrl.convert_feedback = plier_ecd_input_convert;
  pid_struct_init(&(plier->cascade.outer), 100, 40, 1, 0, 0);   // test 1
  pid_struct_init(&(plier->cascade.inter), 3000, 150, 1, 0, 0); // test 1
  plier->step = STEP_1;
  err = cascade_controller_register(&(plier->ctrl), motor_name[PLIER_MOTOR_INDEX_L],
                                    &(plier->cascade),
                                    &(plier->cascade_fdb), 1);
  err_js = err;
  if (err != RM_OK)
    goto end;

  return RM_OK;
end:
  object_detach(&(plier->parent));

  return err;
}

int32_t plier_set_offset(struct plier *plier, float angle)
{
  if (plier == NULL)
    return -RM_INVAL;
  plier->ecd_center = angle;
  return RM_OK;
}

int32_t plier_motor_enable(struct plier *plier)
{
  if (plier == NULL)
    return -RM_INVAL;
  controller_enable(&(plier->ctrl));

  return RM_OK;
}

int32_t plier_motor_disable(struct plier *plier)
{
  if (plier == NULL)
    return -RM_INVAL;
  controller_disable(&(plier->ctrl));

  return RM_OK;
}

plier_t plier_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Plier);

  return (plier_t)object;
}

static int32_t plier_ecd_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  plier_t data = (plier_t)input;
  cascade_fdb->outer_fdb = data->ecd_angle;
  cascade_fdb->inter_fdb = data->ecd_speed;
  cascade_outer_fdb_js = cascade_fdb->outer_fdb; //test 1
  cascade_inter_fdb_js = cascade_fdb->inter_fdb;
  return RM_OK;
}

int32_t plier_set_angle(struct plier *plier, float target)
{
  if (plier == NULL)
    return -RM_INVAL;

  float ramp;
  //if (target >= plier->ecd_angle + 2 * PLIER_RAMP_CO)
  //  ramp = plier->ecd_angle + PLIER_RAMP_CO;
  //else if (target <= plier->ecd_angle - 2 * PLIER_RAMP_CO)
 //   ramp = plier->ecd_angle - PLIER_RAMP_CO;
 // else
  {
    ramp = target;
  }

  VAL_LIMIT(ramp, PLIER_ANGLE_MIN, PLIER_ANGLE_MAX);
  plier->ramp_angle = ramp;
  //target_angle_js = plier->target_angle; //Test 1

  return RM_OK;
}
// Error
/*float plier_ramp(float target, float ecd)
{
  if (target >= ecd + 2 * PLIER_RAMP_CO)
    return ecd + PLIER_RAMP_CO;
  else if (target <= ecd - 2 * PLIER_RAMP_CO)
    return ecd - PLIER_RAMP_CO;
  return target;
}*/

static int16_t plier_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
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

int32_t plier_execute(struct plier *plier)
{
  float motor_out;
  struct motor_data *pdata;

  if (plier == NULL)
    return -RM_INVAL;

  struct controller *ctrl;
  float angle;
  angle = plier->target_angle;
  ctrl = &(plier->ctrl);
  VAL_LIMIT(angle, PLIER_ANGLE_MIN, PLIER_ANGLE_MAX);
  controller_set_input(ctrl, angle);

  pdata = motor_device_get_data(&(plier->motor[PLIER_MOTOR_INDEX_L]));

  plier->motor->data.ecd = fmod(plier->motor->data.total_ecd / 36.0f, 8192);
  plier->ecd_angle = PLIER_MOTOR_POSITIVE_DIR * plier_get_ecd_angle(pdata->ecd, plier->ecd_center) / ENCODER_ANGLE_RATIO;
  plier->ecd_speed = PLIER_MOTOR_POSITIVE_DIR * pdata->speed_rpm;
  controller_ex_js = controller_execute(&(plier->ctrl), (void *)plier);
  controller_get_output(&(plier->ctrl), &motor_out);
  motor_device_set_current(&(plier->motor[PLIER_MOTOR_INDEX_L]), (int16_t)PLIER_MOTOR_POSITIVE_DIR * motor_out); //motor_out //test 1

  //pdata = motor_device_get_data(&(plier->motor[PLIER_MOTOR_INDEX_R]));
  motor_device_set_current(&(plier->motor[PLIER_MOTOR_INDEX_R]), -(int16_t)PLIER_MOTOR_POSITIVE_DIR * motor_out); // test 1

  ecd_speed_js = plier->ecd_speed; //test 1
  ecd_angle_js = plier->ecd_angle;
  target_angle_js = plier->target_angle;
  ramp_angle_js = plier->ramp_angle;
  motor_out_js = motor_out;

  ecd_center_js = plier->ecd_center;
  controller_input_js = plier->ctrl.input;
  controller_output_js = plier->ctrl.output;
  cascade_inter_out_js = plier->cascade.inter.out;
  cascade_outer_out_js = plier->cascade.outer.out;
  center_offset_js = plier->ecd_center;

  return RM_OK;
}
