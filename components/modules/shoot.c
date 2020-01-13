/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "shoot.h"
#include "drv_io.h"
#include "referee_system.h"

static uint8_t trigger_motor_status(struct shoot *shoot, uint32_t index);
static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input);
static int32_t fric_shoot_pid_input_convert(struct controller *ctrl, void *input);
static int32_t shoot_fric_ctrl(struct shoot *shoot);
static int32_t shoot_cmd_ctrl(struct shoot *shoot);
static int32_t shoot_block_check(struct shoot *shoot);

int32_t shoot_pid_register(struct shoot *shoot, const char *name, enum device_can can)
{
  char motor_name[4][OBJECT_NAME_MAX_LEN];
  uint8_t name_len;
  int32_t err;

  if (shoot == NULL)
    return -RM_INVAL;
  if (shoot_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(shoot->parent), Object_Class_Shoot, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }

  for (int i = 0; i < 4; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    shoot->motor[i].can_periph = can;
    shoot->motor[i].can_id = 0x207 + i;
    shoot->motor[i].init_offset_f = 1;
  }
  memcpy(&motor_name[2], name, name_len);
  memcpy(&motor_name[3], name, name_len);

  shoot->ctrl[SHOOT_MOTOR_INDEX_L].convert_feedback = shoot_pid_input_convert;
  pid_struct_init(&(shoot->motor_pid[SHOOT_MOTOR_INDEX_L]), 15000, 7500, 10, 0.3, 0);

  shoot->ctrl[SHOOT_MOTOR_INDEX_R].convert_feedback = shoot_pid_input_convert;
  pid_struct_init(&(shoot->motor_pid[SHOOT_MOTOR_INDEX_R]), 15000, 7500, 10, 0.3, 0);

  shoot->fric_ctrl[SHOOT_MOTOR_INDEX_L].convert_feedback = fric_shoot_pid_input_convert;
  pid_struct_init(&(shoot->fric_motor_pid[SHOOT_MOTOR_INDEX_L]), 15000, 7500, 10, 0.3, 0); // TODO: readjust the PID parameter

  shoot->fric_ctrl[SHOOT_MOTOR_INDEX_R].convert_feedback = fric_shoot_pid_input_convert;
  pid_struct_init(&(shoot->fric_motor_pid[SHOOT_MOTOR_INDEX_R]), 15000, 7500, 10, 0.3, 0); // TODO: readjust the PID parameter

  shoot->param.block_current = BLOCK_CURRENT_DEFAULT;
  shoot->param.block_speed = BLOCK_SPEED_DEFAULT;
  shoot->param.block_timeout = BLOCK_TIMEOUT_DEFAULT;
  shoot->param.turn_speed = TURN_SPEED_DEFAULT;
  shoot->param.check_timeout = BLOCK_CHECK_TIMEOUT_DEFAULT;

  shoot->fric_spd[0][0] = FRIC_MIN_SPEED; //  TODO: the purpose of these line is not defined
  shoot->fric_spd[1][0] = FRIC_MIN_SPEED;
  shoot->fric_spd[0][1] = FRIC_MIN_SPEED;
  shoot->fric_spd[1][1] = FRIC_MIN_SPEED;

  memcpy(&motor_name[SHOOT_MOTOR_INDEX_L][name_len], "_SHOOT1\0", 8); // Potential bugs
  memcpy(&motor_name[SHOOT_MOTOR_INDEX_R][name_len], "_SHOOT2\0", 8);
  for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(shoot->motor[i]), motor_name[i], 0);
    if (err != RM_OK)
      goto end;
  }

  memcpy(&motor_name[SHOOT_MOTOR_INDEX_L][name_len], "_CTL_1\0", 7);
  memcpy(&motor_name[SHOOT_MOTOR_INDEX_R][name_len], "_CTL_2\0", 7);

  for (int i = 0; i < 2; i++)
  {
    err = pid_controller_register(&(shoot->ctrl[i]), motor_name[i], &(shoot->motor_pid[i]), &(shoot->motor_feedback[i]), 1);
    if (err != RM_OK)
      goto end;
  }

  memcpy(&motor_name[2][name_len], "_CTL_FRIC_1\0", 12);
  memcpy(&motor_name[3][name_len], "_CTL_FRIC_2\0", 12);
  for (int i = 0; i < 2; i++)
  {
    err = pid_controller_register(&(shoot->fric_ctrl[i]), motor_name[i], &(shoot->fric_motor_pid[i]), &(shoot->fric_motor_feedback[i]), 1);
    if (err != RM_OK)
      goto end;
  }

  for (int i = 0; i < 2; i++)
  {
    shoot_state_update(shoot, i);
    controller_disable(&(shoot->ctrl[i]));
  }

  for (int i = 0; i < 2; i++)
  {
    controller_disable(&(shoot->fric_ctrl[i]));
  }

  return RM_OK;
end:
  object_detach(&(shoot->parent));

  return err;
}

/**Edited by Y.H. Liu
 * @Jul 8, 2019: Change the slope to the function: shoot_fric_ctrl
 * 
 * Set the target speed of the trigger motor
 */
int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd, uint16_t index)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot->target[index].fric_spd = fric_spd;
  return RM_OK;
}

// int32_t shoot_get_fric_speed(struct shoot *shoot, float *fric_spd1, float *fric_spd2)
// {
//   if (shoot == NULL)
//     return -RM_INVAL;
//   uint16_t fric_pwm_1, fric_pwm_2;
//   fric_get_speed(&fric_pwm_1, &fric_pwm_2);
//   *fric_spd1 = *fric_spd1 - (int16_t)*fric_spd1; // set fric_spd to 0
//   *fric_spd2 = *fric_spd2 - (int16_t)*fric_spd2;
//   *fric_spd1 += fric_pwm_1; // set fric_spd to fric_pwm
//   *fric_spd2 += fric_pwm_2;
//   return RM_OK;
// }

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullets
 * @Jul 4, 2019: Not count the bullets
 * 
 * Receive the command from shoot_task.c
 */
int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num, uint32_t index)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot->cmd[index] = cmd;
  /*------ No matter what command it is, always count the bullets ------*/
  shoot->target[index].shoot_num = shoot->shoot_num[index] + shoot_num;
  return RM_OK;
}

int32_t shoot_execute(struct shoot *shoot)
{
  float motor_out;
  struct motor_data *pdata;

  if (shoot == NULL)
    return -RM_INVAL;

  shoot_fric_ctrl(shoot);
  shoot_cmd_ctrl(shoot); // shoot_block_check(shoot); Inside the shoot_cmd_ctrl and inside state_update

  for (int i = 0; i < 2; i++)
  {
    pdata = motor_device_get_data(&(shoot->motor[i]));
    controller_set_input(&(shoot->ctrl[i]), shoot->target[i].motor_speed);
    controller_execute(&(shoot->ctrl[i]), (void *)pdata);
    controller_get_output(&(shoot->ctrl[i]), &motor_out);

    motor_device_set_current(&shoot->motor[i], (int16_t)motor_out);
  }

  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullet
 * @Jun 13, 2019: Bypass the trigger switch
 * @Jul 4, 2019: Change the FSM
 * 
 * Detect whether a bullet has been shot and then update the state
 */
int32_t shoot_state_update(struct shoot *shoot, uint32_t index)
{
  if (shoot == NULL)
    return -RM_INVAL;

  // shoot->trigger_key = get_trig_status();
  shoot->trigger_key[index] = trigger_motor_status(shoot, index);
  /*------ Use the encoder of the trigger motor to bypass the switch ------*/
  switch (shoot->state[index])
  {
  case SHOOT_READY:
    if (shoot->cmd[index] != SHOOT_STOP_CMD) // One bullet to be shot
      shoot->state[index] = SHOOT_INIT;
    //else, remain in ready
    break;
  case SHOOT_INIT:
    if (shoot->cmd[index] == SHOOT_ONCE_CMD)
      shoot->cmd[index] = SHOOT_STOP_CMD;
    if (shoot->trigger_key[index] == TRIG_BOUNCE_UP) //One bullet in chamber
      shoot->state[index] = SHOOT_RUNNING;
    //else, remain in INIT
    break;
  case SHOOT_RUNNING:
    if (shoot->trigger_key[index] == TRIG_PRESS_DOWN) // One bullet away
    {
      shoot->state[index] = SHOOT_READY;
      shoot->shoot_num[index] += 1;
    }
    //else, remain in RUNNING
    break;
  default:
    shoot->state[index] = SHOOT_READY;
  }
  return RM_OK;
}

int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed)
{
  if (shoot == NULL)
    return -RM_INVAL;
  VAL_LIMIT(speed, 1000, 2500);
  shoot->param.turn_speed = speed;

  return RM_OK;
}

shoot_t shoot_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Shoot);

  return (shoot_t)object;
}

int32_t shoot_enable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    controller_enable(&(shoot->ctrl[i]));
    controller_enable(&(shoot->fric_ctrl[i]));
  }

  return RM_OK;
}

int32_t shoot_disable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot_set_fric_speed(shoot, FRIC_STOP_SPEED, 0);
  shoot_set_fric_speed(shoot, FRIC_STOP_SPEED, 1);
  for (int i = 0; i < 2; i++)
  {
    controller_disable(&(shoot->ctrl[i]));
    controller_disable(&(shoot->fric_ctrl[i]));
  }

  return RM_OK;
}

static int32_t shoot_block_check(struct shoot *shoot)
{
  static uint8_t first_block_f[2] = {0, 0}; // The block flag for the first motor
  static uint32_t check_time[2];

  if (shoot == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    if (shoot->motor[i].current > shoot->param.block_current)
    {
      if (first_block_f[i] == 0)
      {
        first_block_f[i] = 1;
        check_time[i] = get_time_ms();
      }
      else if (get_time_ms() - check_time[i] > shoot->param.check_timeout)
      {
        first_block_f[i] = 0;
        shoot->block_time[i] = get_time_ms();
        shoot->state[i] = SHOOT_BLOCK;
      }
    }
    else
    {
      first_block_f[i] = 0;
    }
  }

  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jun 13, 2019: Count the bullets and if too many are shot, stop shooting
 * @Jul 4, 2019: Simpify the FSM output
 * @Jul 24, 2019: change the state-updating sequence
 * 
 * Set the controlling signals for the trigger motor
 */
static int32_t shoot_cmd_ctrl(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot_block_check(shoot);
  for (int i = 0; i < 2; i++)
  {
    if (shoot->state[i] == SHOOT_BLOCK)
    {
      shoot->target[i].motor_speed = shoot->param.block_speed;
      if (get_time_ms() - shoot->block_time[i] > shoot->param.block_timeout)
      {
        shoot_state_update(shoot, i);
      }
    }
    else
    {
      shoot_state_update(shoot, i);                                          //update according to cmd and trigger status
      if (shoot->state[i] == SHOOT_INIT || shoot->state[i] == SHOOT_RUNNING) //start to shoot, trigger motor running
      {
        shoot->target[i].motor_speed = shoot->param.turn_speed;
      }
      else if (shoot->state[i] == SHOOT_READY) //ready for the next bullet, tirgger motor stopped
      {
        shoot_state_update(shoot, i); //update according to cmd and trigger status
        shoot->target[i].motor_speed = 0;
      }
      else
        return RM_INVAL;
    }

    // TODO: when the encoder arrive, modify the getter for the friction wheel speed !!!
    if (shoot->fric_spd[i][0] < (FRIC_MAX_SPEED + FRIC_MIN_SPEED) / 2 || shoot->fric_spd[i][1] < (FRIC_MAX_SPEED + FRIC_MIN_SPEED) / 2)
    {
      controller_disable(&(shoot->ctrl[i]));
    }
  }

  return RM_OK;
}

/**Edited by Y.H. Liu
 * @Jul 8, 2019: Slow down the speeding up
 * @Jul 30, 2019: Use strlen to determine whether the given shooter is shoot2
 * @Jan 13, 2020: Edited by Y.Z. Yang  Adaption to sentry(now control two barral by PID Close loop)
 * Send the friction wheel motor signals by PWM
 */
static int32_t shoot_fric_ctrl(struct shoot *shoot)
{
  float *fric_rpm;
  float fric_PWM_out;
  if (shoot == NULL)
    return -RM_INVAL;

  for (int i = 0; i < 2; i++)
  {
    fric_rpm = shoot_get_fric_speed();   // Potential bugs
    controller_set_input(&(shoot->fric_ctrl[i]), shoot->target[i].fric_spd);
    controller_execute(&(shoot->fric_ctrl[i]), (void *)fric_rpm);
    controller_get_output(&(shoot->fric_ctrl[i]), &fric_PWM_out);
    VAL_LIMIT(fric_PWM_out, FRIC_STOP_SPEED, FRIC_MAX_SPEED);  // Potential bugs
    if (i == 0)
    {
      fric_set_output((uint16_t)fric_PWM_out, (uint16_t)fric_PWM_out);
    }
    else
    {
      fric_set_output2((uint16_t)fric_PWM_out, (uint16_t)fric_PWM_out);
    }
  }
  return RM_OK;
}

static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  motor_data_t data = (motor_data_t)input;
  pid_fdb->feedback = data->speed_rpm;

  return RM_OK;
}

static int32_t fric_shoot_pid_input_convert(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  pid_fdb->feedback = *((float *)input);

  return RM_OK;
}

/**Added by Y.H. Liu
 * @Jun 13, 2019: Define the function
 * @Jul 4, 2019: Change the threashold and using abs value
 * @Jul 24, 2019: Simplify the control logic
 * @Jan 13,2020: Adaption Uniquely for sentry robot  Edited by Yang Yuezhi
 * Replace the trigger switch by the total angle of the trigger motor
 */
static uint8_t trigger_motor_status(struct shoot *shoot, uint32_t index)
{
  int32_t bullet_passing_offset = ((shoot->motor[index].data.total_angle / 36) % 360) % 45;
  bullet_passing_offset = abs(bullet_passing_offset);
  if (bullet_passing_offset >= 5 && bullet_passing_offset < 40 && shoot->motor[index].data.ecd != shoot->motor[index].data.last_ecd)
    return TRIG_BOUNCE_UP;
  else
    return TRIG_PRESS_DOWN;
}

/**Edited by Y.H Liu
 * @Jun 15, 2019: declare the functions
 * 
 * Control the laser. 
 * @param cmd 0----laser off
 *            1----laser on
 */
int32_t laser_cmd(uint8_t cmd)
{
  if (cmd)
    WRITE_HIGH_LASER();
  else
    WRITE_LOW_LASER();
  return 0;
}

/**Edited by Y.H Liu
 * @Jun 15, 2019: declare the functions
 * 
 * Control the lid. 
 * @param cmd 0----lid off
 *            1----lid on
 */
int32_t magazine_lid_cmd(uint8_t cmd)
{
  if (cmd)
    MAGA_SERVO = 45;
  else
    MAGA_SERVO = 170;
  return 0;
}

/**Edited by Y. Z. Yang
 * @Jan 12, 2020: declare the functions to update the shoot overhear event
 * 
 *  
 * @param shoot_eve--- 3 level to determine the degreee of shooting overheat
 * //TODO: update this function when new referee system is installed
 */
void shoot_event_update(shoot_event_t *shoot_eve)
{
  ext_power_heat_data_t *referee_power = get_heat_power();
  if (referee_power->shooter_heat0 > 320)
  { // Q0 = 160 for sentry
    shoot_eve->shoot1_state = SHOOT_OVERHEAT_2X;
    shoot_eve->shoot2_state = SHOOT_OVERHEAT_2X;
  }
  else if (referee_power->shooter_heat0 > 160 && referee_power->shooter_heat0 < 320)
  {
    shoot_eve->shoot1_state = SHOOT_OVERHEAT_1X;
    shoot_eve->shoot2_state = SHOOT_OVERHEAT_1X;
  }
  else
  {
    shoot_eve->shoot1_state = SHOOT_NORMAL;
    shoot_eve->shoot2_state = SHOOT_NORMAL;
  }
}

/**Edited by Y. Z. Yang
 * @Jan 13, 2020: declare the functions to get the friction wheel speed
 * 
 *  
 * 
 * //TODO: update this function when new referee system is installed
 */
float *shoot_get_fric_speed(void)
{

  // TODO: when the encoder is obtained
  float b = 5;
  float *a = &b;
  return a;
}
