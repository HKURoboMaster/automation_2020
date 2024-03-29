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

#include "main.h"
#include "can.h"
#include "board.h"
#include "motor.h"
#include "dbus.h"
#include "detect.h"
#include "test.h"
#include "chassis.h"

#include "chassis_task.h"
#include "timer_task.h"
#include "communicate.h"
#include "infantry_cmd.h"
#include "init.h"

#include "protocol.h"
#include "ulog.h"
#include "param.h"
#include "offline_check.h"
#include "referee_system.h"

#include "engineer.h"
#include "dualmotor.h"
#include "moonrover.h"
#include "upper.h"
#include "sub_engineer.h"
#include "raiser.h"

#include "flipper.h"
#include "seizer.h"
#include "extender.h"

struct chassis chassis;
static struct rc_device rc_dev;

static uint8_t glb_sys_cfg;

extern int ulog_console_backend_init(void);
extern Engineer engg;
extern upper_ctrl upper_controller;

void system_config(void)
{
  glb_sys_cfg = HAL_GPIO_ReadPin(SYS_CFG_Port, SYS_CFG_Pin);
}

uint8_t get_sys_cfg(void)
{
  return glb_sys_cfg;
}

void hw_init(void)
{
  cali_param_init();
  board_config();
  test_init();
  system_config();
  ulog_init();
  ulog_console_backend_init();
  
  referee_param_init();
  usart3_rx_callback_register(referee_uart_rx_data_handle);
  referee_send_data_register(usart3_transmit);

  if(glb_sys_cfg == CHASSIS_APP)
  {
    rc_device_register(&rc_dev, "uart_rc", 0);
    dr16_forword_callback_register(rc_data_forword_by_can);
    chassis_pid_register(&chassis, "chassis", DEVICE_CAN1);
    chassis_disable(&chassis);
		moonrover_pid_register(&engg, "moonrover", DEVICE_CAN1);
		moonrover_disable(&engg);
		//raiser_cascade_register(&engg, "raiser", DEVICE_CAN1);
		//raiser_disable(&engg);
  }
	else if (glb_sys_cfg == UPPER_APP) {
		rc_device_register(&rc_dev, "can_rc", 0);
		//dualmotor_cascade_register(&upper_controller, "dualmotor", DEVICE_CAN1);
		//dualmotor_disable(&upper_controller);
	}

  offline_init();
}

osThreadId timer_task_t;
osThreadId chassis_task_t;
osThreadId communicate_task_t;
osThreadId cmd_task_t;
osThreadId dualmotor_task_t;
osThreadId moonrover_task_t;
osThreadId engineer_task_t;
osThreadId grab_task_t;
osThreadId locomotion_task_t;
osThreadId upper_task_t;
osThreadId subengineer_task_t;
osThreadId raiser_task_t;

osThreadId flipper_task_t;
osThreadId seizer_task_t;
osThreadId extender_task_t;

void task_init(void)
{
  uint8_t app;
  app = get_sys_cfg();

  osThreadDef(TIMER_1MS, timer_task, osPriorityHigh, 0, 512);
  timer_task_t = osThreadCreate(osThread(TIMER_1MS), NULL);

  osThreadDef(COMMUNICATE_TASK, communicate_task, osPriorityHigh, 0, 4096);
  communicate_task_t = osThreadCreate(osThread(COMMUNICATE_TASK), NULL);

  osThreadDef(CMD_TASK, infantry_cmd_task, osPriorityNormal, 0, 4096);
  cmd_task_t = osThreadCreate(osThread(CMD_TASK), NULL);
  
  if (app == CHASSIS_APP)
  {
    osThreadDef(CHASSIS_TASK, chassis_task, osPriorityRealtime, 0, 512);
    chassis_task_t = osThreadCreate(osThread(CHASSIS_TASK), NULL);
		
		osThreadDef(ENGINEER_TASK, engineer_task, osPriorityNormal, 0, 512);
		engineer_task_t = osThreadCreate(osThread(ENGINEER_TASK), NULL);
		
		osThreadDef(MOONROVER_TASK, moonrover_task, osPriorityNormal, 0, 512);
		moonrover_task_t = osThreadCreate(osThread(MOONROVER_TASK), NULL);
		
		osThreadDef(LOCOMOTION_TASK, locomotion_task, osPriorityNormal, 0, 512);
		locomotion_task_t = osThreadCreate(osThread(LOCOMOTION_TASK), NULL);
		
		//osThreadDef(RAISER_TASK, raiser_task, osPriorityNormal, 0, 512);
		//raiser_task_t = osThreadCreate(osThread(RAISER_TASK), NULL);
  }
	else if (app == UPPER_APP)
	{
		//osThreadDef(DUALMOTOR_TASK, dualmotor_task, osPriorityRealtime, 0, 512);
		//dualmotor_task_t = osThreadCreate(osThread(DUALMOTOR_TASK), NULL);
		
		osThreadDef(UPPER_TASK, upper_task, osPriorityNormal, 0, 512);
		upper_task_t = osThreadCreate(osThread(UPPER_TASK), NULL);
		
		osThreadDef(SUBENGINEER_TASK, sub_engineer_task, osPriorityNormal, 0, 512);
		subengineer_task_t = osThreadCreate(osThread(SUBENGINEER_TASK), NULL);
		
		osThreadDef(GRAB_TASK, grab_task, osPriorityNormal, 0, 512);
		grab_task_t = osThreadCreate(osThread(GRAB_TASK), NULL);
		
		osThreadDef(FLIPPER_TASK, flipper_task, osPriorityNormal, 0, 512);
		flipper_task_t = osThreadCreate(osThread(FLIPPER_TASK), NULL);
		
		osThreadDef(SEIZER_TASK, seizer_task, osPriorityNormal, 0, 512);
		seizer_task_t = osThreadCreate(osThread(SEIZER_TASK), NULL);
		
		osThreadDef(EXTENDER_TASK, extender_task, osPriorityNormal, 0, 512);
		extender_task_t = osThreadCreate(osThread(EXTENDER_TASK), NULL);
	}
}
