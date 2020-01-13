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

#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "shoot_task.h"
#include "referee_system.h"
#include "infantry_cmd.h"

int32_t shoot_firction_toggle(shoot_t pshoot, uint8_t toggled);


//Seneory shootting control flag
int32_t shoot1_flag = 0;
int32_t shoot2_flag = 0;

/**Edited by Y.H. Liu
 * @Jun 13, 2019: change the FSM for shooting
 * @Jun 20, 2019: adaption for hero
 * @Jul 3, 2019: retrieve the heat data from refree system
 * @Jul 7, 2019: modify the adaption for hero
 * @Jul 14, 2019: adaption for sentry
 * 
 * Implement the control logic described in Control.md
 */
/**Edited by Y.Z. Yang
 * @Jan 11: adaptation for sentry
 */
enum mouse_cmd{non, click, press};
typedef enum mouse_cmd mouse_cmd_e;
mouse_cmd_e mouse_shoot_control(rc_device_t rc_dev);
static uint16_t get_heat_limit(void);
shoot_event_t shoot_eve = {SHOOT_NORMAL,SHOOT_NORMAL};

void shoot_task(void const *argument)
{
  uint32_t period = osKernelSysTick();     // Potentail bugs in remote control
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  shoot_t pshoot = NULL;
	shoot_t pshoot2 = NULL;
  pshoot = shoot_find("shoot");
	pshoot2 = shoot_find("shoot2");
  prc_dev = rc_device_find("can_rc");

  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
  else
  {
  }

  static uint8_t fric_on = 0; //0 (0X00)for off, 1 (0xff) for on
  shoot_firction_toggle(pshoot, 1);
  shoot_firction_toggle(pshoot2, 1);
  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) == RM_OK)  // 失能模式 
    {
      shoot_disable(pshoot);
      shoot_disable(pshoot2);
		//	shoot_set_fric_speed(pshoot,FRIC_MIN_SPEED,FRIC_MIN_SPEED);
      shoot_firction_toggle(pshoot,1); //1: Close the friction wheel while 0: starts the friction wheel
      shoot_firction_toggle(pshoot2,1); 
      fric_on = 0;    // The frction wheel is always close and 0 here!
    }
    



    if (rc_device_get_state(prc_dev, RC_S2_MID)== RM_OK){  // 手动模式
      shoot_enable(pshoot2);
      shoot_enable(pshoot); 
      /*------ implement the keyboard and remote control controlling over friction wheel ------*/
      if (rc_device_get_state(prc_dev, RC_S1_MID2UP) == RM_OK)  // Close or open the friction wheel
      {
        shoot_firction_toggle(pshoot, fric_on);
        shoot_firction_toggle(pshoot2, fric_on);
        fric_on = ~fric_on;
      }

      if(prc_dev->rc_info.kb.bit.Z )  // Z+F Close the frinction wheel   F open the frction wheel
      {
        if(prc_dev->rc_info.kb.bit.F && !prc_dev->last_rc_info.kb.bit.F) //turn off the fric regardless the situation
        {
          shoot_firction_toggle(pshoot,1);
          shoot_firction_toggle(pshoot2,1); 
		  		fric_on = 0;//set fric_on to be 0x00 i.e. off
        }
      }
      else
      {
        if(prc_dev->rc_info.kb.bit.F && !prc_dev->last_rc_info.kb.bit.F) //turn on the fric regardless the situation
        {
          shoot_firction_toggle(pshoot,0); //assume that currently the fric is off
          shoot_firction_toggle(pshoot2,0); //Leo assume that currently the fric is off
	  			fric_on = 1; //set fric_on to be 0xFF i.e. on
        }
      }
    
      /*------ implement the function of a trigger ------*/
      uint16_t * shooter_heat_ptr = shooter_heat_get_via_can();  // TODO: read the power seperatelhy from the referee system
      uint16_t heatLimit = get_heat_limit();   // The heat limit for sentery 480

      if (shooter_heat_ptr[0]< heatLimit && rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK && fric_on) //not in disabled mode
      {
        if (rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK || mouse_shoot_control(prc_dev)==press)
        {
          shoot_set_fric_speed(pshoot, FRIC_CON_SPEED, FRIC_CON_SPEED);
          shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);				
        }
        else if ((rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300)
              || mouse_shoot_control(prc_dev)==click)
        {
          shoot_set_fric_speed(pshoot, FRIC_MAX_SPEED, FRIC_MAX_SPEED);
          shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
        }
        else
        {
          shoot_set_fric_speed(pshoot, FRIC_MAX_SPEED, FRIC_MAX_SPEED);
          shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);
        }
      }

      if (shooter_heat_ptr[0]< heatLimit && rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK && fric_on) //not in disabled mode
      {
        if (rc_device_get_state(prc_dev, RC_WHEEL_UP) == RM_OK || mouse_shoot_control(prc_dev)==press)
        {
          shoot_set_fric_speed(pshoot2, FRIC_CON_SPEED, FRIC_CON_SPEED);
          shoot_set_cmd(pshoot2, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);				
        }
        else if ((rc_device_get_state(prc_dev, RC_WHEEL_DOWN) == RM_OK && prc_dev->last_rc_info.wheel > -300)
              || mouse_shoot_control(prc_dev)==click)
        {
          shoot_set_fric_speed(pshoot2, FRIC_MAX_SPEED, FRIC_MAX_SPEED);
          shoot_set_cmd(pshoot2, SHOOT_ONCE_CMD, 1);
        }
        else
        {
          shoot_set_fric_speed(pshoot2, FRIC_MAX_SPEED, FRIC_MAX_SPEED);
          shoot_set_cmd(pshoot2, SHOOT_STOP_CMD, 0);
        }
      }
    }


    if (rc_device_get_state(prc_dev, RC_S2_UP)== RM_OK){   //自动模式
      shoot_event_update(&shoot_eve);
      shoot_enable(pshoot2);
      shoot_enable(pshoot); 
      if (fric_on == 0){
        shoot_firction_toggle(pshoot, fric_on);
        shoot_firction_toggle(pshoot2,fric_on);
        fric_on = 1;
      }
      if (shoot1_flag){
        shoot_set_fric_speed(pshoot, FRIC_CON_SPEED, FRIC_CON_SPEED);
        shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);
      }
      else{
        shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);
      }
      if (shoot2_flag){
        shoot_set_fric_speed(pshoot2, FRIC_CON_SPEED, FRIC_CON_SPEED);
        shoot_set_cmd(pshoot2, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);
      } 
      else{
        shoot_set_cmd(pshoot2, SHOOT_STOP_CMD, 0);
      }
    }

    shoot_execute(pshoot);
		shoot_execute(pshoot2);
    osDelayUntil(&period,5);
  }
}

/**Modified by Y.H. Liu
 * @Jun 13, 2019: to support the control by keyboards
 * @Jul 30, 2019: use strlen to determine whether the shooter is shoot2
 * 
 * Switch on/off the friction wheel
 * @param toggled 0----not turned on yet, so turn the devices on
 *                1----turned on already, so turn off them now
 */
int32_t shoot_firction_toggle(shoot_t pshoot, uint8_t toggled)
{
  if (toggled)
  {
    shoot_set_fric_speed(pshoot, FRIC_STOP_SPEED, FRIC_STOP_SPEED);
    turn_off_laser();
		shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);	
  }
  else
  {
    shoot_set_fric_speed(pshoot, FRIC_MAX_SPEED, FRIC_MAX_SPEED);
    turn_on_laser();
    shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, CONTIN_BULLET_NUM);	    
  }
  return 0;
}

/**Added by Y.H. Liu
 * @Jun 13, 2019: Declare the funtion, while leaving the defination empty
 * 
 * Send signals to the servo via interfaces to open/close the magazine lid
 * @param toggled 0----not turned on yet, so turn the devices on
 *                1----turned on already, so turn off them now
 */
// int32_t shoot_lid_toggle(shoot_t pshoot, uint8_t toggled)
// {
//   if(toggled)
//   {
//     return close_lid();
//   }
//   else
//   {
// 		return open_lid();
//   }
// }

/**Added by Y.H. Liu
 * @Jun 13, 2019: Define the function
 * 
 * Detect the left key of mouse to instruct the shoot
 */
mouse_cmd_e mouse_shoot_control(rc_device_t rc_dev)
{
  static mouse_cmd_e ret_val = non;
  static int32_t pressed_count = 0;
  uint8_t lKey = rc_dev->rc_info.mouse.l;

  if(lKey)
  {
    switch(ret_val)
    {
      case non:
        ret_val = click;
        pressed_count = 1;
        break;
      case click:
        pressed_count += 1;
        ret_val = pressed_count>CONTINUE_SHOOT_TH ? press : click;
        break;
      default:
        break;
    }
  }
  else
  {
    pressed_count = 0;
    ret_val = non;
  }
  return (pressed_count>=10 && ret_val==click) ? non : ret_val;
} 

/**Addd by Y. H. Liu
 * @Jul 3, 2019: declare the function and create the defination framework
 * @Jul 14, 2019: change for sentry
 * 
 * Calculate the heat limit
 */
static uint16_t get_heat_limit(void)
{
  return 480u; 
}
