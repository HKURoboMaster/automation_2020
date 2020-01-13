#include "can.h"
#include "tong.h"
#include "dbus.h"
#include "tong_task.h"
#include "offline_check.h"
#include "param.h"

void tong_task(void const *argument)
{
    tong_t ptong;
    rc_device_t prc_dev;
    rc_info_t prc_info;
    uint32_t period;

    ptong = tong_find("tong");
    prc_dev = rc_device_find("can_rc");
    prc_info = rc_device_get_info(prc_dev);
    period = osKernelSysTick();


    tong_set_offset(ptong, TONG_OFFSET);
    ptong->step = 1;

    if (prc_dev != NULL)
    {
        prc_info = rc_device_get_info(prc_dev);
    }

    while(1)
    {
        if(rc_device_get_state(prc_dev, RC_S2_DOWN2MID) == RM_OK)
        {
            tong_motor_enable(ptong);
        }
        if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK
    ||  rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK || rc_device_get_state(prc_dev,RC_S2_UP2MID == RM_OK))
        {

            

            if (prc_info->kb.bit.G == 1)
            {
                if (ptong->step == 1)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center + 90.0);
                    if (abs(ptong->tong_angle - 90.0) <= 5.0 )
                        ptong->step += 1;
                }

                if (ptong->step == 2)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center + 90.0);

                    if ()
                        ptong->step += 1;
                }

                if (ptong->step == 3)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center + 180.0);
                    if (abs(ptong->tong_angle - 180.0) <= 5.0)
                        ptong->step += 1;
                }

                if (ptong->step == 4)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center + 180.0);
                    linear_actuator_command(ON);
                    HAL_Delay(1000);
                    if (abs(ptong->tong_angle - 180.0) <= 5.0)
                        ptong->step += 1;

                if (ptong->step == 5)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center);
                    if (abs(ptong->tong_angle) <= 5.0)
                        ptong->step += 1;
                }

                if (ptong->step == 6)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center + 135.0);
                    if (abs(ptong->tong_angle - 90.0) < 5.0)
                        linear_actuator_command(OFF);
                    if (abs(ptong->tong_angle - 135.0) < 5.0)
                        ptong->step += 1;
                }

                if (ptong->step == 7)
                {
                    tong_set_angle(ptong, ptong->tong_ecd_center);
                    if (abs(ptong->tong_angle) < 5.0)
                        ptong->step = 1;

                }

            }

            if (prc_info->kb.bit.X == 1)
            {
                tong_set_angle(ptong, ptong->tong_ecd_center);
                linear_actuator_command(OFF);
            }

            tong_execute(ptong);
            osDelayUntil(&period, 5);
        }
        
    }


}



