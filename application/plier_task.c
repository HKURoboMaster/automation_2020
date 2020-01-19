
#include "can.h"
#include "board.h"
#include "sys.h"
#include "plier.h"
#include "dbus.h"
#include "plier_task.h"
#include "referee_system.h"
#include "ramp.h"
#include "offline_check.h"
#include "motor.h"
#include "infantry_cmd.h"

uint32_t plier_tim_ms = 0;
uint32_t plier_last_tim = 0;
uint8_t plier_auto_init_f = 0;
int8_t rc_js; //test 1
enum step step_js;

//int16_t i = 0; //test 1

void plier_task(void const *argument)
{
    uint32_t period = osKernelSysTick();
    rc_device_t prc_dev = NULL;
    rc_info_t prc_info = NULL;
    plier_t pplier = NULL;

    pplier = plier_find("plier");
    prc_dev = rc_device_find("can_rc");
    plier_set_offset(pplier, PLIER_OFFSET);
    //plier_set_angle(pplier, pplier->ecd_center); //Test 1

    prc_info = rc_device_get_info(prc_dev);

    plier_motor_disable(pplier);

    while (1)
    {
        rc_js = rc_device_get_state(prc_dev, RC_S2_UP); //test 1
        if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK) //enable condition
        {
            plier_motor_enable(pplier);
        }
        if  (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
        {
            plier_motor_disable(pplier);
        }
        if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_UP2MID) == RM_OK) //catch dump throw condition //test 1
        {
            //if (prc_info->kb.bit.G) //catch dump throw condition
            if (1) // test 1
            {//
                if (pplier->step == STEP_1)
                {
                    pplier->target_angle = pplier->ecd_center + 90.0f;
                    plier_set_angle(pplier, pplier->target_angle);
                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                         //test 1
                        pplier->step = STEP_2;
                    }
                }

                else if (pplier->step == STEP_2)
                {
                    pplier->target_angle = pplier->ecd_center + 90.0f;
                    plier_set_angle(pplier, pplier->target_angle);
                    //laser aim
                    if (1) //aimed // test 1
                    {
                         //test 1
                        pplier->step = STEP_3;
                    }
                }

                else if (pplier->step == STEP_3)
                {
                    pplier->target_angle = pplier->ecd_center + 180.0f;
                    plier_set_angle(pplier, pplier->target_angle);
                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                        set_linear_actuator(ON);
                         //debug
                        pplier->step = STEP_4;
                    }
                }

                // 可用 counter 延时

                else if (pplier->step == STEP_4)
                {
                    pplier->target_angle = pplier->ecd_center;
                    plier_set_angle(pplier, pplier->target_angle);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                         //debug
                        pplier->step = STEP_5;
                    }
                }

                else if (pplier->step == STEP_5)
                {
                    pplier->target_angle = pplier->ecd_center + 135.0f;
                    plier_set_angle(pplier, pplier->target_angle);
                    
                    if (fabs(pplier->ecd_angle - (pplier->ecd_center + 90.0f)) < 5.0f)
                        set_linear_actuator(OFF);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) < 5.0f)
                    {
                         //test 1
                        pplier->step = STEP_6;
                    }
                }

                else if (pplier->step == STEP_6)
                {
                    pplier->target_angle = pplier->ecd_center;
                    plier_set_angle(pplier, pplier->target_angle);
                    
                    if (fabs(pplier->ecd_angle - pplier->target_angle) < 5.0f)
                    {
                         //test 1
                        pplier->step = STEP_1;
                    }
                }//
            }

            if (prc_info->kb.bit.F)
            {
                set_linear_actuator(OFF);
                plier_set_angle(pplier, pplier->ecd_center);
                pplier->step = STEP_1;
            }
        }

        plier_tim_ms = HAL_GetTick() - plier_last_tim;
        plier_last_tim = HAL_GetTick();

        step_js = pplier->step;
        plier_set_angle(pplier, pplier->target_angle);
        plier_execute(pplier);
        osDelayUntil(&period, 5);

    }
}
 