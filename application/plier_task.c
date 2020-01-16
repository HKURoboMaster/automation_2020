
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
int8_t rc_js;

ramp_t plier_ramp = RAMP_GEN_DAFAULT;

int16_t i = 0; //test 1

void plier_task(void const *argument)
{
    uint32_t period = osKernelSysTick();
    rc_device_t prc_dev = NULL;
    rc_info_t prc_info = NULL;
    plier_t pplier = NULL;

    pplier = plier_find("plier");
    prc_dev = rc_device_find("can_rc");
    plier_set_offset(pplier, PLIER_OFFSET);
    plier_set_angle(pplier, pplier->ecd_center); //Test 1

    prc_info = rc_device_get_info(prc_dev);

    pplier->step = STEP_1;

    plier_init_state_reset();
    plier_motor_disable(pplier);

    while (1)
    {
        rc_js = rc_device_get_state(prc_dev, RC_S2_UP);
        if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK) //enable condition
        {
            plier_motor_disable(pplier);
        }

        plier_motor_enable(pplier);

        if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK || rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK || rc_device_get_state(prc_dev, RC_S2_UP2MID) == RM_OK) //catch dump throw condition //test 1
        {
            //if (prc_info->kb.bit.G) //catch dump throw condition
            if (1) // test 1
            {//
                if (pplier->step == STEP_1)
                {
                    plier_set_angle(pplier, pplier->ecd_center + 90.0f);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                        HAL_Delay(5000); //test 1
                        pplier->step = STEP_2;
                    }
                }

                else if (pplier->step == STEP_2)
                {
                    plier_set_angle(pplier, pplier->ecd_center + 90.0f);
                    //laser aim
                    if (1) //aimed // test 1
                    {
                        HAL_Delay(5000); //test 1
                        pplier->step = STEP_3;
                    }
                }

                else if (pplier->step == STEP_3)
                {
                    plier_set_angle(pplier, pplier->ecd_center + 180.0f);
                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                        set_linear_actuator(ON);
                        HAL_Delay(5000); //debug
                        pplier->step = STEP_4;
                    }
                }

                else if (pplier->step == STEP_4)
                {
                    plier_set_angle(pplier, pplier->ecd_center);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) <= 5.0f)
                    {
                        HAL_Delay(5000); //debug
                        pplier->step = STEP_5;
                    }
                }

                else if (pplier->step == STEP_5)
                {
                    plier_set_angle(pplier, pplier->ecd_center + 135.0f);

                    if (fabs(pplier->ecd_angle - (pplier->ecd_center + 90.0f)) < 5.0f)
                        set_linear_actuator(OFF);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) < 5.0f)
                    {
                        HAL_Delay(5000); //test 1
                        pplier->step = STEP_6;
                    }
                }

                else if (pplier->step == STEP_6)
                {
                    plier_set_angle(pplier, pplier->ecd_center);

                    if (fabs(pplier->ecd_angle - pplier->target_angle) < 5.0f)
                    {
                        HAL_Delay(5000); //test 1
                        pplier->step = STEP_1;
                    }
                }//
            }

            /*HAL_Delay(5000);
            if (i = 0)
            {
                plier_set_angle(pplier, pplier->ecd_center);
                i = 1;
            }
            else if (i = 1)
            {
                plier_set_angle(pplier, pplier->ecd_center + 90.0f);
                i = 2;
            }
            else if (i = 2)
            {
                plier_set_angle(pplier, pplier->ecd_center + 180.0f);
                i = 0;
            }*/

            if (prc_info->kb.bit.F)
            {
                set_linear_actuator(OFF);
                plier_set_angle(pplier, pplier->ecd_center);
                pplier->step = STEP_1;
            }
        }

        plier_tim_ms = HAL_GetTick() - plier_last_tim;
        plier_last_tim = HAL_GetTick();

        plier_execute(pplier);
        osDelayUntil(&period, 5);
    }
}

void plier_init_state_reset(void)
{
    ramp_init(&plier_ramp, BACK_CENTER_TIME / PLIER_PERIOD);
    plier_auto_init_f = 0;
}
