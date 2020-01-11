#include "chassis_calc.h"
#include <stdlib.h>

chassis_movement_t movement = {1, 0, 0}; // current movement of chassis
duration_settings_t sduration = {500, 2500}; // movement duration settings
bounded_movement_settings_t bmove = {0, 0}; // bounded movement settings
int chassis_loc = 0; // [for rail: __/--] 0: __ 1: / 2: --
int curr_loc_delta = 0, last_loc_delta = 0;
float chassis_position = 0;
float accumulated_distance = 0;
int acc_dis_js = 0;


/**
 * Jerry 14 Jul
 * @brief Calculate and output chassis random movement speed.
 * @param speed: constant speed of the state
 * @retval chassis output
 */
float chassis_random_movement(chassis_t pchassis, float speed) 
{
    uint32_t now = HAL_GetTick();
    struct chassis_info info;
    chassis_get_info(pchassis, &info);
    chassis_position = info.position_y_mm;
    if (now - movement.start_time > movement.duration) 
    {
        generate_random_movement(); // current movement expires, generate a new one
    }
    float output = movement.spd_ind * speed;
    if (bmove.activated) 
    {
        if (chassis_position > bmove.right_position || chassis_position < bmove.left_position) 
        {
            generate_random_movement();
        }
    }
    acc_dis_js = accumulated_distance;
    return output;
}

/**
 * Yemi 9 Jan
 * @brief normal state movement
 * reverse speed direction.
 */
float chassis_patrol_movement(chassis_t pchassis, float speed)
{
    uint32_t now = HAL_GetTick();
    struct chassis_info info;
    chassis_get_info(pchassis, &info);
    chassis_position = info.position_y_mm;
    if (now - movement.start_time > movement.duration)
    {
        generate_patrol_movement();
    }
    float output = movement.spd_ind * speed;
    if (bmove.activated) 
    {
        if (chassis_position > bmove.right_position || chassis_position < bmove.left_position) 
        {
            generate_patrol_movement();
        }
    }
    acc_dis_js = accumulated_distance;
    return output;
}
/**
 * Jerry 14 Jul
 * @brief Generate new random movement object.
 * Randomly choose duration between floor and ceiling,
 * reverse speed direction.
 */
void generate_random_movement(void) 
{
    movement.start_time = HAL_GetTick();
    srand(movement.start_time);
    movement.duration = sduration.floor + rand() % (sduration.ceiling - sduration.floor);
    movement.spd_ind = -movement.spd_ind;
}
/**
 * Yemi 11 Jan
 * @brief Generate new patrol movement object.
 * reverse speed direction.
 */
void generate_patrol_movement(void) 
{
    movement.start_time = HAL_GetTick();
    srand(movement.start_time);
    movement.duration = sduration.ceiling - sduration.floor;
    movement.spd_ind = -movement.spd_ind;
}
/**
 * Jerry 14 Jul
 * @brief Shorthand solution for Set duration.
 * @param if you do not wish to change a value, set it to -1
 */
void set_duration(int new_duration_floor, int new_duration_ceiling) 
{
    sduration.ceiling = new_duration_ceiling > 0 ? new_duration_ceiling : sduration.ceiling;
    sduration.floor = new_duration_floor > 0 ? new_duration_floor : sduration.floor;
}

/**
 * Yemi 9 Jan
 * @brief Set the chassis state to be one of the three states.
 * Example: set_state(&state, MEDIUM_MODE);
 */
void set_state(chassis_state_t * state, chassis_state_name_t dest_state) 
{
  state->state_name = dest_state;
  switch (dest_state) 
  {
    case LOW_MODE:
      state->constant_spd = LOW_CONSTANT_SPEED;
      set_duration(500, 2500);
      break;
    case MEDIUM_MODE:
      state->constant_spd = MEDIUM_CONSTANT_SPEED;
      set_duration(400, 1500);
      break;
    case BOOST_MODE:
      state->constant_spd = HIGH_CONSTANT_SPEED;
      set_duration(300, 1000);
  }
}

/**
 * Jerry 10 Jul
 * @brief Get the constant speed under current state.
 */
float get_spd(const chassis_state_t * state) 
{
  return state->constant_spd;
}
