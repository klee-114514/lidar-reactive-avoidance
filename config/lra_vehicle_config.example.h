#ifndef LRA_VEHICLE_CONFIG_EXAMPLE_H
#define LRA_VEHICLE_CONFIG_EXAMPLE_H

#include "lidar_reactive_avoidance.h"

/*
 * 复制本文件为 lra_vehicle_config.h 后填写 TODO 值。
 * TODO 常量故意未定义，避免未标定的参数被误用到真实车辆。
 */

#define LRA_CAR_WIDTH_MM          TODO_CAR_WIDTH_MM
#define LRA_CAR_LENGTH_MM         TODO_CAR_LENGTH_MM
#define LRA_AXLE_TO_FRONT_MM      TODO_AXLE_TO_FRONT_MM
#define LRA_SENSOR_TO_FRONT_MM    TODO_SENSOR_TO_FRONT_MM
#define LRA_SIDE_MARGIN_MM        TODO_SIDE_MARGIN_MM
#define LRA_TARGET_SIDE_MM        TODO_TARGET_SIDE_MM
#define LRA_MAX_PHI_DEG           TODO_MAX_PHI_DEG
#define LRA_RETURN_TOLERANCE_MM   TODO_RETURN_TOLERANCE_MM
#define LRA_MIN_LOOKAHEAD_MM      TODO_MIN_LOOKAHEAD_MM
#define LRA_SIDE_GAIN             TODO_SIDE_GAIN
#define LRA_AIM_TOLERANCE_DEG     TODO_AIM_TOLERANCE_DEG
#define LRA_TRIGGER_MM            TODO_TRIGGER_MM
#define LRA_BLOCKED_MM            TODO_BLOCKED_MM
#define LRA_MIN_RANGE_MM          TODO_MIN_RANGE_MM
#define LRA_SWEEP_HORIZON_MM      TODO_SWEEP_HORIZON_MM
#define LRA_CRUSH_FRONT_MM        TODO_CRUSH_FRONT_MM
#define LRA_CRUSH_SIDE_MM         TODO_CRUSH_SIDE_MM
#define LRA_PASS_MARGIN_MM        TODO_PASS_MARGIN_MM
#define LRA_RETURN_CLEARANCE_MM   TODO_RETURN_CLEARANCE_MM
#define LRA_RETURN_ENTRY_FRONT_MM TODO_RETURN_ENTRY_FRONT_MM
#define LRA_RETURN_ENTRY_SIDE_MM  TODO_RETURN_ENTRY_SIDE_MM
#define LRA_STEP_MM               TODO_STEP_MM
#define LRA_MAX_EXTRA_MM          TODO_MAX_EXTRA_MM
#define LRA_MIN_FREE_SECTORS      TODO_MIN_FREE_SECTORS
#define LRA_FRONT_LOST_SCANS      TODO_FRONT_LOST_SCANS
#define LRA_MAX_TICKS             TODO_MAX_TICKS

static inline void lra_load_vehicle_config(lra_config_t *config)
{
    lra_default_config(config);

    config->car_width_mm = LRA_CAR_WIDTH_MM;
    config->car_length_mm = LRA_CAR_LENGTH_MM;
    config->axle_to_front_mm = LRA_AXLE_TO_FRONT_MM;
    config->sensor_to_front_mm = LRA_SENSOR_TO_FRONT_MM;
    config->side_margin_mm = LRA_SIDE_MARGIN_MM;
    config->target_side_clearance_mm = LRA_TARGET_SIDE_MM;
    config->max_phi_deg = LRA_MAX_PHI_DEG;
    config->return_tolerance_mm = LRA_RETURN_TOLERANCE_MM;
    config->min_lookahead_mm = LRA_MIN_LOOKAHEAD_MM;
    config->side_gain_deg_per_mm = LRA_SIDE_GAIN;
    config->aim_tolerance_deg = LRA_AIM_TOLERANCE_DEG;
    config->trigger_mm = LRA_TRIGGER_MM;
    config->blocked_mm = LRA_BLOCKED_MM;
    config->min_range_mm = LRA_MIN_RANGE_MM;
    config->sweep_horizon_mm = LRA_SWEEP_HORIZON_MM;
    config->crush_front_mm = LRA_CRUSH_FRONT_MM;
    config->crush_side_mm = LRA_CRUSH_SIDE_MM;
    config->pass_margin_mm = LRA_PASS_MARGIN_MM;
    config->return_clearance_mm = LRA_RETURN_CLEARANCE_MM;
    config->return_entry_front_mm = LRA_RETURN_ENTRY_FRONT_MM;
    config->return_entry_side_mm = LRA_RETURN_ENTRY_SIDE_MM;
    config->step_mm = LRA_STEP_MM;
    config->max_extra_mm = LRA_MAX_EXTRA_MM;
    config->min_free_sectors = LRA_MIN_FREE_SECTORS;
    config->front_lost_scans = LRA_FRONT_LOST_SCANS;
    config->max_ticks = LRA_MAX_TICKS;
}

#endif
