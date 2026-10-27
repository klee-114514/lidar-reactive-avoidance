#ifndef LRA_VEHICLE_CONFIG_EXAMPLE_H
#define LRA_VEHICLE_CONFIG_EXAMPLE_H

#include "lidar_reactive_avoidance.h"

/*
 * 复制本文件为 lra_vehicle_config.h 后填写 TODO 值。
 * TODO 常量故意未定义，避免未标定的参数被误用到真实车辆。
 */

#define LRA_CAR_WIDTH_MM          TODO_CAR_WIDTH_MM          /* 实测车体总宽：含轮子和保险杠，单位 mm */
#define LRA_CAR_LENGTH_MM         TODO_CAR_LENGTH_MM         /* 实测车体总长：含前后保险杠，单位 mm */
#define LRA_AXLE_TO_FRONT_MM      TODO_AXLE_TO_FRONT_MM      /* 实测后轮轴到车头前缘的距离，单位 mm */
#define LRA_SENSOR_TO_FRONT_MM    TODO_SENSOR_TO_FRONT_MM    /* 实测雷达光心到车头前缘的距离，单位 mm */
#define LRA_SIDE_MARGIN_MM        TODO_SIDE_MARGIN_MM        /* 侧向安全余量：车体到障碍的最小允许距离，单位 mm */
#define LRA_TARGET_SIDE_MM        TODO_TARGET_SIDE_MM        /* 跟随障碍时希望保持的侧向距离，单位 mm */
#define LRA_MAX_PHI_DEG           TODO_MAX_PHI_DEG           /* 允许使用的最大绕行转角，单位度 */
#define LRA_RETURN_TOLERANCE_MM   TODO_RETURN_TOLERANCE_MM   /* 回线完成允许保留的横向误差，单位 mm */
#define LRA_MIN_LOOKAHEAD_MM      TODO_MIN_LOOKAHEAD_MM      /* 计算回线角时使用的最小前视距离，单位 mm */
#define LRA_SIDE_GAIN             TODO_SIDE_GAIN             /* 侧距误差到转角修正量的比例增益，单位 度/mm */
#define LRA_AIM_TOLERANCE_DEG     TODO_AIM_TOLERANCE_DEG     /* 跟随阶段允许的航向偏差，单位度 */
#define LRA_TRIGGER_MM            TODO_TRIGGER_MM            /* 正前障碍触发避障的距离，单位 mm */
#define LRA_BLOCKED_MM            TODO_BLOCKED_MM            /* 直方图中判定扇区被阻挡的距离，单位 mm */
#define LRA_MIN_RANGE_MM          TODO_MIN_RANGE_MM          /* 雷达最小有效距离：用于过滤盲区，单位 mm */
#define LRA_SWEEP_HORIZON_MM      TODO_SWEEP_HORIZON_MM      /* 车体扫掠检查关注的最大前方距离，单位 mm */
#define LRA_CRUSH_FRONT_MM        TODO_CRUSH_FRONT_MM        /* 正前碰撞保护距离：小于该值立即停车，单位 mm */
#define LRA_CRUSH_SIDE_MM         TODO_CRUSH_SIDE_MM         /* 侧向碰撞保护距离：小于该值立即停车，单位 mm */
#define LRA_PASS_MARGIN_MM        TODO_PASS_MARGIN_MM        /* 判断整车越过障碍时额外保留的纵向距离，单位 mm */
#define LRA_RETURN_CLEARANCE_MM   TODO_RETURN_CLEARANCE_MM   /* 允许开始回线所需的障碍侧净空，单位 mm */
#define LRA_RETURN_ENTRY_FRONT_MM TODO_RETURN_ENTRY_FRONT_MM /* 进入回线前要求的最小正前净空，单位 mm */
#define LRA_RETURN_ENTRY_SIDE_MM  TODO_RETURN_ENTRY_SIDE_MM  /* 进入回线前要求的最小侧向净空，单位 mm */
#define LRA_STEP_MM               TODO_STEP_MM               /* 边界跟随和回线阶段的单步运动距离，单位 mm */
#define LRA_MAX_EXTRA_MM          TODO_MAX_EXTRA_MM          /* 绕障允许产生的最大额外路程，单位 mm */
#define LRA_MIN_FREE_SECTORS      TODO_MIN_FREE_SECTORS      /* 能通过的连续自由扇区最低数量，单位 格 */
#define LRA_FRONT_LOST_SCANS      TODO_FRONT_LOST_SCANS      /* 正前障碍连续消失多少圈后才允许回线，单位 圈 */
#define LRA_MAX_TICKS             TODO_MAX_TICKS             /* 一次绕障允许运行的最大控制节拍数，单位 tick */

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
