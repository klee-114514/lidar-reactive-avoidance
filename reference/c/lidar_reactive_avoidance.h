#ifndef LIDAR_REACTIVE_AVOIDANCE_H
#define LIDAR_REACTIVE_AVOIDANCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LRA_SECTOR_COUNT 72u
#define LRA_INVALID_DISTANCE UINT16_MAX

typedef enum {
    LRA_STATE_IDLE = 0,
    LRA_STATE_PIVOT_OUT,
    LRA_STATE_FOLLOW,
    LRA_STATE_FOLLOW_PIVOT,
    LRA_STATE_RETURN_ENTRY,
    LRA_STATE_RETURN,
    LRA_STATE_RETURN_PIVOT,
    LRA_STATE_PIVOT_HOME,
    LRA_STATE_FINISHED,
    LRA_STATE_FAILED
} lra_state_t;

typedef enum {
    LRA_ACTION_STOP = 0,
    LRA_ACTION_HOLD_HEADING,
    LRA_ACTION_PIVOT_TO_HEADING
} lra_action_t;

typedef enum {
    LRA_REASON_NONE = 0,
    LRA_REASON_RUNNING,
    LRA_REASON_OBSTACLE_PASSED,
    LRA_REASON_SAFE_RESUME,
    LRA_REASON_BAD_SCAN,
    LRA_REASON_BAD_HEADING,
    LRA_REASON_NO_FEASIBLE_SIDE,
    LRA_REASON_STALL,
    LRA_REASON_ENVELOPE,
    LRA_REASON_TURN_MISSED,
    LRA_REASON_RETURN_BLOCKED,
    LRA_REASON_EXTRA_TRAVEL,
    LRA_REASON_TIMEOUT
} lra_reason_t;

typedef struct {
    float car_width_mm;
    float car_length_mm;
    float axle_to_front_mm;
    float sensor_to_front_mm;
    float side_margin_mm;
    float target_side_clearance_mm;
    float max_phi_deg;
    float return_tolerance_mm;
    float min_lookahead_mm;
    float side_gain_deg_per_mm;
    float aim_tolerance_deg;
    float return_aim_tolerance_deg;
    float pivot_tolerance_deg;

    uint16_t trigger_mm;
    uint16_t blocked_mm;
    uint16_t min_range_mm;
    uint16_t sweep_horizon_mm;
    uint16_t crush_front_mm;
    uint16_t crush_side_mm;
    uint16_t pass_margin_mm;
    uint16_t return_clearance_mm;
    uint16_t return_entry_front_mm;
    uint16_t return_entry_side_mm;
    uint16_t side_seen_mm;
    uint16_t step_mm;
    uint16_t max_extra_mm;
    uint8_t min_free_sectors;
    uint8_t front_lost_scans;
    uint32_t max_ticks;
} lra_config_t;

typedef struct {
    uint16_t bins[LRA_SECTOR_COUNT];
    int scan_valid;
    int scan_done;
    int front_lost;
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;

    float heading_deg;
    int heading_valid;
    float travel_delta_mm;
    int step_done;
    int stall;
} lra_input_t;

typedef struct {
    lra_state_t state;
    lra_action_t action;
    lra_reason_t reason;
    float target_heading_deg;
    uint16_t step_mm;
    int abort;
} lra_output_t;

typedef struct {
    lra_config_t config;
    lra_state_t state;
    lra_reason_t reason;

    float home_heading_deg;
    float target_heading_deg;
    float phi_deg;
    int side_sign;
    int side_seen;

    float forward_mm;
    float lateral_mm;
    float total_travel_mm;
    float extra_travel_mm;
    float obstacle_near_axle_mm;

    uint8_t front_lost_count;
    uint32_t tick_count;
    int last_step_done;
} lra_controller_t;

void lra_default_config(lra_config_t *config);
void lra_reset(lra_controller_t *controller, const lra_config_t *config);
void lra_update(lra_controller_t *controller,
                const lra_input_t *input,
                lra_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
