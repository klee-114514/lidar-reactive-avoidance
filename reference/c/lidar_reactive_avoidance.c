#include "lidar_reactive_avoidance.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define LRA_PI 3.14159265358979323846f
#define LRA_DEG_TO_RAD (LRA_PI / 180.0f)
#define LRA_RAD_TO_DEG (180.0f / LRA_PI)

static float lra_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static float lra_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float lra_wrap_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float lra_angle_error(float actual, float target)
{
    return lra_absf(lra_wrap_deg(actual - target));
}

static uint16_t lra_side_clearance(const lra_controller_t *controller,
                                   const lra_input_t *input)
{
    /* 向左绕行时，障碍保持在车辆右侧。 */
    return controller->side_sign > 0 ? input->right_mm : input->left_mm;
}

static int lra_valid_distance(const lra_controller_t *controller,
                              uint16_t distance)
{
    return distance != LRA_INVALID_DISTANCE &&
           distance >= controller->config.min_range_mm;
}

static int lra_candidate_feasible(const lra_controller_t *controller,
                                  const lra_input_t *input,
                                  float candidate_deg)
{
    const lra_config_t *config = &controller->config;
    const float half_width =
        config->car_width_mm * 0.5f + config->side_margin_mm;
    const float axle_to_sensor =
        config->axle_to_front_mm - config->sensor_to_front_mm;
    const float rear_overhang =
        config->car_length_mm > config->axle_to_front_mm
            ? config->car_length_mm - config->axle_to_front_mm
            : 0.0f;
    const float front_limit =
        config->axle_to_front_mm + (float)config->sweep_horizon_mm;
    const float pivot_radius =
        sqrtf((config->car_length_mm * 0.5f) *
              (config->car_length_mm * 0.5f) +
              (config->car_width_mm * 0.5f) *
              (config->car_width_mm * 0.5f)) +
        config->side_margin_mm;
    const float candidate_rad = candidate_deg * LRA_DEG_TO_RAD;
    const float candidate_cos = cosf(candidate_rad);
    const float candidate_sin = sinf(candidate_rad);
    uint32_t i;

    for (i = 0; i < LRA_SECTOR_COUNT; ++i) {
        const float distance = (float)input->bins[i];
        float angle_deg;
        float angle_rad;
        float x;
        float y;
        float u;
        float v;
        float x_axle;

        if (!lra_valid_distance(controller, input->bins[i])) {
            continue;
        }
        if (distance > (float)config->sweep_horizon_mm &&
            distance > pivot_radius) {
            continue;
        }

        angle_deg = (float)i * 5.0f;
        if (angle_deg > 180.0f) {
            angle_deg -= 360.0f;
        }
        angle_rad = angle_deg * LRA_DEG_TO_RAD;
        x = distance * cosf(angle_rad);
        y = distance * sinf(angle_rad);

        if (sqrtf(x * x + y * y) <= pivot_radius) {
            return 0;
        }

        x_axle = x + axle_to_sensor;
        u = x_axle * candidate_cos + y * candidate_sin;
        v = -x_axle * candidate_sin + y * candidate_cos;

        if (u >= -rear_overhang && u <= front_limit &&
            lra_absf(v) <= half_width) {
            return 0;
        }
    }
    return 1;
}

static int lra_find_gap(const lra_controller_t *controller,
                        const lra_input_t *input,
                        int *preferred_side)
{
    const lra_config_t *config = &controller->config;
    int run_start = 0;
    int run_len = 0;
    int best_len = 0;
    int best_center = 0;
    int best_score = 1000;
    int relative;

    for (relative = -18; relative <= 18; ++relative) {
        const uint32_t index = (uint32_t)(relative + 72) % LRA_SECTOR_COUNT;
        const uint16_t distance = input->bins[index];
        const int free = distance == LRA_INVALID_DISTANCE ||
                         distance >= config->blocked_mm;

        if (free) {
            if (run_len == 0) {
                run_start = relative;
            }
            run_len++;
        }
        if ((!free || relative == 18) && run_len > 0) {
            const int center = run_start + (run_len - 1) / 2;
            const int score = center < 0 ? -center : center;

            if (run_len >= (int)config->min_free_sectors &&
                (score < best_score ||
                 (score == best_score && run_len > best_len))) {
                best_len = run_len;
                best_center = center;
                best_score = score;
            }
            run_len = 0;
        }
    }

    if (best_len < (int)config->min_free_sectors) {
        return 0;
    }
    *preferred_side = best_center >= 0 ? 1 : -1;
    return 1;
}

static int lra_find_direction(const lra_controller_t *controller,
                              const lra_input_t *input,
                              int *side_sign,
                              float *phi_deg)
{
    const lra_config_t *config = &controller->config;
    const float step = 5.0f;
    float best_left = config->max_phi_deg + step;
    float best_right = config->max_phi_deg + step;
    int preferred_side = 1;
    float phi;

    if (!lra_find_gap(controller, input, &preferred_side)) {
        return 0;
    }

    for (phi = 0.0f; phi <= config->max_phi_deg; phi += step) {
        if (lra_candidate_feasible(controller, input, phi)) {
            best_left = phi + step;
            break;
        }
    }
    for (phi = 0.0f; phi <= config->max_phi_deg; phi += step) {
        if (lra_candidate_feasible(controller, input, -phi)) {
            best_right = phi + step;
            break;
        }
    }

    if (best_left > config->max_phi_deg &&
        best_right > config->max_phi_deg) {
        return 0;
    }
    if (best_left < best_right ||
        (best_left == best_right && preferred_side > 0)) {
        *side_sign = 1;
        *phi_deg = lra_clampf(best_left, 0.0f, config->max_phi_deg);
    } else {
        *side_sign = -1;
        *phi_deg = lra_clampf(best_right, 0.0f, config->max_phi_deg);
    }
    return 1;
}

static void lra_set_stop(lra_output_t *output)
{
    output->action = LRA_ACTION_STOP;
    output->target_heading_deg = 0.0f;
    output->step_mm = 0;
}

static void lra_fail(lra_controller_t *controller,
                     lra_reason_t reason,
                     lra_output_t *output)
{
    controller->state = LRA_STATE_FAILED;
    controller->reason = reason;
    output->state = controller->state;
    output->reason = reason;
    output->abort = 1;
    lra_set_stop(output);
}

static void lra_start_avoidance(lra_controller_t *controller,
                                const lra_input_t *input,
                                lra_output_t *output)
{
    int side_sign = 1;
    float phi_deg = 0.0f;

    if (!input->scan_valid) {
        lra_fail(controller, LRA_REASON_BAD_SCAN, output);
        return;
    }
    if (!input->heading_valid) {
        lra_fail(controller, LRA_REASON_BAD_HEADING, output);
        return;
    }
    if (!lra_find_direction(controller, input, &side_sign, &phi_deg)) {
        lra_fail(controller, LRA_REASON_NO_FEASIBLE_SIDE, output);
        return;
    }

    controller->state = LRA_STATE_PIVOT_OUT;
    controller->reason = LRA_REASON_RUNNING;
    controller->home_heading_deg = input->heading_deg;
    controller->side_sign = side_sign;
    controller->phi_deg = phi_deg;
    controller->target_heading_deg =
        lra_wrap_deg(controller->home_heading_deg +
                     (float)side_sign * phi_deg);
    controller->side_seen = 0;
    controller->forward_mm = 0.0f;
    controller->lateral_mm = 0.0f;
    controller->total_travel_mm = 0.0f;
    controller->extra_travel_mm = 0.0f;
    controller->obstacle_near_axle_mm =
        (float)input->front_mm + controller->config.axle_to_front_mm;
    controller->front_lost_count = 0;
    controller->tick_count = 0;

    output->state = controller->state;
    output->reason = controller->reason;
    output->action = LRA_ACTION_PIVOT_TO_HEADING;
    output->target_heading_deg = controller->target_heading_deg;
    output->step_mm = 0;
    output->abort = 0;
}

static void lra_update_travel(lra_controller_t *controller,
                              const lra_input_t *input)
{
    const float heading_offset =
        lra_wrap_deg(input->heading_deg - controller->home_heading_deg);
    const float heading_rad = heading_offset * LRA_DEG_TO_RAD;
    const float travel = input->travel_delta_mm > 0.0f
                             ? input->travel_delta_mm
                             : 0.0f;
    const float forward_component = travel * cosf(heading_rad);

    controller->forward_mm += forward_component;
    controller->lateral_mm += travel * sinf(heading_rad);
    controller->total_travel_mm += travel;
    if (forward_component < travel) {
        controller->extra_travel_mm += travel - forward_component;
    }
}

static int lra_front_is_clear(const lra_controller_t *controller,
                              const lra_input_t *input,
                              uint16_t threshold_mm)
{
    (void)controller;
    return input->front_mm != LRA_INVALID_DISTANCE &&
           input->front_mm > threshold_mm;
}

static void lra_prepare_follow(lra_controller_t *controller,
                               const lra_input_t *input,
                               lra_output_t *output)
{
    const lra_config_t *config = &controller->config;
    const uint16_t side_clearance = lra_side_clearance(controller, input);

    if (side_clearance != LRA_INVALID_DISTANCE) {
        const float correction =
            config->side_gain_deg_per_mm *
            (config->target_side_clearance_mm - (float)side_clearance);
        float candidate_phi = controller->phi_deg + correction;

        if (side_clearance <= config->side_seen_mm) {
            controller->side_seen = 1;
        }
        if (!controller->side_seen && candidate_phi < controller->phi_deg) {
            candidate_phi = controller->phi_deg;
        }
        controller->phi_deg =
            lra_clampf(candidate_phi, 0.0f, config->max_phi_deg);
    }

    controller->target_heading_deg =
        lra_wrap_deg(controller->home_heading_deg +
                     (float)controller->side_sign * controller->phi_deg);

    output->state = controller->state;
    output->reason = controller->reason;
    output->abort = 0;
    output->step_mm = config->step_mm;
    output->target_heading_deg = controller->target_heading_deg;

    if (lra_angle_error(input->heading_deg,
                        controller->target_heading_deg) >
        config->aim_tolerance_deg) {
        controller->state = LRA_STATE_FOLLOW_PIVOT;
        output->state = controller->state;
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->step_mm = 0;
    } else {
        output->action = LRA_ACTION_HOLD_HEADING;
    }
}

static void lra_prepare_return(lra_controller_t *controller,
                               const lra_input_t *input,
                               lra_output_t *output)
{
    const lra_config_t *config = &controller->config;
    const float lat_abs = lra_absf(controller->lateral_mm);
    const float lookahead =
        controller->forward_mm > config->min_lookahead_mm
            ? controller->forward_mm
            : config->min_lookahead_mm;
    const float ratio =
        lookahead > 0.0f ? lat_abs / lookahead : 1.0f;
    float return_angle;

    if (ratio <= 0.5f) {
        return_angle = 30.0f;
    } else if (ratio <= 0.9f) {
        return_angle = 34.0f;
    } else {
        return_angle = 38.0f;
    }

    controller->target_heading_deg =
        lra_wrap_deg(controller->home_heading_deg -
                     (float)controller->side_sign * return_angle);
    output->state = controller->state;
    output->reason = controller->reason;
    output->abort = 0;
    output->target_heading_deg = controller->target_heading_deg;
    output->step_mm = config->step_mm;

    if (lra_angle_error(input->heading_deg,
                        controller->target_heading_deg) >
        config->return_aim_tolerance_deg) {
        controller->state = LRA_STATE_RETURN_PIVOT;
        output->state = controller->state;
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->step_mm = 0;
    } else {
        output->action = LRA_ACTION_HOLD_HEADING;
    }
}

void lra_default_config(lra_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->car_width_mm = 220.0f;
    config->car_length_mm = 240.0f;
    config->axle_to_front_mm = 175.0f;
    config->sensor_to_front_mm = 150.0f;
    config->side_margin_mm = 90.0f;
    config->target_side_clearance_mm = 200.0f;
    config->max_phi_deg = 60.0f;
    config->return_tolerance_mm = 15.0f;
    config->min_lookahead_mm = 300.0f;
    config->side_gain_deg_per_mm = 0.1f;
    config->aim_tolerance_deg = 15.0f;
    config->return_aim_tolerance_deg = 5.0f;
    config->pivot_tolerance_deg = 4.0f;

    config->trigger_mm = 450u;
    config->blocked_mm = 600u;
    config->min_range_mm = 150u;
    config->sweep_horizon_mm = 1500u;
    config->crush_front_mm = 200u;
    config->crush_side_mm = 155u;
    config->pass_margin_mm = 85u;
    config->return_clearance_mm = 300u;
    config->return_entry_front_mm = 400u;
    config->return_entry_side_mm = 260u;
    config->side_seen_mm = 600u;
    config->step_mm = 80u;
    config->max_extra_mm = 800u;
    config->min_free_sectors = 8u;
    config->front_lost_scans = 3u;
    config->max_ticks = 3200u;
}

void lra_reset(lra_controller_t *controller, const lra_config_t *config)
{
    if (controller == NULL) {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    if (config != NULL) {
        controller->config = *config;
    } else {
        lra_default_config(&controller->config);
    }
    controller->state = LRA_STATE_IDLE;
    controller->reason = LRA_REASON_NONE;
    controller->side_sign = 1;
}

void lra_update(lra_controller_t *controller,
                const lra_input_t *input,
                lra_output_t *output)
{
    const lra_config_t *config;
    const int step_event =
        input->step_done && !controller->last_step_done;
    const uint16_t side_clearance = lra_side_clearance(controller, input);

    memset(output, 0, sizeof(*output));
    output->state = controller->state;
    output->reason = controller->reason;
    lra_set_stop(output);

    controller->last_step_done = input->step_done;
    config = &controller->config;

    if (controller->state == LRA_STATE_FAILED) {
        output->abort = 1;
        return;
    }
    if (controller->state == LRA_STATE_FINISHED) {
        output->reason = LRA_REASON_SAFE_RESUME;
        return;
    }

    if (controller->state != LRA_STATE_IDLE) {
        if (input->stall) {
            lra_fail(controller, LRA_REASON_STALL, output);
            return;
        }
        if ((input->front_mm != LRA_INVALID_DISTANCE &&
             input->front_mm < config->crush_front_mm) ||
            (side_clearance != LRA_INVALID_DISTANCE &&
             side_clearance < config->crush_side_mm)) {
            lra_fail(controller, LRA_REASON_ENVELOPE, output);
            return;
        }
        if (!input->scan_valid) {
            lra_fail(controller, LRA_REASON_BAD_SCAN, output);
            return;
        }
        if (!input->heading_valid) {
            lra_fail(controller, LRA_REASON_BAD_HEADING, output);
            return;
        }
        if (controller->tick_count >= config->max_ticks) {
            lra_fail(controller, LRA_REASON_TIMEOUT, output);
            return;
        }
    }

    controller->tick_count++;

    if (input->scan_done && controller->state != LRA_STATE_IDLE) {
        if (input->front_lost) {
            if (controller->front_lost_count < config->front_lost_scans) {
                controller->front_lost_count++;
            }
        } else {
            controller->front_lost_count = 0;
        }
    }

    switch (controller->state) {
    case LRA_STATE_IDLE:
        if (input->front_mm != LRA_INVALID_DISTANCE &&
            input->front_mm <= config->trigger_mm) {
            lra_start_avoidance(controller, input, output);
        }
        break;

    case LRA_STATE_PIVOT_OUT:
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->target_heading_deg = controller->target_heading_deg;
        if (step_event) {
            if (lra_angle_error(input->heading_deg,
                                controller->target_heading_deg) >
                    config->pivot_tolerance_deg) {
                lra_fail(controller, LRA_REASON_TURN_MISSED, output);
            } else {
                controller->state = LRA_STATE_FOLLOW;
                lra_prepare_follow(controller, input, output);
            }
        }
        break;

    case LRA_STATE_FOLLOW:
        if (step_event) {
            lra_update_travel(controller, input);
            if (controller->extra_travel_mm >
                (float)config->max_extra_mm) {
                lra_fail(controller, LRA_REASON_EXTRA_TRAVEL, output);
                break;
            }
            if (controller->forward_mm >=
                    controller->obstacle_near_axle_mm +
                        (float)config->pass_margin_mm &&
                (side_clearance == LRA_INVALID_DISTANCE ||
                 side_clearance >= config->return_clearance_mm) &&
                (controller->front_lost_count >=
                     config->front_lost_scans ||
                 lra_front_is_clear(controller,
                                    input,
                                    (uint16_t)(config->trigger_mm + 50u)))) {
                controller->state = LRA_STATE_RETURN_ENTRY;
                output->state = controller->state;
                output->reason = LRA_REASON_OBSTACLE_PASSED;
            }
        }
        if (controller->state == LRA_STATE_FOLLOW) {
            lra_prepare_follow(controller, input, output);
        } else {
            lra_prepare_return(controller, input, output);
        }
        break;

    case LRA_STATE_FOLLOW_PIVOT:
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->target_heading_deg = controller->target_heading_deg;
        if (step_event) {
            if (lra_angle_error(input->heading_deg,
                                controller->target_heading_deg) >
                    config->pivot_tolerance_deg) {
                lra_fail(controller, LRA_REASON_TURN_MISSED, output);
            } else {
                controller->state = LRA_STATE_FOLLOW;
                lra_prepare_follow(controller, input, output);
            }
        }
        break;

    case LRA_STATE_RETURN_ENTRY:
        if ((input->front_mm != LRA_INVALID_DISTANCE &&
             input->front_mm < config->return_entry_front_mm) ||
            (side_clearance != LRA_INVALID_DISTANCE &&
             side_clearance < config->return_entry_side_mm)) {
            controller->state = LRA_STATE_FOLLOW;
            output->reason = LRA_REASON_RETURN_BLOCKED;
            lra_prepare_follow(controller, input, output);
        } else {
            controller->state = LRA_STATE_RETURN;
            lra_prepare_return(controller, input, output);
        }
        break;

    case LRA_STATE_RETURN:
        if (step_event) {
            lra_update_travel(controller, input);
        }
        if (lra_absf(controller->lateral_mm) <=
            config->return_tolerance_mm) {
            controller->state = LRA_STATE_PIVOT_HOME;
            controller->target_heading_deg = controller->home_heading_deg;
            output->state = controller->state;
            output->action = LRA_ACTION_PIVOT_TO_HEADING;
            output->target_heading_deg = controller->home_heading_deg;
        } else if (input->front_mm != LRA_INVALID_DISTANCE &&
                   input->front_mm < config->return_entry_front_mm) {
            controller->state = LRA_STATE_FOLLOW;
            output->reason = LRA_REASON_RETURN_BLOCKED;
            lra_prepare_follow(controller, input, output);
        } else {
            lra_prepare_return(controller, input, output);
        }
        break;

    case LRA_STATE_RETURN_PIVOT:
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->target_heading_deg = controller->target_heading_deg;
        if (step_event) {
            if (lra_angle_error(input->heading_deg,
                                controller->target_heading_deg) >
                    config->return_aim_tolerance_deg) {
                lra_fail(controller, LRA_REASON_TURN_MISSED, output);
            } else {
                controller->state = LRA_STATE_RETURN;
                lra_prepare_return(controller, input, output);
            }
        }
        break;

    case LRA_STATE_PIVOT_HOME:
        output->action = LRA_ACTION_PIVOT_TO_HEADING;
        output->target_heading_deg = controller->home_heading_deg;
        if (step_event) {
            if (lra_angle_error(input->heading_deg,
                                controller->home_heading_deg) >
                    config->pivot_tolerance_deg) {
                lra_fail(controller, LRA_REASON_TURN_MISSED, output);
            } else if (!lra_front_is_clear(
                           controller,
                           input,
                           (uint16_t)(config->trigger_mm + 50u))) {
                lra_fail(controller, LRA_REASON_RETURN_BLOCKED, output);
            } else {
                controller->state = LRA_STATE_FINISHED;
                controller->reason = LRA_REASON_SAFE_RESUME;
                output->state = controller->state;
                output->reason = controller->reason;
                output->action = LRA_ACTION_STOP;
            }
        }
        break;

    case LRA_STATE_FINISHED:
    case LRA_STATE_FAILED:
    default:
        break;
    }

    output->state = controller->state;
    if (controller->state == LRA_STATE_FAILED) {
        output->reason = controller->reason;
        output->abort = 1;
    }
}
