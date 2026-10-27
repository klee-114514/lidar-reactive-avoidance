#ifndef LIDAR_AVOIDANCE_ADAPTER_H
#define LIDAR_AVOIDANCE_ADAPTER_H

#include "lidar_reactive_avoidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t bins[LRA_SECTOR_COUNT];
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;
    float heading_deg;
    float travel_delta_mm;
    uint8_t scan_valid;
    uint8_t scan_done;
    uint8_t front_lost;
    uint8_t heading_valid;
    uint8_t step_done;
    uint8_t stall;
} lra_platform_snapshot_t;

typedef struct {
    int (*read_snapshot)(void *context,
                         lra_platform_snapshot_t *snapshot);
    void (*apply_action)(void *context,
                         lra_action_t action,
                         float target_heading_deg,
                         uint16_t step_mm);
    void (*stop)(void *context);
    void (*emergency_stop)(void *context);
} lra_platform_io_t;

typedef struct {
    lra_controller_t controller;
    const lra_platform_io_t *io;
    void *context;
    uint8_t initialized;
} lra_adapter_t;

int lra_adapter_init(lra_adapter_t *adapter,
                     const lra_config_t *config,
                     const lra_platform_io_t *io,
                     void *context);

void lra_adapter_tick(lra_adapter_t *adapter);

#ifdef __cplusplus
}
#endif

#endif
