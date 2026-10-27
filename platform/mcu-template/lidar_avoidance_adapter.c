#include "lidar_avoidance_adapter.h"

#include <stddef.h>
#include <string.h>

static void lra_platform_emergency_stop(lra_adapter_t *adapter)
{
    if (adapter->io != NULL &&
        adapter->io->emergency_stop != NULL) {
        adapter->io->emergency_stop(adapter->context);
    } else if (adapter->io != NULL &&
               adapter->io->stop != NULL) {
        adapter->io->stop(adapter->context);
    }
}

int lra_adapter_init(lra_adapter_t *adapter,
                     const lra_config_t *config,
                     const lra_platform_io_t *io,
                     void *context)
{
    if (adapter == NULL || io == NULL ||
        io->read_snapshot == NULL ||
        io->apply_action == NULL ||
        io->stop == NULL ||
        io->emergency_stop == NULL) {
        return 0;
    }

    memset(adapter, 0, sizeof(*adapter));
    lra_reset(&adapter->controller, config);
    adapter->io = io;
    adapter->context = context;
    adapter->initialized = 1;
    return 1;
}

void lra_adapter_tick(lra_adapter_t *adapter)
{
    lra_platform_snapshot_t snapshot;
    lra_input_t input;
    lra_output_t output;
    uint32_t i;

    if (adapter == NULL || !adapter->initialized) {
        return;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    if (!adapter->io->read_snapshot(adapter->context, &snapshot)) {
        adapter->io->stop(adapter->context);
        return;
    }

    memset(&input, 0, sizeof(input));
    for (i = 0; i < LRA_SECTOR_COUNT; ++i) {
        input.bins[i] = snapshot.bins[i];
    }
    input.scan_valid = snapshot.scan_valid ? 1 : 0;
    input.scan_done = snapshot.scan_done ? 1 : 0;
    input.front_lost = snapshot.front_lost ? 1 : 0;
    input.front_mm = snapshot.front_mm;
    input.left_mm = snapshot.left_mm;
    input.right_mm = snapshot.right_mm;
    input.heading_deg = snapshot.heading_deg;
    input.heading_valid = snapshot.heading_valid ? 1 : 0;
    input.travel_delta_mm = snapshot.travel_delta_mm;
    input.step_done = snapshot.step_done ? 1 : 0;
    input.stall = snapshot.stall ? 1 : 0;

    lra_update(&adapter->controller, &input, &output);

    if (output.abort) {
        lra_platform_emergency_stop(adapter);
    } else if (output.action == LRA_ACTION_STOP) {
        adapter->io->stop(adapter->context);
    } else if (adapter->io->apply_action != NULL) {
        adapter->io->apply_action(adapter->context,
                                  output.action,
                                  output.target_heading_deg,
                                  output.step_mm);
    }
}
