#include "lidar_reactive_avoidance.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    lra_config_t config;
    lra_controller_t controller;
    lra_input_t input;
    lra_output_t output;
    unsigned int i;

    lra_default_config(&config);
    lra_reset(&controller, &config);

    memset(&input, 0, sizeof(input));
    for (i = 0; i < LRA_SECTOR_COUNT; ++i) {
        input.bins[i] = 4000u;
    }
    input.scan_valid = 1;
    input.heading_valid = 1;
    input.front_mm = 430u;
    input.left_mm = 1200u;
    input.right_mm = 1200u;

    lra_update(&controller, &input, &output);

    printf("state=%d action=%d target=%.1f step=%u abort=%d\n",
           (int)output.state,
           (int)output.action,
           (double)output.target_heading_deg,
           (unsigned int)output.step_mm,
           output.abort);
    return output.abort ? 1 : 0;
}
