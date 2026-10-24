#include <assert.h>
#include <math.h>

#include "../src/harness/paced_schedule.h"

int main(void)
{
    const double epoch = 500.0;
    const double first_timestamp = 1e9;
    const double speed = 1.0;
    const double first_frame_offset = 0.05;

    assert(fabs(source_paced_target(epoch, first_timestamp,
                                    first_timestamp, speed,
                                    first_frame_offset) - 500.05) < 1e-6);
    assert(fabs(source_paced_target(epoch, first_timestamp,
                                    first_timestamp + 0.05, speed,
                                    first_frame_offset) - 500.10) < 1e-6);
    assert(fabs(source_paced_target(epoch, first_timestamp,
                                    first_timestamp + 0.15, speed,
                                    first_frame_offset) - 500.20) < 1e-6);
    assert(fabs(source_paced_target(epoch, first_timestamp,
                                    first_timestamp + 0.15, 2.0,
                                    first_frame_offset / 2.0) - 500.10) < 1e-6);
    return 0;
}
