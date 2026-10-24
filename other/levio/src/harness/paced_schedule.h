#ifndef LEVIO_PACED_SCHEDULE_H
#define LEVIO_PACED_SCHEDULE_H

static inline double source_paced_target(double epoch_s,
                                         double first_timestamp_s,
                                         double frame_timestamp_s,
                                         double speed,
                                         double first_frame_offset_s)
{
    return epoch_s + first_frame_offset_s +
           (frame_timestamp_s - first_timestamp_s) / speed;
}

#endif
