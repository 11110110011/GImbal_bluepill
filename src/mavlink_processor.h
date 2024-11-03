#ifndef MAVLINK_PROCESSOR_H
#define MAVLINK_PROCESSOR_H

void mavlink_processor_init();
void process_mavlink_message();
void send_gimbal_status();

#endif
