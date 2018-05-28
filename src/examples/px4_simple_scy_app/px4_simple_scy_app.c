
/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_scy_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_scy_app_main(int argc, char *argv[]);

int px4_simple_scy_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello sky!");
	PX4_INFO("test_app by rain....");

        int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
        orb_set_interval(sensor_sub_fd,1000);

        struct vehicle_attitude_s att;
        memset(&att,0,sizeof(att));
        orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude),&att);

        px4_pollfd_struct_t fds[] =
        {
            {.fd = sensor_sub_fd, .events = POLLIN},
        };

        int error_counter = 0;

        for(int i = 0; i < 5; i++)
        {
            int poll_ret = px4_poll(fds,1,1000);

            if(poll_ret == 0)
            {
                PX4_ERR("[px4_simple_app] Got no data within a second";

            }else if(poll_ret < 0)
            {
                if(error_counter <10 || error_counter % 50 ==0)
                {
                    PX4_ERR("[px4_simple_app] ERROR return value from poll(): %d",poll_ret);
                }

                 error_counter++;

            }else
            {
                if(fds[0].revents & POLLIN)
                {
                    struct sensor_combined_s raw;
                    orb_copy(ORB_ID(sensor_combined),sensor_sub_fd, &raw);
                    PX4_WARN("[px4_simple_scy_app] Accelerometer: \t%8.4f\t8.4f\t8.4f",
                             (double)raw.accelerometer_m_s2[0],
                             (double)raw.accelerometer_m_s2[1],
                             (double)raw.accelerometer_m_s2[2]);

                    att.roll = raw.accelerometer_m_s2[0];
                    att.pitch = raw.accelerometer_m_s2[1];
                    att.yaw = raw.accelerometer_m_s2[2];

                    orb_publish(ORB_ID(vehicle_attitude),att_pub, &att);
                }
            }

        }

        PX4_INFO("exiting");
        return 0;
}

