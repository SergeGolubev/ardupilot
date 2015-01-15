// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Mount driver backend class. Each supported mount type
  needs to have an object derived from this class.
 */

#ifndef __AP_MOUNT_BACKEND_H__
#define __AP_MOUNT_BACKEND_H__

#include <AP_Common.h>
#include <AP_Mount.h>

class AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Backend(AP_Mount &frontend, uint8_t instance) :
        _frontend(frontend),
        _instance(instance)
    {}

    // Virtual destructor
    virtual ~AP_Mount_Backend(void) {}

    // init - performs any required initialisation for this instance
    virtual void init() = 0;

    // update mount position - should be called periodically
    virtual void update() = 0;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const = 0;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode) = 0;

    // set_roi_target - sets target location that mount should attempt to point towards
    virtual void set_roi_target(const struct Location &target_loc) = 0;

    // configure_msg - process MOUNT_CONFIGURE messages received from GCS
    virtual void configure_msg(mavlink_message_t* msg) {};

    // control_msg - process MOUNT_CONTROL messages received from GCS
    virtual void control_msg(mavlink_message_t* msg) {};

    // status_msg - called to allow mounts to send their status to GCS via MAVLink
    virtual void status_msg(mavlink_channel_t chan) {};

protected:

    // update_targets_from_rc - updates angle targets (i.e. _angle_ef_target_rad) using input from receiver
    void update_targets_from_rc();

    // angle_input, angle_input_rad - convert RC input into an earth-frame target angle
    int32_t angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);
    float angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    // calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
    void calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad, bool calc_tilt, bool calc_pan);

    AP_Mount    &_frontend; // reference to the front end which holds parameters
    uint8_t     _instance;  // this instance's number
    Vector3f    _angle_ef_target_rad;   // desired earth-frame roll, tilt and pan angles in radians
};

#endif // __AP_MOUNT_BACKEND_H__