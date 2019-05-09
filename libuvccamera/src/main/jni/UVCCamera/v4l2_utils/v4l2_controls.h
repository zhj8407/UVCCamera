/*
 * v4l2_core.c
 *
 *  Created on: May 6, 2019
 *      Author: jiezhang
 */

#ifndef V4L2_CONTROLS_H
#define V4L2_CONTROLS_H

#include "uvc_dev.h"
#include "v4l2_core.h"

/*
 * ioctl with a number of retries in the case of I/O failure
 * args:
 *   fd - device descriptor
 *   IOCTL_X - ioctl reference
 *   arg - pointer to ioctl data
 *
 * asserts:
 *   none
 *
 * returns - ioctl result
 */
int xioctl(int fd, int IOCTL_X, void *arg);

/*
 * enumerate device (read/write) controls
 * and creates list in vd->list_device_controls
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *   vd->list_device_controls is null
 *
 * returns: error code
 */
int enumerate_v4l2_control(v4l2_dev_t *vd);

/*
 * subscribe for v4l2 control events
 * args:
 *  vd - pointer to video device data
 *  control_id - id of control to subscribe events for
 *
 * asserts:
 *  vd is not null
 *
 * return: none
 */
void v4l2_subscribe_control_events(v4l2_dev_t *vd, unsigned int control_id);

/*
 * unsubscribev4l2 control events
 * args:
 *  vd - pointer to video device data
 *
 * asserts:
 *  vd is not null
 *
 * return: none
 */
void v4l2_unsubscribe_control_events(v4l2_dev_t *vd);

/*
 * return the control associated to id from device list
 * args:
 *   vd - pointer to video device data
 *   id - control id
 *
 * asserts:
 *   vd is not null
 *
 * returns: pointer to v4l2_control if succeded or null otherwise
 */
v4l2_ctrl_t *get_control_by_id(v4l2_dev_t *vd, int id);

/*
 * return the control associated to name from device list
 * args:
 *   vd - pointer to video device data
 *   id - control's name
 *
 * asserts:
 *   vd is not null
 *
 * returns: pointer to v4l2_control if succeded or null otherwise
 */
v4l2_ctrl_t *get_control_by_name(v4l2_dev_t *vd, const std::string &name);

/*
 * updates the value for control id from the device
 * also updates control flags
 * args:
 *   vd - pointer to video device data
 *   id - control id
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid
 *
 * returns: ioctl result
 */
int get_control_value_by_id(v4l2_dev_t *vd, int id);

/*
 * sets the value of control id in device
 * args:
 *   vd - pointer to video device data
 *   id - control id
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid
 *
 * returns: ioctl result
 */
int set_control_value_by_id(v4l2_dev_t *vd, int id);


/*
 * sets the value of control id in device
 * args:
 *   vd - pointer to video device data
 *   id - control id
 *   reserved - temporal id for eu controls
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid
 *
 * returns: ioctl result
 */
int set_control_value_extended_by_id(v4l2_dev_t *vd, int id, int reserved);

/*
 * goes trough the control list and updates/retrieves current values
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void get_v4l2_control_values(v4l2_dev_t *vd);

/*
 * goes trough the control list and sets values in device
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void set_v4l2_control_values(v4l2_dev_t *vd);

/*
 * goes trough the control list and sets values in device to default
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void set_control_defaults(v4l2_dev_t *vd);

/*
 * Disables special auto-controls with higher IDs than
 * their absolute/relative counterparts
 * this is needed before restoring controls state
 *
 * args:
 *   vd - pointer to video device data
 *   id - control id
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void disable_special_auto(v4l2_dev_t *vd, int id);

/*
 * free control list
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void free_v4l2_control_list(v4l2_dev_t *vd);

#endif
