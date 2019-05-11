/*
 * v4l2_core.c
 *
 *  Created on: May 6, 2018
 *      Author: jiezhang
 */

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include <assert.h>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

#include "uvc_dev.h"
#include "v4l2_controls.h"
#include "v4l2_core.h"
#include "v4l2_video_formats.h"

#include "utilbase.h"

/*verbosity (global scope)*/
int verbosity = 3;

static int set_v4l2_framerate(v4l2_dev_t *vd);
static void clean_v4l2_dev(v4l2_dev_t *vd);

static void *_v4l2_frame_ready_caller(void *arg);

#define __PMUTEX &(vd->mutex)

/*
 * Query video device capabilities and supported formats
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *
 * returns: error code  (E_OK)
 */
static int check_v4l2_dev(v4l2_dev_t *vd)
{
    unsigned long stream_id = 0;
    /*assertions*/
    assert(vd != NULL);
    assert(vd->fd > 0);

    memset(&vd->cap, 0, sizeof(struct v4l2_capability));

    if (xioctl(vd->fd, VIDIOC_QUERYCAP, &vd->cap) < 0)
    {
        LOGE("V4L2_CORE: (VIDIOC_QUERYCAP) error: %s\n",
             strerror(errno));
        return E_QUERYCAP_ERR;
    }

    LOGI("V4l2 Device Cap: v4l2_cap.card: %s\n", vd->cap.card);
    LOGI("V4l2 Device Cap: v4l2_cap.driver: %s\n", vd->cap.driver);
    LOGI("V4l2 Device Cap: v4l2_cap.bus_info: %s\n", vd->cap.bus_info);

    if (strcmp((char *)vd->cap.driver, "uvcvideo") != 0)
    {
        LOGE("V4L2_CORE: Not a USB UVC Camera.\n");
        return E_DEVICE_ERR;
    }

    if (strncmp((char *)vd->cap.card, "Simul_", 6) == 0)
    {
        stream_id = strtoul(((char *)vd->cap.card) + 6, NULL, 10);

        if (verbosity > 0)
            LOGI("V4L2_CORE: Found simulcast stream, stream_id: %lu\n",
                 stream_id);
    }

    vd->stream_id = (uint8_t)stream_id;

    if ((vd->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0)
    {
        LOGE("V4L2_CORE: Error opening device %s: video capture not supported.\n",
             vd->videodevice);
        return E_QUERYCAP_ERR;
    }

    if (!(vd->cap.capabilities & V4L2_CAP_STREAMING))
    {
        LOGE("V4L2_CORE: %s does not support streaming i/o\n",
             vd->videodevice);
        return E_QUERYCAP_ERR;
    }

    if (vd->cap_meth == IO_READ)
    {

        vd->mem[vd->buf.index] = NULL;

        if (!(vd->cap.capabilities & V4L2_CAP_READWRITE))
        {
            LOGE("V4L2_CORE: %s does not support read, try with mmap\n",
                 vd->videodevice);
            return E_READ_ERR;
        }
    }

    if (verbosity > 0)
        LOGI("V4L2_CORE: Init. %s (location: %s)\n", vd->cap.card,
             vd->cap.bus_info);

    /*enumerate frame formats supported by device*/
    int ret = enum_frame_formats(vd);

    if (ret != E_OK)
    {
        LOGE("V4L2_CORE: no valid frame formats (with valid sizes) found for device\n");
        return ret;
    }

    /*enumerate device controls.*/
    enumerate_v4l2_control(vd);
    /*gets the current control values and sets their flags*/
    get_v4l2_control_values(vd);

    return E_OK;
}

/*
 * unmaps v4l2 buffers
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code  (0- E_OK)
 */
static int unmap_buff(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: unmapping v4l2 buffers\n");
    }

    int i = 0;
    int ret = E_OK;

    switch (vd->cap_meth)
    {
        case IO_READ:
            break;

        case IO_MMAP:
            for (i = 0; i < NB_BUFFER; i++)
            {
                // unmap old buffer
                if ((vd->mem[i] != MAP_FAILED) && vd->buff_length[i])
                    if ((ret = munmap(vd->mem[i], vd->buff_length[i])) < 0)
                    {
                        LOGE("V4L2_CORE: couldn't unmap buff: %s\n",
                             strerror(errno));
                    }
            }
    }

    return ret;
}

/*
 * maps v4l2 buffers
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code  (0- E_OK)
 */
static int map_buff(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: mapping v4l2 buffers\n");
    }

    int i = 0;

    // map new buffer
    for (i = 0; i < NB_BUFFER; i++)
    {
        vd->mem[i] = mmap(NULL, // start anywhere
                          vd->buff_length[i],
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED, vd->fd, vd->buff_offset[i]);

        if (vd->mem[i] == MAP_FAILED)
        {
            LOGE("V4L2_CORE: Unable to map buffer: %s\n",
                 strerror(errno));
            return E_MMAP_ERR;
        }

        if (verbosity > 1)
            LOGI("V4L2_CORE: mapped buffer[%i] with length %i to pos %p\n", i,
                 vd->buff_length[i], vd->mem[i]);
    }

    return (E_OK);
}

/*
 * Query and map buffers
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code  (0- E_OK)
 */
static int query_buff(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: query v4l2 buffers\n");
    }

    int i = 0;
    int ret = E_OK;

    switch (vd->cap_meth)
    {
        case IO_READ:
            break;

        case IO_MMAP:
            for (i = 0; i < NB_BUFFER; i++)
            {
                memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
                vd->buf.index = i;
                vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                //vd->buf.flags = V4L2_BUF_FLAG_TIMECODE;
                //vd->buf.timecode = vd->timecode;
                //vd->buf.timestamp.tv_sec = 0;
                //vd->buf.timestamp.tv_usec = 0;
                vd->buf.memory = V4L2_MEMORY_MMAP;
                ret = xioctl(vd->fd, VIDIOC_QUERYBUF, &vd->buf);

                if (ret < 0)
                {
                    LOGE("V4L2_CORE: (VIDIOC_QUERYBUF) Unable to query buffer[%i]: %s\n",
                         i, strerror(errno));

                    if (errno == EINVAL)
                    {
                        LOGE("         try with read method instead\n");
                    }

                    return E_QUERYBUF_ERR;
                }

                if (vd->buf.length <= 0)
                    LOGE("V4L2_CORE: (VIDIOC_QUERYBUF) - buffer length is %i\n",
                         vd->buf.length);

                vd->buff_length[i] = vd->buf.length;
                vd->buff_offset[i] = vd->buf.m.offset;
            }

            // map the new buffers
            if (map_buff(vd) != 0)
            {
                ret = E_MMAP_ERR;
            }

            break;
    }

    return ret;
}

/*
 * Queue Buffers
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code  (0- E_OK)
 */
static int queue_buff(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: queue v4l2 buffers\n");
    }

    int i = 0;
    int ret = E_OK;

    switch (vd->cap_meth)
    {
        case IO_READ:
            break;

        case IO_MMAP:
        default:
            for (i = 0; i < NB_BUFFER; ++i)
            {
                memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
                vd->buf.index = i;
                vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                vd->buf.memory = V4L2_MEMORY_MMAP;
                ret = xioctl(vd->fd, VIDIOC_QBUF, &vd->buf);

                if (ret < 0)
                {
                    LOGE("V4L2_CORE: (VIDIOC_QBUF) Unable to queue buffer: %s\n",
                         strerror(errno));
                    return E_QBUF_ERR;
                }
            }

            vd->buf.index = 0; /*reset index*/
    }

    return ret;
}

/*
 * do a VIDIOC_S_PARM ioctl for setting frame rate
 * args:
 *    vd - pointer to v4l2 device handler
 *
 * asserts:
 *    vd is not null
 *
 * returns: error code
 */
static int do_v4l2_framerate_update(v4l2_dev_t *vd)
{
    /*asserts*/
    assert(vd != NULL);

    int ret = 0;

    /*get the current stream parameters*/
    vd->streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(vd->fd, VIDIOC_G_PARM, &vd->streamparm);

    if (ret < 0)
    {
        LOGE("V4L2_CORE: (VIDIOC_G_PARM) error: %s\n",
             strerror(errno));
        LOGE("V4L2_CORE: Unable to set %d/%d fps\n", vd->fps_num,
             vd->fps_denom);
        return ret;
    }

    if (!(vd->streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
    {
        LOGE("V4L2_CORE: V4L2_CAP_TIMEPERFRAME not supported\n");
    }

    vd->streamparm.parm.capture.timeperframe.numerator = vd->fps_num;
    vd->streamparm.parm.capture.timeperframe.denominator = vd->fps_denom;

    /*request the new frame rate*/
    ret = xioctl(vd->fd, VIDIOC_S_PARM, &vd->streamparm);

    if (ret < 0)
    {
        LOGE("V4L2_CORE: (VIDIOC_S_PARM) error: %s\n",
             strerror(errno));
        LOGE("V4L2_CORE: Unable to set %d/%d fps\n", vd->fps_num,
             vd->fps_denom);
    }

    return ret;
}

/*
 * Initiate video device handler with default values
 * args:
 *   device - device name (e.g: "/dev/video0")
 *
 * asserts:
 *   device is not null
 *
 * returns: pointer to v4l2 device handler (or NULL on error)
 */
v4l2_dev_t *v4l2core_init_dev(const char *device)
{
    /*assertions*/
    assert(device != NULL);

    /*alloc the device data*/
    v4l2_dev_t *vd = new v4l2_dev_t();

    assert(vd != NULL);

    /*init the device mutex*/
    __INIT_MUTEX(__PMUTEX);

    /*MMAP by default*/
    vd->cap_meth = IO_MMAP;

    vd->videodevice = strdup(device);

    if (verbosity > 0)
    {
        LOGI("V4L2_CORE: capture method mmap (%i)\n", vd->cap_meth);
        LOGI("V4L2_CORE: video device: %s \n", vd->videodevice);
    }

    /*set some defaults*/
    vd->fps_num = 1;
    vd->fps_denom = 30;
    vd->curr_rate_control_mode = V4L2_ENC_RATECONTROL_MODE_VBR;

    /*open device*/
    if ((vd->fd = open(vd->videodevice, O_RDWR | O_NONBLOCK, 0)) < 0)
    {
        LOGE("V4L2_CORE: ERROR opening V4L interface: %s\n",
             strerror(errno));
        clean_v4l2_dev(vd);
        return (NULL);
    }

    /*zero structs*/
    memset(&vd->cap, 0, sizeof(struct v4l2_capability));
    memset(&vd->format, 0, sizeof(struct v4l2_format));
    memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
    memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
    memset(&vd->streamparm, 0, sizeof(struct v4l2_streamparm));

    if (check_v4l2_dev(vd) != E_OK)
    {
        clean_v4l2_dev(vd);
        return (NULL);
    }

    int i = 0;

    for (i = 0; i < NB_BUFFER; i++)
    {
        vd->mem[i] = MAP_FAILED; /*not mmaped yet*/
    }

    return (vd);
}

/*
 * cleans video device data and allocations
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   none
 *
 * returns: void
 */
void v4l2core_close_dev(v4l2_dev_t *vd)
{
    if (vd == NULL)
    {
        return;
    }

    /* thread must be joined before destroying the mutex
     * so no need to unlock before destroying it
     */

    /*destroy the device mutex*/
    __CLOSE_MUTEX(__PMUTEX);

    v4l2core_clean_buffers(vd);
    clean_v4l2_dev(vd);
}

/*
 * clean video device data allocation
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
static void clean_v4l2_dev(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (vd->videodevice)
    {
        free(vd->videodevice);
    }

    vd->videodevice = NULL;

    if (vd->list_device_controls)
    {
        free_v4l2_control_list(vd);
    }

    if (vd->list_stream_formats)
    {
        free_frame_formats(vd);
    }

    /*close descriptor*/
    if (vd->fd > 0)
    {
        close(vd->fd);
    }

    vd->fd = 0;

    delete vd;
}

/*
 * prepare new format (index)
 * args:
 *   vd - pointer to v4l2 device handler
 *   format_index - new format index
 *
 * asserts:
 *    vd is not null
 *
 * returns: none
 */
void v4l2core_prepare_new_format_index(v4l2_dev_t *vd, int format_index)
{
    /*asserts*/
    assert(vd != NULL);

    if (format_index < 0)
    {
        format_index = 0;
    }

    if (format_index >= vd->numb_formats)
    {
        format_index = 0;
    }

    vd->curr_pixelformat = vd->list_stream_formats[format_index].format;
}

/*
 * prepare new format
 * args:
 *   vd - pointer to v4l2 device handler
 *   new_format - new format
 *
 * asserts:
 *    vd is not null
 *
 * returns: none
 */
void v4l2core_prepare_new_format(v4l2_dev_t *vd, int new_format)
{
    /*asserts*/
    assert(vd != NULL);

    int format_index = get_frame_format_index(vd, new_format);

    if (format_index < 0)
    {
        format_index = 0;
    }

    vd->curr_pixelformat = vd->list_stream_formats[format_index].format;
}

/*
 * prepare new resolution
 * args:
 *   vd - pointer to v4l2 device handler
 *   new_width - new width
 *   new_height - new height
 *
 * asserts:
 *    vd is not null
 *
 * returns: none
 */
void v4l2core_prepare_new_resolution(v4l2_dev_t *vd, int new_width,
                                     int new_height, int new_profile, int new_ucconfig,
                                     int new_stream_0_layout, int new_stream_1_layout,
                                     int new_stream_2_layout, int new_stream_3_layout)
{
    /*asserts*/
    assert(vd != NULL);

    int format_index = get_frame_format_index(vd, vd->curr_pixelformat);

    if (format_index < 0)
    {
        format_index = 0;
    }

    if (vd->curr_pixelformat != V4L2_PIX_FMT_H264 &&
            vd->curr_pixelformat != V4L2_PIX_FMT_H264_SIMULCAST)
    {
        new_profile = 0;
    }

    int resolution_index = get_format_resolution_index(vd, format_index,
                           new_width, new_height, new_profile);

    if (verbosity > 2)
    {
        LOGI("Found resolution index: %d\n", resolution_index);
    }

    if (resolution_index < 0)
    {
        resolution_index = 0;
    }

    vd->curr_width =
        vd->list_stream_formats[format_index].list_stream_cap[resolution_index].width;
    vd->curr_height =
        vd->list_stream_formats[format_index].list_stream_cap[resolution_index].height;
    vd->curr_profile =
        vd->list_stream_formats[format_index].list_stream_cap[resolution_index].profile;

    vd->curr_ucconfig = new_ucconfig;
    vd->stream_0_layout = new_stream_0_layout;
    vd->stream_1_layout = new_stream_1_layout;
    vd->stream_2_layout = new_stream_2_layout;
    vd->stream_3_layout = new_stream_3_layout;
}

/*
 * Start video stream
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: VIDIOC_STREAMON ioctl result (E_OK or E_STREAMON_ERR)
 */
int v4l2core_start_stream(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (vd->streaming == STRM_OK)
    {
        LOGE("V4L2_CORE: (stream already started) stream_status = STRM_OK\n");
        return E_OK;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret = E_OK;

    switch (vd->cap_meth)
    {
        case IO_READ:
            //do nothing
            break;

        case IO_MMAP:
        default:
            ret = xioctl(vd->fd, VIDIOC_STREAMON, &type);

            if (ret < 0)
            {
                LOGE("V4L2_CORE: (VIDIOC_STREAMON) Unable to start stream: %s \n",
                     strerror(errno));
                return E_STREAMON_ERR;
            }

            break;
    }

    vd->streaming = STRM_OK;

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: (VIDIOC_STREAMON) stream_status = STRM_OK\n");
    }

    if (vd->running_thread == 0)
    {
        int err = __THREAD_CREATE(&vd->running_thread, _v4l2_frame_ready_caller, vd);

        if (err)
        {
            LOGE("V4L2_CORE: Failed to create the running thread\n");
        }
    }

    return ret;
}

/*
 * Stops the video stream
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: VIDIOC_STREAMON ioctl result (E_OK)
 */
int v4l2core_stop_stream(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret = E_OK;

    /*Clean up the control sets.*/
    vd->controls_set_list.clear();

    switch (vd->cap_meth)
    {
        case IO_READ:
        case IO_MMAP:
        default:
            ret = xioctl(vd->fd, VIDIOC_STREAMOFF, &type);

            if (ret < 0)
            {
                if (errno == 9)
                {
                    /* stream allready stoped*/
                    vd->streaming = STRM_STOP;
                    ret = 0;
                }
                else
                {
                    LOGE("V4L2_CORE: (VIDIOC_STREAMOFF) Unable to stop stream: %s\n",
                         strerror(errno));
                    return E_STREAMOFF_ERR;
                }
            }

            break;
    }

    vd->streaming = STRM_STOP;

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: (VIDIOC_STREAMOFF) stream_status = STRM_STOP\n");
    }

    if (vd->running_thread != 0)
    {
        __THREAD_JOIN(vd->running_thread);
    }

    if (verbosity > 2)
    {
        LOGI("V4L2_CORE: (VIDIOC_STREAMOFF) stream stop done!!!\n");
    }

    vd->running_thread = 0;

    return ret;
}

/*
 * clean v4l2 buffers
 * args:
 *    vd - pointer to v4l2 device handler
 *
 * asserts:
 *    vd is not null
 *
 * return: int
 */
int v4l2core_clean_buffers(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);
    int ret = 0;

    if (verbosity > 1)
    {
        LOGI("V4L2_CORE: cleaning v4l2 buffers\n");
    }

    if (vd->streaming == STRM_OK)
    {
        /* Ignore the return value. We must unmap
         * the buffers anyway.
         */
        v4l2core_stop_stream(vd);
    }

    // unmap queue buffers
    switch (vd->cap_meth)
    {
        case IO_READ:
            if (vd->mem[vd->buf.index] != NULL)
            {
                free(vd->mem[vd->buf.index]);
                vd->mem[vd->buf.index] = NULL;
            }

            break;

        case IO_MMAP:
        default:
            //delete requested buffers
            unmap_buff(vd);
            memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
            vd->rb.count = 0;
            vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            vd->rb.memory = V4L2_MEMORY_MMAP;

            if ((ret = xioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb) < 0))
            {
                LOGE("V4L2_CORE: (VIDIOC_REQBUFS) Failed to delete buffers: %s (errno %d)\n",
                     strerror(errno), errno);
            }

            break;
    }

    return ret;
}

/*
 * define fps values
 * args:
 *   vd - pointer to v4l2 device handler
 *   num - fps numerator
 *   denom - fps denominator
 *
 * asserts:
 *   vd is not null
 *
 * returns - void
 */
void v4l2core_define_fps(v4l2_dev_t *vd, int num, int denom)
{
    /*assertions*/
    assert(vd != NULL);

    if (num > 0)
    {
        vd->fps_num = num;
    }

    if (denom > 0)
    {
        vd->fps_denom = denom;
    }

    if (verbosity > 2)
        LOGI("V4L2_CORE: fps configured to %i/%i\n", vd->fps_num,
             vd->fps_denom);
}

void v4l2core_set_frame_callback(v4l2_dev_t *vd,
                                 uvc_frame_callback_t *cb, void *user_ptr)
{
    /*assertions*/
    assert(vd != NULL);

    vd->user_cb = cb;
    vd->user_ptr = user_ptr;
}

/*
 * request a fps update - this locks the mutex
 *   (can't be called while the mutex is being locked)
 * args:
 *    vd - pointer to v4l2 device handler
 *
 * asserts:
 *    vd is not null
 *
 * returns: none
 */
void v4l2core_request_framerate_update(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    /*
     * if we are streaming flag a fps change when retrieving frame
     * else change fps immediatly
     */
    if (vd->streaming == STRM_OK)
    {
        vd->flag_fps_change = 1;
    }
    else
    {
        set_v4l2_framerate(vd);
    }
}

/*
 * gets video device defined frame rate (not real - consider it a maximum value)
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: VIDIOC_G_PARM ioctl result value
 * (sets vd->fps_denom and vd->fps_num to device value)
 */
int v4l2core_get_framerate(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    int ret = 0;

    vd->streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(vd->fd, VIDIOC_G_PARM, &vd->streamparm);

    if (ret < 0)
    {
        LOGE("V4L2_CORE: (VIDIOC_G_PARM) error: %s\n",
             strerror(errno));
        return ret;
    }
    else
    {
        if (vd->streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)
        {
            vd->fps_denom =
                vd->streamparm.parm.capture.timeperframe.denominator;
            vd->fps_num = vd->streamparm.parm.capture.timeperframe.numerator;
        }
    }

    if (vd->fps_denom == 0)
    {
        vd->fps_denom = 1;
    }

    if (vd->fps_num == 0)
    {
        vd->fps_num = 1;
    }

    return ret;
}

/*
 * sets video device frame rate
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: VIDIOC_S_PARM ioctl result value
 * (sets vd->fps_denom and vd->fps_num to device value)
 */
static int set_v4l2_framerate(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);

    if (verbosity > 2)
        LOGI("V4L2_CORE: trying to change fps to %i/%i\n", vd->fps_num,
             vd->fps_denom);

    int ret = 0;

    /*lock the mutex*/
    __LOCK_MUTEX(__PMUTEX);

    /*store streaming flag*/
    uint8_t stream_status = vd->streaming;

    /*try to stop the video stream*/
    if (stream_status == STRM_OK)
    {
        v4l2core_stop_stream(vd);
    }

    switch (vd->cap_meth)
    {
        case IO_READ:
            ret = do_v4l2_framerate_update(vd);
            break;

        case IO_MMAP:
            if (stream_status == STRM_OK)
            {
                /*unmap the buffers*/
                unmap_buff(vd);
            }

            ret = do_v4l2_framerate_update(vd);

            break;
    }

    if (stream_status == STRM_OK)
    {
        query_buff(vd); /*also mmaps the buffers*/
        queue_buff(vd);
    }

    /*try to start the video stream*/
    if (stream_status == STRM_OK)
    {
        v4l2core_start_stream(vd);
    }

    /*unlock the mutex*/
    __UNLOCK_MUTEX(__PMUTEX);

    return ret;
}

/*
 * Try/Set device video stream format
 * args:
 *   vd -pointer to device data
 *   width - requested video frame width
 *   height - requested video frame height
 *   pixelformat - requested v4l2 pixelformat
 *   profile - requested h264 profile
 *   ucconfig - requested UCConfig value (0: AVC, 1: SVC)
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code ( E_OK)
 */
static int try_video_stream_format(v4l2_dev_t *vd, int width, int height,
                                   int pixelformat, int profile,
                                   int ucconfig,
                                   int rate_control_mode,
                                   int stream_0_layout,
                                   int stream_1_layout,
                                   int stream_2_layout,
                                   int stream_3_layout)
{
    /*assertions*/
    assert(vd != NULL);

    int ret = E_OK;

    /*lock the mutex*/
    __LOCK_MUTEX(__PMUTEX);

    vd->requested_fmt = pixelformat;

    uint8_t stream_status = vd->streaming;

    if (stream_status == STRM_OK)
    {
        v4l2core_stop_stream(vd);
    }

    vd->format.fmt.pix.pixelformat = pixelformat;
    vd->format.fmt.pix.width = width;
    vd->format.fmt.pix.height = height;

    /* make sure we set a valid format*/
    if (verbosity > 0)
        LOGI("V4L2_CORE: checking format: %dX%d@%c%c%c%c\n",
             vd->format.fmt.pix.width,
             vd->format.fmt.pix.height,
             (vd->format.fmt.pix.pixelformat) & 0xFF,
             ((vd->format.fmt.pix.pixelformat) >> 8) & 0xFF,
             ((vd->format.fmt.pix.pixelformat) >> 16) & 0xFF,
             ((vd->format.fmt.pix.pixelformat) >> 24) & 0xFF);

    /*override field and type entries*/
    vd->format.fmt.pix.field = V4L2_FIELD_ANY;
    vd->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd->format.fmt.pix.priv = 0;

    if (vd->requested_fmt == V4L2_PIX_FMT_H264)
    {
        vd->format.fmt.pix.priv |= ((profile & 0xFFFF) << 16 |
                                    (rate_control_mode) << 8 |
                                    (stream_0_layout & 0x0F) << 4 |
                                    ((ucconfig + 1) & 0xFF));

        if (verbosity > 0)
        {
            LOGI("V4L2_CORE: request H264 stream. Profile: 0x%04x, UCConfig: %d\n",
                 profile, ucconfig);
            LOGI("V4L2_CORE: private data is 0x%08x\n",
                 vd->format.fmt.pix.priv);
        }
    }
    else if (vd->requested_fmt == V4L2_PIX_FMT_H264_SIMULCAST)
    {
        uint16_t layout = generate_layout_structure(ucconfig, stream_0_layout,
                          stream_1_layout, stream_2_layout, stream_3_layout);

        if (verbosity > 0)
        {
            LOGI(
                "V4L2_Core: request S264 stream. ucConfig: %d, Stream Layout: %d:%d:%d:%d - 0x%04x\n",
                ucconfig, stream_0_layout, stream_1_layout, stream_2_layout,
                stream_3_layout, layout);
        }

        vd->format.fmt.pix.priv |= (layout << 16 | (rate_control_mode) << 8 | ((ucconfig + 1) & 0xFF));
    }

    ret = xioctl(vd->fd, VIDIOC_S_FMT, &vd->format);

    /*unlock the mutex*/
    __UNLOCK_MUTEX(__PMUTEX);

    if (ret != 0)
    {
        LOGE("V4L2_CORE: (VIDIOC_S_FORMAT) Unable to set format: %s\n",
             strerror(errno));
        return E_FORMAT_ERR;
    }

    if ((vd->format.fmt.pix.width != width) || (vd->format.fmt.pix.height != height))
    {
        LOGE("V4L2_CORE: Requested resolution unavailable: got width %d height %d\n",
             vd->format.fmt.pix.width, vd->format.fmt.pix.height);
    }

    if (vd->requested_fmt == V4L2_PIX_FMT_H264_SIMULCAST)
    {
        uint8_t real_ucconfig = (vd->format.fmt.pix.priv & 0x0F) - 1;
        uint16_t layouts = vd->format.fmt.pix.priv >> 16;
        uint8_t layout0 = get_layers_from_layout_structure(0, layouts);
        uint8_t layout1 = get_layers_from_layout_structure(1, layouts);
        uint8_t layout2 = get_layers_from_layout_structure(2, layouts);
        uint8_t layout3 = get_layers_from_layout_structure(3, layouts);

        if (verbosity > 0)
            LOGI("V4L2_Core: get S264 stream layout: %d:%d:%d:%d (UCConfig: %d)\n",
                 layout0,
                 layout1,
                 layout2,
                 layout3,
                 real_ucconfig);

    }
    else if (vd->requested_fmt == V4L2_PIX_FMT_H264)
    {
        uint8_t real_ucconfig = (vd->format.fmt.pix.priv & 0x0F) - 1;
        uint8_t layout = ((vd->format.fmt.pix.priv >> 4) & 0x0F);

        if (verbosity > 0)
            LOGI("V4L2_Core: get H264 stream layout: %d (UCConfig: %d)\n",
                 layout,
                 real_ucconfig);
    }

    switch (vd->cap_meth)
    {
        case IO_READ: /*allocate buffer for read*/
            /*lock the mutex*/
            __LOCK_MUTEX(__PMUTEX);

            memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
            vd->buf.length = (vd->format.fmt.pix.width) * (vd->format.fmt.pix.height) * 3; //worst case (rgb)
            vd->mem[vd->buf.index] = calloc(vd->buf.length, sizeof(uint8_t));

            if (vd->mem[vd->buf.index] == NULL)
            {
                LOGE("V4L2_CORE: FATAL memory allocation failure (try_video_stream_format): %s\n",
                     strerror(errno));
                return -1;
            }

            /*unlock the mutex*/
            __UNLOCK_MUTEX(__PMUTEX);
            break;

        case IO_MMAP:
        default:
            /* request buffers */
            memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
            vd->rb.count = NB_BUFFER;
            vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            vd->rb.memory = V4L2_MEMORY_MMAP;

            ret = xioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb);

            if (ret < 0)
            {
                LOGE("V4L2_CORE: (VIDIOC_REQBUFS) Unable to allocate buffers: %s\n",
                     strerror(errno));
                return E_REQBUFS_ERR;
            }

            /* map the buffers */
            if (query_buff(vd))
            {
                LOGE("V4L2_CORE: (VIDIOC_QBUFS) Unable to query buffers: %s\n",
                     strerror(errno));

                /*
                     * delete requested buffers
                     * no need to unmap as mmap failed for sure
                     */
                if (verbosity > 0)
                {
                    LOGI("V4L2_CORE: cleaning request buffers\n");
                }

                memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
                vd->rb.count = 0;
                vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                vd->rb.memory = V4L2_MEMORY_MMAP;

                if (xioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb) < 0)
                    LOGE("V4L2_CORE: (VIDIOC_REQBUFS) Unable to delete buffers: %s\n",
                         strerror(errno));

                return E_QUERYBUF_ERR;
            }

            /* Queue the buffers */
            if (queue_buff(vd))
            {
                LOGE("V4L2_CORE: (VIDIOC_QBUFS) Unable to queue buffers: %s\n",
                     strerror(errno));

                /*delete requested buffers */
                if (verbosity > 0)
                {
                    LOGI("V4L2_CORE: cleaning request buffers\n");
                }

                unmap_buff(vd);
                memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
                vd->rb.count = 0;
                vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                vd->rb.memory = V4L2_MEMORY_MMAP;

                if (xioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb) < 0)
                    LOGE("V4L2_CORE: (VIDIOC_REQBUFS) Unable to delete buffers: %s\n",
                         strerror(errno));

                return E_QBUF_ERR;
            }
    }

    /*this locks the mutex (can't be called while the mutex is being locked)*/
    v4l2core_request_framerate_update(vd);

    if (stream_status == STRM_OK)
    {
        v4l2core_start_stream(vd);
    }

    /*update the current framerate for the device*/
    v4l2core_get_framerate(vd);

    return E_OK;
}

/*
 * update the current format (pixelformat, width and height)
 * args:
 *    vd - pointer to v4l2 device handler
 *
 * asserts:
 *    vd is not null
 *
 * returns:
 *    error code
 */
int v4l2core_update_current_format(v4l2_dev_t *vd)
{
    /*asserts*/
    assert(vd != NULL);

    return try_video_stream_format(vd, vd->curr_width, vd->curr_height, vd->curr_pixelformat,
                                   vd->curr_profile, vd->curr_ucconfig, vd->curr_rate_control_mode,
                                   vd->stream_0_layout, vd->stream_1_layout,
                                   vd->stream_2_layout, vd->stream_3_layout);
}

static void *_v4l2_frame_ready_caller(void *arg)
{
    v4l2_dev_t *vd = (v4l2_dev_t *)arg;

    while (1)
    {
        /* We wil handle the data in process_input_buffer.
         * So, the NULL ptrs are passed in
         */
        int qind = v4l2core_get_frame(vd, NULL, NULL);

        if (qind < 0)
        {
            break;
        }

        v4l2core_release_frame(vd, qind);
    }
}

/*
 * process input buffer
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * returns: frame_queue index
 */
static int process_input_buffer(v4l2_dev_t *vd)
{
    uvc_frame_t *uvc_frame = reinterpret_cast<uvc_frame_t *>(calloc(1, sizeof(uvc_frame_t)));

    uvc_frame->width = vd->format.fmt.pix.width;
    uvc_frame->height = vd->format.fmt.pix.height;

    switch (vd->requested_fmt)
    {
        case V4L2_PIX_FMT_YUYV:
            uvc_frame->frame_format = UVC_FRAME_FORMAT_YUYV;
            break;

        case V4L2_PIX_FMT_MJPEG:
            uvc_frame->frame_format = UVC_FRAME_FORMAT_MJPEG;
            break;

        case V4L2_PIX_FMT_NV12:
            uvc_frame->frame_format = UVC_FRAME_FORMAT_NV12;
            break;

        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H264_SIMULCAST:
            uvc_frame->frame_format = UVC_FRAME_FORMAT_H_264;
            break;

        default:
            uvc_frame->frame_format = UVC_FRAME_FORMAT_UNKNOWN;
            break;
    }

    uvc_frame->data = vd->mem[vd->buf.index];
    uvc_frame->data_bytes = uvc_frame->actual_bytes = vd->buf.bytesused;

    if (vd->user_cb)
    {
        vd->user_cb(uvc_frame, vd->user_ptr);
    }

    free(uvc_frame);

    return vd->buf.index;
}

/*
 * checks if frame data is available
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code  (0- E_OK)
 */
static int check_frame_available(v4l2_dev_t *vd)
{
    /*asserts*/
    assert(vd != NULL);

    int ret = E_OK;
    fd_set rdset;
    struct timeval timeout;

    /*lock the mutex*/
    __LOCK_MUTEX(__PMUTEX);
    int stream_state = vd->streaming;
    /*unlock the mutex*/
    __UNLOCK_MUTEX(__PMUTEX);

    /*make sure streaming is on*/
    if (stream_state != STRM_OK)
    {
        if (stream_state == STRM_REQ_STOP)
        {
            v4l2core_stop_stream(vd);
        }

        LOGE("V4L2_CORE: (get_v4l2_frame) video stream must be started first\n");
        return E_NO_STREAM_ERR;
    }

    /*a fps change was requested while streaming*/
    if (vd->flag_fps_change > 0)
    {
        if (verbosity > 2)
        {
            LOGI("V4L2_CORE: fps change request detected\n");
        }

        set_v4l2_framerate(vd);
        vd->flag_fps_change = 0;
    }

    FD_ZERO(&rdset);
    FD_SET(vd->fd, &rdset);
    timeout.tv_sec = 2; /* 2 sec timeout*/
    timeout.tv_usec = 0;
    /* select - wait for data or timeout*/
    ret = select(vd->fd + 1, &rdset, NULL, NULL, &timeout);

    if (ret < 0)
    {
        LOGE("V4L2_CORE: Could not grab image (select error): %s\n",
             strerror(errno));
        return E_SELECT_ERR;
    }

    if (ret == 0)
    {
        LOGE("V4L2_CORE: Could not grab image (select timeout): %s\n",
             strerror(errno));
        return E_SELECT_TIMEOUT_ERR;
    }

    if ((ret > 0) && (FD_ISSET(vd->fd, &rdset)))
    {
        return E_OK;
    }

    return E_UNKNOWN_ERR;
}

/*
 * gets the next video frame (must be released after processing)
 * args:
 *   vd - pointer to v4l2 device handler
 *
 * asserts:
 *   vd is not null
 *
 * returns: buffer index
 */
int v4l2core_get_frame(v4l2_dev_t *vd, void **frame_data, uint32_t *frame_size)
{
    /*asserts*/
    assert(vd != NULL);

    int res = 0;
    int ret = check_frame_available(vd);

    int qind = -1;

    if (ret < 0)
    {
        return -1;
    }

    int bytes_used = 0;

    switch (vd->cap_meth)
    {
        case IO_READ:

            /*lock the mutex*/
            __LOCK_MUTEX(__PMUTEX);

            if (vd->streaming == STRM_OK)
            {
                vd->buf.bytesused = read(vd->fd, vd->mem[vd->buf.index],
                                         vd->buf.length);
                LOGI("vd->buf.bytesused=%u\n", vd->buf.bytesused);
                bytes_used = vd->buf.bytesused;

                if (bytes_used)
                {
                    if (frame_data)
                    {
                        *frame_data = vd->mem[vd->buf.index];
                    }

                    if (frame_size)
                    {
                        *frame_size = vd->buf.bytesused;
                    }

                    qind = process_input_buffer(vd);
                }
            }
            else
            {
                res = -1;
            }

            /*unlock the mutex*/
            __UNLOCK_MUTEX(__PMUTEX);

            if (res < 0)
            {
                return res;
            }

            if (-1 == bytes_used)
            {
                switch (errno)
                {
                    case EAGAIN:
                        LOGE("V4L2_CORE: No data available for read: %s\n",
                             strerror(errno));
                        break;

                    case EINVAL:
                        LOGE("V4L2_CORE: Read method error, try mmap instead: %s\n",
                             strerror(errno));
                        break;

                    case EIO:
                        LOGE("V4L2_CORE: read I/O Error: %s\n",
                             strerror(errno));
                        break;

                    default:
                        LOGE("V4L2_CORE: read: %s\n", strerror(errno));
                        break;
                }

                return -1;
            }

            break;

        case IO_MMAP:
        default:

            /*lock the mutex*/
            __LOCK_MUTEX(__PMUTEX);

            if (vd->streaming == STRM_OK)
            {
                memset(&vd->buf, 0, sizeof(struct v4l2_buffer));

                vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                vd->buf.memory = V4L2_MEMORY_MMAP;

                ret = xioctl(vd->fd, VIDIOC_DQBUF, &vd->buf);

                if (!ret)
                {
                    if (frame_data)
                    {
                        *frame_data = vd->mem[vd->buf.index];
                    }

                    if (frame_size)
                    {
                        *frame_size = vd->buf.bytesused;
                    }

                    qind = process_input_buffer(vd);
                }
                else
                    LOGE("V4L2_CORE: (VIDIOC_DQBUF) Unable to dequeue buffer: %s\n",
                         strerror(errno));
            }
            else
            {
                res = -1;
            }

            /*unlock the mutex*/
            __UNLOCK_MUTEX(__PMUTEX);

            if (res < 0 || ret < 0)
            {
                return -1;
            }
    }

    return qind;
}

/*
 * releases the video frame (so that it can be reused by the driver)
 * args:
 *   vd - pointer to v4l2 device handler
 *   frame - pointer to decoded frame buffer
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code (E_OK)
 */
int v4l2core_release_frame(v4l2_dev_t *vd, int index)
{
    int ret = 0;

    //match the v4l2_buffer with the corresponding frame
    vd->buf.index = index;

    switch (vd->cap_meth)
    {
        case IO_READ:
            break;

        case IO_MMAP:
        default:
            /* queue the buffer */
            ret = xioctl(vd->fd, VIDIOC_QBUF, &vd->buf);

            if (ret)
                LOGE("V4L2_CORE: (VIDIOC_QBUF) Unable to queue buffer %i: %s\n",
                     index, strerror(errno));

            break;
    }

    if (ret < 0)
    {
        return E_QBUF_ERR;
    }

    return E_OK;
}

/*
 * set the v4l2 ctrl value by ctrl id
 * args:
 *   vd - pointer to v4l2 device handler
 *   ctrl_id - control id
 *   value - control value
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code (E_OK)
 */
int v4l2core_ctrl_set(v4l2_dev_t *vd, int ctrl_id, int value)
{
    /*asserts*/
    assert(vd != NULL);

    int ret = E_OK;
    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control ctrl = {0};
    ctrl.id = ctrl_id;
    ctrl.value = value;
    ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl_id);
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrls.reserved[0] = 0;

    ret = xioctl(vd->fd, VIDIOC_S_EXT_CTRLS, &ctrls);

    if (ret < 0)
    {
        LOGE("Failed to set the ctrl. id = 0x%08x, value=%d. error: %s\n",
             ctrl_id, value, strerror(errno));
        return E_UNKNOWN_CID_ERR;
    }

    return ret;
}

/*
 * get the v4l2 ctrl value by ctrl id
 * args:
 *   vd - pointer to v4l2 device handler
 *   ctrl_id - control id
 *   value - control value
 *
 * asserts:
 *   vd is not null
 *
 * returns: error code (E_OK)
 */
int v4l2core_ctrl_get(v4l2_dev_t *vd, int ctrl_id, int *value)
{
    /*asserts*/
    assert(vd != NULL);

    int ret = E_OK;
    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control ctrl = {0};
    ctrl.id = ctrl_id;
    ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl_id);
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrls.reserved[0] = 0;

    ret = xioctl(vd->fd, VIDIOC_G_EXT_CTRLS, &ctrls);

    if (ret < 0)
    {
        LOGE("Failed to get the ctrl. id = 0x%08x. error: %s\n",
             ctrl_id, strerror(errno));
        return E_UNKNOWN_CID_ERR;
    }

    *value = ctrl.value;

    return ret;
}

static std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}

static std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

static std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    return ltrim(rtrim(str, chars), chars);
}

void v4l2core_gen_ctrl_list(v4l2_dev_t *vd, const std::string &str, char delim)
{
    assert(vd != nullptr);

    std::stringstream ss(str);
    std::string token;

    /* Firstly, we need to clean up the list. */
    vd->controls_set_list.clear();

    while (std::getline(ss, token, delim))
    {
        trim(token);
        size_t sepr = token.find_first_of('=');

        if (sepr != std::string::npos)
        {
            const auto &set_ctrl_pair =
                std::pair<std::string, std::string>(token.substr(0, sepr), token.substr(sepr + 1));
            vd->controls_set_list.push_back(set_ctrl_pair);
        }
    }
}

typedef std::map<unsigned, std::vector<struct v4l2_ext_control> > class2ctrls_map;

#define V4L2_CTRL_ID_MASK         (0x0fffffff)
#define V4L2_CTRL_ID2CLASS(id)    ((id) & 0x0fff0000UL)
#define V4L2_CTRL_ID2WHICH(id)    ((id) & 0x0fff0000UL)
#define V4L2_CTRL_DRIVER_PRIV(id) (((id) & 0xffff) >= 0x1000)
#define V4L2_CTRL_MAX_DIMS    (4)
#define V4L2_CTRL_WHICH_CUR_VAL   0
#define V4L2_CTRL_WHICH_DEF_VAL   0x0f000000

void v4l2core_do_ctrl_list_set(v4l2_dev_t *vd)
{
    assert(vd != nullptr);

    if (vd->controls_set_list.empty())
    {
        return;
    }

    struct v4l2_ext_controls ctrls;

    class2ctrls_map class2ctrls;

    bool use_ext_ctrls = false;

    memset(&ctrls, 0, sizeof(ctrls));

    for (auto &iter : vd->controls_set_list)
    {
        struct v4l2_ext_control ctrl;
        v4l2_ctrl_t *vc = get_control_by_name(vd, iter.first);

        if (vc == nullptr)
        {
            continue;
        }

        struct v4l2_queryctrl &qc = vc->control;

        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = qc.id;

        ctrl.value = strtol(iter.second.c_str(), NULL, 0);

        if (qc.type == V4L2_CTRL_TYPE_INTEGER64)
        {
            use_ext_ctrls = true;
        }

        if (V4L2_CTRL_DRIVER_PRIV(ctrl.id))
        {
            use_ext_ctrls = true;
        }

        class2ctrls[V4L2_CTRL_ID2WHICH(ctrl.id)].push_back(ctrl);
    }

    for (class2ctrls_map::iterator iter = class2ctrls.begin();
            iter != class2ctrls.end(); ++iter)
    {
        if (!use_ext_ctrls &&
                (iter->first == V4L2_CTRL_CLASS_USER ||
                 iter->first == V4L2_CID_PRIVATE_BASE))
        {
            for (unsigned i = 0; i < iter->second.size(); i++)
            {
                struct v4l2_control ctrl;

                ctrl.id = iter->second[i].id;
                ctrl.value = iter->second[i].value;

                if (xioctl(vd->fd, VIDIOC_S_CTRL, &ctrl))
                {
                    LOGE("V4L2_CORE: (VIDIOC_S_CTRL) 0x%08x - 0x%x: %s(%d)\n",
                            ctrl.id,
                            ctrl.value,
                            strerror(errno),
                            errno);
                }
            }

            continue;
        }

        for (auto &ctrl : iter->second)
        {
            static __u32 temporal_id = 0;

            if (ctrl.id == V4L2_CID_ENCODER_SELECT_LAYER)
            {
                temporal_id = ((ctrl.value >> 7) & 0x0F);
                continue;
            }

            memset(&ctrls, 0x00, sizeof(ctrls));

            ctrls.which = iter->first;
            ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl.id);
            ctrls.count = 1;

            ctrls.controls = &ctrl;
            ctrls.reserved[0] = temporal_id;

            if (xioctl(vd->fd, VIDIOC_S_EXT_CTRLS, &ctrls))
            {
                if (ctrls.error_idx >= ctrls.count)
                {
                    LOGE("Error setting controls: %s\n", strerror(errno));
                }
                else
                {
                    LOGE("V4L2_CORE: (VIDIOC_S_CTRL) 0x%08x - 0x%x: %s(%d)\n",
                            iter->second[ctrls.error_idx].id,
                            iter->second[ctrls.error_idx].value,
                            strerror(errno),
                            errno);
                }
            }
        }
    }
}
