/*
 * v4l2_core.h
 *
 *  Created on: May 6, 2018
 *      Author: jiezhang
 */

#ifndef V4L2_CORE_H_
#define V4L2_CORE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "uvc_dev.h"

/*
 * set ioctl retries to 4
 */
#define IOCTL_RETRY 4

/*  Four-character-code (FOURCC) */
#ifndef v4l2_fourcc
#define v4l2_fourcc(a, b, c, d)\
    ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#endif

#define v4l2_pixformat2fourcc(pixelformat)\
    {  pixelformat & 0x000000FF,\
        (pixelformat >> 8) & 0x000000FF,\
        (pixelformat >> 16) & 0x000000FF,\
        (pixelformat >> 24) & 0x000000FF,\
        0\
    }

#ifndef V4L2_PIX_FMT_H264_SIMULCAST
#define V4L2_PIX_FMT_H264_SIMULCAST    v4l2_fourcc('S', '2', '6', '4') /* H264 simulcast */
#endif

/*
 * stream status codes
 */
#define STRM_STOP        (0)
#define STRM_REQ_STOP    (1)
#define STRM_OK          (2)

/*
 * IO methods
 */
#define IO_MMAP 1
#define IO_READ 2

/*
 * min()/max()/clamp() macros
 */
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

#define clamp(val, lo, hi) min(max(val, lo), hi)

#ifndef V4L2_CID_ENCODER_START
/* Controls found in UVC 1.5 encoding cameras */
#define V4L2_CID_ENCODER_START              (V4L2_CID_CAMERA_CLASS_BASE+34)
#define V4L2_CID_ENCODER_MIN_FRAME_INTERVAL (V4L2_CID_CAMERA_CLASS_BASE+34)
#define V4L2_CID_ENCODER_RATE_CONTROL_MODE  (V4L2_CID_CAMERA_CLASS_BASE+35)
#define V4L2_CID_ENCODER_AVERAGE_BITRATE    (V4L2_CID_CAMERA_CLASS_BASE+36)
#define V4L2_CID_ENCODER_CPB_SIZE       (V4L2_CID_CAMERA_CLASS_BASE+37)
#define V4L2_CID_ENCODER_PEAK_BIT_RATE      (V4L2_CID_CAMERA_CLASS_BASE+38)
#define V4L2_CID_ENCODER_QP_PARAM_I     (V4L2_CID_CAMERA_CLASS_BASE+39)
#define V4L2_CID_ENCODER_QP_PARAM_P     (V4L2_CID_CAMERA_CLASS_BASE+40)
#define V4L2_CID_ENCODER_QP_PARAM_BG        (V4L2_CID_CAMERA_CLASS_BASE+41)
#define V4L2_CID_ENCODER_NUM_GDR_FRAMES     (V4L2_CID_CAMERA_CLASS_BASE+42)
#define V4L2_CID_ENCODER_LTR_BUFFER_CONTROL (V4L2_CID_CAMERA_CLASS_BASE+43)
#define V4L2_CID_ENCODER_LTR_BUFFER_TRUST_MODE  (V4L2_CID_CAMERA_CLASS_BASE+44)
#define V4L2_CID_ENCODER_LTR_PICTURE_POSITION   (V4L2_CID_CAMERA_CLASS_BASE+45)
#define V4L2_CID_ENCODER_LTR_PICTURE_MODE   (V4L2_CID_CAMERA_CLASS_BASE+46)
#define V4L2_CID_ENCODER_LTR_VALIDATION     (V4L2_CID_CAMERA_CLASS_BASE+47)
#define V4L2_CID_ENCODER_MIN_QP         (V4L2_CID_CAMERA_CLASS_BASE+48)
#define V4L2_CID_ENCODER_MAX_QP         (V4L2_CID_CAMERA_CLASS_BASE+49)
#define V4L2_CID_ENCODER_SYNC_FRAME_INTERVAL    (V4L2_CID_CAMERA_CLASS_BASE+50)
#define V4L2_CID_ENCODER_ERROR_RESILIENCY   (V4L2_CID_CAMERA_CLASS_BASE+51)
#define V4L2_CID_ENCODER_TEMPORAL_LAYER_ENABLE  (V4L2_CID_CAMERA_CLASS_BASE+52)

/* VP8-specific controls */
#define V4L2_CID_ENCODER_VP8_SLICE_MODE     (V4L2_CID_CAMERA_CLASS_BASE+53)
#define V4L2_CID_ENCODER_VP8_DCT_PARTS_PER_FRAME (V4L2_CID_CAMERA_CLASS_BASE+54)
#define V4L2_CID_ENCODER_VP8_SYNC_FRAME_TYPE    (V4L2_CID_CAMERA_CLASS_BASE+55)

/* H.264-specific controls */
#define V4L2_CID_ENCODER_H264_PROFILE_TOOLSET   (V4L2_CID_CAMERA_CLASS_BASE+56)
#define V4L2_CID_ENCODER_H264_LEVEL_IDC_LIMIT   (V4L2_CID_CAMERA_CLASS_BASE+57)
#define V4L2_CID_ENCODER_H264_SEI_PAYLOAD_TYPE  (V4L2_CID_CAMERA_CLASS_BASE+58)
#define V4L2_CID_ENCODER_H264_LAYER_PRIORITY    (V4L2_CID_CAMERA_CLASS_BASE+59)

/* more UVC 1.5 controls */
#define V4L2_CID_ENCODER_SELECT_LAYER           (V4L2_CID_CAMERA_CLASS_BASE+60)
#define V4L2_CID_ENCODER_VIDEO_RESOLUTION       (V4L2_CID_CAMERA_CLASS_BASE+61)
#define V4L2_CID_ENCODER_H264_SETTINGS          (V4L2_CID_CAMERA_CLASS_BASE+62)
#define V4L2_CID_ENCODER_H264_SLICE_MODE        (V4L2_CID_CAMERA_CLASS_BASE+63)

#define V4L2_CID_ENCODER_END                    (V4L2_CID_CAMERA_CLASS_BASE+63)
#endif

static inline uint16_t generate_layout_structure(
    int ucConfig,
    uint16_t stream_0_layout,
    uint16_t stream_1_layout,
    uint16_t stream_2_layout,
    uint16_t stream_3_layout)
{
    uint16_t layout = 0;

    if (ucConfig == 0) {
        stream_0_layout = clamp(stream_0_layout, 0 , 1);
        stream_1_layout = clamp(stream_1_layout, 0 , 1);
        stream_2_layout = clamp(stream_2_layout, 0 , 1);
        stream_3_layout = clamp(stream_3_layout, 0 , 1);
    } else if (ucConfig == 1) {
        stream_0_layout = clamp(stream_0_layout, 0 , 2);
        stream_1_layout = clamp(stream_1_layout, 0 , 2);
        stream_2_layout = clamp(stream_2_layout, 0 , 2);
        stream_3_layout = clamp(stream_3_layout, 0 , 2);
    }

    layout |= ((stream_0_layout & 0x07) |
               ((stream_1_layout & 0x07) << 3) |
               ((stream_2_layout & 0x07) << 6) |
               ((stream_3_layout & 0x07) << 9));

    return layout;
}

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
 * Initiate video device handler with default values
 * args:
 *   device - device name (e.g: "/dev/video0")
 *
 * asserts:
 *   device is not null
 *
 * returns: pointer to v4l2 device handler (or NULL on error)
 */
v4l2_dev_t* v4l2core_init_dev(const char *device);

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
void v4l2core_close_dev(v4l2_dev_t *vd);

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
int v4l2core_start_stream(v4l2_dev_t *vd);
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
int v4l2core_stop_stream(v4l2_dev_t *vd);

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
void v4l2core_prepare_new_format(v4l2_dev_t *vd, int new_format);

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
void v4l2core_prepare_new_format_index(v4l2_dev_t *vd, int format_index);

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
void v4l2core_prepare_new_resolution(v4l2_dev_t *vd,
                                     int new_width, int new_height,
                                     int new_profile, int new_ucconfig,
                                     int new_stream_0_layout,
                                     int new_stream_1_layout,
                                     int new_stream_2_layout,
                                     int new_stream_3_layout);

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
int v4l2core_update_current_format(v4l2_dev_t *vd);

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
void v4l2core_define_fps(v4l2_dev_t *vd, int num, int denom);

void v4l2core_set_frame_callback(v4l2_dev_t *vd, 
        uvc_frame_callback_t *cb, void *user_ptr);

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
    int v4l2core_clean_buffers(v4l2_dev_t *vd);

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
int v4l2core_get_frame(v4l2_dev_t *vd, void **frame_data, uint32_t *frame_size);

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
int v4l2core_release_frame(v4l2_dev_t *vd, int index);

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
int v4l2core_ctrl_set(v4l2_dev_t *vd, int ctrl_id, int value);

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
int v4l2core_ctrl_get(v4l2_dev_t *vd, int ctrl_id, int *value);

#ifdef __cplusplus
}
#endif

#endif /* V4L2_CORE_H_ */
