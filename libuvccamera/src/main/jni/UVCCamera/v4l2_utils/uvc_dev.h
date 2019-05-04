/*
 * uvc_dev.h
 *
 *  Created on: May 6, 2018
 *      Author: jiezhang
 */

#ifndef UVC_DEV_H_
#define UVC_DEV_H_

#include "linux/videodev2.h"
//#include "linux/uvcvideo.h"
#include <pthread.h>
#include <inttypes.h>
#include <sys/types.h>

#include "libuvc.h"

/*
 * buffer number (for driver mmap ops)
 */
#define NB_BUFFER 4

#define __THREAD_TYPE pthread_t
#define __THREAD_CREATE(t,f,d) (pthread_create(t,NULL,f,d))
#define __THREAD_CREATE_ATTRIB(t,a,f,d) (pthread_create(t,a,f,d))
#define __THREAD_JOIN(t) (pthread_join(t, NULL))

#define __ATTRIB_TYPE pthread_attr_t
#define __INIT_ATTRIB(t) (pthread_attr_init(t))
#define __ATTRIB_JOINABLE(t) (pthread_attr_setdetachstate(t, PTHREAD_CREATE_JOINABLE))
#define __CLOSE_ATTRIB(t) (pthread_attr_destroy(t))

#define __MUTEX_TYPE pthread_mutex_t
#define __STATIC_MUTEX_INIT PTHREAD_MUTEX_INITIALIZER
#define __INIT_MUTEX(m) ( pthread_mutex_init(m, NULL) )
#define __CLOSE_MUTEX(m) ( pthread_mutex_destroy(m) )
#define __LOCK_MUTEX(m) ( pthread_mutex_lock(m) )
#define __UNLOCK_MUTEX(m) ( pthread_mutex_unlock(m) )

#define __COND_TYPE pthread_cond_t
#define __INIT_COND(c)  ( pthread_cond_init (c, NULL) )
#define __CLOSE_COND(c) ( pthread_cond_destroy(c) )
#define __COND_BCAST(c) ( pthread_cond_broadcast(c) )
#define __COND_SIGNAL(c) ( pthread_cond_signal(c) )
#define __COND_TIMED_WAIT(c,m,t) ( pthread_cond_timedwait(c,m,t) )

/*
 * Error Codes
 */
#define E_OK                      (0)
#define E_ALLOC_ERR               (-1)
#define E_QUERYCAP_ERR            (-2)
#define E_READ_ERR                (-3)
#define E_MMAP_ERR                (-4)
#define E_QUERYBUF_ERR            (-5)
#define E_QBUF_ERR                (-6)
#define E_DQBUF_ERR               (-7)
#define E_STREAMON_ERR            (-8)
#define E_STREAMOFF_ERR           (-9)
#define E_FORMAT_ERR              (-10)
#define E_REQBUFS_ERR             (-11)
#define E_DEVICE_ERR              (-12)
#define E_SELECT_ERR              (-13)
#define E_SELECT_TIMEOUT_ERR      (-14)
#define E_FBALLOC_ERR             (-15)
#define E_NO_STREAM_ERR           (-16)
#define E_NO_DATA                 (-17)
#define E_NO_CODEC                (-18)
#define E_DECODE_ERR              (-19)
#define E_BAD_TABLES_ERR          (-20)
#define E_NO_SOI_ERR              (-21)
#define E_NOT_8BIT_ERR            (-22)
#define E_BAD_WIDTH_OR_HEIGHT_ERR (-23)
#define E_TOO_MANY_COMPPS_ERR     (-24)
#define E_ILLEGAL_HV_ERR          (-25)
#define E_QUANT_TBL_SEL_ERR       (-26)
#define E_NOT_YCBCR_ERR           (-27)
#define E_UNKNOWN_CID_ERR         (-28)
#define E_WRONG_MARKER_ERR        (-29)
#define E_NO_EOI_ERR              (-30)
#define E_FILE_IO_ERR             (-31)
#define E_UNKNOWN_ERR             (-40)

#ifndef V4L2_PIX_FMT_H264_SIMULCAST
#define V4L2_PIX_FMT_H264_SIMULCAST    v4l2_fourcc('S', '2', '6', '4') /* H264 simulcast */
#endif

/*
 * v4l2 stream capability data
 */
typedef struct _v4l2_stream_cap_t {
    int width;            //width
    int height;           //height
    int *framerate_num; //list of numerator values - should be 1 in almost all cases
    int *framerate_denom; //list of denominator values - gives fps
    int numb_frates; //number of frame rates (numerator and denominator lists size)
    int profile;          //profile for H.264
} v4l2_stream_cap_t;

/*
 * v4l2 stream format data
 */
typedef struct _v4l2_stream_format_t {
    uint8_t dec_support; //decoder support (1-supported; 0-not supported)
    int format;          //v4l2 pixel format
    char fourcc[5];      //corresponding fourcc (mode)
    char description[32];      //format description
    int numb_res; //available number of resolutions for format (v4l2_stream_cap_t list size)
    v4l2_stream_cap_t *list_stream_cap; //list of stream capabilities for format
} v4l2_stream_formats_t;

typedef struct _v4l2_dev_t {
    int fd;             // device file descriptor
    char *videodevice;  // video device string (default "/dev/video0")
    uint8_t stream_id;  // video stream id for simulcasting

    __MUTEX_TYPE mutex;                // device mutex

    int cap_meth;       // capture method: IO_READ or IO_MMAP
    v4l2_stream_formats_t* list_stream_formats; // list of available stream formats
    int numb_formats;

    struct v4l2_capability cap;          // v4l2 capability struct
    struct v4l2_format format;           // v4l2 format struct
    struct v4l2_buffer buf;              // v4l2 buffer struct
    struct v4l2_requestbuffers rb;       // v4l2 request buffers struct
    struct v4l2_streamparm streamparm;   // v4l2 stream parameters struct

    int requested_fmt;                   // requested fomat

    int fps_num;                         // fps numerator
    int fps_denom;                       // fps denominator

    uint8_t streaming; // flag device stream : STRM_STOP ; STRM_REQ_STOP; STRM_OK
    uint64_t frame_index;        // captured frame index from 0 to max(uint64_t)
    void *mem[NB_BUFFER];               // memory buffers for mmap driver frames
    uint32_t buff_length[NB_BUFFER]; // memory buffers length as set by VIDIOC_QUERYBUF
    uint32_t buff_offset[NB_BUFFER]; // memory buffers offset as set by VIDIOC_QUERYBUF

    uvc_frame_callback_t *user_cb;
    void *user_ptr;

    __THREAD_TYPE running_thread;       // stream running thread

} v4l2_dev_t;

#endif /* UVC_DEV_H_ */
