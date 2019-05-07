/*
 * v4l2_video_formats.h
 *
 *  Created on: May 6, 2018
 *      Author: jiezhang
 */

#ifndef V4L2_VIDEO_FORMATS_H_
#define V4L2_VIDEO_FORMATS_H_

#include "uvc_dev.h"

/*
 * check pixelformat against decoder support formats
 * args:
 *    pixelformat - v4l2 pixelformat
 *
 * asserts:
 *    none
 *
 * returns: TRUE(1) if format is supported
 *          FALSE(0) if not
 */
uint8_t can_decode_format(uint32_t pixelformat);

/*
 * enumerate frame formats (pixelformats, resolutions and fps)
 * and creates list in vd->list_stream_formats
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *   vd->list_stream_formats is null
 *
 * returns: 0 (E_OK) if enumeration succeded or error otherwise
 */
int enum_frame_formats(v4l2_dev_t *vd);

/* get frame format index from format list
 * args:
 *   vd - pointer to video device data
 *   format - v4l2 pixel format
 *
 * asserts:
 *   vd is not null
 *   vd->list_stream_formats is not null
 *
 * returns: format list index or -1 if not available
 */
int get_frame_format_index(v4l2_dev_t *vd, int format);

/* get resolution index for format index from format list
 * args:
 *   vd - pointer to video device data
 *   format - format index from format list
 *   width - requested width
 *   height - requested height
 *   profile - requested profile
 *
 * asserts:
 *   vd is not null
 *   vd->list_stream_formats is not null
 *
 * returns: resolution list index for format index or -1 if not available
 */
int get_format_resolution_index(v4l2_dev_t *vd, int format, int width,
                                int height, int profile);

/*
 * free frame formats list
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *   vd->list_stream_formats is not null
 *
 * returns: void
 */
void free_frame_formats(v4l2_dev_t *vd);

#endif /* V4L2_VIDEO_FORMATS_H_ */
