/*
 * v4l2_video_formats.c
 *
 *  Created on: May 6, 2018
 *      Author: jiezhang
 */

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <time.h>

#include <sstream>

#include "linux/videodev2.h"
#include "v4l2_video_formats.h"

#include "utilbase.h"

extern int verbosity;

extern int xioctl(int fd, int IOCTL_X, void *arg);

static uint32_t decoder_supported_formats[] =
{
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_MJPEG,
    V4L2_PIX_FMT_JPEG,
    V4L2_PIX_FMT_H264,
    V4L2_PIX_FMT_YVYU,
    V4L2_PIX_FMT_UYVY,
    V4L2_PIX_FMT_VYUY,
    V4L2_PIX_FMT_YYUV,
    V4L2_PIX_FMT_YUV444,
    V4L2_PIX_FMT_YUV555,
    V4L2_PIX_FMT_YUV565,
    V4L2_PIX_FMT_YUV32,
    V4L2_PIX_FMT_Y41P,
    V4L2_PIX_FMT_GREY,
    V4L2_PIX_FMT_Y10BPACK,
    V4L2_PIX_FMT_Y16,
#ifdef V4L2_PIX_FMT_Y16_BE
    V4L2_PIX_FMT_Y16_BE,
#endif
    V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_YUV422P,
    V4L2_PIX_FMT_YVU420,
    V4L2_PIX_FMT_NV12,
    V4L2_PIX_FMT_NV21,
    V4L2_PIX_FMT_NV16,
    V4L2_PIX_FMT_NV61,
    V4L2_PIX_FMT_NV24,
    V4L2_PIX_FMT_NV42,
    V4L2_PIX_FMT_SPCA501,
    V4L2_PIX_FMT_SPCA505,
    V4L2_PIX_FMT_SPCA508,
    V4L2_PIX_FMT_SGBRG8,
    V4L2_PIX_FMT_SGRBG8,
    V4L2_PIX_FMT_SBGGR8,
    V4L2_PIX_FMT_SRGGB8,
    V4L2_PIX_FMT_RGB24,
    V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_RGB332,
    V4L2_PIX_FMT_RGB565,
    V4L2_PIX_FMT_RGB565X,
    V4L2_PIX_FMT_RGB444,
    V4L2_PIX_FMT_RGB555,
    V4L2_PIX_FMT_RGB555X,
    V4L2_PIX_FMT_BGR666,
    V4L2_PIX_FMT_BGR32,
    V4L2_PIX_FMT_RGB32,
#ifdef V4L2_PIX_FMT_ARGB444
    V4L2_PIX_FMT_ARGB444,
    V4L2_PIX_FMT_XRGB444,
#endif
#ifdef V4L2_PIX_FMT_ARGB555
    V4L2_PIX_FMT_ARGB555,
    V4L2_PIX_FMT_XRGB555,
#endif
#ifdef V4L2_PIX_FMT_ARGB555X
    V4L2_PIX_FMT_ARGB555X,
    V4L2_PIX_FMT_XRGB555X,
#endif
#ifdef V4L2_PIX_FMT_ABGR32
    V4L2_PIX_FMT_ABGR32,
    V4L2_PIX_FMT_XBGR32,
#endif
#ifdef V4L2_PIX_FMT_ARGB32
    V4L2_PIX_FMT_ARGB32,
    V4L2_PIX_FMT_XRGB32,
#endif
#ifdef V4L2_PIX_FMT_H264_SIMULCAST
    V4L2_PIX_FMT_H264_SIMULCAST,
#endif
    /*last one (zero terminated)*/
    0
};

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
static uint8_t can_decode_format(uint32_t pixelformat)
{
    int i = 0;
    uint32_t sup_fmt = 0;

    do
    {
        sup_fmt = decoder_supported_formats[i];

        if (pixelformat == sup_fmt)
        {
            return 1;
        }

        i++;
    }
    while (sup_fmt);   /*last format is always 0*/

    return 0;
}

/*
 * enumerate frame intervals (fps)
 * args:
 *   vd - pointer to video device data
 *   pixfmt - v4l2 pixel format that we want to list frame intervals for
 *   width - video width that we want to list frame intervals for
 *   height - video height that we want to list frame intervals for
 *   fmtind - current index of format list
 *   fsizeind - current index of frame size list
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *   fmtind is valid
 *   fsizeind is valid
 *
 * returns 0 if enumeration succeded or errno otherwise
 */
static int enum_frame_intervals(v4l2_dev_t *vd, uint32_t pixfmt, uint32_t width,
                                uint32_t height, int fmtind, int fsizeind)
{
    /*assertions*/
    assert(vd != NULL);
    assert(vd->fd > 0);
    assert(vd->stream_formats.size() >= fmtind);
    assert(vd->stream_formats[fmtind - 1].stream_caps.size() >= fsizeind);

    int ret = 0;
    std::ostringstream intervals_oss;

    struct v4l2_frmivalenum fival;
    int discrete = 0;
    memset(&fival, 0, sizeof(fival));
    fival.index = 0;
    fival.pixel_format = pixfmt;
    fival.width = width;
    fival.height = height;

    vd->stream_formats[fmtind - 1].stream_caps[fsizeind - 1].framerates.clear();

    if (verbosity > 0)
    {
        LOGI("\tTime interval between frame: ");
    }

    while ((ret = xioctl(vd->fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
    {
        fival.index++;

        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
        {
            discrete = 1;

            if (verbosity > 0)
            {
                intervals_oss << fival.discrete.numerator << '/' << fival.discrete.denominator << ", ";
            }

            vd->stream_formats[fmtind - 1].stream_caps[fsizeind - 1].framerates.push_back(std::pair<int, int>(fival.discrete.numerator,
                    fival.discrete.denominator));
        }
        else if (fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS)
        {
            if (verbosity > 0)
                LOGI("{min { %u/%u } .. max { %u/%u } }, ",
                     fival.stepwise.min.numerator,
                     fival.stepwise.min.numerator,
                     fival.stepwise.max.denominator,
                     fival.stepwise.max.denominator);

            break;
        }
        else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE)
        {
            if (verbosity > 0)
                LOGI("{min { %u/%u } .. max { %u/%u } / "
                     "stepsize { %u/%u } }, ", fival.stepwise.min.numerator,
                     fival.stepwise.min.denominator,
                     fival.stepwise.max.numerator,
                     fival.stepwise.max.denominator,
                     fival.stepwise.step.numerator,
                     fival.stepwise.step.denominator);

            break;
        }
    }

    if (discrete && verbosity > 0)
    {
        LOGI("{ %s }\n", intervals_oss.str().c_str());
    }

    if (vd->stream_formats[fmtind - 1].stream_caps[fsizeind - 1].framerates.empty())
    {
        vd->stream_formats[fmtind - 1].stream_caps[fsizeind - 1].framerates.push_back(
            std::pair<int, int>(1, 1));
    }

    if (verbosity > 0)
    {
        LOGI("\n");
    }

    if (ret != 0 && errno != EINVAL)
    {
        LOGE("V4L2_CORE: (VIDIOC_ENUM_FRAMEINTERVALS) Error enumerating frame intervals: %s(%d)\n",
             strerror(errno), errno);
        return -1;
    }

    return 0;
}

/*
 * enumerate frame sizes (width and height)
 * args:
 *   vd - pointer to video device data
 *   pixfmt - v4l2 pixel format that we want to list frame sizes for
 *   fmtind - current index of format list
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *   fmtind is valid
 *
 * returns 0 if enumeration succeded or errno otherwise
 */
static int enum_frame_sizes(v4l2_dev_t *vd, uint32_t pixfmt, int fmtind)
{
    /*assertions*/
    assert(vd != NULL);
    assert(vd->fd > 0);
    assert(vd->stream_formats.size() >= fmtind);

    int ret = 0;
    int fsizeind = 0; /*index for supported sizes*/
    vd->stream_formats[fmtind - 1].stream_caps.clear();

    struct v4l2_frmsizeenum fsize;
    memset(&fsize, 0, sizeof(fsize));
    fsize.index = 0;
    fsize.pixel_format = pixfmt;

    while ((ret = xioctl(vd->fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
    {
        fsize.index++;

        if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
        {
            if (verbosity > 0)
                LOGI("{ discrete: width = %u, height = %u }\n",
                     fsize.discrete.width, fsize.discrete.height);

            v4l2_stream_cap_t stream_cap;

            stream_cap.width = fsize.discrete.width;
            stream_cap.height = fsize.discrete.height;
            stream_cap.profile = fsize.reserved[0];

            fsizeind++;

            vd->stream_formats[fmtind - 1].stream_caps.push_back(stream_cap);

            ret = enum_frame_intervals(vd, pixfmt, stream_cap.width,
                                       stream_cap.height, fmtind, fsizeind);

            if (ret != 0)
                LOGE("V4L2_CORE:  Unable to enumerate frame sizes %s\n",
                     strerror(ret));
        }
        else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS
                 || fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
        {
            if (verbosity > 0)
            {
                if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
                    LOGI("{ continuous: min { width = %u, height = %u } .. "
                         "max { width = %u, height = %u } }\n",
                         fsize.stepwise.min_width, fsize.stepwise.min_height,
                         fsize.stepwise.max_width,
                         fsize.stepwise.max_height);
                else
                    LOGI("{ stepwise: min { width = %u, height = %u } .. "
                         "max { width = %u, height = %u } / "
                         "stepsize { width = %u, height = %u } }\n",
                         fsize.stepwise.min_width, fsize.stepwise.min_height,
                         fsize.stepwise.max_width, fsize.stepwise.max_height,
                         fsize.stepwise.step_width,
                         fsize.stepwise.step_height);
            }

            /*add at least min and max values*/
            fsizeind++; /*min*/

            v4l2_stream_cap_t stream_cap;

            stream_cap.width = fsize.stepwise.min_width;
            stream_cap.height = fsize.stepwise.min_height;

            vd->stream_formats[fmtind - 1].stream_caps.push_back(stream_cap);

            ret = enum_frame_intervals(vd, pixfmt, stream_cap.width,
                                       stream_cap.height, fmtind, fsizeind);

            if (ret != 0)
                LOGE("V4L2_CORE:  Unable to enumerate frame sizes %s\n",
                     strerror(ret));

            fsizeind++; /*max*/

            stream_cap.width = fsize.stepwise.max_width;
            stream_cap.height = fsize.stepwise.max_height;

            vd->stream_formats[fmtind - 1].stream_caps.push_back(stream_cap);

            ret = enum_frame_intervals(vd, pixfmt, stream_cap.width,
                                       stream_cap.height, fmtind, fsizeind);

            if (ret != 0)
                LOGE("V4L2_CORE:  Unable to enumerate frame sizes %s\n",
                     strerror(ret));

        }
        else
        {
            LOGE("V4L2_CORE: fsize.type not supported: %d\n",
                 fsize.type);
            LOGE("    (Discrete: %d   Continuous: %d  Stepwise: %d)\n",
                 V4L2_FRMSIZE_TYPE_DISCRETE, V4L2_FRMSIZE_TYPE_CONTINUOUS,
                 V4L2_FRMSIZE_TYPE_STEPWISE);
        }
    }

    if (ret != 0 && errno != EINVAL)
    {
        LOGE("V4L2_CORE: (VIDIOC_ENUM_FRAMESIZES) - Error enumerating frame sizes\n");
        return errno;
    }
    else if ((ret != 0) && (fsizeind == 0))
    {
        /* ------ some drivers don't enumerate frame sizes ------ */
        /*         negotiate with VIDIOC_TRY_FMT instead          */

        fsizeind++;
        struct v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = vd->format.fmt.pix.width; /* check defined size*/
        fmt.fmt.pix.height = vd->format.fmt.pix.height;
        fmt.fmt.pix.pixelformat = pixfmt;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        xioctl(vd->fd, VIDIOC_TRY_FMT, &fmt);

        /*use the returned values*/
        int width = fmt.fmt.pix.width;
        int height = fmt.fmt.pix.height;

        if (width <= 0 || height <= 0)
        {
            LOGI(
                "{ VIDIOC_TRY_FMT (invalid values): width = %u, height = %u }\n",
                vd->format.fmt.pix.width, vd->format.fmt.pix.height);
            return EINVAL;
        }

        if (verbosity > 0)
        {
            LOGI("{ VIDIOC_TRY_FMT : width = %u, height = %u }\n",
                 vd->format.fmt.pix.width, vd->format.fmt.pix.height);
            LOGI("fmtind:%i fsizeind: %i\n", fmtind, fsizeind);
        }

        assert(vd->stream_formats[fmtind - 1].stream_caps.empty());

        /*don't enumerateintervals, use a default of 1/25 fps instead*/
        v4l2_stream_cap_t stream_cap;

        stream_cap.width = width;
        stream_cap.height = height;
        stream_cap.framerates.push_back(std::pair<int, int>(1, 25));
    }

    return 0;
}

/*
 * enumerate frame formats (pixelformats, resolutions and fps)
 * and creates container in vd->stream_formats
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *   vd->fd is valid ( > 0 )
 *   vd->stream_formats is empty
 *
 * returns: 0 (E_OK) if enumeration succeded or error otherwise
 */
int enum_frame_formats(v4l2_dev_t *vd)
{
    /*assertions*/
    assert(vd != NULL);
    assert(vd->fd > 0);
    assert(vd->stream_formats.empty());

    int ret = E_OK;

    int fmtind = 0;
    int valid_formats = 0; /*number of valid formats found (with valid frame sizes)*/
    struct v4l2_fmtdesc fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while ((ret = xioctl(vd->fd, VIDIOC_ENUM_FMT, &fmt)) == 0)
    {
        uint8_t dec_support = can_decode_format(fmt.pixelformat);

        uint32_t pix_format = fmt.pixelformat;

        fmt.index++;

        /* We can not choose the H.264 format in Simulcast sub videodev */
        if (vd->stream_id && pix_format == V4L2_PIX_FMT_H264)
        {
            continue;
        }

        if (verbosity > 0)
        {
            if ((fmt.pixelformat & (1 << 31)) != 0)
            {
                pix_format &= ~(1 << 31); //need to fix fourcc string
                LOGI("{ pixelformat = '%c%c%c%c'(BE), description = '%s' }\n",
                     pix_format & 0xFF, (pix_format >> 8) & 0xFF,
                     (pix_format >> 16) & 0xFF, (pix_format >> 24) & 0xFF,
                     fmt.description);
            }
            else
                LOGI("{ pixelformat = '%c%c%c%c', description = '%s' }\n",
                     fmt.pixelformat & 0xFF, (fmt.pixelformat >> 8) & 0xFF,
                     (fmt.pixelformat >> 16) & 0xFF,
                     (fmt.pixelformat >> 24) & 0xFF, fmt.description);

        }

        if (!dec_support)
        {
            LOGI("    - FORMAT NOT SUPPORTED BY DECODER -\n");
        }

        fmtind++;

        _v4l2_stream_format_t stream_format;

        stream_format.dec_support = dec_support;
        stream_format.format = fmt.pixelformat;

        if ((fmt.pixelformat & (1 << 31)) != 0) //be format flag
        {
            pix_format &= ~(1 << 31);    //need to fix fourcc string
        }

        stream_format.fourcc =
            stringFormat("%c%c%c%c",
                         pix_format & 0xFF, (pix_format >> 8) & 0xFF,
                         (pix_format >> 16) & 0xFF, (pix_format >> 24) & 0xFF);
        stream_format.description = reinterpret_cast<char *>(fmt.description);

        vd->stream_formats.push_back(stream_format);

        //enumerate frame sizes
        ret = enum_frame_sizes(vd, fmt.pixelformat, fmtind);

        if (ret != 0)
            LOGE("v4L2_CORE: Unable to enumerate frame sizes :%s\n",
                 strerror(ret));

        if (dec_support && !ret)
        {
            valid_formats++;    /*the format can be decoded and it has valid frame sizes*/
        }
    }

    if (errno != EINVAL)
        LOGE("v4L2_CORE: (VIDIOC_ENUM_FMT) - Error enumerating frame formats: %s\n",
             strerror(errno));

    if (valid_formats > 0)
    {
        return E_OK;
    }
    else
    {
        return E_DEVICE_ERR;
    }
}

/* get frame format index from format list
 * args:
 *   vd - pointer to video device data
 *   format - v4l2 pixel format
 *
 * asserts:
 *   vd is not null
 *
 * returns: format list index or -1 if not available
 */
int get_frame_format_index(v4l2_dev_t *vd, int format)
{
    /*asserts*/
    assert(vd != NULL);

    int i = 0;

    for (const auto &stream_format : vd->stream_formats)
    {
        if (format == stream_format.format)
        {
            return (i);
        }

        i++;
    }

    return (-1);
}

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
 *
 * returns: resolution list index for format index or -1 if not available
 */
int get_format_resolution_index(v4l2_dev_t *vd, int format, int width,
                                int height, int profile)
{
    /*asserts*/
    assert(vd != NULL);

    if (format >= vd->stream_formats.size() || format < 0)
    {
        LOGE("V4L2_CORE: [get resolution index] format index (%i) is not valid [0 - %i]\n",
             format, vd->stream_formats.size() - 1);
        return (-1);
    }

    int i = 0;
    int found_idx = -1;
    int resolution_matched_idx = -1;

    for (const auto &stream_cap : vd->stream_formats[format].stream_caps)
    {
        if (width == stream_cap.width && height == stream_cap.height)
        {
            resolution_matched_idx = i;

            if (profile == 0)
            {
                /* Non H264 or H264_Simulcast formats. */
                return (i);
            }
            else if (profile
                     == vd->stream_formats[format].stream_caps[i].profile)
            {
                /* Find the matched profile. */
                return (i);
            }
            else if ((profile & 0xFF00)
                     == (((vd->stream_formats[format].stream_caps[i].profile)) & 0xFF00))
            {
                /* Found the partially matched profile.
                 * We will continue.
                 */
                found_idx = i;
                i++;
                continue;
            }
        }

        i++;
    }

    if (found_idx < 0 && resolution_matched_idx >= 0)
    {
        LOGI("Not found a fully or partially matched profile.\n");
        found_idx = resolution_matched_idx;
    }

    return (found_idx);
}

/*
 * free frame formats list
 * args:
 *   vd - pointer to video device data
 *
 * asserts:
 *   vd is not null
 *
 * returns: void
 */
void free_frame_formats(v4l2_dev_t *vd)
{
    /*asserts*/
    assert(vd != NULL);

    vd->stream_formats.clear();
}
