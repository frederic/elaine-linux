/*
 * drivers/amlogic/media_modules/frame_provider/decoder/yuv/vyuv.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/errno.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/amlogic/media/vfm/vframe.h>
#include <linux/amlogic/media/vfm/vframe_provider.h>
#include <linux/amlogic/media/vfm/vframe_receiver.h>
#include <linux/amlogic/media/canvas/canvas.h>

#define VF_POOL_SIZE 4
#define MAX_WIDTH 1920U
#define MAX_HEIGHT 1088U

struct yuvdecoder_buffer {
	struct vb2_buffer vb;
	struct list_head list;
	struct vframe_s vf;
};

struct yuvdecoder_dev {
	struct platform_device *pdev;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;

	struct mutex lock;

	struct vframe_provider_s video_vf_prov;

	bool streaming_on;

	spinlock_t qlock;
	struct list_head buf_list;

	struct v4l2_format fmt;
};

#define DRIVER_NAME "amvdec_yuv"
#define MODULE_NAME "amvdec_yuv"
#define PROVIDER_NAME   "decoder.yuv"

/* ------------------------------------------------------------------
 * vframe_operations_s
 * ------------------------------------------------------------------
 */

static struct vframe_s *yuv_vf_peek(void *);
static struct vframe_s *yuv_vf_get(void *);
static void yuv_vf_put(struct vframe_s *, void *);
static int yuv_vf_states(struct vframe_states *states, void *);
static int yuv_event_cb(int type, void *data, void *private_data);

static const struct vframe_operations_s yuv_vf_provider_ops = {
	.peek = yuv_vf_peek,
	.get = yuv_vf_get,
	.put = yuv_vf_put,
	.event_cb = yuv_event_cb,
	.vf_states = yuv_vf_states,
};

static struct vframe_s *yuv_vf_peek(void *op_arg)
{
	struct yuvdecoder_dev *dev = (struct yuvdecoder_dev *)op_arg;
	struct yuvdecoder_buffer *buf;
	unsigned long flags;

	if (!dev->streaming_on)
		return NULL;

	spin_lock_irqsave(&dev->qlock, flags);

	if (list_empty(&dev->buf_list)) {
		spin_unlock_irqrestore(&dev->qlock, flags);
		return NULL;
	}

	buf = list_first_entry(&dev->buf_list, struct yuvdecoder_buffer, list);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return &buf->vf;
}

static struct vframe_s *yuv_vf_get(void *op_arg)
{
	struct yuvdecoder_dev *dev = (struct yuvdecoder_dev *)op_arg;
	struct yuvdecoder_buffer *buf;
	unsigned long flags;

	if (!dev->streaming_on)
		return NULL;

	spin_lock_irqsave(&dev->qlock, flags);

	if (list_empty(&dev->buf_list)) {
		spin_unlock_irqrestore(&dev->qlock, flags);
		return NULL;
	}

	buf = list_first_entry(&dev->buf_list, struct yuvdecoder_buffer, list);
	list_del(&buf->list);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return &buf->vf;
}

static void yuv_vf_put(struct vframe_s *vf, void *op_arg)
{
	struct yuvdecoder_dev *dev = (struct yuvdecoder_dev *)op_arg;
	struct vb2_buffer *vb = (struct vb2_buffer *)vf->private_data;

	if (!dev->streaming_on)
		return;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
}

static int yuv_event_cb(int type, void *data, void *private_data)
{
	return 0;
}

static int yuv_vf_states(struct vframe_states *states, void *op_arg)
{
	states->vf_pool_size = 0;
	states->buf_free_num = 0;
	states->buf_avail_num = 0;
	states->buf_recycle_num = 0;

	return 0;
}

/* ------------------------------------------------------------------
 * vb2_ops
 * ------------------------------------------------------------------
 */

static int queue_setup(struct vb2_queue *vq,
	unsigned int *nbuffers, unsigned int *nplanes,
	unsigned int sizes[], struct device *alloc_devs[])
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vq);

	if (vq->num_buffers + *nbuffers < VF_POOL_SIZE)
		*nbuffers = VF_POOL_SIZE - vq->num_buffers;

	if (*nplanes) {
		struct v4l2_plane_pix_format *fmt =
			dev->fmt.fmt.pix_mp.plane_fmt;
		if ((*nplanes != 3) ||
			(sizes[0] < fmt[0].sizeimage) ||
			(sizes[1] < fmt[1].sizeimage) ||
			(sizes[2] < fmt[2].sizeimage))
			return -EINVAL;
		else
			return 0;
	}

	*nplanes = 3;
	sizes[0] = dev->fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
	sizes[1] = dev->fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
	sizes[2] = dev->fmt.fmt.pix_mp.plane_fmt[2].sizeimage;

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct yuvdecoder_buffer *buf = container_of(vb,
		struct yuvdecoder_buffer, vb);
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->qlock, flags);
	list_add_tail(&buf->list, &dev->buf_list);
	spin_unlock_irqrestore(&dev->qlock, flags);

	vf_notify_receiver(PROVIDER_NAME,
		VFRAME_EVENT_PROVIDER_VFRAME_READY, NULL);

}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vq);

	dev->streaming_on = true;

	vf_provider_init(&dev->video_vf_prov, PROVIDER_NAME,
		&yuv_vf_provider_ops, dev);
	vf_reg_provider(&dev->video_vf_prov);
	vf_notify_receiver(PROVIDER_NAME, VFRAME_EVENT_PROVIDER_START, NULL);

	return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vq);
	unsigned long flags;

	vf_unreg_provider(&dev->video_vf_prov);

	dev->streaming_on = false;

	spin_lock_irqsave(&dev->qlock, flags);

	while (!list_empty(&dev->buf_list)) {
		struct yuvdecoder_buffer *buf;

		buf = list_entry(dev->buf_list.next, struct yuvdecoder_buffer,
			list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&dev->qlock, flags);
}

static void yuvdecoder_lock(struct vb2_queue *vq)
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vq);

	mutex_lock(&dev->lock);
}

static void yuvdecoder_unlock(struct vb2_queue *vq)
{
	struct yuvdecoder_dev *dev = vb2_get_drv_priv(vq);

	mutex_unlock(&dev->lock);
}

static const struct vb2_ops yuvdecoder_qops = {
	.queue_setup = queue_setup,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = yuvdecoder_unlock,
	.wait_finish = yuvdecoder_lock,
};

/* ------------------------------------------------------------------
 * v4l2_file_operations
 * ------------------------------------------------------------------
 */

static const struct v4l2_file_operations yuvdecoder_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2, /* V4L2 ioctl handler */
	.mmap = vb2_fop_mmap,
};

/* ------------------------------------------------------------------
 * v4l2_ioctl_ops
 * ------------------------------------------------------------------
 */

static int vidioc_querycap(struct file *file, void *priv,
	struct v4l2_capability *cap)
{
	struct yuvdecoder_dev *dev = video_drvdata(file);

	strcpy(cap->driver, "yuvdecoder");
	strcpy(cap->card, "yuvdecoder");
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		"platform:%s", dev->v4l2_dev.name);
	cap->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING |
		V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *priv,
	struct v4l2_fmtdesc *f)
{
	if (f->index >= 1)
		return -EINVAL;

	strlcpy(f->description, "YUV420P", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_YUV420;

	return 0;
}

static int vidioc_g_fmt_vid_out_mplane(struct file *file, void *priv,
	struct v4l2_format *f)
{
	struct yuvdecoder_dev *dev = video_drvdata(file);

	if (dev->fmt.fmt.pix_mp.width == 0 || dev->fmt.fmt.pix_mp.height == 0)
		return -EINVAL;

	*f = dev->fmt;

	return 0;
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
	struct v4l2_format *f)
{
	struct yuvdecoder_dev *dev = video_drvdata(file);
	int i;

	f->fmt.pix_mp.width = min(f->fmt.pix_mp.width, MAX_WIDTH);
	f->fmt.pix_mp.height = min(f->fmt.pix_mp.height, MAX_HEIGHT);
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix_mp.num_planes = 3;

	for (i = 0; i < 3; i++) {
		f->fmt.pix_mp.plane_fmt[i].sizeimage =
			ALIGN(f->fmt.pix_mp.width, 64) *
			f->fmt.pix_mp.height / ((i == 0) ? 1 : 4);
		f->fmt.pix_mp.plane_fmt[i].bytesperline =
			ALIGN(f->fmt.pix_mp.width, 64);
	}

	dev->fmt = *f;

	return 0;
}

static int vidioc_s_fmt_vid_out_mplane(struct file *file, void *priv,
	struct v4l2_format *f) {
	struct yuvdecoder_dev *dev = video_drvdata(file);
	struct vb2_queue *q = &dev->queue;
	int ret;

	if (vb2_is_busy(q))
		return -EBUSY;

	ret = vidioc_try_fmt_vid_out_mplane(file, priv, f);
	if (ret < 0)
		return ret;

	return 0;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct yuvdecoder_dev *dev = video_drvdata(file);
	struct vb2_buffer *vb = dev->queue.bufs[p->index];
	struct yuvdecoder_buffer *buf =
		container_of(vb, struct yuvdecoder_buffer, vb);
	struct vframe_s *vf = &buf->vf;
	int ret;

	if (vf->width == 0) {
		unsigned long addr;
		int canvas_id = 128 + 3 * p->index;
		/* construct vframe */
		vf->index = p->index;
		vf->width = dev->fmt.fmt.pix_mp.width;
		vf->bufWidth = dev->fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
		vf->height = dev->fmt.fmt.pix_mp.height;
		vf->type = VIDTYPE_PROGRESSIVE | VIDTYPE_VIU_FIELD;
		vf->type_original = vf->type;
		vf->canvas0Addr = vf->canvas1Addr =
			(((canvas_id + 2) & 0xff) << 16) |
			(((canvas_id + 1) & 0xff) << 8) |
			(canvas_id & 0xff);
		vf->duration = 3000;
		vf->duration_pulldown = 0;
		vf->signal_type = 0;
		vf->ratio_control = (0x100 * vf->height / vf->width) <<
			DISP_RATIO_ASPECT_RATIO_BIT;
		vf->orientation = 0;
		vf->flag = 0;
		vf->trans_fmt = 0;
		vf->private_data = (void *)(dev->queue.bufs[p->index]);
		vf->mem_handle = 0;

		addr = vb2_dma_contig_plane_dma_addr(vb, 0);
		canvas_config_ex(canvas_id, addr,
			vf->bufWidth, vf->height,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR, 7);
		addr = vb2_dma_contig_plane_dma_addr(vb, 1);
		canvas_config_ex(canvas_id + 1, addr,
			vf->bufWidth / 2, vf->height / 2,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR, 7);
		addr = vb2_dma_contig_plane_dma_addr(vb, 2);
		canvas_config_ex(canvas_id + 2, addr,
			vf->bufWidth / 2, vf->height / 2,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR, 7);
	}

	vf->pts = p->timestamp.tv_usec & 0xFFFFFFFF;
	vf->ready_jiffies64 = jiffies_64;

	ret = vb2_ioctl_qbuf(file, priv, p);
	if (ret)
		return ret;

	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
	struct v4l2_frmsizeenum *fsize)
{
	const struct v4l2_frmsize_stepwise sizes = {
		96, MAX_WIDTH, 16, 96, MAX_HEIGHT, 16 };

	if (fsize->pixel_format != V4L2_PIX_FMT_YUV420)
		return -EINVAL;

	if (fsize->index > 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = sizes;

	return 0;
}

static const struct v4l2_ioctl_ops yuvdecoder_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt_vid_out_mplane,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt_vid_out_mplane,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt_vid_out_mplane,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------------
 * platform device
 * ------------------------------------------------------------------
 */

static int amvdec_yuv_probe(struct platform_device *pdev)
{
	return 0;
}

static int amvdec_yuv_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver amvdec_yuv_driver = {
	.probe = amvdec_yuv_probe,
	.remove = amvdec_yuv_remove,
	.driver = {
		.name = "amvdec_yuvdec",
	}
};

static int amvdec_yuvdecoder_probe(struct platform_device *pdev)
{
	struct yuvdecoder_dev *dev;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(struct yuvdecoder_dev),
		GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->pdev = pdev;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto free_dev;

	mutex_init(&dev->lock);

	/* initialize the vb2 queue */
	q = &dev->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	q->io_modes = VB2_MMAP;
	q->dev = &pdev->dev;
	q->drv_priv = dev;
	q->buf_struct_size = sizeof(struct yuvdecoder_buffer);
	q->ops = &yuvdecoder_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 0;
	q->lock = &dev->lock;
	q->gfp_flags = GFP_DMA32;

	ret = vb2_queue_init(q);
	if (ret)
		goto free_hdl;

	INIT_LIST_HEAD(&dev->buf_list);
	spin_lock_init(&dev->qlock);

	/* initialize video_device */
	vdev = &dev->vdev;
	strlcpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->fops = &yuvdecoder_fops;
	vdev->ioctl_ops = &yuvdecoder_ioctl_ops;
	vdev->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING;
	vdev->lock = &dev->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_TX;

	video_set_drvdata(vdev, dev);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto free_hdl;

	dev_info(&pdev->dev, "yubdecoder V4L2 device registered\n");
	return 0;

free_hdl:
	v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
	return ret;
}

static int amvdec_yuvdecoder_remove(struct platform_device *pdev)
{
	struct yuvdecoder_dev *dev = platform_get_drvdata(pdev);

	video_unregister_device(&dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static const struct of_device_id amvdec_yuvdecoder_of_match[] = {
	{ .compatible = "amlogic, amvdec_yuv" },
	{}
};

static struct platform_driver amvdec_yuvdecoder_driver = {
	.probe = amvdec_yuvdecoder_probe,
	.remove = amvdec_yuvdecoder_remove,
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "yuvdecoder",
		.of_match_table = amvdec_yuvdecoder_of_match,
	},
};

static int __init amvdec_yuvdecoder_driver_init(void)
{
	int r;

	r = platform_driver_register(&(amvdec_yuv_driver));
	if (r)
		return r;
	r = platform_driver_register(&(amvdec_yuvdecoder_driver));
	if (r)
		return r;
	return 0;
}

static void __exit amvdec_yuvdecoder_driver_exit(void)
{
	platform_driver_unregister(&(amvdec_yuv_driver));
	platform_driver_unregister(&(amvdec_yuvdecoder_driver));
}

module_init(amvdec_yuvdecoder_driver_init);
module_exit(amvdec_yuvdecoder_driver_exit);

MODULE_DEVICE_TABLE(of, amvdec_yuvdecoder_of_match);

MODULE_DESCRIPTION("AMLOGIC YUV Video Decoder Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <tim.yao@amlogic.com>");
