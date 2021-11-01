/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*
* Description:
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <uapi/linux/swab.h>
#include "../vdec_drv_if.h"
#include "../aml_vcodec_util.h"
#include "../aml_vcodec_dec.h"
#include "../aml_vcodec_adapt.h"
#include "../vdec_drv_base.h"
#include "../aml_vcodec_vfm.h"
#include "aml_vp9_parser.h"
#include "vdec_vp9_trigger.h"

#define PREFIX_SIZE	(16)

#define NAL_TYPE(value)				((value) & 0x1F)
#define HEADER_BUFFER_SIZE			(32 * 1024)

bool need_trigger;
int dump_cnt = 0;

/**
 * struct vp9_fb - vp9 decode frame buffer information
 * @vdec_fb_va  : virtual address of struct vdec_fb
 * @y_fb_dma    : dma address of Y frame buffer (luma)
 * @c_fb_dma    : dma address of C frame buffer (chroma)
 * @poc         : picture order count of frame buffer
 * @reserved    : for 8 bytes alignment
 */
struct vp9_fb {
	uint64_t vdec_fb_va;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	int32_t poc;
	uint32_t reserved;
};

/**
 * struct vdec_vp9_dec_info - decode information
 * @dpb_sz		: decoding picture buffer size
 * @resolution_changed  : resoltion change happen
 * @reserved		: for 8 bytes alignment
 * @bs_dma		: Input bit-stream buffer dma address
 * @y_fb_dma		: Y frame buffer dma address
 * @c_fb_dma		: C frame buffer dma address
 * @vdec_fb_va		: VDEC frame buffer struct virtual address
 */
struct vdec_vp9_dec_info {
	uint32_t dpb_sz;
	uint32_t resolution_changed;
	uint32_t reserved;
	uint64_t bs_dma;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	uint64_t vdec_fb_va;
};

/**
 * struct vdec_vp9_vsi - shared memory for decode information exchange
 *                        between VPU and Host.
 *                        The memory is allocated by VPU then mapping to Host
 *                        in vpu_dec_init() and freed in vpu_dec_deinit()
 *                        by VPU.
 *                        AP-W/R : AP is writer/reader on this item
 *                        VPU-W/R: VPU is write/reader on this item
 * @hdr_buf      : Header parsing buffer (AP-W, VPU-R)
 * @list_free    : free frame buffer ring list (AP-W/R, VPU-W)
 * @list_disp    : display frame buffer ring list (AP-R, VPU-W)
 * @dec          : decode information (AP-R, VPU-W)
 * @pic          : picture information (AP-R, VPU-W)
 * @crop         : crop information (AP-R, VPU-W)
 */
struct vdec_vp9_vsi {
	char *header_buf;
	int sps_size;
	int pps_size;
	int sei_size;
	int head_offset;
	struct vdec_vp9_dec_info dec;
	struct vdec_pic_info pic;
	struct v4l2_rect crop;
	bool is_combine;
	int nalu_pos;
	struct vp9_head_info_t head;
};

/**
 * struct vdec_vp9_inst - vp9 decoder instance
 * @num_nalu : how many nalus be decoded
 * @ctx      : point to aml_vcodec_ctx
 * @vsi      : VPU shared information
 */
struct vdec_vp9_inst {
	unsigned int num_nalu;
	struct aml_vcodec_ctx *ctx;
	struct aml_vdec_adapt vdec;
	struct vdec_vp9_vsi *vsi;
	struct vcodec_vfm_s vfm;
};

struct vp9_superframe_split {
	/*in data*/
	u8 *data;
	u32 data_size;

	/*out data*/
	int nb_frames;
	int size;
	int next_frame;
	u32 next_frame_offset;
	int sizes[8];
};

#if 1
#define DUMP_FILE_NAME "/data/dump/dump.tmp"
static struct file *filp;
static loff_t file_pos;

void dump_write(const char __user *buf, size_t count)
{
	mm_segment_t old_fs;

	if (!filp)
		return;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (count != vfs_write(filp, buf, count, &file_pos))
		pr_err("Failed to write file\n");

	set_fs(old_fs);
}

void dump_init(void)
{
	filp = filp_open(DUMP_FILE_NAME, O_CREAT | O_RDWR, 0644);
	if (IS_ERR(filp)) {
		pr_err("open dump file failed\n");
		filp = NULL;
	}
}

void dump_deinit(void)
{
	if (filp) {
		filp_close(filp, current->files);
		filp = NULL;
		file_pos = 0;
	}
}

void swap_uv(void *uv, int size)
{
	int i;
	__u16 *p = uv;

	size /= 2;

	for (i = 0; i < size; i++, p++)
		*p = __swab16(*p);
}
#endif

static void get_pic_info(struct vdec_vp9_inst *inst,
			 struct vdec_pic_info *pic)
{
	*pic = inst->vsi->pic;

	aml_vcodec_debug(inst, "pic(%d, %d), buf(%d, %d)",
			 pic->visible_width, pic->visible_height,
			 pic->coded_width, pic->coded_height);
	aml_vcodec_debug(inst, "Y(%d, %d), C(%d, %d)", pic->y_bs_sz,
			 pic->y_len_sz, pic->c_bs_sz, pic->c_len_sz);
}

static void get_crop_info(struct vdec_vp9_inst *inst, struct v4l2_rect *cr)
{
	cr->left = inst->vsi->crop.left;
	cr->top = inst->vsi->crop.top;
	cr->width = inst->vsi->crop.width;
	cr->height = inst->vsi->crop.height;

	aml_vcodec_debug(inst, "l=%d, t=%d, w=%d, h=%d",
			 cr->left, cr->top, cr->width, cr->height);
}

static void get_dpb_size(struct vdec_vp9_inst *inst, unsigned int *dpb_sz)
{
	*dpb_sz = 20;//inst->vsi->dec.dpb_sz;
	aml_vcodec_debug(inst, "sz=%d", *dpb_sz);
}

static int vdec_vp9_init(struct aml_vcodec_ctx *ctx, unsigned long *h_vdec)
{
	struct vdec_vp9_inst *inst = NULL;
	int ret = -1;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;

	inst->vdec.format = VFORMAT_VP9;
	inst->vdec.dev	= ctx->dev->vpu_plat_dev;
	inst->vdec.filp	= ctx->dev->filp;
	inst->vdec.ctx	= ctx;

	/* set play mode.*/
	if (ctx->is_drm_mode)
		inst->vdec.port.flag |= PORT_FLAG_DRM;

	/* to eable vp9 hw.*/
	inst->vdec.port.type = PORT_TYPE_HEVC;

	/* init vfm */
	inst->vfm.ctx	= ctx;
	inst->vfm.ada_ctx = &inst->vdec;
	vcodec_vfm_init(&inst->vfm);

	/* probe info from the stream */
	inst->vsi = kzalloc(sizeof(struct vdec_vp9_vsi), GFP_KERNEL);
	if (!inst->vsi) {
		ret = -ENOMEM;
		goto error_free_inst;
	}

	/* alloc the header buffer to be used cache sps or spp etc.*/
	inst->vsi->header_buf = vzalloc(HEADER_BUFFER_SIZE);
	if (!inst->vsi) {
		ret = -ENOMEM;
		goto error_free_vsi;
	}

	inst->vsi->pic.visible_width	= 1920;
	inst->vsi->pic.visible_height	= 1080;
	inst->vsi->pic.coded_width	= 1920;
	inst->vsi->pic.coded_height	= 1088;
	inst->vsi->pic.y_bs_sz	= 0;
	inst->vsi->pic.y_len_sz	= (1920 * 1088);
	inst->vsi->pic.c_bs_sz	= 0;
	inst->vsi->pic.c_len_sz	= (1920 * 1088 / 2);

	aml_vcodec_debug(inst, "vp9 Instance >> %p", inst);

	ctx->ada_ctx = &inst->vdec;
	*h_vdec = (unsigned long)inst;

	/* init decoder. */
	ret = video_decoder_init(&inst->vdec);
	if (ret) {
		aml_vcodec_err(inst, "vdec_vp9 init err=%d", ret);
		goto error_free_inst;
	}

	dump_init();

	return 0;

error_free_vsi:
	kfree(inst->vsi);
error_free_inst:
	kfree(inst);
	*h_vdec = 0;

	return ret;
}

#if 0
static int refer_buffer_num(int level_idc, int poc_cnt,
	int mb_width, int mb_height)
{
	return 20;
}
#endif

static void fill_vdec_params(struct vdec_vp9_inst *inst)
{
	struct vdec_pic_info *pic = &inst->vsi->pic;
	struct vdec_vp9_dec_info *dec = &inst->vsi->dec;
	struct v4l2_rect *rect = &inst->vsi->crop;
	unsigned int mb_w = 0, mb_h = 0, width, height;
	//unsigned int crop_unit_x = 0, crop_unit_y = 0;
	//unsigned int poc_cnt = 0;

	/* calc width & height. */
	width = 1920;
	height = 1080;

	/* fill visible area size that be used for EGL. */
	pic->visible_width	= width;
	pic->visible_height	= height;

	/* calc visible ares. */
	rect->left		= 0;
	rect->top		= 0;
	rect->width		= pic->visible_width;
	rect->height		= pic->visible_height;

	/* config canvas size that be used for decoder. */
	pic->coded_width	= ALIGN(mb_w, 4) << 4;
	pic->coded_height	= ALIGN(mb_h, 4) << 4;

	pic->coded_width = 1920;
	pic->coded_height = 1088;//temp

	pic->y_len_sz		= pic->coded_width * pic->coded_height;
	pic->c_len_sz		= pic->y_len_sz >> 1;

	/* calc DPB size */
	dec->dpb_sz = 20;//refer_buffer_num(sps->level_idc, poc_cnt, mb_w, mb_h);

	pr_info("[%d] The stream infos, coded:(%d x %d), visible:(%d x %d), DPB: %d\n",
		inst->ctx->id, pic->coded_width, pic->coded_height,
		pic->visible_width, pic->visible_height, dec->dpb_sz);
}

#if 0
static int vp9_parse_nal_header(u32 val)
{
	if (val & 0x80) {
		pr_err("the nal data is invalid.\n");
		return -1;
	}

	return (val & 0x7f) >> 1;
}
#endif

static void vp9_parse(struct vp9_head_info_t *head, u8 *buf, u32 size)
{
	//int ret = -1;
	//u8 *p = buf;

	head->parsed = true;

	return;
}

static int stream_parse(struct vdec_vp9_inst *inst, u8 *buf, u32 size)
{
	//struct vp9_stream_t s;
	//struct vp9_SPS_t *sps;
	//unsigned int nal_type;
	int nal_idx = 0;
	int real_data_pos, real_data_size;
	bool is_combine = false;

	vp9_parse(&inst->vsi->head, buf, size);

	if (!inst->vsi->head.parsed)
		return -1;

	/* if the st compose from csd + slice that is the combine data. */
	inst->vsi->is_combine = is_combine;
	inst->vsi->nalu_pos = nal_idx;

	/* start code plus nal type. */
	real_data_pos = nal_idx + 1;
	real_data_size = size - real_data_pos;

	//sps = kzalloc(sizeof(struct vp9_SPS_t), GFP_KERNEL);
	//if (sps == NULL)
		//return -ENOMEM;

	/* the extra data would be parsed. */
	//vp9_stream_set(&s, &buf[real_data_pos], real_data_size);
	//vp9_sps_parse(&s, sps);
	//vp9_sps_info(sps);

	//fill_vdec_params(inst, sps);
	fill_vdec_params(inst);

	//kfree(sps);

	return 0;
}

static int vdec_vp9_probe(unsigned long h_vdec,
	struct aml_vcodec_mem *bs, void *out)
{
	struct vdec_vp9_inst *inst =
		(struct vdec_vp9_inst *)h_vdec;
	struct stream_info *st;
	u8 *buf = (u8 *)bs->va;
	u32 size = bs->size;
	int ret = 0;

	st = (struct stream_info *)buf;
	if (inst->ctx->is_drm_mode && (st->magic == DRMe || st->magic == DRMn))
		return 0;

	if (st->magic == NORe || st->magic == NORn)
		ret = stream_parse(inst, st->data, st->length);
	else
		ret = stream_parse(inst, buf, size);

	return ret;
}

static void vdec_vp9_deinit(unsigned long h_vdec)
{
	struct vdec_vp9_inst *inst = (struct vdec_vp9_inst *)h_vdec;

	if (!inst)
		return;

	aml_vcodec_debug_enter(inst);

	video_decoder_release(&inst->vdec);

	vcodec_vfm_release(&inst->vfm);

	dump_deinit();

	if (inst->vsi && inst->vsi->header_buf)
		vfree(inst->vsi->header_buf);

	if (inst->vsi)
		kfree(inst->vsi);

	kfree(inst);
	need_trigger = false;
	dump_cnt = 0;
}

static int vdec_vp9_get_fb(struct vdec_vp9_inst *inst, struct vdec_fb **out)
{
	return get_fb_from_queue(inst->ctx, out);
}

static void vdec_vp9_get_vf(struct vdec_vp9_inst *inst, struct vdec_fb **out)
{
	struct vframe_s *vf = NULL;
	struct vdec_fb *fb = NULL;

	aml_vcodec_debug(inst, "%s() [%d], vfm: %p",
		__func__, __LINE__, &inst->vfm);

	vf = peek_video_frame(&inst->vfm);
	if (!vf) {
		aml_vcodec_debug(inst, "there is no vframe.");
		*out = NULL;
		return;
	}

	vf = get_video_frame(&inst->vfm);
	if (!vf) {
		aml_vcodec_debug(inst, "the vframe is avalid.");
		*out = NULL;
		return;
	}

	atomic_set(&vf->use_cnt, 1);

	aml_vcodec_debug(inst, "%s() [%d], vf: %p, v4l_mem_handle: %lx, idx: %d\n",
		__func__, __LINE__, vf, vf->v4l_mem_handle, vf->index);

	fb = (struct vdec_fb *)vf->v4l_mem_handle;
	fb->vf_handle = (unsigned long)vf;
	fb->status = FB_ST_DISPLAY;

	*out = fb;

	//pr_info("%s, %d\n", __func__, fb->base_y.bytes_used);
	//dump_write(fb->base_y.va, fb->base_y.bytes_used);
	//dump_write(fb->base_c.va, fb->base_c.bytes_used);

	/* convert yuv format. */
	//swap_uv(fb->base_c.va, fb->base_c.size);

	aml_vcodec_debug(inst, "%s() [%d], va: %p, phy: %x, size: %zu",
		__func__, __LINE__, fb->base_y.va,
		(unsigned int)virt_to_phys(fb->base_y.va), fb->base_y.size);
	aml_vcodec_debug(inst, "%s() [%d], va: %p, phy: %x, size: %zu",
		__func__, __LINE__, fb->base_c.va,
		(unsigned int)virt_to_phys(fb->base_c.va), fb->base_c.size);
}

static int vp9_superframe_split_filter(struct vp9_superframe_split *s)
{
	int i, j, ret, marker;
	bool is_superframe = false;

	if (!s->data)
		return -1;

	marker = s->data[s->data_size - 1];
	if ((marker & 0xe0) == 0xc0) {
		int length_size = 1 + ((marker >> 3) & 0x3);
		int   nb_frames = 1 + (marker & 0x7);
		int    idx_size = 2 + nb_frames * length_size;

		if (s->data_size >= idx_size &&
			s->data[s->data_size - idx_size] == marker) {
			s64 total_size = 0;
			int idx = s->data_size + 1 - idx_size;

			for (i = 0; i < nb_frames; i++) {
				int frame_size = 0;
				for (j = 0; j < length_size; j++)
					frame_size |= s->data[idx++] << (j * 8);

				total_size += frame_size;
				if (frame_size < 0 ||
					total_size > s->data_size - idx_size) {
					pr_err( "Invalid frame size in a sframe: %d\n",
						frame_size);
					ret = -EINVAL;
					goto fail;
				}
				s->sizes[i] = frame_size;
			}

			s->nb_frames         = nb_frames;
			s->size              = total_size;
			s->next_frame        = 0;
			s->next_frame_offset = 0;
			is_superframe        = true;
		}
	}else {
		s->nb_frames = 1;
		s->sizes[0]  = s->data_size;
		s->size      = s->data_size;
	}

	/*pr_info("sframe: %d, frames: %d, IN: %x, OUT: %x\n",
		is_superframe, s->nb_frames,
		s->data_size, s->size);*/

	/* parse uncompressed header. */
	if (is_superframe) {
		/* bitstream profile. */
		/* frame type. (intra or inter) */
		/* colorspace descriptor */
		/* ... */

		pr_info("the frame is a superframe.\n");
	}

	/*pr_err("in: %x, %d, out: %x, sizes %d,%d,%d,%d,%d,%d,%d,%d\n",
		s->data_size,
		s->nb_frames,
		s->size,
		s->sizes[0],
		s->sizes[1],
		s->sizes[2],
		s->sizes[3],
		s->sizes[4],
		s->sizes[5],
		s->sizes[6],
		s->sizes[7]);*/

	return 0;
fail:
	return ret;
}

static void add_prefix_data(struct vp9_superframe_split *s,
	u8 **out, u32 *out_size)
{
	int i;
	u8 *p = NULL;
	u32 length;

	length = s->size + s->nb_frames * PREFIX_SIZE;
	p = vzalloc(length);
	if (!p) {
		pr_err("alloc size %d failed.\n" ,length);
		return;
	}

	memcpy(p, s->data, s->size);
	p += s->size;

	for (i = s->nb_frames; i > 0; i--) {
		u32 frame_size = s->sizes[i - 1];
		u8 *prefix = NULL;

		p -= frame_size;
		memmove(p + PREFIX_SIZE * i, p, frame_size);
		prefix = p + PREFIX_SIZE * (i - 1);

		/*add amlogic frame headers.*/
		frame_size += 4;
		prefix[0]  = (frame_size >> 24) & 0xff;
		prefix[1]  = (frame_size >> 16) & 0xff;
		prefix[2]  = (frame_size >> 8 ) & 0xff;
		prefix[3]  = (frame_size >> 0 ) & 0xff;
		prefix[4]  = ((frame_size >> 24) & 0xff) ^ 0xff;
		prefix[5]  = ((frame_size >> 16) & 0xff) ^ 0xff;
		prefix[6]  = ((frame_size >> 8 ) & 0xff) ^ 0xff;
		prefix[7]  = ((frame_size >> 0 ) & 0xff) ^ 0xff;
		prefix[8]  = 0;
		prefix[9]  = 0;
		prefix[10] = 0;
		prefix[11] = 1;
		prefix[12] = 'A';
		prefix[13] = 'M';
		prefix[14] = 'L';
		prefix[15] = 'V';
		frame_size -= 4;
	}

	*out = p;
	*out_size = length;
}

static void trigger_decoder(struct aml_vdec_adapt *vdec)
{
	int i, ret;
	u32 frame_size = 0;
	u8 *p = vp9_trigger_header;

	for (i = 0; i < ARRAY_SIZE(vp9_trigger_framesize); i++) {
		frame_size = vp9_trigger_framesize[i];
		ret = vdec_vframe_write(vdec, p,
			frame_size, 0);
		pr_err("write trigger frame %d\n", ret);
		p += frame_size;
	}
}

static int vdec_write_nalu(struct vdec_vp9_inst *inst,
	u8 *buf, u32 size, u64 ts)
{
	int ret = 0;
	struct aml_vdec_adapt *vdec = &inst->vdec;
	struct vp9_superframe_split s;
	u8 *data = NULL;
	u32 length = 0;

	memset(&s, 0, sizeof(s));

	/*trigger.*/
	if (0 && !need_trigger) {
		trigger_decoder(vdec);
		need_trigger = true;
	}

	/*parse superframe.*/
	s.data = buf;
	s.data_size = size;
	ret = vp9_superframe_split_filter(&s);
	if (ret) {
		pr_err("parse frames failed.\n");
		return ret;
	}

	/*add headers.*/
	add_prefix_data(&s, &data, &length);

	ret = vdec_vframe_write(vdec, data, length, ts);

	aml_vcodec_debug(inst, "buf: %p, buf size: %u, write to: %d",
		data, length, ret);

	vfree(data);

	return 0;
}

static int vdec_vp9_decode(unsigned long h_vdec, struct aml_vcodec_mem *bs,
			 unsigned long int timestamp, bool *res_chg)
{
	struct vdec_vp9_inst *inst = (struct vdec_vp9_inst *)h_vdec;
	struct aml_vdec_adapt *vdec = &inst->vdec;
	struct stream_info *st;
	u8 *buf;
	u32 size;
	int ret = 0;

	/* bs NULL means flush decoder */
	if (bs == NULL)
		return 0;

	buf = (u8 *)bs->va;
	size = bs->size;
	st = (struct stream_info *)buf;

	if (inst->ctx->is_drm_mode && (st->magic == DRMe || st->magic == DRMn))
		ret = vdec_vbuf_write(vdec, st->m.buf, sizeof(st->m.drm));
	else if (st->magic == NORe)
		ret = vdec_vbuf_write(vdec, st->data, st->length);
	else if (st->magic == NORn)
		ret = vdec_write_nalu(inst, st->data, st->length, timestamp);
	else if (inst->ctx->is_stream_mode)
		ret = vdec_vbuf_write(vdec, buf, size);
	else
		ret = vdec_write_nalu(inst, buf, size, timestamp);

	return ret;
}

static int vdec_vp9_get_param(unsigned long h_vdec,
			       enum vdec_get_param_type type, void *out)
{
	int ret = 0;
	struct vdec_vp9_inst *inst = (struct vdec_vp9_inst *)h_vdec;

	if (!inst) {
		pr_err("the vp9 inst of dec is invalid.\n");
		return -1;
	}

	switch (type) {
	case GET_PARAM_DISP_FRAME_BUFFER:
		vdec_vp9_get_vf(inst, out);
		break;

	case GET_PARAM_FREE_FRAME_BUFFER:
		ret = vdec_vp9_get_fb(inst, out);
		break;

	case GET_PARAM_PIC_INFO:
		get_pic_info(inst, out);
		break;

	case GET_PARAM_DPB_SIZE:
		get_dpb_size(inst, out);
		break;

	case GET_PARAM_CROP_INFO:
		get_crop_info(inst, out);
		break;

	default:
		aml_vcodec_err(inst, "invalid get parameter type=%d", type);
		ret = -EINVAL;
	}

	return ret;
}

static struct vdec_common_if vdec_vp9_if = {
	vdec_vp9_init,
	vdec_vp9_probe,
	vdec_vp9_decode,
	vdec_vp9_get_param,
	vdec_vp9_deinit,
};

struct vdec_common_if *get_vp9_dec_comm_if(void);

struct vdec_common_if *get_vp9_dec_comm_if(void)
{
	return &vdec_vp9_if;
}

