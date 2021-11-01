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
#include "aml_hevc_parser.h"

#define NAL_TYPE(value)				((value) & 0x1F)
#define HEADER_BUFFER_SIZE			(32 * 1024)

/**
 * struct hevc_fb - hevc decode frame buffer information
 * @vdec_fb_va  : virtual address of struct vdec_fb
 * @y_fb_dma    : dma address of Y frame buffer (luma)
 * @c_fb_dma    : dma address of C frame buffer (chroma)
 * @poc         : picture order count of frame buffer
 * @reserved    : for 8 bytes alignment
 */
struct hevc_fb {
	uint64_t vdec_fb_va;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	int32_t poc;
	uint32_t reserved;
};

/**
 * struct vdec_hevc_dec_info - decode information
 * @dpb_sz		: decoding picture buffer size
 * @resolution_changed  : resoltion change happen
 * @reserved		: for 8 bytes alignment
 * @bs_dma		: Input bit-stream buffer dma address
 * @y_fb_dma		: Y frame buffer dma address
 * @c_fb_dma		: C frame buffer dma address
 * @vdec_fb_va		: VDEC frame buffer struct virtual address
 */
struct vdec_hevc_dec_info {
	uint32_t dpb_sz;
	uint32_t resolution_changed;
	uint32_t reserved;
	uint64_t bs_dma;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	uint64_t vdec_fb_va;
};

/**
 * struct vdec_hevc_vsi - shared memory for decode information exchange
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
struct vdec_hevc_vsi {
	char *header_buf;
	int sps_size;
	int pps_size;
	int sei_size;
	int head_offset;
	struct vdec_hevc_dec_info dec;
	struct vdec_pic_info pic;
	struct v4l2_rect crop;
	bool is_combine;
	int nalu_pos;
	struct HEVCParamSets ps;
};

/**
 * struct vdec_hevc_inst - hevc decoder instance
 * @num_nalu : how many nalus be decoded
 * @ctx      : point to aml_vcodec_ctx
 * @vsi      : VPU shared information
 */
struct vdec_hevc_inst {
	unsigned int num_nalu;
	struct aml_vcodec_ctx *ctx;
	struct aml_vdec_adapt vdec;
	struct vdec_hevc_vsi *vsi;
	struct vcodec_vfm_s vfm;
};

#if 0
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

static void get_pic_info(struct vdec_hevc_inst *inst,
			 struct vdec_pic_info *pic)
{
	*pic = inst->vsi->pic;

	aml_vcodec_debug(inst, "pic(%d, %d), buf(%d, %d)",
			 pic->visible_width, pic->visible_height,
			 pic->coded_width, pic->coded_height);
	aml_vcodec_debug(inst, "Y(%d, %d), C(%d, %d)", pic->y_bs_sz,
			 pic->y_len_sz, pic->c_bs_sz, pic->c_len_sz);
}

static void get_crop_info(struct vdec_hevc_inst *inst, struct v4l2_rect *cr)
{
	cr->left = inst->vsi->crop.left;
	cr->top = inst->vsi->crop.top;
	cr->width = inst->vsi->crop.width;
	cr->height = inst->vsi->crop.height;

	aml_vcodec_debug(inst, "l=%d, t=%d, w=%d, h=%d",
			 cr->left, cr->top, cr->width, cr->height);
}

static void get_dpb_size(struct vdec_hevc_inst *inst, unsigned int *dpb_sz)
{
	*dpb_sz = 20;//inst->vsi->dec.dpb_sz;
	aml_vcodec_debug(inst, "sz=%d", *dpb_sz);
}

static int find_start_code(unsigned char *data, unsigned int data_sz)
{
	if (data_sz > 3 && data[0] == 0 && data[1] == 0 && data[2] == 1)
		return 3;

	if (data_sz > 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 &&
	    data[3] == 1)
		return 4;

	return -1;
}

static int vdec_hevc_init(struct aml_vcodec_ctx *ctx, unsigned long *h_vdec)
{
	struct vdec_hevc_inst *inst = NULL;
	int ret = -1;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;

	inst->vdec.format = VFORMAT_HEVC;
	inst->vdec.dev	= ctx->dev->vpu_plat_dev;
	inst->vdec.filp	= ctx->dev->filp;
	inst->vdec.ctx	= ctx;

	/* set play mode.*/
	if (ctx->is_drm_mode)
		inst->vdec.port.flag |= PORT_FLAG_DRM;

	/* to eable hevc hw.*/
	inst->vdec.port.type = PORT_TYPE_HEVC;

	/* init vfm */
	inst->vfm.ctx	= ctx;
	inst->vfm.ada_ctx = &inst->vdec;
	vcodec_vfm_init(&inst->vfm);

	ret = video_decoder_init(&inst->vdec);
	if (ret) {
		aml_vcodec_err(inst, "vdec_hevc init err=%d", ret);
		goto error_free_inst;
	}

	/* probe info from the stream */
	inst->vsi = kzalloc(sizeof(struct vdec_hevc_vsi), GFP_KERNEL);
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

	aml_vcodec_debug(inst, "hevc Instance >> %p", inst);

	ctx->ada_ctx = &inst->vdec;
	*h_vdec = (unsigned long)inst;

	//dump_init();

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

//static void fill_vdec_params(struct vdec_hevc_inst *inst, struct hevc_SPS_t *sps)
static void fill_vdec_params(struct vdec_hevc_inst *inst)
{
	struct vdec_pic_info *pic = &inst->vsi->pic;
	struct vdec_hevc_dec_info *dec = &inst->vsi->dec;
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

static int hevc_parse_nal_header(u32 val)
{
	if (val & 0x80) {
		pr_err("the nal data is invalid.\n");
		return -1;
	}

	return (val & 0x7f) >> 1;
}

static void hevc_parse(struct HEVCParamSets *ps, u8 *buf, u32 size)
{
	//int ret = -1;
	int i = 0, j = 0, cnt = 0;
	int pos, nal_size, nal_type;
	u8 *p = buf;

	for (i = 4; i < size; i++) {
		j = find_start_code(p, i);
		if (j > 0) {
			pos = p - buf + j;
			nal_size = size - pos;
			nal_type = hevc_parse_nal_header(buf[pos]);

			switch (nal_type) {
			case HEVC_NAL_VPS:
				//ret = hevc_parse_nal_vps(&ps->vps, p, nal_size);
				//if (!ret)
					//ps->vps_parsed = true;
				pr_err("------%s,%d nal type: %u\n",__func__, __LINE__,nal_type);
				break;
			case HEVC_NAL_SPS:
				//ret = hevc_parse_nal_sps(&ps->sps, p, nal_size);
				//if (!ret)
					ps->sps_parsed = true;
				pr_err("------%s,%d nal type: %u\n",__func__, __LINE__,nal_type);
				break;
			case HEVC_NAL_PPS:
				//ret = hevc_parse_nal_pps(&ps->pps, p, nal_size);
				//if (!ret)
					//ps->sps_parsed = true;
				pr_err("------%s,%d nal type: %u\n",__func__, __LINE__,nal_type);
				break;
			case HEVC_NAL_SEI_PREFIX:
			case HEVC_NAL_SEI_SUFFIX:
				//ret = hevc_parse_nal_sei(&ps->sei, p, nal_size);
				//if (!ret)
					//ps->sei_parsed = true;
				pr_err("------%s,%d nal type: %u\n",__func__, __LINE__,nal_type);
				break;
			default:
				pr_info("ignoring NAL type %d in extradata\n", nal_type);
				break;
			}

			cnt++;
			p += j;
			i = pos;
		}
		p++;
	}
}

static int stream_parse(struct vdec_hevc_inst *inst, u8 *buf, u32 size)
{
	//struct hevc_stream_t s;
	//struct hevc_SPS_t *sps;
	//unsigned int nal_type;
	int nal_idx = 0;
	int real_data_pos, real_data_size;
	bool is_combine = false;

	hevc_parse(&inst->vsi->ps, buf, size);

	if (!inst->vsi->ps.sps_parsed)
		return -1;

	/* if the st compose from csd + slice that is the combine data. */
	inst->vsi->is_combine = is_combine;
	inst->vsi->nalu_pos = nal_idx;

	/* start code plus nal type. */
	real_data_pos = nal_idx + 1;
	real_data_size = size - real_data_pos;

	//sps = kzalloc(sizeof(struct hevc_SPS_t), GFP_KERNEL);
	//if (sps == NULL)
		//return -ENOMEM;

	/* the extra data would be parsed. */
	//hevc_stream_set(&s, &buf[real_data_pos], real_data_size);
	//hevc_sps_parse(&s, sps);
	//hevc_sps_info(sps);

	//fill_vdec_params(inst, sps);
	fill_vdec_params(inst);

	//kfree(sps);

	return 0;
}

static int vdec_hevc_probe(unsigned long h_vdec,
	struct aml_vcodec_mem *bs, void *out)
{
	struct vdec_hevc_inst *inst =
		(struct vdec_hevc_inst *)h_vdec;
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

static void vdec_hevc_deinit(unsigned long h_vdec)
{
	struct vdec_hevc_inst *inst = (struct vdec_hevc_inst *)h_vdec;

	if (!inst)
		return;

	aml_vcodec_debug_enter(inst);

	video_decoder_release(&inst->vdec);

	vcodec_vfm_release(&inst->vfm);

	//dump_deinit();

	if (inst->vsi && inst->vsi->header_buf)
		vfree(inst->vsi->header_buf);

	if (inst->vsi)
		kfree(inst->vsi);

	kfree(inst);
}

static int vdec_hevc_get_fb(struct vdec_hevc_inst *inst, struct vdec_fb **out)
{
	return get_fb_from_queue(inst->ctx, out);
}

static void vdec_hevc_get_vf(struct vdec_hevc_inst *inst, struct vdec_fb **out)
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

static int vdec_write_nalu(struct vdec_hevc_inst *inst,
	u8 *buf, u32 size, u64 ts)
{
	int ret = 0, err = 0;
	struct aml_vdec_adapt *vdec = &inst->vdec;
	int nalu_pos;
	u32 nal_type;
	char *write_buf;
	nalu_pos = find_start_code(buf, size);
	if (nalu_pos < 0)
		goto err;

	nal_type = hevc_parse_nal_header(buf[nalu_pos]);
	aml_vcodec_debug(inst, "NALU type: %d, size: %u\n", nal_type, size);

	write_buf = vmalloc(inst->vsi->head_offset + size);

	memcpy(write_buf, inst->vsi->header_buf, inst->vsi->head_offset);
	memcpy(write_buf + inst->vsi->head_offset, buf, size);

	ret = vdec_vframe_write(vdec, write_buf,
		inst->vsi->head_offset + size, ts);

	aml_vcodec_debug(inst, "buf: %p, buf size: %u, write to: %d",
		write_buf, inst->vsi->head_offset + size, ret);

	memset(inst->vsi->header_buf, 0, HEADER_BUFFER_SIZE);
	inst->vsi->head_offset = 0;
	inst->vsi->sps_size = 0;
	inst->vsi->pps_size = 0;
	inst->vsi->sei_size = 0;

	vfree(write_buf);

	return 0;
err:
	aml_vcodec_err(inst, "%s err(%d)", __func__, err);

	return err;
}

static int vdec_hevc_decode(unsigned long h_vdec, struct aml_vcodec_mem *bs,
			 unsigned long int timestamp, bool *res_chg)
{
	struct vdec_hevc_inst *inst = (struct vdec_hevc_inst *)h_vdec;
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

static int vdec_hevc_get_param(unsigned long h_vdec,
			       enum vdec_get_param_type type, void *out)
{
	int ret = 0;
	struct vdec_hevc_inst *inst = (struct vdec_hevc_inst *)h_vdec;

	if (!inst) {
		pr_err("the hevc inst of dec is invalid.\n");
		return -1;
	}

	switch (type) {
	case GET_PARAM_DISP_FRAME_BUFFER:
		vdec_hevc_get_vf(inst, out);
		break;

	case GET_PARAM_FREE_FRAME_BUFFER:
		ret = vdec_hevc_get_fb(inst, out);
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

static struct vdec_common_if vdec_hevc_if = {
	vdec_hevc_init,
	vdec_hevc_probe,
	vdec_hevc_decode,
	vdec_hevc_get_param,
	vdec_hevc_deinit,
};

struct vdec_common_if *get_hevc_dec_comm_if(void);

struct vdec_common_if *get_hevc_dec_comm_if(void)
{
	return &vdec_hevc_if;
}
