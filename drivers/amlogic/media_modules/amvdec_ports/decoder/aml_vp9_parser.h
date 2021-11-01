/*
 * drivers/amvdec_ports/decoder/aml_vp9_parser.h
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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


#ifndef AML_VP9_PARSER_H
#define AML_VP9_PARSER_H

enum BlockPartition {
    PARTITION_NONE,    // [ ] <-.
    PARTITION_H,       // [-]   |
    PARTITION_V,       // [|]   |
    PARTITION_SPLIT,   // [+] --'
};

enum InterPredMode {
    NEARESTMV = 10,
    NEARMV    = 11,
    ZEROMV    = 12,
    NEWMV     = 13,
};

enum CompPredMode {
    PRED_SINGLEREF,
    PRED_COMPREF,
    PRED_SWITCHABLE,
};

enum BlockLevel {
    BL_64X64,
    BL_32X32,
    BL_16X16,
    BL_8X8,
};

enum BlockSize {
    BS_64x64,
    BS_64x32,
    BS_32x64,
    BS_32x32,
    BS_32x16,
    BS_16x32,
    BS_16x16,
    BS_16x8,
    BS_8x16,
    BS_8x8,
    BS_8x4,
    BS_4x8,
    BS_4x4,
    N_BS_SIZES,
};

enum FilterMode {
    MODE_NONE,
    MODE_INTERLEAVE,
    MODE_DEINTERLEAVE
};

enum TxfmMode {
    TX_4X4,
    TX_8X8,
    TX_16X16,
    TX_32X32,
    N_TXFM_SIZES,
    TX_SWITCHABLE = N_TXFM_SIZES,
    N_TXFM_MODES
};

enum TxfmType {
    DCT_DCT,
    DCT_ADST,
    ADST_DCT,
    ADST_ADST,
    N_TXFM_TYPES
};

enum IntraPredMode {
    VERT_PRED,
    HOR_PRED,
    DC_PRED,
    DIAG_DOWN_LEFT_PRED,
    DIAG_DOWN_RIGHT_PRED,
    VERT_RIGHT_PRED,
    HOR_DOWN_PRED,
    VERT_LEFT_PRED,
    HOR_UP_PRED,
    TM_VP8_PRED,
    LEFT_DC_PRED,
    TOP_DC_PRED,
    DC_128_PRED,
    DC_127_PRED,
    DC_129_PRED,
    N_INTRA_PRED_MODES
};

struct VP9BitstreamHeader {
    // bitstream header
    u8 profile;
    u8 bpp;
    u8 keyframe;
    u8 invisible;
    u8 errorres;
    u8 intraonly;
    u8 resetctx;
    u8 refreshrefmask;
    u8 highprecisionmvs;
    enum FilterMode filtermode;
    u8 allowcompinter;
    u8 refreshctx;
    u8 parallelmode;
    u8 framectxid;
    u8 use_last_frame_mvs;
    u8 refidx[3];
    u8 signbias[3];
    u8 fixcompref;
    u8 varcompref[2];
    struct {
        u8 level;
        int8_t sharpness;
    } filter;
    struct {
        u8 enabled;
        u8 updated;
        char mode[2];
        char ref[4];
    } lf_delta;
    u8 yac_qi;
    char ydc_qdelta, uvdc_qdelta, uvac_qdelta;
    u8 lossless;
#define MAX_SEGMENT 8
    struct {
        u8 enabled;
        u8 temporal;
        u8 absolute_vals;
        u8 update_map;
        u8 prob[7];
        u8 pred_prob[3];
        struct {
            u8 q_enabled;
            u8 lf_enabled;
            u8 ref_enabled;
            u8 skip_enabled;
            u8 ref_val;
            s16 q_val;
            char lf_val;
            s16 qmul[2][2];
            u8 lflvl[4][2];
        } feat[MAX_SEGMENT];
    } segmentation;
    enum TxfmMode txfmmode;
    enum CompPredMode comppredmode;
    struct {
        u32 log2_tile_cols, log2_tile_rows;
        u32 tile_cols, tile_rows;
    } tiling;

    int uncompressed_header_size;
    int compressed_header_size;
};

struct vp9_head_info_t {
	bool parsed;
	struct VP9BitstreamHeader info;
};

#endif //AML_VP9_PARSER_H
