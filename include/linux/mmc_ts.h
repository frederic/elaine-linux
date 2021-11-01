/*
 * MMC-based transactional key-value store
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */
#ifndef _LINUX_MMC_TS_H_
#define _LINUX_MMC_TS_H_

#include <asm/ioctl.h>
#include <asm/types.h>
#include <linux/mmc/emmc_partitions.h>

#define MAX_MMC_GPT_PART_NUM 128
#define MMC_TS_MAX_KEY_SIZE	64
#define MMC_TS_MAX_VAL_SIZE	2048

struct mmc_ts_io_req {
	char key[MMC_TS_MAX_KEY_SIZE];
	char val[MMC_TS_MAX_VAL_SIZE];
};

struct mmc_gpt_partitions_fmt {
	char magic[4];
	unsigned char version[12];
	int part_num;
	int checksum;
	struct partitions partitions[MAX_MMC_GPT_PART_NUM];
};

/*mmc info data struct*/
struct fts_mmc_info {
	int bit;
	int blk_size; /* size of a block */
	struct mutex lock;
};

#define MMC_TS_IO_MAGIC   0xFE
#define MMC_TS_IO_SET     _IOW(MMC_TS_IO_MAGIC, 0, struct mmc_ts_io_req)
#define MMC_TS_IO_GET     _IOWR(MMC_TS_IO_MAGIC, 1, struct mmc_ts_io_req)
#define MMC_TS_IO_REINIT  _IOR(MMC_TS_IO_MAGIC, 2, struct mmc_ts_io_req)
#define MMC_TS_IO_CHECK   _IOR(MMC_TS_IO_MAGIC, 3, struct mmc_ts_io_req)

#endif  /* _LINUX_MMC_TS_H_ */

void get_mmc_card(struct fts_mmc_info **fts_mi, struct mmc_card **card);
void get_mmc_efi_part_info(struct mmc_gpt_partitions_fmt **dest);
