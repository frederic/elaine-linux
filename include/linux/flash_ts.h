/*
 * Flash-based transactional key-value store
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */
#ifndef _LINUX_FLASH_TS_H_
#define _LINUX_FLASH_TS_H_

#include <asm/ioctl.h>
#include <asm/types.h>

#define FLASH_TS_MAX_KEY_SIZE	64
#define FLASH_TS_MAX_VAL_SIZE	2048

struct flash_ts_io_req {
	char key[FLASH_TS_MAX_KEY_SIZE];
	char val[FLASH_TS_MAX_VAL_SIZE];
};

#define FLASH_TS_IO_MAGIC   0xFE
#define FLASH_TS_IO_SET     _IOW(FLASH_TS_IO_MAGIC, 0, struct flash_ts_io_req)
#define FLASH_TS_IO_GET     _IOWR(FLASH_TS_IO_MAGIC, 1, struct flash_ts_io_req)
#define FLASH_TS_IO_REINIT  _IOR(FLASH_TS_IO_MAGIC, 2, struct flash_ts_io_req)
#define FLASH_TS_IO_CHECK   _IOR(FLASH_TS_IO_MAGIC, 3, struct flash_ts_io_req)

#endif  /* _LINUX_FLASH_TS_H_ */
