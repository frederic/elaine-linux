/*
 * MMC-based transactional key-value store
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Bill he <yuegui.he@amlogic.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */

#include <linux/mutex.h>
#include <linux/mmc_ts.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/mmc/emmc_partitions.h>
#include <linux/seq_file.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/sizes.h>

#include "../mmc/card/queue.h"
#include "../mmc/card/block.h"


#define DEBUG              0
#define DRV_NAME        "fts"
#define DRV_VERSION     "0.999"
#define DRV_DESC        "MMC-based key-value storage"

/* Keep in sync with 'struct mmc_ts' */
#define MMC_TS_HDR_SIZE	(4 * sizeof(u32))
#define MMC_TS_MAX_SIZE	(16 * 1024)
#define MMC_TS_MAX_DATA_SIZE	(MMC_TS_MAX_SIZE - MMC_TS_HDR_SIZE)

#define MMC_TS_MAGIC		0x53542a46

#define MAX_PART_NAME_LEN               16
//#define     MAX_MMC_PART_NUM                32
#define MAX_RETRY 10

/*mmc erase group count*/
#define MMC_ERASE_GROUP_CNT 8192
/*mmc block size*/
#define MMC_BLK_SIZE 512

static struct mmc_card *p_mmc_card;

static struct fts_mmc_info *__mmc_info;

/* Physical mmc layout */
struct mmc_ts {
	u32 magic;		/* "F*TS" */
	u32 crc;
	u32 len;		/* real size of data */
	u32 version;		/* generation counter, must be positive */

	/* data format is very similar to Unix environment:
	 *   key1=value1\0key2=value2\0\0
	 */
	char data[MMC_TS_MAX_DATA_SIZE];
};

/* Internal state */
struct mmc_ts_priv {
	struct mutex lock;
	struct mmc *mmc;
	struct partitions part_info;

	/* chunk size, >= sizeof(struct mmc_ts) */
	size_t chunk;

	/*read write unit: a block*/
	int mmc_read_write_unit;

	/* current record offset within mmc partition device */
	loff_t offset;

	/* fts partition offset on*/
	u64 mmc_ts_offset;

	/* fts partition size */
	u64 mmc_ts_size;

	/* mmc dev*/
	int dev;

	/* in-memory copy of mmc content */
	struct mmc_ts cache;

	/* temporary buffers
	 *  - one backup for failure rollback
	 *  - another for read-after-write verification
	 */
	struct mmc_ts cache_tmp_backup;
	struct mmc_ts cache_tmp_verify;
};

static DEFINE_MUTEX(fts_inited_lock);
static struct mmc_ts_priv *__ts;
int mmc_ts_init(void);

static inline void __mmc_ts_put(struct mmc_ts_priv *ts)
{
	mutex_unlock(&ts->lock);
}

static inline void __mmc_info_put(struct fts_mmc_info *mmc_info)
{
	//mutex_unlock(&mmc_info->lock);
	return;
}
static void set_to_default_empty_state(struct mmc_ts_priv *ts)
{
	ts->offset = ts->part_info.size - ts->chunk;
	ts->cache.version = 0;
	ts->cache.len = 1;
	ts->cache.magic = MMC_TS_MAGIC;
	ts->cache.data[0] = '\0';
}

static struct mmc_ts_priv *__mmc_ts_get(void)
{
	struct mmc_ts_priv *ts = __ts;

	if (likely(ts))
		mutex_lock(&ts->lock);
	else
		pr_err(DRV_NAME ": mmc_ts not initialized yet\n");

	return ts;
}

static struct fts_mmc_info *__mmc_info_get(void)
{
	struct fts_mmc_info *mmc_info = __mmc_info;
	if (likely(mmc_info))
		pr_debug(DRV_NAME ":  mmc_info initialized\n");
	else
		pr_err(DRV_NAME ":  mmc_info not initialized yet\n");

	return mmc_info;
}


static char *mmc_ts_find(struct mmc_ts_priv *ts,
		const char *key, size_t key_len)
{
	char *s = ts->cache.data;
	while (*s) {
		if (!strncmp(s, key, key_len)) {
			if (s[key_len] == '=')
				return s;
		}

		s += strlen(s) + 1;
	}
	return NULL;
}

static inline u32 mmc_ts_check_header(const struct mmc_ts *cache)
{
	if (cache->magic == MMC_TS_MAGIC &&
	    cache->version &&
	    cache->len && cache->len <= sizeof(cache->data) &&
	    /* check correct null-termination */
	    !cache->data[cache->len - 1] &&
	    (cache->len == 1 || !cache->data[cache->len - 2])) {
		/* all is good */
		return cache->version;
	}

	return 0;
}

static int mmc_ts_read(struct fts_mmc_info *mmc_info,
		struct mmc_ts_priv *ts,
		loff_t off,
		void *buf,
		size_t size,
		u64 part_start_offset)
{
	ulong start_blk;
	void *addr_byte, *addr = buf, *addr_tmp;
	u64 cnt = 0, sz_byte = 0, blk = 0, res;

	/* blk shift : normal is 9 */
	int blk_shift = mmc_info->bit;

	/* start blk offset */
	blk = (part_start_offset + off) >> blk_shift;

	/* seziof(ts->cache_tmp_verify) = cnt * ts->chunk + sz_byte */
	cnt = size >> blk_shift;
	sz_byte = size - (cnt << blk_shift);

	mmc_claim_host(p_mmc_card->host);

	if (size >= mmc_info->blk_size) {
		/* read cnt* ts->chunk bytes */
		res = mmc_read_internal(p_mmc_card, blk, cnt, addr);
		if (res) {
			mmc_release_host(p_mmc_card->host);
			pr_err("mmc read error %llu, %s:%d\n", res, __func__, __LINE__);
			return -1;
		}
	}

	/* read sz_byte bytes */
	if (sz_byte != 0) {
		//printf("sz_byte=%#llx bytes\n",sz_byte);
		addr_tmp = kmalloc(mmc_info->blk_size, GFP_KERNEL);
		addr_byte = (void *)(addr+cnt*(mmc_info->blk_size));
		start_blk = blk+cnt;

		if (addr_tmp == NULL) {
			pr_info("mmc read: kmalloc fail\n");
			mmc_release_host(p_mmc_card->host);
			kfree(addr);
			return 1;
		}

		if (mmc_read_internal(p_mmc_card, start_blk, 1, addr_tmp)) {
			//read 1 block
			kfree(addr_tmp);
			pr_info("mmc read 1 block fail\n");
			mmc_release_host(p_mmc_card->host);
			return 1;
		}
		memcpy(addr_byte, addr_tmp, sz_byte);
		kfree(addr_tmp);
	}

	mmc_release_host(p_mmc_card->host);
	//__mmc_info_put(mmc_info);
	return 0;
}

static int mmc_is_blank(const void *buf, size_t size)
{
	size_t i;
	const unsigned int *data = (const unsigned int *)buf;
	size /= sizeof(data[0]);

	for (i = 0; i < size; i++)
		if (data[i] != 0xffffffff)
			return 0;
	return 1;
}

static int __init mmc_ts_scan(struct mmc_ts_priv *ts,
		struct fts_mmc_info *mmc_info)
{
	int res;
	loff_t off = 0;
	struct mmc_ts *tmp_scan_backup;
	u64 mmc_ts_size = ts->mmc_ts_size;
	u64 part_start_offset = ts->mmc_ts_offset;
	void *scan_addr;

	tmp_scan_backup = kzalloc(sizeof(*tmp_scan_backup), GFP_KERNEL);
	if (unlikely(!tmp_scan_backup))
		pr_err("tmp_scan_backup : kzalloc failure\n");

	scan_addr = (void *) tmp_scan_backup;

	do {
		/* read MMC_TS_MAX_DATA_SIZE  data to ts->cache_tmp_verify */
		res = mmc_ts_read(mmc_info, ts, off, scan_addr,
				sizeof(*tmp_scan_backup), part_start_offset);
		if (!res) {
			/* check data struct */
			u32 version = mmc_ts_check_header(scan_addr);
			if (version > 0) {
				if (version > ts->cache.version) {
					memcpy(&ts->cache, scan_addr,
						sizeof(*tmp_scan_backup));
					ts->offset = off;
				}
				break;
			} else if (0 == version && mmc_is_blank(tmp_scan_backup,
						sizeof(*tmp_scan_backup))) {
				off = (off + (ts->chunk * 4))
					& ~((ts->chunk * 4) - 1);
				//off = (off + ts->mmc->erase_grp_size)
				//& ~(ts->mmc->erase_grp_size - 1);
			} else {
				off += ts->chunk;
			}

		} else {
			off += ts->chunk;
		}

	} while (off < mmc_ts_size) ;/* while done*/

	//	mmc_release_host(mmc_info->card.host);
	return 0;
}

static int mmc_write(struct fts_mmc_info *mmc_info,
		struct mmc_ts_priv *ts,
		loff_t off,
		void *buf,
		size_t size,
		u64 part_start_offset)
{
	ulong start_blk;
	//struct mmc *mmc = ts->mmc;
	void *addr_byte, *addr_tmp, *addr, *addr_read;
	u64 cnt = 0, sz_byte = 0, blk = 0, res;
	struct mmc_ts *tmp_write_backup, *tmp_read_backup;
	int blk_shift;
	//struct fts_mmc_info *mmc_info = __mmc_info_get();

	tmp_write_backup = kzalloc(sizeof(*tmp_write_backup), GFP_KERNEL);
	tmp_read_backup = kzalloc(sizeof(*tmp_read_backup), GFP_KERNEL);

	/*********init local data struct**************/
	//memset(tmp_write_backup, 0, sizeof(tmp_write_backup));
	//memset(&tmp_read_backup, 0, sizeof(tmp_read_backup));
	memcpy(tmp_write_backup, buf, size);
	addr = (void *) tmp_write_backup;
	addr_read = (void *) tmp_read_backup;
	/********************************************/

	/* blk shift : normal is 9 */
	blk_shift = mmc_info->bit;

	/* start blk offset */
	blk = (part_start_offset + off) >> blk_shift;

	/* seziof(ts->cache_tmp_verify) = cnt * ts->chunk + sz_byte */
	cnt = size >> blk_shift;
	sz_byte = size - (cnt << blk_shift);

	mmc_claim_host(p_mmc_card->host);

	if (mmc_write_internal(p_mmc_card, blk, cnt, addr)) {
		pr_info("%s:%d error!\n", __func__, __LINE__);
		mmc_release_host(p_mmc_card->host);
		res = -1;
		goto free_EXIT;
	}

	//write sz_byte bytes
	if (sz_byte != 0) {
		// printf("sz_byte=%#llx bytes\n",sz_byte);
		addr_tmp = kmalloc(mmc_info->blk_size, GFP_KERNEL);
		addr_byte = (void*)(addr+cnt*(mmc_info->blk_size));
		start_blk = blk+cnt;

		if (addr_tmp == NULL) {
			pr_info("mmc write: kmalloc fail\n");
			mmc_release_host(p_mmc_card->host);
			res = 1;
			goto free_EXIT;
		}

		if (mmc_read_internal(p_mmc_card, start_blk, 1, addr_tmp)) {
			//read 1 block
			kfree(addr_tmp);
			pr_info("mmc read 1 block fail\n");
			mmc_release_host(p_mmc_card->host);
			res = 1;
			goto free_EXIT;
		}

		memcpy(addr_tmp, addr_byte, sz_byte);
		if (mmc_write_internal(p_mmc_card, start_blk, 1, addr_tmp)) {
			// write 1 block
			kfree(addr_tmp);
			pr_info("mmc write 1 block fail\n");
			mmc_release_host(p_mmc_card->host);
			res = 1;
			goto free_EXIT;
		}
		kfree(addr_tmp);
	}

	mmc_release_host(p_mmc_card->host);
	res = mmc_ts_read(mmc_info, ts, off, addr_read, sizeof(*tmp_read_backup), part_start_offset);

	if (!res) {
		if (! memcmp(tmp_write_backup, tmp_read_backup, size)) {
			memcpy(&ts->cache, tmp_write_backup, size);
			pr_info("key write successfull!\n");
			res = 0;
		} else {
			pr_err("%s:%d error!\n", __func__, __LINE__);
			res = -1;
		}

	} else {
		pr_err("check key failure!\n");
		res = -2;
	}

free_EXIT:
	kfree(tmp_write_backup);
	kfree(tmp_read_backup);
	return res;
}

static int mmc_ts_commit(struct fts_mmc_info *mmc_info, struct mmc_ts_priv *ts)
{
	int res;
	//int dev = ts->dev;
	u64 part_start_offset = ts->mmc_ts_offset;

	res = mmc_write(mmc_info, ts, 0, &ts->cache, sizeof(ts->cache), part_start_offset);

	return res;
}

int mmc_ts_set(const char *key, const char *value)
{
	int res;
	char *p;
	struct mmc_ts_priv *ts;
	struct fts_mmc_info *mmc_info;
	size_t klen = strlen(key);
	size_t vlen = strlen(value);

	ts = __mmc_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	mmc_info = __mmc_info_get();
	if (unlikely(!mmc_info))
		return -EINVAL;

	/* save current cache contents so we can restore it on failure */
	memcpy(&ts->cache_tmp_backup, &ts->cache, sizeof(ts->cache_tmp_backup));

	p = mmc_ts_find(ts, key, klen);
	if (p) {
		/* we are replacing existing entry,
		 * empty value (vlen == 0) removes entry completely.
		 */
		size_t cur_len = strlen(p) + 1;
		size_t new_len = vlen ? klen + 1 + vlen + 1 : 0;
		size_t move_len = 0;

		move_len = ts->cache.len - (p - ts->cache.data + cur_len) - 1;
		if (cur_len != new_len) {
			/* we need to move stuff around */

			if ((ts->cache.len - cur_len) + new_len >
			     sizeof(ts->cache.data))
				goto no_space;

			memmove(p + new_len, p + cur_len, move_len);
			if (cur_len > new_len) {
				/*Clean up the excess bytes*/
				memset(p + new_len + move_len, 0, cur_len - new_len);
			}
			ts->cache.len = (ts->cache.len - cur_len) + new_len;
			/*The last character needs to be set to 0*/
			*(ts->cache.data + ts->cache.len - 1) = '\0';
		} else if (!strcmp(p + klen + 1, value)) {
			/* skip update if new value is the same as the old one */
			res = 0;
			goto out;
		}

		if (vlen) {
			p += klen + 1;
			memcpy(p, value, vlen);
			p[vlen] = '\0';
		}
	} else {
		size_t len = klen + 1 + vlen + 1;

		/* don't do anything if value is empty */
		if (!vlen) {
			res = 0;
			goto out;
		}

		if (ts->cache.len + len > sizeof(ts->cache.data))
			goto no_space;

		/* add new entry at the end */
		p = ts->cache.data + ts->cache.len - 1;
		memcpy(p, key, klen);
		p += klen;
		*p++ = '=';
		memcpy(p, value, vlen);
		p += vlen;
		*p++ = '\0';
		*p = '\0';
		ts->cache.len += len;
	}
	++ts->cache.version;
	res = mmc_ts_commit(mmc_info, ts);
	if (unlikely(res))
		memcpy(&ts->cache, &ts->cache_tmp_backup, sizeof(ts->cache));

	goto out;

    no_space:
	pr_err(DRV_NAME ": no space left for '%s=%s'\n", key, value);
	res = -ENOSPC;

    out:
	__mmc_info_put(mmc_info);
	__mmc_ts_put(ts);

	return res;

}

void mmc_ts_get(const char *key, char *value, unsigned int size)
{
	const char *p;
	struct mmc_ts_priv *ts;
	size_t klen = strlen(key);

	BUG_ON(!size);

	*value = '\0';

	ts = __mmc_ts_get();
	if (unlikely(!ts))
		return;

	p = mmc_ts_find(ts, key, klen);
	if (p)
		strlcpy(value, p + klen + 1, size);

	__mmc_ts_put(ts);

}

/* write zero into blk_cnt, start address is start_blk*/
static int mmc_write_zero(int blk_cnt, int start_blk)
{
	unsigned int seg_size, nents;
	void* zero_buff;
	u64 blk_nonius;
	int fts_size_blk_cnt = blk_cnt, i;

	seg_size = p_mmc_card->host->max_seg_size;
	zero_buff = kzalloc(seg_size, GFP_KERNEL);
	if (zero_buff == NULL) {
		pr_err("%s:%d kzalloc fail\n", __func__, __LINE__);
		return 1;
	}

	memset(zero_buff, 0, seg_size);
	/*How many seg sizes*/
	nents = (fts_size_blk_cnt*MMC_BLK_SIZE)/seg_size;
	blk_nonius = start_blk;
	for (i = 0; i < nents; i++) {
		if (mmc_write_internal(p_mmc_card, blk_nonius, (seg_size/MMC_BLK_SIZE), zero_buff))
		{
			pr_err("%s:%d error!\n", __func__, __LINE__);
			mmc_release_host(p_mmc_card->host);
			kfree(zero_buff);
			return 1;
		}
		blk_nonius += (seg_size/MMC_BLK_SIZE);
	}

	if (mmc_write_internal(p_mmc_card, blk_nonius, ((fts_size_blk_cnt*MMC_BLK_SIZE)%(seg_size))/MMC_BLK_SIZE, zero_buff))
	{
		pr_err("%s:%d error!\n", __func__, __LINE__);
		mmc_release_host(p_mmc_card->host);
		kfree(zero_buff);
		return 1;
	}

	kfree(zero_buff);
	return 0;
}

/* erases the whole fts MMC device and re-initializes
 * the in-memory cache to default empty state
 */
static int mmc_reinit(void)
{
	struct mmc_ts_priv *ts;
	u64 mmc_ts_size;
	struct fts_mmc_info *mmc_info = __mmc_info_get();
	unsigned int nr_new, nr_mod, nr_offset;

	/* blk shift : normal is 9 */
	int blk_shift = mmc_info->bit;
	//char fts_reset_buff[mmc_info->blk_size];
	int fts_size_blk_cnt;
	int fts_size_blk_cn_other;
	int res;
	u64 part_start_offset, blk;

	ts = __mmc_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	mmc_ts_size = ts->mmc_ts_size;
	fts_size_blk_cnt = mmc_ts_size >> blk_shift;
	fts_size_blk_cn_other = mmc_ts_size - (fts_size_blk_cnt << blk_shift);
	if (fts_size_blk_cn_other > 0) {
		fts_size_blk_cnt +=1;
	}

	part_start_offset = ts->mmc_ts_offset;
	/* start blk offset */
	blk = part_start_offset >> blk_shift;

	mmc_claim_host(p_mmc_card->host);
	if (!mmc_can_erase(p_mmc_card)) {
		pr_info("%s:%d\n", __func__, __LINE__);
		return 0;
	}

	nr_new = round_down(fts_size_blk_cnt, p_mmc_card->erase_size);
	if (nr_new == 0)
	{
		if (mmc_write_zero(fts_size_blk_cnt, blk))
		{
			pr_err("%s:%d erase sectors fail!\n", __func__, __LINE__);
			return 1;
		} else {
			res = 0;
		}
	} else {
		nr_mod = fts_size_blk_cnt % (nr_new * MMC_ERASE_GROUP_CNT);
		nr_offset = fts_size_blk_cnt / (nr_new * MMC_ERASE_GROUP_CNT);
		res = mmc_erase(p_mmc_card, blk, fts_size_blk_cnt, MMC_ERASE_ARG);
		if (res) {
			pr_err("%s:%d erase sectors fail!\n", __func__, __LINE__);
			return 1;
		}
		if (mmc_write_zero(nr_mod, blk + nr_offset))
		{
			pr_err("%s:%d erase sectors fail!\n", __func__, __LINE__);
			return 1;
		} else {
			res = 0;
		}
	}
	mmc_release_host(p_mmc_card->host);

	if (likely(!res)) {
		/* restore to default empty state */
		set_to_default_empty_state(ts);

		/* Fill the unused part of the cache. set_to_default_empty_state
		 * resets the cache by setting the first character to the null
		 * terminator and length to 1; this preserves that while wiping
		 * out any real data remaining in the cache.
		 */
		memset(ts->cache.data + ts->cache.len, 0xff,
		       sizeof(ts->cache.data) - ts->cache.len);
	}

	__mmc_ts_put(ts);
	__mmc_info_put(mmc_info);
	return res;
}

/* checks integrity of the mtd device and prints info about its contents */
static int mmc_ts_check(void)
{
	int res, bad_chunks = 0;
	loff_t off = 0;
	struct mmc_ts_priv *ts;
	struct mmc_ts *tmp_scan_backup;
	struct fts_mmc_info *mmc_info;
	u64 mmc_ts_size, part_start_offset;
	void *scan_addr = NULL;

	ts = __mmc_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	mmc_info = __mmc_info_get();
	if (unlikely(!mmc_info))
		return -EINVAL;

    tmp_scan_backup = kzalloc(sizeof(*tmp_scan_backup), GFP_KERNEL);

	mmc_ts_size = ts->mmc_ts_size;
	part_start_offset = ts->mmc_ts_offset;
	scan_addr = (void *) tmp_scan_backup;

	memset(tmp_scan_backup, 0, sizeof(*tmp_scan_backup));

	do {
		/* read MMC_TS_MAX_DATA_SIZE  data to ts->cache_tmp_verify */
		res = mmc_ts_read(mmc_info, ts, off, scan_addr, sizeof(*tmp_scan_backup), part_start_offset);
		if (!res) {
			/* check data struct */
			u32 version = mmc_ts_check_header(scan_addr);
			if (version == 0) {
				if (mmc_is_blank(tmp_scan_backup, sizeof(*tmp_scan_backup))) {
					off = (off + (ts->chunk)) & ~((ts->chunk) - 1);
					/* skip the whole block if chunk is blank */
					pr_info(DRV_NAME ": blank chunk @ 0x%08llx\n", off);
				} else {
					/* header didn't check out and flash is not blank */
					pr_err(DRV_NAME ": bad chunk @ 0x%08llx\n", off);
					++bad_chunks;
					off += ts->chunk;
				}

			} else {
				/* header checked out, so move on */
				pr_info(DRV_NAME ": record v%u @ 0x%08llx\n", version, off);
				off += ts->chunk;
				break;
			}

		}
	} while(off < mmc_ts_size) ;/* while done*/

	if (unlikely(bad_chunks)) {
		pr_err(DRV_NAME ": %d bad chunks\n", bad_chunks);
		__mmc_ts_put(ts);
		kfree(tmp_scan_backup);
		return -EIO;
	}

	__mmc_ts_put(ts);
	kfree(tmp_scan_backup);
	return res;

}
/* User-space access */
struct mmc_ts_dev {
	struct mutex lock;
	struct mmc_ts_io_req req;
};

static int mmc_ts_open(struct inode *inode, struct file *file)
{
	struct mmc_ts_dev *dev = NULL;

	if (mmc_ts_init())
		return -EAGAIN;

	dev = kmalloc(sizeof(*dev), GFP_KERNEL);

	if (unlikely(!dev))
		return -ENOMEM;

	mutex_init(&dev->lock);
	file->private_data = dev;
	return 0;
}

static int mmc_ts_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static long mmc_ts_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mmc_ts_dev *dev = file->private_data;
	struct mmc_ts_io_req *req = &dev->req;
	int res;
	if (unlikely(mutex_lock_interruptible(&dev->lock))) {
		pr_err("%s:%d error!\n", __func__, __LINE__);
		return -ERESTARTSYS;
	}

	if (unlikely(copy_from_user(req, (const void* __user)arg,
				    sizeof(*req)))) {
		res = -EFAULT;
		pr_err("%s:%d error!\n", __func__, __LINE__);
		goto out;
	}

	req->key[sizeof(req->key) - 1] = '\0';

	switch (cmd) {
	case MMC_TS_IO_SET:
		req->val[sizeof(req->val) - 1] = '\0';
		pr_info("mmc ts set: key: %s , val: %s\n", (char *)req->key, (char *)req->val);
		res = mmc_ts_set(req->key, req->val);
		break;

	case MMC_TS_IO_GET:
		mmc_ts_get(req->key, req->val, sizeof(req->val));
#if DEBUG
		pr_info("mmc ts get: key: %s , val: %s\n", (char *)req->key, (char *)req->val);
#endif
		res = copy_to_user((void* __user)arg, req,
				   sizeof(*req)) ? -EFAULT : 0;
		break;

	case MMC_TS_IO_REINIT:
		pr_info("mmc ts reinit! \n");
		res = mmc_reinit();
		break;

	case MMC_TS_IO_CHECK:
		pr_info("mmc ts check! \n");
		res = mmc_ts_check();
		break;

	default:
		printk("%s:%d error!\n", __func__, __LINE__);
		res = -ENOTTY;
	}

    out:
	mutex_unlock(&dev->lock);
	return res;
}

#ifdef CONFIG_COMPAT
static long mmc_ts_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return mmc_ts_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static struct file_operations mmc_ts_fops = {
	.owner = THIS_MODULE,
	.open = mmc_ts_open,
	.unlocked_ioctl = mmc_ts_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc_ts_compat_ioctl,
#endif
	.release = mmc_ts_release,
};

static struct miscdevice mmc_ts_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRV_NAME,
	.fops = &mmc_ts_fops,
};

/* Debugging (procfs) */
static void *mmc_ts_proc_start(struct seq_file *m, loff_t *pos)
{
	if (*pos == 0) {
		struct mmc_ts_priv *ts = __mmc_ts_get();
		if (ts) {
			BUG_ON(m->private);
			m->private = ts;
			return ts->cache.data;
		}
	}

	*pos = 0;
	return NULL;
}

static void *mmc_ts_proc_next(struct seq_file *m, void *v, loff_t *pos)
{
	char *s = (char *)v;
	s += strlen(s) + 1;
	++(*pos);
	return *s ? s : NULL;
}

static void mmc_ts_proc_stop(struct seq_file *m, void *v)
{
	struct mmc_ts_priv *ts = m->private;
	if (ts) {
		m->private = NULL;
		__mmc_ts_put(ts);
	}
}

static int mmc_ts_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", (char*)v);
	return 0;
}

static struct seq_operations mmc_ts_seq_ops = {
	.start	= mmc_ts_proc_start,
	.next	= mmc_ts_proc_next,
	.stop	= mmc_ts_proc_stop,
	.show	= mmc_ts_proc_show,
};

static int mmc_ts_proc_open(struct inode *inode, struct file *file)
{
	if (mmc_ts_init())
		return -EAGAIN;
	return seq_open(file, &mmc_ts_seq_ops);
}

static const struct file_operations mmc_ts_proc_fops = {
	.owner = THIS_MODULE,
	.open = mmc_ts_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* Round-up to the next power-of-2,
 * from "Hacker's Delight" by Henry S. Warren.
 */
static inline u32 clp2(u32 x)
{
	--x;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	return x + 1;
}

static int mmc_ts_init_part_info(struct mmc_ts_priv *ts,
		struct mmc_gpt_partitions_fmt *gpt_part)
{
	int i;
	for (i = 0; i < gpt_part->part_num; i++) {
		if (!strcmp(gpt_part->partitions[i].name, CONFIG_FLASH_TS_PARTITION)) {
			ts->mmc_ts_offset = gpt_part->partitions[i].offset;
			ts->mmc_ts_size = gpt_part->partitions[i].size;
			/* init partiton info to ts data struct*/
			memcpy(&ts->part_info, &(gpt_part->partitions[i]), sizeof(ts->part_info));
			return 0;
		}
	}

	return 1;
}

static struct notifier_block reboot_notifier;
/*
 * BCB (boot control block) support
 * Handle reboot command and set boot params for bootloader
 */
static int bcb_fts_reboot_hook(struct notifier_block *notifier,
                   unsigned long code, void *cmd)
{
	if (code == SYS_RESTART && cmd && !strcmp(cmd, "recovery")) {
		if (mmc_ts_set("bootloader.command", "boot-recovery") ||
				mmc_ts_set("bootloader.status", "") ||
				mmc_ts_set("bootloader.recovery", ""))
			pr_err("Failed to set bootloader command\n");
	}
	if (code == SYS_RESTART && cmd && !strcmp(cmd, "backupsys")) {
		if (mmc_ts_set("bootloader.command", "boot-backupsys") ||
				mmc_ts_set("bootloader.status", "") ||
				mmc_ts_set("bootloader.recovery", ""))
			pr_err("Failed to set bootloader command\n");
	}

	return NOTIFY_DONE;
}

int mmc_ts_init(void)
{
	int res = 0;
	int retry = 0;
	struct mmc_ts_priv *ts = NULL;
	struct fts_mmc_info *mmc_info = NULL;
	struct mmc_gpt_partitions_fmt *gpt_part = NULL;

	if (mutex_is_locked(&fts_inited_lock)) {
		return -EAGAIN; /* tells try it later */
	}

	mutex_lock(&fts_inited_lock);
	if (__ts)
		goto unlock_inited;

	get_mmc_card(&mmc_info, &p_mmc_card);

	/*init fts data struct*/
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (unlikely(!ts)) {
		res = -ENOMEM;
		mutex_unlock(&fts_inited_lock);
		pr_err(DRV_NAME ": failed to allocate memory\n");
		return res;
	}

	/*gpt partition check*/
	do {
		pr_info("%s get_mmc_part, retry:%d\n", __func__, retry);
		get_mmc_efi_part_info(&gpt_part);
		usleep_range(500, 600);
	} while (retry++ < MAX_RETRY &&
			(gpt_part->part_num <= 0 ||
			 gpt_part->part_num > MAX_MMC_GPT_PART_NUM));

	if (retry >= MAX_RETRY)
		goto out_kfree;

	mutex_init(&ts->lock);
	mutex_init(&mmc_info->lock);
	/*init ts partition info*/
	if (mmc_ts_init_part_info(ts, gpt_part))
	{
		pr_info("[%s]: mmc ts part init failure\n", __func__);
	}

	/*init chunk*/
	ts->chunk = clp2((sizeof(struct mmc_ts) + mmc_info->blk_size - 1) &
			~(mmc_info->blk_size - 1));

	/*mmc blk size*/
	ts->mmc_read_write_unit = mmc_info->blk_size;

	/* default empty state */
	set_to_default_empty_state(ts);

	/* scan mmc partition for the most recent record */
	res = mmc_ts_scan(ts, mmc_info);
	if (unlikely(res)) {
		pr_err("%s: %d error!\n", __func__, __LINE__);
		goto out_kfree;
	}

	if (ts->cache.version)
		pr_info(DRV_NAME ": v%u loaded from 0x%08llx\n",
		       ts->cache.version, ts->offset);
	__ts = ts;
	__mmc_info = mmc_info;

	mutex_unlock(&fts_inited_lock);

	smp_mb();
	return 0;

out_kfree:
	kfree(ts);
unlock_inited:
	mutex_unlock(&fts_inited_lock);
	return 0;
}

int mmc_ts_initcall(void)
{
	int res = misc_register(&mmc_ts_miscdev);

	if (unlikely(res))
		return 0;

	proc_create(DRV_NAME, 0, NULL, &mmc_ts_proc_fops);

	/* Register optional reboot hook */
	reboot_notifier.notifier_call = bcb_fts_reboot_hook;
	register_reboot_notifier(&reboot_notifier);
	return 0;
}

/* Make sure mmc subsystem is already initialized */
late_initcall(mmc_ts_initcall);
