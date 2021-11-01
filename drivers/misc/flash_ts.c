/*
 * Flash-based transactional key-value store
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Eugene Surovegin <es@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 */
#include <linux/crc32.h>
#include <linux/flash_ts.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#include <linux/notifier.h>

#define DRV_NAME        	"fts"
#define DRV_VERSION     	"0.999"
#define DRV_DESC        	"MTD-based key-value storage"

MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Eugene Surovegin <es@google.com>");
MODULE_LICENSE("GPL");

/* Keep in sync with 'struct flash_ts' */
#define FLASH_TS_HDR_SIZE	(4 * sizeof(u32))
#define FLASH_TS_MAX_SIZE	(16 * 1024)
#define FLASH_TS_MAX_DATA_SIZE	(FLASH_TS_MAX_SIZE - FLASH_TS_HDR_SIZE)

#define FLASH_TS_MAGIC		0x53542a46

/* Physical flash layout */
struct flash_ts {
	u32 magic;		/* "F*TS" */
	u32 crc;		/* doesn't include magic and crc fields */
	u32 len;		/* real size of data */
	u32 version;		/* generation counter, must be positive */

	/* data format is very similar to Unix environment:
	 *   key1=value1\0key2=value2\0\0
	 */
	char data[FLASH_TS_MAX_DATA_SIZE];
};

/* Internal state */
struct flash_ts_priv {
	struct mutex lock;
	struct mtd_info *mtd;

	/* chunk size, >= sizeof(struct flash_ts) */
	size_t chunk;

	/* current record offset within MTD device */
	loff_t offset;

	/* in-memory copy of flash content */
	struct flash_ts cache;

	/* temporary buffers
	 *  - one backup for failure rollback
	 *  - another for read-after-write verification
	 */
	struct flash_ts cache_tmp_backup;
	struct flash_ts cache_tmp_verify;
};
static struct flash_ts_priv *__ts;

static int flash_is_blank(const void *buf, size_t size)
{
	size_t i;
	const unsigned int *data = (const unsigned int *)buf;
	size /= sizeof(data[0]);

	for (i = 0; i < size; i++)
		if (data[i] != 0xffffffff)
			return 0;
	return 1;
}

static void flash_erase_callback(struct erase_info *ctx)
{
	wake_up((wait_queue_head_t*)ctx->priv);
}

static int flash_erase(struct mtd_info *mtd, loff_t off)
{
	struct erase_info ei = {0};
	int res;

	wait_queue_head_t waitq;
	DECLARE_WAITQUEUE(wait, current);
        init_waitqueue_head(&waitq);

	ei.mtd = mtd;
        ei.len = mtd->erasesize;
	ei.addr = off;
	ei.callback = flash_erase_callback;
        ei.priv = (unsigned long)&waitq;

	/* Yes, this is racy, but safer than just leaving
	 * partition writeable all the time.
	 */
	mtd->flags |= MTD_WRITEABLE;

	res = mtd_erase(mtd, &ei);
        if (!res) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&waitq, &wait);
		if (ei.state != MTD_ERASE_DONE && ei.state != MTD_ERASE_FAILED)
			schedule();
		remove_wait_queue(&waitq, &wait);
		set_current_state(TASK_RUNNING);

		res = ei.state == MTD_ERASE_FAILED ? -EIO : 0;
	}
	mtd->flags &= ~MTD_WRITEABLE;

	if (unlikely(res))
		printk(KERN_ERR DRV_NAME
		       ": flash_erase(0x%08llx) failed, errno %d\n",
		       off, res);
	return res;
}

/* erases a whole mtd device, skipping bad blocks */
static int flash_erase_all(struct mtd_info *mtd)
{
	int res = 0;
	loff_t off = 0;

	do {
		if (mtd_block_isbad(mtd, off)) {
			printk(KERN_INFO DRV_NAME
			       ": not erasing bad block @ 0x%08llx\n",
			       off);
		} else {
			printk(KERN_INFO DRV_NAME
			       ": erasing block @ 0x%08llx\n",
			       off);
			res = flash_erase(mtd, off);
			if (unlikely(res)) {
				printk(KERN_ERR DRV_NAME
				       ": flash_erase_all failed, errno %d\n",
				       res);
				break;
			}
		}

		off += mtd->erasesize;
	} while (off < mtd->size);

	return res;
}

static int flash_write(struct mtd_info *mtd, loff_t off,
		       const void *buf, size_t size)
{
	int res = 0;

	mtd->flags |= MTD_WRITEABLE;
	while (size) {
		size_t retlen;
		res = mtd_write(mtd, off, size, &retlen, buf);
		if (likely(!res)) {
			off += retlen;
			buf += retlen;
			size -= retlen;
		} else {
			printk(KERN_ERR DRV_NAME
			       ": flash_write(0x%08llx, %zu) failed, errno %d\n",
			       off, size, res);
			break;
		}
	}
	mtd->flags &= ~MTD_WRITEABLE;

	return res;
}

static int flash_read(struct mtd_info *mtd, loff_t off, void *buf, size_t size)
{
	int res = 0;
	while (size) {
		size_t retlen;
		res = mtd_read(mtd, off, size, &retlen, buf);
		if (!res || res == -EUCLEAN) {
			off += retlen;
			buf += retlen;
			size -= retlen;
		} else {
			printk(KERN_WARNING DRV_NAME
			       ": flash_read() failed, errno %d\n", res);
			break;
		}
	}
	return res;
}

static char *flash_ts_find(struct flash_ts_priv *ts, const char *key,
			   size_t key_len)
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


static inline u32 flash_ts_crc(const struct flash_ts *cache)
{
	/* skip magic and crc fields */
	return crc32(0, &cache->len, cache->len + 2 * sizeof(u32)) ^ ~0;
}

static void set_to_default_empty_state(struct flash_ts_priv *ts)
{
	ts->offset = ts->mtd->size - ts->chunk;
	ts->cache.magic = FLASH_TS_MAGIC;
	ts->cache.version = 0;
	ts->cache.len = 1;
	ts->cache.data[0] = '\0';
	ts->cache.crc = flash_ts_crc(&ts->cache);
}

/* Verifies cache consistency and locks it */
static struct flash_ts_priv *__flash_ts_get(void)
{
	struct flash_ts_priv *ts = __ts;

	if (likely(ts)) {
		mutex_lock(&ts->lock);
		if (unlikely(ts->cache.crc != flash_ts_crc(&ts->cache))) {
			printk(KERN_CRIT DRV_NAME
			       ": memory corruption detected\n");
			mutex_lock(&ts->lock);
			ts = NULL;
		}
	} else {
		printk(KERN_ERR DRV_NAME ": not initialized yet\n");
	}

	return ts;
}

static inline void __flash_ts_put(struct flash_ts_priv *ts)
{
	mutex_unlock(&ts->lock);
}

static int flash_ts_commit(struct flash_ts_priv *ts)
{
	struct mtd_info *mtd = ts->mtd;
	loff_t off = ts->offset + ts->chunk;
	/* we try to make two passes to handle non-erased blocks
	 * this should only matter for the inital pass over the whole device.
	 */
	int max_iterations = mtd_div_by_eb(mtd->size, mtd) * 2;
	size_t size = ALIGN(FLASH_TS_HDR_SIZE + ts->cache.len, ts->chunk);

	/* fill unused part of data */
	memset(ts->cache.data + ts->cache.len, 0xff,
	       sizeof(ts->cache.data) - ts->cache.len);

	while (max_iterations--) {
		/* wrap around */
		if (off >= mtd->size)
			off = 0;

		/* new block? */
		if (!(off & (mtd->erasesize - 1))) {
			if (mtd_block_isbad(mtd, off)) {
				/* skip this block */
				off += mtd->erasesize;
				continue;
			}

			if (unlikely(flash_erase(mtd, off))) {
				/* skip this block */
				off += mtd->erasesize;
				continue;
			}
		}

		/* write and read back to veryfy */
		if (flash_write(mtd, off, &ts->cache, size) ||
		    flash_read(mtd, off, &ts->cache_tmp_verify, size)) {
			/* hmm, probably unclean block, skip it for now */
			off = (off + mtd->erasesize) & ~(mtd->erasesize - 1);
			continue;
		}

		/* compare */
		if (memcmp(&ts->cache, &ts->cache_tmp_verify, size)) {
			printk(KERN_WARNING DRV_NAME
			       ": record v%u read mismatch @ 0x%08llx\n",
				ts->cache.version, off);
			/* skip this block for now */
			off = (off + mtd->erasesize) & ~(mtd->erasesize - 1);
			continue;
		}

		/* for new block, erase the previous block after write done,
		 * it's to speed up flash_ts_scan
		 */
		if (!(off & (mtd->erasesize - 1))) {
			loff_t pre_block_base = ts->offset & ~(mtd->erasesize - 1);
			loff_t cur_block_base = off & ~(mtd->erasesize - 1);
			if (cur_block_base != pre_block_base)
				flash_erase(mtd, pre_block_base);
		}
		ts->offset = off;
		printk(KERN_DEBUG DRV_NAME ": record v%u commited @ 0x%08llx\n",
		       ts->cache.version, off);
		return 0;
	}

	printk(KERN_ERR DRV_NAME ": commit failure\n");
	return -EIO;
}

static int flash_ts_set(const char *key, const char *value)
{
	struct flash_ts_priv *ts;
	size_t klen = strlen(key);
	size_t vlen = strlen(value);
	int res;
	char *p;

	ts = __flash_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	/* save current cache contents so we can restore it on failure */
	memcpy(&ts->cache_tmp_backup, &ts->cache, sizeof(ts->cache_tmp_backup));

	p = flash_ts_find(ts, key, klen);
	if (p) {
		/* we are replacing existing entry,
		 * empty value (vlen == 0) removes entry completely.
		 */
		size_t cur_len = strlen(p) + 1;
		size_t new_len = vlen ? klen + 1 + vlen + 1 : 0;

		if (cur_len != new_len) {
			/* we need to move stuff around */

			if ((ts->cache.len - cur_len) + new_len >
			     sizeof(ts->cache.data))
				goto no_space;

			memmove(p + new_len, p + cur_len,
				ts->cache.len - (p - ts->cache.data + cur_len));

			ts->cache.len = (ts->cache.len - cur_len) + new_len;
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
	ts->cache.crc = flash_ts_crc(&ts->cache);
	res = flash_ts_commit(ts);
	if (unlikely(res))
		memcpy(&ts->cache, &ts->cache_tmp_backup, sizeof(ts->cache));
	goto out;

    no_space:
	printk(KERN_WARNING DRV_NAME ": no space left for '%s=%s'\n",
	       key, value);
	res = -ENOSPC;
    out:
	__flash_ts_put(ts);

	return res;
}

static void flash_ts_get(const char *key, char *value, unsigned int size)
{
	size_t klen = strlen(key);
	struct flash_ts_priv *ts;
	const char *p;

	BUG_ON(!size);

	*value = '\0';

	ts = __flash_ts_get();
	if (unlikely(!ts))
		return;

	p = flash_ts_find(ts, key, klen);
	if (p)
		strlcpy(value, p + klen + 1, size);

	__flash_ts_put(ts);
}

/* erases the whole mtd device and re-initializes
 * the in-memory cache to default empty state
 */
static int flash_reinit(void)
{
	int res = 0;

	struct flash_ts_priv *ts = __flash_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	/* erase the whole mtd device */
	res = flash_erase_all(ts->mtd);

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

	__flash_ts_put(ts);
	return res;
}

static inline u32 flash_ts_check_header(const struct flash_ts *cache)
{
	if (cache->magic == FLASH_TS_MAGIC &&
	    cache->version &&
	    cache->len && cache->len <= sizeof(cache->data) &&
	    cache->crc == flash_ts_crc(cache) &&
	    /* check correct null-termination */
	    !cache->data[cache->len - 1] &&
	    (cache->len == 1 || !cache->data[cache->len - 2])) {
		/* all is good */
		return cache->version;
	}

	return 0;
}

/* checks integrity of the mtd device and prints info about its contents */
static int flash_ts_check(void)
{
	struct mtd_info *mtd;
	int res, good_blocks = 0, bad_chunks = 0;
	loff_t off = 0;
	u32 version = 0;

	struct flash_ts_priv *ts = __flash_ts_get();
	if (unlikely(!ts))
		return -EINVAL;

	mtd = ts->mtd;

	do {
		/* new block? */
		if (!(off & (mtd->erasesize - 1))) {
			printk(KERN_INFO DRV_NAME
			       ": new block @ 0x%08llx\n", off);
			if (mtd_block_isbad(mtd, off)) {
				printk(KERN_INFO DRV_NAME
				       ": skipping bad block @ 0x%08llx\n",
				       off);
				off += mtd->erasesize;
				continue;
			} else {
				++good_blocks;
			}
		}

		res = flash_read(mtd, off, &ts->cache_tmp_verify,
				 sizeof(ts->cache_tmp_verify));
		if (res) {
			printk(KERN_WARNING DRV_NAME
			       ": could not read flash @ 0x%08llx\n", off);
			off += ts->chunk;
			continue;
		}

		version = flash_ts_check_header(&ts->cache_tmp_verify);
		if (0 == version) {
			if (flash_is_blank(&ts->cache_tmp_verify,
					   sizeof(ts->cache_tmp_verify))) {
				/* skip the whole block if chunk is blank */
				printk(KERN_INFO DRV_NAME
				       ": blank chunk @ 0x%08llx\n", off);
				off = (off + mtd->erasesize) & ~(mtd->erasesize - 1);
			} else {
				/* header didn't check out and flash is not blank */
				printk(KERN_ERR DRV_NAME
				       ": bad chunk @ 0x%08llx\n", off);
				++bad_chunks;
				off += ts->chunk;
			}
		} else {
			/* header checked out, so move on */
			printk(KERN_INFO DRV_NAME
			       ": record v%u @ 0x%08llx\n", version, off);
			off += ts->chunk;
		}
	} while (off < mtd->size);

	if (unlikely(!good_blocks)) {
		printk(KERN_ERR DRV_NAME ": no good blocks\n");
		__flash_ts_put(ts);
		return -ENODEV;
	}

	if (unlikely(good_blocks < 2))
		printk(KERN_WARNING DRV_NAME ": less than 2 good blocks,"
		       " reliability is not guaranteed\n");

	if (unlikely(bad_chunks)) {
		printk(KERN_ERR DRV_NAME ": %d bad chunks\n", bad_chunks);
		__flash_ts_put(ts);
		return -EIO;
	}

	__flash_ts_put(ts);
	return 0;
}

static int __init flash_ts_scan(struct flash_ts_priv *ts)
{
	struct mtd_info *mtd = ts->mtd;
	int res, good_blocks = 0;
	loff_t off = 0;

	do {
		/* new block ? */
		if (!(off & (mtd->erasesize - 1))) {
			if (mtd_block_isbad(mtd, off)) {
				printk(KERN_INFO DRV_NAME
				       ": skipping bad block @ 0x%08llx\n",
				       off);
				off += mtd->erasesize;
				continue;
			} else
				++good_blocks;
		}

		res = flash_read(mtd, off, &ts->cache_tmp_verify,
				 sizeof(ts->cache_tmp_verify));
		if (!res) {
			u32 version =
			    flash_ts_check_header(&ts->cache_tmp_verify);
			if (version > ts->cache.version) {
				memcpy(&ts->cache, &ts->cache_tmp_verify,
				       sizeof(ts->cache));
				ts->offset = off;
			}
			if (0 == version &&
				flash_is_blank(&ts->cache_tmp_verify,
					sizeof(ts->cache_tmp_verify))) {
				/* skip the whole block if chunk is blank */
				off = (off + mtd->erasesize) & ~(mtd->erasesize - 1);
			} else {
				off += ts->chunk;
			}
		} else {
			off += ts->chunk;
		}
	} while (off < mtd->size);

	if (unlikely(!good_blocks)) {
		printk(KERN_ERR DRV_NAME ": no good blocks\n");
		return -ENODEV;
	}

	if (unlikely(good_blocks < 2))
		printk(KERN_WARNING DRV_NAME ": less than 2 good blocks,"
					     " reliability is not guaranteed\n");
	return 0;
}

/* User-space access */
struct flash_ts_dev {
	struct mutex lock;
	struct flash_ts_io_req req;
};

static int flash_ts_open(struct inode *inode, struct file *file)
{
	struct flash_ts_dev *dev = NULL;
	dev = kmalloc(sizeof(*dev), GFP_KERNEL);

	if (unlikely(!dev))
		return -ENOMEM;

	mutex_init(&dev->lock);
	file->private_data = dev;
	return 0;
}

static int flash_ts_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static long flash_ts_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct flash_ts_dev *dev = file->private_data;
	struct flash_ts_io_req *req = &dev->req;
	int res;

	if (unlikely(mutex_lock_interruptible(&dev->lock)))
		return -ERESTARTSYS;

	if (unlikely(copy_from_user(req, (const void* __user)arg,
				    sizeof(*req)))) {
		res = -EFAULT;
		goto out;
	}

	req->key[sizeof(req->key) - 1] = '\0';

	switch (cmd) {
	case FLASH_TS_IO_SET:
		req->val[sizeof(req->val) - 1] = '\0';
		res = flash_ts_set(req->key, req->val);
		break;

	case FLASH_TS_IO_GET:
		flash_ts_get(req->key, req->val, sizeof(req->val));
		res = copy_to_user((void* __user)arg, req,
				   sizeof(*req)) ? -EFAULT : 0;
		break;

	case FLASH_TS_IO_REINIT:
		res = flash_reinit();
		break;

	case FLASH_TS_IO_CHECK:
		res = flash_ts_check();
		break;

	default:
		res = -ENOTTY;
	}

    out:
	mutex_unlock(&dev->lock);
	return res;
}

#ifdef CONFIG_COMPAT
static long flash_ts_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return flash_ts_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static struct file_operations flash_ts_fops = {
	.owner = THIS_MODULE,
	.open = flash_ts_open,
	.unlocked_ioctl = flash_ts_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = flash_ts_compat_ioctl,
#endif
	.release = flash_ts_release,
};

static struct miscdevice flash_ts_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRV_NAME,
	.fops = &flash_ts_fops,
};

/* Debugging (procfs) */
static void *flash_ts_proc_start(struct seq_file *m, loff_t *pos)
{
	if (*pos == 0) {
		struct flash_ts_priv *ts = __flash_ts_get();
		if (ts) {
			BUG_ON(m->private);
			m->private = ts;
			return ts->cache.data;
		}
	}

	*pos = 0;
	return NULL;
}

static void *flash_ts_proc_next(struct seq_file *m, void *v, loff_t *pos)
{
	char *s = (char *)v;
	s += strlen(s) + 1;
	++(*pos);
	return *s ? s : NULL;
}

static void flash_ts_proc_stop(struct seq_file *m, void *v)
{
	struct flash_ts_priv *ts = m->private;
	if (ts) {
		m->private = NULL;
		__flash_ts_put(ts);
	}
}

static int flash_ts_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", (char*)v);
	return 0;
}

static struct seq_operations flash_ts_seq_ops = {
	.start	= flash_ts_proc_start,
	.next	= flash_ts_proc_next,
	.stop	= flash_ts_proc_stop,
	.show	= flash_ts_proc_show,
};

static int flash_ts_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &flash_ts_seq_ops);
}

static const struct file_operations flash_ts_proc_fops = {
	.owner = THIS_MODULE,
	.open = flash_ts_proc_open,
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

static struct notifier_block reboot_notifier;
/*
 * BCB (boot control block) support
 * Handle reboot command and set boot params for bootloader
 */
static int bcb_fts_reboot_hook(struct notifier_block *notifier,
                   unsigned long code, void *cmd)
{
	if (code == SYS_RESTART && cmd && !strcmp(cmd, "recovery")) {
		if (flash_ts_set("bootloader.command", "boot-recovery") ||
				flash_ts_set("bootloader.status", "") ||
				flash_ts_set("bootloader.recovery", ""))
			printk(KERN_ERR "Failed to set bootloader command\n");
	}
	if (code == SYS_RESTART && cmd && !strcmp(cmd, "backupsys")) {
		if (flash_ts_set("bootloader.command", "boot-backupsys") ||
				flash_ts_set("bootloader.status", "") ||
				flash_ts_set("bootloader.recovery", ""))
			printk(KERN_ERR "Failed to set bootloader command\n");
	}

	return NOTIFY_DONE;
}

static int __init flash_ts_init(void)
{
	struct flash_ts_priv *ts;
	struct mtd_info *mtd;
	int res;

	mtd = get_mtd_device_nm(CONFIG_FLASH_TS_PARTITION);
	if (unlikely(IS_ERR(mtd))) {
		printk(KERN_ERR DRV_NAME
		       ": mtd partition '" CONFIG_FLASH_TS_PARTITION
		       "' not found\n");
		return -ENODEV;
	}

	/* we need at least two erase blocks */
	if (unlikely(mtd->size < 2 * mtd->erasesize)) {
		printk(KERN_ERR DRV_NAME ": mtd partition is too small\n");
		res = -ENODEV;
		goto out_put;
	}

	/* make sure both page and block sizes are power-of-2
	 * (this will make chunk size determination simpler).
	 */
	if (unlikely(!is_power_of_2(mtd->writesize) ||
		     !is_power_of_2(mtd->erasesize))) {
		res = -ENODEV;
		printk(KERN_ERR DRV_NAME ": unsupported MTD geometry\n");
		goto out_put;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (unlikely(!ts)) {
		res = -ENOMEM;
		printk(KERN_ERR DRV_NAME ": failed to allocate memory\n");
		goto out_put;
	}

	mutex_init(&ts->lock);
	ts->mtd = mtd;

	/* determine chunk size so it doesn't cross block boundary,
	 * is multiple of page size and there is no wasted space in a block.
	 * We assume page and block sizes are power-of-2.
	 */
	ts->chunk = clp2((sizeof(struct flash_ts) + mtd->writesize - 1) &
			  ~(mtd->writesize - 1));
	if (unlikely(ts->chunk > mtd->erasesize)) {
		res = -ENODEV;
		printk(KERN_ERR DRV_NAME ": MTD block size is too small\n");
		goto out_free;
	}

	/* default empty state */
	set_to_default_empty_state(ts);

	/* scan flash partition for the most recent record */
	res = flash_ts_scan(ts);
	if (unlikely(res))
		goto out_free;

	if (ts->cache.version)
		printk(KERN_INFO DRV_NAME ": v%u loaded from 0x%08llx\n",
		       ts->cache.version, ts->offset);

	/* "Protect" MTD partition from direct user-space write access */
	mtd->flags &= ~MTD_WRITEABLE;

	res = misc_register(&flash_ts_miscdev);
	if (unlikely(res))
		goto out_free;

	smp_mb();
	__ts = ts;

	proc_create(DRV_NAME, 0, NULL, &flash_ts_proc_fops);

	/* Register optional reboot hook */
	reboot_notifier.notifier_call = bcb_fts_reboot_hook;
	register_reboot_notifier(&reboot_notifier);

	return 0;

    out_free:
	kfree(ts);

    out_put:
	put_mtd_device(mtd);
	return res;
}

/* Make sure MTD subsystem is already initialized */
late_initcall(flash_ts_init);
