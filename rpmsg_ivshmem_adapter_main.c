/*!
 * \file rpmsg_ivshmem_adapter.c
 * \author D. Anderson
 * \brief This kernel module wraps ivshmem endpoint functionality and exports
 * an interface so other kernel code can use it
 * 
 * Copyright (c) 2022 Presonus Audio Electronics
 * 
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/div64.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>

#include "ivshmem-rpmsg.h"
#include "ivshmem-iovec.h"
#include "jh_kern_pipe.h"

//TODO: get this from devicetree
#define PIPE_NAME "rpmsg_ivshmem_adapter"

#define MIN(x,y) ((x) < (y) ? x : y)
#define MAX(x,y) ((x) > (y) ? x : y)

#ifndef IVHSM_CONSOLE_BUFFER_SIZE
#define IVHSM_CONSOLE_BUFFER_SIZE (512 * 1024)
#endif

#define to_ivshm_console(x) ((struct rpmsg_cbuf *)(x->priv))

typedef unsigned long spin_lock_saved_state_t;

struct rpmsg_ivshmem_adapter_eptdev {
	struct rpmsg_device *rpdev;
	struct rpmsg_endpoint *ept;
	struct miscdevice mdev;
	wait_queue_head_t readq;
	struct rpmsg_cbuf cbuf;
	int flags;
	struct jh_kern_pipe *kern_pipe;
};

#define mdev_to_eptdev(d) container_of(d, struct rpmsg_ivshmem_adapter_eptdev, mdev)

/* Iface for other kernel code */
int rpmsg_ivshmem_adapter_test (void)
{
	pr_info ("rpmsg_ivshmem_adapter_test()\n");
	return 0;
}
EXPORT_SYMBOL(rpmsg_ivshmem_adapter_test);



static ssize_t rpmsg_ivshmem_adapter_eptdev_read(struct file *filp, char *buf,
										size_t len, loff_t *f_pos)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = mdev_to_eptdev(filp->private_data);
	struct rpmsg_cbuf *cbuf = &eptdev->cbuf;
	iovec_t vec[2];
	unsigned count, i;
	ssize_t ret;
	size_t sz, remaining, offset;
	unsigned long long num_chars;
	loff_t iovec_ppos;

	ret = wait_event_interruptible(eptdev->readq, cbuf->avail > 0);
	if (ret)
		goto out;

	spin_lock(&cbuf->lock);
	count = rpmsg_cbuf_get_buffer(cbuf, vec);
	if (count ==  0) {
		ret = 0;
		goto out;
	}

	spin_unlock(&cbuf->lock);

	offset = 0;
	remaining = len;

	for (i = 0; i < count && remaining > 0; i++) {
		sz = MIN(remaining, vec[i].iov_len);
		iovec_ppos = (loff_t) (vec[i].iov_base - (void *)cbuf->buf);

		ret = simple_read_from_buffer(buf + offset, sz, &iovec_ppos, cbuf->buf, cbuf->size);

		if (ret > 0)
			*f_pos += ret;
		else
			break;

		BUG_ON(ret != sz);

		/* Update cbuf pointers */
		spin_lock(&cbuf->lock);
		num_chars = cbuf->tail + ret;
		cbuf->tail = do_div(num_chars, cbuf->size);
		cbuf->avail -= ret;
		remaining -= ret;
		spin_unlock(&cbuf->lock);

		offset += ret;
	}

	ret = (offset > 0) ? offset : ret;

out:
	return ret;
}

static ssize_t rpmsg_ivshmem_adapter_eptdev_write(struct file *filp, const char __user *buf,
										 size_t len, loff_t *f_pos)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = mdev_to_eptdev(filp->private_data);
	struct rpmsg_device *rpdev = eptdev->rpdev;
	void *kbuf;
	int ret;

	pr_debug("--> %s entry: send %lu bytes\n", __func__, len);

	kbuf = memdup_user_nul(buf, len);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	ret = rpmsg_send(rpdev->ept, kbuf, len + 1);

	kfree(kbuf);
	return (ret == 0) ? len : 0;
}


static unsigned int rpmsg_ivshmem_adapter_eptdev_poll(struct file *filp, poll_table *wait)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = mdev_to_eptdev(filp->private_data);
	struct rpmsg_cbuf *cbuf;
	unsigned int mask = 0;

	if (!eptdev || !eptdev->ept)
		return POLLERR;

	poll_wait(filp, &eptdev->readq, wait);

	cbuf = &eptdev->cbuf;
	if (cbuf->avail > 0) {
		mask |= POLLIN | POLLRDNORM; /* readable */
		return mask;
	}

	return mask;
}


/* RPMSG functions */
static int rpmsg_ivshmem_adapter_cb(struct rpmsg_device *rpdev, void *data, int len,
							void *priv, u32 src)
{
	struct rpmsg_cbuf *console = to_ivshm_console(rpdev->ept);
	size_t count, room = rpmsg_cbuf_space_avail(console);
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_cbuf *cbuf = &eptdev->cbuf;
	size_t room_cbuf = rpmsg_cbuf_space_avail(cbuf);

	/* If needed, free up some space in cbuf to store new buffer */
	if (len > room)
		rpmsg_cbuf_read(console, NULL, len - room);

	count = rpmsg_cbuf_write(console, data, len);
	WARN_ON(count != len);

	// TODO: kernel-level notify?
	// sysfs_notify(&rpdev->dev.kobj, NULL, "console");

	if (len > room_cbuf)
		rpmsg_cbuf_read(cbuf, NULL, len - room);

	/* Save received data into cbuf */
	count = rpmsg_cbuf_write(cbuf, data, len);

	BUG_ON(cbuf->avail > cbuf->size);

	WARN_ON(count != len);

	/* wake up any blocking processes waiting for data */
	wake_up_interruptible(&eptdev->readq);

	return 0;
}

static int pipeSend (struct jh_kern_pipe *pipe, const char * buf, int len)
{
	pr_alert ("%s\n", __FUNCTION__);
	return 0;
}

static int rpmsg_ivshmem_adapter_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo = {
		.src = rpdev->src,
		.dst = RPMSG_ADDR_ANY
	};

	struct device_node *np = rpdev->dev.of_node;
	struct rpmsg_ivshmem_adapter_eptdev *eptdev;
	struct ivshm_ept_param ept_param = { 0 };
	struct rpmsg_cbuf *cbuf;
	struct rpmsg_cbuf *console;
	unsigned val;
	size_t cbuf_size = IVHSM_CONSOLE_BUFFER_SIZE;
	int ret;
	struct jh_kern_pipe * kernPipe;

	pr_alert ("rpmsg_ivshmem_adapter probed, porra!\n");

	// Create and register a jh_kern_pipe
	kernPipe = kzalloc (sizeof(*kernPipe), GFP_KERNEL);
	if (!kernPipe)
	{
		pr_err ("%s: could not allocate jh_kern_pipe!\n", __FUNCTION__);
		return -ENOMEM;
	}

	kernPipe->name = PIPE_NAME;
	kernPipe->send = pipeSend;

	ret = jh_kern_pipe_register_pipe (kernPipe);
	if (ret)
	{
		pr_alert ("%s: failed to register pipe!\n", __FUNCTION__);
		goto free_kern_pipe;
	}
	pr_alert ("%s: kernel pipe registered\n", __FUNCTION__);




	/* Prepare console logger */
	console = kzalloc(sizeof(*console), GFP_KERNEL);
	if (!console)
	{
		ret = -ENOMEM;
		goto unreg_kern_pipe;
	}

	console->size = IVHSM_CONSOLE_BUFFER_SIZE;
	console->head = 0;
	console->tail = 0;
	console->buf = vmalloc(console->size);
	if (!console->buf) {
		kfree(console);
		return -ENOMEM;
	}

	ept_param.priv = console;

	/* Retrieve information from device tree */
	ret = of_property_read_u32(np, "id", &val);
	if (ret)
		val = 0;
	ept_param.id = val;

	ret = of_property_read_u32(np, "size", &val);
	if (!ret)
		ept_param.bufsize = (size_t)val;

	dev_dbg(&rpdev->dev, "chnl: 0x%x -> 0x%x : id %d : bufsize %llu\n", rpdev->src,
			 rpdev->dst, ept_param.id, ept_param.bufsize);

	/* Create endpoint */
	rpdev->ept = rpmsg_create_ept(rpdev, rpmsg_ivshmem_adapter_cb, &ept_param, chinfo);

	if (IS_ERR(rpdev->ept)) {
		ret = PTR_ERR(rpdev->ept);
		goto free_console;
	}

	dev_dbg(&rpdev->dev, "rpmsg endpoint created at probe %p\n", rpdev->ept);

	dev_dbg(&rpdev->dev,
			"chnl: 0x%x -> 0x%x : id %d - bufsize %llu - cbuf_size %zu\n",
			rpdev->src, rpdev->dst, ept_param.id, ept_param.bufsize, cbuf_size);

	dev_dbg(&rpdev->dev, "rpmsg endpoint created at probe %p\n", rpdev->ept);

	/* Setup miscdevice */
	eptdev = kzalloc(sizeof(*eptdev), GFP_KERNEL);
	if (!eptdev) {
		ret = -ENOMEM;
		goto free_ept;
	}

	eptdev->mdev.minor = MISC_DYNAMIC_MINOR;
	eptdev->mdev.name = np->name;
	eptdev->mdev.fops = NULL;
	eptdev->mdev.parent = &rpdev->dev;
	eptdev->kern_pipe = kernPipe;

	ret = misc_register(&eptdev->mdev);
	if (ret)
		goto free_eptdev;

	dev_dbg(&rpdev->dev, "got minor %d\n", eptdev->mdev.minor);

	/* Prepare circular buffer */
	cbuf = &eptdev->cbuf;
	cbuf->size = cbuf_size;
	cbuf->buf = vmalloc(cbuf->size);
	if (!cbuf->buf)
		goto free_eptdev;

	cbuf->tail = cbuf->head = 0;
	spin_lock_init(&cbuf->lock);

	init_waitqueue_head(&eptdev->readq);
	eptdev->rpdev = rpdev;
	eptdev->ept = rpdev->ept;
	dev_set_drvdata(&rpdev->dev, eptdev);

	return ret;

free_eptdev:
	kfree(eptdev);
free_ept:
	rpmsg_destroy_ept(rpdev->ept);
free_console:
	vfree(console->buf);
	kfree(console);
unreg_kern_pipe:
	jh_kern_pipe_unregister_pipe (kernPipe);
free_kern_pipe:
	kfree(kernPipe);

	return ret;
}

static void rpmsg_ivshmem_adapter_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_cbuf *console = to_ivshm_console(rpdev->ept);
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);

	dev_dbg(&rpdev->dev, "--> %s : ept %p\n", __func__, rpdev->ept);

	/* Free console structure and associated buf */
	vfree(console->buf);
	kfree(console);

	/* Do not destroy endpoint here. This is handled by rpmsg framework as ept
	 * is stored into rpdev->ept
	 * rpmsg_destroy_ept(rpdev->ept); */
	misc_deregister(&eptdev->mdev);
	vfree(eptdev->cbuf.buf);

	jh_kern_pipe_unregister_pipe (eptdev->kern_pipe);
	kfree (eptdev->kern_pipe);
	kfree(eptdev);

	pr_alert ("rpmsg_ivshmem_adapter removed!\n");
}

static const struct of_device_id rpmsg_ivshmem_adapter_of_match[] = {
	{ .compatible = "rpmsg_ivshmem_adapter" },
	{}
};
MODULE_DEVICE_TABLE(of, rpmsg_ivshmem_adapter_of_match);

static struct rpmsg_driver rpmsg_ivshmem_adapter_driver = {
	.drv = {
		.name   = KBUILD_MODNAME,
		.owner  = THIS_MODULE,
		.of_match_table = rpmsg_ivshmem_adapter_of_match,
	},
	.probe      = rpmsg_ivshmem_adapter_probe,
	.remove     = rpmsg_ivshmem_adapter_remove,
};
module_rpmsg_driver(rpmsg_ivshmem_adapter_driver);

MODULE_AUTHOR("Antoine Bouyer <antoine.bouyer@nxp.com>");
MODULE_DESCRIPTION("NXP rpmsg device driver for LK console");
MODULE_LICENSE("GPL v2");

