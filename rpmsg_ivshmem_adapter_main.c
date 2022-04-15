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
#include <linux/kthread.h>

#include "ivshmem-rpmsg.h"
#include "ivshmem-iovec.h"
#include "jh_kern_pipe.h"

//TODO: get this from devicetree
#define PIPE_NAME "rpmsg_ivshmem_adapter"
#define THREADNAME_MAX_LEN (128)

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
	struct task_struct *kthread;
};

#define mdev_to_eptdev(d) container_of(d, struct rpmsg_ivshmem_adapter_eptdev, mdev)

/* Iface for other kernel code */
int rpmsg_ivshmem_adapter_test (void)
{
	pr_info ("rpmsg_ivshmem_adapter_test()\n");
	return 0;
}
EXPORT_SYMBOL(rpmsg_ivshmem_adapter_test);




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


static int rpmsg_ivshmem_adapter_rx_thread_func (void *pv)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = (struct rpmsg_ivshmem_adapter_eptdev *)pv;
	struct rpmsg_cbuf *cbuf = &eptdev->cbuf;
	struct jh_kern_pipe *kern_pipe = eptdev->kern_pipe;
	char * rxbuf = kern_pipe->rxbuf;
	unsigned rxbufsize = kern_pipe->rxbuf_size;
	size_t chars_read;
	size_t in_buffer;
	iovec_t iovec;

	pr_alert ("%s: starting rx thread loop for %s\n", __func__, eptdev->kern_pipe->name);

	while (!kthread_should_stop() )
	{
		if (kern_pipe->rx_callback)
		{
			in_buffer = rpmsg_cbuf_get_buffer (cbuf, &iovec);
			chars_read = rpmsg_cbuf_read (cbuf, rxbuf, rxbufsize);
			kern_pipe->rx_callback (rxbuf, chars_read, kern_pipe->priv_data);
		}

		wait_event_interruptible (eptdev->readq, 
			cbuf->avail > 0 || kthread_should_stop() );
	}

	do_exit(0);
}


/* RPMSG functions */
static int rpmsg_ivshmem_adapter_cb(struct rpmsg_device *rpdev, void *data, int len,
							void *priv, u32 src)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_cbuf *cbuf = &eptdev->cbuf;
	size_t room_cbuf = rpmsg_cbuf_space_avail(cbuf);
	int discarded;
	size_t count;

	if (len > room_cbuf)
	{
		discarded = len - room_cbuf;
		rpmsg_cbuf_read(cbuf, NULL, discarded);
		pr_err ("%s: dropped %d bytes!\n", __func__, discarded );
	}

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
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = (struct rpmsg_ivshmem_adapter_eptdev*)pipe->priv_data;
	struct rpmsg_device *rpdev = eptdev->rpdev;

	pr_alert("--> %s entry: send %d bytes\n", __func__, len);

	if (!buf)
		return -EINVAL;
	
	if (!eptdev)
	{
		pr_err ("%s: no valid endpoint on that pipe!\n", __func__);
		return -EINVAL;
	}

	return rpmsg_send(rpdev->ept, (void*)buf, len + 1);
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
	unsigned val;
	size_t cbuf_size = IVHSM_CONSOLE_BUFFER_SIZE;
	int ret;
	struct jh_kern_pipe * kernPipe;
	char namebuf[THREADNAME_MAX_LEN];

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

	/* Retrieve information from device tree */
	ret = of_property_read_u32(np, "id", &val);
	if (ret)
		val = 0;
	ept_param.id = val;

	ret = of_property_read_u32(np, "size", &val);
	if (!ret)
		ept_param.bufsize = (size_t)val;
	
	// Set rxbuf according to rpmsg buffer size
	kernPipe->rxbuf = kzalloc (val + 1, GFP_KERNEL);
	if (!kernPipe->rxbuf)
	{
		pr_err ("%s: failed to allocate rx buffer!\n", __func__);
		goto unreg_kern_pipe;
	}
	kernPipe->rxbuf_size = val + 1;

	dev_dbg(&rpdev->dev, "chnl: 0x%x -> 0x%x : id %d : bufsize %llu\n", rpdev->src,
			 rpdev->dst, ept_param.id, ept_param.bufsize);

	/* Create endpoint */
	rpdev->ept = rpmsg_create_ept(rpdev, rpmsg_ivshmem_adapter_cb, &ept_param, chinfo);

	if (IS_ERR(rpdev->ept)) {
		ret = PTR_ERR(rpdev->ept);
		goto free_rxbuf;
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
	kernPipe->priv_data = eptdev;

	dev_dbg (&rpdev->dev, "starting rx thread..\n");
	
	ret = snprintf (namebuf, THREADNAME_MAX_LEN, "rpmsg_ivshmem_rx_thread_%d", ept_param.id);
	if (ret >= THREADNAME_MAX_LEN)
	{
		dev_dbg (&rpdev->dev, "thread name was truncated\n");
	}
	ret = 0;


	eptdev->kthread = kthread_run (rpmsg_ivshmem_adapter_rx_thread_func, 
		eptdev, namebuf);
	if (eptdev->kthread)
	{
		dev_dbg (&rpdev->dev, "rx thread started for id 0x%x\n", ept_param.id);
	}
	else
	{
		dev_err (&rpdev->dev, "Failed to start RX thread for id 0x%x!\n", ept_param.id);
		ret = -ENOMEM;
		goto free_eptdev;
	}
	

	return ret;

free_eptdev:
	kfree(eptdev);
free_ept:
	rpmsg_destroy_ept(rpdev->ept);
free_rxbuf:
	kfree (kernPipe->rxbuf);
unreg_kern_pipe:
	jh_kern_pipe_unregister_pipe (kernPipe);
free_kern_pipe:
	kfree(kernPipe);

	return ret;
}

static void rpmsg_ivshmem_adapter_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);
	struct jh_kern_pipe *kern_pipe = eptdev->kern_pipe;
	int ret;

	dev_dbg(&rpdev->dev, "--> %s : ept %p\n", __func__, rpdev->ept);

	ret = kthread_stop (eptdev->kthread);
	if (ret)
	{
		dev_dbg (&rpdev->dev, "failed to stop rx thread with: %d\n", ret);
	}

	/* Free console structure and associated buf */

	/* Do not destroy endpoint here. This is handled by rpmsg framework as ept
	 * is stored into rpdev->ept
	 * rpmsg_destroy_ept(rpdev->ept); */
	misc_deregister(&eptdev->mdev);
	vfree(eptdev->cbuf.buf);

	if (kern_pipe->rxbuf)
		kfree (kern_pipe->rxbuf);
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


#if 0

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

#endif