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
#include <linux/mutex.h>
#include <linux/types.h>
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

typedef unsigned long spin_lock_saved_state_t;

struct rpmsg_ivshmem_adapter_eptdev {
	struct rpmsg_device *rpdev;
	struct rpmsg_endpoint *ept;
	struct miscdevice mdev;
	int flags;
	struct jh_kern_pipe *kern_pipe;

	struct mutex cb_mutex; //protect cb and priv_data

	jh_kern_pipe_cb_t cb; //callback for rx messages
	void *priv_data;	//private data for callback
};



static int pipe_send (struct jh_kern_pipe *pipe, const char * buf, int len)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = (struct rpmsg_ivshmem_adapter_eptdev*)pipe->priv_data_impl;
	struct rpmsg_device *rpdev = eptdev->rpdev;
	int status;

	pr_alert("--> %s entry: send %d bytes\n", __func__, len);

	if (!buf)
		return -EINVAL;
	
	if (!eptdev)
	{
		pr_err ("%s: no valid endpoint on that pipe!\n", __func__);
		return -EINVAL;
	}

	
	status = rpmsg_send (rpdev->ept, (void*)buf, len);

	return status;
}

int pipe_set_rx_callback (struct jh_kern_pipe *pipe, void *priv_data, jh_kern_pipe_cb_t cb)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = (struct rpmsg_ivshmem_adapter_eptdev*)pipe->priv_data_impl;
	
	pr_alert("--> %s entry: set rx callback\n", __func__);

	if (!eptdev)
	{
		pr_err ("%s: no valid endpoint on that pipe!\n", __func__);
		return -EINVAL;
	}

	mutex_lock (&(eptdev->cb_mutex));
	
	if (cb != NULL && eptdev->cb != NULL)
	{
		mutex_unlock (&(eptdev->cb_mutex));
		pr_err ("%s: trying to overwrite callback\n", __func__);
		return -EINVAL;		
	}
	eptdev->priv_data = priv_data;
	eptdev->cb = cb;
	mutex_unlock (&(eptdev->cb_mutex));
	return 0;
}

/* RPMSG functions */
static int rpmsg_ivshmem_adapter_cb(struct rpmsg_device *rpdev, void *data, int len,
							void *priv, u32 src)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);
	if (eptdev == NULL)
	{
		pr_err ("%s: Race condition, callback called before driver fully initialized\n", __func__);
		return 0;
	}
	struct jh_kern_pipe *kern_pipe = eptdev->kern_pipe;
	if (kern_pipe == NULL)
	{
		pr_err ("%s: Fatal, eptdev is not initialized correctly\n", __func__);
		return 0;
	}

	if (eptdev->cb)
	{
		eptdev->cb(kern_pipe, eptdev->priv_data, data, len);
	}

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
	
	unsigned val;
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
	
	kernPipe->name = np->name; //PIPE_NAME; //FIXME: should be np->name but we are only creating one of these so okay.
	kernPipe->send = pipe_send;
	kernPipe->set_rx_callback = pipe_set_rx_callback;


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
		goto free_kern_pipe;
	}

	dev_dbg(&rpdev->dev, "rpmsg endpoint created at probe %p\n", rpdev->ept);

	dev_dbg(&rpdev->dev,
			"chnl: 0x%x -> 0x%x : id %d - bufsize %llu - cbuf_size %zu\n",
			rpdev->src, rpdev->dst, ept_param.id, ept_param.bufsize, cbuf_size);

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

	mutex_init (&(eptdev->cb_mutex));
	eptdev->cb = NULL;
	eptdev->priv_data = NULL;

	ret = misc_register(&eptdev->mdev);
	if (ret)
		goto free_eptdev;

	dev_dbg(&rpdev->dev, "got minor %d\n", eptdev->mdev.minor);


	eptdev->rpdev = rpdev;
	eptdev->ept = rpdev->ept;
	dev_set_drvdata(&rpdev->dev, eptdev);
	kernPipe->priv_data_impl = eptdev;


	ret = jh_kern_pipe_register_pipe (kernPipe);
	if (ret)
	{
		pr_alert ("%s: failed to register pipe!\n", __FUNCTION__);
		goto unreg_eptdev;
	}
	pr_alert ("%s: kernel pipe registered\n", __FUNCTION__);

	ret = 0;	

	return ret;

unreg_eptdev:
	misc_deregister (&eptdev->mdev);
free_eptdev:
	kfree(eptdev);
free_ept:
	rpmsg_destroy_ept(rpdev->ept);
free_kern_pipe:
	kfree(kernPipe);


	return ret;
}

static void rpmsg_ivshmem_adapter_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_ivshmem_adapter_eptdev *eptdev = dev_get_drvdata(&rpdev->dev);

	dev_dbg(&rpdev->dev, "--> %s : ept %p\n", __func__, rpdev->ept);


	/* Do not destroy endpoint here. This is handled by rpmsg framework as ept
	 * is stored into rpdev->ept
	 * rpmsg_destroy_ept(rpdev->ept); */
	misc_deregister(&eptdev->mdev);

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
