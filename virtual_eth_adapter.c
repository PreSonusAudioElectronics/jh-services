/*!
 * \file virtual_eth_adapter.c
 * \author D. Anderson
 * \brief This kernel module wraps ivshmem endpoint functionality and exports
 * an interface so other kernel code can use it
 * 
 * Copyright (c) 2022 Presonus Audio Electronics
 * 
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/div64.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>

#include "rpmsg_ivshmem_adapter.h"


typedef unsigned long spin_lock_saved_state_t;


static int virtual_eth_adapter_probe(struct rpmsg_device *rpdev)
{
	pr_alert ("virtual_eth_adapter probed!\n");

	pr_alert ("waiting to make sure ivshmem adapter is up..\n");

	msleep(50);

	pr_alert ("trying ivshmem adapter test..\n");

	rpmsg_ivshmem_adapter_test();

	return 0;
}

static void virtual_eth_adapter_remove(struct rpmsg_device *rpdev)
{
	pr_alert ("virtual_eth_adapter removed!\n");
}

static const struct of_device_id virtual_eth_adapter_of_match[] = {
	{ .compatible = "virtual_eth_adapter" },
	{}
};
MODULE_DEVICE_TABLE(of, virtual_eth_adapter_of_match);

static struct rpmsg_driver virtual_eth_adapter_driver = {
	.drv = {
		.name   = KBUILD_MODNAME,
		.owner  = THIS_MODULE,
		.of_match_table = virtual_eth_adapter_of_match,
	},
	.probe      = virtual_eth_adapter_probe,
	.remove     = virtual_eth_adapter_remove,
};
module_rpmsg_driver(virtual_eth_adapter_driver);

MODULE_AUTHOR("Dave Anderson");
MODULE_DESCRIPTION("PAE virtual rtos ethernet adapter");
MODULE_LICENSE("closed");

