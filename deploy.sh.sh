#!/usr/bin/bash

# will rmmmod the currently running instance of this module (if any)
# then move the local copy to the target and insmod it

if [ -z ${TARGET_IP_ADDR} ]; then
    echo "Cannot launch."
    echo "Please set environment variable TARGET_IP_ADDR.";
else
    echo "Deploying.."

    # kill jailhouse in case its running, so we don't hot-swap the driver
    ssh root@${TARGET_IP_ADDR} ./jkill.sh

    # remove any old copy of the modules that might be loaded
    ssh root@${TARGET_IP_ADDR} rmmod ivshmem
    ssh root@${TARGET_IP_ADDR} rmmod rpmsg-console
    ssh root@${TARGET_IP_ADDR} rmmod virtual_eth_adapter
    ssh root@${TARGET_IP_ADDR} rmmod rpmsg_ivshmem_adapter
    ssh root@${TARGET_IP_ADDR} rmmod rpmsg-rpc 

    # move it to temp, temporarily
    scp ivshmem.ko root@${TARGET_IP_ADDR}:/tmp
    scp rpmsg-console.ko root@${TARGET_IP_ADDR}:/tmp
    scp rpmsg_ivshmem_adapter.ko root@${TARGET_IP_ADDR}:/tmp
    scp rpmsg-rpc.ko root@${TARGET_IP_ADDR}:/tmp
    scp virtual_eth_adapter.ko root@${TARGET_IP_ADDR}:/tmp

    # execute remote command to move it to the right spot
    ssh root@${TARGET_IP_ADDR} 'mv /tmp/ivshmem.ko /lib/modules/$(uname -r)/extra '
    ssh root@${TARGET_IP_ADDR} 'mv /tmp/rpmsg-console.ko /lib/modules/$(uname -r)/extra '
    ssh root@${TARGET_IP_ADDR} 'mv /tmp/rpmsg_ivshmem_adapter.ko /lib/modules/$(uname -r)/extra '
    ssh root@${TARGET_IP_ADDR} 'mv /tmp/rpmsg-rpc.ko /lib/modules/$(uname -r)/extra '
    ssh root@${TARGET_IP_ADDR} 'mv /tmp/virtual_eth_adapter.ko /lib/modules/$(uname -r)/extra '

    echo "done deploying"
fi


