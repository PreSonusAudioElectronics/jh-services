/*!
 * \file rpmsg_ivshmem_adapter.h
 * \author D. Anderson
 * \brief expose rpmsg_ivshmem_adapter functions for other modules to use
 * 
 * Copyright (c) 2022 Presonus Audio Electronics
 * 
 */

#ifndef _rpmsg_ivshmem_adapter_h
#define _rpmsg_ivshmem_adapter_h

typedef void (*rpmsg_ivshmem_cb_t)(char * buf, int len);

extern int rpmsg_ivshmem_adapter_test (void);

extern int rpmsg_ivshmem_adapter_send (char * buf, int len);

extern int rpmsg_ivshmem_adapter_register_cb (rpmsg_ivshmem_cb_t callback);

extern int rpmsg_ivshmem_adapter_receive (char * buf, int len);

#endif // _rpmsg_ivshmem_adapter_h
