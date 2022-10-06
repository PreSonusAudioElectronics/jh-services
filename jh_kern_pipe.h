
#ifndef _jh_kern_pipe_h
#define _jh_kern_pipe_h

#include <linux/list.h>
#include <linux/wait.h>
#include <linux/types.h>

struct jh_kern_pipe;

typedef void (*jh_kern_pipe_cb_t)(struct jh_kern_pipe *pipe, void *priv_data, const char *buf, int len);


struct jh_kern_pipe 
{
	const char *name;

	/*!
	 * Send a single message of len bytes to the peer.
	 * \param	pipe	pointer to the pipe
	 * \param	buf		pointer to buffer with message to send
	 * \param	len		the exact number of bytes to send for this message
	 * \return	0 if success and -1 in case of an error (unlikely)
	 */
	int (*send) (struct jh_kern_pipe *pipe, const char *buf, int len);


	/*!
	 * Set a call back for receiving messages from the peer. The callback will be called when
	 * a message has been received.
	 * 
	 * \param	pipe		pointer to the pipe
	 * \param	priv_data	private data pointer provided in the callback
	 * \param	cb			the callback to be called. Use NULL to ininstall the callback
	 * \return	0 if success and -1 in case of an error (overwriting existing callback)
	 */
	int (*set_rx_callback) (struct jh_kern_pipe *pipe, void *priv_data, jh_kern_pipe_cb_t cb);


	void *priv_data_impl;	//!< private pointer from the creator of the pipe
	struct list_head list;	//!< douple linked list element

};

// Called by pipe implementers
int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe);
int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe);


struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name);

void jh_kern_pipe_print_all_pipes (void);

#endif // _jh_kern_pipe_h

