
#ifndef _jh_kern_pipe_h
#define _jh_kern_pipe_h

#include <linux/list.h>
#include <linux/wait.h>
#include <linux/types.h>

struct jh_kern_pipe;

typedef void (*jh_kern_pipe_cb_t)(struct jh_kern_pipe *pipe, size_t written);
typedef void (*jh_kern_pipe_txdone_cb_t)(struct jh_kern_pipe *pipe, void *priv_data);

struct jh_kern_pipe 
{
	const char *name;

	/*!
	 * \brief Blocking. Return num sent or err.
	 */
	int (*send) (struct jh_kern_pipe *pipe, const char *buf, int len);

	/*!
	 * \brief Non-blocking. Return success or err.
	 * tx_complete_event_q will be notified when transmission completes
	 * Return 0 if send was initiated (does not mean it was completed),
	 * otherwise error
	 */
	int (*send_nonblock) (struct jh_kern_pipe *pipe, const char *buf, int len);

	/*!
	 * \brief Non-blocking.
	 * - returns immediately with number of bytes read or error
	 */
	int (*read) (struct jh_kern_pipe *pipe, char *buf, int len);
	bool (*data_ready) (struct jh_kern_pipe *pipe);
	void *priv_data_impl;
	struct list_head list;
	wait_queue_head_t read_event_q; // will get notified when data is put to the pipe from the other side

	/*!
	 *	Will get notified on completion of transmit
	 */
	wait_queue_head_t tx_complete_event_q;
	bool nonblock_tx_completed;

};

// Called by pipe implementers
int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe);
int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe);


struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name);

void jh_kern_pipe_print_all_pipes (void);

#endif // _jh_kern_pipe_h

