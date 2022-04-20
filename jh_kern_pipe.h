
#ifndef _jh_kern_pipe_h
#define _jh_kern_pipe_h

#include <linux/list.h>
#include <linux/wait.h>
#include <linux/types.h>

struct jh_kern_pipe;

typedef void (*jh_kern_pipe_cb_t)(struct jh_kern_pipe *pipe, size_t written);

struct jh_kern_pipe 
{
	const char *name;
	int (*send) (struct jh_kern_pipe *pipe, const char *buf, int len);
	int (*read) (struct jh_kern_pipe *pipe, char *buf, int len);
	bool (*data_ready) (struct jh_kern_pipe *pipe);
	jh_kern_pipe_cb_t rx_callback;
	void *priv_data;
	struct list_head list;
	wait_queue_head_t read_event_q; // will get notified when data is put to the pipe from the other side
};

// Called by pipe implementers
int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe);
int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe);


// Called by pipe users
int jh_kern_pipe_register_callback (struct jh_kern_pipe *pipe, jh_kern_pipe_cb_t callback, void *priv_data);
int jh_kern_pipe_unregister_callback (struct jh_kern_pipe *pipe);

struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name);

void jh_kern_pipe_print_all_pipes (void);

#endif // _jh_kern_pipe_h

