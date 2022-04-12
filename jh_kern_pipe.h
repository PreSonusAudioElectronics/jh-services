
#ifndef _jh_kern_pipe_h
#define _jh_kern_pipe_h

#include <linux/list.h>

extern struct list_head jh_kern_pipe_list;

struct jh_kern_pipe;

typedef void (*jh_kern_pipe_cb_t)(char * buf, int len, void *priv_data);

struct jh_kern_pipe 
{
    const char *name;
    int (*send) (struct jh_kern_pipe * pipe, const char * buf, int len);
    struct list_head list;
};

int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe);
int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe);

int jh_kern_pipe_register_callback (struct jh_kern_pipe *pipe, jh_kern_pipe_cb_t callback, void *priv_data);

struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name);

#endif // _jh_kern_pipe_h

