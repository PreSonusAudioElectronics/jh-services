
#define DEBUG 1

#include "jh_kern_pipe.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>

#define MAXCOMPLEN (128)

LIST_HEAD(jh_kern_pipe_list);
EXPORT_SYMBOL_GPL(jh_kern_pipe_list);


int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe)
{
	if (!pipe)
	{
		pr_err ("%s: bad argument!\n", __FUNCTION__);
		return -EINVAL;
	}

	if (!pipe->name)
	{
		pr_err ("%s: pipe has no name!\n", __FUNCTION__);
		return -EINVAL;
	}

	pr_debug ("adding pipe \'%s\'\n", pipe->name);

	msleep (30);

	list_add_tail (&pipe->list, &jh_kern_pipe_list);

	return 0;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_register_pipe);

int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe)
{
	struct list_head *listPtr;
	struct jh_kern_pipe *it;
	bool found = false;

	if (!pipe)
	{
		pr_err ("%s: pipe does not exist!\n", __FUNCTION__);
		return -EINVAL;
	}

	list_for_each (listPtr, &jh_kern_pipe_list )
	{
		it = list_entry(listPtr, struct jh_kern_pipe, list);
		if (0 == strncmp(pipe->name, it->name, MAXCOMPLEN) )
		{
			found = true;
			break;
		}
	}
	
	if (found)
	{
		list_del (listPtr);
		pr_debug ("%s: deleting pipe \'%s\'\n", __FUNCTION__, pipe->name);
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_unregister_pipe);

struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name)
{
	return NULL;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_get_pipe_by_name);

