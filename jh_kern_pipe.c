
#define DEBUG 1

#include "jh_kern_pipe.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>

#define MAXCOMPLEN (256)

static LIST_HEAD(jh_kern_pipe_list);

static struct list_head * find_pipe (struct list_head * list_to_search, struct jh_kern_pipe *pipe)
{
	struct list_head *it_list_head;
	struct jh_kern_pipe *it_pipe;
	bool found = false;

	if (!list_to_search || !pipe)
	{
		return NULL;
	}

	list_for_each (it_list_head, list_to_search)
	{
		it_pipe = list_entry(it_list_head, struct jh_kern_pipe, list);
		if (0 == strncmp(pipe->name, it_pipe->name, MAXCOMPLEN) )
		{
			found = true;
			break;
		}
	}

	if (found)
		return it_list_head;
	else
		return NULL;	
}

int jh_kern_pipe_register_pipe (struct jh_kern_pipe *pipe)
{
	struct list_head * it;

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

	it = find_pipe (&jh_kern_pipe_list, pipe);
	if (it)
	{
		pr_err ("%s: user tried to register a pipe that is already registered: %s\n",
			__FUNCTION__, pipe->name);
		return -EINVAL;
	}

	pr_debug ("adding pipe \'%s\'\n", pipe->name);

	list_add_tail (&pipe->list, &jh_kern_pipe_list);

	return 0;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_register_pipe);

int jh_kern_pipe_unregister_pipe (struct jh_kern_pipe *pipe)
{
	struct list_head *listPtr;

	if (!pipe)
	{
		pr_err ("%s: pipe does not exist!\n", __FUNCTION__);
		return -EINVAL;
	}

	listPtr = find_pipe (&jh_kern_pipe_list, pipe);
	
	if (listPtr)
	{
		list_del (listPtr);
		pr_debug ("%s: unregistering pipe \'%s\'\n", __FUNCTION__, pipe->name);
		return 0;
	}

	pr_err ("%s: could not find pipe!\n", __FUNCTION__);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_unregister_pipe);

struct jh_kern_pipe *jh_kern_pipe_get_pipe_by_name (const char *name)
{
	struct list_head *it;
	struct jh_kern_pipe *it_pipe;

	if (!name)
		return NULL;

	list_for_each (it, &jh_kern_pipe_list)
	{
		it_pipe = list_entry(it, struct jh_kern_pipe, list);
		if (0 == strncmp(it_pipe->name, name, MAXCOMPLEN) )
		{
			return it_pipe;
		}
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(jh_kern_pipe_get_pipe_by_name);

void jh_kern_pipe_print_all_pipes (void)
{
	struct list_head *it;
	struct jh_kern_pipe *it_pipe;

	pr_info ("jh_kern_pipe list:\n");
	list_for_each (it, &jh_kern_pipe_list)
	{
		it_pipe = list_entry(it, struct jh_kern_pipe, list);
		if (it_pipe->name)
		{
			pr_info ("	%s\n", it_pipe->name);
		}
	}
}
EXPORT_SYMBOL(jh_kern_pipe_print_all_pipes);

int jh_kern_pipe_register_callback (struct jh_kern_pipe *pipe, jh_kern_pipe_cb_t callback, void *priv_data)
{
	if (!pipe)
		return -EINVAL;
	
	if (pipe->rx_callback)
	{
		pr_err ("%s: pipe already has a registered callback!\n", __FUNCTION__);
		return -EPERM;
	}
	
	pipe->rx_callback = callback;
	pipe->priv_data = priv_data;
	return 0;
}
EXPORT_SYMBOL(jh_kern_pipe_register_callback);

int jh_kern_pipe_unregister_callback (struct jh_kern_pipe *pipe)
{
	if (!pipe)
		return -EINVAL;
	
	pipe->rx_callback = NULL;
	pipe->priv_data = NULL;
	return 0;
}
EXPORT_SYMBOL(jh_kern_pipe_unregister_callback);
