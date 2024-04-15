/*
 * Copyright (C) 2023 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "msched_sysfs.h"
#include "msched_common.h"

#define MOTO_SCHED_PROC_DIR		"moto_sched"

#define MAX_SET (128)

int moto_sched_enabled;
EXPORT_SYMBOL(moto_sched_enabled);
int moto_sched_scene;
EXPORT_SYMBOL(moto_sched_scene);

pid_t global_task_pid_to_read = -1;
pid_t global_systemserver_tgid = -1;

struct proc_dir_entry *d_moto_sched;

enum {
	OPT_STR_TYPE = 0,
	OPT_STR_PID,
	OPT_STR_VAL,
	OPT_STR_MAX = 3,
};

static ssize_t proc_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	moto_sched_enabled = val;

#if IS_ENABLED(CONFIG_SCHED_WALT)
	set_moto_sched_enabled(moto_sched_enabled);
#endif

	return count;
}

static ssize_t proc_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_sched_enabled);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_ux_scene_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;
	static DEFINE_MUTEX(ux_scene_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	mutex_lock(&ux_scene_mutex);
	moto_sched_scene = val;
#if IS_ENABLED(CONFIG_SCHED_WALT)
	set_ux_scene(moto_sched_scene);
#endif
	mutex_unlock(&ux_scene_mutex);
	return count;
}

static ssize_t proc_ux_scene_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_sched_scene);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

/*
 * echo "w 1211 2" > proc/moto_sched/ux_task
 * set 1611's ux_type -> 2
 *
 * echo "c 1211 2" > proc/moto_sched/ux_task
 * clear 1611's ux_type 2
 *
 * echo "r 1211" > proc/moto_sched/ux_task
 * cat proc/moto_sched/ux_task
 * read thread "1211"'s ux info
 */
static ssize_t proc_ux_task_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_SET];
	char *str, *token;
	char opt_str[OPT_STR_MAX][8] = {"0", "0", "0"};
	int cnt = 0;
	int pid = 0;
	int ux_type = 0;
	int err = 0;
	struct task_struct *ux_task = NULL;
	static DEFINE_MUTEX(ux_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	// at least 2 params.
	if (cnt < OPT_STR_MAX -1)
		return -EFAULT;

	// 2nd param must be pid.
	err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
	if (err || pid <= 0 || pid > PID_MAX_DEFAULT)
		return err;

	// read pid state
	if (!strncmp(opt_str[OPT_STR_TYPE], "r", 1) && cnt == (OPT_STR_MAX - 1)) {
		global_task_pid_to_read = pid;

	// write pid state
	} else if (!strncmp(opt_str[OPT_STR_TYPE], "w", 1) && cnt == OPT_STR_MAX) {
		err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ux_type);
		if (err || ux_type <= 0)
			return err;

		mutex_lock(&ux_mutex);
		rcu_read_lock();
		ux_task = find_task_by_vpid(pid);
		if (ux_task)
			get_task_struct(ux_task);
		rcu_read_unlock();

		if (ux_task && (ux_type & UX_TYPE_PERF_DAEMON)) {
			// perf daemon is in systemserver, so use its tgid.
			global_systemserver_tgid = ux_task->tgid;
		}

		if (ux_task) {
#if IS_ENABLED(CONFIG_SCHED_WALT)
			struct walt_task_struct *wts = (struct walt_task_struct *) ux_task->android_vendor_data1;
			wts->ux_type |= ux_type;
			if (ux_type & UX_TYPE_PERF_DAEMON) {
				set_systemserver_tgid(global_systemserver_tgid);
			}
#endif
			put_task_struct(ux_task);
		}
		mutex_unlock(&ux_mutex);

	// clear pid state
	} else if (!strncmp(opt_str[OPT_STR_TYPE], "c", 1) && cnt == OPT_STR_MAX) {
		err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ux_type);
		if (err || ux_type < 0)
			return err;

		mutex_lock(&ux_mutex);
		rcu_read_lock();
		ux_task = find_task_by_vpid(pid);
		if (ux_task)
			get_task_struct(ux_task);
		rcu_read_unlock();

		if (ux_task) {
#if IS_ENABLED(CONFIG_SCHED_WALT)
			struct walt_task_struct *wts = (struct walt_task_struct *) ux_task->android_vendor_data1;
			if (ux_type == 0) // clear all if pass 0.
				wts->ux_type = 0;
			else
				wts->ux_type &= ~ux_type;
#endif
			put_task_struct(ux_task);
		}
		mutex_unlock(&ux_mutex);
	} else {
		return -EFAULT;
	}

	return count;
}

static ssize_t proc_ux_task_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;
	struct task_struct *task = NULL;
	u16 ux_type = 0;

	task = find_task_by_vpid(global_task_pid_to_read);
	if (task)
		get_task_struct(task);

	if (task) {
#if IS_ENABLED(CONFIG_SCHED_WALT)
		struct walt_task_struct *wts = (struct walt_task_struct *) task->android_vendor_data1;
		ux_type = wts->ux_type;
#endif
		len = snprintf(buffer, sizeof(buffer), "comm=%s pid=%d tgid=%d ux_type=%d\n",
			task->comm, task->pid, task->tgid, ux_type);
		put_task_struct(task);
	} else {
		len = snprintf(buffer, sizeof(buffer), "Can not find task\n");
	}

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_enabled_fops = {
	.proc_write		= proc_enabled_write,
	.proc_read		= proc_enabled_read,
};

static const struct proc_ops proc_ux_scene_fops = {
	.proc_write		= proc_ux_scene_write,
	.proc_read		= proc_ux_scene_read,
};

static const struct proc_ops proc_ux_task_fops = {
	.proc_write		= proc_ux_task_write,
	.proc_read		= proc_ux_task_read,
};

int moto_sched_proc_init(void)
{
	struct proc_dir_entry *proc_node;

	d_moto_sched = proc_mkdir(MOTO_SCHED_PROC_DIR, NULL);
	if (!d_moto_sched) {
		sched_err("failed to create proc dir moto_sched\n");
		goto err_creat_d_moto_sched;
	}

	proc_node = proc_create("enabled", 0666, d_moto_sched, &proc_enabled_fops);
	if (!proc_node) {
		sched_err("failed to create proc node moto_sched_enabled\n");
		goto err_creat_moto_sched_enabled;
	}

	proc_node = proc_create("ux_scene", 0666, d_moto_sched, &proc_ux_scene_fops);
	if (!proc_node) {
		sched_err("failed to create proc node moto_sched_scene\n");
		goto err_creat_moto_sched_scene;
	}

	proc_node = proc_create("ux_task", 0666, d_moto_sched, &proc_ux_task_fops);
	if (!proc_node) {
		sched_err("failed to create proc node ux_task\n");
		goto err_creat_ux_task;
	}

	return 0;

err_creat_ux_task:
	remove_proc_entry("ux_scene", d_moto_sched);

err_creat_moto_sched_scene:
	remove_proc_entry("enabled", d_moto_sched);

err_creat_moto_sched_enabled:
	remove_proc_entry(MOTO_SCHED_PROC_DIR, NULL);

err_creat_d_moto_sched:
	return -ENOENT;
}

void moto_sched_proc_deinit(void)
{
	remove_proc_entry("ux_task", d_moto_sched);
	remove_proc_entry("ux_scene", d_moto_sched);
	remove_proc_entry("enabled", d_moto_sched);
	remove_proc_entry(MOTO_SCHED_PROC_DIR, NULL);
}

