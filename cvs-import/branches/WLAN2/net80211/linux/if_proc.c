#include "ath-linux.h"
#include "ieee80211_var.h"

#ifdef CONFIG_PROC_FS
#ifdef IEEE80211_DEBUG
#include <linux/proc_fs.h>
#include <linux/ctype.h>

#include "if_stats.h"

#include "if_proc.h"

static int
ieee80211_proc_debug_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	struct ieee80211com *ic = (struct ieee80211com *)data;
	if (off != 0) {
		*eof = 1;
		return 0;
	}
	return sprintf(page, "%d\n", ic->ieee80211_debug);
}

static int
ieee80211_proc_debug_write(struct file *file, const char *buf,
			   unsigned long count, void *data)
{
	struct ieee80211com *ic = (struct ieee80211com *)data;
	int v;
	
	if (sscanf(buf, "%d", &v) == 1) {
		ic->ieee80211_debug = v;
		return count;
	} else
		return -EINVAL;
}

void
ieee80211_proc_init(struct ieee80211com *ic, char const *dev_name)
{
	struct proc_dir_entry *dp;

	snprintf(ic->ic_stats.ic_procname, sizeof(ic->ic_stats.ic_procname), "wlan-%s", dev_name);
	ic->ic_stats.ic_proc = proc_mkdir(ic->ic_stats.ic_procname, proc_net);
	if (ic->ic_stats.ic_proc == NULL) {
		printk(KERN_INFO "/proc/net/%s: failed to create\n",
		       ic->ic_stats.ic_procname);
		return;
	}
	dp = create_proc_entry("debug", 0644, ic->ic_stats.ic_proc);
	if (dp) {
		dp->read_proc = ieee80211_proc_debug_read;
		dp->write_proc = ieee80211_proc_debug_write;
		dp->data = ic;
	}
}

void
ieee80211_proc_remove(struct ieee80211com *ic)
{
	if (ic->ic_stats.ic_proc != NULL) {
		remove_proc_entry("debug", ic->ic_stats.ic_proc);
		remove_proc_entry(ic->ic_stats.ic_procname, proc_net);
		ic->ic_stats.ic_proc = NULL;
	}
}
#endif /* IEEE80211_DEBUG */
#endif /* CONFIG_PROC_FS */
