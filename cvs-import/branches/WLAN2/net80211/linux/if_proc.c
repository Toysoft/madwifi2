#include "ath-linux.h"
#include "ieee80211_var.h"

#ifdef CONFIG_PROC_FS
#ifdef IEEE80211_DEBUG
#include <linux/proc_fs.h>
#include <linux/ctype.h>

#include "if_stats.h"

#include "if_proc.h"

extern int      ieee80211_debug;

static int
ieee80211_proc_debug_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	if (off != 0) {
		*eof = 1;
		return 0;
	}
	return sprintf(page, "%d\n", ieee80211_debug);
}

static int
ieee80211_proc_debug_write(struct file *file, const char *buf,
			   unsigned long count, void *data)
{
	/*struct ieee80211_stats *stats = data;*/
	int v;
	
	if (sscanf(buf, "%d", &v) == 1) {
		ieee80211_debug = v;
		return count;
	} else
		return -EINVAL;
}

void
ieee80211_proc_init(struct ieee80211_stats *stats, char const *dev_name)
{
	struct proc_dir_entry *dp;

	snprintf(stats->ic_procname, sizeof(stats->ic_procname), "wlan-%s", dev_name);
	stats->ic_proc = proc_mkdir(stats->ic_procname, proc_net);
	if (stats->ic_proc == NULL) {
		printk(KERN_INFO "/proc/net/%s: failed to create\n",
		       stats->ic_procname);
		return;
	}
	dp = create_proc_entry("debug", 0644, stats->ic_proc);
	if (dp) {
		dp->read_proc = ieee80211_proc_debug_read;
		dp->write_proc = ieee80211_proc_debug_write;
		dp->data = stats;
	}
}

void
ieee80211_proc_remove(struct ieee80211_stats *stats)
{
	if (stats->ic_proc != NULL) {
		remove_proc_entry("debug", stats->ic_proc);
		remove_proc_entry(stats->ic_procname, proc_net);
		stats->ic_proc = NULL;
	}
}
#endif /* IEEE80211_DEBUG */
#endif /* CONFIG_PROC_FS */
