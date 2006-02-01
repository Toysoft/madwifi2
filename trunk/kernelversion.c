/* This file is used for a trick to determine the version of the kernel
 * build tree. Simply grepping <linux/version.h> doesn't work, since
 * some distributions have multiple UTS_RELEASE definitions in that
 * file.
 * Taken from the lm_sensors project.
 *
 * $Id$
 */
#include <linux/version.h>
char *uts_release = UTS_RELEASE;
