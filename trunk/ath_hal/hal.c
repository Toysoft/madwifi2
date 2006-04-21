/*
 * This file is here only to fool the build system into thinking that
 * there is a source for hal.o
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>

void ath_hal_getwirelessmodes(void) {}
void ath_hal_init_channels(void) {}
char *ath_hal_buildopts[] = { "dummy", NULL };
char *ath_hal_probe(void) { return NULL; }
char ath_hal_version[] = "dummy";
void ath_hal_mhz2ieee(void) {}
void ath_hal_computetxtime(void) {}
void *ath_hal_attach(int a, void *b, int c, void *d, int *status)
{
	*status = 1;
	return NULL;
}
void ath_hal_process_noisefloor(void) {}
