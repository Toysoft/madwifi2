/*-
 * Copyright (c) 2005 John Bicket
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_RATE_SAMPLE_H
#define _DEV_ATH_RATE_SAMPLE_H

/* per-device state */
struct sample_softc {
	struct ath_ratectrl arc;	/* base state */
};

struct rate_info {
  int rate;
  int rix;
  int rateCode;
  int shortPreambleRateCode;

  int average_tx_time;
  int successive_failures;
  int tries;
  int packets_acked;
  int perfect_tx_time; /* transmit time for 0 retries */
};


/* per-node state */
struct sample_node {
  int packets_sent;

  int static_rate_ndx;
  int num_rates;

  struct rate_info rates[IEEE80211_RATE_MAXSIZE];
  int sample_num;
};


#define WIFI_SLOT_B 20
#define WIFI_DIFS_B 50
#define WIFI_SIFS_B 10
#define WIFI_ACK_B 304
#define WIFI_PLCP_HEADER_LONG_B 192
#define WIFI_PLCP_HEADER_SHORT_B 192

#define WIFI_SLOT_A 9
#define WIFI_DIFS_A 28
#define WIFI_SIFS_A 9
#define WIFI_ACK_A 30
#define WIFI_PLCP_HEADER_A 20


#define is_b_rate(b) ((b == 2) || (b == 4) || (b == 11) || (b == 22))

#define WIFI_CW_MIN 31
#define WIFI_CW_MAX 1023

#define	MIN(a,b)	((a) < (b) ? (a) : (b))
#define	MAX(a,b)	((a) > (b) ? (a) : (b))

/*
 * transmit time for data payload + plcp_header
 */
unsigned calc_transmit_time(int rate, int length) {
  unsigned t_plcp_header = 96;
  if (rate == 1) {
    t_plcp_header = 192;
  } else if (!is_b_rate(rate)) {
    t_plcp_header = 20;
  }
  return (2 * (t_plcp_header + ((length * 8))))/ rate;
}

/*
 * expected backoff for t tries.
 */
unsigned calc_backoff(int rate, int t) 
{
  int t_slot = is_b_rate(rate) ? WIFI_SLOT_B : WIFI_SLOT_A;
  int cw = WIFI_CW_MIN;
  int x = 0;

  /* there is backoff, even for the first packet */
  for (x = 0; x < t; x++) {
    cw = MIN(WIFI_CW_MAX, (cw + 1) * 2);
  }
  return t_slot * cw / 2;
}



unsigned calc_usecs_wifi_packet_tries(int length, 
					      int rate, 
					      int try0, int tryN) {
  if (!rate || !length || try0 > tryN) {
    return 99999;
  }
  
  /* pg 205 ieee.802.11.pdf */
  unsigned t_slot = 20;
  unsigned t_ack = 304; // 192 + 14*8/1
  unsigned t_difs = 50; 
  unsigned t_sifs = 10; 


  if (!is_b_rate(rate)) {
    /* with 802.11g, things are at 6 mbit/s */
    t_slot = 9;
    t_sifs = 9;
    t_difs = 28;
    t_ack = 30; 
  }
  
  int tt = 0;
  int x = 0;
  for (x = try0; x <= tryN; x++) {
    tt += calc_backoff(rate, x) + 
          calc_transmit_time(rate, length) +
          t_sifs + t_ack;
  }
  return tt;
}

 unsigned calc_usecs_wifi_packet(int length, 
					      int rate, int retries) {
  return calc_usecs_wifi_packet_tries(length, rate,
				0, retries);
}

#define	ATH_NODE_SAMPLE(an)	((struct sample_node *)&an[1])
#endif /* _DEV_ATH_RATE_SAMPLE_H */
