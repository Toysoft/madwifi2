/*
 * This software is distributed under the terms of the
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
 * $Id: if_ath_radar.h 2464 2007-06-15 22:51:56Z mtaylor $
 */
/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _IF_ATH_RADAR_H
#define _IF_ATH_RADAR_H

/* AR5K_AR5212_PHY_ERR_FIL register definition taken from openhal */
#define AR5K_AR5212_PHY_ERR_FIL		    0x810c
#define AR5K_AR5212_PHY_ERR_FIL_RADAR	0x00000020

/* AR5K_PHY_RADAR register definition reverse engineered with 
 * ATH_REVERSE_ENGINEERING. */

/* PHY radar detection register [5111+] */
#define	AR5K_PHY_RADAR			0x9954

/* Radar enable 			........ ........ ........ .......1 */
#define	AR5K_PHY_RADAR_ENABLE		0x00000001
#define	AR5K_PHY_RADAR_ENABLE_S		0

/* This is the value found on the card  .1.111.1 .1.1.... 111....1 1...1...
at power on. */
#define	AR5K_PHY_RADAR_PWONDEF_AR5213	0x5d50e188 

/* This is the value found on the card 	.1.1.111 ..11...1 .1...1.1 1...11.1
after DFS is enabled */
#define	AR5K_PHY_RADAR_ENABLED_AR5213	0x5731458d

/* Finite Impulse Response (FIR) filter .1111111 ........ ........ ........ 
 * power out threshold.
 * 7-bits, standard power range {0..127} in 1/2 dBm units. */
#define AR5K_PHY_RADAR_FIRPWROUTTHR    	0x7f000000 
#define AR5K_PHY_RADAR_FIRPWROUTTHR_S	24

/* Radar RSSI/SNR threshold.		........ 111111.. ........ ........ 
 * 6-bits, dBm range {0..63} in dBm units. */
#define AR5K_PHY_RADAR_RADARRSSITHR    	0x00fc0000 
#define AR5K_PHY_RADAR_RADARRSSITHR_S	18

/* Pulse height threshold 		........ ......11 1111.... ........ 
 * 6-bits, dBm range {0..63} in dBm units. */
#define AR5K_PHY_RADAR_PULSEHEIGHTTHR   0x0003f000
#define AR5K_PHY_RADAR_PULSEHEIGHTTHR_S	12

/* Pulse RSSI/SNR threshold		........ ........ ....1111 11...... 
 * 6-bits, dBm range {0..63} in dBm units. */
#define AR5K_PHY_RADAR_PULSERSSITHR    	0x00000fc0
#define AR5K_PHY_RADAR_PULSERSSITHR_S	6

/* Inband threshold  			........ ........ ........ ..11111. 
 * 5-bits, units unknown {0..31} (? MHz ?) */
#define AR5K_PHY_RADAR_INBANDTHR    	0x0000003e
#define AR5K_PHY_RADAR_INBANDTHR_S	1

/* This struct defines the supported PHY error detection parameters for radar
pulse detection logic */
typedef struct {
	/* Finite Impulse Response (FIR) filter - power out threshold.
	 * 7-bits, standard power range {0..127} in 1/2 dBm units. */
	int32_t rp_fir_filter_output_power_thr; 		

	/* Radar RSSI/SNR threshold.
	 * 6-bits, dBm range {0..63} in dBm units. */
	int32_t rp_radar_rssi_thr;	

	/* Pulse height threshold
	 * 6-bits, dBm range {0..63} in dBm units. */
	int32_t rp_pulse_height_thr;

	/* Pulse RSSI/SNR threshold
	 * 6-bits, dBm range {0..63} in dBm units. */
	int32_t rp_pulse_rssi_thr;

 	/* Inband threshold.  
	 * 5-bits, units unknown {0..31} (? MHz ?) */
	int32_t rp_inband_thr;

} RADAR_PARAM;

/* Any value in RADAR_PARAM can be set to this magic value in order to use
the default for that field */
#define RADAR_PARAM_USE_DEFAULT 0xffff

/* This is called on channel change to enable radar detection for 5211+ chips.  
 * NOTE: AR5210 doesn't have radar pulse detection support. */
int ath_radar_update(struct ath_softc *sc);
/* Returns true if radar detection is enabled. */
int ath_radar_is_enabled(struct ath_softc *sc);
/* Read the radar pulse detection parameters. */
void ath_radar_get_params(struct ath_softc *sc, RADAR_PARAM* rp);
/* Update the radar pulse detection parameters. 
 * If rp is NULL, defaults are used for all fields.
 * If any member of rp is set to RADAR_PARAM_USE_DEFAULT, the default
 * is used for that field. */
void ath_radar_set_params(struct ath_softc *sc, RADAR_PARAM* rp);
/* Update channel's DFS flags based upon whether DFS is reqired */
int ath_radar_correct_dfs_flags(struct ath_softc *sc, HAL_CHANNEL *hchan);
/* Returns true if DFS is required for the regulatory domain, country and 
 * combination in use. */
int ath_radar_is_dfs_required(struct ath_softc *sc, HAL_CHANNEL *hchan);

#endif /* #ifndef _IF_ATH_RADAR_H */
