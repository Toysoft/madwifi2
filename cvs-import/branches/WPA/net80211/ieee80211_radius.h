/*-
 * Copyright (c) 2004 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _NET80211_RADIUS_H_
#define _NET80211_RADIUS_H_

/*
 * IEEE 802.1x radius client support.
 *
 * Radius client support for the 802.1x authenticator.
 * This could be a user process that communicates with
 * the authenticator through messages; but for now its
 * in the kernel where it's easy to share state with the
 * authenticator state machine.  We structure this as a
 * module so it is not always resident.  Also we instantiate
 * this support only as needed so the cost of having it
 * is only incurred when in use (e.g. not for stations).
 */

struct vendor_key {
	u_int32_t	vk_vid;		/* vendor id */
	u_int8_t	vk_type;	/* key type */
	u_int8_t	vk_len;		/* key length */
	u_int16_t	vk_salt;
	/* variable length data follows */
};

struct eapol_auth_radius_node {
	struct eapol_auth_node	ern_base;
	LIST_ENTRY(eapol_auth_radius_node) ern_next;	/* reply list */
	struct sk_buff		*ern_msg;	/* supplicant reply */
	u_int8_t		*ern_radmsg;	/* radius server message */
	u_int			ern_radmsglen;	/* length of server message */
	u_int8_t		ern_msgauth[16];
	u_int8_t		ern_reqauth[16];/* request authenticator */
	u_int8_t		ern_state[255];
	u_int8_t		ern_statelen;
	u_int8_t		ern_class[255];
	u_int8_t		ern_classlen;
	union {
		struct vendor_key vk;
		u_int8_t	data[64];
	} ern_txu;
#define	ern_txkey	ern_txu.vk
	union {
		struct vendor_key vk;
		u_int8_t	data[64];
	} ern_rxu;
#define	ern_rxkey	ern_rxu.vk
};
#define	EAPOL_RADIUSNODE(_x)	((struct eapol_auth_radius_node *)(_x))

#define	RAD_MAXMSG	4096			/* max message size */

struct crypto_tfm;

struct radiuscom {
	struct eapolcom	*rc_ec;			/* back reference */
	struct socket	*rc_sock;		/* open socket */
	pid_t		rc_pid;			/* pid of client thread */
	struct sockaddr_in rc_server;		/* server's address */
	struct sockaddr_in rc_local;		/* local address */
	u_int8_t	*rc_secret;		/* shared secret */
	u_int		rc_secretlen;		/* length of shared secret */
	struct crypto_tfm *rc_md5;		/* crypto handle */
	u_int8_t	rc_buf[RAD_MAXMSG];	/* recv thread msg buffer */
	ATH_LIST_HEAD(, eapol_auth_radius_node) rc_replies;
	/* saved copies of eapolcom methods */
	struct eapol_auth_node *(*rc_node_alloc)(struct eapolcom *);
	void		(*rc_node_free)(struct eapol_auth_node *);
	void		(*rc_node_reset)(struct eapol_auth_node *);
	/* callbacks used by the main .1x code */
	void		(*rc_identity_input)(struct eapol_auth_node *,
				struct sk_buff *);
	void		(*rc_txreq)(struct eapol_auth_node *);
	void		(*rc_sendsrvr)(struct eapol_auth_node *);
	void		(*rc_txkey)(struct eapol_auth_node *);
};

#if defined(__KERNEL__) || defined(_KERNEL)
extern	int ieee80211_radius_attach(struct eapolcom *);
extern	void ieee80211_radius_detach(struct eapolcom *);
#endif
#endif /* _NET80211_RADIUS_H_ */
