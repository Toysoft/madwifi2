/*From _ieee80211.h*/
enum ieee80211_opmode {
        IEEE80211_M_STA         = 1,    /* infrastructure station */
        IEEE80211_M_IBSS        = 0,    /* IBSS (adhoc) station */
        IEEE80211_M_AHDEMO      = 3,    /* Old lucent compatible adhoc demo */
        IEEE80211_M_HOSTAP      = 6,    /* Software Access Point */
        IEEE80211_M_MONITOR     = 8,    /* Monitor mode */
        IEEE80211_M_WDS         = 2     /* WDS link */
};

/*
 * Channels are specified by frequency and attributes.
 */
struct net80211_channel {
        u_int16_t ic_freq;      /* setting in Mhz */
        u_int16_t ic_flags;     /* see below */
        u_int8_t ic_ieee;       /* IEEE channel number */
        int8_t ic_maxregpower;  /* maximum regulatory tx power in dBm */
        int8_t ic_maxpower;     /* maximum tx power in dBm */
        int8_t ic_minpower;     /* minimum tx power in dBm */
};


/*From ieee80211.h*/
#define IEEE80211_WEP_KEYLEN            5       /* 40bit */
#define IEEE80211_WEP_IVLEN             3       /* 24bit */
#define IEEE80211_WEP_KIDLEN            1       /* 1 octet */
#define IEEE80211_WEP_CRCLEN            4       /* CRC-32 */
#define IEEE80211_WEP_NKID              4       /* number of key ids */
#define IEEE80211_CRC_LEN               4
#define IEEE80211_MAX_LEN               (2300 + IEEE80211_CRC_LEN + \
    (IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN))

/*From ieee80211_proto.h*/
enum ieee80211_state {
        IEEE80211_S_INIT        = 0,    /* default state */
        IEEE80211_S_SCAN        = 1,    /* scanning */
        IEEE80211_S_AUTH        = 2,    /* try to authenticate */
        IEEE80211_S_ASSOC       = 3,    /* try to assoc */
        IEEE80211_S_RUN         = 4,    /* associated */
};
#define IEEE80211_S_MAX         (IEEE80211_S_RUN+1)

