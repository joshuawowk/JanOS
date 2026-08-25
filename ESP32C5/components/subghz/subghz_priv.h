/* Private shared definitions for the subghz component. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "cc1101.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SUBGHZ_MAX_EDGES     512    /* max timing transitions captured per signal */
#define SUBGHZ_MAX_SLOTS     24     /* in-RAM captured-signal ring */
#define SUBGHZ_SUB_DIR       "/sdcard/lab/subghz"
#define SUBGHZ_DEFAULT_FREQ  433.92f

/* A captured / stored sub-GHz signal. timings[] holds alternating pulse
 * durations in microseconds (first entry is a HIGH pulse). */
typedef struct {
    int      idx;            /* 1-based handle; 0 = empty slot */
    char     type[32];       /* decoder family, or "RAW" */
    float    freq;           /* MHz */
    int      bits;
    char     serial[32];
    int      btn;
    char     proto[32];
    char     learn[24];
    char     mf[32];
    int      cnt;            /* repeat count (RX_DUP increments) */
    int      te;             /* base timing element (us) */
    int      edges;          /* number of valid entries in timings[] */
    char     name[40];
    uint16_t timings[SUBGHZ_MAX_EDGES];
    bool     decoded;        /* a protocol decoder recognised it */
    bool     from_sd;        /* loaded from an SD .sub file */
} subghz_signal_t;

/* ---- Signal store (subghz_store.c) ---- */
void              subghz_store_init(void);
void              subghz_store_clear_mem(void);
/* Allocate/overwrite a RAM slot, returns assigned idx (>0) or 0 on failure. */
int               subghz_store_add(const subghz_signal_t *sig);
subghz_signal_t  *subghz_store_get(int idx);          /* RAM slot by idx or NULL */
int               subghz_store_count(void);
subghz_signal_t  *subghz_store_slot(int slot);        /* 0..SUBGHZ_MAX_SLOTS-1 */

/* SD-backed operations. Return 0 on success, negative on error.
 * On success *out_name receives the file/base name (no path). */
int  subghz_sd_save(const subghz_signal_t *sig, char *out_name, size_t out_sz);
int  subghz_sd_delete(int idx, char *out_reason, size_t rsz);
int  subghz_sd_rename(int idx, const char *newname, char *out_reason, size_t rsz);
/* Emit [SUBGHZ_LIST] rows for source ("sd" or "mem") then [SUBGHZ_LIST_END]. */
void subghz_list_emit(const char *source);
/* Load an SD signal by list-index into *out. Returns 0 on success. */
int  subghz_sd_load(int idx, subghz_signal_t *out);

/* ---- Capture / replay (subghz_capture.c) ---- */
/* Start continuous RMT RX on GDO0. Returns 0 on success. */
int  subghz_cap_rx_start(int gdo0_pin);
void subghz_cap_rx_stop(void);
/* Wait up to timeout_ms for one captured burst -> timings[] (us). Returns edge
 * count (>0) or 0 on timeout. Filters bursts shorter than min_edges. */
int  subghz_cap_rx_get(uint16_t *timings, int max_edges, int min_edges, uint32_t timeout_ms);
/* Count GDO0 edges within a dwell window (for scanner/analyzer). */
unsigned subghz_cap_edge_count(int gdo0_pin, uint32_t dwell_ms);
/* Replay a timing array via RMT TX on GDO0. Returns 0 on success. */
int  subghz_cap_tx_replay(int gdo0_pin, const uint16_t *timings, int edges, int repeats);

/* ---- Decoders (subghz_decode.c) ---- */
/* Try to decode an OOK timing burst. Fills type/proto/serial/btn/bits/te/mf/
 * learn on success. Returns true if recognised. */
bool subghz_decode_ook(const uint16_t *timings, int edges, subghz_signal_t *out);
/* Try to decode a weather-sensor burst. On success fills proto/id/ch/temp/hum/
 * batt strings (caller formats the [SUBGHZ_WEATHER] line). Returns true. */
bool subghz_decode_weather(const uint16_t *timings, int edges,
                           char *proto, size_t proto_sz,
                           unsigned long *id, char *ch, size_t ch_sz,
                           char *temp, size_t temp_sz,
                           char *hum, size_t hum_sz,
                           char *batt, size_t batt_sz);

/* ---- Shared radio state (subghz.c) ---- */
extern cc1101_t g_subghz_radio;
extern bool     g_subghz_radio_ok;
extern float    g_subghz_freq;        /* base carrier (MHz) */
extern float    g_subghz_correction;  /* freq correction (MHz) */
/* Lazy init + presence print. Returns true if radio present.
 * Prints "CC1101 initialized" / "CC1101 NOT DETECTED" exactly once per state. */
bool subghz_ensure_radio(void);
float subghz_effective_freq(void);    /* base + correction */

#ifdef __cplusplus
}
#endif
