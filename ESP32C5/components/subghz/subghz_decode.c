/* OOK protocol + weather-sensor decoders operating on captured timing bursts.
 *
 * timings[] holds alternating pulse durations in microseconds, starting with a
 * HIGH (carrier-on) pulse: timings[0]=high0, timings[1]=low0, timings[2]=high1...
 *
 * Coverage note: the fixed-code decoder recognises the rc-switch "Princeton"
 * family (PT2262 / EV1527 / SC2260 clones -- the vast majority of garage, gate
 * and doorbell remotes). Anything else that still looks like a framed remote is
 * reported as type "Unknown" with a content hash as the serial, and its raw
 * timing is preserved so replay/save still work (Flipper "RAW" equivalent).
 * Rolling-code remotes (KeeLoq) cannot be cloned and are reported, not decoded.
 * Weather: the Nexus TH family (common cheap thermo-hygrometers) is decoded;
 * the framework is structured to add more. All timing thresholds are marked for
 * a bench-calibration pass on real hardware.
 */
#include "subghz_priv.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static inline bool near_ratio(int v, int unit, int mult, int tol_pct) {
    int target = unit * mult;
    int tol = target * tol_pct / 100;
    if (tol < unit / 2) tol = unit / 2;
    return v >= target - tol && v <= target + tol;
}

/* Estimate the base timing element (te) as the smallest well-represented pulse. */
static int estimate_te(const uint16_t *t, int n) {
    int mn = 100000;
    for (int i = 0; i < n; i++) {
        if (t[i] >= 80 && t[i] < mn) mn = t[i];   /* ignore <80us glitches */
    }
    if (mn == 100000) return 0;
    /* Average all pulses within 1.6x of the minimum -> stable te. */
    long sum = 0; int cnt = 0;
    for (int i = 0; i < n; i++) {
        if (t[i] >= mn - mn / 4 && t[i] <= mn + mn * 3 / 5) { sum += t[i]; cnt++; }
    }
    return cnt ? (int)(sum / cnt) : mn;
}

/* Try rc-switch "Princeton" (P1): bit0={te hi,3te lo}, bit1={3te hi,te lo},
 * 24 data bits, sync separator = very long low (~31 te). */
static bool decode_princeton(const uint16_t *t, int n, int te, subghz_signal_t *out) {
    if (te < 150 || te > 1200) return false;   /* typical 250-500us */
    /* Find a run of 24 clean bit-pairs (48 entries) starting on a HIGH. */
    for (int start = 0; start + 48 <= n; start += 2) {
        uint32_t val = 0; int bits = 0; bool ok = true;
        for (int b = 0; b < 24; b++) {
            int hi = t[start + b * 2];
            int lo = t[start + b * 2 + 1];
            int bit;
            if (near_ratio(hi, te, 1, 60) && near_ratio(lo, te, 3, 45))      bit = 0;
            else if (near_ratio(hi, te, 3, 45) && near_ratio(lo, te, 1, 60)) bit = 1;
            else { ok = false; break; }
            val = (val << 1) | bit;
            bits++;
        }
        if (ok && bits == 24) {
            snprintf(out->type,   sizeof(out->type),   "Princeton");
            snprintf(out->proto,  sizeof(out->proto),  "Princeton");
            snprintf(out->learn,  sizeof(out->learn),  "static");
            snprintf(out->mf,     sizeof(out->mf),     "PT2262");
            snprintf(out->serial, sizeof(out->serial), "0x%05lX", (unsigned long)(val >> 4));
            out->btn  = (int)(val & 0x0F);
            out->bits = 24;
            out->te   = te;
            out->decoded = true;
            return true;
        }
    }
    return false;
}

/* FNV-1a over the quantised bit pattern -> stable pseudo-serial for unknowns. */
static uint32_t hash_pattern(const uint16_t *t, int n, int te) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) {
        uint8_t sym = (te > 0 && t[i] > te * 2) ? 1 : 0;
        h = (h ^ sym) * 16777619u;
    }
    return h;
}

bool subghz_decode_ook(const uint16_t *timings, int edges, subghz_signal_t *out) {
    if (edges < 16) return false;
    int te = estimate_te(timings, edges);
    if (te <= 0) return false;
    if (decode_princeton(timings, edges, te, out)) return true;

    /* Generic framed-but-unknown OOK: still a usable capture (replay works). */
    int bits = 0;
    for (int i = 0; i < edges; i += 2) bits++;
    snprintf(out->type,   sizeof(out->type),   "Unknown");
    snprintf(out->proto,  sizeof(out->proto),  "RAW");
    snprintf(out->learn,  sizeof(out->learn),  "unknown");
    snprintf(out->mf,     sizeof(out->mf),     "%de", edges);
    snprintf(out->serial, sizeof(out->serial), "0x%08lX",
             (unsigned long)hash_pattern(timings, edges, te));
    out->btn  = 0;
    out->bits = bits;
    out->te   = te;
    out->decoded = false;
    return true;   /* accept so the user sees + can replay the capture */
}

/* ---- Weather: Nexus TH (PWM, ~500us pulse; 0=~1000us gap, 1=~2000us gap;
 * 36 bits: id[8] batt[1] ch[2..] temp[12 signed /10] const[4=0xF] hum[8]). ---- */
bool subghz_decode_weather(const uint16_t *timings, int edges,
                           char *proto, size_t proto_sz,
                           unsigned long *id, char *ch, size_t ch_sz,
                           char *temp, size_t temp_sz,
                           char *hum, size_t hum_sz,
                           char *batt, size_t batt_sz) {
    if (edges < 2 * 32) return false;
    /* Nexus gaps are the LOW pulses (odd indices). Align after the long sync
     * gap (~4ms). Find the last sync gap, then read 36 bits after it. */
    int best = -1;
    for (int i = 1; i < edges; i += 2) {
        if (timings[i] > 3000 && timings[i] < 6000) best = i; /* sync ~4ms */
    }
    if (best < 0) return false;
    int start = best + 1;               /* first HIGH pulse after sync */
    uint64_t v = 0; int bits = 0;
    for (int b = 0; b < 36; b++) {
        int gi = start + b * 2 + 1;      /* the gap (low) that encodes the bit */
        if (gi >= edges) break;
        int gap = timings[gi];
        int bit;
        if (gap > 700 && gap < 1500) bit = 0;
        else if (gap >= 1500 && gap < 2600) bit = 1;
        else break;
        v = (v << 1) | bit;
        bits++;
    }
    if (bits < 36) return false;
    unsigned long sid = (unsigned long)((v >> 28) & 0xFF);
    int battery = (int)((v >> 27) & 0x1);
    int channel = (int)((v >> 24) & 0x07);
    int t12     = (int)((v >> 12) & 0xFFF);
    int constant= (int)((v >> 8)  & 0x0F);
    int humidity= (int)(v & 0xFF);
    if (constant != 0x0F) return false;             /* validation gate */
    if (humidity > 100) return false;
    if (t12 & 0x800) t12 -= 0x1000;                 /* sign-extend 12-bit */
    float tc = t12 / 10.0f;
    snprintf(proto, proto_sz, "Nexus-TH");
    *id = sid;
    snprintf(ch,   ch_sz,   "%d", channel);
    snprintf(temp, temp_sz, "%.1f", tc);
    snprintf(hum,  hum_sz,  "%d", humidity);
    snprintf(batt, batt_sz, "%s", battery ? "ok" : "low");
    return true;
}
