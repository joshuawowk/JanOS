/* In-RAM captured-signal ring + SD-backed Flipper-compatible .sub storage.
 *
 * RAM idx: monotonically increasing 1-based handle (stable while the slot lives).
 * SD  idx: 1-based position in the sorted directory listing (as the Tab5 Manage
 *          screen expects -- it lists, then addresses save/tx/rename/delete by
 *          that list index).
 * SD files live in /sdcard/lab/subghz as <name>.sub in the Flipper SubGhz RAW format.
 */
#include "subghz_priv.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include "mbedtls/base64.h"

static subghz_signal_t s_slots[SUBGHZ_MAX_SLOTS];
static int s_next_idx = 1;

void subghz_store_init(void) { memset(s_slots, 0, sizeof(s_slots)); s_next_idx = 1; }
void subghz_store_clear_mem(void) { subghz_store_init(); }

subghz_signal_t *subghz_store_slot(int slot) {
    if (slot < 0 || slot >= SUBGHZ_MAX_SLOTS) return NULL;
    return &s_slots[slot];
}

int subghz_store_count(void) {
    int c = 0;
    for (int i = 0; i < SUBGHZ_MAX_SLOTS; i++) if (s_slots[i].idx > 0) c++;
    return c;
}

subghz_signal_t *subghz_store_get(int idx) {
    if (idx <= 0) return NULL;
    for (int i = 0; i < SUBGHZ_MAX_SLOTS; i++)
        if (s_slots[i].idx == idx) return &s_slots[i];
    return NULL;
}

int subghz_store_add(const subghz_signal_t *sig) {
    int free_slot = -1, oldest_slot = 0, oldest_idx = 0x7FFFFFFF;
    for (int i = 0; i < SUBGHZ_MAX_SLOTS; i++) {
        if (s_slots[i].idx == 0) { free_slot = i; break; }
        if (s_slots[i].idx < oldest_idx) { oldest_idx = s_slots[i].idx; oldest_slot = i; }
    }
    int slot = (free_slot >= 0) ? free_slot : oldest_slot;
    s_slots[slot] = *sig;
    s_slots[slot].idx = s_next_idx++;
    if (s_next_idx <= 0) s_next_idx = 1;
    return s_slots[slot].idx;
}

/* ---- SD helpers ---- */
static void ensure_dirs(void) {
    mkdir("/sdcard/lab", 0777);
    mkdir(SUBGHZ_SUB_DIR, 0777);
}

/* Sorted enumeration of *.sub base names. Returns count; fills names[] (each
 * up to 63 chars) up to max. */
static int list_sub_files(char names[][64], int max) {
    DIR *d = opendir(SUBGHZ_SUB_DIR);
    if (!d) return 0;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && n < max) {
        const char *nm = e->d_name;
        size_t len = strlen(nm);
        if (len > 4 && strcasecmp(nm + len - 4, ".sub") == 0) {
            strncpy(names[n], nm, 63); names[n][63] = 0; n++;
        }
    }
    closedir(d);
    /* simple insertion sort for a stable list index */
    for (int i = 1; i < n; i++) {
        char key[64]; strcpy(key, names[i]);
        int j = i - 1;
        while (j >= 0 && strcmp(names[j], key) > 0) { strcpy(names[j + 1], names[j]); j--; }
        strcpy(names[j + 1], key);
    }
    return n;
}

static void base_no_ext(const char *fname, char *out, size_t sz) {
    strncpy(out, fname, sz - 1); out[sz - 1] = 0;
    char *dot = strrchr(out, '.');
    if (dot) *dot = 0;
}

/* Write a signal as Flipper SubGhz RAW .sub. */
/* Stream an in-memory file to the host (Tab5), which writes it to its SD at
 * /sdcard/<relpath>. This board has no SD of its own, so all storage is routed
 * to the Tab5 -- mirrors the handshake [PCAPX] path with a generic [FILEX] frame:
 *   [FILEX name=<relpath> size=<n>]  [FILED]<base64>...  [FILEX-END sum=<hex>] */
static void subghz_stream_file(const char *relpath, const uint8_t *data, size_t len) {
    printf("[FILEX name=%s size=%u]\n", relpath, (unsigned)len);
    unsigned char b64[80];
    uint32_t sum = 0;
    size_t off = 0;
    while (off < len) {
        size_t chunk = (len - off > 48u) ? 48u : (len - off);
        size_t olen = 0;
        if (mbedtls_base64_encode(b64, sizeof(b64), &olen, data + off, chunk) == 0) {
            b64[olen] = '\0';
            printf("[FILED]%s\n", (char *)b64);
        }
        for (size_t i = 0; i < chunk; i++) sum += data[off + i];
        off += chunk;
    }
    printf("[FILEX-END sum=%08lX]\n", (unsigned long)sum);
    fflush(stdout);
}

int subghz_sd_save(const subghz_signal_t *sig, char *out_name, size_t out_sz) {
    char base[48];
    if (sig->name[0]) snprintf(base, sizeof(base), "%s", sig->name);
    else              snprintf(base, sizeof(base), "subghz_%04d", sig->idx);
    /* sanitise base (no spaces/slashes) */
    for (char *p = base; *p; p++) if (*p == ' ' || *p == '/' || *p == '\\') *p = '_';

    /* Build the Flipper .sub content in memory so it can be written to a local SD
     * OR streamed to the host. ~8 chars per timing (max " -65535") + header. */
    size_t cap = 256 + (size_t)sig->edges * 8u;
    char *buf = malloc(cap);
    if (!buf) return -1;
    int n = 0;
    long freq_hz = (long)(sig->freq * 1000000.0f + 0.5f);
    n += snprintf(buf + n, cap - n, "Filetype: Flipper SubGhz RAW File\n");
    n += snprintf(buf + n, cap - n, "Version: 1\n");
    n += snprintf(buf + n, cap - n, "Frequency: %ld\n", freq_hz);
    n += snprintf(buf + n, cap - n, "Preset: FuriHalSubGhzPresetOok650Async\n");
    n += snprintf(buf + n, cap - n, "Protocol: RAW\n");
    /* RAW_Data lines: +dur for HIGH, -dur for LOW, alternating from index 0. */
    int per_line = 0;
    for (int i = 0; i < sig->edges && n < (int)cap - 16; i++) {
        if (per_line == 0) n += snprintf(buf + n, cap - n, "RAW_Data:");
        int sign = (i % 2 == 0) ? 1 : -1;
        n += snprintf(buf + n, cap - n, " %d", sign * (int)sig->timings[i]);
        if (++per_line >= 40) { n += snprintf(buf + n, cap - n, "\n"); per_line = 0; }
    }
    if (per_line && n < (int)cap - 2) n += snprintf(buf + n, cap - n, "\n");
    if (n > (int)cap) n = (int)cap;

    /* Local SD first (a board that has one), else stream to the host (Tab5). */
    ensure_dirs();
    char path[160];
    snprintf(path, sizeof(path), "%s/%s.sub", SUBGHZ_SUB_DIR, base);
    FILE *f = fopen(path, "w");
    if (f) {
        fwrite(buf, 1, (size_t)n, f);
        fclose(f);
    } else {
        char relpath[80];
        snprintf(relpath, sizeof(relpath), "lab/subghz/%s.sub", base);
        subghz_stream_file(relpath, (const uint8_t *)buf, (size_t)n);
    }
    free(buf);
    if (out_name) snprintf(out_name, out_sz, "%s.sub", base);
    return 0;   /* saved locally or streamed to the host */
}

/* Parse a .sub file into a signal (RAW_Data -> timings, Frequency -> freq). */
static int parse_sub(const char *path, subghz_signal_t *out) {
    /* Slurp the whole file so arbitrarily long (multi-line) Flipper RAW_Data
     * records are tokenized without line-buffer truncation. */
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 131072) { fclose(f); return -1; }
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return -1; }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = 0;

    memset(out, 0, sizeof(*out));
    out->freq = SUBGHZ_DEFAULT_FREQ;
    char *fp = strstr(buf, "Frequency:");
    if (fp) { long hz = atol(fp + 10); if (hz > 0) out->freq = hz / 1000000.0f; }

    int edges = 0;
    char *rp = strstr(buf, "RAW_Data:");
    if (rp) {
        char *save = NULL;
        char *tok = strtok_r(rp, " \t\r\n", &save);
        while (tok && edges < SUBGHZ_MAX_EDGES) {
            if (strcmp(tok, "RAW_Data:") != 0) {
                long v = atol(tok);
                if (v != 0) {
                    long m = (v < 0) ? -v : v;
                    if (m > 0xFFFF) m = 0xFFFF;
                    out->timings[edges++] = (uint16_t)m;
                }
            }
            tok = strtok_r(NULL, " \t\r\n", &save);
        }
    }
    free(buf);
    out->edges = edges;
    snprintf(out->type, sizeof(out->type), "RAW");
    snprintf(out->proto, sizeof(out->proto), "RAW");
    return edges > 0 ? 0 : -1;
}

int subghz_sd_load(int idx, subghz_signal_t *out) {
    char names[64][64];
    int n = list_sub_files(names, 64);
    if (idx < 1 || idx > n) return -1;
    char path[160];
    snprintf(path, sizeof(path), "%s/%s", SUBGHZ_SUB_DIR, names[idx - 1]);
    if (parse_sub(path, out) != 0) return -1;
    out->idx = idx;
    base_no_ext(names[idx - 1], out->name, sizeof(out->name));
    out->from_sd = true;
    return 0;
}

int subghz_sd_delete(int idx, char *reason, size_t rsz) {
    char names[64][64];
    int n = list_sub_files(names, 64);
    if (idx < 1 || idx > n) { if (reason) snprintf(reason, rsz, "no_such_index"); return -1; }
    char path[160];
    snprintf(path, sizeof(path), "%s/%s", SUBGHZ_SUB_DIR, names[idx - 1]);
    if (remove(path) != 0) { if (reason) snprintf(reason, rsz, "unlink_failed"); return -1; }
    return 0;
}

int subghz_sd_rename(int idx, const char *newname, char *reason, size_t rsz) {
    char names[64][64];
    int n = list_sub_files(names, 64);
    if (idx < 1 || idx > n) { if (reason) snprintf(reason, rsz, "no_such_index"); return -1; }
    char base[48];
    snprintf(base, sizeof(base), "%s", newname);
    for (char *p = base; *p; p++) if (*p == ' ' || *p == '/' || *p == '\\') *p = '_';
    char oldp[160], newp[160];
    snprintf(oldp, sizeof(oldp), "%s/%s", SUBGHZ_SUB_DIR, names[idx - 1]);
    snprintf(newp, sizeof(newp), "%s/%s.sub", SUBGHZ_SUB_DIR, base);
    struct stat st;
    if (stat(newp, &st) == 0) { if (reason) snprintf(reason, rsz, "name_exists"); return -1; }
    if (rename(oldp, newp) != 0) { if (reason) snprintf(reason, rsz, "rename_failed"); return -1; }
    return 0;
}

void subghz_list_emit(const char *source) {
    bool sd = (source && strncmp(source, "sd", 2) == 0);
    int count = 0;
    if (sd) {
        char names[64][64];
        int n = list_sub_files(names, 64);
        for (int i = 0; i < n; i++) {
            subghz_signal_t sig;
            char path[160];
            snprintf(path, sizeof(path), "%s/%s", SUBGHZ_SUB_DIR, names[i]);
            char base[64]; base_no_ext(names[i], base, sizeof(base));
            float freq = SUBGHZ_DEFAULT_FREQ;
            if (parse_sub(path, &sig) == 0) freq = sig.freq;
            printf("[SUBGHZ_LIST] idx=%d type=RAW freq=%.2f serial=- btn=0 mf=RAW name=%s\n",
                   i + 1, freq, base);
            fflush(stdout);
            count++;
        }
        printf("[SUBGHZ_LIST_END] count=%d source=sd\n", count);
    } else {
        for (int i = 0; i < SUBGHZ_MAX_SLOTS; i++) {
            subghz_signal_t *s = &s_slots[i];
            if (s->idx <= 0) continue;
            printf("[SUBGHZ_LIST] idx=%d type=%s freq=%.2f serial=%s btn=%d mf=%s name=%s\n",
                   s->idx, s->type[0] ? s->type : "RAW", s->freq,
                   s->serial[0] ? s->serial : "-", s->btn,
                   s->mf[0] ? s->mf : "-", s->name[0] ? s->name : "-");
            fflush(stdout);
            count++;
        }
        printf("[SUBGHZ_LIST_END] count=%d source=mem\n", count);
    }
    fflush(stdout);
}
