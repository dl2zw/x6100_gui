#pragma once

#include "subjects.h"

#include <pthread.h>
#include <sqlite3.h>

typedef struct cfg_item_t {
    const char     *db_name;
    int             pk;
    subject_t       val;
    struct dirty_t *dirty;
    int (*load)(struct cfg_item_t *item);
    int (*save)(struct cfg_item_t *item);
} cfg_item_t;

/* configuration structs. Should contain same types (for correct initialization) */
typedef struct {
    cfg_item_t key_tone;
    cfg_item_t vol;
    cfg_item_t band_id;
} cfg_t;

struct vfo_params {
    cfg_item_t freq;
    cfg_item_t mode;
};

typedef struct {
    struct vfo_params vfo_a;
    struct vfo_params vfo_b;
    cfg_item_t        vfo;
} cfg_band_t;

extern cfg_t      cfg;
extern cfg_band_t cfg_band;

/* Current params */

typedef struct {
    subject_t   freq;
    subject_t   mode;
} cfg_cur_t;

extern cfg_cur_t cfg_cur;

int cfg_init(sqlite3 *db);
