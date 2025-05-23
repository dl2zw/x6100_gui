/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */

#include "info.h"

#include "cfg/transverter.h"
#include "styles.h"
#include "params/params.h"
#include "pubsub_ids.h"
#include "wifi.h"

typedef enum {
    INFO_VFO = 0,
    INFO_MODE,
    INFO_AGC,
    INFO_PRE,
    INFO_ATT,
    INFO_ATU,
    INFO_WIFI
} info_items_t;

static lv_obj_t     *obj;
static lv_obj_t     *items[7];

static Subject *mode_lock;

static void wifi_state_change_cb(void *s, lv_msg_t *m);

static void vfo_label_update(Subject *subj, void * user_data);
static void mode_label_update(Subject *subj, void * user_data);
static void atu_label_update(Subject *subj, void * user_data);
static void agc_label_update(Subject *subj, void * user_data);
static void att_pre_label_update(Subject *subj, void * user_data);

lv_obj_t * info_init(lv_obj_t * parent) {

    mode_lock = subject_create_int(false);

    obj = lv_obj_create(parent);

    lv_obj_add_style(obj, &info_style, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *row1 = lv_obj_create(obj);
    lv_obj_add_style(row1, &info_row_style, 0);
    lv_obj_set_size(row1, 190, 24);
    lv_obj_align(row1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(row1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);

    uint8_t i = 0;
    for (; i < 3; i++) {
        lv_obj_t *item = lv_label_create(row1);
        lv_obj_set_flex_grow(item, 1);
        items[i] = item;
    }

    lv_obj_t *row2 = lv_obj_create(obj);
    lv_obj_add_style(row2, &info_row_style, 0);
    lv_obj_set_size(row2, 190, 24);
    lv_obj_align(row2, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(row2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);

    for (; i < sizeof(items) / sizeof(*items); i++) {
        lv_obj_t *item = lv_label_create(row2);
        lv_obj_set_flex_grow(item, 3);
        items[i] = item;
    }

    lv_obj_set_flex_grow(items[INFO_WIFI], 2);

    for (i = 0; i < sizeof(items) / sizeof(*items); i++)
    {
        lv_obj_add_style(items[i], &info_item_style, 0);
        lv_obj_set_style_text_align(items[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(items[i], lv_color_white(), 0);
    }

    lv_label_set_text(items[INFO_PRE], "PRE");
    lv_label_set_text(items[INFO_ATT], "ATT");
    lv_label_set_text(items[INFO_WIFI], LV_SYMBOL_WIFI " ");
    lv_obj_set_style_text_color(items[INFO_WIFI], lv_color_hex(0x909090), 0);

    subject_add_delayed_observer(cfg_cur.band->vfo.val, vfo_label_update, NULL);
    subject_add_delayed_observer(cfg_cur.band->split.val, vfo_label_update, NULL);
    vfo_label_update(cfg_cur.band->vfo.val, NULL);

    subject_add_delayed_observer(cfg_cur.mode, mode_label_update, NULL);
    subject_add_delayed_observer(mode_lock, mode_label_update, NULL);
    mode_label_update(cfg_cur.band->vfo.val, NULL);

    subject_add_delayed_observer(cfg.ant_id.val, atu_label_update, NULL);
    subject_add_delayed_observer(cfg_cur.fg_freq, atu_label_update, NULL);
    subject_add_delayed_observer(cfg_cur.atu->loaded, atu_label_update, NULL);
    subject_add_delayed_observer(cfg.atu_enabled.val, atu_label_update, NULL);
    atu_label_update(cfg.atu_enabled.val, NULL);

    subject_add_delayed_observer(cfg_cur.agc, agc_label_update, NULL);
    agc_label_update(cfg_cur.agc, NULL);

    subject_add_delayed_observer_and_call(cfg_cur.att, att_pre_label_update, NULL);
    subject_add_delayed_observer_and_call(cfg_cur.pre, att_pre_label_update, NULL);

    lv_msg_subscribe(MSG_WIFI_STATE_CHANGED, wifi_state_change_cb, NULL);

    return obj;
}

const char* info_params_mode_label_get() {
    x6100_mode_t    mode = subject_get_int(cfg_cur.mode);
    char            *str;

    switch (mode) {
        case x6100_mode_lsb:
            str = "LSB";
            break;

        case x6100_mode_lsb_dig:
            str = "LSB-D";
            break;

        case x6100_mode_usb:
            str = "USB";
            break;

        case x6100_mode_usb_dig:
            str = "USB-D";
            break;

        case x6100_mode_cw:
            str = "CW";
            break;

        case x6100_mode_cwr:
            str = "CW-R";
            break;

        case x6100_mode_am:
            str = "AM";
            break;

        case x6100_mode_nfm:
            str = "NFM";
            break;

        default:
            str = "?";
            break;
    }

    return str;
}

const char* info_params_agc() {
    x6100_agc_t     agc = subject_get_int(cfg_cur.agc);
    char            *str;

    switch (agc) {
        case x6100_agc_off:
            str = "OFF";
            break;

        case x6100_agc_slow:
            str = "SLOW";
            break;

        case x6100_agc_fast:
            str = "FAST";
            break;

        case x6100_agc_auto:
            str = "AUTO";
            break;

        default:
            str = "?";
            break;

    }

    return str;
}

const char* info_params_vfo_label_get() {
    x6100_vfo_t cur_vfo = subject_get_int(cfg_cur.band->vfo.val);
    char            *str;

    if (subject_get_int(cfg_cur.band->split.val)) {
        str = cur_vfo == X6100_VFO_A ? "SPL-A" : "SPL-B";
    } else {
        str = cur_vfo == X6100_VFO_A ? "VFO-A" : "VFO-B";
    }

    return str;
}

void info_lock_mode(bool lock) {
    subject_set_int(mode_lock, lock);
}

static void wifi_state_change_cb(void *s, lv_msg_t *m) {
    lv_color_t color;
    switch (wifi_get_status()) {
    case WIFI_CONNECTED:
        color = lv_color_white();
        break;
    case WIFI_OFF:
        color = lv_color_black();
        break;
    default:
        color = lv_color_hex(0x909090);
        break;
    }
    lv_obj_set_style_text_color(items[INFO_WIFI], color, 0);
}


static void vfo_label_update(Subject *subj, void * user_data) {
    lv_label_set_text(items[INFO_VFO], info_params_vfo_label_get());
}

static void mode_label_update(Subject *subj, void *user_data) {
    lv_label_set_text(items[INFO_MODE], info_params_mode_label_get());
    x6100_mode_t mode = subject_get_int(cfg_cur.mode);
    if ((mode == x6100_mode_lsb_dig) || (mode == x6100_mode_usb_dig)) {
        lv_obj_set_style_text_color(items[INFO_MODE], lv_color_hex(COLOR_LIGHT_RED), 0);
    } else if (subject_get_int(mode_lock)) {
        lv_obj_set_style_text_color(items[INFO_MODE], lv_color_hex(0xAAAAAA), 0);
    } else {
        lv_obj_set_style_text_color(items[INFO_MODE], lv_color_white(), 0);
    }
}

static void atu_label_update(Subject *subj, void * user_data) {
    int32_t ant = subject_get_int(cfg.ant_id.val);
    lv_label_set_text_fmt(items[INFO_ATU], "ATU%i", ant);
    int32_t freq = subject_get_int(cfg_cur.fg_freq);

    if (!subject_get_int(cfg.atu_enabled.val)) {
        lv_obj_set_style_text_color(items[INFO_ATU], lv_color_white(), 0);
        lv_obj_set_style_bg_color(items[INFO_ATU], lv_color_black(), 0);
        lv_obj_set_style_bg_opa(items[INFO_ATU], LV_OPA_0, 0);
    } else {
        if (cfg_transverter_get_shift(freq)) {
            lv_obj_set_style_text_color(items[INFO_ATU], lv_color_hex(0xAAAAAA), 0);
            lv_obj_set_style_bg_opa(items[INFO_ATU], LV_OPA_20, 0);
        } else {
            lv_obj_set_style_text_color(items[INFO_ATU], subject_get_int(cfg_cur.atu->loaded) ? lv_color_black() : lv_color_hex(0xFF0000), 0);
            lv_obj_set_style_bg_opa(items[INFO_ATU], LV_OPA_50, 0);
        }
        lv_obj_set_style_bg_color(items[INFO_ATU], lv_color_white(), 0);
    }
}

static void agc_label_update(Subject *subj, void * user_data) {
    lv_label_set_text(items[INFO_AGC], info_params_agc());
}

static void att_pre_label_update(Subject *subj, void * user_data) {
    info_items_t item_id;
    if (subj == cfg_cur.att) {
        item_id = INFO_ATT;
    } else {
        item_id = INFO_PRE;
    }
    if (subject_get_int(subj)) {
        lv_obj_set_style_text_color(items[item_id], lv_color_black(), 0);
        lv_obj_set_style_bg_color(items[item_id], lv_color_white(), 0);
        lv_obj_set_style_bg_opa(items[item_id], LV_OPA_50, 0);
    } else {
        lv_obj_set_style_text_color(items[item_id], lv_color_white(), 0);
        lv_obj_set_style_bg_color(items[item_id], lv_color_black(), 0);
        lv_obj_set_style_bg_opa(items[item_id], LV_OPA_0, 0);
    }
}
