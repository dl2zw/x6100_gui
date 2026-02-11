/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */
#include <cstring>

#include "panel.h"
#include "knobs.h"
#include "util.h"

extern "C" {
    #include "rtty.h"
    #include "styles.h"
    #include "scheduler.h"
    #include "radio.h"
    #include "params/params.h"
}

static lv_obj_t           *obj;
static lv_anim_t           dim_anim;
static char                buf[1024];
static char                tmp_buf[1024];
static char               *last_line;

static void update_visibility(Subject *subj, void *user_data);
static void on_freq_change(Subject *subj, void *user_data);

static void set_opa(void * panel_obj, int32_t opa);

static void check_lines() {
    char        *second_line = NULL;
    char        *ptr = (char *) &buf;
    uint16_t    count = 0;

    while (*ptr) {
        if (*ptr == '\n') {
            count++;

            if (count == 1) {
                second_line = ptr + 1;
            }
        }
        ptr++;
    }

    if (count > 4) {
        strcpy(tmp_buf, second_line);
        strcpy(buf, tmp_buf);
    }

    ptr = (char *) &buf;

    while (*ptr) {
        if (*ptr == '\n') {
            last_line = ptr + 1;
        }
        ptr++;
    }

    *last_line = '\0';
}

static void panel_update_cb(const char *text) {
    lv_point_t line_size;
    lv_point_t text_size;

    if (!last_line) {
        return;
    }

    if (strcmp(text, "\n") == 0) {
        if (last_line[strlen(last_line) - 1] != '\n') {
            strcat(last_line, text);
            check_lines();
        }
    } else {
        // lv_txt_get_size(&line_size, last_line, &sony_38, 0, 0, LV_COORD_MAX, 0);
        // lv_txt_get_size(&text_size, text, &sony_38, 0, 0, LV_COORD_MAX, 0);

        if (line_size.x + text_size.x > (lv_obj_get_width(obj) - 40)) {
            strcat(last_line, "\n");
            check_lines();
        }

        strcat(last_line, text);
    }

    lv_label_set_text_static(obj, buf);
}

lv_obj_t * panel_init(lv_obj_t *parent) {
    obj = lv_label_create(parent);

    lv_obj_add_style(obj, &panel_style, 0);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);

    lv_anim_init(&dim_anim);
    lv_anim_set_exec_cb(&dim_anim, set_opa);
    lv_anim_set_var(&dim_anim, obj);
    lv_anim_set_time(&dim_anim, 200);

    subject_add_delayed_observer(cfg_cur.mode, update_visibility, NULL);
    subject_add_delayed_observer_and_call(cfg.cw_decoder.val, update_visibility, NULL);
    subject_add_delayed_observer(cfg_cur.fg_freq, on_freq_change, NULL);
    return obj;
}

void panel_add_text(const char * text) {
    scheduler_put((void(*)(void*))panel_update_cb, (void*)text, strlen(text) + 1);
}

void panel_hide() {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    knobs_display(true);
}

void panel_visible() {
    x6100_mode_t    mode = (x6100_mode_t)subject_get_int(cfg_cur.mode);
    bool            on = false;

    switch (mode) {
        case x6100_mode_cw:
        case x6100_mode_cwr:
            on = subject_get_int(cfg.cw_decoder.val);
            break;

        case x6100_mode_usb:
        case x6100_mode_lsb:
        case x6100_mode_usb_dig:
        case x6100_mode_lsb_dig:
            on = rtty_get_state() != RTTY_OFF;
            break;
    }

    if (on) {
        if (lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN)) {
            strcpy(buf, "");
            last_line = (char *) &buf;
            lv_label_set_text_static(obj, buf);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
            knobs_display(false);
        }
    } else {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        knobs_display(true);
    }
}

static void update_visibility(Subject *subj, void *user_data) {
    panel_visible();
}

static void set_opa(void * panel_obj, int32_t opa) {
    lv_obj_set_style_opa((lv_obj_t *)panel_obj, opa, 0);
}

static void revert_opa(struct _lv_anim_t * acct) {
    lv_anim_set_delay(&dim_anim, 200);
    lv_anim_set_values(&dim_anim, LV_OPA_40, LV_OPA_COVER);
    lv_anim_set_ready_cb(&dim_anim, NULL);
    lv_anim_start(&dim_anim);
}

static void on_freq_change(Subject *subj, void *user_data) {
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }
    lv_anim_del(obj, set_opa);

    lv_opa_t cur_opa = lv_obj_get_style_opa(obj, 0);
    lv_anim_set_ready_cb(&dim_anim, revert_opa);
    lv_anim_set_values(&dim_anim, cur_opa, LV_OPA_40);
    lv_anim_set_delay(&dim_anim, 0);
    lv_anim_start(&dim_anim);









    // if (anim_tl) {
    //     lv_anim_timeline_del(anim_tl);
    // }

    // lv_anim_timeline_stop(anim_tl);
    // lv_ani
    // lv_anim_set_values(&dim_anim, cur_opa, LV_OPA_40);
    // lv_anim_set_values(&undim_anim, LV_OPA_40, LV_OPA_COVER);

}
