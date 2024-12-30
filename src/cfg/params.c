/**
 * Work with params table on DB
 */
#include "params.private.h"

#include "subjects.private.h"

#include "../lvgl/lvgl.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static sqlite3      *db;
static sqlite3_stmt *insert_stmt;
static sqlite3_stmt *read_stmt;
static pthread_mutex_t write_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t read_mutex = PTHREAD_MUTEX_INITIALIZER;


void cfg_params_init(sqlite3 *database) {
    db = database;
    int rc;
    rc = sqlite3_prepare_v2(db, "SELECT val FROM params WHERE name = :name", -1, &read_stmt, 0);
    if (rc != SQLITE_OK) {
        LV_LOG_ERROR("Failed prepare read statement: %s", sqlite3_errmsg(db));
        exit(1);
    }
    rc = sqlite3_prepare_v2(db, "INSERT OR REPLACE INTO params(name, val) VALUES(:name, :val)", -1, &insert_stmt, 0);
    if (rc != SQLITE_OK) {
        LV_LOG_ERROR("Failed prepare write statement: %s", sqlite3_errmsg(db));
        exit(1);
    }
}


int cfg_params_load_item(cfg_item_t *item) {
    int rc;
    pthread_mutex_lock(&read_mutex);
    rc = sqlite3_bind_text(read_stmt, sqlite3_bind_parameter_index(read_stmt, ":name"), item->db_name, strlen(item->db_name), 0);
    if (rc != SQLITE_OK) {
        LV_LOG_ERROR("Failed to bind name %s: %s", item->db_name, sqlite3_errmsg(db));
        pthread_mutex_unlock(&read_mutex);
        return rc;
    }
    int32_t int_val;
    uint64_t uint64_val;
    rc = sqlite3_step(read_stmt);
    if (rc == SQLITE_ROW) {
        switch (item->val->dtype) {
            case DTYPE_INT:
                int_val = sqlite3_column_int(read_stmt, 0);
                LV_LOG_USER("Loaded %s=%i (pk=%i)", item->db_name, int_val, item->pk);
                subject_set_int(item->val, int_val);
                break;
            case DTYPE_UINT64:
                uint64_val = sqlite3_column_int64(read_stmt, 0);
                LV_LOG_USER("Loaded %s=%llu (pk=%i)", item->db_name, uint64_val, item->pk);
                subject_set_uint64(item->val, uint64_val);
                break;
            default:
                LV_LOG_WARN("Unknown item %s dtype: %u, can't load", item->db_name, item->val->dtype);
                pthread_mutex_unlock(&read_mutex);
                return -1;
        }
        rc = 0;
    } else {
        LV_LOG_WARN("No results for load %s", item->db_name);
        rc = -1;
    }
    sqlite3_reset(read_stmt);
    sqlite3_clear_bindings(read_stmt);
    pthread_mutex_unlock(&read_mutex);
    return rc;
}


int cfg_params_save_item(cfg_item_t *item) {
    int rc;
    pthread_mutex_lock(&write_mutex);
    rc = sqlite3_bind_text(insert_stmt, sqlite3_bind_parameter_index(insert_stmt, ":name"), item->db_name, strlen(item->db_name), 0);
    if (rc != SQLITE_OK) {
        LV_LOG_WARN("Can't bind name %s to save params query", item->db_name);
        pthread_mutex_unlock(&write_mutex);
        return rc;
    }
    int val_index = sqlite3_bind_parameter_index(insert_stmt, ":val");
    switch (item->val->dtype) {
        case DTYPE_INT:
            rc = sqlite3_bind_int(insert_stmt, val_index, item->val->int_val);
            if (rc != SQLITE_OK) {
                LV_LOG_WARN("Can't bind val %i to save params query", item->val->int_val);
            }
            break;
        case DTYPE_UINT64:
            rc = sqlite3_bind_int64(insert_stmt, val_index, item->val->uint64_val);
            if (rc != SQLITE_OK) {
                LV_LOG_WARN("Can't bind val %llu to save params query", item->val->uint64_val);
            }
            break;
        default:
            LV_LOG_WARN("Unknown item %s dtype: %u, will not save", item->db_name, item->val->dtype);
            sqlite3_reset(insert_stmt);
            sqlite3_clear_bindings(insert_stmt);
            pthread_mutex_unlock(&write_mutex);
            return -1;
            break;
    }
    if (rc == SQLITE_OK) {
        rc = sqlite3_step(insert_stmt);
        if (rc != SQLITE_DONE) {
            LV_LOG_ERROR("Failed save item %s: %s", item->db_name, sqlite3_errmsg(db));
        } else {
            if (item->val->dtype == DTYPE_INT) {
                LV_LOG_USER("Saved %s=%i (pk=%i)", item->db_name, item->val->int_val, item->pk);
            } else {
                LV_LOG_USER("Saved %s=%llu (pk=%i)", item->db_name, item->val->uint64_val, item->pk);
            }
            rc = 0;
        }
    }
    sqlite3_reset(insert_stmt);
    sqlite3_clear_bindings(insert_stmt);
    pthread_mutex_unlock(&write_mutex);
    return rc;
}
