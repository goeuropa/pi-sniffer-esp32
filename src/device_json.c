/**
 * Device Report JSON Payload Builder
 *
 * See device_json.h. Moved out of http_client.c so WiFi-MQTT and
 * cellular-MQTT reporting (mqtt_report.c, cellular.c) can build the exact
 * same payload the REST path used to, without depending on http_client.c.
 */

#include "device_json.h"
#include "config.h"

#include "cJSON.h"

#include <math.h>
#include <time.h>

// "timestamp" is always an ISO 8601 UTC string ("YYYY-MM-DDTHH:MM:SSZ"),
// never a number - Ian's explicit direction. gmtime_r() (not localtime_r())
// since both the GPS fix_time source (device_json_build_minimal()) and
// time(NULL) are already UTC; converting to local time here would make the
// two inconsistent depending on which one a given payload used.
#define ISO8601_BUF_SIZE 21

static void format_iso8601_utc(time_t t, char *buf, size_t buf_size) {
    struct tm tm_utc;
    gmtime_r(&t, &tm_utc);
    strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
}

char *device_json_build(const device_list_t *list, const char *device_id) {
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    // Add device ID and timestamp
    char ts_buf[ISO8601_BUF_SIZE];
    format_iso8601_utc(time(NULL), ts_buf, sizeof(ts_buf));
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "timestamp", ts_buf);

    // Create devices array
    cJSON *devices_array = cJSON_CreateArray();
    if (!devices_array) {
        cJSON_Delete(root);
        return NULL;
    }

    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!list->devices[i].active) {
            continue;
        }

        const ble_device_t *dev = &list->devices[i];

        cJSON *device_obj = cJSON_CreateObject();
        if (!device_obj) {
            continue;
        }

        cJSON_AddStringToObject(device_obj, "mac", dev->mac_str);
        cJSON_AddNumberToObject(device_obj, "rssi", dev->raw_rssi);
        cJSON_AddNumberToObject(device_obj, "distance", dev->distance);

        if (dev->name[0] != '\0') {
            cJSON_AddStringToObject(device_obj, "name", dev->name);
        }

        cJSON_AddStringToObject(device_obj, "category", category_to_string(dev->category));
        cJSON_AddStringToObject(device_obj, "address_type",
                               dev->address_type == ADDRESS_TYPE_PUBLIC ? "public" : "random");
        cJSON_AddNumberToObject(device_obj, "seen_count", dev->seen_count);
        cJSON_AddNumberToObject(device_obj, "first_seen", (double)dev->first_seen);
        cJSON_AddNumberToObject(device_obj, "last_seen", (double)dev->last_seen);

        if (dev->has_superseded_by) {
            char superseded_mac[18];
            mac_to_string(dev->superseded_by, superseded_mac);
            cJSON_AddStringToObject(device_obj, "superseded_by", superseded_mac);
        } else {
            cJSON_AddNullToObject(device_obj, "superseded_by");
        }
        cJSON_AddNumberToObject(device_obj, "superseded_probability", dev->superseded_probability);

        cJSON_AddItemToArray(devices_array, device_obj);
    }

    cJSON_AddItemToObject(root, "devices", devices_array);

    // Add summary
    device_summary_t summary;
    device_get_summary(list, &summary);

    cJSON *summary_obj = cJSON_CreateObject();
    if (summary_obj) {
        cJSON_AddNumberToObject(summary_obj, "total_devices", summary.total_devices);
        cJSON_AddNumberToObject(summary_obj, "phones", summary.phones);
        cJSON_AddNumberToObject(summary_obj, "computers", summary.computers);
        cJSON_AddNumberToObject(summary_obj, "wearables", summary.wearables);
        cJSON_AddNumberToObject(summary_obj, "tablets", summary.tablets);
        cJSON_AddNumberToObject(summary_obj, "beacons", summary.beacons);
        cJSON_AddNumberToObject(summary_obj, "watches", summary.watches);
        cJSON_AddNumberToObject(summary_obj, "headphones", summary.headphones);
        cJSON_AddNumberToObject(summary_obj, "speakers", summary.speakers);
        cJSON_AddNumberToObject(summary_obj, "other", summary.other);
        cJSON_AddItemToObject(root, "summary", summary_obj);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json_str;
}

char *device_json_build_minimal(bool has_fix, float latitude, float longitude,
                                 time_t fix_time, int phone_count, const char *device_id) {
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    // Prefer the GPS's own clock (fix_time, from AT+CGNSSINFO's date/UTC-time
    // fields) over the ESP32's system clock - falls back to time(NULL) only
    // when there's no fix yet, or gnss_parse_cgnssinfo() couldn't parse a
    // date/time out of the response (fix_time left at 0 in both cases).
    time_t ts = (has_fix && fix_time > 0) ? fix_time : time(NULL);
    char ts_buf[ISO8601_BUF_SIZE];
    format_iso8601_utc(ts, ts_buf, sizeof(ts_buf));

    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "timestamp", ts_buf);
    cJSON_AddNumberToObject(root, "phones", phone_count);

    if (has_fix) {
        // Round to 4 decimal places (~11m resolution, plenty for a
        // per-device position report) before handing to cJSON - raw
        // GNSS floats otherwise print with a dozen+ noisy digits
        // (e.g. 47.585407257080078) once promoted to cJSON's double.
        // Rounding done in double precision (not float): doing the
        // divide in float would leave enough binary-fraction error
        // that converting to cJSON's double for printing brings the
        // noisy digits right back.
        cJSON_AddNumberToObject(root, "lat", round((double)latitude * 10000.0) / 10000.0);
        cJSON_AddNumberToObject(root, "lon", round((double)longitude * 10000.0) / 10000.0);
    } else {
        cJSON_AddNullToObject(root, "lat");
        cJSON_AddNullToObject(root, "lon");
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json_str;
}
