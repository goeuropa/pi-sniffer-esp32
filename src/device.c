/**
 * BLE Device Tracking Implementation
 * Manages the list of discovered BLE devices
 */

#include "device.h"
#include "config.h"
#include "apple_heuristic.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

// Apple manufacturer ID
#define APPLE_MANUFACTURER_ID       0x004C

// Microsoft manufacturer ID  
#define MICROSOFT_MANUFACTURER_ID   0x0006

// Samsung manufacturer ID
#define SAMSUNG_MANUFACTURER_ID     0x0075

// Bluetooth SIG-assigned company identifiers seen most often in the wild
static const struct {
    uint16_t id;
    const char *name;
} manufacturer_names[] = {
    { 0x0000, "Ericsson" },
    { 0x0001, "Nokia" },
    { 0x0002, "Intel" },
    { 0x0003, "IBM" },
    { 0x0006, "Microsoft" },
    { 0x000F, "Broadcom" },
    { APPLE_MANUFACTURER_ID, "Apple" },
    { 0x0059, "Nordic Semiconductor" },
    { SAMSUNG_MANUFACTURER_ID, "Samsung" },
    { 0x0087, "Garmin" },
    { 0x00E0, "Google" },
    { 0x0171, "Amazon" },
    { 0x2502, "Murata" },  // module vendor for many OEM devices, e.g. Nespresso machines
};

// Known MAC OUI (first 3 bytes) prefixes that reliably indicate a specific
// device type, sourced from the original pi-sniffer project's
// heuristic-mac.c (git history prior to the ESP32 rewrite). Only meaningful
// for PUBLIC-address devices - random/private BLE addresses don't carry
// real vendor OUI information.
static const struct {
    uint8_t oui[3];
    device_category_t category;
} mac_oui_categories[] = {
    { {0xB8, 0xBC, 0x5B}, CATEGORY_TV },  // Samsung TV
    { {0xD4, 0x9D, 0xC0}, CATEGORY_TV },  // Samsung TV
    { {0xF8, 0x3F, 0x51}, CATEGORY_TV },  // Samsung TV
    { {0x5C, 0xC1, 0xD7}, CATEGORY_TV },  // Samsung TV
    { {0xC8, 0xA6, 0xEF}, CATEGORY_TV },  // Samsung TV (assumed, unconfirmed)
};

// Category name lookup table
static const char* category_names[] = {
    "unknown",
    "phone",
    "wearable",
    "tablet",
    "headphones",
    "computer",
    "tv",
    "beacon",
    "car",
    "watch",
    "fitness",
    "speaker",
    "fixed",
    "appliance",
    "other"
};

void device_list_init(device_list_t *list) {
    memset(list, 0, sizeof(device_list_t));
    list->count = 0;
    list->total_discovered = 0;
}

void mac_to_string(const uint8_t *mac, char *str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void device_set_name(ble_device_t *device, const char *value, name_confidence_t confidence) {
    if (value == NULL || value[0] == '\0') {
        return;
    }
    if (device->name_confidence < confidence) {
        strncpy(device->name, value, MAX_NAME_LENGTH - 1);
        device->name[MAX_NAME_LENGTH - 1] = '\0';
        device->name_confidence = confidence;
    }
}

static void store_stream_payload(stream_window_t *slot, const uint8_t *payload, uint8_t payload_len) {
    if (payload == NULL) {
        slot->payload_len = 0;
        return;
    }
    uint8_t n = payload_len;
    if (n > STREAM_PAYLOAD_MAX_LEN) {
        n = STREAM_PAYLOAD_MAX_LEN;
    }
    memcpy(slot->payload, payload, n);
    slot->payload_len = n;
}

void device_record_stream(ble_device_t *device, uint8_t stream_type, time_t when,
                           const uint8_t *payload, uint8_t payload_len) {
    for (int i = 0; i < device->stream_count; i++) {
        if (device->streams[i].type == stream_type) {
            device->streams[i].last_seen = when;
            store_stream_payload(&device->streams[i], payload, payload_len);
            return;
        }
    }

    if (device->stream_count < MAC_PACK_MAX_TRACKED_STREAMS) {
        stream_window_t *slot = &device->streams[device->stream_count++];
        slot->type = stream_type;
        slot->first_seen = when;
        slot->last_seen = when;
        store_stream_payload(slot, payload, payload_len);
        return;
    }

    // Table full and stream_type isn't already tracked: evict the
    // least-recently-seen slot to make room.
    int lru_index = 0;
    for (int i = 1; i < MAC_PACK_MAX_TRACKED_STREAMS; i++) {
        if (device->streams[i].last_seen < device->streams[lru_index].last_seen) {
            lru_index = i;
        }
    }
    device->streams[lru_index].type = stream_type;
    device->streams[lru_index].first_seen = when;
    device->streams[lru_index].last_seen = when;
    store_stream_payload(&device->streams[lru_index], payload, payload_len);
}

ble_device_t* device_find(device_list_t *list, const uint8_t *mac) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (list->devices[i].active && 
            memcmp(list->devices[i].mac, mac, 6) == 0) {
            return &list->devices[i];
        }
    }
    return NULL;
}

static ble_device_t* find_empty_slot(device_list_t *list) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!list->devices[i].active) {
            return &list->devices[i];
        }
    }
    return NULL;
}

float calculate_distance(int8_t rssi, int8_t tx_power) {
    // Use calibrated value if tx_power isn't provided
    if (tx_power == 0) {
        tx_power = RSSI_ONE_METER;
    }
    
    if (rssi == 0) {
        return -1.0f; // Invalid RSSI
    }
    
    // Log-distance path loss model
    // distance = 10 ^ ((tx_power - rssi) / (10 * n))
    // where n is the path loss exponent
    float exponent = ((float)tx_power - (float)rssi) / (10.0f * PATH_LOSS_EXPONENT);
    float distance = powf(10.0f, exponent);
    
    // Cap unreasonable values
    if (distance > MAX_DISTANCE_METERS) {
        distance = MAX_DISTANCE_METERS;
    }
    if (distance < 0.0f) {
        distance = 0.0f;
    }
    
    return distance;
}

void device_categorize(ble_device_t *device) {
    // Already categorized
    if (device->category != CATEGORY_UNKNOWN) {
        return;
    }

    // A known-vendor MAC prefix is the most specific signal we have, but
    // only meaningful for public (non-randomized) addresses.
    if (device->address_type == ADDRESS_TYPE_PUBLIC) {
        for (size_t i = 0; i < sizeof(mac_oui_categories) / sizeof(mac_oui_categories[0]); i++) {
            if (memcmp(device->mac, mac_oui_categories[i].oui, 3) == 0) {
                device->category = mac_oui_categories[i].category;
                return;
            }
        }
    }

    // Categorize based on manufacturer ID
    switch (device->manufacturer_id) {
        case APPLE_MANUFACTURER_ID:
            // Apple devices - likely phone, tablet, or watch
            // Could refine based on manufacturer data bytes
            device->category = CATEGORY_PHONE;
            break;

        case MICROSOFT_MANUFACTURER_ID:
            // Microsoft - likely computer
            device->category = CATEGORY_COMPUTER;
            break;

        case SAMSUNG_MANUFACTURER_ID:
            // Samsung's company ID (0x0075) is used by phones, watches, AND
            // TVs alike - the original pi-sniffer ran into this exact
            // ambiguity (see heuristic-manufacturers.c in git history) and
            // deliberately did NOT assume phone, instead planning to
            // connect and query the device's real name - which this
            // passive-only scanner doesn't do. Leave uncategorized rather
            // than guess wrong; the OUI check above and the name/
            // random-address fallbacks below still get a chance.
            break;

        default:
            break;
    }
    
    // Categorize based on name patterns
    if (device->name[0] != '\0' && device->category == CATEGORY_UNKNOWN) {
        // Convert name to lowercase for comparison
        char lower_name[MAX_NAME_LENGTH];
        for (int i = 0; i < MAX_NAME_LENGTH && device->name[i]; i++) {
            lower_name[i] = (device->name[i] >= 'A' && device->name[i] <= 'Z') 
                          ? device->name[i] + 32 
                          : device->name[i];
        }
        lower_name[MAX_NAME_LENGTH - 1] = '\0';
        
        if (strstr(lower_name, "iphone") || strstr(lower_name, "android") ||
            strstr(lower_name, "pixel") || strstr(lower_name, "galaxy") ||
            strstr(lower_name, "oneplus") || strstr(lower_name, "xiaomi")) {
            device->category = CATEGORY_PHONE;
        }
        else if (strstr(lower_name, "ipad") || strstr(lower_name, "tablet")) {
            device->category = CATEGORY_TABLET;
        }
        else if (strstr(lower_name, "macbook") || strstr(lower_name, "laptop") ||
                 strstr(lower_name, "windows") || strstr(lower_name, "pc")) {
            device->category = CATEGORY_COMPUTER;
        }
        else if (strstr(lower_name, "watch") || strstr(lower_name, "band") ||
                 strstr(lower_name, "fitbit") || strstr(lower_name, "garmin")) {
            device->category = CATEGORY_WATCH;
        }
        else if (strstr(lower_name, "airpod") || strstr(lower_name, "buds") ||
                 strstr(lower_name, "headphone") || strstr(lower_name, "earphone")) {
            device->category = CATEGORY_HEADPHONES;
        }
        else if (strstr(lower_name, "speaker") || strstr(lower_name, "sonos") ||
                 strstr(lower_name, "bose") || strstr(lower_name, "jbl")) {
            device->category = CATEGORY_SPEAKER;
        }
        else if (strstr(lower_name, "beacon") || strstr(lower_name, "tile") ||
                 strstr(lower_name, "airtag")) {
            device->category = CATEGORY_BEACON;
        }
        else if (strstr(lower_name, "tv") || strstr(lower_name, "roku") ||
                 strstr(lower_name, "fire stick") || strstr(lower_name, "chromecast")) {
            device->category = CATEGORY_TV;
        }
        else if (strstr(lower_name, "venus")) {
            // Nespresso machines advertise as "Venus_<serial>" (over a
            // Murata-manufactured BLE module, manufacturer ID 0x2502, but
            // Murata modules are used by many unrelated products - the
            // "Venus" name prefix is what actually identifies this as a
            // Nespresso machine)
            device->category = CATEGORY_APPLIANCE;
        }
        else if (strstr(lower_name, "droplet")) {
            // e.g. "Droplet-46A4" - a smart irrigation/appliance controller
            device->category = CATEGORY_APPLIANCE;
        }
    }
    
    // Categorize based on address type
    // Random addresses are more likely to be phones/tablets (MAC randomization)
    if (device->category == CATEGORY_UNKNOWN && 
        device->address_type == ADDRESS_TYPE_RANDOM) {
        device->category = CATEGORY_PHONE; // Assume phone for random addresses
    }
}

ble_device_t* device_update(device_list_t *list,
                            const uint8_t *mac,
                            int8_t rssi,
                            address_type_t addr_type,
                            const char *name,
                            uint16_t manufacturer_id,
                            int8_t tx_power,
                            const uint8_t *manufacturer_payload,
                            uint8_t manufacturer_payload_len) {
    time_t now = time(NULL);
    ble_device_t *device = device_find(list, mac);
    
    if (device == NULL) {
        // New device
        device = find_empty_slot(list);
        if (device == NULL) {
            // List is full, could implement LRU here
            return NULL;
        }
        
        // Initialize new device
        memset(device, 0, sizeof(ble_device_t));
        memcpy(device->mac, mac, 6);
        mac_to_string(mac, device->mac_str);
        device->active = true;
        device->first_seen = now;
        device->category = CATEGORY_UNKNOWN;
        device->tx_power = TX_POWER_UNKNOWN;
        kalman_init(&device->rssi_filter);
        
        list->count++;
        list->total_discovered++;
    }
    
    // Update device data
    device->last_seen = now;
    device->seen_count++;
    device->raw_rssi = rssi;
    device->address_type = addr_type;
    
    // The advertised local name is the most trustworthy source we have
    device_set_name(device, name, NAME_CONF_KNOWN);

    // Update manufacturer ID. A manufacturer-specific AD structure was
    // present whenever a payload pointer came back, regardless of whether
    // the ID itself happens to be 0x0000 (Ericsson's real company ID).
    if (manufacturer_payload != NULL) {
        device->manufacturer_id = manufacturer_id;
        device->has_manufacturer_data = true;
    }

    // Decode Apple's Continuity protocol manufacturer data, if present, to
    // fill in name/category with more confidence than the generic
    // manufacturer-ID/random-address guesses below can. Must run before
    // device_categorize() so it gets first claim on the category.
#if ENABLE_APPLE_HEURISTICS
    if (device->manufacturer_id == APPLE_MANUFACTURER_ID &&
        manufacturer_payload != NULL && manufacturer_payload_len > 0) {
        apple_heuristic_process(device, manufacturer_payload, manufacturer_payload_len);
    }
#endif
    
    // Update TX power
    if (tx_power != TX_POWER_UNKNOWN) {
        device->tx_power = tx_power;
    }

    // Always use the calibrated RSSI-at-1m constant rather than deriving it
    // from the device's self-reported TX power: a free-space-path-loss
    // conversion from TX power is systematically far too optimistic (e.g. a
    // phone reporting 12 dBm TX power implies -29 dBm at 1m, but real
    // phones - antenna orientation, hand/pocket/chassis attenuation - never
    // get close to that), which blew distance estimates out by 10x-100x.
    int8_t reference_rssi_at_1m = RSSI_ONE_METER;

    // Apply Kalman filter to RSSI
    float filtered_rssi = kalman_update(&device->rssi_filter, (float)rssi);

    // Calculate distance from filtered RSSI
    device->distance = calculate_distance((int8_t)lroundf(filtered_rssi), reference_rssi_at_1m);
    
    // Try to categorize the device
#if ENABLE_CATEGORIZATION
    device_categorize(device);
#endif
    
    return device;
}

int device_cleanup(device_list_t *list, int max_age_sec) {
    time_t now = time(NULL);
    int removed = 0;
    
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (list->devices[i].active) {
            double age = difftime(now, list->devices[i].last_seen);
            if (age > max_age_sec) {
                list->devices[i].active = false;
                list->count--;
                removed++;
            }
        }
    }
    
    return removed;
}

void device_get_summary(const device_list_t *list, device_summary_t *summary) {
    memset(summary, 0, sizeof(device_summary_t));
    
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!list->devices[i].active) {
            continue;
        }
        if (list->devices[i].has_superseded_by) {
            continue;  // MAC-rotated ghost, already counted under its newer MAC
        }
        if (list->devices[i].seen_count < DEVICE_MIN_SEEN_COUNT_FOR_SUMMARY) {
            continue;  // one-off blip, not enough evidence of a real device yet
        }
        if (list->devices[i].raw_rssi < MIN_RSSI_FOR_SUMMARY) {
            continue;  // signal too weak to trust - risk of a misdecoded MAC
        }

        summary->total_devices++;

        switch (list->devices[i].category) {
            case CATEGORY_PHONE:
                summary->phones++;
                break;
            case CATEGORY_COMPUTER:
                summary->computers++;
                break;
            case CATEGORY_WEARABLE:
                summary->wearables++;
                break;
            case CATEGORY_TABLET:
                summary->tablets++;
                break;
            case CATEGORY_BEACON:
                summary->beacons++;
                break;
            case CATEGORY_WATCH:
                summary->watches++;
                break;
            case CATEGORY_HEADPHONES:
                summary->headphones++;
                break;
            case CATEGORY_SPEAKER:
                summary->speakers++;
                break;
            default:
                summary->other++;
                break;
        }
    }
}

const char* category_to_string(device_category_t category) {
    if (category >= 0 && category < sizeof(category_names) / sizeof(category_names[0])) {
        return category_names[category];
    }
    return "unknown";
}

const char* manufacturer_to_string(bool has_manufacturer_id, uint16_t manufacturer_id, char *buf, size_t buf_len) {
    if (!has_manufacturer_id) {
        return "unknown";
    }
    for (size_t i = 0; i < sizeof(manufacturer_names) / sizeof(manufacturer_names[0]); i++) {
        if (manufacturer_names[i].id == manufacturer_id) {
            return manufacturer_names[i].name;
        }
    }
    snprintf(buf, buf_len, "0x%04X", manufacturer_id);
    return buf;
}
