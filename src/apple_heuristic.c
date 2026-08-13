/**
 * Apple Continuity Protocol Heuristics
 *
 * Decodes Apple's manufacturer-specific advertisement data (company ID
 * 0x004C) to identify what kind of message a device is sending - Proximity
 * Pairing (AirPods), Nearby Info (iPhone/iPad/Watch), Handoff, AirDrop,
 * AirPlay, HomeKit, etc. - and uses that to set the device's name/category
 * with more confidence than the generic manufacturer/random-address guesses
 * in device_categorize() can.
 *
 * Ported from the original pi-sniffer project's
 * src/bluetooth/heuristic-apple.c (recoverable via
 * `git show a8146a9~1:src/bluetooth/heuristic-apple.c` in this repo's
 * history, prior to the ESP32 rewrite), adapted to this firmware's flat
 * ble_device_t model and name-confidence scheme (device_set_name()) in
 * place of the original's name_type precedence system. Every message type
 * logs at INFO for now, including the ones that fire continuously (HomeKit,
 * Nearby Info) - tune this down once the detection logic has been observed
 * in practice. Detection/categorization logic itself is unchanged from the
 * original.
 */

#include "apple_heuristic.h"

#include <stdio.h>

// ESP_PLATFORM is defined by the ESP-IDF build system for every component;
// fall back to plain printf when compiled outside it (e.g. host-side unit
// tests), so this file stays testable without pulling in ESP-IDF headers.
#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#define ESP_LOGI(tag, fmt, ...) printf("I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) printf("D (%s) " fmt "\n", tag, ##__VA_ARGS__)
#endif

static const char *TAG = "APPLE_HEUR";

static void soft_set_category(device_category_t *field, device_category_t value) {
    if (*field == CATEGORY_UNKNOWN) {
        *field = value;
    }
}

void apple_heuristic_process(ble_device_t *device, const uint8_t *payload, uint8_t payload_len) {
    if (payload_len < 1) {
        return;
    }

    uint8_t apple_device_type = payload[0];
    device_record_stream(device, apple_device_type, device->last_seen, payload, payload_len);

    switch (apple_device_type) {
        case 0x01:
            // An iMac causes this. Mostly iPhone? iWatch too?
            device_set_name(device, "Apple", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Apple device type 0x01 - what is this?", device->mac_str, device->name);
            break;

        case 0x02:
            // iBeacon
            device_set_name(device, "Beacon", NAME_CONF_MANUFACTURER);
            if (device->category != CATEGORY_BEACON) {
                ESP_LOGD(TAG, "%s '%s' Beacon", device->mac_str, device->name);
            }
            device->category = CATEGORY_BEACON;  // hard set: unambiguous once seen
            break;

        case 0x03:
            // AirPrint - on user action
            device_set_name(device, "AirPrint", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' AirPrint", device->mac_str, device->name);
            break;

        case 0x05:
            // AirDrop - on user action
            device_set_name(device, "AirDrop", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' AirDrop", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);
            break;

        case 0x06:
            // HomeKit - sent constantly by HomeKit accessories
            device_set_name(device, "HomeKit", NAME_CONF_DEVICE);
            ESP_LOGD(TAG, "%s '%s' HomeKit", device->mac_str, device->name);
            break;

        case 0x07:
            // Proximity Pairing (AirPods) - sent constantly, but rare to catch
            device_set_name(device, "AirPods", NAME_CONF_DEVICE);
            ESP_LOGD(TAG, "%s '%s' Proximity Pairing", device->mac_str, device->name);
            device->category = CATEGORY_HEADPHONES;  // hard set: unambiguous once seen
            break;

        case 0x08:
            // Siri - on user action, rare
            device_set_name(device, "Siri", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Siri", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);  // could be anything, assume phone
            break;

        case 0x09:
            // AirPlay - on user action for some
            device_set_name(device, "AirPlay", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' AirPlay", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_FIXED);  // probably an Apple TV?
            break;

        case 0x0a:
            // AirPlay Source - not in the original pi-sniffer (it left this
            // as an unrecognized mystery type); documented by furiousMAC's
            // Continuity protocol research. This is the *sending* device
            // announcing an AirPlay stream (the counterpart to 0x09 AirPlay
            // Target, which is the receiver, e.g. an Apple TV), so it's a
            // phone/computer, not a fixed receiver.
            device_set_name(device, "AirPlay Source", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' AirPlay Source", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);  // could be a Mac, assume phone
            break;

        case 0x0b:
            // Magic Switch - sent when an Apple Watch has lost pairing to its phone
            device_set_name(device, "iWatch", NAME_CONF_DEVICE);
            ESP_LOGD(TAG, "%s '%s' Magic Switch", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_WATCH);
            break;

        case 0x0c:
            // Handoff - phones, iPads, and Macs all do this
            device_set_name(device, "Apple Handoff", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Handoff", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);  // might be an iPad or Mac
            break;

        case 0x0d:
            // Instant Hotspot - on user action
            device_set_name(device, "Apple WifiSet", NAME_CONF_DEVICE);
            ESP_LOGD(TAG, "%s '%s' WifiSet", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);  // might be an iPad, assume phone
            break;

        case 0x0e:
            // Instant Hotspot - reaction to target presence
            device_set_name(device, "Apple Hotspot", NAME_CONF_DEVICE);
            ESP_LOGD(TAG, "%s '%s' Hotspot", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_PHONE);  // could be Mac, iPad or iPhone
            break;

        case 0x0f: {
            // Nearby Action - on user action, rare (e.g. WiFi password sharing)
            if (payload_len < 4) break;
            char temp_name[MAX_NAME_LENGTH];
            snprintf(temp_name, sizeof(temp_name), "Apple Near af=%.2x at=%.2x", payload[2], payload[3]);
            device_set_name(device, temp_name, NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Nearby Action 0x0f", device->mac_str, device->name);
            break;
        }

        case 0x10: {
            // Nearby Info - sent constantly, almost certainly an iPhone.
            // 1 byte length, 1 byte activity level, 1 byte information, 3 bytes auth tag
            if (payload_len < 4) break;

            uint8_t device_bit = (payload[2] >> 1) & 0x01;  // combined with information byte
            uint8_t information_byte = payload[3];
            uint8_t activity_bits = payload[2] & 0xf9;      // everything but the device/screen bits

            ESP_LOGD(TAG, "%s '%s' Nearby Info: d=%.1x info=%.2x act=%.2x",
                     device->mac_str, device->name, device_bit, information_byte, activity_bits);

            // temp_name deliberately excludes activity_bits: it reflects
            // momentary phone state (locked/unlocked, screen on/off) and
            // changes packet to packet. Since device_set_name() only
            // overwrites on higher confidence, a name is effectively frozen
            // at first sight - if it included activity_bits, the same
            // physical phone would freeze two different names before and
            // after a MAC rotation, and mac_pack's exact-name-match would
            // then treat them as different devices instead of linking them
            // (this was an observed bug: real MAC rotations of the same
            // iPhone were not being linked, inflating the device count).
            // activity_bits is still logged above for debugging.
            char temp_name[MAX_NAME_LENGTH];
            device_category_t guessed_category = CATEGORY_PHONE;

            if (device_bit == 0x0 && information_byte == 0x00) {
                // Seems to be Apple Watch
                guessed_category = CATEGORY_WATCH;
                snprintf(temp_name, sizeof(temp_name), "Apple di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x0 && information_byte == 0x19) {
                // Confirmed MacBook Air: observed di=019 appear/disappear in
                // lockstep with toggling the laptop's Bluetooth off and on.
                guessed_category = CATEGORY_COMPUTER;
                snprintf(temp_name, sizeof(temp_name), "MacBook di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x1 && information_byte == 0x19) {
                // Inferred, not independently confirmed the way di=019 was:
                // same information_byte as the confirmed MacBook Air above,
                // just a different device_bit - following the same pattern
                // already established for iPad below (di=01d and di=11d
                // both map to iPad; device_bit appears to be a state/
                // modifier bit, not a type differentiator). Likely the same
                // MacBook Air in a different state (e.g. lid open/closed).
                guessed_category = CATEGORY_COMPUTER;
                snprintf(temp_name, sizeof(temp_name), "MacBook di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x0 && information_byte == 0x1c) {
                snprintf(temp_name, sizeof(temp_name), "Apple di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x1 && information_byte == 0x1c && activity_bits == 0x11) {
                // Mostly a phone
                snprintf(temp_name, sizeof(temp_name), "iPhone di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x0 && information_byte == 0x1d) {
                // Seems to be mostly iPad
                guessed_category = CATEGORY_TABLET;
                snprintf(temp_name, sizeof(temp_name), "iPad di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x1 && information_byte == 0x1d) {
                guessed_category = CATEGORY_TABLET;
                snprintf(temp_name, sizeof(temp_name), "iPad di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x1 && information_byte == 0x1a) {
                // Seems to always be a phone
                snprintf(temp_name, sizeof(temp_name), "iPhone di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x0 && information_byte == 0x1f) {
                // Seems to always be a phone
                snprintf(temp_name, sizeof(temp_name), "iPhone di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x01 && information_byte == 0x18 && activity_bits == 0x01) {
                guessed_category = CATEGORY_WATCH;
                snprintf(temp_name, sizeof(temp_name), "Apple Watch di=%.1x%.2x", device_bit, information_byte);
            } else if (device_bit == 0x01 && information_byte == 0x18) {
                // watch or Macbook Pro?
                guessed_category = CATEGORY_WATCH;
                snprintf(temp_name, sizeof(temp_name), "Apple Watch/Macbook di=%.1x%.2x", device_bit, information_byte);
            } else if (information_byte == 0x98) {
                guessed_category = CATEGORY_WATCH;
                snprintf(temp_name, sizeof(temp_name), "Apple Watch di=%.1x%.2x", device_bit, information_byte);
            } else {
                snprintf(temp_name, sizeof(temp_name), "Apple di=%.1x%.2x", device_bit, information_byte);
            }

            soft_set_category(&device->category, guessed_category);
            device_set_name(device, temp_name, NAME_CONF_MANUFACTURER);
            break;
        }

        case 0x12:
            // Find My (Offline Finding) - an AirTag, or another Apple device
            // broadcasting a "separated from owner" beacon so nearby Apple
            // devices can relay its location via the Find My network. Not
            // in the original pi-sniffer (predates AirTags); documented by
            // later public reverse-engineering of the Continuity protocol.
            device_set_name(device, "Find My", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Find My", device->mac_str, device->name);
            device->category = CATEGORY_BEACON;  // hard set: unambiguous once seen, like iBeacon
            break;

        case 0x13:
            // New Apple device type - what is it? M1 laptop?
            device_set_name(device, "Apple Type 0x13", NAME_CONF_MANUFACTURER);
            ESP_LOGD(TAG, "%s '%s' Apple 0x13", device->mac_str, device->name);
            soft_set_category(&device->category, CATEGORY_COMPUTER);  // 100% sure this is a laptop not a phone
            break;

        default:
            ESP_LOGD(TAG, "%s '%s' Did not recognize Apple device type 0x%.2x",
                     device->mac_str, device->name, apple_device_type);
            break;
    }
}
