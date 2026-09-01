/**
 * MAC-Rotation Packing Implementation
 */

#include "mac_pack.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

bool mac_pack_overlaps(time_t a_first, time_t a_last, time_t b_first, time_t b_last) {
    if (a_first >= b_last) return false;  // a entirely after b
    if (b_first >= a_last) return false;  // b entirely after a
    return true;                          // otherwise they must overlap
}

bool mac_pack_is_blip(time_t older_first, time_t older_last, uint32_t older_count,
                       time_t newer_first, time_t newer_last, uint32_t newer_count) {
    (void)older_first;
    (void)newer_last;

    if (older_count != 1 && newer_count != 1) {
        return false;  // both sides have multiple observations, not a blip
    }

    double gap = difftime(newer_first, older_last);
    if (gap < 0) gap = 0;

    return gap < MAC_PACK_BLIP_MIN_GAP_SEC || gap > MAC_PACK_BLIP_MAX_GAP_SEC;
}

bool mac_pack_windows_conflict(const ble_device_t *older, const ble_device_t *newer) {
    if (older->stream_count == 0 || newer->stream_count == 0) {
        // No per-type data on one/both sides (non-Apple device, Apple
        // heuristics disabled, or an Apple device never seen with a usable
        // payload) - fall back to the aggregate-window check.
        return mac_pack_overlaps(older->first_seen, older->last_seen,
                                  newer->first_seen, newer->last_seen);
    }

    // Both sides have per-type data: only a *shared* type's own window
    // overlapping is a real conflict. Overlap on different types is the
    // same phone's independently-rotating concurrent Continuity streams (a
    // ragged handover) and must not disqualify the pair.
    for (int i = 0; i < older->stream_count; i++) {
        for (int j = 0; j < newer->stream_count; j++) {
            if (newer->streams[j].type != older->streams[i].type) continue;
            if (mac_pack_overlaps(older->streams[i].first_seen, older->streams[i].last_seen,
                                   newer->streams[j].first_seen, newer->streams[j].last_seen)) {
                return true;
            }
        }
    }
    return false;
}

bool mac_pack_payload_matches(const ble_device_t *older, const ble_device_t *newer) {
    for (int i = 0; i < older->stream_count; i++) {
        for (int j = 0; j < newer->stream_count; j++) {
            if (newer->streams[j].type != older->streams[i].type) continue;

            uint8_t len = older->streams[i].payload_len;
            if (len < MAC_PACK_MIN_PAYLOAD_MATCH_LEN || len != newer->streams[j].payload_len) {
                continue;
            }
            if (memcmp(older->streams[i].payload, newer->streams[j].payload, len) == 0) {
                return true;
            }
        }
    }
    return false;
}

bool mac_pack_compatible(const ble_device_t *older, const ble_device_t *newer) {
    // Cannot be the same device if one is public and the other random, or
    // either address is public (public addresses don't rotate).
    bool different_address_types =
        (older->address_type != ADDRESS_TYPE_UNKNOWN &&
         newer->address_type != ADDRESS_TYPE_UNKNOWN &&
         older->address_type != newer->address_type);
    bool either_public =
        (older->address_type == ADDRESS_TYPE_PUBLIC || newer->address_type == ADDRESS_TYPE_PUBLIC);
    if (different_address_types || either_public) {
        return false;
    }

    // Cannot be the same device if the older device's category is unknown,
    // or both categories are known and differ.
    bool different_categories =
        (older->category != newer->category) || (older->category == CATEGORY_UNKNOWN);
    if (different_categories) {
        return false;
    }

    // Cannot be the same device if both have a *real advertised* name (the
    // highest confidence level) and they differ. Heuristic-derived names
    // (e.g. Apple Continuity guesses like "iPhone di=01f" or "AirPlay
    // Source") are excluded from this check: they're inherently volatile -
    // different message types and even the same message type at a
    // different lock/screen state produce different strings for the same
    // physical device - so treating them as a hard identity mismatch was
    // blocking real MAC-rotation links between observations of the same
    // phone (observed in practice: three real iPhones were showing up as
    // five-plus separate tracked devices).
    bool different_names =
        (older->name_confidence == NAME_CONF_KNOWN && newer->name_confidence == NAME_CONF_KNOWN &&
         older->name[0] != '\0' && newer->name[0] != '\0' && strcmp(older->name, newer->name) != 0);
    if (different_names) {
        return false;
    }

    return true;
}

float mac_pack_probability(const ble_device_t *older, const ble_device_t *newer) {
    // Symmetric exponential decay on the *magnitude* of the gap, not just a
    // clamped-to-zero positive gap. A small negative gap (older's last
    // sighting falls a moment after newer's first) is normal scan-cycle
    // timing jitter around a genuine near-simultaneous handover (see the
    // ragged cross-type overlap case in mac_pack_windows_conflict) and still
    // scores near 1.0. But a *large* negative gap - older still being
    // actively, concurrently seen long after newer already appeared - is
    // just as implausible as a large positive gap and must decay toward 0
    // the same way. Without this, an unrelated device that's still live
    // scored a spurious 1.0 (any negative gap clamped straight to zero) and
    // could outrank - and steal the single predecessor slot from - the
    // device that actually rotated in.
    double gap = difftime(newer->first_seen, older->last_seen);
    return expf((float)(-fabs(gap) / MAC_PACK_TIME_CONSTANT_SEC));
}

static int compare_by_first_seen_desc(const void *pa, const void *pb) {
    const ble_device_t *a = *(const ble_device_t * const *)pa;
    const ble_device_t *b = *(const ble_device_t * const *)pb;
    if (a->first_seen > b->first_seen) return -1;
    if (a->first_seen < b->first_seen) return 1;
    return 0;
}

void mac_pack_run(device_list_t *list) {
    ble_device_t *active[MAX_DEVICES];
    int active_count = 0;

    for (int i = 0; i < MAX_DEVICES; i++) {
        ble_device_t *device = &list->devices[i];
        if (!device->active) continue;

        device->has_superseded_by = false;
        memset(device->superseded_by, 0, sizeof(device->superseded_by));
        device->superseded_probability = 0.0f;

        active[active_count++] = device;
    }

    qsort(active, active_count, sizeof(ble_device_t *), compare_by_first_seen_desc);

    // Newest first: each device (as 'newer') looks backward for the best
    // still-unclaimed, older, compatible, non-overlapping predecessor.
    for (int i = 0; i < active_count; i++) {
        ble_device_t *newer = active[i];
        ble_device_t *best = NULL;
        float best_prob = 0.0f;

        for (int j = 0; j < active_count; j++) {
            ble_device_t *older = active[j];
            if (older == newer) continue;
            if (older->first_seen > newer->first_seen) continue;
            // Tie-break equal first_seen values (common for two concurrent
            // identities of the same device, e.g. a payload match below,
            // that were both already broadcasting when tracking started) by
            // MAC bytes, so exactly one direction is ever eligible - without
            // this, older and newer could each treat the other as their own
            // predecessor and both end up superseded, mutually pointing at
            // each other and vanishing from every summary count.
            if (older->first_seen == newer->first_seen &&
                memcmp(older->mac, newer->mac, sizeof(older->mac)) >= 0) continue;
            if (older->has_superseded_by) continue;  // already claimed by someone else

            if (!mac_pack_compatible(older, newer)) continue;

            // A byte-identical payload on a shared stream type is decisive
            // same-device evidence (see mac_pack_payload_matches()) - strong
            // enough to override the windows-conflict/blip checks below,
            // which would otherwise reject this pair for looking like two
            // devices concurrently, rather than one device broadcasting on
            // two BLE identities at once.
            bool payload_match = mac_pack_payload_matches(older, newer);
            if (!payload_match) {
                if (mac_pack_windows_conflict(older, newer)) continue;
                if (mac_pack_is_blip(older->first_seen, older->last_seen, older->seen_count,
                                      newer->first_seen, newer->last_seen, newer->seen_count)) continue;
            }

            float p = payload_match ? 1.0f : mac_pack_probability(older, newer);
            if (p > best_prob) {
                best = older;
                best_prob = p;
            }
        }

        if (best != NULL && best_prob > MAC_PACK_PROBABILITY_THRESHOLD) {
            best->has_superseded_by = true;
            memcpy(best->superseded_by, newer->mac, sizeof(best->superseded_by));
            best->superseded_probability = best_prob;
        }
    }
}
