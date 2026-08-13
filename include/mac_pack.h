/**
 * MAC-Rotation Packing
 *
 * Detects when a BLE device's random MAC address has likely rotated (a
 * privacy feature of iOS/Android) so the same physical device isn't counted
 * twice. Ported from the original pi-sniffer project's overlaps.c /
 * pack_closest_columns(), simplified for a single access point and the flat
 * device_list_t model used by this firmware (see the ESP32 port's plan for
 * the full mapping from the original multi-access-point algorithm).
 */

#ifndef MAC_PACK_H
#define MAC_PACK_H

#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include "device.h"

/**
 * Do two observation windows overlap in time? If they do, the two MACs
 * cannot be the same physical device. Touching endpoints (one window's
 * first_seen equal to the other's last_seen) do not count as overlap.
 */
bool mac_pack_overlaps(time_t a_first, time_t a_last, time_t b_first, time_t b_last);

/**
 * Do older and newer conflict in time, i.e. can they NOT be the same
 * physical device rotating its MAC? If both devices have recorded
 * per-Continuity-type stream windows (device_record_stream()), only a
 * *shared* stream type whose own windows overlap counts as a conflict -
 * overlap on different types is treated as the same phone's independently
 * rotating concurrent Continuity streams (a "ragged handover") and does not
 * disqualify the pair. Falls back to mac_pack_overlaps() on the aggregate
 * first_seen/last_seen window whenever either device has no recorded
 * stream data (non-Apple devices, Apple heuristics disabled, or an Apple
 * device never seen with a usable payload).
 */
bool mac_pack_windows_conflict(const ble_device_t *older, const ble_device_t *newer);

/**
 * Do older and newer share a stream type whose most recently recorded raw
 * payload content is byte-identical (see stream_window_t.payload)? Several
 * Continuity message types (Handoff, Instant Hotspot, Nearby Info's trailing
 * auth tag) carry several bytes of effectively pseudo-random per-device/
 * per-session content, so two different MACs matching on it is far stronger
 * evidence of being the *same* physical device than two independent devices
 * coincidentally matching by chance - strong enough to override
 * mac_pack_windows_conflict()'s "same type + overlapping window = two
 * devices" assumption, which has no way to tell that apart from one device
 * broadcasting concurrently on two BLE identities for that stream (an
 * observed real behavior, not a bug). Requires at least
 * MAC_PACK_MIN_PAYLOAD_MATCH_LEN bytes on both sides to guard against
 * matching on short/low-entropy payloads.
 */
bool mac_pack_payload_matches(const ble_device_t *older, const ble_device_t *newer);

/**
 * Was the older or newer device's data too sparse/oddly-timed to trust as a
 * real MAC rotation? True if either side has a single observation and the
 * gap between them is under MAC_PACK_BLIP_MIN_GAP_SEC (same burst, noise) or
 * over MAC_PACK_BLIP_MAX_GAP_SEC (too far apart to be related).
 */
bool mac_pack_is_blip(time_t older_first, time_t older_last, uint32_t older_count,
                       time_t newer_first, time_t newer_last, uint32_t newer_count);

/**
 * Could `newer` plausibly be `older` after a MAC rotation, based on address
 * type, category, and name? Does not check timing.
 */
bool mac_pack_compatible(const ble_device_t *older, const ble_device_t *newer);

/**
 * Confidence [0,1] that `newer` is `older` after a MAC rotation, given the
 * pair already passed the compatibility/overlap/blip checks. Symmetric
 * exponential decay on the *magnitude* of the gap between older's last
 * sighting and newer's first sighting: a small negative gap (older still
 * seen a moment after newer's first sighting - normal scan-cycle jitter
 * around a genuine near-simultaneous handover) scores near 1.0, same as a
 * small positive gap, but a large negative gap (older still actively,
 * concurrently live long after newer appeared - two live identities, not a
 * rotation) decays toward 0 just like a large positive gap does.
 */
float mac_pack_probability(const ble_device_t *older, const ble_device_t *newer);

/**
 * Run a full pass over the device list, resetting and recomputing the
 * has_superseded_by / superseded_by / superseded_probability fields for
 * every active device.
 */
void mac_pack_run(device_list_t *list);

#endif // MAC_PACK_H
