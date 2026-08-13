/**
 * Host-side unit tests for mac_pack.c.
 *
 * Not part of the ESP-IDF/PlatformIO firmware build. Compile and run with a
 * plain host compiler, e.g.:
 *
 *   gcc -I../include -o /tmp/test_mac_pack test_mac_pack.c \
 *       ../src/mac_pack.c ../src/device.c ../src/kalman.c \
 *       ../src/apple_heuristic.c -lm
 *   /tmp/test_mac_pack
 */

#include "mac_pack.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);    \
            failures++;                                               \
        } else {                                                      \
            printf("ok:   %s\n", msg);                                \
        }                                                              \
    } while (0)

static ble_device_t make_device(uint8_t mac_last_byte, time_t first_seen, time_t last_seen,
                                 uint32_t seen_count, address_type_t addr_type,
                                 device_category_t category, const char *name) {
    ble_device_t d;
    memset(&d, 0, sizeof(d));
    d.mac[5] = mac_last_byte;
    d.first_seen = first_seen;
    d.last_seen = last_seen;
    d.seen_count = seen_count;
    d.address_type = addr_type;
    d.category = category;
    d.active = true;
    if (name) {
        strncpy(d.name, name, MAX_NAME_LENGTH - 1);
    }
    return d;
}

// Populates the next free streams[] slot directly, bypassing
// device_record_stream(), so window values in these tests are exact and
// independent of that function's eviction policy (which gets its own
// dedicated tests below).
static void set_stream(ble_device_t *d, uint8_t type, time_t first_seen, time_t last_seen) {
    stream_window_t *slot = &d->streams[d->stream_count++];
    slot->type = type;
    slot->first_seen = first_seen;
    slot->last_seen = last_seen;
}

// Same as set_stream(), but also records a raw payload - for
// mac_pack_payload_matches() tests.
static void set_stream_with_payload(ble_device_t *d, uint8_t type, time_t first_seen, time_t last_seen,
                                     const uint8_t *payload, uint8_t payload_len) {
    stream_window_t *slot = &d->streams[d->stream_count++];
    slot->type = type;
    slot->first_seen = first_seen;
    slot->last_seen = last_seen;
    memcpy(slot->payload, payload, payload_len);
    slot->payload_len = payload_len;
}

static void test_overlap(void) {
    // Windows that share time cannot be the same rotated device.
    CHECK(mac_pack_overlaps(0, 10, 5, 15) == true, "overlapping windows overlap");
    CHECK(mac_pack_overlaps(0, 10, 10, 20) == false, "touching windows do not overlap");
    CHECK(mac_pack_overlaps(0, 10, 20, 30) == false, "disjoint windows do not overlap");
    CHECK(mac_pack_overlaps(20, 30, 0, 10) == false, "disjoint windows do not overlap (reversed)");
}

static void test_blip(void) {
    // Single-observation device with a very short gap: same burst, noise.
    CHECK(mac_pack_is_blip(0, 100, 1, 101, 101, 5) == true, "single obs + 1s gap is a blip");
    // Single-observation device with a very long gap: too far apart to trust.
    CHECK(mac_pack_is_blip(0, 100, 1, 300, 300, 5) == true, "single obs + 200s gap is a blip");
    // Single-observation device with a plausible gap: not a blip.
    CHECK(mac_pack_is_blip(0, 100, 1, 105, 105, 5) == false, "single obs + 5s gap is not a blip");
    // Both sides have multiple observations: never a blip, regardless of gap.
    CHECK(mac_pack_is_blip(0, 100, 10, 101, 101, 10) == false, "multi-obs short gap is not a blip");
    CHECK(mac_pack_is_blip(0, 100, 10, 300, 300, 10) == false, "multi-obs long gap is not a blip");
}

static void test_compatible(void) {
    ble_device_t older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "Ian's iPhone");
    ble_device_t newer = make_device(2, 105, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "Ian's iPhone");
    older.name_confidence = NAME_CONF_KNOWN;
    newer.name_confidence = NAME_CONF_KNOWN;
    CHECK(mac_pack_compatible(&older, &newer) == true, "matching address/category/name is compatible");

    ble_device_t public_older = older;
    public_older.address_type = ADDRESS_TYPE_PUBLIC;
    CHECK(mac_pack_compatible(&public_older, &newer) == false, "public address is never compatible");

    ble_device_t public_newer = newer;
    public_newer.address_type = ADDRESS_TYPE_PUBLIC;
    CHECK(mac_pack_compatible(&older, &public_newer) == false, "public address is never compatible (newer side)");

    ble_device_t different_addr_newer = newer;
    different_addr_newer.address_type = ADDRESS_TYPE_UNKNOWN;
    CHECK(mac_pack_compatible(&older, &different_addr_newer) == true,
          "unknown address type on either side does not block a match");

    ble_device_t unknown_category_older = older;
    unknown_category_older.category = CATEGORY_UNKNOWN;
    CHECK(mac_pack_compatible(&unknown_category_older, &newer) == false,
          "older device with unknown category is never compatible (matches original asymmetric behavior)");

    ble_device_t different_category_newer = newer;
    different_category_newer.category = CATEGORY_TABLET;
    CHECK(mac_pack_compatible(&older, &different_category_newer) == false,
          "differing known categories are incompatible");

    ble_device_t conflicting_name_newer = newer;
    strncpy(conflicting_name_newer.name, "Someone Else's Pixel", MAX_NAME_LENGTH - 1);
    CHECK(mac_pack_compatible(&older, &conflicting_name_newer) == false,
          "conflicting non-empty names are incompatible");

    ble_device_t empty_name_newer = newer;
    empty_name_newer.name[0] = '\0';
    CHECK(mac_pack_compatible(&older, &empty_name_newer) == true,
          "an empty name on either side does not block a match");

    // Heuristic-derived names (anything below NAME_CONF_KNOWN) are inherently
    // volatile - e.g. Apple Continuity guesses differ by message type and
    // even lock/screen state for the same physical phone - so a mismatch
    // there must NOT block a match the way a real advertised name does.
    ble_device_t heuristic_older = make_device(3, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "iPhone di=01f");
    heuristic_older.name_confidence = NAME_CONF_MANUFACTURER;
    ble_device_t heuristic_newer = make_device(4, 105, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "Apple di=11f");
    heuristic_newer.name_confidence = NAME_CONF_MANUFACTURER;
    CHECK(mac_pack_compatible(&heuristic_older, &heuristic_newer) == true,
          "conflicting heuristic-confidence names do not block a match");

    // The name check only fires when BOTH sides are real known names - a
    // known name on one side and a mere heuristic guess on the other is
    // not a conflict either (the guess isn't trustworthy enough to compare).
    ble_device_t known_vs_guess_older = heuristic_older;
    known_vs_guess_older.name_confidence = NAME_CONF_KNOWN;
    strncpy(known_vs_guess_older.name, "Ian's iPhone", MAX_NAME_LENGTH - 1);
    CHECK(mac_pack_compatible(&known_vs_guess_older, &heuristic_newer) == true,
          "a known name vs. a lower-confidence guess is not treated as a conflict");
}

static void test_probability(void) {
    ble_device_t older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "phone");
    ble_device_t near = make_device(2, 105, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "phone");
    ble_device_t far = make_device(3, 100 + (time_t)(MAC_PACK_TIME_CONSTANT_SEC * 10), 500, 5,
                                    ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "phone");

    float near_prob = mac_pack_probability(&older, &near);
    float far_prob = mac_pack_probability(&older, &far);

    CHECK(near_prob > 0.8f, "a short gap yields high confidence");
    CHECK(far_prob < 0.01f, "a very long gap yields near-zero confidence");
    CHECK(near_prob > far_prob, "a shorter gap is always more confident than a longer one");

    // A small negative gap (older's last sighting falls a moment after
    // newer's first) is normal scan-cycle timing jitter around a genuine
    // near-simultaneous handover and must still score high.
    ble_device_t barely_over = make_device(4, 0, 106, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "phone");
    float jitter_prob = mac_pack_probability(&barely_over, &near); // gap = 105 - 106 = -1s
    CHECK(jitter_prob > 0.9f, "a small negative gap still scores high (scan-cycle jitter)");

    // A large negative gap (older still being actively, concurrently seen
    // long after newer's first sighting) must decay toward zero just like a
    // large *positive* gap does - it's not a rotation, it's two live
    // identities. Before this was fixed, any negative gap was clamped
    // straight to a "perfect" 1.0 match.
    ble_device_t still_live = make_device(5, 0, 300, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "phone");
    float stolen_prob = mac_pack_probability(&still_live, &near); // gap = 105 - 300 = -195s
    CHECK(stolen_prob < 0.05f,
          "a large negative gap (older still concurrently live) scores near-zero, not a perfect match");
}

static void test_summary_excludes_blips(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // A real, repeatedly-seen phone.
    list.devices[0] = make_device(1, 0, 50, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    // A one-off blip: a single stray packet, never seen again - weak
    // evidence of a real device, must not inflate the phone count.
    list.devices[1] = make_device(2, 20, 20, 1, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");

    device_summary_t summary;
    device_get_summary(&list, &summary);

    CHECK(summary.total_devices == 1, "a one-off single-observation blip is excluded from the summary");
    CHECK(summary.phones == 1, "the blip is excluded from its category count too");

    // Once it's seen a second time, it counts.
    list.devices[1].seen_count = 2;
    device_get_summary(&list, &summary);
    CHECK(summary.total_devices == 2, "a device seen twice is no longer treated as a blip");
    CHECK(summary.phones == 2, "the now-confirmed device counts toward its category");
}

static void test_summary_excludes_weak_rssi(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // A real phone with a solid signal.
    list.devices[0] = make_device(1, 0, 50, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[0].raw_rssi = -95;

    // Seen multiple times (so it's not caught by the blip filter instead),
    // but at a signal weaker than MIN_RSSI_FOR_SUMMARY - near/below typical
    // BLE receiver sensitivity, where a misdecoded MAC can masquerade as a
    // spurious "device".
    list.devices[1] = make_device(2, 0, 50, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[1].raw_rssi = -105;

    device_summary_t summary;
    device_get_summary(&list, &summary);

    CHECK(summary.total_devices == 1, "a device with RSSI weaker than MIN_RSSI_FOR_SUMMARY is excluded");
    CHECK(summary.phones == 1, "the weak-signal device is excluded from its category count too");

    // Once its signal improves (e.g. it moves closer), it counts.
    list.devices[1].raw_rssi = -90;
    device_get_summary(&list, &summary);
    CHECK(summary.total_devices == 2, "a device with an acceptable RSSI is no longer excluded");
}

static void test_run_links_and_dedupes(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Same physical phone: MAC 1 active 0-100, then rotates to MAC 2 at 105-200.
    list.devices[0] = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[1] = make_device(2, 105, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");

    // Unrelated device, active the whole time (overlaps everything): must never be linked.
    list.devices[2] = make_device(3, 0, 200, 20, ADDRESS_TYPE_RANDOM, CATEGORY_TABLET, "");

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == true, "older MAC gets linked to its rotated successor");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[1].mac, 6) == 0,
          "the link points at the correct newer MAC");
    CHECK(list.devices[1].has_superseded_by == false, "the newer MAC is not itself marked superseded");
    CHECK(list.devices[2].has_superseded_by == false, "an overlapping unrelated device is never linked");

    device_summary_t summary;
    device_get_summary(&list, &summary);
    CHECK(summary.total_devices == 2, "deduped summary counts the rotated pair once");
}

static void test_run_chain_of_rotations(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Same physical device rotating MAC twice in a row: A0 -> A1 -> A2.
    list.devices[0] = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[1] = make_device(2, 105, 150, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[2] = make_device(3, 155, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == true, "first rotation in a chain is linked");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[1].mac, 6) == 0,
          "first device links to the second, not the third");
    CHECK(list.devices[1].has_superseded_by == true, "second rotation in a chain is linked");
    CHECK(memcmp(list.devices[1].superseded_by, list.devices[2].mac, 6) == 0,
          "second device links to the third");
    CHECK(list.devices[2].has_superseded_by == false, "the newest device in the chain has no successor");
}

static void test_run_older_device_claimed_at_most_once(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // A0 is a valid, non-overlapping predecessor for both A1 and A2, but A1
    // and A2 overlap each other, so A1 is not eligible as A2's predecessor.
    // A2 (processed first, being newest) claims A0. A1 must then find no
    // available predecessor, even though it individually matches A0 fine.
    list.devices[0] = make_device(1, 0, 50, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[1] = make_device(2, 60, 140, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    list.devices[2] = make_device(3, 130, 300, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");

    mac_pack_run(&list);

    CHECK(mac_pack_overlaps(60, 140, 130, 300) == true, "test setup: device 1 and 2 do overlap each other");
    CHECK(list.devices[0].has_superseded_by == true, "the oldest device is claimed by exactly one successor");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[2].mac, 6) == 0,
          "the newest device claims it first, since devices are processed newest-first");
    CHECK(list.devices[1].has_superseded_by == false,
          "a device that overlaps the claimant cannot also claim the same predecessor");
}

static void test_windows_conflict_fallback_no_data(void) {
    // Neither device has any recorded stream data (e.g. non-Apple devices,
    // or ENABLE_APPLE_HEURISTICS disabled): must behave exactly like the
    // plain aggregate-window overlap check.
    ble_device_t overlapping_older = make_device(1, 0, 10, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    ble_device_t overlapping_newer = make_device(2, 5, 15, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    CHECK(mac_pack_windows_conflict(&overlapping_older, &overlapping_newer) == true,
          "no stream data + overlapping aggregate windows falls back to a conflict");

    ble_device_t disjoint_older = make_device(1, 0, 10, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    ble_device_t disjoint_newer = make_device(2, 20, 30, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    CHECK(mac_pack_windows_conflict(&disjoint_older, &disjoint_newer) == false,
          "no stream data + disjoint aggregate windows falls back to no conflict");
}

static void test_windows_conflict_fallback_asymmetric_data(void) {
    // Only one side has recorded stream data: still not enough to compare
    // stream-by-stream, so fall back to the aggregate check.
    ble_device_t older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&older, 0x10, 0, 100);
    ble_device_t newer = make_device(2, 95, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");

    CHECK(mac_pack_windows_conflict(&older, &newer) == true,
          "stream data on only one side falls back to the aggregate check (overlapping)");
}

static void test_windows_conflict_ragged_different_types_no_conflict(void) {
    // Same physical phone: MAC A's Nearby Info stream (0x10) is still
    // trailing off while MAC B's Instant Hotspot stream (0x0e) has already
    // started - the aggregate windows overlap (95-100), but no single
    // stream type is live on both MACs at once, so this must NOT be treated
    // as a conflict.
    ble_device_t older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&older, 0x10, 0, 100);
    ble_device_t newer = make_device(2, 95, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&newer, 0x0e, 95, 200);

    CHECK(mac_pack_windows_conflict(&older, &newer) == false,
          "overlapping aggregate windows on different stream types is a ragged handover, not a conflict");
}

static void test_windows_conflict_same_type_genuine_overlap(void) {
    // Two distinct physical devices both broadcasting Nearby Info (0x10) at
    // the same time: a real conflict, must not be linked.
    ble_device_t older = make_device(1, 0, 150, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&older, 0x10, 0, 150);
    ble_device_t newer = make_device(2, 50, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&newer, 0x10, 50, 200);

    CHECK(mac_pack_windows_conflict(&older, &newer) == true,
          "overlapping windows on the same stream type is a genuine conflict");
}

static void test_windows_conflict_only_shared_types_compared(void) {
    // older has types 0x10 (0-50) and 0x0e (90-100); newer has 0x10
    // (60-100, touches older's 0x10 window but doesn't overlap it) and 0x0f
    // (95-150, unshared with older). Nothing shared actually overlaps, so
    // this must not be a conflict even though older's 0x0e and newer's 0x0f
    // windows overlap - they're different, unshared types.
    ble_device_t older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&older, 0x10, 0, 50);
    set_stream(&older, 0x0e, 90, 100);
    ble_device_t newer = make_device(2, 60, 150, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&newer, 0x10, 60, 100);
    set_stream(&newer, 0x0f, 95, 150);

    CHECK(mac_pack_windows_conflict(&older, &newer) == false,
          "only shared stream types are compared; unshared-type overlap is irrelevant");
}

static void test_run_ragged_cross_type_overlap_links(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Same physical phone, ragged handover: aggregate windows overlap
    // (95-100), but the overlap is explained by two different Continuity
    // stream types, not a real collision.
    list.devices[0] = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[0], 0x10, 0, 100);
    list.devices[1] = make_device(2, 95, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[1], 0x0e, 95, 200);

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == true,
          "a ragged cross-type overlap still links as a MAC rotation");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[1].mac, 6) == 0,
          "the ragged-handover link points at the correct newer MAC");
}

static void test_run_stale_but_live_candidate_does_not_steal_slot(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Reproduces a real observed bug, with timestamps taken directly from a
    // real device summary log ("window: <first_seen>s-<last_seen>s ago",
    // converted to forward time from an arbitrary t=0): a genuine MAC
    // rotation (device 1 -> 2, sharing stream type 0x0c, device 1 stopping
    // 1s before device 2 starts) competing for the same successor's single
    // predecessor slot against a device (3) that is unrelated and still
    // actively/concurrently live - sharing no stream type with either, so
    // mac_pack_windows_conflict() doesn't rule it out on its own. Before
    // mac_pack_probability() decayed symmetrically on gap magnitude, device
    // 3's large negative gap (still being seen long after device 2 already
    // started) was clamped to a "perfect" 1.0 score and could out-compete -
    // and steal the slot from - the genuine, correctly-timed rotation.
    list.devices[0] = make_device(1, 0, 453, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[0], 0x0c, 0, 453);
    list.devices[1] = make_device(2, 454, 748, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[1], 0x0c, 454, 748);
    list.devices[2] = make_device(3, 25, 737, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[2], 0x10, 25, 737);

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == true,
          "the genuine, correctly-timed rotation still links");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[1].mac, 6) == 0,
          "the genuine rotation links to its actual successor, not stolen by the stale-but-live device");
    CHECK(list.devices[2].has_superseded_by == false,
          "the stale-but-live device (still active long after the successor appeared) is not linked to anything");
}

static void test_run_same_type_concurrent_devices_not_linked(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Two distinct physical phones both broadcasting Nearby Info (0x10) with
    // genuinely overlapping windows: must not be linked, even though they'd
    // otherwise be compatible (same category/address type, no name conflict).
    list.devices[0] = make_device(1, 0, 150, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[0], 0x10, 0, 150);
    list.devices[1] = make_device(2, 50, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&list.devices[1], 0x10, 50, 200);

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == false,
          "genuinely concurrent same-type broadcasts from two devices are never linked");
}

static void test_payload_matches(void) {
    const uint8_t handoff_a[] = {0x0c, 0x0e, 0x08, 0xce, 0x98, 0xb3, 0x22, 0xc4, 0xb6, 0x24, 0xd5, 0x47, 0x0a, 0x27, 0x7f};
    const uint8_t handoff_b[] = {0x0c, 0x0e, 0x08, 0xcf, 0x98, 0xe3, 0x07, 0x61, 0xa6, 0xbc, 0x76, 0x46, 0x96, 0x4a, 0x5f};

    ble_device_t older, newer;

    // Identical payload on a shared type: matches.
    older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&older, 0x0c, 0, 100, handoff_a, sizeof(handoff_a));
    newer = make_device(2, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&newer, 0x0c, 0, 100, handoff_a, sizeof(handoff_a));
    CHECK(mac_pack_payload_matches(&older, &newer) == true,
          "identical payload content on a shared stream type matches");

    // Different payload (a real different Handoff session) on the same type: no match.
    newer = make_device(2, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&newer, 0x0c, 0, 100, handoff_b, sizeof(handoff_b));
    CHECK(mac_pack_payload_matches(&older, &newer) == false,
          "different payload content on a shared stream type does not match");

    // Identical payload but on *different* types: not comparable, no match.
    newer = make_device(2, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&newer, 0x10, 0, 100, handoff_a, sizeof(handoff_a));
    CHECK(mac_pack_payload_matches(&older, &newer) == false,
          "identical bytes on a different stream type does not count as a match");

    // Identical but too short (below MAC_PACK_MIN_PAYLOAD_MATCH_LEN): no match -
    // not enough entropy to trust as decisive evidence.
    const uint8_t short_payload[] = {0x10, 0x19};
    older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&older, 0x10, 0, 100, short_payload, sizeof(short_payload));
    newer = make_device(2, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&newer, 0x10, 0, 100, short_payload, sizeof(short_payload));
    CHECK(mac_pack_payload_matches(&older, &newer) == false,
          "an identical but too-short payload is not trusted as a match");

    // No payload recorded on either side: no match (falls back to the
    // ordinary windows-conflict path).
    older = make_device(1, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&older, 0x0c, 0, 100);
    newer = make_device(2, 0, 100, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream(&newer, 0x0c, 0, 100);
    CHECK(mac_pack_payload_matches(&older, &newer) == false,
          "no recorded payload on either side never matches");
}

static void test_run_concurrent_devices_with_matching_payload_are_linked(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Reproduces a real observed case: two MACs broadcasting Handoff (0x0c)
    // with genuinely overlapping windows (like
    // test_run_same_type_concurrent_devices_not_linked above) - except this
    // time their raw payload content is byte-identical, which is decisive
    // evidence they're one phone maintaining two concurrent BLE identities
    // for the same stream, not two different phones. Must link despite the
    // overlap that would otherwise disqualify the pair.
    const uint8_t handoff_payload[] = {0x0c, 0x0e, 0x08, 0xce, 0x98, 0xb3, 0x22, 0xc4, 0xb6, 0x24, 0xd5, 0x47, 0x0a, 0x27, 0x7f};

    list.devices[0] = make_device(1, 0, 150, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&list.devices[0], 0x0c, 0, 150, handoff_payload, sizeof(handoff_payload));
    list.devices[1] = make_device(2, 50, 200, 5, ADDRESS_TYPE_RANDOM, CATEGORY_PHONE, "");
    set_stream_with_payload(&list.devices[1], 0x0c, 50, 200, handoff_payload, sizeof(handoff_payload));

    mac_pack_run(&list);

    CHECK(list.devices[0].has_superseded_by == true,
          "concurrent devices with matching payload content are linked despite overlapping windows");
    CHECK(memcmp(list.devices[0].superseded_by, list.devices[1].mac, 6) == 0,
          "the link points at the correct newer MAC");
    CHECK(list.devices[0].superseded_probability == 1.0f,
          "a payload match is recorded at full confidence, not the gap-based probability");

    device_summary_t summary;
    device_get_summary(&list, &summary);
    CHECK(summary.total_devices == 1, "the pair is deduped down to a single counted device");
}

static void test_run_tied_first_seen_matching_payload_links_one_direction_only(void) {
    device_list_t list;
    memset(&list, 0, sizeof(list));

    // Reproduces a real observed bug: two concurrent identities of the same
    // device (e.g. an iPad broadcasting on two BLE MACs at once) that were
    // BOTH already active when tracking started share the exact same
    // first_seen - unlike a normal sequential MAC rotation, where the two
    // timestamps always differ. With a payload match making them mutually
    // compatible regardless of direction, and the old strict '>' check
    // letting equal first_seen values through both ways, each device could
    // claim the other as its own predecessor: both ended up has_superseded_by
    // = true, mutually pointing at each other, and both vanished from every
    // summary count (Total/Tablets/etc. all read 0 despite two real tracked
    // devices existing).
    const uint8_t nearby_info_payload[] = {0x10, 0x06, 0x01, 0x1d, 0x35, 0x04, 0xbe, 0x28};

    list.devices[0] = make_device(1, 500, 900, 5, ADDRESS_TYPE_RANDOM, CATEGORY_TABLET, "");
    set_stream_with_payload(&list.devices[0], 0x10, 500, 900, nearby_info_payload, sizeof(nearby_info_payload));
    list.devices[1] = make_device(2, 500, 900, 5, ADDRESS_TYPE_RANDOM, CATEGORY_TABLET, "");
    set_stream_with_payload(&list.devices[1], 0x10, 500, 900, nearby_info_payload, sizeof(nearby_info_payload));

    mac_pack_run(&list);

    bool both_superseded = list.devices[0].has_superseded_by && list.devices[1].has_superseded_by;
    CHECK(!both_superseded, "tied-first_seen devices never mutually supersede each other");

    device_summary_t summary;
    device_get_summary(&list, &summary);
    CHECK(summary.total_devices == 1, "the tied pair is deduped down to exactly one counted device, not zero");
    CHECK(summary.tablets == 1, "the surviving device still counts toward its category");
}

static void test_device_record_stream_updates_existing(void) {
    ble_device_t d;
    memset(&d, 0, sizeof(d));

    device_record_stream(&d, 0x10, 10, NULL, 0);
    device_record_stream(&d, 0x10, 50, NULL, 0);

    CHECK(d.stream_count == 1, "recording the same type twice does not grow the table");
    CHECK(d.streams[0].type == 0x10, "the tracked slot has the right type");
    CHECK(d.streams[0].first_seen == 10, "first_seen is set on first sight and left alone after");
    CHECK(d.streams[0].last_seen == 50, "last_seen is bumped on a repeat sighting");
}

static void test_device_record_stream_eviction(void) {
    ble_device_t d;
    memset(&d, 0, sizeof(d));

    // Fill the table (MAC_PACK_MAX_TRACKED_STREAMS == 4) with distinct types
    // at increasing timestamps, oldest last-seen first.
    device_record_stream(&d, 0x10, 10, NULL, 0);
    device_record_stream(&d, 0x0e, 20, NULL, 0);
    device_record_stream(&d, 0x0f, 30, NULL, 0);
    device_record_stream(&d, 0x0c, 40, NULL, 0);
    CHECK(d.stream_count == MAC_PACK_MAX_TRACKED_STREAMS, "table is full after 4 distinct types");

    // A 5th distinct type must evict the least-recently-seen slot (0x10 @ 10).
    device_record_stream(&d, 0x07, 50, NULL, 0);
    CHECK(d.stream_count == MAC_PACK_MAX_TRACKED_STREAMS, "stream_count stays at capacity after eviction");

    bool found_evicted_type = false;
    bool found_new_type = false;
    for (int i = 0; i < d.stream_count; i++) {
        if (d.streams[i].type == 0x10) found_evicted_type = true;
        if (d.streams[i].type == 0x07) found_new_type = true;
    }
    CHECK(found_evicted_type == false, "the least-recently-seen type (0x10) was evicted");
    CHECK(found_new_type == true, "the new type (0x07) took its place");
}

int main(void) {
    test_overlap();
    test_blip();
    test_compatible();
    test_probability();
    test_summary_excludes_blips();
    test_summary_excludes_weak_rssi();
    test_run_links_and_dedupes();
    test_run_chain_of_rotations();
    test_run_older_device_claimed_at_most_once();
    test_windows_conflict_fallback_no_data();
    test_windows_conflict_fallback_asymmetric_data();
    test_windows_conflict_ragged_different_types_no_conflict();
    test_windows_conflict_same_type_genuine_overlap();
    test_windows_conflict_only_shared_types_compared();
    test_run_ragged_cross_type_overlap_links();
    test_run_stale_but_live_candidate_does_not_steal_slot();
    test_run_same_type_concurrent_devices_not_linked();
    test_payload_matches();
    test_run_concurrent_devices_with_matching_payload_are_linked();
    test_run_tied_first_seen_matching_payload_links_one_direction_only();
    test_device_record_stream_updates_existing();
    test_device_record_stream_eviction();

    if (failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    }
    printf("\n%d test(s) failed.\n", failures);
    return 1;
}
