/*
 * Copyright 2021 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: AMD
 *
 */

/* FILE POLICY AND INTENDED USAGE:
 *
 * This file implements basic dpcd read/write functionality. It also does basic
 * dpcd range check to ensure that every dpcd request is compliant with specs
 * range requirements.
 */

#include "link_dpcd.h"
#include "link_ddc.h"
#include <drm/display/drm_dp_helper.h>
#include "dm_helpers.h"

#define END_ADDRESS(start, size) (start + size - 1)
#define ADDRESS_RANGE_SIZE(start, end) (end - start + 1)
struct dpcd_address_range {
	uint32_t start;
	uint32_t end;
};

static enum dc_status internal_link_read_dpcd(
	struct dc_link *link,
	uint32_t address,
	uint8_t *data,
	uint32_t size)
{
	if (!link->aux_access_disabled &&
			!dm_helpers_dp_read_dpcd(link->ctx,
			link, address, data, size)) {
		return DC_ERROR_UNEXPECTED;
	}

	return DC_OK;
}

static enum dc_status internal_link_write_dpcd(
	struct dc_link *link,
	uint32_t address,
	const uint8_t *data,
	uint32_t size)
{
	if (!link->aux_access_disabled &&
			!dm_helpers_dp_write_dpcd(link->ctx,
			link, address, data, size)) {
		return DC_ERROR_UNEXPECTED;
	}

	return DC_OK;
}

/*
 * Partition the entire DPCD address space
 * XXX: This partitioning must cover the entire DPCD address space,
 * and must contain no gaps or overlapping address ranges.
 */
static const struct dpcd_address_range mandatory_dpcd_partitions[] = {
	{ 0, DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR1) - 1},
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR1), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR2) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR2), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR3) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR3), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR4) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR4), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR5) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR5), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR6) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR6), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR7) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR7), DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR8) - 1 },
	{ DP_TRAINING_PATTERN_SET_PHY_REPEATER(DP_PHY_LTTPR8), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR1) - 1 },
	/*
	 * The FEC registers are contiguous
	 */
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR1), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR1) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR2), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR2) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR3), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR3) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR4), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR4) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR5), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR5) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR6), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR6) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR7), DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR7) - 1 },
	{ DP_FEC_STATUS_PHY_REPEATER(DP_PHY_LTTPR8), DP_LTTPR_MAX_ADD },
	/* all remaining DPCD addresses */
	{ DP_LTTPR_MAX_ADD + 1, DP_DPCD_MAX_ADD } };

static inline bool do_addresses_intersect_with_range(
		const struct dpcd_address_range *range,
		const uint32_t start_address,
		const uint32_t end_address)
{
	return start_address <= range->end && end_address >= range->start;
}

static uint32_t dpcd_get_next_partition_size(const uint32_t address, const uint32_t size)
{
	const uint32_t end_address = END_ADDRESS(address, size);
	uint32_t partition_iterator = 0;

	/*
	 * find current partition
	 * this loop spins forever if partition map above is not surjective
	 */
	while (!do_addresses_intersect_with_range(&mandatory_dpcd_partitions[partition_iterator],
				address, end_address))
		partition_iterator++;
	if (end_address < mandatory_dpcd_partitions[partition_iterator].end)
		return size;
	return ADDRESS_RANGE_SIZE(address, mandatory_dpcd_partitions[partition_iterator].end);
}

/*
 * Ranges of DPCD addresses that must be read in a single transaction
 * XXX: Do not allow any two address ranges in this array to overlap
 */
static const struct dpcd_address_range mandatory_dpcd_blocks[] = {
	{ DP_LT_TUNABLE_PHY_REPEATER_FIELD_DATA_STRUCTURE_REV, DP_PHY_REPEATER_128B132B_RATES }};

/*
 * extend addresses to read all mandatory blocks together
 */
static void dpcd_extend_address_range(
		const uint32_t in_address,
		uint8_t * const in_data,
		const uint32_t in_size,
		uint32_t *out_address,
		uint8_t **out_data,
		uint32_t *out_size)
{
	const uint32_t end_address = END_ADDRESS(in_address, in_size);
	const struct dpcd_address_range *addr_range;
	struct dpcd_address_range new_addr_range;
	uint32_t i;

	new_addr_range.start = in_address;
	new_addr_range.end = end_address;
	for (i = 0; i < ARRAY_SIZE(mandatory_dpcd_blocks); i++) {
		addr_range = &mandatory_dpcd_blocks[i];
		if (addr_range->start <= in_address && addr_range->end >= in_address)
			new_addr_range.start = addr_range->start;

		if (addr_range->start <= end_address && addr_range->end >= end_address)
			new_addr_range.end = addr_range->end;
	}
	*out_address = in_address;
	*out_size = in_size;
	*out_data = in_data;
	if (new_addr_range.start != in_address || new_addr_range.end != end_address) {
		*out_address = new_addr_range.start;
		*out_size = ADDRESS_RANGE_SIZE(new_addr_range.start, new_addr_range.end);
		*out_data = kcalloc(*out_size, sizeof(**out_data), GFP_KERNEL);
		ASSERT(*out_data);
	}
}

/*
 * Reduce the AUX reply down to the values the caller requested
 */
static void dpcd_reduce_address_range(
		const uint32_t extended_address,
		uint8_t * const extended_data,
		const uint32_t extended_size,
		const uint32_t reduced_address,
		uint8_t * const reduced_data,
		const uint32_t reduced_size)
{
	const uint32_t offset = reduced_address - extended_address;

	/*
	 * If the address is same, address was not extended.
	 * So we do not need to free any memory.
	 * The data is in original buffer(reduced_data).
	 */
	if (extended_data == reduced_data)
		return;

	memcpy(&extended_data[offset], reduced_data, reduced_size);
	kfree(extended_data);
}

enum dc_status core_link_read_dpcd(
	struct dc_link *link,
	uint32_t address,
	uint8_t *data,
	uint32_t size)
{
	uint32_t extended_address;
	uint32_t partitioned_address;
	uint8_t *extended_data;
	uint32_t extended_size;
	/* size of the remaining partitioned address space */
	uint32_t size_left_to_read;
	enum dc_status status = DC_ERROR_UNEXPECTED;
	/* size of the next partition to be read from */
	uint32_t partition_size;
	uint32_t data_index = 0;

	dpcd_extend_address_range(address, data, size, &extended_address, &extended_data, &extended_size);
	partitioned_address = extended_address;
	size_left_to_read = extended_size;
	while (size_left_to_read) {
		partition_size = dpcd_get_next_partition_size(partitioned_address, size_left_to_read);
		status = internal_link_read_dpcd(link, partitioned_address, &extended_data[data_index], partition_size);
		if (status != DC_OK)
			break;
		partitioned_address += partition_size;
		data_index += partition_size;
		size_left_to_read -= partition_size;
	}
	dpcd_reduce_address_range(extended_address, extended_data, extended_size, address, data, size);
	return status;
}

enum dc_status core_link_write_dpcd(
	struct dc_link *link,
	uint32_t address,
	const uint8_t *data,
	uint32_t size)
{
	uint32_t partition_size;
	uint32_t data_index = 0;
	enum dc_status status = DC_ERROR_UNEXPECTED;

	while (size) {
		partition_size = dpcd_get_next_partition_size(address, size);
		status = internal_link_write_dpcd(link, address, &data[data_index], partition_size);
		if (status != DC_OK)
			break;
		address += partition_size;
		data_index += partition_size;
		size -= partition_size;
	}
	return status;
}

/* Apple 5K dual-tile internal panel: DPCD 0x4F1 = root panel-latch (1 =
 * present the TILED identity / arm for the combined dual-tile enable; 0 =
 * reset to the BASE presentation -- in native mode a 0-write live-drops the
 * panel to compat), 0x425 = root panel mode STATUS (bit1 set = compat, clear
 * = native). */
#define APPLE_5K_DPCD_ROOT_PANEL_LATCH 0x4F1
#define APPLE_5K_DPCD_ROOT_PANEL_MODE_STATUS 0x425
#define APPLE_5K_DPCD_ROOT_PANEL_STATUS_BLOCK 0x420
#define APPLE_5K_DPCD_ROOT_PANEL_MODE_MARKER 0x41C

#define DC_LOGGER \
	dc_logger
#define DC_LOGGER_INIT(logger) \
	struct dal_logger *dc_logger = logger

bool link_apple_5k_sample_panel_state(struct dc_link *root_link,
				      const char *stage,
				      struct apple5k_panel_state *state)
{
	struct apple5k_panel_state st = { 0 };
	enum dc_status status;
	DC_LOGGER_INIT(root_link->ctx->logger);

	if (state)
		*state = st;
	if (!dc_link_has_tiled_root_panel_patch(root_link))
		return false;

	status = core_link_read_dpcd(root_link,
				     APPLE_5K_DPCD_ROOT_PANEL_STATUS_BLOCK,
				     st.block, sizeof(st.block));
	if (status != DC_OK) {
		if (stage)
			DC_LOG_INFO("APPLE5K: panel mode (%s) link[%u] AUX READ FAILED (status=%d)\n",
				    stage, root_link->link_index, status);
		return false;
	}
	core_link_read_dpcd(root_link, APPLE_5K_DPCD_ROOT_PANEL_MODE_MARKER,
			    &st.marker, sizeof(st.marker));
	core_link_read_dpcd(root_link, APPLE_5K_DPCD_ROOT_PANEL_LATCH,
			    &st.latch, sizeof(st.latch));

	st.valid = true;
	st.native = !(st.block[5] & 0x02);
	st.fault = (st.block[3] & 0x04) || (st.block[4] & 0x04);

	if (stage)
		DC_LOG_INFO("APPLE5K: panel mode (%s) link[%u] 0x425=0x%02x 0x41C=0x%02x 0x4F1=0x%02x 0x423=0x%02x 0x424=0x%02x block=%8ph -> %s%s (native_boot was %d)\n",
			    stage, root_link->link_index,
			    st.block[5], st.marker, st.latch,
			    st.block[3], st.block[4], st.block,
			    st.native ? "NATIVE" : "compat",
			    st.fault ? " [TCON-FAULT]" : "",
			    root_link->apple5k_native_boot);

	if (state)
		*state = st;
	return true;
}

/*
 * The EFI ComplexDisplayInit arm handshake, RE'd instruction-level from
 * CoreEG2 fcn.00017cf1 (J137 firmware): the panel presents its BASE identity
 * (AE1D, EDID product LSB & 3 == 1) or its TILED identity (AE1E, LSB & 3 ==
 * 2); 0x4F1 selects which. Each write is followed by a 10 ms stall, exactly
 * like the firmware:
 *
 *   phase 1: WHILE the EDID reads tiled, write 0x4F1=0 + re-read -- drive
 *            the panel to a clean base state (resets any stale armed state);
 *   phase 2: write 0x4F1=1 + re-read, REQUIRE the tiled identity;
 *   failure: write 0x4F1=0 -- the firmware's abort paths always disarm.
 *            A latch left at 1 without the combined enable is the hard wedge
 *            that survives warm reboot and blocks the EFI native restore.
 *
 * All four firmware write sites run with the panel powered and AUX alive,
 * before any video enable; ComplexDisplayInit touches NO other panel-private
 * register (0x41C/0x41F/0x425 are panel-maintained status).
 */
enum dc_status link_apple_5k_arm_handshake(struct dc_link *root_link)
{
	struct apple5k_panel_state st;
	uint8_t latch;
	uint8_t hdr[16] = { 0 };
	uint8_t offset = 0;
	int try;
	DC_LOGGER_INIT(root_link->ctx->logger);

	if (!dc_link_has_tiled_root_panel_patch(root_link) || !root_link->ddc)
		return DC_ERROR_UNEXPECTED;

	/*
	 * Wedge detector: if the TCON fault flags (0x423/0x424 bit2) are
	 * already latched, the TCON is jammed from an earlier failed arm --
	 * it refuses latch writes and only a cold power-off clears it.
	 * Arming a faulted TCON is pointless and may deepen the wedge.
	 */
	if (link_apple_5k_sample_panel_state(root_link, "arm-entry", &st) &&
	    st.fault) {
		DC_LOG_WARNING("APPLE5K: TCON fault latched (0x423=0x%02x 0x424=0x%02x) -- REFUSING to arm; cold power-off required to clear\n",
			       st.block[3], st.block[4]);
		root_link->apple5k_armed = false;
		root_link->apple5k_arming = false;
		return DC_ERROR_UNEXPECTED;
	}

	for (try = 0; try < 3; try++) {
		if (!link_query_ddc_data(root_link->ddc, 0x50, &offset, 1,
					 hdr, sizeof(hdr)))
			break;
		if ((hdr[0xa] & 3) != 2)
			break; /* base presentation: clean state */
		latch = 0;
		core_link_write_dpcd(root_link,
				     APPLE_5K_DPCD_ROOT_PANEL_LATCH,
				     &latch, sizeof(latch));
		msleep(10);
	}

	latch = 1;
	core_link_write_dpcd(root_link, APPLE_5K_DPCD_ROOT_PANEL_LATCH,
			     &latch, sizeof(latch));
	msleep(10);

	if (link_query_ddc_data(root_link->ddc, 0x50, &offset, 1,
				hdr, sizeof(hdr)) &&
	    (hdr[0xa] & 3) == 2) {
		/*
		 * Armed: the panel presents the tiled EDID. Mark arm-in-
		 * progress so the eDP VDD is held on through to the combined
		 * enable (a power-off while armed wedges the TCON).
		 */
		root_link->apple5k_armed = true;
		root_link->apple5k_arming = true;
		link_apple_5k_sample_panel_state(root_link, "arm-exit", NULL);
		return DC_OK;
	}

	/* The arm did not take: disarm rather than leave a wedge-armed latch. */
	latch = 0;
	core_link_write_dpcd(root_link, APPLE_5K_DPCD_ROOT_PANEL_LATCH,
			     &latch, sizeof(latch));
	root_link->apple5k_armed = false;
	root_link->apple5k_arming = false;
	link_apple_5k_sample_panel_state(root_link, "arm-FAILED-disarmed", NULL);
	return DC_ERROR_UNEXPECTED;
}

enum dc_status link_apple_5k_root_panel_latch_pulse(struct dc_link *root_link)
{
	uint8_t payload = 1;

	if (!dc_link_has_tiled_root_panel_patch(root_link))
		return DC_OK;

	/*
	 * iMac Pro only: this is the earliest tiled-root AUX access at boot, before
	 * amdgpu's modeset touches the panel, so sample the EFI-handed-off panel
	 * mode (DPCD 0x425) ONCE here -- apple5k_native_boot then gates every
	 * preservation path (dc_link_apple5k_preserve). On a native boot the
	 * latch is NEVER touched. On a compat boot, run the firmware's verified
	 * arm handshake ONCE: it makes the root re-present the tiled EDID for
	 * detection, and the 1-write is also what exposes the slave tile (the
	 * firmware discovers tile 1 right after its arm). The pre-training
	 * check in enable_link_dp() re-verifies/re-arms if the boot teardown
	 * dropped it. Every other iMac model keeps the unconditional latch
	 * write below to wake its slave tile for detection.
	 */
	if (root_link->apple5k_imac_pro) {
		if (!root_link->apple5k_native_sampled) {
			uint8_t mode = 0;

			core_link_read_dpcd(root_link,
					    APPLE_5K_DPCD_ROOT_PANEL_MODE_STATUS,
					    &mode, sizeof(mode));
			root_link->apple5k_native_boot = !(mode & 0x02);
			root_link->apple5k_native_sampled = true;
		}
		/*
		 * The compat->native arm is DANGEROUS (a failed arm wedges the
		 * panel until cold power-off) and opt-in. When disabled, never
		 * touch the latch on a compat boot -- the panel is presented at
		 * its native per-tile size instead.
		 */
		if (root_link->apple5k_native_boot ||
		    root_link->apple5k_armed ||
		    !root_link->apple5k_compat_arm_enable)
			return DC_OK;
		root_link->apple5k_armed = true;
		return link_apple_5k_arm_handshake(root_link);
	}

	return core_link_write_dpcd(root_link, APPLE_5K_DPCD_ROOT_PANEL_LATCH,
				    &payload, sizeof(payload));
}
