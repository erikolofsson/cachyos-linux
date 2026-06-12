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

#ifndef __LINK_DPCD_H__
#define __LINK_DPCD_H__
#include "link_service.h"
#include "dpcd_defs.h"

enum dc_status core_link_read_dpcd(
		struct dc_link *link,
		uint32_t address,
		uint8_t *data,
		uint32_t size);

enum dc_status core_link_write_dpcd(
		struct dc_link *link,
		uint32_t address,
		const uint8_t *data,
		uint32_t size);

/*
 * Pulse the Apple 5K root panel-latch DPCD (0x4F1 = 1). Used by the slave-side
 * pre-detect / source-DPCD / link-training paths to wake the panel before
 * touching the slave's AUX. Safe to call with NULL or non-root link — no-op.
 * On iMacPro1,1 this instead runs the firmware arm handshake once per boot.
 */
enum dc_status link_apple_5k_root_panel_latch_pulse(struct dc_link *root_link);

/*
 * The EFI ComplexDisplayInit arm handshake (verified 0->1 latch edge with
 * EDID re-reads): reset the panel to its base presentation, arm it to present
 * the tiled identity, verify it took; disarm on failure so the latch is never
 * left armed without the combined enable (the hard-wedge state).
 */
enum dc_status link_apple_5k_arm_handshake(struct dc_link *root_link);

/*
 * Snapshot of the Apple 5K root panel's private mode/status registers
 * (root AUX). Ground truth from native-vs-wedged DPCD dumps:
 *   0x420-0x427  status block. 0x425 bit1 = compat (clear = native).
 *                0x423 bit2 + 0x424 bit2 = TCON FAULT flags -- set only in
 *                the wedged armed-compat state (latch stuck at 1 without an
 *                accepted combined enable), clear in both native and plain
 *                compat. While faulted the TCON refuses latch writes and
 *                only a cold power-off recovers it.
 *   0x41C        native marker (bit4; panel-maintained, sticks at 0x15 once
 *                armed).
 *   0x4F1        the mode latch itself.
 */
struct apple5k_panel_state {
	bool valid;	/* AUX reads succeeded */
	bool native;	/* 0x425 bit1 clear */
	bool fault;	/* 0x423 bit2 or 0x424 bit2 set */
	uint8_t block[8];	/* DPCD 0x420-0x427 */
	uint8_t marker;	/* DPCD 0x41C */
	uint8_t latch;	/* DPCD 0x4F1 */
};

/*
 * Read the snapshot above off the tiled root's AUX. Logs one
 * "APPLE5K: panel mode (<stage>)" line unless @stage is NULL (quiet, for
 * change-polling). Returns false if @root_link is not the tiled root or the
 * AUX read fails.
 */
bool link_apple_5k_sample_panel_state(struct dc_link *root_link,
				      const char *stage,
				      struct apple5k_panel_state *state);

/*
 * Read the same register window off the slave tile's own AUX (log-only).
 */
void link_apple_5k_sample_slave_state(struct dc_link *slave_link,
				      const char *stage);

/*
 * Armed-window bisect probe: log root + slave-own panel state at a named
 * phase of the tiled pair's link bring-up. No-op when not arming.
 */
void link_apple_5k_lt_bisect(struct dc_link *link, const char *stage);

/*
 * Diagnostic: if the root TCON fault bits are set, attempt the standard
 * recovery writes (W1C 0x424/0x423, then 0x426=0 reset) and log whether the
 * fault is host-clearable or sticky. No-op when not faulted.
 */
void link_apple_5k_try_clear_fault(struct dc_link *root_link, const char *stage);

#endif
