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
 * APPLE5K experiment toggle. Set to 0 to build a panel-NON-DESTRUCTIVE kernel:
 * NO DPCD writes to the Apple 5K panel (0x4F1 latch, 0x41C/0x425 mode triplet,
 * 0x300 source-OUI). Hypothesis (per warm-boot test): our panel DPCD writes
 * wedge the TCON so even the Apple EFI firmware can't restore native on a warm
 * boot, whereas the vanilla kernel leaves it EFI-recoverable. With this at 0 the
 * kernel should behave (toward the panel) like vanilla -> warm-boot into EFI
 * should restore native if our writes are the wedge. Set to 1 for the prior
 * write-the-latch behavior.
 */
#define APPLE5K_PANEL_DPCD_WRITES 0

/*
 * Pulse the Apple 5K root panel-latch DPCD (0x4F1 = 1). Used by the slave-side
 * pre-detect / source-DPCD / link-training paths to wake the panel before
 * touching the slave's AUX. Safe to call with NULL or non-root link — no-op.
 */
enum dc_status link_apple_5k_root_panel_latch_pulse(struct dc_link *root_link);

/*
 * APPLE5K read-only probe: log the root panel's mode triplet (0x41C/0x425/0x4F1)
 * from any tiled link, to bisect which driver action flips it native->compat.
 */
void apple5k_probe_mode(struct dc_link *link, const char *tag);

/*
 * APPLE5K experiment: write the Apple *source* OUI (00:10:FA "AAPL" ...) to the
 * root panel DPCD 0x300, mirroring macOS AuxChannelProxy::initializeAppleOUI.
 * Hypothesis: the panel TCON keeps/enters native only when it sees an Apple
 * source OUI; presenting it before amdgpu blanks/retrains the firmware-native
 * display may stop the native->compat flip. This is a 0x300 source-OUI write
 * only (NOT the 0x4F1 latch), so it does not hard-wedge the panel.
 */
#define APPLE5K_WRITE_SRC_OUI 1
void apple5k_write_src_oui(struct dc_link *link, const char *tag);

#endif
