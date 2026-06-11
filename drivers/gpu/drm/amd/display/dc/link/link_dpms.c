/*
 * Copyright 2023 Advanced Micro Devices, Inc.
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
 * This file owns the programming sequence of stream's dpms state associated
 * with the link and link's enable/disable sequences as result of the stream's
 * dpms state change.
 *
 * TODO - The reason link owns stream's dpms programming sequence is
 * because dpms programming sequence is highly dependent on underlying signal
 * specific link protocols. This unfortunately causes link to own a portion of
 * stream state programming sequence. This creates a gray area where the
 * boundary between link and stream is not clearly defined.
 */

#include "link_dpms.h"
#include "link_hwss.h"
#include "link_validation.h"
#include "accessories/link_dp_trace.h"
#include "protocols/link_dpcd.h"
#include "protocols/link_ddc.h"
#include "protocols/link_hpd.h"
#include "protocols/link_dp_phy.h"
#include "protocols/link_dp_capability.h"
#include "protocols/link_dp_training.h"
#include "protocols/link_edp_panel_control.h"
#include "protocols/link_dp_panel_replay.h"
#include "protocols/link_dp_dpia_bw.h"

#include "dm_helpers.h"
#include "link_enc_cfg.h"
#include "resource.h"
#include "dsc.h"
#include "dccg.h"
#include "clk_mgr.h"
#include "atomfirmware.h"
#include "vpg.h"
#include "grph_object_id.h"

#define DC_LOGGER \
	dc_logger
#define DC_LOGGER_INIT(logger) \
	struct dal_logger *dc_logger = logger

#define LINK_INFO(...) \
	DC_LOG_HW_HOTPLUG(  \
		__VA_ARGS__)

#define RETIMER_REDRIVER_INFO(...) \
	DC_LOG_RETIMER_REDRIVER(  \
		__VA_ARGS__)

#define MAX_MTP_SLOT_COUNT 64
#define LINK_TRAINING_ATTEMPTS 4
#define PEAK_FACTOR_X1000 1006
#define APPLE_5K_DPCD_PANEL_LATCH 0x4F1

static void dp_write_tiled_stream_enable_latch(struct dc_link *link)
{
	struct dc_link *root_link;
	uint8_t payload = 1;
	uint8_t readback = 0;
	uint8_t root_readback = 0;
	enum dc_status status;
	enum dc_status read_status;
	enum dc_status root_status;
	enum dc_status root_read_status;

	if (!dc_link_needs_tiled_stream_enable_latch(link) || !link->local_sink)
		return;

	/*
	 * The stream-enable re-pulse is the slave bring-up mechanism for the
	 * OTHER tiled iMac models. The iMac Pro firmware writes no 0x4F1
	 * during the mode-set (the RE'd ComplexDisplayInit arms once BEFORE
	 * programming), and a mid-enable write can re-latch/wedge its TCON --
	 * skip on iMacPro1,1 in both boot modes. The compat-boot arm happens
	 * before root link training in enable_link_dp() instead.
	 */
	if (link->apple5k_imac_pro ||
	    (link->tiled_peer && link->tiled_peer->apple5k_imac_pro))
		return;

	DC_LOGGER_INIT(link->ctx->logger);

	root_link = link->tiled_peer;
	if (dc_link_has_tiled_root_panel_patch(root_link)) {
		root_status = link_apple_5k_root_panel_latch_pulse(root_link);
		root_read_status = core_link_read_dpcd(root_link,
						       APPLE_5K_DPCD_PANEL_LATCH,
						       &root_readback,
						       sizeof(root_readback));
		DC_LOG_INFO("APPLE5K: stream-enable root latch 0x4F1 slave_link[%u] root_link[%u] status=%d value=0x%02x read_status=%d readback=0x%02x root_sink=%p slave_sink=%p\n",
			    link->link_index, root_link->link_index, root_status,
			    payload, root_read_status, root_readback,
			    root_link->local_sink, link->local_sink);
	}

	status = core_link_write_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH,
				      &payload, sizeof(payload));
	read_status = core_link_read_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH,
					  &readback, sizeof(readback));
	DC_LOG_INFO("APPLE5K: stream-enable latch 0x4F1 link[%u] status=%d value=0x%02x read_status=%d readback=0x%02x sink=%p\n",
		    link->link_index, status, payload, read_status, readback,
		    link->local_sink);
}

/*
 * eDP VDD power-off guard for the Apple 5K tiled root. Powering the panel down
 * while the arm latch is set (0x4F1=1) faults the TCON into a stuck-stretched
 * state that survives warm reboot and is only cleared by a cold power-off --
 * the EFI firmware never powers down armed (ComplexDisplayInit never power-
 * cycles between the arm and the combined enable, and its teardown disarms
 * first). Returns true if the caller must SKIP the power-off:
 *
 *   - arm in progress (apple5k_arming): the modeset disable-phase is trying to
 *     drop VDD between our arm and the combined enable. SKIP it -- keep the
 *     panel powered (OTG stays blanked separately) so the latch survives to
 *     the enable, exactly like the firmware's quiet-powered arm.
 *   - otherwise armed (latch=1) and genuinely powering down: DISARM (0x4F1=0)
 *     first, then allow the power-off -- never power down armed.
 */
bool link_apple5k_power_off_guard(struct dc_link *link)
{
	uint8_t latch = 0;
	DC_LOGGER_INIT(link->ctx->logger);

	if (!dc_link_has_tiled_root_panel_patch(link))
		return false;

	if (link->apple5k_arming) {
		DC_LOG_INFO("APPLE5K: keep tiled root eDP VDD on across modeset (arm in progress) link[%u] -- latch must survive to the combined enable\n",
			    link->link_index);
		return true; /* suppress the power-off */
	}

	core_link_read_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH,
			    &latch, sizeof(latch));
	if (latch == 1) {
		latch = 0;
		core_link_write_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH,
				     &latch, sizeof(latch));
		link->apple5k_armed = false;
		DC_LOG_INFO("APPLE5K: disarmed 0x4F1=0 before a genuine eDP VDD power-off link[%u] (never power down armed)\n",
			    link->link_index);
	}
	return false;
}

/*
 * Find the master pipe in @state driving the other tile of @pipe_ctx's
 * dual-tile pair (dc_link.tiled_peer), or NULL. Used to order the pair's
 * unblanks so the panel never sees a solo tile.
 */
static struct pipe_ctx *get_tiled_peer_pipe(struct dc_state *state,
					    struct pipe_ctx *pipe_ctx)
{
	struct dc_link *peer_link = pipe_ctx->stream->link->tiled_peer;
	int i;

	if (!peer_link ||
	    pipe_ctx->stream->link->tiled_role == DC_TILED_ROLE_NONE)
		return NULL;

	for (i = 0; i < MAX_PIPES; i++) {
		struct pipe_ctx *p = &state->res_ctx.pipe_ctx[i];

		if (p->stream && !p->top_pipe && !p->prev_odm_pipe &&
		    p->stream->link == peer_link && !p->stream->dpms_off)
			return p;
	}

	return NULL;
}

#define APPLE_5K_DPCD_PANEL_MODE_MARKER 0x41C
#define APPLE_5K_DPCD_PANEL_MODE_STATUS 0x425

/*
 * After a combined dual-tile enable, re-sample the root panel mode status
 * (DPCD 0x425, bit1 set = compat) to learn whether the TCON latched native
 * dual-tile mode. 0x41C (native marker, bit4 -- self-asserted by the panel
 * with a delay) and 0x4F1 (the latch) are read alongside for diagnostics.
 * On a compat boot that just latched, flip apple5k_native_boot so the
 * preservation paths (keep-stream-on-blank, slave skip-retrain, no 0x4F1
 * re-latch) protect the freshly latched state exactly like an EFI-native
 * handoff -- otherwise the next blank/probe churn drives a tile solo and
 * drops the panel back to compat. Returns whether the panel reads native.
 */
static bool tiled_pair_sample_native_latch(struct dc_link *root_link,
					   const char *stage)
{
	struct apple5k_panel_state st;
	DC_LOGGER_INIT(root_link->ctx->logger);

	if (!link_apple_5k_sample_panel_state(root_link, stage, &st))
		return false;

	if (st.native && !root_link->apple5k_native_boot) {
		root_link->apple5k_native_boot = true;
		DC_LOG_INFO("APPLE5K: panel latched native after combined enable -- preservation paths now active\n");
	}

	return st.native;
}

/*
 * Sample the panel state via the pair's ROOT from any pair-member link --
 * used to bracket the teardown paths (dpms-off) so the log shows exactly
 * which step flips the panel mode / fault bits. No-op for non-tiled links.
 */
static void tiled_pair_sample_via_root(struct dc_link *link, const char *stage)
{
	struct dc_link *root = NULL;

	if (dc_link_has_tiled_root_panel_patch(link))
		root = link;
	else if (link && dc_link_has_tiled_root_panel_patch(link->tiled_peer))
		root = link->tiled_peer;

	if (root)
		link_apple_5k_sample_panel_state(root, stage, NULL);
}

/*
 * Sink-side link health for both tiles: trained-lane status (DPCD
 * 0x202-0x207) and per-lane symbol error counters (0x210-0x217, bit15 =
 * count valid). A marginal or dead slave link is invisible in compat mode
 * (the panel displays only the root tile), yet would make the TCON refuse
 * native no matter how clean the arm sequence is -- so read what the PANEL
 * says about each link.
 */
static void tiled_pair_log_link_health(struct pipe_ctx *root_pipe,
				       struct pipe_ctx *peer_pipe,
				       const char *stage)
{
	struct pipe_ctx *pipes[2] = { root_pipe, peer_pipe };
	int i;
	DC_LOGGER_INIT(root_pipe->stream->link->ctx->logger);

	for (i = 0; i < 2; i++) {
		uint8_t lane_status[6] = { 0 };
		uint8_t err[8] = { 0 };
		struct dc_link *l;

		if (!pipes[i] || !pipes[i]->stream)
			continue;
		l = pipes[i]->stream->link;
		core_link_read_dpcd(l, DP_LANE0_1_STATUS,
				    lane_status, sizeof(lane_status));
		/* 0x210-0x217 SYMBOL_ERROR_COUNT_LANE0-3 (no drm_dp.h define) */
		core_link_read_dpcd(l, 0x210, err, sizeof(err));
		DC_LOG_INFO("APPLE5K: link health (%s) link[%u] lane01=0x%02x lane23=0x%02x align=0x%02x sink=0x%02x adj=0x%02x,0x%02x symerr L0=%u%s L1=%u%s L2=%u%s L3=%u%s\n",
			    stage, l->link_index,
			    lane_status[0], lane_status[1], lane_status[2],
			    lane_status[3], lane_status[4], lane_status[5],
			    ((err[1] & 0x7f) << 8) | err[0],
			    (err[1] & 0x80) ? "" : "(invalid)",
			    ((err[3] & 0x7f) << 8) | err[2],
			    (err[3] & 0x80) ? "" : "(invalid)",
			    ((err[5] & 0x7f) << 8) | err[4],
			    (err[5] & 0x80) ? "" : "(invalid)",
			    ((err[7] & 0x7f) << 8) | err[6],
			    (err[7] & 0x80) ? "" : "(invalid)");
	}
}

/*
 * Light a dual-tile pair whose unblanks were deferred through
 * apply_ctx_to_hw(). Called from dc_commit_state_no_check() right after
 * program_timing_sync() has phase-aligned the still-blanked OTGs, so the
 * panel's first dual-tile video is coherent: root tile first, slave tile
 * back-to-back -- the EFI firmware's combined-enable order. Then sample the
 * panel mode to learn whether the TCON latched native.
 */
void link_tiled_pair_post_sync_unblank(struct dc *dc, struct dc_state *context)
{
	int i;
	DC_LOGGER_INIT(dc->ctx->logger);

	for (i = 0; i < MAX_PIPES; i++) {
		struct pipe_ctx *pipe = &context->res_ctx.pipe_ctx[i];
		struct pipe_ctx *peer_pipe;

		if (!pipe->stream || !pipe->tiled_unblank_deferred)
			continue;
		if (pipe->stream->link->tiled_role != DC_TILED_ROLE_ROOT)
			continue; /* handle each pair once, root first */

		peer_pipe = get_tiled_peer_pipe(context, pipe);

		/*
		 * Bisect sample: panel state right before first video. If the
		 * TCON fault is already set here, the training/teardown phase
		 * caused it; if it appears only in the post-unblank sample,
		 * the TCON is rejecting the video/enable itself.
		 */
		link_apple_5k_sample_panel_state(pipe->stream->link,
						 "pre-unblank", NULL);

		pipe->tiled_unblank_deferred = false;
		dc->hwss.unblank_stream(pipe,
			&pipe->stream->link->cur_link_settings);
		if (peer_pipe && peer_pipe->tiled_unblank_deferred) {
			peer_pipe->tiled_unblank_deferred = false;
			dc->hwss.unblank_stream(peer_pipe,
				&peer_pipe->stream->link->cur_link_settings);
		}

		DC_LOG_INFO("APPLE5K: post-sync joint unblank tiled pair link[%u] + link[%d]\n",
			    pipe->stream->link->link_index,
			    peer_pipe ? (int)peer_pipe->stream->link->link_index : -1);

		/* What does the PANEL say about each tile's link right now? */
		tiled_pair_log_link_health(pipe, peer_pipe, "post-unblank");

		/*
		 * Settle window: hold the commit here -- AUX and commit
		 * silence, the same quiet the firmware gives the TCON after
		 * its combined enable -- and watch the TCON decide. Poll the
		 * panel state, logging every change, until it goes NATIVE
		 * (accepted), latches the fault bits (rejected), or the
		 * window expires. The single former 20ms sample could not
		 * distinguish "not decided yet" from "refused".
		 */
		{
			struct apple5k_panel_state prev = { 0 };
			struct apple5k_panel_state cur;
			int elapsed = 0;

			msleep(20);
			link_apple_5k_sample_panel_state(pipe->stream->link,
					"post-sync joint unblank", &prev);
			while (elapsed < 2000 && prev.valid &&
			       !prev.native && !prev.fault) {
				msleep(50);
				elapsed += 50;
				if (!link_apple_5k_sample_panel_state(
					    pipe->stream->link, NULL, &cur))
					break;
				if (memcmp(cur.block, prev.block,
					   sizeof(cur.block)) ||
				    cur.marker != prev.marker ||
				    cur.latch != prev.latch)
					DC_LOG_INFO("APPLE5K: panel state CHANGED at +%dms 0x425=0x%02x 0x41C=0x%02x 0x4F1=0x%02x 0x423=0x%02x 0x424=0x%02x block=%8ph -> %s%s\n",
						    elapsed + 20,
						    cur.block[5], cur.marker,
						    cur.latch, cur.block[3],
						    cur.block[4], cur.block,
						    cur.native ? "NATIVE" : "compat",
						    cur.fault ? " [TCON-FAULT]" : "");
				prev = cur;
			}
		}

		/* Final verdict; flips native_boot -> preservation on success. */
		tiled_pair_sample_native_latch(pipe->stream->link,
					       "post-unblank settle end");
		tiled_pair_log_link_health(pipe, peer_pipe, "settle end");
		/*
		 * NOTE: the failure "reset 0x4F1=0" is intentionally DISABLED.
		 * Leave the latch in whatever state the arm left it so the
		 * post-enable panel/AUX state can be inspected on a wedged
		 * boot; re-enable the disarm once the arm mechanism works.
		 */

		/*
		 * The arm sequence has resolved (the combined enable ran): the
		 * latch no longer needs to survive a power-off mid-sequence.
		 * Clear arm-in-progress on both tiles so future genuine power-
		 * offs disarm-then-power-down normally.
		 */
		pipe->stream->link->apple5k_arming = false;
		pipe->stream->link->wa_flags.dp_keep_receiver_powered = false;
		if (peer_pipe) {
			peer_pipe->stream->link->apple5k_arming = false;
			peer_pipe->stream->link->wa_flags.dp_keep_receiver_powered = false;
			peer_pipe->stream->link->skip_fallback_on_link_loss = false;
		}
	}
}

void link_blank_all_dp_displays(struct dc *dc)
{
	unsigned int i;
	uint8_t dpcd_power_state = '\0';
	enum dc_status status = DC_ERROR_UNEXPECTED;

	for (i = 0; i < dc->link_count; i++) {
		if ((dc->links[i]->connector_signal != SIGNAL_TYPE_DISPLAY_PORT) ||
			(dc->links[i]->priv == NULL) || (dc->links[i]->local_sink == NULL))
			continue;

		/* APPLE5K: skip ALL boot-cleanup link touches for the iMac Pro's native
		 * tiled panel -- dp_retrieve_lttpr_cap() writes DP_PHY_REPEATER_MODE and
		 * re-latches the firmware-native panel to compat. Preserve it. */
		if (dc_link_apple5k_protect(dc->links[i]))
			continue;

		/* DP 2.0 spec requires that we read LTTPR caps first */
		dp_retrieve_lttpr_cap(dc->links[i]);
		/* if any of the displays are lit up turn them off */
		status = core_link_read_dpcd(dc->links[i], DP_SET_POWER,
							&dpcd_power_state, sizeof(dpcd_power_state));

		if (status == DC_OK && dpcd_power_state == DP_POWER_STATE_D0)
			link_blank_dp_stream(dc->links[i], true);
	}

}

void link_blank_all_edp_displays(struct dc *dc)
{
	unsigned int i;
	uint8_t dpcd_power_state = '\0';
	enum dc_status status = DC_ERROR_UNEXPECTED;

	for (i = 0; i < dc->link_count; i++) {
		if ((dc->links[i]->connector_signal != SIGNAL_TYPE_EDP) ||
			(!dc->links[i]->edp_sink_present))
			continue;

		/* APPLE5K: preserve the iMac Pro's firmware-native tiled root -- skip
		 * boot cleanup so it isn't re-latched to compat. */
		if (dc_link_apple5k_protect(dc->links[i]))
			continue;

		/* if any of the displays are lit up turn them off */
		status = core_link_read_dpcd(dc->links[i], DP_SET_POWER,
							&dpcd_power_state, sizeof(dpcd_power_state));

		if (status == DC_OK && dpcd_power_state == DP_POWER_STATE_D0)
			link_blank_dp_stream(dc->links[i], true);
	}
}

void link_blank_dp_stream(struct dc_link *link, bool hw_init)
{
	unsigned int j;
	struct dc  *dc = link->ctx->dc;
	enum signal_type signal = link->connector_signal;

	/* APPLE5K: preserve the iMac Pro's firmware-handed-off native 5K display --
	 * do NOT blank/power-down the tiled root/slave streams during boot cleanup,
	 * since that resets the latched panel TCON to compat. Runtime display-off
	 * goes through dce110_blank_stream, not here. */
	if (dc_link_apple5k_protect(link))
		return;

	if ((signal == SIGNAL_TYPE_EDP) ||
		(signal == SIGNAL_TYPE_DISPLAY_PORT)) {
		if (link->ep_type == DISPLAY_ENDPOINT_PHY &&
			link->link_enc->funcs->get_dig_frontend &&
			link->link_enc->funcs->is_dig_enabled(link->link_enc)) {
			int fe = link->link_enc->funcs->get_dig_frontend(link->link_enc);

			if (fe != ENGINE_ID_UNKNOWN)
				for (j = 0; j < dc->res_pool->stream_enc_count; j++) {
					if (fe == dc->res_pool->stream_enc[j]->id) {
						dc->res_pool->stream_enc[j]->funcs->dp_blank(link,
									dc->res_pool->stream_enc[j]);
						break;
					}
				}
		}

		if (((!dc->is_switch_in_progress_dest) && ((!link->wa_flags.dp_keep_receiver_powered) || hw_init)) &&
			(link->type != dc_connection_none))
			dpcd_write_rx_power_ctrl(link, false);
	}
}

void link_set_all_streams_dpms_off_for_link(struct dc_link *link)
{
	struct pipe_ctx *pipes[MAX_PIPES];
	struct dc_stream_state *streams[MAX_PIPES];
	struct dc_state *state = link->dc->current_state;
	uint8_t count;
	int i;
	struct dc_stream_update stream_update;
	bool dpms_off = true;
	struct link_resource link_res = {0};

	memset(&stream_update, 0, sizeof(stream_update));
	stream_update.dpms_off = &dpms_off;

	link_get_master_pipes_with_dpms_on(link, state, &count, pipes);

	/* The subsequent call to dc_commit_updates_for_stream for a full update
	 * will release the current state and swap to a new state. Releasing the
	 * current state results in the stream pointers in the pipe_ctx structs
	 * to be zero'd. Hence, cache all streams prior to dc_commit_updates_for_stream.
	 */
	for (i = 0; i < count; i++)
		streams[i] = pipes[i]->stream;

	for (i = 0; i < count; i++) {
		stream_update.stream = streams[i];
		dc_commit_updates_for_stream(link->ctx->dc, NULL, 0,
				streams[i], &stream_update,
				state);
	}

	/* link can be also enabled by vbios. In this case it is not recorded
	 * in pipe_ctx. Disable link phy here to make sure it is completely off
	 */
	dp_disable_link_phy(link, &link_res, link->connector_signal);
}

void link_resume(struct dc_link *link)
{
	if (link->connector_signal != SIGNAL_TYPE_VIRTUAL)
		program_hpd_filter(link);
}

void dc_apple5k_tiled_panel_blank(struct dc *dc, struct dc_link *root_link, bool off)
{
	bool was_on;

	if (!dc_link_has_tiled_root_panel_patch(root_link) ||
	    !dc->hwss.edp_backlight_control)
		return;

	DC_LOGGER_INIT(dc->ctx->logger);

	was_on = root_link->panel_cntl &&
		 root_link->panel_cntl->funcs->is_panel_backlight_on &&
		 root_link->panel_cntl->funcs->is_panel_backlight_on(root_link->panel_cntl);

	/* off: disable the backlight (BL_PWM_EN off) -- darkens the panel while the
	 * pipe/OTG/link stay up (keep-stream gates) so the TCON keeps its native
	 * latch, like macOS doDoze(). on: re-enable on wake -- the keep-stream
	 * display-off turned BL_PWM_EN off and the standard unblank path does not
	 * reach the root's edp_backlight_control(true), so do it explicitly here.
	 * (edp_backlight_control no-ops if it reads the backlight already in the
	 * target state, so calling it at boot-on is harmless.) */
	dc->hwss.edp_backlight_control(root_link, !off);
	DC_LOG_INFO("APPLE5K: tiled panel backlight %s link[%u] (was_on=%d)\n",
		    off ? "OFF" : "ON", root_link->link_index, was_on);
}

/* This function returns true if the pipe is used to feed video signal directly
 * to the link.
 */
static bool is_master_pipe_for_link(const struct dc_link *link,
		const struct pipe_ctx *pipe)
{
	return resource_is_pipe_type(pipe, OTG_MASTER) &&
			pipe->stream->link == link;
}

/*
 * This function finds all master pipes feeding to a given link with dpms set to
 * on in given dc state.
 */
void link_get_master_pipes_with_dpms_on(const struct dc_link *link,
		struct dc_state *state,
		uint8_t *count,
		struct pipe_ctx *pipes[MAX_PIPES])
{
	int i;
	struct pipe_ctx *pipe = NULL;

	*count = 0;
	for (i = 0; i < MAX_PIPES; i++) {
		pipe = &state->res_ctx.pipe_ctx[i];

		if (is_master_pipe_for_link(link, pipe) &&
				pipe->stream->dpms_off == false) {
			pipes[(*count)++] = pipe;
		}
	}
}

static bool get_ext_hdmi_settings(struct pipe_ctx *pipe_ctx,
		enum engine_id eng_id,
		struct ext_hdmi_settings *settings)
{
	bool result = false;
	int i = 0;
	struct integrated_info *integrated_info =
			pipe_ctx->stream->ctx->dc_bios->integrated_info;

	if (integrated_info == NULL)
		return false;

	/*
	 * Get retimer settings from sbios for passing SI eye test for DCE11
	 * The setting values are varied based on board revision and port id
	 * Therefore the setting values of each ports is passed by sbios.
	 */

	// Check if current bios contains ext Hdmi settings
	if (integrated_info->gpu_cap_info & 0x20) {
		switch (eng_id) {
		case ENGINE_ID_DIGA:
			settings->slv_addr = integrated_info->dp0_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp0_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp0_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp0_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp0_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp0_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp0_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGB:
			settings->slv_addr = integrated_info->dp1_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp1_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp1_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp1_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp1_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp1_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp1_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGC:
			settings->slv_addr = integrated_info->dp2_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp2_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp2_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp2_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp2_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp2_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp2_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGD:
			settings->slv_addr = integrated_info->dp3_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp3_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp3_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp3_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp3_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp3_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp3_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		default:
			break;
		}

		if (result == true) {
			// Validate settings from bios integrated info table
			if (settings->slv_addr == 0)
				return false;
			if (settings->reg_num > 9)
				return false;
			if (settings->reg_num_6g > 3)
				return false;

			for (i = 0; i < settings->reg_num; i++) {
				if (settings->reg_settings[i].i2c_reg_index > 0x20)
					return false;
			}

			for (i = 0; i < settings->reg_num_6g; i++) {
				if (settings->reg_settings_6g[i].i2c_reg_index > 0x20)
					return false;
			}
		}
	}

	return result;
}

static bool write_i2c(struct pipe_ctx *pipe_ctx,
		uint8_t address, uint8_t *buffer, uint32_t length)
{
	struct i2c_command cmd = {0};
	struct i2c_payload payload = {0};

	memset(&payload, 0, sizeof(payload));
	memset(&cmd, 0, sizeof(cmd));

	cmd.number_of_payloads = 1;
	cmd.engine = I2C_COMMAND_ENGINE_DEFAULT;
	cmd.speed = pipe_ctx->stream->ctx->dc->caps.i2c_speed_in_khz;

	payload.address = address;
	payload.data = buffer;
	payload.length = length;
	payload.write = true;
	cmd.payloads = &payload;

	if (dm_helpers_submit_i2c(pipe_ctx->stream->ctx,
			pipe_ctx->stream->link, &cmd))
		return true;

	return false;
}

static void write_i2c_retimer_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_vga_mode,
		bool is_over_340mhz,
		struct ext_hdmi_settings *settings)
{
	uint8_t slave_address = (settings->slv_addr >> 1);
	uint8_t buffer[2];
	const uint8_t apply_rx_tx_change = 0x4;
	uint8_t offset = 0xA;
	uint8_t value = 0;
	int i = 0;
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	/* Start Ext-Hdmi programming*/

	for (i = 0; i < settings->reg_num; i++) {
		/* Apply 3G settings */
		if (settings->reg_settings[i].i2c_reg_index <= 0x20) {

			buffer[0] = settings->reg_settings[i].i2c_reg_index;
			buffer[1] = settings->reg_settings[i].i2c_reg_val;
			i2c_success = write_i2c(pipe_ctx, slave_address,
						buffer, sizeof(buffer));
			RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
				offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
				slave_address, buffer[0], buffer[1], i2c_success?1:0);

			if (!i2c_success)
				goto i2c_write_fail;

			/* Based on DP159 specs, APPLY_RX_TX_CHANGE bit in 0x0A
			 * needs to be set to 1 on every 0xA-0xC write.
			 */
			if (settings->reg_settings[i].i2c_reg_index == 0xA ||
				settings->reg_settings[i].i2c_reg_index == 0xB ||
				settings->reg_settings[i].i2c_reg_index == 0xC) {

				/* Query current value from offset 0xA */
				if (settings->reg_settings[i].i2c_reg_index == 0xA)
					value = settings->reg_settings[i].i2c_reg_val;
				else {
					i2c_success =
						link_query_ddc_data(
						pipe_ctx->stream->link->ddc,
						slave_address, &offset, 1, &value, 1);
					if (!i2c_success)
						goto i2c_write_fail;
				}

				buffer[0] = offset;
				/* Set APPLY_RX_TX_CHANGE bit to 1 */
				buffer[1] = value | apply_rx_tx_change;
				i2c_success = write_i2c(pipe_ctx, slave_address,
						buffer, sizeof(buffer));
				RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
					offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
					slave_address, buffer[0], buffer[1], i2c_success?1:0);
				if (!i2c_success)
					goto i2c_write_fail;
			}
		}
	}

	/* Apply 3G settings */
	if (is_over_340mhz) {
		for (i = 0; i < settings->reg_num_6g; i++) {
			/* Apply 3G settings */
			if (settings->reg_settings[i].i2c_reg_index <= 0x20) {

				buffer[0] = settings->reg_settings_6g[i].i2c_reg_index;
				buffer[1] = settings->reg_settings_6g[i].i2c_reg_val;
				i2c_success = write_i2c(pipe_ctx, slave_address,
							buffer, sizeof(buffer));
				RETIMER_REDRIVER_INFO("above 340Mhz: retimer write to slave_address = 0x%x,\
					offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
					slave_address, buffer[0], buffer[1], i2c_success?1:0);

				if (!i2c_success)
					goto i2c_write_fail;

				/* Based on DP159 specs, APPLY_RX_TX_CHANGE bit in 0x0A
				 * needs to be set to 1 on every 0xA-0xC write.
				 */
				if (settings->reg_settings_6g[i].i2c_reg_index == 0xA ||
					settings->reg_settings_6g[i].i2c_reg_index == 0xB ||
					settings->reg_settings_6g[i].i2c_reg_index == 0xC) {

					/* Query current value from offset 0xA */
					if (settings->reg_settings_6g[i].i2c_reg_index == 0xA)
						value = settings->reg_settings_6g[i].i2c_reg_val;
					else {
						i2c_success =
								link_query_ddc_data(
								pipe_ctx->stream->link->ddc,
								slave_address, &offset, 1, &value, 1);
						if (!i2c_success)
							goto i2c_write_fail;
					}

					buffer[0] = offset;
					/* Set APPLY_RX_TX_CHANGE bit to 1 */
					buffer[1] = value | apply_rx_tx_change;
					i2c_success = write_i2c(pipe_ctx, slave_address,
							buffer, sizeof(buffer));
					RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
						offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
						slave_address, buffer[0], buffer[1], i2c_success?1:0);
					if (!i2c_success)
						goto i2c_write_fail;
				}
			}
		}
	}

	if (is_vga_mode) {
		/* Program additional settings if using 640x480 resolution */

		/* Write offset 0xFF to 0x01 */
		buffer[0] = 0xff;
		buffer[1] = 0x01;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
				offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
				slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0x00 to 0x23 */
		buffer[0] = 0x00;
		buffer[1] = 0x23;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0xff to 0x00 */
		buffer[0] = 0xff;
		buffer[1] = 0x00;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

	}

	return;

i2c_write_fail:
	DC_LOG_DEBUG("Set retimer failed");
}

static void write_i2c_default_retimer_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_vga_mode,
		bool is_over_340mhz)
{
	uint8_t slave_address = (0xBA >> 1);
	uint8_t buffer[2];
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	/* Program Slave Address for tuning single integrity */
	/* Write offset 0x0A to 0x13 */
	buffer[0] = 0x0A;
	buffer[1] = 0x13;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer writes default setting to slave_address = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0B to 0xDA or 0xD8 */
	buffer[0] = 0x0B;
	buffer[1] = is_over_340mhz ? 0xDA : 0xD8;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0C to 0x1D or 0x91 */
	buffer[0] = 0x0C;
	buffer[1] = is_over_340mhz ? 0x1D : 0x91;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;


	if (is_vga_mode) {
		/* Program additional settings if using 640x480 resolution */

		/* Write offset 0xFF to 0x01 */
		buffer[0] = 0xff;
		buffer[1] = 0x01;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0x00 to 0x23 */
		buffer[0] = 0x00;
		buffer[1] = 0x23;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
			offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0xff to 0x00 */
		buffer[0] = 0xff;
		buffer[1] = 0x00;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write default setting to slave_addr = 0x%x,\
			offset = 0x%x, reg_val= 0x%x, i2c_success = %d end here\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;
	}

	return;

i2c_write_fail:
	DC_LOG_DEBUG("Set default retimer failed");
}

static void write_i2c_redriver_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_over_340mhz)
{
	uint8_t slave_address = (0xF0 >> 1);
	uint8_t buffer[16];
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	// Program Slave Address for tuning single integrity
	buffer[3] = 0x4E;
	buffer[4] = 0x4E;
	buffer[5] = 0x4E;
	buffer[6] = is_over_340mhz ? 0x4E : 0x4A;

	i2c_success = write_i2c(pipe_ctx, slave_address,
					buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("redriver write 0 to all 16 reg offset expect following:\n\
		\t slave_addr = 0x%x, offset[3] = 0x%x, offset[4] = 0x%x,\
		offset[5] = 0x%x,offset[6] is_over_340mhz = 0x%x,\
		i2c_success = %d\n",
		slave_address, buffer[3], buffer[4], buffer[5], buffer[6], i2c_success?1:0);

	if (!i2c_success)
		DC_LOG_DEBUG("Set redriver failed");
}

static void update_psp_stream_config(struct pipe_ctx *pipe_ctx, bool dpms_off)
{
	struct cp_psp *cp_psp = &pipe_ctx->stream->ctx->cp_psp;
	struct link_encoder *link_enc = pipe_ctx->link_res.dio_link_enc;
	struct cp_psp_stream_config config = {0};
	enum dp_panel_mode panel_mode =
			dp_get_panel_mode(pipe_ctx->stream->link);

	if (cp_psp == NULL || cp_psp->funcs.update_stream_config == NULL)
		return;
	if (!pipe_ctx->stream->ctx->dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(pipe_ctx->stream->link);
	ASSERT(link_enc);
	if (link_enc == NULL)
		return;

	/* otg instance */
	config.otg_inst = (uint8_t) pipe_ctx->stream_res.tg->inst;

	/* dig front end */
	config.dig_fe = (uint8_t) pipe_ctx->stream_res.stream_enc->stream_enc_inst;

	/* stream encoder index */
	config.stream_enc_idx = pipe_ctx->stream_res.stream_enc->id - ENGINE_ID_DIGA;
	if (dp_is_128b_132b_signal(pipe_ctx))
		config.stream_enc_idx =
				pipe_ctx->stream_res.hpo_dp_stream_enc->id - ENGINE_ID_HPO_DP_0;

	/* dig back end */
	config.dig_be = pipe_ctx->stream->link->link_enc_hw_inst;

	/* link encoder index */
	config.link_enc_idx = link_enc->transmitter - TRANSMITTER_UNIPHY_A;
	if (dp_is_128b_132b_signal(pipe_ctx))
		config.link_enc_idx = pipe_ctx->link_res.hpo_dp_link_enc->inst;

	/* dio output index is dpia index for DPIA endpoint & dcio index by default */
	if (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		config.dio_output_idx = pipe_ctx->stream->link->link_id.enum_id - ENUM_ID_1;
	else
		config.dio_output_idx = link_enc->transmitter - TRANSMITTER_UNIPHY_A;


	/* phy index */
	config.phy_idx = resource_transmitter_to_phy_idx(
			pipe_ctx->stream->link->dc, link_enc->transmitter);
	if (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		/* USB4 DPIA doesn't use PHY in our soc, initialize it to 0 */
		config.phy_idx = 0;

	/* stream properties */
	config.assr_enabled = (panel_mode == DP_PANEL_MODE_EDP) ? 1 : 0;
	config.mst_enabled = (pipe_ctx->stream->signal ==
			SIGNAL_TYPE_DISPLAY_PORT_MST) ? 1 : 0;
	config.dp2_enabled = dp_is_128b_132b_signal(pipe_ctx) ? 1 : 0;
	config.usb4_enabled = (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA) ?
			1 : 0;
	config.dpms_off = dpms_off;

	/* dm stream context */
	config.dm_stream_ctx = pipe_ctx->stream->dm_stream_context;

	cp_psp->funcs.update_stream_config(cp_psp->handle, &config);
}

static void set_avmute(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct dc  *dc = pipe_ctx->stream->ctx->dc;

	if (!dc_is_hdmi_signal(pipe_ctx->stream->signal))
		return;

	dc->hwss.set_avmute(pipe_ctx, enable);
}

static void enable_mst_on_sink(struct dc_link *link, bool enable)
{
	unsigned char mstmCntl = 0;

	core_link_read_dpcd(link, DP_MSTM_CTRL, &mstmCntl, 1);
	if (enable)
		mstmCntl |= DP_MST_EN;
	else
		mstmCntl &= (~DP_MST_EN);

	core_link_write_dpcd(link, DP_MSTM_CTRL, &mstmCntl, 1);
}

static void dsc_optc_config_log(struct display_stream_compressor *dsc,
		struct dsc_optc_config *config)
{
	uint32_t precision = 1 << 28;
	uint32_t bytes_per_pixel_int = config->bytes_per_pixel / precision;
	uint32_t bytes_per_pixel_mod = config->bytes_per_pixel % precision;
	uint64_t ll_bytes_per_pix_fraq = bytes_per_pixel_mod;
	DC_LOGGER_INIT(dsc->ctx->logger);

	/* 7 fractional digits decimal precision for bytes per pixel is enough because DSC
	 * bits per pixel precision is 1/16th of a pixel, which means bytes per pixel precision is
	 * 1/16/8 = 1/128 of a byte, or 0.0078125 decimal
	 */
	ll_bytes_per_pix_fraq *= 10000000;
	ll_bytes_per_pix_fraq /= precision;

	DC_LOG_DSC("\tbytes_per_pixel 0x%08x (%d.%07d)",
			config->bytes_per_pixel, bytes_per_pixel_int, (uint32_t)ll_bytes_per_pix_fraq);
	DC_LOG_DSC("\tis_pixel_format_444 %d", config->is_pixel_format_444);
	DC_LOG_DSC("\tslice_width %d", config->slice_width);
}

static bool dp_set_dsc_on_rx(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	bool result = false;

	if (dc_is_virtual_signal(stream->signal))
		result = true;
	else
		result = dm_helpers_dp_write_dsc_enable(dc->ctx, stream, enable);
	return result;
}

static bool dp_set_hblank_reduction_on_rx(struct pipe_ctx *pipe_ctx)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	bool result = false;

	if (dc_is_virtual_signal(stream->signal))
		result = true;
	else
		result = dm_helpers_dp_write_hblank_reduction(dc->ctx, stream);
	return result;
}


/* The stream with these settings can be sent (unblanked) only after DSC was enabled on RX first,
 * i.e. after dp_enable_dsc_on_rx() had been called
 */
void link_set_dsc_on_stream(struct pipe_ctx *pipe_ctx, bool enable)
{
	/* TODO: Move this to HWSS as this is hardware programming sequence not a
	 * link layer sequence
	 */
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct pipe_ctx *odm_pipe;
	int opp_cnt = 1;
	struct dccg *dccg = dc->res_pool->dccg;
	/* It has been found that when DSCCLK is lower than 16Mhz, we will get DCN
	 * register access hung. When DSCCLk is based on refclk, DSCCLk is always a
	 * fixed value higher than 16Mhz so the issue doesn't occur. When DSCCLK is
	 * generated by DTO, DSCCLK would be based on 1/3 dispclk. For small timings
	 * with DSC such as 480p60Hz, the dispclk could be low enough to trigger
	 * this problem. We are implementing a workaround here to keep using dscclk
	 * based on fixed value refclk when timing is smaller than 3x16Mhz (i.e
	 * 48Mhz) pixel clock to avoid hitting this problem.
	 */
	bool should_use_dto_dscclk = (dccg->funcs->set_dto_dscclk != NULL) &&
			stream->timing.pix_clk_100hz > 480000;
	DC_LOGGER_INIT(dsc->ctx->logger);

	for (odm_pipe = pipe_ctx->next_odm_pipe; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe)
		opp_cnt++;

	if (enable) {
		struct dsc_config dsc_cfg;
		struct dsc_optc_config dsc_optc_cfg = {0};
		enum optc_dsc_mode optc_dsc_mode;

		/* Enable DSC hw block */
		dsc_cfg.pic_width = (stream->timing.h_addressable + pipe_ctx->dsc_padding_params.dsc_hactive_padding +
				stream->timing.h_border_left + stream->timing.h_border_right) / opp_cnt;
		dsc_cfg.pic_height = stream->timing.v_addressable + stream->timing.v_border_top + stream->timing.v_border_bottom;
		dsc_cfg.pixel_encoding = stream->timing.pixel_encoding;
		dsc_cfg.color_depth = stream->timing.display_color_depth;
		dsc_cfg.is_odm = pipe_ctx->next_odm_pipe ? true : false;
		dsc_cfg.dc_dsc_cfg = stream->timing.dsc_cfg;
		ASSERT(dsc_cfg.dc_dsc_cfg.num_slices_h % opp_cnt == 0);
		dsc_cfg.dc_dsc_cfg.num_slices_h /= opp_cnt;
		dsc_cfg.dsc_padding = 0;

		if (should_use_dto_dscclk)
			dccg->funcs->set_dto_dscclk(dccg, dsc->inst, dsc_cfg.dc_dsc_cfg.num_slices_h);
		dsc->funcs->dsc_set_config(dsc, &dsc_cfg, &dsc_optc_cfg);
		dsc->funcs->dsc_enable(dsc, pipe_ctx->stream_res.opp->inst);
		for (odm_pipe = pipe_ctx->next_odm_pipe; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe) {
			struct display_stream_compressor *odm_dsc = odm_pipe->stream_res.dsc;

			if (should_use_dto_dscclk)
				dccg->funcs->set_dto_dscclk(dccg, odm_dsc->inst, dsc_cfg.dc_dsc_cfg.num_slices_h);
			odm_dsc->funcs->dsc_set_config(odm_dsc, &dsc_cfg, &dsc_optc_cfg);
			odm_dsc->funcs->dsc_enable(odm_dsc, odm_pipe->stream_res.opp->inst);
		}
		dsc_cfg.dc_dsc_cfg.num_slices_h *= opp_cnt;
		dsc_cfg.pic_width *= opp_cnt;
		dsc_cfg.dsc_padding = pipe_ctx->dsc_padding_params.dsc_hactive_padding;

		optc_dsc_mode = dsc_optc_cfg.is_pixel_format_444 ? OPTC_DSC_ENABLED_444 : OPTC_DSC_ENABLED_NATIVE_SUBSAMPLED;

		/* Enable DSC in encoder */
		if (dc_is_dp_signal(stream->signal) && !dp_is_128b_132b_signal(pipe_ctx)) {
			DC_LOG_DSC("Setting stream encoder DSC config for engine %d:", (int)pipe_ctx->stream_res.stream_enc->id);
			dsc_optc_config_log(dsc, &dsc_optc_cfg);
			if (pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config)
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config(pipe_ctx->stream_res.stream_enc,
										optc_dsc_mode,
										dsc_optc_cfg.bytes_per_pixel,
										dsc_optc_cfg.slice_width);

			/* PPS SDP is set elsewhere because it has to be done after DIG FE is connected to DIG BE */
		}

		/* Enable DSC in OPTC */
		DC_LOG_DSC("Setting optc DSC config for tg instance %d:", pipe_ctx->stream_res.tg->inst);
		dsc_optc_config_log(dsc, &dsc_optc_cfg);
		pipe_ctx->stream_res.tg->funcs->set_dsc_config(pipe_ctx->stream_res.tg,
							optc_dsc_mode,
							dsc_optc_cfg.bytes_per_pixel,
							dsc_optc_cfg.slice_width);
	} else {
		/* disable DSC in OPTC */
		pipe_ctx->stream_res.tg->funcs->set_dsc_config(
				pipe_ctx->stream_res.tg,
				OPTC_DSC_DISABLED, 0, 0);

		/* disable DSC in stream encoder */
		if (dc_is_dp_signal(stream->signal)) {
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										false,
										NULL,
										true);
			else {
				if (pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config)
					pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config(
							pipe_ctx->stream_res.stream_enc,
							OPTC_DSC_DISABLED, 0, 0);
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
							pipe_ctx->stream_res.stream_enc, false, NULL, true);
			}
		}

		/* disable DSC block */
		for (odm_pipe = pipe_ctx; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe) {
			odm_pipe->stream_res.dsc->funcs->dsc_disconnect(odm_pipe->stream_res.dsc);
			/*
			 * TODO - dsc_disconnect is a double buffered register.
			 * by the time we call dsc_disable, dsc may still remain
			 * connected to OPP. In this case OPTC will no longer
			 * get correct pixel data because DSCC is off. However
			 * we also can't wait for the  disconnect pending
			 * complete, because this function can be called
			 * with/without OTG master lock acquired. When the lock
			 * is acquired we will never get pending complete until
			 * we release the lock later. So there is no easy way to
			 * solve this problem especially when the lock is
			 * acquired. DSC is a front end hw block it should be
			 * programmed as part of front end sequence, where the
			 * commit sequence without lock and update sequence
			 * with lock are completely separated. However because
			 * we are programming dsc as part of back end link
			 * programming sequence, we don't know if front end OPTC
			 * master lock is acquired. The back end should be
			 * agnostic to front end lock. DSC programming shouldn't
			 * belong to this sequence.
			 */
			odm_pipe->stream_res.dsc->funcs->dsc_disable(odm_pipe->stream_res.dsc);
			if (dccg->funcs->set_ref_dscclk)
				dccg->funcs->set_ref_dscclk(dccg, odm_pipe->stream_res.dsc->inst);
		}
	}
}

/*
 * For dynamic bpp change case, dsc is programmed with MASTER_UPDATE_LOCK enabled;
 * hence PPS info packet update need to use frame update instead of immediate update.
 * Added parameter immediate_update for this purpose.
 * The decision to use frame update is hard-coded in function dp_update_dsc_config(),
 * which is the only place where a "false" would be passed in for param immediate_update.
 *
 * immediate_update is only applicable when DSC is enabled.
 */
bool link_set_dsc_pps_packet(struct pipe_ctx *pipe_ctx, bool enable, bool immediate_update)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	struct dc_stream_state *stream = pipe_ctx->stream;

	if (!pipe_ctx->stream->timing.flags.DSC)
		return false;

	if (!dsc)
		return false;

	DC_LOGGER_INIT(dsc->ctx->logger);

	if (enable) {
		struct dsc_config dsc_cfg;
		uint8_t dsc_packed_pps[128];

		memset(&dsc_cfg, 0, sizeof(dsc_cfg));
		memset(dsc_packed_pps, 0, 128);

		/* Enable DSC hw block */
		dsc_cfg.pic_width = stream->timing.h_addressable + stream->timing.h_border_left + stream->timing.h_border_right;
		dsc_cfg.pic_height = stream->timing.v_addressable + stream->timing.v_border_top + stream->timing.v_border_bottom;
		dsc_cfg.pixel_encoding = stream->timing.pixel_encoding;
		dsc_cfg.color_depth = stream->timing.display_color_depth;
		dsc_cfg.is_odm = pipe_ctx->next_odm_pipe ? true : false;
		dsc_cfg.dc_dsc_cfg = stream->timing.dsc_cfg;
		dsc_cfg.dsc_padding = pipe_ctx->dsc_padding_params.dsc_hactive_padding;

		dsc->funcs->dsc_get_packed_pps(dsc, &dsc_cfg, &dsc_packed_pps[0]);
		memcpy(&stream->dsc_packed_pps[0], &dsc_packed_pps[0], sizeof(stream->dsc_packed_pps));
		if (dc_is_dp_signal(stream->signal)) {
			DC_LOG_DSC("Setting stream encoder DSC PPS SDP for engine %d\n", (int)pipe_ctx->stream_res.stream_enc->id);
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										true,
										&dsc_packed_pps[0],
										immediate_update);
			else
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
						pipe_ctx->stream_res.stream_enc,
						true,
						&dsc_packed_pps[0],
						immediate_update);
		}
	} else {
		/* disable DSC PPS in stream encoder */
		memset(&stream->dsc_packed_pps[0], 0, sizeof(stream->dsc_packed_pps));
		if (dc_is_dp_signal(stream->signal)) {
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										false,
										NULL,
										true);
			else
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
						pipe_ctx->stream_res.stream_enc, false, NULL, true);
		}
	}

	return true;
}

bool link_set_dsc_enable(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	bool result = false;

	if (!pipe_ctx->stream->timing.flags.DSC)
		goto out;
	if (!dsc)
		goto out;

	if (enable) {
		{
			link_set_dsc_on_stream(pipe_ctx, true);
			result = true;
		}
	} else {
		dp_set_dsc_on_rx(pipe_ctx, false);
		link_set_dsc_on_stream(pipe_ctx, false);
		result = true;
	}
out:
	return result;
}

bool link_update_dsc_config(struct pipe_ctx *pipe_ctx)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;

	if (!pipe_ctx->stream->timing.flags.DSC)
		return false;
	if (!dsc)
		return false;

	link_set_dsc_on_stream(pipe_ctx, true);
	link_set_dsc_pps_packet(pipe_ctx, true, false);
	return true;
}

static void enable_stream_features(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;

	if (pipe_ctx->stream->signal != SIGNAL_TYPE_DISPLAY_PORT_MST) {
		struct dc_link *link = stream->link;
		union down_spread_ctrl old_downspread;
		union down_spread_ctrl new_downspread;

		memset(&old_downspread, 0, sizeof(old_downspread));

		core_link_read_dpcd(link, DP_DOWNSPREAD_CTRL,
				&old_downspread.raw, sizeof(old_downspread));

		new_downspread.raw = old_downspread.raw;

		new_downspread.bits.IGNORE_MSA_TIMING_PARAM =
				(stream->ignore_msa_timing_param) ? 1 : 0;

		if (new_downspread.raw != old_downspread.raw) {
			core_link_write_dpcd(link, DP_DOWNSPREAD_CTRL,
				&new_downspread.raw, sizeof(new_downspread));
		}

		dp_write_tiled_stream_enable_latch(link);
	} else {
		dm_helpers_mst_enable_stream_features(stream);
	}
}

static void log_vcp_x_y(const struct dc_link *link, struct fixed31_32 avg_time_slots_per_mtp)
{
	const uint32_t VCP_Y_PRECISION = 1000;
	uint64_t vcp_x, vcp_y;
	DC_LOGGER_INIT(link->ctx->logger);

	// Add 0.5*(1/VCP_Y_PRECISION) to round up to decimal precision
	avg_time_slots_per_mtp = dc_fixpt_add(
			avg_time_slots_per_mtp,
			dc_fixpt_from_fraction(
				1,
				2*VCP_Y_PRECISION));

	vcp_x = dc_fixpt_floor(
			avg_time_slots_per_mtp);
	vcp_y = dc_fixpt_floor(
			dc_fixpt_mul_int(
				dc_fixpt_sub_int(
					avg_time_slots_per_mtp,
					dc_fixpt_floor(
							avg_time_slots_per_mtp)),
				VCP_Y_PRECISION));


	if (link->type == dc_connection_mst_branch)
		DC_LOG_DP2("MST Update Payload: set_throttled_vcp_size slot X.Y for MST stream "
				"X: %llu "
				"Y: %llu/%d",
				vcp_x,
				vcp_y,
				VCP_Y_PRECISION);
	else
		DC_LOG_DP2("SST Update Payload: set_throttled_vcp_size slot X.Y for SST stream "
				"X: %llu "
				"Y: %llu/%d",
				vcp_x,
				vcp_y,
				VCP_Y_PRECISION);
}

static struct fixed31_32 get_pbn_per_slot(struct dc_stream_state *stream)
{
	struct fixed31_32 mbytes_per_sec;
	uint32_t link_rate_in_mbytes_per_sec = dp_link_bandwidth_kbps(stream->link,
			&stream->link->cur_link_settings);
	link_rate_in_mbytes_per_sec /= 8000; /* Kbits to MBytes */

	mbytes_per_sec = dc_fixpt_from_int(link_rate_in_mbytes_per_sec);

	return dc_fixpt_div_int(mbytes_per_sec, 54);
}

static struct fixed31_32 get_pbn_from_bw_in_kbps(uint64_t kbps)
{
	struct fixed31_32 peak_kbps;
	uint32_t numerator = 0;
	uint32_t denominator = 1;

	/*
	 * The 1.006 factor (margin 5300ppm + 300ppm ~ 0.6% as per spec) is not
	 * required when determining PBN/time slot utilization on the link between
	 * us and the branch, since that overhead is already accounted for in
	 * the get_pbn_per_slot function.
	 *
	 * The unit of 54/64Mbytes/sec is an arbitrary unit chosen based on
	 * common multiplier to render an integer PBN for all link rate/lane
	 * counts combinations
	 * calculate
	 * peak_kbps *= (64/54)
	 * peak_kbps /= (8 * 1000) convert to bytes
	 */

	numerator = 64;
	denominator = 54 * 8 * 1000;
	kbps *= numerator;
	peak_kbps = dc_fixpt_from_fraction(kbps, denominator);

	return peak_kbps;
}

static struct fixed31_32 get_pbn_from_timing(struct pipe_ctx *pipe_ctx)
{
	uint64_t kbps;
	enum dc_link_encoding_format link_encoding;

	if (dp_is_128b_132b_signal(pipe_ctx))
		link_encoding = DC_LINK_ENCODING_DP_128b_132b;
	else
		link_encoding = DC_LINK_ENCODING_DP_8b_10b;

	kbps = dc_bandwidth_in_kbps_from_timing(&pipe_ctx->stream->timing, link_encoding);
	return get_pbn_from_bw_in_kbps(kbps);
}


// TODO - DP2.0 Link: Fix get_lane_status to handle LTTPR offset (SST and MST)
static void get_lane_status(
	struct dc_link *link,
	uint32_t lane_count,
	union lane_status *status,
	union lane_align_status_updated *status_updated)
{
	unsigned int lane;
	uint8_t dpcd_buf[3] = {0};

	if (status == NULL || status_updated == NULL) {
		return;
	}

	core_link_read_dpcd(
			link,
			DP_LANE0_1_STATUS,
			dpcd_buf,
			sizeof(dpcd_buf));

	for (lane = 0; lane < lane_count; lane++) {
		status[lane].raw = dp_get_nibble_at_index(&dpcd_buf[0], lane);
	}

	status_updated->raw = dpcd_buf[2];
}

static bool poll_for_allocation_change_trigger(struct dc_link *link)
{
	/*
	 * wait for ACT handled
	 */
	int i;
	const int act_retries = 30;
	enum act_return_status result = ACT_FAILED;
	enum dc_connection_type display_connected = (link->type != dc_connection_none);
	union payload_table_update_status update_status = {0};
	union lane_status dpcd_lane_status[LANE_COUNT_DP_MAX];
	union lane_align_status_updated lane_status_updated;
	DC_LOGGER_INIT(link->ctx->logger);

	if (!display_connected || link->aux_access_disabled)
		return true;
	for (i = 0; i < act_retries; i++) {
		get_lane_status(link, link->cur_link_settings.lane_count, dpcd_lane_status, &lane_status_updated);

		if (!dp_is_cr_done(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_ch_eq_done(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_symbol_locked(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_interlane_aligned(lane_status_updated)) {
			DC_LOG_ERROR("SST Update Payload: Link loss occurred while "
					"polling for ACT handled.");
			result = ACT_LINK_LOST;
			break;
		}
		core_link_read_dpcd(
				link,
				DP_PAYLOAD_TABLE_UPDATE_STATUS,
				&update_status.raw,
				1);

		if (update_status.bits.ACT_HANDLED == 1) {
			DC_LOG_DP2("SST Update Payload: ACT handled by downstream.");
			result = ACT_SUCCESS;
			break;
		}

		fsleep(5000);
	}

	if (result == ACT_FAILED) {
		DC_LOG_ERROR("SST Update Payload: ACT still not handled after retries, "
				"continue on. Something is wrong with the branch.");
	}

	return (result == ACT_SUCCESS);
}

static void update_mst_stream_alloc_table(
	struct dc_link *link,
	struct stream_encoder *stream_enc,
	struct hpo_dp_stream_encoder *hpo_dp_stream_enc, // TODO: Rename stream_enc to dio_stream_enc?
	const struct dc_dp_mst_stream_allocation_table *proposed_table)
{
	struct link_mst_stream_allocation work_table[MAX_CONTROLLER_NUM] = { 0 };
	struct link_mst_stream_allocation *dc_alloc;

	int i;
	int j;

	/* if DRM proposed_table has more than one new payload */
	ASSERT(proposed_table->stream_count -
			link->mst_stream_alloc_table.stream_count < 2);

	/* copy proposed_table to link, add stream encoder */
	for (i = 0; i < proposed_table->stream_count; i++) {

		for (j = 0; j < link->mst_stream_alloc_table.stream_count; j++) {
			dc_alloc =
			&link->mst_stream_alloc_table.stream_allocations[j];

			if (dc_alloc->vcp_id ==
				proposed_table->stream_allocations[i].vcp_id) {

				work_table[i] = *dc_alloc;
				work_table[i].slot_count = proposed_table->stream_allocations[i].slot_count;
				break; /* exit j loop */
			}
		}

		/* new vcp_id */
		if (j == link->mst_stream_alloc_table.stream_count) {
			work_table[i].vcp_id =
				proposed_table->stream_allocations[i].vcp_id;
			work_table[i].slot_count =
				proposed_table->stream_allocations[i].slot_count;
			work_table[i].stream_enc = stream_enc;
			work_table[i].hpo_dp_stream_enc = hpo_dp_stream_enc;
		}
	}

	/* update link->mst_stream_alloc_table with work_table */
	link->mst_stream_alloc_table.stream_count =
			proposed_table->stream_count;
	for (i = 0; i < MAX_CONTROLLER_NUM; i++)
		link->mst_stream_alloc_table.stream_allocations[i] =
				work_table[i];
}

static void remove_stream_from_alloc_table(
		struct dc_link *link,
		struct stream_encoder *dio_stream_enc,
		struct hpo_dp_stream_encoder *hpo_dp_stream_enc)
{
	int i = 0;
	struct link_mst_stream_allocation_table *table =
			&link->mst_stream_alloc_table;

	if (hpo_dp_stream_enc) {
		for (; i < table->stream_count; i++)
			if (hpo_dp_stream_enc == table->stream_allocations[i].hpo_dp_stream_enc)
				break;
	} else {
		for (; i < table->stream_count; i++)
			if (dio_stream_enc == table->stream_allocations[i].stream_enc)
				break;
	}

	if (i < table->stream_count) {
		i++;
		for (; i < table->stream_count; i++)
			table->stream_allocations[i-1] = table->stream_allocations[i];
		memset(&table->stream_allocations[table->stream_count-1], 0,
				sizeof(struct link_mst_stream_allocation));
		table->stream_count--;
	}
}

static void print_mst_streams(struct dc_link *link)
{
	int i;

	DC_LOGGER_INIT(link->ctx->logger);

	DC_LOG_MST("%s stream_count: %d:\n",
		   __func__,
		   link->mst_stream_alloc_table.stream_count);

	for (i = 0; i < MAX_CONTROLLER_NUM; i++) {
		DC_LOG_MST("stream_enc[%d]: %p\n", i,
			   (void *) link->mst_stream_alloc_table.stream_allocations[i].stream_enc);
		DC_LOG_MST("stream[%d].hpo_dp_stream_enc: %p\n", i,
			   (void *) link->mst_stream_alloc_table.stream_allocations[i].hpo_dp_stream_enc);
		DC_LOG_MST("stream[%d].vcp_id: %d\n", i,
			   link->mst_stream_alloc_table.stream_allocations[i].vcp_id);
		DC_LOG_MST("stream[%d].slot_count: %d\n", i,
			   link->mst_stream_alloc_table.stream_allocations[i].slot_count);
	}
}

static enum dc_status deallocate_mst_payload(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp = dc_fixpt_from_int(0);
	bool mst_mode = (link->type == dc_connection_mst_branch);
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	const struct dc_link_settings empty_link_settings = {0};
	DC_LOGGER_INIT(link->ctx->logger);

	/* deallocate_mst_payload is called before disable link. When mode or
	 * disable/enable monitor, new stream is created which is not in link
	 * stream[] yet. For this, payload is not allocated yet, so de-alloc
	 * should not done. For new mode set, map_resources will get engine
	 * for new stream, so stream_enc->id should be validated until here.
	 */

	/* slot X.Y */
	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&empty_link_settings,
				avg_time_slots_per_mtp);

	if (mst_mode) {
		/* when link is in mst mode, reply on mst manager to remove
		 * payload
		 */
		if (dm_helpers_dp_mst_write_payload_allocation_table(
				stream->ctx,
				stream,
				&proposed_table,
				false))
			update_mst_stream_alloc_table(
					link,
					pipe_ctx->stream_res.stream_enc,
					pipe_ctx->stream_res.hpo_dp_stream_enc,
					&proposed_table);
		else
			DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
					pipe_ctx->pipe_idx);
	} else {
		/* when link is no longer in mst mode (mst hub unplugged),
		 * remove payload with default dc logic
		 */
		remove_stream_from_alloc_table(link, pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc);
	}

	print_mst_streams(link);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_DEBUG("Unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	if (mst_mode)
		dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	dm_helpers_dp_mst_update_mst_mgr_for_deallocation(
			stream->ctx,
			stream);

	return DC_OK;
}

/* convert link_mst_stream_alloc_table to dm dp_mst_stream_alloc_table
 * because stream_encoder is not exposed to dm
 */
static enum dc_status allocate_mst_payload(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	enum act_return_status ret;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* enable_link_dp_mst already check link->enabled_stream_count
	 * and stream is in link->stream[]. This is called during set mode,
	 * stream_enc is available.
	 */

	/* get calculate VC payload for stream: stream_alloc */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
		stream->ctx,
		stream,
		&proposed_table,
		true))
		update_mst_stream_alloc_table(
					link,
					pipe_ctx->stream_res.stream_enc,
					pipe_ctx->stream_res.hpo_dp_stream_enc,
					&proposed_table);
	else
		DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
				pipe_ctx->pipe_idx);

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* program DP source TX for payload */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link,
			&pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* send down message */
	ret = dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	if (ret != ACT_LINK_LOST)
		dm_helpers_dp_mst_send_payload_allocation(
				stream->ctx,
				stream);

	/* slot X.Y for only current stream */
	pbn_per_slot = get_pbn_per_slot(stream);
	if (pbn_per_slot.value == 0) {
		DC_LOG_ERROR("Failure: pbn_per_slot==0 not allowed. Cannot continue, returning DC_UNSUPPORTED_VALUE.\n");
		return DC_UNSUPPORTED_VALUE;
	}
	pbn = get_pbn_from_timing(pipe_ctx);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	log_vcp_x_y(link, avg_time_slots_per_mtp);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	return DC_OK;
}

struct fixed31_32 link_calculate_sst_avg_time_slots_per_mtp(
		const struct dc_stream_state *stream,
		const struct dc_link *link)
{
	struct fixed31_32 link_bw_effective =
			dc_fixpt_from_int(
					dp_link_bandwidth_kbps(link, &link->cur_link_settings));
	struct fixed31_32 timeslot_bw_effective =
			dc_fixpt_div_int(link_bw_effective, MAX_MTP_SLOT_COUNT);
	struct fixed31_32 timing_bw =
			dc_fixpt_from_int(
					dc_bandwidth_in_kbps_from_timing(&stream->timing,
							dc_link_get_highest_encoding_format(link)));
	struct fixed31_32 avg_time_slots_per_mtp =
			dc_fixpt_div(timing_bw, timeslot_bw_effective);

	return avg_time_slots_per_mtp;
}


static bool write_128b_132b_sst_payload_allocation_table(
		const struct dc_stream_state *stream,
		struct dc_link *link,
		struct link_mst_stream_allocation_table *proposed_table,
		bool allocate)
{
	const uint8_t vc_id = 1; /// VC ID always 1 for SST
	const uint8_t start_time_slot = 0; /// Always start at time slot 0 for SST
	bool result = false;
	uint8_t req_slot_count = 0;
	struct fixed31_32 avg_time_slots_per_mtp = { 0 };
	union payload_table_update_status update_status = { 0 };
	const uint32_t max_retries = 30;
	uint32_t retries = 0;
	enum dc_connection_type display_connected = (link->type != dc_connection_none);
	DC_LOGGER_INIT(link->ctx->logger);

	if (allocate)	{
		avg_time_slots_per_mtp = link_calculate_sst_avg_time_slots_per_mtp(stream, link);
		req_slot_count = dc_fixpt_ceil(avg_time_slots_per_mtp);
		/// Validation should filter out modes that exceed link BW
		ASSERT(req_slot_count <= MAX_MTP_SLOT_COUNT);
		if (req_slot_count > MAX_MTP_SLOT_COUNT)
			return false;
	} else {
		/// Leave req_slot_count = 0 if allocate is false.
	}

	proposed_table->stream_count = 1; /// Always 1 stream for SST
	proposed_table->stream_allocations[0].slot_count = req_slot_count;
	proposed_table->stream_allocations[0].vcp_id = vc_id;

	if (!display_connected || link->aux_access_disabled)
		return true;

	/// Write DPCD 2C0 = 1 to start updating
	update_status.bits.VC_PAYLOAD_TABLE_UPDATED = 1;
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_TABLE_UPDATE_STATUS,
			&update_status.raw,
			1);

	/// Program the changes in DPCD 1C0 - 1C2
	ASSERT(vc_id == 1);
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_SET,
			&vc_id,
			1);

	ASSERT(start_time_slot == 0);
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_START_TIME_SLOT,
			&start_time_slot,
			1);

	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_TIME_SLOT_COUNT,
			&req_slot_count,
			1);

	/// Poll till DPCD 2C0 read 1
	/// Try for at least 150ms (30 retries, with 5ms delay after each attempt)

	while (retries < max_retries) {
		if (core_link_read_dpcd(
				link,
				DP_PAYLOAD_TABLE_UPDATE_STATUS,
				&update_status.raw,
				1) == DC_OK) {
			if (update_status.bits.VC_PAYLOAD_TABLE_UPDATED == 1) {
				DC_LOG_DP2("SST Update Payload: downstream payload table updated.");
				result = true;
				break;
			}
		} else {
			union dpcd_rev dpcdRev = {0};

			if (core_link_read_dpcd(
					link,
					DP_DPCD_REV,
					&dpcdRev.raw,
					1) != DC_OK) {
				DC_LOG_ERROR("SST Update Payload: Unable to read DPCD revision "
						"of sink while polling payload table "
						"updated status bit.");
				break;
			}
		}
		retries++;
		fsleep(5000);
	}

	if (!result && retries == max_retries) {
		DC_LOG_ERROR("SST Update Payload: Payload table not updated after retries, "
				"continue on. Something is wrong with the branch.");
		// TODO - DP2.0 Payload: Read and log the payload table from downstream branch
	}

	return result;
}

/*
 * Payload allocation/deallocation for SST introduced in DP2.0
 */
static enum dc_status update_sst_payload(struct pipe_ctx *pipe_ctx,
						 bool allocate)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct link_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp;
	const struct dc_link_settings empty_link_settings = {0};
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* slot X.Y for SST payload deallocate */
	if (!allocate) {
		avg_time_slots_per_mtp = dc_fixpt_from_int(0);

		log_vcp_x_y(link, avg_time_slots_per_mtp);

		if (link_hwss->ext.set_throttled_vcp_size)
			link_hwss->ext.set_throttled_vcp_size(pipe_ctx,
					avg_time_slots_per_mtp);
		if (link_hwss->ext.set_hblank_min_symbol_width)
			link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
					&empty_link_settings,
					avg_time_slots_per_mtp);
	}

	/* calculate VC payload and update branch with new payload allocation table*/
	if (!write_128b_132b_sst_payload_allocation_table(
			stream,
			link,
			&proposed_table,
			allocate)) {
		DC_LOG_ERROR("SST Update Payload: Failed to update "
						"allocation table for "
						"pipe idx: %d\n",
						pipe_ctx->pipe_idx);
		return DC_FAIL_DP_PAYLOAD_ALLOCATION;
	}

	proposed_table.stream_allocations[0].hpo_dp_stream_enc = pipe_ctx->stream_res.hpo_dp_stream_enc;

	ASSERT(proposed_table.stream_count == 1);

	//TODO - DP2.0 Logging: Instead of hpo_dp_stream_enc pointer, log instance id
	DC_LOG_DP2("SST Update Payload: hpo_dp_stream_enc: %p      "
		"vcp_id: %d      "
		"slot_count: %d\n",
		(void *) proposed_table.stream_allocations[0].hpo_dp_stream_enc,
		proposed_table.stream_allocations[0].vcp_id,
		proposed_table.stream_allocations[0].slot_count);

	/* program DP source TX for payload */
	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&proposed_table);

	/* poll for ACT handled */
	if (!poll_for_allocation_change_trigger(link)) {
		// Failures will result in blackscreen and errors logged
		BREAK_TO_DEBUGGER();
	}

	/* slot X.Y for SST payload allocate */
	if (allocate && link_dp_get_encoding_format(&link->cur_link_settings) ==
			DP_128b_132b_ENCODING) {
		avg_time_slots_per_mtp = link_calculate_sst_avg_time_slots_per_mtp(stream, link);

		log_vcp_x_y(link, avg_time_slots_per_mtp);

		if (link_hwss->ext.set_throttled_vcp_size)
			link_hwss->ext.set_throttled_vcp_size(pipe_ctx,
					avg_time_slots_per_mtp);
		if (link_hwss->ext.set_hblank_min_symbol_width)
			link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
					&link->cur_link_settings,
					avg_time_slots_per_mtp);
	}

	/* Always return DC_OK.
	 * If part of sequence fails, log failure(s) and show blackscreen
	 */
	return DC_OK;
}

enum dc_status link_reduce_mst_payload(struct pipe_ctx *pipe_ctx, uint32_t bw_in_kbps)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* decrease throttled vcp size */
	pbn_per_slot = get_pbn_per_slot(stream);
	pbn = get_pbn_from_bw_in_kbps(bw_in_kbps);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	/* send ALLOCATE_PAYLOAD sideband message with updated pbn */
	dm_helpers_dp_mst_send_payload_allocation(
			stream->ctx,
			stream);

	/* notify immediate branch device table update */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
			stream->ctx,
			stream,
			&proposed_table,
			true)) {
		/* update mst stream allocation table software state */
		update_mst_stream_alloc_table(
				link,
				pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc,
				&proposed_table);
	} else {
		DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
				pipe_ctx->pipe_idx);
	}

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* poll for immediate branch device ACT handled */
	dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	return DC_OK;
}

enum dc_status link_increase_mst_payload(struct pipe_ctx *pipe_ctx, uint32_t bw_in_kbps)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	enum act_return_status ret;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* notify immediate branch device table update */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
				stream->ctx,
				stream,
				&proposed_table,
				true)) {
		/* update mst stream allocation table software state */
		update_mst_stream_alloc_table(
				link,
				pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc,
				&proposed_table);
	}

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* poll for immediate branch device ACT handled */
	ret = dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	if (ret != ACT_LINK_LOST) {
		/* send ALLOCATE_PAYLOAD sideband message with updated pbn */
		dm_helpers_dp_mst_send_payload_allocation(
				stream->ctx,
				stream);
	}

	/* increase throttled vcp size */
	pbn = get_pbn_from_bw_in_kbps(bw_in_kbps);
	pbn_per_slot = get_pbn_per_slot(stream);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	return DC_OK;
}

static void disable_link_dp(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	struct dc_link_settings link_settings = link->cur_link_settings;

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST &&
			link->mst_stream_alloc_table.stream_count > 0)
		/* disable MST link only when last vc payload is deallocated */
		return;

	dp_disable_link_phy(link, link_res, signal);

	if (link->connector_signal == SIGNAL_TYPE_EDP) {
		if (!link->skip_implict_edp_power_control)
			link->dc->hwss.edp_power_control(link, false);
	}

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST && link->sink_count == 0)
		/* set the sink to SST mode after disabling the link */
		enable_mst_on_sink(link, false);

	if (link_dp_get_encoding_format(&link_settings) ==
			DP_8b_10b_ENCODING) {
		dp_set_fec_enable(link, link_res, false);
		dp_set_fec_ready(link, link_res, false);
	}
}

static void disable_link(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	/*
	 * APPLE5K: never disable the tiled panel's PHY/transmitter while the
	 * panel is native-preserved OR a compat-boot arm is in progress.
	 * dp_disable_link_phy()/disable_link_output() (and the eDP VDD power-off
	 * inside disable_link_dp) re-latch the TCON to compat -- they reset a
	 * native panel and reset the arm latch before the combined enable. Ride
	 * the EFI-trained link instead (the modeset only reprograms the OTG).
	 */
	if (dc_link_apple5k_protect(link)) {
		DC_LOGGER_INIT(link->ctx->logger);
		DC_LOG_INFO("APPLE5K: skip disable_link for tiled link[%u] (protect: preserve/arming) -- no PHY/output disable, latch survives\n",
			    link->link_index);
		return;
	}

	if (dc_is_dp_signal(signal)) {
		disable_link_dp(link, link_res, signal);
	} else if (signal == SIGNAL_TYPE_VIRTUAL) {
		link->dc->hwss.disable_link_output(link, link_res, SIGNAL_TYPE_DISPLAY_PORT);
	} else {
		link->dc->hwss.disable_link_output(link, link_res, signal);
	}

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		/* MST disable link only when no stream use the link */
		if (link->mst_stream_alloc_table.stream_count <= 0)
			link->link_status.link_active = false;
	} else {
		link->link_status.link_active = false;
	}
}

static void enable_link_hdmi(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	enum dc_color_depth display_color_depth;
	enum engine_id eng_id;
	struct ext_hdmi_settings settings = {0};
	bool is_over_340mhz = false;
	bool is_vga_mode = (stream->timing.h_addressable == 640)
			&& (stream->timing.v_addressable == 480);
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);

	if (stream->phy_pix_clk == 0)
		stream->phy_pix_clk = stream->timing.pix_clk_100hz / 10;
	if (stream->phy_pix_clk > 340000)
		is_over_340mhz = true;
	if (dc_is_tmds_signal(stream->signal) && stream->phy_pix_clk > 6000000UL) {
		ASSERT(false);
		return;
	}

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		unsigned short masked_chip_caps = pipe_ctx->stream->link->chip_caps &
				AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK;
		if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_TISN65DP159RSBT) {
			/* DP159, Retimer settings */
			eng_id = pipe_ctx->stream_res.stream_enc->id;

			if (get_ext_hdmi_settings(pipe_ctx, eng_id, &settings)) {
				write_i2c_retimer_setting(pipe_ctx,
						is_vga_mode, is_over_340mhz, &settings);
			} else {
				write_i2c_default_retimer_setting(pipe_ctx,
						is_vga_mode, is_over_340mhz);
			}
		} else if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_PI3EQX1204) {
			/* PI3EQX1204, Redriver settings */
			write_i2c_redriver_setting(pipe_ctx, is_over_340mhz);
		}
	}

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
		write_scdc_data(
			stream->link->ddc,
			stream->phy_pix_clk,
			stream->timing.flags.LTE_340MCSC_SCRAMBLE);

	memset(&stream->link->cur_link_settings, 0,
			sizeof(struct dc_link_settings));

	display_color_depth = stream->timing.display_color_depth;
	if (stream->timing.pixel_encoding == PIXEL_ENCODING_YCBCR422)
		display_color_depth = COLOR_DEPTH_888;

	/* We need to enable stream encoder for TMDS first to apply 1/4 TMDS
	 * character clock in case that beyond 340MHz.
	 */
	if (dc_is_hdmi_tmds_signal(pipe_ctx->stream->signal) || dc_is_dvi_signal(pipe_ctx->stream->signal))
		link_hwss->setup_stream_encoder(pipe_ctx);

	dc->hwss.enable_tmds_link_output(
			link,
			&pipe_ctx->link_res,
			pipe_ctx->stream->signal,
			pipe_ctx->clock_source->id,
			display_color_depth,
			stream->phy_pix_clk);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
		read_scdc_data(link->ddc);
}

static enum dc_status enable_link_dp(struct dc_state *state,
				     struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	enum dc_status status;
	bool skip_video_pattern;
	struct dc_link *link = stream->link;
	const struct dc_link_settings *link_settings =
			&pipe_ctx->link_config.dp_link_settings;
	bool fec_enable;
	int i;
	bool apply_seamless_boot_optimization = false;
	uint32_t bl_oled_enable_delay = 50; // in ms
	uint32_t post_oui_delay = 30; // 30ms
	/* Reduce link bandwidth between failed link training attempts. */
	bool do_fallback = false;
	int lt_attempts = LINK_TRAINING_ATTEMPTS;

	DC_LOGGER_INIT(link->ctx->logger);

	// Increase retry count if attempting DP1.x on FIXED_VS link
	if (((link->chip_caps & AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK) == AMD_EXT_DISPLAY_PATH_CAPS__DP_FIXED_VS_EN) &&
			link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		lt_attempts = 10;

	// check for seamless boot
	for (i = 0; i < state->stream_count; i++) {
		if (state->streams[i]->apply_seamless_boot_optimization) {
			apply_seamless_boot_optimization = true;
			break;
		}
	}

	/* Train with fallback when enabling DPIA link. Conventional links are
	 * trained with fallback during sink detection.
	 */
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA &&
			!link->dc->config.enable_dpia_pre_training)
		do_fallback = true;

	/*
	 * Temporary w/a to get DP2.0 link rates to work with SST.
	 * TODO DP2.0 - Workaround: Remove w/a if and when the issue is resolved.
	 */
	if (link_dp_get_encoding_format(link_settings) == DP_128b_132b_ENCODING &&
			pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT &&
			link->dc->debug.set_mst_en_for_sst) {
		enable_mst_on_sink(link, true);
	} else if (link->dpcd_caps.is_mst_capable &&
		pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT) {
		/* disable mst on sink */
		enable_mst_on_sink(link, false);
	}

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP) {
		/*in case it is not on*/
		if (!link->dc->config.edp_no_power_sequencing)
			link->dc->hwss.edp_power_control(link, true);
		link->dc->hwss.edp_wait_for_hpd_ready(link, true);

		/*
		 * Apple tiled root, compat boot: the panel was armed by the
		 * firmware handshake at detection (0->1 verified latch edge,
		 * link_apple_5k_arm_handshake()). The boot-state teardown and
		 * panel VDD cycle since then may have dropped it -- verify
		 * here, right before link training with both tiles still
		 * blanked, and re-run the handshake if the panel is no longer
		 * armed (latch reads 1 AND the EDID presents the tiled
		 * identity, product LSB & 3 == 2). Skipped entirely on a
		 * preserved native boot.
		 */
		if (dc_link_has_tiled_root_panel_patch(link) &&
		    !dc_link_apple5k_preserve(link) &&
		    link->apple5k_compat_arm_enable) {
			uint8_t latch = 0;
			uint8_t hdr[16] = { 0 };
			uint8_t edid_offset = 0;
			bool armed;
			enum dc_status hs_status = DC_OK;

			core_link_read_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH,
					    &latch, sizeof(latch));
			link_query_ddc_data(link->ddc, 0x50, &edid_offset, 1,
					    hdr, sizeof(hdr));
			armed = latch == 1 && (hdr[0xa] & 3) == 2;
			if (!armed)
				hs_status = link_apple_5k_arm_handshake(link);
			DC_LOG_INFO("APPLE5K: pre-training arm check link[%u] latch=0x%02x edid_lsb=0x%02x -> %s (handshake status=%d)\n",
				    link->link_index, latch, hdr[0xa],
				    armed ? "already armed" : "re-ran handshake",
				    hs_status);

			/*
			 * Either way the latch is now armed and the combined
			 * enable is imminent: hold VDD on until it resolves,
			 * and forbid sink D3 on the pair (suspected TCON
			 * fault trigger) for the rest of the armed window.
			 */
			if (armed || hs_status == DC_OK) {
				link->apple5k_arming = true;
				link->wa_flags.dp_keep_receiver_powered = true;
				if (link->tiled_peer) {
					link->tiled_peer->wa_flags.dp_keep_receiver_powered = true;
					link->tiled_peer->skip_fallback_on_link_loss = true;
				}
			}

			/* Diagnostic: panel mode state right after the arm. */
			tiled_pair_sample_native_latch(link, "post re-arm");
		}
	}

	if (link_dp_get_encoding_format(link_settings) == DP_128b_132b_ENCODING) {
		/* TODO - DP2.0 HW: calculate 32 symbol clock for HPO encoder */
	} else {
		pipe_ctx->stream_res.pix_clk_params.requested_sym_clk =
				link_settings->link_rate * LINK_RATE_REF_FREQ_IN_KHZ;
		if (state->clk_mgr && !apply_seamless_boot_optimization)
			state->clk_mgr->funcs->update_clocks(state->clk_mgr,
					state, false);
	}

	// during mode switch we do DP_SET_POWER off then on, and OUI is lost
	dpcd_set_source_specific_data(link);
	if (link->dpcd_sink_ext_caps.raw != 0) {
		post_oui_delay += link->panel_config.pps.extra_post_OUI_ms;
		msleep(post_oui_delay);
	}

	// similarly, mode switch can cause loss of cable ID
	dpcd_write_cable_id_to_dprx(link);

	skip_video_pattern = true;

	if (link_settings->link_rate == LINK_RATE_LOW)
		skip_video_pattern = false;

	if (stream->sink_patches.oled_optimize_display_on)
		set_default_brightness_aux(link);

	if (perform_link_training_with_retries(link_settings,
					       skip_video_pattern,
					       lt_attempts,
					       pipe_ctx,
					       pipe_ctx->stream->signal,
					       do_fallback)) {
		status = DC_OK;
	} else {
		status = DC_FAIL_DP_LINK_TRAINING;
	}

	if (link->preferred_training_settings.fec_enable)
		fec_enable = *link->preferred_training_settings.fec_enable;
	else
		fec_enable = true;

	if (link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		dp_set_fec_enable(link, &pipe_ctx->link_res, fec_enable);

	// during mode set we do DP_SET_POWER off then on, aux writes are lost
	if (link->dpcd_sink_ext_caps.bits.oled == 1 ||
		link->dpcd_sink_ext_caps.bits.sdr_aux_backlight_control == 1 ||
		link->dpcd_sink_ext_caps.bits.hdr_aux_backlight_control == 1) {
		if (!stream->sink_patches.oled_optimize_display_on) {
			set_default_brightness_aux(link);
			if (link->dpcd_sink_ext_caps.bits.oled == 1)
				msleep(bl_oled_enable_delay);
			edp_backlight_enable_aux(link, true);
		} else {
			edp_backlight_enable_aux(link, true);
		}
	}

	return status;
}

static enum dc_status enable_link_edp(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	return enable_link_dp(state, pipe_ctx);
}

static void enable_link_lvds(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc *dc = stream->ctx->dc;

	if (stream->phy_pix_clk == 0)
		stream->phy_pix_clk = stream->timing.pix_clk_100hz / 10;

	memset(&stream->link->cur_link_settings, 0,
			sizeof(struct dc_link_settings));
	dc->hwss.enable_lvds_link_output(
			link,
			&pipe_ctx->link_res,
			pipe_ctx->clock_source->id,
			stream->phy_pix_clk);

}

static enum dc_status enable_link_dp_mst(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;
	unsigned char mstm_cntl = 0;

	/* sink signal type after MST branch is MST. Multiple MST sinks
	 * share one link. Link DP PHY is enable or training only once.
	 */
	if (link->link_status.link_active)
		return DC_OK;

	/* clear payload table */
	core_link_read_dpcd(link, DP_MSTM_CTRL, &mstm_cntl, 1);
	if (mstm_cntl & DP_MST_EN)
		dm_helpers_dp_mst_clear_payload_allocation_table(link->ctx, link);

	/* to make sure the pending down rep can be processed
	 * before enabling the link
	 */
	dm_helpers_dp_mst_poll_pending_down_reply(link->ctx, link);

	/* set the sink to MST mode before enabling the link */
	enable_mst_on_sink(link, true);

	return enable_link_dp(state, pipe_ctx);
}

static enum dc_status enable_link_analog(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;

	link->dc->hwss.enable_analog_link_output(
		link, pipe_ctx->stream->timing.pix_clk_100hz);

	return DC_OK;
}

static enum dc_status enable_link_virtual(struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;

	link->dc->hwss.enable_dp_link_output(link,
			&pipe_ctx->link_res,
			SIGNAL_TYPE_DISPLAY_PORT,
			pipe_ctx->clock_source->id,
			&pipe_ctx->link_config.dp_link_settings);
	return DC_OK;
}

static enum dc_status enable_link(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	enum dc_status status = DC_ERROR_UNEXPECTED;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = NULL;

	if (stream == NULL)
		return DC_ERROR_UNEXPECTED;
	link = stream->link;

	/* There's some scenarios where driver is unloaded with display
	 * still enabled. When driver is reloaded, it may cause a display
	 * to not light up if there is a mismatch between old and new
	 * link settings. Need to call disable first before enabling at
	 * new link settings.
	 */
	if (link->link_status.link_active)
		disable_link(link, &pipe_ctx->link_res, pipe_ctx->stream->signal);

	switch (pipe_ctx->stream->signal) {
	case SIGNAL_TYPE_DISPLAY_PORT:
		status = enable_link_dp(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_EDP:
		status = enable_link_edp(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_DISPLAY_PORT_MST:
		status = enable_link_dp_mst(state, pipe_ctx);
		msleep(200);
		break;
	case SIGNAL_TYPE_DVI_SINGLE_LINK:
	case SIGNAL_TYPE_DVI_DUAL_LINK:
	case SIGNAL_TYPE_HDMI_TYPE_A:
		enable_link_hdmi(pipe_ctx);
		status = DC_OK;
		break;
	case SIGNAL_TYPE_LVDS:
		enable_link_lvds(pipe_ctx);
		status = DC_OK;
		break;
	case SIGNAL_TYPE_RGB:
		status = enable_link_analog(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_VIRTUAL:
		status = enable_link_virtual(pipe_ctx);
		break;
	default:
		break;
	}

	if (status == DC_OK) {
		pipe_ctx->stream->link->link_status.link_active = true;
	}

	return status;
}

static bool allocate_usb4_bandwidth_for_stream(struct dc_stream_state *stream, int bw)
{
	struct dc_link *link = stream->sink->link;
	int req_bw = bw;

	DC_LOGGER_INIT(link->ctx->logger);

	if (!link->dpia_bw_alloc_config.bw_alloc_enabled)
		return false;

	if (stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		int sink_index = 0;
		int i = 0;

		for (i = 0; i < link->sink_count; i++) {
			if (link->remote_sinks[i] == NULL)
				continue;

			if (stream->sink->sink_id != link->remote_sinks[i]->sink_id)
				req_bw += link->dpia_bw_alloc_config.remote_sink_req_bw[i];
			else
				sink_index = i;
		}

		link->dpia_bw_alloc_config.remote_sink_req_bw[sink_index] = bw;
	}

	link->dpia_bw_alloc_config.dp_overhead = link_dpia_get_dp_overhead(link);
	req_bw += link->dpia_bw_alloc_config.dp_overhead;

	link_dp_dpia_allocate_usb4_bandwidth_for_stream(link, req_bw);

	if (stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		int i = 0;

		for (i = 0; i < link->sink_count; i++) {
			if (link->remote_sinks[i] == NULL)
				continue;
			DC_LOG_DEBUG("%s, remote_sink=%s, request_bw=%d\n", __func__,
					(const char *)(&link->remote_sinks[i]->edid_caps.display_name[0]),
					link->dpia_bw_alloc_config.remote_sink_req_bw[i]);
		}
	}

	return true;
}

static bool allocate_usb4_bandwidth(struct dc_stream_state *stream)
{
	bool ret;

	int bw = dc_bandwidth_in_kbps_from_timing(&stream->timing,
			dc_link_get_highest_encoding_format(stream->sink->link));

	ret = allocate_usb4_bandwidth_for_stream(stream, bw);

	return ret;
}

static bool deallocate_usb4_bandwidth(struct dc_stream_state *stream)
{
	bool ret;

	ret = allocate_usb4_bandwidth_for_stream(stream, 0);

	return ret;
}

void link_set_dpms_off(struct pipe_ctx *pipe_ctx)
{
	struct dc  *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->sink->link;
	struct vpg *vpg = pipe_ctx->stream_res.stream_enc->vpg;
	enum dp_panel_mode panel_mode_dp = dp_get_panel_mode(link);

	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	ASSERT(is_master_pipe_for_link(link, pipe_ctx));

	if (dp_is_128b_132b_signal(pipe_ctx))
		vpg = pipe_ctx->stream_res.hpo_dp_stream_enc->vpg;
	if (dc_is_virtual_signal(pipe_ctx->stream->signal))
		return;

	if (pipe_ctx->stream->sink) {
		if (pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_VIRTUAL &&
			pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_NONE) {
			DC_LOG_DC("%s pipe_ctx dispname=%s signal=%x link=%d sink_count=%d\n", __func__,
			pipe_ctx->stream->sink->edid_caps.display_name,
			pipe_ctx->stream->signal, link->link_index, link->sink_count);
		}
	}

	if (!pipe_ctx->stream->sink->edid_caps.panel_patch.skip_avmute) {
		if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
			set_avmute(pipe_ctx, true);
	}

	if (dc_link_has_tiled_root_panel_patch(link) ||
	    dc_link_has_tiled_slave_panel_patch(link))
		tiled_pair_sample_via_root(link,
			link->connector_signal == SIGNAL_TYPE_EDP ?
			"dpms-off entry root" : "dpms-off entry slave");

	dc->hwss.disable_audio_stream(pipe_ctx);

	update_psp_stream_config(pipe_ctx, true);
	/*
	 * APPLE5K: dce110_blank_stream() does the dp_blank that re-latches the
	 * tiled TCON to compat. Skip it while preserving native or arming (the
	 * OTG is still freed by disable_stream below); the runtime display-off
	 * backlight path is handled separately via dc_apple5k_tiled_panel_blank.
	 */
	if (!dc_link_apple5k_protect(link))
		dc->hwss.blank_stream(pipe_ctx);

	if (pipe_ctx->link_config.dp_tunnel_settings.should_use_dp_bw_allocation)
		deallocate_usb4_bandwidth(pipe_ctx->stream);

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST)
		deallocate_mst_payload(pipe_ctx);
	else if (dc_is_dp_sst_signal(pipe_ctx->stream->signal) &&
			dp_is_128b_132b_signal(pipe_ctx))
		update_sst_payload(pipe_ctx, false);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		struct ext_hdmi_settings settings = {0};
		enum engine_id eng_id = pipe_ctx->stream_res.stream_enc->id;

		unsigned short masked_chip_caps = link->chip_caps &
				AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK;
		//Need to inform that sink is going to use legacy HDMI mode.
		write_scdc_data(
			link->ddc,
			165000,//vbios only handles 165Mhz.
			false);
		if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_TISN65DP159RSBT) {
			/* DP159, Retimer settings */
			if (get_ext_hdmi_settings(pipe_ctx, eng_id, &settings))
				write_i2c_retimer_setting(pipe_ctx,
						false, false, &settings);
			else
				write_i2c_default_retimer_setting(pipe_ctx,
						false, false);
		} else if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_PI3EQX1204) {
			/* PI3EQX1204, Redriver settings */
			write_i2c_redriver_setting(pipe_ctx, false);
		}
	}

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT &&
			!dp_is_128b_132b_signal(pipe_ctx)) {

		/* In DP1.x SST mode, our encoder will go to TPS1
		 * when link is on but stream is off.
		 * Disabling link before stream will avoid exposing TPS1 pattern
		 * during the disable sequence as it will confuse some receivers
		 * state machine.
		 * In DP2 or MST mode, our encoder will stay video active
		 */
		disable_link(pipe_ctx->stream->link, &pipe_ctx->link_res, pipe_ctx->stream->signal);
		dc->hwss.disable_stream(pipe_ctx);
	} else {
		dc->hwss.disable_stream(pipe_ctx);
		disable_link(pipe_ctx->stream->link, &pipe_ctx->link_res, pipe_ctx->stream->signal);
	}
	edp_set_panel_assr(link, pipe_ctx, &panel_mode_dp, false);

	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal))
			link_set_dsc_enable(pipe_ctx, false);
	}
	if (dp_is_128b_132b_signal(pipe_ctx)) {
		if (pipe_ctx->stream_res.tg->funcs->set_out_mux)
			pipe_ctx->stream_res.tg->funcs->set_out_mux(pipe_ctx->stream_res.tg, OUT_MUX_DIO);
	}

	if (vpg && vpg->funcs->vpg_powerdown)
		vpg->funcs->vpg_powerdown(vpg);

	/* for psp not exist case */
	if (link->connector_signal == SIGNAL_TYPE_EDP && dc->debug.psp_disabled_wa) {
		/* reset internal save state to default since eDP is  off */
		enum dp_panel_mode panel_mode = dp_get_panel_mode(pipe_ctx->stream->link);
		/* since current psp not loaded, we need to reset it to default */
		link->panel_mode = panel_mode;
	}

	if (dc_link_has_tiled_root_panel_patch(link) ||
	    dc_link_has_tiled_slave_panel_patch(link))
		tiled_pair_sample_via_root(link,
			link->connector_signal == SIGNAL_TYPE_EDP ?
			"dpms-off exit root" : "dpms-off exit slave");
}

void link_set_dpms_on(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->sink->link;
	enum dc_status status;
	struct link_encoder *link_enc = pipe_ctx->link_res.dio_link_enc;
	enum otg_out_mux_dest otg_out_dest = OUT_MUX_DIO;
	struct vpg *vpg = pipe_ctx->stream_res.stream_enc->vpg;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	struct pipe_ctx *tiled_peer_pipe;
	bool apply_edp_fast_boot_optimization =
		pipe_ctx->stream->apply_edp_fast_boot_optimization;

	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	ASSERT(is_master_pipe_for_link(link, pipe_ctx));

	if (dp_is_128b_132b_signal(pipe_ctx))
		vpg = pipe_ctx->stream_res.hpo_dp_stream_enc->vpg;
	if (dc_is_virtual_signal(pipe_ctx->stream->signal))
		return;

	if (pipe_ctx->stream->sink) {
		if (pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_VIRTUAL &&
			pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_NONE) {
			DC_LOG_DC("%s pipe_ctx dispname=%s signal=%x link=%d sink_count=%d\n", __func__,
			pipe_ctx->stream->sink->edid_caps.display_name,
			pipe_ctx->stream->signal,
			link->link_index,
			link->sink_count);
		}
	}

	if (!dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(link);
	ASSERT(link_enc);

	if (!dc_is_virtual_signal(pipe_ctx->stream->signal)
			&& !dp_is_128b_132b_signal(pipe_ctx)) {
		if (link_enc)
			link_enc->funcs->setup(
				link_enc,
				pipe_ctx->stream->signal);
	}

	pipe_ctx->stream->link->link_state_valid = true;

	if (pipe_ctx->stream_res.tg->funcs->set_out_mux) {
		if (dp_is_128b_132b_signal(pipe_ctx))
			otg_out_dest = OUT_MUX_HPO_DP;
		else
			otg_out_dest = OUT_MUX_DIO;
		pipe_ctx->stream_res.tg->funcs->set_out_mux(pipe_ctx->stream_res.tg, otg_out_dest);
	}

	link_hwss->setup_stream_attribute(pipe_ctx);

	pipe_ctx->stream->apply_edp_fast_boot_optimization = false;

	// Enable VPG before building infoframe
	if (vpg && vpg->funcs->vpg_poweron)
		vpg->funcs->vpg_poweron(vpg);

	resource_build_info_frame(pipe_ctx);
	dc->hwss.update_info_frame(pipe_ctx);

	if (dc_is_dp_signal(pipe_ctx->stream->signal))
		dp_trace_source_sequence(link, DPCD_SOURCE_SEQ_AFTER_UPDATE_INFO_FRAME);

	/* Do not touch link on seamless boot optimization. */
	if (pipe_ctx->stream->apply_seamless_boot_optimization) {
		pipe_ctx->stream->dpms_off = false;

		/* Still enable stream features & audio on seamless boot for DP external displays */
		if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT) {
			enable_stream_features(pipe_ctx);
			dc->hwss.enable_audio_stream(pipe_ctx);
		}

		update_psp_stream_config(pipe_ctx, false);
		return;
	}

	/* eDP lit up by bios already, no need to enable again. */
	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP &&
				apply_edp_fast_boot_optimization &&
				!pipe_ctx->stream->timing.flags.DSC &&
				!pipe_ctx->next_odm_pipe) {
		pipe_ctx->stream->dpms_off = false;
		update_psp_stream_config(pipe_ctx, false);

		if (link->is_dds) {
			uint32_t post_oui_delay = 30; // 30ms

			dpcd_set_source_specific_data(link);
			msleep(post_oui_delay);
		}

		return;
	}

	if (pipe_ctx->stream->dpms_off)
		return;

	/* For Dp tunneling link, a pending HPD means that we have a race condition between processing
	 * current link and processing the pending HPD. If we enable the link now, we may end up with a
	 * link that is not actually connected to a sink. So we skip enabling the link in this case.
	 */
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA && link->is_hpd_pending) {
		DC_LOG_DEBUG("%s, Link%d HPD is pending, not enable it.\n", __func__, link->link_index);
		return;
	}

	/* Have to setup DSC before DIG FE and BE are connected (which happens before the
	 * link training). This is to make sure the bandwidth sent to DIG BE won't be
	 * bigger than what the link and/or DIG BE can handle. VBID[6]/CompressedStream_flag
	 * will be automatically set at a later time when the video is enabled
	 * (DP_VID_STREAM_EN = 1).
	 */
	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal) ||
		    dc_is_virtual_signal(pipe_ctx->stream->signal))
			link_set_dsc_enable(pipe_ctx, true);
	}

	if (link->replay_settings.config.replay_supported && !dc_is_embedded_signal(link->connector_signal))
		dp_setup_replay(link, stream);

	status = enable_link(state, pipe_ctx);

	if (status != DC_OK) {
		DC_LOG_WARNING("enabling link %u failed: %d\n",
		pipe_ctx->stream->link->link_index,
		status);

		/* Abort stream enable *unless* the failure was due to
		 * DP link training - some DP monitors will recover and
		 * show the stream anyway. But MST displays can't proceed
		 * without link training.
		 */
		if (status != DC_FAIL_DP_LINK_TRAINING ||
				pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
			/*
			 * Don't leave a tiled-pair peer dark because this tile
			 * failed: unblank a deferred peer now (solo lighting
			 * beats a black panel).
			 */
			tiled_peer_pipe = get_tiled_peer_pipe(state, pipe_ctx);
			if (tiled_peer_pipe &&
			    tiled_peer_pipe->tiled_unblank_deferred) {
				tiled_peer_pipe->tiled_unblank_deferred = false;
				dc->hwss.unblank_stream(tiled_peer_pipe,
					&tiled_peer_pipe->stream->link->cur_link_settings);
			}
			if (false == stream->link->link_status.link_active)
				disable_link(stream->link, &pipe_ctx->link_res,
						pipe_ctx->stream->signal);
			BREAK_TO_DEBUGGER();
			return;
		}
	}

	/* turn off otg test pattern if enable */
	if (pipe_ctx->stream_res.tg->funcs->set_test_pattern)
		pipe_ctx->stream_res.tg->funcs->set_test_pattern(pipe_ctx->stream_res.tg,
				CONTROLLER_DP_TEST_PATTERN_VIDEOMODE,
				COLOR_DEPTH_UNDEFINED);

	/* This second call is needed to reconfigure the DIG
	 * as a workaround for the incorrect value being applied
	 * from transmitter control.
	 */
	if (!(dc_is_virtual_signal(pipe_ctx->stream->signal) ||
			dp_is_128b_132b_signal(pipe_ctx))) {

			if (link_enc)
				link_enc->funcs->setup(
					link_enc,
					pipe_ctx->stream->signal);

		}

	dc->hwss.enable_stream(pipe_ctx);

	/* Set DPS PPS SDP (AKA "info frames") */
	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal) ||
				dc_is_virtual_signal(pipe_ctx->stream->signal)) {
			dp_set_dsc_on_rx(pipe_ctx, true);
			link_set_dsc_pps_packet(pipe_ctx, true, true);
		}
	}

	if (dc_is_dp_signal(pipe_ctx->stream->signal))
		dp_set_hblank_reduction_on_rx(pipe_ctx);

	if (pipe_ctx->link_config.dp_tunnel_settings.should_use_dp_bw_allocation)
		allocate_usb4_bandwidth(pipe_ctx->stream);

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST)
		allocate_mst_payload(pipe_ctx);
	else if (dc_is_dp_sst_signal(pipe_ctx->stream->signal) &&
			dp_is_128b_132b_signal(pipe_ctx))
		update_sst_payload(pipe_ctx, true);

	/* Corruption was observed on systems with display mux when stream gets
	 * enabled after the mux switch. Having a small delay between link
	 * training and stream unblank resolves the corruption issue.
	 * This is workaround.
	 */
	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP &&
			link->is_display_mux_present)
		msleep(20);

	/*
	 * Dual-tile pair: never light one tile solo. The Apple 5K TCON latches
	 * native dual-tile mode only when both tiles first show video together
	 * (EFI ComplexDisplayInit trains both tiles, THEN enables both); a tile
	 * driven solo drops it to single-tile compat. If the peer tile is
	 * enabled later in this same pass (comes after us in pipe order, link
	 * not yet trained, and not a seamless/fast-boot pipe that skips its
	 * unblank), defer this unblank; the peer's set_dpms_on() then defers
	 * itself too and the pair is lit together AFTER program_timing_sync()
	 * has phase-aligned the OTGs (link_tiled_pair_post_sync_unblank()).
	 * dce110_apply_ctx_to_hw() unblanks any ORPHANED deferred pipe (peer
	 * enable failed or bailed early), so a miss degrades to the old
	 * solo-lighting order instead of a dark tile. For eDP the
	 * backlight-on rides inside unblank_stream and is deferred with it.
	 */
	tiled_peer_pipe = get_tiled_peer_pipe(state, pipe_ctx);
	if (tiled_peer_pipe &&
	    !tiled_peer_pipe->stream->link->link_status.link_active &&
	    tiled_peer_pipe->pipe_idx > pipe_ctx->pipe_idx &&
	    !tiled_peer_pipe->stream->apply_seamless_boot_optimization &&
	    !tiled_peer_pipe->stream->apply_edp_fast_boot_optimization) {
		pipe_ctx->tiled_unblank_deferred = true;
		DC_LOG_INFO("APPLE5K: defer unblank link[%u] until tiled peer link[%u] trains (no solo tile)\n",
			    link->link_index,
			    tiled_peer_pipe->stream->link->link_index);
	} else if (tiled_peer_pipe && tiled_peer_pipe->tiled_unblank_deferred) {
		/*
		 * Second tile of the pair: both links are trained now, but the
		 * two OTGs are still at ARBITRARY phase relative to each other
		 * (they were enabled hundreds of ms apart), and dc only
		 * phase-aligns them in program_timing_sync(), which runs after
		 * apply_ctx_to_hw(). Defer THIS unblank too: the pair is lit
		 * together in link_tiled_pair_post_sync_unblank(), right after
		 * the sync has aligned the still-blanked OTGs. Unaligned tiles
		 * read as incoherent dual-tile video and the panel TCON
		 * refuses the native latch (the EFI firmware programs both
		 * OTGs in ONE combined config, so they start aligned).
		 */
		pipe_ctx->tiled_unblank_deferred = true;
		DC_LOG_INFO("APPLE5K: defer unblank link[%u] too -- pair lights together after OTG phase sync\n",
			    link->link_index);
	} else {
		dc->hwss.unblank_stream(pipe_ctx,
			&pipe_ctx->stream->link->cur_link_settings);
	}

	if (stream->sink_patches.delay_ignore_msa > 0)
		msleep(stream->sink_patches.delay_ignore_msa);

	if (dc_is_dp_signal(pipe_ctx->stream->signal))
		enable_stream_features(pipe_ctx);
	update_psp_stream_config(pipe_ctx, false);

	dc->hwss.enable_audio_stream(pipe_ctx);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		set_avmute(pipe_ctx, false);
	}
}
