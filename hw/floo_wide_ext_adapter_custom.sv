// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Lars Kroeger <lkroeger@ethz.ch>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "axi/typedef.svh"
`include "axi/assign.svh"

/// AXI data-width adapter for the FlooNoC wide channel.
///
/// Transparently connects an external AXI port at ``AxiCfgWExt.DataWidth`` to the
/// internal NoC-side AXI at ``AxiCfgW.DataWidth``.  The two data widths must satisfy:
///
///   ``AxiCfgW.DataWidth == WideAggFactor * AxiCfgWExt.DataWidth``  (power-of-2 factor)
///
/// **Manager path** (cluster DMA → NoC ingress):
///   * W beats are aggregated: ``WideAggFactor`` narrow beats → 1 wide beat
///   * R beats are split:       1 wide beat → ``WideAggFactor`` narrow beats
///   * AW/AR ``len`` is divided by ``WideAggFactor``; ``size`` is incremented accordingly
///
/// **Subordinate path** (NoC egress → cluster memory/TCDM):
///   * W beats are split:       1 wide beat → ``WideAggFactor`` narrow beats
///   * R beats are aggregated: ``WideAggFactor`` narrow beats → 1 wide beat
///   * AW/AR ``len`` is multiplied by ``WideAggFactor``; ``size`` is decremented accordingly
///
/// All non-data fields (id, addr, user, resp) pass through; the user field is
/// truncated or zero-extended as needed when ``AxiCfgWExt.UserWidth != AxiCfgW.UserWidth``.
///
/// **Known limitation**: the manager AW/AR ``len`` must be a multiple of ``WideAggFactor``.
/// Transactions where ``(len+1)`` is not divisible by ``WideAggFactor`` will receive an
/// incorrect number of R beats.  For write transactions, the aggregator flushes early on
/// ``last``, so they are handled correctly.
module floo_wide_ext_adapter
  import floo_pkg::*;
#(
  /// NoC-side wide AXI config (DataWidth = full flit width, e.g. 1024 b).
  parameter floo_pkg::axi_cfg_t AxiCfgW    = '0,
  /// Cluster-side wide AXI config (DataWidth = external port width, e.g. 512 b).
  parameter floo_pkg::axi_cfg_t AxiCfgWExt = '0,
  // ---------------------------------------------------------------------------
  // Type parameters — cluster side (AxiCfgWExt data width)
  // ---------------------------------------------------------------------------
  /// External manager-path request type  (cluster DMA output, InIdWidth).
  parameter type axi_wide_ext_in_req_t  = logic,
  /// External manager-path response type.
  parameter type axi_wide_ext_in_rsp_t  = logic,
  /// External subordinate-path request type (cluster memory input, OutIdWidth).
  parameter type axi_wide_ext_out_req_t = logic,
  /// External subordinate-path response type.
  parameter type axi_wide_ext_out_rsp_t = logic,
  // ---------------------------------------------------------------------------
  // Type parameters — NoC side (AxiCfgW data width)
  // ---------------------------------------------------------------------------
  /// NoC manager-path request type  (→ chimney axi_wide_in_req_i, InIdWidth).
  parameter type axi_wide_in_req_t  = logic,
  /// NoC manager-path response type (← chimney axi_wide_in_rsp_o).
  parameter type axi_wide_in_rsp_t  = logic,
  /// NoC subordinate-path request type  (← chimney axi_wide_out_req_o, OutIdWidth).
  parameter type axi_wide_out_req_t = logic,
  /// NoC subordinate-path response type (→ chimney axi_wide_out_rsp_i).
  parameter type axi_wide_out_rsp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,

  // ---------------------------------------------------------------------------
  // Cluster side (external, AxiCfgWExt data width)
  // ---------------------------------------------------------------------------
  /// Manager path: cluster DMA output → NoC ingress.
  input  axi_wide_ext_in_req_t  axi_wide_ext_in_req_i,
  output axi_wide_ext_in_rsp_t  axi_wide_ext_in_rsp_o,
  /// Subordinate path: NoC egress → cluster memory/TCDM.
  output axi_wide_ext_out_req_t axi_wide_ext_out_req_o,
  input  axi_wide_ext_out_rsp_t axi_wide_ext_out_rsp_i,

  // ---------------------------------------------------------------------------
  // NoC side (internal, AxiCfgW data width — connects to floo_nw_chimney)
  // ---------------------------------------------------------------------------
  /// Manager path → chimney axi_wide_in_req_i / ← chimney axi_wide_in_rsp_o.
  output axi_wide_in_req_t  axi_wide_in_req_o,
  input  axi_wide_in_rsp_t  axi_wide_in_rsp_i,
  /// Subordinate path ← chimney axi_wide_out_req_o / → chimney axi_wide_out_rsp_i.
  input  axi_wide_out_req_t axi_wide_out_req_i,
  output axi_wide_out_rsp_t axi_wide_out_rsp_o
);

  // ---------------------------------------------------------------------------
  // Derived parameters
  // ---------------------------------------------------------------------------
  localparam int unsigned WideAggFactor = AxiCfgW.DataWidth / AxiCfgWExt.DataWidth;

  /// AXI size field for one full-width narrow beat  (log2 of bytes-per-beat).
  /// e.g. 512-bit bus → 64 B/beat → size = 6.
  localparam int unsigned SizeExt   = $clog2(AxiCfgWExt.DataWidth / 8);
  /// AXI size field for one full-width wide beat.
  /// e.g. 1024-bit bus → 128 B/beat → size = 7.
  localparam int unsigned SizeInt   = $clog2(AxiCfgW.DataWidth / 8);
  /// Width of the sub-beat lane-select index (= $clog2(WideAggFactor), minimum 1).
  localparam int unsigned LaneWidth = (WideAggFactor > 1) ? $clog2(WideAggFactor) : 1;
  // ---------------------------------------------------------------------------
  // Internal type declarations
  //   NoC-side channel types are derived locally from AxiCfgW so the submodule
  //   instantiations have concrete types to bind against.
  // ---------------------------------------------------------------------------

  // Shared address type
  typedef logic [AxiCfgW.AddrWidth-1:0]    axi_addr_t;

  // Manager-path (InIdWidth) NoC types
  typedef logic [AxiCfgW.InIdWidth-1:0]    axi_wide_in_id_t;
  typedef logic [AxiCfgW.DataWidth-1:0]    axi_wide_data_t;
  typedef logic [AxiCfgW.DataWidth/8-1:0]  axi_wide_strb_t;
  typedef logic [AxiCfgW.UserWidth-1:0]    axi_wide_user_t;
  `AXI_TYPEDEF_W_CHAN_T(axi_wide_w_chan_t,
      axi_wide_data_t, axi_wide_strb_t, axi_wide_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_wide_in_r_chan_t,
      axi_wide_data_t, axi_wide_in_id_t, axi_wide_user_t)

  // Subordinate-path (OutIdWidth) NoC types
  typedef logic [AxiCfgW.OutIdWidth-1:0]   axi_wide_out_id_t;
  `AXI_TYPEDEF_R_CHAN_T(axi_wide_out_r_chan_t,
      axi_wide_data_t, axi_wide_out_id_t, axi_wide_user_t)

  // External (AxiCfgWExt) channel types for submodule parameter binding
  typedef logic [AxiCfgWExt.DataWidth-1:0]    axi_ext_data_t;
  typedef logic [AxiCfgWExt.DataWidth/8-1:0]  axi_ext_strb_t;
  typedef logic [AxiCfgWExt.UserWidth-1:0]    axi_ext_user_t;
  typedef logic [AxiCfgWExt.InIdWidth-1:0]    axi_ext_in_id_t;
  typedef logic [AxiCfgWExt.OutIdWidth-1:0]   axi_ext_out_id_t;
  // W channel is id-less; one type serves both mgr and sbr paths
  `AXI_TYPEDEF_W_CHAN_T(axi_ext_w_chan_t,
      axi_ext_data_t, axi_ext_strb_t, axi_ext_user_t)
  // R channel carries an id; separate types for mgr (InIdWidth) and sbr (OutIdWidth)
  `AXI_TYPEDEF_R_CHAN_T(axi_ext_in_r_chan_t,
      axi_ext_data_t, axi_ext_in_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_ext_out_r_chan_t,
      axi_ext_data_t, axi_ext_out_id_t, axi_ext_user_t)

  // ---------------------------------------------------------------------------
  // Intermediate signals driven by submodule instances
  // ---------------------------------------------------------------------------

  // Forward declarations needed by the AW context FIFO
  // (aw_pop references mgr_w_agg_chan and mgr_w_valid before their full declarations).
  axi_wide_w_chan_t mgr_w_agg_chan;
  logic             mgr_w_valid;

  // ===========================================================================
  // Upsize-mode decision
  //
  //   AR/AW are upsized (size+1, len/WideAggFactor) only when BOTH hold:
  //     (a) Request uses full-width narrow beats:  size == SizeExt
  //     (b) Burst length divides evenly:           (len+1) % WideAggFactor == 0
  //   All other requests pass through at their original size/len.
  // ===========================================================================

  logic mgr_ar_upsize;
  logic mgr_aw_upsize;

  // Upsize when: full-width narrow beats (size == SizeExt), 128B-aligned start
  // address (so beat 0 lands in lane 0), and burst length divisible by WideAggFactor.
  assign mgr_ar_upsize =
      (axi_wide_ext_in_req_i.ar.size == axi_pkg::size_t'(SizeExt)) &&
      (axi_wide_ext_in_req_i.ar.addr[$clog2(AxiCfgWExt.DataWidth/8)] == 1'b0) &&
      (((int'(axi_wide_ext_in_req_i.ar.len) + 1) % int'(WideAggFactor)) == 0);

  assign mgr_aw_upsize =
      (axi_wide_ext_in_req_i.aw.size == axi_pkg::size_t'(SizeExt)) &&
      (axi_wide_ext_in_req_i.aw.addr[$clog2(AxiCfgWExt.DataWidth/8)] == 1'b0) &&
      (((int'(axi_wide_ext_in_req_i.aw.len) + 1) % int'(WideAggFactor)) == 0);

  // ===========================================================================
  // AR read-context tracker — per-ID register file
  //
  // AXI guarantees that a master will not re-issue an AR with ID X until all
  // R beats for the previous AR with that ID have been accepted.  A single
  // register per ID therefore suffices for any number of concurrently
  // outstanding transactions across different IDs (full ID-interleaving support).
  //
  // Each 2-bit entry stores:
  //   upsize   — 1: full-split (emit all WideAggFactor sub-beats per NoC R beat)
  //              0: passthrough (emit the single sub-beat selected by lane_bit)
  //   lane_bit — addr[$clog2(ExtBytes)] at AR time; selects which 512-bit half
  //              of the 1024-bit NoC beat to return (ignored when upsize = 1)
  // ===========================================================================

  typedef struct packed {
    logic upsize;
    logic lane_bit;
  } ar_ctx_t;

  localparam int unsigned NumArIds = 2 ** AxiCfgW.InIdWidth;

  ar_ctx_t ar_ctx_q [NumArIds];  // one entry per possible AR transaction ID

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < NumArIds; i++) ar_ctx_q[i] <= '0;
    end else if (axi_wide_ext_in_req_i.ar_valid && axi_wide_in_rsp_i.ar_ready) begin
      ar_ctx_q[axi_wide_ext_in_req_i.ar.id] <= '{
          upsize:   mgr_ar_upsize,
          lane_bit: axi_wide_ext_in_req_i.ar.addr[$clog2(AxiCfgWExt.DataWidth/8)]
      };
    end
  end

  // Look up context for each incoming R beat via the R channel's ID.
  logic mgr_r_full_split;
  logic mgr_r_start_beat;
  assign mgr_r_full_split = ar_ctx_q[axi_wide_in_rsp_i.r.id].upsize;
  assign mgr_r_start_beat = ar_ctx_q[axi_wide_in_rsp_i.r.id].lane_bit;

  // ===========================================================================
  // AW write-context tracker — ordered FIFO
  //
  // The AXI W channel carries no ID; W beats follow AW in issue order.  An
  // ordered FIFO captures {upsize, lane_bit} for each AW so that when the
  // corresponding W beats arrive the aggregator applies the right upsize mode
  // and the lane-shift mux places data in the correct 512-bit half.
  //
  // Push: AW handshake.  Pop: NoC accepts the last W beat of a transaction.
  // AW is stalled (aw_valid/aw_ready gated) when the FIFO is full.
  // ===========================================================================

  typedef struct packed {
    logic upsize;
    logic lane_bit;
  } aw_ctx_t;

  localparam int unsigned AwFifoDepth = 4;
  localparam int unsigned AwCntW      = $clog2(AwFifoDepth) + 1;

  aw_ctx_t           aw_ctx_q [AwFifoDepth];
  logic [AwCntW-1:0] aw_ctx_cnt_q;
  logic              aw_fifo_full;
  logic              aw_push, aw_pop;

  assign aw_fifo_full = (aw_ctx_cnt_q == AwCntW'(AwFifoDepth));
  assign aw_push      = axi_wide_ext_in_req_i.aw_valid && axi_wide_in_rsp_i.aw_ready;
  assign aw_pop       = mgr_w_valid && axi_wide_in_rsp_i.w_ready && mgr_w_agg_chan.last;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      aw_ctx_cnt_q <= '0;
      for (int i = 0; i < AwFifoDepth; i++) aw_ctx_q[i] <= '0;
    end else begin
      unique case ({aw_push, aw_pop})
        2'b10: begin  // push only
          aw_ctx_q[aw_ctx_cnt_q] <= '{
              upsize:   mgr_aw_upsize,
              lane_bit: axi_wide_ext_in_req_i.aw.addr[$clog2(AxiCfgWExt.DataWidth/8)]
          };
          aw_ctx_cnt_q <= aw_ctx_cnt_q + 1'b1;
        end
        2'b01: begin  // pop only — shift entries towards head
          for (int i = 0; i < AwFifoDepth - 1; i++) aw_ctx_q[i] <= aw_ctx_q[i+1];
          aw_ctx_cnt_q <= aw_ctx_cnt_q - 1'b1;
        end
        2'b11: begin  // push and pop simultaneously — shift down, append new tail
          for (int i = 0; i < AwFifoDepth - 1; i++) aw_ctx_q[i] <= aw_ctx_q[i+1];
          aw_ctx_q[aw_ctx_cnt_q - 1] <= '{
              upsize:   mgr_aw_upsize,
              lane_bit: axi_wide_ext_in_req_i.aw.addr[$clog2(AxiCfgWExt.DataWidth/8)]
          };
          // count stays the same
        end
        default: ;  // 2'b00: no change
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Intermediate signals driven by submodule instances
  // ---------------------------------------------------------------------------

  // Manager path
  axi_wide_w_chan_t    mgr_w_chan;      // 1024-bit W beat → NoC (post lane-shift mux)
  logic                mgr_w_ready;    // backpressure from aggregator → cluster
  // mgr_w_agg_chan and mgr_w_valid declared above (needed by AW context FIFO)
  axi_ext_in_r_chan_t  mgr_r_chan;
  logic                mgr_r_valid;
  logic                mgr_r_slv_rdy;  // splitter absorb-ready → NoC (axi_wide_in_req.r_ready)

  // Subordinate path
  axi_ext_w_chan_t      sbr_w_chan;     // 512-bit W beat sliced by splitter → memory
  logic                 sbr_w_valid;
  logic                 sbr_w_slv_rdy; // splitter absorb-ready → NoC (axi_wide_out_rsp.w_ready)
  axi_wide_out_r_chan_t sbr_r_chan;    // 1024-bit R beat assembled by aggregator → NoC
  logic                 sbr_r_valid;
  logic                 sbr_r_ext_rdy; // aggregator absorb-ready → memory (ext_out_req.r_ready)

  // ===========================================================================
  // Manager path: cluster DMA → NoC
  //
  //  AW/AR: direct field copy + len/size adjustment (>> log2(WideAggFactor))
  //  B:     direct field copy (user auto-truncated to AxiCfgW.UserWidth)
  //  W:     aggregation via i_wide_w_agg (WideAggFactor × 512-bit → 1024-bit)
  //  R:     splitting via i_wide_r_split  (1024-bit → WideAggFactor × 512-bit)
  // ===========================================================================

  always_comb begin
    axi_wide_in_req_o = '0;

    // AW: copy all fields, then upsize.
    // size is always promoted to SizeInt so the downstream memory adapter's
    // fixed-expansion formula receives the expected SizeInt input and issues
    // correct SizeExt memory accesses.  len is compressed only for full-upsize
    // bursts; for passthrough (odd-len) the W aggregator flushes on `last`
    // and unused upper lanes keep strb=0.
    `AXI_SET_AW_STRUCT(axi_wide_in_req_o.aw, axi_wide_ext_in_req_i.aw)
    if (mgr_aw_upsize && axi_wide_ext_in_req_i.aw.size == axi_pkg::size_t'(SizeExt)) begin
      axi_wide_in_req_o.aw.size = axi_pkg::size_t'(SizeInt);
    end
    if (mgr_aw_upsize) begin
      axi_wide_in_req_o.aw.len  = axi_wide_ext_in_req_i.aw.len >> $clog2(WideAggFactor);
    end
    // aw_valid: gated by FIFO — prevent AW acceptance when context storage is full.
    axi_wide_in_req_o.aw_valid = axi_wide_ext_in_req_i.aw_valid && !aw_fifo_full;

    // AR: copy all fields, then upsize.
    // size is always promoted to SizeInt so the downstream memory adapter
    // correctly expands to SizeExt memory accesses and the R aggregator
    // returns a full SizeInt-wide beat.  The R splitter then selects the
    // correct SizeExt-wide lane (passthrough) or emits all lanes (full-split).
    `AXI_SET_AR_STRUCT(axi_wide_in_req_o.ar, axi_wide_ext_in_req_i.ar)
    if (mgr_ar_upsize && axi_wide_ext_in_req_i.ar.size == axi_pkg::size_t'(SizeExt)) begin
      axi_wide_in_req_o.ar.size = axi_pkg::size_t'(SizeInt);
    end
    if (mgr_ar_upsize) begin
      axi_wide_in_req_o.ar.len  = axi_wide_ext_in_req_i.ar.len >> $clog2(WideAggFactor);
    end
    // ar_valid: no gating needed — per-ID register file covers all possible IDs.
    axi_wide_in_req_o.ar_valid = axi_wide_ext_in_req_i.ar_valid;

    // B: pass through (no data field; user auto-truncated to AxiCfgW.UserWidth)
    axi_wide_in_req_o.b_ready = axi_wide_ext_in_req_i.b_ready;

    // W and R_ready: driven by submodule instances via intermediates
    axi_wide_in_req_o.w       = mgr_w_chan;
    axi_wide_in_req_o.w_valid = mgr_w_valid;
    axi_wide_in_req_o.r_ready = mgr_r_slv_rdy;
  end

  always_comb begin
    axi_wide_ext_in_rsp_o = '0;
    // aw_ready: gated by FIFO — stall cluster when AW context storage is full.
    axi_wide_ext_in_rsp_o.aw_ready = axi_wide_in_rsp_i.aw_ready && !aw_fifo_full;
    // ar_ready: no gating needed — per-ID register file covers all possible IDs.
    axi_wide_ext_in_rsp_o.ar_ready = axi_wide_in_rsp_i.ar_ready;
    // B: pass through (user auto-extended to AxiCfgWExt.UserWidth)
    `AXI_SET_B_STRUCT(axi_wide_ext_in_rsp_o.b, axi_wide_in_rsp_i.b)
    axi_wide_ext_in_rsp_o.b_valid = axi_wide_in_rsp_i.b_valid;
    // W_ready and R: driven by submodule instances via intermediates
    axi_wide_ext_in_rsp_o.w_ready = mgr_w_ready;
    axi_wide_ext_in_rsp_o.r       = mgr_r_chan;
    axi_wide_ext_in_rsp_o.r_valid = mgr_r_valid;
  end

  // W: aggregate WideAggFactor narrow W beats from the cluster into one wide NoC beat
  floo_wide_w_aggregator #(
    .ExtDataWidth ( AxiCfgWExt.DataWidth ),
    .IntDataWidth ( AxiCfgW.DataWidth    ),
    .IsRChan      ( 1'b0                 ),
    .ExtUserWidth ( AxiCfgWExt.UserWidth ),
    .IntUserWidth ( AxiCfgW.UserWidth    ),
    .ext_w_chan_t  ( axi_ext_w_chan_t    ),
    .int_w_chan_t  ( axi_wide_w_chan_t   )
  ) i_wide_w_agg (
    .clk_i,
    .rst_ni,
    .upsize_i      ( aw_ctx_q[0].upsize             ),  // from AW FIFO head
    .slv_w_chan_i  ( axi_wide_ext_in_req_i.w       ),
    .slv_w_valid_i ( axi_wide_ext_in_req_i.w_valid ),
    .slv_w_ready_o ( mgr_w_ready                   ),
    .mst_w_chan_o  ( mgr_w_agg_chan                 ),
    .mst_w_valid_o ( mgr_w_valid                   ),
    .mst_w_ready_i ( axi_wide_in_rsp_i.w_ready     )
  );

  // W lane-shift mux: in passthrough mode, if addr[6] = 1 the cluster beat belongs
  // in the upper 512-bit lane of the 1024-bit NoC beat.  Shift data + strb up by
  // ExtDataWidth bits so the memory tile writes to the correct byte offset.
  // In upsize mode the aggregator packs both lanes correctly — no shift needed.
  always_comb begin
    mgr_w_chan = mgr_w_agg_chan;
    if (!aw_ctx_q[0].upsize && aw_ctx_q[0].lane_bit) begin
      mgr_w_chan.data = {mgr_w_agg_chan.data[AxiCfgWExt.DataWidth-1:0],
                         {AxiCfgWExt.DataWidth{1'b0}}};
      mgr_w_chan.strb = {mgr_w_agg_chan.strb[AxiCfgWExt.DataWidth/8-1:0],
                         {(AxiCfgWExt.DataWidth/8){1'b0}}};
    end
  end

  // R: split one wide NoC R beat into WideAggFactor narrow R beats for the cluster
  floo_wide_r_splitter #(
    .ExtDataWidth ( AxiCfgWExt.DataWidth  ),
    .IntDataWidth ( AxiCfgW.DataWidth     ),
    .IsRChan      ( 1'b1                  ),
    .IdWidth      ( AxiCfgW.InIdWidth     ),
    .ExtUserWidth ( AxiCfgWExt.UserWidth  ),
    .IntUserWidth ( AxiCfgW.UserWidth     ),
    .ext_r_chan_t  ( axi_ext_in_r_chan_t  ),
    .int_r_chan_t  ( axi_wide_in_r_chan_t )
  ) i_wide_r_split (
    .clk_i,
    .rst_ni,
    .full_split_i  ( mgr_r_full_split              ),  // from AR context register file
    .start_beat_i  ( mgr_r_start_beat              ),  // from AR context register file
    .slv_r_chan_i  ( axi_wide_in_rsp_i.r            ),
    .slv_r_valid_i ( axi_wide_in_rsp_i.r_valid      ),
    .slv_r_ready_o ( mgr_r_slv_rdy                  ),
    .mst_r_chan_o  ( mgr_r_chan                      ),
    .mst_r_valid_o ( mgr_r_valid                    ),
    .mst_r_ready_i ( axi_wide_ext_in_req_i.r_ready  )
  );

  // ===========================================================================
  // Subordinate path: NoC → cluster memory
  //
  //  AW/AR: direct field copy + len/size restoration (* WideAggFactor, >> log2(WideAggFactor))
  //  B:     direct field copy (user auto-extended to AxiCfgWExt.UserWidth)
  //  W:     splitting via i_wide_sbr_w_split  (1024-bit → WideAggFactor × 512-bit)
  //  R:     aggregation via i_wide_sbr_r_agg  (WideAggFactor × 512-bit → 1024-bit)
  // ===========================================================================

  always_comb begin
    axi_wide_ext_out_req_o = '0;

    // AW: copy all fields then restore beat count and size for the narrower memory bus
    `AXI_SET_AW_STRUCT(axi_wide_ext_out_req_o.aw, axi_wide_out_req_i.aw)
    axi_wide_ext_out_req_o.aw.len  = axi_pkg::len_t'(
        (int'(axi_wide_out_req_i.aw.len) + 1) * WideAggFactor - 1);
    axi_wide_ext_out_req_o.aw.size = axi_wide_out_req_i.aw.size
                                        - axi_pkg::size_t'($clog2(WideAggFactor));
    axi_wide_ext_out_req_o.aw_valid = axi_wide_out_req_i.aw_valid;

    // AR: same restoration
    `AXI_SET_AR_STRUCT(axi_wide_ext_out_req_o.ar, axi_wide_out_req_i.ar)
    axi_wide_ext_out_req_o.ar.len  = axi_pkg::len_t'(
        (int'(axi_wide_out_req_i.ar.len) + 1) * WideAggFactor - 1);
    axi_wide_ext_out_req_o.ar.size = axi_wide_out_req_i.ar.size
                                        - axi_pkg::size_t'($clog2(WideAggFactor));
    axi_wide_ext_out_req_o.ar_valid = axi_wide_out_req_i.ar_valid;

    // B: pass through (user auto-extended to AxiCfgWExt.UserWidth)
    axi_wide_ext_out_req_o.b_ready = axi_wide_out_req_i.b_ready;

    // W and R_ready: driven by submodule instances via intermediates
    axi_wide_ext_out_req_o.w       = sbr_w_chan;
    axi_wide_ext_out_req_o.w_valid = sbr_w_valid;
    axi_wide_ext_out_req_o.r_ready = sbr_r_ext_rdy;
  end

  always_comb begin
    axi_wide_out_rsp_o = '0;
    axi_wide_out_rsp_o.aw_ready = axi_wide_ext_out_rsp_i.aw_ready;
    axi_wide_out_rsp_o.ar_ready = axi_wide_ext_out_rsp_i.ar_ready;
    // B: pass through (user auto-truncated to AxiCfgW.UserWidth)
    `AXI_SET_B_STRUCT(axi_wide_out_rsp_o.b, axi_wide_ext_out_rsp_i.b)
    axi_wide_out_rsp_o.b_valid  = axi_wide_ext_out_rsp_i.b_valid;
    // W_ready and R: driven by submodule instances via intermediates
    axi_wide_out_rsp_o.w_ready  = sbr_w_slv_rdy;
    axi_wide_out_rsp_o.r        = sbr_r_chan;
    axi_wide_out_rsp_o.r_valid  = sbr_r_valid;
  end

  // W: split one wide NoC W beat into WideAggFactor narrow W beats for the memory port.
  // floo_wide_r_splitter is reused parameterised with W-channel types (IsRChan=0 selects
  // the gen_w_splitter path, which slices data+strb instead of data+id+resp).
  floo_wide_r_splitter #(
    .ExtDataWidth ( AxiCfgWExt.DataWidth ),
    .IntDataWidth ( AxiCfgW.DataWidth    ),
    .IsRChan      ( 1'b0                 ),
    .ExtUserWidth ( AxiCfgWExt.UserWidth ),
    .IntUserWidth ( AxiCfgW.UserWidth    ),
    .ext_r_chan_t  ( axi_ext_w_chan_t    ),  // narrow W type bound to ext_r_chan_t
    .int_r_chan_t  ( axi_wide_w_chan_t   )   // wide  W type bound to int_r_chan_t
  ) i_wide_sbr_w_split (
    .clk_i,
    .rst_ni,
    .full_split_i  ( 1'b1                            ),  // always full-split on sbr path
    .start_beat_i  ( '0                              ),
    .slv_r_chan_i  ( axi_wide_out_req_i.w            ),
    .slv_r_valid_i ( axi_wide_out_req_i.w_valid      ),
    .slv_r_ready_o ( sbr_w_slv_rdy                   ),
    .mst_r_chan_o  ( sbr_w_chan                       ),
    .mst_r_valid_o ( sbr_w_valid                     ),
    .mst_r_ready_i ( axi_wide_ext_out_rsp_i.w_ready  )
  );

  // R: aggregate WideAggFactor narrow R beats from the memory into one wide NoC R beat.
  // floo_wide_w_aggregator is reused parameterised with R-channel types (IsRChan=1 selects
  // the gen_r_aggregator path, which accumulates data and replicates id+resp+user).
  floo_wide_w_aggregator #(
    .ExtDataWidth ( AxiCfgWExt.DataWidth  ),
    .IntDataWidth ( AxiCfgW.DataWidth     ),
    .IsRChan      ( 1'b1                  ),
    .IdWidth      ( AxiCfgW.OutIdWidth    ),
    .ExtUserWidth ( AxiCfgWExt.UserWidth  ),
    .IntUserWidth ( AxiCfgW.UserWidth     ),
    .ext_w_chan_t  ( axi_ext_out_r_chan_t  ),  // narrow R type bound to ext_w_chan_t
    .int_w_chan_t  ( axi_wide_out_r_chan_t )   // wide  R type bound to int_w_chan_t
  ) i_wide_sbr_r_agg (
    .clk_i,
    .rst_ni,
    .upsize_i      ( 1'b1                            ),  // sbr path always aggregates fully
    .slv_w_chan_i  ( axi_wide_ext_out_rsp_i.r        ),
    .slv_w_valid_i ( axi_wide_ext_out_rsp_i.r_valid  ),
    .slv_w_ready_o ( sbr_r_ext_rdy                   ),
    .mst_w_chan_o  ( sbr_r_chan                       ),
    .mst_w_valid_o ( sbr_r_valid                     ),
    .mst_w_ready_i ( axi_wide_out_req_i.r_ready      )
  );

  // ---------------------------------------------------------------------------
  // Assertions
  // ---------------------------------------------------------------------------
  `ASSERT_INIT(WideExtDataWidthSmaller,
      AxiCfgWExt.DataWidth < AxiCfgW.DataWidth)
  `ASSERT_INIT(WideExtDataWidthDivisible,
      (AxiCfgW.DataWidth % AxiCfgWExt.DataWidth) == 0)
  `ASSERT_INIT(WideExtAggFactorPow2,
      (WideAggFactor & (WideAggFactor - 1)) == 0)
  `ASSERT_INIT(WideExtAddrWidthMatch,
      AxiCfgWExt.AddrWidth == AxiCfgW.AddrWidth)
  `ASSERT_INIT(WideExtInIdWidthMatch,
      AxiCfgWExt.InIdWidth == AxiCfgW.InIdWidth)
  `ASSERT_INIT(WideExtOutIdWidthMatch,
      AxiCfgWExt.OutIdWidth == AxiCfgW.OutIdWidth)

endmodule
