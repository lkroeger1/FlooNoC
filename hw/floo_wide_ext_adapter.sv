// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Lars Kroeger <lkroeger@ethz.ch>

`include "axi/typedef.svh"
`include "axi/assign.svh"
`include "common_cells/assertions.svh"
`include "floo_noc/typedef.svh"

/// AXI data-width adapter for the FlooNoC wide channel.
///
/// Transparently connects an external AXI port at ``AxiCfgWExt.DataWidth`` to the
/// internal NoC-side AXI at ``AxiCfgW.DataWidth`` using the standard
/// ``axi_dw_converter`` (upsizer / downsizer) from the pulp-platform AXI library.
///
/// **Manager path** (cluster DMA → NoC ingress): ``axi_dw_converter`` upsizer.
/// **Subordinate path** (NoC egress → cluster memory): ``axi_dw_converter`` downsizer.

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
  input  axi_wide_ext_in_req_t  axi_wide_ext_in_req_i,
  output axi_wide_ext_in_rsp_t  axi_wide_ext_in_rsp_o,
  output axi_wide_ext_out_req_t axi_wide_ext_out_req_o,
  input  axi_wide_ext_out_rsp_t axi_wide_ext_out_rsp_i,

  // ---------------------------------------------------------------------------
  // NoC side (internal, AxiCfgW data width)
  // ---------------------------------------------------------------------------
  output axi_wide_in_req_t  axi_wide_in_req_o,
  input  axi_wide_in_rsp_t  axi_wide_in_rsp_i,
  input  axi_wide_out_req_t axi_wide_out_req_i,
  output axi_wide_out_rsp_t axi_wide_out_rsp_o
);

  // ---------------------------------------------------------------------------
  // Derived parameters
  // ---------------------------------------------------------------------------

  /// Maximum number of outstanding read transactions tracked by the upsizer.
  /// Each entry consumes registers proportional to AxiCfgW.DataWidth, so keep
  /// this small but large enough to cover the DMA's pipeline depth.
  localparam int unsigned AxiMaxReads = 8;

  // ---------------------------------------------------------------------------
  // Internal type declarations
  //
  // The axi_dw_converter shares a single aw_chan_t / ar_chan_t / b_chan_t for
  // both ports (address and response channels are the same width on both sides).
  // We use the cluster-side (ExtUserWidth) variants throughout the converters
  // and adapt the user field at the NoC boundary.
  // ---------------------------------------------------------------------------

  typedef logic [AxiCfgW.AddrWidth-1:0]       axi_addr_t;
  typedef logic [AxiCfgWExt.UserWidth-1:0]     axi_ext_user_t;

  // --- Manager path types (InIdWidth) ---
  typedef logic [AxiCfgW.InIdWidth-1:0]        axi_mgr_id_t;
  typedef logic [AxiCfgWExt.DataWidth-1:0]     axi_mgr_slv_data_t;
  typedef logic [AxiCfgWExt.DataWidth/8-1:0]   axi_mgr_slv_strb_t;
  typedef logic [AxiCfgW.DataWidth-1:0]        axi_mgr_mst_data_t;
  typedef logic [AxiCfgW.DataWidth/8-1:0]      axi_mgr_mst_strb_t;

  `FLOO_WIDE_AXI_TYPEDEF_AW_CHAN_T(axi_mgr_aw_t, axi_addr_t, axi_mgr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_W_CHAN_T (axi_mgr_slv_w_t, axi_mgr_slv_data_t, axi_mgr_slv_strb_t, axi_ext_user_t)
  `AXI_TYPEDEF_W_CHAN_T (axi_mgr_mst_w_t, axi_mgr_mst_data_t, axi_mgr_mst_strb_t, axi_ext_user_t)
  `AXI_TYPEDEF_B_CHAN_T (axi_mgr_b_t,  axi_mgr_id_t, axi_ext_user_t)
  `FLOO_WIDE_AXI_TYPEDEF_AR_CHAN_T(axi_mgr_ar_t, axi_addr_t, axi_mgr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_R_CHAN_T (axi_mgr_slv_r_t, axi_mgr_slv_data_t, axi_mgr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_R_CHAN_T (axi_mgr_mst_r_t, axi_mgr_mst_data_t, axi_mgr_id_t, axi_ext_user_t)

  `AXI_TYPEDEF_REQ_T (axi_mgr_slv_req_t,  axi_mgr_aw_t, axi_mgr_slv_w_t, axi_mgr_ar_t)
  `AXI_TYPEDEF_RESP_T(axi_mgr_slv_resp_t, axi_mgr_b_t,  axi_mgr_slv_r_t)
  `AXI_TYPEDEF_REQ_T (axi_mgr_mst_req_t,  axi_mgr_aw_t, axi_mgr_mst_w_t, axi_mgr_ar_t)
  `AXI_TYPEDEF_RESP_T(axi_mgr_mst_resp_t, axi_mgr_b_t,  axi_mgr_mst_r_t)

  // --- Subordinate path types (OutIdWidth) ---
  typedef logic [AxiCfgW.OutIdWidth-1:0]       axi_sbr_id_t;
  typedef logic [AxiCfgW.DataWidth-1:0]        axi_sbr_slv_data_t;
  typedef logic [AxiCfgW.DataWidth/8-1:0]      axi_sbr_slv_strb_t;
  typedef logic [AxiCfgWExt.DataWidth-1:0]     axi_sbr_mst_data_t;
  typedef logic [AxiCfgWExt.DataWidth/8-1:0]   axi_sbr_mst_strb_t;

  `FLOO_WIDE_AXI_TYPEDEF_AW_CHAN_T(axi_sbr_aw_t, axi_addr_t, axi_sbr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_W_CHAN_T (axi_sbr_slv_w_t, axi_sbr_slv_data_t, axi_sbr_slv_strb_t, axi_ext_user_t)
  `AXI_TYPEDEF_W_CHAN_T (axi_sbr_mst_w_t, axi_sbr_mst_data_t, axi_sbr_mst_strb_t, axi_ext_user_t)
  `AXI_TYPEDEF_B_CHAN_T (axi_sbr_b_t,  axi_sbr_id_t, axi_ext_user_t)
  `FLOO_WIDE_AXI_TYPEDEF_AR_CHAN_T(axi_sbr_ar_t, axi_addr_t, axi_sbr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_R_CHAN_T (axi_sbr_slv_r_t, axi_sbr_slv_data_t, axi_sbr_id_t, axi_ext_user_t)
  `AXI_TYPEDEF_R_CHAN_T (axi_sbr_mst_r_t, axi_sbr_mst_data_t, axi_sbr_id_t, axi_ext_user_t)

  `AXI_TYPEDEF_REQ_T (axi_sbr_slv_req_t,  axi_sbr_aw_t, axi_sbr_slv_w_t, axi_sbr_ar_t)
  `AXI_TYPEDEF_RESP_T(axi_sbr_slv_resp_t, axi_sbr_b_t,  axi_sbr_slv_r_t)
  `AXI_TYPEDEF_REQ_T (axi_sbr_mst_req_t,  axi_sbr_aw_t, axi_sbr_mst_w_t, axi_sbr_ar_t)
  `AXI_TYPEDEF_RESP_T(axi_sbr_mst_resp_t, axi_sbr_b_t,  axi_sbr_mst_r_t)

  // ---------------------------------------------------------------------------
  // Intermediate signals: converter ↔ NoC boundary (wide, ext user width)
  // ---------------------------------------------------------------------------

  axi_mgr_mst_req_t  mgr_conv_req;   // upsizer output  (1024-bit, 52-bit user)
  axi_mgr_mst_resp_t mgr_conv_resp;  // upsizer input   (1024-bit, 52-bit user)

  axi_sbr_slv_req_t  sbr_conv_req;   // downsizer input (1024-bit, 52-bit user)
  axi_sbr_slv_resp_t sbr_conv_resp;  // downsizer output(1024-bit, 52-bit user)

  // Cluster-boundary adaptation signals.
  // snitch_cluster_pkg uses AXI_TYPEDEF_ALL (3-bit size_t), but the DW converters
  // share a single aw/ar_chan_t for both slave and master ports and must use
  // FLOO_WIDE_AXI_TYPEDEF (4-bit size) so that AxiMstPortMaxSize=8 is not
  // truncated on the NoC side.  The two packed struct layouts differ by 1 bit
  // per address channel, so direct port connection corrupts len/addr/id fields.
  // These signals bridge that gap with field-by-field copies.
  axi_mgr_slv_req_t  mgr_slv_req;   // cluster input adapted to FLOO_WIDE (4-bit size)
  axi_sbr_mst_req_t  sbr_mst_req;   // converter output before cluster adaptation (4-bit size)

  // ---------------------------------------------------------------------------
  // Cluster-boundary adaptation: cluster (3-bit size) ↔ converter (4-bit size)
  // ---------------------------------------------------------------------------

  // Manager path: cluster → converter  (zero-extend size to 4-bit)
  always_comb begin
    mgr_slv_req            = '0;
    mgr_slv_req.aw_valid   = axi_wide_ext_in_req_i.aw_valid;
    mgr_slv_req.aw         = '0;
    `AXI_SET_AW_STRUCT(mgr_slv_req.aw, axi_wide_ext_in_req_i.aw)
    mgr_slv_req.w_valid    = axi_wide_ext_in_req_i.w_valid;
    mgr_slv_req.w.data     = axi_wide_ext_in_req_i.w.data;
    mgr_slv_req.w.strb     = axi_wide_ext_in_req_i.w.strb;
    mgr_slv_req.w.last     = axi_wide_ext_in_req_i.w.last;
    mgr_slv_req.w.user     = axi_wide_ext_in_req_i.w.user;
    mgr_slv_req.b_ready    = axi_wide_ext_in_req_i.b_ready;
    mgr_slv_req.ar_valid   = axi_wide_ext_in_req_i.ar_valid;
    mgr_slv_req.ar         = '0;
    `AXI_SET_AR_STRUCT(mgr_slv_req.ar, axi_wide_ext_in_req_i.ar)
    mgr_slv_req.r_ready    = axi_wide_ext_in_req_i.r_ready;
  end

  // Subordinate path: converter → cluster  (truncate size to 3-bit; always safe
  //   since AxiMstPortMaxSize = $clog2(AxiCfgWExt.DataWidth/8) <= 6 for cluster)
  always_comb begin
    axi_wide_ext_out_req_o            = '0;
    axi_wide_ext_out_req_o.aw_valid   = sbr_mst_req.aw_valid;
    axi_wide_ext_out_req_o.aw         = '0;
    `AXI_SET_AW_STRUCT(axi_wide_ext_out_req_o.aw, sbr_mst_req.aw)
    axi_wide_ext_out_req_o.w_valid    = sbr_mst_req.w_valid;
    axi_wide_ext_out_req_o.w.data     = sbr_mst_req.w.data;
    axi_wide_ext_out_req_o.w.strb     = sbr_mst_req.w.strb;
    axi_wide_ext_out_req_o.w.last     = sbr_mst_req.w.last;
    axi_wide_ext_out_req_o.w.user     = sbr_mst_req.w.user;
    axi_wide_ext_out_req_o.b_ready    = sbr_mst_req.b_ready;
    axi_wide_ext_out_req_o.ar_valid   = sbr_mst_req.ar_valid;
    axi_wide_ext_out_req_o.ar         = '0;
    `AXI_SET_AR_STRUCT(axi_wide_ext_out_req_o.ar, sbr_mst_req.ar)
    axi_wide_ext_out_req_o.r_ready    = sbr_mst_req.r_ready;
  end

  // ---------------------------------------------------------------------------
  // User-field adaptation: NoC ↔ converter boundary
  //
  // The NoC AXI port carries AxiCfgW.UserWidth (= 1) user bits.
  // The converters run at AxiCfgWExt.UserWidth (= 52) user bits internally.
  // Truncate on the way out (mgr → NoC), zero-extend on the way in (NoC → sbr).
  // ---------------------------------------------------------------------------

  // Manager path: converter → NoC  (truncate user to AxiCfgW.UserWidth)
  always_comb begin
    axi_wide_in_req_o            = '0;
    axi_wide_in_req_o.aw_valid   = mgr_conv_req.aw_valid;
    axi_wide_in_req_o.aw         = '0;
    `AXI_SET_AW_STRUCT(axi_wide_in_req_o.aw, mgr_conv_req.aw)
    axi_wide_in_req_o.aw.user    = AxiCfgW.UserWidth'(mgr_conv_req.aw.user);
    axi_wide_in_req_o.w_valid    = mgr_conv_req.w_valid;
    axi_wide_in_req_o.w          = '0;
    axi_wide_in_req_o.w.data     = mgr_conv_req.w.data;
    axi_wide_in_req_o.w.strb     = mgr_conv_req.w.strb;
    axi_wide_in_req_o.w.last     = mgr_conv_req.w.last;
    axi_wide_in_req_o.w.user     = AxiCfgW.UserWidth'(mgr_conv_req.w.user);
    axi_wide_in_req_o.b_ready    = mgr_conv_req.b_ready;
    axi_wide_in_req_o.ar_valid   = mgr_conv_req.ar_valid;
    axi_wide_in_req_o.ar         = '0;
    `AXI_SET_AR_STRUCT(axi_wide_in_req_o.ar, mgr_conv_req.ar)
    axi_wide_in_req_o.ar.user    = AxiCfgW.UserWidth'(mgr_conv_req.ar.user);
    axi_wide_in_req_o.r_ready    = mgr_conv_req.r_ready;
  end

  // Manager path: NoC → converter  (zero-extend user to AxiCfgWExt.UserWidth)
  always_comb begin
    mgr_conv_resp            = '0;
    mgr_conv_resp.aw_ready   = axi_wide_in_rsp_i.aw_ready;
    mgr_conv_resp.w_ready    = axi_wide_in_rsp_i.w_ready;
    mgr_conv_resp.b_valid    = axi_wide_in_rsp_i.b_valid;
    mgr_conv_resp.b          = '0;
    `AXI_SET_B_STRUCT(mgr_conv_resp.b, axi_wide_in_rsp_i.b)
    mgr_conv_resp.b.user     = axi_ext_user_t'(axi_wide_in_rsp_i.b.user);
    mgr_conv_resp.ar_ready   = axi_wide_in_rsp_i.ar_ready;
    mgr_conv_resp.r_valid    = axi_wide_in_rsp_i.r_valid;
    mgr_conv_resp.r          = '0;
    mgr_conv_resp.r.data     = axi_wide_in_rsp_i.r.data;
    mgr_conv_resp.r.id       = axi_wide_in_rsp_i.r.id;
    mgr_conv_resp.r.resp     = axi_wide_in_rsp_i.r.resp;
    mgr_conv_resp.r.last     = axi_wide_in_rsp_i.r.last;
    mgr_conv_resp.r.user     = axi_ext_user_t'(axi_wide_in_rsp_i.r.user);
  end

  // Subordinate path: NoC → converter  (zero-extend user to AxiCfgWExt.UserWidth)
  always_comb begin
    sbr_conv_req             = '0;
    sbr_conv_req.aw_valid    = axi_wide_out_req_i.aw_valid;
    sbr_conv_req.aw          = '0;
    `AXI_SET_AW_STRUCT(sbr_conv_req.aw, axi_wide_out_req_i.aw)
    sbr_conv_req.aw.user     = axi_ext_user_t'(axi_wide_out_req_i.aw.user);
    sbr_conv_req.w_valid     = axi_wide_out_req_i.w_valid;
    sbr_conv_req.w           = '0;
    sbr_conv_req.w.data      = axi_wide_out_req_i.w.data;
    sbr_conv_req.w.strb      = axi_wide_out_req_i.w.strb;
    sbr_conv_req.w.last      = axi_wide_out_req_i.w.last;
    sbr_conv_req.w.user      = axi_ext_user_t'(axi_wide_out_req_i.w.user);
    sbr_conv_req.b_ready     = axi_wide_out_req_i.b_ready;
    sbr_conv_req.ar_valid    = axi_wide_out_req_i.ar_valid;
    sbr_conv_req.ar          = '0;
    `AXI_SET_AR_STRUCT(sbr_conv_req.ar, axi_wide_out_req_i.ar)
    sbr_conv_req.ar.user     = axi_ext_user_t'(axi_wide_out_req_i.ar.user);
    sbr_conv_req.r_ready     = axi_wide_out_req_i.r_ready;
  end

  // Subordinate path: converter → NoC  (truncate user to AxiCfgW.UserWidth)
  always_comb begin
    axi_wide_out_rsp_o           = '0;
    axi_wide_out_rsp_o.aw_ready  = sbr_conv_resp.aw_ready;
    axi_wide_out_rsp_o.w_ready   = sbr_conv_resp.w_ready;
    axi_wide_out_rsp_o.b_valid   = sbr_conv_resp.b_valid;
    axi_wide_out_rsp_o.b         = '0;
    `AXI_SET_B_STRUCT(axi_wide_out_rsp_o.b, sbr_conv_resp.b)
    axi_wide_out_rsp_o.b.user    = AxiCfgW.UserWidth'(sbr_conv_resp.b.user);
    axi_wide_out_rsp_o.ar_ready  = sbr_conv_resp.ar_ready;
    axi_wide_out_rsp_o.r_valid   = sbr_conv_resp.r_valid;
    axi_wide_out_rsp_o.r         = '0;
    axi_wide_out_rsp_o.r.data    = sbr_conv_resp.r.data;
    axi_wide_out_rsp_o.r.id      = sbr_conv_resp.r.id;
    axi_wide_out_rsp_o.r.resp    = sbr_conv_resp.r.resp;
    axi_wide_out_rsp_o.r.last    = sbr_conv_resp.r.last;
    axi_wide_out_rsp_o.r.user    = AxiCfgW.UserWidth'(sbr_conv_resp.r.user);
  end

  // ===========================================================================
  // Manager path: cluster DMA (512-bit) → NoC (1024-bit) (first example)
  //   axi_dw_converter selects axi_dw_upsizer because AxiMstPortDataWidth >
  //   AxiSlvPortDataWidth.
  // ===========================================================================

  floo_axi_dw_converter #(
    .AxiMaxReads         ( AxiMaxReads            ),
    .AxiSlvPortDataWidth ( AxiCfgWExt.DataWidth   ),
    .AxiMstPortDataWidth ( AxiCfgW.DataWidth      ),
    .AxiAddrWidth        ( AxiCfgW.AddrWidth      ),
    .AxiIdWidth          ( AxiCfgW.InIdWidth      ),
    .aw_chan_t            ( axi_mgr_aw_t           ),
    .slv_w_chan_t         ( axi_mgr_slv_w_t        ),
    .mst_w_chan_t         ( axi_mgr_mst_w_t        ),
    .b_chan_t             ( axi_mgr_b_t            ),
    .ar_chan_t            ( axi_mgr_ar_t           ),
    .slv_r_chan_t         ( axi_mgr_slv_r_t        ),
    .mst_r_chan_t         ( axi_mgr_mst_r_t        ),
    .axi_slv_req_t        ( axi_mgr_slv_req_t      ),
    .axi_slv_resp_t       ( axi_mgr_slv_resp_t     ),
    .axi_mst_req_t        ( axi_mgr_mst_req_t      ),
    .axi_mst_resp_t       ( axi_mgr_mst_resp_t     )
  ) i_mgr_dw_converter (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( mgr_slv_req            ),
    .slv_resp_o ( axi_wide_ext_in_rsp_o  ),
    .mst_req_o  ( mgr_conv_req           ),
    .mst_resp_i ( mgr_conv_resp          )
  );

  // ===========================================================================
  // Subordinate path: NoC (1024-bit) → cluster memory (512-bit)
  //   axi_dw_converter selects axi_dw_downsizer because AxiMstPortDataWidth <
  //   AxiSlvPortDataWidth.
  // ===========================================================================

  floo_axi_dw_converter #(
    .AxiMaxReads         ( AxiMaxReads            ),
    .AxiSlvPortDataWidth ( AxiCfgW.DataWidth      ),
    .AxiMstPortDataWidth ( AxiCfgWExt.DataWidth   ),
    .AxiAddrWidth        ( AxiCfgW.AddrWidth      ),
    .AxiIdWidth          ( AxiCfgW.OutIdWidth     ),
    .aw_chan_t            ( axi_sbr_aw_t           ),
    .slv_w_chan_t         ( axi_sbr_slv_w_t        ),
    .mst_w_chan_t         ( axi_sbr_mst_w_t        ),
    .b_chan_t             ( axi_sbr_b_t            ),
    .ar_chan_t            ( axi_sbr_ar_t           ),
    .slv_r_chan_t         ( axi_sbr_slv_r_t        ),
    .mst_r_chan_t         ( axi_sbr_mst_r_t        ),
    .axi_slv_req_t        ( axi_sbr_slv_req_t      ),
    .axi_slv_resp_t       ( axi_sbr_slv_resp_t     ),
    .axi_mst_req_t        ( axi_sbr_mst_req_t      ),
    .axi_mst_resp_t       ( axi_sbr_mst_resp_t     )
  ) i_sbr_dw_converter (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( sbr_conv_req           ),
    .slv_resp_o ( sbr_conv_resp          ),
    .mst_req_o  ( sbr_mst_req            ),
    .mst_resp_i ( axi_wide_ext_out_rsp_i )
  );

  // ---------------------------------------------------------------------------
  // Assertions
  // ---------------------------------------------------------------------------
  `ASSERT_INIT(WideExtDataWidthSmaller,
      AxiCfgWExt.DataWidth < AxiCfgW.DataWidth)
  `ASSERT_INIT(WideExtDataWidthDivisible,
      (AxiCfgW.DataWidth % AxiCfgWExt.DataWidth) == 0)
  `ASSERT_INIT(WideExtDataWidthPow2,
      (AxiCfgW.DataWidth / AxiCfgWExt.DataWidth & ((AxiCfgW.DataWidth / AxiCfgWExt.DataWidth) - 1)) == 0)
  `ASSERT_INIT(WideExtAddrWidthMatch,
      AxiCfgWExt.AddrWidth == AxiCfgW.AddrWidth)
  `ASSERT_INIT(WideExtInIdWidthMatch,
      AxiCfgWExt.InIdWidth == AxiCfgW.InIdWidth)
  `ASSERT_INIT(WideExtOutIdWidthMatch,
      AxiCfgWExt.OutIdWidth == AxiCfgW.OutIdWidth)

endmodule
