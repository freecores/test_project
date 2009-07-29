//module ref_design_top
module orpsoc_top
  (
   output spi_sd_sclk_pad_o  ,
   output spi_sd_ss_pad_o    ,
   input  spi_sd_miso_pad_i  ,
   output spi_sd_mosi_pad_o  ,
`ifdef USE_SDRAM
   // SDRAM bus signals
   inout [15:0]  mem_dat_pad_io,
   output [12:0] mem_adr_pad_o ,
   output [1:0]  mem_dqm_pad_o ,
   output [1:0]  mem_ba_pad_o  ,
   output 	 mem_cs_pad_o  ,
   output 	 mem_ras_pad_o ,
   output 	 mem_cas_pad_o ,
   output 	 mem_we_pad_o  ,
   output 	 mem_cke_pad_o ,
   // SPI bus signals for flash memory
   output spi_flash_sclk_pad_o  ,
   output spi_flash_ss_pad_o    ,
   input  spi_flash_miso_pad_i  ,
   output spi_flash_mosi_pad_o  ,  
   output spi_flash_w_n_pad_o   ,
   output spi_flash_hold_n_pad_o,
`endif //  `ifdef USE_SDRAM
`ifdef USE_ETHERNET
   output [1:1] eth_sync_pad_o,
   output [1:1] eth_tx_pad_o,
   input [1:1]  eth_rx_pad_i,
   input 	eth_clk_pad_i,
   inout [1:1]  eth_md_pad_io,
   output [1:1] eth_mdc_pad_o,   
`endif //  `ifdef USE_ETHERNET
   output spi1_mosi_pad_o,
   input  spi1_miso_pad_i,
   output spi1_ss_pad_o  ,
   output spi1_sclk_pad_o,
`ifdef DISABLE_IOS_FOR_VERILATOR
   output [8-1:0] gpio_a_pad_io,
`else   
   inout [8-1:0] gpio_a_pad_io,
`endif
   input  uart0_srx_pad_i ,  
   output uart0_stx_pad_o ,
   input  dbg_tdi_pad_i,
   input  dbg_tck_pad_i,
   input  dbg_tms_pad_i,  
   output dbg_tdo_pad_o,
   input rst_pad_i,
   output rst_pad_o,
   input clk_pad_i
   ) 
;
   wire 	 wb_rst;
   wire 	 wb_clk, clk50, clk100, usbClk, dbg_tck;
   wire 	 pll_lock;
   wire 	 mem_io_req, mem_io_gnt, mem_io_busy;
   wire [15:0] 	 mem_dat_pad_i, mem_dat_pad_o;
   wire [30:0] 	 pic_ints;
   wire 	 spi3_irq, spi2_irq, spi1_irq, spi0_irq, uart0_irq;
   wire 	 eth0_int_o;
parameter [31:0] wbm_or12_i_dat_o = 32'h0;
wire [31:0] wbm_or12_i_adr_o;
wire [3:0] wbm_or12_i_sel_o;
wire wbm_or12_i_we_o;
wire [1:0] wbm_or12_i_bte_o;
wire [2:0] wbm_or12_i_cti_o;
wire wbm_or12_i_stb_o;
wire wbm_or12_i_cyc_o;
wire [31:0] wbm_or12_i_dat_i;
wire wbm_or12_i_ack_i;
wire wbm_or12_i_err_i;
wire wbm_or12_i_rty_i;
wire [31:0] wbm_or12_debug_dat_o;
wire [31:0] wbm_or12_debug_adr_o;
wire [3:0] wbm_or12_debug_sel_o;
wire wbm_or12_debug_we_o;
wire [1:0] wbm_or12_debug_bte_o;
wire [2:0] wbm_or12_debug_cti_o;
wire wbm_or12_debug_stb_o;
wire wbm_or12_debug_cyc_o;
wire [31:0] wbm_or12_debug_dat_i;
wire wbm_or12_debug_ack_i;
wire wbm_or12_debug_err_i;
wire wbm_or12_debug_rty_i;
wire [31:0] wbm_or12_d_dat_o;
wire [31:0] wbm_or12_d_adr_o;
wire [3:0] wbm_or12_d_sel_o;
wire wbm_or12_d_we_o;
wire [1:0] wbm_or12_d_bte_o;
wire [2:0] wbm_or12_d_cti_o;
wire wbm_or12_d_stb_o;
wire wbm_or12_d_cyc_o;
wire [31:0] wbm_or12_d_dat_i;
wire wbm_or12_d_ack_i;
wire wbm_or12_d_err_i;
wire wbm_or12_d_rty_i;
wire [31:0] wbm_eth1_dat_o;
wire [31:0] wbm_eth1_adr_o;
wire [3:0] wbm_eth1_sel_o;
wire wbm_eth1_we_o;
wire [1:0] wbm_eth1_bte_o;
wire [2:0] wbm_eth1_cti_o;
wire wbm_eth1_stb_o;
wire wbm_eth1_cyc_o;
wire [31:0] wbm_eth1_dat_i;
wire wbm_eth1_ack_i;
wire wbm_eth1_err_i;
wire wbm_eth1_rty_i;
wire [31:0] wbs_eth1_cfg_dat_o;
wire [31:0] wbs_eth1_cfg_dat_i;
wire [31:0] wbs_eth1_cfg_adr_i;
wire [3:0] wbs_eth1_cfg_sel_i;
wire [1:0] wbs_eth1_cfg_bte_i;
wire [2:0] wbs_eth1_cfg_cti_i;
wire wbs_eth1_cfg_stb_i;
wire wbs_eth1_cfg_cyc_i;
wire wbs_eth1_cfg_ack_o;
wire wbs_eth1_cfg_err_o;
parameter wbs_eth1_cfg_rty_o = 1'b0;
wire [31:0] wbs_rom_dat_o;
wire [31:0] wbs_rom_dat_i;
wire [31:0] wbs_rom_adr_i;
wire [3:0] wbs_rom_sel_i;
wire [1:0] wbs_rom_bte_i;
wire [2:0] wbs_rom_cti_i;
wire wbs_rom_stb_i;
wire wbs_rom_cyc_i;
wire wbs_rom_ack_o;
parameter wbs_rom_err_o = 1'b0;
parameter wbs_rom_rty_o = 1'b0;
wire [31:0] wbs_mc_m_dat_o;
wire [31:0] wbs_mc_m_dat_i;
wire [31:0] wbs_mc_m_adr_i;
wire [3:0] wbs_mc_m_sel_i;
wire [1:0] wbs_mc_m_bte_i;
wire [2:0] wbs_mc_m_cti_i;
wire wbs_mc_m_stb_i;
wire wbs_mc_m_cyc_i;
wire wbs_mc_m_ack_o;
wire wbs_mc_m_err_o;
parameter wbs_mc_m_rty_o = 1'b0;
wire [31:0] wbs_spi_flash_dat_o;
wire [31:0] wbs_spi_flash_dat_i;
wire [31:0] wbs_spi_flash_adr_i;
wire [3:0] wbs_spi_flash_sel_i;
wire [1:0] wbs_spi_flash_bte_i;
wire [2:0] wbs_spi_flash_cti_i;
wire wbs_spi_flash_stb_i;
wire wbs_spi_flash_cyc_i;
wire wbs_spi_flash_ack_o;
parameter wbs_spi_flash_err_o = 1'b0;
parameter wbs_spi_flash_rty_o = 1'b0;
wire [31:0] wbs_uart0_dat_o;
wire [31:0] wbs_uart0_dat_i;
wire [31:0] wbs_uart0_adr_i;
wire [3:0] wbs_uart0_sel_i;
wire [1:0] wbs_uart0_bte_i;
wire [2:0] wbs_uart0_cti_i;
wire wbs_uart0_stb_i;
wire wbs_uart0_cyc_i;
wire wbs_uart0_ack_o;
parameter wbs_uart0_err_o = 1'b0;
parameter wbs_uart0_rty_o = 1'b0;
wire [31:0] wbs_ds1_dat_o;
wire [31:0] wbs_ds1_dat_i;
wire [31:0] wbs_ds1_adr_i;
wire [3:0] wbs_ds1_sel_i;
wire [1:0] wbs_ds1_bte_i;
wire [2:0] wbs_ds1_cti_i;
wire wbs_ds1_stb_i;
wire wbs_ds1_cyc_i;
wire wbs_ds1_ack_o;
parameter wbs_ds1_err_o = 1'b0;
parameter wbs_ds1_rty_o = 1'b0;
wire [31:0] wbs_ds2_dat_o;
wire [31:0] wbs_ds2_dat_i;
wire [31:0] wbs_ds2_adr_i;
wire [3:0] wbs_ds2_sel_i;
wire [1:0] wbs_ds2_bte_i;
wire [2:0] wbs_ds2_cti_i;
wire wbs_ds2_stb_i;
wire wbs_ds2_cyc_i;
wire wbs_ds2_ack_o;
parameter wbs_ds2_err_o = 1'b0;
parameter wbs_ds2_rty_o = 1'b0;
   wire 	       eth_clk;
   wire [1:1] eth_int;
intercon intercon1 (
    .wbm_or12_isaw_dat_o(wbm_or12_i_dat_o),
    .wbm_or12_isaw_adr_o(wbm_or12_i_adr_o),
    .wbm_or12_isaw_sel_o(wbm_or12_i_sel_o),
    .wbm_or12_isaw_we_o(wbm_or12_i_we_o),
    .wbm_or12_isaw_bte_o(wbm_or12_i_bte_o),
    .wbm_or12_isaw_cti_o(wbm_or12_i_cti_o),
    .wbm_or12_isaw_stb_o(wbm_or12_i_stb_o),
    .wbm_or12_isaw_cyc_o(wbm_or12_i_cyc_o),
    .wbm_or12_isaw_dat_i(wbm_or12_i_dat_i),
    .wbm_or12_isaw_ack_i(wbm_or12_i_ack_i),
    .wbm_or12_isaw_err_i(wbm_or12_i_err_i),
    .wbm_or12_isaw_rty_i(wbm_or12_i_rty_i),
    .wbm_or12_debug_dat_o(wbm_or12_debug_dat_o),
    .wbm_or12_debug_adr_o(wbm_or12_debug_adr_o),
    .wbm_or12_debug_sel_o(wbm_or12_debug_sel_o),
    .wbm_or12_debug_we_o(wbm_or12_debug_we_o),
    .wbm_or12_debug_bte_o(wbm_or12_debug_bte_o),
    .wbm_or12_debug_cti_o(wbm_or12_debug_cti_o),
    .wbm_or12_debug_stb_o(wbm_or12_debug_stb_o),
    .wbm_or12_debug_cyc_o(wbm_or12_debug_cyc_o),
    .wbm_or12_debug_dat_i(wbm_or12_debug_dat_i),
    .wbm_or12_debug_ack_i(wbm_or12_debug_ack_i),
    .wbm_or12_debug_err_i(wbm_or12_debug_err_i),
    .wbm_or12_debug_rty_i(wbm_or12_debug_rty_i),
    .wbm_or12_d_dat_o(wbm_or12_d_dat_o),
    .wbm_or12_d_adr_o(wbm_or12_d_adr_o),
    .wbm_or12_d_sel_o(wbm_or12_d_sel_o),
    .wbm_or12_d_we_o(wbm_or12_d_we_o),
    .wbm_or12_d_bte_o(wbm_or12_d_bte_o),
    .wbm_or12_d_cti_o(wbm_or12_d_cti_o),
    .wbm_or12_d_stb_o(wbm_or12_d_stb_o),
    .wbm_or12_d_cyc_o(wbm_or12_d_cyc_o),
    .wbm_or12_d_dat_i(wbm_or12_d_dat_i),
    .wbm_or12_d_ack_i(wbm_or12_d_ack_i),
    .wbm_or12_d_err_i(wbm_or12_d_err_i),
    .wbm_or12_d_rty_i(wbm_or12_d_rty_i),
    .wbm_eth1_dat_o(wbm_eth1_dat_o),
    .wbm_eth1_adr_o(wbm_eth1_adr_o),
    .wbm_eth1_sel_o(wbm_eth1_sel_o),
    .wbm_eth1_we_o(wbm_eth1_we_o),
    .wbm_eth1_bte_o(wbm_eth1_bte_o),
    .wbm_eth1_cti_o(wbm_eth1_cti_o),
    .wbm_eth1_stb_o(wbm_eth1_stb_o),
    .wbm_eth1_cyc_o(wbm_eth1_cyc_o),
    .wbm_eth1_dat_i(wbm_eth1_dat_i),
    .wbm_eth1_ack_i(wbm_eth1_ack_i),
    .wbm_eth1_err_i(wbm_eth1_err_i),
    .wbm_eth1_rty_i(wbm_eth1_rty_i),
    .wbs_eth1_cfg_dat_i(wbs_eth1_cfg_dat_i),
    .wbs_eth1_cfg_adr_i(wbs_eth1_cfg_adr_i),
    .wbs_eth1_cfg_sel_i(wbs_eth1_cfg_sel_i),
    .wbs_eth1_cfg_we_i(wbs_eth1_cfg_we_i),
    .wbs_eth1_cfg_bte_i(wbs_eth1_cfg_bte_i),
    .wbs_eth1_cfg_cti_i(wbs_eth1_cfg_cti_i),
    .wbs_eth1_cfg_stb_i(wbs_eth1_cfg_stb_i),
    .wbs_eth1_cfg_cyc_i(wbs_eth1_cfg_cyc_i),
    .wbs_eth1_cfg_dat_o(wbs_eth1_cfg_dat_o),
    .wbs_eth1_cfg_ack_o(wbs_eth1_cfg_ack_o),
    .wbs_eth1_cfg_err_o(wbs_eth1_cfg_err_o),
    .wbs_eth1_cfg_rty_o(wbs_eth1_cfg_rty_o),
    .wbs_rom_dat_i(wbs_rom_dat_i),
    .wbs_rom_adr_i(wbs_rom_adr_i),
    .wbs_rom_sel_i(wbs_rom_sel_i),
    .wbs_rom_we_i(wbs_rom_we_i),
    .wbs_rom_bte_i(wbs_rom_bte_i),
    .wbs_rom_cti_i(wbs_rom_cti_i),
    .wbs_rom_stb_i(wbs_rom_stb_i),
    .wbs_rom_cyc_i(wbs_rom_cyc_i),
    .wbs_rom_dat_o(wbs_rom_dat_o),
    .wbs_rom_ack_o(wbs_rom_ack_o),
    .wbs_rom_err_o(wbs_rom_err_o),
    .wbs_rom_rty_o(wbs_rom_rty_o),
    .wbs_mc_m_dat_i(wbs_mc_m_dat_i),
    .wbs_mc_m_adr_i(wbs_mc_m_adr_i),
    .wbs_mc_m_sel_i(wbs_mc_m_sel_i),
    .wbs_mc_m_we_i(wbs_mc_m_we_i),
    .wbs_mc_m_bte_i(wbs_mc_m_bte_i),
    .wbs_mc_m_cti_i(wbs_mc_m_cti_i),
    .wbs_mc_m_stb_i(wbs_mc_m_stb_i),
    .wbs_mc_m_cyc_i(wbs_mc_m_cyc_i),
    .wbs_mc_m_dat_o(wbs_mc_m_dat_o),
    .wbs_mc_m_ack_o(wbs_mc_m_ack_o),
    .wbs_mc_m_err_o(wbs_mc_m_err_o),
    .wbs_mc_m_rty_o(wbs_mc_m_rty_o),
    .wbs_spi_flash_dat_i(wbs_spi_flash_dat_i),
    .wbs_spi_flash_adr_i(wbs_spi_flash_adr_i),
    .wbs_spi_flash_sel_i(wbs_spi_flash_sel_i),
    .wbs_spi_flash_we_i(wbs_spi_flash_we_i),
    .wbs_spi_flash_bte_i(wbs_spi_flash_bte_i),
    .wbs_spi_flash_cti_i(wbs_spi_flash_cti_i),
    .wbs_spi_flash_stb_i(wbs_spi_flash_stb_i),
    .wbs_spi_flash_cyc_i(wbs_spi_flash_cyc_i),
    .wbs_spi_flash_dat_o(wbs_spi_flash_dat_o),
    .wbs_spi_flash_ack_o(wbs_spi_flash_ack_o),
    .wbs_spi_flash_err_o(wbs_spi_flash_err_o),
    .wbs_spi_flash_rty_o(wbs_spi_flash_rty_o),
    .wbs_uart0_dat_i(wbs_uart0_dat_i),
    .wbs_uart0_adr_i(wbs_uart0_adr_i),
    .wbs_uart0_sel_i(wbs_uart0_sel_i),
    .wbs_uart0_we_i(wbs_uart0_we_i),
    .wbs_uart0_bte_i(wbs_uart0_bte_i),
    .wbs_uart0_cti_i(wbs_uart0_cti_i),
    .wbs_uart0_stb_i(wbs_uart0_stb_i),
    .wbs_uart0_cyc_i(wbs_uart0_cyc_i),
    .wbs_uart0_dat_o(wbs_uart0_dat_o),
    .wbs_uart0_ack_o(wbs_uart0_ack_o),
    .wbs_uart0_err_o(wbs_uart0_err_o),
    .wbs_uart0_rty_o(wbs_uart0_rty_o),
    .wbs_ds1_dat_i(wbs_ds1_dat_i),
    .wbs_ds1_adr_i(wbs_ds1_adr_i),
    .wbs_ds1_sel_i(wbs_ds1_sel_i),
    .wbs_ds1_we_i(wbs_ds1_we_i),
    .wbs_ds1_bte_i(wbs_ds1_bte_i),
    .wbs_ds1_cti_i(wbs_ds1_cti_i),
    .wbs_ds1_stb_i(wbs_ds1_stb_i),
    .wbs_ds1_cyc_i(wbs_ds1_cyc_i),
    .wbs_ds1_dat_o(wbs_ds1_dat_o),
    .wbs_ds1_ack_o(wbs_ds1_ack_o),
    .wbs_ds1_err_o(wbs_ds1_err_o),
    .wbs_ds1_rty_o(wbs_ds1_rty_o),
    .wbs_ds2_dat_i(wbs_ds2_dat_i),
    .wbs_ds2_adr_i(wbs_ds2_adr_i),
    .wbs_ds2_sel_i(wbs_ds2_sel_i),
    .wbs_ds2_we_i(wbs_ds2_we_i),
    .wbs_ds2_bte_i(wbs_ds2_bte_i),
    .wbs_ds2_cti_i(wbs_ds2_cti_i),
    .wbs_ds2_stb_i(wbs_ds2_stb_i),
    .wbs_ds2_cyc_i(wbs_ds2_cyc_i),
    .wbs_ds2_dat_o(wbs_ds2_dat_o),
    .wbs_ds2_ack_o(wbs_ds2_ack_o),
    .wbs_ds2_err_o(wbs_ds2_err_o),
    .wbs_ds2_rty_o(wbs_ds2_rty_o),
    .wb_clk_i(wb_clk),
    .wb_rst_i(wb_rst)
);
    assign 	 pic_ints[30] = 1'b0;
   assign 	 pic_ints[29] = 1'b0;
   assign 	 pic_ints[28] = 1'b0;
   assign 	 pic_ints[27] = 1'b0;
   assign 	 pic_ints[26] = 1'b0;
   assign 	 pic_ints[25] = 1'b0;
   assign 	 pic_ints[24] = 1'b0;
   assign 	 pic_ints[23] = 1'b0;
   assign 	 pic_ints[22] = 1'b0;
   assign 	 pic_ints[21] = 1'b0;
   assign 	 pic_ints[20] = 1'b0;
   assign 	 pic_ints[19] = 1'b0;
   assign 	 pic_ints[18] = 1'b0;
   assign 	 pic_ints[17] = 1'b0;
   assign 	 pic_ints[16] = 1'b0;
   assign 	 pic_ints[15] = 1'b0;
   assign 	 pic_ints[14] = 1'b0;
   assign 	 pic_ints[13] = 1'b0;
   assign 	 pic_ints[12] = 1'b0;
   assign 	 pic_ints[11] = 1'b0;
   assign 	 pic_ints[10] = 1'b0;
   assign 	 pic_ints[9]  = 1'b0;
   assign 	 pic_ints[8]  = 1'b0;
   assign 	 pic_ints[7] = 1'b0;
   assign 	 pic_ints[6] = 1'b0;
   assign 	 pic_ints[5] = 1'b0;
   assign 	 pic_ints[4] = 1'b0;
   assign 	 pic_ints[3] = 1'b0;
   assign 	 pic_ints[2] = uart0_irq;
   assign 	 pic_ints[1] = 1'b0;
   assign 	 pic_ints[0] = 1'b0;
    or1k_top i_or1k
     (
      .clk_i      (wb_clk),
      .rst_i      (wb_rst), 
      .pic_ints_i (pic_ints[19:0]),
      .iwb_clk_i  (wb_clk), 
      .iwb_rst_i  (wb_rst), 
      .iwb_ack_i  (wbm_or12_i_ack_i), 
      .iwb_err_i  (wbm_or12_i_err_i), 
      .iwb_rty_i  (wbm_or12_i_rty_i), 
      .iwb_dat_i  (wbm_or12_i_dat_i),
      .iwb_cyc_o  (wbm_or12_i_cyc_o), 
      .iwb_adr_o  (wbm_or12_i_adr_o), 
      .iwb_stb_o  (wbm_or12_i_stb_o), 
      .iwb_we_o   (wbm_or12_i_we_o ), 
      .iwb_sel_o  (wbm_or12_i_sel_o), 
      .iwb_cti_o  (wbm_or12_i_cti_o), 
      .iwb_bte_o  (wbm_or12_i_bte_o),
      .dwb_clk_i  (wb_clk), 
      .dwb_rst_i  (wb_rst), 
      .dwb_ack_i  (wbm_or12_d_ack_i), 
      .dwb_err_i  (wbm_or12_d_err_i), 
      .dwb_rty_i  (wbm_or12_d_rty_i), 
      .dwb_dat_i  (wbm_or12_d_dat_i),
      .dwb_cyc_o  (wbm_or12_d_cyc_o), 
      .dwb_adr_o  (wbm_or12_d_adr_o), 
      .dwb_stb_o  (wbm_or12_d_stb_o), 
      .dwb_we_o   (wbm_or12_d_we_o), 
      .dwb_sel_o  (wbm_or12_d_sel_o), 
      .dwb_dat_o  (wbm_or12_d_dat_o),
      .dwb_cti_o  (wbm_or12_d_cti_o), 
      .dwb_bte_o  (wbm_or12_d_bte_o),
      .dbgwb_clk_i (wb_clk), 
      .dbgwb_rst_i (wb_rst), 
      .dbgwb_ack_i (wbm_or12_debug_ack_i), 
      .dbgwb_err_i (wbm_or12_debug_err_i), 
      .dbgwb_dat_i (wbm_or12_debug_dat_i),
      .dbgwb_cyc_o (wbm_or12_debug_cyc_o), 
      .dbgwb_adr_o (wbm_or12_debug_adr_o), 
      .dbgwb_stb_o (wbm_or12_debug_stb_o), 
      .dbgwb_we_o  (wbm_or12_debug_we_o), 
      .dbgwb_sel_o (wbm_or12_debug_sel_o), 
      .dbgwb_dat_o (wbm_or12_debug_dat_o),
      .dbgwb_cti_o (wbm_or12_debug_cti_o), 
      .dbgwb_bte_o (wbm_or12_debug_bte_o),  
      .tms_pad_i   (dbg_tms_pad_i), 
      .tck_pad_i   (dbg_tck),
      .tdi_pad_i   (dbg_tdi_pad_i),
      .tdo_pad_o   (dbg_tdo_pad_o),
      .tdo_padoe_o (             )             
      );
 OR1K_startup OR1K_startup0
  (
    .wb_adr_i(wbs_rom_adr_i[6:2]),
    .wb_stb_i(wbs_rom_stb_i),
    .wb_cyc_i(wbs_rom_cyc_i),
    .wb_dat_o(wbs_rom_dat_o),
    .wb_ack_o(wbs_rom_ack_o),
    .wb_clk(wb_clk),
    .wb_rst(wb_rst)
   );
wire spi_flash_mosi, spi_flash_miso, spi_flash_sclk;
wire [1:0] spi_flash_ss;
spi_flash_top #
  (
   .divider(0),
   .divider_len(2)
   )
spi_flash_top0
  (
   .wb_clk_i(wb_clk), 
   .wb_rst_i(wb_rst),
   .wb_adr_i(wbs_spi_flash_adr_i[4:2]),
   .wb_dat_i(wbs_spi_flash_dat_i), 
   .wb_dat_o(wbs_spi_flash_dat_o),
   .wb_sel_i(wbs_spi_flash_sel_i),
   .wb_we_i(wbs_spi_flash_we_i),
   .wb_stb_i(wbs_spi_flash_stb_i), 
   .wb_cyc_i(wbs_spi_flash_cyc_i),
   .wb_ack_o(wbs_spi_flash_ack_o), 
   .mosi_pad_o(spi_flash_mosi),
   .miso_pad_i(spi_flash_miso),
   .sclk_pad_o(spi_flash_sclk),
   .ss_pad_o(spi_flash_ss)
   );


`ifdef USE_SDRAM
  wb_sdram_ctrl wb_sdram_ctrl0
  (
    .wb_dat_i(wbs_mc_m_dat_i),
    .wb_dat_o(wbs_mc_m_dat_o),
    .wb_sel_i(wbs_mc_m_sel_i),
    .wb_adr_i(wbs_mc_m_adr_i[24:2]),
    .wb_we_i (wbs_mc_m_we_i),
    .wb_cti_i(wbs_mc_m_cti_i),
    .wb_stb_i(wbs_mc_m_stb_i),
    .wb_cyc_i(wbs_mc_m_cyc_i),
    .wb_ack_o(wbs_mc_m_ack_o),
    .sdr_cke_o(mem_cke_pad_o),   
    .sdr_cs_n_o(mem_cs_pad_o),  
    .sdr_ras_n_o(mem_ras_pad_o), 
    .sdr_cas_n_o(mem_cas_pad_o), 
    .sdr_we_n_o(mem_we_pad_o),  
    .sdr_a_o(mem_adr_pad_o),
    .sdr_ba_o(mem_ba_pad_o),
    .sdr_dq_io(mem_dat_pad_io),
    .sdr_dqm_o(mem_dqm_pad_o),
    .sdram_clk(wb_clk),
    .wb_clk(wb_clk),
    .wb_rst(wb_rst)
   );

   // SPI flash memory signals
   assign spi_flash_mosi_pad_o = !spi_flash_ss[0] ? spi_flash_mosi : 1'b1;
   assign spi_flash_sclk_pad_o = !spi_flash_ss[0] ? spi_flash_sclk : 1'b1;
   assign spi_flash_ss_pad_o   =  spi_flash_ss[0];
   assign spi_flash_w_n_pad_o    = 1'b1;
   assign spi_flash_hold_n_pad_o = 1'b1;
   assign spi_sd_mosi_pad_o = !spi_flash_ss[1] ? spi_flash_mosi : 1'b1;
   assign spi_sd_sclk_pad_o = !spi_flash_ss[1] ? spi_flash_sclk : 1'b1;
   assign spi_sd_ss_pad_o   =  spi_flash_ss[1];
   assign spi_flash_miso = !spi_flash_ss[0] ? spi_flash_miso_pad_i :
			!spi_flash_ss[1] ? spi_sd_miso_pad_i :
			1'b0;
   
`else // !`ifdef USE_SDRAM
   
   parameter ram_wb_dat_width = 32;
   parameter ram_wb_adr_width = 24;
   //parameter ram_wb_mem_size  = 2097152; // 8MB
   parameter ram_wb_mem_size  = 8388608; // 32MB -- for linux test

  ram_wb
    #
    (
     .dat_width(ram_wb_dat_width),
     .adr_width(ram_wb_adr_width),
     .mem_size(ram_wb_mem_size)
     )
   ram_wb0
   (
    .dat_i(wbs_mc_m_dat_i),
    .dat_o(wbs_mc_m_dat_o),
    .sel_i(wbs_mc_m_sel_i),
    .adr_i(wbs_mc_m_adr_i[ram_wb_adr_width-1:2]),
    .we_i (wbs_mc_m_we_i),
    .cti_i(wbs_mc_m_cti_i),
    .stb_i(wbs_mc_m_stb_i),
    .cyc_i(wbs_mc_m_cyc_i),
    .ack_o(wbs_mc_m_ack_o),
    .clk_i(wb_clk),
    .rst_i(wb_rst)
   );

`endif // !`ifdef USE_SDRAM

assign wbs_mc_m_err_o = 1'b0;

     uart_top 
     #( 32, 5) 
   i_uart_0_top
     (
      .wb_dat_o   (wbs_uart0_dat_o),
      .wb_dat_i   (wbs_uart0_dat_i),
      .wb_sel_i   (wbs_uart0_sel_i),
      .wb_adr_i   (wbs_uart0_adr_i[4:0]),
      .wb_we_i    (wbs_uart0_we_i),
      .wb_stb_i   (wbs_uart0_stb_i),
      .wb_cyc_i   (wbs_uart0_cyc_i),
      .wb_ack_o   (wbs_uart0_ack_o),
      .wb_clk_i   (wb_clk),
      .wb_rst_i   (wb_rst),
      .int_o      (uart0_irq),
      .srx_pad_i  (uart0_srx_pad_i),
      .stx_pad_o  (uart0_stx_pad_o),
      .cts_pad_i  (1'b0),
      .rts_pad_o  ( ),
      .dtr_pad_o  ( ),
      .dcd_pad_i  (1'b0),
      .dsr_pad_i  (1'b0),
      .ri_pad_i   (1'b0)
      );
   assign gpio_a_pad_io[7:0] = 8'hfe;

`ifdef USE_ETHERNET   
  wire 	     m1tx_clk;
wire [3:0] 	     m1txd;
wire 	     m1txen;
wire 	     m1txerr;
wire 	     m1rx_clk;
wire [3:0] 	     m1rxd;
wire 	     m1rxdv;
wire 	     m1rxerr;
wire 	     m1coll;
wire 	     m1crs;   
//wire [1:10] 	     state;
wire [10:1] 	     state;   // Changed for verilator -- jb
wire              sync;
wire [1:1]    rx, tx;
wire [1:1]    mdc_o, md_i, md_o, md_oe;
smii_sync smii_sync1
  (
   .sync(sync),
   .state(state),
   .clk(eth_clk),
   .rst(wb_rst)
   );
eth_top eth_top1
	(
	 .wb_clk_i(wb_clk),
	 .wb_rst_i(wb_rst),
	 .wb_dat_i(wbs_eth1_cfg_dat_i),
	 .wb_dat_o(wbs_eth1_cfg_dat_o),
	 .wb_adr_i(wbs_eth1_cfg_adr_i[11:2]),
	 .wb_sel_i(wbs_eth1_cfg_sel_i),
	 .wb_we_i(wbs_eth1_cfg_we_i),
	 .wb_cyc_i(wbs_eth1_cfg_cyc_i),
	 .wb_stb_i(wbs_eth1_cfg_stb_i),
	 .wb_ack_o(wbs_eth1_cfg_ack_o),
	 .wb_err_o(wbs_eth1_cfg_err_o),
	 .m_wb_adr_o(wbm_eth1_adr_o),
	 .m_wb_sel_o(wbm_eth1_sel_o),
	 .m_wb_we_o(wbm_eth1_we_o),
	 .m_wb_dat_o(wbm_eth1_dat_o),
	 .m_wb_dat_i(wbm_eth1_dat_i),
	 .m_wb_cyc_o(wbm_eth1_cyc_o),
	 .m_wb_stb_o(wbm_eth1_stb_o),
	 .m_wb_ack_i(wbm_eth1_ack_i),
	 .m_wb_err_i(wbm_eth1_err_i),
	 .m_wb_cti_o(wbm_eth1_cti_o),
	 .m_wb_bte_o(wbm_eth1_bte_o),
	 .mtx_clk_pad_i(m1tx_clk),
	 .mtxd_pad_o(m1txd),
	 .mtxen_pad_o(m1txen),
	 .mtxerr_pad_o(m1txerr),
	 .mrx_clk_pad_i(m1rx_clk),
	 .mrxd_pad_i(m1rxd),
	 .mrxdv_pad_i(m1rxdv),
	 .mrxerr_pad_i(m1rxerr),
	 .mcoll_pad_i(m1coll),
	 .mcrs_pad_i(m1crs),
	 .mdc_pad_o(mdc_o[1]),
	 .md_pad_i(md_i[1]),
	 .md_pad_o(md_o[1]),
	 .md_padoe_o(md_oe[1]),
	 .int_o(eth_int[1])
	 );
iobuftri iobuftri1
  (
   .i(md_o[1]),
   .oe(md_oe[1]),
   .o(md_i[1]),
   .pad(eth_md_pad_io[1])
   );
obuf obuf1
  (
   .i(mdc_o[1]),
   .pad(eth_mdc_pad_o[1])
   );
smii_txrx smii_txrx1
  (
   .tx(tx[1]),
   .rx(rx[1]),
   .mtx_clk(m1tx_clk),
   .mtxd(m1txd),
   .mtxen(m1txen),
   .mtxerr(m1txerr),
   .mrx_clk(m1rx_clk),
   .mrxd(m1rxd),
   .mrxdv(m1rxdv),
   .mrxerr(m1rxerr),
   .mcoll(m1coll),
   .mcrs(m1crs),
   .state(state),
   .clk(eth_clk),
   .rst(wb_rst)
   );
obufdff obufdff_sync1
  (
   .d(sync),
   .pad(eth_sync_pad_o[1]),
   .clk(eth_clk),
   .rst(wb_rst)
   );
obufdff obufdff_tx1
  (
   .d(tx[1]),
   .pad(eth_tx_pad_o[1]),
   .clk(eth_clk),
   .rst(wb_rst)
   );
ibufdff ibufdff_rx1
  (
   .pad(eth_rx_pad_i[1]),
   .q(rx[1]),
   .clk(eth_clk),
   .rst(wb_rst)
   );
`else // !`ifdef USE_ETHERNET
   // If ethernet core is disabled, still ack anyone who tries
   // to access its config port. This allows linux to boot in
   // the verilated ORPSoC.
   reg 	      wbs_eth1_cfg_ack_r;
   always @(posedge wb_clk) 
     wbs_eth1_cfg_ack_r <= (wbs_eth1_cfg_cyc_i & wbs_eth1_cfg_stb_i);
   
   // Tie off WB arbitor inputs
   assign wbs_eth1_cfg_dat_o = 0;
   assign wbs_eth1_cfg_ack_o = wbs_eth1_cfg_ack_r;
   assign wbs_eth1_cfg_err_o = 0;
   // Tie off ethernet master ctrl signals
   assign wbm_eth1_adr_o = 0;
   assign wbm_eth1_sel_o = 0;
   assign wbm_eth1_we_o = 0;
   assign wbm_eth1_dat_o = 0;
   assign wbm_eth1_cyc_o = 0;
   assign wbm_eth1_stb_o = 0;
   assign wbm_eth1_cti_o = 0;
   assign wbm_eth1_bte_o = 0;
`endif //  `ifdef USE_ETHERNET
   
   dummy_slave
     # ( .value(32'hc0000000))
   ds1 
     ( 
       .dat_o(wbs_ds1_dat_o), 
       .stb_i(wbs_ds1_stb_i), 
       .cyc_i(wbs_ds1_cyc_i), 
       .ack_o(wbs_ds1_ack_o), 
       .clk(wb_clk), 
       .rst(wb_rst) 
       );
   dummy_slave 
     # ( .value(32'hf0000000))
     ds2
     ( 
       .dat_o(wbs_ds2_dat_o), 
       .stb_i(wbs_ds2_stb_i), 
       .cyc_i(wbs_ds2_cyc_i), 
       .ack_o(wbs_ds2_ack_o), 
       .clk(wb_clk), 
       .rst(wb_rst) 
       );
   clk_gen iclk_gen 
     (
      .POWERDOWN (1'b1),
      .CLKA (clk_pad_i),
      .LOCK (pll_lock),
      .GLA(wb_clk),
      .GLB(usbClk_pll),
      .GLC()
      );
   
   assign rst_pad_o = pll_lock;
   
   gbuf gbufi1
     (
      .CLK(~(pll_lock & rst_pad_i)),
      .GL(wb_rst));
   gbuf gbufi2
     (
      .CLK(dbg_tck_pad_i),
      .GL(dbg_tck));
   gbuf gbufi3
     (
      .CLK(usbClk_pll),
      .GL(usbClk));
   gbuf gbufi4
     (
      .CLK(eth_clk_pad_i),
      .GL(eth_clk));

   
endmodule
