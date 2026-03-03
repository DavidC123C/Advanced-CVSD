module top # (
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 15,
    parameter STRB_WIDTH = (DATA_WIDTH/8)
)
(
    // Clock and Active-Low Reset
    input clk_top,
    input clk_tb ,
    input clk_ram,
    input rst_n  ,

    // Finish Flag
    output reg finish,

    // AXI port
    output [           ADDR_WIDTH-1:0] awaddr ,
    output [                      7:0] awlen  ,
    output [                      2:0] awsize ,
    output [                      1:0] awburst,
    output                             awvalid,
    input                              awready,

    output [           DATA_WIDTH-1:0] wdata  ,
    output [           STRB_WIDTH-1:0] wstrb  ,
    output                             wlast  ,
    output                             wvalid ,
    input                              wready , 

    input  [                      1:0] bresp  ,
    input                              bvalid ,
    output                             bready ,

    output [           ADDR_WIDTH-1:0] araddr ,
    output [                      7:0] arlen  ,
    output [                      2:0] arsize ,
    output [                      1:0] arburst,
    output                             arvalid,
    input                              arready,

    input  [           DATA_WIDTH-1:0] rdata  ,
    input  [                      1:0] rresp  ,
    input                              rlast  ,
    input                              rvalid ,
    output                             rready
);

parameter IDLE = 0;
parameter READ = 1; 
parameter UNSKIP = 2;
parameter PARAMET = 3;
parameter COMPUTE = 4;
parameter BACK = 5;
parameter FINISH = 6;
integer i;
integer j;



reg [14:0] bias, nxt_bias;
reg [7:0] kernel_0, nxt_kernel_0;
reg [7:0] kernel_1, nxt_kernel_1;
reg [7:0] kernel_2, nxt_kernel_2;
reg [7:0] kernel_3, nxt_kernel_3;
reg [7:0] kernel_4, nxt_kernel_4;
reg [7:0] kernel_5, nxt_kernel_5;
reg [7:0] kernel_6, nxt_kernel_6;
reg [7:0] kernel_7, nxt_kernel_7;
reg [7:0] kernel_8, nxt_kernel_8;
reg [15:0] mul_0, nxt_mul_0;
reg [15:0] mul_1, nxt_mul_1;
reg [15:0] mul_2, nxt_mul_2;
reg [15:0] mul_3, nxt_mul_3;
reg [15:0] mul_4, nxt_mul_4;
reg [15:0] mul_5, nxt_mul_5;
reg [15:0] mul_6, nxt_mul_6;
reg [15:0] mul_7, nxt_mul_7;
reg [15:0] mul_8, nxt_mul_8;


reg [24:0] scale, nxt_scale;
reg [7:0] skip, nxt_skip;
wire [42:0] tmp_quanti;

reg [17:0] max_pool;
reg [3:0] para_cnt, nxt_para_cnt;
reg [2:0] state, nxt_state;
reg awvalid_w, awvalid_r;

wire [8:0] round;
reg [8:0] datain_cnt, nxt_datain_cnt;
reg ar_flag, nxt_ar_flag;
reg sram_wen, nxt_sram_wen;
reg [7:0] skipped, nxt_skipped;
wire [8:0] skipped_tmp;
reg [27:0] bit_0, nxt_bit_0, bit_1, nxt_bit_1, bit_2, nxt_bit_2, bit_3, nxt_bit_3;
reg [4:0] addr_x_0, nxt_addr_x_0, addr_x_1, nxt_addr_x_1, addr_x_2, nxt_addr_x_2, addr_x_3, nxt_addr_x_3;
reg [4:0] addr_y, nxt_addr_y;
reg [3:0] bit_cnt, nxt_bit_cnt;
reg [1:0] row_cnt, nxt_row_cnt;
reg [3:0] x_cnt, nxt_x_cnt;
reg [8:0] addr_0, nxt_addr_0, addr_1, nxt_addr_1, addr_2, nxt_addr_2, addr_3, nxt_addr_3;
reg bit_flag, nxt_bit_flag;
reg bit_flag_tmp1 , nxt_bit_flag_tmp1;
reg [6:0] bit_addr, nxt_bit_addr;
reg [4:0] addr_3_plus, nxt_addr_3_plus;
reg plus_flag, nxt_plus_flag;
reg bit_is_one_t1, nxt_bit_is_one_t1;
reg bit_is_one_t2, nxt_bit_is_one_t2;
reg [7:0] pixel [0:12];
reg [7:0] nxt_pixel [0:12];
reg [17:0] max_0, nxt_max_0, max_1, nxt_max_1, max_2, nxt_max_2, max_3, nxt_max_3;
reg [7:0] max_quanti, nxt_max_quanti;
reg max_flag;
reg rready_tmp, nxt_rready_tmp;
reg arvalid_tmp ,nxt_arvalid_tmp;
reg [6:0] small_bitmap, nxt_small_bitmap;
reg bitmap_flag;
reg [7:0] addr_bitmap, nxt_addr_bitmap;
reg finish_flag, nxt_finish_flag;

// clk ram
reg [6:0] ram_cnt, nxt_ram_cnt;
reg bready_tmp, nxt_bready_tmp;
reg [8:0] total_unskip_ram, nxt_total_unskip_ram;
reg araddr_over_127, nxt_araddr_over_127;
reg [14:0] araddr_tmp, nxt_araddr_tmp;
reg [8:0] total_unskip, nxt_total_unskip;
reg bitmap_flag_tmp, nxt_bitmap_flag_tmp;


wire [7:0] RAM_data;
wire bit_is_one;
wire finish_tmp;
wire push_req_n;
wire push_full;
wire pop_req_n;
wire pop_empty;
wire [8:0] sram_addr;
wire sram_cen;
wire [7:0] sram_out;
wire [8:0] unskip_addr_0, unskip_addr_1, unskip_addr_2, unskip_addr_3;
wire [15:0] pix_mul_new;
wire [15:0] pix_mul_old_0, pix_mul_old_1, pix_mul_old_2, pix_mul_old_3;
wire [7:0] kernel_new;
wire [7:0] kernel_old_0, kernel_old_1, kernel_old_2, kernel_old_3;
wire [17:0] add_bias;
wire [17:0] max_tmp;
wire [7:0] pixel_0, pixel_2, pixel_6;
reg [7:0] small_bitmap_tmp;
reg [14:0] awaddr_unskip, nxt_awaddr_unskip, awaddr_bitmap, nxt_awaddr_bitmap;

wire push_req_n_2, pop_req_n_2, push_full_2, pop_empty_2;
wire [7:0] AFIFO_datain_2, AFIFO_dataout_2;

always @(*) begin
    finish = finish_tmp;
end



assign push_req_n = (rvalid && rready) ? 0 : 1;
assign pop_req_n = ((state == READ || state == UNSKIP || state == PARAMET) && !pop_empty) ? 0 : 1;

assign bit_is_one =  (row_cnt == 0) ? (bit_0[27]) :
                     (row_cnt == 1) ? (bit_1[27]) :
                     (row_cnt == 2) ? (bit_2[27]) :
                     (row_cnt == 3) ? (bit_3[27]) : 1'b0;
always @(*) begin
    nxt_bit_is_one_t1 = bit_is_one;
    nxt_bit_is_one_t2 = bit_is_one_t1;
end

assign awlen = ((addr_y == 0) && bit_flag && (x_cnt == 0) && (bit_cnt < 8)) ? 8'd3 : 8'd0;
assign awsize = 3'd1;
assign awburst = 2'b01;
//   awvalid  ///
always @(*) begin
    if (awvalid && awready) begin
        awvalid_r = 0;
    end
    else if (max_flag && (max_quanti != skipped)) begin
        awvalid_r = 1;
    end
    else if (bitmap_flag) begin
        awvalid_r = 1;
    end
    else awvalid_r = awvalid_w;
end

always @(*) begin
    if (awvalid && awready) begin
        nxt_bitmap_flag_tmp = 0;
    end
    else if (bitmap_flag) begin
        nxt_bitmap_flag_tmp = 1;
    end
    else nxt_bitmap_flag_tmp = bitmap_flag_tmp;
end

// aw , w channel 
assign awaddr = ((addr_y == 0) && bit_flag && (x_cnt == 0) && (bit_cnt < 8)) ? awaddr_bitmap:
                (bitmap_flag_tmp) ? awaddr_bitmap: awaddr_unskip;
assign awvalid = ((addr_y == 0) && bit_flag && (x_cnt == 0) && (ram_cnt == 16 || ram_cnt == 20 || ram_cnt == 24)) ? 1 :
                awvalid_w;
assign wdata = AFIFO_dataout_2;
assign wvalid = !pop_req_n_2;
assign wlast = ((ram_cnt == 0) && (!pop_req_n_2)) ? 1 :
               (ram_cnt == 19 || ram_cnt == 23 || ram_cnt == 24) ? 1 : 0;
assign wstrb = 1'b1;
// aw , w channel 
assign bready = bready_tmp;
always @(*) begin
    if (wlast) begin
        nxt_bready_tmp = 1;
    end
    else if (bready && bvalid) begin
        nxt_bready_tmp = 0;
    end
    else nxt_bready_tmp = 0;
end


assign arlen = (state == UNSKIP && (total_unskip_ram > 127)) ? (127) :
               (state == UNSKIP) ? (total_unskip_ram - 1) :
               (state == PARAMET) ? 8'd15 : 8'd95;
always @(*) begin
    if (state == READ && rready && rvalid) begin
        nxt_total_unskip_ram = total_unskip_ram + rdata[0] + rdata[1] + rdata[2] + rdata[3] + rdata[4] + rdata[5] + rdata[6] + rdata[7];
    end
    else if (arvalid && arready && (total_unskip_ram > 127)) begin
        nxt_total_unskip_ram = total_unskip_ram - 128;
    end
    else nxt_total_unskip_ram = total_unskip_ram;
end
always @(*) begin
    if ((state == READ) && rready && rvalid && rlast) begin
        nxt_total_unskip = total_unskip_ram;
    end
    else nxt_total_unskip = total_unskip;
end

assign arsize = 3'd1; 
assign araddr = (state == UNSKIP) ? araddr_tmp :
                ((state == PARAMET) && (para_cnt == 0)) ? 1152 :
                ((state == PARAMET) && (para_cnt == 1)) ? 1168 :
                ((state == PARAMET) && (para_cnt == 2)) ? 1184 :
                ((state == PARAMET) && (para_cnt == 3)) ? 1200 :
                ((state == PARAMET) && (para_cnt == 4)) ? 1216 :
                ((state == PARAMET) && (para_cnt == 5)) ? 1232 :
                ((state == PARAMET) && (para_cnt == 6)) ? 1248 :
                ((state == PARAMET) && (para_cnt == 7)) ? 1264 : 16;
always @(*) begin
    if (state == UNSKIP && arvalid && arready && (total_unskip_ram > 127)) begin
        nxt_araddr_tmp = araddr_tmp + 128;
    end
    else nxt_araddr_tmp = araddr_tmp;
end
assign arvalid = (state == PARAMET) ? arvalid_tmp :
                 (state == UNSKIP && ram_cnt > 6) ? arvalid_tmp :
                 (state == COMPUTE) ? 0: ar_flag;
assign arburst = 2'b01;
assign rready = ((state == READ) || (state == UNSKIP)) ? 1 :
                (state == PARAMET) ? rready_tmp : 0;

always @(*) begin
    if (arvalid && arready) begin
        nxt_arvalid_tmp = 0;
    end
    else if (!arready && (state == PARAMET)) begin
        nxt_arvalid_tmp = 0;
    end
    else if ((state == UNSKIP) && (ram_cnt == 126) && (total_unskip_ram <= 127) && araddr_over_127) begin
        nxt_arvalid_tmp = 1;
    end
    else if ((state == UNSKIP) && (ram_cnt > 5) && ((!araddr_over_127) || (total_unskip_ram <= 127))) begin
        nxt_arvalid_tmp = 0;
    end
    else if ((state == PARAMET) && (ram_cnt > 5)) begin
        nxt_arvalid_tmp = 0;
    end
    else if (state == PARAMET || state == UNSKIP) begin
        nxt_arvalid_tmp = 1;
    end
    else nxt_arvalid_tmp = arvalid_tmp;
end

always @(*) begin
    if (arvalid && arready) begin
        nxt_rready_tmp = 1;
    end
    else if ((state == PARAMET) && (ram_cnt == 15)) begin
        nxt_rready_tmp = 0;
    end
    else nxt_rready_tmp = rready_tmp;
end
always @(*) begin
    if ((state == UNSKIP) && arvalid && arready && (total_unskip_ram > 127)) begin
        nxt_araddr_over_127 = 1;
    end
    else if (state == PARAMET) begin
        nxt_araddr_over_127 = 0;
    end
    else nxt_araddr_over_127 = araddr_over_127;
end

assign unskip_addr_0 = 96 + addr_0;
assign unskip_addr_1 = 96 + addr_1;
assign unskip_addr_2 = 96 + addr_2;
assign unskip_addr_3 = 96 + addr_3;

                             
assign sram_addr = ((state == COMPUTE) && (bit_flag)) ? bit_addr :
                   ((state == COMPUTE) && (row_cnt == 0)) ? unskip_addr_0 :
                   ((state == COMPUTE) && (row_cnt == 1)) ? unskip_addr_1 :
                   ((state == COMPUTE) && (row_cnt == 2)) ? unskip_addr_2 :
                   ((state == COMPUTE) && (row_cnt == 3)) ? unskip_addr_3 : datain_cnt;


always @(*) begin
    if ((addr_y > 2) && (bit_flag) && (bit_cnt == 0)) begin 
        nxt_addr_0 = addr_1;
    end
    else if ((state == COMPUTE) && (row_cnt == 0) && (bit_is_one)) begin
        nxt_addr_0 = addr_0 + 1;
    end
    else if (state == PARAMET) begin
        nxt_addr_0 = 0;
    end
    else nxt_addr_0 = addr_0;
end
always @(*) begin
    if ((addr_y > 2) && (bit_flag) && (bit_cnt == 0)) begin
        nxt_addr_1 = addr_2;
    end
    else if ((state == COMPUTE) && (row_cnt == 1) && (bit_is_one)) begin
        nxt_addr_1 = addr_1 + 1;
    end
    else if (state == PARAMET) begin
        nxt_addr_1 = 0;
    end
    else nxt_addr_1 = addr_1;
end
always @(*) begin
    if ((addr_y == 0) && (bit_flag) && (bit_cnt == 10)) begin
        nxt_addr_2 = addr_3_plus;
    end
    else if ((addr_y > 2) && (bit_flag) && (bit_cnt == 0)) begin
        nxt_addr_2 = addr_3;
    end
    else if ((addr_y == 2) && (bit_flag) && (bit_cnt == 0)) begin
        nxt_addr_2 = addr_2;
    end
    else if ((state == COMPUTE) && (row_cnt == 2) && (bit_is_one)) begin
        nxt_addr_2 = addr_2 + 1;
    end
    else if (state == PARAMET) begin
        nxt_addr_2 = 0;
    end
    else nxt_addr_2 = addr_2;
end

always @(*) begin
    if ((addr_y == 2) && (bit_cnt == 14)) begin
        nxt_addr_3 = addr_2 + addr_3_plus;
    end
    else if ((addr_y != 2) && (bit_cnt == 14)) begin 
        nxt_addr_3 = addr_3 + addr_3_plus;
    end
    else if ((state == COMPUTE) && (row_cnt == 3) && (bit_is_one)) begin
        nxt_addr_3 = addr_3 + 1;
    end
    else if (state == PARAMET) begin
        nxt_addr_3 = 0;
    end
    else nxt_addr_3 = addr_3;
end


always @(*) begin
    if ((addr_y == 0) && (plus_flag) && (bit_cnt == 7 || bit_cnt == 8)) begin 
        nxt_addr_3_plus = addr_3_plus + bit_1[0] + bit_1[1] + bit_1[2] + bit_1[3] + bit_1[4] + bit_1[5] + bit_1[6] + bit_1[7];
    end
    else if ((addr_y == 0) && (plus_flag) && (bit_cnt == 6 || bit_cnt == 9)) begin
        nxt_addr_3_plus = addr_3_plus + bit_1[0] + bit_1[1] + bit_1[2] + bit_1[3] + bit_1[4] + bit_1[5];
    end
    else if ((plus_flag) && (bit_cnt == 11 || bit_cnt == 12)) begin
        nxt_addr_3_plus = addr_3_plus + bit_2[0] + bit_2[1] + bit_2[2] + bit_2[3] + bit_2[4] + bit_2[5] + bit_2[6] + bit_2[7];
    end
    else if ((addr_y == 0) && (x_cnt == 13) && (bit_cnt == 1)) begin 
        nxt_addr_3_plus = 0;
    end
    else if ((addr_y != 0) && (plus_flag) && (bit_cnt == 10 || bit_cnt == 13)) begin  
        nxt_addr_3_plus = addr_3_plus + bit_2[0] + bit_2[1] + bit_2[2] + bit_2[3] + bit_2[4] + bit_2[5];
    end
    else if ((bit_flag) && (bit_cnt == 14)) begin 
        nxt_addr_3_plus = 0;
    end
    else nxt_addr_3_plus = addr_3_plus;
end



always @(*) begin
    if ((state == COMPUTE) && (row_cnt == 0) && (bit_is_one)) begin
        nxt_addr_x_0 = addr_x_0 + 1;
    end
    else if ((x_cnt == 13) && (bit_cnt == 15)) begin
        nxt_addr_x_0 = 0;
    end
    else nxt_addr_x_0 = addr_x_0;
end
always @(*) begin
    if ((state == COMPUTE) && (row_cnt == 1) && (bit_is_one)) begin
        nxt_addr_x_1 = addr_x_1 + 1;
    end
    else if ((x_cnt == 13) && (bit_cnt == 15)) begin
        nxt_addr_x_1 = 0;
    end
    else nxt_addr_x_1 = addr_x_1;
end
always @(*) begin
    if ((state == COMPUTE) && (row_cnt == 2) && (bit_is_one)) begin
        nxt_addr_x_2 = addr_x_2 + 1;
    end
    else if ((x_cnt == 13) && (bit_cnt == 15)) begin
        nxt_addr_x_2 = 0;
    end
    else nxt_addr_x_2 = addr_x_2;
end
always @(*) begin
    if ((state == COMPUTE) && (row_cnt == 3) && (bit_is_one)) begin
        nxt_addr_x_3 = addr_x_3 + 1;
    end
    else if ((x_cnt == 13) && (bit_cnt == 15)) begin
        nxt_addr_x_3 = 0;
    end
    else nxt_addr_x_3 = addr_x_3;
end
always @(*) begin
    if ((state == COMPUTE) && (addr_y == 0) && (bit_cnt == 3) && (x_cnt == 14)) begin
        nxt_addr_y = addr_y + 2; 
    end
    else if ((state == COMPUTE) && (addr_y != 0) && (bit_cnt == 3) && (x_cnt == 13)) begin
        nxt_addr_y = addr_y + 2;
    end
    else if (state == PARAMET) begin
        nxt_addr_y = 0;
    end
    else nxt_addr_y = addr_y;
end


always @(*) begin
    if ((state == COMPUTE) && (datain_cnt == 1)) begin 
        nxt_bit_addr = bit_addr + 1;
    end
    else if ((x_cnt == 13) && (bit_cnt == 2) && (!bit_flag)) begin
        nxt_bit_addr = bit_addr - 8;
    end
    else if (state == PARAMET) begin
        nxt_bit_addr = 0;
    end
    else nxt_bit_addr = bit_addr;
end



assign pixel_0 = (addr_y == 0 && x_cnt == 14 && bit_cnt == 1) ? 8'b0 : pixel[0];
assign pixel_2 = ((addr_y != 0 || addr_y != 22) && (bit_cnt == 1) && (x_cnt == 0) && (bit_flag)) ? 8'b0 : pixel[2];
assign pixel_6 = ((addr_y == 22) && (bit_cnt == 2) && (x_cnt != 0 || x_cnt != 14)) ? 8'b0 : pixel[6];
always @(*) begin
    nxt_mul_0 = $signed({1'b0,pixel_0}) * $signed(kernel_0);
    nxt_mul_1 = $signed({1'b0,pixel[1]}) * $signed(kernel_1);
    nxt_mul_2 = $signed({1'b0,pixel_2}) * $signed(kernel_2);
    nxt_mul_3 = $signed({1'b0,pixel[3]}) * $signed(kernel_3);
    nxt_mul_4 = $signed({1'b0,pixel[4]}) * $signed(kernel_4);
    nxt_mul_5 = $signed({1'b0,pixel[5]}) * $signed(kernel_5);
    nxt_mul_6 = $signed({1'b0,pixel_6}) * $signed(kernel_6);
    nxt_mul_7 = $signed({1'b0,pixel[7]}) * $signed(kernel_7);
    nxt_mul_8 = $signed({1'b0,pixel[8]}) * $signed(kernel_8);
end

assign add_bias = $signed(mul_0) + $signed(mul_1) + $signed(mul_2) + $signed(mul_3) +
                    $signed(mul_4) + $signed(mul_5) + $signed(mul_6) + $signed(mul_7) +
                    $signed(mul_8) + $signed(bias);

always @(*) begin 
    for (j = 0;j<13 ;j=j+1) begin
        nxt_pixel[j] = pixel[j];
    end
    case (addr_y)  
        0:begin 
            case (x_cnt)
                0:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = 8'b0;
                                nxt_pixel[10] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[5] = sram_out;
                                nxt_pixel[8] = 8'b0; 
                            end
                            else if (bit_cnt == 6 || bit_cnt == 8) begin
                                nxt_pixel[8] = sram_out;
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = sram_out;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[5];
                                nxt_pixel[10] = pixel[8];
                            end
                        end 
                        0:begin
                            if (bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = 8'b0;
                                nxt_pixel[10] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0; 
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[5]; 
                                nxt_pixel[10] = pixel[8];
                            end
                        end
                    endcase                    
                end 
                14:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 0) begin 
                                nxt_pixel[0] = sram_out;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 1) begin 
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = pixel[0];
                            end
                            else if (bit_cnt == 2) begin 
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                        end 
                        default: begin
                            if (bit_cnt == 0) begin 
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 1) begin 
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = pixel[0];
                            end
                            else if (bit_cnt == 2) begin 
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                        end
                    endcase
                end
                default:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[9];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[10];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = 8'b0;
                                nxt_pixel[10] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 1) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = sram_out;
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                            end
                            else if (bit_cnt == 2 || bit_cnt == 4) begin
                                nxt_pixel[8] = sram_out;
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = sram_out;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = pixel[5];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                        end 
                        0:begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[9];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[10];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = 8'b0;
                                nxt_pixel[10] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 1) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = 8'b0;
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[5];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                        end
                    endcase
                end
                 
            endcase
        end 
        22:begin
            case (x_cnt)
                0:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4 || bit_cnt == 5 || bit_cnt == 6) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = 8'b0; //歸零
                                nxt_pixel[10] = 8'b0; //歸零
                                nxt_pixel[11] = 8'b0; //歸零
                                nxt_pixel[12] = 8'b0; //歸零
                            end
                            else if (bit_cnt == 0 && bit_flag) begin // addr_y = 20 角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[9];
                                nxt_pixel[10] = pixel[10];
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 8) begin
                                nxt_pixel[5] = sram_out;
                            end
                        end 
                        default:begin
                            if ((bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4 || bit_cnt == 5 || bit_cnt == 6) && (!bit_flag)) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = 8'b0; //歸零
                                nxt_pixel[10] = 8'b0; //歸零
                                nxt_pixel[11] = 8'b0; //歸零
                                nxt_pixel[12] = 8'b0; //歸零
                            end
                            else if (bit_cnt == 0 && bit_flag) begin // addr_y = 20 角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[9];
                                nxt_pixel[10] = pixel[10];
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 1 && bit_flag) begin // addr_y = 20 角落
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = pixel[2];
                            end
                            else if (bit_cnt == 2 && bit_flag) begin // addr_y = 20 角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                        end
                    endcase
                end 
                1:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 1) begin 
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = sram_out;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                            else if (bit_cnt == 2) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[6];
                                nxt_pixel[12] = sram_out;
                            end
                            else if (bit_cnt == 3) begin // 
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[5] = sram_out;
                            end
                        end 
                        default: begin
                            if (bit_cnt == 1) begin 
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                            else if (bit_cnt == 2) begin 
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[6];
                                nxt_pixel[12] = 8'b0;;
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                        end
                    endcase
                end
                14:begin
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = 8'b0;
                                nxt_pixel[10] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                        end 
                        default: begin
                            if (bit_cnt == 1) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 2) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[10];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                            end
                        end
                    endcase
                end
                default:begin
                    case (bit_is_one_t1)
                        1: begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[8] = sram_out;
                            end
                            else if (bit_cnt == 1) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = sram_out;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                            else if (bit_cnt == 2) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[6];
                                nxt_pixel[12] = sram_out;
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 5) begin
                                nxt_pixel[5] = sram_out;
                            end
                        end
                        default: begin
                            if (bit_cnt == 1) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                            else if (bit_cnt == 2) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[6];
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 3) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                        end
                    endcase
                end 
            endcase
        end
        default:begin //中間層
            case (x_cnt)
                0:begin
                    case (bit_is_one_t1)
                        1:begin
                            if ((bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4 || bit_cnt == 5 || bit_cnt == 6)/* && (!bit_flag)*/) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = 8'b0; //歸零
                                nxt_pixel[10] = 8'b0; //歸零
                                nxt_pixel[11] = 8'b0; //歸零
                                nxt_pixel[12] = 8'b0; //歸零
                            end
                            else if (bit_cnt == 0 && bit_flag) begin //角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[9];
                                nxt_pixel[10] = pixel[10];
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                            end
                            else if (bit_cnt == 8 || bit_cnt == 9) begin
                                nxt_pixel[5] = pixel[8];
                                nxt_pixel[8] = sram_out;
                            end
                            else if (bit_cnt == 10) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = sram_out;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                            else if (bit_cnt == 11) begin
                                nxt_pixel[8] = sram_out;
                            end
                        end
                        default: begin 
                            if ((bit_cnt == 1 || bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 4 || bit_cnt == 5 || bit_cnt == 6) && (!bit_flag)) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = 8'b0; //歸零
                                nxt_pixel[10] = 8'b0; //歸零
                                nxt_pixel[11] = 8'b0; //歸零
                                nxt_pixel[12] = 8'b0; //歸零
                            end
                            else if (bit_cnt == 0 && bit_flag) begin //角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[9];
                                nxt_pixel[10] = pixel[10];
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 1 && bit_flag) begin //角落
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = pixel[2];
                            end
                            else if (bit_cnt == 2 && bit_flag) begin //角落
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                            end
                            else if (bit_cnt == 8 || bit_cnt == 9) begin
                                nxt_pixel[5] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 10) begin
                                nxt_pixel[0] = 8'b0;
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = 8'b0;
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = 8'b0;
                                nxt_pixel[7] = 8'b0;
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                        end
                    endcase
                end
                default: begin // 中間區域
                    case (bit_is_one_t1)
                        1:begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 1) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 5 || bit_cnt == 6) begin
                                nxt_pixel[5] = pixel[8];
                                nxt_pixel[8] = sram_out;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = sram_out;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = sram_out;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                        end 
                        default:begin
                            if (bit_cnt == 0) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = pixel[11];
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = pixel[12];
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 1) begin
                                nxt_pixel[0] = pixel[9];
                                nxt_pixel[1] = pixel[10];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[1];
                                nxt_pixel[4] = pixel[2];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[4];
                                nxt_pixel[7] = pixel[5];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[7];
                                nxt_pixel[10] = pixel[8];
                                nxt_pixel[11] = 8'b0;
                                nxt_pixel[12] = 8'b0;
                            end
                            else if (bit_cnt == 4) begin
                                nxt_pixel[0] = pixel[1];
                                nxt_pixel[1] = pixel[2];
                                nxt_pixel[2] = 8'b0;
                                nxt_pixel[3] = pixel[4];
                                nxt_pixel[4] = pixel[5];
                                nxt_pixel[5] = 8'b0;
                                nxt_pixel[6] = pixel[7];
                                nxt_pixel[7] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[11] = pixel[3];
                                nxt_pixel[12] = pixel[6];
                            end
                            else if (bit_cnt == 2 || bit_cnt == 3 || bit_cnt == 5 || bit_cnt == 6) begin
                                nxt_pixel[5] = pixel[8];
                                nxt_pixel[8] = 8'b0;
                            end
                            else if (bit_cnt == 7) begin
                                nxt_pixel[0] = pixel[11];
                                nxt_pixel[1] = pixel[3];
                                nxt_pixel[2] = pixel[4];
                                nxt_pixel[3] = pixel[12];
                                nxt_pixel[4] = pixel[6];
                                nxt_pixel[5] = pixel[7];
                                nxt_pixel[6] = pixel[9];
                                nxt_pixel[7] = pixel[10];
                                nxt_pixel[8] = 8'b0;
                                nxt_pixel[9] = pixel[1];
                                nxt_pixel[10] = pixel[2];
                                nxt_pixel[11] = pixel[5];
                                nxt_pixel[12] = pixel[8];
                            end
                        end
                    endcase
                end
            endcase
        end
    endcase
end

always @(*) begin
    nxt_max_0 = max_0;
    case (addr_y)
        0:begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 6) begin
                        nxt_max_0 = add_bias;
                    end
                end 
                14:begin
                    if (bit_cnt == 1) begin
                        nxt_max_0 = add_bias;
                    end
                end
                default:begin
                    if (bit_cnt == 4) begin
                        nxt_max_0 = add_bias;
                    end
                end 
            endcase
        end 
        22:begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 8) begin
                        nxt_max_0 = add_bias;
                    end
                end 
                14:begin
                    if (bit_cnt == 2) begin
                        nxt_max_0 = add_bias;
                    end
                end
                default: begin
                    if (bit_cnt == 5) begin
                        nxt_max_0 = add_bias;
                    end
                end
            endcase
        end
        default: begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 8 && (!bit_flag)) begin
                        nxt_max_0 = add_bias;
                    end
                    else if ((bit_cnt == 1) && (bit_flag)) begin
                        nxt_max_0 = add_bias;
                    end
                end 
                default: begin
                    if (bit_cnt == 5) begin
                        nxt_max_0 = add_bias;
                    end
                end
            endcase
        end
    endcase
end
//max1
always @(*) begin
    nxt_max_1 = max_1;
    case (addr_y)
        0:begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 8) begin
                        nxt_max_1 = add_bias;
                    end
                end 
                1:begin
                    if (bit_cnt == 0) begin
                        nxt_max_1 = max_1;
                    end
                end
                14:begin
                    if (bit_cnt == 2) begin
                        nxt_max_1 = add_bias;
                    end
                end
                default:begin
                    if (bit_cnt == 0) begin
                        nxt_max_1 = add_bias;
                    end
                end 
            endcase
        end 
        22:begin
            case (x_cnt)
                1:begin
                    if (bit_cnt == 2) begin
                        nxt_max_1 = add_bias;
                    end
                end 
                14:begin
                    if (bit_cnt == 3) begin
                        nxt_max_1 = add_bias;
                    end
                end
                default: begin
                    if (bit_cnt == 2) begin
                        nxt_max_1 = add_bias;
                    end
                end
            endcase
        end
        default: begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 11 && (!bit_flag)) begin
                        nxt_max_1 = add_bias;
                    end
                    else if ((bit_cnt == 2) && (bit_flag)) begin
                        nxt_max_1 = add_bias;
                    end
                end 
                1:begin
                    if (bit_cnt == 0) begin
                        nxt_max_1 = max_1;
                    end
                end
                default: begin
                    if (bit_cnt == 0) begin
                        nxt_max_1 = add_bias;
                    end
                end
            endcase
        end
    endcase
end
//max2
always @(*) begin
    nxt_max_2 = max_2;
    case (addr_y)
        0:begin
            case (x_cnt)
                // 1:begin
                //     if (bit_cnt == 1) begin //可以跟下面default合併
                //         nxt_max_2 = add_bias;
                //     end
                // end 
                14:begin
                    if (bit_cnt == 3) begin
                        nxt_max_2 = add_bias;
                    end
                end
                default:begin
                    if (bit_cnt == 1) begin
                        nxt_max_2 = add_bias;
                    end
                end 
            endcase
        end 
        22:begin
            case (x_cnt)
                // 1:begin
                //     if (bit_cnt == 3) begin
                //         nxt_max_2 = add_bias;
                //     end
                // end 
                14:begin
                    if (bit_cnt == 4) begin
                        nxt_max_2 = add_bias;
                    end
                end
                default: begin
                    if (bit_cnt == 3) begin
                        nxt_max_2 = add_bias;
                    end
                end
            endcase
        end
        default: begin
            case (x_cnt)
                0:begin
                    if (bit_cnt == 3 && bit_flag) begin
                        nxt_max_2 = add_bias;
                    end
                end
                1:begin
                    if (bit_cnt == 1 && (!bit_flag)) begin
                        nxt_max_2 = add_bias;
                    end
                    else if ((bit_cnt == 3) && (bit_flag)) begin
                        nxt_max_2 = add_bias;
                    end
                end 
                default: begin
                    if (bit_cnt == 1) begin
                        nxt_max_2 = add_bias;
                    end
                end
            endcase
        end
    endcase
end
//max3
always @(*) begin
    nxt_max_3 = max_3;
    case (addr_y)
        0:begin
            case (x_cnt)
                // 1:begin
                //     if (bit_cnt == 2) begin //可以跟下面default合併
                //         nxt_max_3 = add_bias;
                //     end
                // end 
                14:begin
                    if (bit_cnt == 2) begin
                        nxt_max_3 = max_3;
                    end
                end
                default:begin
                    if (bit_cnt == 2) begin
                        nxt_max_3 = add_bias;
                    end
                end 
            endcase
        end 
        2:begin
            // if (bit_flag && (bit_cnt == 0) && (x_cnt == 0)) begin // 最上層下來的條件
            //     nxt_max_3 = add_bias;
            // end
            case (x_cnt)
                0:begin
                    if ((bit_cnt == 0) && (bit_flag)) begin
                        nxt_max_3 = add_bias;
                    end
                end
                1:begin
                    if (bit_cnt == 2 && (!bit_flag)) begin
                        nxt_max_3 = add_bias;
                    end
                    else if ((bit_cnt == 4) && (bit_flag)) begin
                        nxt_max_3 = add_bias;
                    end
                end 
                default: begin
                    if (bit_cnt == 2) begin
                        nxt_max_3 = add_bias;
                    end
                end
            endcase
        end
        22:begin
            case (x_cnt)
                14:begin
                    if (bit_cnt == 5) begin
                        nxt_max_3 = add_bias;
                    end
                end
                default: begin
                    if (bit_cnt == 4) begin
                        nxt_max_3 = add_bias;
                    end
                end
            endcase
        end
        default: begin
            case (x_cnt)
                1:begin
                    if (bit_cnt == 2 && (!bit_flag)) begin
                        nxt_max_3 = add_bias;
                    end
                    else if ((bit_cnt == 4) && (bit_flag)) begin
                        nxt_max_3 = add_bias;
                    end
                end 
                default: begin
                    if (bit_cnt == 2) begin
                        nxt_max_3 = add_bias;
                    end
                end
            endcase
        end
    endcase
end
///    max flag            ///
always @(*) begin
max_flag = 0;
    case (addr_y)
        0: begin
            case (x_cnt)
                0: begin
                    if (bit_cnt == 4) begin
                        max_flag = 0;
                    end
                end
                14:begin
                    if (bit_cnt == 1) begin
                        max_flag = 1;
                    end
                end
                default: begin
                    if (bit_cnt == 4) begin
                        max_flag = 1;
                    end
                end
            endcase
        end
        2:begin
            case (x_cnt)
                0: begin
                    if ((bit_flag) && (bit_cnt == 2)) begin
                        max_flag = 1;
                    end
                    else if ((bit_cnt == 4)) begin
                        max_flag = 0;
                    end
                end
                default: begin
                    if (bit_cnt == 4) begin
                       max_flag = 1; 
                    end
                end
            endcase
        end
        22:begin
            case (state)
                COMPUTE: begin
                    case (x_cnt)
                        0: begin
                            if ((bit_cnt == 0) && (!bit_flag)) begin
                                max_flag = 0;
                            end
                            else if ((bit_cnt == 0 || bit_cnt == 10) && (bit_flag)) begin
                                max_flag = 1;
                            end
                        end
                        1: begin
                            if (bit_cnt == 0) begin
                                max_flag = 0;
                            end
                        end
                        14: begin
                            if (bit_cnt == 7 || bit_cnt == 3) begin
                                max_flag = 1;
                            end
                        end
                        default: begin
                            if (bit_cnt == 0) begin
                                max_flag = 1;
                            end
                        end
                    endcase
                end 
            endcase
        end
        default: begin //中間層
            case (x_cnt)
                0: begin
                    if ((bit_cnt == 4)) begin
                        max_flag = 0;
                    end
                    else if ((bit_cnt == 0 || bit_cnt == 6) && (bit_flag)) begin
                        max_flag = 1;
                    end
                end
                default: begin
                    if (bit_cnt == 4) begin
                       max_flag = 1; 
                    end
                end
            endcase
        end
    endcase
end
///    max flag            ///

///    max pooling         ///
always @(*) begin
    if ((addr_y == 0) && (x_cnt == 0) && (bit_cnt == 0) && (bit_flag) && (bias[14])) begin
        max_pool = 0;
    end
    else if ((addr_y == 0) && (x_cnt == 0) && (bit_cnt == 0) && (bit_flag) && (!bias[14])) begin
        max_pool = bias;
    end
    else if (max_tmp[17]) begin
        max_pool = 0;
    end
    else max_pool = max_tmp;
end
///    max pooling         ///


///   做 quantization   ///
assign tmp_quanti = max_pool * scale;


assign round = tmp_quanti[39:31];

always @(*) begin
    if ((round[0]) && (addr_y == 0) && (x_cnt == 0) && (bit_cnt == 0) && (bit_flag)) begin
        nxt_skipped = round[8:1] + 1;
    end
    else if ((!round[0]) && (addr_y == 0) && (x_cnt == 0) && (bit_cnt == 0) && (bit_flag)) begin
        nxt_skipped = round[8:1];
    end
    else nxt_skipped = skipped;
end

always @(*) begin
    if (round[0]) begin
        nxt_max_quanti = round[8:1] + 1;
    end
    else if (!round[0]) begin
        nxt_max_quanti = round[8:1];
    end
    else nxt_max_quanti = max_quanti;
end
///   做 quantization   ///
///   計算小的bitmap    ///
always @(*) begin
nxt_small_bitmap = small_bitmap;
    case (max_flag)
        1:begin
            if (max_quanti != skipped) begin
                nxt_small_bitmap[0] = 1'b1;
                nxt_small_bitmap[6:1] = small_bitmap[5:0];  
            end
            else if (max_quanti == skipped) begin
                nxt_small_bitmap[0] = 1'b0;
                nxt_small_bitmap[6:1] = small_bitmap[5:0];  
            end
        end 
    endcase
end


always @(*) begin
    if (bitmap_flag) begin
        nxt_addr_bitmap = addr_bitmap + 1;
    end
    else if (state == PARAMET) begin
        nxt_addr_bitmap = 232;
    end
    else nxt_addr_bitmap = addr_bitmap;
end


// counter
always @(*) begin
    if ((state == PARAMET) && (arvalid) && (arready)) begin
        nxt_para_cnt = para_cnt + 1;
    end
    else nxt_para_cnt = para_cnt;
end


always @(*) begin
    if (state == IDLE) begin
        nxt_datain_cnt = 0;
    end
    else if ((state == UNSKIP) && (datain_cnt == 95 + total_unskip)) begin
        nxt_datain_cnt = 0;
    end
    else if ((state == PARAMET) && (datain_cnt == 15)) begin 
        nxt_datain_cnt = 1;        
    end
    else if ((addr_y == 0) && (x_cnt == 14) && (bit_cnt == 3)) begin
        nxt_datain_cnt = 1;
    end
    else if ((state == FINISH)) begin
        nxt_datain_cnt = datain_cnt + 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 13) && (bit_cnt == 3)) begin 
        nxt_datain_cnt = 1;
    end
    else if ((state == UNSKIP || state == PARAMET || state == READ) && (!pop_req_n) && (!pop_empty)) begin
        nxt_datain_cnt = datain_cnt + 1;
    end
    else if ((addr_y == 0) && (state == COMPUTE) && (bit_flag) && (bit_cnt == 11)) begin 
        nxt_datain_cnt = 0;
    end
    else if ((addr_y != 0) && (state == COMPUTE) && (bit_cnt == 15)) begin
        nxt_datain_cnt = 0;
    end
    else if (state == PARAMET) begin
        nxt_datain_cnt = 0;
    end

    else if (addr_y == 22 && x_cnt == 14 && bit_cnt == 8 && (state == COMPUTE) && (para_cnt != 8)) begin
        nxt_datain_cnt = 232;
    end
    else nxt_datain_cnt = datain_cnt;
end
always @(*) begin 
    if ((addr_y == 0) && (state == COMPUTE) && (bit_cnt == 12) && (!bit_flag)) begin 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y == 0 || addr_y == 22) && (x_cnt == 0) && (bit_cnt == 8) && (!bit_flag)) begin 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y == 0 || addr_y == 22) && (x_cnt == 13) && (bit_cnt == 2) && (!bit_flag)) begin // 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y == 0) && (x_cnt == 14) && (bit_cnt == 3) && (!bit_flag)) begin // 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y == 22) && (x_cnt == 14) && (bit_cnt == 9) && (!bit_flag)) begin 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y == 0 || addr_y == 22) && (x_cnt != 0 && x_cnt != 13) && (bit_cnt == 5) && (!bit_flag) && x_cnt != 14) begin
        nxt_bit_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 0) && (bit_cnt == 11) && (!bit_flag)) begin 
        nxt_bit_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 13) && (bit_cnt == 3) && (!bit_flag)) begin
        nxt_bit_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt != 0 && x_cnt != 13) && (bit_cnt == 7) && (!bit_flag)) begin
        nxt_bit_cnt = 0;
    end
    else if (state == COMPUTE) begin
        nxt_bit_cnt = bit_cnt + 1;
    end
    else if (state == PARAMET && (datain_cnt == 0)) begin
        nxt_bit_cnt = 0;
    end
    else nxt_bit_cnt = bit_cnt;
end

always @(*) begin
    if ((addr_y == 0) && (bit_cnt == 1 || bit_cnt == 4 || bit_cnt == 5) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 0) && (bit_cnt == 0 || bit_cnt == 2 || bit_cnt == 3) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 0) && (bit_cnt == 0 || bit_cnt == 1) && (x_cnt == 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 0) && (bit_cnt == 3) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 0) && (bit_cnt == 1) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 0) && (bit_cnt == 8) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 0) && (bit_cnt == 5) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((bit_flag) && (bit_cnt == 0)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (bit_cnt == 1 || bit_cnt == 3 || bit_cnt == 6 || bit_cnt == 7 || bit_cnt == 8 || bit_cnt == 11) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (bit_cnt == 0 || bit_cnt == 1 || bit_cnt == 3 || bit_cnt == 4 || bit_cnt == 5 || bit_cnt == 7) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (bit_cnt >= 0 && bit_cnt <= 3) && (x_cnt == 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y != 0) && (bit_cnt == 5) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (bit_cnt == 2) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 22) && (bit_cnt == 1 || bit_cnt == 3 || bit_cnt == 6 || bit_cnt == 7) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 22) && (bit_cnt == 0 || bit_cnt == 1 || bit_cnt == 3 || bit_cnt == 4) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 22) && (bit_cnt == 0 || bit_cnt == 1) && (x_cnt == 13) && (!bit_flag)) begin
        nxt_row_cnt = row_cnt + 1;
    end
    else if ((addr_y == 22) && (bit_cnt == 8) && (x_cnt == 0) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 22) && (bit_cnt == 2 || bit_cnt == 5) && (x_cnt != 0 && x_cnt != 13) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if ((addr_y == 22) && (bit_cnt == 2) && (x_cnt == 13) && (!bit_flag)) begin
        nxt_row_cnt = 0;
    end
    else if (state == PARAMET) begin
        nxt_row_cnt = 0;
    end
    else nxt_row_cnt = row_cnt;
end

always @(*) begin
    if ((addr_y == 0 || addr_y == 22) && (x_cnt == 0) && (bit_cnt == 8) && (!bit_flag)) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if (state == FINISH) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if ((addr_y == 22) && (x_cnt == 14)) begin
        nxt_x_cnt = 14;
    end
    else if ((addr_y == 0 || addr_y == 22) && (x_cnt != 0 && x_cnt != 13) && (bit_cnt == 5) && (!bit_flag)) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if ((addr_y == 0 || addr_y == 22) && (x_cnt == 13) && (bit_cnt == 2) && (!bit_flag)) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 0) && (bit_cnt == 11) && (!bit_flag)) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt != 0 && x_cnt != 13) && (bit_cnt == 7) && (!bit_flag)) begin
        nxt_x_cnt = x_cnt + 1;
    end
    else if ((addr_y == 0) && (x_cnt == 14) && (bit_cnt == 3) && (!bit_flag)) begin
        nxt_x_cnt = 0;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 13) && (bit_cnt == 3) && (!bit_flag)) begin
        nxt_x_cnt = 0;
    end
    else if (state == PARAMET) begin
        nxt_x_cnt = 0;
    end

    else nxt_x_cnt = x_cnt;
end





// ram cnt  ****************
always @(*) begin
    if (rvalid && rready) begin
        nxt_ram_cnt = ram_cnt + 1;
    end
    else if ((state == PARAMET) && (ram_cnt == 15)) begin 
        nxt_ram_cnt = 0;
    end
    else if (ram_cnt == 25) begin
        nxt_ram_cnt = 0;
    end
    else if ((state == COMPUTE) && (wready) && (wvalid) && (bit_flag) && (addr_y == 0)) begin
        nxt_ram_cnt = ram_cnt + 1;
    end
    else if (state == UNSKIP) begin
        nxt_ram_cnt = 0;
    end
    else nxt_ram_cnt = ram_cnt;
end
// ram cnt



// ram last

// kernel
always @(*) begin
    if (state == PARAMET && (datain_cnt == 0)) begin
        nxt_kernel_0 = RAM_data;
    end
    else nxt_kernel_0 = kernel_0;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 1)) begin
        nxt_kernel_1 = RAM_data;
    end
    else nxt_kernel_1 = kernel_1;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 2)) begin
        nxt_kernel_2 = RAM_data;
    end
    else nxt_kernel_2 = kernel_2;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 3)) begin
        nxt_kernel_3 = RAM_data;
    end
    else nxt_kernel_3 = kernel_3;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 4)) begin
        nxt_kernel_4 = RAM_data;
    end
    else nxt_kernel_4 = kernel_4;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 5)) begin
        nxt_kernel_5 = RAM_data;
    end
    else nxt_kernel_5 = kernel_5;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 6)) begin
        nxt_kernel_6 = RAM_data;
    end
    else nxt_kernel_6 = kernel_6;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 7)) begin
        nxt_kernel_7 = RAM_data;
    end
    else nxt_kernel_7 = kernel_7;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 8)) begin
        nxt_kernel_8 = RAM_data;
    end
    else nxt_kernel_8 = kernel_8;
end
always @(*) begin
    if (state == PARAMET && (datain_cnt == 10)) begin
        nxt_bias[14:8] = RAM_data[6:0];
        nxt_bias[7:0] = bias[7:0];
    end
    else if (state == PARAMET && (datain_cnt == 11)) begin
        nxt_bias[7:0] = RAM_data;
        nxt_bias[14:8] = bias[14:8];
    end
    else nxt_bias = bias;
end
always @(*) begin
    nxt_scale = scale;
    if (state == PARAMET && (datain_cnt == 12)) begin
        nxt_scale[24] = RAM_data[0];
    end
    else if (state == PARAMET && (datain_cnt == 13)) begin
        nxt_scale[23:16] = RAM_data;
    end
    else if (state == PARAMET && (datain_cnt == 14)) begin
        nxt_scale[15:8] = RAM_data;
    end
    else if (state == PARAMET && (datain_cnt == 15)) begin
        nxt_scale[7:0] = RAM_data;
    end   
    else nxt_scale = scale;
end
// 取 SRAM bitmap 資料
// 取橫排資料
always @(*) begin
    if ((state == PARAMET) && (datain_cnt == 15)) begin
        nxt_bit_flag = 1;
    end
    else if ((addr_y == 0) && (x_cnt == 14) && (bit_cnt == 3)) begin 
        nxt_bit_flag = 1;
    end
    else if ((addr_y != 0 && addr_y != 22) && (x_cnt == 13) && (bit_cnt == 3)) begin
        nxt_bit_flag = 1;
    end
    else if ((addr_y == 0) && (state == COMPUTE) && (datain_cnt == 1) && (bit_cnt == 11)) begin 
        nxt_bit_flag = 0;
    end
    else if ((addr_y != 0) && (state == COMPUTE) && (datain_cnt == 1) && (bit_cnt == 15)) begin 
        nxt_bit_flag = 0;
    end
    else nxt_bit_flag = bit_flag;
end
always @(*) begin
    nxt_bit_flag_tmp1 = bit_flag;
end
always @(*) begin
    nxt_plus_flag = bit_flag_tmp1;
end


always @(*) begin
    nxt_bit_0 = bit_0;
    if ((bit_flag) && (bit_cnt == 1)) begin
        nxt_bit_0[5:0] = sram_out[5:0];
    end
    else if ((bit_flag) && (bit_cnt == 2 || bit_cnt == 3)) begin
        nxt_bit_0[7:0] = sram_out;
        nxt_bit_0[21:8] = bit_0[13:0];
    end
    else if ((bit_flag) && (bit_cnt == 4)) begin
        nxt_bit_0[5:0] = sram_out[7:2];
        nxt_bit_0[27:6] = bit_0[21:0];
    end
    else if ((row_cnt == 0) && (!bit_flag)) begin
        nxt_bit_0[27:1] = bit_0[26:0];
        nxt_bit_0[0] = 1'b0; 
    end
    else nxt_bit_0 = bit_0;
end
always @(*) begin  
    nxt_bit_1 = bit_1;
    if ((bit_flag) && (bit_cnt == 5)) begin
        nxt_bit_1[5:0] = sram_out[5:0];
    end
    else if ((bit_flag) && (bit_cnt == 6 || bit_cnt == 7)) begin
        nxt_bit_1[7:0] = sram_out;
        nxt_bit_1[21:8] = bit_1[13:0];
    end
    else if ((bit_flag) && (bit_cnt == 8)) begin
        nxt_bit_1[5:0] = sram_out[7:2];
        nxt_bit_1[27:6] = bit_1[21:0];
    end
    else if ((row_cnt == 1) && (!bit_flag)) begin
        nxt_bit_1[27:1] = bit_1[26:0];
        nxt_bit_1[0] = 1'b0; 
    end
    else nxt_bit_1 = bit_1;
end
always @(*) begin 
    nxt_bit_2 = bit_2;
    if ((bit_flag) && (bit_cnt == 9)) begin
        nxt_bit_2[5:0] = sram_out[5:0];
    end
    else if ((bit_flag) && (bit_cnt == 10 || bit_cnt == 11)) begin
        nxt_bit_2[7:0] = sram_out;
        nxt_bit_2[21:8] = bit_2[13:0];
    end
    else if ((bit_cnt == 12)) begin
        nxt_bit_2[5:0] = sram_out[7:2];
        nxt_bit_2[27:6] = bit_2[21:0];
    end
    else if ((row_cnt == 2)  && (!bit_flag)) begin
        nxt_bit_2[27:1] = bit_2[26:0];
        nxt_bit_2[0] = 1'b0; 
    end
    else nxt_bit_2 = bit_2;
end


always @(*) begin
    nxt_bit_3 = bit_3;
    if (bit_cnt == 13) begin
        nxt_bit_3[5:0] = sram_out[5:0];
    end
    else if (bit_cnt == 14 || bit_cnt == 15) begin
        nxt_bit_3[7:0] = sram_out;
        nxt_bit_3[21:8] = bit_3[13:0];
    end
    else if ((bit_flag_tmp1) && (bit_cnt == 0)) begin 
        nxt_bit_3[5:0] = sram_out[7:2];
        nxt_bit_3[27:6] = bit_3[21:0];
    end
    else if ((row_cnt == 3) && (!bit_flag)) begin
        nxt_bit_3[27:1] = bit_3[26:0];
        nxt_bit_3[0] = 1'b0; 
    end
    else nxt_bit_3 = bit_3;
end


// 取 bitmap 順序


always @(*) begin
    case (state)
        IDLE:begin 
            nxt_ar_flag = 1;
        end 
        READ:begin 
            if (ar_flag) begin
                nxt_ar_flag = 0;
            end
            else if (datain_cnt == 95) begin
                nxt_ar_flag = 1;
            end
            else nxt_ar_flag = ar_flag;
        end
        UNSKIP:begin 
            if (rvalid && rready) begin
                nxt_ar_flag = 0;
            end
            else if (datain_cnt == (95 + total_unskip)) begin
                nxt_ar_flag = 1;
            end
            else nxt_ar_flag = ar_flag;
        end
        PARAMET:begin 
            if (rvalid && rready) begin
                nxt_ar_flag = 0;
            end
            else nxt_ar_flag = ar_flag;
        end
        default: nxt_ar_flag = ar_flag;
    endcase
end


//  FSM                     
always @(*) begin
    case (state)
        IDLE:begin
            nxt_state = READ;
        end 
        READ:begin
            if (datain_cnt == 95) begin
                nxt_state = UNSKIP;
            end
            else nxt_state = state;
        end
        UNSKIP:begin
            if (datain_cnt == 95 + total_unskip) begin
                nxt_state = PARAMET;
            end
            else nxt_state = state;
        end
        PARAMET:begin
            if (datain_cnt == 15) begin
                nxt_state = COMPUTE;
            end
            else nxt_state = state;
        end
        COMPUTE:begin  
            if ((para_cnt == 8) && addr_y == 22 && x_cnt == 14 && bit_cnt == 8) begin 
                nxt_state = FINISH;
            end
            else if (addr_y == 22 && x_cnt == 14 && bit_cnt == 8) begin
                nxt_state = PARAMET;
            end
            else nxt_state = state;
        end
        default: nxt_state = state;
    endcase
end



always @(*) begin
    if (datain_cnt == 95 + total_unskip) begin
        nxt_sram_wen = 1;
    end
    else nxt_sram_wen = sram_wen;
end

always @(*) begin
    bitmap_flag = 0;
    case (addr_y)
        0: begin
            if (x_cnt == 8 && bit_cnt == 0) begin 
                bitmap_flag = 1;
            end
        end
        2:begin
            if (x_cnt == 0 && bit_flag && bit_cnt == 3) begin
                bitmap_flag = 1;
            end
            else if (x_cnt == 8 && bit_cnt == 0) begin
                bitmap_flag = 1;
            end
        end
        22:begin
            if (x_cnt == 0 && bit_flag && bit_cnt == 11 && (state == COMPUTE)) begin
                bitmap_flag = 1;
            end
            else if (x_cnt == 8 && bit_cnt == 2 && (state == COMPUTE)) begin 
                bitmap_flag = 1;
            end
            else if (x_cnt == 14 && bit_cnt == 8 && (state == COMPUTE)) begin
                bitmap_flag = 1;
            end
        end
        default: begin
            if (x_cnt == 8 && bit_cnt == 0) begin 
                bitmap_flag = 1;
            end
            else if (x_cnt == 0 && bit_flag && bit_cnt == 7) begin
                bitmap_flag = 1;
            end
        end
    endcase
end

always @(*) begin
    small_bitmap_tmp = 0;
    case (addr_y)
        0: begin
            if (x_cnt == 8 && bit_cnt == 0) begin  
                small_bitmap_tmp = {1'b0,small_bitmap};
            end
        end
        2:begin
            if (x_cnt == 0 && bit_flag && bit_cnt == 3) begin
                small_bitmap_tmp = {small_bitmap,1'b0};
            end
            else if (x_cnt == 8 && bit_cnt == 0) begin  
                small_bitmap_tmp = {1'b0,small_bitmap};
            end
        end
        22:begin
            if (x_cnt == 0 && bit_flag && bit_cnt == 11) begin
                small_bitmap_tmp = {small_bitmap,1'b0};
            end
            else if (x_cnt == 8 && bit_cnt == 2) begin 
                small_bitmap_tmp = {1'b0,small_bitmap};
            end
            else if (x_cnt == 14 && bit_cnt == 8) begin
                small_bitmap_tmp = {small_bitmap,1'b0};
            end
        end
        default: begin
            if (x_cnt == 8 && bit_cnt == 0) begin 
                small_bitmap_tmp = {1'b0,small_bitmap};
            end
            else if (x_cnt == 0 && bit_flag && bit_cnt == 7) begin
                small_bitmap_tmp = {small_bitmap,1'b0};
            end
        end
    endcase
end

// Finish
always @(*) begin
    if ((state == FINISH) && (datain_cnt == 0 || datain_cnt == 75 || datain_cnt == 150)) begin
        nxt_finish_flag = 1;
    end
    else nxt_finish_flag = 0;
end

Pluse_syn P0 (.aclk(clk_top), .bclk(clk_tb), .rst(rst_n), .IN(finish_flag), .b_p(finish_tmp));

assign AFIFO_datain_2 = ((addr_y == 0) && bit_flag && (x_cnt == 0) && (bit_cnt < 8)) ? 8'b0: 
                        (bitmap_flag) ? small_bitmap_tmp :                                 
                        ((addr_y == 0) && bit_flag && (x_cnt == 0) && (bit_cnt == 8)) ? skipped : max_quanti;
                    

assign push_req_n_2 = ((addr_y == 0) && bit_flag && (x_cnt == 0) && (bit_cnt <= 8)) ? 1'b0:
                      (bitmap_flag) ? 1'b0:
                      (max_flag && (max_quanti != skipped)) ? 1'b0 : 1'b1;

assign pop_req_n_2 = (!pop_empty_2) ? 1'b0 : 1'b1;


Max_compare M0 (.in0(max_0) ,.in1(max_1) ,.in2(max_2) ,.in3(max_3) ,.max(max_tmp));
TS1N16FFCLLULVTA512X8M4SWBSHO U0(
            .SLP(1'b0),
            .DSLP(1'b0),
            .SD(1'b0),
            .PUDELAY(),
            .CLK(clk_top), .CEB(1'b0), .WEB(sram_wen),
            .BIST(1'b0),
            .CEBM(1'b0), .WEBM(1'b0),
            .A(sram_addr), .D(RAM_data),
            .BWEB({8{1'b0}}),
            .AM({9{1'b0}}), .DM({8{1'b0}}), 
            .BWEBM({8{1'b0}}),
            .RTSEL(2'b01),
            .WTSEL(2'b01),
            .Q(sram_out));


always @(*) begin
nxt_awaddr_unskip = awaddr_unskip;
    if ((addr_y == 22) && (x_cnt == 14) && (bit_cnt == 7)) begin
        case (para_cnt)
            1: nxt_awaddr_unskip = 1600;
            2: nxt_awaddr_unskip = 1888;
            3: nxt_awaddr_unskip = 2176;
            4: nxt_awaddr_unskip = 2464;
            5: nxt_awaddr_unskip = 2752;
            6: nxt_awaddr_unskip = 3040;
            7: nxt_awaddr_unskip = 3328;
        endcase
    end
    else if ((addr_y == 0) && (bit_cnt == 10) && (bit_flag)) begin
        nxt_awaddr_unskip = awaddr_unskip + 1;
    end
    else if (max_flag && (skipped != max_quanti)) begin
        nxt_awaddr_unskip = awaddr_unskip + 1;
    end
    else nxt_awaddr_unskip = awaddr_unskip;
end

always @(*) begin
    if (state == COMPUTE && ram_cnt == 16 && awready && awvalid) begin
        nxt_awaddr_bitmap = awaddr_bitmap + 28;
    end
    else if (state == COMPUTE && ram_cnt == 20 && awready && awvalid) begin
        nxt_awaddr_bitmap = awaddr_bitmap - 24;
    end
    else if (x_cnt == 14 && (bit_cnt > 7) && awready && awvalid) begin
        nxt_awaddr_bitmap = awaddr_bitmap + 261;
    end
    else if (awready && awvalid && (ram_cnt == 0) && bitmap_flag_tmp && awready && awvalid) begin
        nxt_awaddr_bitmap = awaddr_bitmap + 1;
    end
    else nxt_awaddr_bitmap = awaddr_bitmap;
end



DW_fifo_s2_sf #(8, 179, 2, 2, 2, 2, 0, 2, 2, 1)F1(
        .clk_push(clk_ram),   .clk_pop(clk_top),
        .rst_n(rst_n),   .push_req_n(push_req_n),
        .pop_req_n(pop_req_n),   .data_in(rdata),
        .push_empty(),   .push_ae(),
        .push_hf(),   .push_af(),
        .push_full(push_full),   .push_error(),
        .pop_empty(pop_empty),   .pop_ae(),
        .pop_hf(),   .pop_af(),
        .pop_full(),   .pop_error(),
        .data_out(RAM_data) 
);

DW_fifo_s2_sf #(8, 8, 2, 2, 2, 2, 0, 2, 2, 1)F2(
        .clk_push(clk_top),   .clk_pop(clk_ram),
        .rst_n(rst_n),   .push_req_n(push_req_n_2),
        .pop_req_n(pop_req_n_2),   .data_in(AFIFO_datain_2),
        .push_empty(),   .push_ae(),
        .push_hf(),   .push_af(),
        .push_full(push_full_2),   .push_error(),
        .pop_empty(pop_empty_2),   .pop_ae(),
        .pop_hf(),   .pop_af(),
        .pop_full(),   .pop_error(),
        .data_out(AFIFO_dataout_2) 
);



always @(posedge clk_top) begin
    if (!rst_n) begin
        for (j = 0;j<13 ;j=j+1) begin
            pixel[j] <= 0;
        end
        state <= 0;
        datain_cnt <= 0;
        ar_flag <= 0;
        sram_wen <= 0;
        kernel_0 <= 0;
        kernel_1 <= 0;
        kernel_2 <= 0;
        kernel_3 <= 0;
        kernel_4 <= 0;
        kernel_5 <= 0;
        kernel_6 <= 0;
        kernel_7 <= 0;
        kernel_8 <= 0;
        bias <= 0;
        scale <= 0;
        skipped <= 0;
        bit_0 <= 0;
        bit_1 <= 0;
        bit_2 <= 0;
        bit_3 <= 0;
        addr_x_0 <= 0;
        addr_x_1 <= 0;
        addr_x_2 <= 0;
        addr_x_3 <= 0;
        addr_y <= 0;
        bit_cnt <= 0;
        row_cnt <= 0;
        x_cnt <= 0;
        addr_0 <= 0;
        addr_1 <= 0;
        addr_2 <= 0;
        addr_3 <= 0;
        bit_flag <= 0;
        bit_flag_tmp1 <= 0;
        bit_addr <= 0;
        addr_3_plus <= 0;
        plus_flag <= 0;
        bit_is_one_t1 <= 0;
        bit_is_one_t2 <= 0;
        mul_0 <= 0;
        mul_1 <= 0;
        mul_2 <= 0;
        mul_3 <= 0;
        mul_4 <= 0;
        mul_5 <= 0;
        mul_6 <= 0;
        mul_7 <= 0;
        mul_8 <= 0;
        max_0 <= 0;
        max_1 <= 0;
        max_2 <= 0;
        max_3 <= 0;
        max_quanti <= 0;
        small_bitmap <= 0;
        addr_bitmap <= 232;
        awaddr_unskip <= 1312;
        finish_flag <= 0;
    end
    else begin     
        for (j = 0;j<13 ;j=j+1) begin
            pixel[j] <= nxt_pixel[j];
        end
        state <= nxt_state;
        datain_cnt <= nxt_datain_cnt;
        ar_flag <= nxt_ar_flag;
        sram_wen <= nxt_sram_wen;
        kernel_0 <= nxt_kernel_0;
        kernel_1 <= nxt_kernel_1;
        kernel_2 <= nxt_kernel_2;
        kernel_3 <= nxt_kernel_3;
        kernel_4 <= nxt_kernel_4;
        kernel_5 <= nxt_kernel_5;
        kernel_6 <= nxt_kernel_6;
        kernel_7 <= nxt_kernel_7;
        kernel_8 <= nxt_kernel_8;
        bias <= nxt_bias;
        scale <= nxt_scale;
        skipped <= nxt_skipped;
        bit_0 <= nxt_bit_0;
        bit_1 <= nxt_bit_1;
        bit_2 <= nxt_bit_2;
        bit_3 <= nxt_bit_3;
        addr_x_0 <= nxt_addr_x_0;
        addr_x_1 <= nxt_addr_x_1;
        addr_x_2 <= nxt_addr_x_2;
        addr_x_3 <= nxt_addr_x_3;
        addr_y <= nxt_addr_y;
        bit_cnt <= nxt_bit_cnt;
        row_cnt <= nxt_row_cnt;
        x_cnt <= nxt_x_cnt;
        addr_0 <= nxt_addr_0;
        addr_1 <= nxt_addr_1;
        addr_2 <= nxt_addr_2;
        addr_3 <= nxt_addr_3;
        bit_flag <= nxt_bit_flag;
        bit_flag_tmp1 <= nxt_bit_flag_tmp1;
        bit_addr <= nxt_bit_addr;
        addr_3_plus <= nxt_addr_3_plus;
        plus_flag <= nxt_plus_flag;
        bit_is_one_t1 <= nxt_bit_is_one_t1;
        bit_is_one_t2 <= nxt_bit_is_one_t2;
        mul_0 <= nxt_mul_0;
        mul_1 <= nxt_mul_1;
        mul_2 <= nxt_mul_2;
        mul_3 <= nxt_mul_3;
        mul_4 <= nxt_mul_4;
        mul_5 <= nxt_mul_5;
        mul_6 <= nxt_mul_6;
        mul_7 <= nxt_mul_7;
        mul_8 <= nxt_mul_8;
        max_0 <= nxt_max_0;
        max_1 <= nxt_max_1;
        max_2 <= nxt_max_2;
        max_3 <= nxt_max_3;
        max_quanti <= nxt_max_quanti;
        small_bitmap <= nxt_small_bitmap;
        addr_bitmap <= nxt_addr_bitmap;
        awaddr_unskip <= nxt_awaddr_unskip;
        finish_flag <= nxt_finish_flag;
    end
end

always @(posedge clk_ram) begin
    if (!rst_n) begin
        ram_cnt <= 0;
        rready_tmp <= 0;
        arvalid_tmp <= 0;
        para_cnt <= 0;
        bready_tmp <= 0;
        total_unskip_ram <= 0;
        araddr_over_127 <= 0;
        araddr_tmp <= 129;
        total_unskip <= 0;
        awvalid_w <= 0;
        awaddr_bitmap <= 1280;
        bitmap_flag_tmp <= 0;
    end
    else begin
        ram_cnt <= nxt_ram_cnt;
        rready_tmp <= nxt_rready_tmp;
        arvalid_tmp <= nxt_arvalid_tmp;
        para_cnt <= nxt_para_cnt;
        bready_tmp <= nxt_bready_tmp;
        total_unskip_ram <= nxt_total_unskip_ram;
        araddr_over_127 <= nxt_araddr_over_127;
        araddr_tmp <= nxt_araddr_tmp;
        total_unskip <= nxt_total_unskip;
        awvalid_w <= awvalid_r;
        awaddr_bitmap <= nxt_awaddr_bitmap;
        bitmap_flag_tmp <= nxt_bitmap_flag_tmp;
    end
end


endmodule

module Max_compare (in0, in1, in2, in3, max);
    input [17:0] in0, in1, in2, in3;
    output [17:0] max;

wire [17:0] tmp_0, tmp_1;

assign tmp_0 = ($signed(in0) > $signed(in1)) ? in0 : in1;
assign tmp_1 = ($signed(in2) > $signed(in3)) ? in2 : in3;
assign max = ($signed(tmp_0) > $signed(tmp_1)) ? tmp_0 : tmp_1;

endmodule

module Pluse_syn (aclk, bclk, rst, IN, b_p);
    input aclk;
    input bclk;
    input IN;
    input rst;
    output b_p;

reg A, nxt_A;
reg B1, nxt_B1;
reg B2, nxt_B2;
reg B3, nxt_B3;

wire a_p_in;
assign a_p_in = IN ^ A;
always @(*) begin
    nxt_A = a_p_in;
end
always @(*) begin
    nxt_B1 = A;
end
always @(*) begin
    nxt_B2 = B1;
end
always @(*) begin
    nxt_B3 = B2;
end
assign b_p = B2 ^ B3;



always @(posedge aclk) begin
    if (!rst) begin
        A <= 0;
    end
    else begin
        A <= nxt_A;
    end
end
always @(posedge bclk) begin
    if (!rst) begin
        B1 <= 0;
        B2 <= 0;
        B3 <= 0;
    end
    else begin
        B1 <= nxt_B1;
        B2 <= nxt_B2;
        B3 <= nxt_B3;
    end
end

endmodule