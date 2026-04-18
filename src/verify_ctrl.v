// verify_ctrl.v — Verification-only FSM for AS-68M Fingerprint Sensor
// Flow: RESET -> BOOT_WAIT -> VFY_SEND -> VFY_WAIT -> SEC_SEND -> SEC_WAIT
//       -> IDLE -> GETIMG -> GETIMG_WAIT -> IMG2TZ -> IMG2TZ_WAIT
//       -> SEARCH -> SEARCH_WAIT -> RESULT -> IDLE
`default_nettype none
module verify_ctrl #(
    parameter CLK_FREQ = 50_000_000
)(
    input  wire        clk, rst, finger_on,
    output reg         tx_start,
    output reg  [7:0]  tx_data,
    input  wire        tx_busy,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         sensor_ready, busy, matched, no_match, error_flag
);

localparam STARTUP_DLY = CLK_FREQ / 5;
localparam INTER_CMD   = CLK_FREQ / 100;
localparam RX_TIMEOUT  = CLK_FREQ * 8;
localparam RESULT_HOLD = CLK_FREQ * 3;

localparam [3:0]
    ST_RESET=4'd0, ST_BOOT_WAIT=4'd1, ST_VFY_SEND=4'd2,  ST_VFY_WAIT=4'd3,
    ST_SEC_SEND=4'd4, ST_SEC_WAIT=4'd5, ST_IDLE=4'd6, ST_GETIMG=4'd7,
    ST_GETIMG_WAIT=4'd8, ST_IMG2TZ=4'd9, ST_IMG2TZ_WAIT=4'd10,
    ST_SEARCH=4'd11, ST_SEARCH_WAIT=4'd12, ST_RESULT=4'd13, ST_ERROR=4'd14;

reg [3:0]  state;
reg [31:0] timer;
reg [7:0]  pkt[0:16]; reg [4:0] pkt_len, pkt_ptr; reg sending, prev_busy;
reg [3:0]  rx_idx, ack_expect; /* verilator lint_off UNUSEDSIGNAL */ reg [7:0] ack_conf; /* verilator lint_on UNUSEDSIGNAL */
reg        ack_ready, ack_ok, rx_active;
reg [31:0] rx_timer; reg rx_timeout;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state<=ST_RESET; timer<=0;
        sensor_ready<=0; busy<=0; matched<=0; no_match<=0; error_flag<=0;
        tx_start<=0; tx_data<=0; pkt_len<=0; pkt_ptr<=0; sending<=0; prev_busy<=0;
        rx_idx<=0; ack_conf<=8'hFF; ack_ready<=0; ack_ok<=0; ack_expect<=4'd12;
        rx_active<=0; rx_timer<=0; rx_timeout<=0;
        pkt[0]<=8'h00; pkt[1]<=8'h00; pkt[2]<=8'h00; pkt[3]<=8'h00;
        pkt[4]<=8'h00; pkt[5]<=8'h00; pkt[6]<=8'h00; pkt[7]<=8'h00;
        pkt[8]<=8'h00; pkt[9]<=8'h00; pkt[10]<=8'h00; pkt[11]<=8'h00;
        pkt[12]<=8'h00; pkt[13]<=8'h00; pkt[14]<=8'h00; pkt[15]<=8'h00; pkt[16]<=8'h00;
    end else begin
        tx_start<=0; ack_ready<=0; prev_busy<=tx_busy; busy<=sending;

        // Packet send engine
        if (sending) begin
            if (prev_busy && !tx_busy) begin
                if (pkt_ptr < pkt_len) begin
                    tx_data<=pkt[pkt_ptr]; tx_start<=1; pkt_ptr<=pkt_ptr+1;
                end else sending<=0;
            end
        end

        // RX parser
        if (rx_active && rx_valid) begin
            if (rx_idx==4'd9) begin ack_conf<=rx_data; ack_ok<=(rx_data==8'h00); end
            if (rx_idx==ack_expect-1) begin ack_ready<=1; rx_active<=0; end
            else rx_idx<=rx_idx+1;
        end

        // RX timeout
        if (!rx_active) begin rx_timer<=0; rx_timeout<=0; end
        else if (ack_ready) begin rx_timer<=0; rx_timeout<=0; end
        else if (rx_timer>=RX_TIMEOUT) rx_timeout<=1;
        else rx_timer<=rx_timer+1;

        // FSM
        case (state)
        ST_RESET: begin
            timer<=timer+1;
            if (timer>=STARTUP_DLY) begin timer<=0; state<=ST_BOOT_WAIT; end
        end
        ST_BOOT_WAIT: begin
            if (rx_valid && rx_data==8'h55) begin timer<=0; state<=ST_VFY_SEND; end
            else begin
                timer<=timer+1;
                if (timer>=CLK_FREQ/2) begin timer<=0; state<=ST_VFY_SEND; end
            end
        end
        ST_VFY_SEND: begin
            if (!sending) begin
                pkt[0]<=8'hEF; pkt[1]<=8'h01; pkt[2]<=8'hFF; pkt[3]<=8'hFF;
                pkt[4]<=8'hFF; pkt[5]<=8'hFF; pkt[6]<=8'h01; pkt[7]<=8'h00;
                pkt[8]<=8'h07; pkt[9]<=8'h13; pkt[10]<=8'h00; pkt[11]<=8'h00;
                pkt[12]<=8'h00; pkt[13]<=8'h00; pkt[14]<=8'h00; pkt[15]<=8'h1B;
                rx_idx<=0; ack_conf<=8'hFF; ack_ok<=0; rx_active<=1; ack_expect<=4'd12;
                pkt_len<=5'd16; tx_data<=8'hEF; tx_start<=1; pkt_ptr<=5'd1; sending<=1;
                state<=ST_VFY_WAIT;
            end
        end
        ST_VFY_WAIT: begin
            if (ack_ready) begin
                if (ack_ok) begin sensor_ready<=1; state<=ST_SEC_SEND; end
                else begin error_flag<=1; state<=ST_ERROR; end
            end
            if (rx_timeout) begin error_flag<=1; state<=ST_ERROR; end
        end
        ST_SEC_SEND: begin
            if (!sending) begin
                pkt[0]<=8'hEF; pkt[1]<=8'h01; pkt[2]<=8'hFF; pkt[3]<=8'hFF;
                pkt[4]<=8'hFF; pkt[5]<=8'hFF; pkt[6]<=8'h01; pkt[7]<=8'h00;
                pkt[8]<=8'h05; pkt[9]<=8'h0E; pkt[10]<=8'h05; pkt[11]<=8'h01;
                pkt[12]<=8'h00; pkt[13]<=8'h1A;
                rx_idx<=0; ack_conf<=8'hFF; ack_ok<=0; rx_active<=1; ack_expect<=4'd12;
                pkt_len<=5'd14; tx_data<=8'hEF; tx_start<=1; pkt_ptr<=5'd1; sending<=1;
                state<=ST_SEC_WAIT;
            end
        end
        ST_SEC_WAIT: begin
            if (ack_ready) state<=ST_IDLE;
            if (rx_timeout) state<=ST_IDLE;
        end
        ST_IDLE: begin
            matched<=0; no_match<=0;
            if (finger_on) begin
                timer<=timer+1;
                if (timer>=INTER_CMD) begin timer<=0; state<=ST_GETIMG; end
            end else timer<=0;
        end
        ST_GETIMG: begin
            if (!sending) begin
                pkt[0]<=8'hEF; pkt[1]<=8'h01; pkt[2]<=8'hFF; pkt[3]<=8'hFF;
                pkt[4]<=8'hFF; pkt[5]<=8'hFF; pkt[6]<=8'h01; pkt[7]<=8'h00;
                pkt[8]<=8'h03; pkt[9]<=8'h01; pkt[10]<=8'h00; pkt[11]<=8'h05;
                rx_idx<=0; ack_conf<=8'hFF; ack_ok<=0; rx_active<=1; ack_expect<=4'd12;
                pkt_len<=5'd12; tx_data<=8'hEF; tx_start<=1; pkt_ptr<=5'd1; sending<=1;
                state<=ST_GETIMG_WAIT;
            end
        end
        ST_GETIMG_WAIT: begin
            if (ack_ready) begin
                if (ack_ok) state<=ST_IMG2TZ;
                else begin timer<=0; state<=ST_IDLE; end
            end
            if (rx_timeout) begin error_flag<=1; state<=ST_ERROR; end
        end
        ST_IMG2TZ: begin
            if (!sending) begin
                pkt[0]<=8'hEF; pkt[1]<=8'h01; pkt[2]<=8'hFF; pkt[3]<=8'hFF;
                pkt[4]<=8'hFF; pkt[5]<=8'hFF; pkt[6]<=8'h01; pkt[7]<=8'h00;
                pkt[8]<=8'h04; pkt[9]<=8'h02; pkt[10]<=8'h01; pkt[11]<=8'h00;
                pkt[12]<=8'h08;
                rx_idx<=0; ack_conf<=8'hFF; ack_ok<=0; rx_active<=1; ack_expect<=4'd12;
                pkt_len<=5'd13; tx_data<=8'hEF; tx_start<=1; pkt_ptr<=5'd1; sending<=1;
                state<=ST_IMG2TZ_WAIT;
            end
        end
        ST_IMG2TZ_WAIT: begin
            if (ack_ready) begin
                if (ack_ok) state<=ST_SEARCH;
                else begin timer<=0; state<=ST_IDLE; end
            end
            if (rx_timeout) begin error_flag<=1; state<=ST_ERROR; end
        end
        ST_SEARCH: begin
            if (!sending) begin
                pkt[0]<=8'hEF; pkt[1]<=8'h01; pkt[2]<=8'hFF; pkt[3]<=8'hFF;
                pkt[4]<=8'hFF; pkt[5]<=8'hFF; pkt[6]<=8'h01; pkt[7]<=8'h00;
                pkt[8]<=8'h08; pkt[9]<=8'h04; pkt[10]<=8'h01; pkt[11]<=8'h00;
                pkt[12]<=8'h00; pkt[13]<=8'h00; pkt[14]<=8'h64; pkt[15]<=8'h00;
                pkt[16]<=8'h72;
                rx_idx<=0; ack_conf<=8'hFF; ack_ok<=0; rx_active<=1; ack_expect<=4'd12;
                pkt_len<=5'd17; tx_data<=8'hEF; tx_start<=1; pkt_ptr<=5'd1; sending<=1;
                state<=ST_SEARCH_WAIT;
            end
        end
        ST_SEARCH_WAIT: begin
            if (ack_ready) begin
                matched  <= ack_ok ? 1'b1 : 1'b0;
                no_match <= ack_ok ? 1'b0 : 1'b1;
                timer<=0; state<=ST_RESULT;
            end
            if (rx_timeout) begin error_flag<=1; state<=ST_ERROR; end
        end
        ST_RESULT: begin
            timer<=timer+1;
            if (timer>=RESULT_HOLD && !finger_on) begin timer<=0; state<=ST_IDLE; end
        end
        ST_ERROR: error_flag<=1;
        default:  state<=ST_RESET;
        endcase
    end
end
endmodule
