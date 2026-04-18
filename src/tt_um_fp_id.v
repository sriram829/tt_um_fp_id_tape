// =============================================================================
// FILE    : tt_um_fp_id.v
// PROJECT : Tiny Tapeout TTSKY26a — AS-68M Fingerprint Verification ASIC
// PDK     : SKY130B (SkyWater 130 nm)
//
// AUTHOR  : Dr. Sriram Anbalagan
//           Post Doctoral Fellow, Visvesvaraya PhD Scheme (MeitY)
//           SASTRA Deemed University, Thanjavur
//
// GUIDE   : Dr. T.N. Prabakar
//           Associate Professor, SEEE, SASTRA Deemed University
//
// FPGA PROTOTYPE TEAM:
//           Rufina A Albert, Rojhaasree S V, Sri Hariharan Senthil
//           3rd Year B.Tech Electronics Engineering (VLSI Design & Technology)
//           SASTRA Deemed University
//
// REQUIREMENTS TRACEABILITY (DO254-003):
//   REQ-001 : System shall verify enrolled fingerprint via AS-68M over UART
//   REQ-002 : MATCH_OUT shall assert HIGH for 3 s on enrolled finger detection
//   REQ-003 : System shall operate from a single 50 MHz clock (single domain)
//   REQ-004 : All external async inputs shall be double-flop synchronised (CDC)
//   REQ-005 : System shall report error state on protocol or timeout failure
//
// PURPOSE:
//   Verification-only ASIC — assumes fingerprint already enrolled in sensor flash.
//   No enrollment logic. Flow: power-on → VfyPwd → SetSysPara → IDLE →
//   GetImg → Img2Tz → 1:N Search → MATCH_OUT HIGH 3 s → repeat.
//
// RTL REVIEW FIXES (ref: rtl_report_tnpsir.csv):
//   1. Module renamed to tt_um_fp_id per TT naming requirement
//   2. Ports remapped to TT interface (ui_in[7:0], uo_out[7:0], uio_*)
//   3. FIXED: Added 5 missing output ports (HEARTBEAT, SENSOR_RDY,
//             FINGER_DET, UART_BUSY, ERROR_OUT) — present in QSF pin
//             assignments but absent from original top.v (top.v mismatch)
//   4. FIXED: Added heartbeat counter register and logic (was missing)
//   5. FIXED: Added all 7 output assign statements (only 2 were in top.v)
//   6. FIXED: Added `default_nettype none (PVR-004 / SI-004)
//   7. FIXED: Added port-level documentation comments (PVR-004)
//   8. FIXED: Added _unused wire for TT unused inputs (Verilator lint)
//
// CLOCK DOMAIN (STARC-003 waiver — false positive):
//   Single clock domain: clk (50 MHz). rst is asynchronous reset only,
//   not a clock source. Double-flop synchronisers present on all async
//   inputs per REQ-004:
//     FP_TOUCH  → ts1/ts2 in this module
//     UART_RX   → s1/s2  in uart_rx.v
//
// TT PIN MAP
// ─────────────────────────────────────────────────────────────────────────────
//  ui_in[0]  ← uart_rx     UART data in from sensor TD pin 4 (57600 8-N-1)
//  ui_in[1]  ← fp_touch    Finger-present from sensor ST pin 2 (active HIGH)
//  ui_in[7:2] ← unused     Tie LOW on PCB
//
//  uo_out[0] → uart_tx     UART command to sensor RD pin 5
//  uo_out[1] → match_out   HIGH 3 s on enrolled finger  *** PRIMARY OUTPUT ***
//  uo_out[2] → no_match_out  HIGH 3 s on unknown finger
//  uo_out[3] → error_out   HIGH on protocol/timeout error; release rst_n to recover
//  uo_out[4] → sensor_rdy  HIGH after VfyPwd ACK — sensor authenticated
//  uo_out[5] → finger_det  HIGH while finger on sensor (double-flopped fp_touch)
//  uo_out[6] → uart_busy   HIGH during UART transaction
//  uo_out[7] → heartbeat   ~1.5 Hz toggle — ASIC core alive
//
//  uio[7:0]  — unused; uio_oe = 0x00 (all configured as inputs)
//
// SENSOR WIRING (AS-68M / AS-68ML)
//   Sensor Pin 1  VT  → 3.3 V
//   Sensor Pin 2  ST  → ui_in[1]  (fp_touch)
//   Sensor Pin 3  VIN → 3.3 V
//   Sensor Pin 4  TD  → ui_in[0]  (uart_rx into ASIC)
//   Sensor Pin 5  RD  → uo_out[0] (uart_tx from ASIC)
//   Sensor Pin 6  GND → GND
// =============================================================================

`default_nettype none   // Prevents implicit wire declarations (SI-004 fix)

module tt_um_fp_id (
    // ── TT dedicated inputs ──────────────────────────────────────────────────
    input  wire [7:0] ui_in,   // [0]=uart_rx (sensor TD), [1]=fp_touch (sensor ST)
    // ── TT dedicated outputs ─────────────────────────────────────────────────
    output wire [7:0] uo_out,  // [0]=uart_tx [1]=match_out [2]=no_match [3]=error
                               // [4]=sensor_rdy [5]=finger_det [6]=uart_busy [7]=heartbeat
    // ── TT bidirectional IOs (unused — configured as inputs) ─────────────────
    input  wire [7:0] uio_in,  // not connected
    output wire [7:0] uio_out, // driven to 0x00
    output wire [7:0] uio_oe,  // 0x00 = all inputs (DIR-002 waiver: async reset OK)
    // ── TT control ───────────────────────────────────────────────────────────
    input  wire       ena,     // tile enable — always 1 when selected; unused
    input  wire       clk,     // project clock — 50 MHz single clock domain (REQ-003)
    input  wire       rst_n    // active-LOW async reset; release after power-on settle
);

// ── Parameters ────────────────────────────────────────────────────────────────
// CLK_FREQ must match clock_hz in info.yaml and CLOCK_PERIOD in config.json.
// BAUD_RATE 57600 → CLKS_PER_BIT = 50_000_000/57600 = 868 (0.06% error, well
// within AS-68M UART tolerance).
parameter CLK_FREQ  = 50_000_000; // Hz  (single clock domain per REQ-003)
parameter BAUD_RATE = 57600;      // bps (AS-68M default UART rate)

// ── Internal active-HIGH reset derived from TT active-LOW rst_n ───────────────
// Async reset is intentional: ensures deterministic power-on state on SKY130.
// STARC-003 / DIR-002: rst is a reset signal, not a second clock domain.
wire rst = ~rst_n;

// ── FP_TOUCH double-flop synchroniser (REQ-004 CDC compliance) ────────────────
// FP_TOUCH (sensor ST pin) is asynchronous to clk. Two flip-flops eliminate
// metastability before the signal enters any combinational logic.
// SI-004 waiver: fp_touch_in is async by design; synchronised here.
reg ts1, ts2;
always @(posedge clk or posedge rst)
    if (rst) begin ts1 <= 1'b0; ts2 <= 1'b0; end
    else     begin ts1 <= ui_in[1]; ts2 <= ts1; end

wire finger_on = ts2; // synchronised, metastability-free finger detect

// ── UART TX ───────────────────────────────────────────────────────────────────
// Sends AS-68M command packets (VfyPwd / SetSysPara / GetImg / Img2Tz / Search)
wire tx_start;          // pulse HIGH 1 clock to begin byte transmission
wire [7:0] tx_data;     // byte to transmit (from verify_ctrl packet engine)
wire tx_busy;           // HIGH during transmission — prevents new tx_start
wire uart_tx_pin;       // serial output → sensor RD pin 5

uart_tx #(
    .CLK_FREQ  (CLK_FREQ),
    .BAUD_RATE (BAUD_RATE)
) u_tx (
    .clk      (clk),
    .rst      (rst),
    .tx_start (tx_start), // input: start transmission trigger
    .tx_data  (tx_data),  // input: byte to send
    .tx_busy  (tx_busy),  // output: busy flag
    .tx_pin   (uart_tx_pin) // output: serial TX to sensor
);

// ── UART RX ───────────────────────────────────────────────────────────────────
// Receives ACK packets from AS-68M sensor.
// uart_rx.v has its own internal double-flop on rx_pin (REQ-004).
wire rx_valid;          // pulses HIGH 1 clock when a complete byte is received
wire [7:0] rx_data;     // received byte value

uart_rx #(
    .CLK_FREQ  (CLK_FREQ),
    .BAUD_RATE (BAUD_RATE)
) u_rx (
    .clk      (clk),
    .rst      (rst),
    .rx_pin   (ui_in[0]), // input: serial RX from sensor TD pin 4
    .rx_valid (rx_valid), // output: byte-ready pulse
    .rx_data  (rx_data)   // output: received byte
);

// ── Verification FSM ──────────────────────────────────────────────────────────
// Implements the full AS-68M command protocol state machine.
// States: RESET → BOOT_WAIT → VFY_SEND → VFY_WAIT → SEC_SEND → SEC_WAIT
//         → IDLE → GETIMG → GETIMG_WAIT → IMG2TZ → IMG2TZ_WAIT
//         → SEARCH → SEARCH_WAIT → RESULT → ERROR
wire sensor_ready; // HIGH after VfyPwd ACK conf=0x00 (REQ-001)
wire busy;         // HIGH while any UART transaction is in progress
wire matched;      // HIGH during RESULT state when Search conf=0x00 (REQ-002)
wire no_match;     // HIGH during RESULT state when Search conf=0x09
wire error_flag;   // HIGH on timeout or protocol error (REQ-005)

verify_ctrl #(
    .CLK_FREQ (CLK_FREQ)
) u_ctrl (
    .clk          (clk),
    .rst          (rst),
    .finger_on    (finger_on),    // synchronised fp_touch
    .tx_start     (tx_start),     // to uart_tx
    .tx_data      (tx_data),      // to uart_tx
    .tx_busy      (tx_busy),      // from uart_tx
    .rx_valid     (rx_valid),     // from uart_rx
    .rx_data      (rx_data),      // from uart_rx
    .sensor_ready (sensor_ready), // status output
    .busy         (busy),         // status output
    .matched      (matched),      // result output
    .no_match     (no_match),     // result output
    .error_flag   (error_flag)    // fault output
);

// ── Heartbeat counter (~1.5 Hz at 50 MHz) ────────────────────────────────────
// FIX: heartbeat register and logic were missing from original top.v.
// bit[25] of a 26-bit counter toggles every 2^25 = 33,554,432 clocks
// → period = 2 × 33,554,432 / 50,000,000 ≈ 1.34 s (≈ 1.5 Hz)
// Provides visual confirmation that the ASIC core is alive.
reg [25:0] hb_cnt;
always @(posedge clk or posedge rst)
    if (rst) hb_cnt <= 26'd0;
    else     hb_cnt <= hb_cnt + 1'b1;

// ── Output assignments ────────────────────────────────────────────────────────
// FIX: original top.v had only 2 of 7 assign statements.
// All 7 outputs mapped here to uo_out bits per TT pin map above.
assign uo_out[0] = uart_tx_pin;  // UART TX to sensor RD pin
assign uo_out[1] = matched;      // MATCH_OUT    *** PRIMARY OUTPUT (REQ-002) ***
assign uo_out[2] = no_match;     // NO_MATCH_OUT
assign uo_out[3] = error_flag;   // ERROR_OUT    (REQ-005)
assign uo_out[4] = sensor_ready; // SENSOR_RDY   (REQ-001)
assign uo_out[5] = finger_on;    // FINGER_DET   (double-flopped, REQ-004)
assign uo_out[6] = busy;         // UART_BUSY
assign uo_out[7] = hb_cnt[25];   // HEARTBEAT    (~1.5 Hz)

// ── Bidirectional IOs — all unused, driven as inputs ──────────────────────────
assign uio_out = 8'h00;
assign uio_oe  = 8'h00; // 0 = input direction

// ── Suppress unused-input lint warnings (Verilator UNUSEDSIGNAL) ─────────────
wire _unused = &{ena, ui_in[7:2], uio_in, 1'b0};

endmodule
