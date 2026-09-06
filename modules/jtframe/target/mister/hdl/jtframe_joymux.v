/* SPDX-FileCopyrightText: 2026 Jose Tejada Gomez
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Date: 24-10-2021 */

// USER port joystick mux. Selects between the USB joysticks from the HPS,
// Antonio Villena's DB15 interface and a PlayStation pad on a PSX SNAC
// board. The PSX pad drives player 1 only and also replaces the player 1
// analog sticks so that analog pads (DualShock, neGcon) work with cores that
// read joyana_l1/joyana_r1.

module jtframe_joymux(
    input             rst,
    input             clk,
    output reg        show_osd,

    // MiSTer pins
    input      [ 6:0] USER_IN,
    output     [ 6:0] USER_OUT,

    // joystick mux
    input             db15_en,
    input             psx_en,
    input      [15:0] joyusb_1,
    input      [15:0] joyusb_2,
    input      [15:0] anausb_l1,
    input      [15:0] anausb_r1,
    input      [ 8:0] spinusb_1,
    input      [ 7:0] padusb_1,
    output reg [15:0] joymux_1,
    output reg [15:0] joymux_2,
    output reg [15:0] anamux_l1,
    output reg [15:0] anamux_r1,
    output reg [ 8:0] spinmux_1,   // MiSTer spinner: [8] toggles per step, [7:0] signed step
    output reg [ 7:0] padmux_1     // MiSTer paddle 0..255
);

parameter BUTTONS = 2;
parameter CLK_HZ  = `ifdef JTFRAME_SDRAM96 96_000_000 `else 48_000_000 `endif;

// same as defined in jtframe_inputs
localparam START_BIT  = 6+(BUTTONS-2);
localparam COIN_BIT   = 7+(BUTTONS-2);

localparam [7:0] ID_NEGCON = 8'h23;
localparam [7:0] ID_JOGCON = 8'he3;

// Jogcon: dial counts either side of centre that mean full lock (the MiSTer
// JogConUSB firmware uses the same 80), and the motor hold strength 1-15.
localparam JOG_RANGE = 80;
localparam JOG_FORCE = 15;
// One spinner step per JOG_SPIN_STEP dial counts, paced at JOG_SPIN_HZ so
// jtframe_dial never sees two toggles before it has drained the first.
localparam JOG_SPIN_STEP = 1;
localparam JOG_SPIN_HZ   = 10_000;
localparam JOG_SPIN_DIV  = CLK_HZ/JOG_SPIN_HZ;
// Jogcon role follows the input the core consumes: dial cores get a free
// spinner, paddle cores a paddle with motor stops at both ends, the rest
// a self-centring wheel on left X.
localparam ROLE_WHEEL = 0, ROLE_SPINNER = 1, ROLE_PADDLE = 2;
localparam JOG_ROLE = `ifdef JTFRAME_DIAL ROLE_SPINNER `elsif JTFRAME_PADDLE ROLE_PADDLE `else ROLE_WHEEL `endif;

wire [15:0] joydb15_1,joydb15_2;
wire        joy_din, joy_clk, joy_load;

// PSX pad
wire        psx_att_n, psx_cmd, psx_sck, psx_dat, psx_ack_n, psx_conn;
wire [ 7:0] psx_id;
wire [15:0] psx_btn;
wire [31:0] psx_ana;
wire [ 7:0] psx_motor;
wire        psx_upd;
wire        is_jog = psx_conn && psx_id==ID_JOGCON;

// USER_OUT: pins not driven by the selected mode stay high (input)
// PSX SNAC pin roles follow PSX_MiSTer: 0 /ATT2, 1 /ATT1, 2 CMD, 3 ACK, 4 DAT, 5 CLK, 6 IRQ
assign USER_OUT  = psx_en  ? { 1'b1, psx_sck, 2'b11, psx_cmd, psx_att_n, 1'b1 } :
                   db15_en ? { 5'h1f, joy_clk, joy_load } : 7'h7f;
assign joy_din   = USER_IN[5];
assign psx_dat   = USER_IN[4];
assign psx_ack_n = USER_IN[3];

function [15:0] assign_joy(
    input [15:0] joydb,
    input [15:0] joyusb
);
    if( db15_en ) begin
        assign_joy = 0;
        assign_joy[BUTTONS+3:0] = joydb[BUTTONS+3:0];
        assign_joy[COIN_BIT]    = joydb[11]; // select
        assign_joy[START_BIT]   = joydb[10]; // start
    end else begin
        assign_joy = joyusb;
    end
endfunction

// PSX halfword 1 (0 = pressed) to the active-high MiSTer joystick layout:
// bits 3:0 up/down/left/right, then buttons, then start and coin.
// Regular pads: cross, circle, square, triangle, L1, R1 as buttons 1-6.
// neGcon: A, B, R as buttons 1-3 (I and II are analog, see below).
function [15:0] psx2joy(
    input [ 7:0] pad_id,
    input [15:0] b
);
    reg [15:0] j;
    j = 0;
    j[0] = ~b[5];   // right
    j[1] = ~b[7];   // left
    j[2] = ~b[6];   // down
    j[3] = ~b[4];   // up
    if( pad_id == ID_NEGCON ) begin
        j[4] = ~b[13];  // A
        j[5] = ~b[12];  // B
        j[6] = ~b[11];  // R
    end else begin
        j[4] = ~b[14];  // cross
        j[5] = ~b[13];  // circle
        j[6] = ~b[15];  // square
        j[7] = ~b[12];  // triangle
        j[8] = ~b[10];  // L1
        j[9] = ~b[11];  // R1
    end
    // do not let unused button positions bleed into start/coin
    j = j & ((16'd1 << (BUTTONS+4)) - 16'd1);
    j[START_BIT] = ~b[3];   // start
    j[COIN_BIT]  = ~b[0];   // select
    psx2joy = j;
endfunction

// neGcon twist centre: the value seen in the first frame after the pad
// connects is taken as zero (the pad is at rest when the core starts, and
// toggling the OSD option re-centres it). Small dead zones keep a pad that
// does not return exactly to rest from steering or creeping the pedals.
localparam TWIST_DZ = 3;   // counts either side of centre
localparam PEDAL_DZ = 8;   // counts, out of 255

reg  [7:0] twist_off = 8'h80;
reg        conn_l    = 0;

always @(posedge clk) begin
    conn_l <= psx_conn;
    if( psx_conn && !conn_l ) twist_off <= psx_id==ID_NEGCON ? psx_ana[7:0] : 8'h80;
end

// centre, clamp to -128..127 and apply the dead zone
function [7:0] centre_twist( input [7:0] raw, input [7:0] off );
    reg signed [9:0] d;
    d = $signed({2'b0,raw}) - $signed({2'b0,off});
    if( d > 10'sd127 )  d = 10'sd127;
    if( d < -10'sd128 ) d = -10'sd128;
    if( d <= $signed(TWIST_DZ[9:0]) && d >= -$signed(TWIST_DZ[9:0]) ) d = 0;
    centre_twist = d[7:0];
endfunction

function [7:0] pedal_up( input [7:0] raw );   // 00..FF -> 0..-127, pushing the stick up
    pedal_up = raw <= PEDAL_DZ[7:0] ? 8'd0 : 8'd0 - {1'b0, raw[7:1]};
endfunction

// Jogcon dial counter (signed, zero at init) to a signed stick byte:
// clamp to +/-JOG_RANGE, then scale by 127/JOG_RANGE (813/512 for 80)
function [7:0] jog2axis( input [15:0] pos );
    reg        neg;
    reg [15:0] mag;
    reg [25:0] m;
    neg = pos[15];
    mag = neg ? (16'd0 - pos) : pos;
    if( mag > JOG_RANGE[15:0] ) mag = JOG_RANGE[15:0];
    m   = mag * 26'd813;
    jog2axis = neg ? (8'd0 - m[16:9]) : m[16:9];
endfunction

// Analog halfwords to MiSTer signed sticks ({Y,X}, right/down positive).
// neGcon: twist on left X, I on left Y pushed up, II on right Y pushed up,
// so a core's "steering wheel with pedals" mode sees wheel, gas and brake.
function [31:0] psx2ana(     // { r1, l1 }
    input [ 7:0] pad_id,
    input [31:0] a,          // { byte6, byte5, byte4, byte3 }
    input [ 7:0] off
);
    if( pad_id == ID_NEGCON ) begin
        psx2ana = { pedal_up(a[23:16]), 8'h00, pedal_up(a[15:8]), centre_twist(a[7:0], off) };
    end else if( pad_id == ID_JOGCON ) begin
        // dial on left X, self-centred by the motor hold command
        psx2ana = { 16'h0000, 8'h00, jog2axis(a[15:0]) };
    end else begin
        // DualShock / analog stick: bytes are RX, RY, LX, LY with 80h centre
        psx2ana = { a[15:8]^8'h80, a[7:0]^8'h80, a[31:24]^8'h80, a[23:16]^8'h80 };
    end
endfunction

wire [15:0] psxjoy_1 = psx_conn ? psx2joy( psx_id, psx_btn ) : 16'd0;
wire [31:0] psxana_1 = psx_conn ? psx2ana( psx_id, psx_ana, twist_off ) : 32'd0;
// Jogcon motor: hold the dial at its zero position with the configured
// strength. Wheel role: always. Paddle role: only past either end, which
// pushes the dial back like a stopper. Spinner role: never.
wire signed [15:0] jog_pos = $signed(psx_ana[15:0]);
wire        jog_past_end = jog_pos > $signed(JOG_RANGE[15:0]) || jog_pos < -$signed(JOG_RANGE[15:0]);
wire        jog_hold = JOG_ROLE==ROLE_WHEEL ? 1'b1 : JOG_ROLE==ROLE_PADDLE ? jog_past_end : 1'b0;
assign psx_motor = (is_jog && jog_hold) ? { 4'h3, JOG_FORCE[3:0] } : 8'h00;

// Jogcon dial as paddle: absolute position, 0..255 across +/-JOG_RANGE
function [7:0] jog2paddle( input [15:0] pos );
    reg signed [15:0] p;
    reg        [15:0] u;
    reg        [23:0] m;
    p = $signed(pos);
    if( p >  $signed(JOG_RANGE[15:0]) ) p =  JOG_RANGE[15:0];
    if( p < -$signed(JOG_RANGE[15:0]) ) p = -JOG_RANGE[15:0];
    u = p + JOG_RANGE[15:0];                 // 0..2*JOG_RANGE
    m = u * 24'd408;                         // 255/160 = 408/256 for a range of 80
    jog2paddle = m[15:8];
endfunction

// Jogcon dial as spinner: accumulate the counter change per poll and emit
// one paced toggle per JOG_SPIN_STEP counts, sign in bit 7 (MiSTer style)
reg  signed [15:0] jog_prev = 0, jog_acc = 0;
reg  [15:0] spin_div = 0;
reg         psx_upd_l = 0, jog_l = 0;
wire signed [15:0] jog_delta = jog_pos - jog_prev;

always @(posedge clk) begin
    psx_upd_l <= psx_upd;
    jog_l     <= is_jog;
    if( !is_jog ) begin
        jog_prev <= 0;
        jog_acc  <= 0;
    end else if( psx_upd_l ) begin
        if( jog_l ) jog_acc <= jog_acc + jog_delta;   // first frame after connect sets the reference only
        jog_prev <= jog_pos;
    end
    spin_div <= spin_div==JOG_SPIN_DIV-1 ? 16'd0 : spin_div + 16'd1;
    if( spin_div==16'd0 && is_jog && !psx_upd_l ) begin
        if( jog_acc >= $signed(JOG_SPIN_STEP[15:0]) ) begin
            jog_acc   <= jog_acc - JOG_SPIN_STEP[15:0];
            spinmux_1 <= { ~spinmux_1[8], 8'h01 };
        end else if( jog_acc <= -$signed(JOG_SPIN_STEP[15:0]) ) begin
            jog_acc   <= jog_acc + JOG_SPIN_STEP[15:0];
            spinmux_1 <= { ~spinmux_1[8], 8'hff };
        end
    end
    if( !psx_en ) spinmux_1 <= spinusb_1;
end

always @(posedge clk) begin
    joymux_1  <= psx_en ? psxjoy_1 : assign_joy( joydb15_1, joyusb_1 );
    joymux_2  <= assign_joy( joydb15_2, joyusb_2 );
    anamux_l1 <= psx_en ? psxana_1[15: 0] : anausb_l1;
    anamux_r1 <= psx_en ? psxana_1[31:16] : anausb_r1;
    padmux_1  <= !psx_en ? padusb_1 : is_jog ? jog2paddle(psx_ana[15:0]) : 8'h80;
    show_osd  <= db15_en & ((joydb15_1[10] & joydb15_1[6]) | (joydb15_2[10]&joydb15_2[6]));
end

joy_db15 u_db15
(
  .clk       ( clk       ), //48MHz
  .JOY_CLK   ( joy_clk   ),
  .JOY_DATA  ( joy_din   ),
  .JOY_LOAD  ( joy_load  ),
  .joystick1 ( joydb15_1 ),
  .joystick2 ( joydb15_2 )
);

jtframe_psxpad #(.CLK_HZ(CLK_HZ)) u_psxpad(
    .rst       ( rst       ),
    .clk       ( clk       ),
    .en        ( psx_en    ),
    .motor     ( psx_motor ),
    .att_n     ( psx_att_n ),
    .cmd       ( psx_cmd   ),
    .sck       ( psx_sck   ),
    .dat       ( psx_dat   ),
    .ack_n     ( psx_ack_n ),
    .conn      ( psx_conn  ),
    .id        ( psx_id    ),
    .btn       ( psx_btn   ),
    .ana       ( psx_ana   ),
    .upd       ( psx_upd   )
);

endmodule
