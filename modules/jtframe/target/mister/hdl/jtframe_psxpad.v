/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * PlayStation controller bus master for the MiSTer USER port (SNAC).
 *
 * Polls port 1 of a PSX SNAC board and exposes the raw pad state. The
 * protocol is the one documented in psx-spx, "Controllers - Communication
 * Sequence": /ATT low, then bytes exchanged LSB first over a ~250 kHz clock,
 * CMD changing on the falling edge and DAT sampled on the rising edge
 * (CPHA=1). After every byte except the last the pad pulls /ACK low for a
 * few microseconds; a pad that never answers the first byte is absent.
 *
 * Host sends 01h 42h 00h M 00h ..., pad answers HiZ, ID, 5Ah, then
 * 2*(ID & 0Fh) data bytes. Only the first three halfwords are read (digital
 * switches plus two analog halfwords), which covers digital pads (41h),
 * DualShock (73h), the analog stick (53h), the neGcon (23h) and the Jogcon
 * (E3h). M is the motor byte for pads with a mapped motor (Jogcon hold).
 *
 * A pad that answers 41h gets the configuration sequence used by the MiSTer
 * JogConUSB firmware once: 43h 01h (enter config), 44h 01h 02h (analog/jog
 * mode, unlocked), 4Dh 00h 01h (map motors to poll bytes 3 and 4), 43h 00h
 * (exit config). A Jogcon then reports E3h with its dial counter zeroed at
 * the current position, a DualShock reports 73h with the sticks enabled and
 * a plain digital pad is unaffected. The sequence runs again if the pad
 * falls back to 41h (Mode button), which also re-zeroes the dial.
 *
 * Pin roles match the PSX_MiSTer core's SNAC implementation so the same
 * board works in both cores.
 */

module jtframe_psxpad #(
    parameter CLK_HZ = 48_000_000
)(
    input             rst,
    input             clk,
    input             en,      // low: bus idle, outputs cleared
    input      [ 7:0] motor,   // poll byte 3, e.g. Jogcon hold command

    // pad bus, port 1
    output reg        att_n,   // /ATT (chip select), active low
    output reg        cmd,     // host -> pad
    output reg        sck,     // idle high
    input             dat,     // pad -> host
    input             ack_n,   // pad -> host, pulses low after a byte

    // decoded state
    output reg        conn,    // a pad answered the last poll
    output reg [ 7:0] id,      // 41 digital, 23 neGcon, 73 DualShock, E3 Jogcon...
    output reg [15:0] btn,     // halfword 1, as on the wire (0 = pressed)
    output reg [31:0] ana,     // { byte6, byte5, byte4, byte3 }
    output reg        upd      // one-clock pulse when outputs refresh
);

// Timing in clock cycles
localparam HALF     = CLK_HZ/500_000;   // 2 us, half a 250 kHz period
localparam CS_SETUP = HALF*10;          // /ATT low to first clock
localparam CS_HOLD  = HALF*5;           // last clock to /ATT high
localparam ACK_MASK = HALF;             // ignore /ACK right after the byte
localparam ACK_TOUT = CLK_HZ/10_000;    // 100 us, as the PSX kernel does
localparam GAP      = HALF*2;           // /ACK seen to next byte
localparam POLL     = CLK_HZ/500;       // 2 ms between polls
localparam CFG_GAP  = CLK_HZ/1_000;     // 1 ms between configuration frames
localparam MAXBYTES = 9;                // HiZ, ID, 5A, 6 data bytes

localparam [2:0] IDLE   = 3'd0,
                 SETUP  = 3'd1,
                 BITLO  = 3'd2,
                 BITHI  = 3'd3,
                 ACKW   = 3'd4,
                 ACKGAP = 3'd5,
                 DONE   = 3'd6;

// frame kinds
localparam [2:0] F_POLL   = 3'd0,
                 F_CFGON  = 3'd1,
                 F_MODE   = 3'd2,
                 F_MOTOR  = 3'd3,
                 F_CFGOFF = 3'd4;

reg  [ 2:0] st;
reg  [23:0] tmr;
reg  [ 2:0] bitcnt;
reg  [ 3:0] bytecnt, nbytes;
reg  [ 7:0] txsr, rxsr;
reg  [ 7:0] rx [0:MAXBYTES-1];
reg  [ 2:0] dat_s, ack_s;
reg         fail;
reg  [ 2:0] frame;       // kind of the frame in flight
reg  [ 2:0] cfg_step;    // next configuration frame to send, 0 = none
reg  [ 7:0] last_id;     // ID seen by the previous poll, FF when none
wire        ack_low = ~ack_s[2] & ~ack_s[1];  // two consecutive samples
integer     i;

// Bytes in a frame for a given ID: HiZ, ID, 5A plus 2*(ID & 0Fh) data
// bytes, capped at the three halfwords this module keeps.
function [3:0] frame_len( input [7:0] pad_id );
    frame_len = pad_id[3:0] >= 4'd3 ? MAXBYTES[3:0] : 4'd3 + {pad_id[2:0],1'b0};
endfunction

// Command byte for a given position in a frame of a given kind
function [7:0] tx_byte( input [2:0] kind, input [3:0] pos );
    case( pos )
        4'd0: tx_byte = 8'h01;
        4'd1: case( kind )
                F_CFGON, F_CFGOFF: tx_byte = 8'h43;
                F_MODE:            tx_byte = 8'h44;
                F_MOTOR:           tx_byte = 8'h4d;
                default:           tx_byte = 8'h42;
              endcase
        4'd3: case( kind )
                F_POLL:  tx_byte = motor;
                F_CFGON: tx_byte = 8'h01;
                F_MODE:  tx_byte = 8'h01;
                default: tx_byte = 8'h00;
              endcase
        4'd4: case( kind )
                F_MODE:  tx_byte = 8'h02;   // do not lock the mode
                F_MOTOR: tx_byte = 8'h01;
                default: tx_byte = 8'h00;
              endcase
        default: tx_byte = kind==F_MOTOR && pos>=4'd5 ? 8'hff : 8'h00;
    endcase
endfunction

wire        id_bad   = rxsr==8'hff || rxsr[3:0]==4'd0;   // valid on the ID byte
wire [3:0]  nbytes_id= id_bad ? 4'd2 : frame_len(rxsr);  // frame length once ID known
wire [3:0]  nbytes_eff = bytecnt==4'd1 ? nbytes_id : nbytes;
wire        last_byte = (bytecnt+4'd1) >= nbytes_eff;
wire [2:0]  next_frame = cfg_step==3'd0 ? F_POLL : cfg_step;   // cfg_step 1..4 = F_CFGON..F_CFGOFF

always @(posedge clk) begin
    dat_s <= { dat_s[1:0], dat   };
    ack_s <= { ack_s[1:0], ack_n };
end

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        st      <= IDLE;
        tmr     <= 0;
        att_n   <= 1;
        cmd     <= 1;
        sck     <= 1;
        conn    <= 0;
        id      <= 8'hff;
        btn     <= 16'hffff;
        ana     <= 32'h8080_8080;
        upd     <= 0;
        bitcnt  <= 0;
        bytecnt <= 0;
        nbytes  <= 4'd3;
        txsr    <= 8'hff;
        rxsr    <= 8'hff;
        fail    <= 0;
        frame   <= F_POLL;
        cfg_step<= 0;
        last_id <= 8'hff;
        for( i=0; i<MAXBYTES; i=i+1 ) rx[i] <= 8'hff;
    end else begin
        upd <= 0;
        if( !en ) begin
            st       <= IDLE;
            tmr      <= 0;
            att_n    <= 1;
            cmd      <= 1;
            sck      <= 1;
            cfg_step <= 0;
            last_id  <= 8'hff;
            if( conn ) begin
                conn <= 0;
                id   <= 8'hff;
                btn  <= 16'hffff;
                ana  <= 32'h8080_8080;
                upd  <= 1;
            end
        end else begin
            tmr <= tmr + 1'd1;
            case( st )
            IDLE: begin
                att_n <= 1;
                cmd   <= 1;
                sck   <= 1;
                if( tmr >= (cfg_step!=3'd0 ? CFG_GAP : POLL) ) begin
                    tmr     <= 0;
                    att_n   <= 0;
                    bytecnt <= 0;
                    nbytes  <= 4'd3;   // until the ID is known
                    fail    <= 0;
                    frame   <= next_frame;
                    for( i=0; i<MAXBYTES; i=i+1 ) rx[i] <= 8'hff;
                    st      <= SETUP;
                end
            end
            // /ATT is low. Start the first byte: falling edge with bit 0 on CMD
            SETUP: if( tmr >= CS_SETUP ) begin
                tmr    <= 0;
                bitcnt <= 0;
                txsr   <= tx_byte(frame, bytecnt);
                cmd    <= tx_byte(frame, bytecnt) & 8'h01 ? 1'b1 : 1'b0;
                sck    <= 0;
                st     <= BITLO;
            end
            // SCK low, CMD stable. At the end sample DAT and raise SCK.
            BITLO: if( tmr >= HALF ) begin
                tmr  <= 0;
                sck  <= 1;
                rxsr <= { dat_s[1], rxsr[7:1] };
                st   <= BITHI;
            end
            BITHI: if( tmr >= HALF ) begin
                tmr <= 0;
                if( bitcnt == 3'd7 ) begin
                    rx[bytecnt] <= rxsr;
                    cmd         <= 1;
                    if( bytecnt == 4'd1 ) nbytes <= nbytes_id;
                    if( last_byte ) begin
                        st <= DONE;          // no /ACK after the last byte
                    end else begin
                        st <= ACKW;
                    end
                end else begin
                    bitcnt <= bitcnt + 1'd1;
                    sck    <= 0;
                    txsr   <= { 1'b1, txsr[7:1] };
                    cmd    <= txsr[1];
                    st     <= BITLO;
                end
            end
            ACKW: begin
                if( tmr > ACK_MASK && ack_low ) begin
                    tmr <= 0;
                    st  <= ACKGAP;
                end else if( tmr >= ACK_TOUT ) begin
                    fail <= 1;               // pad absent or gave up
                    tmr  <= 0;
                    st   <= DONE;
                end
            end
            ACKGAP: if( tmr >= GAP ) begin
                tmr     <= 0;
                bytecnt <= bytecnt + 1'd1;
                bitcnt  <= 0;
                txsr    <= tx_byte(frame, bytecnt+4'd1);
                cmd     <= tx_byte(frame, bytecnt+4'd1) & 8'h01 ? 1'b1 : 1'b0;
                sck     <= 0;
                st      <= BITLO;
            end
            DONE: if( tmr >= CS_HOLD ) begin
                tmr   <= 0;
                att_n <= 1;
                st    <= IDLE;
                if( frame != F_POLL ) begin
                    // configuration frame: advance, outputs untouched
                    cfg_step <= cfg_step==3'd4 ? 3'd0 : cfg_step + 3'd1;
                end else begin
                    upd <= 1;
                    if( !fail && nbytes >= 4'd5 && rx[2]==8'h5a ) begin
                        conn <= 1;
                        id   <= rx[1];
                        btn  <= { rx[4], rx[3] };
                        ana  <= nbytes >= 4'd9 ? { rx[8], rx[7], rx[6], rx[5] } :
                                nbytes >= 4'd7 ? { 8'h80, 8'h80, rx[6], rx[5] } :
                                                 32'h8080_8080;
                        last_id <= rx[1];
                        // a pad that just turned up as digital gets configured once
                        if( rx[1]==8'h41 && last_id!=8'h41 ) cfg_step <= 3'd1;
                    end else begin
                        conn    <= 0;
                        id      <= 8'hff;
                        btn     <= 16'hffff;
                        ana     <= 32'h8080_8080;
                        last_id <= 8'hff;
                    end
                end
            end
            default: st <= IDLE;
            endcase
        end
    end
end

endmodule
