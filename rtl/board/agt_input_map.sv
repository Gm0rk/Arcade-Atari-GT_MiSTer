// agt_input_map.sv -- MiSTer joystick words -> the two switch ports the
// 68EC020 reads
//
// tb/tb_input_map.sv checks every input of both players, on both control
// panels, against vectors that tools/check_inputs.py derives from atarigt.cpp.
//
// Joystick words: MiSTer packs [0] right, [1] left, [2] down, [3] up, then
// one bit per entry of the CONF_STR J1 string, in order:
//   "J1,Quick High,Fierce High,Quick Low,Fierce Low,Start,Coin;"
//        [4]        [5]         [6]       [7]        [8]   [9]
// (the cabinet's 2x2 block read row by row). J1 is positional: an entry
// inserted in the middle renumbers every one after it, here and in the .mra's
// <buttons>, which gen_mra.py reads from J1.
//
// Ports: every bit is active low; idle is all ones. Unused bits and the COIN
// port's six IPT_CUSTOM bits stay high (the memmap XORs its interrupt status
// into those).
//
// The three revisions have two different control panels. The game's own
// controls test says which bit is which button: a table in each program ROM
// gives, per switch, its mask, a screen position and a label ("QCK" or
// "FRC"), laid out as the cabinet's 2x2 block, top row high, bottom row low:
//
//   button        primrage (v2.3 Jan 1995)      primrageo, primrage20
//                 panel 0, table 0xC164         panel 1, 0xC164 / 0xBF70
//   Quick High    bit 25   (P2 bit 9)           bit 24 (P2 bit 8), also Start
//   Fierce High   bit 26   (P2 bit 10)          bit 25 (P2 bit 9)
//   Quick Low     bit 27   (P2 bit 11)          bit 26 (P2 bit 10)
//   Fierce Low    bit 1    (P2 bit 3)           bit 27 (P2 bit 11)
//   Start         bit 24   (P2 bit 8)           -- (Quick High starts)
//
// These are MAME's BUTTON1..4 in both INPUT_PORTS blocks: the parent gave
// START its own switch on bit 24 and shifted the four buttons up one, the
// last landing on bit 1.
//
// panel_id comes from the .mra (rom index 1, byte 1; agt_rom_download).
// 0, anything unrecognised, or an .mra without the byte selects the parent
// layout.
module agt_input_map (
	input  wire [31:0] joy0,          // MiSTer joystick_0 (player 1)
	input  wire [31:0] joy1,          // MiSTer joystick_1 (player 2)
	input  wire [7:0]  panel_id,      // 0 = dedicated start, 1 = Quick High starts
	output wire [31:0] p1_p2_port,    // 0xE80000, active low
	output wire [15:0] coin_in        // COIN port (special port 3), active low
);
	wire older = (panel_id == 8'd1);

	// J1 positions
	localparam int QH = 4, FH = 5, QL = 6, FL = 7, ST = 8, CN = 9;

	// bits 24..27 and 1 for player 1; 8..11 and 3 for player 2
	function automatic [4:0] panel_bits(input logic [31:0] j, input logic old);
		// returns {bit27, bit26, bit25, bit24, bit1} (P1) / {11,10,9,8,3} (P2)
		panel_bits = old ? { j[FL], j[QL], j[FH], j[QH] | j[ST], 1'b0 }
						 : { j[QL], j[FH], j[QH], j[ST],         j[FL] };
	endfunction

	wire [4:0] p1 = panel_bits(joy0, older);
	wire [4:0] p2 = panel_bits(joy1, older);

	assign p1_p2_port = ~{
		joy0[3],          // 31 P1 UP
		joy0[2],          // 30 P1 DOWN
		joy0[1],          // 29 P1 LEFT
		joy0[0],          // 28 P1 RIGHT
		p1[4],            // 27 parent Quick Low   | older Fierce Low
		p1[3],            // 26 parent Fierce High | older Quick Low
		p1[2],            // 25 parent Quick High  | older Fierce High
		p1[1],            // 24 parent START1      | older Quick High, also starts
		8'd0,             // 23:16 IPT_UNUSED
		joy1[3],          // 15 P2 UP
		joy1[2],          // 14 P2 DOWN
		joy1[1],          // 13 P2 LEFT
		joy1[0],          // 12 P2 RIGHT
		p2[4],            // 11 parent Quick Low   | older Fierce Low
		p2[3],            // 10 parent Fierce High | older Quick Low
		p2[2],            // 9  parent Quick High  | older Fierce High
		p2[1],            // 8  parent START2      | older Quick High, also starts
		4'd0,             // 7:4 IPT_UNUSED
		p2[0],            // 3  parent P2 Fierce Low | older unused
		1'b0,             // 2  IPT_UNUSED
		p1[0],            // 1  parent P1 Fierce Low | older unused
		1'b0              // 0  IPT_UNUSED
	};

	assign coin_in = ~{
		8'd0,             // 15:8 IPT_UNUSED
		joy0[CN],         // 7    COIN1 (COINL)
		joy1[CN],         // 6    COIN2 (COINR)
		6'd0              // 5:0 IPT_CUSTOM, left high for the memmap to xor
	};
endmodule
