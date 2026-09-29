// PLA Control - instruction layout pattern matching
// Generated from doc/microcode/microcode_rom1_260103.txt
//
// Input format:
//   state[6:0]       - bits mnopqrs
//   instruction[7:0] - bits 76543210
//   mode[4:0]        - bits tuvwx
//
// Output format:
//   [11:0] - bits abcdefghijkl
//
// Reduced active frontend view.
// Source data: doc/microcode/microcode_rom1_260103.txt, filtered to the
// initial opcode/prefix state used by 21.z386/22.z386_MiSTer:
//   state = 7'b0000000
//   mode  = {has_0f, 4'b0000}
// The full ROM1 table (all states and modes) is in the upstream mirror's
// copy of this file; only the opcode view is synthesized.
function automatic logic [11:0] pla_control_opcode_lookup(
    input        has_0f,
    input [7:0] instruction
);
    // Priority order as generated: the hardware captures (PUSHF/POPF push a
    // word) follow the first matching term, not the OR of all matches. The
    // three die terms an earlier wider term fully covers are listed at the
    // end as comments instead of unreachable case items.
    casez ({has_0f, instruction})
        // has_0f 76543210 abcdefghijkl
        9'b0_10100010: pla_control_opcode_lookup = 12'b100110000111;
        9'b1_10100011: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_10101?11: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_1001?00?: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_1001?100: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_1001?1?1: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_1001??10: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_10?1?011: pla_control_opcode_lookup = 12'b101100010100;
        9'b1_1010?101: pla_control_opcode_lookup = 12'b101100010100;
        9'b0_0110001?: pla_control_opcode_lookup = 12'b101100000100;
        9'b0_100011??: pla_control_opcode_lookup = 12'b101100000100;
        9'b1_1010011?: pla_control_opcode_lookup = 12'b101100000100;
        9'b1_10110010: pla_control_opcode_lookup = 12'b101100000100;
        9'b1_1011?10?: pla_control_opcode_lookup = 12'b101100000100;
        9'b0_1100010?: pla_control_opcode_lookup = 12'b101100000100;
        9'b0_00???0??: pla_control_opcode_lookup = 12'b101100010110;
        9'b0_100001??: pla_control_opcode_lookup = 12'b101100010110;
        9'b1_1011?11?: pla_control_opcode_lookup = 12'b101100010110;
        9'b1_000100??: pla_control_opcode_lookup = 12'b101100000110;
        9'b0_100010??: pla_control_opcode_lookup = 12'b101100000110;
        9'b1_001000?1: pla_control_opcode_lookup = 12'b101101000100;
        9'b1_00100??0: pla_control_opcode_lookup = 12'b101101000100;
        9'b1_10100100: pla_control_opcode_lookup = 12'b001100010100;
        9'b1_10101100: pla_control_opcode_lookup = 12'b001100010100;
        9'b0_01101001: pla_control_opcode_lookup = 12'b011100010100;
        9'b0_100000?0: pla_control_opcode_lookup = 12'b001100000110;
        9'b0_10000001: pla_control_opcode_lookup = 12'b011100000110;
        9'b0_10000011: pla_control_opcode_lookup = 12'b111100000110;
        9'b0_1100000?: pla_control_opcode_lookup = 12'b001100000110;
        9'b0_110100??: pla_control_opcode_lookup = 12'b101000000110;
        9'b0_11110110: pla_control_opcode_lookup = 12'b110000000110;
        9'b0_11110111: pla_control_opcode_lookup = 12'b010000000110;
        9'b0_1111111?: pla_control_opcode_lookup = 12'b101000000110;
        9'b1_000000??: pla_control_opcode_lookup = 12'b101100000100;
        9'b1_10111010: pla_control_opcode_lookup = 12'b001100000100;
        9'b0_11000110: pla_control_opcode_lookup = 12'b001100000110;
        9'b0_11000111: pla_control_opcode_lookup = 12'b011100000110;
        9'b0_00???100: pla_control_opcode_lookup = 12'b100111010111;
        9'b0_00???101: pla_control_opcode_lookup = 12'b110111010111;
        9'b0_00001111: pla_control_opcode_lookup = 12'b000000000100;
        9'b0_001??110: pla_control_opcode_lookup = 12'b000000000100;
        9'b0_011001??: pla_control_opcode_lookup = 12'b000000000100;
        9'b0_11110000: pla_control_opcode_lookup = 12'b000000000100;
        9'b0_1111001?: pla_control_opcode_lookup = 12'b000000000100;
        9'b0_01101010: pla_control_opcode_lookup = 12'b000111000101;
        9'b0_1100?010: pla_control_opcode_lookup = 12'b010111100101;
        9'b1_1000????: pla_control_opcode_lookup = 12'b011111010101;
        9'b0_1110100?: pla_control_opcode_lookup = 12'b011111100101;
        9'b0_10010???: pla_control_opcode_lookup = 12'b100000000001;
        9'b1_0000010?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_000010?0: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00011000: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_000?1001: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_001001?1: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_0010100?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_001100??: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00?10100: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00?101?1: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00?11010: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00?1?110: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00??110?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_00??1?11: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_01000000: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_0100?001: pla_control_opcode_lookup = 12'b100000000101;
        9'b0_0110000?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_0?001110: pla_control_opcode_lookup = 12'b100000000101;
        9'b0_1001110?: pla_control_opcode_lookup = 12'b100000000101;
        9'b0_10011110: pla_control_opcode_lookup = 12'b100000000101;
        9'b0_10011?11: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_10100010: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_1011000?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_1100000?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?100?010: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1010000: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?101?001: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?10?1000: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?11000?0: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1101000: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?110?001: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?111000?: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1?001?0: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1?10100: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1?1??10: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1??1100: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1???101: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_?1????11: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_??101?10: pla_control_opcode_lookup = 12'b100000000101;
        9'b1_??11100?: pla_control_opcode_lookup = 12'b100000000101;
        9'b0_1010101?: pla_control_opcode_lookup = 12'b100000000111;
        9'b0_1010?1??: pla_control_opcode_lookup = 12'b100000000111;
        9'b0_1101011?: pla_control_opcode_lookup = 12'b100000000111;
        9'b0_?11011??: pla_control_opcode_lookup = 12'b100000000111;
        9'b0_0000011?: pla_control_opcode_lookup = 12'b100000001101;
        9'b0_00010110: pla_control_opcode_lookup = 12'b100000001101;
        9'b0_0001?111: pla_control_opcode_lookup = 12'b100000001101;
        9'b0_000?1110: pla_control_opcode_lookup = 12'b100000001101;
        9'b1_1010?00?: pla_control_opcode_lookup = 12'b100000001101;
        9'b0_010?????: pla_control_opcode_lookup = 12'b100000010001;
        9'b1_00000110: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_00100111: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_00101111: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_00110111: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_00111111: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111000: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111001: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111010: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111011: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11110100: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11110101: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111100: pla_control_opcode_lookup = 12'b100000010101;
        9'b0_11111101: pla_control_opcode_lookup = 12'b100000010101;
        9'b1_00000111: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11000011: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11001100: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11001011: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11001111: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11110001: pla_control_opcode_lookup = 12'b100000100101;
        9'b0_11100011: pla_control_opcode_lookup = 12'b100010000101;
        9'b0_0111????: pla_control_opcode_lookup = 12'b100010010101;
        9'b0_11100000: pla_control_opcode_lookup = 12'b100010100101;
        9'b0_11100001: pla_control_opcode_lookup = 12'b100010100101;
        9'b0_11100010: pla_control_opcode_lookup = 12'b100010100101;
        9'b0_11101011: pla_control_opcode_lookup = 12'b100010100101;
        9'b0_101000??: pla_control_opcode_lookup = 12'b100110000111;
        9'b0_10110???: pla_control_opcode_lookup = 12'b100111000011;
        9'b0_111001??: pla_control_opcode_lookup = 12'b100111000111;
        9'b0_1101010?: pla_control_opcode_lookup = 12'b100111010101;
        9'b0_10101000: pla_control_opcode_lookup = 12'b100111010111;
        9'b0_11001101: pla_control_opcode_lookup = 12'b100111100101;
        9'b0_11011???: pla_control_opcode_lookup = 12'b101011000100;
        9'b0_11001000: pla_control_opcode_lookup = 12'b101111000101;
        9'b0_10111???: pla_control_opcode_lookup = 12'b110111000011;
        9'b0_01101000: pla_control_opcode_lookup = 12'b110111000101;
        9'b0_10101001: pla_control_opcode_lookup = 12'b110111010111;
        9'b0_01101011: pla_control_opcode_lookup = 12'b111100010100;
        9'b0_10011010: pla_control_opcode_lookup = 12'b111111100101;
        9'b0_11101010: pla_control_opcode_lookup = 12'b111111100101;
        default: pla_control_opcode_lookup = 12'b0;
    endcase
        // shadowed: 9'b0_10011100 -> 12'b100000010011
        // shadowed: 9'b0_10011101 -> 12'b100000010011
        // shadowed: 9'b0_00???101 -> 12'b110111010111
endfunction
