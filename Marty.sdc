derive_pll_clocks
derive_clock_uncertainty

# core specific constraints


# The CPU (am386sx and the z386 inside it) advances only on ce_cpu, a
# 16 MHz enable derived from the 57.27 MHz system clock, so consecutive
# enables are at least three clocks apart. Register-to-register paths that
# both start and end inside the CPU therefore have three clocks of setup
# budget and two of hold. Paths that leave the CPU (bus pins into the
# mainboard, block RAM ports) keep the single-clock default.
set_multicycle_path -setup 3 -from [get_registers {emu|mainboard|cpu|*}] -to [get_registers {emu|mainboard|cpu|*}]
set_multicycle_path -hold  2 -from [get_registers {emu|mainboard|cpu|*}] -to [get_registers {emu|mainboard|cpu|*}]

# The SDRAM ports answer in the controller clock and the core reads the
# answer in clk_sys. The two come from one PLL with clk_sdram at twice
# clk_sys, so every clk_sys edge coincides with a controller edge, and the
# two clock networks arrive about a nanosecond apart - more than the ~0.4 ns
# a one-level answer path takes. The default same-edge hold check therefore
# cannot be met by data delay, and whether it passes comes down to placement
# (it held in builds 18-20 and 22-26 and failed in 21 and 27).
#
# towns_sdram_port holds its answer well past the edge that writes it: dout
# is written while pend is still set, pend clears on the next clk_sys edge,
# and ready only reaches the consumer the edge after that, so the data is
# stable for a full clk_sys period before anything captures it. Check hold
# one controller period back to match what the handshake guarantees.
#
# The port's ready also shows on the trailing acked_s, published a
# controller edge behind dout.
set_multicycle_path -hold 1 -from [get_registers {*towns_sdram_port:*|dout[*]}]
# The request side is the mirror image: sd_addr/sd_be/sd_we/sd_din are written
# with pend, sd_req rises a controller edge later, and the controller only
# captures them on the grant edge after that.
set_multicycle_path -hold 1 -from [get_registers {*towns_sdram_port:*|sd_din[*] *towns_sdram_port:*|sd_addr[*] *towns_sdram_port:*|sd_be[*] *towns_sdram_port:*|sd_we}]
# The queued CPU port keeps the same shape in both directions: the
# controller reads a queue slot only after seeing q_valid on two of its
# edges (q_valid_d), and clk_sys reads an answer on q_done_s, a controller
# edge behind q_data. Each flag trails its data by a full controller
# period, so hold is checked one period back.
set_multicycle_path -hold 1 -from [get_registers {*towns_sdram_qport:*|q_addr[*] *towns_sdram_qport:*|q_be[*] *towns_sdram_qport:*|q_we[*] *towns_sdram_qport:*|q_din[*]}]
set_multicycle_path -hold 1 -from [get_registers {*towns_sdram_qport:*|q_data[*]}]
