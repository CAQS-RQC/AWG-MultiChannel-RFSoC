packet_generator_1.0 is copied unchanged from
https://github.com/Xilinx/RFSoC4x2-PYNQ  (boards/ip/packet_generator_1.0)
Copyright (c) 2021 Xilinx, Inc. - BSD 3-Clause (licence text is in each VHDL file header).
It turns the free-running ADC AXI-Stream into a single TLAST-terminated packet on request
(packetsize at offset 0x00, transfer at 0x04) so an AXI DMA S2MM channel can store it.
