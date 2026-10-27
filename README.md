# AWG-MultiChannel-RFSoC

[![Platform](https://img.shields.io/badge/Platform-RealDigital%20RFSoC%204x2-red)]([https://redpitaya.com/](https://www.realdigital.org/hardware/rfsoc-4x2))
[![FPGA](https://img.shields.io/badge/FPGA-ZYNQ%20Ultrascale+%20RFSoC%20ZU48DR-blue)]([https://www.xilinx.com/](https://www.amd.com/en/products/adaptive-socs-and-fpgas/soc/zynq-ultrascale-plus-rfsoc.html))


## Overview
High-sampling-rate, high-memory-density, multi-channel arbitrary waveform generator. Waveforms are transferred from PS to PL via AXI DMA, stored in RAM, and streamed to the on-chip DACs.

## Features

- Data sampling at 9.8304 GSps
- Two output channels: DAC A (DAC0) and DAC B (DAC2)
- Four waveforms total, each $2^{18}$ samples
  - DAC A (DAC0): waveform A or B
  - DAC B (DAC2): waveform C or D
  - Waveform selection via software (AXI GPIO) or an external signal.
- Sample loading within 5-7 ms.


## System Architecture
The implementation is summarized with the help of the following diagrams. It is separated into the waveform-write and the waveform-read blocks.

### Waveform write block
The sample comprising the waveforms A,B,C, and D are fed into the PL via an AXI-DMA with samples in the sequence shown in the diagram. The stream then passes through a Data Sorter module, which reorders the incoming samples and writes them into the RAM banks using the memory layout depicted in the diagram. During this process, the `write_enable` signal is held high. 
For the memory block RAM[0] and RAM[1], the first $2^{14}$ memory address contain waveform-A samples, and the rest $2^{14}$ address (from $2^{14}$ to $2^{15}$) contain waveform-B samples. A similar arrangement is used for waveform-C and -D on RAM blocks RAM[2], and RAM[3].

### Waveform read block
On the read side, the address counter increments with a clock signal clk0 for RAM[0],and RAM[1]) and with clk2 for RAM[2], and RAM[3]). Both clocks operate at 614.4 MHz -  16x slower than the DAC sampling rate. 
At each rising edge of clk0, the 16 samples - 8 samples on RAM[0], and 8 samples on RAM[1] at the same address counter - are concatenated and sent to the DAC IP for output on DAC0 (DAC A). When the waveform select signal : (`control_trigger` || `axi_control`) is high, an offset of $2^{14}$ is added to the base address, so that waveform-B is output on the same dac channel. A similar behavior is implemented to output waveform-C or waveform-D at DAC2.



<table cellspacing="0" cellpadding="10">
  <tr>
    <th align="left">Input</th>
    <th align="left">Output</th>
  </tr>
  <tr>
    <td><img src="./assets/arb_RFSoC_input_bkg.png" width="500" alt="Write"></td>
    <td><img src="./assets/arb_RFSoC_output_bkg.png" width="500" alt="Read"></td>
  </tr>
</table>
Internally, the design is pipelined to meet timing requirements for performance of the FPGA logic at an incredible clock rate of 614.4 MHz. There is no particular reason, for such arrangement of the memory blocks, other than historical. This design is an extension of earlier design of an AWG that outputs only one waveform over one DAC channel.

## Input/Output 
The relevant I/O pins and ports on the board are
| Name | Pin/Port |  Type | Description |
| -------- | -------- | -------- | -------- |
| control_trigger | PMOD0_1 | Input | External waveform-select trigger. Toggles DAC A between waveform-A/waveform-B and DAC B between waveform-C/waveform-D. Effective select: (axi_control || control_trigger)|
| pmod_out| PMOD0_0  | Output | Outputs control_trigger signal sampled on a 614.4 MHz clock.  |
| DAC 0| DAC A  | Output | AWG output |
| DAC 2| DAC B  | Output | AWG output |

The design contains AXI GPIOs that Python (`python/rfsoc_awg.py`) drives as software-visible registers.
| Name | Memory Address |  Type | Description |
| -------- | -------- | -------- | -------- |
| rst_n | 0x00_A007_0000 | Output |1 bit. Software reset. Active low signal. |
| write_enable | 0x00_A005_0000  | Output | 1 bit. Enables writing waveform to RAM bank. Set to 0 to enable output; set enable_ch0 and enambe_ch2 accordingly. |
| MAX_POINTS| 0x00_A006_0000  | Output | 32 bit. (MAX_POINTS<<4) is the maximum number of points to output. |
| enable_ch0 | 0x00_A009_0000 | Output | 1 bit. Enables output on DAC A when write_enable is low. |
| enable_ch2 | 0x00_A00A_0000 | Output | 1 bit. Enables output on DAC B when write_enable is low. |
| axi_control| 0x00_A008_0000 | Output | 1 bit. Software trigger signal.  Switches between waveform-A and waveform-B for DAC0 output and between waveform-C and waveform-D for DAC2 output. The effective select signal is (axi_control || control_trigger) |
| axi_dma_0| 0x00_A004_0000 | Input | AMD's DMA IP for direct memory access and fast transfer of data from PS to PL.|


## Repository layout
- `fpga/` — RTL (`src/` AWG, `adc/` ADC capture), constraints, block-design Tcl, `awg_multichannel_rfsoc.tcl` (the single build script), `bitstreams/`, and `legacy/` (unused older attempts).
- `fpga/bitstreams/` — `awg_adc.bit` with its `.hwh`, and `awg_clock_syncd.7z` with its `.hwh`. Extract the archive to get `awg_clock_syncd.bit`.
- `python/` — current dashboard (`rfsoc_dashboard.py`, `AWG_dashboard.ipynb`) and the modules it imports (`rfsoc_awg.py`, `adc_scope.py`, `bode_sweep.py`, `bode.ipynb`), plus the LMK and LMX register files. `xrfclk.set_ref_clks` looks for those register files in the working directory.
- `python/dac_adc/` — earlier DAC+ADC scripts, including their own `rfsoc_awg.py`. `AWG_notebook.ipynb` in that folder loads `awg_adc.bit`.
- `python/awg_only/` — original AWG-only scripts. `AWG_notebook.ipynb` in that folder loads `awg_clock_syncd.bit`.
- `assets/` — architecture diagrams.

`fpga/awg_multichannel_rfsoc.tcl` is the single build script (see Modifications). It sources `fpga/bd/block_design.tcl` (Vivado 2024.1 export of the AWG block design), optionally adds the ADC capture path with `fpga/tcl/add_adc.tcl` (RTL in `fpga/adc/hdl/adc_core.v`), and uses `fpga/constrs/awg_top.xdc`. `fpga/legacy/` holds the earlier DMA-based ADC capture attempt (`add_adc_capture.tcl`, `adc_capture.xdc`, `packet_generator_1.0`), older block-design exports and the original constraints. Nothing in it is used by the build.

## Usage
1. On the board, put in one folder: the notebook you want from `python/`, `python/dac_adc/`, or `python/awg_only/`; `python/LMK04828_245.76.txt` and `python/LMX2594_491.52.txt`; and the matching `.bit` and `.hwh` from `fpga/bitstreams/`. For the original design, extract `awg_clock_syncd.7z` to obtain the `.bit`.
2. For the dashboard, copy `rfsoc_dashboard.py` and `rfsoc_awg.py` into that same folder. Packages beyond the PYNQ image are listed in `requirements.txt`.
3. Run the notebook from the Linux OS on the FPGA.


## Modifications
Requires Vivado 2024.1. One script builds everything (project, block design, ADC capture, bitstream, ~50 min) and copies `awg_adc_v3.bit/.hwh` into `fpga/bitstreams/`:

```
# Vivado Tcl console
source <repo>/fpga/awg_multichannel_rfsoc.tcl
# or batch
vivado -mode batch -source fpga/awg_multichannel_rfsoc.tcl -tclargs [--no-adc] [--no-build] [--jobs N] [--proj-dir DIR]
```
`--no-adc` builds the AWG only, `--no-build` stops after the block design and wrapper are made, `--proj-dir` puts the Vivado project in another (shorter) folder if Windows paths get too long. In the console, set `AWG_ADC`, `AWG_BUILD`, `AWG_JOBS` or `AWG_PROJ_DIR` before sourcing instead.
The board file can be installed from
https://github.com/RealDigitalOrg/RFSoC4x2-BSP
