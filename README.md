# vexriscv-ulx3s-simple-plugin
A simple custom instruction for the vexriscv, deploy- and debuggable on a ulx3s board.

## Description
You got yourself a ULX3S FPGA Board and want to try out
1. building a Vexriscv from SpinalHDL.
2. including a custom instruction for the Vexriscv.
3. generate the bitstream for the ULX3S.
4. upload the bitstream to the ULX3S.
5. build a C-Project to test the custom instruction.
6. upload the ELF-file via a JTAG-Adapter, using GDB.
7. seeing the UART output in a console.

That all is include in this Repository.

## Prerequisites
### You will need:
1. The neccessary tools are contained in this Virtual Machine (VM):

    [Virtual Machine](https://random-oracles.org/risc-v/)

    Download the VM and follow the documentation for starting it up.

2. An ULX3S FPGA Board (85k Version):

    [Radiona ULX3S](https://radiona.org/ulx3s/)

3. A JTAG Adapter (TF232H) from Adafruit:

    [Adafruit TF2323H](https://www.adafruit.com/product/2264)

### Wiring:
Wire the ULX3S FPGA Board and the TF232H together with 5 jumper cables:
![ULX3S wiring](/images/ulx3s.jpg)
![FT232H wiring](/images/ft232h.jpg)
![Schematic](/images/wiring.pdf)

## Usage
Your first step must be to start up the Virtual Machine.
After starting the VM, the rest of this tutorial happens completely inside the VM.
### Clone the Repository
Clone this Repositoy to a place of your choice inside the VM via:

`git clone https://github.com/ThorKn/vexriscv-ulx3s-simple-plugin.git`

### Structure of the repository
The repository contains three main folders:
```
|-- vexriscv
|-- ulx3s
|-- c_project
|-- images
```
`vexriscv` contains the spinalHDL code for generating the Vexriscv CPU as a single Verilog file, named `Murax.v`. The custom instruction (a Vexriscv plugin inside the pipeline) is also already contained in `Murax.v`.

`ulx3s` contains the `makefile` and the constraint file `ulx3s_v20_constraints.lpf` for building the bitstream out of the Verilog. The bitstream will be named `Murax.bit` and can directly be uploaded to the ULX3S Board.

`c_project` contains the whole C project to build the example ELF file `simple_plugin.elf` to interact with the custom instruction inside the Vexriscv. This ELF file can be uploaded to the Vexriscv (on the ULX3S Board) via Openocd and Riscv-GDB.

`images` contains the pictures for the readme.

### Build and upload it all
#### Build the Murax.v from spinalHDL with SBT
We'll start with the Vexriscv in spinalHDL and generate the Verilog from it:
```
cd vexriscv
sbt "runMain vexriscv.demo.Murax"
```
Then copy the Verilog file `Murax.v` to the directory `ulx3s` and leave the directory:
```
cp Murax.v ../ulx3s/
cd ..
```

#### Build the bitstream for the ULX3S and upload it:
Get into the `ulx3s` directory and build the bitstream:
```
cd ulx3s
make Murax.bit
```
The bitstream got generated as the file `Murax.bit`. This bitstream file can now be uploaded to the ULX3S FPGA board via:
```
ujprog Murax.bit
```
It might be, that you have to execute `ujprog` as `sudo`.

After programming the FPGA, leave the directory:
```
cd ..
```
