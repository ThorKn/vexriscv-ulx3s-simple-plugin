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
![Schematic](/images/wiring.jpg)

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

#### Build the ELF file from the C-Project and upload it to the Vexriscv:

Get into the `c_project` directory and build the ELF-file:
```
cd c_project
make
```
The ELF-file `simple_plugin.elf` got generated inside the `build` directory.

The next step is to connect the TF232H adapter to the Vexriscv via `openocd-vexriscv`:
```
openocd-vexriscv -f tf2323h_openocd.cfg
```
This should give you the console output of connecting openocd to the vexriscv. After connecting openocd should listen for a GDB connection on port 3333 (search for this in the console output).

After connecting openocd you must **open up a new console window** (the last one is idle now) and navigate to the `build` folder:
```
cd c_project/build
```
Now you can upload the ELF-file `simple_plugin.elf` to the vexriscv by using the Riscv-GDB:
```
riscv64-unknown-elf-gdb -tui -ex 'set remotetimeout 10' -ex 'target remote :3333' -ex load -ex 'break main' -ex 'break 43' simple_plugin.elf
```

Now the Vexriscv is running on the FPGA and the compiled c-code is loaded into the Vexriscv CPU. To see the results of the execution of the custom instruction, you must open up a serial connection to the Vexriscv. Therefore we'll use `GTKTerm`. Again you need to **open up a new console window** for that (as the last one is idle again). Then start `GTKTerm`:
```
sudo gtkterm
```

Configure `GTKTerm` for the fitting `ttyUSBX` Interface (mostly `ttyUSB0` or `ttyUSB1`) and set the protocol to `115200 - 8 - N - 1`. Additional you want to see the output as `ASCII`.

When you step through the c-program with using the GDB (`c` is for `continue`), you can see the printouts of the Vexriscv in `GTKTerm`.

That is it.

## Explanation:
 Todo
