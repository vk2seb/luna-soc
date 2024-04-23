#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from luna                                        import configure_default_logging, top_level_cli
from luna.gateware.usb.usb2.device               import USBDevice

from luna_soc.gateware.cpu.vexriscv              import VexRiscv
from luna_soc.gateware.soc                       import LunaSoC
from luna_soc.gateware.csr                       import GpioPeripheral, LedPeripheral
from luna_soc.gateware.csr.hyperram              import HyperRAMPeripheral

from amaranth                                    import Elaboratable, Module, Cat, Instance, ClockSignal, ResetSignal, Signal
from amaranth.build                              import *
from amaranth.hdl.rec                            import Record

import logging
import os
import sys

CLOCK_FREQUENCIES_MHZ = {
    'sync': 60
}

def gpdi_from_pmod(platform, pmod_index):
    gpdi = [
        Resource(f"gpdi{pmod_index}", pmod_index,
            Subsignal("data2_p", Pins("1",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("data1_p", Pins("2",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("data0_p", Pins("3",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("clk_p",   Pins("4",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("data2_n", Pins("7",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("data1_n", Pins("8",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("data0_n", Pins("9",  conn=("pmod", pmod_index), dir='o')),
            Subsignal("clk_n",   Pins("10", conn=("pmod", pmod_index), dir='o')),
            Attrs(IO_TYPE="LVCMOS33"),
        )
    ]
    platform.add_resources(gpdi)
    return platform.request(f"gpdi{pmod_index}")

class LxVideo(Elaboratable):

    def __init__(self):
        super().__init__()
        pass

    def elaborate(self, platform) -> Module:
        m = Module()

        gpdi = gpdi_from_pmod(platform, 0)

        platform.add_file("lxvid.v", open("lxvid.v"))

        dma_stream_position = Signal(32)
        dma_stream_first = Signal()
        dma_stream_last  = Signal()
        dma_stream_ready  = Signal()

        with m.If(dma_stream_ready):
            with m.If(dma_stream_position != 720*720 - 1):
                m.d.sync += [
                    dma_stream_position.eq(dma_stream_position + 1)
                ]
            with m.Else():
                m.d.sync += [
                    dma_stream_position.eq(0)
                ]
            m.d.sync += [
                dma_stream_first.eq(dma_stream_position == 0),
                dma_stream_last.eq(dma_stream_position == 720*720-1),
            ]

        m.submodules.vlxvid = Instance("lxvid",
            i_clk_sys = ClockSignal("sync"),
            i_clk_hdmi = ClockSignal("hdmi"),
            i_clk_hdmi5x = ClockSignal("hdmi5x"),

            i_rst_sys = ResetSignal("sync"),
            i_rst_hdmi = ResetSignal("hdmi"),
            i_rst_hdmi5x = ResetSignal("hdmi5x"),

            i_dma_stream_valid = 1,
            i_dma_stream_data  = dma_stream_position,
            i_dma_stream_first = dma_stream_first,
            i_dma_stream_last  = dma_stream_last,
            i_dma_stream_reset  = dma_stream_first,
            o_dma_stream_ready  = dma_stream_ready,

            o_gpdi_clk_n = gpdi.clk_n.o,
            o_gpdi_clk_p = gpdi.clk_p.o,
            o_gpdi_data0_n = gpdi.data0_n.o,
            o_gpdi_data0_p = gpdi.data0_p.o,
            o_gpdi_data1_n = gpdi.data1_n.o,
            o_gpdi_data1_p = gpdi.data1_p.o,
            o_gpdi_data2_n = gpdi.data2_n.o,
            o_gpdi_data2_p = gpdi.data2_p.o,
        )

        return m

# - HelloSoc ------------------------------------------------------------------

class HelloSoc(Elaboratable):
    def __init__(self, clock_frequency):

        # create a stand-in for our UART
        self.uart_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])

        # create our SoC
        self.soc = LunaSoC(
            cpu=VexRiscv(reset_addr=0x00000000, variant="cynthion"),
            clock_frequency=clock_frequency,
            internal_sram_size=32768,
        )

        # ... add bios and core peripherals ...
        self.soc.add_bios_and_peripherals(uart_pins=self.uart_pins)

        # ... add our LED peripheral, for simple output ...
        self.leds = LedPeripheral()
        self.soc.add_peripheral(self.leds, addr=0xf0001000)

        # ... add memory-mapped hyperram peripheral (128Mbit)
        self.soc.hyperram = HyperRAMPeripheral(size=16*1024*1024)
        self.soc.add_peripheral(self.soc.hyperram, addr=0x20000000)

    def elaborate(self, platform):
        m = Module()
        m.submodules.soc = self.soc

        # generate our domain clocks/resets
        m.submodules.car = platform.clock_domain_generator(clock_frequencies=CLOCK_FREQUENCIES_MHZ)

        # connect up our UART
        uart_io = platform.request("uart", 0)
        m.d.comb += [
            uart_io.tx.o.eq(self.uart_pins.tx),
            self.uart_pins.rx.eq(uart_io.rx)
        ]
        if hasattr(uart_io.tx, 'oe'):
            m.d.comb += uart_io.tx.oe.eq(~self.soc.uart._phy.tx.rdy),

        # video
        m.submodules.video = LxVideo()

        return m


# - main ----------------------------------------------------------------------

from luna.gateware.platform import get_appropriate_platform

from luna_soc.generate      import Generate, Introspect

if __name__ == "__main__":
    # disable UnusedElaborable warnings
    from amaranth._unused import MustUse
    MustUse._MustUse__silence = True

    build_dir = os.path.join("build")

    # configure logging
    configure_default_logging()
    logging.getLogger().setLevel(logging.DEBUG)

    # select platform
    platform = get_appropriate_platform()
    if platform is None:
        logging.error("Failed to identify a supported platform")
        sys.exit(1)

    # configure clock frequency
    clock_frequency = int(platform.default_clk_frequency)
    logging.info(f"Platform clock frequency: {clock_frequency}")

    # create design
    design = HelloSoc(clock_frequency=clock_frequency)

    # TODO fix litex build
    thirdparty = os.path.join(build_dir, "lambdasoc.soc.cpu/bios/3rdparty/litex")
    if not os.path.exists(thirdparty):
        logging.info("Fixing build, creating output directory: {}".format(thirdparty))
        os.makedirs(thirdparty)

    # build litex bios
    logging.info("Building bios")
    design.soc.build(name="soc",
                     build_dir=build_dir,
                     do_init=True)

    # build soc
    logging.info("Building soc")
    overrides = {
        "debug_verilog": True,
        "verbose": False,
        "nextpnr_opts": "--timing-allow-fail"
    }
    products = platform.build(design, do_program=False, build_dir=build_dir, **overrides)

    # log resources and prepare to generate artifacts
    Introspect(design.soc).log_resources()
    generate = Generate(design.soc)

    # generate: svd file
    path = os.path.join(build_dir, "gensvd")
    if not os.path.exists(path):
        os.makedirs(path)

    logging.info("Generating svd file: {}".format(path))
    with open(os.path.join(path, "lunasoc.svd"), "w") as f:
        generate.svd(file=f)

    # generate: rust memory.x file
    path = os.path.join(build_dir, "genrust")
    if not os.path.exists(path):
        os.makedirs(path)

    logging.info("Generating memory.x file: {}".format(path))
    with open(os.path.join(path, "memory.x"), "w") as f:
        generate.memory_x(file=f)

    print("Build completed. Use 'make load' to load bitstream to device.")

    # TODO
    #top_level_cli(design)
