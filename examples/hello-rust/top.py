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

from amaranth_soc            import wishbone

from luna.gateware.debug.ila import AsyncSerialILA

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

from amaranth.lib.fifo import AsyncFIFO

class LxVideo(Elaboratable):

    def __init__(self, fb_base=None, fifo_depth=64):
        super().__init__()

        self.bus = wishbone.Interface(addr_width=30, data_width=32, granularity=8,
                                      features={"cti", "bte", "err"})

        self.fifo = AsyncFIFO(width=32, depth=fifo_depth, r_domain='hdmi', w_domain='sync')

        self.fifo_depth = fifo_depth
        self.fb_base = fb_base
        self.fb_hsize = 720
        self.fb_vsize = 720

    def elaborate(self, platform) -> Module:
        m = Module()

        m.submodules.fifo = self.fifo

        gpdi = gpdi_from_pmod(platform, 0)

        platform.add_file("build/lxvid.v", open("lxvid.v"))

        vtg_hcount = Signal(12)
        vtg_vcount = Signal(12)

        phy_r = Signal(8)
        phy_g = Signal(8)
        phy_b = Signal(8)

        m.submodules.vlxvid = Instance("lxvid",
            i_clk_sys = ClockSignal("sync"),
            i_clk_hdmi = ClockSignal("hdmi"),
            i_clk_hdmi5x = ClockSignal("hdmi5x"),

            i_rst_sys = ResetSignal("sync"),
            i_rst_hdmi = ResetSignal("hdmi"),
            i_rst_hdmi5x = ResetSignal("hdmi5x"),

            o_gpdi_clk_n = gpdi.clk_n.o,
            o_gpdi_clk_p = gpdi.clk_p.o,
            o_gpdi_data0_n = gpdi.data0_n.o,
            o_gpdi_data0_p = gpdi.data0_p.o,
            o_gpdi_data1_n = gpdi.data1_n.o,
            o_gpdi_data1_p = gpdi.data1_p.o,
            o_gpdi_data2_n = gpdi.data2_n.o,
            o_gpdi_data2_p = gpdi.data2_p.o,

            o_vtg_hcount = vtg_hcount,
            o_vtg_vcount = vtg_vcount,

            i_phy_r = phy_r,
            i_phy_g = phy_g,
            i_phy_b = phy_b,
        )

        # how?

        # 2 separate state machines, one in each sys / hdmi clk domain?

        # START
        # - load fifos
        # - wait for vtg 0, 0 (or 1 clock before)
        # THEN
        # - drain fifo on every clock
        # - burst read hyperram on every level = depth/2
        # - make sure correct burst size wraps correctly

        bus = self.bus

        dma_addr = Signal(32)

        fb_len_words = (self.fb_hsize*self.fb_vsize) // 4

        """
        # bus -> FIFO
        # todo bursts
        m.d.comb += [
            bus.stb.eq(self.fifo.w_rdy),
            bus.cyc.eq(self.fifo.w_rdy),
            bus.we.eq(0),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(self.fb_base + dma_addr),
            self.fifo.w_data.eq(bus.dat_r),
        ]
        with m.If(bus.stb & bus.ack):
            m.d.comb += self.fifo.w_en.eq(1)
        """

        # bus -> FIFO
        # burst until FIFO is full, then wait until half empty.

        # sync domain
        with m.FSM() as fsm:
            with m.State('BURST'):
                with m.If(self.fifo.w_rdy):
                    m.d.comb += [
                        bus.stb.eq(1),
                        bus.cyc.eq(1),
                        bus.we.eq(0),
                        bus.sel.eq(2**(bus.data_width//8)-1),
                        bus.adr.eq(self.fb_base + dma_addr),
                        self.fifo.w_data.eq(bus.dat_r),
                    ]
                    with m.If(self.fifo.w_level == self.fifo_depth-1):
                        m.d.comb += bus.cti.eq(
                                wishbone.CycleType.END_OF_BURST)
                    with m.Else():
                        m.d.comb += bus.cti.eq(
                                wishbone.CycleType.INCR_BURST)
                    with m.If(bus.stb & bus.ack):
                        m.d.comb += self.fifo.w_en.eq(1)
                        with m.If(dma_addr < (fb_len_words-1) << 2):
                            m.d.sync += dma_addr.eq(dma_addr + 4)
                        with m.Else():
                            m.d.sync += dma_addr.eq(0)
                with m.Else():
                    # FIFO full, hold off for next burst.
                    m.next = 'WAIT'
            with m.State('WAIT'):
                with m.If(self.fifo.w_level < self.fifo_depth//2):
                    m.next = 'BURST'

        # FIFO -> PHY (1 word -> 4 pixels)

        bytecounter = Signal(2)
        last_word   = Signal(32)
        consume_started = Signal(1, reset=0)

        with m.If((vtg_hcount == 0) & (vtg_vcount == 0) &
                  self.fifo.r_level > self.fifo_depth//2):
            m.d.hdmi += consume_started.eq(1)

        with m.If(consume_started):
            m.d.hdmi += bytecounter.eq(bytecounter+1)

        with m.If(bytecounter == 0):
            m.d.hdmi += last_word.eq(self.fifo.r_data)
        with m.Else():
            m.d.hdmi += last_word.eq(last_word >> 8)

        m.d.comb += [
            self.fifo.r_en.eq(bytecounter == 3),
            phy_r.eq(last_word[0:7]),
            phy_g.eq(last_word[0:7]),
            phy_b.eq(last_word[0:7]),
        ]

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
        hyperram_base = 0x20000000
        self.soc.hyperram = HyperRAMPeripheral(size=16*1024*1024)
        self.soc.add_peripheral(self.soc.hyperram, addr=hyperram_base)

        # ... add memory-mapped framebuffer output that drives hyperram
        self.video = LxVideo(fb_base=hyperram_base)
        self.soc._bus_arbiter.add(self.video.bus)

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
        m.submodules.video = self.video

        # ila
        test_signal = Signal(32, reset=0xDEADBEEF)

        ila_signals = [
            test_signal,

            self.video.bus.cyc,
            self.video.bus.stb,
            self.video.bus.adr,
            self.video.bus.we,
            self.video.bus.dat_r,
            self.video.bus.dat_w,
            self.video.bus.ack,
            self.video.bus.cti,

            self.video.fifo.w_level,

            self.soc.hyperram.psram.address,
            self.soc.hyperram.psram.register_space,
            self.soc.hyperram.psram.perform_write,
            self.soc.hyperram.psram.single_page,
            self.soc.hyperram.psram.start_transfer,

            self.soc.hyperram.psram.final_word,
            self.soc.hyperram.psram.read_data,
            self.soc.hyperram.psram.write_data,

            self.soc.hyperram.psram.idle,
            self.soc.hyperram.psram.read_ready,
            self.soc.hyperram.psram.write_ready,

            self.soc.hyperram.psram.fsm_state,
        ]

        for s in ila_signals:
            print(s)

        self.ila = AsyncSerialILA(signals=ila_signals,
                                  sample_depth=4096, divisor=60,
                                  domain='sync', sample_rate=60e6) # 1MBaud on USB clock

        pmod_uart = [
            Resource("pmod_uart", 0,
                Subsignal("tx",  Pins("1", conn=("pmod", 1), dir='o')),
                Subsignal("rx",  Pins("2", conn=("pmod", 1), dir='i')),
                Attrs(IO_TYPE="LVCMOS33"),
            )
        ]

        platform.add_resources(pmod_uart)

        m.d.comb += [
            self.ila.trigger.eq(ClockSignal("sync")),
            platform.request("pmod_uart").tx.o.eq(self.ila.tx),
        ]

        m.submodules.ila = self.ila

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

    print("waiting for ILA")

    from luna.gateware.debug.ila import AsyncSerialILAFrontend
    frontend = AsyncSerialILAFrontend("/dev/ttyUSB1", baudrate=1000000, ila=design.ila)
    frontend.emit_vcd("out.vcd")

    # TODO
    #top_level_cli(design)
