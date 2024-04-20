#!/usr/bin/env python3
#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth                      import Cat, ClockSignal, Elaboratable, Module, ResetSignal, Signal
from amaranth.hdl.rec              import Record

from amaranth_stdio.serial         import AsyncSerial

from lambdasoc.cpu.minerva         import MinervaCPU
from lambdasoc.periph              import Peripheral
from lambdasoc.periph.serial       import AsyncSerialPeripheral
from lambdasoc.periph.timer        import TimerPeripheral

from luna                          import configure_default_logging
#from luna.gateware.interface.psram import HyperRAMInterface, HyperRAMPHY
from hyper import HyperRAMX2, HyperRAMX2Tuner
from luna.gateware.interface.ulpi  import ULPIRegisterWindow
from luna.gateware.usb.usb2.device import USBDevice

from luna_soc.gateware.soc         import LunaSoC
from luna_soc.gateware.csr         import GpioPeripheral, LedPeripheral, UARTPeripheral

from luna_soc.util.readbin         import get_mem_data

from luna.gateware.debug.ila import AsyncSerialILA

from amaranth.build import *

import tiliqua

import logging
import os
import sys

# Run our tests at a slower clock rate, for now.
# TODO: bump up the fast clock rate, to test the HyperRAM at speed?
CLOCK_FREQUENCIES_MHZ = {
    "sync":  60,
    "usb":   60,
    "fast": 120,
}


# - ULPIRegisterPeripheral ----------------------------------------------------

class ULPIRegisterPeripheral(Peripheral, Elaboratable):
    """ Peripheral that provides access to a ULPI PHY, and its registers. """

    def __init__(self, name="ulpi", io_resource_name="usb"):
        super().__init__(name=name)
        self._io_resource = io_resource_name

        # Create our registers...
        bank            = self.csr_bank()
        self._address   = bank.csr(8, "w")
        self._value     = bank.csr(8, "rw")
        self._busy      = bank.csr(1, "r")

        # ... and convert our register into a Wishbone peripheral.
        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus


    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        # Grab a connection to our ULPI PHY.
        target_ulpi = platform.request(self._io_resource)

        #
        # ULPI Register Window
        #
        ulpi_reg_window  = ULPIRegisterWindow()
        m.submodules  += ulpi_reg_window

        # Connect up the window.
        m.d.comb += [
            ulpi_reg_window.ulpi_data_in  .eq(target_ulpi.data.i),
            ulpi_reg_window.ulpi_dir      .eq(target_ulpi.dir.i),
            ulpi_reg_window.ulpi_next     .eq(target_ulpi.nxt.i),

            target_ulpi.clk.o             .eq(ClockSignal("usb")),
            target_ulpi.rst.o             .eq(ResetSignal("usb")),
            target_ulpi.stp.o             .eq(ulpi_reg_window.ulpi_stop),
            target_ulpi.data.o            .eq(ulpi_reg_window.ulpi_data_out),
            target_ulpi.data.oe           .eq(~target_ulpi.dir.i)
        ]

        #
        # Address register logic.
        #

        # Perform a read request whenever the user writes to ULPI address...
        m.d.sync += ulpi_reg_window.read_request.eq(self._address.w_stb)

        # And update the register address accordingly.
        with m.If(self._address.w_stb):
            m.d.sync += ulpi_reg_window.address.eq(self._address.w_data)


        #
        # Value register logic.
        #

        # Always report back the last read data.
        m.d.comb += self._value.r_data.eq(ulpi_reg_window.read_data)

        # Perform a write whenever the user writes to our ULPI value.
        m.d.sync += ulpi_reg_window.write_request.eq(self._value.w_stb)
        with m.If(self._address.w_stb):
            m.d.sync += ulpi_reg_window.write_data.eq(self._value.w_data)


        #
        # Busy register logic.
        #
        m.d.comb += self._busy.r_data.eq(ulpi_reg_window.busy)

        return m

class CRGTuner(Peripheral, Elaboratable):

    def __init__(self):
        super().__init__()

        # peripheral control registers
        regs                = self.csr_bank()
        self._slip_hr2x     = regs.csr(1, "w")
        self._slip_hr2x90   = regs.csr(1, "w")
        self._phase_sel     = regs.csr(2, "w")
        self._phase_dir     = regs.csr(1, "w")
        self._phase_step    = regs.csr(1, "w")
        self._phase_load    = regs.csr(1, "w")

        # peripheral bus
        self._bridge = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus     = self._bridge.bus

    def set_crg(self, crg_instance):
        self.crg = crg_instance

    def elaborate(self, platform):
        m = Module()

        with m.If(self._slip_hr2x.w_stb):
            m.d.sync += self.crg.slip_hr2x.eq(self._slip_hr2x.w_data)

        with m.If(self._slip_hr2x90.w_stb):
            m.d.sync += self.crg.slip_hr2x90.eq(self._slip_hr2x90.w_data)

        with m.If(self._phase_sel.w_stb):
            m.d.sync += self.crg.phase_sel.eq(self._phase_sel.w_data)

        with m.If(self._phase_dir.w_stb):
            m.d.sync += self.crg.phase_dir.eq(self._phase_dir.w_data)

        with m.If(self._phase_step.w_stb):
            m.d.sync += self.crg.phase_step.eq(self._phase_step.w_data)

        with m.If(self._phase_load.w_stb):
            m.d.sync += self.crg.phase_load.eq(self._phase_load.w_data)

        m.submodules.bridge = self._bridge

        return m



# - SelftestCore --------------------------------------------------------------

# TODO delete
from lambdasoc.periph.sram   import SRAMPeripheral

class SelftestCore(Elaboratable):
    def __init__(self):
        clock_frequency = int(CLOCK_FREQUENCIES_MHZ["sync"] * 1e6)

        # create our SoC
        self.soc = LunaSoC(
            cpu=MinervaCPU(with_muldiv=True, with_debug=False),
            clock_frequency=clock_frequency,
        )

        # ... read our firmware binary ...
        firmware = get_mem_data("selftest.bin", data_width=32, endianness="little")

        # ... add a ROM for firmware ...
        self.soc.bootrom = SRAMPeripheral(size=0x4000, writable=False, init=firmware)
        self.soc.add_peripheral(self.soc.bootrom, addr=0x00000000, as_submodule=False)

        # ... and a RAM for execution ...
        self.soc.scratchpad = SRAMPeripheral(size=0x4000)
        self.soc.add_peripheral(self.soc.scratchpad, addr=0x00004000, as_submodule=False)

        # ... add a timer, so our software can get precise timing ...
        self.soc.timer = TimerPeripheral(width=32)
        self.soc.add_peripheral(self.soc.timer, as_submodule=False)

        # ... add our UART peripheral ...
        self.uart_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])
        self.soc.uart = AsyncSerialPeripheral(core=AsyncSerial(
            data_bits = 8,
            divisor   = int(clock_frequency // 115200),
            pins      = self.uart_pins,
        ))
        self.soc.add_peripheral(self.soc.uart, as_submodule=False)

        # ... and add our peripherals under test.
        peripherals = [
            LedPeripheral(name="leds"),
            ULPIRegisterPeripheral(name="target_ulpi",   io_resource_name="target_phy"),
        ]

        hram = HyperRAMX2(name="hyperramx2")
        hyperram_tune = HyperRAMX2Tuner(hyperram_instance=hram)
        self.hram = hram
        self.hyperram_tune = hyperram_tune

        self.crg_tune = CRGTuner()

        peripherals += [hram, hyperram_tune, self.crg_tune]

        for peripheral in peripherals:
            self.soc.add_peripheral(peripheral)


    def elaborate(self, platform):
        m = Module()

        # generate our domain clocks/resets
        m.submodules.car = platform.clock_domain_generator(clock_frequencies=CLOCK_FREQUENCIES_MHZ)

        self.crg_tune.set_crg(m.submodules.car)

        # add our SoC to the design
        m.submodules.soc = self.soc

        # connect up our UART
        uart_io = platform.request("uart", 0)
        m.d.comb += [
            uart_io.tx.o.eq(self.uart_pins.tx),
            self.uart_pins.rx.eq(uart_io.rx)
        ]
        if hasattr(uart_io.tx, 'oe'):
            m.d.comb += uart_io.tx.oe.eq(~self.soc.uart._phy.tx.rdy)

        test_signal = Signal(32, reset=0xDEADBEEF)

        phy_dq_i = Signal(32)
        phy_dq_o = Signal(32)
        phy_dq_oe = Signal(32)
        phy_rwds_i = Signal(4)
        phy_rwds_o = Signal(4)
        phy_rwds_oe = Signal(4)

        m.d.comb += [
            phy_dq_i.eq(self.hram.phy.dq.i),
            phy_dq_o.eq(self.hram.phy.dq.o),
            phy_dq_oe.eq(self.hram.phy.dq.oe),
            phy_rwds_i.eq(self.hram.phy.rwds.i),
            phy_rwds_o.eq(self.hram.phy.rwds.o),
            phy_rwds_oe.eq(self.hram.phy.rwds.oe),
        ]

        ila_signals = [
            test_signal,

            self.hram.fsm_dbg,

            self.hram.bus.cyc,
            self.hram.bus.stb,
            self.hram.bus.adr,
            self.hram.bus.we,
            self.hram.bus.dat_r,
            self.hram.bus.dat_w,
            self.hram.bus.ack,
            self.hram.bus.cti,

            self.hram.sr_out,
            self.hram.sr_in,
            self.hram.sr_rwds_in,
            self.hram.sr_rwds_out,
            self.hram.cs,
            self.hram.clk,

            phy_dq_i,
            phy_dq_o,
            phy_dq_oe,
            phy_rwds_i,
            phy_rwds_o,
            phy_rwds_oe,
        ]

        for s in ila_signals:
            print(s)

        self.ila = AsyncSerialILA(signals=ila_signals,
                                  sample_depth=2048, divisor=60,
                                  domain='sync', sample_rate=60e6) # 1MBaud on USB clock

        pmod_uart = [
            Resource("pmod_uart", 0,
                Subsignal("tx",  Pins("1", conn=("pmod", 0), dir='o')),
                Subsignal("rx",  Pins("2", conn=("pmod", 0), dir='i')),
                Attrs(IO_TYPE="LVCMOS33"),
            )
        ]

        platform.add_resources(pmod_uart)

        m.d.comb += [
            self.ila.trigger.eq(self.hyperram_tune._io_loadn.w_stb),
            platform.request("pmod_uart").tx.o.eq(self.ila.tx),
        ]

        m.submodules.ila = self.ila

        return m


# - main ----------------------------------------------------------------------

from luna.gateware.platform import get_appropriate_platform

from luna_soc.generate      import Generate, Introspect

if __name__ == "__old_main__":
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

    # create design
    design = SelftestCore()

    # build soc
    logging.info("Building soc")
    overrides = {
        "debug_verilog": True,
        "verbose": False,
    }
    MustUse._MustUse__silence = False
    products = platform.build(design, do_program=False, build_dir=build_dir, **overrides)
    MustUse._MustUse__silence = True

    # log resources and prepare to generate artifacts
    Introspect(design.soc).log_resources()
    generate = Generate(design.soc)

    # generate: c-header and ld-script
    path = os.path.join(build_dir, "genc")
    if not os.path.exists(path):
        os.makedirs(path)

    logging.info("Generating c-header and ld-script: {}".format(path))
    with open(os.path.join(path, "resources.h"), "w") as f:
        generate.c_header(platform_name=platform.name, file=f)
    with open(os.path.join(path, "soc.ld"), "w") as f:
        generate.ld_script(file=f)

    print("Build completed. Use 'make load' to load bitstream to device.")


if __name__ == "__main__":
    from luna_soc import top_level_cli

    design = SelftestCore()
    top_level_cli(design)

    from luna.gateware.debug.ila import AsyncSerialILAFrontend
    frontend = AsyncSerialILAFrontend("/dev/ttyUSB1", baudrate=1000000, ila=design.ila)
    frontend.emit_vcd("out.vcd")
