#
# This file is part of LUNA.
#
# Copyright (c) 2023 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth import *
from amaranth.utils import log2_int

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap
from amaranth_soc.periph import ConstantMap

from lambdasoc.periph import Peripheral

from luna.gateware.interface.psram import HyperRAMInterface, HyperRAMPHY

__all__ = ["HyperRAMPeripheral"]

class HyperRAMPeripheral(Peripheral, Elaboratable):
    """HyperRAM peripheral.

    Parameters
    ----------
    size : int
        Memory size in bytes.
    data_width : int
        Bus data width.
    granularity : int
        Bus granularity.

    Attributes
    ----------
    bus : :class:`amaranth_soc.wishbone.Interface`
        Wishbone bus interface.
    """
    def __init__(self, *, size, data_width=32, granularity=8):
        super().__init__()

        if not isinstance(size, int) or size <= 0 or size & size-1:
            raise ValueError("Size must be an integer power of two, not {!r}"
                             .format(size))
        if size < data_width // granularity:
            raise ValueError("Size {} cannot be lesser than the data width/granularity ratio "
                             "of {} ({} / {})"
                              .format(size, data_width // granularity, data_width, granularity))

        self.mem_depth = (size * granularity) // data_width

        self.bus = wishbone.Interface(addr_width=log2_int(self.mem_depth),
                                      data_width=data_width, granularity=granularity,
                                      features={"cti", "bte"}, name="hram_upstream")

        self._hram_arbiter = wishbone.Arbiter(addr_width=log2_int(self.mem_depth),
                                              data_width=data_width, granularity=granularity,
                                              features={"cti", "bte"})
        self._hram_arbiter.add(self.bus)
        self.shared_bus = self._hram_arbiter.bus

        map = MemoryMap(addr_width=log2_int(size), data_width=granularity, name=self.name)
        map.add_resource("placeholder_hyperram", name="mem", size=size)
        self.bus.memory_map = map

        self.size        = size
        self.granularity = granularity

        self.psram_phy = HyperRAMPHY(bus=None)
        self.psram = HyperRAMInterface(phy=self.psram_phy.phy)

    def add_master(self, bus):
        self._hram_arbiter.add(bus)

    @property
    def constant_map(self):
        return ConstantMap(
            SIZE = self.size,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.arbiter    = self._hram_arbiter

        self.psram_phy.bus = platform.request('ram')
        m.submodules += [self.psram_phy, self.psram]
        psram = self.psram

        m.d.comb += [
            self.psram_phy.bus.reset.o        .eq(0),
            psram.single_page      .eq(0),
            psram.register_space   .eq(0),
            psram.perform_write.eq(self.shared_bus.we),
        ]

        with m.FSM() as fsm:
            with m.State('IDLE'):
                m.d.sync += self.shared_bus.ack.eq(0)
                with m.If(self.shared_bus.cyc & self.shared_bus.stb & psram.idle):
                    m.d.sync += [
                        psram.address.eq(self.shared_bus.adr << 1),
                    ]
                    m.next = 'WORD1'
            with m.State('WORD1'):
                m.d.sync += [
                    psram.final_word.eq(0),
                    psram.write_data.eq(self.shared_bus.dat_w[0:16]),
                ]
                m.d.comb += [
                    psram.start_transfer.eq(1)
                ]
                m.next = 'WORD1-WAIT'
            with m.State('WORD1-WAIT'):
                m.d.sync += self.shared_bus.ack.eq(0)
                with m.If(psram.read_ready):
                    m.d.sync += [
                        self.shared_bus.dat_r[0:16].eq(psram.read_data),
                    ]

                with m.If(psram.write_ready):
                    m.d.sync += [
                        psram.write_data.eq(self.shared_bus.dat_w[16:32]),
                    ]

                with m.If(psram.read_ready | psram.write_ready):
                    with m.If(self.shared_bus.cti != wishbone.CycleType.INCR_BURST):
                        m.d.sync += psram.final_word.eq(1)
                    with m.Else():
                        m.d.sync += psram.final_word.eq(0)
                    m.next = 'WORD2'

            with m.State('WORD2'):
                with m.If(psram.read_ready | psram.write_ready):
                    m.d.sync += self.shared_bus.ack.eq(1)
                    m.d.sync += [
                        self.shared_bus.dat_r[16:32].eq(psram.read_data),
                        psram.write_data.eq(self.shared_bus.dat_w[0:16]),
                    ]
                    with m.If(self.shared_bus.cti != wishbone.CycleType.INCR_BURST):
                        m.next = 'IDLE'
                        m.d.sync += psram.final_word.eq(1)
                    with m.Else():
                        m.next = 'WORD1-WAIT'

        return m

