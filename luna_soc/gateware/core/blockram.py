#
# This file is part of LUNA.
#
# Copyright (c) 2023-2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth             import *
from amaranth.lib         import wiring, memory
from amaranth.lib.wiring  import In
from amaranth.utils       import exact_log2

from amaranth_soc         import wishbone
from amaranth_soc.memory  import MemoryMap
from amaranth_soc.periph  import ConstantMap


class Peripheral(wiring.Component):
    """SRAM storage peripheral.

    Parameters
    ----------
    size : int
        Memory size in bytes.
    data_width : int
        The width of each memory word.
    granularity : int
        The number of bits of data per each address.
    writable : bool
        Memory is writable.
    init : list[byte] Optional
        The initial value of the relevant memory.
    name : str
        A descriptive name for the given memory.

    Attributes
    ----------
    bus : :class:`amaranth_soc.wishbone.Interface`
        Wishbone bus interface.
    """

    def __init__(self, *, size, data_width=32, granularity=8, writable=True, init=[], name="blockram"):
        if not isinstance(size, int) or size <= 0 or size & size-1:
            raise ValueError("Size must be an integer power of two, not {!r}"
                             .format(size))
        if size < data_width // granularity:
            raise ValueError("Size {} cannot be lesser than the data width/granularity ratio "
                             "of {} ({} / {})"
                              .format(size, data_width // granularity, data_width, granularity))

        self.size        = size
        self.granularity = granularity
        self.writable    = writable
        self.name        = name

        depth = (size * granularity) // data_width
        self._mem = memory.Memory(shape=data_width, depth=depth, init=init)

        super().__init__({
            "bus": In(wishbone.Signature(addr_width=exact_log2(depth),
                                         data_width=data_width,
                                         granularity=granularity,
                                         features={"cti", "bte"})),
        })

        memory_map = MemoryMap(addr_width=exact_log2(size), data_width=granularity)
        memory_map.add_resource(name=("memory", self.name,), size=size, resource=self)
        self.bus.memory_map = memory_map

    @property
    def init(self):
        return self._mem.init

    @init.setter
    def init(self, init):
        self._mem.init = init

    @property
    def constant_map(self):
        return ConstantMap(
            SIZE = self.size,
        )

    def elaborate(self, platform):
        m = Module()
        m.submodules.mem = self._mem

        mem_rp = self._mem.read_port()
        m.d.comb += self.bus.dat_r.eq(mem_rp.data)

        next_addr = Signal.like(self.bus.adr)

        with m.Switch(self.bus.bte):
            with m.Case(wishbone.BurstTypeExt.LINEAR):
                m.d.comb += next_addr.eq(self.bus.adr + 1)
            with m.Case(wishbone.BurstTypeExt.WRAP_4):
                m.d.comb += next_addr.eq((self.bus.adr & ~0b11) | ((self.bus.adr + 1) & 0b11))
            with m.Case(wishbone.BurstTypeExt.WRAP_8):
                m.d.comb += next_addr.eq((self.bus.adr & ~0b111) | ((self.bus.adr + 1) & 0b111))
            with m.Case(wishbone.BurstTypeExt.WRAP_16):
                m.d.comb += next_addr.eq((self.bus.adr & ~0b1111) | ((self.bus.adr + 1) & 0b1111))

        current_addr = Signal.like(self.bus.adr)
        in_burst = Signal(reset=0)

        with m.If(self.bus.cyc):
            with m.If(self.bus.stb & self.bus.ack & (self.bus.cti == wishbone.CycleType.END_OF_BURST)):
                m.d.sync += in_burst.eq(0)
            with m.Elif(self.bus.stb & ~self.bus.ack & ~in_burst & 
                      (self.bus.cti != wishbone.CycleType.CLASSIC) & 
                      (self.bus.cti != wishbone.CycleType.END_OF_BURST)):
                m.d.sync += [
                    in_burst.eq(1),
                    current_addr.eq(self.bus.adr)
                ]
            with m.Elif(self.bus.stb & self.bus.ack & in_burst & 
                       (self.bus.cti != wishbone.CycleType.END_OF_BURST)):
                m.d.sync += current_addr.eq(next_addr)
        with m.Else():
            m.d.sync += in_burst.eq(0)

        with m.If(self.bus.cyc & self.bus.stb & ~self.bus.ack):
            m.d.sync += self.bus.ack.eq(1)
            m.d.comb += mem_rp.addr.eq(self.bus.adr)
        with m.Elif(self.bus.cyc & self.bus.stb & self.bus.ack & in_burst):
            m.d.comb += mem_rp.addr.eq(next_addr)
        with m.Elif(self.bus.cyc & ~self.bus.stb):
            m.d.sync += self.bus.ack.eq(0)
        with m.Else():
            m.d.sync += self.bus.ack.eq(0)

        if self.writable:
            mem_wp = self._mem.write_port(granularity=self.granularity)
            m.d.comb += mem_wp.addr.eq(self.bus.adr)  # Always use immediate address for writes
            m.d.comb += mem_wp.data.eq(self.bus.dat_w)
            with m.If(self.bus.cyc & self.bus.stb & self.bus.we & in_burst):
                m.d.comb += mem_wp.en.eq(self.bus.sel)

        return m
