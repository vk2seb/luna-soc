from amaranth import *
from amaranth.hdl.rec              import Record
from amaranth.lib.cdc import FFSynchronizer

from amaranth.utils import log2_int
from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap
from amaranth_soc.periph import ConstantMap

from lambdasoc.periph import Peripheral

class HyperBusPHY(Elaboratable):
    """HyperBusPHY x2 for ECP5

    Provides I/O support for a 32bit datapath from HyperRAM x2 module
    - Uses ECP5 primitives IDDRX2F / ODDRX2F / DELAYF
      - Not technically supported under diamond outside of DQS modes
      - Only available on Left/Right I/O banks

    - PLL required to produce 2*sys_clk, then use CLKDIVF to create sys_clk

    - Clocks
     - hr2x    : 2* sys_freq - I/O clock
     - hr      : sys_freq    - core clock
     - hr2x_90 : 2* sys_freq - phase shifted clock output to HyperRAM
     - hr_90   : sys_freq    - phase shifted clock for SCLK

    """
    def __init__(self):

        def io_bus(n):
            return Record([("oe", 1),("i", n),("o", n)])

        self.clk_enable = Signal()
        self.cs         = Signal()
        self.dq         = io_bus(32)
        self.rwds       = io_bus(4)

        # IO Delay shifting
        self.dly_io  = Record([("loadn", 1),("move", 1),("direction", 1)])
        self.dly_clk = Record([("loadn", 1),("move", 1),("direction", 1)])

    def elaborate(self, platform):

        m = Module()

        pads = platform.request("ram")

        # Shift non DDR signals to match the FF's inside DDR modules.
        m.submodules += [
            FFSynchronizer(self.cs,      pads.cs.o,    stages=3),
            FFSynchronizer(self.rwds.oe, pads.rwds.oe, stages=3),
            FFSynchronizer(self.dq.oe,   pads.dq.oe,   stages=3),
        ]

        # Don't hold HyperRAM in reset.
        m.d.comb += [
            pads.reset.o.eq(0)
        ]

        # Mask off clock when no CS
        clk_en = Signal()
        m.d.comb += clk_en.eq(self.clk_enable & ~self.cs)

        clk = Signal()
        m.submodules += [
            Instance("ODDRX2F",
                i_SCLK = ClockSignal("hr_90"),
                i_ECLK = ClockSignal("hr2x_90"),
                i_RST  = ResetSignal("hr"),
                i_D3   = clk_en,
                i_D2   = 0,
                i_D1   = clk_en,
                i_D0   = 0,
                o_Q    = clk
            ),
            Instance("DELAYF",
                p_DEL_MODE  = "USER_DEFINED",
                p_DEL_VALUE = 0, # (25ps per tap)
                i_A         = clk,
                i_LOADN     = self.dly_clk.loadn,
                i_MOVE      = self.dly_clk.move,
                i_DIRECTION = self.dly_clk.direction,
                o_Z         = pads.clk
            )
        ]

        # DQ_out
        for i in range(8):
            m.submodules += Instance("ODDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D3   = self.dq.o[i],
                i_D2   = self.dq.o[8+i],
                i_D1   = self.dq.o[16+i],
                i_D0   = self.dq.o[24+i],
                o_Q    = pads.dq.o[i]
            )


        # DQ_in
        for i in range(8):
            dq_in = Signal()
            m.submodules += [
                Instance("IDDRX2F",
                    i_SCLK = ClockSignal("hr"),
                    i_ECLK = ClockSignal("hr2x"),
                    i_RST  = ResetSignal("hr"),
                    i_D    = dq_in,
                    o_Q3   = self.dq.i[i],
                    o_Q2   = self.dq.i[i+8],
                    o_Q1   = self.dq.i[i+16],
                    o_Q0   = self.dq.i[i+24]
                ),
                Instance("DELAYF",
                    p_DEL_MODE  = "USER_DEFINED",
                    p_DEL_VALUE = 0, # (25ps per tap)
                    i_A         = pads.dq.i[i],
                    i_LOADN     = self.dly_io.loadn,
                    i_MOVE      = self.dly_io.move,
                    i_DIRECTION = self.dly_io.direction,
                    o_Z         = dq_in
                )
            ]

        # RWDS_out
        m.submodules += [
            Instance("ODDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D3   = self.rwds.o[0],
                i_D2   = self.rwds.o[1],
                i_D1   = self.rwds.o[2],
                i_D0   = self.rwds.o[3],
                o_Q    = pads.rwds.o
            )
        ]

        # RWDS_in
        rwds_in = Signal()
        m.submodules += [
            Instance("IDDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D    = rwds_in,
                o_Q3   = self.rwds.i[0],
                o_Q2   = self.rwds.i[1],
                o_Q1   = self.rwds.i[2],
                o_Q0   = self.rwds.i[3]
            ),
            Instance("DELAYF",
                p_DEL_MODE  = "USER_DEFINED",
                p_DEL_VALUE = 0, # (25ps per tap)
                i_A         = pads.rwds.i,
                i_LOADN     = self.dly_io.loadn,
                i_MOVE      = self.dly_io.move,
                i_DIRECTION = self.dly_io.direction,
                o_Z         = rwds_in
            )
        ]

        return m


class HyperRAMX2(Peripheral, Elaboratable):
    """HyperRAMX2

    Provides a HyperRAM core that works at 2:1 system clock speeds
    - PHY is device dependent for DDRx2 primitives
      - ECP5 (done)
    - 90 deg phase shifted clock required from PLL
    - Burst R/W supported if bus is ready
    - Latency indepedent reads (uses RWDS pattern)

    This core favors performance over portability
    This core has only been tested on ECP5 platforms so far.

    TODO:
     - Handle R/W of config registers
     - Configure Latency
     - Handle variable latency writes
     - Add Litex automated tests
    """
    def __init__(self, *, name, latency = 6, cr0_preset = None,
                 cr1_preset = None, dual_die_control = True):

        super().__init__(name=name)

        self.phy = HyperBusPHY()
        self.name = name

        self.bus = wishbone.Interface(addr_width=22,
                                      data_width=32,
                                      granularity=8,
                                      features={"cti"})

        bytes_per_word = self.bus.data_width // self.bus.granularity
        granularity_bits = log2_int(bytes_per_word)
        print(granularity_bits)

        mmap = MemoryMap(addr_width=self.bus.addr_width + granularity_bits,
                         data_width=self.bus.granularity,
                         name=self.name)
        mmap.add_resource("hyperbus_dummy", name="hyperbus_mem",
                          size=2 ** (self.bus.addr_width + granularity_bits))
        self.bus.memory_map = mmap

        self.dly_io  = Record([("loadn", 1),("move", 1),("direction", 1)])
        self.dly_clk = Record([("loadn", 1),("move", 1),("direction", 1)])

        self.latency = latency
        self.cr0_preset = cr0_preset
        self.cr1_preset = cr1_preset
        self.dual_die_control = dual_die_control

    @property
    def constant_map(self):
        return ConstantMap(
            SIZE = self.size,
        )

    def elaborate(self, platform):

        m = Module()

        bus = self.bus

        # # #

        clk         = Signal()
        cs          = Signal()
        ca          = Signal(48)
        sr_in       = Signal(64)
        sr_out      = Signal(64)
        sr_rwds_in  = Signal(8)
        sr_rwds_out = Signal(8)

        timeout_counter = Signal(6)

        phy = self.phy
        m.submodules += phy

        m.d.comb += [
            phy.dly_io.eq(self.dly_io),
            phy.dly_clk.eq(self.dly_clk),
        ]

        # CR0/CR1 preset ---------------------------------------------------------------------------
        preset_cr = True
        multi_cr = False
        die_address = Signal()
        cr_select = Signal()
        cr_value = Signal(16)

        cr0_preset = self.cr0_preset
        cr1_preset = self.cr1_preset
        dual_die_control = self.dual_die_control

        if cr0_preset is not None and cr1_preset is not None:
            multi_cr = True
            m.d.comb += cr_value.eq(Mux(cr_select, cr1_preset, cr0_preset))
        elif cr0_preset is not None:
            m.d.comb += [
                cr_value.eq(cr0_preset),
                cr_select.eq(0)
            ]
        elif cr1_preset is not None:
            m.d.comb += [
                cr_value.eq(cr1_preset),
                cr_select.eq(1)
            ]
        else:
            preset_cr = False

        m.d.comb += [
            phy.cs.eq(~cs),
            phy.clk_enable.eq(clk)
        ]

        # Data In/Out Shift Registers --------------------------------------------------------------
        m.d.sync += [
            sr_out.eq(Cat(Signal(32), sr_out[:32])),
            sr_in.eq(Cat(phy.dq.i, sr_in[:32])),
            sr_rwds_in.eq(Cat(phy.rwds.i, sr_rwds_in[:4])),
            sr_rwds_out.eq(Cat(phy.rwds.i, sr_rwds_out[:4])),
        ]

        m.d.comb += [
            bus.dat_r.eq(Cat(phy.dq.i[-16:], sr_in[:16])), # To Wishbone
            phy.dq.o.eq(sr_out[-32:]),                     # To HyperRAM
            phy.rwds.o.eq(sr_rwds_out[-4:])                # To HyperRAM
        ]

        # Command generation -----------------------------------------------------------------------
        m.d.comb += [
            ca[47].eq(~self.bus.we),          # R/W#
            ca[45].eq(1),                     # Burst Type (Linear)
            ca[16:14+self.bus.addr_width].eq(
                self.bus.adr[2:]),            # Row & Upper Column Address
            ca[1:3].eq(self.bus.adr[0:2]),    # Lower Column Address
            ca[0].eq(0),                      # Lower Column Address
        ]

        # FSM Sequencer ----------------------------------------------------------------------------

        def delayed_enter(fsm, name, target, delay):
            """exactly emulate 'delayed_enter' from migen."""
            assert(delay > 0)
            state = name
            for i in range(delay):
                if i == delay - 1:
                    next_state = target
                else:
                    next_state = f"{state}_d{i}"
                with m.State(state):
                    m.next = next_state
                state = next_state

        with m.FSM(reset="WRITE-CR" if preset_cr else "IDLE") as fsm:
            with m.State("IDLE"):
                with m.If(bus.cyc & bus.stb):
                    m.d.sync += cs.eq(1)
                    m.next = "CA-SEND"
            with m.State("CA-SEND"):
                m.d.sync += [
                    clk.eq(1),
                    phy.dq.oe.eq(1),
                    sr_out.eq(Cat(Signal(16), ca))
                ]
                m.next = "CA-WAIT"
            with m.State("CA-WAIT"):
                m.d.sync += timeout_counter.eq(0)
                m.next = "LATENCY"
            with m.State("LATENCY"):
                m.d.sync += phy.dq.oe.eq(0),
                m.next = "LATENCY-WAIT"

            delayed_enter(fsm, "LATENCY-WAIT", "READ-WRITE-SETUP", self.latency - 3)

            with m.State("READ-WRITE-SETUP"):
                m.d.sync += [
                    phy.dq.oe.eq(self.bus.we),
                    phy.rwds.oe.eq(self.bus.we),
                ]
                m.next = "READ-WRITE"
            with m.State("READ-WRITE"):
                m.next = "READ-ACK"
                with m.If(self.bus.we):
                    m.d.sync += [
                        phy.dq.oe.eq(1), # Write Cycle
                        sr_out[:32].eq(0),
                        sr_out[32:].eq(self.bus.dat_w),
                        sr_rwds_out[:4].eq(0),
                        sr_rwds_out[4:].eq(~bus.sel[0:4]),
                    ]
                    m.d.comb += bus.ack.eq(1), # Get next byte
                    with m.If(bus.cti == 0b010):
                        m.next = "READ-WRITE"
                    with m.Else():
                        m.next = "CLK-OFF"
                with m.If(~self.bus.cyc): # Cycle ended
                    m.d.sync += clk.eq(0),
                    m.next = "CLEANUP"
            with m.State("READ-ACK"):
                m.d.sync += timeout_counter.eq(timeout_counter + 1),
                with m.If(phy.rwds.i[3]):
                    m.d.sync += timeout_counter.eq(0),
                    m.d.comb += bus.ack.eq(1),
                    with m.If(bus.cti != 0b010):
                        m.d.sync += clk.eq(0),
                        m.next = "CLEANUP"
                with m.If(~self.bus.cyc | (timeout_counter > 20)):
                    m.next = "CLK-OFF"
            with m.State("CLK-OFF"):
                m.d.sync += clk.eq(0),
                m.next = "CLEANUP"
            with m.State("CLEANUP"):
                m.d.sync += [
                    cs.eq(0),
                    phy.rwds.oe.eq(0),
                    phy.dq.oe.eq(0),
                ]
                m.next = "HOLD-WAIT"
            with m.State("HOLD-WAIT"):
                m.d.sync += [
                    sr_out.eq(0),
                    sr_rwds_out.eq(0),
                ]
                m.next = "WAIT"

            delayed_enter(fsm, "WAIT", "IDLE", 10)

            if preset_cr:
                with m.State("WRITE-CR"):
                    m.d.sync += [
                        cs.eq(1),
                        phy.rwds.oe.eq(0),
                    ]
                    m.next = "CR-CA-SEND"
                with m.State("CR-CA-SEND"):
                    m.d.sync += [
                        clk.eq(1),
                        phy.dq.oe.eq(1),
                        sr_out.eq(Cat(cr_value, cr_select, C(0, 23),
                                  C(1, 11), die_address, C(0x600, 12))),
                    ]
                    m.next = "CR-CA-WAIT"
                with m.State("CR-CA-WAIT"):
                    m.next = "WRITE-CR-CLK-OFF"
                with m.State("WRITE-CR-CLK-OFF"):
                    m.d.sync += clk.eq(0),
                    m.next = "WRITE-CR-DONE"
                with m.State("WRITE-CR-DONE"):
                    m.d.sync += [
                        cs.eq(0),
                        phy.dq.oe.eq(0),
                    ]
                    with m.If((die_address if dual_die_control else 1) &
                              (cr_select if multi_cr else 1)):
                        m.d.sync += cr_select.eq(0) if multi_cr else [],
                        m.d.sync += die_address.eq(0),
                        m.next = "CLEANUP"
                    with m.Else():
                        m.d.sync += cr_select.eq(~cr_select) if multi_cr else [],
                        m.d.sync += die_address.eq(
                            die_address|cr_select if multi_cr else 1),
                        m.next = "WRITE-CR"

        return m
