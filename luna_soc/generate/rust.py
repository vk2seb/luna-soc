#
# This file is part of LUNA.
#
# Copyright (c) 2023-2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

"""Generate Rust support files for SoC designs."""

import datetime
import logging

from amaranth_soc.memory import MemoryMap

class LinkerScript:
    def __init__(self, memory_map: MemoryMap, reset_addr: int = 0x00000000):
        self.memory_map = memory_map
        self.reset_addr = reset_addr

    def generate(self, file=None):
        """ Generate a memory.x file for the given SoC design"""

        def emit(content):
            """ Utility function that emits a string to the targeted file. """
            print(content, file=file)

        # TODO this should be determined by introspection
        memories = ["ram", "rom", "blockram", "spiflash",
                    "bootrom", "scratchpad", "mainram", "psram", "psram_xip"]

        # warning header
        emit("/*")
        emit(" * Automatically generated by LUNA; edits will be discarded on rebuild.")
        emit(" * (Most header files phrase this 'Do not edit.'; be warned accordingly.)")
        emit(" *")
        emit(f" * Generated: {datetime.datetime.now()}.")
        emit(" */")
        emit("")

        # memory regions
        regions = set()
        emit("MEMORY {")
        window: MemoryMap
        name:   MemoryMap.Name
        for window, name, (start, end, ratio) in self.memory_map.windows():
            name = name[0]
            if name not in memories:
                logging.debug("Skipping non-memory resource: {}".format(name))
                continue
            if self.reset_addr >= start and self.reset_addr < end:
                start = self.reset_addr
            emit(f"    {name} : ORIGIN = 0x{start:08x}, LENGTH = 0x{end-start:08x}")
            regions.add(name)
        emit("}")
        emit("")

        # region aliases
        ram = "blockram" if "blockram" in regions else "scratchpad"
        if "psram_xip" in regions:
            rom = "psram_xip"
        else if "spiflash" in regions:
            rom = "spiflash"
        else:
            rom = ram
        aliases = {
            "REGION_TEXT":   rom,
            "REGION_RODATA": rom,
            "REGION_DATA":   ram,
            "REGION_BSS":    ram,
            "REGION_HEAP":   ram,
            "REGION_STACK":  ram,
        }
        for alias, region in aliases.items():
            emit(f"REGION_ALIAS(\"{alias}\", {region});")
