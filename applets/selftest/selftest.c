/**
 * This file is part of LUNA.
 *
 * Copyright (c) 2021 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"
#include "uart.h"
#include "ulpi.h"
//#include "psram.h"

// Create a type alias for our tests.
typedef bool (*simple_test)(void);


/**
 * Runs a named test.
 */
uint32_t run_test(char *description, simple_test test)
{
    // Identify which test we're running.
    uart_puts(description);

    // Run the test, and print its results.
    if (test()) {
        uart_puts("✅ OK\n");
        return 0;
    } else {
        return 1;
    }
}


/**
 * Core tests.
 */
bool debug_controller_tests(void)
{
    return true;
}


/**
 * ULPI PHY tests.
 */
bool ulpi_phy_tests(enum ulpi_phy phy)
{
    int16_t scratch;

    //
    // Check that the ULPI PHY matches the VID/PID for a Microchip USB3343.
    //
    const bool id_matches =
        (read_ulpi_register(phy, 0) == 0x24) &&
        (read_ulpi_register(phy, 1) == 0x04) &&
        (read_ulpi_register(phy, 2) == 0x09) &&
        (read_ulpi_register(phy, 3) == 0x00);
    if (!id_matches) {
        uart_puts("❌ FAIL: PHY ID read failure! ");
        return false;
    }

    //
    // Check that we can set the scratch register to every binary-numbered value.
    // This checks each of the lines connected to the scratch register.
    //
    for (uint16_t i = 0; i < 8; i++) {
        uint8_t mask = (1 << i);

        // Perform a write followed by a read, to make sure the write took.
        //
        // For now, we seem to have an issue somwhere in timing that makes it
        // so these writes only take if multiply written. This doesn't affect actual
        // gateware, so for now, we're duplicating the writes.
        write_ulpi_register(phy, 0x16, mask);
        write_ulpi_register(phy, 0x16, mask);
        write_ulpi_register(phy, 0x16, mask);

        //write_ulpi_register(phy, 0x16, mask);
        scratch = read_ulpi_register(phy, 0x16);

        if (scratch != mask) {
            uart_puts("❌ FAIL: Scratch register readback failure (bit ");
            print_char('0' + i);
            uart_puts(")!\n");
            return false;
        }
    }

    return true;
}


bool target_phy_tests(void)
{
    return ulpi_phy_tests(TARGET_PHY);
}


#if 0
/**
 * RAM tests.
 */
bool ram_tests(void)
{
    //
    // Check that the ULPI PHY matches the VID/PID for a Winbond or Cypress PSRAM.
    //
    /*
    const uint32_t psram_id = read_psram_register(0);
    const bool id_matches =
        (psram_id == 0x0c81) ||
        (psram_id == 0x0c86);


    if (psram_id == 0xFFFF) {
        uart_puts("❌ FAIL: RAM ID read failure! (RAM did not respond)\n");
        return false;
    }

    if (!id_matches) {
        uart_puts("❌ FAIL: RAM ID read failure! (was: ");
        uart_print_word(psram_id);
        uart_puts(")\n");
        return false;
    }
    */

    uint32_t addr = 0x1000;
    uint32_t nbit = 16;
    uint32_t n = 32;

    for (uint32_t i = 0; i != n; ++i) {

        psram_wvalue_write(1 << (i%16));
        psram_write_write(0x1);
        psram_address_write(addr + 4*i);
        psram_address_write(addr + 4*i);

        // Wait for things to be come ready.
        if(while_with_timeout(psram_busy_read, 100)) {
            return -1;
        }

    }


    for (uint32_t i = 0; i != n; ++i) {

        psram_write_write(0x0);
        psram_address_write(addr + 4*i);
        psram_address_write(addr + 4*i);

        // Wait for things to be come ready.
        if(while_with_timeout(psram_busy_read, 100)) {
            return -1;
        }

        uint32_t rd = psram_rvalue_read();
        uart_print_word(rd);
        uart_puts("\n");
    }

    return true;
}
#endif

/*
bool ram_tests(void)
{
    uint32_t n = 0xFF;

    uint32_t *hr_ptr =
        ((volatile uint32_t*)HYPERRAMX2_HYPERBUS_MEM_ADDRESS);

    uart_puts("read_before\n");
    for(uint32_t i = 0; i != n; ++i) {
        uart_print_word(i);
        uart_puts(" ");
        uart_print_word(*(hr_ptr + i));
        uart_puts("\n");
    }

    uart_puts("write\n");
    for(uint32_t i = 0; i != n; ++i) {
        *(hr_ptr + i) = 0xDEADBEEF;
    }

    uart_puts("read_after\n");
    for(uint32_t i = 0; i != n; ++i) {
        uart_print_word(i);
        uart_puts(" ");
        uart_print_word(*(hr_ptr + i));
        uart_puts("\n");
    }

    return true;
}
*/

#define HYPERRAM_BASE HYPERRAMX2_HYPERBUS_MEM_ADDRESS

void set_io_delay(int);
void set_clk_delay(int);
static int basic_memtest(void);


void set_io_delay(int cnt){
	hyperram_tune_io_loadn_write(0);
	hyperram_tune_io_loadn_write(1);
	hyperram_tune_io_direction_write(0);

	/* 25ps of delay per tap.
	   Each rising edge adds to the io delay */
	for(int i = 0; i < cnt; i++){ 
		hyperram_tune_io_move_write(1);
		hyperram_tune_io_move_write(0);
	}
}

void set_clk_delay(int cnt){
	hyperram_tune_clk_loadn_write(0);
	hyperram_tune_clk_loadn_write(1);
	hyperram_tune_clk_direction_write(0);

	/* 25ps of delay per tap.
	   Each rising edge adds to the io delay */
	for(int i = 0; i < cnt; i++){ 
		hyperram_tune_clk_move_write(1);
		hyperram_tune_clk_move_write(0);
	}
}

static void flush_caches(void) {
    // TODO
}

/*
	Test memory location by writing a value and attempting read-back.
	Try twice to avoid situation where memory is read-only and set from a previous test.
*/
static int basic_memtest(void){

	*((volatile uint32_t*)HYPERRAM_BASE) = 0xFF55AACD;

    flush_caches();

	if(*((volatile uint32_t*)HYPERRAM_BASE) != 0xFF55AACD)
		return 0;

	*((volatile uint32_t*)HYPERRAM_BASE) = 0xA3112233;

    flush_caches();

	if(*((volatile uint32_t*)HYPERRAM_BASE) != 0xA3112233)
		return 0;

	return 1;
}


void hyperram_init(void){
    uart_puts("\n--==========-- \e[1mHyperRAM Init\e[0m ===========--\n");

	int window = 0;
	int clk_del = 0;
	int io_del = 0;

	while(clk_del < 128){
		set_clk_delay(clk_del >> 2);
		set_io_delay(io_del);
		int i = 0;

		//printf("%u,%u, %u |", clk_del >> 2, clk_del & 1 ? 1 : 0, clk_del & 2 ? 1 : 0);
        uart_print_word(clk_del >> 2);
        uart_puts(",");
        uart_print_word(clk_del & 1 ? 1 : 0);
        uart_puts(", ");
        uart_print_word(clk_del & 2 ? 1 : 0);
        uart_puts(" |");

		for(i = 0; i < 64; i++){

			int pass = basic_memtest();

			// Shift our PLL
			crg_tune_phase_sel_write(0);
			crg_tune_phase_dir_write(0);
			crg_tune_phase_step_write(0);
			crg_tune_phase_step_write(1);

			if(i & 1)
				print_char(pass > 0 ? '0' : '-');

			if(pass == 1){
				window++;
			}
			else if(pass != 1){
				if(window >= 6){
					break;
				}else {
					window = 0;
				}
			}

		}
		uart_puts("| ");
        uart_print_word(window);
		uart_puts("    \n");
		if(window >= 5){
			for(i = 0; i < window/2; i++){
				// Shift our PLL up
				crg_tune_phase_sel_write(0);
				crg_tune_phase_dir_write(1);
				crg_tune_phase_step_write(0);
				crg_tune_phase_step_write(1);
			}
			return;
		}
		window = 0;
		clk_del = (clk_del + 1);

		crg_tune_slip_hr2x90_write(clk_del & 1 ? 1 : 0);
		crg_tune_slip_hr2x_write(clk_del & 2 ? 1 : 0);

		crg_tune_slip_hr2x90_write(0);
		crg_tune_slip_hr2x_write(0);
	}

	uart_puts("\n\n Error: RAM Init failed :( ... ");
}

bool ram_tests() {
    hyperram_init();
    return false;
}


/**
 * Identifies itself to the user.
 */
void print_greeting(void)
{
    uart_puts("\n _     _   _ _   _   ___  \n");
    uart_puts("| |   | | | | \\ | | / _ \\ \n");
    uart_puts("| |   | | | |  \\| |/ /_\\ \\\n");
    uart_puts("| |   | | | | . ` ||  _  |\n");
    uart_puts("| |___| |_| | |\\  || | | |\n");
    uart_puts("\\_____/\\___/\\_| \\_/\\_| |_/\n\n\b");

    uart_puts("Self-test firmware booted. 🌙\n");
    uart_puts("Running on a Minerva RISC-V softcore on a ");
    uart_puts(PLATFORM_NAME);
    uart_puts(" board.\n\n");
}


/**
 * Core self-test routine.
 */
int main(void)
{
    uint32_t failures = 0;

    // Perform our platform initialization.
    platform_bringup();

    // Turn on the yellow LED, indicating that we're performing the tests.
    leds_output_write(0b01);

    // Wait for a bit, so we know the other side is listening and ready.
    // FIXME: remove this?
    sleep_ms(1000);

    // Print a nice header for our tests.
    print_greeting();

    while (1) {
        char command = 0;

        //uart_puts("\n\n");
        //uart_puts("Press 's' to run the simple self-test (no connections required).");
        //uart_puts("Press 'f' to run the factory self-test (setup required).\n");
        //uart_puts("Press 'a' to run the full self-test (setup required).\n");
        //uart_puts("Press Ctrl+] to terminate test.\n");

        // FIXME: enable test switching

        // Wait for input from the user.
        //while (!command) {
        //	command = uart_getchar();
        //}
        command = 's';

        switch(command) {

            // Run all tests.
            case 'a':
                // Falls through.

            // Run factory tests.
            case 'f':
                // Falls through.

            // Run our core tests.
            case 's':
                command = 0;

                failures += run_test("Debug controller & communications:     ", debug_controller_tests);
                failures += run_test("Target ULPI PHY:                       ", target_phy_tests);
                failures += run_test("External RAM:                          ", ram_tests);

                uart_puts("\n\n");

                if (failures) {
                    uart_puts("❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌\n");
                    uart_puts("------------------------------------------------\n");
                    uart_puts("--------------- TESTS FAILED! ------------------\n");
                    uart_puts("------------------------------------------------\n");
                    uart_puts("❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌\n");
                }

                else {
                    leds_output_write(0b10);
                    uart_puts("All tests passed. ✅ \n\n");
                }

                while(1);

                break;

            default:
                uart_puts("Unknown command.\n");
        }

    }
}
