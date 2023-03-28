#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "gic.h"

#define GPFSEL2 0x3F200008  // GPIO Function Select 2
#define GPSET0  0x3F20001C  // GPIO Pin Output Set 0
#define GPCLR0  0x3F200028  // GPIO Pin Output Clear 0
#define GPLEV0  0x3F200034  // GPIO Pin Level 0
#define IRQ_GIC1 33         // IRQ number for GPIO interrupts on GIC1

// Handler function for the interrupt
void interrupt_handler(void) {
    printf("Interrupt occurred!\n");
}

int main(void) {
    // Map the GIC CPU interface registers
    uint32_t *gicc_base = (uint32_t *) 0x2C001000;
    uint32_t *gicc_ctlr = gicc_base + 0x00;
    uint32_t *gicc_pmr = gicc_base + 0x04;
    uint32_t *gicc_iar = gicc_base + 0x0C;
    uint32_t *gicc_eoir = gicc_base + 0x10;

    // Map the GIC distributor interface registers
    uint32_t *gicd_base = (uint32_t *) 0x2C010000;
    uint32_t *gicd_ctlr = gicd_base + 0x00;
    uint32_t *gicd_igroupr = gicd_base + 0x80;
    uint32_t *gicd_isenabler1 = gicd_base + 0x100;
    uint32_t *gicd_icenabler1 = gicd_base + 0x180;
    uint32_t *gicd_ispendr1 = gicd_base + 0x200;
    uint32_t *gicd_icpendr1 = gicd_base + 0x280;
    uint32_t *gicd_icdabrir1 = gicd_base + 0x300;
    uint32_t *gicd_icdipr8 = gicd_base + 0x420;
    uint32_t *gicd_icdiptr8 = gicd_base + 0x480;

    // Map the GPIO registers
    uint32_t *gpio_base = (uint32_t *) 0x3F200000;

    // Set GPIO pin 26 as an input
    uint32_t *gpfsel2 = gpio_base + GPFSEL2/4;
    *gpfsel2 &= ~(0x7 << 18);

    // Enable rising edge detection on GPIO pin 26
    uint32_t *gpeds = gpio_base + GPREN0/4;
    *gpeds |= (1 << 26);

    // Initialize the GIC
    *gicc_ctlr = 0;
    *gicd_ctlr = 0;
    *gicc_pmr = 0xFF;
    *gicc_ctlr = 1;
    *gicd_ctlr = 1;

    // Enable the GPIO interrupt on the GIC
    *gicd_igroupr &= ~(1 << IRQ_GIC1);
    *gicd_icdiptr8 |= (IRQ_GIC1 & 0x1F);

    // Register the interrupt handler function
    uint32_t *handler_address = (uint32_t *) &interrupt_handler;
    *gicd_icdabrir1 |= (1 << IRQ_GIC1);
    *(volatile uint32_t *)(0xFFFF000C) = (uint32_t)handler_address; // Route the interrupt to the CPU

    // Wait for the interrupt
    while (1) {
        asm("wfi"); // Wait for interrupt instruction
    }

    // Clean up and exit
    return 0;
}
