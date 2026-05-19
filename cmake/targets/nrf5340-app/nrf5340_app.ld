/*
 * GNU ld linker script for nRF5340 application core, secure mode.
 *
 * Translates dotbot-libs's SES flash placement (nRF/Setup/
 * nRF5340_xxAA_Application_flash_placement.xml + MemoryMap.xml) to
 * GNU ld syntax. Defines exactly the symbols dotbot-libs's pure-C
 * startup (nRF/System/cpu.c + per-chip cpu.c) expects.
 *
 * Memory map (per SES MemoryMap.xml):
 *   FLASH1  0x00000000  1024 KiB
 *   RAM1    0x20000000   256 KiB    ← we use this one
 *   RAM2    0x20040000   252 KiB    ← left unused for now
 *
 * Pinned section: .shared_data at 0x20004000 (16 KiB into RAM1) —
 * the TrustZone IPC region nRF5340_xxAA_Application_cpu.c's
 * ipc_shared_data symbol lives in. Stack: 8 KiB at top of RAM1.
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(reset_handler)

__STACKSIZE__         = 8K;
__STACKSIZE_PROCESS__ = 0;
__HEAPSIZE__          = 0;

/* Pinned address for the TrustZone IPC shared-data region. cpu.c
 * places `ipc_shared_data` here so secure + non-secure sides agree. */
__shared_data_addr__  = 0x20004000;

SECTIONS
{
    /* ── FLASH ─────────────────────────────────────────────────── */

    .vectors :
    {
        KEEP(*(.vectors))
        KEEP(*(.isr_vector))   /* legacy / Nordic naming, just in case */
    } > FLASH

    .text :
    {
        *(.text*)
        *(.init)
        *(.init_rodata)
        . = ALIGN(4);
    } > FLASH

    .rodata :
    {
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ── Initialized data: copied flash → RAM by reset_handler ──── */

    .data :
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data*)
        *(.data_run)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM AT > FLASH
    __data_load_start__ = LOADADDR(.data);

    .fast :
    {
        . = ALIGN(4);
        __fast_start__ = .;
        *(.fast*)
        *(.fast_run)
        . = ALIGN(4);
        __fast_end__ = .;
    } > RAM AT > FLASH
    __fast_load_start__ = LOADADDR(.fast);

    .ctors :
    {
        . = ALIGN(4);
        __ctors_start__ = .;
        KEEP(*(SORT(.ctors.*)))
        KEEP(*(.ctors))
        __ctors_end__ = .;
    } > RAM AT > FLASH
    __ctors_load_start__ = LOADADDR(.ctors);

    .dtors :
    {
        . = ALIGN(4);
        __dtors_start__ = .;
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        __dtors_end__ = .;
    } > RAM AT > FLASH
    __dtors_load_start__ = LOADADDR(.dtors);

    /* .rodata copy-to-RAM only happens under DEBUG; emit placeholder
     * symbols at the same address so cpu.c's load-check `if (dst == src)`
     * trips and the copy is skipped. */
    __rodata_start__       = .;
    __rodata_end__         = .;
    __rodata_load_start__  = .;

    .tdata :
    {
        . = ALIGN(4);
        __tdata_start__ = .;
        *(.tdata*)
        *(.tdata_run)
        . = ALIGN(4);
        __tdata_end__ = .;
    } > RAM AT > FLASH
    __tdata_load_start__ = LOADADDR(.tdata);

    /* Same trick for the runtime-test pattern in cpu.c: if start == load,
     * the copy loop skips. */

    /* ── Uninitialized data: zero'd by reset_handler ────────────── */

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .tbss (NOLOAD) :
    {
        . = ALIGN(4);
        __tbss_start__ = .;
        *(.tbss*)
        __tbss_end__ = .;
    } > RAM

    /* ── Pinned TrustZone shared-data region at 0x20004000 ──────── */

    .shared_data __shared_data_addr__ (NOLOAD) :
    {
        __shared_data_start__ = .;
        KEEP(*(.shared_data))
        . = ALIGN(4);
        __shared_data_end__ = .;
    } > RAM

    /* ── Heap (empty for now) ──────────────────────────────────── */

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_start__ = .;
        . = . + __HEAPSIZE__;
        __heap_end__ = .;
    } > RAM

    /* `end` is newlib's _sbrk marker (heap start). We don't malloc but
     * linking pulls _sbrk in regardless via libc, so the symbol has
     * to be defined or the link fails. */
    PROVIDE(end  = __heap_start__);
    PROVIDE(_end = __heap_start__);

    /* ── Process stack (empty — no RTOS) ───────────────────────── */

    .stack_process (NOLOAD) :
    {
        . = ALIGN(8);
        __stack_process_start__ = .;
        . = . + __STACKSIZE_PROCESS__;
        __stack_process_end__ = .;
    } > RAM

    /* ── Main stack at top of RAM1 ─────────────────────────────── */

    .stack ORIGIN(RAM) + LENGTH(RAM) - __STACKSIZE__ (NOLOAD) :
    {
        . = ALIGN(8);
        __stack_start__ = .;
        . = . + __STACKSIZE__;
        __stack_end__ = .;
    } > RAM

    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}
