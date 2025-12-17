
/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
    RAM_DTCM (xrw)  : ORIGIN = 0x20000000,  LENGTH = 128K
    RAM_D1 (xrw)    : ORIGIN = 0x24000000,  LENGTH = 512K
    RAM_D2 (xrw)    : ORIGIN = 0x30000000,  LENGTH = 288K
    RAM_D3 (xrw)    : ORIGIN = 0x38000000,  LENGTH = 64K
    RAM_ITCM (xr)   : ORIGIN = 0x00000000,  LENGTH = 64K
    FLASH (rx)      : ORIGIN = 0x08040000,  LENGTH = 2M	
    /* SDRAM (xrw)     : ORIGIN = 0xD0000000,  LENGTH = 8M */
}

_Main_Stack_Size = 0x400; /* required amount of stack */

_sram = ORIGIN(RAM_D2);
_eram = ORIGIN(RAM_D2) + LENGTH(RAM_D2);

/* Define output sections */
SECTIONS
{
    /* The startup code goes first into FLASH */    
	/* RTOS related file put in ITCM */
    
    _itcm_load_s = LOADADDR(.itcm);
    .itcm :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        */tx_*.o (.text .text*)           /* .text sections (code) */
        */txe_*.o (.text .text*)          /* .text* sections (code) */
        *ringbuffer.o (.text .text*)
        *uart_agent.o (.text .text*)
        . = ALIGN(4);
        _itcm_end = .;
    } >RAM_ITCM AT> FLASH
    
    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)

        /* modified placement: need to exclude the RTOS related file */ /* *stm32h7xx*.o */
        *(EXCLUDE_FILE(*/tx_*.o */txe_*.o *ringbuffer.o *uart_agent.o) .text .text*) /* .text and .text* sections (code) */
        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;        /* define a global symbols at end of code */
    } >FLASH


    /* Constant data goes into FLASH */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >FLASH

    .ARM.extab   : 
    { 
        *(.ARM.extab* .gnu.linkonce.armextab.*) 
    } >FLASH

    .ARM : 
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array     :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH
    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* assign space for main stack */
    .main_stack :
    {
        . = ALIGN(4);
        . = . + _Main_Stack_Size;    /* main stack */

	    . = ALIGN(4);
        _estack = .;        /* a global symbol at MSP end */
    }>RAM_DTCM

    /* used by the startup to initialize data */
    _sidata = LOADADDR(.data);
    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data : 
    {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end */
    } >RAM_DTCM AT> FLASH

  
    /* Uninitialized data section */
    . = ALIGN(4);
    .bss :
    {
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } >RAM_D1

    /* Remove information from the standard libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


