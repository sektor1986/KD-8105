;====================================================================
;
;    Startup file for memory and basic controller initialisation
;
;    MB96600 Family C Compiler
;
;====================================================================
; 1  Disclaimer
;====================================================================
;                                                                      
/************************************************************************/
/*               (C) Fujitsu Semiconductor Europe GmbH (FSEU)           */
/*                                                                      */
/* The following software deliverable is intended for and must only be  */
/* used for reference and in an evaluation laboratory environment.      */
/* It is provided on an as-is basis without charge and is subject to    */
/* alterations.                                                         */
/* It is the user’s obligation to fully test the software in its        */
/* environment and to ensure proper functionality, qualification and    */
/* compliance with component specifications.                            */
/*                                                                      */
/* In the event the software deliverable includes the use of open       */
/* source components, the provisions of the governing open source       */
/* license agreement shall apply with respect to such software          */
/* deliverable.                                                         */
/* FSEU does not warrant that the deliverables do not infringe any      */
/* third party intellectual property right (IPR). In the event that     */
/* the deliverables infringe a third party IPR it is the sole           */
/* responsibility of the customer to obtain necessary licenses to       */
/* continue the usage of the deliverable.                               */
/*                                                                      */
/* To the maximum extent permitted by applicable law FSEU disclaims all */
/* warranties, whether express or implied, in particular, but not       */
/* limited to, warranties of merchantability and fitness for a          */
/* particular purpose for which the deliverable is not designated.      */
/*                                                                      */
/* To the maximum extent permitted by applicable law, FSEU’s liability  */
/* is restricted to intentional misconduct and gross negligence.        */
/* FSEU is not liable for consequential damages.                        */
/*                                                                      */
/* (V1.5)                                                               */
/************************************************************************/

                                                         
;
;====================================================================

          .PROGRAM  STARTUP
          .TITLE    "STARTUP FILE FOR MEMORY INITIALISATION"

;====================================================================
; 2  Contents
;====================================================================
;
; 1  Disclaimer
;
; 2  Contents
;
; 3  History
;
; 4  Settings
; 4.1   Controller Series, Device
; 4.2   C-language Memory model
; 4.3   Function-Call Interface
; 4.4   Constant Data Handling
; 4.5   Stack Type and Stack Size
; 4.6   General Register Bank
; 4.7   Low-Level Library Interface
; 4.8   Clock Selection
; 4.9   Clock Stabilization Time
; 4.10  ROM Mirror configuration
; 4.11  Enable RAMCODE Copying
; 4.12  Enable information stamp in ROM
; 4.13  Flash Security
; 4.14  Flash Write Protection
; 4.15  Boot Vector
; 4.16  Vector table base register configuration
; 4.17  UART scanning
; 4.18  On-chip debug system configuration
; 4.19  NMI configuration
; 4.20  Low voltage detection configuration
; 4.21  Watchdog timer interval configuration
;
; 5  Section and Data Declaration
; 5.1   Several fixed addresses (fixed for MB966xx controllers)
; 5.2   Set ROM Configuration for Boot Vector
; 5.3   Set ROM configuration for UART Scanning
; 5.4   Set Flash Security
; 5.5   Set Flash write protection
; 5.6   Set ROM configuration for OCDS
; 5.7   Set ROM configuration for NMI
; 5.8   Set ROM configuration for Table Base Register (TBR)
; 5.9   Set ROM configuration for sub oscillator
; 5.10  Set ROM configuration for low voltage detection
; 5.11  Set ROM configuration for watchdog interval
; 5.12  Store version information
; 5.13  Declaration of __near addressed data sections
; 5.14  Declaration of RAMCODE section and labels
; 5.15  Declaration of sections containing other sections description
; 5.16  Stack area and stack top definition/declaration
; 5.17  Direct page register dummy label definition
;
; 6  Start-Up Code
; 6.1   Import external symbols
; 6.2   Program start (the reset vector should point here)
; 6.3   Initialisation of processor status
; 6.4   Set clock ratio (ignore subclock)
; 6.5   Set ROM mirror configuaration
; 6.6   Prepare stacks and set the default stack type
; 6.7   Copy initial values to data areas.
; 6.8   Clear uninitialized data areas to zero
; 6.9   Set Data Bank Register (DTB) and Direct Page Register (DPR)
; 6.10  Wait for clocks to stabilize
; 6.11  Initialise Low-Level Library Interface
; 6.12  Call C-language main function
; 6.13  Shut down library
; 6.14  Program end loop
;
;====================================================================
; 3  History
;====================================================================
; Id: START.ASM 3734 2011-02-03 08:57:19Z ceyric

#define VERSION  "1.00"
/*
Revision 1.00 Id: START.ASM 3734 2011-02-03 08:57:19Z ceyric
- initial version based on start.asm for MB96300 Family, version 1.59
*/
;====================================================================

;====================================================================
; 4  Settings
;====================================================================
;
; CHECK ALL OPTIONS WHETHER THEY FIT TO THE APPLICATION
;
; Configure this startup file in the "Settings" section. Search for
; comments with leading "; <<<". This points to the items to be set.
;====================================================================
#set      OFF       0
#set      ON        1

;====================================================================
; 4.1   Controller Series, Device
;====================================================================


#set      MB96670   670
#set      MB96680   680

#set      SERIES    MB96670        ; <<< select Series


; Please specify the device according to the following selection;
; Note: Do not change order because of potential device number
; dependency in certain sections of this start-up file

; MB96680 series
#set      MB96673RA   67031
#set      MB96673AA   67032
#set      MB96675RA   67051
#set      MB96675AA   67052

; MB96680 series
#set      MB96683RA   68031
#set      MB96683AA   68032
#set      MB96685RA   68051
#set      MB96685AA   68052

#set      DEVICE    MB96675RA      ; <<< select device

;====================================================================
; 4.2   C-language Memory model
;====================================================================

                                   ;      data      code
#set      SMALL     0              ;     16 Bit    16 Bit
#set      MEDIUM    1              ;     16 Bit    24 Bit
#set      COMPACT   2              ;     24 Bit    16 Bit
#set      LARGE     3              ;     24 Bit    24 Bit
#set      AUTOMODEL 4              ; works always, might occupy two
                                   ; additional bytes


#set      MEMMODEL  AUTOMODEL      ; <<< C-memory model

; The selected memory model should be set in order to fit to the
; model selected for the compiler.
; Note, in this startup version AUTOMODEL will work for all
; C-models. However, if the compiler is configured for SMALL or
; COMPACT, two additional bytes on stack are occupied. If this is not
; acceptable, the above setting should be set to the correct model.

;====================================================================
; 4.3   Function-Call Interface
;====================================================================

          #if __REG_PASS__
            .REG_PASS
          #endif

; Above statement informs Assembler on compatibility of start-up code
; to  Function Call Interface  as selected for the application. There
; is nothing to configure.
; The Function-Call Interface specifies the method of passing parame-
; ter from function caller to callee.  The standard method of FCC907S
; compiler  uses  "stack argument passing".  Alternatively,  language
; tools can be configured for "register argument passing".
; For details see the compiler manual.
; This start-up file is compatible to both interfaces.

;====================================================================
; 4.4   Constant Data Handling
;====================================================================

#set      ROMCONST    0            ; works only with compiler ROMCONST
#set      RAMCONST    1            ; works with BOTH compiler settings
#set      AUTOCONST   RAMCONST     ; works with BOTH compiler settings

#set      CONSTDATA   AUTOCONST    ; <<< set RAM/ROM/AUTOCONST

; - AUTOCONST (default) is the same as RAMCONST
; - RAMCONST/AUTOCONST should always work, even if compiler is set to
;   ROMCONST. If compiler is set to ROMCONST and this startup file is
;   set to RAMCONST or AUTOCONST, this startup file will generate an
;   empty section CINIT in RAM. However, the code, which copies from
;   CONST to CINIT will not have any effect, because size of section is 0.
; - It is highly recommended to set the compiler to ROMCONST for
;   single-chip mode or internal ROM+ext bus. The start-up file
;   should be set to AUTOCONST.
; - ROMCONST setting on systems with full external bus requires exter-
;   nal address mapping.
;   Single-chip can be emulated by the emulator debugger.
;   ROM mirror can also be used with simulator.
;
; see also ROM MIRROR options

;====================================================================
; 4.5   Stack Type and Stack Size
;====================================================================

#set      USRSTACK       0      ; user stack: for main program
#set      SYSSTACK       1      ; system stack: for main program and interrupts

#set      STACKUSE  SYSSTACK    ; <<< set active stack

#set      STACK_RESERVE  ON     ; <<< reserve stack area in this module
#set      STACK_SYS_SIZE 768    ; <<< byte size of System stack
#set      STACK_USR_SIZE 2      ; <<< byte size of User stack

#set      STACK_FILL     ON     ; <<< fills the stack area with pattern
#set      STACK_PATTERN  0x55AA ; <<< the pattern to write to stack

; - If the active stack is set to SYSSTACK, it is used for main program
;   and interrupts. In this case, the user stack can be set to a dummy
;   size.
;   If the active stack is set to user stack, it is used for the main
;   program but the system stack is automatically activated, if an inter-
;   rupt is serviced. Both stack areas must have a reasonable size.
; - If STACK_RESERVE is ON, the sections USTACK and SSTACK are reserved
;   in this module. Otherwise, they have to be reserved in other modules.
;   If STACK_RESERVE is OFF, the size definitions STACK_SYS_SIZE and
;   STACK_USR_SIZE have no meaning.
; - Even if they are reserved in other modules, they are still initialised
;   in this start-up file.
; - Filling the stack with a pattern allows to dynamically check the stack
;   area, which had already been used.
;
; - If only system stack is used and SSB is linked to a different bank
;   than USB, make sure that all C-modules (which generate far pointers
;   to stack data) have "#pragma SSB". Applies only to exclusive confi-
;   gurations.
; - Note, several library functions require quite a big stack (due to
;   ANSI). Check the stack information files (*.stk) in the LIB\907
;   directory.

;====================================================================
; 4.6   General Register Bank
;====================================================================

#set      REGBANK   0           ; <<< set default register bank

; set the General Register Bank that is to be used after startup.
; Usually, this is bank 0, which applies to address H'180..H'18F. Set
; in the range from 0 to 31.
; Note: All used register banks have to be reserved (linker options).

#if REGBANK > 31 || REGBANK < 0
#  error REGBANK setting out of range
#endif

;====================================================================
; 4.7   Low-Level Library Interface
;====================================================================

#set      CLIBINIT       OFF    ; <<< select extended library usage

; This option has only to be set, if stream-IO/standard-IO function of
; the C-library have to be used (printf(), fopen()...). This also
; requires low-level functions to be defined by the application
; software.
; For other library functions (like e.g. sprintf()) all this is not
; necessary. However, several functions consume a large amount of stack.

;====================================================================
; 4.8   Clock Selection
;====================================================================

; The clock selection requires that a 4 MHz or 8 MHz external clock
; is provided as the Main Clock. If a different frequency is used,
; the Flash Memory Timing settings must be checked!

#set      CLOCKWAIT      ON     ; <<<  wait for stabilized clock, if
                                ;      Main Clock or PLL is used

; The clock is set quite early. However, if CLOCKWAIT is ON, polling
; for machine clock to be switched to Main Clock or PLL is done at
; the end of this file. Therefore, the stabilization time is not
; wasted. Main() will finally start at correct speed. Resources can
; be used immediately.
; Note: If CLOCKWAIT is switched off, Flash timing settings may
; be set to a value that is slower than optimal.
;
; This startup file version does not support subclock.

#set      FREQ_4MHZ       D'4000000L
#set      FREQ_8MHZ       D'8000000L
#set      FREQ_16MHZ      D'16000000L

#set      CRYSTAL         FREQ_4MHZ  ; <<< select external crystal frequency


#set      CPU_4MHZ_MAIN_CLKP2_4MHZ            0x00000000L
#set      CPU_4MHZ_PLL_CLKP2_4MHZ             0x04040404L
#set      CPU_8MHZ_CLKP2_8MHZ                 0x08080808L
#set      CPU_12MHZ_CLKP2_12MHZ               0x0C0C0C0CL
#set      CPU_16MHZ_CLKP2_16MHZ               0x10101010L
#set      CPU_24MHZ_CLKP2_12MHZ               0x18180C18L
#set      CPU_24MHZ_CLKP2_16MHZ               0x18181018L
#set      CPU_24MHZ_CLKP2_24MHZ               0x18181818L
#set      CPU_32MHZ_CLKP2_16MHZ               0x20201020L
#set      CPU_32MHZ_CLKP1_16MHZ_CLKP2_16MHZ   0x20101020L

#set      CLOCK_SPEED     CPU_32MHZ_CLKP2_16MHZ  ; <<< set clock speeds

; If not specified seperately, the peripheral clock CLKP1 is set to
; the same frequency as the CPU.
; The peripheral clock CLKP2 has its own setting. This is because it
; feeds only the CAN controllers and Sound Generators. These do not
; need high frequency clocks.


; The BootROM is able to configure sub oscillator availability and mode
#set      SUBOSC_DISABLED     1      ; Sub oscillator is disabled, pins can be used as GPIO pins
#set      SUBOSC_CRYSTAL      2      ; Sub oscillator is enabled in "oscillation mode", connect crystal/resonator to X0A/X1A pins
#set      SUBOSC_EXT_CLOCK    3      ; Sub oscillator is enabled in "external clock input mode", connect ext. clock to X0A pin, X1A can bes used as GPIO

#set      SUBOSC_MODE         SUBOSC_DISABLED   ; <<< select sub oscillator mode


;====================================================================
; 4.9   Clock Stabilization Time
;====================================================================

; Main clock stabilization time is given in clock cycles, where
; MC_2_X_CYCLES means 2 to power of X cycles.

#set      MC_2_10_CYCLES   0
#set      MC_2_12_CYCLES   1
#set      MC_2_13_CYCLES   2
#set      MC_2_14_CYCLES   3
#set      MC_2_15_CYCLES   4
#set      MC_2_16_CYCLES   5
#set      MC_2_17_CYCLES   6
#set      MC_2_18_CYCLES   7

#set      MC_STAB_TIME     MC_2_15_CYCLES ; <<< select Main Clock Stabilization Time


;====================================================================
; 4.10  ROM Mirror configuration
;====================================================================

#set      MIRROR_8KB    0
#set      MIRROR_16KB   1
#set      MIRROR_24KB   2
#set      MIRROR_32KB   3

#set      ROMMIRROR     ON          ; <<< ROM mirror function ON/OFF
#set      MIRROR_BANK   0xF         ; <<< ROM Mirror bank, allowed entries: 0x0..0xF for the banks 0xF0..0xFF
#set      MIRROR_SIZE   MIRROR_32KB ; <<< ROM Mirror size

; One can select which ROM area to mirror into the upper half of bank 00.
; If ROMMIRROR = OFF is selected, the address range 0x008000..0x00FFFF
; shows the contents of the respective area of bank 1: 0x018000..0x01FFFF.
; If ROMMIRROR = ON is selected, the memory bank to mirror can be selected.
; Available banks are 0xF0 to 0xFF. Furthermore, the ROM Mirror area size can
; be selected. 4 sizes are available: 8 kB, 16 kB, 24 kB, or 32 kB. The ROM Mirror
; from the highest address of the selected bank downwards, e.g. if bank 0xFF and
; mirror size 24 kB is selected, the memory range 0xFFA000..0xFFFFFF is mirrored
; to address range 0x00A000..0x00FFFF. The memory area not selected for
; ROM Mirror is still mirrored from bank 0x01.
; This is necessary to get the compiler ROMCONST option working. This is intended
; to increase performance, if a lot of dynamic data have to be accessed.
; In SMALL and MEDIUM model these data can be accessed within bank 0,
; which allows to use near addressing. Please make sure to have the linker
; setting adjusted accordingly!


;====================================================================
; 4.11  Enable RAMCODE Copying
;====================================================================

#set      COPY_RAMCODE      OFF     ; <<< enable RAMCODE section to
                                    ; be copied from ROM to RAM

; To get this option properly working the code to be executed has to
; be linked to section RAMCODE (e.g. by #pragma section). The section
; RAMCODE has be located in RAM and the section @RAMCODE has to be
; located at a fixed address in ROM by linker settings.


;====================================================================
; 4.12  Enable information stamp in ROM
;====================================================================

#set      VERSION_STAMP     OFF     ; <<< enable version number in
                                    ; separated section


;====================================================================
; 4.13  Flash Security
;====================================================================
; All settings regarding Flash B are ignored on devices that do not
; have a Flash B.

#set      FLASH_A_SECURITY_ENABLE     OFF ; <<< enable Flash Security for Flash A
#set      FLASH_B_SECURITY_ENABLE     OFF ; <<< enable Flash Security for Flash B

; set the Flash Security unlock key (16 bytes)
; all 0: unlock not possible
#set      FLASH_A_UNLOCK_0           0x00
#set      FLASH_A_UNLOCK_1           0x00
#set      FLASH_A_UNLOCK_2           0x00
#set      FLASH_A_UNLOCK_3           0x00
#set      FLASH_A_UNLOCK_4           0x00
#set      FLASH_A_UNLOCK_5           0x00
#set      FLASH_A_UNLOCK_6           0x00
#set      FLASH_A_UNLOCK_7           0x00
#set      FLASH_A_UNLOCK_8           0x00
#set      FLASH_A_UNLOCK_9           0x00
#set      FLASH_A_UNLOCK_10          0x00
#set      FLASH_A_UNLOCK_11          0x00
#set      FLASH_A_UNLOCK_12          0x00
#set      FLASH_A_UNLOCK_13          0x00
#set      FLASH_A_UNLOCK_14          0x00
#set      FLASH_A_UNLOCK_15          0x00

#set      FLASH_B_UNLOCK_0           0x00
#set      FLASH_B_UNLOCK_1           0x00
#set      FLASH_B_UNLOCK_2           0x00
#set      FLASH_B_UNLOCK_3           0x00
#set      FLASH_B_UNLOCK_4           0x00
#set      FLASH_B_UNLOCK_5           0x00
#set      FLASH_B_UNLOCK_6           0x00
#set      FLASH_B_UNLOCK_7           0x00
#set      FLASH_B_UNLOCK_8           0x00
#set      FLASH_B_UNLOCK_9           0x00
#set      FLASH_B_UNLOCK_10          0x00
#set      FLASH_B_UNLOCK_11          0x00
#set      FLASH_B_UNLOCK_12          0x00
#set      FLASH_B_UNLOCK_13          0x00
#set      FLASH_B_UNLOCK_14          0x00
#set      FLASH_B_UNLOCK_15          0x00

;====================================================================
; 4.14  Flash Write Protection
;====================================================================
; All settings regarding sectors that do not exist on the particular
; device will have no effect.

#set      FLASH_A_WRITE_PROTECT           ON      ; <<< select Flash A write protection
#set      PROTECT_SECTOR_SAS              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA0              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA1              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA2              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA3              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA39             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA38             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA37             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA36             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA35             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA34             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA33             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SA32             OFF       ; <<< select individual sector to protect

; All settings regarding Flash B are ignored on devices that do not
; have a Flash B.
#set      FLASH_B_WRITE_PROTECT           OFF       ; <<< select Flash write protection
#set      PROTECT_SECTOR_SBS              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB0              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB1              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB2              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB3              OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB31             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB30             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB29             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB28             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB27             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB26             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB25             OFF       ; <<< select individual sector to protect
#set      PROTECT_SECTOR_SB24             OFF       ; <<< select individual sector to protect


;====================================================================
; 4.15  Boot Vector
;====================================================================

#set      BOOT_VECTOR_TABLE        1      ; boot vector setting in vector table
#set      BOOT_VECTOR_ALTERNATIVE  2      ; alternative boot vector

#set      BOOT_VECTOR              BOOT_VECTOR_TABLE   ; <<< select type of boot vector


; If boot vector generation is enabled (BOOT_VECTOR_TABLE, BOOT_VECTOR_ALTERNATIVE),
; appropriate code is generated. If it is disabled (OFF), start-up file does
; not care about.
;
;         BOOT_VECTOR_TABLE: - Create table entry at appropriate address of vector table.
;                            - Any start address can be set and start-up file will
;                              set address of this start code.
;   BOOT_VECTOR_ALTERNATIVE: - Instead of vector table entry, a special marker is set in
;                              ROM Configuration Block, which enables the alternative
;                              boot vector address programmed to 0xDF0040.
;                              This is prefered setting for user boot loaders.
;                       OFF: - Do not set table entry and marker. This might be used
;                              for application to be loaded by boot loader.
;
; Note
; BOOT_VECTOR_TABLE setting can also be used, if all other interrupt vectors
; are specified via "pragma intvect". Only if interrupts 0..7 are specified
; via "pragma intvect", these will conflict with the vector in this module.
; The reason is the INTVECT section, which includes the whole area from the
; lowest to the highest specified vector.


;====================================================================
; 4.16  Vector table base register configuration
;====================================================================

; The BootROM is able to set the vector table base register (TBR) that
; defines the memory area where the interrupt vectors are located

#set      TBR_INIT_VALUE         0xFFFF   ; <<< define TBR value (address bits [23:8])

; Note that address bits [9:8] are always treated as '0', which means
; that 0xFFFF will also locate the vector table to its default address 0xFF:FC00

; ATTENTION: Linker must place INTVECT section to the appropriate address


;====================================================================
; 4.17  UART scanning
;====================================================================

#set      UART_SCANNING   OFF        ; <<< enable UART scanning in
                                     ;     Internal Vector Mode
;
; By default, the MCU scans in Internal Vector Mode for a UART
; communication after reset. This enables to establish a serial
; communication without switching to Serial Communication Mode.
; For the final application, set this switch to OFF to achieve the
; fastest start-up time.


;====================================================================
; 4.18  On-chip debug system configuration
;====================================================================
#set      DEBUG_SECURITY_ENABLE     OFF           ; <<< enable Debug Security

; Set the debug security password length 1-14 (number of 16-bit words)
; only evaluated if DEBUG_SECURITY_ENABLE is ON, otherwise a length of 0 will be automatically chosen
#set      DEBUG_SECURITY_PASSWORD_LENGTH     14   ; <<< set Debug Security password length

; Set the debug security password
#set      DEBUG_SECURITY_PASSWORD_1    0x0000
#set      DEBUG_SECURITY_PASSWORD_2    0x0000
#set      DEBUG_SECURITY_PASSWORD_3    0x0000
#set      DEBUG_SECURITY_PASSWORD_4    0x0000
#set      DEBUG_SECURITY_PASSWORD_5    0x0000
#set      DEBUG_SECURITY_PASSWORD_6    0x0000
#set      DEBUG_SECURITY_PASSWORD_7    0x0000
#set      DEBUG_SECURITY_PASSWORD_8    0x0000
#set      DEBUG_SECURITY_PASSWORD_9    0x0000
#set      DEBUG_SECURITY_PASSWORD_10   0x0000
#set      DEBUG_SECURITY_PASSWORD_11   0x0000
#set      DEBUG_SECURITY_PASSWORD_12   0x0000
#set      DEBUG_SECURITY_PASSWORD_13   0x0000
#set      DEBUG_SECURITY_PASSWORD_14   0x0000




; Set the on-chip debug system boot mode
#set      OCDS_BOOT_NORMAL          1      ; normal boot with handshaking, mode decided by MDI init sequence
#set      OCDS_BOOT_FAST_FREERUN    2      ; fast boot, don't wait for end of MDI init sequence, start in free-run mode
#set      OCDS_BOOT_FAST_EMULATION  3      ; fast boot, don't wait for end of MDI init sequence, start in emulation mode

#set      OCDS_BOOT_MODE            OCDS_BOOT_NORMAL   ; <<< select OCDS boot mode
; Note, it is recommended to choose the OCDS_BOOT_FAST_FREERUN mode for the final application in order to speed up the boot phase


; Set on-chip debug system pin mode
#set      OCDS_DEBUG_PIN_MDI        1      ; debug pin enabled
#set      OCDS_DEBUG_PIN_SOFTGPIO   2      ; debug pin temporarily disabled, but can be enabled by the application
#set      OCDS_DEBUG_PIN_GPIO       3      ; debug pin permanently disabled, no debugging possible any more (strongest security)

#set      OCDS_DEBUG_PIN_MODE       OCDS_DEBUG_PIN_MDI   ; <<< select OCDS debug pin mode
; Note, while debug pin is disabled it can be used as a GPIO pin


;====================================================================
; 4.19  NMI / WAKE configuration
;====================================================================

; The BootROM is able to setup and enable the NMI pin,
; so that the NMI can be active during the whole execution of the user code
#set      NMIPIN_MODE_DISABLED                 1  ; BootROM does not enable the NMI function
#set      NMIPIN_MODE_NMI_FUNCTION_LOW_ACT     2  ; BootROM enables the low active NMI function
#set      NMIPIN_MODE_NMI_FUNCTION_HIGH_ACT    3  ; BootROM enables the high active NMI function


#set      NMIPIN_MODE             NMI_MODE_DISABLED   ; <<< select mode of the NMI pin

; The BootROM is able to relocate the NMI pin
#set      NMIPIN_RELOCATION       OFF    ; <<< select NMI pin relocation

; Note, the relocation setting is device specific, if the selected target MCU 
; does not support NMI relocation a pre-processor error will be generated


;====================================================================
; 4.20  Low voltage detection configuration
;====================================================================

; The BootROM is able to lock the low voltage detector which means that
; low voltage detection and corresponding reset is always enabled and
; user code cannot change the setting by accident
#set      LVD_LOCK            OFF   ; <<< select locking of LVD state


;====================================================================
; 4.21  Watchdog timer interval configuration
;====================================================================

; BootROM can change the default Watchdog timer interval

; Watchdog timer interval is given in clock cycles, where
; WT_2_X_CYCLES means 2 to power of X cycles of Watchdog timer clock.

#set      WT_2_8_CYCLES    0
#set      WT_2_9_CYCLES    1
#set      WT_2_10_CYCLES   2
#set      WT_2_11_CYCLES   3
#set      WT_2_12_CYCLES   4
#set      WT_2_13_CYCLES   5
#set      WT_2_14_CYCLES   6
#set      WT_2_15_CYCLES   7
#set      WT_2_16_CYCLES   8
#set      WT_2_17_CYCLES   9
#set      WT_2_18_CYCLES   10
#set      WT_2_19_CYCLES   11
#set      WT_2_20_CYCLES   12
#set      WT_2_21_CYCLES   13
#set      WT_2_22_CYCLES   14
#set      WT_2_23_CYCLES   15

#set      WT_INTERVAL     WT_2_23_CYCLES    ; <<< select Watchdog timer interval


; <<< END OF SETTINGS >>>

;====================================================================
; 5  Section and Data Declaration
;====================================================================

;====================================================================
; 5.1   Several fixed addresses (fixed for MB966xx controllers)
;====================================================================

ROMM       .EQU      0x03AE          ; ROM mirror control register
CKSR       .EQU      0x0401          ; Clock selection register
CKSSR      .EQU      0x0402          ; Clock stabilization select register
CKMR       .EQU      0x0403          ; Clock monitor register
CKFCR      .EQU      0x0404          ; Clock frequency control register
PLLCR      .EQU      0x0406          ; PLL control register


;====================================================================
; 5.2   Set ROM Configuration for Boot Vector
;====================================================================
#set VECTOR_TABLE_BASE_ADDRESS    ((TBR_INIT_VALUE & 0xFFFC) << 8)

#if BOOT_VECTOR == BOOT_VECTOR_TABLE
          .SECTION        BOOT_SELECT, CONST, LOCATE=H'DF0030
          .DATA.L 0xFFFFFFFF
          .SECTION        RESVECT, CONST, LOCATE=(VECTOR_TABLE_BASE_ADDRESS + 0x3DC)
          .DATA.E _start
          
#elif BOOT_VECTOR == BOOT_VECTOR_ALTERNATIVE
          .SECTION        BOOT_SELECT, CONST, LOCATE=H'DF0030
          .DATA.L 0x292D3A7B        ; "Magic Word"
          .SECTION        ALTRESVECT, CONST, LOCATE=H'DF0040
          .DATA.E _start
#else
          .SECTION        BOOT_SELECT, CONST, LOCATE=H'DF0030
          .SKIP   4
#endif

;====================================================================
; 5.3   Set ROM configuration for UART Scanning
;====================================================================


#if UART_SCANNING == ON
          .SECTION        UART_SCAN_SELECT, CONST, LOCATE=H'DF0034
          .DATA.L 0xFFFFFFFF
#else
          .SECTION        UART_SCAN_SELECT, CONST, LOCATE=H'DF0034
          .DATA.L 0x292D3A7B        ; Deactivation "Magic Word"
#endif



;====================================================================
; 5.4   Set Flash Security
;====================================================================

#set FLASH_B_AVAILABLE  0;                                      \
  ; (                                                           \
    ; (                                                         \
      ; (SERIES == MB96340) &&                                  \
      ; (                                                       \
        ; (DEVICE == MB96348HxA) ||                             \
        ; (DEVICE == MB96348TxA) ||                             \
        ; (DEVICE == MB96348CxA) ||                             \
        ; (DEVICE == MB96348HxB) ||                             \
        ; (DEVICE == MB96348TxB) ||                             \
        ; (DEVICE == MB96348HxC) ||                             \
        ; (DEVICE == MB96348TxC) ||                             \
        ; (DEVICE == MB96348CxC)                                \
      ; )                                                       \
    ; )                                                         \
    ; ||                                                        \
    ; (SERIES == MB96370)                                       \
    ; ||                                                        \
    ; (                                                         \
      ; (SERIES == MB96380) &&                                  \
      ; (                                                       \
        ; (DEVICE == MB96388HxB) ||                             \
        ; (DEVICE == MB96389RxB)                                \
      ; )                                                       \
    ; )                                                         \
  ; )


        
          .SECTION FLASH_A_SECURITY, CONST, LOCATE=H'DF0000
#if FLASH_A_SECURITY_ENABLE == OFF
	      .DATA.W 0x6666 ; Security DISABLED (mandatory for mask ROM)
	      .SKIP   16
#else FLASH_A_SECURITY_ENABLE == ON
	      .DATA.W 0x9999 ; Security ENABLED
	      .DATA.W ((FLASH_A_UNLOCK_1  << 8) | FLASH_A_UNLOCK_0)
	      .DATA.W ((FLASH_A_UNLOCK_3  << 8) | FLASH_A_UNLOCK_2)
	      .DATA.W ((FLASH_A_UNLOCK_5  << 8) | FLASH_A_UNLOCK_4)
	      .DATA.W ((FLASH_A_UNLOCK_7  << 8) | FLASH_A_UNLOCK_6)
	      .DATA.W ((FLASH_A_UNLOCK_9  << 8) | FLASH_A_UNLOCK_8)
	      .DATA.W ((FLASH_A_UNLOCK_11 << 8) | FLASH_A_UNLOCK_10)
	      .DATA.W ((FLASH_A_UNLOCK_13 << 8) | FLASH_A_UNLOCK_12)
	      .DATA.W ((FLASH_A_UNLOCK_15 << 8) | FLASH_A_UNLOCK_14)
#endif

	      
#if FLASH_B_AVAILABLE == ON
          .SECTION FLASH_B_SECURITY, CONST, LOCATE=H'DE0000
#  if FLASH_B_SECURITY_ENABLE == OFF
	      .DATA.W 0x6666 ; Security DISABLED (mandatory for mask ROM)
	      .SKIP   16
#  else FLASH_B_SECURITY_ENABLE == ON
	      .DATA.W 0x9999 ; Security ENABLED
	      .DATA.W ((FLASH_B_UNLOCK_1  << 8) | FLASH_B_UNLOCK_0)
	      .DATA.W ((FLASH_B_UNLOCK_3  << 8) | FLASH_B_UNLOCK_2)
	      .DATA.W ((FLASH_B_UNLOCK_5  << 8) | FLASH_B_UNLOCK_4)
	      .DATA.W ((FLASH_B_UNLOCK_7  << 8) | FLASH_B_UNLOCK_6)
	      .DATA.W ((FLASH_B_UNLOCK_9  << 8) | FLASH_B_UNLOCK_8)
	      .DATA.W ((FLASH_B_UNLOCK_11 << 8) | FLASH_B_UNLOCK_10)
	      .DATA.W ((FLASH_B_UNLOCK_13 << 8) | FLASH_B_UNLOCK_12)
	      .DATA.W ((FLASH_B_UNLOCK_15 << 8) | FLASH_B_UNLOCK_14)
#  endif
	      
#endif ; FLASH_B_AVAILABLE == ON

;====================================================================
; 5.5   Set Flash write protection
;====================================================================

          .SECTION FLASH_A_PROTECT, CONST, LOCATE=H'DF001C
#if FLASH_A_WRITE_PROTECT == ON
          .DATA.L 0x292D3A7B
          .DATA.B ~((PROTECT_SECTOR_SA3 << 4) | (PROTECT_SECTOR_SA2 << 3) | (PROTECT_SECTOR_SA1 << 2) | (PROTECT_SECTOR_SA0 << 1) | PROTECT_SECTOR_SAS)
          .SKIP   3
          .DATA.B ~((PROTECT_SECTOR_SA39 << 7) | (PROTECT_SECTOR_SA38 << 6) | (PROTECT_SECTOR_SA37 << 5) | (PROTECT_SECTOR_SA36 << 4) | (PROTECT_SECTOR_SA35 << 3) | (PROTECT_SECTOR_SA34 << 2) | (PROTECT_SECTOR_SA33 << 1) | PROTECT_SECTOR_SA32)
          .SKIP   1
#else
          .DATA.L 0xFFFFFFFF
          .SKIP   6
#endif ; FLASH_A_WRITE_PROTECT


#if FLASH_B_AVAILABLE == ON
          .SECTION FLASH_B_PROTECT, CONST, LOCATE=H'DE001C
#  if FLASH_B_WRITE_PROTECT == ON
          .DATA.L 0x292D3A7B
          .DATA.B ~((PROTECT_SECTOR_SB3 << 4) | (PROTECT_SECTOR_SB2 << 3) | (PROTECT_SECTOR_SB1 << 2) | (PROTECT_SECTOR_SB0 << 1) | PROTECT_SECTOR_SBS)
          .SKIP   3
          .DATA.B ~((PROTECT_SECTOR_SB31 << 7) | (PROTECT_SECTOR_SB30 << 6) | (PROTECT_SECTOR_SB29 << 5) | (PROTECT_SECTOR_SB28 << 4) | (PROTECT_SECTOR_SB27 << 3) | (PROTECT_SECTOR_SB26 << 2) | (PROTECT_SECTOR_SB25 << 1) | PROTECT_SECTOR_SB24)
          .SKIP   1
#  else
          .DATA.L 0xFFFFFFFF
          .SKIP   6
#  endif ; FLASH_B_WRITE_PROTECT

#endif ; FLASH_B_AVAILABLE == ON

;====================================================================
; 5.6   Set ROM configuration for OCDS
;====================================================================

#if (DEBUG_SECURITY_ENABLE == ON) && ((DEBUG_SECURITY_PASSWORD_LENGTH > 14) || (DEBUG_SECURITY_PASSWORD_LENGTH < 1))
#  error DEBUG_SECURITY_PASSWORD_LENGTH out of range
#endif


          .SECTION        OCDS_BOOT_SELECT, CONST, LOCATE=H'DF0056

#if OCDS_BOOT_MODE == OCDS_BOOT_FAST_FREERUN
          .DATA.B 0x55
#elif OCDS_BOOT_MODE == OCDS_BOOT_FAST_EMULATION
          .DATA.B 0xAA
#else
          .DATA.B 0xFF
#endif

          .SECTION OCDS_SECURITY, CONST, LOCATE=H'DF0058

#if DEBUG_SECURITY_ENABLE == ON
          .DATA.W DEBUG_SECURITY_PASSWORD_LENGTH
          .DATA.W DEBUG_SECURITY_PASSWORD_1
          .DATA.W DEBUG_SECURITY_PASSWORD_2
          .DATA.W DEBUG_SECURITY_PASSWORD_3
          .DATA.W DEBUG_SECURITY_PASSWORD_4
          .DATA.W DEBUG_SECURITY_PASSWORD_5
          .DATA.W DEBUG_SECURITY_PASSWORD_6
          .DATA.W DEBUG_SECURITY_PASSWORD_7
          .DATA.W DEBUG_SECURITY_PASSWORD_8
          .DATA.W DEBUG_SECURITY_PASSWORD_9
          .DATA.W DEBUG_SECURITY_PASSWORD_10
          .DATA.W DEBUG_SECURITY_PASSWORD_11
          .DATA.W DEBUG_SECURITY_PASSWORD_12
          .DATA.W DEBUG_SECURITY_PASSWORD_13
          .DATA.W DEBUG_SECURITY_PASSWORD_14
#else
          .DATA.W 0x00
          .SKIP   (14 * 2)
#endif

          .SECTION        OCDS_PINMODE_SELECT, CONST, LOCATE=H'DF0076

#if OCDS_DEBUG_PIN_MODE == OCDS_DEBUG_PIN_SOFTGPIO
          .DATA.B 0x82
          .DATA.B 0xB8
          .DATA.B 0xDB
          .DATA.B 0x6B
          .DATA.B 0x71
          .DATA.B 0xCE
          .DATA.B 0x39
          .DATA.B 0x84
          .DATA.B 0x53
          .DATA.B 0x58
          .DATA.B 0xE3
          .DATA.B 0x25
          .DATA.B 0x1B
          .DATA.B 0x63
          .DATA.B 0x7C
          .DATA.B 0x96
          .DATA.B 0xE4
          .DATA.B 0x36
#elif OCDS_DEBUG_PIN_MODE == OCDS_DEBUG_PIN_GPIO
          .DATAB.B 18, 0x00
#else
          .DATAB.B 18, 0xFF
#endif




;====================================================================
; 5.7   Set ROM configuration for NMI
;====================================================================

          .SECTION        NMIRELOC_SELECT, CONST, LOCATE=H'DF0026


#if NMI_RELOCATION == ON
#  if (SERIES == MB96670) || (SERIES == MB96680)
#    error NMI relocation is not possible for the selected device series.
#  else
          .DATA.B 0x55
#  endif
#else
          .DATA.B 0xFF
#endif

          .SECTION        NMIPINMODE_SELECT, CONST, LOCATE=H'DF0027
#if NMIPIN_MODE == NMIPIN_MODE_NMI_FUNCTION_LOW_ACT
          .DATA.B 0x55
#elif NMIPIN_MODE == NMIPIN_MODE_NMI_FUNCTION_HIGH_ACT
          .DATA.B 0xAA
#else ; NMI disabled
          .DATA.B 0xFF
          .DATA.B 0xFF
          .DATA.B 0xFF
#endif


;====================================================================
; 5.8   Set ROM configuration for Table Base Register (TBR)
;====================================================================

          .SECTION        TBR_SELECT, CONST, LOCATE=H'DF002A

          .DATA.W TBR_INIT_VALUE


;====================================================================
; 5.9   Set ROM configuration for sub oscillator
;====================================================================

          .SECTION        SUBOSC_SELECT, CONST, LOCATE=H'DF003A

#if SUBOSC_MODE == SUBOSC_CRYSTAL
          .DATA.B 0x55
#elif SUBOSC_MODE == SUBOSC_EXT_CLOCK
          .DATA.B 0xAA
#else
          .DATA.B 0xFF
#endif


;====================================================================
; 5.10  Set ROM configuration for low voltage detection
;====================================================================

          .SECTION        LVD_SELECT, CONST, LOCATE=H'DF003B

#if LVD_LOCK == ON
          .DATA.B 0x55
#else
          .DATA.B 0xFF
#endif


;====================================================================
; 5.11  Set ROM configuration for watchdog interval
;====================================================================

          .SECTION        WTINTERVAL_SELECT, CONST, LOCATE=H'DF003C
          
          .DATA.B WT_INTERVAL


;====================================================================
; 5.12  Store version information
;====================================================================

#if VERSION_STAMP == ON
          .SECTION  VERSIONS, CONST ; change name, if necessary
          .SDATA    "Start ", VERSION, "\n\0"
#endif

;====================================================================
; 5.13  Declaration of __near addressed data sections
;====================================================================

; sections to be cleared
          .SECTION  DATA,      DATA,   ALIGN=2  ; zero clear area
          .SECTION  DATA2,     DATA,   ALIGN=2  ; zero clear area
          .SECTION  DIRDATA,   DIR,    ALIGN=2  ; zero clear direct
          .SECTION  LIBDATA,   DATA,   ALIGN=2  ; zero clear lib area

; sections to be initialised with start-up values
          .SECTION  INIT,      DATA,   ALIGN=2  ; initialised area
          .SECTION  INIT2,     DATA,   ALIGN=2  ; initialised area
          .SECTION  DIRINIT,   DIR,    ALIGN=2  ; initialised dir
          .SECTION  LIBINIT,   DATA,   ALIGN=2  ; initialised lib area
#if CONSTDATA == RAMCONST
          .SECTION  CINIT,     DATA,   ALIGN=2  ; initialised const
          .SECTION  CINIT2,    DATA,   ALIGN=2  ; initialised const
#endif

; sections containing start-up values for initialised sections above
          .SECTION  DCONST,    CONST,  ALIGN=2  ; DINIT initialisers
          .SECTION  DIRCONST, DIRCONST,ALIGN=2  ; DIRINIT initialisers
          .SECTION  LIBDCONST, CONST,  ALIGN=2  ; LIBDCONST init val

          ; following section is either copied to CINIT (RAMCONST) or
          ; mapped by ROM-mirror function (ROMCONST)
          .SECTION  CONST,     CONST,  ALIGN=2  ; CINIT initialisers
          .SECTION  CONST2,    CONST,  ALIGN=2  ; CINIT initialisers

;====================================================================
; 5.14  Declaration of RAMCODE section and labels
;====================================================================

#if COPY_RAMCODE == ON
          .SECTION  RAMCODE,   CODE,  ALIGN=1
          .IMPORT _RAM_RAMCODE                  ; provided by linker
          .IMPORT _ROM_RAMCODE                  ; provided by linker
#endif


;====================================================================
; 5.15  Declaration of sections containing other sections description
;====================================================================

; DCLEAR contains start address and size of all sections to be cleared
; DTRANS contains source and destination address and size of all
; sections to be initialised with start-up values
; The compiler automatically adds a descriptor for each __far addressed
; data section to DCLEAR or DTRANS. These __far sections are separated
; for each C-module.

; In addition the start-up file adds the descriptors of the previously
; declared __near section here. This way the same code in the start-up
; file can be used for initialising all sections.

   .SECTION  DCLEAR,    CONST,  ALIGN=2  ; zero clear table
   ;    Address         Bank            Size
   .DATA.H DATA,    BNKSEC DATA,    SIZEOF(DATA   )
   .DATA.H DIRDATA, BNKSEC DIRDATA, SIZEOF(DIRDATA)
   .DATA.H LIBDATA, BNKSEC LIBDATA, SIZEOF(LIBDATA)

   .SECTION  DTRANS,    CONST,  ALIGN=2  ; copy table
   ;    Address         Bank               Address     Bank          Size
   .DATA.H DCONST,   BNKSEC DCONST,   INIT,   BNKSEC INIT,   SIZEOF INIT
   .DATA.H DIRCONST, BNKSEC DIRCONST, DIRINIT,BNKSEC DIRINIT,SIZEOF DIRINIT
   .DATA.H LIBDCONST,BNKSEC LIBDCONST,LIBINIT,BNKSEC LIBINIT,SIZEOF LIBINIT

#if CONSTDATA == RAMCONST
   .DATA.H CONST,    BNKSEC CONST,    CINIT,  BNKSEC CINIT,  SIZEOF CINIT
   .DATA.H CONST2,   BNKSEC CONST,    CINIT2, BNKSEC CINIT2, SIZEOF CINIT2
#endif

#if COPY_RAMCODE == ON
   .DATA.L _ROM_RAMCODE, _RAM_RAMCODE
   .DATA.H SIZEOF RAMCODE
#endif

;====================================================================
; 5.16  Stack area and stack top definition/declaration
;====================================================================
#if STACK_RESERVE == ON
            .SECTION  SSTACK, STACK, ALIGN=2

            .EXPORT __systemstack, __systemstack_top
__systemstack:
            .RES.B    (STACK_SYS_SIZE + 1) & 0xFFFE
__systemstack_top:
SSTACK_TOP:

            .SECTION  USTACK, STACK, ALIGN=2

            .EXPORT __userstack, __userstack_top
__userstack:
            .RES.B    (STACK_USR_SIZE + 1) & 0xFFFE
__userstack_top:
USTACK_TOP:

#else
            .SECTION  SSTACK, STACK, ALIGN=2
            .SECTION  USTACK, STACK, ALIGN=2
#endif

;====================================================================
; 5.17  Direct page register dummy label definition
;====================================================================

          .SECTION  DIRDATA  ; zero clear direct
DIRDATA_S:                                      ; label for DPR init

; This label is used to get the page of the __direct data.
; Depending on the linkage order of this startup file the label is
; placed anywhere within the __direct data page. However, the
; statement "PAGE (DIRDATA_S)" is processed. Therefore, the lower
; 8 Bit of the address of DIRDATA_S are not relevant and this feature
; becomes linkage order independent.
; Note, the linker settings have to make sure that all __direct
; data are located within the same physical page (256 Byte block).



;====================================================================
; 6  Start-Up Code
;====================================================================

;====================================================================
; 6.1   Import external symbols
;====================================================================

          .IMPORT   _main                    ; user code entrance
#if CLIBINIT == ON
          .IMPORT   __stream_init
          .IMPORT   _exit
          .EXPORT   __exit
#endif
          .EXPORT   _start

;====================================================================
;   ___  _____   __    ___  _____
;  /       |    /  \  |   \   |
;  \___    |   |    | |___/   |
;      \   |   |----| |  \    |
;   ___/   |   |    | |   \   |      Begin of actual code section
;
;====================================================================
          .SECTION  CODE_START, CODE, ALIGN=1

;====================================================================
; 6.2   Program start (the reset vector should point here)
;====================================================================
_start:
          NOP  ; This NOP is only for debugging. On debugger the IP
               ; (instruction pointer) should point here after reset

;====================================================================
; 6.3   Initialisation of processor status
;====================================================================
          AND  CCR, #0x80          ; disable interrupts
          MOV  ILM,#7              ; set interrupt level mask to ALL
          MOV  RP,#REGBANK         ; set register bank pointer

;====================================================================
; 6.4   Set clock ratio (ignore subclock)
;====================================================================

          MOVN A, #0               ; set bank 0 in DTB for the case that
          MOV  DTB, A              ; start-up code was not jumped by reset
		
          MOV  CKSSR, #(0x38 | MC_STAB_TIME)  ; set clock stabilization time

#set CLOCK_RC     0     ; RC clock is input to clock divider
#set CLOCK_MAIN   1     ; MAIN clock is input to clock divider
#set CLOCK_PLL    2     ; PLL clock is input to clock divider
#set CLOCK_SUB    3     ; SUB clock is input to clock divider

; select default source for CLKB and CLKP1
#set CLKB_CLKP1_SOURCE  CLOCK_PLL
; select default source for CLKP2
#set CLKP2_SOURCE       CLOCK_PLL


;                          ++-----    PLL multiplier (CLKS1, CLKS2)
;                          ||++----   VCO clock multiplier (PLL internal)
;                          ||||+---   CLKB divider (CPU clock)
;                          |||||+--   CLKP1 divider (peripherals except CAN, Sound, USB)
;                          ||||||+-   CLKP2 divider (CAN, Sound)
;                          |||||||
#if CLOCK_SPEED == CPU_4MHZ_MAIN_CLKP2_4MHZ
#  set   CLK_PARAMS_4MHZ 0x0110111   ; CLKS1=CLKS2= 4MHZ, CLK_VCO=64MHZ
#  set   CLK_PARAMS_8MHZ 0x010C222   ; CLKS1=CLKS2= 8MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0106444   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ
#  set CLKB_CLKP1_SOURCE CLOCK_MAIN  ; select MAIN clock as source for CLKB, CLKP1 divider
#  set CLKP2_SOURCE      CLOCK_MAIN  ; select MAIN clock as source for CLKP2, CLKP3 divider

#elif CLOCK_SPEED == CPU_4MHZ_PLL_CLKP2_4MHZ
#  set   CLK_PARAMS_4MHZ 0x0110111   ; CLKS1=CLKS2= 4MHZ, CLK_VCO=64MHZ
#  set   CLK_PARAMS_8MHZ 0x010C222   ; CLKS1=CLKS2= 8MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0106444   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_8MHZ_CLKP2_8MHZ
#  set   CLK_PARAMS_4MHZ 0x020C111   ; CLKS1=CLKS2= 8MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x010C111   ; CLKS1=CLKS2= 8MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0106222   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_12MHZ_CLKP2_12MHZ
#  set   CLK_PARAMS_4MHZ 0x0308111   ; CLKS1=CLKS2=12MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x0304222   ; CLKS1=CLKS2=24MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0302444   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_16MHZ_CLKP2_16MHZ
#  set   CLK_PARAMS_4MHZ 0x0406111   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x0206111   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0106111   ; CLKS1=CLKS2=16MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_24MHZ_CLKP2_12MHZ
#  set   CLK_PARAMS_4MHZ 0x0604112   ; CLKS1=CLKS2=24MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x0304112   ; CLKS1=CLKS2=24MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0302224   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_24MHZ_CLKP2_16MHZ
#  set   CLK_PARAMS_4MHZ 0x0C02223   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x0602223   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0302223   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_24MHZ_CLKP2_24MHZ
#  set   CLK_PARAMS_4MHZ 0x0604111   ; CLKS1=CLKS2=24MHZ, CLK_VCO=96MHZ
#  set   CLK_PARAMS_8MHZ 0x0304111   ; CLKS1=CLKS2=24MHZ, CLK_VCO=96MHZ
#  set  CLK_PARAMS_16MHZ 0x0302222   ; CLKS1=CLKS2=48MHZ, CLK_VCO=96MHZ

#elif CLOCK_SPEED == CPU_32MHZ_CLKP2_16MHZ
#  set   CLK_PARAMS_4MHZ 0x0802112   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ
#  set   CLK_PARAMS_8MHZ 0x0402112   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ
#  set  CLK_PARAMS_16MHZ 0x0202112   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ

#elif CLOCK_SPEED == CPU_32MHZ_CLKP1_16MHZ_CLKP2_16MHZ
#  set   CLK_PARAMS_4MHZ 0x0802122   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ
#  set   CLK_PARAMS_8MHZ 0x0402122   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ
#  set  CLK_PARAMS_16MHZ 0x0202122   ; CLKS1=CLKS2=32MHZ, CLK_VCO=64MHZ

#else
#  error The selected setting of "CLOCK_SPEED" is not supported.
#endif

#if CRYSTAL == FREQ_4MHZ
#  set CLK_PARAMS    CLK_PARAMS_4MHZ
#elif CRYSTAL == FREQ_8MHZ
#  set CLK_PARAMS    CLK_PARAMS_8MHZ
#elif CRYSTAL == FREQ_16MHZ
#  set CLK_PARAMS    CLK_PARAMS_16MHZ
#else
#  error specified crystal frequency ("CRYSTAL") is not supported by this file.
#endif

#set      FREQ_1MHZ       D'1000000L

;====================================================================
; macros to evaluate parameter settings

#define PLL_MULTIPLIER(params) \
  ((params & 0xFF00000L) >> 20)

#define VCO_MULTIPLIER(params) \
  ((params & 0x00FF000L) >> 12)
  
#define CLKB_DIVIDER(params) \
  ((params & 0x0000F00L) >> 8)

#define CLKP1_DIVIDER(params) \
  ((params & 0x00000F0L) >> 4)

#define CLKP2_DIVIDER(params) \
  ((params & 0x000000FL) >> 0)



#define  CLKVCO(params)\
  (CRYSTAL * PLL_MULTIPLIER(params) * VCO_MULTIPLIER(params))

#define CLKS1_CLKS2(params) \
  (CRYSTAL * PLL_MULTIPLIER(params))

#define CLKB(params) \
  (CLKS1_CLKS2(params) / CLKB_DIVIDER(params))

#define CLKP1(params) \
  (CLKS1_CLKS2(params) / CLKP1_DIVIDER(params))

#if (CLKP2_SOURCE == CLOCK_PLL)
#  define CLKP2(params) \
     (CLKS1_CLKS2(params) / CLKP2_DIVIDER(params))
#elif (CLKP2_SOURCE == CLOCK_MAIN)
#  define CLKP2(params) \
     (CRYSTAL / CLKP2_DIVIDER(params))
#else
#  error Clock source for CLKP2 is not supported.
#endif


;====================================================================
; protect all devices from common illegal settings


#if (CLKS1_CLKS2(CLK_PARAMS) > (54 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (CLKB(CLK_PARAMS) > (32 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (CLKP1(CLK_PARAMS) > (32 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (CLKP2(CLK_PARAMS) > (32 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif

#if (CLKVCO(CLK_PARAMS) < (54 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (CLKVCO(CLK_PARAMS) > (108 * FREQ_1MHZ))
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (VCO_MULTIPLIER(CLK_PARAMS) & 1) ; VCO multiplier must be even!
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif
#if (VCO_MULTIPLIER(CLK_PARAMS) > 16)
#  error The current clock setting ("CLOCK_SPEED") is not allowed for the selected device.
#endif




;====================================================================
; define register settings

#define PLLCR_SETTING(params)                                                  \
     (                                                                         \
      (((VCO_MULTIPLIER(params) - 2) / 2) << 5)                                \
      |                                                                        \
      ((PLL_MULTIPLIER(params) - 1) << 0)                                      \
     )

#define CKFCR_SETTING(params)                                                  \
  (                                                                            \
    ((CLKP2_DIVIDER(params) - 1) << 12)                                        \
    |                                                                          \
    ((CLKP1_DIVIDER(params) - 1) <<  8)                                        \
    |                                                                          \
    ((CLKB_DIVIDER(params)  - 1) <<  4)                                        \
    |                                                                          \
    1                                                                          \
  )

#define CKSR_SETTING(CLKS1_SOURCE, CLKS2_SOURCE)                               \
  (                                                                            \
    0xF0                                                                       \
    |                                                                          \
    ((CLKS2_SOURCE & 0x3)                                            << 2)     \
    |                                                                          \
    ((CLKS1_SOURCE & 0x3)                                            << 0)     \
  )

;====================================================================
; PLL startup sequence

          MOVW PLLCR, # PLLCR_SETTING(CLK_PARAMS)
          MOVW CKFCR, # CKFCR_SETTING(CLK_PARAMS)
          MOV  CKSR,  # CKSR_SETTING(CLKB_CLKP1_SOURCE, CLKP2_SOURCE)


;====================================================================
; 6.5   Set ROM mirror configuaration
;====================================================================

ROMM_CONFIG    .EQU     ((MIRROR_BANK << 4) | (MIRROR_SIZE << 1) | (ROMMIRROR))
           MOV  ROMM, #ROMM_CONFIG


;====================================================================
; 6.6   Prepare stacks and set the default stack type
;====================================================================

; Use word aligned stack section. Stack symbols are ignored.
#define SSTACK_L    ((SSTACK + 1) & ~1)
#define SSTACK_H    ((SSTACK + SIZEOF(SSTACK)) & ~1)
#define SSTACK_BNK  BNKSEC SSTACK

#define USTACK_L    ((USTACK + 1) & ~1)
#define USTACK_H    ((USTACK + SIZEOF(USTACK)) & ~1)
#define USTACK_BNK  BNKSEC USTACK

          AND  CCR,#H'DF            ; clear system stack flag
          MOVL A, #USTACK_H
          MOVW SP,A                 ; load offset of stack top to pointer
          MOV  A, #USTACK_BNK       ; load bank of stack start address to A
          MOV  USB, A               ; set bank

#if STACK_FILL == ON                ; preset the stack
          MOVL A, #(USTACK_L)       ; load start stack address to A
          MOVW A, #STACK_PATTERN    ; AL -> AH, pattern in AL
          MOVW RW0, #(USTACK_H - USTACK_L) / 2 ; get word count
; Compiler WARNING W1807A for the following string isntruction can be ignored, usage is safe at this location
          FILSWI    SPB             ; write pattern to stack
#endif

          OR   CCR,#H'20            ; set System stack flag
          MOVL A, #SSTACK_H
          MOVW SP,A                 ; load offset of stack top to pointer
          MOV  A, #SSTACK_BNK       ; load bank of stack start address to A
          MOV  SSB, A               ; set bank

#if STACK_FILL == ON                ; preset the stack
          MOVL A, #(SSTACK_L)       ; load start stack address to A
          MOVW A, #STACK_PATTERN    ; AL -> AH, pattern in AL
          MOVW RW0, #(SSTACK_H - SSTACK_L) / 2; get byte count
; Compiler WARNING W1807A for the following string isntruction can be ignored, usage is safe at this location
          FILSWI    SPB             ; write pattern to stack
#endif

#if STACKUSE == USRSTACK
          AND  CCR,#H'DF            ; clear system stack flag
#endif


;   The following macro is needed because of the AUTOMODEL option. If the
;   model is not known while assembling the module, one has to expect
;   completion of streaminit() by RET or RETP. Because RET removes 2 bytes
;   from stack and RETP removes 4 bytes from stack, SP is reloaded.

#macro RELOAD_SP

#  if STACKUSE == USRSTACK
          MOVW A, #USTACK_H
#  else
          MOVW A, #SSTACK_H
#  endif
          MOVW SP,A
#endm

;====================================================================
; 6.7   Copy initial values to data areas
;====================================================================
;
; Each C-module has its own __far INIT section. The names are generic.
; DCONST_module contains the initializers for the far data of the one
; module. INIT_module reserves the RAM area, which has to be loaded
; with the data from DCONST_module. ("module" is the name of the *.c
; file)
; All separated DCONST_module/INIT_module areas are described in
; DTRANS section by start addresses and length of each far section.
;   0000 1. source address (ROM)
;   0004 1. destination address (RAM)
;   0008 length of sections 1
;   000A 2. source address  (ROM)
;   000E 2. destination address (RAM)
;   0012 length of sections 2
;   0014 3. source address ...
; In addition the start-up file adds the descriptors of the __near
; sections to this table. The order of the descriptors in this table
; depends on the linkage order.
;
; Note: Sections cannot exceed bank boundaries.
;====================================================================
          MOVL A, #DTRANS          ; get address of table
          MOVL RL2, A              ; store address in RL2 (RW4/RW5)
          BRA  LABEL2              ; branch to loop condition
LABEL1:
          MOVW A, @RL2+6           ; get bank of destination
          MOV  DTB, A              ; save dest bank in DTB
          MOVW A, @RL2+2           ; get source bank
          MOV  ADB, A              ; save source bank in ADB
          MOVW A, @RL2+8           ; number of bytes to copy -> A
          MOVW RW0, A              ; number of bytes to copy -> RW0
          MOVW A, @RL2+4           ; move destination addr in AL
          MOVW A, @RL2             ; AL -> AH, src addr -> AL
; Compiler WARNING W1807A for the following string isntruction can be ignored, usage is safe at this location
          MOVSI DTB, ADB           ; copy data
          MOVN A, #10              ; length of one table entry is 10
          ADDW RW4, A              ; set pointer to next table entry
LABEL2:
          MOVW A, RW4              ; get address of next block
          CMPW A, #DTRANS + SIZEOF (DTRANS) ; all blocks processed ?
          BNE  LABEL1              ; if not, branch

;====================================================================
; 6.8   Clear uninitialized data areas to zero
;====================================================================
;
; Each C-module has its own __far DATA section. The names are generic.
; DATA_module contains the reserved area (RAM) to be cleared.
; ("module" is the name of the *.c file)
; All separated DATA_module areas are described in DCLEAR section by
; start addresses and length of all far section.
;   0000 1. section address (RAM)
;   0004 length of section 1
;   0006 2. section address (RAM)
;   000A length of section 2
;   000C 3. section address (RAM)
;   0010 length of section 3 ...
; In addition the start-up file adds the descriptors of the __near
; sections to this table. The order of the descriptors in this table
; depends on the linkage order.
;====================================================================
          MOV  A, #BNKSEC DCLEAR   ; get bank of table
          MOV  DTB, A              ; store bank in DTB
          MOVW RW1, #DCLEAR        ; get start offset of table
          BRA  LABEL4              ; branch to loop condition
LABEL3:
          MOV  A, @RW1+2           ; get section bank
          MOV  ADB, A              ; save section bank in ADB
          MOVW RW0, @RW1+4         ; number of bytes to copy -> RW0
          MOVW A, @RW1             ; move section addr in AL
          MOVN A, #0               ; AL -> AH, init value -> AL
; Compiler WARNING W1807A for the following string isntruction can be ignored, usage is safe at this location
          FILSI     ADB            ; write 0 to section
          MOVN A, #6               ; length of one table entry is 6
          ADDW RW1, A              ; set pointer to next table entry
LABEL4:
          MOVW A, RW1              ; get address of next block
          SUBW A, #DCLEAR          ; sub address of first block
          CMPW A, #SIZEOF (DCLEAR) ; all blocks processed ?
          BNE  LABEL3              ; if not, branch



;====================================================================
; 6.9   Set Data Bank Register (DTB) and Direct Page Register (DPR)
;====================================================================
          MOV  A,#BNKSEC DATA          ; User data bank offset
          MOV  DTB,A

          MOV  A,#PAGE DIRDATA_S       ; User direct page
          MOV  DPR,A

;====================================================================
; 6.10  Wait for clocks to stabilize
;====================================================================

#if CLOCKWAIT == ON
#  if CLKB_CLKP1_SOURCE == CLOCK_MAIN
no_MC_yet:
          BBC  CKMR:5,no_MC_yet        ; check MCM and wait for
                                       ; Main Clock to stabilize
#  elif CLKB_CLKP1_SOURCE == CLOCK_PLL
no_PLL_yet:
          BBC  CKMR:6,no_PLL_yet       ; check PCM and wait for
                                       ; PLL to stabilize
#  else
#    error Setting of CLKB_CLKP1_SOURCE is not supported.
#  endif
#endif

;====================================================================
; 6.11  Initialise Low-Level Library Interface
;====================================================================
;
; Call lib init function and reload stack afterwards, if AUTOMODEL
;====================================================================
#if CLIBINIT == ON
#  if MEMMODEL == SMALL || MEMMODEL == COMPACT
          CALL __stream_init       ; initialise library IO
#  else                            ; MEDIUM, LARGE, AUTOMODEL
          CALLP __stream_init      ; initialise library IO
#    if MEMMODEL == AUTOMODEL
          RELOAD_SP                ; reload stack since stream_init was
                                   ; possibly left by RET (not RETP)
#    endif  ; AUTOMODEL
#  endif  ; MEDIUM, LARGE, AUTOMODEL
#endif  ; LIBINI

;====================================================================
; 6.12  Call C-language main function
;====================================================================
#if MEMMODEL == SMALL || MEMMODEL == COMPACT
          CALL _main               ; Start main function
#else                              ; MEDIUM, LARGE, AUTOMODEL
          CALLP _main              ; Start main function
                                   ; ignore remaining word on stack,
                                   ; if main was completed by RET
#endif
;====================================================================
; 6.13  Shut down library
;====================================================================
#if CLIBINIT == ON
#  if MEMMODEL == SMALL || MEMMODEL == COMPACT
          CALL _exit
#  else                            ; MEDIUM, LARGE, AUTOMODEL
          CALLP _exit              ; ignore remaining word on stack,
                                   ; if main was completed by RET
#  endif
__exit:
#endif

;====================================================================
; 6.14  Program end loop
;====================================================================

end:      BRA  end                 ; Loop

          .END


;====================================================================
; ----------------------- End of Start-up file ---------------------
;====================================================================
