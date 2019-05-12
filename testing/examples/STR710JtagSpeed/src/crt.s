/****************************************************************************
*  Copyright (c) 2006 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
*
****************************************************************************
*
*  History:
*
*  04.03.06  mifi   First Version
*                   This version based on an example from Ethernut and
*                   "ARM Cross Development with Eclipse" from James P. Lynch
****************************************************************************/

/*
 * Some defines for the program status registers
 */
   ARM_MODE_USER  = 0x10      /* Normal User Mode 					               */ 
   ARM_MODE_FIQ   = 0x11      /* FIQ Fast Interrupts Mode 				         */
   ARM_MODE_IRQ   = 0x12      /* IRQ Standard Interrupts Mode 			         */
   ARM_MODE_SVC   = 0x13      /* Supervisor Interrupts Mode 			         */
   ARM_MODE_ABORT = 0x17      /* Abort Processing memory Faults Mode 		   */
   ARM_MODE_UNDEF = 0x1B      /* Undefined Instructions Mode 		            */
   ARM_MODE_SYS   = 0x1F      /* System Running in Priviledged Operating Mode */
   ARM_MODE_MASK  = 0x1F
   
   I_BIT          = 0x80      /* disable IRQ when I bit is set */
   F_BIT          = 0x40      /* disable IRQ when I bit is set */
   
/*
 * Register Base Address
 */
   PRCCU_BASE     = 0xA0000000
   RCCU_CFR       = 0x08
   RCCU_PLL1CR    = 0x18
   PCU_MDIVR      = 0x40
   PCU_PDIVR      = 0x44
   PCU_BOOTCR     = 0x50
          
      
      
      
   .section .vectors,"ax"
   .code 32
        
/****************************************************************************/
/*               Vector table and reset entry                               */
/****************************************************************************/
_vectors:
   ldr pc, ResetAddr    /* Reset                 */
   ldr pc, UndefAddr    /* Undefined instruction */
   ldr pc, SWIAddr      /* Software interrupt    */
   ldr pc, PAbortAddr   /* Prefetch abort        */
   ldr pc, DAbortAddr   /* Data abort            */
   ldr pc, ReservedAddr /* Reserved              */
   ldr pc, IRQAddr      /* IRQ interrupt         */
   ldr pc, FIQAddr      /* FIQ interrupt         */


ResetAddr:     .word ResetHandler
UndefAddr:     .word UndefHandler
SWIAddr:       .word SWIHandler
PAbortAddr:    .word PAbortHandler
DAbortAddr:    .word DAbortHandler
ReservedAddr:  .word 0
IRQAddr:       .word IRQHandler
FIQAddr:       .word FIQHandler

   .ltorg


   .section .init, "ax"
   .code 32
   
   .global ResetHandler
   .global ExitFunction
   .extern main
/****************************************************************************/
/*                           Reset handler                                  */
/****************************************************************************/
ResetHandler:
/*
 * Wait for the oscillator is stable
 */   
   nop
   nop
   nop
   nop
   nop
   nop
   nop
   nop
   
/*
 * Setup STR71X, for more information about the register
 * take a look in the STR71x Microcontroller Reference Manual.
 *
 * Reference is made to: Rev. 6 March 2005
 * 
 * 1. Map internal RAM to address 0
 *    In this case, we are running always in the RAM
 *    this make no sence. But if we are in flash, we
 *    can copy the interrupt vectors into the ram and
 *    switch to RAM mode.
 *
 * 2. Setup the PLL, the eval board HITEX STR7 is equipped
 *    with an external 16MHz oscillator. We want:
 *
 *    RCLK:  32MHz = (CLK2 * 16) / 4
 *    MCLK:  32Mhz
 *    PCLK1: 32MHz
 *    PCLK2: 32MHz
 *
 */   
 
   /* 
    * 1. Map RAM to the boot memory 0x00000000
    */
   ldr   r0, =PRCCU_BASE
   ldr   r1, =0x01C2          
   str   r1, [r0, #PCU_BOOTCR] 
   
   
   /*
    * 2. Setup PLL start
    */
   
   /* Set the prescaling factor for APB and APB1 group */
   ldr   r0, =PRCCU_BASE
   ldr   r1, =0x0000             /* no prescaling PCLKx = RCLK */          
   str   r1, [r0, #PCU_PDIVR] 

   /* Set the prescaling factor for the Main System Clock MCLK */
   ldr   r0, =PRCCU_BASE
   ldr   r1, =0x0000             /* no prescaling MCLK = RCLK
   str   r1, [r0, #PCU_MDIVR] 
   
   /* Configure the PLL1 ( * 16 , / 4 ) */
   ldr   r0, =PRCCU_BASE
   ldr   r1, =0x0073          
   str   r1, [r0, #RCCU_PLL1CR]        

   /* Check if the PLL is locked */
pll_lock_loop:
   ldr   r1, [r0, #RCCU_CFR]
   tst   r1, #0x0002
   beq   pll_lock_loop
   
   /*  Select PLL1_Output as RCLK clock */
   ldr   r0, =PRCCU_BASE
   ldr   r1, =0x8009          
   str   r1, [r0, #RCCU_CFR]        
   
   /*
    * Setup PLL end
    */
    
    
   /*
    * Setup a stack for each mode
    */    
   msr   CPSR_c, #ARM_MODE_UNDEF | I_BIT | F_BIT   /* Undefined Instruction Mode */     
   ldr   sp, =__stack_und_end
   
   msr   CPSR_c, #ARM_MODE_ABORT | I_BIT | F_BIT   /* Abort Mode */
   ldr   sp, =__stack_abt_end
   
   msr   CPSR_c, #ARM_MODE_FIQ | I_BIT | F_BIT     /* FIQ Mode */   
   ldr   sp, =__stack_fiq_end
   
   msr   CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT     /* IRQ Mode */   
   ldr   sp, =__stack_irq_end
   
   msr   CPSR_c, #ARM_MODE_SVC | I_BIT | F_BIT     /* Supervisor Mode */
   ldr   sp, =__stack_svc_end


   /*
    * Clear .bss section
    */
   ldr   r1, =__bss_start
   ldr   r2, =__bss_end
   ldr   r3, =0
bss_clear_loop:
   cmp   r1, r2
   strne r3, [r1], #+4
   bne   bss_clear_loop
   
   
   /*
    * Jump to main
    */
   mrs   r0, cpsr
   bic   r0, r0, #I_BIT | F_BIT     /* Enable FIQ and IRQ interrupt */
   msr   cpsr, r0
   
   mov   r0, #0 /* No arguments */
   mov   r1, #0 /* No arguments */
   ldr   r2, =main
   mov   lr, pc
   bx    r2     /* And jump... */
                       
ExitFunction:
   nop
   nop
   nop
   b ExitFunction   
   

/****************************************************************************/
/*                         Default interrupt handler                        */
/****************************************************************************/

UndefHandler:
   b UndefHandler
   
SWIHandler:
   b SWIHandler

PAbortHandler:
   b PAbortHandler

DAbortHandler:
   b DAbortHandler
   
IRQHandler:
   b IRQHandler
   
FIQHandler:
   b FIQHandler
   
   .weak ExitFunction
   .weak UndefHandler, PAbortHandler, DAbortHandler
   .weak IRQHandler, FIQHandler

   .ltorg
/*** EOF ***/
