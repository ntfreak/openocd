/***************************************************************************
 *   Copyright (C) 2011 by Rodrigo L. Rosa                                 *
 *   rodrigorosa.LG@gmail.com                                              *
 *                                                                         *
 *   Based on dsp563xx_once.h written by Mathias Kuester                   *
 *   mkdorg@users.sourceforge.net                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "dsp5680xx.h"

struct dsp5680xx_common dsp5680xx_context;

#define err_check(retval,dsp5680xx_error_code,err_msg) if(retval != ERROR_OK){LOG_ERROR("DSP5680XX_ERROR:%d\nAt:%s:%d:%s",dsp5680xx_error_code,__FUNCTION__,__LINE__,err_msg);return retval;}
#define err_check_propagate(retval) if(retval!=ERROR_OK){return retval;}

int dsp5680xx_execute_queue(void){
  int retval;
  retval = jtag_execute_queue();
  return retval;
}

// Reset state machine
static int reset_jtag(void){
  int retval;
  tap_state_t states[2];
  const char *cp = "RESET";
  states[0] = tap_state_by_name(cp);
  retval = jtag_add_statemove(states[0]);
  err_check_propagate(retval);
  retval = jtag_execute_queue();
  err_check_propagate(retval);
  jtag_add_pathmove(0, states + 1);
  retval = jtag_execute_queue();
  return retval;
}

static int dsp5680xx_drscan(struct target * target, uint8_t * data_to_shift_into_dr, uint8_t * data_shifted_out_of_dr, int len){
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//
// Inputs:
//     - data_to_shift_into_dr: This is the data that will be shifted into the JTAG DR reg.
//     - data_shifted_out_of_dr: The data that will be shifted out of the JTAG DR reg will stored here
//     - len: Length of the data to be shifted to JTAG DR.
//
// Note:  If  data_shifted_out_of_dr  == NULL, discard incoming bits.
//
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  int retval = ERROR_OK;
  if (NULL == target->tap){
	retval = ERROR_FAIL;
	err_check(retval,DSP5680XX_ERROR_JTAG_INVALID_TAP,"Invalid tap");
  }
  if (len > 32){
	retval = ERROR_FAIL;
	err_check(retval,DSP5680XX_ERROR_JTAG_DR_LEN_OVERFLOW,"dr_len overflow, maxium is 32");
  }
  //TODO what values of len are valid for jtag_add_plain_dr_scan?
  //can i send as many bits as i want?
  //is the casting necessary?
  jtag_add_plain_dr_scan(len,data_to_shift_into_dr,data_shifted_out_of_dr, TAP_IDLE);
  if(dsp5680xx_context.flush){
	retval = dsp5680xx_execute_queue();
	err_check(retval,DSP5680XX_ERROR_JTAG_DRSCAN,"drscan failed!");
  }
  if(data_shifted_out_of_dr!=NULL){
    LOG_DEBUG("Data read (%d bits): 0x%04X",len,*data_shifted_out_of_dr);
  }else
    LOG_DEBUG("Data read was discarded.");
  return retval;
}

static int dsp5680xx_irscan(struct target * target, uint32_t * data_to_shift_into_ir, uint32_t * data_shifted_out_of_ir, uint8_t ir_len){
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Inputs:
//     - data_to_shift_into_ir: This is the data that will be shifted into the JTAG IR reg.
//     - data_shifted_out_of_ir: The data that will be shifted out of the JTAG IR reg will stored here
//     - len: Length of the data to be shifted to JTAG IR.
//
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  int retval = ERROR_OK;
  if (NULL == target->tap){
	retval = ERROR_FAIL;
	err_check(retval,DSP5680XX_ERROR_JTAG_INVALID_TAP,"Invalid tap");
  }
  if (ir_len != target->tap->ir_length){
    if(target->tap->enabled){
      retval = ERROR_FAIL;
      err_check(retval,DSP5680XX_ERROR_INVALID_IR_LEN,"Invalid irlen");
    }else{
      struct jtag_tap * master_tap = jtag_tap_by_string("dsp568013.chp");
      if((master_tap == NULL) || ((master_tap->enabled) && (ir_len != DSP5680XX_JTAG_MASTER_TAP_IRLEN))){
        retval = ERROR_FAIL;
        err_check(retval,DSP5680XX_ERROR_INVALID_IR_LEN,"Invalid irlen");
      }
    }
  }
  jtag_add_plain_ir_scan(ir_len,(uint8_t *)data_to_shift_into_ir,(uint8_t *)data_shifted_out_of_ir, TAP_IDLE);
  if(dsp5680xx_context.flush){
    retval = dsp5680xx_execute_queue();
    err_check(retval,DSP5680XX_ERROR_JTAG_IRSCAN,"irscan failed!");
  }
  return retval;
}

static int dsp5680xx_jtag_status(struct target *target, uint8_t * status){
  uint32_t read_from_ir;
  uint32_t instr;
  int retval;
  instr =  JTAG_INSTR_ENABLE_ONCE;
  retval = dsp5680xx_irscan(target,& instr, & read_from_ir,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  if(status!=NULL)
    *status = (uint8_t)read_from_ir;
  return ERROR_OK;
}

static int jtag_data_read(struct target * target, uint8_t * data_read, int num_bits){
  uint32_t bogus_instr = 0;
  int retval = dsp5680xx_drscan(target,(uint8_t *) & bogus_instr,data_read,num_bits);
  LOG_DEBUG("Data read (%d bits): 0x%04X",num_bits,*data_read);//TODO remove this or move to jtagio?
  return retval;
}

#define jtag_data_read8(target,data_read)  jtag_data_read(target,data_read,8)
#define jtag_data_read16(target,data_read) jtag_data_read(target,data_read,16)
#define jtag_data_read32(target,data_read) jtag_data_read(target,data_read,32)

static uint32_t data_read_dummy;
static int jtag_data_write(struct target * target, uint32_t instr,int num_bits, uint32_t * data_read){
  int retval;
  retval = dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & data_read_dummy,num_bits);
  err_check_propagate(retval);
  if(data_read != NULL)
    *data_read = data_read_dummy;
  return retval;
}

#define jtag_data_write8(target,instr,data_read)  jtag_data_write(target,instr,8,data_read)
#define jtag_data_write16(target,instr,data_read) jtag_data_write(target,instr,16,data_read)
#define jtag_data_write24(target,instr,data_read) jtag_data_write(target,instr,24,data_read)
#define jtag_data_write32(target,instr,data_read) jtag_data_write(target,instr,32,data_read)

/**
 * Executes EOnCE instruction.
 *
 * @param target
 * @param instr Instruction to execute.
 * @param rw
 * @param go
 * @param ex
 * @param eonce_status Value read from the EOnCE status register.
 *
 * @return
 */
static int eonce_instruction_exec_single(struct target * target, uint8_t instr, uint8_t rw, uint8_t go, uint8_t ex,uint8_t * eonce_status){
  int retval;
  uint32_t dr_out_tmp;
  uint8_t instr_with_flags = instr|(rw<<7)|(go<<6)|(ex<<5);
  retval = jtag_data_write(target,instr_with_flags,8,&dr_out_tmp);
  err_check_propagate(retval);
  if(eonce_status != NULL)
    *eonce_status =  (uint8_t) dr_out_tmp;
  return retval;
}

///wrappers for multi opcode instructions
#define dsp5680xx_exe_1(target,opcode1,opcode2,opcode3)	 dsp5680xx_exe1(target,opcode1)
#define dsp5680xx_exe_2(target,opcode1,opcode2,opcode3)	 dsp5680xx_exe2(target,opcode1,opcode2)
#define dsp5680xx_exe_3(target,opcode1,opcode2,opcode3)	 dsp5680xx_exe3(target,opcode1,opcode2,opcode3)
#define dsp5680xx_exe_generic(target,words,opcode1,opcode2,opcode3) dsp5680xx_exe_##words(target,opcode1,opcode2,opcode3)

/// Executes one word DSP instruction
static int dsp5680xx_exe1(struct target * target, uint16_t opcode){
  int retval;
  retval = eonce_instruction_exec_single(target,0x04,0,1,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode,NULL);
  err_check_propagate(retval);
  return retval;
}

/// Executes two word DSP instruction
static int dsp5680xx_exe2(struct target * target,uint16_t opcode1, uint16_t opcode2){
  int retval;
  retval = eonce_instruction_exec_single(target,0x04,0,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode1,NULL);
  err_check_propagate(retval);
  retval = eonce_instruction_exec_single(target,0x04,0,1,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode2,NULL);
  err_check_propagate(retval);
  return retval;
}

/// Executes three word DSP instruction
static int dsp5680xx_exe3(struct target * target, uint16_t opcode1,uint16_t opcode2,uint16_t opcode3){
  int retval;
  retval = eonce_instruction_exec_single(target,0x04,0,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode1,NULL);
  err_check_propagate(retval);
  retval = eonce_instruction_exec_single(target,0x04,0,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode2,NULL);
  err_check_propagate(retval);
  retval = eonce_instruction_exec_single(target,0x04,0,1,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,opcode3,NULL);
  err_check_propagate(retval);
  return retval;
}

/**
 * --------------- Real-time data exchange ---------------
 * The EOnCE Transmit (OTX) and Receive (ORX) registers are data memory mapped, each with an upper and lower 16 bit word.
 * Transmit and receive directions are defined from the core’s perspective.
 * The core writes to the Transmit register and reads the Receive register, and the host through JTAG writes to the Receive register and reads the Transmit register.
 * Both registers have a combined data memory mapped OTXRXSR which provides indication when each may be accessed.
 *ref: eonce_rev.1.0_0208081.pdf@36
*/

/// writes data into upper ORx register of the target
static int core_tx_upper_data(struct target * target, uint16_t data, uint32_t * eonce_status_low){
  int retval;
  retval = eonce_instruction_exec_single(target,DSP5680XX_ONCE_ORX1,0,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_write16(target,data,eonce_status_low);
  err_check_propagate(retval);
  return retval;
}

/// writes data into lower ORx register of the target
#define core_tx_lower_data(target,data) eonce_instruction_exec_single(target,DSP5680XX_ONCE_ORX,0,0,0,NULL);\
								  jtag_data_write16(target,data)

/**
 *
 * @param target
 * @param data_read: Returns the data read from the upper OTX register via JTAG.
 * @return: Returns an error code (see error code documentation)
 */
static int core_rx_upper_data(struct target * target, uint8_t * data_read)
{
  int retval;
  retval = eonce_instruction_exec_single(target,DSP5680XX_ONCE_OTX1,1,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_read16(target,data_read);
  err_check_propagate(retval);
  return retval;
}

/**
 *
 * @param target
 * @param data_read: Returns the data read from the lower OTX register via JTAG.
 * @return: Returns an error code (see error code documentation)
 */
static int core_rx_lower_data(struct target * target,uint8_t * data_read)
{
  int retval;
  retval = eonce_instruction_exec_single(target,DSP5680XX_ONCE_OTX,1,0,0,NULL);
  err_check_propagate(retval);
  retval = jtag_data_read16(target,data_read);
  err_check_propagate(retval);
  return retval;
}

/**
 * -- -- -- -- --- -- -- -- --- -- -- -- --- -- -- -- --- -- -- -- --- --
 * -- -- -- -- --- -- -- -Core Instructions- -- -- -- --- -- -- -- --- --
 * -- -- -- -- --- -- -- -- --- -- -- -- --- -- -- -- --- -- -- -- --- --
 */

/// move.l #value,r0
#define core_move_long_to_r0(target,value)	dsp5680xx_exe_generic(target,3,0xe418,value&0xffff,value>>16)

/// move.l #value,n
#define core_move_long_to_n(target,value)		dsp5680xx_exe_generic(target,3,0xe41e,value&0xffff,value>>16)

/// move x:(r0),y0
#define core_move_at_r0_to_y0(target)			dsp5680xx_exe_generic(target,1,0xF514,0,0)

/// move x:(r0),y1
#define core_move_at_r0_to_y1(target)			dsp5680xx_exe_generic(target,1,0xF714,0,0)

/// move.l x:(r0),y
#define core_move_long_at_r0_y(target) dsp5680xx_exe_generic(target,1,0xF734,0,0)

/// move y0,x:(r0)
#define core_move_y0_at_r0(target)			dsp5680xx_exe_generic(target,1,0xd514,0,0)

/// bfclr #value,x:(r0)
#define eonce_bfclr_at_r0(target,value)		dsp5680xx_exe_generic(target,2,0x8040,value,0)

/// move #value,y0
#define core_move_value_to_y0(target,value)	dsp5680xx_exe_generic(target,2,0x8745,value,0)

/// move.w y0,x:(r0)+
#define core_move_y0_at_r0_inc(target)		dsp5680xx_exe_generic(target,1,0xd500,0,0)

/// move.w y0,p:(r0)+
#define core_move_y0_at_pr0_inc(target)		dsp5680xx_exe_generic(target,1,0x8560,0,0)

/// move.w p:(r0)+,y0
#define core_move_at_pr0_inc_to_y0(target)	dsp5680xx_exe_generic(target,1,0x8568,0,0)

/// move.w p:(r0)+,y1
#define core_move_at_pr0_inc_to_y1(target)	dsp5680xx_exe_generic(target,1,0x8768,0,0)

/// move.l #value,r2
#define core_move_long_to_r2(target,value)	dsp5680xx_exe_generic(target,3,0xe41A,value&0xffff,value>>16)

/// move y0,x:(r2)
#define core_move_y0_at_r2(target)             dsp5680xx_exe_generic(target,1,0xd516,0,0)

/// move.w #<value>,x:(r2)
#define core_move_value_at_r2(target,value)	dsp5680xx_exe_generic(target,2,0x8642,value,0)

/// move.w #<value>,x:(r0)
#define core_move_value_at_r0(target,value)	dsp5680xx_exe_generic(target,2,0x8640,value,0)

/// move.w #<value>,x:(R2+<disp>)
#define core_move_value_at_r2_disp(target,value,disp)	dsp5680xx_exe_generic(target,3,0x8646,value,disp)

/// move.w x:(r2),Y0
#define core_move_at_r2_to_y0(target)		dsp5680xx_exe_generic(target,1,0xF516,0,0)

/// move.w p:(r2)+,y0
#define core_move_at_pr2_inc_to_y0(target)	dsp5680xx_exe_generic(target,1,0x856A,0,0)

/// move.l #value,r3
#define core_move_long_to_r1(target,value)	dsp5680xx_exe_generic(target,3,0xE419,value&0xffff,value>>16)

/// move.l #value,r3
#define core_move_long_to_r3(target,value)	dsp5680xx_exe_generic(target,3,0xE41B,value&0xffff,value>>16)

/// move.w y0,p:(r3)+
#define core_move_y0_at_pr3_inc(target)		dsp5680xx_exe_generic(target,1,0x8563,0,0)

/// move.w y0,x:(r3)
#define core_move_y0_at_r3(target)			dsp5680xx_exe_generic(target,1,0xD503,0,0)

/// move.l #value,r4
#define core_move_long_to_r4(target,value)	dsp5680xx_exe_generic(target,3,0xE41C,value&0xffff,value>>16)

/// move pc,r4
#define core_move_pc_to_r4(target)			dsp5680xx_exe_generic(target,1,0xE716,0,0)

/// move.l r4,y
#define core_move_r4_to_y(target)			dsp5680xx_exe_generic(target,1,0xe764,0,0)

/// move.w p:(r0)+,y0
#define core_move_at_pr0_inc_to_y0(target)	dsp5680xx_exe_generic(target,1,0x8568,0,0)

/// move.w x:(r0)+,y0
#define core_move_at_r0_inc_to_y0(target)	dsp5680xx_exe_generic(target,1,0xf500,0,0)

/// move x:(r0),y0
#define core_move_at_r0_y0(target)			dsp5680xx_exe_generic(target,1,0xF514,0,0)

/// nop
#define eonce_nop(target)		dsp5680xx_exe_generic(target,1,0xe700,0,0)

/// move.w x:(R2+<disp>),Y0
#define core_move_at_r2_disp_to_y0(target,disp) dsp5680xx_exe_generic(target,2,0xF542,disp,0)

/// move.w y1,x:(r2)
#define core_move_y1_at_r2(target) dsp5680xx_exe_generic(target,1,0xd716,0,0)

/// move.w y1,x:(r0)
#define core_move_y1_at_r0(target) dsp5680xx_exe_generic(target,1,0xd714,0,0)

/// move.bp y0,x:(r0)+
#define core_move_byte_y0_at_r0(target) dsp5680xx_exe_generic(target,1,0xd5a0,0,0)

/// move.w y1,p:(r0)+
#define core_move_y1_at_pr0_inc(target) dsp5680xx_exe_generic(target,1,0x8760,0,0)

/// move.w y1,x:(r0)+
#define core_move_y1_at_r0_inc(target) dsp5680xx_exe_generic(target,1,0xD700,0,0)

/// move.l #value,y
#define core_move_long_to_y(target,value) dsp5680xx_exe_generic(target,3,0xe417,value&0xffff,value>>16)

static int core_move_value_to_pc(struct target * target, uint32_t value){
  if (!(target->state == TARGET_HALTED)){
    LOG_ERROR("Target must be halted to move PC. Target state = %d.",target->state);
    return ERROR_TARGET_NOT_HALTED;
  };
  int retval;
  retval = dsp5680xx_exe_generic(target,3,0xE71E,value&0xffff,value>>16);
  err_check_propagate(retval);
  return retval;
}

static int eonce_load_TX_RX_to_r0(struct target * target)
{
  int retval;
  retval = core_move_long_to_r0(target,((MC568013_EONCE_TX_RX_ADDR)+(MC568013_EONCE_OBASE_ADDR<<16)));
  return retval;
}

static int core_load_TX_RX_high_addr_to_r0(struct target * target)
{
  int retval = 0;
  retval = core_move_long_to_r0(target,((MC568013_EONCE_TX1_RX1_HIGH_ADDR)+(MC568013_EONCE_OBASE_ADDR<<16)));
  return retval;
}

static int dsp5680xx_read_core_reg(struct target * target, uint8_t reg_addr, uint16_t * data_read)
{
  //TODO implement a general version of this which matches what openocd uses.
  int retval;
  uint32_t dummy_data_to_shift_into_dr;
  retval = eonce_instruction_exec_single(target,reg_addr,1,0,0,NULL);
  err_check_propagate(retval);
  retval = dsp5680xx_drscan(target,(uint8_t *)& dummy_data_to_shift_into_dr,(uint8_t *) data_read, 8);
  err_check_propagate(retval);
  LOG_DEBUG("Reg. data: 0x%02X.",*data_read);
  return retval;
}

static int eonce_read_status_reg(struct target * target, uint16_t * data){
  int retval;
  retval = dsp5680xx_read_core_reg(target,DSP5680XX_ONCE_OSR,data);
  err_check_propagate(retval);
  return retval;
}

/**
 * Takes the core out of debug mode.
 *
 * @param target
 * @param eonce_status Data read from the EOnCE status register.
 *
 * @return
 */
static int eonce_exit_debug_mode(struct target * target,uint8_t * eonce_status){
  int retval;
  retval = eonce_instruction_exec_single(target,0x1F,0,0,1,eonce_status);
  err_check_propagate(retval);
  return retval;
}

static int switch_tap(struct target * target, struct jtag_tap * master_tap,struct jtag_tap * core_tap){
  int retval = ERROR_OK;
  uint32_t instr;
  uint32_t ir_out;//not used, just to make jtag happy.
  if(master_tap == NULL){
    master_tap = jtag_tap_by_string("dsp568013.chp");
    if(master_tap == NULL){
      retval = ERROR_FAIL;
      err_check(retval,DSP5680XX_ERROR_JTAG_TAP_FIND_MASTER,"Failed to get master tap.");
    }
  }
  if(core_tap == NULL){
    core_tap = jtag_tap_by_string("dsp568013.cpu");
    if(core_tap == NULL){
      retval = ERROR_FAIL;
      err_check(retval,DSP5680XX_ERROR_JTAG_TAP_FIND_CORE,"Failed to get core tap.");
    }
  }

  if(!(((int)master_tap->enabled) ^ ((int)core_tap->enabled))){
      LOG_WARNING("Wrong tap enabled/disabled status:\nMaster tap:%d\nCore Tap:%d\nOnly one tap should be enabled at a given time.\n",(int)master_tap->enabled,(int)core_tap->enabled);
  }

  if(master_tap->enabled){
    instr = 0x5;
    retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_MASTER_TAP_IRLEN);
    err_check_propagate(retval);
    instr = 0x2;
    retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,4);
    err_check_propagate(retval);
    core_tap->enabled = true;
    master_tap->enabled = false;
  }else{
    instr = 0x08;
    retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
    err_check_propagate(retval);
    instr = 0x1;
    retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,4);
    err_check_propagate(retval);
    core_tap->enabled = false;
    master_tap->enabled = true;
  }
  return retval;
}

/**
 * Puts the core into debug mode, enabling the EOnCE module.
 * This will not always work, eonce_enter_debug_mode executes much
 * more complicated routine, which is guaranteed to work, but requires
 * a reset. This will complicate comm with the flash module, since
 * after a reset clock divisors must be set again.
 * This implementation works most of the time, and is not accesible to the
 * user.
 *
 * @param target
 * @param eonce_status Data read from the EOnCE status register.
 *
 * @return
 */
static int eonce_enter_debug_mode_without_reset(struct target * target, uint16_t * eonce_status){
  int retval;
  uint32_t instr = JTAG_INSTR_DEBUG_REQUEST;
  uint32_t ir_out;//not used, just to make jtag happy.
  // Debug request #1
  retval = dsp5680xx_irscan(target,& instr,& ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);

  // Enable EOnCE module
  instr = JTAG_INSTR_ENABLE_ONCE;
  //Two rounds of jtag 0x6  (enable eonce) to enable EOnCE.
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  // Verify that debug mode is enabled
  uint16_t data_read_from_dr;
  retval = eonce_read_status_reg(target,&data_read_from_dr);
  err_check_propagate(retval);
  if((data_read_from_dr&0x30) == 0x30){
    LOG_DEBUG("EOnCE successfully entered debug mode.");
    target->state = TARGET_HALTED;
    retval = ERROR_OK;
  }else{
    retval = ERROR_TARGET_FAILURE;
    err_check_propagate(retval);// No error msg here, since there is still hope with full halting sequence
  }
  if(eonce_status!=NULL)
    *eonce_status = data_read_from_dr;
	return retval;
}

/**
 * Puts the core into debug mode, enabling the EOnCE module.
 *
 * @param target
 * @param eonce_status Data read from the EOnCE status register.
 *
 * @return
 */
static int eonce_enter_debug_mode(struct target * target, uint16_t * eonce_status){
  int retval = ERROR_OK;
  uint32_t instr = JTAG_INSTR_DEBUG_REQUEST;
  uint32_t ir_out;//not used, just to make jtag happy.
  uint16_t instr_16;
  uint16_t read_16;

  // First try the easy way
  retval = eonce_enter_debug_mode_without_reset(target,eonce_status);
  if(retval == ERROR_OK)
    return retval;

  struct jtag_tap * tap_chp;
  struct jtag_tap * tap_cpu;
  tap_chp = jtag_tap_by_string("dsp568013.chp");
  if(tap_chp == NULL){
    retval = ERROR_FAIL;
    err_check(retval,DSP5680XX_ERROR_JTAG_TAP_FIND_MASTER,"Failed to get master tap.");
  }
  tap_cpu = jtag_tap_by_string("dsp568013.cpu");
  if(tap_cpu == NULL){
    retval = ERROR_FAIL;
    err_check(retval,DSP5680XX_ERROR_JTAG_TAP_FIND_CORE,"Failed to get master tap.");
  }

  // Enable master tap
  tap_chp->enabled = true;
  tap_cpu->enabled = false;

  instr = MASTER_TAP_CMD_IDCODE;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_MASTER_TAP_IRLEN);
  err_check_propagate(retval);
  jtag_add_sleep(TIME_DIV_FREESCALE*100*1000);

  // Enable EOnCE module
  jtag_add_reset(0,1);
  jtag_add_sleep(TIME_DIV_FREESCALE*200*1000);
  instr = 0x0606ffff;// This was selected experimentally.
  retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,32);
  err_check_propagate(retval);
  // ir_out now hold tap idcode

  // Enable core tap
  tap_chp->enabled = true;
  retval = switch_tap(target,tap_chp,tap_cpu);
  err_check_propagate(retval);

  instr = JTAG_INSTR_ENABLE_ONCE;
  //Two rounds of jtag 0x6  (enable eonce) to enable EOnCE.
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  instr = JTAG_INSTR_DEBUG_REQUEST;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  instr_16 = 0x1;
  retval = dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,8);
	err_check_propagate(retval);
  instr_16 = 0x20;
  retval = dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,8);
	err_check_propagate(retval);
  jtag_add_sleep(TIME_DIV_FREESCALE*100*1000);
  jtag_add_reset(0,0);
  jtag_add_sleep(TIME_DIV_FREESCALE*300*1000);

  instr = JTAG_INSTR_ENABLE_ONCE;
  //Two rounds of jtag 0x6  (enable eonce) to enable EOnCE.
  for(int i = 0; i<3; i++){
    retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
    err_check_propagate(retval);
  }

  for(int i = 0; i<3; i++){
    instr_16 = 0x86;
    dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,16);
    instr_16 = 0xff;
    dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,16);
  }

  // Verify that debug mode is enabled
  uint16_t data_read_from_dr;
  retval = eonce_read_status_reg(target,&data_read_from_dr);
  err_check_propagate(retval);
  if((data_read_from_dr&0x30) == 0x30){
    LOG_DEBUG("EOnCE successfully entered debug mode.");
    target->state = TARGET_HALTED;
    retval = ERROR_OK;
  }else{
    retval = ERROR_TARGET_FAILURE;
    err_check(retval,DSP5680XX_ERROR_ENTER_DEBUG_MODE,"Failed to set EOnCE module to debug mode");
  }
  if(eonce_status!=NULL)
    *eonce_status = data_read_from_dr;
  return retval;
}

/**
 * Reads the current value of the program counter and stores it.
 *
 * @param target
 *
 * @return
 */
static int eonce_pc_store(struct target * target){
  uint8_t tmp[2];
  int retval;
  retval = core_move_pc_to_r4(target);
  err_check_propagate(retval);
  retval = core_move_r4_to_y(target);
  err_check_propagate(retval);
  retval = eonce_load_TX_RX_to_r0(target);
  err_check_propagate(retval);
  retval = core_move_y0_at_r0(target);
  err_check_propagate(retval);
  retval = core_rx_lower_data(target,tmp);
  err_check_propagate(retval);
  LOG_USER("PC value: 0x%X%X\n",tmp[1],tmp[0]);
  dsp5680xx_context.stored_pc = (tmp[0]|(tmp[1]<<8));
  return ERROR_OK;
}

static int dsp5680xx_target_create(struct target *target, Jim_Interp * interp){
  struct dsp5680xx_common *dsp5680xx = calloc(1, sizeof(struct dsp5680xx_common));
  target->arch_info = dsp5680xx;
  return ERROR_OK;
}

static int dsp5680xx_init_target(struct command_context *cmd_ctx, struct target *target){
  dsp5680xx_context.stored_pc = 0;
  dsp5680xx_context.flush = 1;
  LOG_DEBUG("target initiated!");
  //TODO core tap must be enabled before running these commands, currently this is done in the .cfg tcl script.
  return ERROR_OK;
}

static int dsp5680xx_arch_state(struct target *target){
  LOG_USER("%s not implemented yet.",__FUNCTION__);
  return ERROR_OK;
}

int dsp5680xx_target_status(struct target * target, uint8_t * jtag_st, uint16_t * eonce_st){
  return target->state;
}

static int dsp5680xx_assert_reset(struct target *target){
  target->state = TARGET_RESET;
  return ERROR_OK;
}

static int dsp5680xx_deassert_reset(struct target *target){
  target->state = TARGET_RUNNING;
  return ERROR_OK;
}

static int dsp5680xx_halt(struct target *target){
  int retval;
  uint16_t eonce_status = 0xbeef;
  if(target->state == TARGET_HALTED){
    LOG_USER("Target already halted.");
    return ERROR_OK;
  }
  retval = eonce_enter_debug_mode(target,&eonce_status);
  err_check_propagate(retval);
  retval = eonce_pc_store(target);
  err_check_propagate(retval);
  //TODO is it useful to store the pc?
  return retval;
}

static int dsp5680xx_poll(struct target *target){
  int retval;
  uint8_t jtag_status;
  uint8_t eonce_status;
  uint16_t read_tmp;
  retval = dsp5680xx_jtag_status(target,&jtag_status);
  err_check_propagate(retval);
  if (jtag_status == JTAG_STATUS_DEBUG)
    if (target->state != TARGET_HALTED){
      retval = eonce_enter_debug_mode(target,&read_tmp);
	  err_check_propagate(retval);
      eonce_status = (uint8_t) read_tmp;
      if((eonce_status&EONCE_STAT_MASK) != DSP5680XX_ONCE_OSCR_DEBUG_M){
		LOG_WARNING("%s: Failed to put EOnCE in debug mode. Is flash locked?...",__FUNCTION__);
		return ERROR_TARGET_FAILURE;
      }else{
		target->state = TARGET_HALTED;
		return ERROR_OK;
      }
    }
  if (jtag_status == JTAG_STATUS_NORMAL){
    if(target->state == TARGET_RESET){
      retval = dsp5680xx_halt(target);
	  err_check_propagate(retval);
      retval = eonce_exit_debug_mode(target,&eonce_status);
	  err_check_propagate(retval);
      if((eonce_status&EONCE_STAT_MASK) != DSP5680XX_ONCE_OSCR_NORMAL_M){
		LOG_WARNING("%s: JTAG running, but cannot make EOnCE run. Try resetting...",__FUNCTION__);
		return ERROR_TARGET_FAILURE;
      }else{
		target->state = TARGET_RUNNING;
		return ERROR_OK;
      }
    }
    if(target->state != TARGET_RUNNING){
      retval = eonce_read_status_reg(target,&read_tmp);
	  err_check_propagate(retval);
      eonce_status = (uint8_t) read_tmp;
      if((eonce_status&EONCE_STAT_MASK) != DSP5680XX_ONCE_OSCR_NORMAL_M){
		LOG_WARNING("Inconsistent target status. Restart!");
		return ERROR_TARGET_FAILURE;
      }
    }
    target->state = TARGET_RUNNING;
    return ERROR_OK;
  }
  if(jtag_status == JTAG_STATUS_DEAD){
    LOG_ERROR("%s: Cannot communicate with JTAG. Check connection...",__FUNCTION__);
    target->state = TARGET_UNKNOWN;
    return ERROR_TARGET_FAILURE;
  };
  if (target->state == TARGET_UNKNOWN){
    LOG_ERROR("%s: Target status invalid - communication failure",__FUNCTION__);
    return ERROR_TARGET_FAILURE;
  };
  return ERROR_OK;
}

static int dsp5680xx_resume(struct target *target, int current, uint32_t address,int handle_breakpoints, int debug_execution){
  if(target->state == TARGET_RUNNING){
    LOG_USER("Target already running.");
    return ERROR_OK;
  }
  int retval;
  uint8_t eonce_status;
  if(!current){
    retval = core_move_value_to_pc(target,address);
    err_check_propagate(retval);
  }

  int retry = 20;
  while(retry-- > 1){
    retval = eonce_exit_debug_mode(target,&eonce_status );
	err_check_propagate(retval);
    if(eonce_status == DSP5680XX_ONCE_OSCR_NORMAL_M)
      break;
  }
  if(retry == 0){
    retval = ERROR_TARGET_FAILURE;
    err_check(retval,DSP5680XX_ERROR_RESUME,"Failed to resume...");
  }else{
    target->state = TARGET_RUNNING;
  }
  LOG_DEBUG("EOnCE status: 0x%02X.",eonce_status);
  return ERROR_OK;
}






/**
 * The value of @address determines if it corresponds to P: (program) or X: (data) memory. If the address is over 0x200000 then it is considered X: memory, and @pmem = 0.
 * The special case of 0xFFXXXX is not modified, since it allows to read out the memory mapped EOnCE registers.
 *
 * @param address
 * @param pmem
 *
 * @return
 */
static int dsp5680xx_convert_address(uint32_t * address, int * pmem){
  // Distinguish data memory (x:) from program memory (p:) by the address.
  // Addresses over S_FILE_DATA_OFFSET are considered (x:) memory.
  if(*address >= S_FILE_DATA_OFFSET){
    *pmem = 0;
    if(((*address)&0xff0000)!=0xff0000)
      *address -= S_FILE_DATA_OFFSET;
  }
  return ERROR_OK;
}

static int dsp5680xx_read_16_single(struct target * target, uint32_t address, uint8_t * data_read, int r_pmem){
  int retval;
  retval = core_move_long_to_r0(target,address);
  err_check_propagate(retval);
  if(r_pmem)
    retval = core_move_at_pr0_inc_to_y0(target);
  else
    retval = core_move_at_r0_to_y0(target);
  err_check_propagate(retval);
  retval = eonce_load_TX_RX_to_r0(target);
  err_check_propagate(retval);
  retval = core_move_y0_at_r0(target);
  err_check_propagate(retval);
  // at this point the data i want is at the reg eonce can read
  retval = core_rx_lower_data(target,data_read);
  err_check_propagate(retval);
  LOG_DEBUG("%s: Data read from 0x%06X: 0x%02X%02X",__FUNCTION__, address,data_read[1],data_read[0]);
  return retval;
}

static int dsp5680xx_read_32_single(struct target * target, uint32_t address, uint8_t * data_read, int r_pmem){
  int retval;
  address = (address & 0xFFFFFE);
  // Get data to an intermediate register
  retval = core_move_long_to_r0(target,address);
  err_check_propagate(retval);
  if(r_pmem){
    retval = core_move_at_pr0_inc_to_y0(target);
	err_check_propagate(retval);
    retval = core_move_at_pr0_inc_to_y1(target);
	err_check_propagate(retval);
  }else{
    retval = core_move_at_r0_inc_to_y0(target);
	err_check_propagate(retval);
    retval = core_move_at_r0_to_y1(target);
	err_check_propagate(retval);
  }
  // Get lower part of data to TX/RX
  retval = eonce_load_TX_RX_to_r0(target);
  err_check_propagate(retval);
  retval = core_move_y0_at_r0_inc(target); // This also load TX/RX high to r0
  err_check_propagate(retval);
  // Get upper part of data to TX/RX
  retval = core_move_y1_at_r0(target);
  err_check_propagate(retval);
  // at this point the data i want is at the reg eonce can read
  retval = core_rx_lower_data(target,data_read);
  err_check_propagate(retval);
  retval = core_rx_upper_data(target,data_read+2);
  err_check_propagate(retval);
  return retval;
}

static int dsp5680xx_read(struct target * target, uint32_t address, unsigned size, unsigned count, uint8_t * buffer){
  if(target->state != TARGET_HALTED){
    LOG_USER("Target must be halted.");
    return ERROR_FAIL;
  }
  int retval = ERROR_OK;
  int pmem = 1;

  retval = dsp5680xx_convert_address(&address, &pmem);
  err_check_propagate(retval);

  dsp5680xx_context.flush = 0;
  int counter = FLUSH_COUNT_READ_WRITE;

  for (unsigned i=0; i<count; i++){
    if(--counter==0){
      dsp5680xx_context.flush = 1;
      counter = FLUSH_COUNT_READ_WRITE;
    }
    switch (size){
    case 1:
      if(!(i%2)){
		retval = dsp5680xx_read_16_single(target, address + i/2, buffer + i, pmem);
      }
      break;
    case 2:
      retval = dsp5680xx_read_16_single(target, address + i, buffer+2*i, pmem);
      break;
    case 4:
      retval = dsp5680xx_read_32_single(target, address + 2*i, buffer + 4*i, pmem);
      break;
    default:
      LOG_USER("%s: Invalid read size.",__FUNCTION__);
      break;
    }
	err_check_propagate(retval);
    dsp5680xx_context.flush = 0;
  }

  dsp5680xx_context.flush = 1;
  retval = dsp5680xx_execute_queue();
  err_check_propagate(retval);

  return retval;
}

static int dsp5680xx_write_16_single(struct target *target, uint32_t address, uint16_t data, uint8_t w_pmem){
  int retval = 0;
  retval = core_move_long_to_r0(target,address);
  err_check_propagate(retval);
  if(w_pmem){
    retval = core_move_value_to_y0(target,data);
	err_check_propagate(retval);
    retval = core_move_y0_at_pr0_inc(target);
	err_check_propagate(retval);
  }else{
    retval = core_move_value_at_r0(target,data);
	err_check_propagate(retval);
  }
  return retval;
}

static int dsp5680xx_write_32_single(struct target *target, uint32_t address, uint32_t data, int w_pmem){
  int retval = 0;
  retval = core_move_long_to_r0(target,address);
  err_check_propagate(retval);
  retval = core_move_long_to_y(target,data);
  err_check_propagate(retval);
  if(w_pmem)
    retval = core_move_y0_at_pr0_inc(target);
  else
    retval = core_move_y0_at_r0_inc(target);
  err_check_propagate(retval);
  if(w_pmem)
    retval = core_move_y1_at_pr0_inc(target);
  else
    retval = core_move_y1_at_r0_inc(target);
  err_check_propagate(retval);
  return retval;
}

static int dsp5680xx_write_8(struct target * target, uint32_t address, uint32_t count, const uint8_t * data, int pmem){
  if(target->state != TARGET_HALTED){
    LOG_ERROR("%s: Target must be halted.",__FUNCTION__);
    return ERROR_OK;
  };
  int retval = 0;
  uint16_t data_16;
  uint32_t iter;

  int counter = FLUSH_COUNT_READ_WRITE;
  for(iter = 0; iter<count/2; iter++){
    if(--counter==0){
      dsp5680xx_context.flush = 1;
      counter = FLUSH_COUNT_READ_WRITE;
    }
    data_16=(data[2*iter]|(data[2*iter+1]<<8));
    retval = dsp5680xx_write_16_single(target,address+iter,data_16, pmem);
    if(retval != ERROR_OK){
      LOG_ERROR("%s: Could not write to p:0x%04X",__FUNCTION__,address);
      dsp5680xx_context.flush = 1;
      return retval;
    }
    dsp5680xx_context.flush = 0;
  }
  dsp5680xx_context.flush = 1;

  // Only one byte left, let's not overwrite the other byte (mem is 16bit)
  // Need to retrieve the part we do not want to overwrite.
  uint16_t data_old;
  if((count==1)||(count%2)){
    retval = dsp5680xx_read(target,address+iter,1,1,(uint8_t *)&data_old);
	err_check_propagate(retval);
    if(count==1)
      data_old=(((data_old&0xff)<<8)|data[0]);// preserve upper byte
    else
      data_old=(((data_old&0xff)<<8)|data[2*iter+1]);
    retval = dsp5680xx_write_16_single(target,address+iter,data_old, pmem);
	err_check_propagate(retval);
  }
  return retval;
}

static int dsp5680xx_write_16(struct target * target, uint32_t address, uint32_t count, const uint8_t * data, int pmem){
  int retval = ERROR_OK;
  if(target->state != TARGET_HALTED){
	retval = ERROR_TARGET_NOT_HALTED;
	err_check(retval,DSP5680XX_ERROR_WRITE_WITH_TARGET_RUNNING,"Target must be halted.");
  };
  uint32_t iter;
  int counter = FLUSH_COUNT_READ_WRITE;

  for(iter = 0; iter<count; iter++){
	if(--counter==0){
	  dsp5680xx_context.flush = 1;
      counter = FLUSH_COUNT_READ_WRITE;
	}
    retval = dsp5680xx_write_16_single(target,address+iter,data[iter], pmem);
    if(retval != ERROR_OK){
      LOG_ERROR("%s: Could not write to p:0x%04X",__FUNCTION__,address);
	  dsp5680xx_context.flush = 1;
      return retval;
    }
	dsp5680xx_context.flush = 0;
  }
  dsp5680xx_context.flush = 1;
  return retval;
}

static int dsp5680xx_write_32(struct target * target, uint32_t address, uint32_t count, const uint8_t * data, int pmem){
  int retval = ERROR_OK;
  if(target->state != TARGET_HALTED){
	retval = ERROR_TARGET_NOT_HALTED;
	err_check(retval,DSP5680XX_ERROR_WRITE_WITH_TARGET_RUNNING,"Target must be halted.");
  };
  uint32_t iter;
  int counter = FLUSH_COUNT_READ_WRITE;

  for(iter = 0; iter<count; iter++){
	if(--counter==0){
	  dsp5680xx_context.flush = 1;
      counter = FLUSH_COUNT_READ_WRITE;
	}
    retval = dsp5680xx_write_32_single(target,address+(iter<<1),data[iter], pmem);
    if(retval != ERROR_OK){
      LOG_ERROR("%s: Could not write to p:0x%04X",__FUNCTION__,address);
	  dsp5680xx_context.flush = 1;
      return retval;
    }
	dsp5680xx_context.flush = 0;
  }
  dsp5680xx_context.flush = 1;
  return retval;
}

/**
 * Writes @buffer to memory.
 * The parameter @address determines whether @buffer should be written to P: (program) memory or X: (data) memory.
 *
 * @param target
 * @param address
 * @param size Bytes (1), Half words (2), Words (4).
 * @param count In bytes.
 * @param buffer
 *
 * @return
 */
static int dsp5680xx_write(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t * buffer){
  //TODO Cannot write 32bit to odd address, will write 0x12345678  as 0x5678 0x0012
  if(target->state != TARGET_HALTED){
    err_check(ERROR_FAIL,DSP5680XX_ERROR_WRITE_WITH_TARGET_RUNNING,"Target must be halted.");
  }
  int retval = 0;
  int p_mem = 1;
  retval = dsp5680xx_convert_address(&address, &p_mem);
  err_check_propagate(retval);

  switch (size){
  case 1:
    retval = dsp5680xx_write_8(target, address, count, buffer, p_mem);
    break;
  case 2:
    retval = dsp5680xx_write_16(target, address, count, buffer, p_mem);
      break;
  case 4:
    retval = dsp5680xx_write_32(target, address, count, buffer, p_mem);
    break;
  default:
	retval = ERROR_TARGET_DATA_ABORT;
	err_check(retval,DSP5680XX_ERROR_INVALID_DATA_SIZE_UNIT,"Invalid data size.");
	break;
  }
  return retval;
}

static int dsp5680xx_bulk_write_memory(struct target * target,uint32_t address, uint32_t aligned, const uint8_t * buffer){
  LOG_ERROR("Not implemented yet.");
  return ERROR_FAIL;
}

static int dsp5680xx_write_buffer(struct target * target, uint32_t address, uint32_t size, const uint8_t * buffer){
  if(target->state != TARGET_HALTED){
    LOG_USER("Target must be halted.");
    return ERROR_OK;
  }
  return dsp5680xx_write(target, address, 1, size, buffer);
}

/**
 * This function is called by verify_image, it is used to read data from memory.
 *
 * @param target
 * @param address Word addressing.
 * @param size In bytes.
 * @param buffer
 *
 * @return
 */
static int dsp5680xx_read_buffer(struct target * target, uint32_t address, uint32_t size, uint8_t * buffer){
  if(target->state != TARGET_HALTED){
    LOG_USER("Target must be halted.");
    return ERROR_OK;
  }
  // The "/2" solves the byte/word addressing issue.
  return dsp5680xx_read(target,address,2,size/2,buffer);
}

/**
 * This function is not implemented.
 * It returns an error in order to get OpenOCD to do read out the data and calculate the CRC, or try a binary comparison.
 *
 * @param target
 * @param address Start address of the image.
 * @param size In bytes.
 * @param checksum
 *
 * @return
 */
static int dsp5680xx_checksum_memory(struct target * target, uint32_t address, uint32_t size, uint32_t * checksum){
  return ERROR_FAIL;
}

/**
 * Calculates a signature over @word_count words in the data from @buff16. The algorithm used is the same the FM uses, so the @return may be used to compare with the one generated by the FM module, and check if flashing was successful.
 * This algorithm is based on the perl script available from the Freescale website at FAQ 25630.
 *
 * @param buff16
 * @param word_count
 *
 * @return
 */
static int perl_crc(uint8_t * buff8,uint32_t  word_count){
  uint16_t checksum = 0xffff;
  uint16_t data,fbmisr;
  uint32_t i;
  for(i=0;i<word_count;i++){
    data = (buff8[2*i]|(buff8[2*i+1]<<8));
    fbmisr = (checksum & 2)>>1 ^ (checksum & 4)>>2 ^ (checksum & 16)>>4 ^ (checksum & 0x8000)>>15;
    checksum = (data ^ ((checksum << 1) | fbmisr));
  }
  i--;
  for(;!(i&0x80000000);i--){
    data = (buff8[2*i]|(buff8[2*i+1]<<8));
    fbmisr = (checksum & 2)>>1 ^ (checksum & 4)>>2 ^ (checksum & 16)>>4 ^ (checksum & 0x8000)>>15;
    checksum = (data ^ ((checksum << 1) | fbmisr));
  }
  return checksum;
}

/**
 * Resets the SIM. (System Integration Module).
 *
 * @param target
 *
 * @return
 */
int dsp5680xx_f_SIM_reset(struct target * target){
  int retval = ERROR_OK;
  uint16_t sim_cmd = SIM_CMD_RESET;
  uint32_t sim_addr;
  if(strcmp(target->tap->chip,"dsp568013")==0){
	sim_addr = MC568013_SIM_BASE_ADDR+S_FILE_DATA_OFFSET;
	retval = dsp5680xx_write(target,sim_addr,1,2,(const uint8_t *)&sim_cmd);
	err_check_propagate(retval);
  }
  return retval;
}

/**
 * Halts the core and resets the SIM. (System Integration Module).
 *
 * @param target
 *
 * @return
 */
static int dsp5680xx_soft_reset_halt(struct target *target){
  //TODO is this what this function is expected to do...?
  int retval;
  retval = dsp5680xx_halt(target);
  err_check_propagate(retval);
  retval = dsp5680xx_f_SIM_reset(target);
  err_check_propagate(retval);
  return retval;
}

int dsp5680xx_f_protect_check(struct target * target, uint16_t * protected) {
  int retval;
  if (dsp5680xx_target_status(target,NULL,NULL) != TARGET_HALTED){
    retval = dsp5680xx_halt(target);
	err_check_propagate(retval);
  }
  if(protected == NULL){
    err_check(ERROR_FAIL,DSP5680XX_ERROR_PROTECT_CHECK_INVALID_ARGS,"NULL pointer not valid.");
  }
  retval = dsp5680xx_read_16_single(target,HFM_BASE_ADDR|HFM_PROT,(uint8_t *)protected,0);
  err_check_propagate(retval);
  return retval;
}

/**
 * Executes a command on the FM module. Some commands use the parameters @address and @data, others ignore them.
 *
 * @param target
 * @param command Command to execute.
 * @param address Command parameter.
 * @param data Command parameter.
 * @param hfm_ustat FM status register.
 * @param pmem Address is P: (program) memory (@pmem==1) or X: (data) memory (@pmem==0)
 *
 * @return
 */
static int dsp5680xx_f_execute_command(struct target * target, uint16_t command, uint32_t address, uint32_t data, uint16_t * hfm_ustat, int pmem){
  int retval;
  retval = core_load_TX_RX_high_addr_to_r0(target);
  err_check_propagate(retval);
  retval = core_move_long_to_r2(target,HFM_BASE_ADDR);
  err_check_propagate(retval);
  uint8_t i[2];
  int watchdog = 100;
  do{
    retval = core_move_at_r2_disp_to_y0(target,HFM_USTAT);	// read HMF_USTAT
	err_check_propagate(retval);
    retval = core_move_y0_at_r0(target);
	err_check_propagate(retval);
    retval = core_rx_upper_data(target,i);
	err_check_propagate(retval);
    if((watchdog--)==1){
      retval = ERROR_TARGET_FAILURE;
      err_check(retval,DSP5680XX_ERROR_FM_BUSY,"Timed out waiting for FM to finish old command.");
    }
  }while (!(i[0]&0x40));				// wait until current command is complete

  dsp5680xx_context.flush = 0;

  retval = core_move_value_at_r2_disp(target,0x00,HFM_CNFG);	// write to HFM_CNFG (lock=0, select bank) -- flash_desc.bank&0x03,0x01 == 0x00,0x01 ???
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x04,HFM_USTAT);		// write to HMF_USTAT, clear PVIOL, ACCERR & BLANK bits
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x10,HFM_USTAT);		// clear only one bit at a time
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x20,HFM_USTAT);
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x00,HFM_PROT);		// write to HMF_PROT, clear protection
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x00,HFM_PROTB);		// write to HMF_PROTB, clear protection
  err_check_propagate(retval);
  retval = core_move_value_to_y0(target,data);
  err_check_propagate(retval);
  retval = core_move_long_to_r3(target,address);			// write to the flash block
  err_check_propagate(retval);
  if (pmem){
    retval = core_move_y0_at_pr3_inc(target);
	err_check_propagate(retval);
  }else{
    retval = core_move_y0_at_r3(target);
	err_check_propagate(retval);
  }
  retval = core_move_value_at_r2_disp(target,command,HFM_CMD);	// write command to the HFM_CMD reg
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x80,HFM_USTAT);		// start the command
  err_check_propagate(retval);

  dsp5680xx_context.flush = 1;
  retval = dsp5680xx_execute_queue();
  err_check_propagate(retval);

  watchdog = 100;
  do{
    retval = core_move_at_r2_disp_to_y0(target,HFM_USTAT);	// read HMF_USTAT
	err_check_propagate(retval);
    retval = core_move_y0_at_r0(target);
	err_check_propagate(retval);
	retval = core_rx_upper_data(target,i);
	err_check_propagate(retval);
    if((watchdog--)==1){
	  retval = ERROR_TARGET_FAILURE;
	  err_check(retval,DSP5680XX_ERROR_FM_CMD_TIMED_OUT,"FM execution did not finish.");
    }
  }while (!(i[0]&0x40));	    // wait until the command is complete
  *hfm_ustat = ((i[0]<<8)|(i[1]));
  if (i[0]&HFM_USTAT_MASK_PVIOL_ACCER){
    retval = ERROR_TARGET_FAILURE;
    err_check(retval,DSP5680XX_ERROR_FM_EXEC,"pviol and/or accer bits set. HFM command execution error");
  }
  return ERROR_OK;
}

/**
 * Prior to the execution of any Flash module command, the Flash module Clock Divider (CLKDIV) register must be initialized. The values of this register determine the speed of the internal Flash Clock (FCLK). FCLK must be in the range of 150kHz ≤ FCLK ≤ 200kHz for proper operation of the Flash module. (Running FCLK too slowly wears out the module, while running it too fast under programs Flash leading to bit errors.)
 *
 * @param target
 *
 * @return
 */
static int set_fm_ck_div(struct target * target){
  uint8_t i[2];
  int retval;
  retval = core_move_long_to_r2(target,HFM_BASE_ADDR);
  err_check_propagate(retval);
  retval = core_load_TX_RX_high_addr_to_r0(target);
  err_check_propagate(retval);
  retval = core_move_at_r2_to_y0(target);// read HFM_CLKD
  err_check_propagate(retval);
  retval = core_move_y0_at_r0(target);
  err_check_propagate(retval);
  retval = core_rx_upper_data(target,i);
  err_check_propagate(retval);
  unsigned int hfm_at_wrong_value = 0;
  if ((i[0]&0x7f)!=HFM_CLK_DEFAULT) {
    LOG_DEBUG("HFM CLK divisor contained incorrect value (0x%02X).",i[0]&0x7f);
    hfm_at_wrong_value = 1;
  }else{
    LOG_DEBUG("HFM CLK divisor was already set to correct value (0x%02X).",i[0]&0x7f);
    return ERROR_OK;
  }
  retval = core_move_value_at_r2(target,HFM_CLK_DEFAULT);	// write HFM_CLKD
  err_check_propagate(retval);
  retval = core_move_at_r2_to_y0(target); // verify HFM_CLKD
  err_check_propagate(retval);
  retval = core_move_y0_at_r0(target);
  err_check_propagate(retval);
  retval = core_rx_upper_data(target,i);
  err_check_propagate(retval);
  if (i[0]!=(0x80|(HFM_CLK_DEFAULT&0x7f))) {
	retval = ERROR_TARGET_FAILURE;
	err_check(retval,DSP5680XX_ERROR_FM_SET_CLK,"Unable to set HFM CLK divisor.");
  }
  if(hfm_at_wrong_value)
    LOG_DEBUG("HFM CLK divisor set to 0x%02x.",i[0]&0x7f);
  return ERROR_OK;
}

/**
 * Executes the FM calculate signature command. The FM will calculate over the data from @address to @address + @words -1. The result is written to a register, then read out by this function and returned in @signature. The value @signature may be compared to the the one returned by perl_crc to verify the flash was written correctly.
 *
 * @param target
 * @param address Start of flash array where the signature should be calculated.
 * @param words Number of words over which the signature should be calculated.
 * @param signature Value calculated by the FM.
 *
 * @return
 */
static int dsp5680xx_f_signature(struct target * target, uint32_t address, uint32_t words, uint16_t * signature){
  int retval;
  uint16_t hfm_ustat;
  if (dsp5680xx_target_status(target,NULL,NULL) != TARGET_HALTED){
    retval = eonce_enter_debug_mode_without_reset(target,NULL);
    err_check_propagate(retval);
  }
  retval = dsp5680xx_f_execute_command(target,HFM_CALCULATE_DATA_SIGNATURE,address,words,&hfm_ustat,1);
  err_check_propagate(retval);
  retval = dsp5680xx_read_16_single(target, HFM_BASE_ADDR|HFM_DATA, (uint8_t *)signature, 0);
  return retval;
}

int dsp5680xx_f_erase_check(struct target * target, uint8_t * erased,uint32_t sector){
  int retval;
  uint16_t hfm_ustat;
  if (dsp5680xx_target_status(target,NULL,NULL) != TARGET_HALTED){
    retval = dsp5680xx_halt(target);
    err_check_propagate(retval);
  }
  retval = set_fm_ck_div(target);
  err_check_propagate(retval);
  // Check if chip is already erased.
  retval = dsp5680xx_f_execute_command(target,HFM_ERASE_VERIFY,HFM_FLASH_BASE_ADDR+sector*HFM_SECTOR_SIZE/2,0,&hfm_ustat,1); // blank check
  err_check_propagate(retval);
  if(erased!=NULL)
    *erased = (uint8_t)(hfm_ustat&HFM_USTAT_MASK_BLANK);
  return retval;
}

/**
 * Executes the FM page erase command.
 *
 * @param target
 * @param sector Page to erase.
 * @param hfm_ustat FM module status register.
 *
 * @return
 */
static int erase_sector(struct target * target, int sector, uint16_t * hfm_ustat){
  int retval;
  retval = dsp5680xx_f_execute_command(target,HFM_PAGE_ERASE,HFM_FLASH_BASE_ADDR+sector*HFM_SECTOR_SIZE/2,0,hfm_ustat,1);
  err_check_propagate(retval);
  return retval;
}

/**
 * Executes the FM mass erase command. Erases the flash array completely.
 *
 * @param target
 * @param hfm_ustat FM module status register.
 *
 * @return
 */
static int mass_erase(struct target * target, uint16_t * hfm_ustat){
  int retval;
  retval = dsp5680xx_f_execute_command(target,HFM_MASS_ERASE,0,0,hfm_ustat,1);
  return retval;
}

int dsp5680xx_f_erase(struct target * target, int first, int last){
  int retval;
  if (dsp5680xx_target_status(target,NULL,NULL) != TARGET_HALTED){
    retval = dsp5680xx_halt(target);
    err_check_propagate(retval);
  }
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Reset SIM
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  retval = dsp5680xx_f_SIM_reset(target);
  err_check_propagate(retval);
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Set hfmdiv
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  retval = set_fm_ck_div(target);
  err_check_propagate(retval);

  uint16_t hfm_ustat;
  int do_mass_erase = ((!(first|last)) || ((first==0)&&(last == (HFM_SECTOR_COUNT-1))));
  if(do_mass_erase){
    //Mass erase
    retval = mass_erase(target,&hfm_ustat);
	err_check_propagate(retval);
  }else{
    for(int i = first;i<=last;i++){
      retval = erase_sector(target,i,&hfm_ustat);
      err_check_propagate(retval);
    }
  }
  return ERROR_OK;
}

/**
 * Algorithm for programming normal p: flash
 * Follow state machine from "56F801x Peripheral Reference Manual"@163.
 * Registers to set up before calling:
*  r0: TX/RX high address.
*  r2: FM module base address.
*  r3: Destination address in flash.
*
*		hfm_wait:                                           // wait for command to finish
*			brclr	#0x40,x:(r2+0x13),hfm_wait
*		rx_check:					    // wait for input buffer full
*			brclr	#0x01,x:(r0-2),rx_check
*			move.w	x:(r0),y0		   	    // read from Rx buffer
*			move.w	y0,p:(r3)+
*			move.w	#0x20,x:(r2+0x14)		    // write PGM command
*			move.w	#0x80,x:(r2+0x13)		    // start the command
*                      brclr       #0x20,X:(R2+0x13),accerr_check  // protection violation check
*                      bfset       #0x20,X:(R2+0x13)               // clear pviol
*                      bra         hfm_wait
*              accerr_check:
*                      brclr       #0x10,X:(R2+0x13),hfm_wait      // access error check
*                      bfset       #0x10,X:(R2+0x13)               // clear accerr
*			bra	    hfm_wait		            // loop
*0x00000073  0x8A460013407D         brclr       #0x40,X:(R2+0x13),*+0
*0x00000076  0xE700                 nop
*0x00000077  0xE700                 nop
*0x00000078  0x8A44FFFE017B         brclr       #1,X:(R0-2),*-2
*0x0000007B  0xE700                 nop
*0x0000007C  0xF514                 move.w      X:(R0),Y0
*0x0000007D  0x8563                 move.w      Y0,P:(R3)+
*0x0000007E  0x864600200014         move.w      #0x20,X:(R2+0x14)
*0x00000081  0x864600800013         move.w      #0x80,X:(R2+0x13)
*0x00000084  0x8A4600132004         brclr       #0x20,X:(R2+0x13),*+7
*0x00000087  0x824600130020         bfset       #0x20,X:(R2+0x13)
*0x0000008A  0xA968                 bra         *-23
*0x0000008B  0x8A4600131065         brclr       #0x10,X:(R2+0x13),*-24
*0x0000008E  0x824600130010         bfset       #0x10,X:(R2+0x13)
*0x00000091  0xA961                 bra         *-30
*/
const uint16_t pgm_write_pflash[] = {0x8A46,0x0013,0x407D,0xE700,0xE700,0x8A44,0xFFFE,0x017B,0xE700,0xF514,0x8563,0x8646,0x0020,0x0014,0x8646,0x0080,0x0013,0x8A46,0x0013,0x2004,0x8246,0x0013,0x0020,0xA968,0x8A46,0x0013,0x1065,0x8246,0x0013,0x0010,0xA961};
const uint32_t pgm_write_pflash_length = 31;

int dsp5680xx_f_wr(struct target * target, uint8_t *buffer, uint32_t address, uint32_t count, int is_flash_lock){
  int retval = ERROR_OK;
  if (dsp5680xx_target_status(target,NULL,NULL) != TARGET_HALTED){
    retval = eonce_enter_debug_mode(target,NULL);
    err_check_propagate(retval);
  }
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Download the pgm that flashes.
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  uint32_t my_favourite_ram_address = 0x8700; // This seems to be a safe address. This one is the one used by codewarrior in 56801x_flash.cfg
  if(!is_flash_lock){
    retval = dsp5680xx_write(target, my_favourite_ram_address, 1, pgm_write_pflash_length*2,(uint8_t *) pgm_write_pflash);
    err_check_propagate(retval);
    retval = dsp5680xx_execute_queue();
    err_check_propagate(retval);
  }
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Set hfmdiv
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  retval = set_fm_ck_div(target);
  err_check_propagate(retval);
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Setup registers needed by pgm_write_pflash
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

  dsp5680xx_context.flush = 0;

  retval = core_move_long_to_r3(target,address);  // Destination address to r3
  err_check_propagate(retval);
  core_load_TX_RX_high_addr_to_r0(target);  // TX/RX reg address to r0
  err_check_propagate(retval);
  retval = core_move_long_to_r2(target,HFM_BASE_ADDR);// FM base address to r2
  err_check_propagate(retval);
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  // Run flashing program.
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  retval = core_move_value_at_r2_disp(target,0x00,HFM_CNFG); // write to HFM_CNFG (lock=0, select bank)
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x04,HFM_USTAT);// write to HMF_USTAT, clear PVIOL, ACCERR & BLANK bits
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x10,HFM_USTAT);// clear only one bit at a time
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x20,HFM_USTAT);
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x00,HFM_PROT);// write to HMF_PROT, clear protection
  err_check_propagate(retval);
  retval = core_move_value_at_r2_disp(target,0x00,HFM_PROTB);// write to HMF_PROTB, clear protection
  err_check_propagate(retval);
  if(count%2){
    //TODO implement handling of odd number of words.
	retval = ERROR_FAIL;
	err_check(retval,DSP5680XX_ERROR_FLASHING_INVALID_WORD_COUNT,"Cannot handle odd number of words.");
  }

  dsp5680xx_context.flush = 1;
  retval = dsp5680xx_execute_queue();
  err_check_propagate(retval);

  uint32_t drscan_data;
  uint16_t tmp = (buffer[0]|(buffer[1]<<8));
  retval = core_tx_upper_data(target,tmp,&drscan_data);
  err_check_propagate(retval);

  retval = dsp5680xx_resume(target,0,my_favourite_ram_address,0,0);
  err_check_propagate(retval);

  int counter = FLUSH_COUNT_FLASH;
  dsp5680xx_context.flush = 0;
  uint32_t i;
  for(i=1; (i<count/2)&&(i<HFM_SIZE_WORDS); i++){
    if(--counter==0){
      dsp5680xx_context.flush = 1;
      counter = FLUSH_COUNT_FLASH;
    }
    tmp = (buffer[2*i]|(buffer[2*i+1]<<8));
    retval = core_tx_upper_data(target,tmp,&drscan_data);
	if(retval!=ERROR_OK){
	  dsp5680xx_context.flush = 1;
	  err_check_propagate(retval);
	}
	dsp5680xx_context.flush = 0;
  }
  dsp5680xx_context.flush = 1;
  if(!is_flash_lock){
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Verify flash (skip when exec lock sequence)
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uint16_t signature;
    uint16_t pc_crc;
    retval =  dsp5680xx_f_signature(target,address,i,&signature);
    err_check_propagate(retval);
    pc_crc = perl_crc(buffer,i);
    if(pc_crc != signature){
      retval = ERROR_FAIL;
      err_check(retval,DSP5680XX_ERROR_FLASHING_CRC,"Flashed data failed CRC check, flash again!");
    }
  }
  return retval;
}

int dsp5680xx_f_unlock(struct target * target){
  int retval = ERROR_OK;
  uint16_t eonce_status;
  uint32_t instr;
  uint32_t ir_out;
  uint16_t instr_16;
  uint16_t read_16;
  struct jtag_tap * tap_chp;
  struct jtag_tap * tap_cpu;
  tap_chp = jtag_tap_by_string("dsp568013.chp");
  if(tap_chp == NULL){
    retval = ERROR_FAIL;
    err_check(retval,DSP5680XX_ERROR_JTAG_TAP_ENABLE_MASTER,"Failed to get master tap.");
  }
  tap_cpu = jtag_tap_by_string("dsp568013.cpu");
  if(tap_cpu == NULL){
    retval = ERROR_FAIL;
    err_check(retval,DSP5680XX_ERROR_JTAG_TAP_ENABLE_CORE,"Failed to get master tap.");
  }

  retval = eonce_enter_debug_mode(target,&eonce_status);
  if(retval == ERROR_OK){
    LOG_WARNING("Memory was not locked.");
  }

  jtag_add_reset(0,1);
  jtag_add_sleep(TIME_DIV_FREESCALE*200*1000);

  retval = reset_jtag();
  err_check(retval,DSP5680XX_ERROR_JTAG_RESET,"Failed to reset JTAG state machine");
  jtag_add_sleep(150);

  // Enable core tap
  tap_chp->enabled = true;
  retval = switch_tap(target,tap_chp,tap_cpu);
  err_check_propagate(retval);

  instr = JTAG_INSTR_DEBUG_REQUEST;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  jtag_add_sleep(TIME_DIV_FREESCALE*100*1000);
  jtag_add_reset(0,0);
  jtag_add_sleep(TIME_DIV_FREESCALE*300*1000);

  // Enable master tap
  tap_chp->enabled = false;
  retval = switch_tap(target,tap_chp,tap_cpu);
  err_check_propagate(retval);

  // Execute mass erase to unlock
  instr = MASTER_TAP_CMD_FLASH_ERASE;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_MASTER_TAP_IRLEN);
  err_check_propagate(retval);

  instr = HFM_CLK_DEFAULT;
  retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,16);
  err_check_propagate(retval);

  jtag_add_sleep(TIME_DIV_FREESCALE*150*1000);
  jtag_add_reset(0,1);
  jtag_add_sleep(TIME_DIV_FREESCALE*200*1000);

  retval = reset_jtag();
  err_check(retval,DSP5680XX_ERROR_JTAG_RESET,"Failed to reset JTAG state machine");
  jtag_add_sleep(150);

  instr = 0x0606ffff;
  retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,32);
  err_check_propagate(retval);

  // enable core tap
  instr = 0x5;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_MASTER_TAP_IRLEN);
  err_check_propagate(retval);
  instr = 0x2;
  retval =  dsp5680xx_drscan(target,(uint8_t *) & instr,(uint8_t *) & ir_out,4);
  err_check_propagate(retval);

  tap_cpu->enabled = true;
  tap_chp->enabled = false;

  instr = JTAG_INSTR_ENABLE_ONCE;
  //Two rounds of jtag 0x6  (enable eonce) to enable EOnCE.
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  instr = JTAG_INSTR_DEBUG_REQUEST;
  retval =  dsp5680xx_irscan(target, & instr, & ir_out,DSP5680XX_JTAG_CORE_TAP_IRLEN);
  err_check_propagate(retval);
  instr_16 = 0x1;
  retval = dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,8);
	err_check_propagate(retval);
  instr_16 = 0x20;
  retval = dsp5680xx_drscan(target,(uint8_t *) & instr_16,(uint8_t *) & read_16,8);
	err_check_propagate(retval);
  jtag_add_sleep(TIME_DIV_FREESCALE*100*1000);
  jtag_add_reset(0,0);
  jtag_add_sleep(TIME_DIV_FREESCALE*300*1000);
  return retval;
}

int dsp5680xx_f_lock(struct target * target){
  int retval;
  uint16_t lock_word[] = {HFM_LOCK_FLASH};
  retval = dsp5680xx_f_wr(target,(uint8_t *)(lock_word),HFM_LOCK_ADDR_L,2,1);
  err_check_propagate(retval);

  jtag_add_reset(0,1);
  jtag_add_sleep(TIME_DIV_FREESCALE*200*1000);

  retval = reset_jtag();
  err_check(retval,DSP5680XX_ERROR_JTAG_RESET,"Failed to reset JTAG state machine");
  jtag_add_sleep(TIME_DIV_FREESCALE*100*1000);
  jtag_add_reset(0,0);
  jtag_add_sleep(TIME_DIV_FREESCALE*300*1000);

  return retval;
}

static int dsp5680xx_step(struct target * target,int current, uint32_t address, int handle_breakpoints){
  err_check(ERROR_FAIL,DSP5680XX_ERROR_NOT_IMPLEMENTED_STEP,"Not implemented yet.");
}

/** Holds methods for dsp5680xx targets. */
struct target_type dsp5680xx_target = {
  .name = "dsp5680xx",

  .poll = dsp5680xx_poll,
  .arch_state = dsp5680xx_arch_state,

  .target_request_data = NULL,

  .halt = dsp5680xx_halt,
  .resume = dsp5680xx_resume,
  .step = dsp5680xx_step,

  .write_buffer = dsp5680xx_write_buffer,
  .read_buffer = dsp5680xx_read_buffer,

  .assert_reset = dsp5680xx_assert_reset,
  .deassert_reset = dsp5680xx_deassert_reset,
  .soft_reset_halt = dsp5680xx_soft_reset_halt,

  .read_memory = dsp5680xx_read,
  .write_memory = dsp5680xx_write,
  .bulk_write_memory = dsp5680xx_bulk_write_memory,

  .checksum_memory = dsp5680xx_checksum_memory,

  .target_create = dsp5680xx_target_create,
  .init_target = dsp5680xx_init_target,
};
