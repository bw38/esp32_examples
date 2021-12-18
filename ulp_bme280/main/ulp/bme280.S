#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "interface.h"

#define SCL		RTC_IO_SCL
#define SDA		RTC_IO_SDA

//I2C-Addr BME280, Manufactorer-Addr
#define DEV_WR_ADDR (DEV_ADDR<<1) & 0xFE
#define DEV_RD_ADDR (DEV_ADDR<<1) | 0x01


// Macros and defs for lazy writers -----------------------------------------------------

//Port-Status "rtc_port" read => R0
.macro READ_PIN rtc_port
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + \rtc_port, 1)
.endm

//Port-Mode => Input (floating / high impedance)
.macro SET_MODE_INPUT rtc_port
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + \rtc_port, 1, 1)
.endm

//Port-Mode => Output
.macro SET_MODE_OUTPUT rtc_port
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + \rtc_port, 1, 1)
.endm

//Set Port => 1
.macro SET_PIN_HIGH rtc_port
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + \rtc_port, 1, 1)
.endm

//Set Port => 0
.macro SET_PIN_LOW rtc_port
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + \rtc_port, 1, 1)
.endm

#define SDA_H	SET_MODE_INPUT  SDA		//high impedance (external pullup resistor)
#define SDA_L	SET_MODE_OUTPUT SDA		//low impedance -> 0
#define	SCL_H	SET_PIN_HIGH	SCL
#define SCL_L	SET_PIN_LOW		SCL

.macro WR_START
	SDA_H
	SCL_H
	SDA_L
.endm

.macro WR_STOP
	SCL_L
	NOP
	NOP
	SDA_L
	SCL_H
	SDA_H
.endm

.macro WR_BIT_H
	SCL_L
	SDA_H
	SCL_H
.endm

.macro WR_BIT_L
	SCL_L
	SDA_L
	SCL_H
.endm

.macro RD_BIT
	SCL_L
	SDA_H			//SDA Input
	READ_PIN SDA	//SDA R0(0)
	SCL_H
.endm

.macro WR_ACK
	SCL_L
	SDA_L
	SCL_H
.endm

.macro WR_NACK
	SCL_L
	SDA_H
	SCL_H
.endm

.macro RD_ACK
	RD_BIT				// R0 == 0 -> Ok	/ == 1 -> no ACK
	sub r0, r0, 0x01
	jump exit_error, eq
.endm

//Jump-return-addr to R3 (return from subroutines)
//insert before jump to sub
.macro ret_r3
	.set addr, (.+8)
	move r3,addr
.endm

// End Macros --------------------------------------------------------------------------------

//Data-Section Variable ------------------------------------

.bss


//Communiction-Test (ACK from slave)
// 0 -> Ok
	.global com_err
com_err:
	.long 0

//contol oversampling & filter
	.global ctrl_hum	//Reg 0xF2 [2..0]
ctrl_hum:
	.long 0

	.global ctrl_meas	//Reg 0xF4 [7..0]
ctrl_meas:
	.long 0

	.global config		//Reg 0xF5 [4..2]	filter
config:
	.long 0

	.global mraw		//measurement results, raw-data
mraw:
	.long 0		//press_msb
	.long 0		//press lsb
	.long 0		//press xlsb
	.long 0		//temp_msb
	.long 0		//temp_lsb
	.long 0		//temp_xlsb
	.long 0		//hum_msb
	.long 0		//hum_lsb


	//limits for wakeup-conditions
	//set from mainproc after each wakeup
	.global limit	//16bit msb.lsb
limit:
	.long 0		//press low
	.long 0		//press high
	.long 0		//temp  low
	.long 0		//temp  high
	.long 0		//hum   low
	.long 0		//hum   high


	.global wakeup_reason
wakeup_reason:
	.long 0

	//wakeup counter
	.global cnt_wakeup
cnt_wakeup:
	.long 0

//Code-Section -------------------------------------------------

	.text
	.global entry
entry:
	//Reset error-flag
	move r1, 0
	move r2, com_err
	st r1, r2, 0

	//config und start measurement
	WR_START
	move r0, DEV_WR_ADDR //slave-addr
	ret_r3
	jump wr_byte
	RD_ACK				//read ACK from slave
	move r0, 0xF5		//write filter-config
	ret_r3
	jump wr_byte
	RD_ACK
	move r2, config
	ld r0, r2, 0
	ret_r3
	jump wr_byte
	RD_ACK
	move r0, 0xF2		//write h-oversampling
	ret_r3
	jump wr_byte
	RD_ACK
	move r2, ctrl_hum
	ld r0, r2, 0
	ret_r3
	jump wr_byte
	RD_ACK
	move r0, 0xF4		//write p+t oversampling / start in mode
	ret_r3
	jump wr_byte
	RD_ACK
	move r2, ctrl_meas
	ld r0, r2, 0
	ret_r3
	jump wr_byte
	RD_ACK
	WR_STOP

	//wait for ready
k0:	wait 16000	//delay 2ms
	WR_START
	move r0, DEV_WR_ADDR
	ret_r3
	jump wr_byte
	RD_ACK
	move r0, 0xF3		//status register (bit3)
	ret_r3
	jump wr_byte
	RD_ACK
	WR_STOP

	WR_START
	move r0, DEV_RD_ADDR
	ret_r3
	jump wr_byte
	RD_ACK
	ret_r3
	jump rd_byte
	WR_NACK				//close reading
	WR_STOP
	and r0,r0, 0x08		//test bit3 == 0 ?
	jump k1, eq			//ready
	jump k0				//not ready yet


k1:
	//read measure-results
	WR_START
	move r0, DEV_WR_ADDR
	ret_r3
	jump wr_byte
	RD_ACK
	move r0, 0xF7	//start register
	ret_r3
	jump wr_byte
	RD_ACK
	WR_STOP

	WR_START
	move r0, DEV_RD_ADDR
	ret_r3
	jump wr_byte
	RD_ACK
	move r2, mraw	//R2 -> result-var
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 0	//press_msb	0xF7
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 4	//press_lsb	0xF8
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 8	//press_xlsb 0xF9
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 12	//temp_msb 0xFA
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 16	//temp_lsb 0xFB
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 20	//temp_xlsb 0xFC
	ret_r3
	jump rd_byte
	WR_ACK
	st r0, r2, 24	//hum_msb	0xFD
	ret_r3
	jump rd_byte
	WR_NACK			//close reading results
	st r0, r2, 28	//hum_lsb	0xFE
	WR_STOP

	//check counter for wake_up
	move r2, cnt_wakeup
	ld r0, r2, 0
	move r1, 0		//wakeup-reason
	sub r0, r0, 1
	st r0, r2, 0
	jump exit_wake, eq


	//check limits for wakeup main-proc
	move r2, mraw
	move r3, limit
	//press limits	!!!values  low press > high press !!!
	ld r0, r2, 0	//press msb
	ld r1, r2, 4	//press lsb
	lsh r0, r0, 8
	or r0, r0, r1	//R0 = press_msb[F..8]:press_lsb[7..0]
	ld r1, r3, 0	//R1 = press_low limit
	sub r1, r1, r0	//R0 < R1 ?
	move r1, 1		//wakeup_reason
	jump exit_wake, ov
	ld r1, r3, 4	//R1 = press_high limit
	sub r1, r0, r1	//R0 < R1 ?
	move r1, 2
	jump exit_wake, ov

	//temp limits
	ld r0, r2, 12	//temp msb
	ld r1, r2, 16	//temp lsb
	lsh r0, r0, 8
	or r0, r0, r1	//R0 = temp_msb[F..8]:press_lsb[7..0]
	ld r1, r3, 8	//R1 = temp low limit
	sub r1, r0, r1	//R0 < R1 ?
	move r1, 3
	jump exit_wake, ov
	ld r1, r3, 12	//R1 = temp heigh limit
	sub r1, r1, r0	//R0 > R1 ?
	move r1, 4
	jump exit_wake, ov

	//humidity limits
	ld r0, r2, 24	//hum msb
	ld r1, r2, 28	//hum lsb
	lsh r0, r0, 8
	or r0, r0, r1	//R0 = hum_msb[F..8]:press_lsb[7..0]
	ld r1, r3, 16	//R1 = hum low limit
	sub r1, r0, r1	//R0 < R1 ?
	move r1, 5
	jump exit_wake, ov
	ld r1, r3, 20	//R1 = hum heigh limit
	sub r1, r1, r0	//R0 > R1 ?
	move r1, 6
	jump exit_wake, ov

	jump exit_halt


exit_error:
	move r1, 1		//set error-flag
	move r2, com_err
	st r1, r2, 0
	move r1, 0x0F
exit_wake:
	move r3, wakeup_reason
	st r1, r3, 0
	wake

exit_halt:
	SDA_H
	halt



// -----------------------------------------------------
// Subroutines
//Write R0 : Bit 7 .. 0
//R1 changed, R3 return-Addr
wr_byte:
	stage_rst			//Bitzähler => 0
w3:	and r1, r0, 0x80	//Test bit 7
	jump w0, eq
	WR_BIT_H
	jump w1
w0:	WR_BIT_L
w1:	lsh r0, r0, 1		//shift next bit
	stage_inc 1
	jumps w3, 8, lt
	jump r3

//Read 8 Bit -> R0, start MSB
//R1 changed, R3 return-Addr
rd_byte:
	move r1, 0x00
	stage_rst
u0:	lsh r1, r1, 1
	RD_BIT
	and r0, r0, 0x01
	or r1, r0, r1
	stage_inc 1
	jumps u0, 8, lt
	move r0, r1
	jump r3
