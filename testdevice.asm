;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  ADB Test Device
;;;
;


;;; Connections ;;;

;;;                                                          ;;;
;                          .--------.                          ;
;                  Supply -|01 \/ 08|- Ground                  ;
;         ADB <-->    RA5 -|02    07|- RA0    ---> UART TX     ;
;    Data LED <---    RA4 -|03    06|- RA1    <--- UART RX     ;
;             --->    RA3 -|04    05|- RA2    ---> UART CTS    ;
;                          '--------'                          ;
;                                                              ;
;    Data LED is active low.                                   ;
;                                                              ;
;;;                                                          ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_ON
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

;WARNING: do NOT use RA2 for ADB, the Schmitt Trigger takes too long to react
ADB_PIN	equ	RA5	;Pin on PORTA where ADB is connected
LED_PIN	equ	RA4	;Pin on PORTA where data LED is connected
CTS_PIN	equ	RA2	;Pin on PORTA where CTS is connected

			;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too

			;A*_R3H:
A_COL	equ	7	;Set when a collision has occurred
A_EXCEV	equ	6	;Clear when an exceptional event has occurred
A_SRQEN	equ	5	;Set when service requests are enabled


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB state machine
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	A0_R12L	;ADB device 0 lengths (high nibble is R1, low nibble is R2)
	A0_R3H	;ADB device 0 register 3 high byte
	A0_R3L	;ADB device 0 register 3 low byte
	A0_DEFH	;ADB device 0 default handler ID
	A1_R12L	;ADB device 1 lengths (high nibble is R1, low nibble is R2)
	A1_R3H	;ADB device 1 register 3 high byte
	A1_R3L	;ADB device 1 register 3 low byte
	A1_DEFH	;ADB device 1 default handler ID
	T_QPTRS	;Tx queue pointers (high nibble is push, low nibble is pop)
	R_QPUSH	;Rx queue push pointer
	R_QPOP	;Rx queue pop pointer
	
	endc

	;Linear memory:
	;0x2000-0x207F - Inbound string queue
	;0x2080-0x209F - Device 0 handler ID bitmap and default address
	;0x20A0-0x20BF - Device 1 handler ID bitmap and default address
	;                 LSB of first byte is set if handler 0x01 is usable
	;                 Bit 3 of last byte is set if handler 0xFC is usable
	;                 Bits 7:4 of last byte are default address
	;0x20C0-0x20C7 - Device 0 talk register 1 string
	;0x20C8-0x20CF - Device 0 talk register 2 string
	;0x20D0-0x20D7 - Device 1 talk register 1 string
	;0x20D8-0x20DF - Device 1 talk register 2 string
	;0x20E0-0x20EF - UART transmitter queue

	;Packet header for inbound packets:
	;7   - device number
	;6:4 - content:
	;      011: change register 3 behavior
	;            if number of bytes is 0, clear exceptional event flag;
	;            otherwise, set default and current address to first byte
	;            following header; add bytes following first to map of
	;            acceptable handler IDs with last handler ID being set as
	;            default
	;      010: string for talk 2
	;      001: string for talk 1
	;      000: string for talk 0
	;3:0 - number of bytes

	;Packet header for outbound packets:
	;7   - device number
	;6:4 - content:
	;      111: reset (length always 0)
	;      011: successful handler ID change (length always 1)
	;      010: listen register 2
	;      001: listen register 1
	;      000: listen register 0
	;3:0 - number of bytes


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	movlp	0		;Copy the Timer0 flag into the carry bit so it
	bcf	STATUS,C	; doesn't change on us mid-stream
	btfsc	INTCON,TMR0IF	; "
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;If the ADB pin has had a negative or positive
	movlp	0		; edge, handle it as an event for the ADB state
	btfsc	IOCAF,ADB_PIN	; machine
	call	IntAdbEdge	; "
	btfss	INTCON,PEIE	;If peripheral interrupts are disabled, it's
	retfie			; not safe to service the UART right now
	movlb	0		;If the UART transmitter wants a byte, handle
	movlp	0		; it
	btfsc	PIR1,TXIF	; "
	call	IntTx		; "
	movlb	0		;If the UART receiver has a byte, handle it
	btfsc	PIR1,RCIF	; "
	call	IntRx		; "
	retfie

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADB_PIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,ADB_PIN	;Clear the interrupt flag
	btfsc	IOCAN,ADB_PIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return

IntTx
	swapf	T_QPTRS,W	;If the queue is empty, disable the TX
	xorwf	T_QPTRS,W	; interrupt and return
	btfss	STATUS,Z	; "
	bra	IntTx0		; "
	movlb	1		; "
	bcf	PIE1,TXIE	; "
	return			; "
IntTx0	movf	T_QPTRS,W	;Load the Tx pop pointer (low nibble of the
	andlw	B'00001111'	; combined pointer register) into FSR0
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	moviw	FSR0++		;Pop the top byte off the TX queue and load it
	movlb	3		; for transmission, advancing the pointer
	movwf	TXREG		; "
	movlw	B'11110000'	;Copy the advanced pointer back into the low
	andwf	T_QPTRS,F	; nibble of the combined pointer register
	movf	FSR0L,W		; "
	andlw	B'00001111'	; "
	iorwf	T_QPTRS,F	; "
	return

IntRx
	movf	R_QPUSH,W	;Load the queue push point into FSR0 so we can
	movwf	FSR0L		; dereference it
	incf	R_QPUSH,F	;Advance and wrap the queue push point
	bcf	R_QPUSH,7	; "
	movlb	3		;Get the byte from the UART and push it onto the
	movf	RCREG,W		; queue
	movwf	INDF0		; "
	movf	R_QPOP,W	;Queue length is push point minus pop point
	subwf	R_QPUSH,W	; "
	andlw	B'01111111'	; "
	sublw	96		;Set carry if queue length <= 96
	movlb	2		;Deassert CTS, i.e. set it to 1, if queue length
	btfss	STATUS,C	; was > 96, i.e. more than 3/4 full
	bsf	LATA,CTS_PIN	; "
	return


;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	RCSTA		;UART async mode, 115.2 kHz, but receiver not
	movlw	B'01001000'	; enabled just yet
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	68
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10000000'
	movwf	RCSTA
	clrf	TXREG
	
	banksel	IOCAN		;ADB sets IOCAF on negative edge
	movlw	1 << ADB_PIN
	movwf	IOCAN

	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG

	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	LATA		;Ready to pull ADB low when output, CTS asserted
	clrf	LATA

	banksel	TRISA		;TX and CTS and LED out, ADB is open-collector
	movlw	B'00101010'	; output, currently floating
	movwf	TRISA

	banksel	PIE1		;Receive interrupt enabled when peripheral
	movlw	1 << RCIE	; interrupts enabled
	movwf	PIE1

	movlw	12		;Delay approximately 2 ms at an instruction
	movwf	AP_BUF		; clock of 2 MHz until the PLL kicks in and the
PllWait	DELAY	110		; instruction clock gears up to 8 MHz
	decfsz	AP_BUF,F
	bra	PllWait

	movlb	3		;Enable UART receiver
	bsf	RCSTA,CREN
	
	clrf	AP_FLAG		;Set initial values of key globals
	clrf	AP_FSAP
	clrf	A0_R12L
	clrf	A0_R3H
	clrf	A0_R3L
	clrf	A0_DEFH
	clrf	A1_R12L
	clrf	A1_R3H
	clrf	A1_R3L
	clrf	A1_DEFH
	clrf	T_QPTRS
	clrf	R_QPUSH
	clrf	R_QPOP

	movlw	0x20		;Set up FSRs to point more or less permanently
	movwf	FSR0H		; to linear memory
	movwf	FSR1H

	clrf	FSR0L		;Zero out linear memory
ClrLoop	clrf	INDF0
	incfsz	FSR0L,F
	bra	ClrLoop

	movlw	B'11001000'	;On-change interrupt, peripheral interrupts (for
	movwf	INTCON		; UART) and interrupt subsystem on

AdbReset
	bcf	AP_FLAG,AP_RST	;Clear reset flag
	movf	A0_DEFH,W	;Set both devices to their default handlers
	movwf	A0_R3L		; "
	movf	A1_DEFH,W	; "
	movwf	A1_R3L		; "
	movlb	1		;Set both devices to their default addresses
	swapf	0x6F,W		; "
	andlw	B'00001111'	; "
	iorlw	B'01100000'	; "
	movwf	A0_R3H		; "
	movlb	2		; "
	swapf	0x3F,W		; "
	andlw	B'00001111'	; "
	iorlw	B'01100000'	; "
	movwf	A1_R3H		; "
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	movlw	B'01110000'	;Create four mini-packets to be transmitted that
	movwi	FSR0++		; notify host that both devices have been reset
	bcf	FSR0L,4		; and returned to their original handlers
	movlw	B'11110000'	; "
	movwi	FSR0++		; "
	bcf	FSR0L,4		; "
	movlw	B'00110001'	; "
	movwi	FSR0++		; "
	bcf	FSR0L,4		; "
	movf	A0_DEFH,W	; "
	movwi	FSR0++		; "
	bcf	FSR0L,4		; "
	movlw	B'10110001'	; "
	movwi	FSR0++		; "
	bcf	FSR0L,4		; "
	movf	A1_DEFH,W	; "
	movwf	INDF0		; "
	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound packets so they gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	;fall through

CheckCts
	movf	R_QPOP,W	;Queue length is push point minus pop point
	subwf	R_QPUSH,W	; "
	andlw	B'01111111'	; "
	sublw	96		;Set carry if queue length <= 96
	movlb	2		;Assert CTS, i.e. set it to 0, if queue length
	btfsc	STATUS,C	; was <= 96, i.e. less than or equal to 3/4 full
	bcf	LATA,CTS_PIN	; "
	;fall through

Main
	movlb	2		;Turn off LED if it was on
	bsf	LATA,LED_PIN	; "
	call	CanPop		;Skip ahead if the queue is empty or a packet
	btfss	STATUS,C	; has not been fully received
	bra	Main0		; "
	swapf	INDF0,W		;Switch packet handler based on the type (the
	andlw	B'00001111'	; top four bits)
	brw			; "
	bra	Main0		; "
	goto	QueueDev0Talk1	; "
	goto	QueueDev0Talk2	; "
	goto	QueueDev0Set	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
	bra	Main0		; "
	goto	QueueDev1Talk1	; "
	goto	QueueDev1Talk2	; "
	goto	QueueDev1Set	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
	goto	QueueSkip	; "
Main0	btfsc	AP_FLAG,AP_RST	;Branch to reset if a reset was received, else
	bra	AdbReset	; wait until a command was received
	btfss	AP_FLAG,AP_RXCI	; "
	bra	Main		; "
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_RXDI	;Clear other flags too from data activity that
	bcf	AP_FLAG,AP_DONE	; might have happened while waiting for a
	bcf	AP_FLAG,AP_COL	; command
	bcf	AP_FLAG,AP_TXI	; "
	bcf	AP_FLAG,AP_SRQ	;Not calling for service just yet
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	AdbReset	; "
	swapf	A0_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the first device, handle it
	andlw	B'11110000'	; "
	btfsc	STATUS,Z	; "
	goto	AdbDevice0	; "
	swapf	A1_R3H,W	;If the device being addressed matches the
	xorwf	AP_BUF,W	; address of the second device, handle it
	andlw	B'11110000'	; "
	btfsc	STATUS,Z	; "
	goto	AdbDevice1	; "
	btfss	STATUS,C	;If no packet to process on the queue, send no
	goto	Main		; SRQ
	movf	INDF0,W		;If the packet on the top of the queue is not a
	andlw	B'01110000'	; talk 0 packet, don't raise an SRQ, just return
	btfss	STATUS,Z	; to main
	goto	Main		; "
	btfsc	INDF0,7		;If the device whose talk 0 response is on top
	bra	Main1		; of the queue can send an SRQ, then send an
	btfsc	A0_R3H,A_SRQEN	; SRQ, and in either case, return to main
	bsf	AP_FLAG,AP_SRQ	; "
	bra	Main		; "	
Main1	btfsc	A1_R3H,A_SRQEN	; "
	bsf	AP_FLAG,AP_SRQ	; "
	bra	Main		; "

QueueSkip
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	goto	CheckCts	;Return to main

QueueDev0Talk1
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	movlw	B'00001111'	;Zero out the length of the talk 1 response
	andwf	A0_R12L,F	; "
	movlw	0xC0		;Point destination pointer to talk 1 buffer
	movwf	FSR1L		; "
QD0Tk10	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	btfsc	A0_R12L,7	;If the buffer has reached capacity, loop
	bra	QD0Tk10		; without copying a byte
	movf	INDF0,W		;Copy a byte from the packet to the buffer
	movwi	FSR1++		; "
	movlw	0x10		;Increment the buffer length
	addwf	A0_R12L,F	; "
	bra	QD0Tk10		;Loop

QueueDev0Talk2
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	movlw	B'11110000'	;Zero out the length of the talk 2 response
	andwf	A0_R12L,F	; "
	movlw	0xC8		;Point destination pointer to talk 2 buffer
	movwf	FSR1L		; "
QD0Tk20	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	btfsc	A0_R12L,3	;If the buffer has reached capacity, loop
	bra	QD0Tk20		; without copying a byte
	movf	INDF0,W		;Copy a byte from the packet to the buffer
	movwi	FSR1++		; "
	incf	A0_R12L,F	;Increment the buffer length
	bra	QD0Tk20		;Loop

QueueDev1Talk1
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	movlw	B'00001111'	;Zero out the length of the talk 1 response
	andwf	A1_R12L,F	; "
	movlw	0xD0		;Point destination pointer to talk 1 buffer
	movwf	FSR1L		; "
QD1Tk10	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	btfsc	A1_R12L,7	;If the buffer has reached capacity, loop
	bra	QD1Tk10		; without copying a byte
	movf	INDF0,W		;Copy a byte from the packet to the buffer
	movwi	FSR1++		; "
	movlw	0x10		;Increment the buffer length
	addwf	A1_R12L,F	; "
	bra	QD1Tk10		;Loop

QueueDev1Talk2
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	movlw	B'11110000'	;Zero out the length of the talk 2 response
	andwf	A1_R12L,F	; "
	movlw	0xD8		;Point destination pointer to talk 2 buffer
	movwf	FSR1L		; "
QD1Tk20	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	btfsc	A1_R12L,3	;If the buffer has reached capacity, loop
	bra	QD1Tk20		; without copying a byte
	movf	INDF0,W		;Copy a byte from the packet to the buffer
	movwi	FSR1++		; "
	incf	A1_R12L,F	;Increment the buffer length
	bra	QD1Tk20		;Loop

QueueDev0Set
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	bcf	A0_R3H,A_EXCEV	;If the length of this packet is 0, set the
	movf	INDF0,W		; exceptional event flag to 0 and make no other
	andlw	B'00001111'	; changes
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	movlw	0x80		;Zero out the handler ID bitmap
	movwf	FSR1L		; "
QD0Set0	clrf	INDF1		; "
	incf	FSR1L,F		; "
	btfss	FSR1L,5		; "
	bra	QD0Set0		; "
	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	swapf	INDF0,W		;Set the default address
	andlw	B'11110000'	; "
	movlb	1		; "
	movwf	0x6F		; "
	swapf	WREG,W		;Set the current address to the default
	iorlw	B'01100000'	; "
	movwf	A0_R3H		; "
QD0Set1	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	movf	INDF0,W		;Set the current and default handler ID to this
	movwf	A0_DEFH		; (last one wins)
	movwf	A0_R3L		; "
	decf	INDF0,F		;Handler bitmap starts at handler 1
	lsrf	INDF0,W		;Load the upper five bits of the desired handler
	lsrf	WREG,W		; ID into FSR1 to point at the correct byte in
	lsrf	WREG,W		; the 32-byte handler bitmap
	iorlw	B'10000000'	; "
	movwf	FSR1L		; "
	movlw	B'00000001'	;Set the appropriate bit in the byte using the
	btfsc	INDF0,1		; low three bits of the handler ID
	movlw	B'00000100'	; "
	btfsc	INDF0,0		; "
	lslf	WREG,W		; "
	btfsc	INDF0,2		; "
	swapf	WREG,W		; "
	iorwf	INDF1,F		; "
	bra	QD0Set1		;Loop to process the next handler ID

QueueDev1Set
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
	bcf	A1_R3H,A_EXCEV	;If the length of this packet is 0, set the
	movf	INDF0,W		; exceptional event flag to 0 and make no other
	andlw	B'00001111'	; changes
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	movlw	0xA0		;Zero out the handler ID bitmap
	movwf	FSR1L		; "
QD1Set0	clrf	INDF1		; "
	incf	FSR1L,F		; "
	btfss	FSR1L,6		; "
	bra	QD1Set0		; "
	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	swapf	INDF0,W		;Set the default address
	andlw	B'11110000'	; "
	movlb	2		; "
	movwf	0x3F		; "
	swapf	WREG,W		;Set the current address to the default
	iorlw	B'01100000'	; "
	movwf	A1_R3H		; "
QD1Set1	incf	FSR0L,F		;Advance the pointer
	bcf	FSR0L,7		; "
	movf	FSR0L,W		;If the pointer is at the end of the packet,
	xorwf	R_QPOP,W	; break out of the loop
	btfsc	STATUS,Z	; "
	goto	CheckCts	; "
	movf	INDF0,W		;Set the current and default handler ID to this
	movwf	A1_DEFH		; (last one wins)
	movwf	A1_R3L		; "
	decf	INDF0,F		;Handler bitmap starts at handler 1
	lsrf	INDF0,W		;Load the upper five bits of the desired handler
	lsrf	WREG,W		; ID into FSR1 to point at the correct byte in
	lsrf	WREG,W		; the 32-byte handler bitmap
	iorlw	B'10000000'	; "
	movwf	FSR1L		; "
	movlw	B'00000001'	;Set the appropriate bit in the byte using the
	btfsc	INDF0,1		; low three bits of the handler ID
	movlw	B'00000100'	; "
	btfsc	INDF0,0		; "
	lslf	WREG,W		; "
	btfsc	INDF0,2		; "
	swapf	WREG,W		; "
	iorwf	INDF1,F		; "
	bra	QD1Set1		;Loop to process the next handler ID

AdbDevice0
	movf	AP_BUF,W	;Switch handler by the low four bits of the
	andlw	B'00001111'	; command
	brw			; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	AdbDevice0Listen; "
	goto	AdbDevice0Listen; "
	goto	AdbDevice0Listen; "
	goto	AdbDevice0Listn3; "
	goto	AdbDevice0Talk0	; "
	goto	AdbDevice0Talk1	; "
	goto	AdbDevice0Talk2	; "
	goto	AdbDevice0Talk3	; "

AdbDevice1
	movf	AP_BUF,W	;Switch handler by the low four bits of the
	andlw	B'00001111'	; command
	brw			; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	AdbDevice1Listen; "
	goto	AdbDevice1Listen; "
	goto	AdbDevice1Listen; "
	goto	AdbDevice1Listn3; "
	goto	AdbDevice1Talk0	; "
	goto	AdbDevice1Talk1	; "
	goto	AdbDevice1Talk2	; "
	goto	AdbDevice1Talk3	; "

AdbDevice0Listen
	movlb	2		;Device has been hailed, turn data LED on
	bcf	LATA,LED_PIN	; "
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	swapf	AP_BUF,W	;Copy the register being written to into the
	andlw	B'00110000'	; first byte of the outbound packet
	movwf	INDF0		; "
AD0Lsn0	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command or
	btfsc	AP_FLAG,AP_RXCI	; the receive is done, finish up
	bra	AD0Lsn1		; "
	btfsc	AP_FLAG,AP_DONE	; "
	bra	AD0Lsn1		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for a data byte
	bra	AD0Lsn0		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	incf	INDF0,F		;Increment the length of the payload
	movf	INDF0,W		;Advance the pointer to where the next byte will
	andlw	B'00001111'	; be inserted
	addwf	FSR0L,F		; "
	bcf	FSR0L,4		; "
	movf	AP_BUF,W	;Add the received byte to the outbound packet
	movwf	INDF0		; "
	bra	AD0Lsn0		;Loop for the next byte (if any)
AD0Lsn1	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound packet so it gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	goto	Main		;Return to main

AdbDevice0Listn3
	movlb	2		;Device has been hailed, turn data LED on
	bcf	LATA,LED_PIN	; "
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the first data byte
	bra	AdbDevice0Listn3; "
	movf	AP_BUF,W	;Save the first byte of the listen in FSR0 for
	movwf	FSR0L		; later use (as a temp var, no dereferencing)
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
AD0Ls30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	AD0Ls30		; byte
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	AD0Ls32		; unconditionally
	addlw	1		;If handler ID is 0xFF, it means to initiate a
	btfsc	STATUS,Z	; self-test; we ignore this
	goto	Main		; "
	addlw	1		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	AD0Ls31		; detected
	addlw	1		;If handler ID is 0xFD, it means to change the
	btfsc	STATUS,Z	; device's address if its activator is pressed;
	goto	Main		; we ignore this
	decf	AP_BUF,F	;Handler bitmap starts at handler 1
	lsrf	AP_BUF,W	;Load the upper five bits of the desired handler
	lsrf	WREG,W		; ID into FSR0 to point at the correct byte in
	lsrf	WREG,W		; the 32-byte handler bitmap
	iorlw	B'10000000'	; "
	movwf	FSR0L		; "
	movlw	B'00000001'	;Look up the appropriate bit in the byte using
	btfsc	AP_BUF,1	; the low three bits of the handler ID
	movlw	B'00000100'	; "
	btfsc	AP_BUF,0	; "
	lslf	WREG,W		; "
	btfsc	AP_BUF,2	; "
	swapf	WREG,W		; "
	andwf	INDF0,W		; "
	btfsc	STATUS,Z	;If the corresponding bit wasn't set, we ignore
	goto	Main		; this handler ID change and return to main
	incf	AP_BUF,W	;If the corresponding bit was set, change the
	movwf	A0_R3L		; handler ID
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	movlw	B'00110001'	;Create a mini outbound packet for the host that
	movwi	FSR0++		; indicates that handler has been changed
	bcf	FSR0L,4		; "
	incf	AP_BUF,W	; "
	movwf	INDF0		; "
	bra	AD0Lsn1		;Send it and return to main
AD0Ls31	btfss	A0_R3H,A_COL	;If a collision has not been detected, skip
	bra	AD0Ls33		; ahead to change the address; if one has been
	bcf	A0_R3H,A_COL	; detected, clear it and ignore this command
	goto	Main		; "
AD0Ls32	bcf	A0_R3H,A_SRQEN	;Copy the state of the SRQ enable bit to the SRQ
	btfsc	FSR0L,A_SRQEN	; enable flag and to our copy of register 3
	bsf	A0_R3H,A_SRQEN	; "
AD0Ls33	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	FSR0L,F		; byte as our new address and we're done
	movf	A0_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	FSR0L,W		; "
	movwf	A0_R3H		; "
	goto	Main		; "

AdbDevice0Talk0
	btfss	STATUS,C	;If queue is empty or packet is not fully
	goto	Main		; received, we have nothing to say
	movf	INDF0,W		;If the control byte of the next packet is not
	andlw	B'01110000'	; a talk 0 packet, we have nothing to say
	btfss	STATUS,Z	; "
	goto	Main		; "
	btfsc	INDF0,7		;If the control byte of the next packet is for
	bra	AD0Tk03		; the other device, raise a SRQ
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
AD0Tk00	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	CheckCts	; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk02		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD0Tk00		; for a byte
	incf	FSR0L,F		;Advance the FSR to the next byte to send
	bcf	FSR0L,7		; "
	movf	R_QPOP,W	;If we've reached the end of the packet, wait
	xorwf	FSR0L,W		; for transmission to complete
	btfsc	STATUS,Z	; "
	bra	AD0Tk01		; "
	movf	INDF0,W		;Load the byte for transmission and put up the
	movwf	AP_BUF		; flag to say we have
	bsf	AP_FLAG,AP_TXI	; "
	bra	AD0Tk00		;Loop to wait for it to be ready for next byte
AD0Tk01	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	CheckCts	; "
	btfsc	AP_FLAG,AP_DONE	;If transmission completed, return to main
	goto	CheckCts	; "
	btfss	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it,
	bra	AD0Tk01		; else loop to keep waiting until done
AD0Tk02	bsf	A0_R3H,A_COL	;We collided, so set the collision flag and
	goto	CheckCts	; return to main
AD0Tk03	btfsc	A1_R3H,A_SRQEN	;The packet is meant to be sent by the other
	bsf	AP_FLAG,AP_SRQ	; device, so effect an SRQ (if the other device
	goto	Main		; -can- send an SRQ) and return to main

AdbDevice0Talk1
	swapf	A0_R12L,W	;If the length of the talk 1 response is 0, we
	andlw	B'00001111'	; have nothing to say
	btfsc	STATUS,Z	; "
	goto	Main		; "
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movlw	0xC0		;Load the address of the talk 1 response into
	movwf	FSR0L		; FSR0
AD0Tk10	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD0Tk10		; for a byte
	moviw	FSR0++		;Load the next byte for transmission
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
	swapf	A0_R12L,W	;Check if we've reached the end of the response
	xorwf	FSR0L,W		; and loop if we haven't
	andlw	B'00000111'	; "
	btfss	STATUS,Z	; "
	bra	AD0Tk10		; "
AD0Tk11	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD0Tk11		; "
	goto	Main		;Return to main when it is

AdbDevice0Talk2
	movf	A0_R12L,W	;If the length of the talk 2 response is 0, we
	andlw	B'00001111'	; have nothing to say
	btfsc	STATUS,Z	; "
	goto	Main		; "
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movlw	0xC8		;Load the address of the talk 2 response into
	movwf	FSR0L		; FSR0
AD0Tk20	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD0Tk20		; for a byte
	moviw	FSR0++		;Load the next byte for transmission
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
	movf	A0_R12L,W	;Check if we've reached the end of the response
	xorwf	FSR0L,W		; and loop if we haven't
	andlw	B'00000111'	; "
	btfss	STATUS,Z	; "
	bra	AD0Tk20		; "
AD0Tk21	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD0Tk21		; "
	goto	Main		;Return to main when it is

AdbDevice0Talk3
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movf	A0_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	bsf	A0_R3H,A_EXCEV	;Set the exceptional event bit if it was clear
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
AD0Tk30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be done
	bra	AD0Tk30		; "
	movf	A0_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
AD0Tk31	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD0Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD0Tk31		; "
	goto	Main		;Return to main when it is

AdbDevice1Listen
	movlb	2		;Device has been hailed, turn data LED on
	bcf	LATA,LED_PIN	; "
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	swapf	AP_BUF,W	;Copy the register being written to into the
	andlw	B'00110000'	; first byte of the outbound packet
	iorlw	B'10000000'	; "
	movwf	INDF0		; "
AD1Lsn0	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command or
	btfsc	AP_FLAG,AP_RXCI	; the receive is done, finish up
	bra	AD1Lsn1		; "
	btfsc	AP_FLAG,AP_DONE	; "
	bra	AD1Lsn1		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for a data byte
	bra	AD1Lsn0		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	incf	INDF0,F		;Increment the length of the payload
	movf	INDF0,W		;Advance the pointer to where the next byte will
	andlw	B'00001111'	; be inserted
	addwf	FSR0L,F		; "
	bcf	FSR0L,4		; "
	movf	AP_BUF,W	;Add the received byte to the outbound packet
	movwf	INDF0		; "
	bra	AD1Lsn0		;Loop for the next byte (if any)
AD1Lsn1	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound packet so it gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	goto	Main		;Return to main

AdbDevice1Listn3
	movlb	2		;Device has been hailed, turn data LED on
	bcf	LATA,LED_PIN	; "
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the first data byte
	bra	AdbDevice1Listn3; "
	movf	AP_BUF,W	;Save the first byte of the listen in FSR0 for
	movwf	FSR0L		; later use (as a temp var, no dereferencing)
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
AD1Ls30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	AD1Ls30		; byte
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	AD1Ls32		; unconditionally
	addlw	1		;If handler ID is 0xFF, it means to initiate a
	btfsc	STATUS,Z	; self-test; we ignore this
	goto	Main		; "
	addlw	1		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	AD1Ls31		; detected
	addlw	1		;If handler ID is 0xFD, it means to change the
	btfsc	STATUS,Z	; device's address if its activator is pressed;
	goto	Main		; we ignore this
	decf	AP_BUF,F	;Handler bitmap starts at handler 1
	lsrf	AP_BUF,W	;Load the upper five bits of the desired handler
	lsrf	WREG,W		; ID into FSR0 to point at the correct byte in
	lsrf	WREG,W		; the 32-byte handler bitmap
	iorlw	B'10100000'	; "
	movwf	FSR0L		; "
	movlw	B'00000001'	;Look up the appropriate bit in the byte using
	btfsc	AP_BUF,1	; the low three bits of the handler ID
	movlw	B'00000100'	; "
	btfsc	AP_BUF,0	; "
	lslf	WREG,W		; "
	btfsc	AP_BUF,2	; "
	swapf	WREG,W		; "
	andwf	INDF0,W		; "
	btfsc	STATUS,Z	;If the corresponding bit wasn't set, we ignore
	goto	Main		; this handler ID change and return to main
	incf	AP_BUF,W	;If the corresponding bit was set, change the
	movwf	A1_R3L		; handler ID
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	movlw	B'10110001'	;Create a mini outbound packet for the host that
	movwi	FSR0++		; indicates that handler has been changed
	bcf	FSR0L,4		; "
	movf	AP_BUF,W	; "
	movwf	INDF0		; "
	bra	AD1Lsn1		;Send it and return to main
AD1Ls31	btfss	A1_R3H,A_COL	;If a collision has not been detected, skip
	bra	AD1Ls33		; ahead to change the address; if one has been
	bcf	A1_R3H,A_COL	; detected, clear it and ignore this command
	goto	Main		; "
AD1Ls32	bcf	A1_R3H,A_SRQEN	;Copy the state of the SRQ enable bit to the SRQ
	btfsc	FSR0L,A_SRQEN	; enable flag and to our copy of register 3
	bsf	A1_R3H,A_SRQEN	; "
AD1Ls33	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	FSR0L,F		; byte as our new address and we're done
	movf	A1_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	FSR0L,W		; "
	movwf	A1_R3H		; "
	goto	Main		; "

AdbDevice1Talk0
	btfss	STATUS,C	;If queue is empty or packet is not fully
	goto	Main		; received, we have nothing to say
	movf	INDF0,W		;If the control byte of the next packet is not
	andlw	B'01110000'	; a talk 0 packet, we have nothing to say
	btfss	STATUS,Z	; "
	goto	Main		; "
	btfss	INDF0,7		;If the control byte of the next packet is for
	bra	AD1Tk03		; the other device, raise a SRQ
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movf	INDF0,W		;Advance the pop pointer to the next packet in
	andlw	B'00001111'	; the queue
	addlw	1		; "
	addwf	R_QPOP,F	; "
	bcf	R_QPOP,7	; "
AD1Tk00	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	CheckCts	; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk02		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD1Tk00		; for a byte
	incf	FSR0L,F		;Advance the FSR to the next byte to send
	bcf	FSR0L,7		; "
	movf	R_QPOP,W	;If we've reached the end of the packet, wait
	xorwf	FSR0L,W		; for transmission to complete
	btfsc	STATUS,Z	; "
	bra	AD1Tk01		; "
	movf	INDF0,W		;Load the byte for transmission and put up the
	movwf	AP_BUF		; flag to say we have
	bsf	AP_FLAG,AP_TXI	; "
	bra	AD1Tk00		;Loop to wait for it to be ready for next byte
AD1Tk01	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	CheckCts	; "
	btfsc	AP_FLAG,AP_DONE	;If transmission completed, return to main
	goto	CheckCts	; "
	btfss	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it,
	bra	AD1Tk01		; else loop to keep waiting until done
AD1Tk02	bsf	A1_R3H,A_COL	;We collided, so set the collision flag and
	goto	CheckCts	; return to main
AD1Tk03	btfsc	A0_R3H,A_SRQEN	;The packet is meant to be sent by the other
	bsf	AP_FLAG,AP_SRQ	; device, so effect an SRQ (if the other device
	goto	Main		; -can- send an SRQ) and return to main

AdbDevice1Talk1
	swapf	A1_R12L,W	;If the length of the talk 1 response is 0, we
	andlw	B'00001111'	; have nothing to say
	btfsc	STATUS,Z	; "
	goto	Main		; "
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movlw	0xD0		;Load the address of the talk 1 response into
	movwf	FSR0L		; FSR0
AD1Tk10	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD1Tk10		; for a byte
	moviw	FSR0++		;Load the next byte for transmission
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
	swapf	A1_R12L,W	;Check if we've reached the end of the response
	xorwf	FSR0L,W		; and loop if we haven't
	andlw	B'00000111'	; "
	btfss	STATUS,Z	; "
	bra	AD1Tk10		; "
AD1Tk11	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD1Tk11		; "
	goto	Main		;Return to main when it is

AdbDevice1Talk2
	movf	A1_R12L,W	;If the length of the talk 2 response is 0, we
	andlw	B'00001111'	; have nothing to say
	btfsc	STATUS,Z	; "
	goto	Main		; "
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movlw	0xD8		;Load the address of the talk 2 response into
	movwf	FSR0L		; FSR0
AD1Tk20	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AD1Tk20		; for a byte
	moviw	FSR0++		;Load the next byte for transmission
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
	movf	A1_R12L,W	;Check if we've reached the end of the response
	xorwf	FSR0L,W		; and loop if we haven't
	andlw	B'00000111'	; "
	btfss	STATUS,Z	; "
	bra	AD1Tk20		; "
AD1Tk21	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD1Tk21		; "
	goto	Main		;Return to main when it is

AdbDevice1Talk3
	movlb	2		;Device has been hailed and has data, turn data
	bcf	LATA,LED_PIN	; LED on
	movf	A1_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	bsf	A1_R3H,A_EXCEV	;Set the exceptional event bit if it was clear
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
AD1Tk30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be done
	bra	AD1Tk30		; "
	movf	A1_R3L,W	;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
AD1Tk31	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AD1Tk01		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD1Tk31		; "
	goto	Main		;Return to main when it is

;Set carry if the current packet has been fully received and can be popped, else
; clear carry.  Leaves FSR0 pointing to control byte if carry is set.
CanPop
	bcf	STATUS,C	;If the queue is empty, return with carry clear
	movf	R_QPOP,W	; "
	xorwf	R_QPUSH,W	; "
	btfsc	STATUS,Z	; "
	return			; "
	movf	R_QPOP,W	;Copy the queue pop point into FSR0 so we can
	movwf	FSR0L		; dereference the packet's control byte
	bcf	INTCON,PEIE	;Disable UART interrupts so we can change QPUSH
	subwf	R_QPUSH,W	;If the queue push point is less than it, set
	btfss	STATUS,C	; the MSB of the push point so we can reckon
	bsf	R_QPUSH,7	; distance
	incf	INDF0,W		;Get the length of the next packet from its
	andlw	B'00001111'	; control byte, incremented by one
	addwf	R_QPOP,W	;Add it to the pop point and compare; if push
	subwf	R_QPUSH,W	; point is past the end of the packet, set carry
	bcf	R_QPUSH,7	;Clear the MSB of the push point if it was set
	bsf	INTCON,PEIE	;Reenable UART interrupts
	return


;;; State Machines ;;;

AdbFsa	org	0xF00

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,ADB_PIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,ADB_PIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-128		;Shorten the timeout period to 512 us, which is
	movwf	TMR0		; still too long to wait for a transmission
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	bsf	INTCON,TMR0IE	;Set timer to interrupt and bring us to the
	retlw	low AdbFsaTxBitD; transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,ADB_PIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,ADB_PIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,ADB_PIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,ADB_PIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,ADB_PIN	; set doesn't immediately get reset
	bcf	IOCAF,ADB_PIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBD0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBD0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "


;;; End of Program ;;;
	end
