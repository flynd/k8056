; ### Version 1 (2012-02-29) ###
; First version.
;
; ### Version 2 (2023-05-16) ###
; Clean up code style.
; Revise code to lookup text messages to reduce code size.
; Skip OK output after empty input line to better handle LF+CR input.
; Add validation of argument to E command.
; Fix parsing command A treated argument values 1 and 2 as 3.
; Fix missing string terminator after ^C.
; Treat input 0x7f as a backspace character to support more terminals.
; Fix parsing command P didn't reject 0 as the relay argument.


		ERRORLEVEL	-302		; Remove message about using bank 1
		LIST		P=PIC16F616
		include		<p16f616.inc>
		radix		dec
		__CONFIG	_BOR_OFF & _IOSCFS8 & _CP_OFF & _MCLRE_OFF & _PWRTE_OFF & _WDT_ON & _INTOSCIO


; *** I/O Ports ***
; RA0	Out	Relay 3
; RA1	Out	Relay 4
; RA2	In	Button "Test"
; RA3	In	UART RX
; RA4	Out	UART TX
; RA5	Out	LED "Data in"
; RC0	Out	Relay 5
; RC1	Out	Relay 6
; RC2	Out	Relay 7
; RC3	Out	Relay 8
; RC4	Out	Relay 2
; RC5	Out	Relay 1

; *** Resources ***
; WDT	Watchdog @ 2.3s
; TMR0	-
; TMR1	Delay timer @ 128Hz (with help of CCP1)
; TMR2	UART clock @ 3x baudrate
; CCP1	TMR1 period (Compare mode with TMR1 reset)
; CMP1	-
; CMP2	-
; ADC	-
; VREG	-


; *** Constants ***
UARTRATE	EQU	69			; 8000000/4/(9600*3) = 69.44 ticks per interrupt for 9600bps
TMRRATE		EQU	15625			; 8000000/4/128 = 15625 ticks for 128 CCP events per second

PIN_BTN		EQU	2
PIN_RX		EQU	3
PIN_TX		EQU	4
PIN_LED		EQU	5

; Define polarity for UART pins (mark=idle,stop,high, space=start,low)
#DEFINE	SKIP_RX_MARK	btfsc	PORTA, PIN_RX
#DEFINE	SKIP_RX_SPACE	btfss	PORTA, PIN_RX
#DEFINE	SET_TX_MARK	bcf	PORTA, PIN_TX
#DEFINE	SET_TX_SPACE	bsf	PORTA, PIN_TX
#DEFINE	TX_MARK_VAL	0


; Convenience instruction to swap nibbles of a constant
swaplw	MACRO	lit
	movlw	(((lit>>4) & 0x0f) | ((lit<<4) & 0xf0))
	ENDM


; *** RAM ***

sendq		EQU	0xA0			; Queue for messages and data to transmit
SQ_LEN		EQU	0x20
inbuf		EQU	0x20			; Buffer for received command line
IN_LEN		EQU	0x30
timers		EQU	0x50			; Scheduled actions (3 bytes each)
TMR_CNT		EQU	10			;   0: countdown (16 bits), 2: flags (7-4: action, 3-0: relay)
; bit 7 - New (set during allocation, cleared when command line is performed or aborted)
; bit 6:4 - Action (000: Relay off, 001: Relay on, 010: Toggle relay, 011: Get relay, ...)
; bit 3:0 - Argument (relay number)

sq_ptr		EQU	0x6e			; Ptr to next byte to write in send queue
in_ptr		EQU	0x6f			; Ptr to next byte to write in input buffer

; Temporary variables during command parsing 
cmd_tmp1	EQU	inbuf			; Temporary register (not used until after parsing first command char)
cmd_tmp2	EQU	in_ptr			; Temporary register (in_ptr is reset at end of command parsing)
cmd_timel	EQU	0x70			; Time delay (low byte)
cmd_timeh	EQU	0x71			; Time delay (high byte)
cmd_newst	EQU	0x72			; New relay states

butact		EQU	0x73			; Action to perform when button is pressed
; bit 7 - Perform reverse action when button released
; bit 6:4 - Action (000: Reserved, 001: Relay off, 010: Relay on, 011: Toggle relay, ...)
; bit 3:0 - Argument (relay number or zero if button disabled)

tmp1		EQU	0x74			; Temporary register
relays		EQU	0x75			; Current state of relays
sendoffl	EQU	0x76			; Offset in current message for next char to send (low byte)
sendoffh	EQU	0x77			; Offset in current message for next char to send (high byte) (low 4 bits only)
btncnt		EQU	0x77			; Counter for stabilizing button (high 4 bits only)
flags		EQU	0x78			; Various flags
fl_inof		EQU	3			;   Set if input buffer overflow occured (Not used during cmd parsing)
fl_delay	EQU	3			;   Set if cmd_time > 0 (Only used during cmd parsing)
fl_btn		EQU	PIN_BTN			;   Last known stable state of button
fl_echo		EQU	1			;   Set if input on RX should be echoed back on TX
fl_tmp		EQU	0			;   Temp status bit (used in TimerTick)

; UART variables
uartfl		EQU	0x78			; UART flags
uf_trdy		EQU	7			;   Set when TX is ready for next byte, clear to start sending
uf_rrdy		EQU	6			;   Set when new byte is ready in uartin
uf_brk		EQU	5			;   Set when break received
uf_rse		EQU	4			;   Set when RX is waiting for stop bit
uartin		EQU	0x79			; Data input from UART (Note: Nibbles are swapped)
uartout		EQU	0x7a			; Data to send by UART
uartrsr		EQU	0x7b			; UART internal input shift register
tx_cnt		EQU	0x7c			; Bits left to sent
tx_state	EQU	0x7d			; State of UART TX
rx_state	EQU	0x7e			; State of UART RX
isr_w		EQU	0x7f			; Saved W during interrupts



		ORG	0x0000
		movlw	TX_MARK_VAL<<PIN_TX	; Init state of PORTA: LED=Off, TX=Idle, Relays[3,4]=Off
		btfsc	STATUS, NOT_TO		; Different init if reset was due to watchdog
		goto	Init_NoWDT
		goto	Init_WDT


; *** Interrupt handler ***
; Time: 6 + 3..9 + 3 + 3..9 + 3 = 17..30 ticks
		ORG	0x0004
		movwf	isr_w			; Save W
		swapf	isr_w, F
		bcf	PIR1, TMR2IF		; Ack TMR2 interrupt
		swapf	tx_state, W
		movwf	PCL

; TX is Idle, if new byte is ready to send output start bit
TX_Idle		btfsc	uartfl, uf_trdy
		goto	TX_IdleF
		SET_TX_SPACE			; Output start bit
		movlw	10			; 10 bits until idle again
		movwf	tx_cnt
		swaplw	TX_Wait1		; idle & start_send => wait1
		movwf	tx_state
TX_IdleF	swapf	rx_state, W
		movwf	PCL

; TX is waiting to send next bit, flag output buffer as empty if next bit is stop
TX_Wait1	bsf	uartfl, uf_trdy		; cnt=1 => Flag send buffer available for next byte
		decfsz	tx_cnt, W
		bcf	uartfl, uf_trdy
		swaplw	TX_Wait2		; wait1 => wait2
		movwf	tx_state
		swapf	rx_state, W
		movwf	PCL

; TX is waiting to send next bit, decrease bit counter and go to idle if all bits are sent
TX_Wait2	swaplw	TX_Idle			; wait2 & cnt>0 => send
		decfsz	tx_cnt, F
		swaplw	TX_Send			; wait2 & cnt=0 => idle
		movwf	tx_state
		swapf	rx_state, W
		movwf	PCL

; TX is active, send next data or stop bit
TX_Send		btfsc	uartout, 0		; Output next bit (stop will be shifted as ninth bit)
		SET_TX_MARK
		btfss	uartout, 0
		SET_TX_SPACE
		rrf	uartout, F		; Prepare next bit
		rlf	uartout, W		; Restore CF
		bsf	uartout, 7		; Make sure future stop bit is high
		swaplw	TX_Wait1		; send => wait1
		movwf	tx_state
		swapf	rx_state, W
		movwf	PCL

; RX is waiting for start bit
RX_WaitStart	swaplw	RX_Confirm		; wait_start & input => confirm_start
		SKIP_RX_MARK			; wait_start & ~input => wait_start
		movwf	rx_state
		swapf	isr_w, W
		retfie

; RX has got start bit, verify start bit still present and prepare for receive
RX_Confirm	swaplw	RX_Ignore1		; confirm_start & input => ignore1
		SKIP_RX_SPACE
		swaplw	RX_WaitStart		; confirm_start & ~input => wait_start
		movwf	rx_state
		movlw	0x80			; Prepare receive buffer
		movwf	uartrsr
		bcf	uartfl, uf_rse		; Not waiting for stop bit
		swapf	isr_w, W
		retfie

; RX is waiting to receive next bit
RX_Ignore1	swaplw	RX_Ignore2		; ignore1 => ignore2
		movwf	rx_state
		swapf	uartrsr, W		; Copy received byte to input buffer if next bit is stop
		btfsc	uartfl, uf_rse
		movwf	uartin
		swapf	isr_w, W
		retfie

; RX is waiting to receive next bit
RX_Ignore2	swaplw	RX_Sample		; ignore2 & ~full => sample
		btfsc	uartfl, uf_rse
		swaplw	RX_StopExp		; ignore2 & full => stop_exp
		movwf	rx_state
		swapf	isr_w, W
		retfie

; RX is active, get next data bit
RX_Sample	swaplw	RX_Ignore1		; sample & ~full => ignore1
		movwf	rx_state
		btfsc	uartrsr, 0		; Low bit is set if this is last data bit
		bsf	uartfl, uf_rse
		rrf	uartrsr, F
		rlf	uartrsr, W		; Restore CF
		bcf	uartrsr, 7
		SKIP_RX_SPACE
		bsf	uartrsr, 7
		swapf	isr_w, W
		retfie

; RX is expecting stop bit
RX_StopExp	SKIP_RX_MARK
		goto	RX_StopExpF
		bsf	uartfl, uf_rrdy		; Input byte ready
		swaplw	RX_WaitStart		; stop_exp & ~input => wait_start
		movwf	rx_state
		swapf	isr_w, W
		retfie

RX_StopExpF	bsf	uartfl, uf_brk		; Signal break (only if uartin==0xFF)
		swaplw	RX_Break		; stop_exp & input => break
		movwf	rx_state
		swapf	isr_w, W
		retfie

; RX is waiting for break signal (or frame error) to end
RX_Break	swaplw	RX_WaitStart		; break & input => break
		SKIP_RX_SPACE
		movwf	rx_state		; break & ~input => wait_start
		swapf	isr_w, W
		retfie


IF ($>>8) != 0
	ERROR "Calculated jump crosses page boundary"
ENDIF


; *** Initialization ***
Init_WDT	btfsc	relays, 2		; PORTA 0 - Relay 3
		iorlw	0x01
		btfsc	relays, 3		; PORTA 1 - Relay 4
		iorlw	0x02
		movwf	PORTA
		swapf	relays, W		; PORTC 0:3 - Relays 5:8
		andlw	0x0f
		btfsc	relays, 1		; PORTC 4 - Relay 2
		iorlw	0x10
		btfsc	relays, 0		; PORTC 5 - Relay 1
		iorlw	0x20
		movwf	PORTC
		goto	Init_PortsDone

Init_NoWDT	movwf	PORTA			; Power on reset - Set all relays off
		clrf	PORTC
		clrf	relays

Init_PortsDone	bsf	STATUS, RP0		; ** High bank **
		clrf	TRISC			; RC0:5 - Relay controls
		movlw	1<<PIN_BTN | 1<<PIN_RX	; RA0:1 - Relay controls, RA2 - Button, RA3 - RX, RA4 - TX, RA5 - LED
		movwf	TRISA
		clrf	ANSEL			; All ports are digital
		movlw	b'01011111'		; Enable RA pullups, INT on rising edge, TMR0 internal, WDT @ 2.3s
		movwf	OPTION_REG
		movlw	UARTRATE-1		; TMR2 period for UART
		movwf	PR2
		movlw	1<<TMR2IE		; Enable TMR2 interrupt
		movwf	PIE1
		bcf	STATUS, RP0		; ** Low bank **
		movlw	b'00000100'		; Enable TMR2, no postscaler, no prescaler
		movwf	T2CON
		movlw	b'00001011'		; CCP1 in Compare mode with special event trigger
		movwf	CCP1CON
		movlw	LOW(TMRRATE-1)		; TMR1 period for delayed actions
		movwf	CCPR1L
		movlw	HIGH(TMRRATE-1)
		movwf	CCPR1H
		movlw	b'00000001'		; Enable TMR1, internal clock, prescaler 1:1
		movwf	T1CON

		clrf	PCLATH
		swaplw	RX_WaitStart		; UART RX = Idle
		movwf	rx_state
		swaplw	TX_Wait1		; UART TX = Idle for one byte of time before starting to send
		movwf	tx_state
		movlw	16			; Wait 10 bits
		movwf	tx_cnt
		movlw	0xff			; All bits 1 = Keep TX line in idle state
		movwf	uartout
		clrf	uartfl			; Default state, TX is busy until idle bits are done
		clrf	butact			; Button is inactive

		movlw	1<<GIE | 1<<PEIE	; Enable interrupts
		movwf	INTCON

		call	ClearTimers		; Init timers
		call	ClearQueues		; Reset input buffer and send queue

		movlw	MSG_RESET		; Queue reset message
		btfss	STATUS, NOT_TO
		movlw	MSG_RESETWDT		; Different message if it was due to WDT
		call	QueueByte

; *** Main loop ***
Main		clrwdt
		btfsc	PIR1, CCP1IF
		call	TimerTick		; Decrease all active timers
		btfsc	uartfl, uf_brk
		call	Break			; UART has received a break
		btfsc	uartfl, uf_rrdy
		call	DataIn			; UART has received a byte
		btfsc	uartfl, uf_trdy
		call	DataOut			; UART is ready to send next byte
		goto	Main


; *** Break received on UART ***
Break		bcf	uartfl, uf_brk		; Clear break signal
		incfsz	uartin, W
		return				; Data bits != 0xFF => not a break, just a frame error

CtrlC		call	ClearQueues
		movlw	MSG_BREAK		; Queue reply
		goto	QueueByte		; call+return

ClearQueues	movlw	inbuf			; Clear input buffer
		movwf	in_ptr
		bcf	flags, fl_inof		; Clear input buffer overflow
		movlw	sendq			; Clear send queue
		movwf	sq_ptr
		movlw	0xf0
		andwf	sendoffh, F
		return


; *** Valid byte received on UART ***
DataIn		bcf	uartfl, uf_rrdy		; Clear received flag
		swapf	uartin, W
		xorlw	0x7f			; 0x7F = Backspace
		btfsc	STATUS, Z
		goto	Backspace
		swapf	uartin, W
		andlw	0xe0
		btfss	STATUS, Z		; Check for control codes (ascii < 0x20)
		goto	DI_NotCtrl

		swapf	uartin, W
		xorlw	0x03			; 0x03 = Ctrl+C
		btfsc	STATUS, Z
		goto	CtrlC
		xorlw	0x03 ^ 0x08		; 0x08 = Backspace
		btfsc	STATUS, Z
		goto	Backspace
		xorlw	0x08 ^ 0x0a		; 0x0A = Line Feed
		btfsc	STATUS, Z
		goto	Execute
		xorlw	0x0a ^ 0x0d		; 0x0D = Carriage Return
		btfsc	STATUS, Z
		goto	Execute
		swaplw	' '			; Unknown control code, treat as space
		movwf	uartin

DI_NotCtrl	movf	in_ptr, W
		movwf	FSR
		xorlw	inbuf + IN_LEN - 1	; Make sure input buffer isn't already full
		btfsc	STATUS, Z
		goto	DI_Overflow
		swapf	uartin, W		; Add received byte to input buffer
		movwf	INDF
		btfsc	INDF, 7
		movlw	'?'			; Byte with high bit will break output queue, but it's not legal in command anyway
		btfsc	flags, fl_echo
		call	QueueByte		; Echo character back
		incf	in_ptr, F
		return

DI_Overflow	bsf	flags, fl_inof		; Mark input buffer as invalid
		return

Backspace	movf	in_ptr, W
		xorlw	inbuf
		btfss	STATUS, Z		; If buffer isn't empty
		decf	in_ptr, F		;   forget last character
		movlw	MSG_BACKSPACE
		btfsc	flags, fl_echo
		call	QueueByte
		return


Execute		movlw	MSG_NEWLINE
		btfsc	flags, fl_echo
		call	QueueByte	
		btfsc	flags, fl_inof		; Check if input buffer overflowed
		goto	Exec_Overflow
		bsf	PORTA, PIN_LED		; Turn activity LED on
		movf	in_ptr, W
		movwf	FSR
		xorlw	inbuf
		btfsc	STATUS, Z
		return
		clrf	INDF			; Put a null terminator at end of string
		movlw	inbuf
		movwf	FSR			; FSR -> Start of input buffer
		clrf	cmd_timel		; Command delay starts at zero
		clrf	cmd_timeh
		movf	relays, W		; Set initial relay state to current
		movwf	cmd_newst

Exec_Loop	btfsc	INDF, 7			; Only ascii allowed, any byte with high bit set is invalid
		goto	Exec_Invalid
		btfss	INDF, 6			; Letters (0x40-0x6F) are commands
		goto	Exec_NotCmd
		movf	INDF, W
		incf	FSR, F
		andlw	0x5f			; Ignore case
		xorlw	'A'
		btfsc	STATUS, Z
		goto	Exec_CmdA
		xorlw	'A' ^ 'B'
		btfsc	STATUS, Z
		goto	Exec_CmdB
		xorlw	'B' ^ 'C'
		btfsc	STATUS, Z
		goto	Exec_CmdC
		xorlw	'C' ^ 'E'
		btfsc	STATUS, Z
		goto	Exec_CmdE
		xorlw	'E' ^ 'G'
		btfsc	STATUS, Z
		goto	Exec_CmdG
		xorlw	'G' ^ 'H'
		btfsc	STATUS, Z
		goto	Exec_CmdH
		xorlw	'H' ^ 'P'
		btfsc	STATUS, Z
		goto	Exec_CmdP
		xorlw	'P' ^ 'S'
		btfsc	STATUS, Z
		goto	Exec_CmdS
		xorlw	'S' ^ 'T'
		btfsc	STATUS, Z
		goto	Exec_CmdT
		xorlw	'T' ^ 'V'
		btfsc	STATUS, Z
		goto	Exec_CmdV
		xorlw	'V' ^ 'W'
		btfsc	STATUS, Z
		goto	Exec_CmdW
Exec_Invalid	movlw	ERR_INVALID
		goto	Exec_Abort


Exec_CmdA	movlw	1
		movwf	cmd_tmp1
Exec_CmdA_Loop	call	GetArg			; Argument X: relay X state (optional)
		btfsc	STATUS, C
		movlw	3			; Default = 3 (unchanged)
		swapf	cmd_tmp1, F
		iorwf	cmd_tmp1, F
		swapf	cmd_tmp1, F
		btfss	cmd_tmp1, 4
		goto	Exec_CmdA_Not3
		btfsc	cmd_tmp1, 5
		goto	Exec_CmdA_Skip		; State 3 = Unchanged
Exec_CmdA_Not3	addlw	-4			; Check for state >= 4 (invalid)
		btfsc	STATUS, C
		goto	Exec_BadArg
		call	AddCommand		; Apply or schedule command
		btfsc	flags, fl_tmp
		goto	Exec_Abort

Exec_CmdA_Skip	bcf	cmd_tmp1, 4		; Clear previous command
		bcf	cmd_tmp1, 5
		incf	cmd_tmp1, F		; Next relay
		btfss	cmd_tmp1, 3
		goto	Exec_CmdA_Loop
		btfss	cmd_tmp1, 0
		goto	Exec_CmdA_Loop
		goto	Exec_Loop


Exec_CmdB	btfsc	flags, fl_delay		; Delayed button control not supported
		goto	Exec_BadDelay
		call	GetArg			; Argument 1: relay
		btfsc	STATUS, C
		movlw	0			; Default = 0 (disable)
		movwf	butact
		call	GetArg			; Argument 2: Action
		btfsc	STATUS, C
		movlw	2			; Default = 2 (Toggle)
		andlw	0x07
		swapf	butact, F
		iorwf	butact, F
		swapf	butact, F
		addlw	-3			; Check for state >= 3 (invalid)
		btfsc	STATUS, C
		goto	Exec_BadArg
		call	GetArg			; Argument 3: Simple/Pulse
		btfsc	STATUS, C
		goto	Exec_Loop		; Default = 0 (simple)
		btfss	STATUS, Z
		bsf	butact, 7
		addlw	-2			; Check for unbutton action >= 2 (invalid)
		btfsc	STATUS, C
		goto	Exec_BadArg
		goto	Exec_Loop


Exec_CmdC	btfsc	flags, fl_delay		; Delayed timer clearing not supported
		goto	Exec_BadDelay
		movf	FSR, W			; Save FSR
		movwf	cmd_tmp2
		call	GetArg			; Argument 1: timer number
		btfsc	STATUS, C
		goto	Exec_CmdC_All		; Default = all timers

		movwf	FSR			; FSR = num
		addwf	FSR, F			; FSR = num*2
		addlw	timers			; W = timers + num
		addwf	FSR, F			; FSR -> timers + num*3
		clrf	INDF
		incf	FSR, F
		clrf	INDF
		movf	cmd_tmp2, W		; Restore FSR
		addlw	1			; Add one since we lost the addition done by GetArg
		movwf	FSR
		goto	Exec_Loop

Exec_CmdC_All	call	ClearTimers		; Clear all timers
		movf	cmd_tmp2, W		; Restore FSR
		movwf	FSR
		goto	Exec_Loop


Exec_CmdE	btfsc	flags, fl_delay		; Delayed toggling of echo not supported
		goto	Exec_BadDelay
		call	GetArg			; Argument: enable/disable (optional)
		btfsc	STATUS, C
		movlw	2			; Default = 2 (toggle)
		addlw	-3
		btfsc	STATUS, C
		goto	Exec_BadArg		; Invalid if >2
		addlw	3
		btfsc	STATUS, Z
		bcf	flags, fl_echo		; 0 = Disable echo
		addlw	-1
		btfsc	STATUS, Z
		bsf	flags, fl_echo		; 1 = Enable echo
		addlw	-1
		movlw	1<<fl_echo
		btfsc	STATUS, Z
		xorwf	flags, F		; 2 = Toggle echo

		movlw	MSG_ECHOOFF
		btfsc	flags, fl_echo
		movlw	MSG_ECHOON
		call	Cmd_QueueByte
		goto	Exec_Loop


Exec_CmdG	call	GetArg			; Argument: relay (optional)
		btfsc	STATUS, C
		movlw	9			; Default = 9 (all)
		iorlw	0x30			; Command 011 = Get
		movwf	cmd_tmp1
		call	AddCommand		; Apply or schedule command
		btfsc	flags, fl_tmp
		goto	Exec_Abort
		goto	Exec_Loop


Exec_CmdH	btfsc	flags, fl_delay		; Delayed printing of help not supported
		goto	Exec_BadDelay
		movlw	MSG_HELP		; Queue help message
		call	Cmd_QueueByte
		goto	Exec_Loop


Exec_CmdP	call	GetArg			; Argument 1: relay (mandatory)
		btfsc	STATUS, C
		goto	Exec_NoArg
		btfsc	STATUS, Z
		goto	Exec_BadArg
		movwf	cmd_tmp1		; LOW(cmd_tmp1) = relay
		call	GetArgDelay		; Argument 2: time (optional) (stored as shifted bit in tmp1)
		call	GetArg			; Argument 3: state (optional)
		btfsc	STATUS, C
		movlw	2			; Default state = 2 (toggle)
		swapf	cmd_tmp1, F
		iorwf	cmd_tmp1, F
		swapf	cmd_tmp1, F
		addlw	-3			; Check for state >= 3 (invalid)
		btfsc	STATUS, C
		goto	Exec_BadArg
		call	AddCommand		; Apply or schedule first command
		btfsc	flags, fl_tmp
		goto	Exec_Abort

		movf	tmp1, W			; Add time argument to command delay
		andlw	0x3f
		btfsc	tmp1, 7
		goto	Exec_CmdP_HiT1
		swapf	tmp1, W			; Time 0-3, apply in low byte
		addwf	cmd_timel, F
		clrw
		btfsc	STATUS, C
		addlw	1
Exec_CmdP_HiT1	addwf	cmd_timeh, F		; W is either bits for high byte (time 4-9) or carry from low byte
		btfsc	STATUS, C
		goto	Exec_LongTime

		movlw	0x10			; Invert state unless 2 (toggle)
		btfss	cmd_tmp1, 5
		xorwf	cmd_tmp1, F
		call	AddCommand		; Schedule inverse command
		btfsc	flags, fl_tmp
		goto	Exec_Abort

		movf	tmp1, W			; Restore delay by subtracting time argument
		andlw	0x3f
		btfsc	tmp1, 7
		goto	Exec_CmdP_HiT2
		swapf	tmp1, W
		subwf	cmd_timel, F
		clrw
		btfss	STATUS, C
		addlw	1
Exec_CmdP_HiT2	subwf	cmd_timeh, F
		goto	Exec_Loop


Exec_CmdS	call	GetArg			; Argument 1: relay (mandatory)
		btfsc	STATUS, C
		goto	Exec_NoArg
		btfsc	STATUS, Z
		goto	Exec_BadArg
		movwf	cmd_tmp1
		call	GetArg			; Argument 2: state (optional)
		btfsc	STATUS, C
		movlw	2			; Default state = 2 (toggle)
		swapf	cmd_tmp1, F
		iorwf	cmd_tmp1, F
		swapf	cmd_tmp1, F
		addlw	-3			; Check for state >= 3 (invalid)
		btfsc	STATUS, C
		goto	Exec_BadArg
		call	AddCommand		; Apply or schedule command
		btfsc	flags, fl_tmp
		goto	Exec_Abort
		goto	Exec_Loop


Exec_CmdT	btfsc	flags, fl_delay		; Delayed printing of timers not supported
		goto	Exec_BadDelay
		call	GetArg			; Argument: timer
		btfsc	STATUS, C
		goto	Exec_CmdT_All		; Default timer = all
		call	PrintTimer
		goto	Exec_Loop

Exec_CmdT_All	movlw	0			; Start on timer 0, PrintTimer increases W on return
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		call	PrintTimer
		goto	Exec_Loop


Exec_CmdV	btfsc	flags, fl_delay		; Delayed printing of version not supported
		goto	Exec_BadDelay
		movlw	MSG_VERSION
		call	Cmd_QueueByte
		goto	Exec_Loop


Exec_CmdW	bsf	flags, fl_delay		; Flag that delay is active
		call	GetArgDelay		; Argument: time (optional) (stored as shifted bit in tmp1)
		movf	tmp1, W			; Add time argument to command delay
		andlw	0x3f
		btfsc	tmp1, 7
		goto	Exec_CmdW_High
		swapf	tmp1, W			; Time 0-3, apply in high nibble of low byte
		addwf	cmd_timel, F
		clrw
		btfsc	STATUS, C
		addlw	1
Exec_CmdW_High	addwf	cmd_timeh, F		; W is either bits for high byte (time 4-9) or carry from low byte
		btfss	STATUS, C
		goto	Exec_Loop
;		goto	Exec_LongTime

Exec_LongTime	movlw	ERR_LONGTIME
		goto	Exec_Abort


Exec_NotCmd	btfsc	INDF, 5			; Numbers (0x30-0x39) are arguments
		btfss	INDF, 4			;   but arguments should be handled by command parsing
		goto	Exec_Skip
		goto	Exec_Invalid

Exec_Skip	btfss	INDF, 5			; Anything else is ignored
		goto	Exec_EndOk
		incf	FSR, F
		goto	Exec_Loop

Exec_EndOk	bcf	timers+0*3+2, 7		; Clear "new"-flag on all timers
		bcf	timers+1*3+2, 7
		bcf	timers+2*3+2, 7
		bcf	timers+3*3+2, 7
		bcf	timers+4*3+2, 7
		bcf	timers+5*3+2, 7
		bcf	timers+6*3+2, 7
		bcf	timers+7*3+2, 7
		bcf	timers+8*3+2, 7
		bcf	timers+9*3+2, 7
		call	ApplyRelays		; Apply new relay state
		movlw	MSG_CMDOK		; Queue successful reply
		call	QueueByte
		goto	Exec_End

Exec_Overflow	movlw	ERR_INPUTOF		; Queue error message
		call	QueueByte
		goto	Exec_End

Exec_NoArg	movlw	ERR_MISSING
		goto	Exec_Abort

Exec_BadArg	movlw	ERR_BADPARAM
		goto	Exec_Abort

Exec_BadDelay	movlw	ERR_DELAY
;		goto	Exec_Abort

Exec_Abort	call	QueueByte		; Queue error message set by failed function
		movlw	timers+2		; Clear timers with new flag set
		movwf	FSR
Exec_Abort_Loop	btfss	INDF, 7
		goto	Exec_Abort_Skip
		clrf	INDF			; Timer was setup by aborted command, clear it
		decf	FSR, F
		clrf	INDF
		decf	FSR, F
		clrf	INDF
		incf	FSR, F
		incf	FSR, F
Exec_Abort_Skip	movlw	3
		addwf	FSR, F
		movf	FSR, W
		xorlw	timers+TMR_CNT*3+2
		btfss	STATUS, Z
		goto	Exec_Abort_Loop

Exec_End	movlw	inbuf			; Reset input buffer
		movwf	in_ptr
		bcf	flags, fl_inof		; Clear input buffer overflow
		return


; *** Add a relay state as digit to TX preserving FSR ***
; In: W=Relay mask
; Modifies: cmd_tmp2, tmp1, W
AddCmd_QueueR	andwf	relays, W
		movlw	'0'
		btfss	STATUS, Z
		movlw	'1'

; *** Add a byte to TX queue preserving FSR ***
; In: W=Byte
; Modifies: cmd_tmp2, tmp1, W
Cmd_QueueByte	movwf	tmp1
		movf	FSR, W			; Save FSR (will be overwritten by QueueByte)
		movwf	cmd_tmp2
		movf	tmp1, W
		call	QueueByte
		movf	cmd_tmp2, W		; Restore FSR
		movwf	FSR
		return

; *** Get and convert delay argument ***
; Out: tmp1=(01 02 04 08 81 82 84 88 90 A0)
; Modifies: cmd_tmp2, W
GetArgDelay	call	GetArg
		btfsc	STATUS, C
		movlw	0			; Default time = 0

		addlw	-4			; FC FD FE FF 00 01 02 03 04 05
		movwf	cmd_tmp2
		movlw	0x01
		btfsc	cmd_tmp2, 1
		movlw	0x04
		movwf	tmp1			; 01 01 04 04 01 01 04 04 01 01
		bcf	STATUS, C
		btfsc	cmd_tmp2, 0
		rlf	tmp1, F			; 01 02 04 08 01 02 04 08 01 02
		btfsc	cmd_tmp2, 7
		return
		btfsc	cmd_tmp2, 2
		swapf	tmp1, F			; 01 02 04 08 01 02 04 08 10 20
		bsf	tmp1, 7			; 01 02 04 08 81 82 84 88 90 A0
		return


; *** Get and validate next argument ***
; In: FSR->Next input byte
; Out: C=Error, W=(Error ? undef : value), FSR=(Error ? FSR : FSR+1), Z=ArgIsZero
GetArg		movf	INDF, W
		addlw	-0x3a
		btfsc	STATUS, C
		goto	GP_Error		; Value is beyond '9'
		addlw	10
		btfss	STATUS, C
		goto	GP_Error
		incf	FSR, F
		addlw	0			; Clear C and set Z if zero
		return

GP_Error	bsf	STATUS, C
		return


; *** Perform och schedule command ***
; In: cmd_tmp1=command
; Out: fl_tmp=Failure, W=Error message (if failure)
; Modifies: cmd_tmp2, tmp1 (if action=get), W
AddCommand	bcf	flags, fl_tmp		; Clear failure flag
		movf	cmd_timel, W		; Check if command should be delayed
		iorwf	cmd_timeh, W
		btfss	STATUS, Z
		goto	AddCmd_Delayed

AddCmd_Relay	movf	cmd_tmp1, W
		andlw	0x0f
		btfsc	STATUS, Z
		goto	AddCmd_RelayZer
		xorlw	0x09
		btfsc	STATUS, Z
		goto	AddCmd_RelayAll

		movlw	0x01			; Convert relay number 1-8 to mask with single bit set
		btfsc	cmd_tmp1, 1
		movlw	0x04
		movwf	cmd_tmp2		; (01) 01 04 04 01 01 04 04 01
		btfsc	cmd_tmp1, 2
		swapf	cmd_tmp2, F		; (01) 01 04 04 10 10 40 40 01
		bcf	STATUS, C
		btfss	cmd_tmp1, 0
		rrf	cmd_tmp2, F		; (00) 01 02 04 08 10 02 40 00
		btfsc	STATUS, C
		bsf	cmd_tmp2, 7		; (80) 01 02 04 08 10 20 40 80

		movf	cmd_tmp2, W
		btfss	cmd_tmp1, 5
		goto	AddCmd_Relay00x
		btfss	cmd_tmp1, 4
		goto	AddCmd_Relay010

		bcf	cmd_tmp1, 4		; Clear command bits to not interfer with message id
		bcf	cmd_tmp1, 5
AddCmd_GetRelay	andwf	relays, W		; Get status of relay bit
		movlw	TXT_GETRELAY0
		btfss	STATUS, Z
		movlw	TXT_GETRELAY1
		iorwf	cmd_tmp1, W		; Get relay number (command bits are zero for GetRelay)
		goto	Cmd_QueueByte		; call+return

AddCmd_Relay00x	btfsc	cmd_tmp1, 4
		goto	AddCmd_Relay001
AddCmd_Relay000	xorlw	0xff			; Clear relay bit (cmd=000)
		andwf	cmd_newst, F
		return

AddCmd_Relay001	iorwf	cmd_newst, F		; Set relay bit (cmd=001)
		return

AddCmd_Relay010	xorwf	cmd_newst, F		; Invert relay bit (cmd=010)
		return

AddCmd_RelayZer	btfsc	cmd_tmp1, 5
		btfss	cmd_tmp1, 4
		return				; Setting relay zero is NOP
		movlw	MSG_GETSHORT
		call	Cmd_QueueByte
		movlw	0x01
		call	AddCmd_QueueR
		movlw	0x02
		call	AddCmd_QueueR
		movlw	0x04
		call	AddCmd_QueueR
		movlw	0x08
		call	AddCmd_QueueR
		movlw	0x10
		call	AddCmd_QueueR
		movlw	0x20
		call	AddCmd_QueueR
		movlw	0x40
		call	AddCmd_QueueR
		movlw	0x80
		call	AddCmd_QueueR
		movlw	MSG_NEWLINE
		goto	Cmd_QueueByte		; call+return

AddCmd_RelayAll	movlw	0xff			; W = mask for all relays
		btfss	cmd_tmp1, 5
		goto	AddCmd_Relay00x
		btfss	cmd_tmp1, 4
		goto	AddCmd_Relay010
		movlw	0x01			; Get status for all relays
		movwf	cmd_tmp1
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x02
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x04
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x08
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x10
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x20
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x40
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F
		movlw	0x80
		call	AddCmd_GetRelay
		incf	cmd_tmp1, F		; Restore cmd_tmp1 (to 0x09)
		return


AddCmd_Delayed	movf	FSR, W			; Save FSR
		movwf	cmd_tmp2
		movlw	timers
		movwf	FSR
AddCmd_TmrLoop	movf	INDF, W
		incf	FSR, F			; FSR -> high byte
		iorwf	INDF, W
		btfsc	STATUS, Z
		goto	AddCmd_TmrFound
		incf	FSR, F			; FSR -> command byte
		incf	FSR, F			; FSR -> low byte of next timer
		movf	FSR, W
		xorlw	timers + TMR_CNT*3
		btfss	STATUS, Z
		goto	AddCmd_TmrLoop
		bsf	flags, fl_tmp		; Flag failure
		movlw	ERR_NOTIMER
		return

AddCmd_TmrFound	decf	FSR, F			; FSR -> low byte of first available timer
		movf	cmd_timel, W
		movwf	INDF
		incf	FSR, F			; FSR -> high byte
		movf	cmd_timeh, W
		movwf	INDF
		incf	FSR, F			; FSR -> command byte
		movf	cmd_tmp1, W
		iorlw	0x80			; Mark timer as temporary in case command line is aborted
		movwf	INDF
		movf	cmd_tmp2, W		; Restore FSR
		movwf	FSR
		return


; *** Output status for a timer ***
; In: w=timer
; Out: w=w+1
; Modifies: cmd_tmp1, cmd_tmp2, tmp1
PrintTimer	movwf	cmd_tmp1
		movf	FSR, W			; Save FSR
		movwf	cmd_tmp2

		movf	cmd_tmp1, W
		movwf	FSR
		addwf	FSR, F			; FSR = num*2
		addlw	timers
		addwf	FSR, F			; FSR -> timers + num*3

		movf	INDF, W			; Low byte
		incf	FSR, F
		iorwf	INDF, W
		btfsc	STATUS, Z
		goto	PrintTimer_Off
		movf	INDF, W			; Save high byte
		movwf	cmd_timeh
		incf	FSR, F
		movf	INDF, W			; Save command byte
		movwf	cmd_timel

		movf	cmd_tmp1, W
		addlw	TXT_GETTIMER1
		call	QueueByte
		movf	cmd_timeh, W		; Get first argument: Counter high byte
		call	QueueByte
		movf	cmd_timel, W		; Get second argument: Action
		call	QueueByte

		movf	cmd_tmp2, W		; Restore FSR
		movwf	FSR
		clrf	cmd_timeh		; Restore delay for current command line
		clrf	cmd_timel
		incf	cmd_tmp1, W
		return

PrintTimer_Off	movf	cmd_tmp1, W
		addlw	TXT_GETTIMER0
		call	QueueByte
		movf	cmd_tmp2, W		; Restore FSR
		movwf	FSR
		incf	cmd_tmp1, W
		return


; *** Reset all timers ***
; Modifies: FSR, W
ClearTimers	movlw	timers
		movwf	FSR
ClearTmr_Loop	clrf	INDF			; Clear counter high bits
		incf	FSR, F
		clrf	INDF			; Clear counter low bits
		movf	FSR, W
		addlw	2
		movwf	FSR
		xorlw	timers + TMR_CNT*3
		btfss	STATUS, Z
		goto	ClearTmr_Loop
		return


; *** Tick all timers and execute actions ***
TimerTick	bcf	PIR1, CCP1IF		; Ack interrupt

		movf	PORTA, W		; Compare button with last known stable stabe
		xorwf	flags, W
		andlw	1<<PIN_BTN
		btfsc	STATUS, Z
		goto	TT_NoButton

		movlw	0x10			; Button must be same for 8 ticks before we accept the new state
		addwf	btncnt, F
		btfss	STATUS, C
		goto	TT_ButtonEnd
		movlw	1<<PIN_BTN		; Button is stable, save new state
		xorwf	flags, F

		movf	butact, W
		andlw	0x0f
		btfsc	STATUS, Z
		goto	TT_NoButton		; No action configured, button does nothing

		movlw	butact
		btfss	flags, fl_btn
		goto	TT_ButtonDown		; New button state is down

		btfss	butact, 7
		goto	TT_NoButton		; Don't perform any action on button up

		movf	butact, W
		movwf	tmp1
		movlw	0x30
		btfsc	butact, 4
		btfss	butact, 5
		xorwf	tmp1, F			; Invert command unless it was toggle
		movlw	tmp1
TT_ButtonDown	movwf	FSR
		call	Wrap_RelayCmd

TT_NoButton	movlw	0x0f			; Reset button counter
		andwf	btncnt, F
TT_ButtonEnd

		bcf	flags, fl_tmp		; Use temp bit to check if any timer is active
		movlw	timers
		movwf	FSR

TT_Loop		movf	INDF, W
		incf	FSR, F			; FSR -> second byte (high counter bits)
		iorwf	INDF, W
		btfsc	STATUS, Z
		goto	TT_Skip			; Timer count is zero => timer isn't active

		bsf	flags, fl_tmp		; Remember that we have at least one active timer
		decf	FSR, F			; FSR -> first byte (low counter bits)
		movlw	-1			; Decrease timer count
		addwf	INDF, F
		btfsc	STATUS, Z
		goto	TT_FirstZero
		incf	FSR, F			; FSR -> second byte (high counter bits)
		btfsc	STATUS, C
		goto	TT_Skip			; Low byte didn't overflow => don't decrease high byte
		decf	INDF, F

TT_Skip		incf	FSR, F			; FSR -> third byte (action + argument)
TT_Next		incf	FSR, F			; FSR -> first byte of next timer
		movlw	timers + TMR_CNT*3
		xorwf	FSR, W
		btfss	STATUS, Z
		goto	TT_Loop
		btfss	flags, fl_tmp
		bcf	PORTA, PIN_LED		; All timers are idle, turn activity LED off
		return

TT_FirstZero	incf	FSR, F			; FSR -> second byte (high counter bits)
		movf	INDF, F
		btfss	STATUS, Z
		goto	TT_Skip			; High byte wasn't zero, not time yet
		incf	FSR, F			; FSR -> third byte (action + argument)
		call	Wrap_RelayCmd
		goto	TT_Next


; *** Apply a relay command ***
; In: INDF = command
; Modifies: W, tmp1, cmd_newst, cmd_timel, cmd_timeh, FSR
Wrap_RelayCmd	movf	relays, W		; Setup enough to reuse AddCommand from command handling
		movwf	cmd_newst
		movf	cmd_tmp1, W		; Save registers that AddCommand will overwrite
		movwf	cmd_timel
		movf	cmd_tmp2, W
		movwf	cmd_timeh
		movf	INDF, W			; Get command
		movwf	cmd_tmp1
		call	AddCmd_Relay
		movf	cmd_timel, W		; Restore registers
		movwf	cmd_tmp1
		movf	cmd_timeh, W
		movwf	cmd_tmp2
;		goto	ApplyRelays		; Apply new relay state


; *** Apply relay state ***
; In: cmd_newst = new state
ApplyRelays	movf	cmd_newst, W		; Save new relay state
		movwf	relays
		swapf	relays, W		; PORTC 0:3 - Relays 5:8
		andlw	0x0f
		btfsc	relays, 1		; PORTC 4 - Relay 2
		iorlw	0x10
		btfsc	relays, 0		; PORTC 5 - Relay 1
		iorlw	0x20
		movwf	PORTC
		btfsc	relays, 2		; PORTA 0 - Relay 3
		bsf	PORTA, 0
		btfss	relays, 2
		bcf	PORTA, 0
		btfsc	relays, 3		; PORTA 1 - Relay 4
		bsf	PORTA, 1
		btfss	relays, 3
		bcf	PORTA, 1
		return


; *** Queue byte for sending ***
; In: W = data
; Modifies: FSR, tmp1, W
QueueByte	movwf	tmp1
		movf	sq_ptr, W
		movwf	FSR
		xorlw	sendq + SQ_LEN		; Check if queue is already full
		btfsc	STATUS, Z
		goto	QB_Overflow

		movf	tmp1, W			; Queue byte
		movwf	INDF
		incf	sq_ptr, F
		return

QB_Overflow	decf	FSR, F			; FSR -> sendq[SQ_LEN-1]
		movlw	ERR_OUTPUTOF		; Replace last char with output overflow error
		movwf	INDF
		return


; *** Send next byte from send queue ***
DataOut		movf	sq_ptr, W
		xorlw	sendq
		btfsc	STATUS, Z
		return				; Send queue is empty
		movlw	sendq
		movwf	FSR

		btfsc	INDF, 7
		goto	DO_Special

		movf	INDF, W			; Put head from send queue in uart
		movwf	uartout
		bcf	uartfl, uf_trdy		; Start UART send
DO_Del		movlw	sendq+1			; Remove first character from send queue
DO_DelLoop	movwf	FSR
		xorwf	sq_ptr, W		; Check if we've reached queue end
		btfsc	STATUS, Z
		goto	DO_End
		movf	INDF, W			; Copy [ptr] to [ptr-1]
		decf	FSR, F
		movwf	INDF
		movf	FSR, W
		addlw	2
		goto	DO_DelLoop

DO_End		decf	sq_ptr, F
		return


DO_Special	movf	sendoffh, W		; Get next char from message
		andlw	0x0f
		btfsc	STATUS, Z
		call	DO_SpecialInit		; First char of message, setup ptrs
		iorwf	sendoffh, F

		bcf	INTCON, GIE		; Disable interrupts while PCLATH != 0
		movwf	PCLATH
		movf	sendoffl, W
		call	GetTab
		clrf	PCLATH			; Restore PCLATH for interrupt handler
		bsf	INTCON, GIE

		addlw	0
		btfsc	STATUS, Z		; Check for null terminator at end of string
		goto	DO_SpecEnd
		movwf	uartout

		xorlw	'#'			; Replace # with low nibble from message id
		btfss	STATUS, Z
		goto	DO_NotHash
		movf	INDF, W
		call	DO_SpecHex
		goto	DO_ArgEnd
DO_NotHash
		xorlw	'#' ^ '{'		; Replace left brace with high nibble of following byte
		btfss	STATUS, Z
		goto	DO_NotArgHi
		incf	FSR, F
		swapf	INDF, W
		call	DO_SpecHex
		goto	DO_ArgEnd
DO_NotArgHi
		xorlw	'{' ^ '}'		; Replace right brace with low nibble of following byte
		btfss	STATUS, Z
		goto	DO_NotArgLo
		incf	FSR, F
		movf	INDF, W
		call	DO_SpecHex
		movlw	sendq+2			; Remove argument byte from queue
		call	DO_DelLoop
;		goto	DO_ArgEnd
DO_NotArgLo
DO_ArgEnd
		bcf	uartfl, uf_trdy		; Start UART send
		incfsz	sendoffl, F		; Increase char pointer
		return
		incf	sendoffh, F
		return

DO_SpecHex	andlw	0x0f			; 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
		addlw	6			; 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15
		btfsc	STATUS, DC
		addlw	7			; 06 07 08 09 0A 0B 0C 0D 0E 0F 17 18 19 1A 1B 1C
		addlw	'0' - 6			; 30 31 32 33 34 35 36 37 38 39 41 42 43 44 45 46
		movwf	uartout
		return

DO_SpecEnd	movlw	0xf0			; Reset send offset pointer so next char will init
		andwf	sendoffh, F
DO_SkipToNext	call	DO_Del			; Remove control code from output buffer
		goto	DataOut			; Send next byte instead


DO_SpecialInit	btfsc	INDF, 6
		goto	DO_SpecInit_1xx
DO_SpecInit_0xx	bcf	INTCON, GIE		; Disable interrupts while PCLATH != 0
		movlw	HIGH(PtrMsg)
		movwf	PCLATH
		rlf	INDF, W
		andlw	0x1f << 1
		addlw	LOW(PtrMsg)
		call	GetTab
		movwf	sendoffl
		rlf	INDF, W
		andlw	0x1f << 1
		addlw	LOW(PtrMsg) + 1
		call	GetTab
		clrf	PCLATH			; Restore PCLATH for interrupt handler
		bsf	INTCON, GIE
		return

DO_SpecInit_1xx	btfsc	INDF, 5
		goto	DO_SpecInit_11x
DO_SpecInit_10x	movlw	LOW(MessageR0)
		btfsc	INDF, 4
		movlw	LOW(MessageR1)
		movwf	sendoffl
		retlw	HIGH(MessageR0)
DO_SpecInit_11x	movlw	LOW(MessageT0)
		btfsc	INDF, 4
		movlw	LOW(MessageT1)
		movwf	sendoffl
		retlw	HIGH(MessageT0)


TXT_GETMSG	EQU	0x80			; Add message number in low nibble
TXT_GETRELAY0	EQU	0xc0			; Add relay number in low nibble
TXT_GETRELAY1	EQU	0xd0			; Add relay number in low nibble
TXT_GETTIMER0	EQU	0xe0			; Add timer number in low nibble
TXT_GETTIMER1	EQU	0xf0			; Add timer number in low nibble and follow with args

ERR_INVALID	EQU	TXT_GETMSG + 0
ERR_INPUTOF	EQU	TXT_GETMSG + 1
ERR_OUTPUTOF	EQU	TXT_GETMSG + 2
ERR_DELAY	EQU	TXT_GETMSG + 3
ERR_LONGTIME	EQU	TXT_GETMSG + 4
ERR_MISSING	EQU	TXT_GETMSG + 5
ERR_BADPARAM	EQU	TXT_GETMSG + 6
ERR_NOTIMER	EQU	TXT_GETMSG + 7

MSG_BREAK	EQU	TXT_GETMSG + 8
MSG_RESET	EQU	TXT_GETMSG + 9
MSG_RESETWDT	EQU	TXT_GETMSG + 10
MSG_CMDOK	EQU	TXT_GETMSG + 11
MSG_NEWLINE	EQU	TXT_GETMSG + 12
MSG_BACKSPACE	EQU	TXT_GETMSG + 13
MSG_ECHOOFF	EQU	TXT_GETMSG + 14
MSG_ECHOON	EQU	TXT_GETMSG + 15
MSG_GETSHORT	EQU	TXT_GETMSG + 16
MSG_HELP	EQU	TXT_GETMSG + 17
MSG_VERSION	EQU	TXT_GETMSG + 18


; *** Get data from table ***
; In: PCLATH:W = Offset
; Out: W = Data
GetTab		movwf	PCL

Error0		DT	"Error: Invalid command line\r\n", 0
Error1		DT	"Error: Command line too long (max 48 chars)\r\n", 0
Error2		DT	"Error: Send queue overrun\r\n", 0
Error3		DT	"Error: Cannot delay command\r\n", 0
Error4		DT	"Error: Too long delay (max 8 min)\r\n", 0
Error5		DT	"Error: Missing mandatory argument\r\n", 0
Error6		DT	"Error: Invalid argument value\r\n", 0
Error7		DT	"Error: Too many scheduled actions\r\n", 0

Message0	DT	"^C\r\n", 0
Message1	DT	"\r\nReady\r\n", 0
Message2	DT	"\r\nReset by watchdog!\r\n", 0
Message3	DT	"OK"
Message4	DT	"\r\n", 0
Message5	DT	8, " ", 8, 0
Message6	DT	"Echo disabled\r\n", 0
Message7	DT	"Echo enabled\r\n", 0
Message8	DT	"Relays: ", 0
Message9	DT	"Commands:\r\n"
		DT	"\tH\t- Get help\r\n"
		DT	"\tV\t- Get version\r\n"
		DT	"\tEs\t- Terminal echo (def: 2)\r\n"
		DT	"\tGr\t- Get relay (def: 9)\r\n"
		DT	"\tSrs\t- Set relay (def: x2)\r\n"
		DT	"\tAssssssss\t- Set all relays (def: 33333333)\r\n"
		DT	"\tWt\t- Wait (def: 0)\r\n"
		DT	"\tPrts\t- Pulse relay (def: x02)\r\n"
		DT	"\tBrsb\t- Set button action (def: 020)\r\n"
		DT	"\tTc\t- Get timer status (def: all)\r\n"
		DT	"\tCc\t- Clear timer (def: all)\r\n"
		DT	"Arguments:\r\n"
		DT	"\tr\t- Relay (1-8=single relay, 9=all)\r\n"
		DT	"\ts\t- State (0=clear, 1=set, 2=toggle)\r\n"
		DT	"\tt\t- Time (0=125ms, 1=250ms, 2=500ms, 3=1s, ... 9=64s)\r\n"
		DT	"\tb\t- Button release action (0=none, 1=reverse)\r\n"
		DT	"\tc\t- Scheduled timer (0-9)\r\n"
		DT	0
Message10	DT	"Version: 2 (2023-05-16)\r\n", 0

; Code assumes special messages and pointer list below don't cross any page boundaries.
 IF $ < 0x700
	ORG	0x700
 ENDIF

MessageR0	DT	"Relay # is OFF\r\n", 0
MessageR1	DT	"Relay # is ON\r\n", 0

MessageT0	DT	"Timer #: OFF\r\n", 0
MessageT1	DT	"Timer #: Count {}, Action {}\r\n", 0

PtrMsg		DT	LOW(Error0), HIGH(Error0), LOW(Error1), HIGH(Error1)
		DT	LOW(Error2), HIGH(Error2), LOW(Error3), HIGH(Error3)
		DT	LOW(Error4), HIGH(Error4), LOW(Error5), HIGH(Error5)
		DT	LOW(Error6), HIGH(Error6), LOW(Error7), HIGH(Error7)
		DT	LOW(Message0), HIGH(Message0), LOW(Message1), HIGH(Message1)
		DT	LOW(Message2), HIGH(Message2), LOW(Message3), HIGH(Message3)
		DT	LOW(Message4), HIGH(Message4), LOW(Message5), HIGH(Message5)
		DT	LOW(Message6), HIGH(Message6), LOW(Message7), HIGH(Message7)
		DT	LOW(Message8), HIGH(Message8), LOW(Message9), HIGH(Message9)
		DT	LOW(Message10), HIGH(Message10)

		END
