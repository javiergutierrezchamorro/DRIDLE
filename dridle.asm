;;*****************************************;
;------------------------------------------;
; NAME: DRIDLE.ASM
;-------------------------------------------------------------------------;
; Copyright (C) 1990 Digital Research Inc.  All rights reserved.
;-------------------------------------------------------------------------;
;
	include DRMACROS.EQU		; standard DR macros
	include REQHDR.EQU		; request header equates
	include	DRIVER.EQU		; device driver equates
	include IDLE.EQU		; include custom equates
	
ITEST		equ	FALSE
CR		equ	0dh
LF		equ	0ah
EOM		equ	'$'
INT_8		equ	8h
INT_11		equ	11h
INT_16		equ	16h
LOWER_CASE	equ	20h		; convert upper case to lowercase
TAB		equ	8
DOS_INT		equ	21h
MS_C_WRITESTR	equ	09h
MS_S_GETINT	equ	35h
MS_S_SETINT	equ	25h

; ----------------------------------------------------------------------- ;
; This part of the driver resides in RAM even in a ROMed version of $IDLE$.
; ----------------------------------------------------------------------- ;

CGROUP	group	CODE, RCODE, ICODE
CG	equ	offset CGROUP
	Assume	CS:CGROUP, DS:CGROUP, ES:CGROUP, SS:Nothing

CODE	segment public word 'CODE'

public	int8_handler, int8_code, int16_handler, int16_code
public	idle_detect, wait_idle, wait_keyin, wait_idle28 
public	chk_time, chk_activity, chk_indos, chk_int16
public	go_to_sleep, read_timer, calc_idlecntdn, int2f_handler
public	int2f_code, dd_init, init
public	idle_intrpt, dd_open, dd_close, dd_ioctl_output
ifdef SKIP_INT8
public	set_tick, bcd2bin, chk_tsr
endif

; Device driver header.
; ---------------------
		public	idle_drvr
idle_drvr	dw	-1, -1		; link to next driver
myattrib	dw	0C800h		; attribute
		dw	CG:strat
		dw	CG:intrp
idle_id		db	'$IDLE$  '	; must be 8 characters
IDLE_ID_SZ	= $-idle_id

; Local data storage.
; -------------------
		even
req_ptr		label	dword
req_off		dw	?
req_seg		dw	?

param_blk	label	dword
param_blk_off	dw	0
param_blk_seg	dw	0
;
; NB: The following words starting with old_int8 and up to and
; including int28_seg must be in fixed order to support movsw/stosw
;
old_int8	label	dword		; original interrupt handler address
old_int8_off	dw	0
old_int8_seg	dw	0
old_int16	label	dword		; original interrupt handler address
old_int16_off	dw	0
old_int16_seg	dw	0
old_int2f	label	dword		; original interrupt handler address
old_int2f_off	dw	0
old_int2f_seg	dw	0
ifdef SKIP_INT8
int1C_seg	dw	0		; original INT1Ch segment
int28_seg	dw	0		; original INT28h segment
; end of fixed data
int08_seg	dw	0		; initial (our) INT8 segment
endif
int16_seg	dw	0		; initial (our) INT16h segment

ifdef SKIP_INT8
minutes		db	?
hours		db	?
hundredths	db	0
seconds		db	?
endif

;----------------------------------------------------------------------------
; Communication with this device driver is as follows. Firstly INIT is
; called by the CONFIG processor in BIOS.SYS. Having initialized, the
; BDOS then makes a DEVICE OPEN call. We just return good status to say that
; we are here. Next the BDOS makes an IOCTL OUTPUT call, and writes 4 bytes
; of data to us. These four bytes are a DWORD address of the IDLE STATE DATA
; AREA. We must save this address, then reset bit 7 of offset IDLE_FLAGS, in
; the Idle State Data Area followed by writing the DWORD address of our
; IDLE_DETECT routine to offset IDLE_VEC in the idle State Data Area. Finally
; the BDOS makes a device close call, whereby we return a good status.
;
; The Device Driver interface is not used after that. All subsequent
; communication with $IDLE$ is via the BDOS calling the IDLE_DETECT
; routine directly, via a far call to the address stored at offset IDLE_VEC
; in the Idle State Data Area. A function code is in AX
;

	Assume	DS:nothing, SS:nothing, ES:nothing

; Strategy routine in RAM in all versions.
; ----------------------------------------

strat	proc	far
	mov	req_off,bx		; save pointer to request packet
	mov	req_seg,es
	ret
strat	endp

; Interrupt routine with RAM jump table to RAM or ROM.
; ----------------------------------------------------

; This next bit sets up the DS and calls the ROMable code for the $IDLE$
; interrupt procedure.
;
intrp_vector	label	dword
intrp_ptr	dw	CG:init	; This is patched to CG:intrp by dd_init.
rom_cs_fixup	dw	0	; intrp pokes run-time CS here.
		dw	0EDCh	; ROM segment fixup signature
intrp	proc	far	  ; This code lets us decide during INIT which segment
	push	ds		; the far call to the interrupt routine uses
	push	cs
	pop	ds		; point DS to our data segment, here
	cmp	rom_cs_fixup,0
	jne	fixed_up	; If not fixed up yet we are running from RAM
	mov	rom_cs_fixup,cs
fixed_up:
	call	intrp_vector	; call ROMable code through vector
	pop	ds
	ret
intrp	endp

; INT16h handler with RAM jump table to RAM or ROM.
; ------------------------------------------------
; This next bit sets up ES and calls the ROMable code for the $IDLE$
; int16 handler.
int16_vector	label	dword
		dw	CG:int16_code
int16_cs_fixup	dw	?	; dd_init pokes run_time CS here
int16_handler	proc	far
	push	es
	push	cs
	pop	es		; Point ES to our data segment, here
	pushf
	call	int16_vector	; JuMP to ROMable code through vector
	pop	es
	sti
	ret	2		; discard previous flags
int16_handler	endp

; INT8h handler with RAM jump table to RAM or ROM.
; ------------------------------------------------
; This next bit sets up ES and calls the ROMable code for the $IDLE$
; int8 handler.
;
int8_vector	label	dword
		dw	CG:int8_code
int8_cs_fixup	dw	?	; dd_init_pokes run-time CS here
int8_handler	proc	far
	push	es
	push	cs
	pop	es		; Point ES to our data segment, here
	pushf
	call	int8_vector	; JuMP to ROMable code through vector
	pop	es
	iret
int8_handler	endp

; INT2Fh handler with RAM jump table to RAM or ROM.
; ------------------------------------------------
; This next bit sets up ES and calls the ROMable code for the $IDLE$
; int2f handler.
;
int2f_vector	label	dword
		dw	CG:int2f_code
int2f_cs_fixup	dw	?	; dd_init_pokes run-time CS here
int2f_handler	proc	far
	jmp	int2f_vector	; JuMP to ROMable code through vector
int2f_handler	endp

; Idle handler with RAM jump table to RAM or ROM.
; ------------------------------------------------
; This next bit sets up ES and calls the ROMable code for the $IDLE$
; idle detection routines.
;
idle_vector	label	dword
		dw	CG:idle_detect
idle_cs_fixup	dw	?	; dd_init pokes run-time CS here.
idle_handler	proc	far
	push	es
	push	cs
	pop	es		; Point ES to our data segment, here
	call	idle_vector	; JuMP to ROMable code through vector
	pop	es
	ret
idle_handler	endp
CODE	ends

RCODE	segment public 'RCODE'
; ---------------------------------------------------
; The code and data from here on can be run from ROM.
; ---------------------------------------------------

	Assume	CS:CGROUP, DS:CGROUP, ES:CGROUP, SS:Nothing

ifdef SKIP_INT8
	
tick_table	dw	002DDh, 64h
		dw	0AAB6h, 01h
		dw	0071Ch, 00h
		
count_table	dw	CG:hours
		dw	CG:minutes
		dw	CG:seconds
endif
		
idle_intrpt	proc	far

	pushf				; save flags for exit
	pushx	<ax, bx, cx, dx, si, di, ds, es>
	
	les	bx,[req_ptr]		; get request header
	
; Is this a device open?
	cmp	RH_CMD,CMD_DEVICE_OPEN
	je	dd_open			; Yes, then jump
	
; Is this a device close?
	cmp	RH_CMD,CMD_DEVICE_CLOSE
	je	dd_close		; Yes, then jump

; Is this a device write?
	cmp	RH_CMD,CMD_OUTPUT_IOCTL
	je	dd_ioctl_output		; Yes, then jump
					; No, then set error and return
	or	RH_STATUS,INVALID_CMD	; return error status
	jmps	int_err
int_ret:
	les	bx,req_ptr		; get request header again
	or	RH_STATUS,RHS_DONE	; return "done it"
int_err:
	popx	<es, ds, di, si, dx, cx, bx, ax>
	popf
strat_entry:				; if strategy entry then just return
	retf
	
idle_intrpt	endp
	Assume	DS:CGROUP		; normalize Assumes
	
	
dd_open:
;------
; This routine handles a DEVICE OPEN function. We just return with good
; status
	jmp	int_ret
	
	
dd_close:
;-------
; This routine handles a DEVICE OPEN function. We just return with good
; status
	jmp	int_ret
	
	
dd_ioctl_output:
;---------------
; This routine receives the SEG:OFF address of the BDOS idle parameter block
; There is some intricate usage of segment registers here. This is
; required if running in ROM and therefore the need exists to preserve
; segment pointers to the local data, the BDOS data area and the device
; driver request header data segment. Since these three segments must be
; referenced by only two registers DS and ES, care must be taken.
;
; Now install our own INT handlers to complete the $IDLE$ initialization.
;
	push	es
	
	push	ds
	pop	es			; ES -> our data segment

	xor	ax,ax
	mov	ds,ax			; DS -> interrupt vector table

	mov	si,08*4			; Point to current INT8 offs
	mov	di,offset old_int8
	movsw				; Copy old offset
	movsw				; Copy old segment

	mov	si,16h*4		; Point to current INT16h offs
					; DI already points to old_int16
	movsw				; Copy old offset
	movsw				; Copy old segment

	mov 	si,2Fh*4		; Point to current INT2Fh offs
					; DI already points to old_int2f
	movsw				; Save old offset
	movsw				; Save old segment


ifdef SKIP_INT8
	mov	ax,word ptr (1ch*4+2)	; Get the current INT1ch segment
	stosw				; Store to int1C_seg
	mov	ax,word ptr (28h*4+2)	; Get the current INT28h segment
	stosw				; Store to int28_seg
endif

	push	es
	pop	ds			; DS -> our data segment

	xor	ax,ax
	mov	es,ax			; ES -> interrupt vector table

	mov	di,08h*4		; Point back to INT8 offs
	mov	ax,offset int8_handler	; Install our INT8 handler
	stosw
	mov	ax,ds
	stosw
ifdef SKIP_INT8
	mov	[int08_seg],ax		; Save new int8 handler seg
endif

	mov	di,16h*4		; Point to INT16h offs
	mov	ax,offset int16_handler	; Install our INT16h handler
	stosw
	mov	ax,ds
	stosw
	mov	[int16_seg],ax		; Save new int16h handler seg

	mov	di,2Fh*4		; Point to INT2Fh offs
	mov	ax,offset int2f_handler	; Install our INT2Fh handler
	stosw
	mov	ax,ds
	stosw

	pop	es

	push	es			; save ES (-> request header?)
	push	di
	
; Check to see that we are to receive 4 bytes.

	mov	ax,RH4_COUNT		; check the count.
	cmp	ax,DD_OUT_COUNT		; is it the size we require?
	jz	dd_output_10		; yes, skip
	xor	ax,ax			; set count to zero
	mov	RH4_COUNT,0		; set the count
	or	RH_STATUS,OUTPUT_ERR	; return error status
	jmp	dd_output_err		; quit
; length looks good, read the data.
;
dd_output_10:
	cld				; set direction to increment
	push	si
	push	di
	
	push	ds			; save our local DS
	
	push	es			; save someone's ES
	les	si,RH4_BUFFER		; get address of data buffer

; ES:SI=BDOS data packet.
; DS=local data

; Store the address of the parameter block in our data area.
;
	mov	di,es:[si]
	mov	param_blk_off,di
	;add	si,2
	inc		si
	inc		si
	
	mov	ax,es:[si]
	mov	param_blk_seg,ax
	push	ds			; get local DS into ES
	pop	es			; es->local data segment
	mov	ds,ax			; ds:di -> idle param blk
	
; ES=$IDLE$ local data
; DS=BDOS Idle State Data Area segment
; Set the address of our idle detection routine at offset IDLE_FLAGS in the
; Idle State Data Area.
	mov	IDLE_DRIVER_OFF,offset idle_handler
	mov	ax,es
	mov	IDLE_DRIVER_SEG,ax
; Now reset the $IDLE$ driver enabled bit in the idle parameter block, to
; tell the BDOS that we are ready and willing.

ifdef __WASM__
	and	IDLE_FLAGS,7FFFh
else
	and	IDLE_FLAGS,NOT IDLE_INIT
endif
	
; Here is where any values can be poked into the IDLE STATE DATA AREA
; to customize the driver to suit a particular architecture.

	mov	IDLE_MAX,IDLE_MAX_DEF		; poke idle_max
	mov	INT28_RELOAD,INT28_RELOAD_DEF	; poke int28_reload
	
; Now calculate the initial value in IDLE_CNTDN. The idea being to find out
; how long it takes to do an INT16 status check. This operation will give a
; factor to apply which will take acount of the performance of the computer.

	call	calc_idlecntdn
	
	pop	es			; ES->?
	pop	ds			; DS->local data
	
	pop	di
	pop	si
	
	pop	di
	pop	es		; es:di->req hdr
	jmp	int_ret
dd_output_err:
	pop	di
	pop	es		; es:di->req hdr
	jmp	int_err

; =================================================================
; ================ Dynamic IDLE Detection routines ================
; =================================================================
; entry
;	DS = IDLE STATE DATA AREA data segment
;	ES = $IDLE$ local data segment
;	AX = function number
;	1 PROC_IDLE.  BDOS detects an idle condition.
;	2 PROC_INT28. BDOS detects application in int28h idle loop.
;	3 PROC_KEYIN. BDOS detects idle due to wait on keybard event.
; exit
;	none
idle_detect:
	push	di			; Save DI register and initialize
	mov	di,es:param_blk_off	; Parameter block pointer
	cmp	ax,PROC_IDLE		;   CASE PROC_IDLE
	je	idle_20			;     PERFORM PROC_IDLE
					;   ENDCASE
	cmp	ax,PROC_INT28		;   CASE PROC_INT28
	je	idle_30			;     PERFORM PROC_INT28
					;   ENDCASE
	cmp	ax,PROC_KEYIN		;   CASE PROC_KEYIN
	je	idle_40			;     PERFORM PROC_KEYIN
					;   ENDCASE
	cmp	ax,PROC_DEVIN		;   CASE PROC_DEVIN
	je	idle_50			;     PERFORM PROC_DEVIN
					;   ENDCASE
	mov	ax,INVALID_FUNCTION	;     invalid function
					;   ENDOTHERWISE
					; ENDSWITCH
end_idle:
	pop	di
	retf

idle_20:
;------
; This routine sends the CPU to sleep by calling wait_idle. It is
; woken up again on an event such as H/W event/timer tick.
;
	call	wait_idle		; Execute Idle Wait
	jmp	end_idle
	
idle_30:
;------
; This routine sends the CPU to sleep by calling wait_idle. It is
; woken up again on an event such as H/W event/timer tick.
;
	call	wait_idle28		; Execute Idle Wait
	jmp	end_idle

idle_40:
;------
; This routine handles the PROC_KEYIN function. It should send the CPU
; to sleep until a keyboard event occurs.
;
	call	wait_keyin		; Keyboard Wait Checks
	jmp	end_idle		; Return to DR DOS
	

idle_50:
;------
; This routine handles the PROC_DEVIN function.
	xor	ax,ax
	jmp	end_idle

;-------------------------------------------------------------------------- ;
; Module INT_16
; =============
;
; entry
;	ES=$IDLE$ local data segment
;
; exit
;	none.
; function
;	This routine is activated during a ROS keyboard service. If we are
; inside DOS, i.e called by the kernel, then we perform no idle checks and go
; directly to the old handler and perform the read/status call etc. If we are
; not in DOS, then we check to see if it a read or a status call. If it is a
; read, then we convert it to a status call and see if a char is there. If it
; is, we jmp to the old driver to read it. If there is not a char there, then
; we go to sleep by calling WAIT_KEYIN. If it is a status check, then we
; decrement idle count and see if it goes to zero. If it does, then we can
; assume that this status call being made, is because the application is
; twidling its thumbs idling. We therefore reset the idle count and perform
; a WAIT_IDLE call to go to sleep.
;
int16_code:
	push	di
	push	ds
	push	ax
	
	lds	di,es:param_blk		; Get the IDLE State Data Area
	call	chk_indos		; Are we inside the kernel?
	jnz	int16_h10		; Yes, jmp to old int 16 handler
					;  for Kernel calls
	test	ah, ah			; keyboard read?
	je	int16_h20		;  yes, skip
	cmp	ah, 10h			; No, extended keyboard read?
	je	int16_h20		;  yes skip, No ...
; check for IDLE ON
	test	IDLE_FLAGS,IDLE_ON	; Is idle detection turned on?
	jnz	int16_h10		; No, then skip
	dec	IDLE_COUNT		; Yes, then decrement IDLE count
	jnz	int16_h10		; Skip if Non-Zero
	mov	ax,IDLE_MAX		; Reset Counter
	mov	IDLE_COUNT,ax		;
	call	wait_idle		; Execute IDLE Delay
	
int16_h10:
	pop	ax			; and other User registers
	pop	ds			; Restore Function
	pop	di
	jmp	es:[old_int16]		; pass straight through to BIOS
	
int16_h20:
	pop	ax

int16_h30:
	push	ax			; Save Original Function
	inc	ah			; read -> stat; ext read -> ext
	pushf				; status
	call	es:[old_int16]		; ZF=0 - key code ready
	jnz	int16_h10		; code available
	call	wait_keyin
	pop	ax
	jmp short int16_h30
	
;-------------------------------------------------------------------------- ;
; Module INT_8
; =============
;
; entry
;	ES=$IDLE$ local data segment.
; exit
;	none.
; function
;	This routine is activated every int8 timer tick. Its function is to
; reset IDLE_COUNT and INT28_DELAY to the maximum count down values.
; This will ensure that only applications that go idle quickly, within
; a single clock tick, go idle. Thus applications that are busy, but polling
; the keyboard, will not go idle if it takes a long time to count down from
; IDLE_MAX or INT28_RELOAD to zero.

int8_code:
	push	ds
	push	di
	push	ax
	
	lds	di,es:param_blk		; Get the IDLE State Data Area
	mov	ax,IDLE_MAX		; Reset Counter
	mov	IDLE_COUNT,ax		;
	mov	ax,INT28_RELOAD		; Reset Counter
	mov	INT28_DELAY,ax		;
	
	pop	ax
	pop	di
	pop	ds
	jmp	es:[old_int8]

;-------------------------------------------------------------------------- ;
; Module INT_2F
; ===============
;
; entry
;	none.
; exit
;	none.
; function
;	This routine is the INT 2Fh multiplex interrupt handler. It serves
; to process INT 2Fh/1680h idle calls. If user application invokes INT 2Fh
; with AX=1680h, it means it's idle and the system can go to idle state
; immediately. If idling is enabled, INT 2Fh/1680h is handled and not passed
; on. When idling is disabled, we just chain to the previous handler.
; NB: DR-DOS always installs its own INT 2Fh handler before this driver is
; loaded, hence chaining is safe (the 2Fh vector won't be invalid).

int2f_code:
	cmp	ax,1680h		; DPMI Idle function?
	jne	int2f_h20		; If not, chain to old handler
	
	xor	al,al			; Indicate call is supported

	push	es			; Save all registers we'll modify
	push	ds
	push	di
	push	ax

	push	cs			; Local data segment to ES
	pop	es

	lds	di,es:param_blk		; Get the IDLE State Data Area
	test	IDLE_FLAGS,IDLE_ON	; Is idling enabled?
	jz	int2f_h10

	;mov	al,ALL_INTS		; Wait for the next hardware int
	xor		al,al			; Wait for the next hardware int (AL=ALL_INTS)
	call	go_to_sleep

int2f_h10:
	pop	ax
	pop	di
	pop	ds
	pop	es

	iret				; Restore old flags

int2f_h20:
	jmp	cs:[old_int2f]

;-------------------------------------------------------------------------- ;
; Module WAIT_KEYIN
; =================
;
; entry
;	none.
; exit
;	none.
; function
;	This routine attempts to make the CPU sleep until a keyboard event
; occurs. However there are some conditions when this should not happen,
; namely video activity and other TSRs. If the screen had been updated since
; we last checked then we should not go to sleep. If an application or TSR
; has control of the timer interrupts then we should idle but wake up on the
; next clock tick. This will give the TSR a chance to do its stuff, since if
; we blindly sleep until a key is pressed, it may affect other TSRs who rely
; on coming awake on an INT8h or INT1Ch. If none of this applies then it is
; assumed safe to sleep until the next keyboard event, since nothing is going
; to happen until a key is pressed.
;
;
wait_keyin:
	call	chk_int16		; Check if int 16h changed
	call	chk_activity		; Activity detected?
	jnz	wait_k20		; Yes, keep going
ifdef SKIP_INT8
	call	chk_tsr			; Check if interrupt vectors have
	mov	al,KBD_INT_MASK		; been taken over; if not then wait
	jz	wait_k10		; until the next keyboard interrupt
endif
	;mov	al,ALL_INTS
	xor		al,al		;  (AL=ALL_INTS)
wait_k10:
	jmp	go_to_sleep

wait_k20:
	ret

;-------------------------------------------------------------------------- ;
; Module WAIT_IDLE
; ================
;
; entry
;	none.
; exit
;	none.
; function
;	This routine attempts to make the CPU sleep until a H/W event such as
; a timer tick or key press occurs. However there are some conditions when
; this should not happen, namely if an application is busy and occasionally
; checks for a key press. We can only detect few kinds of activity, but
; in that case we should not go_to_sleep. If none of this applies then
; it is assumed safe to sleep until the next H/W event.
;
wait_idle:
	call	chk_activity		; Activity detected?
	jnz	wait_i10		; Yes, keep going
	;mov	al,ALL_INTS		; Wait for the next hardware int
	xor		al,al		; Wait for the next hardware int  (AL=ALL_INTS)

; before we go to sleep, we must check the time taken to detect an idle. If
; this is a long time, we must assume that the application is not really idle
; but is polling the keyboard periodically.

	call	chk_time		; Want to sleep 'ay
	jc	wait_i10		; too slow, skip the delay function.
	jmp	go_to_sleep

wait_i10:
	ret
	
;-------------------------------------------------------------------------- ;
; Module WAIT_IDLE28
; ==================
;
; entry
;	none.
; exit
;	none.
; function
;	This routine attempts to make the CPU sleep until a H/W event such as
; a timer tick or key press occurs. However there are some conditions when
; this should not happen, namely if an application is busy and occasionally
; makes int 28h calls. We can only detect few kinds of activity, but
; in that case we should not go_to_sleep. If none of this applies then
; it is assumed safe to sleep until the next H/W event.
;
wait_idle28:
	call	chk_activity		; Activity detected?
	jnz	wait28_i10		; Yes, keep going
	;mov	al,ALL_INTS		; Wait for the next hardware int
	xor		al,al			; Wait for the next hardware int  (AL=ALL_INTS)
	jmp	go_to_sleep
	
wait28_i10:
	ret

;-------------------------------------------------------------------------- ;
; Module CHK_TIME
; ===============
;
; entry
;	DS=idle state data segment
;	ES=$IDLE$ local data segment
; exit
;	carry flag = Set if the application is not idle.
;		   = Reset if aplication is idle.
; function
;	This routine checks to see how long it has taken to detect that we
; are idle. If it is a short time, then we can assume that the application
; is probably idle, because it is sitting there in a tight loop polling
; the keyboard. If it has taken a long time to detect an idle, we assume that
; the application has been busy in between polling the keyboard, so flag
; that we consider the application to be busy.
;
chk_time:
	push	ax
	push	cx
	push	dx
	push	di
	push	ds
	push	es
	call	read_timer		; how long did it take us to get here
; check the time taken to idle.
	lds	di,es:param_blk		; Get the IDLE State Data Area
	cmp	IDLE_CNTDN,ax		; Skip, if too long
	pop	es
	pop	ds
	pop	di
	pop	dx
	pop	cx
	pop	ax
	ret				; CF=0 if idle; 1 if busy

;-------------------------------------------------------------------------- ;
; Module CHK_ACTIVITY
; ===================
;
; entry
;	ES=$IDLE$ data segment
; exit
;	Zero Flag = Set if no video or serial activity.
;		  = Reset if there has been activity.
; function
;	This routine checks to see if there has been any video or serial
; port activity and if the floppy motor drive time-out has reached zero yet.
; This particular implimentation checks video and serial port status by
; reading from ACTIVITY_PORT which returns bit 6 set if the screen RAM has
; been updated since we last read the port, and bits 0 and 1 set for activity
; on COM1 and COM2 respectively. A value of zero read from the port indicates
; no activity since we last read the port. The floppy motor time-out status
; is checked by reading the byte at 40:3f. If it is 0 then the motor will
; have timed out and stopped. If it is non-zero, then we must return this
; status to the caller so that we do not go to sleep until the motor has
; stopped.
; NB: The ACTIVITY_PORT is only available if a 386 driver monitoring the
; activity using V86 mode is present. That is normally not the case.
;
chk_activity:
if ITEST
	push	dx
	in	al,ACTIVITY_PORT	; read the status
	pop	dx
chk_v10:
	test	al,al			; any activity yet?
	jnz	end_chk_activity	; Yes, then leave now
endif

; Before we go, we must ensure that the floppy disk is not still rotating
; on the time-out count at 40:3f in the BIOS data area. If this byte is
; zero then we can flag all is okay (ZF=1). If not, return ZF=0

chk_floppy:
	push	es
	push	bx
	mov	bx,40h
	mov	es,bx
	mov	bx,3fh
	cmp	byte ptr es:[bx],0	; are we still counting?
	pop	bx			; ZF=Set if not counting
	pop	es
	jz	no_activity		; If nothing happened, Skip
end_chk_activity:

; Since there has been activity, we must reset the idle count to IDLE_MAX
; otherwise we may go to sleep during some activity.

	pushf
	push	ds
	push	di
	lds	di,es:param_blk		; Get the IDLE State Data Area	
	mov	ax,IDLE_MAX		; Reset Counter
	mov	IDLE_COUNT,ax		;
	mov	ax,INT28_RELOAD		; Reset Counter
	mov	INT28_DELAY,ax		;
	pop	di
	pop	ds
	popf
no_activity:
	ret				; return with status in ZF.

ifdef SKIP_INT8

;-------------------------------------------------------------------------- ;
; Module CHK_TSR
; ==============
;
; entry
;	ES=$IDLE$ local data segment
; exit
;	Zero Flag = Set if no TSRs active.
;		  = Reset if TSR detected.
; function
;	This function checks to see if any TSRs have taken over the timer or
; idle interrupts 8h, 1Ch, or 28h. If a TSR has control of these interrupts,
; then it is likely that we will want to come alive and perform some
; processing on such an event. It is therefore essential that we detect them
; and wake up from an idle on such an event, even though the application in
; foreground may appear to be completely idle and not require waking up
; itself.
;
chk_tsr:
	push	ds
	xor	ax,ax
	mov	ds,ax
	mov	ax,ds:word ptr (08*4+2)	; Get the current INT08
	cmp	ax,es:int08_seg		; ISR Segment address if this has
	jnz	chk_tsr10		; changed since INIT then return NZ
	mov	ax,ds:word ptr (1Ch*4+2); Get the current INT1C
	cmp	ax,es:int1C_seg		; ISR Segment address if this has
	jnz	chk_tsr10		; changed since INIT then return NZ
	mov	ax,ds:word ptr (28h*4+2); Get the current INT28
	cmp	ax,es:int28_seg		; ISR Segment address if this has
	jnz	chk_tsr10		; changed since INIT then return NZ
	xor	ax,ax
chk_tsr10:
	pop	ds
	ret

endif	; SKIP_INT8

;-------------------------------------------------------------------------- ;
; Module CHK_INDOS
; ================
;
; entry
;	DS=idle state data area segment.
; exit
;    Zero Flag = zero if we are not inside DOS
;	       = non-zero if inside DOS, i.e. called by Kernel function.
;	This routine checks to see if the caller is the DOS kernel. It
; tests the value of IDLE_INDOS. If it is zero, then the caller was not
; inside DOS when the call was made. If it is > 1 then it must be inside
; DOS, and since it is >1 and not =1, it must be the Kernel. If IDLE_INDOS
; is 1, then we further check to see if we are in DOS because of
; call from within DOS.
;
chk_indos:
	push	ax
	xor	ax,ax
	push	di			; Save the Structure address
	mov	di,IDLE_INDOS		; Get the INDOS flag
	cmp	byte ptr[di],1		; Is the kernel making this call?
	pop	di			; Restore address
	jb	chk_i20			; INDOS is zero therefore outside DOS
	ja	chk_i10			; INDOS > 1 therefore INSIDE DOS
	test	IDLE_FLAGS,IDLE_INT28	; Check if INT28 call being made
	jnz	chk_i20			; TSR taken over INT28
chk_i10:
	mov	al,1
chk_i20:
	test	ax,ax
	pop	ax
	ret

;-------------------------------------------------------------------------- ;
; Module CHK_INT16
; ================
;
; entry
;	DS=idle state data area segment
;	ES=$IDLE$ local data segment
; exit
;	none
; function
;     This routine checks to see if the INT16h handler has been superseded.
; If it has, the time taken to detect an idle state may increase beyond the
; value in IDLE_CNTDN, which would result in never going idle. Hence the
; segment is the value of the INT16 handler. If it changes, the new value is
; saved and the value for IDLE_CNTDN is re-calculated so that the time taken
; to idle is realistic, given that it may now take longer to detect an idle.
;
chk_int16:
	push	es
	push	ds
	push	di
	xor	ax,ax
	mov	ds,ax
	mov	ax,ds:word ptr (16h*4+2); Get the current INT16h
	cmp	ax,es:int16_seg		; ISR Segment address if this has not
	jz	chk_int16_end		; changed since INIT then return
; So, int16h has been changed, we must now save this segment address, and
; then re-calculate the value of IDLE_CNTDN.
	mov	es:int16_seg,ax		; save new value
	lds	di,es:param_blk
	call	calc_idlecntdn		; re-calc IDLE_CNTDN
chk_int16_end:
	pop	di
	pop	ds
	pop	es
	ret

;-------------------------------------------------------------------------- ;
; Module GO_TO_SLEEP
; ==================
;
; entry
;	AL=bit map of events to wake-up on.
; exit
;	none.
; function
;	This routine is activated when we want to put the CPU to sleep.
; This could mean switching it off completely, or slowing down the
; clock rate, or anything else the OEM wishes to implement.
; If the timer bit is not set then we will not come alive on a timer tick.
; The HLT instruction is ideal for this since it only passes onto the
; next instruction following an interrupt.
;
go_to_sleep:
	push	ax
	push	dx
	pushf				; Save INTerrupt State

ifdef SKIP_INT8	
	mov	dx,0021h		; 8259 Control Port
	mov	ah,al			; Copy INTerrupt Vector mask ah
	in	al,dx			; Get the current value and save
	push	ax			; on the stack
	or	al,ah			; Set the requested interrupt mask
	out	dx,al
	sti				; Enable INTerrupts
	hlt				; Wait for the INTerrupt
	pop	ax			; Restore the original INTerrupt mask
	out	dx,al			; and output to 8259

	mov	al,ah			; Recover INTerrupt Vector mask
	not	al			; Complement AL
	test	al,NOT TIMER_INT_MASK	; do we sleep through int8's?
	jnz	go_to_s10		; Did we sleep thru' int8s?
	call	set_tick		; Yes! Must update the ROS tick count
go_to_s10:

else
ifdef VBOX_CPU_HALT
	mov	dx,040Fh		; Use special I/O port to
	mov	ax,8003h		; halt the CPU.
	out	dx,ax			; NB: Does not require to run
					; with interrupts enabled!
else
	sti				; Enable INTerrupts
	hlt				; go to sleep until next interrupt
endif
endif
	popf				; Restore Interrupt State
	pop	dx			; and registers
	pop	ax
	ret

;-------------------------------------------------------------------------- ;
; Module READ_TIMER
; =================
;
; entry
;	none.
; exit
;	AX=timer tick count.
; function
;	To read the tick count from the timer chip.
;
read_timer:
	xor	al,al
	out	TIMER_CMND_REG,al	; latch timer 0
	in	al,TIMER_0_REG		; lsb
	mov	ah,al
	in	al,TIMER_0_REG		; msb
	xchg	al,ah			; swap
	not	ax			; convert to ascending count
	ret

;-------------------------------------------------------------------------- ;
; Module CALC_IDLECNTDN
; =====================
;
; entry
;	ds:di = ptr to idle state data area.
; exit
;	IDLE_CNTDN variable updated.
; function
;	To calculate the time taken to perform IDLE_MAX RTC reads and
; int16 status checks. This value is then used to determine if an IDLE_PROC
; call should go ahead based on the time taken to come to the decision to
; go idle.
;
calc_idlecntdn:
	push	bx
	push	cs
	push	dx
	
calc_10:

; To calculate how long it would take to go idle, a read of the RTC
; and a keyboard status check are made and timed for long it takes. This
; time is then multiplied by IDLE_MAX, to give a time that is typical of how
; long it takes to do idle when the application is idle.

	sti
	hlt
	call	read_timer		; get time taken to do int16h
	push	ax			; save time
	
	mov	ah,READ_RTC		; set function code to read RTC
	int	RTC_INT			; get time using ROS.
	mov	ah,1			; perform int16 kbd status check
	int	16h
	
	pop	bx			; start time in BX time now in AX

; The time taken should be time_now-start_time. However, the timer may
; have wrapped, since an INT8 could have occured in between. If this is the
; case we try again until we get a good reading.

	sub	ax,bx			; is time_now > start_time
	jc	calc_10
	
; AX now contains time taken to perform an int 16H status check and a ROS
; call to read the RTC. IDLE_CNTDN value is calcuated thus:
;          IDLE_CNTDN=(read_RTC_time+int16_time)*(IDLE_MAX*2).

	mov	cx,IDLE_MAX
	shl	cx,1			; multiply by 2
	mul	cx
	add	ax,100h			; allow for round down
	and	ax,0ff00h		; round down to a clean figure
	mov	IDLE_CNTDN,ax		; Time it should take to idle
	
	pop	dx
	pop	cx
	pop	bx
	ret

ifdef SKIP_INT8

;-------------------------------------------------------------------------- ;
; Module SET_TICK
; ===============
;
; entry
;	ES=$IDLE$ local data segment.
; exit
;	ROS data area time updated.
; function
;	This module updates the timer tick count in the ROS data area.
; Since we have been asleep missing int8s, we must read the real-time clock
; and convert it to int8 timer ticks, and store it in the ROS data area.
;
set_tick:

; Since the processing has been asleep missing INT8s, the real-time clock
; must be read and the ROS data area timer tick count must be set.

	push	ax
	push	bx
	push	cx
	push	dx
	push	si
	push	di
	push	ds
	
	push	cs			; ds points to resident data seg
	pop	ds
	
	mov	ah,READ_RTC		; set function code to read RTC
	int	RTC_INT			; get time using ROS.
; CH=hours in BCD. CL=minutes in BCD. DH=seconds in BCD. Convert from BCD to
; binary, then update the ROS data area.

	cli				; stop the time updating
	
	mov	al,ch			; BCD hours in al
	call	bcd2bin			; convert to binary
	mov	es:hours,al
	
	mov	al,cl			; BCD minutes in al
	call	bcd2bin			; convert to binary
	mov	es:minutes,al
	
	mov	al,dh			; BCD seconds in al
	call	bcd2bin			; convert to binary
	mov	es:seconds,al
	
; set hundredths to zero in DX:CX

	sub	dx,dx
	sub	cx,cx			; CX:DX=32 bit tick count for 1/100s
	
	mov	si,CG:tick_table	; counts of ticks/unit
	mov	di,CG:count_table	; address of unit counts
output6:
	mov	bx,cs:[di]		; get unit count address
	mov	al,cs:[bx]		; all of that unit done?
output7:
	test	al,al			; any more hours/minutes/seconds?
	jz	output8			; yes,
	add	dx,cs:0[si]		; add ticks per hour/minute/second
	adc	cx,cs:2[si]
	dec	ax
	jmps	output7			; try again
output8:
	add	si,4			; next unit tick count
	incx	<di, di>		; next unit count address
	cmp	di,CG:count_table+6	; end of table reached
	jne	output6			; repeat until all units done
	
	push	dx			; save lo count
	mov	ax, cx			; do hi count first
	xor	dx, dx			; clear top
	mov	bx, 100
	div	bx			; top word div 100
	mov	cx, ax			; keep it safe
	pop	ax			; recover lo word
	div	bx			; divide it (plus rem of hi)
	mov	dx, ax			; and put result in right place
	
	mov	ah,1			; set system timer
	int	RTC_INT			; CX = high word, DX = low word
	
	sti
	pop	ds
	pop	di
	pop	si
	pop	dx
	pop	cx
	pop	bx
	pop	ax
	sti
	ret

;-------------------------------------------------------------------------- ;
; Module bcd2bin
; ==============
;
; entry
;	AL=BCD value.
; exit
;	AL=binary equivalent of BCD.
; function
;	This routine is called by set_tick, and converts the BCD value in AL
; into a binary equivalent and placing the result in AL.
;
bcd2bin:

	push	bx
	xor	bl,bl			; start off without tens
bcd2bin2:
	cmp	al,10h			; check if more tens
	jb	bcd2bin3		; all tens done
	sub	al,10h			; else subtract 10 in BCD
	add	bl,10			; ...and add it in binary
	jmps	bcd2bin2		; repeat for all tens
bcd2bin3:				; AL = ones, BL = tens
	add	al,bl			; AL = binary value
	pop	bx			; restore BX
	ret

endif	; SKIP_INT8

RCODE	ends				; end of ROM device driver code

; -------------------------------------------------------------------------
; The code from here on is only used during initailization and is discarded
; after driver INIT.
; -------------------------------------------------------------------------
ICODE	segment	public word 'ICODE'	; initialization code

	Assume	CS:CGROUP, DS:CGROUP, ES:CGROUP, SS:Nothing
	
	even				; start this on a word boundary
reusable:
sign_on		db	'DRIDLE R1.10 installed.',CR,LF
		db	EOM

;-------------------------------------------------------------------------- ;
; Module INIT
; ===========
;
; entry
;	DS= $IDLE$ data segment
; exit
;	$IDLE$ initialized
; function
;	This routine initializes the $IDLE$ driver.
;
init	proc	far
	pushx	<ax, bx, cx, dx, si, di, ds, es>
	les	bx,req_ptr		; get request header
	call	dd_init			; call device initialization
	mov	RH_STATUS,ax		; return "general failure"
	popx	<es, ds, di, si, dx, cx, bx, ax>
	ret
init	endp

;-------------------------------------------------------------------------- ;
; Module DD_INIT
; ==============
;
; entry
;	DS= $IDLE$ data segment
;	ES:BX -> device header
; exit
;	AX=0 for okay, else error code.
; purpose
;	To initialize the device driver and process any cmd line parameters.
;
dd_init:
	push	es
	push	bx
	mov	ax,CG:idle_intrpt
	mov	intrp_ptr,ax		; set interrupt entry
	
; This bit fixes up the CS part of the intXX_vector pointers
	mov	ax,rom_cs_fixup
	mov	int2f_cs_fixup,ax
	mov	int16_cs_fixup,ax
	mov	int8_cs_fixup,ax
	mov	idle_cs_fixup,ax
	
parse_ok:
; Tell the world that we are here.
	mov	dx,CG:sign_on
	mov	ah,MS_C_WRITESTR	; print message
	int	DOS_INT			; call DR DOS
	
; Tell DOS how much of us is here.
	mov	ax,CG:reusable		; get pointer to last resident byte 
	les	bx,req_ptr		; ES:BX -> request header
	mov	RH0_RESIDENT,ax		; set end of device driver
	mov	RH0_RESIDENT+2,cs
	
	sub	ax,ax			; initialization succeeded
	pop	bx
	pop	es
	ret
	
inst_fail:
	mov	ax,RHS_ERROR+12		; return "general failure"
	pop	bx
	pop	es
	ret
ICODE	ends
	end

