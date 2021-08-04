; G8RTOS_SchedulerASM.s
; Holds all ASM functions needed for the scheduler
; Note: If you have an h file, do not have a C file and an S file of the same name

	; Functions Defined
	.def G8RTOS_Start, PendSV_Handler

	; Dependencies
	.ref CurrentlyRunningThread, G8RTOS_Scheduler

	.thumb		; Set to thumb mode
	.align 2	; Align by 2 bytes (thumb mode uses allignment by 2 or 4)
	.text		; Text section

; Need to have the address defined in file 
; (label needs to be close enough to asm code to be reached with PC relative addressing)
RunningPtr: .field CurrentlyRunningThread, 32

; G8RTOS_Start
;	Sets the first thread to be the currently running thread
;	Starts the currently running thread by setting Link Register to tcb's Program Counter
G8RTOS_Start:

	.asmfunc
	;Load =CurrentlyRunningThread into R4
	LDR R4, RunningPtr

	;Load the address of the TCB into R4
	LDR R4, [R4, #0]

	;Load the TCB's SP into R5
	LDR R5, [R4, #0]

	;save R5 into SP
	MSR MSP, R5

	;pop R4-R11
	POP {R4-R11}

	;configure the rest of thread A's context
	POP {R0}
	POP {R1}
	POP {R2}
	POP {R3}
	POP {R12}
	POP {LR}
	POP {R4} ;temporarily place PC in R4. this is fine because R4 is 0 at start
	POP {R5} ;pop PSR into R5 temporarily to store into the PSR register
	MSR IPSR, R5
	EOR R5, R5, R5 ;clear R5

	MOV LR, R4

	;Return
	BX LR
	.endasmfunc

; PendSV_Handler
; - Performs a context switch in G8RTOS
; 	- Saves remaining registers into thread stack
;	- Saves current stack pointer to tcb
;	- Calls G8RTOS_Scheduler to get new tcb
;	- Set stack pointer to new stack pointer from new tcb
;	- Pops registers from thread stack
PendSV_Handler:
	
	.asmfunc

	PUSH {R4-R11}

	;SAVE THE STACK POINTER BACK TO THE TCB BLOCK
	;load the main stack pointer into R4
	MRS R4, MSP

	;Write the stack pointer, in R4, to the stack pointer value
	LDR R5, RunningPtr

	LDR R5, [R5, #0]

	STR R4, [R5, #0]

	PUSH {LR}

	BL G8RTOS_Scheduler

	POP {LR}

	;load the pointer in currentlyrunningthread to R4
	LDR R4, RunningPtr

	LDR R4, [R4, #0]

	LDR R5, [R4, #0]

	;write R5 to the stack pointer
	msr msp, R5

	;pop the next thread's variable registers into the new context
	POP {R4-R11}

	;Return from exception
	BX LR

	.endasmfunc
	
	; end of the asm file
	.align
	.end
