.section .text.init
.global eirEntry
eirEntry:
	b .L1
	nop
	.quad 0 // Image offset
	.quad 0 // Image size (0 = unbounded)
	.quad 2 // 4K pages
	.quad 0 // Reserved
	.quad 0 // Reserved
	.quad 0 // Reserved
	.quad 0x644d5241 // Magic
	.quad 0 // Reserved

.L1:
	.extern eirStackTop
	adrp x1, eirStackTop
	add x1, x1, :lo12:eirStackTop
	mov sp, x1

	mov x2, xzr
	orr x2, x2, #(1 << 29)
	orr x2, x2, #(1 << 28)
	orr x2, x2, #(1 << 23)
	orr x2, x2, #(1 << 22)
	orr x2, x2, #(1 << 20)
	orr x2, x2, #(1 << 11)
	orr x2, x2, #(1 << 12)
	orr x2, x2, #(1 << 2)
	msr sctlr_el1, x2

	// Load vector table
	.extern eirExcVectors
	adrp x1, eirExcVectors
	add x1, x1, :lo12:eirExcVectors
	msr vbar_el1, x1

	// Zero out BSS
	.extern eirBssStart
	.extern eirBssEnd
	adrp x5, eirBssStart
	adrp x6, eirBssEnd
	add x5, x5, :lo12:eirBssStart
	add x6, x6, :lo12:eirBssEnd
.loop:
	cmp x5, x6
	b.eq .enter
	str xzr, [x5]
	add x5, x5, #8
	b .loop

.enter:
	.extern eirMsmMain
	// x0 -> pointer to device tree binary
	bl eirMsmMain

.halt:
	wfe
	b .halt

	.section .note.GNU-stack,"",%progbits
