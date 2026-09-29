; Savestate restore window: 4 KB the mainboard maps at FFF000-FFFFFF while
; a state is being restored. The CPU comes out of its own reset at the
; vector in the last paragraph, hops to the stub and ends in LOADALL with
; ES:EDI on the table the engine wrote at the start of the window.
; Offsets follow CS base FFFF0000: window byte n is CS:F000+n.
;
;   000  LOADALL table (0CCh bytes)      0D0  CR3  0D4  CR2  0D8  DR0-DR3
;   0F0  GDT: null, 08 = data at FFF000  100  ten more dwords LOADALL reads
;   130  GDTR image                      140  stub
;   FF0  reset vector: jmp stub
;
; LOADALL takes ES:DI in a 16-bit segment, so ES is based on the window
; and DI is 0; the microcode also reads 100h-127h before the table.
	bits 16
	org 0xF000
table:	times 0xD0 db 0
cr3_v:	dd 0
cr2_v:	dd 0
dr_v:	dd 0, 0, 0, 0
	times 0xF0 - ($ - $$) db 0
gdt:	dq 0
	dq 0x000092FFF000FFFF
	times 0x130 - ($ - $$) db 0
gdtr:	dw 0x000F
	dd 0x00FFF0F0
	times 0x140 - ($ - $$) db 0
stub:
	cli
	lgdt [cs:gdtr]
	mov eax, cr0
	or al, 1
	mov cr0, eax                 ; PE; CS keeps its reset cache
	mov ax, 0x08
	mov es, ax                   ; ES:DI = the table
	xor edi, edi
	mov eax, [cs:cr3_v]
	mov cr3, eax
	mov eax, [cs:cr2_v]
	mov cr2, eax
	mov eax, [cs:dr_v]
	mov dr0, eax
	mov eax, [cs:dr_v + 4]
	mov dr1, eax
	mov eax, [cs:dr_v + 8]
	mov dr2, eax
	mov eax, [cs:dr_v + 12]
	mov dr3, eax
	db 0x0F, 0x07                ; LOADALL: the 22nd instruction from the vector
	hlt
	times 0xFF0 - ($ - $$) db 0
	jmp near stub
	times 0x1000 - ($ - $$) db 0
