; Stage 2 loader for BenOS (16-bit real mode)
; Loaded by boot sector to 0x0000:0x7E00 (physical 0x7E00)

BITS 16
ORG 0x7E00

start_stage2:
    ; Print status
    mov si, msg2
.print_loop:
    lodsb
    test al, al
    jz .after_print
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x0A            ; Light green on black
    int 0x10
    jmp .print_loop

.after_print:
    ; Read stage3 (1 sector) from LBA 2 (CHS C=0,H=0,S=3) to 0x10000
    mov ax, 0x1000
    mov es, ax
    xor bx, bx              ; ES:BX = 0x1000:0x0000
    mov ah, 0x02            ; BIOS read sectors
    mov al, 0x01            ; 1 sector
    mov ch, 0x00            ; cylinder 0
    mov cl, 0x03            ; sector 3 (LBA2)
    mov dh, 0x00            ; head 0
    mov dl, 0x00            ; drive 0 (A:)
    int 0x13
    jc disk_error

    ; Setup GDT for flat 32-bit segments and switch to protected mode
    cli
    lgdt [gdtr]

    mov eax, cr0
    or eax, 1
    mov cr0, eax

    ; Far jump to 32-bit code selector 0x08 at offset 0x0010000
    jmp 0x08:pm_entry

; ------------------------
; 32-bit entry point
; ------------------------
BITS 32
pm_entry:
    ; Set data segments to 0x10 (data selector)
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov fs, ax
    mov gs, ax
    mov esp, 0x90000        ; simple stack in low mem

    ; Jump to loaded stage3 at linear 0x00010000
    jmp dword 0x00010000

; ------------------------
; Back to 16-bit context for data and GDT
; ------------------------
BITS 16
disk_error:
    mov si, disk_msg
.dloop:
    lodsb
    test al, al
    jz .hang
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x04
    int 0x10
    jmp .dloop
.hang:
    hlt
    jmp .hang

; ------------------------
; GDT (Flat model): null, code, data
; ------------------------
gdt_start:
    dq 0x0000000000000000           ; null
    dq 0x00CF9A000000FFFF           ; code: base=0, limit=0xFFFFF, 4K, 32-bit, exec/read
    dq 0x00CF92000000FFFF           ; data: base=0, limit=0xFFFFF, 4K, 32-bit, read/write
gdt_end:

gdtr:
    dw gdt_end - gdt_start - 1
    dd gdt_start

msg2 db "Stage 2: loading stage3 and switching to 32-bit...", 0
disk_msg db " INT13h read error", 0

; Pad stage2 to one sector (512 bytes)
TIMES 512 - ($ - $$) db 0
