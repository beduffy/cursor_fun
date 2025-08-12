; Stage 2 loader for BenOS (16-bit real mode)
; Loaded by boot sector to 0x0000:0x7E00 (physical 0x7E00)

BITS 16
ORG 0x7E00

start_stage2:
    mov si, msg2
.print_loop:
    lodsb
    test al, al
    jz .done
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x0A            ; Light green on black
    int 0x10
    jmp .print_loop

.done:
.hang:
    hlt
    jmp .hang

msg2 db "Stage 2 loaded!", 0

; No signature required for stage 2, but pad to one sector for simplicity
TIMES 512 - ($ - $$) db 0
