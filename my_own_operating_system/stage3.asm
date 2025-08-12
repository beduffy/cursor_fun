; Stage 3: 32-bit flat mode payload
; Prints a message using VGA text buffer (0xB8000) and halts

BITS 32
ORG 0x00008000

start_stage3:
    ; Set VGA text mode cursor to top-left and print string
    mov esi, msg
    mov edi, 0xB8000

.fill:
    lodsb
    test al, al
    jz .done
    mov ah, 0x1F
    stosw
    jmp .fill

.done:
.hang:
    hlt
    jmp .hang

msg db "Stage 3: Hello from 32-bit mode!", 0
msg_len equ $-msg
