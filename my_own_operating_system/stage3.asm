; Stage 3: 32-bit flat mode payload
; Prints a message using VGA text buffer (0xB8000) and halts

BITS 32
ORG 0x00010000

start_stage3:
    mov esi, msg
    mov edi, 0xB8000
    mov ecx, msg_len

.fill:
    lodsb                    ; AL = *ESI++
    test al, al
    jz .done
    mov ah, 0x1F            ; bright white on blue
    stosw                   ; store AX to [EDI], EDI += 2
    jmp .fill

.done:
.hang:
    hlt
    jmp .hang

msg db "Stage 3: Hello from 32-bit mode!", 0
msg_len equ $-msg
