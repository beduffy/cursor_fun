; Minimal 16-bit boot sector for BenOS
; Assembles to 512 bytes with boot signature 0x55AA
; Prints a hello message using BIOS teletype and halts.

BITS 16
ORG 0x7C00

start:
    cli
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x7C00
    sti

    ; Remember BIOS boot drive (DL) and drop a debug marker A to port 0xE9
    mov [boot_drive], dl
    mov dx, 0x00E9
    mov al, 'A'
    out dx, al

    ; Print initial message
    cld                     ; ensure forward for LODSB
    mov si, msg
.print_loop:
    lodsb
    test al, al
    jz .after_msg
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x07
    int 0x10
    jmp .print_loop

.after_msg:
    ; Load stage2 (1 sector) to 0x0000:0x7E00 using BIOS INT 13h
    mov bx, 0x7E00          ; ES:BX = 0000:7E00
    mov es, ax              ; ES = 0x0000
    ; Try CHS read with retries first
.read_retry:
    mov dx, 0x00E9
    mov al, 'B'
    out dx, al
    mov ah, 0x02
    mov al, 0x01
    mov ch, 0x00
    mov cl, 0x02
    mov dh, 0x00
    mov dl, [boot_drive]
    int 0x13
    jnc .read_ok
    ; print error code AH as hex
    push ax
    mov al, ' '
    call bios_putchar
    mov al, '['
    call bios_putchar
    mov al, 'E'
    call bios_putchar
    mov al, '='
    call bios_putchar
    pop ax
    xchg ah, al
    call print_hex8
    mov al, ']'
    call bios_putchar
    ; reset disk and retry up to 2 more times
    mov dl, [boot_drive]
    mov ah, 0x00
    int 0x13
    dec byte [retries]
    jnz .read_retry

    ; Fallback to INT13h extensions (LBA) if supported
    mov dl, [boot_drive]
    mov ah, 0x41
    mov bx, 0x55AA
    int 0x13
    jc .disk_error
    cmp bx, 0xAA55
    jne .disk_error
    ; Prepare DAP for LBA 1 -> 0000:7E00
    mov word [dap+2], 1         ; sectors=1
    mov dword [dap+4], 0x00007E00 ; buffer ptr (offset:segment)
    mov dword [dap+8], 1        ; LBA low dword
    mov dword [dap+12], 0       ; LBA high dword
    mov si, dap
    mov ah, 0x42
    mov dl, [boot_drive]
    int 0x13
    jc .disk_error
.read_ok:
    ; Indicate success; debug marker C
    mov dx, 0x00E9
    mov al, 'C'
    out dx, al
    ; Indicate success
    mov si, ok1_msg
.ok1_loop:
    lodsb
    test al, al
    jz .after_ok1
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x07
    int 0x10
    jmp .ok1_loop
.after_ok1:

    ; Jump to stage2 at 0000:7E00
    jmp 0x0000:0x7E00

.disk_error:
    mov dx, 0x00E9
    mov al, 'E'
    out dx, al
    mov si, disk_msg
.disk_loop:
    lodsb
    test al, al
    jz .hang
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x04            ; Red on black
    int 0x10
    jmp .disk_loop

.hang:
    hlt
    jmp .hang

msg db "BenOS: Loading stage2...", 0
disk_msg db "Disk read error", 0
boot_drive db 0
retries db 3
ok1_msg db " [OK]", 0

; Disk Address Packet (DAP) for INT 13h Extensions read (AH=42h)
dap:
    db 0x10            ; size of packet
    db 0x00            ; reserved
    dw 0x0000          ; sectors to transfer (will set at runtime)
    dd 0x00000000      ; buffer offset:segment (will set at runtime)
    dd 0x00000000      ; LBA low dword (will set at runtime)
    dd 0x00000000      ; LBA high dword (will set at runtime)

; ------------------------
; Tiny BIOS teletype helpers
; ------------------------
bios_putchar:
    push ax
    push bx
    mov ah, 0x0E
    mov bh, 0x00
    mov bl, 0x07
    int 0x10
    pop bx
    pop ax
    ret

print_hex8:
    push ax
    push cx
    mov cx, 2
.hex_loop:
    rol al, 4
    mov ah, al
    and ah, 0x0F
    cmp ah, 10
    sbb ah, 69h
    das
    add ah, '0'
    cmp ah, '9'
    jbe .emit
    add ah, 7
.emit:
    mov al, ah
    call bios_putchar
    loop .hex_loop
    pop cx
    pop ax
    ret

; Pad to 510 bytes, then add boot signature 0x55AA
TIMES 510 - ($ - $$) db 0
DW 0xAA55
