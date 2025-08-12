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

    ; Remember BIOS boot drive (DL)
    mov [boot_drive], dl

    ; Print initial message
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
    ; Try to read stage2 with retries
.read_retry:
    mov ah, 0x02            ; INT 13h, AH=2: Read sectors
    mov al, 0x01            ; AL = number of sectors to read
    mov ch, 0x00            ; Cylinder 0
    mov cl, 0x02            ; Sector 2 (LBA1)
    mov dh, 0x00            ; Head 0
    mov dl, [boot_drive]    ; Use actual boot drive from BIOS
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
    pop ax                   ; AH contains error
    xchg ah, al              ; move error to AL
    call print_hex8
    mov al, ']'
    call bios_putchar
    ; reset disk and retry up to 3 times
    mov dl, [boot_drive]
    mov ah, 0x00
    int 0x13
    dec byte [retries]
    jnz .read_retry
    jmp .disk_error
.read_ok:
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
