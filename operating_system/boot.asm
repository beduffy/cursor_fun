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
    mov ah, 0x02            ; INT 13h, AH=2: Read sectors
    mov al, 0x01            ; AL = number of sectors to read
    mov ch, 0x00            ; Cylinder 0
    mov cl, 0x02            ; Sector 2 (LBA1)
    mov dh, 0x00            ; Head 0
    mov dl, 0x00            ; Drive 0 (A:)
    int 0x13
    jc .disk_error

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

; Pad to 510 bytes, then add boot signature 0x55AA
TIMES 510 - ($ - $$) db 0
DW 0xAA55
