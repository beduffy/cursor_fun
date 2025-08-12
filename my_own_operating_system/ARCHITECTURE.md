# BenOS (my_own_operating_system) – Architecture and Deep Dive

This guide explains how the boot flow works from BIOS to 32-bit protected mode. It also documents the code layout, build process, and how to extend the system.

- Audience: developers learning OS bootstrapping (BIOS, real mode, protected mode)
- State: minimal but robust, with diagnostics and 32-bit payload

## Quickstart

- From repo root:
```bash
make clean
make image
make run
```
- Or only this folder:
```bash
make -C my_own_operating_system clean image run
```
- QEMU writes debug markers to `my_own_operating_system/debug.log` (I/O port 0xE9). Selected QEMU traces go to `qemu.log`.

## High-level architecture

See `diagrams/architecture.png` and `diagrams/memory_map.png`.

Key path:
- Stage0: `boot.asm` (LBA 0, 512B) – boots in real mode at 0x7C00, prints, loads stage2 with CHS then LBA fallback, jumps to 0x0000:0x7E00
- Stage1: `stage2.asm` (LBA 1, 512B) – clears screen, reads stage3 to 0x8000, sets up flat GDT, switches to protected mode, far-jumps to 32-bit and then to 0x00008000
- Stage2: `stage3.asm` (LBA 2, ≤512B) – 32-bit payload that writes to VGA text buffer (0xB8000)

## Disk layout

- LBA 0: `boot.bin` (boot sector, ends with 0x55AA)
- LBA 1: `stage2.bin` (one sector)
- LBA 2: `stage3.bin` (one sector)
- `disk.img` is a 1.44MB floppy image; we only use first 3 sectors

The Makefile always writes exactly one sector for stage2 and stage3.

## Stage0 – `boot.asm` (real mode, 0x7C00)

Responsibilities:
- Minimal setup: `DS=ES=SS=0`, `SP=0x7C00`
- Save boot drive (`DL`)
- Status output via BIOS teletype (`INT 10h, AH=0x0E`)
- Load stage2 to `0000:7E00`
- Robust reads: try CHS (AH=0x02); on error, print `[E=xx]`, reset (`AH=0`), retry; if still failing, use INT 13h Extensions LBA read (AH=0x42) with a Disk Address Packet (DAP)
- Jump to 0x0000:0x7E00

Diagnostics: emits single-byte markers to debug port 0xE9 (captured in `debug.log`):
- `A`: entered boot sector, saved DL
- `B`: attempting CHS read
- `C`: stage2 read success
- `E`: fatal disk error path

## Stage1 – `stage2.asm` (real mode → protected mode)

Responsibilities:
- Clear screen and print status
- Read stage3 (LBA 2) to physical 0x8000
- Install flat GDT (code selector 0x08, data selector 0x10)
- Set `CR0.PE=1` to enable protected mode
- Far jump to 32-bit label (`pm_entry`), load data segments with 0x10, set `ESP`, then `jmp 0x00008000`

Diagnostics: emits `S` at entry and `T` on successful stage3 read to port 0xE9.

## Stage2 – `stage3.asm` (32-bit payload)

- Built with `BITS 32`, `ORG 0x00008000`
- Writes "Stage 3: Hello from 32-bit mode!" to VGA text buffer at 0xB8000
- Confirms mode switch and linear addressing without paging

## Memory map (simplified)

See `diagrams/memory_map.png`.

- Real mode:
  - 0x0000–0x03FF: IVT
  - 0x0400–0x04FF: BIOS data area
  - 0x7C00: boot sector loaded by BIOS
  - 0x7E00: stage2 load/execute
  - 0x8000: stage3 load target
- Protected mode:
  - Linear 0x00008000: stage3 (32-bit)
  - 0x000B8000: VGA text buffer

## Build system

- `NASM` assembles each stage to raw binaries
- `dd` composes `disk.img` with stage binaries at fixed LBAs
- `QEMU` boots the image as a floppy (`-drive file=disk.img,format=raw,if=floppy`)
- Run target adds debug facilities:
  - `-serial mon:stdio` routes serial and monitor to terminal
  - `-debugcon file:debug.log -global isa-debugcon.iobase=0xe9` captures port 0xE9 writes
  - `-D qemu.log -d guest_errors,int` enables selected QEMU tracing

## Extending the system

Short roadmap:
1. Use LBA reads from stage2 for all disk operations (multi-sector)
2. Add 32-bit freestanding C toolchain (i686-elf-gcc), C runtime, kernel entry
3. Install IDT, remap PIC, add PIT timer and keyboard
4. E820 memory map, enable paging, add physical/virtual allocators
5. FAT12/FAT16 reader to load larger kernels
6. Minimal shell and drivers (serial, simple FS commands)

## Safety

Everything runs in QEMU; no writes to host disks. The Makefile only touches files in this folder.

## References

- Intel SDM Vol. 1–3 (IA-32)
- Ralf Brown's Interrupt List (INT 10h/13h)
- OSDev Wiki: https://wiki.osdev.org
