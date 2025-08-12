# BenOS - Minimal 16-bit Boot Sector

This folder contains a tiny boot sector that prints a message using BIOS teletype and halts. It's the first stepping stone toward a simple operating system.

## Requirements
- nasm
- qemu-system-i386

You can check tool availability with:

```bash
make -C operating_system check
```

## Build

```bash
make -C operating_system
```

This produces `boot.bin` (512 bytes) with the `0x55AA` signature.

## Run in QEMU

```bash
make -C operating_system run
```

QEMU will boot the boot sector and print the greeting.

## Next Steps
- Set up a proper GDT and transition to 32-bit protected mode.
- Add a stage-2 loader to read sectors from disk.
- Switch to a freestanding C runtime for higher-level code.

## Disk Image

- Build both stages and create a floppy image:``make -C operating_system image```
- Run with QEMU:``make -C operating_system run```
