# BenOS — a tiny three‑stage boot to 32‑bit mode (explained like you’re new to OS dev)

This folder contains a very small program that your PC can boot. It starts in the oldest, simplest mode of the CPU (16‑bit “real mode”), loads a second piece, switches the CPU into 32‑bit “protected mode,” and then shows a message. It’s not a full operating system, but it’s the first real step: you control what the computer does right after power‑on.

Think of it like a relay race:
- Stage 1 (boot sector) starts running automatically and loads Stage 2.
- Stage 2 prepares the modern 32‑bit track and loads Stage 3.
- Stage 3 is where you can start writing “normal” code that isn’t tied to 1980s rules.


## What you need
- `nasm` (assembler)
- `qemu-system-i386` (virtual machine to run the image safely)

Check if you have them:

```bash
make -C my_own_operating_system check
```

Install (examples):
- Ubuntu/Debian: `sudo apt-get update && sudo apt-get install -y nasm qemu-system-x86`
- macOS (Homebrew): `brew install nasm qemu`


## Quick start
From the repo root:

```bash
make -C my_own_operating_system       # builds boot.bin, stage2.bin, stage3.bin
make -C my_own_operating_system run   # creates disk.img and boots it in QEMU
```

Or step-by-step:

```bash
make -C my_own_operating_system image  # builds and packs all stages into disk.img
make -C my_own_operating_system run    # boots disk.img in QEMU
```

Clean up:

```bash
make -C my_own_operating_system clean
```

What you should see:
- A message that Stage 1 is loading Stage 2, then “ [OK]” if it works.
- Stage 2 says it’s loading Stage 3 and switching to 32‑bit mode.
- Stage 3 prints a 32‑bit hello message using the screen’s text memory, then halts.


## How it works (no heavy jargon)

### Stage 1 — `boot.asm` (16‑bit, runs first)
- The PC’s firmware (BIOS) reads the very first 512 bytes from the “disk” (our `disk.img`) into memory at address 0x7C00 and jumps there. That 512‑byte chunk is called the “boot sector.” It must end with the signature `0x55AA` so BIOS knows it’s bootable.
- Stage 1 prints: “BenOS: Loading stage2…”
- It tries to read the next sector of the disk (Stage 2) into memory at 0x7E00. It first uses an older disk method (called CHS). If that fails, it tries a newer method (called LBA). If there’s an error, it shows `[E=xx]` where `xx` is a hex error code.
- If reading succeeds, it prints ` [OK]` and jumps to 0x7E00 to start Stage 2.

In plain language: read one more block from the disk into memory, then go there.

### Stage 2 — `stage2.asm` (still 16‑bit at entry, then switches to 32‑bit)
- Prints a status message and clears the screen.
- Reads Stage 3 (another 512‑byte block) into memory at 0x8000.
- Prepares a tiny “map” called a GDT (Global Descriptor Table). You can think of the GDT as telling the CPU “treat all memory as one big flat space” so 32‑bit code is nice and simple.
- Flips the CPU into 32‑bit mode (protected mode) by setting a bit in a control register.
- Does a special jump into 32‑bit code.
- In 32‑bit mode it sets all data segments and a stack, then jumps to the Stage 3 code we loaded at 0x00008000.

In plain language: load the final piece, turn on 32‑bit mode, and go run it.

### Stage 3 — `stage3.asm` (32‑bit)
- Writes text directly into the screen’s text buffer at memory address 0xB8000. Each character on screen is two bytes: the letter and its color. This is why you’ll see a colored message.
- Then it halts the CPU in a loop.

In plain language: we’re now in a modern mode and drawing text the simple way.


## File tour
- `boot.asm` — The 16‑bit boot sector (exactly 512 bytes) BIOS runs first. Prints and loads Stage 2.
- `stage2.asm` — Loader that prints, loads Stage 3, sets up 32‑bit mode (GDT + mode switch), and jumps there.
- `stage3.asm` — 32‑bit payload that writes to the VGA text buffer.
- `Makefile` — Builds the stages, creates `disk.img`, runs QEMU.
- `disk.img` — A 1.44MB floppy image created by the build. Layout:
  - LBA 0: `boot.bin` (the boot sector)
  - LBA 1: `stage2.bin`
  - LBA 2: `stage3.bin`


## Memory & disk addresses (just enough to follow along)
- BIOS loads Stage 1 to 0x7C00 (standard PC convention).
- Stage 1 loads Stage 2 to 0x7E00, then jumps there.
- Stage 2 loads Stage 3 to 0x8000.
- After switching to 32‑bit mode, Stage 2 sets a stack at 0x90000 and jumps to 0x8000.
- Stage 3 writes to VGA text memory at 0xB8000.


## Make targets you can use
- `make -C my_own_operating_system` — Build all stages.
- `make -C my_own_operating_system image` — Build and create `disk.img` with all stages placed at the right sectors.
- `make -C my_own_operating_system run` — Boot the image in QEMU.
- `make -C my_own_operating_system clean` — Remove built files and the image.
- `make -C my_own_operating_system check` — Show whether required tools are installed.


## Customize it
- Change messages: look for `msg` strings in `boot.asm`, `stage2.asm`, `stage3.asm` and edit the text.
- Change colors: in `stage3.asm`, the attribute byte `0x1F` controls foreground/background colors.
- Add more code to Stage 3: handle keyboard input, draw more text, etc. At this point you’re in 32‑bit land where programming gets friendlier.


## Troubleshooting
- Missing tools: run `make -C my_own_operating_system check` to see what’s missing.
- Disk read error in Stage 1: if you see ` [E=xx]`, that number is the BIOS disk error. In QEMU this is rare, but if it happens the code automatically falls back to a newer disk method (LBA). Try `make clean` then `make run` again.
- Nothing shows up: make sure QEMU is installed and you ran `make run`. Some terminals hide QEMU’s display behind other windows; check your desktop or try running from a local terminal.
- Safe to try: everything runs inside QEMU and writes to `disk.img`, not your real disks.


## Glossary (plain language)
- BIOS: Very early firmware that runs before your code. It loads the first 512 bytes from the boot disk and jumps there.
- Boot sector: Those first 512 bytes. Must end with `0x55AA` to be considered bootable by BIOS.
- Sector: The basic “block” the disk reads/writes (here, 512 bytes).
- CHS/LBA: Two ways to ask the disk for data. CHS is the old style. LBA is the newer, simpler “give me block N.” The code tries CHS first, then LBA if needed.
- Real mode: The CPU’s “retro” mode (16‑bit), compatible with very old PCs. No memory protection.
- Protected mode: The 32‑bit mode where memory is flat and life is better for writing normal code.
- GDT: A tiny table that tells the CPU how to treat memory when in protected mode. We use it to make a simple “flat” layout.
- VGA text buffer: A memory area at 0xB8000 where writing characters shows up on screen.


## Next steps if you want to grow this
- Add a keyboard driver (poll the keyboard controller and print keys).
- Set up interrupts and an IDT.
- Add paging (virtual memory) and a heap.
- Load and run freestanding C code (a small runtime) from Stage 3.

Have fun — you now control what happens the instant the computer boots! 🚀
