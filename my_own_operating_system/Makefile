# BenOS build
.RECIPEPREFIX := |

NASM := nasm
QEMU := qemu-system-i386
DD := dd

BUILD_DIR := .
BOOT_ASM := $(BUILD_DIR)/boot.asm
STAGE2_ASM := $(BUILD_DIR)/stage2.asm
STAGE3_ASM := $(BUILD_DIR)/stage3.asm
BOOT_BIN := $(BUILD_DIR)/boot.bin
STAGE2_BIN := $(BUILD_DIR)/stage2.bin
STAGE3_BIN := $(BUILD_DIR)/stage3.bin
DISK_IMG := $(BUILD_DIR)/disk.img

.PHONY: all image run clean check

all: $(BOOT_BIN) $(STAGE2_BIN) $(STAGE3_BIN)

$(BOOT_BIN): $(BOOT_ASM)
| $(NASM) -f bin -o $@ $<
| @printf "Built %s (size: %d bytes)\n" $@ $$(stat -c%s $@)

$(STAGE2_BIN): $(STAGE2_ASM)
| $(NASM) -f bin -o $@ $<
| @printf "Built %s (size: %d bytes)\n" $@ $$(stat -c%s $@)

$(STAGE3_BIN): $(STAGE3_ASM)
| $(NASM) -f bin -o $@ $<
| @printf "Built %s (size: %d bytes)\n" $@ $$(stat -c%s $@)

image: $(BOOT_BIN) $(STAGE2_BIN) $(STAGE3_BIN)
| # Create 1.44MB floppy image
| $(DD) if=/dev/zero of=$(DISK_IMG) bs=512 count=2880 status=none
| # Write boot sector at LBA 0
| $(DD) if=$(BOOT_BIN) of=$(DISK_IMG) bs=512 count=1 conv=notrunc status=none
| # Write stage2 to LBA 1 (sector 2 in CHS) – exactly one sector
| $(DD) if=$(STAGE2_BIN) of=$(DISK_IMG) bs=512 count=1 seek=1 conv=notrunc status=none
| # Write stage3 to LBA 2 (sector 3 in CHS) – exactly one sector
| $(DD) if=$(STAGE3_BIN) of=$(DISK_IMG) bs=512 count=1 seek=2 conv=notrunc status=none
| @echo "Created $(DISK_IMG)"

run: image
| $(QEMU) -machine pc -drive file=$(DISK_IMG),format=raw,if=floppy -serial mon:stdio -no-reboot -no-shutdown

clean:
| rm -f $(BOOT_BIN) $(STAGE2_BIN) $(STAGE3_BIN) $(DISK_IMG)

check:
| @which $(NASM) >/dev/null 2>&1 && echo "nasm: OK ($$(nasm -v | head -n1))" || echo "nasm: MISSING"
| @which $(QEMU) >/dev/null 2>&1 && echo "qemu-system-i386: OK ($$(qemu-system-i386 --version | head -n1))" || echo "qemu-system-i386: MISSING"
| @which $(DD) >/dev/null 2>&1 && echo "dd: OK" || echo "dd: MISSING"
