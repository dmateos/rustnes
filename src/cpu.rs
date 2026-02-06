//! MOS Technology 6502 CPU emulation.
//!
//! The 6502 is an 8-bit microprocessor used in the NES, Apple II, Commodore 64, and many other
//! classic systems. This module emulates the CPU's registers, instruction set, and addressing modes.
//!
//! # Architecture
//!
//! - 8-bit accumulator (A)
//! - 8-bit index registers (X, Y)
//! - 8-bit stack pointer (SP) - points to $0100-$01FF
//! - 16-bit program counter (PC)
//! - 8-bit status register (P) with flags: N V - B D I Z C

use crate::bus::Bus;

/// CPU status flags.
///
/// These flags are set/cleared by various instructions and affect conditional branching.
/// In Python, you might use individual boolean variables or bit flags.
/// Rust's bitflags would be common, but we'll use const u8 for clarity.
const CARRY: u8 = 0b0000_0001; // C: Carry flag
const ZERO: u8 = 0b0000_0010; // Z: Zero flag
const INTERRUPT_DISABLE: u8 = 0b0000_0100; // I: Interrupt disable
const DECIMAL: u8 = 0b0000_1000; // D: Decimal mode (not used in NES)
const BREAK: u8 = 0b0001_0000; // B: Break command
const UNUSED: u8 = 0b0010_0000; // -: Unused (always set)
const OVERFLOW: u8 = 0b0100_0000; // V: Overflow flag
const NEGATIVE: u8 = 0b1000_0000; // N: Negative flag

/// MOS 6502 CPU.
///
/// This struct represents the complete state of the 6502 processor.
/// It owns a Bus to access memory and I/O.
pub struct Cpu {
    /// Accumulator register (8-bit)
    pub a: u8,

    /// X index register (8-bit)
    pub x: u8,

    /// Y index register (8-bit)
    pub y: u8,

    /// Stack pointer (8-bit) - points into page $01 ($0100-$01FF)
    /// Grows downward from $01FF
    pub sp: u8,

    /// Program counter (16-bit) - points to next instruction
    pub pc: u16,

    /// Status register (8-bit) - processor flags
    /// Bits: NV-BDIZC
    pub status: u8,

    /// Memory bus
    pub bus: Bus,

    /// Total cycles executed
    pub cycles: u64,
}

impl Cpu {
    /// Create a new CPU with the given bus.
    ///
    /// The CPU starts in an uninitialized state. Call `reset()` to properly initialize it.
    ///
    /// # Arguments
    ///
    /// * `bus` - The memory bus connecting to RAM, ROM, PPU, etc.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use rustnes::cpu::Cpu;
    /// use rustnes::bus::Bus;
    ///
    /// let bus = Bus::new();
    /// let mut cpu = Cpu::new(bus);
    /// cpu.reset();
    /// ```
    pub fn new(bus: Bus) -> Self {
        Cpu {
            a: 0,
            x: 0,
            y: 0,
            sp: 0xFD, // Stack pointer typically starts at 0xFD
            pc: 0,
            status: UNUSED | INTERRUPT_DISABLE, // Unused flag always set, interrupts disabled
            bus,
            cycles: 0,
        }
    }

    /// Reset the CPU to its initial state.
    ///
    /// This simulates the RESET interrupt. The PC is loaded from the reset vector at $FFFC-$FFFD.
    /// All registers are reset to known values.
    pub fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0xFD;
        self.status = UNUSED | INTERRUPT_DISABLE;

        // Load PC from reset vector at $FFFC-$FFFD
        self.pc = self.bus.read_u16(0xFFFC);

        self.cycles = 0;
    }

    /// Handle NMI (Non-Maskable Interrupt).
    ///
    /// This is called when the PPU enters VBlank and NMI is enabled.
    /// The CPU pushes PC and status to stack, then jumps to the NMI handler.
    pub fn nmi(&mut self) {
        // Push PC (high byte first, then low byte)
        self.stack_push_u16(self.pc);

        // Push status (with B flag clear, unused flag set)
        self.stack_push(self.status & !BREAK | UNUSED);

        // Set interrupt disable flag
        self.status |= INTERRUPT_DISABLE;

        // Load PC from NMI vector at $FFFA-$FFFB
        self.pc = self.bus.read_u16(0xFFFA);

        // NMI takes 7 cycles
        self.cycles += 7;
    }

    /// Handle IRQ (Interrupt Request).
    ///
    /// This is called when an IRQ is triggered (e.g., by MMC3 scanline counter).
    /// Unlike NMI, IRQ can be disabled by the I flag in the status register.
    /// The CPU pushes PC and status to stack, then jumps to the IRQ handler.
    pub fn irq(&mut self) {
        // Check if interrupts are disabled
        if (self.status & INTERRUPT_DISABLE) != 0 {
            return;
        }

        // Push PC (high byte first, then low byte)
        self.stack_push_u16(self.pc);

        // Push status (with B flag clear, unused flag set)
        self.stack_push(self.status & !BREAK | UNUSED);

        // Set interrupt disable flag
        self.status |= INTERRUPT_DISABLE;

        // Load PC from IRQ vector at $FFFE-$FFFF
        self.pc = self.bus.read_u16(0xFFFE);

        // IRQ takes 7 cycles
        self.cycles += 7;
    }

    /// Execute one instruction and return the number of cycles it took.
    ///
    /// # Returns
    ///
    /// The number of CPU cycles the instruction took to execute
    pub fn step(&mut self) -> u8 {
        // Debug: Detect when CPU exits wait loop and show what's executing inside
        static mut IN_WAIT_LOOP: bool = false;
        static mut LOOP_INSTRUCTION_COUNT: u32 = 0;
        unsafe {
            if self.pc >= 0xFF5A && self.pc <= 0xFF5F {
                if !IN_WAIT_LOOP {
                    println!("*** CPU ENTERED WAIT LOOP at ${:04X} ***", self.pc);
                    IN_WAIT_LOOP = true;
                    LOOP_INSTRUCTION_COUNT = 0;
                }

                // Show first 20 instructions in wait loop to see the pattern
                LOOP_INSTRUCTION_COUNT += 1;
                let count = LOOP_INSTRUCTION_COUNT;
                if count <= 20 {
                    println!(
                        "  Loop[{}] ${:04X}: {:02X} (A=${:02X} X=${:02X} Y=${:02X} P=${:02X})",
                        count,
                        self.pc,
                        self.bus.read(self.pc),
                        self.a,
                        self.x,
                        self.y,
                        self.status
                    );
                }
            } else if IN_WAIT_LOOP {
                println!(
                    "*** CPU EXITED WAIT LOOP! Now at ${:04X} (A=${:02X}) ***",
                    self.pc, self.a
                );
                IN_WAIT_LOOP = false;
            }

            // Print instructions outside wait loop
            if self.cycles < 500 && !IN_WAIT_LOOP {
                println!(
                    "CPU: ${:04X}: {:02X} (A=${:02X} X=${:02X} Y=${:02X})",
                    self.pc,
                    self.bus.read(self.pc),
                    self.a,
                    self.x,
                    self.y
                );
            }
        }

        let opcode = self.read_pc_byte();
        let cycles_before = self.cycles;

        self.execute(opcode);

        (self.cycles - cycles_before) as u8
    }

    /// Read a byte from the PC and increment PC.
    ///
    /// In Python: `value = memory[pc]; pc += 1`
    fn read_pc_byte(&mut self) -> u8 {
        let value = self.bus.read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }

    /// Read a 16-bit word from the PC and increment PC by 2.
    fn read_pc_word(&mut self) -> u16 {
        let low = self.read_pc_byte() as u16;
        let high = self.read_pc_byte() as u16;
        (high << 8) | low
    }

    // ==================== Status Flag Operations ====================

    /// Set or clear a flag in the status register.
    ///
    /// # Arguments
    ///
    /// * `flag` - The flag constant (e.g., CARRY, ZERO)
    /// * `value` - true to set the flag, false to clear it
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.status |= flag;
        } else {
            self.status &= !flag;
        }
    }

    /// Get the value of a flag in the status register.
    fn get_flag(&self, flag: u8) -> bool {
        (self.status & flag) != 0
    }

    /// Update the Zero and Negative flags based on a value.
    ///
    /// This is commonly done after load and arithmetic operations.
    /// In Python: `zero = (value == 0); negative = (value & 0x80) != 0`
    fn update_zero_and_negative_flags(&mut self, value: u8) {
        self.set_flag(ZERO, value == 0);
        self.set_flag(NEGATIVE, (value & 0x80) != 0);
    }

    // ==================== Stack Operations ====================

    /// Push a byte onto the stack.
    ///
    /// The stack is located at $0100-$01FF and grows downward.
    fn stack_push(&mut self, value: u8) {
        let addr = 0x0100 | (self.sp as u16);
        self.bus.write(addr, value);
        self.sp = self.sp.wrapping_sub(1);
    }

    /// Push a 16-bit word onto the stack (high byte first, then low byte).
    fn stack_push_u16(&mut self, value: u16) {
        let high = (value >> 8) as u8;
        let low = (value & 0xFF) as u8;
        self.stack_push(high);
        self.stack_push(low);
    }

    /// Pop a byte from the stack.
    fn stack_pop(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        let addr = 0x0100 | (self.sp as u16);
        self.bus.read(addr)
    }

    /// Pop a 16-bit word from the stack (low byte first, then high byte).
    fn stack_pop_u16(&mut self) -> u16 {
        let low = self.stack_pop() as u16;
        let high = self.stack_pop() as u16;
        (high << 8) | low
    }

    // ==================== Addressing Modes ====================

    /// Get the effective address for an instruction based on addressing mode.
    ///
    /// Returns (address, page_crossed) where page_crossed indicates if a page boundary was crossed
    /// (which adds an extra cycle for some instructions).
    fn get_operand_address(&mut self, mode: AddressingMode) -> (u16, bool) {
        match mode {
            AddressingMode::Immediate => {
                let addr = self.pc;
                self.pc = self.pc.wrapping_add(1);
                (addr, false)
            }
            AddressingMode::ZeroPage => {
                let addr = self.read_pc_byte() as u16;
                (addr, false)
            }
            AddressingMode::ZeroPageX => {
                let base = self.read_pc_byte();
                let addr = base.wrapping_add(self.x) as u16;
                (addr, false)
            }
            AddressingMode::ZeroPageY => {
                let base = self.read_pc_byte();
                let addr = base.wrapping_add(self.y) as u16;
                (addr, false)
            }
            AddressingMode::Absolute => {
                let addr = self.read_pc_word();
                (addr, false)
            }
            AddressingMode::AbsoluteX => {
                let base = self.read_pc_word();
                let addr = base.wrapping_add(self.x as u16);
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
                (addr, page_crossed)
            }
            AddressingMode::AbsoluteY => {
                let base = self.read_pc_word();
                let addr = base.wrapping_add(self.y as u16);
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
                (addr, page_crossed)
            }
            AddressingMode::Indirect => {
                let ptr = self.read_pc_word();
                // 6502 bug: if ptr is at page boundary (e.g. $xxFF), wraps within page
                let addr = if ptr & 0x00FF == 0x00FF {
                    let low = self.bus.read(ptr) as u16;
                    let high = self.bus.read(ptr & 0xFF00) as u16;
                    (high << 8) | low
                } else {
                    self.bus.read_u16(ptr)
                };
                (addr, false)
            }
            AddressingMode::IndirectX => {
                let base = self.read_pc_byte();
                let ptr = base.wrapping_add(self.x);
                let low = self.bus.read(ptr as u16) as u16;
                let high = self.bus.read(ptr.wrapping_add(1) as u16) as u16;
                let addr = (high << 8) | low;
                (addr, false)
            }
            AddressingMode::IndirectY => {
                let base = self.read_pc_byte();
                let low = self.bus.read(base as u16) as u16;
                let high = self.bus.read(base.wrapping_add(1) as u16) as u16;
                let ptr = (high << 8) | low;
                let addr = ptr.wrapping_add(self.y as u16);
                let page_crossed = (ptr & 0xFF00) != (addr & 0xFF00);
                (addr, page_crossed)
            }
            AddressingMode::Relative => {
                let offset = self.read_pc_byte() as i8;
                let addr = self.pc.wrapping_add(offset as u16);
                let page_crossed = (self.pc & 0xFF00) != (addr & 0xFF00);
                (addr, page_crossed)
            }
            AddressingMode::Implied | AddressingMode::Accumulator => (0, false),
        }
    }

    // ==================== Instruction Execution ====================

    /// Execute a single instruction based on its opcode.
    fn execute(&mut self, opcode: u8) {
        match opcode {
            // LDA - Load Accumulator
            0xA9 => self.lda(AddressingMode::Immediate, 2),
            0xA5 => self.lda(AddressingMode::ZeroPage, 3),
            0xB5 => self.lda(AddressingMode::ZeroPageX, 4),
            0xAD => self.lda(AddressingMode::Absolute, 4),
            0xBD => self.lda(AddressingMode::AbsoluteX, 4),
            0xB9 => self.lda(AddressingMode::AbsoluteY, 4),
            0xA1 => self.lda(AddressingMode::IndirectX, 6),
            0xB1 => self.lda(AddressingMode::IndirectY, 5),

            // LDX - Load X Register
            0xA2 => self.ldx(AddressingMode::Immediate, 2),
            0xA6 => self.ldx(AddressingMode::ZeroPage, 3),
            0xB6 => self.ldx(AddressingMode::ZeroPageY, 4),
            0xAE => self.ldx(AddressingMode::Absolute, 4),
            0xBE => self.ldx(AddressingMode::AbsoluteY, 4),

            // LDY - Load Y Register
            0xA0 => self.ldy(AddressingMode::Immediate, 2),
            0xA4 => self.ldy(AddressingMode::ZeroPage, 3),
            0xB4 => self.ldy(AddressingMode::ZeroPageX, 4),
            0xAC => self.ldy(AddressingMode::Absolute, 4),
            0xBC => self.ldy(AddressingMode::AbsoluteX, 4),

            // STA - Store Accumulator
            0x85 => self.sta(AddressingMode::ZeroPage, 3),
            0x95 => self.sta(AddressingMode::ZeroPageX, 4),
            0x8D => self.sta(AddressingMode::Absolute, 4),
            0x9D => self.sta(AddressingMode::AbsoluteX, 5),
            0x99 => self.sta(AddressingMode::AbsoluteY, 5),
            0x81 => self.sta(AddressingMode::IndirectX, 6),
            0x91 => self.sta(AddressingMode::IndirectY, 6),

            // STX - Store X Register
            0x86 => self.stx(AddressingMode::ZeroPage, 3),
            0x96 => self.stx(AddressingMode::ZeroPageY, 4),
            0x8E => self.stx(AddressingMode::Absolute, 4),

            // STY - Store Y Register
            0x84 => self.sty(AddressingMode::ZeroPage, 3),
            0x94 => self.sty(AddressingMode::ZeroPageX, 4),
            0x8C => self.sty(AddressingMode::Absolute, 4),

            // Transfer instructions
            0xAA => self.tax(2),
            0xA8 => self.tay(2),
            0xBA => self.tsx(2),
            0x8A => self.txa(2),
            0x9A => self.txs(2),
            0x98 => self.tya(2),

            // Stack operations
            0x48 => self.pha(3),
            0x08 => self.php(3),
            0x68 => self.pla(4),
            0x28 => self.plp(4),

            // Logical operations
            0x29 => self.and(AddressingMode::Immediate, 2),
            0x25 => self.and(AddressingMode::ZeroPage, 3),
            0x35 => self.and(AddressingMode::ZeroPageX, 4),
            0x2D => self.and(AddressingMode::Absolute, 4),
            0x3D => self.and(AddressingMode::AbsoluteX, 4),
            0x39 => self.and(AddressingMode::AbsoluteY, 4),
            0x21 => self.and(AddressingMode::IndirectX, 6),
            0x31 => self.and(AddressingMode::IndirectY, 5),

            0x09 => self.ora(AddressingMode::Immediate, 2),
            0x05 => self.ora(AddressingMode::ZeroPage, 3),
            0x15 => self.ora(AddressingMode::ZeroPageX, 4),
            0x0D => self.ora(AddressingMode::Absolute, 4),
            0x1D => self.ora(AddressingMode::AbsoluteX, 4),
            0x19 => self.ora(AddressingMode::AbsoluteY, 4),
            0x01 => self.ora(AddressingMode::IndirectX, 6),
            0x11 => self.ora(AddressingMode::IndirectY, 5),

            0x49 => self.eor(AddressingMode::Immediate, 2),
            0x45 => self.eor(AddressingMode::ZeroPage, 3),
            0x55 => self.eor(AddressingMode::ZeroPageX, 4),
            0x4D => self.eor(AddressingMode::Absolute, 4),
            0x5D => self.eor(AddressingMode::AbsoluteX, 4),
            0x59 => self.eor(AddressingMode::AbsoluteY, 4),
            0x41 => self.eor(AddressingMode::IndirectX, 6),
            0x51 => self.eor(AddressingMode::IndirectY, 5),

            // Arithmetic
            0x69 => self.adc(AddressingMode::Immediate, 2),
            0x65 => self.adc(AddressingMode::ZeroPage, 3),
            0x75 => self.adc(AddressingMode::ZeroPageX, 4),
            0x6D => self.adc(AddressingMode::Absolute, 4),
            0x7D => self.adc(AddressingMode::AbsoluteX, 4),
            0x79 => self.adc(AddressingMode::AbsoluteY, 4),
            0x61 => self.adc(AddressingMode::IndirectX, 6),
            0x71 => self.adc(AddressingMode::IndirectY, 5),

            0xE9 => self.sbc(AddressingMode::Immediate, 2),
            0xE5 => self.sbc(AddressingMode::ZeroPage, 3),
            0xF5 => self.sbc(AddressingMode::ZeroPageX, 4),
            0xED => self.sbc(AddressingMode::Absolute, 4),
            0xFD => self.sbc(AddressingMode::AbsoluteX, 4),
            0xF9 => self.sbc(AddressingMode::AbsoluteY, 4),
            0xE1 => self.sbc(AddressingMode::IndirectX, 6),
            0xF1 => self.sbc(AddressingMode::IndirectY, 5),

            // Increment/Decrement
            0xE6 => self.inc(AddressingMode::ZeroPage, 5),
            0xF6 => self.inc(AddressingMode::ZeroPageX, 6),
            0xEE => self.inc(AddressingMode::Absolute, 6),
            0xFE => self.inc(AddressingMode::AbsoluteX, 7),

            0xC6 => self.dec(AddressingMode::ZeroPage, 5),
            0xD6 => self.dec(AddressingMode::ZeroPageX, 6),
            0xCE => self.dec(AddressingMode::Absolute, 6),
            0xDE => self.dec(AddressingMode::AbsoluteX, 7),

            0xE8 => self.inx(2),
            0xC8 => self.iny(2),
            0xCA => self.dex(2),
            0x88 => self.dey(2),

            // Shifts and rotates
            0x0A => self.asl_accumulator(2),
            0x06 => self.asl(AddressingMode::ZeroPage, 5),
            0x16 => self.asl(AddressingMode::ZeroPageX, 6),
            0x0E => self.asl(AddressingMode::Absolute, 6),
            0x1E => self.asl(AddressingMode::AbsoluteX, 7),

            0x4A => self.lsr_accumulator(2),
            0x46 => self.lsr(AddressingMode::ZeroPage, 5),
            0x56 => self.lsr(AddressingMode::ZeroPageX, 6),
            0x4E => self.lsr(AddressingMode::Absolute, 6),
            0x5E => self.lsr(AddressingMode::AbsoluteX, 7),

            0x2A => self.rol_accumulator(2),
            0x26 => self.rol(AddressingMode::ZeroPage, 5),
            0x36 => self.rol(AddressingMode::ZeroPageX, 6),
            0x2E => self.rol(AddressingMode::Absolute, 6),
            0x3E => self.rol(AddressingMode::AbsoluteX, 7),

            0x6A => self.ror_accumulator(2),
            0x66 => self.ror(AddressingMode::ZeroPage, 5),
            0x76 => self.ror(AddressingMode::ZeroPageX, 6),
            0x6E => self.ror(AddressingMode::Absolute, 6),
            0x7E => self.ror(AddressingMode::AbsoluteX, 7),

            // Jumps and calls
            0x4C => self.jmp_absolute(3),
            0x6C => self.jmp_indirect(5),
            0x20 => self.jsr(6),
            0x60 => self.rts(6),

            // Branches
            0x90 => self.bcc(2),
            0xB0 => self.bcs(2),
            0xF0 => self.beq(2),
            0x30 => self.bmi(2),
            0xD0 => self.bne(2),
            0x10 => self.bpl(2),
            0x50 => self.bvc(2),
            0x70 => self.bvs(2),

            // Status flag changes
            0x18 => self.clc(2),
            0xD8 => self.cld(2),
            0x58 => self.cli(2),
            0xB8 => self.clv(2),
            0x38 => self.sec(2),
            0xF8 => self.sed(2),
            0x78 => self.sei(2),

            // Comparisons
            0xC9 => self.cmp(AddressingMode::Immediate, 2),
            0xC5 => self.cmp(AddressingMode::ZeroPage, 3),
            0xD5 => self.cmp(AddressingMode::ZeroPageX, 4),
            0xCD => self.cmp(AddressingMode::Absolute, 4),
            0xDD => self.cmp(AddressingMode::AbsoluteX, 4),
            0xD9 => self.cmp(AddressingMode::AbsoluteY, 4),
            0xC1 => self.cmp(AddressingMode::IndirectX, 6),
            0xD1 => self.cmp(AddressingMode::IndirectY, 5),

            0xE0 => self.cpx(AddressingMode::Immediate, 2),
            0xE4 => self.cpx(AddressingMode::ZeroPage, 3),
            0xEC => self.cpx(AddressingMode::Absolute, 4),

            0xC0 => self.cpy(AddressingMode::Immediate, 2),
            0xC4 => self.cpy(AddressingMode::ZeroPage, 3),
            0xCC => self.cpy(AddressingMode::Absolute, 4),

            // Bit test
            0x24 => self.bit(AddressingMode::ZeroPage, 3),
            0x2C => self.bit(AddressingMode::Absolute, 4),

            // System
            0x00 => self.brk(7),
            0x40 => self.rti(6),
            0xEA => self.nop(2),

            // Unknown opcode - treat as NOP for now
            _ => {
                self.cycles += 2;
            }
        }
    }

    // ==================== Instruction Implementations ====================
    // These methods implement the actual behavior of each instruction

    /// LDA - Load Accumulator
    fn lda(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);

        // Debug: Count PPUSTATUS reads and show when VBlank detected
        if addr == 0x2002 {
            static mut READ_COUNT: u32 = 0;
            unsafe {
                READ_COUNT += 1;
                let count = READ_COUNT;
                if value == 0x80 {
                    println!(
                        "  *** VBlank detected! (read #{}, value=${:02X}) ***",
                        count, value
                    );
                } else if count % 1000 == 0 {
                    println!(
                        "  ... still waiting (read #{}, value=${:02X}) ...",
                        count, value
                    );
                }
            }
        }

        self.a = value;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// LDX - Load X Register
    fn ldx(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        self.x = value;
        self.update_zero_and_negative_flags(self.x);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// LDY - Load Y Register
    fn ldy(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        self.y = value;
        self.update_zero_and_negative_flags(self.y);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// STA - Store Accumulator
    fn sta(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        self.bus.write(addr, self.a);
        self.cycles += cycles as u64;
    }

    /// STX - Store X Register
    fn stx(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        self.bus.write(addr, self.x);
        self.cycles += cycles as u64;
    }

    /// STY - Store Y Register
    fn sty(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        self.bus.write(addr, self.y);
        self.cycles += cycles as u64;
    }

    /// TAX - Transfer A to X
    fn tax(&mut self, cycles: u8) {
        self.x = self.a;
        self.update_zero_and_negative_flags(self.x);
        self.cycles += cycles as u64;
    }

    /// TAY - Transfer A to Y
    fn tay(&mut self, cycles: u8) {
        self.y = self.a;
        self.update_zero_and_negative_flags(self.y);
        self.cycles += cycles as u64;
    }

    /// TSX - Transfer SP to X
    fn tsx(&mut self, cycles: u8) {
        self.x = self.sp;
        self.update_zero_and_negative_flags(self.x);
        self.cycles += cycles as u64;
    }

    /// TXA - Transfer X to A
    fn txa(&mut self, cycles: u8) {
        self.a = self.x;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// TXS - Transfer X to SP
    fn txs(&mut self, cycles: u8) {
        self.sp = self.x;
        self.cycles += cycles as u64;
    }

    /// TYA - Transfer Y to A
    fn tya(&mut self, cycles: u8) {
        self.a = self.y;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// PHA - Push Accumulator
    fn pha(&mut self, cycles: u8) {
        self.stack_push(self.a);
        self.cycles += cycles as u64;
    }

    /// PHP - Push Processor Status
    fn php(&mut self, cycles: u8) {
        // When pushing status, B flag is set
        self.stack_push(self.status | BREAK | UNUSED);
        self.cycles += cycles as u64;
    }

    /// PLA - Pull Accumulator
    fn pla(&mut self, cycles: u8) {
        self.a = self.stack_pop();
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// PLP - Pull Processor Status
    fn plp(&mut self, cycles: u8) {
        self.status = self.stack_pop();
        self.set_flag(BREAK, false); // B flag is cleared
        self.set_flag(UNUSED, true); // Unused flag always set
        self.cycles += cycles as u64;
    }

    /// AND - Logical AND
    fn and(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        self.a &= value;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// ORA - Logical OR
    fn ora(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        self.a |= value;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// EOR - Logical Exclusive OR
    fn eor(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        self.a ^= value;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// ADC - Add with Carry
    fn adc(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let carry = if self.get_flag(CARRY) { 1 } else { 0 };

        let sum = self.a as u16 + value as u16 + carry as u16;

        self.set_flag(CARRY, sum > 0xFF);
        self.set_flag(
            OVERFLOW,
            ((self.a ^ value) & 0x80) == 0 && ((self.a ^ sum as u8) & 0x80) != 0,
        );

        self.a = sum as u8;
        self.update_zero_and_negative_flags(self.a);

        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// SBC - Subtract with Carry
    fn sbc(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let carry = if self.get_flag(CARRY) { 1 } else { 0 };

        // SBC is equivalent to ADC with inverted value
        let diff = self.a as i16 - value as i16 - (1 - carry) as i16;

        self.set_flag(CARRY, diff >= 0);
        self.set_flag(
            OVERFLOW,
            ((self.a ^ value) & 0x80) != 0 && ((self.a ^ diff as u8) & 0x80) != 0,
        );

        self.a = diff as u8;
        self.update_zero_and_negative_flags(self.a);

        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// INC - Increment Memory
    fn inc(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let value = self.bus.read(addr).wrapping_add(1);
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// DEC - Decrement Memory
    fn dec(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let value = self.bus.read(addr).wrapping_sub(1);
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// INX - Increment X
    fn inx(&mut self, cycles: u8) {
        self.x = self.x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.x);
        self.cycles += cycles as u64;
    }

    /// INY - Increment Y
    fn iny(&mut self, cycles: u8) {
        self.y = self.y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.y);
        self.cycles += cycles as u64;
    }

    /// DEX - Decrement X
    fn dex(&mut self, cycles: u8) {
        self.x = self.x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.x);
        self.cycles += cycles as u64;
    }

    /// DEY - Decrement Y
    fn dey(&mut self, cycles: u8) {
        self.y = self.y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.y);
        self.cycles += cycles as u64;
    }

    /// ASL - Arithmetic Shift Left (Accumulator)
    fn asl_accumulator(&mut self, cycles: u8) {
        self.set_flag(CARRY, (self.a & 0x80) != 0);
        self.a <<= 1;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// ASL - Arithmetic Shift Left (Memory)
    fn asl(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let mut value = self.bus.read(addr);
        self.set_flag(CARRY, (value & 0x80) != 0);
        value <<= 1;
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// LSR - Logical Shift Right (Accumulator)
    fn lsr_accumulator(&mut self, cycles: u8) {
        self.set_flag(CARRY, (self.a & 0x01) != 0);
        self.a >>= 1;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// LSR - Logical Shift Right (Memory)
    fn lsr(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let mut value = self.bus.read(addr);
        self.set_flag(CARRY, (value & 0x01) != 0);
        value >>= 1;
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// ROL - Rotate Left (Accumulator)
    fn rol_accumulator(&mut self, cycles: u8) {
        let old_carry = if self.get_flag(CARRY) { 1 } else { 0 };
        self.set_flag(CARRY, (self.a & 0x80) != 0);
        self.a = (self.a << 1) | old_carry;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// ROL - Rotate Left (Memory)
    fn rol(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let mut value = self.bus.read(addr);
        let old_carry = if self.get_flag(CARRY) { 1 } else { 0 };
        self.set_flag(CARRY, (value & 0x80) != 0);
        value = (value << 1) | old_carry;
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// ROR - Rotate Right (Accumulator)
    fn ror_accumulator(&mut self, cycles: u8) {
        let old_carry = if self.get_flag(CARRY) { 0x80 } else { 0 };
        self.set_flag(CARRY, (self.a & 0x01) != 0);
        self.a = (self.a >> 1) | old_carry;
        self.update_zero_and_negative_flags(self.a);
        self.cycles += cycles as u64;
    }

    /// ROR - Rotate Right (Memory)
    fn ror(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let mut value = self.bus.read(addr);
        let old_carry = if self.get_flag(CARRY) { 0x80 } else { 0 };
        self.set_flag(CARRY, (value & 0x01) != 0);
        value = (value >> 1) | old_carry;
        self.bus.write(addr, value);
        self.update_zero_and_negative_flags(value);
        self.cycles += cycles as u64;
    }

    /// JMP - Jump (Absolute)
    fn jmp_absolute(&mut self, cycles: u8) {
        self.pc = self.read_pc_word();
        self.cycles += cycles as u64;
    }

    /// JMP - Jump (Indirect)
    fn jmp_indirect(&mut self, cycles: u8) {
        let (addr, _) = self.get_operand_address(AddressingMode::Indirect);
        self.pc = addr;
        self.cycles += cycles as u64;
    }

    /// JSR - Jump to Subroutine
    fn jsr(&mut self, cycles: u8) {
        let target = self.read_pc_word();
        // Push return address - 1 (PC points to last byte of JSR instruction + 1)
        self.stack_push_u16(self.pc.wrapping_sub(1));
        self.pc = target;
        self.cycles += cycles as u64;
    }

    /// RTS - Return from Subroutine
    fn rts(&mut self, cycles: u8) {
        self.pc = self.stack_pop_u16().wrapping_add(1);
        self.cycles += cycles as u64;
    }

    /// Branch helper - handles all conditional branches
    fn branch(&mut self, condition: bool, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(AddressingMode::Relative);
        self.cycles += base_cycles as u64;

        if condition {
            self.cycles += 1; // Branch taken adds 1 cycle
            if page_crossed {
                self.cycles += 1; // Page boundary crossed adds another cycle
            }
            self.pc = addr;
        }
    }

    /// BCC - Branch if Carry Clear
    fn bcc(&mut self, cycles: u8) {
        self.branch(!self.get_flag(CARRY), cycles);
    }

    /// BCS - Branch if Carry Set
    fn bcs(&mut self, cycles: u8) {
        self.branch(self.get_flag(CARRY), cycles);
    }

    /// BEQ - Branch if Equal (Zero set)
    fn beq(&mut self, cycles: u8) {
        // Debug: Track BEQ in wait loop
        if self.pc >= 0xFF5A && self.pc <= 0xFF5F {
            let z_flag = self.get_flag(ZERO);
            let will_branch = z_flag;
            static mut BEQ_COUNT: u32 = 0;
            unsafe {
                BEQ_COUNT += 1;
                let count = BEQ_COUNT;
                if count % 1000 == 0 || !will_branch {
                    println!(
                        "  BEQ at ${:04X}: Z={}, A=${:02X}, will_branch={}",
                        self.pc, z_flag, self.a, will_branch
                    );
                }
            }
        }
        self.branch(self.get_flag(ZERO), cycles);
    }

    /// BMI - Branch if Minus (Negative set)
    fn bmi(&mut self, cycles: u8) {
        self.branch(self.get_flag(NEGATIVE), cycles);
    }

    /// BNE - Branch if Not Equal (Zero clear)
    fn bne(&mut self, cycles: u8) {
        self.branch(!self.get_flag(ZERO), cycles);
    }

    /// BPL - Branch if Plus (Negative clear)
    fn bpl(&mut self, cycles: u8) {
        let neg = self.get_flag(NEGATIVE);
        if self.pc >= 0x82DE && self.pc <= 0x82E2 {
            println!(
                "  BPL at ${:04X}: N={}, will_branch={}",
                self.pc.wrapping_sub(1),
                neg,
                !neg
            );
        }
        self.branch(!neg, cycles);
    }

    /// BVC - Branch if Overflow Clear
    fn bvc(&mut self, cycles: u8) {
        self.branch(!self.get_flag(OVERFLOW), cycles);
    }

    /// BVS - Branch if Overflow Set
    fn bvs(&mut self, cycles: u8) {
        self.branch(self.get_flag(OVERFLOW), cycles);
    }

    /// CLC - Clear Carry
    fn clc(&mut self, cycles: u8) {
        self.set_flag(CARRY, false);
        self.cycles += cycles as u64;
    }

    /// CLD - Clear Decimal
    fn cld(&mut self, cycles: u8) {
        self.set_flag(DECIMAL, false);
        self.cycles += cycles as u64;
    }

    /// CLI - Clear Interrupt Disable
    fn cli(&mut self, cycles: u8) {
        self.set_flag(INTERRUPT_DISABLE, false);
        self.cycles += cycles as u64;
    }

    /// CLV - Clear Overflow
    fn clv(&mut self, cycles: u8) {
        self.set_flag(OVERFLOW, false);
        self.cycles += cycles as u64;
    }

    /// SEC - Set Carry
    fn sec(&mut self, cycles: u8) {
        self.set_flag(CARRY, true);
        self.cycles += cycles as u64;
    }

    /// SED - Set Decimal
    fn sed(&mut self, cycles: u8) {
        self.set_flag(DECIMAL, true);
        self.cycles += cycles as u64;
    }

    /// SEI - Set Interrupt Disable
    fn sei(&mut self, cycles: u8) {
        self.set_flag(INTERRUPT_DISABLE, true);
        self.cycles += cycles as u64;
    }

    /// CMP - Compare Accumulator
    fn cmp(&mut self, mode: AddressingMode, base_cycles: u8) {
        let (addr, page_crossed) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let result = self.a.wrapping_sub(value);

        self.set_flag(CARRY, self.a >= value);
        self.update_zero_and_negative_flags(result);

        self.cycles += base_cycles as u64;
        if page_crossed {
            self.cycles += 1;
        }
    }

    /// CPX - Compare X Register
    fn cpx(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let result = self.x.wrapping_sub(value);

        self.set_flag(CARRY, self.x >= value);
        self.update_zero_and_negative_flags(result);

        self.cycles += cycles as u64;
    }

    /// CPY - Compare Y Register
    fn cpy(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let result = self.y.wrapping_sub(value);

        self.set_flag(CARRY, self.y >= value);
        self.update_zero_and_negative_flags(result);

        self.cycles += cycles as u64;
    }

    /// BIT - Bit Test
    fn bit(&mut self, mode: AddressingMode, cycles: u8) {
        let (addr, _) = self.get_operand_address(mode);
        let value = self.bus.read(addr);
        let result = self.a & value;

        // Debug: watch BIT on PPUSTATUS to confirm VBlank visibility
        if addr == 0x2002 {
            static mut BIT_READ_COUNT: u32 = 0;
            unsafe {
                BIT_READ_COUNT += 1;
                let count = BIT_READ_COUNT;
                if value & 0x80 != 0 {
                    println!(
                        "  BIT $2002 saw VBlank (read #{}, value=${:02X}, P before=${:02X})",
                        count, value, self.status
                    );
                } else if count <= 64 {
                    println!("  BIT $2002 read #{} value=${:02X}", count, value);
                } else if count % 5000 == 0 {
                    println!("  BIT $2002 still zero after {} reads", count);
                }
            }
        }

        self.set_flag(ZERO, result == 0);
        self.set_flag(OVERFLOW, (value & 0x40) != 0);
        self.set_flag(NEGATIVE, (value & 0x80) != 0);

        if addr == 0x2002 {
            println!(
                "    BIT $2002 flags after: P=${:02X} (N={},V={},Z={})",
                self.status,
                self.get_flag(NEGATIVE),
                self.get_flag(OVERFLOW),
                self.get_flag(ZERO)
            );
        }

        self.cycles += cycles as u64;
    }

    /// BRK - Force Interrupt
    fn brk(&mut self, cycles: u8) {
        // Push PC + 2 (skip the padding byte after BRK)
        self.pc = self.pc.wrapping_add(1);
        self.stack_push_u16(self.pc);

        // Push status with B flag set
        self.stack_push(self.status | BREAK | UNUSED);

        // Set interrupt disable flag
        self.set_flag(INTERRUPT_DISABLE, true);

        // Load PC from IRQ vector
        self.pc = self.bus.read_u16(0xFFFE);

        self.cycles += cycles as u64;
    }

    /// RTI - Return from Interrupt
    fn rti(&mut self, cycles: u8) {
        self.status = self.stack_pop();
        self.set_flag(BREAK, false);
        self.set_flag(UNUSED, true);
        self.pc = self.stack_pop_u16();

        self.cycles += cycles as u64;
    }

    /// NOP - No Operation
    fn nop(&mut self, cycles: u8) {
        self.cycles += cycles as u64;
    }
}

/// Addressing modes for 6502 instructions.
///
/// These determine how the CPU calculates the effective address for an operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // Some variants used only in pattern matching
enum AddressingMode {
    Immediate,   // #$nn - value is next byte
    ZeroPage,    // $nn - address in zero page ($00-$FF)
    ZeroPageX,   // $nn,X - zero page address + X
    ZeroPageY,   // $nn,Y - zero page address + Y
    Absolute,    // $nnnn - 16-bit address
    AbsoluteX,   // $nnnn,X - 16-bit address + X
    AbsoluteY,   // $nnnn,Y - 16-bit address + Y
    Indirect,    // ($nnnn) - address at 16-bit pointer
    IndirectX,   // ($nn,X) - pointer in zero page + X
    IndirectY,   // ($nn),Y - pointer in zero page, then + Y
    Relative,    // Label - PC-relative offset for branches
    Implied,     // No operand
    Accumulator, // Operates on accumulator
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_cpu() -> Cpu {
        let bus = Bus::new();
        Cpu::new(bus)
    }

    #[test]
    fn test_reset() {
        let mut cpu = create_test_cpu();
        cpu.a = 0xFF;
        cpu.x = 0xFF;
        cpu.y = 0xFF;

        cpu.reset();

        assert_eq!(cpu.a, 0);
        assert_eq!(cpu.x, 0);
        assert_eq!(cpu.y, 0);
        assert_eq!(cpu.sp, 0xFD);
        assert!(cpu.get_flag(INTERRUPT_DISABLE));
        assert!(cpu.get_flag(UNUSED));
    }

    #[test]
    fn test_lda_immediate() {
        let mut cpu = create_test_cpu();
        cpu.bus.write(0x0000, 0xA9); // LDA #$42
        cpu.bus.write(0x0001, 0x42);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.a, 0x42);
        assert!(!cpu.get_flag(ZERO));
        assert!(!cpu.get_flag(NEGATIVE));
    }

    #[test]
    fn test_lda_zero_flag() {
        let mut cpu = create_test_cpu();
        cpu.bus.write(0x0000, 0xA9); // LDA #$00
        cpu.bus.write(0x0001, 0x00);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert!(cpu.get_flag(ZERO));
        assert!(!cpu.get_flag(NEGATIVE));
    }

    #[test]
    fn test_lda_negative_flag() {
        let mut cpu = create_test_cpu();
        cpu.bus.write(0x0000, 0xA9); // LDA #$FF
        cpu.bus.write(0x0001, 0xFF);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.a, 0xFF);
        assert!(!cpu.get_flag(ZERO));
        assert!(cpu.get_flag(NEGATIVE));
    }

    #[test]
    fn test_tax() {
        let mut cpu = create_test_cpu();
        cpu.a = 0x42;
        cpu.bus.write(0x0000, 0xAA); // TAX
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_inx() {
        let mut cpu = create_test_cpu();
        cpu.x = 0x41;
        cpu.bus.write(0x0000, 0xE8); // INX
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.x, 0x42);
        assert!(!cpu.get_flag(ZERO));
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = create_test_cpu();
        cpu.x = 0xFF;
        cpu.bus.write(0x0000, 0xE8); // INX
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.x, 0x00);
        assert!(cpu.get_flag(ZERO));
    }

    #[test]
    fn test_sta_zero_page() {
        let mut cpu = create_test_cpu();
        cpu.a = 0x42;
        cpu.bus.write(0x0000, 0x85); // STA $10
        cpu.bus.write(0x0001, 0x10);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.bus.read(0x10), 0x42);
    }

    #[test]
    fn test_adc_no_carry() {
        let mut cpu = create_test_cpu();
        cpu.a = 0x10;
        cpu.bus.write(0x0000, 0x69); // ADC #$20
        cpu.bus.write(0x0001, 0x20);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.a, 0x30);
        assert!(!cpu.get_flag(CARRY));
        assert!(!cpu.get_flag(ZERO));
        assert!(!cpu.get_flag(NEGATIVE));
    }

    #[test]
    fn test_adc_with_carry() {
        let mut cpu = create_test_cpu();
        cpu.a = 0xFF;
        cpu.bus.write(0x0000, 0x69); // ADC #$01
        cpu.bus.write(0x0001, 0x01);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert!(cpu.get_flag(CARRY));
        assert!(cpu.get_flag(ZERO));
    }

    #[test]
    fn test_jmp_absolute() {
        let mut cpu = create_test_cpu();
        cpu.bus.write(0x0000, 0x4C); // JMP $1234
        cpu.bus.write(0x0001, 0x34);
        cpu.bus.write(0x0002, 0x12);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.pc, 0x1234);
    }

    #[test]
    fn test_beq_not_taken() {
        let mut cpu = create_test_cpu();
        cpu.set_flag(ZERO, false);
        cpu.bus.write(0x0000, 0xF0); // BEQ +5
        cpu.bus.write(0x0001, 0x05);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.pc, 0x0002); // Branch not taken
    }

    #[test]
    fn test_beq_taken() {
        let mut cpu = create_test_cpu();
        cpu.set_flag(ZERO, true);
        cpu.bus.write(0x0000, 0xF0); // BEQ +5
        cpu.bus.write(0x0001, 0x05);
        cpu.pc = 0x0000;

        cpu.step();

        assert_eq!(cpu.pc, 0x0007); // 0x0002 + 0x05
    }

    #[test]
    fn test_stack_operations() {
        let mut cpu = create_test_cpu();
        cpu.a = 0x42;

        // Push A
        cpu.stack_push(cpu.a);
        assert_eq!(cpu.sp, 0xFC);

        // Pop into different variable
        let value = cpu.stack_pop();
        assert_eq!(value, 0x42);
        assert_eq!(cpu.sp, 0xFD);
    }
}
