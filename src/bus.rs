//! NES CPU memory bus implementation.
//!
//! The bus connects the CPU to all other components (RAM, PPU, APU, cartridge, controllers).
//! It handles memory-mapped I/O and address mirroring according to the NES memory map.
//!
//! # NES CPU Memory Map
//!
//! ```text
//! $0000-$07FF: 2KB internal RAM
//! $0800-$1FFF: Mirrors of $0000-$07FF (RAM mirrored 3 times)
//! $2000-$2007: PPU registers
//! $2008-$3FFF: Mirrors of $2000-$2007 (PPU registers repeated)
//! $4000-$4017: APU and I/O registers
//! $4018-$401F: APU and I/O functionality (usually disabled)
//! $4020-$FFFF: Cartridge space (PRG ROM/RAM, mapper registers)
//! ```

use crate::cartridge::{Cartridge, Mirroring};
use crate::ppu::Ppu;

/// Size of the NES CPU's internal RAM in bytes (2KB)
const RAM_SIZE: usize = 2048;

/// MMC1 mapper state (used by many early NES games, e.g., Zelda).
struct Mmc1 {
    shift_reg: u8,
    write_count: u8,
    control: u8,
    chr_bank0: u8,
    chr_bank1: u8,
    prg_bank: u8,
}

impl Mmc1 {
    fn new() -> Self {
        // Control resets to 0x0C (16KB PRG mode, low CHR bank)
        Mmc1 {
            shift_reg: 0x10,
            write_count: 0,
            control: 0x0C,
            chr_bank0: 0,
            chr_bank1: 0,
            prg_bank: 0,
        }
    }
}

/// NES CPU memory bus.
///
/// The bus owns all the components and handles memory-mapped I/O.
/// In Python terms, this is like a class that acts as a central hub,
/// routing read/write requests to the appropriate component based on the address.
pub struct Bus {
    /// 2KB of internal CPU RAM ($0000-$07FF)
    /// This is mirrored across $0800-$1FFF
    ram: [u8; RAM_SIZE],

    /// 8KB of PRG RAM ($6000-$7FFF)
    prg_ram: [u8; 8192],

    /// The loaded game cartridge
    /// Using Option because the bus can be created without a cartridge for testing
    cartridge: Option<Cartridge>,

    /// Picture Processing Unit (graphics)
    pub ppu: Ppu,

    /// MMC1 mapper state (if cartridge uses mapper 1)
    mmc1: Option<Mmc1>,
    // TODO: Add APU when implemented
    // apu: Apu,

    // TODO: Add controller input when implemented
    // controller1: Controller,
    // controller2: Controller,
}

impl Bus {
    /// Create a new bus without a cartridge.
    ///
    /// This is useful for testing. Use `with_cartridge()` to create a bus
    /// with a loaded game.
    ///
    /// # Examples
    ///
    /// ```
    /// use rustnes::bus::Bus;
    ///
    /// let bus = Bus::new();
    /// ```
    pub fn new() -> Self {
        let ppu = Ppu::new(vec![0; 8192], Mirroring::Horizontal);
        Bus {
            ram: [0; RAM_SIZE],
            prg_ram: [0; 8192],
            cartridge: None,
            ppu,
            mmc1: None,
        }
    }

    /// Create a new bus with a loaded cartridge.
    ///
    /// # Arguments
    ///
    /// * `cartridge` - The loaded game cartridge
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use rustnes::bus::Bus;
    /// use rustnes::cartridge::Cartridge;
    ///
    /// let cart = Cartridge::load("game.nes").unwrap();
    /// let bus = Bus::with_cartridge(cart);
    /// ```
    pub fn with_cartridge(cartridge: Cartridge) -> Self {
        let chr_rom = cartridge.chr_rom.clone();
        let mirroring = cartridge.mirroring;
        let ppu = Ppu::new(chr_rom, mirroring);
        let uses_mmc1 = cartridge_uses_mmc1(&cartridge);

        Bus {
            ram: [0; RAM_SIZE],
            prg_ram: [0; 8192],
            cartridge: Some(cartridge),
            ppu,
            mmc1: if uses_mmc1 { Some(Mmc1::new()) } else { None },
        }
    }

    /// Read a byte from the bus at the specified address.
    ///
    /// This method handles all memory mapping and mirroring according to the NES memory map.
    ///
    /// # Arguments
    ///
    /// * `address` - The 16-bit memory address to read from
    ///
    /// # Returns
    ///
    /// The byte value at the specified address
    ///
    /// # Examples
    ///
    /// ```
    /// use rustnes::bus::Bus;
    ///
    /// let mut bus = Bus::new();
    /// let value = bus.read(0x0000);
    /// ```
    pub fn read(&mut self, address: u16) -> u8 {
        match address {
            // RAM and its mirrors ($0000-$1FFF)
            // The NES has 2KB of RAM, mirrored 4 times
            // In Python: value = self.ram[address & 0x07FF]
            0x0000..=0x1FFF => {
                let mirror_address = (address & 0x07FF) as usize;
                self.ram[mirror_address]
            }

            // PPU registers and mirrors ($2000-$3FFF)
            // 8 PPU registers mirrored every 8 bytes
            0x2000..=0x3FFF => {
                let ppu_reg = address & 0x0007; // Mirror every 8 bytes
                self.ppu.read_register(ppu_reg)
            }

            // APU and I/O registers ($4000-$4017)
            0x4000..=0x4013 | 0x4015 => {
                // TODO: Implement APU and I/O reads
                0
            }

            // OAMDMA ($4014) is write-only; reads typically return open bus (0 here)
            0x4014 => 0,

            // Controller ports ($4016-$4017)
            0x4016..=0x4017 => {
                // TODO: Implement controller shift registers
                0
            }

            // APU and I/O functionality that is normally disabled ($4018-$401F)
            0x4018..=0x401F => {
                // Typically unused
                0
            }

            // Cartridge space ($4020-$FFFF)
            // This is where PRG ROM is mapped
            0x4020..=0x5FFF => {
                // Expansion ROM / APU registers would be here
                0
            }

            // Cartridge PRG RAM ($6000-$7FFF)
            0x6000..=0x7FFF => {
                let offset = (address - 0x6000) as usize;
                self.prg_ram[offset]
            }

            0x8000..=0xFFFF => {
                if let Some(ref cartridge) = self.cartridge {
                    self.read_prg_rom(address, cartridge)
                } else {
                    // No cartridge loaded
                    0
                }
            }
        }
    }

    /// Write a byte to the bus at the specified address.
    ///
    /// This method handles all memory mapping and mirroring according to the NES memory map.
    /// Note that some addresses are read-only (like ROM) and writes to them may trigger
    /// mapper operations instead.
    ///
    /// # Arguments
    ///
    /// * `address` - The 16-bit memory address to write to
    /// * `value` - The byte value to write
    ///
    /// # Examples
    ///
    /// ```
    /// use rustnes::bus::Bus;
    ///
    /// let mut bus = Bus::new();
    /// bus.write(0x0000, 0x42);
    /// assert_eq!(bus.read(0x0000), 0x42);
    /// ```
    pub fn write(&mut self, address: u16, value: u8) {
        match address {
            // RAM and its mirrors ($0000-$1FFF)
            0x0000..=0x1FFF => {
                let mirror_address = (address & 0x07FF) as usize;
                self.ram[mirror_address] = value;
            }

            // PPU registers and mirrors ($2000-$3FFF)
            0x2000..=0x3FFF => {
                let ppu_reg = address & 0x0007; // Mirror every 8 bytes
                self.ppu.write_register(ppu_reg, value);
            }

            // APU and I/O registers ($4000-$4017)
            0x4000..=0x4013 | 0x4015 => {
                // TODO: Implement APU and I/O writes
            }

            // OAMDMA - sprite DMA ($4014)
            0x4014 => {
                // The written value specifies the high byte of the source address.
                let base = (value as u16) << 8;
                let mut buffer = [0u8; 256];
                for i in 0..256 {
                    buffer[i] = self.read(base.wrapping_add(i as u16));
                }
                self.ppu.oam_dma(&buffer);
                // Proper CPU stall cycles (513 or 514) are not yet modeled; this is functional only.
            }

            // Controller ports ($4016-$4017)
            0x4016..=0x4017 => {
                // TODO: Implement controller latch/shift behaviour
            }

            // APU and I/O functionality that is normally disabled ($4018-$401F)
            0x4018..=0x401F => {
                // Typically unused
            }

            // Expansion ROM / mapper registers ($4020-$5FFF)
            0x4020..=0x5FFF => {
                // TODO: Implement mapper-specific registers
            }

            // Cartridge PRG RAM ($6000-$7FFF)
            0x6000..=0x7FFF => {
                let offset = (address - 0x6000) as usize;
                self.prg_ram[offset] = value;
            }

            // Cartridge space ($4020-$FFFF)
            0x8000..=0xFFFF => {
                // Mapper writes (e.g., MMC1 bank switching)
                if let Some(ref cartridge) = self.cartridge {
                    match cartridge.mapper {
                        1 => self.mmc1_write(address, value),
                        _ => {
                            // Other mappers not yet implemented
                        }
                    }
                }
            }
        }
    }

    /// Read from PRG ROM with proper mapper handling.
    ///
    /// For mapper 0 (NROM), the memory layout is:
    /// - 16KB ROM: Mirrored at $8000-$BFFF and $C000-$FFFF
    /// - 32KB ROM: Mapped linearly at $8000-$FFFF
    ///
    /// For mapper 1 (MMC1), the power-up state is:
    /// - $8000-$BFFF: First 16KB bank (bank 0)
    /// - $C000-$FFFF: Last 16KB bank (fixed)
    ///
    /// # Arguments
    ///
    /// * `address` - The CPU address to read from
    /// * `cartridge` - Reference to the cartridge
    ///
    /// # Returns
    ///
    /// The byte value from PRG ROM
    fn read_prg_rom(&self, address: u16, cartridge: &Cartridge) -> u8 {
        // Most games use $8000-$FFFF for PRG ROM
        // Adjust address to be relative to $8000
        let rom_address = (address - 0x8000) as usize;

        match cartridge.mapper {
            // Mapper 0 (NROM)
            0 => {
                let prg_rom_size = cartridge.prg_rom.len();

                if prg_rom_size == 16384 {
                    // 16KB ROM: mirror it (both $8000-$BFFF and $C000-$FFFF map to same ROM)
                    // In Python: index = rom_address % 16384
                    cartridge.prg_rom[rom_address % 16384]
                } else {
                    // 32KB or larger ROM: direct mapping
                    if rom_address < prg_rom_size {
                        cartridge.prg_rom[rom_address]
                    } else {
                        0
                    }
                }
            }

            // Mapper 1 (MMC1)
            1 => self.read_prg_mmc1(address, cartridge),

            // Other mappers - TODO: implement as needed
            _ => {
                // For now, just do simple linear mapping
                if rom_address < cartridge.prg_rom.len() {
                    cartridge.prg_rom[rom_address]
                } else {
                    0
                }
            }
        }
    }

    fn read_prg_mmc1(&self, address: u16, cartridge: &Cartridge) -> u8 {
        let rom_address = (address - 0x8000) as usize;
        let prg_rom_size = cartridge.prg_rom.len();
        let bank_size = 0x4000; // 16KB
        let bank_count = prg_rom_size / bank_size;

        let Some(ref mmc1) = self.mmc1 else {
            // Fall back to power-on mapping: bank0 then last bank
            let index = if address < 0xC000 {
                rom_address
            } else {
                let last_bank_start = prg_rom_size.saturating_sub(bank_size);
                last_bank_start + (rom_address - bank_size)
            };
            return cartridge.prg_rom.get(index).copied().unwrap_or(0);
        };

        let prg_mode = (mmc1.control >> 2) & 0x03;

        let index = match prg_mode {
            // 32KB switching ($8000-$FFFF)
            0 | 1 => {
                let bank = ((mmc1.prg_bank & 0x0E) as usize) % bank_count.max(1);
                let base = bank * bank_size;
                base + rom_address.min(bank_size * 2 - 1)
            }
            // Fix first bank at $8000, switch $C000
            2 => {
                if address < 0xC000 {
                    rom_address
                } else {
                    let bank = (mmc1.prg_bank as usize) % bank_count.max(1);
                    let base = bank * bank_size;
                    base + (rom_address - bank_size)
                }
            }
            // Fix last bank at $C000, switch $8000 (power-on default)
            3 | _ => {
                if address < 0xC000 {
                    let bank = (mmc1.prg_bank as usize) % bank_count.max(1);
                    let base = bank * bank_size;
                    base + rom_address
                } else {
                    let last_bank_start = prg_rom_size.saturating_sub(bank_size);
                    last_bank_start + (rom_address - bank_size)
                }
            }
        };

        cartridge.prg_rom.get(index).copied().unwrap_or(0)
    }

    /// Read a 16-bit word from the bus (little-endian).
    ///
    /// The 6502 CPU is little-endian, meaning the low byte comes first.
    /// This is a convenience method for reading 16-bit values like addresses.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to read from (reads address and address+1)
    ///
    /// # Returns
    ///
    /// The 16-bit value in native byte order
    ///
    /// # Examples
    ///
    /// ```
    /// use rustnes::bus::Bus;
    ///
    /// let mut bus = Bus::new();
    /// bus.write(0x0000, 0x34);  // Low byte
    /// bus.write(0x0001, 0x12);  // High byte
    /// assert_eq!(bus.read_u16(0x0000), 0x1234);
    /// ```
    pub fn read_u16(&mut self, address: u16) -> u16 {
        let low = self.read(address) as u16;
        let high = self.read(address.wrapping_add(1)) as u16;
        (high << 8) | low
    }

    /// Write a 16-bit word to the bus (little-endian).
    ///
    /// # Arguments
    ///
    /// * `address` - The address to write to (writes to address and address+1)
    /// * `value` - The 16-bit value to write
    ///
    /// # Examples
    ///
    /// ```
    /// use rustnes::bus::Bus;
    ///
    /// let mut bus = Bus::new();
    /// bus.write_u16(0x0000, 0x1234);
    /// assert_eq!(bus.read(0x0000), 0x34);  // Low byte
    /// assert_eq!(bus.read(0x0001), 0x12);  // High byte
    /// ```
    #[allow(dead_code)] // Will be used by CPU for stack operations and other features
    pub fn write_u16(&mut self, address: u16, value: u16) {
        let low = (value & 0xFF) as u8;
        let high = (value >> 8) as u8;
        self.write(address, low);
        self.write(address.wrapping_add(1), high);
    }

    fn mmc1_write(&mut self, address: u16, value: u8) {
        let Some(ref mut mmc1) = self.mmc1 else {
            return;
        };

        // Reset shift register on bit 7 set
        if (value & 0x80) != 0 {
            mmc1.shift_reg = 0x10;
            mmc1.write_count = 0;
            mmc1.control |= 0x0C; // Force PRG mode to known state
            self.apply_mmc1_banks();
            return;
        }

        // Accumulate 5 writes LSB first
        mmc1.shift_reg = (mmc1.shift_reg >> 1) | ((value & 0x01) << 4);
        mmc1.write_count += 1;

        if mmc1.write_count == 5 {
            let data = mmc1.shift_reg;
            let target = address & 0xE000;

            match target {
                0x8000 => mmc1.control = data,
                0xA000 => mmc1.chr_bank0 = data,
                0xC000 => mmc1.chr_bank1 = data,
                0xE000 => mmc1.prg_bank = data,
                _ => {}
            }

            mmc1.shift_reg = 0x10;
            mmc1.write_count = 0;
            self.apply_mmc1_banks();
        }
    }

    fn apply_mmc1_banks(&mut self) {
        let Some(ref cartridge) = self.cartridge else {
            return;
        };
        let Some(ref mmc1) = self.mmc1 else {
            return;
        };

        // Update mirroring based on control bits 0-1
        let mirroring = match mmc1.control & 0x03 {
            0 => Mirroring::OneScreenLower,
            1 => Mirroring::OneScreenUpper,
            2 => Mirroring::Vertical,
            _ => Mirroring::Horizontal,
        };
        self.ppu.set_mirroring(mirroring);

        // Update CHR bank offsets
        let chr_mode_4k = (mmc1.control & 0x10) != 0;
        let chr_bank_count = (cartridge.chr_rom.len() / 0x1000).max(1);

        let (bank0_offset, bank1_offset) = if chr_mode_4k {
            let bank0 = (mmc1.chr_bank0 as usize) % chr_bank_count;
            let bank1 = (mmc1.chr_bank1 as usize) % chr_bank_count;
            (bank0 * 0x1000, bank1 * 0x1000)
        } else {
            // 8KB mode: ignore bit 0
            let bank = ((mmc1.chr_bank0 & 0x1E) as usize) % chr_bank_count;
            let base = bank * 0x1000;
            (base, base + 0x1000)
        };
        self.ppu
            .set_chr_banks(chr_mode_4k, bank0_offset, bank1_offset);
    }
}

fn cartridge_uses_mmc1(cartridge: &Cartridge) -> bool {
    cartridge.mapper == 1
}

// Implementing Default trait for convenience
// In Python, this is like defining __init__ with no required arguments
impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ram_read_write() {
        let mut bus = Bus::new();

        // Write and read from RAM
        bus.write(0x0000, 0x42);
        assert_eq!(bus.read(0x0000), 0x42);

        bus.write(0x07FF, 0xFF);
        assert_eq!(bus.read(0x07FF), 0xFF);
    }

    #[test]
    fn test_ram_mirroring() {
        let mut bus = Bus::new();

        // Write to RAM base address
        bus.write(0x0000, 0xAA);

        // Read from mirrors - should return the same value
        assert_eq!(bus.read(0x0000), 0xAA); // Base
        assert_eq!(bus.read(0x0800), 0xAA); // Mirror 1
        assert_eq!(bus.read(0x1000), 0xAA); // Mirror 2
        assert_eq!(bus.read(0x1800), 0xAA); // Mirror 3

        // Write to a mirror
        bus.write(0x0800, 0x55);

        // Read from base - should reflect the change
        assert_eq!(bus.read(0x0000), 0x55);
        assert_eq!(bus.read(0x1000), 0x55);
    }

    #[test]
    fn test_ram_mirroring_all_addresses() {
        let mut bus = Bus::new();

        // Test that 0x0123 mirrors to 0x0923, 0x1123, 0x1923
        bus.write(0x0123, 0x77);
        assert_eq!(bus.read(0x0923), 0x77);
        assert_eq!(bus.read(0x1123), 0x77);
        assert_eq!(bus.read(0x1923), 0x77);
    }

    #[test]
    fn test_read_write_u16() {
        let mut bus = Bus::new();

        // Write 16-bit value (little-endian)
        bus.write_u16(0x0000, 0x1234);

        // Verify individual bytes
        assert_eq!(bus.read(0x0000), 0x34); // Low byte
        assert_eq!(bus.read(0x0001), 0x12); // High byte

        // Read back as 16-bit
        assert_eq!(bus.read_u16(0x0000), 0x1234);
    }

    #[test]
    fn test_read_u16_wrapping() {
        let mut bus = Bus::new();

        // Test wrapping within RAM region
        // Write to end of RAM and wrap to beginning
        bus.write(0x07FF, 0x34);
        bus.write(0x0800, 0x12); // This wraps to 0x0000 due to mirroring

        let value = bus.read_u16(0x07FF);
        assert_eq!(value, 0x1234);

        // Also verify the actual wrapping with wrapping_add
        bus.write(0x0100, 0xAA);
        bus.write(0x0101, 0xBB);
        assert_eq!(bus.read_u16(0x0100), 0xBBAA);
    }

    #[test]
    fn test_cartridge_read_16kb() {
        // Create a minimal 16KB cartridge (mapper 0)
        let mut cart = Cartridge {
            prg_rom: vec![0; 16384],
            chr_rom: vec![0; 8192],
            mapper: 0,
            mirroring: crate::cartridge::Mirroring::Horizontal,
            has_battery: false,
            has_trainer: false,
        };

        // Write test pattern to ROM
        cart.prg_rom[0] = 0xAA;
        cart.prg_rom[0x3FFF] = 0xBB;

        let mut bus = Bus::with_cartridge(cart);

        // Read from $8000 (start of PRG ROM)
        assert_eq!(bus.read(0x8000), 0xAA);

        // Read from $BFFF (end of first 16KB)
        assert_eq!(bus.read(0xBFFF), 0xBB);

        // Read from $C000 (should mirror to $8000 for 16KB ROM)
        assert_eq!(bus.read(0xC000), 0xAA);

        // Read from $FFFF (should mirror to $BFFF)
        assert_eq!(bus.read(0xFFFF), 0xBB);
    }

    #[test]
    fn test_cartridge_read_32kb() {
        // Create a 32KB cartridge (mapper 0)
        let mut cart = Cartridge {
            prg_rom: vec![0; 32768],
            chr_rom: vec![0; 8192],
            mapper: 0,
            mirroring: crate::cartridge::Mirroring::Horizontal,
            has_battery: false,
            has_trainer: false,
        };

        // Write test pattern to ROM
        cart.prg_rom[0] = 0xAA;
        cart.prg_rom[0x3FFF] = 0xBB;
        cart.prg_rom[0x4000] = 0xCC;
        cart.prg_rom[0x7FFF] = 0xDD;

        let mut bus = Bus::with_cartridge(cart);

        // Read from both 16KB banks
        assert_eq!(bus.read(0x8000), 0xAA);
        assert_eq!(bus.read(0xBFFF), 0xBB);
        assert_eq!(bus.read(0xC000), 0xCC);
        assert_eq!(bus.read(0xFFFF), 0xDD);
    }

    #[test]
    fn test_no_cartridge() {
        let mut bus = Bus::new();

        // Reading from cartridge space without a cartridge should return 0
        assert_eq!(bus.read(0x8000), 0);
        assert_eq!(bus.read(0xFFFF), 0);
    }
}
