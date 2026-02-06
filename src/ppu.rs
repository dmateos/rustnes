//! NES Picture Processing Unit (PPU) emulation.
//!
//! The PPU is responsible for generating the video output. It has its own memory bus
//! separate from the CPU, and communicates with the CPU through memory-mapped registers.
//!
//! # PPU Memory Map
//!
//! ```text
//! $0000-$0FFF: Pattern Table 0 (CHR ROM)
//! $1000-$1FFF: Pattern Table 1 (CHR ROM)
//! $2000-$23FF: Nametable 0
//! $2400-$27FF: Nametable 1
//! $2800-$2BFF: Nametable 2
//! $2C00-$2FFF: Nametable 3
//! $3000-$3EFF: Mirrors of $2000-$2EFF
//! $3F00-$3F1F: Palette RAM
//! $3F20-$3FFF: Mirrors of $3F00-$3F1F
//! ```
//!
//! # CPU-visible PPU Registers
//!
//! ```text
//! $2000: PPUCTRL   - PPU control register
//! $2001: PPUMASK   - PPU mask register
//! $2002: PPUSTATUS - PPU status register
//! $2003: OAMADDR   - OAM address port
//! $2004: OAMDATA   - OAM data port
//! $2005: PPUSCROLL - PPU scrolling position
//! $2006: PPUADDR   - PPU address port
//! $2007: PPUDATA   - PPU data port
//! ```

use crate::cartridge::Mirroring;

/// Size of nametable memory (2KB)
const NAMETABLE_SIZE: usize = 2048;

/// Size of palette RAM (32 bytes)
const PALETTE_SIZE: usize = 32;

/// Size of OAM (Object Attribute Memory) for sprites (256 bytes)
const OAM_SIZE: usize = 256;

/// NES screen dimensions
const SCREEN_WIDTH: usize = 256;
const SCREEN_HEIGHT: usize = 240;
const FRAMEBUFFER_SIZE: usize = SCREEN_WIDTH * SCREEN_HEIGHT;

/// PPUCTRL register flags
#[allow(dead_code)] // Will be used when implementing rendering
const PPUCTRL_NAMETABLE_X: u8 = 0b0000_0001;
#[allow(dead_code)]
const PPUCTRL_NAMETABLE_Y: u8 = 0b0000_0010;
const PPUCTRL_VRAM_INCREMENT: u8 = 0b0000_0100;
#[allow(dead_code)]
const PPUCTRL_SPRITE_PATTERN: u8 = 0b0000_1000;
#[allow(dead_code)]
const PPUCTRL_BACKGROUND_PATTERN: u8 = 0b0001_0000;
#[allow(dead_code)]
const PPUCTRL_SPRITE_SIZE: u8 = 0b0010_0000;
#[allow(dead_code)]
const PPUCTRL_MASTER_SLAVE: u8 = 0b0100_0000;
#[allow(dead_code)] // Used in step() method
const PPUCTRL_NMI_ENABLE: u8 = 0b1000_0000;

/// PPUMASK register flags
#[allow(dead_code)] // Will be used when implementing rendering
const PPUMASK_GREYSCALE: u8 = 0b0000_0001;
#[allow(dead_code)]
const PPUMASK_SHOW_BACKGROUND_LEFT: u8 = 0b0000_0010;
#[allow(dead_code)]
const PPUMASK_SHOW_SPRITES_LEFT: u8 = 0b0000_0100;
const PPUMASK_SHOW_BACKGROUND: u8 = 0b0000_1000;
const PPUMASK_SHOW_SPRITES: u8 = 0b0001_0000;
#[allow(dead_code)]
const PPUMASK_EMPHASIZE_RED: u8 = 0b0010_0000;
#[allow(dead_code)]
const PPUMASK_EMPHASIZE_GREEN: u8 = 0b0100_0000;
#[allow(dead_code)]
const PPUMASK_EMPHASIZE_BLUE: u8 = 0b1000_0000;

/// PPUSTATUS register flags
#[allow(dead_code)] // Will be used when implementing sprite rendering
const PPUSTATUS_SPRITE_OVERFLOW: u8 = 0b0010_0000;
#[allow(dead_code)] // Will be used when implementing sprite rendering
const PPUSTATUS_SPRITE_ZERO_HIT: u8 = 0b0100_0000;
const PPUSTATUS_VBLANK: u8 = 0b1000_0000;

/// NES system color palette (64 colors)
/// Each color is represented as (R, G, B)
const SYSTEM_PALETTE: [(u8, u8, u8); 64] = [
    (84, 84, 84),
    (0, 30, 116),
    (8, 16, 144),
    (48, 0, 136),
    (68, 0, 100),
    (92, 0, 48),
    (84, 4, 0),
    (60, 24, 0),
    (32, 42, 0),
    (8, 58, 0),
    (0, 64, 0),
    (0, 60, 0),
    (0, 50, 60),
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
    (152, 150, 152),
    (8, 76, 196),
    (48, 50, 236),
    (92, 30, 228),
    (136, 20, 176),
    (160, 20, 100),
    (152, 34, 32),
    (120, 60, 0),
    (84, 90, 0),
    (40, 114, 0),
    (8, 124, 0),
    (0, 118, 40),
    (0, 102, 120),
    (0, 0, 0),
    (0, 0, 0),
    (0, 0, 0),
    (236, 238, 236),
    (76, 154, 236),
    (120, 124, 236),
    (176, 98, 236),
    (228, 84, 236),
    (236, 88, 180),
    (236, 106, 100),
    (212, 136, 32),
    (160, 170, 0),
    (116, 196, 0),
    (76, 208, 32),
    (56, 204, 108),
    (56, 180, 204),
    (60, 60, 60),
    (0, 0, 0),
    (0, 0, 0),
    (236, 238, 236),
    (168, 204, 236),
    (188, 188, 236),
    (212, 178, 236),
    (236, 174, 236),
    (236, 174, 212),
    (236, 180, 176),
    (228, 196, 144),
    (204, 210, 120),
    (180, 222, 120),
    (168, 226, 144),
    (152, 226, 180),
    (160, 214, 228),
    (160, 162, 160),
    (0, 0, 0),
    (0, 0, 0),
];

/// Sprite data for rendering.
struct SpriteData {
    x: u8,
    y: u8,
    tile_index: u8,
    palette: u8,
    priority: bool, // false = front, true = behind background
    flip_h: bool,
    flip_v: bool,
    sprite_0: bool, // Is this sprite 0?
}

/// NES Picture Processing Unit.
///
/// The PPU generates the video signal and handles all graphics rendering.
/// It has separate memory from the CPU and is accessed through memory-mapped registers.
pub struct Ppu {
    // PPU Registers (CPU-accessible via $2000-$2007)
    /// $2000: PPUCTRL - Controls NMI, sprite size, pattern tables, etc.
    ctrl: u8,

    /// $2001: PPUMASK - Controls rendering (background, sprites, color)
    mask: u8,

    /// $2002: PPUSTATUS - Read-only status (VBlank, sprite 0 hit, sprite overflow)
    status: u8,

    /// $2003: OAMADDR - OAM address for read/write
    oam_addr: u8,

    // Internal PPU Memory
    /// Pattern tables - stored in CHR ROM (accessed via cartridge)
    /// Not stored here, accessed through cartridge reference

    /// Nametables - 2KB internal VRAM (can be extended by cartridge)
    /// Stores which tiles to display where
    vram: [u8; NAMETABLE_SIZE],

    /// Palette RAM - 32 bytes for background and sprite colors
    palette: [u8; PALETTE_SIZE],

    /// OAM (Object Attribute Memory) - 256 bytes for sprite data
    /// Each sprite uses 4 bytes: Y position, tile index, attributes, X position
    oam: [u8; OAM_SIZE],

    // Internal Registers and State
    /// Current VRAM address (15-bit)
    vram_addr: u16,

    /// Temporary VRAM address (15-bit) - used during rendering
    temp_vram_addr: u16,

    /// Fine X scroll (3-bit)
    fine_x: u8,

    /// Write toggle for $2005 and $2006 (first write vs second write)
    /// In Python, you might use a boolean like `first_write = True`
    write_latch: bool,

    /// Data buffer for reading PPUDATA ($2007)
    /// Reading PPUDATA is delayed by one read
    data_buffer: u8,

    /// Current scanline (0-261, with 261 being pre-render)
    scanline: u16,

    /// Current cycle within the scanline (0-340)
    cycle: u16,

    /// Current frame number
    frame: u64,

    /// Whether NMI should be triggered this frame
    #[allow(dead_code)] // Used by poll_nmi() method
    nmi_pending: bool,

    /// Mirroring mode (from cartridge)
    mirroring: Mirroring,

    /// CHR bank select for mapper-controlled pattern tables
    chr_bank_mode_4k: bool,
    chr_bank0_offset: usize,
    chr_bank1_offset: usize,

    /// Reference to CHR ROM (for pattern tables)
    chr_rom: Vec<u8>,

    // Rendering State
    /// Framebuffer - 256x240 pixels storing palette indices (0-63)
    /// Each pixel references one of the 64 colors in the NES system palette
    framebuffer: [u8; FRAMEBUFFER_SIZE],
}

impl Ppu {
    /// Create a new PPU.
    ///
    /// # Arguments
    ///
    /// * `chr_rom` - Character ROM data from the cartridge (pattern tables)
    /// * `mirroring` - Nametable mirroring mode from the cartridge
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use rustnes::ppu::Ppu;
    /// use rustnes::cartridge::Mirroring;
    ///
    /// let chr_rom = vec![0; 8192];
    /// let ppu = Ppu::new(chr_rom, Mirroring::Horizontal);
    /// ```
    pub fn new(chr_rom: Vec<u8>, mirroring: Mirroring) -> Self {
        Ppu {
            ctrl: 0,
            mask: 0,
            status: 0,
            oam_addr: 0,
            vram: [0; NAMETABLE_SIZE],
            palette: [0; PALETTE_SIZE],
            oam: [0; OAM_SIZE],
            vram_addr: 0,
            temp_vram_addr: 0,
            fine_x: 0,
            write_latch: false,
            data_buffer: 0,
            scanline: 0,
            cycle: 0,
            frame: 0,
            nmi_pending: false,
            mirroring,
            chr_bank_mode_4k: false,
            chr_bank0_offset: 0,
            chr_bank1_offset: 0x1000,
            chr_rom,
            framebuffer: [0; FRAMEBUFFER_SIZE],
        }
    }

    /// Update CHR bank offsets for mapper-controlled pattern table switching.
    ///
    /// # Arguments
    ///
    /// * `mode_4k` - true to use two 4KB banks, false for a single 8KB bank
    /// * `bank0` - offset in CHR ROM (bytes) for the $0000-$0FFF region
    /// * `bank1` - offset in CHR ROM (bytes) for the $1000-$1FFF region (used when `mode_4k`)
    pub fn set_chr_banks(&mut self, mode_4k: bool, bank0: usize, bank1: usize) {
        self.chr_bank_mode_4k = mode_4k;
        self.chr_bank0_offset = bank0;
        self.chr_bank1_offset = bank1;
    }

    /// Update nametable mirroring at runtime (for mappers that support it).
    pub fn set_mirroring(&mut self, mirroring: Mirroring) {
        self.mirroring = mirroring;
    }

    /// Perform a sprite DMA transfer from CPU memory into OAM.
    ///
    /// The CPU writes to $4014 with the high byte of the source address; 256 bytes are copied.
    /// The DMA starts at the current OAM address and wraps around on overflow.
    pub fn oam_dma(&mut self, data: &[u8]) {
        // OAMADDR is not reset by DMA, so start wherever the register currently points.
        let mut oam_index = self.oam_addr as usize;
        for &byte in data.iter().take(OAM_SIZE) {
            self.oam[oam_index] = byte;
            oam_index = (oam_index + 1) % OAM_SIZE;
        }
    }

    /// Reset the PPU to power-up state.
    #[allow(dead_code)] // Will be used when implementing full emulator loop
    pub fn reset(&mut self) {
        self.ctrl = 0;
        self.mask = 0;
        self.status = 0;
        self.oam_addr = 0;
        self.write_latch = false;
        self.data_buffer = 0;
        self.scanline = 0;
        self.cycle = 0;
        self.nmi_pending = false;
        self.chr_bank_mode_4k = false;
        self.chr_bank0_offset = 0;
        self.chr_bank1_offset = 0x1000;
    }

    // ==================== PPU Register Access ====================

    /// Write to a PPU register.
    ///
    /// Called by the CPU when it writes to $2000-$2007.
    ///
    /// # Arguments
    ///
    /// * `addr` - Register address (0-7 for $2000-$2007)
    /// * `value` - Value to write
    pub fn write_register(&mut self, addr: u16, value: u8) {
        match addr {
            0 => self.write_ctrl(value),
            1 => self.write_mask(value),
            2 => {} // PPUSTATUS is read-only
            3 => self.write_oam_addr(value),
            4 => self.write_oam_data(value),
            5 => self.write_scroll(value),
            6 => self.write_addr(value),
            7 => self.write_data(value),
            _ => {}
        }
    }

    /// Read from a PPU register.
    ///
    /// Called by the CPU when it reads from $2000-$2007.
    ///
    /// # Arguments
    ///
    /// * `addr` - Register address (0-7 for $2000-$2007)
    ///
    /// # Returns
    ///
    /// The value read from the register
    pub fn read_register(&mut self, addr: u16) -> u8 {
        match addr {
            0 => 0, // PPUCTRL is write-only (open bus)
            1 => 0, // PPUMASK is write-only (open bus)
            2 => self.read_status(),
            3 => 0, // OAMADDR is write-only (open bus)
            4 => self.read_oam_data(),
            5 => 0, // PPUSCROLL is write-only (open bus)
            6 => 0, // PPUADDR is write-only (open bus)
            7 => self.read_data(),
            _ => 0,
        }
    }

    /// Write to PPUCTRL ($2000).
    fn write_ctrl(&mut self, value: u8) {
        // Debug output when NMI or rendering settings change significantly
        if (value & 0x80) != (self.ctrl & 0x80) {
            println!(
                "PPUCTRL: NMI {}",
                if (value & 0x80) != 0 {
                    "enabled"
                } else {
                    "disabled"
                }
            );
        }

        self.ctrl = value;

        println!(
            "PPUCTRL write: ${:02X} (BG table={}, SPR table={}, NMI={})",
            value,
            (value >> 4) & 1,
            (value >> 3) & 1,
            (value & 0x80) != 0
        );

        // Bits 0-1 set the nametable select in the temp VRAM address
        // In Python: self.temp_vram_addr = (self.temp_vram_addr & 0xF3FF) | ((value & 0x03) << 10)
        self.temp_vram_addr = (self.temp_vram_addr & 0xF3FF) | ((value as u16 & 0x03) << 10);
    }

    /// Write to PPUMASK ($2001).
    fn write_mask(&mut self, value: u8) {
        // Debug output when rendering is enabled/disabled
        let old_rendering = (self.mask & (PPUMASK_SHOW_BACKGROUND | PPUMASK_SHOW_SPRITES)) != 0;
        let new_rendering = (value & (PPUMASK_SHOW_BACKGROUND | PPUMASK_SHOW_SPRITES)) != 0;

        if old_rendering != new_rendering {
            println!(
                "PPUMASK: Rendering {} (BG={}, SPR={})",
                if new_rendering { "enabled" } else { "disabled" },
                (value & PPUMASK_SHOW_BACKGROUND) != 0,
                (value & PPUMASK_SHOW_SPRITES) != 0
            );
        }

        self.mask = value;
    }

    /// Read from PPUSTATUS ($2002).
    fn read_status(&mut self) -> u8 {
        let result = self.status;

        // Debug: Only log when VBlank flag is set
        if (result & PPUSTATUS_VBLANK) != 0 && self.frame < 5 {
            println!(
                "PPUSTATUS read: ${:02X} (VBlank=TRUE at scanline={}, frame={})",
                result, self.scanline, self.frame
            );
        }

        // Reading PPUSTATUS clears VBlank flag and write latch
        self.status &= !PPUSTATUS_VBLANK;
        self.write_latch = false;

        result
    }

    /// Write to OAMADDR ($2003).
    fn write_oam_addr(&mut self, value: u8) {
        self.oam_addr = value;
    }

    /// Write to OAMDATA ($2004).
    fn write_oam_data(&mut self, value: u8) {
        self.oam[self.oam_addr as usize] = value;
        // OAM address increments after write
        self.oam_addr = self.oam_addr.wrapping_add(1);
    }

    /// Read from OAMDATA ($2004).
    fn read_oam_data(&self) -> u8 {
        self.oam[self.oam_addr as usize]
    }

    /// Write to PPUSCROLL ($2005).
    ///
    /// This is a double-write register. First write sets X scroll, second sets Y scroll.
    fn write_scroll(&mut self, value: u8) {
        if !self.write_latch {
            // First write: X scroll
            self.temp_vram_addr = (self.temp_vram_addr & 0xFFE0) | ((value as u16) >> 3);
            self.fine_x = value & 0x07;
            self.write_latch = true;
        } else {
            // Second write: Y scroll
            self.temp_vram_addr = (self.temp_vram_addr & 0x8FFF) | (((value as u16) & 0x07) << 12);
            self.temp_vram_addr = (self.temp_vram_addr & 0xFC1F) | (((value as u16) & 0xF8) << 2);
            self.write_latch = false;
        }
    }

    /// Write to PPUADDR ($2006).
    ///
    /// This is a double-write register. First write sets high byte, second sets low byte.
    fn write_addr(&mut self, value: u8) {
        if !self.write_latch {
            // First write: high byte
            self.temp_vram_addr = (self.temp_vram_addr & 0x00FF) | (((value as u16) & 0x3F) << 8);
            self.write_latch = true;
        } else {
            // Second write: low byte
            self.temp_vram_addr = (self.temp_vram_addr & 0xFF00) | (value as u16);
            self.vram_addr = self.temp_vram_addr;
            self.write_latch = false;
        }
    }

    /// Write to PPUDATA ($2007).
    fn write_data(&mut self, value: u8) {
        if self.vram_addr >= 0x3F00 && self.vram_addr <= 0x3F1F {
            println!(
                "PPUDATA palette write @${:04X} = ${:02X}",
                self.vram_addr, value
            );
        } else {
            // Log the first handful of nametable/CHR writes to see if the ROM initializes them.
            static mut NON_PALETTE_WRITES: u32 = 0;
            unsafe {
                if NON_PALETTE_WRITES < 64 {
                    NON_PALETTE_WRITES += 1;
                    println!(
                        "PPUDATA write @${:04X} = ${:02X} (CHR/VRAM)",
                        self.vram_addr, value
                    );
                }
            }
        }
        self.ppu_write(self.vram_addr, value);
        self.increment_vram_addr();
    }

    /// Read from PPUDATA ($2007).
    fn read_data(&mut self) -> u8 {
        let addr = self.vram_addr;
        self.increment_vram_addr();

        // Reading from PPUDATA is delayed by one read (buffered)
        // Exception: palette reads are not buffered
        if addr >= 0x3F00 {
            // Palette read - return immediately but still fill buffer with nametable
            self.data_buffer = self.ppu_read(addr - 0x1000);
            self.ppu_read(addr)
        } else {
            // Normal read - return buffered value and update buffer
            let result = self.data_buffer;
            self.data_buffer = self.ppu_read(addr);
            result
        }
    }

    /// Increment VRAM address based on PPUCTRL settings.
    fn increment_vram_addr(&mut self) {
        let increment = if (self.ctrl & PPUCTRL_VRAM_INCREMENT) != 0 {
            32 // Increment by 32 (down) for vertical writes
        } else {
            1 // Increment by 1 (right) for horizontal writes
        };
        self.vram_addr = self.vram_addr.wrapping_add(increment);
    }

    // ==================== PPU Memory Access ====================

    /// Read from PPU memory.
    ///
    /// # Arguments
    ///
    /// * `addr` - PPU memory address (0x0000-0x3FFF)
    ///
    /// # Returns
    ///
    /// The byte at the specified address
    fn ppu_read(&self, addr: u16) -> u8 {
        let addr = addr & 0x3FFF; // Mirror addresses above 0x3FFF

        match addr {
            // Pattern tables (CHR ROM)
            0x0000..=0x1FFF => {
                let (bank_offset, offset_within_bank) = if self.chr_bank_mode_4k {
                    if addr < 0x1000 {
                        (self.chr_bank0_offset, addr as usize)
                    } else {
                        (self.chr_bank1_offset, (addr - 0x1000) as usize)
                    }
                } else {
                    (self.chr_bank0_offset, addr as usize)
                };

                let index = bank_offset + offset_within_bank;
                if index < self.chr_rom.len() {
                    self.chr_rom[index]
                } else {
                    0
                }
            }

            // Nametables
            0x2000..=0x2FFF => {
                let mirrored_addr = self.mirror_nametable_addr(addr);
                self.vram[mirrored_addr]
            }

            // Mirrors of nametables
            0x3000..=0x3EFF => {
                let mirrored_addr = self.mirror_nametable_addr(addr - 0x1000);
                self.vram[mirrored_addr]
            }

            // Palette RAM
            0x3F00..=0x3FFF => {
                let mut palette_addr = (addr & 0x1F) as usize;
                // Mirror $3F10, $3F14, $3F18, $3F1C to $3F00, $3F04, $3F08, $3F0C
                // (sprite palette mirrors background palette for transparent color)
                if palette_addr >= 16 && palette_addr.is_multiple_of(4) {
                    palette_addr -= 16;
                }
                self.palette[palette_addr]
            }

            _ => 0,
        }
    }

    /// Write to PPU memory.
    ///
    /// # Arguments
    ///
    /// * `addr` - PPU memory address (0x0000-0x3FFF)
    /// * `value` - Value to write
    fn ppu_write(&mut self, addr: u16, value: u8) {
        let addr = addr & 0x3FFF; // Mirror addresses above 0x3FFF

        match addr {
            // Pattern tables (CHR ROM) - usually read-only
            // Some games use CHR RAM instead
            0x0000..=0x1FFF => {
                // For now, allow writes to CHR RAM if present
                if (addr as usize) < self.chr_rom.len() {
                    self.chr_rom[addr as usize] = value;
                }
            }

            // Nametables
            0x2000..=0x2FFF => {
                let mirrored_addr = self.mirror_nametable_addr(addr);
                self.vram[mirrored_addr] = value;
            }

            // Mirrors of nametables
            0x3000..=0x3EFF => {
                let mirrored_addr = self.mirror_nametable_addr(addr - 0x1000);
                self.vram[mirrored_addr] = value;
            }

            // Palette RAM
            0x3F00..=0x3FFF => {
                let mut palette_addr = (addr & 0x1F) as usize;
                // Mirror sprite palette transparent color to background palette
                if palette_addr >= 16 && palette_addr.is_multiple_of(4) {
                    palette_addr -= 16;
                }
                self.palette[palette_addr] = value;
            }

            _ => {}
        }
    }

    /// Mirror nametable address based on mirroring mode.
    ///
    /// The NES has 2KB of VRAM for nametables, but the address space has room for 4KB.
    /// Mirroring determines which physical memory each nametable address maps to.
    ///
    /// # Arguments
    ///
    /// * `addr` - Nametable address (0x2000-0x2FFF)
    ///
    /// # Returns
    ///
    /// Physical VRAM address (0x0000-0x07FF)
    fn mirror_nametable_addr(&self, addr: u16) -> usize {
        let addr = (addr - 0x2000) & 0x0FFF; // Normalize to 0-0xFFF
        let table = (addr / 0x400) as usize; // Which nametable (0-3)
        let offset = (addr % 0x400) as usize; // Offset within nametable

        let mirrored_table = match self.mirroring {
            Mirroring::Horizontal => {
                // Tables 0,1 -> 0; Tables 2,3 -> 1
                table / 2
            }
            Mirroring::Vertical => {
                // Tables 0,2 -> 0; Tables 1,3 -> 1
                table % 2
            }
            Mirroring::FourScreen => {
                // Each table is separate (would need 4KB VRAM)
                // For now, mirror to available 2KB
                table % 2
            }
            Mirroring::OneScreenLower => 0,
            Mirroring::OneScreenUpper => 1,
        };

        mirrored_table * 0x400 + offset
    }

    // ==================== Background Rendering ====================

    /// Fetch a nametable byte based on current VRAM address.
    ///
    /// The nametable byte indicates which tile (0-255) to display at this position.
    fn fetch_nametable_byte(&self) -> u8 {
        let addr = 0x2000 | (self.vram_addr & 0x0FFF);
        self.ppu_read(addr)
    }

    /// Fetch an attribute byte for the current tile position.
    ///
    /// The attribute table stores palette information. Each byte controls
    /// the palette for a 4x4 tile region (2 bits per 2x2 tile block).
    fn fetch_attribute_byte(&self) -> u8 {
        // Attribute table is at nametable + $3C0
        let nametable_base = 0x2000 | (self.vram_addr & 0x0C00);
        let coarse_x = (self.vram_addr & 0x001F) >> 2;
        let coarse_y = ((self.vram_addr & 0x03E0) >> 5) >> 2;
        let attr_addr = nametable_base + 0x3C0 + coarse_y * 8 + coarse_x;

        let attribute = self.ppu_read(attr_addr);

        // Extract the 2-bit palette for this specific 2x2 tile block
        let quadrant_x = (self.vram_addr & 0x0002) >> 1;
        let quadrant_y = (self.vram_addr & 0x0040) >> 6;
        let shift = (quadrant_y * 4) + (quadrant_x * 2);

        (attribute >> shift) & 0x03
    }

    /// Fetch a pattern table byte for a tile.
    ///
    /// Each 8x8 tile uses 16 bytes: 2 bytes per row (low and high bitplanes).
    /// Combining the bitplanes gives 2 bits per pixel (4 colors per tile).
    ///
    /// # Arguments
    ///
    /// * `tile_index` - Which tile (0-255)
    /// * `fine_y` - Which row within the tile (0-7)
    /// * `plane` - Which bitplane (0 = low, 1 = high)
    fn fetch_pattern_byte(&self, tile_index: u8, fine_y: u8, plane: u8) -> u8 {
        // Background pattern table address from PPUCTRL bit 4
        let table = if (self.ctrl & PPUCTRL_BACKGROUND_PATTERN) != 0 {
            0x1000
        } else {
            0x0000
        };

        let addr = table + (tile_index as u16 * 16) + fine_y as u16 + (plane as u16 * 8);

        self.ppu_read(addr)
    }

    /// Render one scanline of background tiles.
    ///
    /// This processes the current scanline and writes pixels to the framebuffer.
    fn render_background_scanline(&mut self) {
        if !self.rendering_enabled() || (self.mask & PPUMASK_SHOW_BACKGROUND) == 0 {
            return;
        }

        let scanline = self.scanline;
        if scanline >= 240 {
            return;
        }

        // Save the current VRAM address to restore later
        let saved_vram_addr = self.vram_addr;

        // Fine Y scroll - which row within the current tile
        let fine_y = ((self.vram_addr >> 12) & 0x07) as u8;

        // Render 32 tiles + partial tiles (accounting for fine_x scroll)
        for tile_x in 0..33 {
            // Fetch tile information
            let tile_index = self.fetch_nametable_byte();
            let attribute = self.fetch_attribute_byte();

            // Fetch pattern data (2 bitplanes)
            let pattern_low = self.fetch_pattern_byte(tile_index, fine_y, 0);
            let pattern_high = self.fetch_pattern_byte(tile_index, fine_y, 1);

            // Render the 8 pixels of this tile
            for pixel_offset in 0..8 {
                let screen_x = (tile_x * 8 + pixel_offset) as i32 - self.fine_x as i32;

                // Skip pixels that are off-screen or in hidden left column
                if screen_x < 0 || screen_x >= 256 {
                    continue;
                }

                if screen_x < 8 && (self.mask & PPUMASK_SHOW_BACKGROUND_LEFT) == 0 {
                    continue;
                }

                // Combine bitplanes to get 2-bit color
                let bit_pos = 7 - pixel_offset;
                let color_low = (pattern_low >> bit_pos) & 1;
                let color_high = (pattern_high >> bit_pos) & 1;
                let color = (color_high << 1) | color_low;

                // Look up palette color
                let palette_addr = if color == 0 {
                    0x3F00 // Universal background color
                } else {
                    0x3F00 + (attribute as u16 * 4) + color as u16
                };

                let palette_index = self.ppu_read(palette_addr);

                // Write to framebuffer
                let fb_index = (scanline as usize * SCREEN_WIDTH) + screen_x as usize;
                if fb_index < FRAMEBUFFER_SIZE {
                    self.framebuffer[fb_index] = palette_index;
                }
            }

            // Move to next tile
            self.increment_scroll_x();
        }

        // Restore VRAM address and increment Y
        self.vram_addr = saved_vram_addr;
        self.increment_scroll_y();
    }

    /// Increment the horizontal scroll position (coarse X).
    ///
    /// Called when moving to the next tile horizontally.
    fn increment_scroll_x(&mut self) {
        if (self.vram_addr & 0x001F) == 31 {
            // Wrap to next nametable
            self.vram_addr &= !0x001F;
            self.vram_addr ^= 0x0400; // Switch horizontal nametable
        } else {
            self.vram_addr += 1;
        }
    }

    /// Increment the vertical scroll position (fine Y and coarse Y).
    ///
    /// Called at the end of each scanline.
    fn increment_scroll_y(&mut self) {
        if (self.vram_addr & 0x7000) != 0x7000 {
            // Increment fine Y
            self.vram_addr += 0x1000;
        } else {
            // Fine Y overflow, increment coarse Y
            self.vram_addr &= !0x7000;
            let mut coarse_y = (self.vram_addr & 0x03E0) >> 5;

            if coarse_y == 29 {
                coarse_y = 0;
                self.vram_addr ^= 0x0800; // Switch vertical nametable
            } else if coarse_y == 31 {
                coarse_y = 0;
            } else {
                coarse_y += 1;
            }

            self.vram_addr = (self.vram_addr & !0x03E0) | (coarse_y << 5);
        }
    }

    // ==================== Sprite Rendering ====================

    /// Evaluate which sprites appear on the current scanline.
    ///
    /// The NES can display up to 8 sprites per scanline. This method
    /// scans OAM to find sprites that intersect the current scanline.
    ///
    /// # Arguments
    ///
    /// * `scanline` - The scanline to evaluate (0-239)
    ///
    /// # Returns
    ///
    /// A vector of up to 8 sprites to render on this scanline
    fn evaluate_sprites(&mut self, scanline: u16) -> Vec<SpriteData> {
        let mut sprites = Vec::new();
        let sprite_height = if (self.ctrl & PPUCTRL_SPRITE_SIZE) != 0 {
            16 // 8x16 sprites
        } else {
            8 // 8x8 sprites
        };

        // Scan all 64 sprites in OAM
        for sprite_idx in 0..64 {
            let oam_offset = sprite_idx * 4;
            let y = self.oam[oam_offset] as u16;
            let tile_index = self.oam[oam_offset + 1];
            let attributes = self.oam[oam_offset + 2];
            let x = self.oam[oam_offset + 3];

            // Check if sprite is on this scanline
            // Y position is offset by 1 (Y=0 means sprite appears on scanline 1)
            let sprite_top = y.wrapping_add(1);
            let sprite_bottom = sprite_top.wrapping_add(sprite_height);

            if scanline >= sprite_top && scanline < sprite_bottom {
                // Set overflow flag if more than 8 sprites on this scanline
                if sprites.len() >= 8 {
                    self.status |= PPUSTATUS_SPRITE_OVERFLOW;
                    break;
                }

                sprites.push(SpriteData {
                    x,
                    y: y as u8,
                    tile_index,
                    palette: attributes & 0x03,
                    priority: (attributes & 0x20) != 0,
                    flip_h: (attributes & 0x40) != 0,
                    flip_v: (attributes & 0x80) != 0,
                    sprite_0: sprite_idx == 0,
                });
            }
        }

        sprites
    }

    /// Render sprites for the current scanline.
    ///
    /// This fetches sprite pattern data and composites sprites onto the framebuffer,
    /// handling priority, flipping, and sprite 0 hit detection.
    fn render_sprites_scanline(&mut self) {
        if !self.rendering_enabled() || (self.mask & PPUMASK_SHOW_SPRITES) == 0 {
            return;
        }

        let scanline = self.scanline;
        if scanline >= 240 {
            return;
        }

        let sprites = self.evaluate_sprites(scanline);

        // Render sprites in reverse order (lower priority sprites first)
        // This ensures higher priority sprites (earlier in OAM) appear on top
        for sprite in sprites.iter().rev() {
            let sprite_height = if (self.ctrl & PPUCTRL_SPRITE_SIZE) != 0 {
                16
            } else {
                8
            };

            // Calculate row within sprite
            let sprite_top = sprite.y.wrapping_add(1);
            let mut row = scanline as i16 - sprite_top as i16;

            // Apply vertical flip
            if sprite.flip_v {
                row = (sprite_height - 1) as i16 - row;
            }

            if row < 0 || row >= sprite_height as i16 {
                continue;
            }

            let fine_y = row as u8;

            // Fetch sprite pattern data
            let pattern_table = if (self.ctrl & PPUCTRL_SPRITE_PATTERN) != 0 {
                0x1000
            } else {
                0x0000
            };

            let tile_addr = if sprite_height == 16 {
                // 8x16 sprites: bit 0 of tile index selects pattern table
                let table = if (sprite.tile_index & 0x01) != 0 {
                    0x1000
                } else {
                    0x0000
                };
                let tile = sprite.tile_index & 0xFE; // Even tile
                let tile_offset = if fine_y >= 8 { 1 } else { 0 }; // Top or bottom tile
                table + ((tile + tile_offset) as u16 * 16) + (fine_y % 8) as u16
            } else {
                // 8x8 sprites
                pattern_table + (sprite.tile_index as u16 * 16) + fine_y as u16
            };

            let pattern_low = self.ppu_read(tile_addr);
            let pattern_high = self.ppu_read(tile_addr + 8);

            // Render the 8 pixels
            for pixel_offset in 0..8 {
                let screen_x = sprite.x.wrapping_add(pixel_offset) as usize;

                // Check if pixel is visible
                if screen_x >= 256 {
                    continue;
                }

                // Check left column clipping
                if screen_x < 8 && (self.mask & PPUMASK_SHOW_SPRITES_LEFT) == 0 {
                    continue;
                }

                // Apply horizontal flip
                let bit_pos = if sprite.flip_h {
                    pixel_offset
                } else {
                    7 - pixel_offset
                };

                // Combine bitplanes
                let color_low = (pattern_low >> bit_pos) & 1;
                let color_high = (pattern_high >> bit_pos) & 1;
                let color = (color_high << 1) | color_low;

                // Skip transparent pixels
                if color == 0 {
                    continue;
                }

                // Get current background pixel
                let fb_index = (scanline as usize * SCREEN_WIDTH) + screen_x;
                let bg_pixel = self.framebuffer[fb_index];
                let bg_color_index = bg_pixel & 0x03;

                // Sprite 0 hit detection
                if sprite.sprite_0 && bg_color_index != 0 && screen_x != 255 {
                    self.status |= PPUSTATUS_SPRITE_ZERO_HIT;
                }

                // Check sprite priority
                if sprite.priority && bg_color_index != 0 {
                    // Sprite is behind background and background is opaque
                    continue;
                }

                // Look up sprite palette color
                let palette_addr = 0x3F10 + (sprite.palette as u16 * 4) + color as u16;
                let palette_index = self.ppu_read(palette_addr);

                // Write to framebuffer
                if fb_index < FRAMEBUFFER_SIZE {
                    self.framebuffer[fb_index] = palette_index;
                }
            }
        }
    }

    // ==================== PPU Timing ====================

    /// Step the PPU by one PPU cycle.
    ///
    /// The PPU runs at 3x the CPU speed (approximately).
    /// Returns true if a frame has completed.
    #[allow(dead_code)] // Will be used in main emulator loop
    pub fn step(&mut self) -> bool {
        self.cycle += 1;

        // Rendering happens during visible scanlines (0-239)
        if self.scanline < 240 {
            // At cycle 257, we've finished rendering the visible portion of the scanline
            // This is when we render the background and sprites for this scanline
            if self.cycle == 257 {
                self.render_background_scanline();
                self.render_sprites_scanline();
            }
        }

        // Pre-render scanline (261) - prepare for next frame
        if self.scanline == 261 {
            if self.cycle == 1 {
                // Clear flags at start of pre-render scanline
                self.status &= !PPUSTATUS_VBLANK;
                self.status &= !PPUSTATUS_SPRITE_ZERO_HIT;
                self.status &= !PPUSTATUS_SPRITE_OVERFLOW;

                // Clear framebuffer to universal background color
                let bg_color = self.ppu_read(0x3F00);
                self.clear_framebuffer(bg_color);
            }

            // At cycle 257 of pre-render scanline, copy X scroll from temp
            if self.cycle == 257 && self.rendering_enabled() {
                self.vram_addr = (self.vram_addr & 0x7BE0) | (self.temp_vram_addr & 0x041F);
            }

            // During dots 280-304, copy Y scroll from temp
            if self.cycle >= 280 && self.cycle <= 304 && self.rendering_enabled() {
                self.vram_addr = (self.vram_addr & 0x041F) | (self.temp_vram_addr & 0x7BE0);
            }
        }

        // Each scanline is 341 cycles
        if self.cycle > 340 {
            self.cycle = 0;
            self.scanline += 1;

            // Copy X scroll at the start of each visible scanline
            if self.scanline < 240 && self.cycle == 0 && self.rendering_enabled() {
                self.vram_addr = (self.vram_addr & 0x7BE0) | (self.temp_vram_addr & 0x041F);
            }

            // Total of 262 scanlines (0-261)
            // 0-239: Visible scanlines
            // 240: Post-render (idle)
            // 241-260: VBlank
            // 261: Pre-render
            if self.scanline == 241 {
                // Enter VBlank - this is when we signal frame completion
                // The framebuffer contains the fully rendered frame at this point
                self.status |= PPUSTATUS_VBLANK;
                self.frame += 1;

                // Trigger NMI if enabled
                if (self.ctrl & PPUCTRL_NMI_ENABLE) != 0 {
                    self.nmi_pending = true;
                }

                return true; // Frame completed - framebuffer ready for display
            }

            if self.scanline > 261 {
                self.scanline = 0;
            }
        }

        false
    }

    /// Check if NMI should be triggered and clear the flag.
    ///
    /// The CPU should call this to check for NMI interrupts.
    #[allow(dead_code)] // Will be used in main emulator loop
    pub fn poll_nmi(&mut self) -> bool {
        let nmi = self.nmi_pending;
        self.nmi_pending = false;
        nmi
    }

    /// Check if PPU address line A12 rose (for MMC3 IRQ counter).
    ///
    /// A12 typically rises when switching between pattern tables during rendering.
    /// This is used by MMC3 to clock its scanline counter.
    /// Returns true if A12 transitioned from low to high.
    ///
    /// Simplified implementation: trigger once per scanline during rendering.
    pub fn check_a12_rise(&mut self) -> bool {
        // During rendering (scanlines 0-239 and pre-render 261),
        // A12 changes when fetching sprite vs background pattern data
        // For MMC3 IRQ purposes, trigger once per scanline
        if (self.scanline < 240 || self.scanline == 261) && self.cycle == 260 && self.rendering_enabled() {
            return true;
        }
        false
    }

    /// Get the current frame number.
    #[allow(dead_code)] // Will be used for debugging and timing
    pub fn frame_number(&self) -> u64 {
        self.frame
    }

    /// Check if rendering is enabled.
    #[allow(dead_code)] // Will be used when implementing rendering
    pub fn rendering_enabled(&self) -> bool {
        (self.mask & (PPUMASK_SHOW_BACKGROUND | PPUMASK_SHOW_SPRITES)) != 0
    }

    /// Get the current scanline.
    #[allow(dead_code)] // Will be used for debugging and rendering
    pub fn scanline(&self) -> u16 {
        self.scanline
    }

    /// Get the current cycle.
    #[allow(dead_code)] // Will be used for debugging and rendering
    pub fn cycle(&self) -> u16 {
        self.cycle
    }

    /// Get the PPUCTRL register value (for debugging).
    pub fn ctrl_value(&self) -> u8 {
        self.ctrl
    }

    /// Get the PPUMASK register value (for debugging).
    pub fn mask_value(&self) -> u8 {
        self.mask
    }

    // ==================== Rendering Output ====================

    /// Get the framebuffer as RGBA pixels for display.
    ///
    /// Converts the internal framebuffer (256x240 palette indices) to RGBA format
    /// suitable for rendering to a display. Each pixel becomes 4 bytes (R, G, B, A).
    ///
    /// # Returns
    ///
    /// A vector of RGBA bytes (width * height * 4 bytes)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use rustnes::ppu::Ppu;
    /// use rustnes::cartridge::Mirroring;
    ///
    /// let chr_rom = vec![0; 8192];
    /// let ppu = Ppu::new(chr_rom, Mirroring::Horizontal);
    /// let rgba_data = ppu.get_framebuffer_rgba();
    /// assert_eq!(rgba_data.len(), 256 * 240 * 4);
    /// ```
    pub fn get_framebuffer_rgba(&self) -> Vec<u8> {
        let mut rgba = Vec::with_capacity(FRAMEBUFFER_SIZE * 4);

        for &palette_idx in &self.framebuffer {
            // Ensure palette index is in valid range (0-63)
            let idx = (palette_idx & 0x3F) as usize;
            let (r, g, b) = SYSTEM_PALETTE[idx];

            rgba.push(r); // Red
            rgba.push(g); // Green
            rgba.push(b); // Blue
            rgba.push(255); // Alpha (fully opaque)
        }

        rgba
    }

    /// Clear the framebuffer to a specific color.
    ///
    /// # Arguments
    ///
    /// * `color` - Palette index (0-63) to fill the framebuffer with
    fn clear_framebuffer(&mut self, color: u8) {
        self.framebuffer.fill(color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_ppu() -> Ppu {
        let chr_rom = vec![0; 8192];
        Ppu::new(chr_rom, Mirroring::Horizontal)
    }

    #[test]
    fn test_ppu_reset() {
        let mut ppu = create_test_ppu();
        ppu.ctrl = 0xFF;
        ppu.mask = 0xFF;
        ppu.scanline = 100;

        ppu.reset();

        assert_eq!(ppu.ctrl, 0);
        assert_eq!(ppu.mask, 0);
        assert_eq!(ppu.scanline, 0);
    }

    #[test]
    fn test_ppuctrl_write() {
        let mut ppu = create_test_ppu();
        ppu.write_register(0, 0b1000_0011);

        assert_eq!(ppu.ctrl, 0b1000_0011);
        // Bits 0-1 should update temp_vram_addr
        assert_eq!(ppu.temp_vram_addr & 0x0C00, 0x0C00);
    }

    #[test]
    fn test_ppustatus_read_clears_vblank() {
        let mut ppu = create_test_ppu();
        ppu.status = PPUSTATUS_VBLANK;

        let value = ppu.read_register(2);

        assert_eq!(value, PPUSTATUS_VBLANK);
        assert_eq!(ppu.status & PPUSTATUS_VBLANK, 0);
    }

    #[test]
    fn test_ppustatus_read_clears_write_latch() {
        let mut ppu = create_test_ppu();
        ppu.write_latch = true;

        ppu.read_register(2);

        assert!(!ppu.write_latch);
    }

    #[test]
    fn test_ppuaddr_double_write() {
        let mut ppu = create_test_ppu();

        // First write: high byte
        ppu.write_register(6, 0x20);
        assert_eq!(ppu.temp_vram_addr, 0x2000);
        assert!(ppu.write_latch);

        // Second write: low byte
        ppu.write_register(6, 0x05);
        assert_eq!(ppu.vram_addr, 0x2005);
        assert!(!ppu.write_latch);
    }

    #[test]
    fn test_ppudata_write_increment() {
        let mut ppu = create_test_ppu();
        ppu.vram_addr = 0x2000;

        // Write with increment = 1
        ppu.write_register(7, 0x42);
        assert_eq!(ppu.vram_addr, 0x2001);

        // Write with increment = 32
        ppu.ctrl = PPUCTRL_VRAM_INCREMENT;
        ppu.write_register(7, 0x43);
        assert_eq!(ppu.vram_addr, 0x2001 + 32);
    }

    #[test]
    fn test_ppudata_read_buffer() {
        let mut ppu = create_test_ppu();

        // Write a value to VRAM
        ppu.ppu_write(0x2000, 0x42);

        // Set address to read
        ppu.vram_addr = 0x2000;

        // First read returns old buffer (0)
        let first = ppu.read_register(7);
        assert_eq!(first, 0);

        // Second read returns the actual value
        ppu.vram_addr = 0x2000;
        ppu.read_register(7); // Load buffer
        let second = ppu.read_register(7);
        assert_eq!(second, 0x42);
    }

    #[test]
    fn test_palette_mirroring() {
        let mut ppu = create_test_ppu();

        // Write to sprite palette transparent ($3F10)
        ppu.ppu_write(0x3F10, 0x30);

        // Should mirror to background palette transparent ($3F00)
        assert_eq!(ppu.ppu_read(0x3F00), 0x30);
        assert_eq!(ppu.ppu_read(0x3F10), 0x30);
    }

    #[test]
    fn test_nametable_mirroring_horizontal() {
        let mut ppu = create_test_ppu();
        ppu.mirroring = Mirroring::Horizontal;

        // Horizontal mirroring: Tables 0,1 -> same; Tables 2,3 -> same
        ppu.ppu_write(0x2000, 0xAA); // Table 0
        ppu.ppu_write(0x2800, 0xBB); // Table 2

        // Table 0 and 1 should be the same
        assert_eq!(ppu.ppu_read(0x2000), 0xAA);
        assert_eq!(ppu.ppu_read(0x2400), 0xAA);

        // Table 2 and 3 should be the same
        assert_eq!(ppu.ppu_read(0x2800), 0xBB);
        assert_eq!(ppu.ppu_read(0x2C00), 0xBB);
    }

    #[test]
    fn test_nametable_mirroring_vertical() {
        let mut ppu = create_test_ppu();
        ppu.mirroring = Mirroring::Vertical;

        // Vertical mirroring: Tables 0,2 -> same; Tables 1,3 -> same
        ppu.ppu_write(0x2000, 0xAA); // Table 0
        ppu.ppu_write(0x2400, 0xBB); // Table 1

        // Table 0 and 2 should be the same
        assert_eq!(ppu.ppu_read(0x2000), 0xAA);
        assert_eq!(ppu.ppu_read(0x2800), 0xAA);

        // Table 1 and 3 should be the same
        assert_eq!(ppu.ppu_read(0x2400), 0xBB);
        assert_eq!(ppu.ppu_read(0x2C00), 0xBB);
    }

    #[test]
    fn test_vblank_flag() {
        let mut ppu = create_test_ppu();

        // Step through to VBlank (scanline 241)
        ppu.scanline = 240;
        ppu.cycle = 340;

        ppu.step();

        assert_eq!(ppu.scanline, 241);
        assert!(ppu.status & PPUSTATUS_VBLANK != 0);
    }

    #[test]
    fn test_nmi_generation() {
        let mut ppu = create_test_ppu();
        ppu.ctrl = PPUCTRL_NMI_ENABLE;

        // Step to VBlank
        ppu.scanline = 240;
        ppu.cycle = 340;

        ppu.step();

        assert!(ppu.poll_nmi());
        // Second poll should return false
        assert!(!ppu.poll_nmi());
    }

    #[test]
    fn test_oam_write() {
        let mut ppu = create_test_ppu();

        // Set OAM address
        ppu.write_register(3, 0x10);

        // Write to OAM
        ppu.write_register(4, 0x42);

        // OAM address should increment
        assert_eq!(ppu.oam_addr, 0x11);
        assert_eq!(ppu.oam[0x10], 0x42);
    }

    #[test]
    fn test_frame_completion() {
        let mut ppu = create_test_ppu();

        // Step to end of frame
        ppu.scanline = 261;
        ppu.cycle = 340;

        let frame_complete = ppu.step();

        assert!(frame_complete);
        assert_eq!(ppu.scanline, 0);
        assert_eq!(ppu.frame, 1);
    }
}
