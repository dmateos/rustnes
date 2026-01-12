//! NES cartridge handling and iNES ROM file loading.
//!
//! This module provides functionality to load and parse NES ROM files in the iNES format,
//! which is the most common format for NES ROM images.

use std::fs::File;
use std::io::{self, Read};
use std::path::Path;
use thiserror::Error;

const NES_MAGIC: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A]; // "NES\x1A"
const PRG_ROM_PAGE_SIZE: usize = 16384; // 16 KB
const CHR_ROM_PAGE_SIZE: usize = 8192; // 8 KB
const TRAINER_SIZE: usize = 512;

/// Errors that can occur when loading or parsing NES cartridge ROM files.
#[derive(Debug, Error)]
pub enum CartridgeError {
    /// I/O error occurred while reading the ROM file
    #[error("I/O error: {0}")]
    IoError(#[from] io::Error),

    /// The ROM file has an invalid format or structure
    #[error("Invalid ROM format: {0}")]
    InvalidFormat(String),
}

/// Nametable mirroring modes used by the NES PPU.
///
/// In Python terms, this is similar to an Enum class with three values.
/// Rust enums are more powerful than Python's - they can hold data in variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    /// Horizontal mirroring (vertical arrangement)
    Horizontal,
    /// Vertical mirroring (horizontal arrangement)
    Vertical,
    /// Four-screen VRAM (used by some mappers)
    FourScreen,
    /// One-screen mirroring using the lower nametable ($2000)
    OneScreenLower,
    /// One-screen mirroring using the upper nametable ($2400)
    OneScreenUpper,
}

/// Represents a loaded NES cartridge ROM.
///
/// This struct contains all the data and metadata from an iNES format ROM file.
/// The cartridge contains both program code (PRG ROM) and graphics data (CHR ROM),
/// along with configuration flags that tell the emulator how to run the game.
#[derive(Debug, Clone, PartialEq)]
pub struct Cartridge {
    /// Program ROM - contains the game's executable code (CPU 6502 instructions)
    pub prg_rom: Vec<u8>,

    /// Character ROM - contains graphics/tile data for the PPU
    /// If the ROM has no CHR ROM, this is CHR RAM (8KB, all zeros initially)
    pub chr_rom: Vec<u8>,

    /// Mapper number - determines how the ROM banks are mapped into memory
    /// Different games use different mappers for bank switching
    pub mapper: u8,

    /// Nametable mirroring mode
    pub mirroring: Mirroring,

    /// Whether the cartridge has battery-backed RAM for save data
    pub has_battery: bool,

    /// Whether the ROM includes a 512-byte trainer (rarely used)
    pub has_trainer: bool,
}

impl Cartridge {
    /// Load a NES ROM file in iNES format.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the .nes ROM file
    ///
    /// # Returns
    ///
    /// A `Cartridge` struct containing all ROM data and metadata
    ///
    /// # Errors
    ///
    /// Returns `CartridgeError::IoError` if the file cannot be read
    /// Returns `CartridgeError::InvalidFormat` if the file is not a valid iNES ROM
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use rustnes::cartridge::Cartridge;
    ///
    /// let cart = Cartridge::load("game.nes").expect("Failed to load ROM");
    /// println!("Mapper: {}", cart.mapper);
    /// ```
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, CartridgeError> {
        let mut file = File::open(path)?;

        // Read the 16-byte iNES header
        let mut header = [0u8; 16];
        file.read_exact(&mut header)?;

        // Verify the NES magic number ("NES\x1A")
        // In Python, you'd use: if header[:4] != b"NES\x1A"
        if header[0..4] != NES_MAGIC {
            return Err(CartridgeError::InvalidFormat(
                "Invalid iNES header: missing NES magic number".to_string(),
            ));
        }

        // Parse header fields (bytes are accessed by index like Python)
        let prg_rom_pages = header[4] as usize;
        let chr_rom_pages = header[5] as usize;
        let flags6 = header[6];
        let flags7 = header[7];

        // Parse flags using bitwise operations
        // In Python: has_trainer = bool(flags6 & 0x04)
        let has_trainer = (flags6 & 0x04) != 0;
        let has_battery = (flags6 & 0x02) != 0;
        let four_screen = (flags6 & 0x08) != 0;
        let vertical_mirroring = (flags6 & 0x01) != 0;

        // Determine mirroring mode
        let mirroring = if four_screen {
            Mirroring::FourScreen
        } else if vertical_mirroring {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };

        // Calculate mapper number from both flag bytes
        // Upper 4 bits of flags7 and upper 4 bits of flags6
        let mapper = (flags7 & 0xF0) | (flags6 >> 4);

        // Skip trainer data if present (512 bytes)
        if has_trainer {
            let mut trainer = vec![0u8; TRAINER_SIZE];
            file.read_exact(&mut trainer)?;
        }

        // Read PRG ROM (program code)
        // In Python: prg_rom = file.read(prg_rom_size)
        let prg_rom_size = prg_rom_pages * PRG_ROM_PAGE_SIZE;
        let mut prg_rom = vec![0u8; prg_rom_size];
        file.read_exact(&mut prg_rom)?;

        // Read CHR ROM (graphics data), or allocate CHR RAM if none
        let chr_rom_size = chr_rom_pages * CHR_ROM_PAGE_SIZE;
        let chr_rom = if chr_rom_size > 0 {
            let mut data = vec![0u8; chr_rom_size];
            file.read_exact(&mut data)?;
            data
        } else {
            // No CHR ROM means the game uses CHR RAM
            // Allocate 8KB of RAM initialized to zero
            vec![0u8; CHR_ROM_PAGE_SIZE]
        };

        Ok(Cartridge {
            prg_rom,
            chr_rom,
            mapper,
            mirroring,
            has_battery,
            has_trainer,
        })
    }

    /// Get the size of PRG ROM in bytes.
    pub fn prg_rom_size(&self) -> usize {
        self.prg_rom.len()
    }

    /// Get the size of CHR ROM in bytes.
    pub fn chr_rom_size(&self) -> usize {
        self.chr_rom.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper function to create a minimal valid iNES ROM for testing
    fn create_test_rom(prg_pages: u8, chr_pages: u8, flags6: u8, flags7: u8) -> Vec<u8> {
        let mut rom = Vec::new();

        // Write header
        rom.extend_from_slice(&NES_MAGIC);
        rom.push(prg_pages);
        rom.push(chr_pages);
        rom.push(flags6);
        rom.push(flags7);
        rom.extend_from_slice(&[0; 8]); // Rest of header (zeros)

        // Write PRG ROM
        rom.extend_from_slice(&vec![0xFF; prg_pages as usize * PRG_ROM_PAGE_SIZE]);

        // Write CHR ROM
        rom.extend_from_slice(&vec![0xAA; chr_pages as usize * CHR_ROM_PAGE_SIZE]);

        rom
    }

    #[test]
    fn test_valid_rom_loading() {
        // Create a test ROM: 2 PRG pages, 1 CHR page, horizontal mirroring, mapper 0
        let rom_data = create_test_rom(2, 1, 0x00, 0x00);

        let temp_file = std::env::temp_dir().join("test_rom.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).expect("Failed to load test ROM");

        assert_eq!(cart.prg_rom_size(), 32768); // 2 * 16KB
        assert_eq!(cart.chr_rom_size(), 8192); // 1 * 8KB
        assert_eq!(cart.mapper, 0);
        assert_eq!(cart.mirroring, Mirroring::Horizontal);
        assert!(!cart.has_battery);
        assert!(!cart.has_trainer);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_vertical_mirroring() {
        let rom_data = create_test_rom(1, 1, 0x01, 0x00); // Flag bit 0 set = vertical

        let temp_file = std::env::temp_dir().join("test_rom_vertical.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).unwrap();
        assert_eq!(cart.mirroring, Mirroring::Vertical);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_four_screen_mirroring() {
        let rom_data = create_test_rom(1, 1, 0x08, 0x00); // Flag bit 3 set = four-screen

        let temp_file = std::env::temp_dir().join("test_rom_fourscreen.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).unwrap();
        assert_eq!(cart.mirroring, Mirroring::FourScreen);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_mapper_number_parsing() {
        // Mapper 3: lower nibble in flags6 upper 4 bits, upper nibble in flags7 upper 4 bits
        let rom_data = create_test_rom(1, 1, 0x30, 0x00); // Mapper 3

        let temp_file = std::env::temp_dir().join("test_rom_mapper.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).unwrap();
        assert_eq!(cart.mapper, 3);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_battery_flag() {
        let rom_data = create_test_rom(1, 1, 0x02, 0x00); // Flag bit 1 set = battery

        let temp_file = std::env::temp_dir().join("test_rom_battery.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).unwrap();
        assert!(cart.has_battery);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_chr_ram_allocation() {
        // ROM with no CHR ROM (0 pages) should allocate 8KB CHR RAM
        let rom_data = create_test_rom(1, 0, 0x00, 0x00);

        let temp_file = std::env::temp_dir().join("test_rom_chrram.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let cart = Cartridge::load(&temp_file).unwrap();
        assert_eq!(cart.chr_rom_size(), CHR_ROM_PAGE_SIZE);

        std::fs::remove_file(temp_file).ok();
    }

    #[test]
    fn test_invalid_magic_number() {
        let mut rom_data = vec![0x00, 0x00, 0x00, 0x00]; // Invalid magic
        rom_data.extend_from_slice(&[1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]); // Rest of header
        rom_data.extend_from_slice(&vec![0; PRG_ROM_PAGE_SIZE + CHR_ROM_PAGE_SIZE]);

        let temp_file = std::env::temp_dir().join("test_rom_invalid.nes");
        std::fs::write(&temp_file, rom_data).unwrap();

        let result = Cartridge::load(&temp_file);
        assert!(result.is_err());

        if let Err(CartridgeError::InvalidFormat(msg)) = result {
            assert!(msg.contains("magic number"));
        } else {
            panic!("Expected InvalidFormat error");
        }

        std::fs::remove_file(temp_file).ok();
    }
}
