# RustNES

A cycle-accurate Nintendo Entertainment System (NES) emulator written in Rust with full audio support, real-time debugging capabilities, and variable speed control.

## Features

### Core Emulation
- **Cycle-accurate CPU** - Full MOS 6502 instruction set with accurate timing
- **PPU (2C02)** - Complete graphics processing with scanline-accurate rendering
- **APU (2A03)** - Full audio processing unit with all 4 main channels:
  - 2 Pulse channels (melody/harmony)
  - 1 Triangle channel (bass)
  - 1 Noise channel (percussion/effects)
- **Mapper Support**:
  - Mapper 0 (NROM) - Basic games
  - Mapper 1 (MMC1) - The Legend of Zelda, etc.
  - Mapper 4 (MMC3) - Super Mario Bros. 3, Mega Man, etc.

### Audio System
- **48 kHz audio output** using cpal library
- **Non-linear mixing** matching original NES hardware
- **High-pass filter** (90 Hz) to remove DC offset
- **Low-pass filter** (14 kHz) to reduce aliasing
- **Frame counter** with 4-step and 5-step modes
- **IRQ support** for accurate timing

### Debug Features
- **Real-time debug overlay** (Toggle with `~`)
  - CPU registers (A, X, Y, PC, SP)
  - CPU flags (N, V, B, D, I, Z, C)
  - Current opcode
  - Cycle counter
  - Current speed multiplier
- **60 FPS overlay updates** even at slow CPU speeds
- **Variable speed control**:
  - Slow motion (0.0001x minimum)
  - Fast forward (10x maximum)
  - 1Hz debug mode for instruction-by-instruction execution
  - Pause functionality

### Controls
#### Game Controls
- **Arrow Keys** - D-pad (Up, Down, Left, Right)
- **Z** - A button
- **X** - B button
- **Enter** - Start
- **Right Shift** - Select
- **Escape** - Exit emulator

#### Debug Controls
- **`~` (Backquote)** - Toggle debug overlay
- **`-` (Minus)** - Slow down (halve speed)
- **`+` (Equal)** - Speed up (double speed)
- **`[` (Left Bracket)** - 1Hz debug mode
- **`]` (Right Bracket)** - Normal speed (1.0x)
- **`0` (Zero)** - Pause

## Building

### Prerequisites
- Rust 1.70 or later (2024 edition)
- Cargo (comes with Rust)

### Dependencies
The following crates are used:
- `pixels` - Fast pixel buffer rendering
- `winit` - Cross-platform window creation
- `cpal` - Cross-platform audio output
- `font8x8` - Bitmap font for debug overlay
- `thiserror` - Error handling

### Build Commands

```bash
# Debug build
cargo build

# Release build (recommended for performance)
cargo build --release

# Run directly
cargo run --release <path_to_rom.nes>
```

## Usage

### Running a ROM

```bash
cargo run --release mario.nes
```

The emulator accepts any `.nes` ROM file that uses a supported mapper (0, 1, or 4).

### Debug Mode Workflow

1. Launch a game: `cargo run --release zelda.nes`
2. Press `~` to show the debug overlay
3. Press `[` to enter 1Hz debug mode
4. Watch the CPU execute one instruction per second
5. Observe register changes, flag updates, and opcode execution in real-time
6. Press `]` to return to normal speed
7. Press `-` or `+` to fine-tune speed

### Tested Games

- Super Mario Bros.
- Super Mario Bros. 3
- The Legend of Zelda
- Mega Man series
- And many more...

## Architecture

### Bus-Centric Design
The emulator uses a bus-centric architecture where the `Bus` owns all major components:
- CPU (MOS 6502)
- PPU (Picture Processing Unit)
- APU (Audio Processing Unit)
- Cartridge/Mapper
- Controller state

Components communicate through the bus using memory-mapped I/O.

### Timing
- **CPU**: 1.79 MHz (1,789,773 cycles per second)
- **PPU**: 3x CPU speed (5.37 MHz)
- **APU**: CPU speed (1.79 MHz)
- **Audio**: 48 kHz sample rate (~37.3 CPU cycles per sample)
- **Display**: 60 FPS (262 scanlines per frame)

### Memory Map
```
$0000-$07FF: Internal RAM (2KB)
$0800-$1FFF: Mirrors of RAM
$2000-$2007: PPU registers
$2008-$3FFF: Mirrors of PPU registers
$4000-$4013: APU registers
$4014:       PPU OAM DMA
$4015:       APU status/control
$4016:       Controller 1
$4017:       Controller 2 / APU frame counter
$4018-$401F: APU/IO functionality
$4020-$FFFF: Cartridge space (PRG-ROM, PRG-RAM, mapper registers)
```

## Implementation Details

### APU Architecture
Each audio channel is implemented as a separate module with:
- `write_register(addr, value)` - Handle register writes from CPU
- `step_timer()` - Called every CPU cycle to advance timers
- `output()` - Return current amplitude (0-15)
- `clock_envelope()`, `clock_length_counter()`, etc. - Frame counter clocking

### Sample Generation
The APU accumulates CPU cycles and generates audio samples when the threshold is reached:
```rust
self.sample_accumulator += 1.0;
if self.sample_accumulator >= CYCLES_PER_SAMPLE {
    // Generate sample using non-linear mixing
    let pulse_out = 95.88 / ((8128.0 / (pulse1 + pulse2)) + 100.0);
    let tnd_out = 159.79 / ((1.0 / (tri/8227.0 + noise/12241.0)) + 100.0);
    let sample = pulse_out + tnd_out;
    // Apply filters and push to buffer
}
```

### Speed Control
Speed is controlled via cycle debt accumulation:
- Each frame adds `speed_multiplier` to `cycle_debt`
- CPU only executes when `cycle_debt >= 1.0`
- Allows precise control from pause (0.0x) to fast-forward (10.0x)
- Display updates at 60 FPS regardless of CPU speed

## Project Structure

```
src/
├── main.rs           - Entry point, window management, rendering loop
├── cpu.rs            - MOS 6502 CPU implementation
├── ppu.rs            - Picture Processing Unit (2C02)
├── apu.rs            - Audio Processing Unit (2A03)
│   ├── pulse.rs      - Pulse channels (2)
│   ├── triangle.rs   - Triangle channel
│   └── noise.rs      - Noise channel
├── bus.rs            - System bus and memory management
├── cartridge.rs      - ROM loading and cartridge management
└── shader.wgsl       - GPU shader for display rendering
```

## Performance

- **CPU Usage**: ~5-10% on modern hardware (release build)
- **Frame Rate**: Stable 60 FPS
- **Audio Latency**: ~85ms (4096 sample buffer)
- **Memory Usage**: ~50MB typical

## Known Limitations

- DMC (Delta Modulation Channel) not yet implemented
- No save state functionality
- No rewind feature
- Limited to mappers 0, 1, and 4

## Future Enhancements

Potential features for future development:
- Additional mapper support (UNROM, CNROM, etc.)
- DMC audio channel
- Save states
- Rewind functionality
- Screenshot capture
- TAS (Tool-Assisted Speedrun) recording
- Netplay support
- APU audio visualization

## Troubleshooting

### Audio Issues
If you experience audio crackling or popping:
- Ensure you're using a release build (`--release` flag)
- Check that your audio device supports 48 kHz output
- Close other applications using audio

### Performance Issues
If the emulator runs slowly:
- Always use release builds: `cargo build --release`
- Check CPU usage - debug builds can be 10x slower
- Disable the debug overlay if not needed

### ROM Not Loading
- Verify the ROM file is not corrupted
- Check that the mapper is supported (0, 1, or 4)
- Ensure the file has `.nes` extension

## License

This project is provided as-is for educational purposes.

## Credits

Developed as a learning project to understand:
- NES hardware architecture
- Emulation techniques
- Rust systems programming
- Real-time audio processing
- Cycle-accurate timing

Special thanks to the NES development community for extensive documentation:
- NESDev Wiki
- 6502.org
- Various NES emulator source codes

## Contributing

This is a personal learning project, but feedback and suggestions are welcome.

## Resources

- [NESDev Wiki](http://wiki.nesdev.com/) - Comprehensive NES documentation
- [6502 Reference](http://www.6502.org/) - CPU instruction reference
- [NES Audio Guide](https://wiki.nesdev.com/w/index.php/APU) - APU documentation
