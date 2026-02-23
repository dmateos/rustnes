use crate::cartridge::{Cartridge, CartridgeError};
use crate::cpu::Cpu;
use crate::bus::Bus;
use std::path::Path;

#[derive(Debug, Clone, Copy)]
pub struct StepInfo {
    pub frames_advanced: u32,
    pub frame_number: u64,
}

pub struct EmulatorCore {
    cpu: Cpu,
}

impl EmulatorCore {
    pub fn from_cartridge(cartridge: Cartridge) -> Self {
        let bus = Bus::with_cartridge(cartridge);
        let mut cpu = Cpu::new(bus);
        cpu.reset();
        Self { cpu }
    }

    pub fn from_rom_path<P: AsRef<Path>>(path: P) -> Result<Self, CartridgeError> {
        let cartridge = Cartridge::load(path)?;
        Ok(Self::from_cartridge(cartridge))
    }

    pub fn reset(&mut self) {
        self.cpu.reset();
    }

    pub fn set_controller1(&mut self, buttons: u8) {
        self.cpu.bus.set_controller1(buttons);
    }

    pub fn step_until_frame(&mut self) {
        loop {
            if self.cpu.bus.ppu.poll_nmi() {
                self.cpu.nmi();
            }

            if self.cpu.bus.apu.poll_irq() {
                self.cpu.irq();
            }

            let cpu_cycles = self.cpu.step();

            for _ in 0..cpu_cycles {
                self.cpu.bus.apu.step();
            }

            for _ in 0..(cpu_cycles * 3) {
                if self.cpu.bus.ppu.check_a12_rise() && self.cpu.bus.mmc3_clock_irq() {
                    self.cpu.irq();
                }

                if self.cpu.bus.ppu.step() {
                    return;
                }
            }
        }
    }

    pub fn step_frames(&mut self, action: u8, frames: u8) -> StepInfo {
        self.set_controller1(action);
        let count = frames.max(1) as u32;
        for _ in 0..count {
            self.step_until_frame();
        }
        StepInfo {
            frames_advanced: count,
            frame_number: self.cpu.bus.ppu.frame_number(),
        }
    }

    pub fn frame_rgba(&self) -> Vec<u8> {
        self.cpu.bus.ppu.get_framebuffer_rgba()
    }

    pub fn frame_number(&self) -> u64 {
        self.cpu.bus.ppu.frame_number()
    }

    /// Return a compact grayscale observation (80x84) for RL clients.
    ///
    /// This downsamples the full 256x240 RGBA framebuffer by taking every 3rd pixel.
    pub fn frame_gray_80x84(&self) -> Vec<u8> {
        let rgba = self.frame_rgba();
        let width = 256usize;
        let height = 240usize;
        let mut out = Vec::with_capacity(80 * 84);

        for y in (0..height).step_by(3).take(80) {
            for x in (0..width).step_by(3).take(84) {
                let idx = (y * width + x) * 4;
                let r = rgba[idx] as u16;
                let g = rgba[idx + 1] as u16;
                let b = rgba[idx + 2] as u16;
                out.push(((r + g + b) / 3) as u8);
            }
        }

        out
    }

    pub fn cpu(&self) -> &Cpu {
        &self.cpu
    }

    pub fn cpu_mut(&mut self) -> &mut Cpu {
        &mut self.cpu
    }

    pub fn cpu_ram_snapshot(&self) -> Vec<u8> {
        self.cpu.bus.cpu_ram().to_vec()
    }

    pub fn set_apu_sample_rate(&mut self, sample_rate: f64) {
        self.cpu.bus.apu.set_sample_rate(sample_rate);
    }
}
