// Minimal APU stub for compilation
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

pub struct Apu {
    sample_buffer: Arc<Mutex<VecDeque<f32>>>,
}

impl Apu {
    pub fn new() -> Self {
        Apu {
            sample_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(4096))),
        }
    }

    pub fn read_register(&mut self, _addr: u8) -> u8 {
        0
    }

    pub fn write_register(&mut self, _addr: u8, _value: u8) {
    }

    pub fn step(&mut self) {
    }

    pub fn poll_irq(&mut self) -> bool {
        false
    }

    pub fn get_sample_buffer(&self) -> Arc<Mutex<VecDeque<f32>>> {
        Arc::clone(&self.sample_buffer)
    }

    pub fn set_sample_rate(&mut self, _sample_rate: f64) {
    }
}

impl Default for Apu {
    fn default() -> Self {
        Self::new()
    }
}
