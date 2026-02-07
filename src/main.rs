mod apu;
mod bus;
mod cartridge;
mod cpu;
mod ppu;

use bus::Bus;
use cartridge::Cartridge;
use cpu::Cpu;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::Stream;
use pixels::{Pixels, SurfaceTexture};
use std::env;
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

const SCREEN_WIDTH: u32 = 256;
const SCREEN_HEIGHT: u32 = 240;
const SCALE: u32 = 3; // 3x scaling for better visibility

/// Controller button bit masks
const BTN_A: u8 = 0x01;
const BTN_B: u8 = 0x02;
const BTN_SELECT: u8 = 0x04;
const BTN_START: u8 = 0x08;
const BTN_UP: u8 = 0x10;
const BTN_DOWN: u8 = 0x20;
const BTN_LEFT: u8 = 0x40;
const BTN_RIGHT: u8 = 0x80;

/// Application state for the emulator
struct EmulatorApp {
    window_ref: Option<&'static Window>,
    pixels: Option<Pixels<'static>>,
    cpu: Option<Cpu>,
    /// Current controller 1 button state
    controller1: u8,
    /// Audio output stream (kept alive for audio playback)
    _audio_stream: Option<Stream>,
}

impl EmulatorApp {
    fn new() -> Self {
        EmulatorApp {
            window_ref: None,
            pixels: None,
            cpu: None,
            controller1: 0,
            _audio_stream: None,
        }
    }

    fn set_cpu(&mut self, cpu: Cpu) {
        self.cpu = Some(cpu);
    }

    /// Initialize audio output stream.
    ///
    /// Creates a cpal audio stream that pulls samples from the APU's sample buffer.
    fn init_audio(&mut self, sample_buffer: Arc<Mutex<VecDeque<f32>>>) {
        // Get default audio output device
        let host = cpal::default_host();
        let device = match host.default_output_device() {
            Some(d) => d,
            None => {
                eprintln!("No audio output device available");
                return;
            }
        };

        // Use default output config
        let config = match device.default_output_config() {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to get audio config: {}", e);
                return;
            }
        };

        println!("Audio output: {} Hz, {} channels", config.sample_rate().0, config.channels());

        // Build the audio stream
        let stream = match config.sample_format() {
            cpal::SampleFormat::F32 => self.build_audio_stream::<f32>(&device, &config.into(), sample_buffer),
            cpal::SampleFormat::I16 => self.build_audio_stream::<i16>(&device, &config.into(), sample_buffer),
            cpal::SampleFormat::U16 => self.build_audio_stream::<u16>(&device, &config.into(), sample_buffer),
            _ => {
                eprintln!("Unsupported audio sample format");
                return;
            }
        };

        // Start playback
        if let Some(ref stream) = stream {
            if let Err(e) = stream.play() {
                eprintln!("Failed to start audio stream: {}", e);
                return;
            }
            println!("Audio stream started successfully");
        }

        self._audio_stream = stream;
    }

    /// Build an audio stream for a specific sample format.
    fn build_audio_stream<T>(
        &self,
        device: &cpal::Device,
        config: &cpal::StreamConfig,
        sample_buffer: Arc<Mutex<VecDeque<f32>>>,
    ) -> Option<Stream>
    where
        T: cpal::Sample + cpal::SizedSample + cpal::FromSample<f32>,
    {
        let channels = config.channels as usize;

        // Track last sample to avoid discontinuities when buffer is empty
        let mut last_sample = 0.0f32;

        let stream = device.build_output_stream(
            config,
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                // Fill output buffer with samples from APU
                if let Ok(mut buffer) = sample_buffer.lock() {
                    // Wait for buffer to warm up (at least 512 samples ~10ms at 48kHz)
                    // to prevent initial underruns
                    if buffer.len() < 512 {
                        // Output silence during warmup
                        for frame in data.chunks_mut(channels) {
                            let value: T = cpal::Sample::from_sample(0.0f32);
                            for out in frame.iter_mut() {
                                *out = value;
                            }
                        }
                        return;
                    }

                    for frame in data.chunks_mut(channels) {
                        // Get next sample from buffer, or hold last sample if buffer is empty
                        // This prevents pops from discontinuities
                        let sample = buffer.pop_front().unwrap_or(last_sample);
                        last_sample = sample;

                        // Convert to output format and duplicate for all channels
                        let value: T = cpal::Sample::from_sample(sample);
                        for out in frame.iter_mut() {
                            *out = value;
                        }
                    }
                }
            },
            |err| eprintln!("Audio stream error: {}", err),
            None,
        );

        match stream {
            Ok(s) => Some(s),
            Err(e) => {
                eprintln!("Failed to build audio stream: {}", e);
                None
            }
        }
    }

    /// Map a key code to controller button mask
    fn key_to_button(key: KeyCode) -> Option<u8> {
        match key {
            KeyCode::KeyX => Some(BTN_A),      // X = A
            KeyCode::KeyZ => Some(BTN_B),      // Z = B
            KeyCode::ShiftRight => Some(BTN_SELECT),
            KeyCode::Enter => Some(BTN_START),
            KeyCode::ArrowUp => Some(BTN_UP),
            KeyCode::ArrowDown => Some(BTN_DOWN),
            KeyCode::ArrowLeft => Some(BTN_LEFT),
            KeyCode::ArrowRight => Some(BTN_RIGHT),
            _ => None,
        }
    }

    fn render_frame(&mut self) {
        if let (Some(cpu), Some(pixels)) = (&mut self.cpu, &mut self.pixels) {
            // Update controller state
            cpu.bus.set_controller1(self.controller1);

            // Run emulation until a frame completes
            let mut nmi_triggered = false;

            loop {
                // Check for NMI from PPU
                if cpu.bus.ppu.poll_nmi() {
                    if !nmi_triggered {
                        nmi_triggered = true;
                    }
                    cpu.nmi();
                }

                // Check for IRQ from APU frame counter
                if cpu.bus.apu.poll_irq() {
                    cpu.irq();
                }

                // Step CPU once and get cycles consumed
                let cpu_cycles = cpu.step();

                // Step APU for each CPU cycle (APU runs at CPU speed)
                for _ in 0..cpu_cycles {
                    cpu.bus.apu.step();
                }

                // Step PPU 3 times for each CPU cycle (PPU runs at 3x CPU speed)
                for _ in 0..(cpu_cycles * 3) {
                    // Check for MMC3 IRQ (A12 rise during rendering)
                    if cpu.bus.ppu.check_a12_rise() {
                        if cpu.bus.mmc3_clock_irq() {
                            cpu.irq();
                        }
                    }

                    if cpu.bus.ppu.step() {
                        // Frame completed
                        // Get the framebuffer from PPU and render it
                        let framebuffer_rgba = cpu.bus.ppu.get_framebuffer_rgba();
                        let frame = pixels.frame_mut();

                        // Copy RGBA data to pixels buffer
                        frame.copy_from_slice(&framebuffer_rgba);

                        // Render to window
                        if let Err(e) = pixels.render() {
                            eprintln!("Render error: {}", e);
                        }

                        return;
                    }
                }
            }
        }
    }
}

impl ApplicationHandler for EmulatorApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window_ref.is_none() {
            let window_attributes = Window::default_attributes()
                .with_title("RustNES Emulator")
                .with_inner_size(LogicalSize::new(
                    SCREEN_WIDTH * SCALE,
                    SCREEN_HEIGHT * SCALE,
                ))
                .with_resizable(true);

            match event_loop.create_window(window_attributes) {
                Ok(window) => {
                    // Leak the window to get a 'static reference
                    // This is safe because the window lives for the entire program duration
                    let window_ref: &'static Window = Box::leak(Box::new(window));

                    // Create Pixels framebuffer using 'static reference
                    // Use window's inner size for the surface (for fullscreen scaling)
                    let window_size = window_ref.inner_size();
                    let surface_texture =
                        SurfaceTexture::new(window_size.width, window_size.height, window_ref);

                    match Pixels::new(SCREEN_WIDTH, SCREEN_HEIGHT, surface_texture) {
                        Ok(pixels) => {
                            self.pixels = Some(pixels);
                            self.window_ref = Some(window_ref);
                            println!("Window and display initialized successfully");
                        }
                        Err(e) => {
                            eprintln!("Failed to create Pixels: {}", e);
                            event_loop.exit();
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Failed to create window: {}", e);
                    event_loop.exit();
                }
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(key) = event.physical_key {
                    // Handle escape to exit
                    if key == KeyCode::Escape && event.state == ElementState::Pressed {
                        event_loop.exit();
                        return;
                    }

                    // Handle controller buttons
                    if let Some(button) = Self::key_to_button(key) {
                        match event.state {
                            ElementState::Pressed => self.controller1 |= button,
                            ElementState::Released => self.controller1 &= !button,
                        }
                    }
                }
            }
            WindowEvent::Resized(new_size) => {
                if let Some(pixels) = &mut self.pixels {
                    let _ = pixels.resize_surface(new_size.width, new_size.height);
                }
            }
            WindowEvent::RedrawRequested => {
                self.render_frame();
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        // Request a redraw to keep the emulation running
        if let Some(window_ref) = self.window_ref {
            window_ref.request_redraw();
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: {} <rom_file.nes>", args[0]);
        std::process::exit(1);
    }

    let rom_path = &args[1];

    // Load the ROM
    let cartridge = match Cartridge::load(rom_path) {
        Ok(cart) => {
            println!("Successfully loaded ROM: {}", rom_path);
            println!(
                "PRG ROM size: {} bytes ({} KB)",
                cart.prg_rom_size(),
                cart.prg_rom_size() / 1024
            );
            println!(
                "CHR ROM size: {} bytes ({} KB)",
                cart.chr_rom_size(),
                cart.chr_rom_size() / 1024
            );
            println!("Mapper: {}", cart.mapper);
            println!("Mirroring: {:?}", cart.mirroring);
            cart
        }
        Err(e) => {
            eprintln!("Error loading ROM: {:?}", e);
            std::process::exit(1);
        }
    };

    // Create bus and CPU
    let bus = Bus::with_cartridge(cartridge);
    let mut cpu = Cpu::new(bus);
    cpu.reset();

    println!("CPU initialized");
    println!("Reset vector points to: ${:04X}", cpu.pc);
    println!("Starting emulation...");

    // Create event loop and application
    let event_loop = EventLoop::new().expect("Failed to create event loop");
    let mut app = EmulatorApp::new();

    // Configure APU sample rate to match audio device
    let host = cpal::default_host();
    if let Some(device) = host.default_output_device() {
        if let Ok(config) = device.default_output_config() {
            let sample_rate = config.sample_rate().0 as f64;
            cpu.bus.apu.set_sample_rate(sample_rate);
        }
    }

    // Get sample buffer reference before moving CPU
    let sample_buffer = cpu.bus.apu.get_sample_buffer();

    app.set_cpu(cpu);
    app.init_audio(sample_buffer);

    // Run the application
    event_loop
        .run_app(&mut app)
        .expect("Failed to run event loop");
}
