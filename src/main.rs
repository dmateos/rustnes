use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::Stream;
use font8x8::{UnicodeFonts, BASIC_FONTS};
use pixels::{Pixels, SurfaceTexture};
use rustnes::api::run_uds_server_shared;
use rustnes::cartridge::Cartridge;
use rustnes::cpu::Cpu;
use rustnes::EmulatorCore;
use std::env;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::thread;
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
    core: Option<Arc<Mutex<EmulatorCore>>>,
    /// Current controller 1 button state
    controller1: u8,
    /// Audio output stream (kept alive for audio playback)
    _audio_stream: Option<Stream>,
    /// Show debug overlay (toggle with ~)
    show_debug: bool,
    /// Speed multiplier (1.0 = normal, 0.001 = ~1Hz, 10.0 = 10x speed)
    speed_multiplier: f64,
    /// Accumulated cycles for speed control
    cycle_debt: f64,
}

impl EmulatorApp {
    fn new() -> Self {
        EmulatorApp {
            window_ref: None,
            pixels: None,
            core: None,
            controller1: 0,
            _audio_stream: None,
            show_debug: false,
            speed_multiplier: 1.0, // Start at normal speed
            cycle_debt: 0.0,
        }
    }

    fn set_core(&mut self, core: Arc<Mutex<EmulatorCore>>) {
        self.core = Some(core);
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


    /// Draws a semi-transparent debug overlay on the pixel buffer
    fn draw_debug_overlay(frame: &mut [u8], cpu: &mut Cpu, speed_multiplier: f64) {
        // Draw a semi-transparent dark background box for the debug info
        // Box position: x=5, y=5, width=200, height=105 (expanded for speed info)
        for y in 5..110 {
            for x in 5..205 {
                let offset = ((y * SCREEN_WIDTH as usize) + x) * 4;
                if offset + 3 < frame.len() {
                    // Darken the background (semi-transparent black)
                    frame[offset] = frame[offset] / 3;         // R
                    frame[offset + 1] = frame[offset + 1] / 3; // G
                    frame[offset + 2] = frame[offset + 2] / 3; // B
                    frame[offset + 3] = 255;                   // A
                }
            }
        }

        // Helper to draw text using font8x8 bitmap font
        let mut draw_text = |text: &str, x: usize, y: usize, r: u8, g: u8, b: u8| {
            for (i, ch) in text.chars().enumerate() {
                let char_x = x + (i * 8); // 8 pixels wide per character
                if char_x + 8 <= SCREEN_WIDTH as usize && y + 8 <= SCREEN_HEIGHT as usize {
                    // Get the glyph from font8x8 library
                    if let Some(glyph) = BASIC_FONTS.get(ch) {
                        // Draw each row of the 8x8 character
                        for (row, &byte) in glyph.iter().enumerate() {
                            // Draw 8 pixels per row (checking each bit)
                            for col in 0..8 {
                                if (byte & (1 << col)) != 0 {
                                    let px = char_x + col;
                                    let py = y + row;
                                    let offset = ((py * SCREEN_WIDTH as usize) + px) * 4;

                                    if offset + 3 < frame.len() {
                                        frame[offset] = r;
                                        frame[offset + 1] = g;
                                        frame[offset + 2] = b;
                                        frame[offset + 3] = 255;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        };

        // Draw CPU state
        draw_text(&format!("A:{:02X} X:{:02X} Y:{:02X}", cpu.a, cpu.x, cpu.y), 8, 8, 0, 255, 0);
        draw_text(&format!("PC:{:04X} SP:{:02X}", cpu.pc, cpu.sp), 8, 18, 0, 255, 0);

        // Draw flags
        let flags = cpu.status;
        let flag_str = format!("{}{}{}{}{}{}{}{}",
            if flags & 0x80 != 0 { 'N' } else { 'n' },
            if flags & 0x40 != 0 { 'V' } else { 'v' },
            if flags & 0x10 != 0 { 'B' } else { 'b' },
            if flags & 0x08 != 0 { 'D' } else { 'd' },
            if flags & 0x04 != 0 { 'I' } else { 'i' },
            if flags & 0x02 != 0 { 'Z' } else { 'z' },
            if flags & 0x01 != 0 { 'C' } else { 'c' },
            ' '
        );
        draw_text(&flag_str, 8, 28, 255, 255, 0);

        // Draw next instruction
        let opcode = cpu.bus.read(cpu.pc);
        draw_text(&format!("OP:{:02X}", opcode), 8, 38, 255, 128, 0);

        // Draw cycle count (truncated)
        draw_text(&format!("CY:{}", cpu.cycles % 1000000), 8, 48, 128, 128, 255);

        // Draw speed multiplier
        let speed_text = if speed_multiplier == 0.0 {
            "SPD:PAUSED".to_string()
        } else if speed_multiplier < 0.001 {
            format!("SPD:~{}Hz", (1_790_000.0 * speed_multiplier) as u32)
        } else {
            format!("SPD:{:.2}x", speed_multiplier)
        };
        draw_text(&speed_text, 8, 58, 0, 255, 255);

        // Draw hints
        draw_text("~ hide", 8, 73, 100, 100, 100);
        draw_text("- slow  + fast", 8, 83, 100, 100, 100);
        draw_text("[ 1Hz   ] norm", 8, 93, 100, 100, 100);
        draw_text("0 pause", 8, 103, 100, 100, 100);
    }

    fn render_frame(&mut self) {
        if let (Some(core), Some(pixels)) = (&self.core, &mut self.pixels) {
            let Ok(mut core) = core.lock() else {
                eprintln!("Failed to lock emulator core");
                return;
            };

            // Update controller state
            core.set_controller1(self.controller1);

            // Apply speed control using frame units for shared core stepping.
            self.cycle_debt += self.speed_multiplier;

            if self.cycle_debt >= 1.0 {
                while self.cycle_debt >= 1.0 {
                    core.step_until_frame();
                    self.cycle_debt -= 1.0;
                }
            }

            let framebuffer_rgba = core.frame_rgba();
            let frame = pixels.frame_mut();
            frame.copy_from_slice(&framebuffer_rgba);

            if self.show_debug {
                let cpu = core.cpu_mut();
                Self::draw_debug_overlay(frame, cpu, self.speed_multiplier);
            }

            if let Err(e) = pixels.render() {
                eprintln!("Render error: {}", e);
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

                    // Toggle debug overlay with ~ (backquote)
                    if key == KeyCode::Backquote && event.state == ElementState::Pressed {
                        self.show_debug = !self.show_debug;
                        return;
                    }

                    // Speed controls
                    if event.state == ElementState::Pressed {
                        match key {
                            KeyCode::Minus => {
                                // Decrease speed (halve it)
                                self.speed_multiplier = (self.speed_multiplier * 0.5).max(0.0001);
                                println!("Speed: {:.4}x", self.speed_multiplier);
                                return;
                            }
                            KeyCode::Equal => {
                                // Increase speed (double it)
                                self.speed_multiplier = (self.speed_multiplier * 2.0).min(10.0);
                                println!("Speed: {:.4}x", self.speed_multiplier);
                                return;
                            }
                            KeyCode::BracketLeft => {
                                // Very slow (1Hz debug mode)
                                self.speed_multiplier = 0.00056; // ~1Hz (1.79M / 0.00056 ≈ 1)
                                println!("Speed: ~1 Hz (debug mode)");
                                return;
                            }
                            KeyCode::BracketRight => {
                                // Normal speed
                                self.speed_multiplier = 1.0;
                                println!("Speed: 1.0x (normal)");
                                return;
                            }
                            KeyCode::Digit0 => {
                                // Pause
                                self.speed_multiplier = 0.0;
                                println!("Speed: PAUSED");
                                return;
                            }
                            _ => {}
                        }
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
        eprintln!("Usage: {} <rom_file.nes> [--api-socket <path>]", args[0]);
        std::process::exit(1);
    }

    let rom_path = &args[1];
    let mut api_socket: Option<String> = None;
    let mut i = 2;
    while i < args.len() {
        match args[i].as_str() {
            "--api-socket" => {
                if i + 1 >= args.len() {
                    eprintln!("Missing value for --api-socket");
                    std::process::exit(1);
                }
                api_socket = Some(args[i + 1].clone());
                i += 2;
            }
            other => {
                eprintln!("Unknown argument: {}", other);
                eprintln!("Usage: {} <rom_file.nes> [--api-socket <path>]", args[0]);
                std::process::exit(1);
            }
        }
    }

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

    // Create shared emulator core
    let mut core = EmulatorCore::from_cartridge(cartridge);

    println!("CPU initialized");
    println!("Reset vector points to: ${:04X}", core.cpu().pc);
    println!("Starting emulation...");

    // Create event loop and application
    let event_loop = EventLoop::new().expect("Failed to create event loop");
    let mut app = EmulatorApp::new();

    // Configure APU sample rate to match audio device
    let host = cpal::default_host();
    if let Some(device) = host.default_output_device() {
        if let Ok(config) = device.default_output_config() {
            let sample_rate = config.sample_rate().0 as f64;
            core.set_apu_sample_rate(sample_rate);
        }
    }

    // Get sample buffer reference before sharing core
    let sample_buffer = core.cpu().bus.apu.get_sample_buffer();
    let shared_core = Arc::new(Mutex::new(core));

    if let Some(socket_path) = api_socket {
        let api_core = Arc::clone(&shared_core);
        thread::spawn(move || {
            if let Err(err) = run_uds_server_shared(socket_path, api_core) {
                eprintln!("UDS API server error: {}", err);
            }
        });
    }

    app.set_core(shared_core);
    app.init_audio(sample_buffer);

    // Run the application
    event_loop
        .run_app(&mut app)
        .expect("Failed to run event loop");
}
