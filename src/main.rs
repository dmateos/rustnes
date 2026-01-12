mod bus;
mod cartridge;
mod cpu;
mod ppu;

use bus::Bus;
use cartridge::Cartridge;
use cpu::Cpu;
use pixels::{Pixels, SurfaceTexture};
use std::env;
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
}

impl EmulatorApp {
    fn new() -> Self {
        EmulatorApp {
            window_ref: None,
            pixels: None,
            cpu: None,
            controller1: 0,
        }
    }

    fn set_cpu(&mut self, cpu: Cpu) {
        self.cpu = Some(cpu);
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

                // Step CPU once and get cycles consumed
                let cpu_cycles = cpu.step();

                // Step PPU 3 times for each CPU cycle (PPU runs at 3x CPU speed)
                for _ in 0..(cpu_cycles * 3) {
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
    app.set_cpu(cpu);

    // Run the application
    event_loop
        .run_app(&mut app)
        .expect("Failed to run event loop");
}
