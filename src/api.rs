use crate::protocol::*;
use crate::EmulatorCore;
use std::fs;
use std::io::{self, Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::sync::{Arc, Mutex};

pub fn run_uds_server<P: AsRef<Path>>(socket_path: P, mut core: EmulatorCore) -> io::Result<()> {
    let socket_path = socket_path.as_ref();
    if socket_path.exists() {
        let _ = fs::remove_file(socket_path);
    }

    let listener = UnixListener::bind(socket_path)?;
    println!("UDS API listening on {}", socket_path.display());

    for stream in listener.incoming() {
        match stream {
            Ok(mut stream) => {
                println!("Client connected");
                if let Err(err) = handle_client(&mut stream, &mut core) {
                    eprintln!("Client error: {}", err);
                }
                println!("Client disconnected");
            }
            Err(err) => eprintln!("Accept error: {}", err),
        }
    }

    Ok(())
}

pub fn run_uds_server_shared<P: AsRef<Path>>(
    socket_path: P,
    core: Arc<Mutex<EmulatorCore>>,
) -> io::Result<()> {
    let socket_path = socket_path.as_ref();
    if socket_path.exists() {
        let _ = fs::remove_file(socket_path);
    }

    let listener = UnixListener::bind(socket_path)?;
    println!("UDS API listening on {}", socket_path.display());

    for stream in listener.incoming() {
        match stream {
            Ok(mut stream) => {
                println!("Client connected");
                if let Err(err) = handle_client_shared(&mut stream, &core) {
                    eprintln!("Client error: {}", err);
                }
                println!("Client disconnected");
            }
            Err(err) => eprintln!("Accept error: {}", err),
        }
    }

    Ok(())
}

fn handle_client(stream: &mut UnixStream, core: &mut EmulatorCore) -> io::Result<()> {
    loop {
        let req = match read_message(stream) {
            Ok(msg) => msg,
            Err(err) if err.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(err) => return Err(err),
        };

        if req.is_empty() {
            write_error(stream, "empty request")?;
            continue;
        }

        match req[0] {
            OP_PING => {
                write_message(stream, &[RESP_OK])?;
            }
            OP_RESET => {
                core.reset();
                write_message(stream, &[RESP_OK])?;
            }
            OP_STEP => {
                if req.len() < 3 {
                    write_error(stream, "STEP requires action + frame_skip")?;
                    continue;
                }
                let action = req[1];
                let frame_skip = req[2];
                let step = core.step_frames(action, frame_skip);

                let mut out = Vec::with_capacity(1 + 4 + 1 + 8 + 4);
                out.push(RESP_STEP);
                out.extend_from_slice(&0.0f32.to_le_bytes()); // reward placeholder
                out.push(0); // done = false placeholder
                out.extend_from_slice(&step.frame_number.to_le_bytes());
                out.extend_from_slice(&step.frames_advanced.to_le_bytes());
                write_message(stream, &out)?;
            }
            OP_GET_FRAME => {
                let frame = core.frame_rgba();
                let mut out = Vec::with_capacity(1 + 2 + 2 + 1 + frame.len());
                out.push(RESP_FRAME);
                out.extend_from_slice(&SCREEN_WIDTH.to_le_bytes());
                out.extend_from_slice(&SCREEN_HEIGHT.to_le_bytes());
                out.push(FRAME_CHANNELS);
                out.extend_from_slice(&frame);
                write_message(stream, &out)?;
            }
            OP_GET_FRAME_GRAY_80X84 => {
                let frame = core.frame_gray_80x84();
                let mut out = Vec::with_capacity(1 + 2 + 2 + frame.len());
                out.push(RESP_FRAME_GRAY_80X84);
                out.extend_from_slice(&(80u16).to_le_bytes());
                out.extend_from_slice(&(84u16).to_le_bytes());
                out.extend_from_slice(&frame);
                write_message(stream, &out)?;
            }
            OP_GET_RAM => {
                if req.len() < 5 {
                    write_error(stream, "GET_RAM requires start + len")?;
                    continue;
                }
                let start = u16::from_le_bytes([req[1], req[2]]) as usize;
                let len = u16::from_le_bytes([req[3], req[4]]) as usize;
                let ram = core.cpu_ram_snapshot();
                let end = start.saturating_add(len).min(ram.len());
                let slice = if start < end { &ram[start..end] } else { &[] };

                let mut out = Vec::with_capacity(1 + 2 + slice.len());
                out.push(RESP_RAM);
                out.extend_from_slice(&(slice.len() as u16).to_le_bytes());
                out.extend_from_slice(slice);
                write_message(stream, &out)?;
            }
            _ => {
                write_error(stream, "unknown opcode")?;
            }
        }
    }
}

fn handle_client_shared(
    stream: &mut UnixStream,
    core: &Arc<Mutex<EmulatorCore>>,
) -> io::Result<()> {
    loop {
        let req = match read_message(stream) {
            Ok(msg) => msg,
            Err(err) if err.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(err) => return Err(err),
        };

        if req.is_empty() {
            write_error(stream, "empty request")?;
            continue;
        }

        let mut core = core
            .lock()
            .map_err(|_| io::Error::other("emulator core lock poisoned"))?;

        match req[0] {
            OP_PING => {
                write_message(stream, &[RESP_OK])?;
            }
            OP_RESET => {
                core.reset();
                write_message(stream, &[RESP_OK])?;
            }
            OP_STEP => {
                if req.len() < 3 {
                    write_error(stream, "STEP requires action + frame_skip")?;
                    continue;
                }
                let action = req[1];
                let frame_skip = req[2];
                let step = core.step_frames(action, frame_skip);

                let mut out = Vec::with_capacity(1 + 4 + 1 + 8 + 4);
                out.push(RESP_STEP);
                out.extend_from_slice(&0.0f32.to_le_bytes());
                out.push(0);
                out.extend_from_slice(&step.frame_number.to_le_bytes());
                out.extend_from_slice(&step.frames_advanced.to_le_bytes());
                write_message(stream, &out)?;
            }
            OP_GET_FRAME => {
                let frame = core.frame_rgba();
                let mut out = Vec::with_capacity(1 + 2 + 2 + 1 + frame.len());
                out.push(RESP_FRAME);
                out.extend_from_slice(&SCREEN_WIDTH.to_le_bytes());
                out.extend_from_slice(&SCREEN_HEIGHT.to_le_bytes());
                out.push(FRAME_CHANNELS);
                out.extend_from_slice(&frame);
                write_message(stream, &out)?;
            }
            OP_GET_FRAME_GRAY_80X84 => {
                let frame = core.frame_gray_80x84();
                let mut out = Vec::with_capacity(1 + 2 + 2 + frame.len());
                out.push(RESP_FRAME_GRAY_80X84);
                out.extend_from_slice(&(80u16).to_le_bytes());
                out.extend_from_slice(&(84u16).to_le_bytes());
                out.extend_from_slice(&frame);
                write_message(stream, &out)?;
            }
            OP_GET_RAM => {
                if req.len() < 5 {
                    write_error(stream, "GET_RAM requires start + len")?;
                    continue;
                }
                let start = u16::from_le_bytes([req[1], req[2]]) as usize;
                let len = u16::from_le_bytes([req[3], req[4]]) as usize;
                let ram = core.cpu_ram_snapshot();
                let end = start.saturating_add(len).min(ram.len());
                let slice = if start < end { &ram[start..end] } else { &[] };

                let mut out = Vec::with_capacity(1 + 2 + slice.len());
                out.push(RESP_RAM);
                out.extend_from_slice(&(slice.len() as u16).to_le_bytes());
                out.extend_from_slice(slice);
                write_message(stream, &out)?;
            }
            _ => {
                write_error(stream, "unknown opcode")?;
            }
        }
    }
}

fn read_message(stream: &mut UnixStream) -> io::Result<Vec<u8>> {
    let mut len_buf = [0u8; 4];
    stream.read_exact(&mut len_buf)?;
    let len = u32::from_le_bytes(len_buf) as usize;
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf)?;
    Ok(buf)
}

fn write_message(stream: &mut UnixStream, payload: &[u8]) -> io::Result<()> {
    let len = payload.len() as u32;
    stream.write_all(&len.to_le_bytes())?;
    stream.write_all(payload)?;
    stream.flush()
}

fn write_error(stream: &mut UnixStream, msg: &str) -> io::Result<()> {
    let mut out = Vec::with_capacity(1 + msg.len());
    out.push(RESP_ERROR);
    out.extend_from_slice(msg.as_bytes());
    write_message(stream, &out)
}
