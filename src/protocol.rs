pub const OP_RESET: u8 = 1;
pub const OP_STEP: u8 = 2;
pub const OP_GET_FRAME: u8 = 3;
pub const OP_GET_RAM: u8 = 4;
pub const OP_PING: u8 = 5;
pub const OP_GET_FRAME_GRAY_80X84: u8 = 6;

pub const RESP_OK: u8 = 101;
pub const RESP_STEP: u8 = 102;
pub const RESP_FRAME: u8 = 103;
pub const RESP_RAM: u8 = 104;
pub const RESP_FRAME_GRAY_80X84: u8 = 105;
pub const RESP_ERROR: u8 = 255;

pub const SCREEN_WIDTH: u16 = 256;
pub const SCREEN_HEIGHT: u16 = 240;
pub const FRAME_CHANNELS: u8 = 4;
