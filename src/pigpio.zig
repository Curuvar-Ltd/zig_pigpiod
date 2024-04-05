// zig fmt: off
// DO NOT REMOVE ABOVE LINE -- I strongly dislike the way Zig formats code.

const std = @import( "std" );

const PiGPIO = @This();

const log = std.log.scoped( .pigpiod );

// =============================================================================
//  Our Fields
// =============================================================================

cmd_stream    : Stream = undefined,
notify_stream : Stream = undefined,
notify_handle : u32    = 0,

// =============================================================================
//  Structure Definitions
// =============================================================================

const CBFunction = fn (x: i32) void;

// -----------------------------------------------------------------------------
//  Zig Errors

pub const PiGPIOError = error
{
  init_failed,       // gpioInitialise failed
  bad_user_gpio,     // GPIO not 0-31
  bad_gpio,          // GPIO not 0-53
  bad_mode,          // mode not 0-7
  bad_level,         // level not 0-1
  bad_pud,           // pud not 0-2
  bad_pulsewidth,    // pulsewidth not 0 or 500-2500
  bad_dutycycle,     // dutycycle outside set range
  bad_timer,         // timer not 0-9
  bad_ms,            // ms not 10-60000
  bad_timetype,      // timetype not 0-1
  bad_seconds,       // seconds < 0
  bad_micros,        // micros not 0-999999
  timer_failed,      // gpioSetTimerFunc failed
  bad_wdog_timeout,  // timeout not 0-60000
  no_alert_func,     // DEPRECATED
  bad_clk_periph,    // clock peripheral not 0-1
  bad_clk_source,    // DEPRECATED
  bad_clk_micros,    // clock micros not 1, 2, 4, 5, 8, or 10
  bad_buf_millis,    // buf millis not 100-10000
  bad_dutyrange,     // dutycycle range not 25-40000
  bad_signum,        // signum not 0-63
  bad_pathname,      // can't open pathname
  no_handle,         // no handle available
  bad_handle,        // unknown handle
  bad_if_flags,      // ifFlags > 4
  bad_channel,       // DMA channel not 0-15
  bad_socket_port,   // socket port not 1024-32000
  bad_fifo_command,  // unrecognized fifo command
  bad_seco_channel,  // DMA secondary channel not 0-15
  not_initialised,   // function called before gpioInitialise
  initialised,       // function called after gpioInitialise
  bad_wave_mode,     // waveform mode not 0-3
  bad_cfg_internal,  // bad parameter in gpioCfgInternals call
  bad_wave_baud,     // baud rate not 50-250K(RX)/50-1M(TX)
  too_many_pulses,   // waveform has too many pulses
  too_many_chars,    // waveform has too many chars
  not_serial_gpio,   // no bit bang serial read on GPIO
  bad_serial_struc,  // bad (null) serial structure parameter
  bad_serial_buf,    // bad (null) serial buf parameter
  not_permitted,     // GPIO operation not permitted
  some_permitted,    // one or more GPIO not permitted
  bad_wvsc_commnd,   // bad WVSC subcommand
  bad_wvsm_commnd,   // bad WVSM subcommand
  bad_wvsp_commnd,   // bad WVSP subcommand
  bad_pulselen,      // trigger pulse length not 1-100
  bad_script,        // invalid script
  bad_script_id,     // unknown script pin
  bad_ser_offset,    // add serial data offset > 30 minutes
  gpio_in_use,       // GPIO already in use
  bad_serial_count,  // must read at least a byte at a time
  bad_param_num,     // script parameter id not 0-9
  dup_tag,           // script has duplicate tag
  too_many_tags,     // script has too many tags
  bad_script_cmd,    // illegal script command
  bad_var_num,       // script variable id not 0-149
  no_script_room,    // no more room for scripts
  no_memory,         // can't allocate temporary memory
  sock_read_failed,  // socket read failed
  sock_writ_failed,  // socket write failed
  too_many_param,    // too many script parameters (> 10)
  script_not_ready,  // script initialising
  bad_tag,           // script has unresolved tag
  bad_mics_delay,    // bad MICS delay (too large)
  bad_mils_delay,    // bad MILS delay (too large)
  bad_wave_id,       // non existent wave id
  too_many_cbs,      // No more CBs for waveform
  too_many_ool,      // No more OOL for waveform
  empty_waveform,    // attempt to create an empty waveform
  no_waveform_id,    // no more waveforms
  i2c_open_failed,   // can't open I2C device
  ser_open_failed,   // can't open serial device
  spi_open_failed,   // can't open SPI device
  bad_i2c_bus,       // bad I2C bus
  bad_i2c_addr,      // bad I2C address
  bad_spi_channel,   // bad SPI channel
  bad_flags,         // bad i2c/spi/ser open flags
  bad_spi_speed,     // bad SPI speed
  bad_ser_device,    // bad serial device name
  bad_ser_speed,     // bad serial baud rate
  bad_param,         // bad i2c/spi/ser parameter
  i2c_write_failed,  // i2c write failed
  i2c_read_failed,   // i2c read failed
  bad_spi_count,     // bad SPI count
  ser_write_failed,  // ser write failed
  ser_read_failed,   // ser read failed
  ser_read_no_data,  // ser read no data available
  unknown_command,   // unknown command
  spi_xfer_failed,   // spi xfer/read/write failed
  bad_pointer,       // bad (NULL) pointer
  no_aux_spi,        // no auxiliary SPI on Pi A or B
  not_pwm_gpio,      // GPIO is not in use for PWM
  not_servo_gpio,    // GPIO is not in use for servo pulses
  not_hclk_gpio,     // GPIO has no hardware clock
  not_hpwm_gpio,     // GPIO has no hardware PWM
  bad_hpwm_freq,     // invalid hardware PWM frequency
  bad_hpwm_duty,     // hardware PWM dutycycle not 0-1M
  bad_hclk_freq,     // invalid hardware clock frequency
  bad_hclk_pass,     // need password to use hardware clock 1
  hpwm_illegal,      // illegal, PWM in use for main clock
  bad_databits,      // serial data bits not 1-32
  bad_stopbits,      // serial (half) stop bits not 2-8
  msg_toobig,        // socket/pipe message too big
  bad_malloc_mode,   // bad memory allocation mode
  too_many_segs,     // too many I2C transaction segments
  bad_i2c_seg,       // an I2C transaction segment failed
  bad_smbus_cmd,     // SMBus command not supported by driver
  not_i2c_gpio,      // no bit bang I2C in progress on GPIO
  bad_i2c_wlen,      // bad I2C write length
  bad_i2c_rlen,      // bad I2C read length
  bad_i2c_cmd,       // bad I2C command
  bad_i2c_baud,      // bad I2C baud rate, not 50-500k
  chain_loop_cnt,    // bad chain loop count
  bad_chain_loop,    // empty chain loop
  chain_counter,     // too many chain counters
  bad_chain_cmd,     // bad chain command
  bad_chain_delay,   // bad chain delay micros
  chain_nesting,     // chain counters nested too deeply
  chain_too_big,     // chain is too long
  deprecated,        // deprecated function removed
  bad_ser_invert,    // bit bang serial invert not 0 or 1
  bad_edge,          // bad ISR edge value, not 0-2
  bad_isr_init,      // bad ISR initialisation
  bad_forever,       // loop forever must be last command
  bad_filter,        // bad filter parameter
  bad_pad,           // bad pad number
  bad_strength,      // bad pad drive strength
  fil_open_failed,   // file open failed
  bad_file_mode,     // bad file mode
  bad_file_flag,     // bad file flag
  bad_file_read,     // bad file read
  bad_file_write,    // bad file write
  file_not_ropen,    // file not open for read
  file_not_wopen,    // file not open for write
  bad_file_seek,     // bad file seek
  no_file_match,     // no files match pattern
  no_file_access,    // no permission to access file
  file_is_a_dir,     // file is a directory
  bad_shell_status,  // bad shell return status
  bad_script_name,   // bad script name
  bad_spi_baud,      // bad SPI baud rate, not 50-500k
  not_spi_gpio,      // no bit bang SPI in progress on GPIO
  bad_event_id,      // bad event id
  cmd_interrupted,   // Used by Python
  not_on_bcm2711,    // not available on BCM2711
  only_on_bcm2711,   // only available on BCM2711
  bad_send,
  bad_recv,
  bad_getaddrinfo,
  bad_connect,
  bad_socket,
  bad_noib,
  duplicate_callback,
  bad_malloc,
  bad_callback,
  notify_failed,
  callback_not_found,
  unconnected_pi,
  too_many_pis,
  unknown_error
};

pub const InitError  =    std.net.TcpConnectToHostError
                       || std.mem.Allocator.Error;

pub const WriteError =    std.posix.WriteError;

pub const ReadError  =    std.posix.ReadError
                       || PiGPIOError;

pub const ComError   =    std.posix.WriteError
                       || std.posix.ReadError;

pub const Error      =    std.posix.WriteError
                       || std.posix.ReadError
                       || PiGPIOError;

pub const SPIError   =    Error
                       || error{ SPINotOpen };

// =============================================================================
//  Constant Definitions
// =============================================================================

const Command = enum(u8)
{
  MODES   = 0,
  MODEG   = 1,
  PUD     = 2,
  READ    = 3,
  WRITE   = 4,
  PWM     = 5,
  PRS     = 6,
  PFS     = 7,
  SERVO   = 8,
  WDOG    = 9,
  BR1    = 10,
  BR2    = 11,
  BC1    = 12,
  BC2    = 13,
  BS1    = 14,
  BS2    = 15,
  TICK   = 16,
  HWVER  = 17,
  NO     = 18,
  NB     = 19,
  NP     = 20,
  NC     = 21,
  PRG    = 22,
  PFG    = 23,
  PRRG   = 24,
  HELP   = 25,
  PIGPV  = 26,
  WVCLR  = 27,
  WVAG   = 28,
  WVAS   = 29,
  WVGO   = 30,
  WVGOR  = 31,
  WVBSY  = 32,
  WVHLT  = 33,
  WVSM   = 34,
  WVSP   = 35,
  WVSC   = 36,
  TRIG   = 37,
  PROC   = 38,
  PROCD  = 39,
  PROCR  = 40,
  PROCS  = 41,
  SLRO   = 42,
  SLR    = 43,
  SLRC   = 44,
  PROCP  = 45,
  MICS   = 46,
  MILS   = 47,
  PARSE  = 48,
  WVCRE  = 49,
  WVDEL  = 50,
  WVTX   = 51,
  WVTXR  = 52,
  WVNEW  = 53,

  I2CO   = 54,
  I2CC   = 55,
  I2CRD  = 56,
  I2CWD  = 57,
  I2CWQ  = 58,
  I2CRS  = 59,
  I2CWS  = 60,
  I2CRB  = 61,
  I2CWB  = 62,
  I2CRW  = 63,
  I2CWW  = 64,
  I2CRK  = 65,
  I2CWK  = 66,
  I2CRI  = 67,
  I2CWI  = 68,
  I2CPC  = 69,
  I2CPK  = 70,

  SPIO   = 71,
  SPIC   = 72,
  SPIR   = 73,
  SPIW   = 74,
  SPIX   = 75,

  SERO   = 76,
  SERC   = 77,
  SERRB  = 78,
  SERWB  = 79,
  SERR   = 80,
  SERW   = 81,
  SERDA  = 82,

  GDC    = 83,
  GPW    = 84,

  HC     = 85,
  HP     = 86,

  CF1    = 87,
  CF2    = 88,

  BI2CC  = 89,
  BI2CO  = 90,
  BI2CZ  = 91,

  I2CZ   = 92,

  WVCHA  = 93,

  SLRI   = 94,

  CGI    = 95,
  CSI    = 96,

  FG     = 97,
  FN     = 98,

  NOIB   = 99,  // Inform server that this is the notify stream.

  WVTXM  = 100,
  WVTAT  = 101,

  PADS   = 102,
  PADG   = 103,

  FO     = 104,
  FC     = 105,
  FR     = 106,
  FW     = 107,
  FS     = 108,
  FL     = 109,

  SHELL  = 110,

  BSPIC  = 111,
  BSPIO  = 112,
  BSPIX  = 113,

  BSCX   = 114,

  EVM    = 115,
  EVT    = 116,

  PROCU  = 117,
  WVCAP  = 118,
};

const Header = struct
{
  cmd : u32,
  p1  : u32,
  p2  : u32,
  p3  : u32,
};

// =============================================================================
//  Public Functions
// =============================================================================

// ---------------------------------------------------------------------------
//  Public function: connect
// ---------------------------------------------------------------------------
/// Initialize a PiGPIO structure.
///

pub fn connect( self     : *PiGPIO,
                in_alloc : std.mem.Allocator,
                in_addr  : ?[]const u8,
                in_port  : ?u16 ) InitError!void
{
  const addr = if (in_addr) |addr| addr else "::";
  const port = if (in_port) |port| port else 8888;

  try self.cmd_stream.open( in_alloc,addr, port );
  errdefer self.cmd_stream.close();

  try self.cmd_stream.open( in_alloc,addr, port );
  errdefer self.notify_stream.close();

  // const notify_handle = try pigpiod.notify_stream.doCommandBasic( NOIB, 0, 0, null );


  log.debug( "-- PiGPIO Connected --", .{} );
}

// ---------------------------------------------------------------------------
//  Public function: disconnect
// ---------------------------------------------------------------------------
/// Deinitialize a PiGPIO structure
///
pub fn disconnect( self : *PiGPIO ) void
{
  self.cmd_stream.close();
  self.notify_stream.close();

  log.debug( "-- PiGPIO Disconnected --", .{} );
}

// ---------------------------------------------------------------------------
//  Public function: pin
// ---------------------------------------------------------------------------
//  Return an initialized Pin

pub fn pin( self : *PiGPIO, in_pin : u32 ) Pin
{
  return .{ .gpio = self, .pin = in_pin };
}

// // ---------------------------------------------------------------------------
// //  Public function: spi
// // ---------------------------------------------------------------------------
// ///  Allocate and return an unitialized SPI instance
// ///
// ///  Usage:
// ///     var spi = gpio.spi( alloc, chan, rate, flags );

// pub fn spi( self         : *PiGPIO,
//             in_allocator : std.mem.Allocator,
//             in_channel   : u32,
//             in_bit_rate  : u32,
//             in_flags     : u32 ) (Error||error{OutOfMemory})!*SPI
// {
//   return try PiGPIO.SPI.init( in_allocator,
//                                self,
//                                in_channel,
//                                in_bit_rate,
//                                in_flags );
// }

// -----------------------------------------------------------------------------
//  Public function: Pin.readBank1
// -----------------------------------------------------------------------------

pub fn readBank1( self : PiGPIO ) ComError!u32
{
  return try self.cmd_stream.doCmdBasic( .BR1, 0, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: Pin.readBank2
// -----------------------------------------------------------------------------

pub fn readBank2( self : PiGPIO )  ComError!u32
{
  return try self.cmd_stream.doCmdBasic( .BR2, 0, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: Pin.clearBank1
// -----------------------------------------------------------------------------

pub fn clearBank1(  self : PiGPIO, in_mask : u32 ) Error!void
{
  _ = try self.cmd_stream.doCmd( .BC1, in_mask, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: Pin.clearBank2
// -----------------------------------------------------------------------------

pub fn clearBank2(  self : PiGPIO, in_mask : u32 ) Error!void
{
  _ = try self.cmd_stream.doCmd( .BC2, in_mask, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: Pin.setBank1
// -----------------------------------------------------------------------------

pub fn setBank1(  self : PiGPIO, in_mask : u32 ) Error!void
{
  _ = try self.cmd_stream.doCmd( .BS1, in_mask, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: Pin.setBank2
// -----------------------------------------------------------------------------

pub fn setBank2( self : PiGPIO, in_mask : u32 ) Error!void
{
  _ = try self.cmd_stream.doCmd( .BS2, in_mask, 0, 0, null );
}

// =============================================================================
//  Private Functions
// =============================================================================

  // ---------------------------------------------------------------------------
  //  Function: Stream.doCmd
  // ---------------------------------------------------------------------------

  fn doCmd( self         : *PiGPIO,
            in_cmd       : Command,
            in_p1        : u32,
            in_p2        : u32,
            in_ext_len   : u32,
            in_extension : ?* const anyopaque ) Error!u32
  {
    return self.cmd_stream.doCmd( in_cmd,
                                  in_p1,
                                  in_p2,
                                  in_ext_len,
                                  in_extension );
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.doCmdBasic
  // ---------------------------------------------------------------------------

  fn doCmdBasic( self         : *PiGPIO,
                 in_cmd       : Command,
                 in_p1        : u32,
                 in_p2        : u32,
                 in_ext_len   : u32,
                 in_extension : ?* const anyopaque ) Error!u32
  {
    return self.cmd_stream.doCmdBasic( in_cmd,
                                       in_p1,
                                       in_p2,
                                       in_ext_len,
                                       in_extension );
  }
// -----------------------------------------------------------------------------
//  Function: convertError
// -----------------------------------------------------------------------------

fn convertError( in_err : i32 ) PiGPIOError
{
  switch (in_err)
  {
       -1 => { return error.init_failed; },
       -2 => { return error.bad_user_gpio; },
       -3 => { return error.bad_gpio; },
       -4 => { return error.bad_mode; },
       -5 => { return error.bad_level; },
       -6 => { return error.bad_pud; },
       -7 => { return error.bad_pulsewidth; },
       -8 => { return error.bad_dutycycle; },
       -9 => { return error.bad_timer; },
      -10 => { return error.bad_ms; },
      -11 => { return error.bad_timetype; },
      -12 => { return error.bad_seconds; },
      -13 => { return error.bad_micros; },
      -14 => { return error.timer_failed; },
      -15 => { return error.bad_wdog_timeout; },
      -16 => { return error.no_alert_func; },
      -17 => { return error.bad_clk_periph; },
      -18 => { return error.bad_clk_source; },
      -19 => { return error.bad_clk_micros; },
      -20 => { return error.bad_buf_millis; },
      -21 => { return error.bad_dutyrange; },
      -22 => { return error.bad_signum; },
      -23 => { return error.bad_pathname; },
      -24 => { return error.no_handle; },
      -25 => { return error.bad_handle; },
      -26 => { return error.bad_if_flags; },
      -27 => { return error.bad_channel; },
      -28 => { return error.bad_socket_port; },
      -29 => { return error.bad_fifo_command; },
      -30 => { return error.bad_seco_channel; },
      -31 => { return error.not_initialised; },
      -32 => { return error.initialised; },
      -33 => { return error.bad_wave_mode; },
      -34 => { return error.bad_cfg_internal; },
      -35 => { return error.bad_wave_baud; },
      -36 => { return error.too_many_pulses; },
      -37 => { return error.too_many_chars; },
      -38 => { return error.not_serial_gpio; },
      -39 => { return error.bad_serial_struc; },
      -40 => { return error.bad_serial_buf; },
      -41 => { return error.not_permitted; },
      -42 => { return error.some_permitted; },
      -43 => { return error.bad_wvsc_commnd; },
      -44 => { return error.bad_wvsm_commnd; },
      -45 => { return error.bad_wvsp_commnd; },
      -46 => { return error.bad_pulselen; },
      -47 => { return error.bad_script; },
      -48 => { return error.bad_script_id; },
      -49 => { return error.bad_ser_offset; },
      -50 => { return error.gpio_in_use; },
      -51 => { return error.bad_serial_count; },
      -52 => { return error.bad_param_num; },
      -53 => { return error.dup_tag; },
      -54 => { return error.too_many_tags; },
      -55 => { return error.bad_script_cmd; },
      -56 => { return error.bad_var_num; },
      -57 => { return error.no_script_room; },
      -58 => { return error.no_memory; },
      -59 => { return error.sock_read_failed; },
      -60 => { return error.sock_writ_failed; },
      -61 => { return error.too_many_param; },
      -62 => { return error.script_not_ready; },
      -63 => { return error.bad_tag; },
      -64 => { return error.bad_mics_delay; },
      -65 => { return error.bad_mils_delay; },
      -66 => { return error.bad_wave_id; },
      -67 => { return error.too_many_cbs; },
      -68 => { return error.too_many_ool; },
      -69 => { return error.empty_waveform; },
      -70 => { return error.no_waveform_id; },
      -71 => { return error.i2c_open_failed; },
      -72 => { return error.ser_open_failed; },
      -73 => { return error.spi_open_failed; },
      -74 => { return error.bad_i2c_bus; },
      -75 => { return error.bad_i2c_addr; },
      -76 => { return error.bad_spi_channel; },
      -77 => { return error.bad_flags; },
      -78 => { return error.bad_spi_speed; },
      -79 => { return error.bad_ser_device; },
      -80 => { return error.bad_ser_speed; },
      -81 => { return error.bad_param; },
      -82 => { return error.i2c_write_failed; },
      -83 => { return error.i2c_read_failed; },
      -84 => { return error.bad_spi_count; },
      -85 => { return error.ser_write_failed; },
      -86 => { return error.ser_read_failed; },
      -87 => { return error.ser_read_no_data; },
      -88 => { return error.unknown_command; },
      -89 => { return error.spi_xfer_failed; },
      -90 => { return error.bad_pointer; },
      -91 => { return error.no_aux_spi; },
      -92 => { return error.not_pwm_gpio; },
      -93 => { return error.not_servo_gpio; },
      -94 => { return error.not_hclk_gpio; },
      -95 => { return error.not_hpwm_gpio; },
      -96 => { return error.bad_hpwm_freq; },
      -97 => { return error.bad_hpwm_duty; },
      -98 => { return error.bad_hclk_freq; },
      -99 => { return error.bad_hclk_pass; },
     -100 => { return error.hpwm_illegal; },
     -101 => { return error.bad_databits; },
     -102 => { return error.bad_stopbits; },
     -103 => { return error.msg_toobig; },
     -104 => { return error.bad_malloc_mode; },
     -105 => { return error.too_many_segs; },
     -106 => { return error.bad_i2c_seg; },
     -107 => { return error.bad_smbus_cmd; },
     -108 => { return error.not_i2c_gpio; },
     -109 => { return error.bad_i2c_wlen; },
     -110 => { return error.bad_i2c_rlen; },
     -111 => { return error.bad_i2c_cmd; },
     -112 => { return error.bad_i2c_baud; },
     -113 => { return error.chain_loop_cnt; },
     -114 => { return error.bad_chain_loop; },
     -115 => { return error.chain_counter; },
     -116 => { return error.bad_chain_cmd; },
     -117 => { return error.bad_chain_delay; },
     -118 => { return error.chain_nesting; },
     -119 => { return error.chain_too_big; },
     -120 => { return error.deprecated; },
     -121 => { return error.bad_ser_invert; },
     -122 => { return error.bad_edge; },
     -123 => { return error.bad_isr_init; },
     -124 => { return error.bad_forever; },
     -125 => { return error.bad_filter; },
     -126 => { return error.bad_pad; },
     -127 => { return error.bad_strength; },
     -128 => { return error.fil_open_failed; },
     -129 => { return error.bad_file_mode; },
     -130 => { return error.bad_file_flag; },
     -131 => { return error.bad_file_read; },
     -132 => { return error.bad_file_write; },
     -133 => { return error.file_not_ropen; },
     -134 => { return error.file_not_wopen; },
     -135 => { return error.bad_file_seek; },
     -136 => { return error.no_file_match; },
     -137 => { return error.no_file_access; },
     -138 => { return error.file_is_a_dir; },
     -139 => { return error.bad_shell_status; },
     -140 => { return error.bad_script_name; },
     -141 => { return error.bad_spi_baud; },
     -142 => { return error.not_spi_gpio; },
     -143 => { return error.bad_event_id; },
     -144 => { return error.cmd_interrupted; },
     -145 => { return error.not_on_bcm2711; },
     -146 => { return error.only_on_bcm2711; },
    -2000 => { return error.bad_send; },
    -2001 => { return error.bad_recv; },
    -2002 => { return error.bad_getaddrinfo; },
    -2003 => { return error.bad_connect; },
    -2004 => { return error.bad_socket; },
    -2005 => { return error.bad_noib; },
    -2006 => { return error.duplicate_callback; },
    -2007 => { return error.bad_malloc; },
    -2008 => { return error.bad_callback; },
    -2009 => { return error.notify_failed; },
    -2010 => { return error.callback_not_found; },
    -2011 => { return error.unconnected_pi; },
    -2012 => { return error.too_many_pis; },
    else  => { return error.unknown_error; },
  }

  unreachable;
}

// =============================================================================
//  Structure Pin
// =============================================================================

pub const Pin = struct
{
  gpio  : *PiGPIO,
  pin   : u32,

  // ---------------------------------------------------------------------------
  //  structure Pin.Mode
  // ---------------------------------------------------------------------------

  pub const Mode = enum(u8)
  {
    input  = 0,
    output = 1,
    alt0   = 4,
    alt1   = 5,
    alt2   = 6,
    alt3   = 7,
    alt4   = 3,
    alt5   = 2,
  };

  pub const Pull = enum(u8)
  {
    none = 0,
    down = 1,
    up   = 2,
  };

  pub const Edge = enum(u8)
  {
    rising  = 0,
    falling = 1,
    either  = 2,
  };

  // ---------------------------------------------------------------------------
  //  Function: Pin.setHigh
  // ---------------------------------------------------------------------------
  //  Set the pin output to the high state.
  //
  //  Does nothing if the pin is not set to be an output pin.

  pub fn setHigh( self : Pin ) Error!void
  {
    _ = try self.gpio.doCmd( .WRITE, self.pin, 1, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setLow
  // ---------------------------------------------------------------------------
  //  Set the pin output to the low state.
  //
  //  Does nothing if the pin is not set to be an output pin.

  pub fn setLow( self : Pin ) Error!void
  {
    _ = try self.gpio.doCmd( .WRITE, self.pin, 0, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.set
  // ---------------------------------------------------------------------------
  //  Set the pin output state based on boolean parameter.
  //
  //  Does nothing if the pin is not set to be an output pin.

  pub fn set( self : Pin, in_value : bool ) Error!void
  {
    _ = try self.gpio.doCmd( .WRITE,
                                 self.pin,
                                 @intFromBool( in_value ),
                                 0,
                                 null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.get
  // ---------------------------------------------------------------------------
  //  Get the logic state of a pin.

  pub fn get( self : Pin ) Error!bool
  {
    return try self.gpio.doCmd( .READ, self.pin, 0, 0, null ) != 0;
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setMode
  // ---------------------------------------------------------------------------
  // Set the pin mode.

  pub fn setMode(  self : Pin, in_mode : Mode ) Error!void
  {
    _ = try self.gpio.doCmd( .MODES,
                                 self.pin,
                                 @intFromEnum( in_mode ),
                                 0,
                                 null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getMode
  // ---------------------------------------------------------------------------
  // Get the current pin mode.

  pub fn getMode( self : Pin ) Error!Mode
  {
    const result = try self.gpio.doCmd( .MODEG, self.pin, 0, 0, null );

    return @enumFromInt( result );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setPull
  // ---------------------------------------------------------------------------
  // Set the pin pull resistor state.

  pub fn setPull(  self : Pin, inPull : Pull ) Error!void
  {
    _ = try self.gpio.doCmd( .PUD,
                                 self.pin,
                                 @intFromEnum( inPull ),
                                 0,
                                 null );
  }

  // ==== PWM Functions ========================================================

  // ---------------------------------------------------------------------------
  //  Function: Pin.setPWMFrequency
  // ---------------------------------------------------------------------------

  pub fn setPWMFrequency( self : Pin, in_frequency : u32 ) Error!void
  {
    _ = try self.gpio.doCmd( .PFS,
                                 self.pin,
                                 in_frequency,
                                 0,
                                 null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getPWMFrequency
  // ---------------------------------------------------------------------------

  pub fn getPWMFrequency( self : Pin ) Error!u32
  {
    return try self.gpio.doCmd( .PFG, self.pin, 0, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setPWMRange
  // ---------------------------------------------------------------------------

  pub fn setPWMRange( self : Pin, in_range : u32 ) Error!void
  {
    _ = try self.gpio.doCmd( .PRS, self.pin, in_range, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getPWMRange
  // ---------------------------------------------------------------------------

  pub fn getPWMRange( self : Pin ) Error!u32
  {
    return try self.gpio.doCmd( .PRG, self.pin, 0, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setPWMDutyCycle
  // ---------------------------------------------------------------------------

  pub fn setPWMDutyCycle( self : Pin, in_duty_cycle: u32 ) Error!void
  {
    _ = try self.gpio.doCmd( .PWM, self.pin, in_duty_cycle, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getPWMDutyCycle
  // ---------------------------------------------------------------------------

  pub fn getPWMDutyCycle( self : Pin ) Error!u32
  {
    return try self.gpio.doCmd( .GDC, self.pin, 0, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.setPWMDutyFractiom
  // ---------------------------------------------------------------------------
  // Sets the duty cycle based on a fraction that must be between 0 and
  // 1 (inclusive).

  pub fn setPWMDutyFractiom( self             : Pin,
                             in_duty_fraction : f32 ) Error!void
  {
    var range : f32 =   @floatFromInt( try self.getPWMRange() );

    range *= in_duty_fraction;

    return try self.setPWMDutyCycle( @round( range ) );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getPWMDutyFractiom
  // ---------------------------------------------------------------------------
  // Gets the duty cycle based on a fraction that must be between 0 and
  // 1 (inclusive).

  pub fn getPWMDutyFractiom( self : Pin ) Error!f32
  {
    const range : f32 = @floatFromInt( try self.getPWMRange() );
    const cycle : f32 = @floatFromInt( try self.getPWMDutyCycle() );

    return cycle / range;
  }

  // ==== Servo Functions ======================================================

  // ---------------------------------------------------------------------------
  //  Function: Pin.setServoPulseWidth
  // ---------------------------------------------------------------------------

  pub fn setServoPulseWidth( self : Pin, in_width : u32 ) Error!void
  {
    _ = try self.gpio.doCmd( .SERVO, self.pin, in_width, 0, null );
  }

  // ---------------------------------------------------------------------------
  //  Function: Pin.getServoPulseWidth
  // ---------------------------------------------------------------------------

  pub fn getServoPulseWidth( self : Pin ) Error!u32
  {
    return try self.gpio.doCmd( .GPW, self.pin, 0, 0, null );
  }

  // ==== Pin Change Callback ==================================================

  // ---------------------------------------------------------------------------
  //  Function: Pin.setCallback
  // ---------------------------------------------------------------------------

  // pub fn setCallback( self    : Pin,
  //                     in_edge : Edge,
  //                     in_func : CBFunction ) Error!void
  // {
  //   _ = try self.gpio.doCmd( .XXXX, self.pin, in_edge, in_func, null );
  // }
};


// =============================================================================
//  Structure SPI
// =============================================================================

pub const SPI = struct
{
  gpio      : ?*PiGPIO             = null,
  spi       : u32                = 0,
  allocator : std.mem.Allocator  = undefined,


  // ---------------------------------------------------------------------------
  //  Function: SPI.open
  // ---------------------------------------------------------------------------

  pub fn open( self         : *SPI,
               in_gpio      : *PiGPIO,
               in_allocator : std.mem.Allocator,
               in_channel   : u32,
               in_bit_rate  : u32,
               in_flags     : u32 ) (SPIError||error{OutOfMemory})!void
  {

    log.debug( "Gpio: {?}", .{ self.gpio } );

    self.close(); // will do nothing if SPI not already open.

    self.spi  = try in_gpio.doCmd( .SPIO,
                                   in_channel,
                                   in_bit_rate,
                                   4,
                                   std.mem.asBytes( &in_flags ) );
    self.allocator = in_allocator;
    self.gpio      = in_gpio;
  }

  // ---------------------------------------------------------------------------
  //  Function: SPI.close
  // ---------------------------------------------------------------------------

  pub fn close( self : *SPI ) void
  {
    if (self.gpio) |gpio|
    {
      _ = gpio.doCmd( .SPIC, self.spi, 0, 0, null ) catch |err|
      {
        log.warn( "SPI Deinit error (ignored): {}", .{ err } );
      };

      self.gpio = null;
    }
  }

  // ---------------------------------------------------------------------------
  //  Function: SPI.read
  // ---------------------------------------------------------------------------
  /// This function reads data from the SPI interface into the receive
  /// slice.
  ///
  /// Note: the SPI bus sends zeros when reading data.

  pub fn read( self         : SPI,
               out_rx_slice : []u8 ) SPIError!u32
  {
    std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );

    if (self.gpio) |gpio|
    {
      const result = try gpio.doCmd( .SPIR,
                                     self.spi,
                                     out_rx_slice.len,
                                     0,
                                     null );

      gpio.cmd_stream.read( out_rx_slice );

      return result;
    }

    return error.SPINotOpen;
  }

  // ---------------------------------------------------------------------------
  //  Function: SPI.write
  // ---------------------------------------------------------------------------
  /// This function transmits data over an SPI interface.  It attempts
  /// to transmit all the data in the transmit slice.
  ///
  /// Note: any data received on the SPI bus during the write is ignored.

  pub fn write( self        : SPI,
                in_tx_slice : [] const u8 ) SPIError!void
  {
    std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );

    if (self.gpio) |gpio|
    {
      _ = try gpio.doCmd( .SPIW,
                          self.spi,
                          0,
                          @intCast( in_tx_slice.len ),
                          in_tx_slice.ptr );
      return;
    }

    return error.SPINotOpen;
  }

  // ---------------------------------------------------------------------------
  //  Function: SPI.transfer
  // ---------------------------------------------------------------------------
  /// This function sends the slice transmit slice and simultaniously reads
  /// to the receive buffer.
  ///
  /// The caller must assure that receive slice is the same size as
  /// the transmit slice.  It is permissable for the receeive slice to point
  /// to the same memory as the transmit slice.

  pub fn transfer( self          : SPI,
                   in_tx_slice   : [] const u8,
                   out_rx_slice  : []u8 ) Error!void
  {
    std.debug.assert( in_tx_slice.len ==  out_rx_slice.len );

    _ = try self.gpio.doCmd( .SPIX,
                                 self.spi,
                                 0,
                                 in_tx_slice.len,
                                 in_tx_slice );

    try self.gpio.cmd_stream.read( out_rx_slice );
  }
};


// =============================================================================
//  Structure Stream
// =============================================================================

const Stream = struct
{
  stream    : std.net.Stream   = undefined,
  mutex     : std.Thread.Mutex = .{},


  // ---------------------------------------------------------------------------
  //  Function: Stream.open
  // ---------------------------------------------------------------------------

  fn open( self     : *Stream,
           in_alloc : std.mem.Allocator,
           in_addr  : []const u8,
           in_port  : u16 ) InitError!void
  {
    self.stream   = try std.net.tcpConnectToHost( in_alloc,
                                                  in_addr,
                                                  in_port );
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.close
  // ---------------------------------------------------------------------------

  fn close( self : *Stream ) void
  {
    self.stream.close();
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.doCmd
  // ---------------------------------------------------------------------------

  fn doCmd( self         : *Stream,
            in_cmd       : Command,
            in_p1        : u32,
            in_p2        : u32,
            in_ext_len   : u32,
            in_extension : ?* const anyopaque ) Error!u32
  {
    const result = try self.doCmdBasic( in_cmd,
                                        in_p1,
                                        in_p2,
                                        in_ext_len,
                                        in_extension );

    const status : i32 = @bitCast( result );

    if (status < 0) return convertError( status );

    return result;
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.doCmdBasic
  // ---------------------------------------------------------------------------

  fn doCmdBasic( self         : *Stream,
                 in_cmd       : Command,
                 in_p1        : u32,
                 in_p2        : u32,
                 in_ext_len   : u32,
                 in_extension : ?* const anyopaque ) Error!u32
  {
    var   hdr : Header  = undefined;
    const cmd : u32     = @intFromEnum( in_cmd );

    self.mutex.lock();
    defer self.mutex.unlock();

    if (in_extension) |ext|
    {
      hdr = .{ .cmd = cmd,
               .p1  = in_p1,
               .p2  = in_p2,
               .p3  = in_ext_len };

      _ = try self.stream.write( std.mem.asBytes( &hdr ) );

      if (in_ext_len > 0)
      {
        const ep : [*] const u8 = @ptrCast( ext );

        _ = try self.stream.write( ep[0..in_ext_len] );
      }
    }
    else
    {
      hdr = .{ .cmd = @intFromEnum( in_cmd ),
              .p1  = in_p1,
              .p2  = in_p2,
              .p3  = 0 };

      _ = try self.stream.write( std.mem.asBytes( &hdr ) );
    }

    _ = try self.stream.read( std.mem.asBytes( &hdr ) );

    if (hdr.cmd != cmd) return error.bad_recv;

    return hdr.p3;
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.write
  // ---------------------------------------------------------------------------

  fn write( self: Stream, buffer: [] const u8 ) WriteError!usize
  {
    self.stream.write( buffer );
  }

  // ---------------------------------------------------------------------------
  //  Function: Stream.read
  // ---------------------------------------------------------------------------

  fn read( self: Stream, buffer: []u8 ) ReadError!usize
  {
    self.stream.read( buffer );
  }

};

// =============================================================================
//  Testing
// =============================================================================

// const testing = std.testing;

// test "create gpio"
// {
//   try testing.expect( connect( null, null ) == .ok );

//   defer disconnect();

//   try testing.expect( gpio.pi > 0 );
// }
