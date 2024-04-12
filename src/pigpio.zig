// zig fmt: off
// DO NOT REMOVE ABOVE LINE -- I strongly dislike the way Zig formats code.

// -----------------------------------------------------------------------------
// Copyright © 2024, Curuvar Ltd.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//
//   2. Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//   3. Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -----------------------------------------------------------------------------

//! PiGPIO-Zig is a module that communicates with the Raspberry Pi's pigpiod
//! GPIO daemon.
//!
//! PiGPIO-Zig provides most of the capabilities of C based pigpiod_if2
//! interface, but is written entirely in zig.  The missing capabilities are
//! functions that pigpiod implements directly on the client, and that Zig
//! provides native function for.

const std = @import( "std" );

const PiGPIO = @This();

const log = std.log.scoped( .PiGPIO );

// =============================================================================
//  Our Fields
// =============================================================================

/// The allocator that we will use as needed.
allocator : std.mem.Allocator = undefined,

/// The DNS name or IP address of pigpiod daemon.
address : [] const u8 = "::",

/// The port the daemon is listening on.
port : u16 = 8888,

/// A stream for sending commands to the daemon and receiving responses.
cmd_stream : std.net.Stream = undefined,

/// A mutex to make sending commands thread safe.
cmd_mutex : std.Thread.Mutex  = .{},

/// A handle to identify us to the daemon for notification control.
notify_handle : u32 = 0,

/// A thread that listens for notification from the server.
notify_thread : ?std.Thread = null,

/// A bitmap of current pins that we want level notifications for.
notify_level_bits : u32 = 0,

/// A bitmap of current pins that we want event notifications for.
notify_event_bits : u32 = 0,

/// A mutex to make callback list manipulation thread safe.
list_mutex : std.Thread.Mutex = .{},

/// Level change notification callbacks.
level_cb_first : ?*LevelCallback = null,

/// Event notification callbacks.
event_cb_first : ?*EventCallback = null,

// =============================================================================
//  Public Constants
// =============================================================================

/// Zig Errors for various status messages returned from the pigpiod daemon.
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
    bad_databits,      // serial data bits not 0-31
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
                       || error{ NotOpen };

pub const I2CError   =    Error
                       || error{ NotOpen };

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Public Type Definitions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// -----------------------------------------------------------------------------
/// Pin mode.  The definition of the alt modes depends on the specific pin.
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

// -----------------------------------------------------------------------------
/// Pull resistor configuration
pub const Pull = enum(u8)
{
    none = 0,
    down = 1,
    up   = 2,
};

// -----------------------------------------------------------------------------
/// Edge for callbacks
pub const Edge = enum(u8)
{
    falling = 0,
    rising  = 1,
    either  = 2,
    timeout = 3, // used for timeout return events only
};

// -----------------------------------------------------------------------------
///  Callback function prototype.

const LevelCBFunc = * const fn ( in_pin     : Pin,
                                 in_edge    : Edge,
                                 in_tick    : u32,
                                 in_context : ?*anyopaque ) void;

const EventCBFunc = * const fn ( in_gpio    : *PiGPIO,
                                 in_event   : u5,
                                 in_tick    : u32,
                                 in_context : ?*anyopaque ) void;


// =============================================================================
//  Private Type Definitions
// =============================================================================

// -----------------------------------------------------------------------------
/// Communincations header for exchanges with pigpiod.
const Header = struct
{
    /// Command to perform
    cmd : u32,
    /// First parameter
    p1  : u32,
    /// Second parameter
    p2  : u32,
    /// On send: length of additional data.  On receive: status or length
    /// of additional data.
    p3  : u32,
};

// -----------------------------------------------------------------------------
/// Items to send with transaction.
const Extent = [] const u8;

// -----------------------------------------------------------------------------
const LevelCallback = struct
{
    next    : ?*LevelCallback,
    pin     : u5,      // Callbacks only on pins 0 - 31
    edge    : Edge,
    func    : LevelCBFunc,
    context : ?*anyopaque,
};

// -----------------------------------------------------------------------------
const EventCallback = struct
{
    next    : ?*EventCallback,
    event   : u5,
    func    : EventCBFunc,
    context : ?*anyopaque,
};
// =============================================================================
//  Private Constants
// =============================================================================

/// Commands supported by pigpiod daemon.
const Command = enum(u8)
{
    mode_set        =   0,
    mode_get        =   1,
    pull_set        =   2,
    pin_get         =   3,
    pin_set         =   4,
    pwm_duty_set    =   5,
    pwm_rate_set    =   6,
    pwm_freq_set    =   7,
    servo_set       =   8,
    wdog_set        =   9,
    band_read_1     =  10,
    band_read_2     =  11,
    band_clear_1    =  12,
    band_clear_2    =  13,
    band_set_1      =  14,
    band_set_2      =  15,
    tick_get        =  16,
    hw_ver_get      =  17,
    NO     = 18,
    level_bits_set  =  19,
    NP     = 20,
    NC     = 21,
    pwm_rate_get    =  22,
    pwm_freq_get    =  23,
    pwm_real_range  =  24,
    HELP   = 25,
    version_get     =  26,
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
    pin_trigger     =  37,
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

    i2c_open        =  54,
    i2c_close       =  55,
    i2c_rx_raw      =  56,
    i2c_tx_raw      =  57,
    i2c_tx_quick    =  58,
    i2c_tx_raw_u8   =  59,
    i2c_rx_raw_u8   =  60,
    i2c_rx_u8       =  61,
    i2c_tx_u8       =  62,
    i2c_rx_u16      =  63,
    i2c_tx_u16      =  64,
    i2c_rx_block    =  65,
    i2c_tx_block    =  66,
    i2c_rx_i2c_blk  =  67,
    i2c_tx_i2c_blk  =  68,
    i2c_proc_call   =  69,
    i2c_zip         =  70,

    spi_open        =  71,
    spi_close       =  72,
    spi_read        =  73,
    spi_write       =  74,
    spi_trasfer     =  75,

    SERO   = 76,
    SERC   = 77,
    SERRB  = 78,
    SERWB  = 79,
    SERR   = 80,
    SERW   = 81,
    SERDA  = 82,

    pwm_duty_get    =  83,
    servo_get       =  84,

    HC     = 85,
    HP     = 86,

    custom_1        =  87,
    custom_2        =  88,

    BI2CC  = 89,
    BI2CO  = 90,
    BI2CZ  = 91,

    I2CZ   = 92,

    WVCHA  = 93,

    SLRI   = 94,

    CGI    = 95,
    CSI    = 96,

    glitch_filter   =  97,
    noise_filter    =  98,

    begin_notify    =  99,  // Inform server that this is the notify stream.

    WVTXM  = 100,
    WVTAT  = 101,

    current_set     = 102,
    current_get     = 103,

    FO     = 104,
    FC     = 105,
    FR     = 106,
    FW     = 107,
    FS     = 108,
    FL     = 109,

    shell_cmd       = 110,

    BSPIC  = 111,
    BSPIO  = 112,
    BSPIX  = 113,

    BSCX   = 114,

    events_bits_set = 115,
    trigger_event   = 116,

    PROCU  = 117,
    WVCAP  = 118,
};

const PI_NTFY_FLAGS_EVENT = 1 << 7;
//const PI_NTFY_FLAGS_ALIVE = 1 << 6;
const PI_NTFY_FLAGS_WDOG  = 1 << 5;


// =============================================================================
//  Public Functions
// =============================================================================

// -----------------------------------------------------------------------------
//  Public function: connect
// -----------------------------------------------------------------------------
/// Initialize a PiGPIO structure.
///
/// Parameter:
///   - in_alloc - an allocator to use as needed.
///   - in_addr  - the domain name, or ip address (as text) of the machine
///                where the pigpiod daemon is running. null = localhost
///   - in_port  - the port on which the pigpiod is listening. null = 8888

pub fn connect( self     : *PiGPIO,
                in_alloc : std.mem.Allocator,
                in_addr  : ?[]const u8,
                in_port  : ?u16 ) InitError!void
{
    self.allocator       = in_alloc;
    self.level_cb_first  = null;

    if (in_addr) |a| self.address = a;
    if (in_port) |p| self.port    = p;

    self.cmd_stream = try std.net.tcpConnectToHost( self.allocator,
                                                    self.address,
                                                    self.port );

    errdefer self.cmd_stream.close();

    const option : u32          = 1;
    const op     : [*] const u8 = @ptrCast( &option );

    try std.posix.setsockopt( self.cmd_stream.handle,
                              std.posix.IPPROTO.TCP,
                              std.posix.TCP.NODELAY,
                              op[0..4] );

    log.debug( "-- PiGPIO Connected --", .{} );
}

// -----------------------------------------------------------------------------
//  Public function: disconnect
// -----------------------------------------------------------------------------
/// Deinitialize a PiGPIO structure

pub fn disconnect( self : *PiGPIO ) void
{
    log.debug( "Started disconnect", .{} );
    defer log.debug( "Finished disconnect", .{} );

    self.list_mutex.lock();

    self.notify_level_bits = 0;
    self.notify_event_bits = 0;

    var a_lvl_cb = self.level_cb_first;
    self.level_cb_first = null;

    while (a_lvl_cb) |cb|
    {
        a_lvl_cb = cb.next;
        self.allocator.destroy( cb );
    }

    var an_evt_cb = self.event_cb_first;
    self.event_cb_first = null;

    while (an_evt_cb) |cb|
    {
        an_evt_cb = cb.next;
        self.allocator.destroy( cb );
    }

    self.list_mutex.unlock();

    log.debug( "waiting for thread close", .{} );

    if (self.notify_thread) |t| t.join();
    self.notify_thread = null;

    self.cmd_stream.close();

    log.debug( "-- PiGPIO Disconnected --", .{} );
}

// -----------------------------------------------------------------------------
//  Public function: pin
// -----------------------------------------------------------------------------
/// Return an initialized Pin structure.
///
/// Paramter:
/// - pin - the Broadcom pin number.

pub fn pin( self : *PiGPIO, in_pin : u6 ) Pin
{
    std.debug.assert( in_pin <= 53 );

    return .{ .gpio = self, .pin = in_pin };
}

// -----------------------------------------------------------------------------
//  Public function: readBank1
// -----------------------------------------------------------------------------
/// Read all bank 1 GPIO pins (0-31) simultaniouslly.
///
/// The result is a u32 with the state of all GPIO pins.  The low order bit
/// is pin 1.

pub fn readBank1( self : *PiGPIO ) Error!u32
{
    return try self.doCmdBasic( .band_read_1, true, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: readBank2
// -----------------------------------------------------------------------------
/// Read all bank 2 GPIO pins (32-53) simultaniouslly.
///
/// The result is a u32 with the state of all GPIO pins.  The low order bit
/// is pin 1.

pub fn readBank2( self : *PiGPIO )  Error!u32
{
    return try self.doCmdBasic( .band_read_2, true, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: clearBank1
// -----------------------------------------------------------------------------
/// Clear multiple bank 1 GPIO pins (0-31) simultaniouslly.
///
/// Parameter:
/// - in_pins - I mask indicating the pins to clear. The low order bit is pin 1.

pub fn clearBank1(  self : *PiGPIO, in_pins : u32 ) Error!void
{
    _ = try self.doCmd( .band_clear_1, true, in_pins, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: clearBank2
// -----------------------------------------------------------------------------
/// Clear multiple bank 2 GPIO pins (32-53) simultaniouslly.
///
/// Parameter:
/// - in_pins - I mask indicating the pins to clear. The low order bit is pin 32.

pub fn clearBank2(  self : *PiGPIO, in_pins : u32 ) Error!void
{
    _ = try self.doCmd( .band_clear_2, true, in_pins, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: setBank1
// -----------------------------------------------------------------------------
/// Set multiple bank 1 GPIO pins (0-31) simultaniouslly.
///
/// Parameter:
/// - in_pins - I mask indicating the pins to set. The low order bit is pin 1.

pub fn setBank1(  self : *PiGPIO, in_pins : u32 ) Error!void
{
    _ = try self.doCmd( .band_set_1, true, in_pins, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: setBank2
// -----------------------------------------------------------------------------
/// Set multiple bank 2 GPIO pins (32-53) simultaniouslly.
///
/// Parameter:
/// - in_pins - I mask indicating the pins to set. The low order bit is pin 32.

pub fn setBank2( self : *PiGPIO, in_pins : u32 ) Error!void
{
    _ = try self.doCmd( .band_set_2, true, in_pins, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: maskedUpdateBank1
// -----------------------------------------------------------------------------
/// Update a subset of bank 1 GPIO pins (0-31) simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 1.

pub fn maskedUpdateBank1( self      : *PiGPIO,
                          in_values : u32,
                          in_mask   : u32 ) Error!void
{
    var p = in_values & in_mask;
    if (p != 0) _ = try self.doCmd( .band_set_1, true, p, 0, null );
    p = ~p & in_mask;
    if (p != 0) _ = try self.doCmd( .band_clear_1, true, p, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: maskedUpdateBank2
// -----------------------------------------------------------------------------
/// Update a subset of bank 1 GPIO pins (32-53) simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 32.

pub fn maskedUpdateBank2( self      : *PiGPIO,
                          in_values : u32,
                          in_mask   : u32 ) Error!void
{
    var p = in_values & in_mask;
    if (p != 0) _ = try self.doCmd( .band_set_2, true, p, 0, null );
    p = ~p & in_mask;
    if (p != 0) _ = try self.doCmd( .band_clear_2, true, p, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: maskedSetModeBank1
// -----------------------------------------------------------------------------
/// Set the mode a subset of bank 1 GPIO pins (0-31) simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 1.

pub fn maskedSetModeBank1( self    : *PiGPIO,
                           in_mode : Mode,
                           in_mask : u32 ) Error!void
{
    for (0..32) |p|
    {
        if ((in_mask & (@as( u32, 1 ) << @intCast( p ))) != 0)
        {
            _ = try self.doCmd( .mode_set,
                                true,
                                @intCast( p ),
                                @intFromEnum( in_mode ),
                                null );
        }
    }
}

// -----------------------------------------------------------------------------
//  Public function: maskedSetModeBank2
// -----------------------------------------------------------------------------
/// Set the mode a subset of bank 1 GPIO pins (32-53) simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 32.

pub fn maskedSetModeBank2( self    : *PiGPIO,
                           in_mode : Mode,
                           in_mask : u32 ) Error!void
{
    for (0..32) |p|
    {
        if ((in_mask & (@as( u32, 1 ) << @intCast( p ))) != 0)
        {
            _ = try self.doCmd( .mode_set,
                                true,
                                @intCast( p + 32 ),
                                @intFromEnum( in_mode ),
                                null );
        }
    }
}

// -----------------------------------------------------------------------------
//  Public function: maskedSetPullBank1
// -----------------------------------------------------------------------------
/// Set the mode a subset of bank 1 GPIO pins simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 1.

pub fn maskedSetPullBank1( self    : *PiGPIO,
                           in_pull : Pull,
                           in_mask : u32 ) Error!void
{
    for (0..32) |p|
    {
        if ((in_mask & (@as( u32, 1 ) << @intCast( p ))) != 0)
        {
            _ = try self.doCmd( .pull_set,
                                true,
                                @intCast( p ),
                                @intFromEnum( in_pull ),
                                null );
        }
    }
}

// -----------------------------------------------------------------------------
//  Public function: maskedSetPullBank2
// -----------------------------------------------------------------------------
/// Set the mode a subset of bank 1 GPIO pins (32-53) simultaniouslly.
///
/// Parameter:
/// - in_values - The value to set the pins to
/// - in_mask   - I mask indicating which pins to update.
/// The low order bit is pin 32.

pub fn maskedSetPullBank2( self    : *PiGPIO,
                           in_pull : Pull,
                           in_mask : u32 ) Error!void
{
    for (0..32) |p|
    {
        if ((in_mask & (@as( u32, 1 ) << @intCast( p ))) != 0)
        {
            _ = try self.doCmd( .pull_set,
                                true,
                                @intCast( p + 32 ),
                                @intFromEnum( in_pull ),
                                null );
        }
    }
}

// -----------------------------------------------------------------------------
//  Public function: getCurrentTick
// -----------------------------------------------------------------------------
/// Get the current tick (microseconds) from the pgpiod daemon.

pub fn getCurrentTick( self : *PiGPIO ) Error!u32
{
    return try self.doCmdBasic( .tick_get, true, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: getHardwareVersion
// -----------------------------------------------------------------------------
/// Get the hardware version from the pgpiod daemon.

pub fn getHardwareVersion( self : *PiGPIO ) Error!u32
{
    return try self.doCmd( .hw_ver_get, true, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: getPiGPIOVersion
// -----------------------------------------------------------------------------
/// Get the version number from the pigpiod daemon.

pub fn getPiGPIOVersion( self : *PiGPIO ) Error!u32
{
    return try self.doCmd( .version_get, true, 0, 0, null );
}

// -----------------------------------------------------------------------------
//  Public function: shell
// -----------------------------------------------------------------------------
/// Run a shell script on the system where the pigpiod daemon is running.
/// The named scrip must exist in /opt/pigpio/cgi on the target system
/// and must be executable.
/// The returned exit status is normally 256 times that set by the shell
/// script's exit function. If the script can't be found 32512 will be returned.

pub fn shell( self     : *PiGPIO,
              in_name  : [] const u8,
              in_param : [] const u8 ) Error!u32
{
    const sep = [1]u8{ 0 };
    const ext = [_]Extent{ in_name, &sep, in_param };

    return try self.doCmd( .shell_cmd, true, 0, 0, &ext );
}

// -----------------------------------------------------------------------------
//  Public function: custom1
// -----------------------------------------------------------------------------
/// Run a custom function on the pigpiod dameon.
/// The daemon needs to be re-built to add the custom function.
///
/// The meaning of the three arguments depend on custom function itself.
///
/// The custom function returns an unsigned value.

pub fn custom1( self    : *PiGPIO,
                in_arg1 : u32,
                in_arg2 : u32,
                in_arg3 : [] const u8 ) Error!i32
{
    std.debug.assert( in_arg3.len <= 0xFFFF_FFFF );

    const ext = [_]Extent{ in_arg3 };

    const result = try self.doCmdBasic( .custom_1,
                                        true,
                                        in_arg1,
                                        in_arg2,
                                        &ext );

    return @bitCast( result );
}

// -----------------------------------------------------------------------------
//  Public function: custom2
// -----------------------------------------------------------------------------
/// Run a custom function on the pigpiod dameon.
/// The daemon needs to be re-built to add the custom function.
///
/// The meaning of the two arguments depend on custom function itself.
/// The in_reply parameter points to a slice that is filled in by the
/// custom function.

pub fn custom2( self    : *PiGPIO,
                in_arg1 : u32,
                in_arg2 : [] const u8,
                in_reply : []u8 ) Error!i32
{
    std.debug.assert( in_arg2.len <= 0xFFFF_FFFF );
    std.debug.assert( in_reply.len <= 0xFFFF_FFFF );

    const ext = [_]Extent{ in_arg2 };

    const result = try self.doCmdBasic( .custom_2,
                                        false,
                                        in_arg1,
                                        @intCast( in_reply.len ),
                                        &ext );
    defer self.cmd_mutex.unlock();

    _ = try self.cmd_stream.read( in_reply );

    return @bitCast( result );
}

// -----------------------------------------------------------------------------
// Function: addEventCallback
// -----------------------------------------------------------------------------
/// Set a callback function that will be called if the pin's state
/// changes.

fn addEventCallback( self       : *PiGPIO,
                     in_event   : u5,
                     in_func    : EventCBFunc,
                     in_context : ?*anyopaque )!void
{
    const cb = try self.allocator.create( EventCallback );
    errdefer self.allocator.destroy( cb );

    cb.* = .{
                .next    = self.event_cb_first,
                .event   = in_event,
                .func    = in_func,
                .context = in_context
            };

    self.list_mutex.lock();
    defer self.list_mutex.unlock();

    const start_notify =    self.level_cb_first  == null
                        and self.event_cb_first  == null;

    self.event_cb_first = cb;

    if (start_notify)
    {
        self.notify_thread = try std.Thread.spawn( .{}, notifyThread, .{ self } );
    }
    else
    {
        try self.updateNotifyEventBits();
    }
}

// -----------------------------------------------------------------------------
// Function: removeEventCallback
// -----------------------------------------------------------------------------
/// Set a callback function that will be called if the pin's state
/// changes.

fn removeEventCallback( self       : *PiGPIO,
                        in_event   : u5,
                        in_func    : EventCBFunc,
                        in_context : ?*anyopaque ) void
{
    self.list_mutex.lock();
    defer self.list_mutex.unlock();

    var a_callback = self.event_cb_first;
    var prior : ?*EventCallback  = null;

    while (a_callback) |cb|
    {
        if (    cb.event   == in_event
            and cb.func    == in_func
            and cb.context == in_context)
        {
            if (prior) |p|
            {
                p.next = cb.next;
            }
            else
            {
               self.event_cb_first = cb.next;
            }

            self.allocator.destroy( cb );

            self.updateNotifyEventBits() catch {};

            // ### TODO ### Can we turn off the notify stream?
            //  ### TODO ### self.notify_event_bits &= ~(1 << in_pin);

            return;
        }

        prior = cb;
        a_callback = cb.next;
    }
}

// -----------------------------------------------------------------------------
//  Public function: triggerEvent
// -----------------------------------------------------------------------------
/// Get the version number from the pigpiod daemon.

pub fn triggerEvent( self : *PiGPIO, in_event : u5 ) Error!void
{
    _ = try self.doCmd( .trigger_event, true, in_event, 0, null );
}

// =============================================================================
//  Private Functions
// =============================================================================

// -----------------------------------------------------------------------------
//  Function: extentFrom
// -----------------------------------------------------------------------------

fn extentFrom( T : type, in_val : * const T ) Extent
{
    const p : [*] const u8 = @ptrCast( in_val );

    const result : Extent = p[0..@sizeOf( T )];

    return result;
}

// -----------------------------------------------------------------------------
//  Function: Stream.doCmd
// -----------------------------------------------------------------------------
/// Run a transaction with a pigpiod daemon.  If the result value returned
/// by the daemon is negative, an appropriate error code is returned.
/// Otherwise the positive result is returned.

fn doCmd( self       : *PiGPIO,
          in_cmd     : Command,
          in_unlock  : bool,
          in_p1      : u32,
          in_p2      : u32,
          in_extents : ?[] const Extent ) Error!u32
{
    const result = try self.doCmdBasic( in_cmd,
                                        in_unlock,
                                        in_p1,
                                        in_p2,
                                        in_extents );

    const status : i32 = @bitCast( result );

    if (status < 0) return convertError( status );

    return result;
}

// -----------------------------------------------------------------------------
//  Function: Stream.doCmdBasic
// -----------------------------------------------------------------------------
/// Run a transaction with a pigpiod daemon.  Returns the
/// result value without checking for error codes.

fn doCmdBasic( self       : *PiGPIO,
               in_cmd     : Command,
               in_unlock  : bool,
               in_p1      : u32,
               in_p2      : u32,
               in_extents : ?[] const Extent ) Error!u32
{
    var   hdr : Header = .{ .cmd = @intFromEnum( in_cmd ),
                            .p1  = in_p1,
                            .p2  = in_p2,
                            .p3  = 0 };

    if (in_extents) |e|
    {
        for (e) |an_extent|
        {
            std.debug.assert( hdr.p3 + an_extent.len < 0xFFFF_FFFF );

            hdr.p3 += @intCast( an_extent.len );
        }

        // log.debug( "send hdr.p3 = {}", .{ hdr.p3 } );
    }

    self.cmd_mutex.lock();
    errdefer self.cmd_mutex.unlock();

    _ = try self.cmd_stream.write( std.mem.asBytes( &hdr ) );

    if (in_extents) |e|
    {
        for (e) |an_extent|
        {
            // log.debug( "send extent len: {} - {any} ", .{ an_extent.len, an_extent } );

            _ = try self.cmd_stream.write( an_extent );
        }
    }

    _ = try self.cmd_stream.read( std.mem.asBytes( &hdr ) );

    if (hdr.cmd != @intFromEnum( in_cmd )) return error.bad_recv;

    if (in_unlock) self.cmd_mutex.unlock();

    return hdr.p3;
}

// -----------------------------------------------------------------------------
// Function: addLevelCallback
// -----------------------------------------------------------------------------
/// Set a callback function that will be called if the pin's state
/// changes.

fn addLevelCallback( self       : *PiGPIO,
                     in_pin     : u5,
                     in_edge    : Edge,
                     in_func    : LevelCBFunc,
                     in_context : ?*anyopaque ) !void
{
    const cb = try self.allocator.create( LevelCallback );
    errdefer self.allocator.destroy( cb );

    cb.* = .{
                .next    = self.level_cb_first,
                .pin     = in_pin,
                .edge    = in_edge,
                .func    = in_func,
                .context = in_context
            };

    self.list_mutex.lock();
    defer self.list_mutex.unlock();

    const start_notify =    self.level_cb_first  == null
                        and self.event_cb_first  == null;

    self.level_cb_first = cb;

    if (start_notify)
    {
      self.notify_thread = try std.Thread.spawn( .{}, notifyThread, .{ self } );
    }
    else
    {
        try self.updateNotifyLevelBits();
    }
}

// -----------------------------------------------------------------------------
// Function: removeLevelCallback
// -----------------------------------------------------------------------------
/// Set a callback function that will be called if the pin's state
/// changes.

fn removeLevelCallback( self       : *PiGPIO,
                        in_pin     : u5,
                        in_edge    : Edge,
                        in_func    : LevelCBFunc,
                        in_context : ?*anyopaque )void
{
    log.debug( "Started removeLevelCallback", .{} );
    defer log.debug( "Finished removeLevelCallback", .{} );

    self.list_mutex.lock();
    defer self.list_mutex.unlock();

    var a_callback = self.level_cb_first;
    var prior : ?*LevelCallback  = null;

    while (a_callback) |cb|
    {
        if (    cb.pin     == in_pin
            and cb.func    == in_func
            and cb.context == in_context
            and cb.edge    == in_edge)
        {
            if (prior) |p|
            {
                p.next = cb.next;
            }
            else
            {
               self.level_cb_first = cb.next;
            }

            self.allocator.destroy( cb );

            self.updateNotifyLevelBits() catch {};

            // ### TODO ### Can we turn off the notify stream?
            //  ### TODO ### self.notify_level_bits &= ~(1 << in_pin);


            return;
        }

        prior = cb;
        a_callback = cb.next;
    }
}

// -----------------------------------------------------------------------------
// Function: updateNotifyLevelBits
// -----------------------------------------------------------------------------

fn updateNotifyLevelBits( self : *PiGPIO ) !void
{
    // Note to self: don't lock the mutex, we should be
    // running under one already.

    var a_callback = self.level_cb_first;

    var bits : u32 = 0;

    while (a_callback) |cb|
    {
        bits |= @as( u32, 1 ) << cb.pin;

        a_callback = cb.next;
    }

    if (bits != self.notify_level_bits)
    {
        self.notify_level_bits = bits;

        // log.debug( "level_bits_set {X} {X}", .{ self.notify_handle, bits } );

        _ = try self.doCmd( .level_bits_set, true, self.notify_handle, bits, null );
    }
}

// -----------------------------------------------------------------------------
// Function: updateNotifyEventBits
// -----------------------------------------------------------------------------

fn updateNotifyEventBits( self : *PiGPIO ) !void
{
    // Note to self: don't lock the mutex, we should be
    // running under one already.

    var a_callback = self.event_cb_first;

    var bits : u32 = 0;

    while (a_callback) |cb|
    {
        bits |= @as( u32, 1 ) << cb.event;

        a_callback = cb.next;
    }

    if (bits != self.notify_event_bits)
    {
        self.notify_event_bits = bits;

        // log.debug( "event_bits_set {X} {X}", .{ self.notify_handle, bits } );

        _ = try self.doCmd( .events_bits_set, true, self.notify_handle, bits, null );
    }
}
// -----------------------------------------------------------------------------
//  Function: notifyThread
// -----------------------------------------------------------------------------

fn notifyThread( self : *PiGPIO ) void
{
    const Report  = extern struct
    {
        seqno : u16 = 0,
        flags : u16 = 0,
        tick  : u32 = 0,
        level : u32 = 0,
    };

    var report     : Report = .{};
    var last_level : u32    = 0;

    const buf : [*]u8 = @ptrCast( &report );

    var notify_stream = std.net.tcpConnectToHost( self.allocator,
                                                  self.address,
                                                  self.port ) catch |err|
    {
        log.err( "Failed to open notify stream: {}", .{ err } );
        return;
    };

    defer notify_stream.close();

    var   hdr : Header = .{ .cmd = @intFromEnum( Command.begin_notify ),
                            .p1  = 0,
                            .p2  = 0,
                            .p3  = 0 };

    self.cmd_mutex.lock();

    _ = notify_stream.write( std.mem.asBytes( &hdr ) ) catch |err|
    {
        log.err( "Failed initial write to notify stream: {}", .{ err } );

        self.cmd_mutex.unlock();

        return;
    };

    _ = notify_stream.read(  std.mem.asBytes( &hdr ) ) catch |err|
    {
        log.err( "Failed initial read from notify stream: {}", .{ err } );

        self.cmd_mutex.unlock();

        return;
    };

    self.cmd_mutex.unlock();

    self.notify_handle = hdr.p3;

    self.updateNotifyLevelBits() catch |err|
    {
        log.err( "updateNotifyLevelBits error {}", .{ err } );
    };

    self.updateNotifyEventBits() catch |err|
    {
        log.err( "updateNotifyEventBits error {}", .{ err } );
    };

    runloop: while(   self.level_cb_first != null
                  or self.event_cb_first != null)

    {
        var pollfd = [_]std.posix.pollfd{ .{ .fd     = notify_stream.handle,
                                            .events  = std.posix.POLL.IN,
                                            .revents = 0 } };

        const poll_result = std.posix.poll( &pollfd, 1000 ) catch |err|
        {
            log.err( "poll error {}", .{ err } );

            break :runloop;
        };

        if (poll_result <= 0) continue :runloop;


        const status = notify_stream.read( buf[0..12] ) catch |err|
        {
            log.err( "Failed reading notify stream: {}", .{ err } );
            break;
        };

        if  (status == 0) break;

        if (report.flags == 0)
        {
            const changed = (report.level ^ last_level) & self.notify_level_bits;

            last_level = report.level;

            var p = self.level_cb_first;

            self.list_mutex.lock();
            defer self.list_mutex.unlock();

            while (p) |a_callback|
            {
                const a_pin_mask = @as( u32, 1 ) << a_callback.pin;

                if ((changed & a_pin_mask) != 0)
                {
                    const level = (report.level & a_pin_mask) != 0;

                    if (    a_callback.edge == .either
                        or (a_callback.edge == .rising  and  level)
                        or (a_callback.edge == .falling and !level))
                    {
                        a_callback.func( Pin{ .pin = a_callback.pin, .gpio = self },
                                         @enumFromInt( @intFromBool( level ) ),
                                         report.tick,
                                         a_callback.context );
                    }
                }
                p = a_callback.next;
            }
        }
        else if ((report.flags & PI_NTFY_FLAGS_WDOG) != 0)
        {
            var p = self.level_cb_first;

            self.list_mutex.lock();
            defer self.list_mutex.unlock();

            while (p) |a_callback|
            {
                if (a_callback.pin == (report.flags & 0x1F))
                {
                    a_callback.func( Pin{ .pin = a_callback.pin, .gpio = self },
                                     .timeout,
                                     report.tick,
                                     a_callback.context );
                }
                p = a_callback.next;
            }
        }
        else if ((report.flags & PI_NTFY_FLAGS_EVENT) != 0)
        {
            var p = self.event_cb_first;

            self.list_mutex.lock();
            defer self.list_mutex.unlock();

            while (p) |a_callback|
            {
                if (a_callback.event == (report.flags & 0x1F))
                {
                    a_callback.func( self,
                                     @intCast( report.flags & 0x1F ),
                                     report.tick,
                                     a_callback.context );
                }
                p = a_callback.next;
            }
        }
    }
}

// -----------------------------------------------------------------------------
//  Function: convertError
// -----------------------------------------------------------------------------
/// Convert an error status returnd from the pidpiod daemon into its
/// coresponding Zig error.

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
//  GPIO Pin Control
// =============================================================================

/// This structure manages a GPIO pin.

pub const Pin = struct
{
    gpio  : *PiGPIO,
    /// Broadcom pin number
    pin   : u6,

    // -------------------------------------------------------------------------
    //  Function: Pin.setHigh
    // -------------------------------------------------------------------------
    ///  Set the pin output to the high state.
    ///
    ///  Does nothing if the pin is not set to be an output pin.

    pub fn setHigh( self : Pin ) Error!void
    {
        _ = try self.gpio.doCmd( .pin_set, true, self.pin, 1, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setLow
    // -------------------------------------------------------------------------
    ///  Set the pin output to the low state.
    ///
    ///  Does nothing if the pin is not set to be an output pin.

    pub fn setLow( self : Pin ) Error!void
    {
        _ = try self.gpio.doCmd( .pin_set, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.set
    // -------------------------------------------------------------------------
    ///  Set the pin output state based on boolean parameter.
    ///
    ///  Does nothing if the pin is not set to be an output pin.

    pub fn set( self : Pin, in_value : bool ) Error!void
    {
        _ = try self.gpio.doCmd( .pin_set,
                                 true,
                                 self.pin,
                                 @intFromBool( in_value ),
                                 null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.get
    // -------------------------------------------------------------------------
    ///  Get the logic state of a pin.

    pub fn get( self : Pin ) Error!bool
    {
        return try self.gpio.doCmd( .pin_get, true, self.pin, 0, null ) != 0;
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.trigger
    // -------------------------------------------------------------------------
    ///  trigger a pulse on the gpio pin.
    ///
    /// Parameters:
    /// - in_width - pulse width in µS. (1 - 100)
    /// - in_level - pulse polarity.

    pub fn trigger( self : Pin, in_width : u32, in_level : bool ) Error!void
    {
        std.debug.assert( in_width >= 1  and  in_width <= 100 );

        const val : u32 = @intFromBool( in_level );
        const ext = [_]Extent{ extentFrom( u32, &val ) };

        _ = try self.gpio.doCmd( .pin_trigger,
                                 true,
                                 self.pin,
                                 in_width,
                                 &ext );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setMode
    // -------------------------------------------------------------------------
    /// Set the pin mode.

    pub fn setMode(  self : Pin, in_mode : Mode ) Error!void
    {
        _ = try self.gpio.doCmd( .mode_set,
                                 true,
                                 self.pin,
                                 @intFromEnum( in_mode ),
                                 null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getMode
    // -------------------------------------------------------------------------
    /// Get the current pin mode.

    pub fn getMode( self : Pin ) Error!Mode
    {
        const result = try self.gpio.doCmd( .mode_get, true, self.pin, 0, null );

        return @enumFromInt( result );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setPull
    // -------------------------------------------------------------------------
    /// Set the pin pull resistor state.

    pub fn setPull(  self : Pin, in_pull : Pull ) Error!void
    {
        _ = try self.gpio.doCmd( .pull_set,
                                 true,
                                 self.pin,
                                 @intFromEnum( in_pull ),
                                 null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setPadStrength
    // -------------------------------------------------------------------------
    /// Set the pin's drive strength.

    pub fn setPadStrength(  self : Pin, in_strength : u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .current_set, true, self.pin, in_strength, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPadStrength
    // -------------------------------------------------------------------------
    /// Get the pin's drive strength.

    pub fn getPadStrength(  self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .current_get, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setGlitchFilter
    // -------------------------------------------------------------------------
    /// Set the glitch filter value for the pin.
    ///
    /// level changes must be stable for the provided duration before being
    /// reported via callback, callbackEX or waitForEdge.
    ///
    /// Parameter:
    /// - in_steady - the number of microseconds the pin must be stable

    pub fn setGlitchFilter(  self : Pin, in_steady : u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .glitch_filter, true, self.pin, in_steady, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setNoiseFilter
    // -------------------------------------------------------------------------
    /// Set the noise filter value for the pin.
    ///
    /// level changes are ignored until a pins state has been stable
    /// for the supplied "steady" duration.  Thereafter all chages are
    /// reported to the "active" druation.
    ///
    /// Parameters:
    /// - in_steady - the number of microseconds the pin must be stable
    /// - in_active - the number of microseconds in the active period.

    pub fn setNoiseFilter(  self      : Pin,
                            in_steady : u32,
                            in_active : u32 ) Error!void
    {
        const ext = [_]Extent{ extentFrom( u32, &in_active ) };

        _ = try self.gpio.doCmd( .noise_filter, true, self.pin, in_steady,
                                 in_active,
                                 &ext );
    }

    // ==== PWM Functions ======================================================

    // -------------------------------------------------------------------------
    //  Function: Pin.setPWMFrequency
    // -------------------------------------------------------------------------
    /// Set the PWM frequency.

    pub fn setPWMFrequency( self : Pin, in_frequency : u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .pwm_freq_set,
                                 true,
                                 self.pin,
                                 in_frequency,
                                 null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPWMFrequency
    // -------------------------------------------------------------------------
    /// Get the currently set PWM frequency in Hz.

    pub fn getPWMFrequency( self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .pwm_freq_get, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setPWMRange
    // -------------------------------------------------------------------------
    /// Set the PWM range.  The range is the number that indicates a 100%
    /// duty cycle when pass the the setPWMDutyCycle function.

    pub fn setPWMRange( self : Pin, in_range : u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .pwm_rate_set, true, self.pin, in_range, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPWMRange
    // -------------------------------------------------------------------------
    /// Get the currently set PWM range.

    pub fn getPWMRange( self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .pwm_rate_get, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPWMRealRange
    // -------------------------------------------------------------------------
    /// Get the currently set PWM range.

    pub fn getPWMRealRange( self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .pwm_real_range, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setPWMDutyCycle
    // -------------------------------------------------------------------------
    /// Set the PWM duty cycle.  A value of 0 indicates a 0% duty cycle, and
    /// a value that matches the currenlty set range (default: 255) indicates
    /// a 100% duty cycle.

    pub fn setPWMDutyCycle( self : Pin, in_duty_cycle: u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .pwm_duty_set, true, self.pin, in_duty_cycle, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPWMDutyCycle
    // -------------------------------------------------------------------------
    /// Get the currently set PWM duty cycle value.

    pub fn getPWMDutyCycle( self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .pwm_duty_get, true, self.pin, 0, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setPWMDutyFractiom
    // -------------------------------------------------------------------------
    /// Sets the duty cycle based on a fraction that must be between 0 and
    /// 1 (inclusive).

    pub fn setPWMDutyFractiom( self             : Pin,
                               in_duty_fraction : f32 ) Error!void
    {
        std.debug.assert(     in_duty_fraction >= 0.0
                          and in_duty_fraction <= 1.0 );

        var range : f32 =   @floatFromInt( try self.getPWMRange() );

        range *= in_duty_fraction;

        return try self.setPWMDutyCycle( @intFromFloat( @round( range ) ) );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getPWMDutyFractiom
    // -------------------------------------------------------------------------
    /// Gets the duty cycle as a fraction that must be between 0 and
    /// 1 (inclusive).

    pub fn getPWMDutyFractiom( self : Pin ) Error!f32
    {
        const range : f32 = @floatFromInt( try self.getPWMRange() );
        const cycle : f32 = @floatFromInt( try self.getPWMDutyCycle() );

        return cycle / range;
    }

    // ==== Servo Functions ====================================================

    // -------------------------------------------------------------------------
    //  Function: Pin.setServoPulseWidth
    // -------------------------------------------------------------------------
    /// Set the servo pulse width.
    ///
    /// This function start sending servo control pulses at the rate of 50
    /// per second.
    ///
    /// The width should be between 500 (full anti-clockwise) and 2500 (full
    /// clockwise) with 1500 being the mid-point.   Set the width to zero to
    /// turn off pulses.

    pub fn setServoPulseWidth( self : Pin, in_width : u32 ) Error!void
    {
        _ = try self.gpio.doCmd( .servo_set, true, self.pin, in_width, null );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.getServoPulseWidth
    // -------------------------------------------------------------------------
    /// Get the current servo pulse width value.

    pub fn getServoPulseWidth( self : Pin ) Error!u32
    {
        return try self.gpio.doCmd( .servo_get, true, self.pin, 0, null );
    }

    // ==== Pin Callback Setup =================================================

    // -------------------------------------------------------------------------
    // Function: Pin.addLevelCallback
    // -------------------------------------------------------------------------
    /// Set a callback function that will be called if the pin's state
    /// changes.

    pub fn addLevelCallback( self       : Pin,
                             in_edge    : Edge,
                             in_func    : LevelCBFunc,
                             in_context : ?*anyopaque )!void
    {
        if (self.pin > 31) return error.bad_user_gpio;

        // We cannot set a callback on the .timeout value.  Use the
        // watchdog command instead.

        std.debug.assert( in_edge != .timeout );

        try self.gpio.addLevelCallback( @intCast( self.pin ),
                                        in_edge,
                                        in_func,
                                        in_context );
    }

   // -------------------------------------------------------------------------
    // Function: Pin.remove callback
    // -------------------------------------------------------------------------
    /// Set a callback function that will be called if the pin's state
    /// changes.

    pub fn removeLevelCallback( self       : Pin,
                                in_edge    : Edge,
                                in_func    : LevelCBFunc,
                                in_context : ?*anyopaque ) void
    {
        if (self.pin > 31) return;

        self.gpio.removeLevelCallback( @intCast( self.pin ),
                                       in_edge,
                                       in_func,
                                       in_context );
    }

    // -------------------------------------------------------------------------
    //  Function: Pin.setWatchdog
    // -------------------------------------------------------------------------
    ///  Set a watchdog timeout on a pine.

    pub fn setWatchdog( self : Pin, in_timeout : u32 ) Error!void
    {
        return try self.gpio.doCmd( .wdog_set,
                                    true,
                                    self.pin,
                                    in_timeout,
                                    null ) != 0;
    }
};


// =============================================================================
//  SPI Master Interface
// =============================================================================

/// This stucture sets up a communication channel with hardware spi pins.

pub const SPI = struct
{
    gpio      : ?*PiGPIO           = null,
    spi       : u32                = 0,
    allocator : std.mem.Allocator  = undefined,

    pub const Flags = packed struct
    {
        /// SPI mode select
        mode         : u2       = 0,
        /// Set true to make CS active high
        cs0_invert   : bool     = false,
        cs1_invert   : bool     = false,
        cs2_invert   : bool     = false,
        /// Set true to inhibit the chip select line
        cs0_inhibit  : bool     = false,
        cs1_inhibit  : bool     = false,
        cs2_inhibit  : bool     = false,
        /// Set true to select the SPI1 interface
        aux          : bool     = false,
        /// Select true for 3-wire operation (main interface only)
        three_wire   : bool     = false,
        /// Number of pad byte in 3-wire mode
        pad          : u4       = 0,
        /// Set true to transmit low order bit first (aux interface only)
        tx_lsb       : bool     = false,
        /// Set true to receive low order bif first (aux interface only)
        rx_lsb       : bool     = false,
        /// Set SPI word size
        word_size    : u6       = 0,
        unused       : u10      = 0,
    };

    // -------------------------------------------------------------------------
    //  Function: SPI.open
    // -------------------------------------------------------------------------
    /// Configure the selected channel for SPI master operation.
    ///
    /// Parameters:
    /// - in_gpio      - the PiGPIO instance to use.
    /// - in_allocator - an allocator to be use as needed.
    /// - in_channel   - the SPI channel to configure.
    /// - in_bit_rate  - the bit rate in Hz for the channel.
    /// - in_flags     - flags.
    ///
    /// There main SPI interface supports two channels and has the following
    /// pin assignments:
    /// - MSIO: 09
    /// - MSOI: 10
    /// - SCLK: 11
    /// - Channel 0 CS: 8
    /// - Channel 1 CS: 9
    ///
    /// Raspberry Pi's with the 40-pin headder also support an auxillary SPI
    /// interface (selected by setting the SPI.Flags aux bit true). It supports
    /// three channels and has the following pin assignments:
    /// - MSIO: 19
    /// - MSOI: 20
    /// - SCLK: 21
    /// - Channel 0 CS: 18
    /// - Channel 1 CS: 17
    /// - Channel 2 CS: 16
    ///
    /// For either interface, chip select signal can be inhibited by setting
    /// the apporpriate Flags.cs_inhibit value to true.  In such a case the
    /// channel nuber is ignored and the calling program must control the
    /// chip select itself.
    ///
    /// Note: it IS safe to call this function for an open SPI instance.  The
    ///       instance will be reconfigured to the specified settings.

    // ### TODO ### document SPI flag bits.

    pub fn open( self         : *SPI,
                 in_gpio      : *PiGPIO,
                 in_allocator : std.mem.Allocator,
                 in_channel   : u2,
                 in_bit_rate  : u32,
                 in_flags     : Flags ) (SPIError||error{OutOfMemory})!void
    {
        self.close(); // will do nothing if SPI not already open.

        const ext = [_]Extent{ extentFrom( u32, @ptrCast( &in_flags ) ) };

        self.spi  = try in_gpio.doCmd( .spi_open, true,
                                       in_channel,
                                       in_bit_rate,
                                       &ext );
        self.allocator = in_allocator;
        self.gpio      = in_gpio;
    }

    // -------------------------------------------------------------------------
    //  Function: SPI.close
    // -------------------------------------------------------------------------
    /// Close an open SPI channel.

    pub fn close( self : *SPI ) void
    {
        if (self.gpio) |gpio|
        {
            _ = gpio.doCmd( .spi_close, true, self.spi, 0, null ) catch |err|
            {
                log.warn( "SPI Deinit error (ignored): {}", .{ err } );
            };

            self.gpio = null;
        }
    }

    // -------------------------------------------------------------------------
    //  Function: SPI.read
    // -------------------------------------------------------------------------
    /// This function reads data from the SPI interface into the receive
    /// slice.
    ///
    /// Note: the SPI bus sends zeros when reading data.

    pub fn read( self         : SPI,
                 out_rx_slice : []u8 ) SPIError!void
    {
        std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .spi_read,
                                false,
                                self.spi,
                                @intCast( out_rx_slice.len ),
                                null );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );
        }
        else
        {
            return error.NotOpen;
        }
    }

    // -------------------------------------------------------------------------
    //  Function: SPI.write
    // -------------------------------------------------------------------------
    /// This function transmits data over an SPI interface.  It attempts
    /// to transmit all the data in the transmit slice.
    ///
    /// Note: any data received on the SPI bus during the write is ignored.

    pub fn write( self        : SPI,
                  in_tx_slice : [] const u8 ) SPIError!void
    {
        std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );

        const ext = [_]Extent{ in_tx_slice };

        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .spi_write, true, self.spi,  0, &ext );
        }
        else
        {
            return error.NotOpen;
        }
    }

    // -------------------------------------------------------------------------
    //  Function: SPI.transfer
    // -------------------------------------------------------------------------
    /// This function sends the slice transmit slice and simultaniously reads
    /// to the receive buffer.
    ///
    /// The caller must assure that receive slice is the same size as
    /// the transmit slice.  It is permissable for the receive slice to point
    /// to the same memory as the transmit slice.

    pub fn transfer( self          : SPI,
                     in_tx_slice   : [] const u8,
                     out_rx_slice  : []u8 ) SPIError!void
    {
        std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );
        std.debug.assert( in_tx_slice.len ==  out_rx_slice.len );

        const ext = [_]Extent{ in_tx_slice };

        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .spi_trasfer, false, self.spi, 0, &ext );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );
        }
        else
        {
            return error.NotOpen;
        }
    }
};

// =============================================================================
//  I2C Master Interface
// =============================================================================

pub const I2C = struct
{
    gpio      : ?*PiGPIO           = null,
    i2c       : u32                = 0,
    allocator : std.mem.Allocator  = undefined,

    // -------------------------------------------------------------------------
    //  Function: I2C.open
    // -------------------------------------------------------------------------
    /// Configure the selected channel for I2C master operation.
    ///
    /// Parameters:
    /// - in_gpio      - the PiGPIO instance to use.
    /// - in_allocator - an allocator to be use as needed.
    /// - in_interface - the I2C interface to configure (0 or 1)
    /// - in_address   - the slave address to communicate with.
    ///
    /// Note: it IS safe to call this function for an open I2C instance.  The
    ///       instance will be reconfigured to the specified settings.

    // ### TODO ### document SPI flag bits.

    pub fn open( self         : *I2C,
                 in_gpio      : *PiGPIO,
                 in_allocator : std.mem.Allocator,
                 in_interface : u1,
                 in_address   : u7 ) (SPIError||error{OutOfMemory})!void
    {
        self.close(); // will do nothing if I2C not already open.

        const zero : u32 = 0;
        const ext = [_]Extent{ extentFrom( u32, &zero ) };

        self.i2c  = try in_gpio.doCmd( .i2c_open, true,
                                       in_interface,
                                       in_address,
                                       &ext );
        self.allocator = in_allocator;
        self.gpio      = in_gpio;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.close
    // -------------------------------------------------------------------------
    /// Close an open I2C channel.

    pub fn close( self : *I2C ) void
    {
        if (self.gpio) |gpio|
        {
            _ = gpio.doCmd( .i2c_close, true, self.i2c, 0, null ) catch |err|
            {
                log.warn( "I2C Deinit error (ignored): {}", .{ err } );
            };

            self.gpio = null;
        }
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readRaw
    // -------------------------------------------------------------------------
    /// This function reads raw data from the I2C bus into the receive
    /// slice.

    pub fn readRaw( self         : I2C,
                    out_rx_slice : []u8 ) I2CError!u32
    {
        std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            const result = try gpio.doCmd( .i2c_rx_raw,
                                           false,
                                           self.i2c,
                                           @intCast( out_rx_slice.len ),
                                           null );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );

            return result;
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeRaw
    // -------------------------------------------------------------------------
    /// This function transmits raw over an I2C bus.

    pub fn writeRaw( self        : I2C,
                     in_tx_slice : [] const u8 ) I2CError!void
    {
        std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );

        const ext = [_]Extent{ in_tx_slice };

        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .i2c_tx_raw, true, self.i2c,  0, &ext );
            return;
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeQuick
    // -------------------------------------------------------------------------
    /// This sends just the address byte over the bus. The in_bit parameter's
    /// value is used to set the address's Rd/Wr bit.  This allows that quick
    /// transfer of a single bit of data to the slave

    pub fn writeQuick( self         : I2C,
                       in_bit       : u1 ) I2CError!void
    {
        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .i2c_tx_quick, true, self.i2c, in_bit, null );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeByte
    // -------------------------------------------------------------------------
    /// This sends a single byte to the slave

    pub fn writeByte( self         : I2C,
                      in_byte      : u8 ) I2CError!void
    {
        if (self.gpio) |gpio|
        {
            _ = try gpio.doCmd( .i2c_tx_raw_u8, true, self.i2c, in_byte, null );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readByte
    // -------------------------------------------------------------------------
    /// This reads a single byte from the slave

    pub fn readByte( self : I2C) I2CError!u8
    {
        if (self.gpio) |gpio|
        {
            return @intCast( try gpio.doCmd( .i2c_rx_raw_u8,
                                             true,
                                             self.i2c,
                                             0,
                                             null ) & 0xFF );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeByteData
    // -------------------------------------------------------------------------
    /// This sends a single byte to selected register on the slave

    pub fn writeByteData( self         : I2C,
                          in_register  : u8,
                          in_byte      : u8 ) I2CError!void
    {
        if (self.gpio) |gpio|
        {
            const data : u32 = in_byte;
            const ext = [_]Extent{ extentFrom( u32, @ptrCast( &data ) ) };

            _ = try gpio.doCmd( .i2c_tx_u8, true, self.i2c, in_register, &ext );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readByteData
    // -------------------------------------------------------------------------
    /// This reads a single byte from the slave

    pub fn readByteData( self : I2C, in_register : u8 ) I2CError!u8
    {
        if (self.gpio) |gpio|
        {
            return @intCast( try gpio.doCmd( .i2c_rx_u8,
                                             true,
                                             self.i2c,
                                             in_register,
                                             null ) & 0xFF );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeWordData
    // -------------------------------------------------------------------------
    /// This sends a 16-bit word to selected register on the slave

    pub fn writeWordData( self         : I2C,
                          in_register  : u8,
                          in_word      : u16 ) I2CError!void
    {
        if (self.gpio) |gpio|
        {
            const data : u32 = in_word;
            const ext = [_]Extent{ extentFrom( u32, @ptrCast( &data ) ) };

            _ = try gpio.doCmd( .i2c_tx_u16, true, self.i2c, in_register, &ext );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readWordData
    // -------------------------------------------------------------------------
    /// This reads a bit word from the slave

    pub fn readWordData( self : I2C, in_register : u8 ) I2CError!u16
    {
        if (self.gpio) |gpio|
        {
            return @intCast( try gpio.doCmd( .i2c_rx_u16,
                                             true,
                                             self.i2c,
                                             in_register,
                                             null ) & 0xFFFF );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeBlockData
    // -------------------------------------------------------------------------
    /// This sends a block of data to the selected register on the slave

    pub fn writeBlockData( self         : I2C,
                           in_register  : u8,
                           in_tx_slice  : [] const u8 ) I2CError!void
    {
        std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            const len : u32 = @intCast( in_tx_slice.len );
            const ext = [_]Extent{ extentFrom( u32, &len ), in_tx_slice };

            _ = try gpio.doCmd( .i2c_tx_block,
                                false,
                                self.i2c,
                                in_register,
                                &ext );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readBlockData
    // -------------------------------------------------------------------------
    /// This reads a block of data from the selected register on the slave

    pub fn readBlockData( self         : I2C,
                          in_register  : u8,
                          out_rx_slice : []u8 ) I2CError!u32
    {
        std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );

        const ext = [_]Extent{ out_rx_slice };

        if (self.gpio) |gpio|
        {
            const result =  try gpio.doCmd( .i2c_rx_block,
                                            false,
                                            self.i2c,
                                            in_register,
                                            &ext );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );

            return @intCast( result & 0xFF );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.writeI2CBlockData
    // -------------------------------------------------------------------------
    /// This sends a block of data to the selected register on the slave

    pub fn writeI2CBlockData( self         : I2C,
                              in_register  : u8,
                              in_tx_slice  : [] const u8 ) I2CError!void
    {
        std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            const len : u32 = @intCast( in_tx_slice.len );
            const ext = [_]Extent{ extentFrom( u32, &len ), in_tx_slice };

            _ = try gpio.doCmd( .i2c_tx_i2c_blk,
                                false,
                                self.i2c,
                                in_register,
                                &ext );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.readI2CBlockData
    // -------------------------------------------------------------------------
    /// This reads a block of data from the selected register on the slave

    pub fn readI2CBlockData( self         : I2C,
                             in_register  : u8,
                             out_rx_slice : []u8 ) I2CError!u32
    {
        std.debug.assert( out_rx_slice.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            const result =  try gpio.doCmd( .i2c_rx_i2c_blk,
                                            false,
                                            self.i2c,
                                            in_register,
                                            null );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );

            return @intCast( result & 0xFF );
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.procedureCall
    // -------------------------------------------------------------------------
    /// This reads a block of data from the selected register on the slave

    pub fn procedureCall( self         : I2C,
                          in_register  : u7,
                          in_tx_slice   : [] const u8,
                          out_rx_slice  : []u8 ) I2CError!u32
    {
        std.debug.assert( in_tx_slice.len <= 0xFFFF_FFFF );
        std.debug.assert( in_tx_slice.len ==  out_rx_slice.len );

        if (self.gpio) |gpio|
        {
            const len : u32 = @intCast( in_tx_slice.len );
            const ext = [_]Extent{ extentFrom( u32, &len ), in_tx_slice };


            const result =  try gpio.doCmd( .i2c_proc_call,
                                            false,
                                            self.i2c,
                                            in_register,
                                            &ext );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );

            return result;
        }

        return error.NotOpen;
    }

    // -------------------------------------------------------------------------
    //  Function: I2C.zip
    // -------------------------------------------------------------------------
    /// This function executes a sequence of I2C operations. The operations to
    /// be performed are specified by the contents of in_operations which
    /// contains the concatenated command codes and associated data.
    ///
    pub fn zip( self          : I2C,
                in_register   : u7,
                in_operations : [] const u8,
                out_rx_slice  : []u8 ) I2CError!u32
    {
        std.debug.assert( in_operations.len <= 0xFFFF_FFFF );

        if (self.gpio) |gpio|
        {
            const len : u32 = @intCast( in_operations.len );
            const ext = [_]Extent{ extentFrom( u32, &len ), in_operations };


            const result =  try gpio.doCmd( .i2c_zip,
                                            false,
                                            self.i2c,
                                            in_register,
                                            &ext );

            defer gpio.cmd_mutex.unlock();

            _ = try gpio.cmd_stream.read( out_rx_slice );

            return result;
        }

        return error.NotOpen;
    }
};

// =============================================================================
//  Testing
// =============================================================================
//  Since these test drive the GPIO pins, the tests rely on some specific
//  connection to the pins.
//
//  GPIO  6 - Connect an LED


const testing = std.testing;

test "Connection"
{
    var gpio : PiGPIO = .{};

    // -- Check for some common failures ---

    try testing.expectError( error.UnknownHostName,
                             gpio.connect( testing.allocator,
                                           "now.where",
                                           null ) );

    try testing.expectError( error.ConnectionRefused,
                             gpio.connect( testing.allocator,
                                           "localhost",
                                           31416 ) );

    // --- check for connect (and ipv6 address parsing) ---

    try gpio.connect( testing.allocator, "::", 8888 );
    defer gpio.disconnect();

    try testing.expectEqual( 0x7F00, try gpio.shell( "foo", "bar" ) );

    std.log.warn("\nTick: {!}", .{ gpio.getCurrentTick() });
    std.log.warn("\nHVer: {!X}", .{ gpio.getHardwareVersion() });
    std.log.warn("\nGVer: {!}", .{ gpio.getPiGPIOVersion() });
}

// -----------------------------------------------------------------------------

test "Block Transfer"
{
    var gpio : PiGPIO = .{};

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    _ = try gpio.readBank1();
    _ = try gpio.readBank2();

    try gpio.clearBank1( 1 << 6 );
    // try.gpio.clearBank2( 1 << 6  );

    try gpio.setBank1( 1 << 6  );
    // try.gpio.setBank2( 1 << 6  );

    try gpio.maskedUpdateBank1( 0xFFFF_FFFF, 1 << 6 );
    try gpio.maskedSetModeBank1( .input, 1 << 6 );
    try gpio.maskedSetPullBank1( .none, 1 << 6 );

}

// -----------------------------------------------------------------------------

test "custom command"
{
    // This test assumes that the pigpiod daemon is built with the
    // default custom1 and custom2 function.  It will fail if those
    // functions were replaced.

    var gpio : PiGPIO = .{};

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    try testing.expect( try gpio.custom1( 1, 2, "three" ) == 116 );

    var reply : [3]u8 = undefined;
    try testing.expect( try gpio.custom2( 1, "two", &reply ) == 3 );
    try testing.expectEqualStrings(  &reply, "owt" );
}

// -----------------------------------------------------------------------------

test "basic pin test"
{
  var   gpio : PiGPIO = .{};
  const digital_pin   = gpio.pin( 6 );
//   const pwm_pin       = gpio.pin(  6 );
//   const servi_pin     = gpio.pin(  6 );

  try gpio.connect( testing.allocator, null, null );
  defer gpio.disconnect();


  try digital_pin.setMode( .input );
  try digital_pin.setPull( .up );
  std.time.sleep( 100_000 );
  try testing.expect( try digital_pin.get() == true );
  try digital_pin.setPull( .down );
  std.time.sleep( 100_000 );
  try testing.expect( try digital_pin.get() == false );
  try digital_pin.setPull( .none );

  try digital_pin.setMode( .output );
  try digital_pin.setLow();
  std.time.sleep( 1_000_000_000 );
  try digital_pin.set( true );
  std.time.sleep( 1_000_000_000 );
  try digital_pin.set( false );
  std.time.sleep( 1_000_000_000 );
  try digital_pin.setHigh();
}

// -----------------------------------------------------------------------------

test "SPI"
{
    var gpio : PiGPIO = .{};
    var spi  : PiGPIO.SPI = .{};

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    try spi.open( &gpio, testing.allocator, 0, 6_000_000, .{} );
    defer spi.close();

    var  buffer : [32]u8 = .{ 0 } ** 32;

    try spi.read( &buffer );
    try spi.write( &buffer );
    try spi.transfer( &buffer, &buffer );
}

// -----------------------------------------------------------------------------

test "I2C"
{
    var gpio : PiGPIO     = .{};
    var i2c  : PiGPIO.I2C = .{};

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    try i2c.open( &gpio, testing.allocator, 0, 0x42 );
    defer i2c.close();

    var  buffer : [32]u8 = .{ 0 } ** 32;

    try i2c.writeRaw( &buffer );
    _ = try i2c.readRaw( &buffer );

    try i2c.writeQuick( 0x01 );

    try i2c.writeByte( 0xCD );
    _ = try i2c.readByte();

    try i2c.writeByteData( 1, 0xCD );
    _ = try i2c.readByteData( 1 );

    try i2c.writeWordData( 1, 0xABCD );
    _ = try i2c.readWordData( 1 );

    try i2c.writeBlockData( 1, &buffer );
    _ = try i2c.readBlockData( 1, &buffer );

    try i2c.writeI2CBlockData( 1, &buffer );
    _ = try i2c.readI2CBlockData( 1, &buffer );

    _ = try i2c.procedureCall( 1, &buffer, &buffer );
}

// -----------------------------------------------------------------------------

fn testLevelCB( in_pin     : PiGPIO.Pin,
                in_edge    : PiGPIO.Edge,
                in_tick    : u32,
                in_context : ?*anyopaque ) void
{
    _ = in_tick;
    _ = in_pin;

    std.log.info("\ncallback saw edge: {any}", .{ in_edge });

    if (in_context) |context|
    {
        const ctx : *PiGPIO.Edge = @ptrCast( context );
        ctx.* = in_edge;
    }
}


test "Level Callback"
{
    var gpio   : PiGPIO        = .{};
    var result : PiGPIO.Edge   = .either;
    var test_pin               = gpio.pin( 6 );

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    try test_pin.addLevelCallback( .rising, testLevelCB, @ptrCast( &result ) );
    defer test_pin.removeLevelCallback( .rising, testLevelCB, @ptrCast( &result ) );

    std.time.sleep( 100_000_000 );

    try test_pin.setMode( .input );
    try test_pin.setPull( .down );

    std.time.sleep( 100_000_000 );

    try test_pin.setPull( .up );

    std.time.sleep( 500_000_000 );

    try testing.expect( .rising == result );

}

// -----------------------------------------------------------------------------

fn testEventCB( in_gpio    : *PiGPIO,
               in_event   : u5,
               in_tick    : u32,
               in_context : ?*anyopaque ) void
{
    _ = in_tick;
    _ = in_gpio;

    std.log.info("\ncallback saw event: {}", .{ in_event });

    if (in_context) |context|
    {
        const ctx : *u8 = @ptrCast( context );
        ctx.* = in_event;
    }
}


test "Event Callback"
{
    var gpio : PiGPIO = .{};
    var result : u32 = 0;

    try gpio.connect( testing.allocator, null, null );
    defer gpio.disconnect();

    try gpio.addEventCallback( 13, testEventCB, @ptrCast( &result ) );
    defer gpio.removeEventCallback( 13, testEventCB, @ptrCast( &result ) );

    std.time.sleep( 100_000_000 );

    try gpio.triggerEvent( 13 );

    std.time.sleep( 100_000_000 );

    try testing.expectEqual( 13, result );

}