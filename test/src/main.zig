
const std          = @import( "std" );
const GPIO         = @import( "zig_pigpio" );

var   gpa          = std.heap.GeneralPurposeAllocator(.{}){};
const allocator    = gpa.allocator();

var   gpio : GPIO  = .{};
const pin3         = gpio.pin( 3 );

pub fn main() !void
{
    try gpio.connect( allocator, null, null );
    defer gpio.disconnect();

    var foo : u32 = 4;
    const bar = &foo;

    // const bank1 = try gpio.readBank1();
    // std.log.debug( "Bank 1: 0x{x:0>8}", .{ bank1 } );

    try pin3.setHigh();

    foo = @intFromBool( try pin3.get() );

    const p = gpio.pin( bar.* );
    _ = try p.setLow();

    std.time.sleep( 2_000_000_000 );
}
