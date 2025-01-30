// zig fmt: off

// =============================================================================
//  Build the Curuvar PiGPIO Library
// =============================================================================

const std = @import( "std" );

pub fn build( b: * std.Build ) void
{
    const target   = b.standardTargetOptions( .{} );
    const optimize = b.standardOptimizeOption( .{} );

    // =========================================================================
    //  Create the pigpio module
    // =========================================================================

    _ = b.addModule( "zig-pigpio",
                     .{
                         .root_source_file = b.path( "src/pigpio.zig" ),
                         .target           = target,
                         .optimize         = optimize,
                       } );


    // =========================================================================
    //  Unit Tests
    // =========================================================================

    // -------------------------------------------------------------------------
    //  Add the tests in src/curuvar_lib.zig to the unit tests.
    // -------------------------------------------------------------------------

    const lib_unit_tests = b.addTest(
        .{
            .root_source_file = b.path( "src/pigpio.zig" ),
            .target           = target,
            .optimize         = optimize,
        } );

    const run_lib_unit_tests = b.addRunArtifact( lib_unit_tests );

    // -------------------------------------------------------------------------
    //  Add a step "test" to "zig build" which builds and runs the tests.
    // -------------------------------------------------------------------------

    const test_step = b.step( "test", "Run unit tests" );

    test_step.dependOn( &run_lib_unit_tests.step );
}
