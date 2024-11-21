// zig fmt: off
// DO NOT REMOVE ABOVE LINE -- I strongly dislike the way Zig formats code.

// =============================================================================
//  Build the PiGP-30 Emulator
// =============================================================================

const std = @import( "std" );

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.

pub fn build( b: * std.Build ) void
{
  const target   = b.standardTargetOptions( .{} );
  const optimize = b.standardOptimizeOption( .{} ); // default to .debug


  // ========================================================================
  //  Build the Executable PiGP-30
  // ========================================================================

  const exe = b.addExecutable(
    .{
      .name             = "pigpio-test",
      .root_source_file = .{ .path = "src/main.zig" },
      .target           = target,
      .optimize         = optimize,
  });

  // Add "zpigpio" (from build.zig.zon) as an importable dependency.

  const zpigpio =  b.dependency( "zpigpio", .{} ).module( "ZPIGPIO" );

  exe.root_module.addImport( "ZPIGPIO", zpigpio );

  // Indicate that his application should be installed in the final
  // installation directory.

  b.installArtifact( exe );

  // ------------------------------------------------------------------------
  //  Create a run step to run the installed application.
  // ------------------------------------------------------------------------

  const run_cmd = b.addRunArtifact( exe );

  run_cmd.step.dependOn( b.getInstallStep() );

  //  Pass command line args (after "--") to the run_cmd
  //  command itself, like this: `zig build run -- arg1 arg2 etc`

  if (b.args) |args|
  {
      run_cmd.addArgs( args );
  }

  //  Add the "run" command to "zig build" which invokes this step
  //  (e.g. "zig build. run")

  const run_step = b.step( "run", "Run the app" );

  run_step.dependOn( &run_cmd.step );

  // ========================================================================
  //  Unit Tests
  // ========================================================================

  // ------------------------------------------------------------------------
  //  Add a test step based on src/main.zig to the unit tests.
  // ------------------------------------------------------------------------

  const exe_unit_tests = b.addTest(
    .{
      .root_source_file = .{ .path = "src/main.zig" },
      .target           = target,
      .optimize         = optimize,
    } );

  //  Add the "test" command to "zig build" which invokes this step

  const run_exe_unit_tests = b.addRunArtifact( exe_unit_tests );
  const test_step          = b.step( "test", "Run unit tests" );

  test_step.dependOn( &run_exe_unit_tests.step );
}
