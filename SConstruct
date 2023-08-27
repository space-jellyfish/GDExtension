env = SConscript('godot-cpp/SConstruct');

env.Append(PPPATH="Pathfinding/");

src = Glob("Pathfinding/*.cpp");

if env['platform'] == 'macos':
	libpath = 'libtest{}{}'.format( env['suffix'], env['SHLIBSUFFIX'] );
	sharedlib = env.SharedLibrary(libpath, src);
	Default(sharedlib);
