project('retro-quake', 'cpp',
  default_options: ['warning_level=1', 'buildtype=debug', 'cpp_std=c++17'])

sdl3 = dependency('sdl3')

dependencies = [sdl3]
cpp_args = ['-DRETRO_WIDTH=320', '-DRETRO_HEIGHT=200']
link_args = []

if host_machine.system() == 'darwin'
  link_args += ['-Wl,-undefined,dynamic_lookup']
endif

executable('quake', [
    'src/quake.cpp',
  ],
  dependencies: dependencies,
  cpp_args: cpp_args,
  link_args: link_args)
