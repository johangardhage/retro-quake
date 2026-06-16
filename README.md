# retro-quake

A simple Quake map viewer using software rendering.

![Screenshot](/screenshots/quake.png "quake")

## Prerequisites

To build retro-quake, you must first install the following tools:

- [Meson](https://mesonbuild.com/)
- [Ninja](https://ninja-build.org/)
- [GCC](https://gcc.gnu.org/) or [Clang](https://clang.llvm.org/)
- [SDL3](https://www.libsdl.org/)

### Install dependencies

#### openSUSE

`$ sudo zypper install meson ninja gcc-c++ SDL3-devel`

#### Ubuntu

`$ sudo apt install meson ninja-build g++ libsdl3-dev`

#### macOS

`$ brew install meson ninja pkg-config sdl3`

## Build instructions

To build the quake demo program, run:

```
$ meson setup build
$ meson compile -C build
```

The `build` directory will contain the demo program.

## Usage

```
Usage: quake [OPTION]...

Options:
 -h, --help           Display this text and exit
 -w, --window         Render in a window
     --fullwindow     Render in a fullscreen window
 -f, --fullscreen     Render in fullscreen
 -s, --stretch        Stretch the screen
     --nostretch      Use fixed resolution for rendering
 -v, --vsync          Enable sync to vertical refresh
     --novsync        Disable sync to vertical refresh
 -l, --linear         Render using linear filtering
     --nolinear       Render using nearest pixel sampling
 -c, --showcursor     Show mouse cursor
     --nocursor       Hide mouse cursor
     --showfps        Show frame rate in window title
     --nofps          Hide frame rate
     --capfps=VALUE   Limit frame rate to the specified VALUE
```

## License

Licensed under MIT license. See [LICENSE](LICENSE) for more information.

## Authors

* Johan Gardhage
