# tiny-kepler

![Simulation Demo](./orbit.gif)

A single-header orbital mechanics and restricted 3-body problem (CR3BP) engine written in C. Includes a custom mission-description DSL parser, RK4 and Verlet integrators, and an event trigger system.

## Getting Started

This project relies on Git submodules for its dependencies (`tinyla` and `raylib`). Make sure to clone the repository recursively:

```bash
git clone --recursive git@github.com:marciorvneto/tiny-kepler.git
cd tiny-kepler
```

If you already cloned it without the submodules, you can fetch them by running:

```bash
git submodule update --init --recursive
```

## Building

The project uses a standard Makefile. Build artifacts are placed in the `./out` directory.

To build the CLI examples:

```bash
make
```

To build the graphical orbit viewer (note: this will compile `raylib` from source the first time you run it, which may take a minute):

```bash
make viewer
```

To clean the build directory:

```bash
make clean
```

## Running

Once compiled, you can run the binaries from the `out/` directory.

Run a mission file through the DSL parser:

```bash
./out/mission_parser examples/missions/reduced-3-body.mission
```

Run the graphical visualizer:

```bash
./out/orbit-viewer
```

## Project Structure

- `tiny-kepler.h`: The core single-header physics engine, integrator, and parser.
- `examples/`: Example applications, including the Raylib visualizer and basic orbital mechanics tests.
- `examples/missions/`: Sample `.mission` files written in the custom domain-specific language.
- `test-scripts/`: Python scripts for external plotting and testing.
- `vendor/`: Third-party dependencies (`tinyla` and `raylib`).
