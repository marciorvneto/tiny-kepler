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

## Mission DSL

Missions are defined in a custom domain-specific language (DSL). Its syntax is shown below.

```mission
SIM_TYPE   CR3BP
SIM_TIME   864000
SIM_DT     50

ENTITIES
BODY 1 Earth 5.972e24 6378
BODY 2 Moon  7.348e22 1737.4 384400

INITIAL_STATE
POS 0 6578 0 0
VEL 0 CIRCULAR(0, 1)

EVENTS
AT   3420                             MANEUVER DELTA_V 3.14529 PROGRADE
WHEN SPACECRAFT_WITHIN_DIST(1, 6578)  ONCE END_SIMULATION
```

At the moment only Classical Restricted 3-Body Problems (CR3BP) simulations are available.

## Running

Once compiled, you can run the binaries from the `out/` directory.

First, run a mission file through the DSL parser to compute the trajectory (this outputs to `./results.out` by default):

```bash
./out/mission_parser examples/missions/reduced-3-body.mission
```

Then, run the graphical visualizer to watch the simulation:

```bash
./out/orbit-viewer
```

_Note: The viewer defaults to reading `./results.out`, but you can optionally pass a custom file path:_

```bash
./out/orbit-viewer path/to/your_custom_results.out
```

### Viewer Controls

- **Arrow Keys:** Pan the camera
- **Mouse Wheel** or **+ / -**: Zoom in and out

## Project Structure

- `tiny-kepler.h`: The core single-header physics engine, integrator, and parser.
- `examples/`: Example applications, including the Raylib visualizer and basic orbital mechanics tests.
- `examples/missions/`: Sample `.mission` files written in the custom domain-specific language.
- `test-scripts/`: Python scripts for external plotting and testing.
- `vendor/`: Third-party dependencies (`tinyla` and `raylib`).
