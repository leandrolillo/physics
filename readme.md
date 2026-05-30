# physics

Header-only C++ physics utilities centered on particle simulation, force application, collision detection, and contact resolution.

## What is included

- `Particle`: movable body with position, velocity, acceleration, mass, damping, and a geometry-backed bounding volume
- `ParticleIntegrator`: advances active particles over time
- `Force` and `Gravity`: force abstraction plus a gravity implementation
- `CollisionDetector`: detects particle/particle and particle/scenery contacts
- `ContactResolver`: resolves penetration and separating velocity
- `ParticleManager`: coordinates forces, integration, collision detection, and contact resolution for a simulation step

## Project layout

- `/tmp/workspace/leandrolillo/physics/src/physics`: core particle, collision, and integration types
- `/tmp/workspace/leandrolillo/physics/src/forces`: force generators
- `/tmp/workspace/leandrolillo/physics/test`: Catch2-based unit tests

## Requirements

- CMake 3.22+
- A C++17 compiler
- Internet access during configure time so CMake can fetch external dependencies

## Build

```bash
cd /tmp/workspace/leandrolillo/physics
rm -rf build
cmake -S . -B build
cmake --build build
```

## Test

```bash
cd /tmp/workspace/leandrolillo/physics
ctest --test-dir build --output-on-failure
```

## Dependencies

This project uses CMake `FetchContent` to download:

- `geometry`
- `Catch2` for tests

## Current validation note

The repository build currently stops in a fetched math dependency before the local targets compile. The dependency is missing standard library includes such as `<stdexcept>` and `<cstring>` in `Math3d.h`, so `cmake --build build` does not currently complete in this environment.