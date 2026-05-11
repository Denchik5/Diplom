# Kinematic CoppeliaSim demo fix report

## Initial project analysis

Important files found in the project:

- `include/sim/CoppeliaSimZmqInterface.h` — C++ interface class for the real CoppeliaSim ZeroMQ mode.
- `src/sim/CoppeliaSimZmqInterface.cpp` — implementation of scene connection, reading/writing module poses and applying configurations through ZeroMQ Remote API.
- `src/sim/CoppeliaSimZmqInterfaceStub.cpp` — stub implementation used when the project is built without the real ZeroMQ client.
- `src/main_coppelia.cpp` — command-line runner for CoppeliaSim mode. It loads configurations, scenarios, weights, creates the planner and executes the plan in CoppeliaSim.
- `src/main_offline.cpp` — offline runner. It is not modified by the kinematic demo fix.
- `include/sim/ISimInterface.h` — common simulation interface used by the runners.
- `src/sim/MockCoppeliaSimInterface.cpp` — mock/offline simulation interface.
- `configurations/*.json` — target robot configurations: WIDE_STABLE, LINE, SNAKE, CLIMB, BRIDGE, MANIPULATOR.
- `scenarios/*.json` — environment scenarios.
- `scenes/SCENE_SETUP.md` — original scene setup guide.
- `docs/coppeliasim_stable_scene_setup.md` — new stable scene setup guide for demo mode.
- `data/motion/demo_motion_settings.json` — new motion settings file for stable CoppeliaSim visualization.

## Locations of CoppeliaSim pose commands

Before the fix, module motion was primarily implemented in `src/sim/CoppeliaSimZmqInterface.cpp`:

- `getModulePose()` used `sim.getObjectPosition()` and `sim.getObjectOrientation()`.
- `setModulePose()` used `sim.setObjectPosition()` and `sim.setObjectOrientation()`.
- `moveModuleInterpolated()` moved one module at a time and stepped the simulation after each module update.
- `applyConfiguration()` updated modules sequentially and created a new `RemoteAPIClient` in several methods.

## Location of executePlan

`executePlan()` is implemented in:

```text
src/sim/CoppeliaSimZmqInterface.cpp
src/sim/CoppeliaSimZmqInterfaceStub.cpp
src/sim/MockCoppeliaSimInterface.cpp
```

For the real CoppeliaSim mode, the relevant implementation is `src/sim/CoppeliaSimZmqInterface.cpp`.

## Where configurations are defined

Configuration JSON files are located in:

```text
configurations/wide_stable.json
configurations/line.json
configurations/snake.json
configurations/climb.json
configurations/bridge.json
configurations/manipulator.json
```

They are loaded by:

```text
src/planning/ConfigurationLibrary.cpp
```

## Places that could destroy the scene

The most likely causes of scene destruction were:

1. Dynamic/respondable module cuboids were moved directly with `setObjectPosition`.
2. A new `RemoteAPIClient` was created inside multiple interface methods instead of using one persistent client.
3. Modules were moved one by one, with a simulation step after each individual module, instead of synchronous group updates.
4. No stable kinematic demo mode was enforced before motion.
5. No preflight scene validation existed, so incorrect object hierarchy could produce unstable behavior or Remote API errors.
6. Visual connectivity was not updated during transitions, so modules could look disconnected.

## Files changed

- `include/sim/CoppeliaSimZmqInterface.h`
- `src/sim/CoppeliaSimZmqInterface.cpp`
- `src/sim/CoppeliaSimZmqInterfaceStub.cpp`
- `src/main_coppelia.cpp`
- `CMakeLists.txt`
- `README.md`
- `docs/practical_chapter_draft.md`
- `docs/coppeliasim_stable_scene_setup.md`
- `data/motion/demo_motion_settings.json`

## Validation performed

The offline and stub builds were checked with:

```bash
cmake -S . -B build_test -DCMAKE_BUILD_TYPE=Release
cmake --build build_test -j2
./build_test/planner_offline scenarios/full_mission.json
```

Expected result: offline mode still produces valid configuration choices and CSV metrics. Real CoppeliaSim ZeroMQ compilation must be checked on a machine that has CoppeliaSim, `external/zmqRemoteApi`, Boost, ZeroMQ and cppzmq installed.

- Подробный пошаговый гайд по локальному тестированию: `docs/local_testing_guide_ru.md`.
