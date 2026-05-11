# Stable CoppeliaSim scene setup for kinematic demo mode

This project demonstrates a SMORES-like modular robot using a simplified kinematic model. The scene is not intended to reproduce the full contact dynamics, magnetic docking or latch mechanics of a real SMORES robot. The goal is to show stable, repeatable and visually clear reconfiguration produced by the planning algorithm.

## Required scene tree

The scene must contain the following objects:

```text
/Robot
/Robot/Module_0
/Robot/Module_1
/Robot/Module_2
/Robot/Module_3
/Robot/Module_4
/Robot/Module_5
/Robot/Module_6
/Robot/Module_7
/Robot/Connectors
/Obstacles
/TargetZone
```

`/Robot` must be a root dummy object. All module cuboids must be direct children of `/Robot`. `/Robot/Connectors` must also be a child of `/Robot`.

The program searches for module paths exactly as shown above. Names such as `Module0`, `module_0`, `Robot_Module_0`, `Module 0` or `/world/Robot/Module_0` are not valid for this demo.

## Module geometry

Each module is represented by a cuboid:

```text
length X = 0.22 m
width  Y = 0.18 m
height Z = 0.10 m
```

For the initial WIDE_STABLE form, use local coordinates relative to `/Robot`:

```text
Module_0: x=0.00, y=-0.12, z=0.05
Module_1: x=0.22, y=-0.12, z=0.05
Module_2: x=0.44, y=-0.12, z=0.05
Module_3: x=0.66, y=-0.12, z=0.05
Module_4: x=0.00, y= 0.12, z=0.05
Module_5: x=0.22, y= 0.12, z=0.05
Module_6: x=0.44, y= 0.12, z=0.05
Module_7: x=0.66, y= 0.12, z=0.05
```

The `z` coordinate is `0.05` because the module height is `0.10 m` and the coordinate is the cuboid center.

## Required module dynamics settings

For every module object `/Robot/Module_i`:

```text
Visible:       ON
Dynamic:       OFF
Respondable:   OFF
Collidable:    OFF for demonstration mode, or at least not used for motion control
```

The C++ runner tries to set `static/non-dynamic` and `non-respondable` properties through the ZeroMQ Remote API. If your CoppeliaSim version does not allow changing a particular property remotely, set it manually in Object properties before running the C++ program.

Reason: the program drives module poses directly with `setObjectPosition` and `setObjectOrientation`. If a shape is dynamic/respondable, the physics engine may try to resolve collisions while C++ teleports/interpolates the object, causing jumps, explosions or scene destruction.

## Obstacles and floor

For obstacles under `/Obstacles`:

```text
Static:       ON
Dynamic:      OFF
Respondable:  optional, but recommended OFF for purely visual demo obstacles
Visible:      ON
```

For the floor:

```text
Static:       ON
Respondable:  ON or OFF
```

The planning algorithm checks geometric constraints from JSON scenarios. The CoppeliaSim obstacles are primarily used for visualization and debugging.

## Visual connectors

Create a dummy:

```text
/Robot/Connectors
```

The C++ code tries to create visual cylinder connectors named `Connector_0`, `Connector_1`, ... automatically under `/Robot/Connectors`. If automatic creation fails in your CoppeliaSim version, create 16 cylinders manually:

```text
/Robot/Connectors/Connector_0
...
/Robot/Connectors/Connector_15
```

Recommended connector settings:

```text
Shape:         cylinder
Diameter:      0.03-0.05 m
Length:        1.0 m initially
Dynamic:       OFF
Respondable:   OFF
Visible:       ON
Color:         dark blue or dark grey
```

The C++ program updates connector positions and orientations at each interpolation step so that graph edges are shown between adjacent modules.

## How to run the stable demo

Do not start the simulation manually before running C++. The C++ program calls `startSimulation()` and enables stepping mode.

First run a simple test motion:

```powershell
.\build_zmq\Release\planner_coppelia.exe --test-motion
```

Expected result:

```text
WIDE_STABLE -> LINE -> WIDE_STABLE
```

The modules should move smoothly and should not jump, fall, scatter or push obstacles away.

Then run a full scenario:

```powershell
.\build_zmq\Release\planner_coppelia.exe .\scenarios\full_mission.json
```

## Troubleshooting checklist

- If modules jump or fly away, at least one `Module_i` is still Dynamic or Respondable.
- If the whole scene is destroyed, turn Respondable OFF for all modules and visual connectors.
- If the C++ program cannot find a module, check the exact path `/Robot/Module_i`.
- If the C++ program reports that a module is not a child of `/Robot`, drag it under the `Robot` dummy in the scene tree.
- If connectors are missing, create `/Robot/Connectors` and either allow C++ to create connectors or create `Connector_0 ... Connector_15` manually.
- If motion is too jerky, increase `interpolation_steps` in `data/motion/demo_motion_settings.json`.
- If you want to avoid any temporary lifting of modules, keep `use_safe_lift` set to `false`.
- If modules pass through obstacles visually, remember that demo mode is kinematic and obstacles are visual. The algorithmic collision checks are performed from JSON environment parameters.

## Recommended demonstration workflow

1. Open CoppeliaSim.
2. Load the clean scene.
3. Ensure the simulation is stopped.
4. Run `planner_coppelia --test-motion` from VS Code.
5. If the test is stable, run individual scenarios.
6. Finally run `full_mission.json`.
7. Save screenshots for the diploma.
8. Use `results/summary.csv` and scenario-specific CSV files for tables and plots.
