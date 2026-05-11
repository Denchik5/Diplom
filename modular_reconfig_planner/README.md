# Modular Reconfiguration Planner

Демонстрационный программно-имитационный комплекс для практической части ВКР по теме «Модели и алгоритмы реконфигурации автономных роботов с адаптивной кинематической структурой».

Проект реализует планирование реконфигурации упрощённой SMORES-подобной кинематической модели модульного робота. Робот состоит из 8 кубоидных модулей `Module_0` ... `Module_7`. Размер одного модуля: длина `0.22 м`, ширина `0.18 м`, высота `0.10 м`. Соединения между модулями задаются графом. Физически точная модель защёлок, магнитной стыковки и динамики SMORES не моделируется: реконфигурация задаётся целевыми положениями модулей и линейной интерполяцией между ними.

## 1. Назначение проекта

Комплекс демонстрирует алгоритм выбора целевой конфигурации модульного робота при прохождении участков среды:

- `FLAT` → `WIDE_STABLE`;
- `NARROW_PASSAGE` → `LINE`;
- `ROUGH_SURFACE` → `SNAKE`;
- `STEP` → `CLIMB`;
- `GAP` → `BRIDGE`;
- `TARGET_ZONE` → `MANIPULATOR`.

Если для типа среды допустимо несколько конфигураций, выбирается конфигурация с минимальной стоимостью.

## 2. Структура проекта

```text
include/core/                  модель данных: RobotGraph, ModuleState, Configuration, EnvironmentState, PlanningResult, Metrics
include/planning/              классификатор среды, ограничения, функция стоимости, планировщик
include/sim/                   общий интерфейс симулятора, offline/mock и CoppeliaSim ZeroMQ interface
include/logging/               CSV-логирование метрик
src/core/                      реализация модели робота и конфигураций
src/planning/                  реализация планировщика
src/sim/                       offline/mock, реальный ZeroMQ interface и stub без ZeroMQ
src/main_offline.cpp           offline runner
src/main_coppelia.cpp          CoppeliaSim runner
configurations/                библиотека целевых конфигураций
scenarios/                     сценарии экспериментов
weights/default_cost_weights.json  веса функции стоимости
docs/practical_chapter_draft.md    черновик текста практической главы
scenes/SCENE_SETUP.md          инструкция по созданию сцены CoppeliaSim
results/                       создаётся при запуске, содержит CSV-метрики
```

Каталоги `data/configurations`, `data/scenarios`, `data/weights` оставлены для обратной совместимости.

## 3. Как собрать проект

### Offline и stub-цель CoppeliaSim без внешних зависимостей

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

В этой сборке создаются:

- `build/planner_offline` — полноценный offline-режим;
- `build/planner_coppelia` — stub-версия, которая компилируется без CoppeliaSim и сообщает, как включить реальный ZeroMQ API.

Для удобства добавлены shell-wrapper-файлы:

```bash
./planner_offline scenarios/full_mission.json
./planner_coppelia scenarios/full_mission.json
```

Wrapper ожидает, что проект уже собран в каталоге `build`.

### Сборка с реальным CoppeliaSim ZeroMQ Remote API

1. Установить CoppeliaSim.
2. Скопировать каталог `programming/zmqRemoteApi` из установки CoppeliaSim в проект:

```text
external/zmqRemoteApi
```

3. Собрать с флагом:

```bash
cmake -S . -B build_zmq -DCMAKE_BUILD_TYPE=Release -DUSE_COPPELIASIM_ZMQ=ON
cmake --build build_zmq -j
```

Если каталог `external/zmqRemoteApi/clients/cpp` отсутствует, CMake выдаст понятную ошибку.

## 4. Как запустить offline mode

Из корня проекта после сборки:

```bash
./planner_offline scenarios/full_mission.json
```

Или напрямую из build-каталога:

```bash
cd build
./planner_offline scenarios/full_mission.json
```

Также поддерживаются необязательные аргументы:

```bash
./planner_offline <scenario.json> [config_dir] [weights.json] [results_dir]
```

Пример с явными путями:

```bash
./build/planner_offline scenarios/gap.json configurations weights/default_cost_weights.json results
```

## 5. Как запустить CoppeliaSim mode

1. Открыть сцену CoppeliaSim, созданную по `scenes/SCENE_SETUP.md`.
2. Убедиться, что объекты называются строго:

```text
/Robot/Module_0
/Robot/Module_1
...
/Robot/Module_7
```

3. Запустить собранную ZeroMQ-версию:

```bash
./build_zmq/planner_coppelia scenarios/full_mission.json
```

Runner выполняет:

1. подключение к CoppeliaSim;
2. поиск объектов `/Robot/Module_0` ... `/Robot/Module_7`;
3. чтение текущих поз модулей;
4. выбор конфигурации;
5. интерполяционное перемещение модулей;
6. пошаговую симуляцию;
7. сохранение метрик в CSV.

## 6. Как создать сцену CoppeliaSim

Подробная инструкция находится в `scenes/SCENE_SETUP.md`. Кратко сцена должна содержать:

- пол;
- dummy `/world`;
- dummy `/Robot`;
- 8 cuboid-модулей `/Robot/Module_0` ... `/Robot/Module_7` размером `0.22 x 0.18 x 0.10 м`;
- `/Obstacles`;
- две стены узкого прохода;
- ступеньку;
- две платформы с зазором;
- неровную поверхность;
- `/TargetZone`;
- небольшой объект для демонстрации манипуляционной формы;
- камеру сверху;
- камеру сбоку.

## 7. Сценарии

```text
scenarios/flat.json
scenarios/narrow_passage.json
scenarios/rough_surface.json
scenarios/step.json
scenarios/gap.json
scenarios/target_zone.json
scenarios/full_mission.json
```

`full_mission.json` задаёт последовательность:

```text
FLAT -> NARROW_PASSAGE -> ROUGH_SURFACE -> STEP -> GAP -> TARGET_ZONE
```

## 8. Функция стоимости

Используется вид:

```text
J = wT*T + wE*E + wC*C + wR*R + wS*S
```

где:

- `T` — оценка времени реконфигурации;
- `E` — оценка энергозатрат;
- `C` — число изменений соединений;
- `R` — риск столкновений;
- `S` — штраф за недостаточную устойчивость.

Веса задаются в `weights/default_cost_weights.json`:

```json
{"weights":{"wT":0.30,"wE":0.15,"wC":0.15,"wR":0.25,"wS":0.15}}
```

## 9. Метрики

Каждый запуск создаёт отдельный CSV и общий `summary.csv` в папке `results/`.

Ожидаемые имена файлов:

```text
results/flat_metrics.csv
results/narrow_passage_metrics.csv
results/rough_surface_metrics.csv
results/step_metrics.csv
results/gap_metrics.csv
results/target_zone_metrics.csv
results/full_mission_metrics.csv
results/summary.csv
```

Формат CSV:

```text
scenario_name,environment_type,selected_configuration,success,reconfiguration_time,number_of_steps,number_of_connection_changes,estimated_energy,collision_risk,stability_penalty,minimum_clearance,total_cost
```

## 10. Основные допущения

1. Используется упрощённая SMORES-подобная кинематическая модель, а не физически точная модель SMORES.
2. Модули представлены кубоидами.
3. Межмодульные соединения описываются графом.
4. Физическая защёлка и магнитная стыковка подробно не моделируются.
5. Реконфигурация выполняется через целевые положения модулей и интерполяцию.
6. Силовые ограничения учитываются оценочно через штраф устойчивости, энергозатраты и допустимость конфигурации.
7. Основной результат — демонстрация алгоритма планирования реконфигурации и воспроизводимые имитационные эксперименты.

## How to fix jumping modules and scene destruction in CoppeliaSim

The CoppeliaSim runner now contains a dedicated **Demo/Kinematic mode** for the diploma demonstration. In this mode the C++ program does not use forces, velocities or contact dynamics to move the modular robot. Instead, it directly updates module poses through the ZeroMQ Remote API and displays graph edges using visual connector rods.

This is intentional. The practical part demonstrates a planning algorithm for a SMORES-like modular robot, not a physically exact SMORES docking mechanism. The VCR task requires target configuration planning, software implementation, visualization/debugging and simulation-based experiments; the implemented demo mode supports these goals while avoiding unstable contact dynamics in CoppeliaSim.

### Why dynamic/respondable modules cause problems

If a module is a dynamic and respondable shape, the physics engine tries to resolve contacts and collisions while the C++ program also changes its pose directly with `setObjectPosition` and `setObjectOrientation`. These two control methods conflict. The result can be:

- modules jumping or flying away;
- walls, platforms or other objects being pushed away;
- unstable collision resolution;
- broken scene hierarchy;
- C++ runner termination after a Remote API error.

For the diploma demo, every `/Robot/Module_i` should therefore be treated as a kinematic visual element.

### Required object settings

For each module:

```text
/Robot/Module_0 ... /Robot/Module_7
```

set:

```text
Dynamic:      OFF
Respondable:  OFF
Visible:      ON
Collidable:   optional OFF for demo mode
```

For connector objects:

```text
/Robot/Connectors/Connector_0 ... Connector_15
```

set:

```text
Dynamic:      OFF
Respondable:  OFF
Visible:      ON
```

For obstacles, use static visual objects. They may remain visible in the scene, but they should not physically push the modules during the kinematic demo.

A full scene checklist is provided in:

```text
docs/coppeliasim_stable_scene_setup.md
```

### Motion settings

The CoppeliaSim demo reads motion parameters from:

```text
data/motion/demo_motion_settings.json
```

Default settings:

```json
{
  "motion_mode": "rigid_visual_reconfiguration",
  "interpolation_steps": 180,
  "step_pause_ms": 0,
  "use_smooth_step": true,
  "use_safe_lift": false,
  "safe_lift_height": 0.08,
  "update_connectors": true,
  "disable_module_dynamics": true,
  "disable_module_respondable": true,
  "move_robot_root_between_segments": true
}
```

Increase `interpolation_steps` to 220-240 if the motion still looks too fast. Enable `use_safe_lift` only for debugging transitions that visually pass through obstacles.

### Test motion

Before running the full mission, test the scene with:

```powershell
.\build_zmq\Release\planner_coppelia.exe --test-motion
```

Expected visual result:

```text
WIDE_STABLE -> LINE -> WIDE_STABLE
```

The scene is considered ready if modules move smoothly, connector rods follow the graph edges and no object jumps or scatters.

### Full CoppeliaSim run

After the test succeeds:

```powershell
.\build_zmq\Release\planner_coppelia.exe .\scenarios\full_mission.json
```

Expected sequence:

```text
FLAT            -> WIDE_STABLE
NARROW_PASSAGE  -> LINE
ROUGH_SURFACE   -> SNAKE
STEP            -> CLIMB
GAP             -> BRIDGE
TARGET_ZONE     -> MANIPULATOR
```

The C++ runner validates the scene before motion. If required objects are missing, it prints a clear error and stops instead of damaging the scene.

## 12. Что делать, если реконфигурация «дергается» или «рассыпается»

Для дипломной демонстрации в CoppeliaSim используйте **кинематический режим**, а не динамическую физику контактов:

1. Включите `disable_module_dynamics=true` и `disable_module_respondable=true`.
2. Увеличьте дискретизацию движения (`interpolation_steps >= 240`).
3. Добавьте паузу между шагами (`step_pause_ms=5..15`) для визуально непрерывного движения.
4. Включите `use_smooth_step=true` для S-образного профиля скорости (без рывков на старте/финише).
5. Включите `use_safe_lift=true`, чтобы переход выполнялся в 3 фазы: подъем → перенос → опускание.

Рекомендуемый профиль уже задан в `data/motion/demo_motion_settings.json`.

### Алгоритм реконфигурации, который реализует проект

На каждом сегменте среды выполняется следующая последовательность:

1. Считывание состояния среды (`EnvironmentState`) и текущего графа модулей (`RobotGraph`).
2. Классификация типа среды (`EnvironmentClassifier`).
3. Выбор целевой конфигурации через ограничения + функцию стоимости (`ConstraintChecker` + `CostFunction`).
4. Построение плана перехода (`ReconfigurationPlanner`).
5. Кинематическое исполнение плана в CoppeliaSim с интерполяцией поз модулей.
6. Логирование метрик в CSV.

Это корректно оформляет практическую часть как алгоритм реконфигурации, а не как ручную анимацию.
