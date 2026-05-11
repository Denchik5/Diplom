# Создание сцены CoppeliaSim для демонстрации реконфигурации

## 1. Обязательная структура объектов

Создай пустую сцену CoppeliaSim и добавь следующие объекты:

```text
/world
/Robot
/Robot/Module_0
/Robot/Module_1
/Robot/Module_2
/Robot/Module_3
/Robot/Module_4
/Robot/Module_5
/Robot/Module_6
/Robot/Module_7
/Obstacles
/TargetZone
```

Имена модулей должны быть строго `Module_0`, `Module_1`, ..., `Module_7`. Если используется полный путь, он должен иметь вид `/Robot/Module_0`, ..., `/Robot/Module_7`.

## 2. Модель модуля

Для бакалаврской демонстрации используется упрощённая кинематическая модель SMORES-типа. Модуль можно создать как `Primitive shape > Cuboid` со следующими размерами:

```text
length = 0.22 m
width  = 0.18 m
height = 0.10 m
```

Рекомендуемая начальная форма — `WIDE_STABLE`, координаты которой заданы в `configurations/wide_stable.json`.

## 3. Элементы среды

Сцена должна содержать:

1. Пол.
2. Робота из 8 модулей.
3. Узкий проход из двух стен.
4. Ступеньку.
5. Зазор между двумя платформами.
6. Неровную поверхность.
7. Целевую зону.
8. Небольшой объект для демонстрации манипуляционной формы.
9. Камеру сверху.
10. Камеру сбоку.

Рекомендуемые группы:

```text
/Obstacles/LeftWall
/Obstacles/RightWall
/Obstacles/Step
/Obstacles/GapPlatformA
/Obstacles/GapPlatformB
/Obstacles/RoughBlock_0
/Obstacles/RoughBlock_1
/Obstacles/ManipulationObject
```

C++-планировщик получает тип среды из JSON-сценария. Геометрия препятствий в сцене нужна для визуальной проверки и отладки.

## 4. Запуск с C++

1. Открой сцену.
2. Не запускай симуляцию вручную: `planner_coppelia` вызывает `startSimulation()` сам.
3. Запусти:

```bash
./build_zmq/planner_coppelia scenarios/full_mission.json
```

## 5. Визуализация и отладка

Минимальная визуализация обеспечивается движением кубоидных модулей между целевыми положениями. Дополнительно можно:

- назначить разный цвет модулям для разных конфигураций;
- добавить child script из `coppeliasim/optional_scene_caption_child_script.lua` к dummy-объекту `/SceneStatus`;
- добавить линии траекторий средствами `sim.addDrawingObject` в Lua-скрипте;
- вывести текстовую подпись текущего сценария через Lua-функцию `setStatusText`.

## 6. Типовые ошибки

- Ошибка `cannot command /Robot/Module_0` означает, что объект назван иначе или не лежит по пути `/Robot/Module_0`.
- Если `planner_coppelia` сообщает, что ZeroMQ Remote API не включён, пересобери проект с `-DUSE_COPPELIASIM_ZMQ=ON` и проверь наличие `external/zmqRemoteApi`.
- Если движение модулей выглядит слишком резким, увеличь число интерполяционных шагов в `src/sim/CoppeliaSimZmqInterface.cpp`.

## Stable kinematic demo update

For the stable diploma demonstration, use the additional checklist in:

```text
docs/coppeliasim_stable_scene_setup.md
```

The C++ runner now supports `rigid_visual_reconfiguration` mode. In this mode modules are controlled kinematically through the ZeroMQ Remote API. Do not use dynamic/respondable modules for the demonstration. The `/Robot` object must be a root dummy, and all `Module_0 ... Module_7` objects must be direct children of `/Robot`.

Required additional object:

```text
/Robot/Connectors
```

The runner can create connector cylinders automatically under `/Robot/Connectors`. If automatic creation fails in your CoppeliaSim version, create `Connector_0 ... Connector_15` manually as non-dynamic, non-respondable cylinders.

Before running `full_mission.json`, execute:

```powershell
.\build_zmq\Release\planner_coppelia.exe --test-motion
```

If the modules move smoothly through `WIDE_STABLE -> LINE -> WIDE_STABLE`, the scene is ready for full mission execution.
