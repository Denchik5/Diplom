# Подробное локальное тестирование проекта `modular_reconfig_planner`

Ниже — пошаговая инструкция «с нуля» для Windows и Linux/macOS, чтобы проверить:
1) что проект собирается;
2) что offline-алгоритм реконфигурации работает;
3) что CoppeliaSim-сценарий отрабатывает плавно и без «рассыпания» модулей.

---

## 0. Что вы проверяете (критерии успеха)

После прохождения шагов должны выполняться условия:
- `planner_offline` запускается без ошибок и сохраняет CSV-метрики.
- `planner_coppelia --test-motion` показывает плавный переход `WIDE_STABLE -> LINE -> WIDE_STABLE`.
- На полном сценарии `full_mission.json` робот меняет конфигурации по типу среды.
- В `results/` появляются метрики по сценарию и `summary.csv`.

---

## 1. Подготовка окружения

## 1.1 Windows

1. Установите **CMake** (не ниже 3.16).
2. Установите **Visual Studio 2022 Community** с компонентом **Desktop development with C++**.
3. Установите **CoppeliaSim** (актуальная стабильная версия).
4. (Опционально, для real ZeroMQ) подготовьте `zmqRemoteApi` из поставки CoppeliaSim.

Проверки в `PowerShell`:
```powershell
cmake --version
cl
```
Если команды не находятся — перезапустите терминал или откройте *x64 Native Tools Command Prompt*.

## 1.2 Linux/macOS

1. Установите `cmake`, `g++`/`clang++`, `make` или `ninja`.
2. Установите CoppeliaSim.
3. (Опционально) подготовьте `zmqRemoteApi`.

Проверки:
```bash
cmake --version
g++ --version
```

---

## 2. Распаковка/получение проекта

1. Перейдите в каталог проекта `modular_reconfig_planner`.
2. Убедитесь, что есть ключевые папки:
   - `src/`, `include/`, `configurations/`, `scenarios/`, `weights/`, `data/motion/`, `docs/`.

Проверка:
```bash
# Linux/macOS
ls

# Windows PowerShell
Get-ChildItem
```

---

## 3. Сборка offline + stub (без ZeroMQ)

Это первый обязательный этап: он проверяет ядро алгоритма и структуру проекта.

### Linux/macOS
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Windows (Visual Studio генератор)
```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Ожидаемый результат:
- успешно собраны `planner_offline` и `planner_coppelia` (stub).
- нет ошибок компиляции.

---

## 4. Тест 1: Offline сценарий (алгоритм планирования)

Запустите полный сценарий:

### Linux/macOS
```bash
./build/planner_offline scenarios/full_mission.json
```

### Windows
```powershell
.\build\Release\planner_offline.exe scenarios/full_mission.json
```

Что проверять в выводе:
- есть классификация среды;
- выбрана целевая конфигурация;
- `Success: yes` для сегментов, где ограничения выполняются;
- указаны пути сохранения метрик.

Проверьте файлы:
- `results/full_mission_metrics.csv`
- `results/summary.csv`

---

## 5. Подключение real ZeroMQ API (если нужен запуск с CoppeliaSim)

1. Скопируйте из CoppeliaSim папку:
   `programming/zmqRemoteApi`
2. Поместите её в проект как:
   `external/zmqRemoteApi`

Проверьте наличие C++ клиента:
- `external/zmqRemoteApi/clients/cpp`

Соберите ZeroMQ-вариант:

### Linux/macOS
```bash
cmake -S . -B build_zmq -DCMAKE_BUILD_TYPE=Release -DUSE_COPPELIASIM_ZMQ=ON
cmake --build build_zmq -j
```

### Windows
```powershell
cmake -S . -B build_zmq -G "Visual Studio 17 2022" -A x64 -DUSE_COPPELIASIM_ZMQ=ON
cmake --build build_zmq --config Release
```

---

## 6. Подготовка сцены CoppeliaSim

Сделайте сцену строго по:
- `scenes/SCENE_SETUP.md`
- `docs/coppeliasim_stable_scene_setup.md`

Критично проверить имена объектов:
- `/Robot`
- `/Robot/Module_0` ... `/Robot/Module_7`
- `/Obstacles`
- `/TargetZone`
- (если включены коннекторы) `/Robot/Connectors`

Важно:
- Модули должны быть дочерними `/Robot`.
- Для стабильной демонстрации включён кинематический режим через настройки движения.

---

## 7. Тест 2: Проверка плавности движения (`--test-motion`)

Запустите CoppeliaSim сцену и нажмите Start Simulation (если требуется для вашей конфигурации API).

Вторым процессом запустите:

### Linux/macOS
```bash
./build_zmq/planner_coppelia --test-motion
```

### Windows
```powershell
.\build_zmq\Release\planner_coppelia.exe --test-motion
```

Ожидаемое поведение:
- переход `WIDE_STABLE -> LINE -> WIDE_STABLE`;
- нет резких скачков по позиции;
- нет «взрывов» физики и разлёта модулей;
- коннекторы визуально следуют за рёбрами графа.

Если есть рывки — проверьте `data/motion/demo_motion_settings.json`:
- `interpolation_steps: 260`
- `step_pause_ms: 10`
- `use_smooth_step: true`
- `use_safe_lift: true`

---

## 8. Тест 3: Полная миссия в CoppeliaSim

Запуск:

### Linux/macOS
```bash
./build_zmq/planner_coppelia scenarios/full_mission.json
```

### Windows
```powershell
.\build_zmq\Release\planner_coppelia.exe scenarios/full_mission.json
```

Что должно происходить по сегментам:
- `FLAT -> WIDE_STABLE`
- `NARROW_PASSAGE -> LINE`
- `ROUGH_SURFACE -> SNAKE`
- `STEP -> CLIMB`
- `GAP -> BRIDGE`
- `TARGET_ZONE -> MANIPULATOR`

Проверьте метрики:
- `results/full_mission_metrics.csv`
- `results/summary.csv`

---

## 9. Рекомендуемый протокол для диплома (как оформить эксперимент)

Для каждого прогона фиксируйте:
1. Дата/время запуска.
2. Версия кода (commit hash).
3. Какой сценарий запускался.
4. Настройки движения (`demo_motion_settings.json`).
5. Результаты CSV (успех, число шагов, total_cost, minimum_clearance).

Минимум 3 прогона на сценарий:
- вычислите среднее `total_cost` и `number_of_steps`;
- укажите отклонения/нестабильные кейсы;
- приложите скриншоты сцены для каждого типа среды.

---

## 10. Частые проблемы и быстрые решения

1. **`Cannot open scenario file`**
   - Запуск не из корня проекта или неверный путь к `scenarios/*.json`.

2. **`Missing object /Robot/Module_i`**
   - Неверные имена в сцене или модуль не под `/Robot`.

3. **Дёрганое движение**
   - Увеличить `interpolation_steps` до 300–360.
   - Проверить `use_smooth_step=true`.
   - Добавить `step_pause_ms=10..20`.

4. **Модули «взрываются» физикой**
   - Убедиться, что demo-режим отключает динамику/respondable;
   - проверить, что в сцене нет конфликтующих child script, переустанавливающих physics параметры.

5. **Не стартует real ZeroMQ**
   - Проверить `external/zmqRemoteApi/clients/cpp`;
   - пересобрать `build_zmq` с `-DUSE_COPPELIASIM_ZMQ=ON`.

---

## 11. Мини-чеклист перед защитой

- [ ] `planner_offline` успешно отрабатывает `full_mission.json`.
- [ ] `planner_coppelia --test-motion` визуально плавный.
- [ ] `planner_coppelia scenarios/full_mission.json` проходит все сегменты.
- [ ] В `results/` есть актуальные CSV с текущей датой.
- [ ] Есть скриншоты/видео демонстрации для презентации.
