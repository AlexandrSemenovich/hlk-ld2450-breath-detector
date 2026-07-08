# План улучшения HLK-LD2450 Breath Detector

## Цель
Сделать ESP32 прозрачным форвардером сырых кадров радара и перенести тяжёлый
анализ дыхания на ПК (Python). Исправить ошибки корректности, мёртвый код,
протокол, улучшить детекцию на ПК и добавить тесты/документацию.

Конвенция координат (единая для firmware и monitor):
- `x`  — поперечная (латеральная) ось, мм, со знаком
- `y`  — продольная (вперёд от радара) ось, мм
- `depth`  = radial = sqrt(x^2 + y^2)  — ось дыхания (грудь движется вдоль r)
- `lateral` = x  — только для зоны/тепловой карты
- зона: R_MIN<=radial<=R_MAX и |x|<=SIDE_MAX

## Этап 1 — Критические исправления
- [x] 1. requirements.txt: добавить numpy, scipy
- [x] 2. ld2450_parser.cpp: убрать Serial.printf debug-спам + лишний #include <Arduino.h>
- [x] 3. main.cpp: единая конвенция координат (depth=radial, lateral=x); удалить computeDistAndDepth и его хак r<150 => 1000
- [x] 4. main.cpp: использовать pickTarget/isPresent/inZone (убрать мёртвый код и naive выбор первой не-нулевой цели)
- [x] 5. breath_detector.h: historySize 20Гц -> 10Гц; расширить zc-границы до 4..60 bpm

## Этап 2 — Протокол и архитектура (PC-centric)
- [x] 6. radar_bridge.h: добавить frame_id и ts_ms в D/S строки; обновить описание протокола
- [x] 7. main.cpp: ESP32 стал прозрачным форвардером — шлёт сырые цели
         R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,<x2>,<y2>,<spd2>,<res2>,
         <ts_ms>,<frame_id>; выбор цели и вся детекция дыхания перенесены на ПК
         (monitor.py: pick_target + Detrender + detect_breath). Удалены
         src/breath_detector.{cpp,h}, cpp_test/ (осиротевший C++-тест),
         native-env из platformio.ini.
- [x] 8. monitor.py: обновить regex под новый D/S; fs/окно FFT считать по ts радара (robust)

## Этап 3 — Улучшение PC-анализа
- [x] 9. monitor.py: backend matplotlib через env MPLBACKEND (TkAgg опционален)
- [x] 10. monitor.py: чтение сериала в фоновый поток + thread-safe буфер; отрисовка из буфера
- [x] 11. monitor.py: resample+Butterworth bandpass 0.12..0.5Гц + SNR/quality + апноэ по отсутствию пика
- [x] 12. monitor.py: согласовать зону/тепловую карту с depth=radial (проверить build_zone_patch)

## Этап 4 — Гигиена/инфра
- [x] 13. .gitignore: .venv, __pycache__, *.bin, *.elf, *.map
- [x] 14. удалить src/code.code-workspace
- [x] 15. radar_bridge.h begin(): убрать delay(1000)
- [x] 16. README.md: протокол, конвенция координат, запуск
- [x] 17. test/:
        - C++: `cpp_test/test_main.cpp` (self-contained, own main, no Unity) — собирается
          нативно через `src_dir = cpp_test` в `[env:native]`. Запуск:
          `pio run -e native` затем `.pio\build\native\program.exe`
          (вариант: `g++ -std=c++17 -I src cpp_test/test_main.cpp src/breath_detector.cpp -o t && t`).
        - Python: `python/tests/test_pc_fft.py` (чистая ф-ция `detect_breath`) — `pytest python/tests/`.
        (в этом окружении pio/зависимости не установлены, поэтому не исполнялись)
- [x] 18. неиспользуемые константы убраны при переписывании main.cpp (MOTION_SPEED_CMS, dbg-*, computeDistAndDepth);
        STATIONARY_SPEED_THRESHOLD/VALID_R*/ZONE_* используются

## Этап 5 — Отладка по реальному выводу (пользователь прислал лог)
- [x] 19. Анализ лога: протокол корректен (ts ~11Гц, frame_id последователен, без дропов),
        все нули = pickTarget не выбрал цель (никого в зоне / вне зоны / движется).
        Исправлено: pickTarget теперь отдаёт fallback — ближайшую присутствующую цель
        (видна на мониторе оранжевым маркером вне зоны для прицеливания);
        детектор питается только когда цель стационарна И в зоне.

## Этап 6 — Баг: маркер на тепловой карте / пропажа детекции
- [x] 20. Координаты LD2450 — signed-magnitude (магнитуда = raw&0x7FFF, знак в бите 15),
        НЕ two's complement. Моя правка на plain int16_t(raw) сломала детекцию: валидная
        координата превращалась в огромное число -> radial > VALID_R_MAX -> isPresent()=false
        -> цель отбрасывалась -> "всегда нули / перестал искать цель".
        Возвращён оригинальный signed-magnitude ((raw&0x8000) ? +mag : -mag) — симметричный
        и корректный. Мой flip знака лишь инвертировал "всегда +" в "всегда -"; суть не менял,
        т.к. тестовая цель стояла с одной стороны. lateral теперь реально подписан: цель с
        другой стороны даст противоположный знак. Если монтаж зеркалит лево/право — инвертировать
        знак в decodeCoord (и при необходимости в lateralOf), одна строка.

## Этап 7 — Выравнивание парсера с референсом (doc/LD2450.cpp)
- [x] 21. НАСТОЯЩИЙ баг найден по референсу: кадр LD2450 — фиксированные 30 байт
        (заголовок 4 + 3 цели по 8 байт, первая цель со смещения 4 + хвост 55 CC). В моём
        парсере цели читались со смещения 6 (`o = 6 + i*8`) — сдвиг на 2 байта, из-за чего
        координаты брались не из тех байтов. Исправлено: `nTargets=MAX_TARGETS(3)`,
        `need = 4 + 3*8 + 2 = 30`, `o = 4 + i*8`. Декод знака signed-magnitude оставлен
        (эквивалентен формуле из LD2450.cpp и проверен на примерах из PDF: x=-782, y=+1713).
        monitor.py менять не нужно — он парсит только строки D/S от ESP и берёт depth/lateral
        напрямую; после правки ESP шлёт корректно подписанные координаты.
        (опц.) можно синхронизировать ZONE_R_MAX в monitor.py (1200) с ESP (2500) для совпадения
         отрисовки зоны на тепловой карте — косметика, не влияет на детекцию.

## Этап 8 — Полный перенос детекции на ПК (прозрачный форвардер)
- [x] 22. ESP32 стал прозрачным форвардером сырых целей (см. п.7). Протокол
         теперь одна строка R на кадр со всеми 3 целями:
         `R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,<x2>,<y2>,<spd2>,<res2>,<ts_ms>,<frame_id>`.
- [x] 23. Удалён `src/breath_detector.{cpp,h}` — алгоритм дыхания теперь целиком
         в Python (`python/monitor.py` + `python/tests/test_pc_fft.py`). `radar_bridge.h` больше
         не зависит от `breath_detector.h`.
- [x] 24. Удалён осиротевший C++-тест `cpp_test/test_main.{cpp,txt}` и `[env:native]`
         из platformio.ini (сборка падала бы без main).
- [x] 25. python/monitor.py: PC выбирает стационарную цель в зоне (`pick_target`,
         radial=√(x²+y²)), детрендит дистанцию (`Detrender`, зеркало fw), кормит
         `detect_breath` (band-pass+FFT+BPM+quality+апноэ). Тепловая карта по
         x/radial. С монитора убрано сравнение FW BPM vs PY BPM — остался только
         PY BPM (FW-BPM был избыточен).
- [x] 27. Весь Python-код перенесён в папку `python/` (monitor.py, requirements.txt,
         tests/test_pc_fft.py, conftest.py для importable). Корень репозитория
         теперь содержит только firmware (src/, platformio.ini) и документацию.
- [x] 26. README.md: протокол/архитектура/тесты обновлены под форвардер.

## Этап 9 — Модульная перестройка python/ (расширяемость)
- [x] 28. `protocol.py` — ЕДИНСТВЕННОЕ место, знающее формат R-строки; `decode_line`
         возвращает `RawFrame` (dataclass Target x3 + ts_ms + frame_id). Парсер
         можно переписать не трогая остальной код.
- [x] 29. `analysis.py` — чистые функции без I/O: `pick_target`, `Detrender`,
         `detect_breath`, `build_zone_patch`, константы. Без matplotlib/serial.
- [x] 30. `state.py` — thread-safe буферы + `ingest(raw)` (выбор цели, детренд,
         лог парсинга в кольцевой `log_lines`) + `analyze()` (FFT). Лог виден в UI.
- [x] 31. `plots.py` — каждый график = класс-наследник `Panel` (`setup`/`update`);
         `LAYOUT` = список (row,col,PanelClass). Добавить/удалить график = правка
         LAYOUT. Добавлены панели: WavePlot, BreathPlot, HeatmapPlot, BarPlot,
         StatsPanel, LogPanel (окно лога парсинга ESP32).
- [x] 32. `app.py` — связка (поток чтения UART, сборка фигуры 3x2, анимация);
         `monitor.py` — тонкая точка входа. `app._sync` ждёт баннер "ready".
- [x] 33. Тесты обновлены: `test_pc_fft.py` импортирует `detect_breath` из analysis;
         добавлен `test_protocol.py` (декодер). Все модули импортируются (проверено
         заглушками тяжёлых зависимостей).
- [x] 34. Каждая панель показывает ВСЕ цели (3 слота), а не одну выбранную:
         `state.py` хранит историю по слотам (`depth/lateral/dist/ac/present[i]` +
         3 `Detrender`); выбранный слот (`selected`) подсвечивается и идёт в
         детекцию дыхания. WavePlot/BreathPlot — линии по целям (выбранная
         толще), HeatmapPlot — маркер на каждую присутствующую цель + плотность
         по всем, BarPlot — радиальная дистанция каждой цели, StatsPanel —
         сводка по целям. Цвета целей в `analysis.TARGET_COLORS`.
