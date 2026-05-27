# FairinoCutTest: движение робота по точкам из файла

## Почему у вас ошибка MSB3644
Ошибка

`MSB3644: не найдены ссылочные сборки для .NETFramework,Version=v4.8.1`

означает, что на ПК **не установлен .NET Framework 4.8.1 Developer Pack (Targeting Pack)**.
Проект `FairinoCutTest` и SDK `FRRobot` собираются под `net481`, поэтому без targeting pack сборка невозможна.

## 1) Что обязательно установить на Windows
Установите **один** из вариантов:

1. **Visual Studio 2022** с компонентом:
   - `.NET Framework 4.8.1 SDK`
   - `.NET Framework 4.8.1 targeting pack`
2. Или **Build Tools for Visual Studio 2022** с теми же компонентами.
3. Или напрямую **.NET Framework 4.8.1 Developer Pack**:
   - https://aka.ms/msbuild/developerpacks

После установки обязательно **перезапустите PowerShell/терминал**.

## 2) Проверка, что targeting pack установлен
В PowerShell:

```powershell
Test-Path "C:\Program Files (x86)\Reference Assemblies\Microsoft\Framework\.NETFramework\v4.8.1"
```

Если команда вернула `True` — можно собирать проект.

## 3) Запуск симулятора и приложения
### 3.1 Поднимите контейнер симулятора
(как в вашем примере с `-p ...` и `--net fairino-net`).

### 3.2 Запустите приложение
Из каталога `FairinoCutTest`:

```powershell
dotnet restore
dotnet run -- 127.0.0.1 trajectory_infinity.json
```

> Если приложение запускается в другом контейнере, используйте IP/hostname из `fairino-net`, а не `127.0.0.1`.

## 4) Формат файла точек
Файл по умолчанию: `trajectory_infinity.json`.

Параметры:
- `tool`, `user` — номера инструмента/системы координат.
- `velocity`, `acceleration`, `ovl`, `blendRadius` — параметры движения.
- `waitMotionDone` — ждать завершения каждой точки.
- `motionDonePollMs` — период опроса завершения движения.
- `loopCount` — количество циклов: `0` = бесконечно.
- `points` — список поз `{x,y,z,rx,ry,rz}`.

## 5) Частые проблемы
- `RPC connect failed`:
  - неверный IP;
  - контейнер не поднят;
  - приложение не видит сеть/порт.
- Робот не двигается при успешном подключении:
  - проверь режимы симулятора;
  - проверь корректность `tool/user` и достижимость точек.

- Ошибка `err=14` / сообщение про изменение joint configuration или singular pose:
  - в первую точку лучше входить через `MoveJ`, а не сразу `MoveL`;
  - для каждой Cartesian-точки нужно считать IK относительно текущей конфигурации суставов.
