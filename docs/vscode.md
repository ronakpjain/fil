# VS Code with cortex-debug

[`cortex-debug`](https://github.com/Marus/cortex-debug) drives any GDB server
through its `"servertype": "external"` mode, so it runs `fil debug` without
extension changes: VS Code starts the server, attaches
`arm-none-eabi-gdb`, and speaks the GDB Remote Serial Protocol to fil.

Verified against cortex-debug 1.12.1 (VS Code Insiders) with `fil debug`:
launch, `load`, `monitor reset halt`, `runToEntryPoint`, breakpoints, stepping,
memory reads, and disconnect all work.

## Start the server

```bash
fil debug configs/boards/main_module.json --gdb-port 3333 --once
```

`--once` makes the server exit when VS Code disconnects, which matches a
debug-session lifetime. Omit it to keep the server and its telnet monitor
(default port 4444) alive between sessions, then stop it with Ctrl+C.

## launch.json

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "fil: Debug (cortex-debug)",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "external",
      "gdbTarget": "127.0.0.1:3333",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/path/to/firmware.elf",
      "device": "STM32G474RE",
      "runToEntryPoint": "main",
      "gdbPath": "arm-none-eabi-gdb",
      "preLaunchTask": "fil debug (gdb server)"
    }
  ]
}
```

Useful additional keys:

| Key | Purpose |
| --- | --- |
| `svdFile` | Peripheral register view (SVD decoding happens in the extension, reading memory over GDB). |
| `armToolchainPath` / `objdumpPath` | Toolchain lookup when the ARM tools are not on `PATH`. |
| `loadFiles` | Program `.hex`/`.bin` files instead of the executable. |
| `preLaunchCommands` / `postLaunchCommands` | Extra GDB commands around the launch sequence. |
| `overrideLaunchCommands` | Replace `load` + `monitor reset halt`, as recommended for external servers. |
| `showDevDebugOutput` | `"raw"` prints the full GDB/MI conversation for troubleshooting. |

`"request": "attach"` also works; cortex-debug then sends `monitor halt`, which
fil accepts as a no-op because the board is already stopped between sessions.

## tasks.json

The background task below starts fil and tells VS Code when the server is
listening. `--once` ends the task when the debug session disconnects.

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "fil debug (gdb server)",
      "type": "shell",
      "command": "fil",
      "args": [
        "debug",
        "${workspaceFolder}/path/to/board.json",
        "--gdb-port", "3333",
        "--once"
      ],
      "isBackground": true,
      "problemMatcher": {
        "owner": "fil",
        "pattern": { "regexp": "^$" },
        "background": {
          "activeOnStart": true,
          "beginsPattern": "fil debug:",
          "endsPattern": "GDB on .*:3333"
        }
      },
      "presentation": { "reveal": "silent", "panel": "dedicated" }
    }
  ]
}
```

If the server must outlive individual debug sessions, drop `--once` and add a
`postDebugTask` that stops it (`"command": "pkill -f 'fil debug' || true"`).

## Launch sequence and server support

For `servertype: "external"`, cortex-debug sends:

1. `target extended-remote <gdbTarget>` (from `gdbTarget`)
2. `load` (`target-download`)
3. `monitor reset halt`
4. a temporary breakpoint at `runToEntryPoint`, then `continue`

fil implements every step: extended-remote attach, memory writes for `load`,
`monitor reset halt` (and `monitor halt`), software/hardware breakpoints,
stepping, and Ctrl+C interrupts. The `Restart`/`Reset` buttons reuse
`monitor reset halt`.

## Boundaries

- **Live Watch, SWO, and RTT** are not available: they need probe-specific
  transport or extra ports that do not exist in an emulator. For reading
  memory while the firmware runs, use the telnet monitor instead
  (`nc 127.0.0.1 4444`, then `mdw`, `mdb`, `regs`). It never halts the target.
- **SVD peripheral views** work while the target is stopped; GDB cannot serve
  memory reads while it is running, so live register refresh is limited the
  same way it is with a hardware probe.
- **Multi-core/chained configurations** are not supported: `fil debug` controls
  one board, and network configs are rejected.
- Hardware watchpoints (`Z2`–`Z4`) are not implemented; GDB falls back to
  software watchpoints, which single-step through fil.
