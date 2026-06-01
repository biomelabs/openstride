# Contributing to OpenStride

Thanks for taking a look. Patches, ideas, and bug reports are very welcome.

[**Read the docs**](https://biomelabs.github.io/openstride) before contributing to get up to speed on the project and set up your development environment.

## What to work on

See [**open issues**](https://github.com/biomelabs/openstride/issues) for ideas and good first tasks. Keep your proposed changes scoped to this repo; fixes to Zephyr itself belong upstream.

## Code style

Our C/C++ source under uses [clangd](https://clangd.llvm.org) as a language server. Instructions for automating this differ based on your IDE. For VSCode, you can install the [clangd extension](vscode:extension/llvm-vs-code-extensions.vscode-clangd) (id: `llvm-vs-code-extensions.vscode-clangd`). Once you run `make` and `build/compile_commands.json` exists, the `.clangd` and `.vscode/settings.json` files should help `clangd` find things properly. You may first need to run `clangd: Restart language server` in the command palette.

Where appropriate, match neighbouring Zephyr style (`LOG_*`, errno returns, etc.).

## Pull requests

- Branch from `main`, and submit one focused change per PR when possible.
- Run `make`.
- Test your code, preferably on hardware if you are able.
- Write a clear pull request title and description. Say what changed, why, how you tested, and any caveats.

## ANT+ and proprietary code

Make sure **NOT** to commit ANT+ SDK sources, keys, or other confidential Garmin/Nordic material.

As best as possible, try to keep ANT-specific interfaces and code self-contained inside the transport layer. Ideally, all non-transport code should be protocol-agnostic.