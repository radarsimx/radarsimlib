# RadarSimLib

<img src="https://raw.githubusercontent.com/radarsimx/.github/refs/heads/main/profile/radarsimlib.svg" alt="logo" width="200"/>

This library serves as the C wrapper for RadarSimCpp, offering a C interface to access its core functions, including GPU parallelization. For instructions on how to use this library, please refer to RadarSimM.

## Features

- **High-fidelity radar simulation** with comprehensive modeling capabilities
- **C-compatible API** with opaque pointer pattern for C++ interoperability  
- **Automatic memory management** with cleanup registry for leak prevention
- **GPU acceleration support** for high-performance simulations
- **Thread-safe operations** with comprehensive error handling
- **Cross-platform compatibility** (Windows, Linux)

## Build

**Building `RadarSimLib` requires to access the source code of `RadarSimCpp`.**

- Windows CPU

```batch
build.bat --arch=cpu
```

- Windows GPU

```batch
build.bat --arch=gpu
```

- Windows CPU with license verification

```batch
build.bat --arch=cpu --license=on
```

- Linux / macOS CPU

```bash
./build.sh --arch=cpu
```

- Linux / macOS GPU

```bash
./build.sh --arch=gpu
```

- Linux / macOS CPU with license verification

```bash
./build.sh --arch=cpu --license=on
```

### Prebuilt Dependency Source

RadarSimCpp links third-party libraries (HDF5, and mbedTLS when license
verification is enabled) as prebuilt static libraries. They are never compiled
as part of a RadarSimLib build; `--deps` only chooses where they are read from.

| `--deps` | Where the libraries come from | Network needed |
|---|---|---|
| `repo` (default) | The committed `libs/` tree in the `radarsimx-deps` submodule, reached through `src/radarsimcpp/deps` | No |
| `release` | The checksum-pinned release archives published by [`radarsimx/radarsimx-deps`](https://github.com/radarsimx/radarsimx-deps), downloaded and cached under `src/radarsimcpp/.deps-cache/` | Yes, on the first build |

```bash
# Linux/macOS
./build.sh --deps=release

# Windows
build.bat --deps=release
```

`repo` is the default because it needs no network and pins the dependencies to
the submodule commit, which makes it the right choice for local and offline
builds. The GitHub Actions workflows all pass `--deps=release`, so CI links the
same tagged, checksum-verified archives every time.
