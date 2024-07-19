# RadarSimLib

<img src="./assets/radarsimlib.svg" alt="logo" width="200"/>

This library serves as the C wrapper for RadarSimCpp, offering a C interface to access its core functions, including GPU parallelization. For instructions on how to use this library, please refer to RadarSimM.

## Build

**Building `RadarSimLib` requires to access the source code of `RadarSimCpp`.**

- Windows CPU

```batch
build_win.bat --arch cpu
```

- Windows GPU

```batch
build_win.bat --arch gpu
```

- Linux CPU

```bash
./build_linux.sh --arch=cpu
```

- Linux GPU

```bash
./build_linux.sh --arch=gpu
```
