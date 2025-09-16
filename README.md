# RadarSimLib

<img src="./assets/radarsimlib.svg" alt="logo" width="200"/>

This library serves as the C wrapper for RadarSimCpp, offering a C interface to access its core functions, including GPU parallelization. For instructions on how to use this library, please refer to RadarSimM.

## Features

- **High-fidelity radar simulation** with comprehensive modeling capabilities
- **C-compatible API** with opaque pointer pattern for C++ interoperability  
- **Automatic memory management** with cleanup registry for leak prevention
- **GPU acceleration support** for high-performance simulations
- **Thread-safe operations** with comprehensive error handling
- **Cross-platform compatibility** (Windows, Linux)

## Memory Management

RadarSimLib includes **automatic memory management** with three compatibility modes:

### Default Mode (Recommended)
- Lock-free implementation using STL containers
- Good balance of performance and compatibility
- Suitable for most single-threaded and multi-threaded applications

### Thread-Safe Mode
- Adds mutex locking for complete thread safety
- Use when multiple threads create/free objects simultaneously
- Enable with: `cmake .. -DRADARSIM_THREAD_SAFE=ON`

### Simple Mode (Maximum Compatibility)
- C-style arrays, no STL dependencies, no mutex
- Ideal for embedded systems or applications with strict requirements
- Enable with: `cmake .. -DRADARSIM_SIMPLE_CLEANUP=ON`

**Key Benefits:**
- Objects created with `Create_*` functions are automatically registered for cleanup
- Manual cleanup with `Free_*` functions is still recommended and will unregister objects
- Automatic cleanup occurs when the DLL/shared library is unloaded
- Same API works across all modes - only internal implementation changes

See [COMPILATION_OPTIONS.md](COMPILATION_OPTIONS.md) for detailed configuration guide.

## Build

**Building `RadarSimLib` requires to access the source code of `RadarSimCpp`.**

- Windows CPU

```batch
build_win.bat --arch=cpu
```

- Windows GPU

```batch
build_win.bat --arch=gpu
```

- Linux CPU

```bash
./build_linux.sh --arch=cpu
```

- Linux GPU

```bash
./build_linux.sh --arch=gpu
```
