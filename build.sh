#!/bin/bash

#===============================================================================
# Cross-Platform Build Script for RadarSimLib - Linux & macOS
#===============================================================================
#
# DESCRIPTION:
#   This script automates the build process for RadarSimLib on both Linux and 
#   macOS systems. It compiles the C++ library (libradarsimc.so/.dylib) with
#   comprehensive error handling and logging optimized for both platforms.
#
# REQUIREMENTS:
#   Linux:
#   - CMake 3.18 or higher
#   - GCC/G++ compiler
#   - CUDA toolkit (for GPU builds)
#
#   macOS:
#   - Xcode Command Line Tools
#   - CMake 3.18 or higher
#   - Clang/Clang++ compiler
#
# USAGE:
#   ./build.sh [OPTIONS]
#
# OPTIONS:
#   --help              Show help message
#   --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
#   --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
#   --test=TEST         Enable unit tests: 'on' or 'off' (default: on)
#   --jobs=N            Number of parallel build jobs (default: auto-detect)
#   --clean=CLEAN       Clean build artifacts: 'true' or 'false' (default: true)
#   --verbose           Enable verbose output (default: false)
#   --cmake-args=ARGS   Additional CMake arguments
#
# EXAMPLES:
#   ./build_linux.sh                                    # Default build
#   ./build_linux.sh --tier=free --arch=gpu           # GPU build with free tier
#   ./build_linux.sh --jobs=8 --verbose               # 8-core parallel build
#   ./build_linux.sh --cmake-args="-DCUSTOM_FLAG=ON"  # Custom CMake flags
#
# EXIT CODES:
#   0  - Success
#   1  - General error (missing dependencies, validation failure, etc.)
#   130 - Interrupted by user (Ctrl+C)
#   >1 - Test failures (number indicates failed test suites)
#
# FILES CREATED:
#   - ./radarsimlib_{platform}_{arch}_{type}[_free]/   # Output directory with built libraries
#   - ./build_logs/build_YYYYMMDD_HHMMSS.log           # Timestamped build log
#
#===============================================================================

# Exit on any error, undefined variables, and pipe failures
set -euo pipefail

# Configuration
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly BUILD_START_TIME=$(date +%s)

# Platform detection
readonly PLATFORM="$(uname -s)"
case "${PLATFORM}" in
    Linux*)     PLATFORM_NAME="Linux";;
    Darwin*)    PLATFORM_NAME="macOS";;
    *)          PLATFORM_NAME="Unknown";;
esac

readonly LOG_FILE="${SCRIPT_DIR}/build_logs/build_$(date +%Y%m%d_%H%M%S).log"
if [ ! -d "${SCRIPT_DIR}/build_logs" ]; then
    mkdir -p "${SCRIPT_DIR}/build_logs"
fi

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# Default configuration
TIER="standard"
ARCH="cpu"
TEST="on"
JOBS="auto"
CLEAN="true"
VERBOSE="true"
CMAKE_ARGS=""

# Error tracking
BUILD_FAILED=0
CMAKE_FAILED=0
TEST_FAILED=0

# Logging functions
#
# log_info() - Displays an informational message with blue coloring
# Arguments:
#   $1 - The message to display
# Output:
#   Writes the message to both stdout and the log file with [INFO] prefix
log_info() {
    local message="$1"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${BLUE}[INFO]${NC} ${message}" | tee -a "${LOG_FILE}"
}

#
# log_success() - Displays a success message with green coloring
# Arguments:
#   $1 - The success message to display
# Output:
#   Writes the message to both stdout and the log file with [SUCCESS] prefix
log_success() {
    local message="$1"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${GREEN}[SUCCESS]${NC} ${message}" | tee -a "${LOG_FILE}"
}

#
# log_warning() - Displays a warning message with yellow coloring
# Arguments:
#   $1 - The warning message to display
# Output:
#   Writes the message to both stdout and the log file with [WARNING] prefix
log_warning() {
    local message="$1"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${YELLOW}[WARNING]${NC} ${message}" | tee -a "${LOG_FILE}"
}

#
# log_error() - Displays an error message with red coloring
# Arguments:
#   $1 - The error message to display
# Output:
#   Writes the message to both stdout and the log file with [ERROR] prefix
log_error() {
    local message="$1"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${RED}[ERROR]${NC} ${message}" | tee -a "${LOG_FILE}"
}

#
# Help() - Displays comprehensive usage information and command line options
# Description:
#   Shows the script's usage syntax, available command line options with descriptions,
#   and practical examples of how to use the script with different configurations.
#   Includes platform-specific information for both Linux and macOS.
# Arguments:
#   None
# Output:
#   Prints formatted help text to stdout
# Exit:
#   This function is typically called before script exit when --help is specified
Help() {
    cat << EOF

Usage: build.sh [OPTIONS]

Cross-platform build script for RadarSimLib - A Radar Simulator Built with C++
Supports both Linux and macOS platforms with automatic platform detection.

Current Platform: ${PLATFORM_NAME}

OPTIONS:
  --help              Show this help message
  --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
  --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
  --test=TEST         Enable unit tests: 'on' or 'off' (default: on)
  --jobs=N            Number of parallel build jobs (default: auto-detect)
  --clean=CLEAN       Clean build artifacts: 'true' or 'false' (default: true)
  --verbose           Enable verbose output (default: false)
  --cmake-args=ARGS   Additional CMake arguments

EXAMPLES:
  ${0##*/}                                    # Default build
  ${0##*/} --tier=free --arch=gpu           # GPU build with free tier
  ${0##*/} --jobs=8 --verbose               # 8-core parallel build
  ${0##*/} --cmake-args="-DCUSTOM_FLAG=ON"  # Custom CMake flags

PLATFORM-SPECIFIC NOTES:
  Linux:
    - Uses GCC/G++ compiler, creates .so files
    - Requires GCC/G++ development tools
    - GPU builds require CUDA toolkit

  macOS:
    - Uses Clang/Clang++ compiler, creates .dylib files
    - Requires Xcode Command Line Tools
    - Compatible with both Intel (x86_64) and Apple Silicon (ARM64)
    - GPU builds not currently supported on macOS
    - Compatible with bash 3.2+ (default macOS bash)

EXIT CODES:
  0  - Success
  1  - General error (missing dependencies, validation failure, etc.)
  130 - Interrupted by user (Ctrl+C)
  >1 - Test failures (number indicates failed test suites)

FILES CREATED:
  - ./radarsimlib_{platform}_{arch}_{type}[_free]/ # Output directory with built libraries
  - ./build_logs/build_YYYYMMDD_HHMMSS.log         # Timestamped build log

EOF
}

#
# cleanup() - Signal handler for build process cleanup
# Description:
#   Handles cleanup operations when the script exits, either normally or due to
#   errors/interruptions. Logs appropriate error messages and ensures proper
#   exit code propagation. Provides detailed error information for debugging.
# Arguments:
#   None (uses $? to get the exit code)
# Global Variables:
#   LOG_FILE - Path to the log file for error reporting
#   BUILD_FAILED, CMAKE_FAILED, TEST_FAILED - Error state tracking
# Exit:
#   Exits with the same code that triggered the cleanup
cleanup() {
    local exit_code=$?
    
    if [ ${exit_code} -ne 0 ]; then
        echo ""
        log_error "======================================================================"
        log_error "BUILD PROCESS FAILED"
        log_error "======================================================================"
        log_error "Build process interrupted or failed with exit code ${exit_code}"
        log_error "Platform: ${PLATFORM_NAME}"
        log_error "Architecture: ${ARCH}"
        log_error "Tier: ${TIER}"
        
        # Provide specific error context
        if [ ${CMAKE_FAILED} -eq 1 ]; then
            log_error "CMake configuration or compilation failed"
        fi
        if [ ${TEST_FAILED} -gt 0 ]; then
            log_error "Unit tests failed (${TEST_FAILED} failures)"
        fi
        
        log_error "Check ${LOG_FILE} for detailed error information"
        log_error "======================================================================"
        
        # On macOS, provide platform-specific troubleshooting tips
        if [ "${PLATFORM_NAME}" = "macOS" ]; then
            echo ""
            log_info "macOS Troubleshooting Tips:"
            log_info "1. Ensure Xcode Command Line Tools are installed: xcode-select --install"
            log_info "2. Check if you need to accept Xcode license: sudo xcodebuild -license accept"
            log_info "3. For M1/M2 Macs, ensure you're using compatible dependencies"
        fi
    else
        log_success "Build process completed successfully"
    fi
    
    exit ${exit_code}
}

#
# detect_cores() - Automatically detects the number of CPU cores available
# Description:
#   Attempts to determine the number of CPU cores using platform-specific and
#   general methods. Supports both Linux and macOS with appropriate fallbacks.
#   Falls back to a safe default if detection fails.
# Arguments:
#   None
# Output:
#   Prints the number of CPU cores to stdout
# Return:
#   Always returns 0 (success)
# Methods used (in order):
#   macOS: sysctl -n hw.ncpu (native method)
#   Linux: nproc command (most reliable)
#   Linux: /proc/cpuinfo parsing (fallback for older systems)
#   Fallback: Hard-coded value of 4
detect_cores() {
    local cores=4  # Default fallback
    
    if [ "${PLATFORM_NAME}" = "macOS" ] && command -v sysctl >/dev/null 2>&1; then
        cores=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)
    elif command -v nproc >/dev/null 2>&1; then
        cores=$(nproc 2>/dev/null || echo 4)
    elif [ -r /proc/cpuinfo ]; then
        cores=$(grep -c ^processor /proc/cpuinfo 2>/dev/null || echo 4)
    fi
    
    # Ensure we have a positive integer
    if ! echo "${cores}" | grep -q '^[1-9][0-9]*$'; then
        cores=4
    fi
    
    echo "${cores}"
}

#
# command_exists() - Checks if a command is available in the system PATH
# Description:
#   Verifies whether a given command/executable is available and can be executed.
#   Used for dependency checking before attempting to use external tools.
# Arguments:
#   $1 - The command name to check for existence
# Return:
#   0 if command exists and is executable
#   1 if command is not found or not executable
# Example:
#   if command_exists cmake; then echo "CMake is available"; fi
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

#
# to_lowercase() - Converts a string to lowercase (macOS bash 3.2 compatible)
# Description:
#   Converts the input string to lowercase using tr, which is available on all platforms
#   and doesn't require bash 4+ features like ${var,,}
# Arguments:
#   $1 - The string to convert to lowercase
# Output:
#   Prints the lowercase string to stdout
# Return:
#   Always returns 0 (success)
# Example:
#   result=$(to_lowercase "GPU")  # result will be "gpu"
to_lowercase() {
    echo "$1" | tr '[:upper:]' '[:lower:]'
}

#
# get_library_extension() - Returns the appropriate library extension for the platform
# Description:
#   Returns the platform-specific shared library extension (.so for Linux, .dylib for macOS)
# Arguments:
#   None
# Output:
#   Prints the library extension to stdout
# Return:
#   Always returns 0 (success)
get_library_extension() {
    case "${PLATFORM_NAME}" in
        Linux) echo "so" ;;
        macOS) echo "dylib" ;;
        *) echo "so" ;;  # Default to .so
    esac
}

#
# get_platform_suffix() - Returns the platform-specific directory suffix
# Description:
#   Returns the appropriate platform suffix for output directories
#   For Ubuntu systems, detects version and returns ubuntu22/ubuntu24
# Arguments:
#   None
# Output:
#   Prints the platform suffix to stdout (e.g., "ubuntu22", "ubuntu24", "macos")
# Return:
#   Always returns 0 (success)
get_platform_suffix() {
    case "${PLATFORM_NAME}" in
        Linux) 
            # Try to detect Ubuntu version
            if [ -f /etc/os-release ]; then
                local ubuntu_version=$(grep -E '^VERSION_ID=' /etc/os-release | cut -d'"' -f2 | cut -d'.' -f1)
                if [ -n "$ubuntu_version" ] && [ "$ubuntu_version" = "22" ]; then
                    echo "ubuntu22"
                elif [ -n "$ubuntu_version" ] && [ "$ubuntu_version" = "24" ]; then
                    echo "ubuntu24"
                else
                    # Fallback to generic linux for non-Ubuntu or unknown versions
                    echo "linux"
                fi
            else
                echo "linux"
            fi
            ;;
        macOS) echo "macos" ;;
        *) echo "linux" ;;
    esac
}

#
# get_arch_suffix() - Returns the architecture-specific directory suffix
# Description:
#   Returns the appropriate architecture suffix for output directories.
#   For macOS, detects actual architecture (x86_64 vs arm64)
# Arguments:
#   None
# Output:
#   Prints the architecture suffix to stdout (e.g., "x86_64", "arm64")
# Return:
#   Always returns 0 (success)
get_arch_suffix() {
    if [ "${PLATFORM_NAME}" = "macOS" ]; then
        # Detect actual architecture on macOS
        local machine_arch="$(uname -m)"
        case "${machine_arch}" in
            x86_64) echo "x86_64" ;;
            arm64) echo "arm64" ;;
            *) echo "x86_64" ;;  # Default fallback
        esac
    else
        # For Linux, assume x86_64
        echo "x86_64"
    fi
}

#
# check_requirements() - Validates all system dependencies and requirements
# Description:
#   Performs comprehensive system dependency checking including:
#   - Required build tools (cmake, platform-specific compilers)
#   - GPU-specific requirements (nvcc for CUDA builds)
#   - Platform-specific requirements (Xcode Command Line Tools on macOS)
#   Exits the script with error code 1 if any dependencies are missing.
# Arguments:
#   None
# Global Variables:
#   ARCH - Build architecture (used to determine if GPU tools are needed)
#   PLATFORM_NAME - Current platform (Linux/macOS)
# Dependencies Checked:
#   - cmake: Build system generator
#   - ctest: Test runner (part of CMake)
#   - Platform-specific compilers (gcc/g++ on Linux, clang++ on macOS)
#   - nvcc: NVIDIA CUDA compiler (GPU builds only)
#   - Xcode Command Line Tools (macOS only)
# Exit:
#   Exits with code 1 if any required dependencies are missing
check_requirements() {
    log_info "Validating system dependencies for ${PLATFORM_NAME}..."
    local missing_deps=0
    
    # Check for CMake
    if ! command_exists cmake; then
        log_error "CMake is not installed or not in PATH"
        log_error "Please install CMake 3.18 or higher"
        missing_deps=1
    else
        local cmake_version=$(cmake --version | head -n1 | grep -o '[0-9]\+\.[0-9]\+')
        log_info "CMake found: version ${cmake_version}"
    fi
    
    # Check for CTest (usually comes with CMake)
    if ! command_exists ctest; then
        log_warning "CTest is not available, unit tests will be skipped"
    fi
    
    # Check for CUDA if GPU build is requested
    if [ "$(to_lowercase "$ARCH")" = "gpu" ]; then
        if [ "${PLATFORM_NAME}" = "macOS" ]; then
            log_warning "GPU builds are not officially supported on macOS"
            log_warning "Continuing with CPU build instead"
            ARCH="cpu"
        elif ! command_exists nvcc; then
            log_error "CUDA toolkit is not installed or not in PATH"
            log_error "Please install CUDA SDK for GPU builds"
            missing_deps=1
        else
            local cuda_version=$(nvcc --version | grep "release" | grep -o '[0-9]\+\.[0-9]\+')
            log_info "CUDA toolkit found: version ${cuda_version}"
        fi
    fi
    
    # Platform-specific checks
    case "${PLATFORM_NAME}" in
        macOS)
            # Check for Xcode Command Line Tools
            if ! xcode-select -p >/dev/null 2>&1; then
                log_warning "Xcode Command Line Tools may not be properly installed"
                log_warning "If build fails, run: xcode-select --install"
            fi
            ;;
        Linux)
            # Check for essential build tools
            for tool in make ar; do
                if ! command_exists "${tool}"; then
                    log_error "Essential build tool '${tool}' is missing"
                    missing_deps=1
                fi
            done
            ;;
    esac
    
    if [ ${missing_deps} -eq 1 ]; then
        log_error "Missing required dependencies. Please install them and try again."
        exit 1
    fi
    
    log_success "All system requirements satisfied for ${PLATFORM_NAME}"
}

#
# parse_arguments() - Parses command line arguments and sets global configuration
# Description:
#   Processes all command line arguments passed to the script and sets global
#   configuration variables. Handles both parameter validation and default value
#   assignment. Also handles special cases like --help and automatic CPU detection.
# Arguments:
#   $@ - All command line arguments passed to the script
# Global Variables Set:
#   TIER - Build tier (standard/free)
#   ARCH - Build architecture (cpu/gpu)  
#   TEST - Unit test flag (on/off)
#   JOBS - Number of parallel build jobs
#   CLEAN - Clean build artifacts flag (true/false)
#   VERBOSE - Verbose output flag (true/false)
#   CMAKE_ARGS - Additional CMake arguments
# Supported Options:
#   --help: Shows help and exits
#   --tier=VALUE: Sets build tier
#   --arch=VALUE: Sets architecture
#   --test=VALUE: Enables/disables tests
#   --jobs=VALUE: Sets parallel job count
#   --clean=VALUE: Enables/disables cleanup
#   --verbose: Enables verbose output
#   --cmake-args=VALUE: Passes additional CMake arguments
# Exit:
#   Exits with code 0 on --help
#   Exits with code 1 on unknown options
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help)
                Help
                exit 0
                ;;
            --tier=*)
                TIER="${1#*=}"
                shift
                ;;
            --arch=*)
                ARCH="${1#*=}"
                shift
                ;;
            --test=*)
                TEST="${1#*=}"
                shift
                ;;
            --jobs=*)
                JOBS="${1#*=}"
                shift
                ;;
            --clean=*)
                CLEAN="${1#*=}"
                shift
                ;;
            --verbose)
                VERBOSE="true"
                shift
                ;;
            --cmake-args=*)
                CMAKE_ARGS="${1#*=}"
                shift
                ;;
            --*)
                log_error "Unknown option: $1"
                log_error "Use --help for usage information"
                exit 1
                ;;
            *)
                log_error "Unknown argument: $1"
                log_error "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # Handle auto-detection of CPU cores
    if [ "${JOBS}" = "auto" ]; then
        JOBS=$(detect_cores)
        log_info "Auto-detected ${JOBS} CPU cores for parallel build"
    fi
}

#
# validate_parameters() - Validates all parsed command line parameters
# Description:
#   Performs comprehensive validation of all configuration parameters set by
#   parse_arguments(). Checks for valid values, proper formatting, and logical
#   constraints. Accumulates all validation errors before reporting them.
# Arguments:
#   None
# Global Variables Used:
#   TIER - Validated against 'standard' and 'free'
#   ARCH - Validated against 'cpu' and 'gpu'
#   TEST - Validated against 'on' and 'off'
#   JOBS - Validated as positive integer
#   CLEAN - Validated against 'true' and 'false'
# Validation Rules:
#   - TIER: Must be 'standard' or 'free' (case insensitive)
#   - ARCH: Must be 'cpu' or 'gpu' (case insensitive)
#   - TEST: Must be 'on' or 'off' (case insensitive)
#   - JOBS: Must be positive integer >= 1
#   - CLEAN: Must be 'true' or 'false' (case insensitive)
# Exit:
#   Exits with code 1 if any validation errors are found
validate_parameters() {
    local validation_errors=0
    
    # Validate TIER
    if [ "$(to_lowercase "$TIER")" != "standard" ] && [ "$(to_lowercase "$TIER")" != "free" ]; then
        log_error "Invalid --tier parameter '${TIER}'. Please choose 'standard' or 'free'"
        validation_errors=1
    fi
    
    # Validate ARCH
    if [ "$(to_lowercase "$ARCH")" != "cpu" ] && [ "$(to_lowercase "$ARCH")" != "gpu" ]; then
        log_error "Invalid --arch parameter '${ARCH}'. Please choose 'cpu' or 'gpu'"
        validation_errors=1
    fi
    
    # Validate TEST
    if [ "$(to_lowercase "$TEST")" != "on" ] && [ "$(to_lowercase "$TEST")" != "off" ]; then
        log_error "Invalid --test parameter '${TEST}'. Please choose 'on' or 'off'"
        validation_errors=1
    fi
    
    # Validate JOBS
    if ! echo "${JOBS}" | grep -q '^[1-9][0-9]*$'; then
        log_error "Invalid --jobs parameter '${JOBS}'. Please provide a positive integer"
        validation_errors=1
    fi
    
    # Validate CLEAN
    if [ "$(to_lowercase "$CLEAN")" != "true" ] && [ "$(to_lowercase "$CLEAN")" != "false" ]; then
        log_error "Invalid --clean parameter '${CLEAN}'. Please choose 'true' or 'false'"
        validation_errors=1
    fi
    
    if [ ${validation_errors} -eq 1 ]; then
        log_error "Parameter validation failed. Use --help for usage information."
        exit 1
    fi
    
    log_info "All parameters validated successfully"
}

#
# display_banner() - Shows project banner and current build configuration
# Description:
#   Displays the RadarSimLib project banner with ASCII art logo and shows
#   the current build configuration settings. Provides visual confirmation
#   of all build parameters before the build process begins.
# Arguments:
#   None
# Global Variables Used:
#   PLATFORM_NAME - Current platform
#   TIER - Build tier setting
#   ARCH - Architecture setting  
#   TEST - Test execution setting
#   JOBS - Number of parallel jobs
#   CLEAN - Clean build setting
#   VERBOSE - Verbose output setting
#   LOG_FILE - Log file path
#   CMAKE_ARGS - Additional CMake arguments (if any)
# Output:
#   Prints formatted banner and configuration to stdout
display_banner() {
    echo ""
    echo "======================================================================"
    echo "RadarSimLib - A Radar Simulator Built with C++"
    echo "Copyright (C) 2023 - PRESENT  radarsimx.com"
    echo "E-mail: info@radarsimx.com"
    echo "Website: https://radarsimx.com"
    echo "======================================================================"
    echo ""
    echo "Build Configuration (${PLATFORM_NAME}):"
    echo "  - Platform: ${PLATFORM_NAME}"
    echo "  - Tier: ${TIER}"
    echo "  - Architecture: ${ARCH}"
    echo "  - Tests: ${TEST}"
    echo "  - Parallel Jobs: ${JOBS}"
    echo "  - Clean Build: ${CLEAN}"
    echo "  - Verbose Output: ${VERBOSE}"
    echo "  - Script Directory: ${SCRIPT_DIR}"
    echo "  - Log File: ${LOG_FILE}"
    if [ -n "${CMAKE_ARGS}" ]; then
        echo "  - Additional CMake Args: ${CMAKE_ARGS}"
    fi
    echo ""
    echo "██████╗  █████╗ ██████╗  █████╗ ██████╗ ███████╗██╗███╗   ███╗██╗  ██╗"
    echo "██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██║████╗ ████║╚██╗██╔╝"
    echo "██████╔╝███████║██║  ██║███████║██████╔╝███████╗██║██╔████╔██║ ╚███╔╝ "
    echo "██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗╚════██║██║██║╚██╔╝██║ ██╔██╗ "
    echo "██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║███████║██║██║ ╚═╝ ██║██╔╝ ██╗"
    echo "╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚═╝     ╚═╝╚═╝  ╚═╝"
    echo ""
}

#
# clean_build_artifacts() - Removes previous build artifacts and generated files
# Description:
#   Performs cleanup of build directories and generated files from previous
#   build attempts. Only executes if CLEAN is set to 'true'. Removes both
#   build directories and generated source files to ensure clean builds.
# Arguments:
#   None
# Global Variables Used:
#   CLEAN - Controls whether cleanup is performed
# Directories/Files Removed:
#   - ./build/ - C++ build directory
#   - ./radarsimlib_{platform}_{arch}_* - Output directories
# Behavior:
#   - If CLEAN='true': Performs full cleanup
#   - If CLEAN='false': Skips cleanup and logs message
clean_build_artifacts() {
    if [ "$(to_lowercase "$CLEAN")" = "true" ]; then
        log_info "Cleaning previous build artifacts..."
        
        # Remove build directory
        if [ -d "./build" ]; then
            rm -rf "./build"
            log_info "Removed build directory"
        fi
        
        log_success "Build artifacts cleaned successfully"
    else
        log_info "Skipping cleanup (CLEAN=false)"
    fi
}

#
# build_cpp_library() - Builds the C++ library (platform-specific)
# Description:
#   Compiles the RadarSimLib C++ library using CMake build system. Configures
#   build options based on architecture (CPU/GPU) and test settings. Supports
#   parallel compilation and both verbose and quiet build modes. Creates
#   platform-specific libraries (.so on Linux, .dylib on macOS).
# Arguments:
#   None
# Global Variables Used:
#   PLATFORM_NAME - Current platform for library naming
#   ARCH - Determines GPU build flags
#   TIER - Determines FREETIER build flags
#   TEST - Controls Google Test compilation
#   CMAKE_ARGS - Additional CMake arguments
#   JOBS - Number of parallel compilation jobs
#   VERBOSE - Controls build output verbosity
#   LOG_FILE - Log file for quiet builds
# Build Process:
#   1. Creates build directory at ./build
#   2. Configures CMake with appropriate flags
#   3. Builds with specified parallel job count
#   4. Times the build process
#   5. Returns to original working directory
# CMake Flags Set:
#   - CMAKE_BUILD_TYPE=Release (always)
#   - GPU_BUILD=ON/OFF (based on ARCH setting)
#   - FREETIER=ON/OFF (based on TIER setting)
#   - GTEST=ON/OFF (based on TEST setting)
#   - Custom flags from CMAKE_ARGS
build_cpp_library() {
    log_info "Building RadarSimLib C++ library..."
    local build_start_time=$(date +%s)
    local workpath=$(pwd)
    
    # Create and enter build directory
    mkdir -p "./build"
    cd "./build"
    
    # Configure CMake options
    local cmake_options="-DCMAKE_BUILD_TYPE=Release"
    
    if [ "$(to_lowercase "$ARCH")" = "gpu" ]; then
        cmake_options="${cmake_options} -DGPU_BUILD=ON"
        log_info "Configuring for GPU build"
        
        # Warn about macOS GPU support
        if [ "${PLATFORM_NAME}" = "macOS" ]; then
            log_warning "GPU builds on macOS may have limited CUDA support"
        fi
    else
        cmake_options="${cmake_options} -DGPU_BUILD=OFF"
        log_info "Configuring for CPU build"
    fi
    
    if [ "$(to_lowercase "$TIER")" = "free" ]; then
        cmake_options="${cmake_options} -DFREETIER=ON"
        log_info "Configuring for free tier"
    else
        cmake_options="${cmake_options} -DFREETIER=OFF"
        log_info "Configuring for standard tier"
    fi
    
    if [ "$(to_lowercase "$TEST")" = "on" ]; then
        cmake_options="${cmake_options} -DGTEST=ON"
        log_info "Enabling unit tests"
    else
        cmake_options="${cmake_options} -DGTEST=OFF"
        log_info "Disabling unit tests"
    fi
    
    # Add custom CMake arguments
    if [ -n "${CMAKE_ARGS}" ]; then
        cmake_options="${cmake_options} ${CMAKE_ARGS}"
        log_info "Adding custom CMake arguments: ${CMAKE_ARGS}"
    fi
    
    # Configure build
    log_info "Configuring CMake build..."
    if [ "$(to_lowercase "$VERBOSE")" = "true" ]; then
        eval "cmake ${cmake_options} .."
    else
        eval "cmake ${cmake_options} .." >> "${LOG_FILE}" 2>&1
    fi
    
    if [ $? -ne 0 ]; then
        log_error "CMake configuration failed"
        CMAKE_FAILED=1
        cd "${workpath}"
        exit 1
    fi
    
    # Build the library
    log_info "Building C++ library with ${JOBS} parallel jobs..."
    if [ "$(to_lowercase "$VERBOSE")" = "true" ]; then
        cmake --build . --parallel "${JOBS}"
    else
        cmake --build . --parallel "${JOBS}" >> "${LOG_FILE}" 2>&1
    fi
    
    if [ $? -ne 0 ]; then
        log_error "C++ library build failed"
        CMAKE_FAILED=1
        cd "${workpath}"
        exit 1
    fi
    
    local build_end_time=$(date +%s)
    local build_duration=$((build_end_time - build_start_time))
    log_success "C++ library build completed in ${build_duration} seconds"
    
    cd "${workpath}"
}

#
# install_libraries() - Copies built libraries to final installation directory
# Description:
#   Installs all built components (shared libraries and header files) into the
#   appropriate output directory structure. Creates the necessary directory
#   hierarchy and handles missing files gracefully. Handles both Linux (.so)
#   and macOS (.dylib) library files.
# Arguments:
#   None
# Global Variables Used:
#   PLATFORM_NAME - Current platform for library file detection
#   ARCH - Architecture setting for output directory naming
#   TIER - Tier setting for output directory naming
# Installation Process:
#   1. Determines output directory based on architecture and tier
#   2. Creates output directory structure
#   3. Copies shared libraries (platform-specific extensions) from build directory
#   4. Copies header files to output directory
# File Sources:
#   - Library files: ./build/libradarsimc.{so,dylib}
#   - Header files: ./src/includes/radarsim.h
install_libraries() {
    log_info "Installing built libraries..."
    
    # Determine output directory based on configuration
    local lib_ext=$(get_library_extension)
    local platform_suffix=$(get_platform_suffix)
    local arch_suffix=$(get_arch_suffix)
    local release_path
    
    # Define file paths
    local lib_file="./build/libradarsimc.${lib_ext}"
    local header_file="./src/includes/radarsim.h"
    
    if [ "$(to_lowercase "$ARCH")" = "gpu" ]; then
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_gpu"
        else
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_gpu_free"
        fi
    else
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_cpu"
        else
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_cpu_free"
        fi
    fi
    
    # Remove and create output directory
    rm -rf "${release_path}"
    mkdir -p "${release_path}"
    
    # Copy files to examples/cpp/radarsimlib directory
    local examples_path="./examples/cpp/radarsimlib"
    
    # Clean old examples directory if it exists
    if [ -d "${examples_path}" ]; then
        log_info "Cleaning old examples directory ${examples_path}..."
        rm -rf "${examples_path}"
    fi
    
    # Create examples directory
    mkdir -p "${examples_path}"
    
    # Copy library file to examples
    if [ -f "${lib_file}" ]; then
        cp "${lib_file}" "${examples_path}/"
        log_success "Copied library to examples: libradarsimc.${lib_ext}"
    else
        log_warning "Library file not found for examples copy: ${lib_file}"
    fi
    
    # Copy header file to examples
    if [ -f "${header_file}" ]; then
        cp "${header_file}" "${examples_path}/"
        log_success "Copied header to examples: radarsim.h"
    else
        log_warning "Header file not found for examples copy: ${header_file}"
    fi
    
    # Copy all examples/cpp files to release path
    if [ -d "./examples/cpp" ]; then
        log_info "Copying examples/cpp files to release folder..."
        cp -r ./examples/cpp/* "${release_path}/" 2>/dev/null || {
            log_warning "Some files from examples/cpp could not be copied"
        }
        log_success "Examples/cpp files copied successfully to ${release_path}"
    else
        log_warning "examples/cpp directory not found"
    fi

    log_success "Libraries installed to: ${release_path}"
    log_success "Libraries also copied to examples: ${examples_path}"
}

#
# validate_build_output() - Validates the final build artifacts and compatibility
# Description:
#   Performs comprehensive validation of the built libraries including:
#   - File existence and permissions
#   - Library architecture verification (macOS specific)
#   - Symbol table verification
#   - Dependency checking
# Arguments:
#   None
# Global Variables Used:
#   ARCH, TIER, PLATFORM_NAME - Used to determine output paths
# Exit:
#   Exits with code 1 if validation fails
validate_build_output() {
    log_info "Validating build output..."
    
    # Determine output directory
    local lib_ext=$(get_library_extension)
    local platform_suffix=$(get_platform_suffix)
    local arch_suffix=$(get_arch_suffix)
    local release_path
    
    if [ "$(to_lowercase "$ARCH")" = "gpu" ]; then
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_gpu"
        else
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_gpu_free"
        fi
    else
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_cpu"
        else
            release_path="./radarsimlib_${platform_suffix}_${arch_suffix}_cpu_free"
        fi
    fi
    
    # Verify output directory exists
    if [ ! -d "${release_path}" ]; then
        log_error "Output directory not found: ${release_path}"
        return 1
    fi
    
    # Verify library file
    local lib_file="${release_path}/radarsimlib/libradarsimc.${lib_ext}"
    if [ ! -f "${lib_file}" ]; then
        log_error "Library file not found: ${lib_file}"
        return 1
    fi
    
    # Verify header file
    local header_file="${release_path}/radarsimlib/radarsim.h"
    if [ ! -f "${header_file}" ]; then
        log_error "Header file not found: ${header_file}"
        return 1
    fi
    
    # Platform-specific validation
    if [ "${PLATFORM_NAME}" = "macOS" ]; then
        # Check library architecture on macOS
        if command_exists file; then
            local file_info=$(file "${lib_file}")
            log_info "Library file info: ${file_info}"
        fi
        
        # Check architecture compatibility with lipo (if available)
        if command_exists lipo; then
            local arch_info=$(lipo -info "${lib_file}" 2>/dev/null || echo "Could not determine architecture")
            log_info "Library architecture: ${arch_info}"
            
            # Verify it matches the expected architecture
            local expected_arch=$(uname -m)
            if echo "${arch_info}" | grep -q "${expected_arch}"; then
                log_success "Library architecture matches system: ${expected_arch}"
            else
                log_warning "Library architecture may not match system architecture"
            fi
        fi
        
        # Check for basic symbols in the library
        if command_exists nm; then
            local symbol_count=$(nm -D "${lib_file}" 2>/dev/null | wc -l || echo "0")
            if [ "${symbol_count}" -gt 0 ]; then
                log_success "Library contains ${symbol_count} exported symbols"
            else
                log_warning "Could not verify library symbols"
            fi
        fi
    elif [ "${PLATFORM_NAME}" = "Linux" ]; then
        # Linux-specific validation
        if command_exists file; then
            local file_info=$(file "${lib_file}")
            log_info "Library file info: ${file_info}"
        fi
        
        # Check dependencies
        if command_exists ldd; then
            log_info "Library dependencies:"
            ldd "${lib_file}" | head -10  # Show first 10 dependencies
        fi
        
        # Check for basic symbols
        if command_exists nm; then
            local symbol_count=$(nm -D "${lib_file}" 2>/dev/null | wc -l || echo "0")
            if [ "${symbol_count}" -gt 0 ]; then
                log_success "Library contains ${symbol_count} exported symbols"
            else
                log_warning "Could not verify library symbols"
            fi
        fi
    fi
    
    # Verify file sizes are reasonable
    local lib_size=$(stat -c%s "${lib_file}" 2>/dev/null || stat -f%z "${lib_file}" 2>/dev/null || echo "0")
    if [ "${lib_size}" -gt 1000 ]; then  # At least 1KB
        log_success "Library file size: ${lib_size} bytes"
    else
        log_error "Library file appears to be too small: ${lib_size} bytes"
        return 1
    fi
    
    local header_size=$(stat -c%s "${header_file}" 2>/dev/null || stat -f%z "${header_file}" 2>/dev/null || echo "0")
    if [ "${header_size}" -gt 100 ]; then  # At least 100 bytes
        log_success "Header file size: ${header_size} bytes"
    else
        log_error "Header file appears to be too small: ${header_size} bytes"
        return 1
    fi
    
    log_success "Build output validation completed successfully"
    return 0
}

#
# run_tests() - Executes unit tests if enabled
# Description:
#   Runs comprehensive test suites including Google Test for C++ components.
#   Tests are only executed if enabled via the --test=on parameter.
# Arguments:
#   None
# Global Variables Used:
#   TEST - Determines if tests should be executed
# Tests Executed:
#   - Google Test via CTest
# Error Handling:
#   Sets TEST_FAILED=1 on any test failures
#   Continues with warnings if test tools are not available
run_tests() {
    if [ "$(to_lowercase "$TEST")" = "off" ]; then
        log_info "Tests are disabled, skipping test execution"
        return 0
    fi
    
    log_info "Running tests..."
    
    # Run Google Test using CTest
    if [ -f "./build/radarsimc_test" ] && command_exists ctest; then
        log_info "Running Google Test for C++ using CTest..."
        if [ "$(to_lowercase "$VERBOSE")" = "true" ]; then
            ctest --test-dir "./build" --verbose
        else
            ctest --test-dir "./build" --verbose >> "${LOG_FILE}" 2>&1
        fi
        
        local test_exit_code=$?
        if [ ${test_exit_code} -ne 0 ]; then
            log_error "Google Test failed with exit code ${test_exit_code}"
            TEST_FAILED=1
        else
            log_success "Google Test passed"
        fi
    else
        log_warning "Google Test executable not found or CTest not available, skipping C++ tests"
    fi
    
    # Check overall test results
    if [ ${TEST_FAILED} -ne 0 ]; then
        log_error "Some tests failed"
        exit $((TEST_FAILED + 1))
    fi
    
    log_success "All tests passed successfully"
}

#
# build_success() - Displays build completion summary
# Description:
#   Shows comprehensive build completion information including timing,
#   configuration summary, and output locations.
# Arguments:
#   None
# Global Variables Used:
#   BUILD_START_TIME - Start time for duration calculation
#   All configuration variables for summary display
build_success() {
    local build_end_time=$(date +%s)
    local total_duration=$((build_end_time - BUILD_START_TIME))
    
    echo ""
    echo "======================================================================"
    echo "BUILD COMPLETED SUCCESSFULLY"
    echo "======================================================================"
    echo ""
    echo "Build Summary (${PLATFORM_NAME}):"
    echo "  - Platform: ${PLATFORM_NAME}"
    echo "  - Tier: ${TIER}"
    echo "  - Architecture: ${ARCH}"
    echo "  - Tests: ${TEST}"
    echo "  - Parallel Jobs: ${JOBS}"
    echo "  - Clean Build: ${CLEAN}"
    echo "  - Verbose Output: ${VERBOSE}"
    echo "  - Total Build Time: ${total_duration} seconds"
    echo ""
    echo "Output Locations:"
    local lib_ext=$(get_library_extension)
    local platform_suffix=$(get_platform_suffix)
    local arch_suffix=$(get_arch_suffix)
    
    if [ "$(to_lowercase "$ARCH")" = "gpu" ]; then
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            echo "  - Output Directory: ./radarsimlib_${platform_suffix}_${arch_suffix}_gpu/"
        else
            echo "  - Output Directory: ./radarsimlib_${platform_suffix}_${arch_suffix}_gpu_free/"
        fi
    else
        if [ "$(to_lowercase "$TIER")" = "standard" ]; then
            echo "  - Output Directory: ./radarsimlib_${platform_suffix}_${arch_suffix}_cpu/"
        else
            echo "  - Output Directory: ./radarsimlib_${platform_suffix}_${arch_suffix}_cpu_free/"
        fi
    fi
    echo "  - Library File: libradarsimc.${lib_ext}"
    echo "  - Header File: radarsim.h"
    echo ""
    if [ "$(to_lowercase "$TEST")" = "on" ]; then
        echo "Test Results: All tests passed successfully"
    else
        echo "Test Results: Tests were skipped (disabled)"
    fi
    echo ""
    echo "Build completed at: $(date)"
    echo "Log file: ${LOG_FILE}"
    echo ""
    echo "======================================================================"
}

#
# build_failed() - Displays build failure summary
# Description:
#   Shows comprehensive build failure information including error summary
#   and troubleshooting suggestions.
# Arguments:
#   None
# Global Variables Used:
#   All error tracking variables and configuration for summary display
build_failed() {
    echo ""
    echo "======================================================================"
    echo "BUILD FAILED"
    echo "======================================================================"
    echo ""
    echo "Build Configuration (${PLATFORM_NAME}):"
    echo "  - Platform: ${PLATFORM_NAME}"
    echo "  - Tier: ${TIER}"
    echo "  - Architecture: ${ARCH}"
    echo "  - Tests: ${TEST}"
    echo "  - Parallel Jobs: ${JOBS}"
    echo ""
    echo "Error Summary:"
    if [ ${CMAKE_FAILED} -ne 0 ]; then
        echo "  - CMake configuration or build failed"
    fi
    if [ ${TEST_FAILED} -ne 0 ]; then
        echo "  - Unit tests failed"
    fi
    echo ""
    echo "Troubleshooting:"
    echo "  - Check the log file for detailed error messages: ${LOG_FILE}"
    echo "  - Ensure all required dependencies are installed"
    case "${PLATFORM_NAME}" in
        Linux)
            echo "  - Install GCC development tools: sudo apt-get install build-essential (Ubuntu/Debian)"
            echo "  - For GPU builds, verify CUDA toolkit installation"
            ;;
        macOS)
            echo "  - Install Xcode Command Line Tools: xcode-select --install"
            echo "  - GPU builds are not officially supported on macOS"
            ;;
    esac
    echo "  - Try running with --verbose for more detailed output"
    echo "  - Check CMAKE_ARGS if using custom CMake flags"
    echo ""
    echo "Build failed at: $(date)"
    echo ""
    echo "======================================================================"
}

# Set up signal handlers
trap cleanup EXIT
trap 'exit 130' INT  # Handle Ctrl+C

# Main execution flow
main() {
    # Parse command line arguments
    parse_arguments "$@"
    
    # Validate parameters
    validate_parameters
    
    # Display banner and configuration
    display_banner
    
    # Check system requirements
    check_requirements
    
    # Start build process
    log_info "Starting build process..."
    log_info "Build started at: $(date)"
    
    # Clean previous build artifacts
    clean_build_artifacts
    
    # Build C++ library
    build_cpp_library
    
    # Install libraries
    install_libraries
    
    # Validate build output
    validate_build_output
    
    # Run tests if enabled
    run_tests
    
    # Display success message
    build_success
    
    exit 0
}

# Call main function with all arguments
main "$@"
exit $?
