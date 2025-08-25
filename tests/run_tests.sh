#!/bin/bash
#
# Test runner script for RadarSim C Wrapper tests
#
# This script provides convenient commands for running the C wrapper tests
# with various options and configurations.
#
# Usage:
#   ./run_tests.sh [options]
#
# Options:
#   --build       Build tests before running
#   --clean       Clean build directory before building
#   --coverage    Enable coverage reporting
#   --verbose     Run tests with verbose output
#   --filter=X    Run only tests matching filter X
#   --help        Show this help message
#
# Examples:
#   ./run_tests.sh --build --verbose
#   ./run_tests.sh --filter="*TransmitterTest*"
#   ./run_tests.sh --coverage
#
# Copyright (C) 2023 - PRESENT  radarsimx.com

set -e  # Exit on any error

# Default configuration
BUILD_TESTS=false
CLEAN_BUILD=false
ENABLE_COVERAGE=false
VERBOSE_OUTPUT=false
TEST_FILTER=""
BUILD_DIR="build"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Show help message
show_help() {
    echo "RadarSim C Wrapper Test Runner"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --build       Build tests before running"
    echo "  --clean       Clean build directory before building"
    echo "  --coverage    Enable coverage reporting"
    echo "  --verbose     Run tests with verbose output"
    echo "  --filter=X    Run only tests matching filter X"
    echo "  --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --build --verbose"
    echo "  $0 --filter=\"*TransmitterTest*\""
    echo "  $0 --coverage"
    echo ""
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --build)
                BUILD_TESTS=true
                shift
                ;;
            --clean)
                CLEAN_BUILD=true
                shift
                ;;
            --coverage)
                ENABLE_COVERAGE=true
                shift
                ;;
            --verbose)
                VERBOSE_OUTPUT=true
                shift
                ;;
            --filter=*)
                TEST_FILTER="${1#*=}"
                shift
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# Check dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check for cmake
    if ! command -v cmake &> /dev/null; then
        print_error "CMake is not installed or not in PATH"
        exit 1
    fi
    
    # Check for make or ninja
    if ! command -v make &> /dev/null && ! command -v ninja &> /dev/null; then
        print_error "Make or Ninja build system is required"
        exit 1
    fi
    
    # Check for Google Test (will be checked by CMake)
    print_info "Dependencies check passed"
}

# Clean build directory
clean_build() {
    if [ "$CLEAN_BUILD" = true ]; then
        print_info "Cleaning build directory..."
        if [ -d "$BUILD_DIR" ]; then
            rm -rf "$BUILD_DIR"
            print_success "Build directory cleaned"
        fi
    fi
}

# Configure CMake
configure_cmake() {
    print_info "Configuring CMake..."
    
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    CMAKE_ARGS=(
        "-DBUILD_CWRAPPER_TESTS=ON"
    )
    
    if [ "$ENABLE_COVERAGE" = true ]; then
        CMAKE_ARGS+=("-DENABLE_COVERAGE=ON")
        print_info "Coverage reporting enabled"
    fi
    
    cmake "${CMAKE_ARGS[@]}" "$ROOT_DIR"
    
    if [ $? -eq 0 ]; then
        print_success "CMake configuration completed"
    else
        print_error "CMake configuration failed"
        exit 1
    fi
}

# Build tests
build_tests() {
    if [ "$BUILD_TESTS" = true ]; then
        print_info "Building C wrapper tests..."
        
        if [ ! -d "$BUILD_DIR" ]; then
            configure_cmake
        else
            cd "$BUILD_DIR"
        fi
        
        # Build the test targets
        make cwrapper_tests_all -j$(nproc 2>/dev/null || echo 4)
        
        if [ $? -eq 0 ]; then
            print_success "Build completed successfully"
        else
            print_error "Build failed"
            exit 1
        fi
    fi
}

# Run tests
run_tests() {
    print_info "Running C wrapper tests..."
    
    # Ensure we're in build directory
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "Build directory not found. Use --build option first."
        exit 1
    fi
    
    cd "$BUILD_DIR"
    
    # Check if test executables exist
    if [ ! -f "cwrapper_tests_all" ]; then
        print_error "Test executables not found. Use --build option first."
        exit 1
    fi
    
    # Prepare test command
    TEST_CMD="./cwrapper_tests_all"
    
    # Add filter if specified
    if [ -n "$TEST_FILTER" ]; then
        TEST_CMD="$TEST_CMD --gtest_filter=$TEST_FILTER"
        print_info "Running tests with filter: $TEST_FILTER"
    fi
    
    # Add verbose output if requested
    if [ "$VERBOSE_OUTPUT" = true ]; then
        TEST_CMD="$TEST_CMD --gtest_verbose"
        print_info "Running tests with verbose output"
    fi
    
    # Run the tests
    print_info "Executing: $TEST_CMD"
    $TEST_CMD
    
    TEST_RESULT=$?
    
    if [ $TEST_RESULT -eq 0 ]; then
        print_success "All tests passed!"
    else
        print_error "Some tests failed (exit code: $TEST_RESULT)"
        exit $TEST_RESULT
    fi
}

# Generate coverage report
generate_coverage() {
    if [ "$ENABLE_COVERAGE" = true ]; then
        print_info "Generating coverage report..."
        
        cd "$BUILD_DIR"
        
        if command -v make &> /dev/null; then
            make coverage_cwrapper
            if [ $? -eq 0 ]; then
                print_success "Coverage report generated in coverage_html/"
                if command -v xdg-open &> /dev/null; then
                    print_info "Opening coverage report in browser..."
                    xdg-open coverage_html/index.html
                elif command -v open &> /dev/null; then
                    print_info "Opening coverage report in browser..."
                    open coverage_html/index.html
                fi
            else
                print_warning "Coverage report generation failed"
            fi
        else
            print_warning "Coverage target not available"
        fi
    fi
}

# Main execution
main() {
    print_info "RadarSim C Wrapper Test Runner"
    print_info "==============================="
    
    # Change to script directory
    cd "$SCRIPT_DIR"
    
    # Parse arguments
    parse_arguments "$@"
    
    # Check dependencies
    check_dependencies
    
    # Clean if requested
    clean_build
    
    # Build if requested
    if [ "$BUILD_TESTS" = true ]; then
        configure_cmake
        build_tests
    fi
    
    # Run tests
    run_tests
    
    # Generate coverage if requested
    generate_coverage
    
    print_success "Test execution completed successfully!"
}

# Execute main function with all arguments
main "$@"
