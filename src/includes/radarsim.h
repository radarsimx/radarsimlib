/*
 * @file radarsim.h
 * @brief C API header for RadarSimCpp library
 *
 * @details
 * This header provides a comprehensive C-compatible interface for the
 * RadarSimCpp library, enabling high-fidelity radar simulation functionality
 * for C applications. The API implements the opaque pointer pattern to hide C++
 * implementation details while providing a clean, stable C interface.
 *
 * Key features include:
 * - Transmitter and receiver configuration with full antenna pattern support
 * - Complete radar system modeling with platform motion
 * - Point target simulation for basic scenarios
 * - Mesh target simulation using Physical Optics ray tracing
 * - Radar Cross Section (RCS) calculation
 * - LiDAR point cloud generation
 * - Radar-to-radar interference analysis
 * - Automatic memory management with RAII principles
 * - Exception-safe resource handling
 * - GPU acceleration support (when available)
 *
 * @note All functions use defensive programming with comprehensive input
 * validation.
 * @note Memory management follows RAII principles with automatic cleanup
 * registration.
 * @warning Free tier has limitations on number of channels, targets, and mesh
 * complexity.
 *
 *    ----------
 *
 *    Copyright (C) 2023 - PRESENT  radarsimx.com
 *    E-mail: info@radarsimx.com
 *    Website: https://radarsimx.com
 *
 *    ██████╗  █████╗ ██████╗  █████╗ ██████╗ ███████╗██╗███╗   ███╗██╗  ██╗
 *    ██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██║████╗ ████║╚██╗██╔╝
 *    ██████╔╝███████║██║  ██║███████║██████╔╝███████╗██║██╔████╔██║ ╚███╔╝
 *    ██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗╚════██║██║██║╚██╔╝██║ ██╔██╗
 *    ██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║███████║██║██║ ╚═╝ ██║██╔╝ ██╗
 *    ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚═╝     ╚═╝╚═╝  ╚═╝
 *
 */

#ifndef RADARSIM_H
#define RADARSIM_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined _WIN64 || defined _WIN32
#define EXPORTED __declspec(dllexport)
#else
#define EXPORTED
#endif

#define VERSION_MAJOR 14
#define VERSION_MINOR 0
#define VERSION_PATCH 0

/*********************************************
 *
 *  Version Information
 *
 *********************************************/
/**
 * @brief Get the RadarSimLib version information
 *
 * @details Retrieves the current version numbers for the RadarSimLib library.
 * Version follows semantic versioning (major.minor.patch).
 *
 * @param[out] version Pre-allocated array to store version numbers {major,
 * minor, patch} Array must have at least 3 elements
 *
 * @note This function is thread-safe and has no failure conditions
 */
EXPORTED void Get_Version(int version[3]);

/*********************************************
 *
 *  Transmitter Configuration
 *
 *********************************************/
/**
 * @brief Opaque pointer to Transmitter implementation
 * @details Forward declaration for C compatibility. Actual implementation is
 * hidden.
 */
struct s_Transmitter;
typedef struct s_Transmitter t_Transmitter;

/**
 * @brief Create a Transmitter object with waveform parameters
 *
 * @details Creates a new transmitter with specified frequency modulation and
 * timing parameters. Performs comprehensive input validation and uses modern
 * C++ memory management with automatic registration for cleanup.
 *
 * @param[in] freq Frequency vector (Hz) - must not be NULL and contain valid
 * frequencies
 * @param[in] freq_time Timestamp vector for the frequency samples (s) - must
 * not be NULL and be monotonically increasing
 * @param[in] waveform_size Length of freq and freq_time arrays - must be > 0
 * @param[in] freq_offset Frequency offset per pulse (Hz) - must not be NULL,
 *                        length should equal num_pulses
 * @param[in] pulse_start_time Pulse start time vector (s) - must not be NULL,
 *                             length should equal num_pulses, monotonically
 * increasing
 * @param[in] num_pulses Number of pulses - must be > 0
 * @param[in] tx_power Transmitter power (dBm) - typical range: -30 to +60 dBm
 *
 * @return t_Transmitter* Pointer to the Transmitter object on success, NULL on
 * failure
 *
 * @note The returned pointer is automatically registered for cleanup at program
 * exit. For manual cleanup, use Free_Transmitter().
 * @warning All input arrays must remain valid during the transmitter's lifetime
 *          or until the data is internally copied.
 */
EXPORTED t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                           int waveform_size,
                                           double *freq_offset,
                                           double *pulse_start_time,
                                           int num_pulses, float tx_power);

/**
 * @brief Add a transmitter channel with antenna pattern and modulation
 *
 * @details Configures a transmitter channel with 3D antenna radiation pattern,
 * polarization characteristics, and temporal/pulse modulation parameters.
 * Supports both amplitude and phase modulation with comprehensive pattern
 * definition.
 *
 * @param[in] location Channel location {x, y, z} (m) - must not be NULL
 * @param[in] polar_real Real part of polarization vector {x, y, z} - must not
 * be NULL
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z} - must
 * not be NULL
 * @param[in] phi Azimuth angles for radiation pattern (rad) - must be uniformly
 * spaced, not NULL
 * @param[in] phi_ptn Normalized phi pattern values (dB) - must not be NULL,
 * same size as phi
 * @param[in] phi_length Length of phi and phi_ptn arrays - must be > 0
 * @param[in] theta Elevation angles for radiation pattern (rad) - must be
 * uniformly spaced, not NULL
 * @param[in] theta_ptn Normalized theta pattern values (dB) - must not be NULL,
 * same size as theta
 * @param[in] theta_length Length of theta and theta_ptn arrays - must be > 0
 * @param[in] antenna_gain Maximum antenna gain (dB) - added to pattern values
 * @param[in] mod_t Modulation timestamps (s) - must be uniformly spaced if
 * mod_length > 0
 * @param[in] mod_var_real Real part of modulation amplitude - must not be NULL
 * if mod_length > 0
 * @param[in] mod_var_imag Imaginary part of modulation amplitude - must not be
 * NULL if mod_length > 0
 * @param[in] mod_length Length of modulation arrays - use 0 for no temporal
 * modulation
 * @param[in] pulse_mod_real Real part of pulse modulation - must not be NULL,
 *                           length should be the same as the number of pulses
 * defined in Transmitter
 * @param[in] pulse_mod_imag Imaginary part of pulse modulation - must not be
 * NULL, length should be the same as the number of pulses defined in
 * Transmitter
 * @param[in] delay Transmitting delay (s) - time offset for this channel
 * @param[in] grid Ray occupancy grid resolution (rad) - angular resolution for
 * ray tracing
 * @param[in] ptr_tx_c Pointer to the Transmitter object - must not be NULL
 *
 * @return int Status code: 0 for success, 1 for failure or free tier channel
 * limit exceeded
 *
 * @note Free tier is limited to 1 transmitter channel. Premium version supports
 * unlimited channels.
 * @warning Antenna patterns should be normalized and provided in dB relative to
 * maximum gain.
 */
EXPORTED int Add_Txchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain, float *mod_t,
                           float *mod_var_real, float *mod_var_imag,
                           int mod_length, float *pulse_mod_real,
                           float *pulse_mod_imag, float delay, float grid,
                           t_Transmitter *ptr_tx_c);

/**
 * @brief Get the number of configured transmitter channels
 *
 * @details Returns the current number of channels added to the transmitter.
 * Each channel represents a separate antenna element with its own pattern and
 * characteristics.
 *
 * @param[in] ptr_tx_c Pointer to the Transmitter object - must not be NULL
 *
 * @return int Number of configured transmitter channels (>= 0)
 *
 * @warning Undefined behavior if ptr_tx_c is NULL. Always validate pointer
 * before use.
 */
EXPORTED int Get_Num_Txchannel(t_Transmitter *ptr_tx_c);

/**
 * @brief Safely release transmitter resources
 *
 * @details Safely releases transmitter resources using modern C++ RAII
 * principles. Unregisters from automatic cleanup system and properly
 * deallocates memory. Safe to call with NULL pointer.
 *
 * @param[in] ptr_tx_c Pointer to the Transmitter object to free - may be NULL
 *
 * @note After calling this function, ptr_tx_c becomes invalid and should not be
 * used.
 * @note This function is exception-safe and will not throw.
 */
EXPORTED void Free_Transmitter(t_Transmitter *ptr_tx_c);

/*********************************************
 *
 *  Receiver Configuration
 *
 *********************************************/
/**
 * @brief Opaque pointer to Receiver implementation
 * @details Forward declaration for C compatibility. Actual implementation is
 * hidden.
 */
struct s_Receiver;
typedef struct s_Receiver t_Receiver;

/**
 * @brief Create a Receiver object with RF and baseband parameters
 *
 * @details Creates a new receiver with specified sampling rate, RF gain, and
 * baseband processing parameters. Uses RAII principles with shared_ptr for
 * automatic cleanup and exception safety.
 *
 * @param[in] fs Sampling rate (Hz) - must be > 0, typically 1 MHz to 100 MHz
 * @param[in] rf_gain RF amplifier gain (dB) - typical range: 0 to 60 dB
 * @param[in] resistor Load resistor (Ohm) - must be > 0, typically 50 or 75 Ohm
 * @param[in] baseband_gain Baseband amplifier gain (dB) - typical range: 0 to
 * 60 dB
 * @param[in] baseband_bw Baseband bandwidth (Hz) - must be >= 0, should be <=
 * fs/2
 *
 * @return t_Receiver* Pointer to the Receiver object on success, NULL on
 * failure
 *
 * @note The returned pointer is automatically registered for cleanup at program
 * exit. For manual cleanup, use Free_Receiver().
 * @warning baseband_bw should satisfy Nyquist criterion: baseband_bw <= fs/2
 */
EXPORTED t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                                     float baseband_gain, float baseband_bw);

/**
 * @brief Add a receiver channel with antenna pattern configuration
 *
 * @details Configures a receiver channel with 3D antenna radiation pattern and
 * polarization characteristics. The receiver channel defines how
 * electromagnetic signals are captured and processed by the radar system.
 *
 * @param[in] location Channel location {x, y, z} (m) - must not be NULL
 * @param[in] polar_real Real part of polarization vector {x, y, z} - must not
 * be NULL
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z} - must
 * not be NULL
 * @param[in] phi Azimuth angles for radiation pattern (rad) - must be uniformly
 * spaced, not NULL
 * @param[in] phi_ptn Normalized phi pattern values (dB) - must not be NULL,
 * same size as phi
 * @param[in] phi_length Length of phi and phi_ptn arrays - must be > 0
 * @param[in] theta Elevation angles for radiation pattern (rad) - must be
 * uniformly spaced, not NULL
 * @param[in] theta_ptn Normalized theta pattern values (dB) - must not be NULL,
 * same size as theta
 * @param[in] theta_length Length of theta and theta_ptn arrays - must be > 0
 * @param[in] antenna_gain Maximum antenna gain (dB) - added to pattern values
 * @param[in] ptr_rx_c Pointer to the Receiver object - must not be NULL
 *
 * @return int Status code: 0 for success, 1 for failure or free tier channel
 * limit exceeded
 *
 * @note Free tier is limited to 1 receiver channel. Premium version supports
 * unlimited channels.
 * @warning Antenna patterns should be normalized and provided in dB relative to
 * maximum gain.
 */
EXPORTED int Add_Rxchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain,
                           t_Receiver *ptr_rx_c);

/**
 * @brief Get the number of configured receiver channels
 *
 * @details Returns the current number of channels added to the receiver.
 * Each channel represents a separate antenna element with its own pattern and
 * characteristics.
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object - must not be NULL
 *
 * @return int Number of configured receiver channels (>= 0)
 *
 * @warning Undefined behavior if ptr_rx_c is NULL. Always validate pointer
 * before use.
 */
EXPORTED int Get_Num_Rxchannel(t_Receiver *ptr_rx_c);

/**
 * @brief Safely release receiver resources
 *
 * @details Safely releases receiver resources using modern C++ RAII principles.
 * Unregisters from automatic cleanup system and properly deallocates memory.
 * Safe to call with NULL pointer.
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object to free - may be NULL
 *
 * @note After calling this function, ptr_rx_c becomes invalid and should not be
 * used.
 * @note This function is exception-safe and will not throw.
 */
EXPORTED void Free_Receiver(t_Receiver *ptr_rx_c);

/*********************************************
 *
 *  Radar System Configuration
 *
 *********************************************/
/**
 * @brief Opaque pointer to Radar system implementation
 * @details Forward declaration for C compatibility. Actual implementation is
 * hidden.
 */
struct s_Radar;
typedef struct s_Radar t_Radar;

/**
 * @brief Create a complete Radar system from transmitter and receiver
 *
 * @details Creates a complete radar system by combining configured transmitter
 * and receiver components with platform motion parameters. Performs
 * comprehensive input validation and uses modern C++ memory management.
 *
 * @param[in] ptr_tx_c Pointer to configured Transmitter object - must not be
 * NULL
 * @param[in] ptr_rx_c Pointer to configured Receiver object - must not be NULL
 * @param[in] frame_start_time Frame start time vector (s) - must not be NULL,
 * monotonically increasing
 * @param[in] num_frames Number of radar frames - must be > 0
 * @param[in] location Initial radar platform location {x, y, z} (m) - must not
 * be NULL
 * @param[in] speed Radar platform velocity {x, y, z} (m/s) - must not be NULL
 * @param[in] rotation Initial radar platform orientation {x, y, z} (rad) - must
 * not be NULL
 * @param[in] rotation_rate Radar platform angular velocity {x, y, z} (rad/s) -
 * must not be NULL
 *
 * @return t_Radar* Pointer to the Radar system object on success, NULL on
 * failure
 *
 * @note The radar system maintains references to the provided transmitter and
 * receiver. Do not free tx/rx objects while the radar system is in use.
 * @note The returned pointer is automatically registered for cleanup at program
 * exit. For manual cleanup, use Free_Radar().
 * @warning Transmitter and receiver objects must remain valid for the radar
 * system's lifetime.
 */
EXPORTED t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c,
                               double *frame_start_time, int num_frames,
                               float *location, float *speed, float *rotation,
                               float *rotation_rate);

/**
 * @brief Safely release radar system resources
 *
 * @details Safely releases radar system resources using modern C++ RAII
 * principles. The underlying transmitter and receiver objects are NOT
 * automatically freed. Safe to call with NULL pointer.
 *
 * @param[in] ptr_radar_c Pointer to the Radar system object to free - may be
 * NULL
 *
 * @note After calling this function, ptr_radar_c becomes invalid and should not
 * be used.
 * @note Transmitter and receiver objects remain valid and must be freed
 * separately.
 * @note This function is exception-safe and will not throw.
 */
EXPORTED void Free_Radar(t_Radar *ptr_radar_c);

/*********************************************
 *
 *  Target Management System
 *
 *********************************************/
/**
 * @brief Opaque pointer to target management system implementation
 * @details Forward declaration for C compatibility. Actual implementation is
 * hidden.
 */
struct s_Targets;
typedef struct s_Targets t_Targets;

/**
 * @brief Initialize the target management system
 *
 * @details Creates and initializes both point and mesh target managers.
 * This function must be called before adding any targets to the simulation.
 * Uses RAII principles for automatic memory management.
 *
 * @return t_Targets* Pointer to the target management system on success, NULL
 * on failure
 *
 * @note The returned pointer is automatically registered for cleanup at program
 * exit. For manual cleanup, use Free_Targets().
 * @note This function creates empty target managers - use Add_Point_Target()
 * and Add_Mesh_Target() to populate with actual targets.
 */
EXPORTED t_Targets *Init_Targets();

/**
 * @brief Add an ideal point scatterer to the simulation
 *
 * @details Adds a point target with specified radar cross section and kinematic
 * properties. Point targets are ideal scatterers used for basic radar
 * simulations and provide fast computation for simple scenarios.
 *
 * @param[in] location Target's initial location {x, y, z} (m) - must not be
 * NULL
 * @param[in] speed Target's velocity vector {x, y, z} (m/s) - must not be NULL
 * @param[in] rcs Target's radar cross section (dBsm) - typical range: -40 to
 * +40 dBsm
 * @param[in] phs Target's initial phase (rad) - range: 0 to 2π, affects
 * interference patterns
 * @param[in] ptr_targets_c Pointer to the target management system - must not
 * be NULL
 *
 * @return int Status code: 0 for success, 1 for failure or free tier target
 * limit exceeded
 *
 * @note Free tier is limited to 2 point targets. Premium version supports
 * unlimited targets.
 * @note Point targets are assumed to be isotropic scatterers (RCS independent
 * of aspect angle).
 */
EXPORTED int Add_Point_Target(float *location, float *speed, float rcs,
                              float phs, t_Targets *ptr_targets_c);

/**
 * @brief Add a complex 3D mesh target to the simulation
 *
 * @details Adds a realistic 3D mesh target with electromagnetic material
 * properties and full 6-DOF motion characteristics. Mesh targets provide
 * accurate scattering behavior using Physical Optics and ray tracing methods.
 *
 * @param[in] points Mesh vertex coordinates array [x₁,y₁,z₁, x₂,y₂,z₂, ...] -
 * must not be NULL
 * @param[in] cells Triangle connectivity array [v₁,v₂,v₃, ...] (0-indexed) -
 * must not be NULL
 * @param[in] cell_size Number of triangular mesh faces - must be > 0
 * @param[in] origin Target's local coordinate origin (m) - must not be NULL
 * @param[in] location Target's initial location (m) - must not be NULL
 * @param[in] speed Target's velocity vector (m/s) - must not be NULL
 * @param[in] rotation Target's initial orientation (rad) - must not be NULL
 * @param[in] rotation_rate Target's angular velocity (rad/s) - must not be NULL
 * @param[in] ep_real Real part of relative permittivity εᵣ - typical range:
 * 1-100
 * @param[in] ep_imag Imaginary part of relative permittivity εᵣ - ≥ 0,
 * represents dielectric loss
 * @param[in] mu_real Real part of relative permeability μᵣ - typically ≈ 1 for
 * non-magnetic materials
 * @param[in] mu_imag Imaginary part of relative permeability μᵣ - ≥ 0,
 * represents magnetic loss
 * @param[in] is_ground Flag indicating if target represents ground surface
 * (affects ray tracing)
 * @param[in] ptr_targets_c Pointer to the target management system - must not
 * be NULL
 *
 * @return int Status code: 0 for success, 1 for failure or free tier limits
 * exceeded
 *
 * @note Free tier limits: 2 mesh targets max, 8 triangles per mesh max.
 * @note Mesh vertices should be in counter-clockwise order for proper normal
 * calculation.
 * @warning Large meshes (>1000 triangles) may require significant computation
 * time and memory.
 */
EXPORTED int Add_Mesh_Target(float *points, int *cells, int cell_size,
                             float *origin, float *location, float *speed,
                             float *rotation, float *rotation_rate,
                             float ep_real, float ep_imag, float mu_real,
                             float mu_imag, bool is_ground,
                             t_Targets *ptr_targets_c);

/**
 * @brief Safely release target management system resources
 *
 * @details Safely releases all target-related resources including point and
 * mesh targets using modern C++ RAII principles. Safe to call with NULL
 * pointer.
 *
 * @param[in] ptr_targets_c Pointer to the target management system to free -
 * may be NULL
 *
 * @note After calling this function, ptr_targets_c becomes invalid and should
 * not be used.
 * @note All point and mesh targets managed by this system are automatically
 * released.
 * @note This function is exception-safe and will not throw.
 */
EXPORTED void Free_Targets(t_Targets *ptr_targets_c);

/*********************************************
 *
 *  Simulation Engines
 *
 *********************************************/
/**
 * @brief Execute comprehensive radar simulation for all configured targets
 *
 * @details Runs complete radar simulation including both point and mesh
 * targets. Initializes baseband signal buffers, processes targets based on
 * their type, and synchronizes results. Supports GPU acceleration when
 * available for improved performance on complex scenarios.
 *
 * @param[in] ptr_radar_c Pointer to the radar system - must not be NULL and
 * fully configured
 * @param[in] ptr_targets_c Pointer to the target management system - must not
 * be NULL and finalized
 * @param[in] level Ray tracing quality level for mesh targets (1-5: higher =
 * more accurate but slower)
 * @param[in] density Ray density for mesh simulation (rays per square
 * wavelength) - typical: 1-10
 * @param[in] ray_filter Ray filter range {min_range, max_range} (m) - must not
 * be NULL
 * @param[out] ptr_bb_real Real part of baseband signal buffer - must be
 * pre-allocated, not NULL
 * @param[out] ptr_bb_imag Imaginary part of baseband signal buffer - must be
 * pre-allocated, not NULL
 *
 * @note Buffer size must match: [num_pulses × num_rx_channels ×
 * samples_per_pulse]
 * @warning Output buffers must be properly allocated before calling this
 * function. Buffer size mismatch will cause undefined behavior.
 */
EXPORTED void Run_RadarSimulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                                 int level, float density, int *ray_filter,
                                 double *ptr_bb_real, double *ptr_bb_imag);

/**
 * @brief Execute radar-to-radar interference simulation
 *
 * @details Simulates electromagnetic interference between two radar systems.
 * The victim radar receives interference from the interfering radar, allowing
 * analysis of interference effects on radar performance and spectrum sharing
 * scenarios.
 *
 * @param[in] ptr_radar_c Pointer to the victim radar system - must not be NULL
 * and fully configured
 * @param[in] ptr_interf_radar_c Pointer to the interfering radar system - must
 * not be NULL and fully configured
 * @param[out] ptr_interf_real Real part of the interference baseband buffer -
 * must be pre-allocated, not NULL
 * @param[out] ptr_interf_imag Imaginary part of the interference baseband
 * buffer - must be pre-allocated, not NULL
 *
 * @note Both radar systems must be fully configured with transmitters,
 * receivers, and motion parameters.
 * @note Buffer size must match the victim radar's baseband buffer dimensions.
 * @warning Output buffers must be properly allocated before calling this
 * function.
 */
EXPORTED void Run_InterferenceSimulator(t_Radar *ptr_radar_c,
                                        t_Radar *ptr_interf_radar_c,
                                        double *ptr_interf_real,
                                        double *ptr_interf_imag);

/**
 * @brief Execute Radar Cross Section (RCS) simulation using Physical Optics
 *
 * @details Calculates the monostatic and bistatic Radar Cross Section of mesh
 * targets using high-fidelity Physical Optics ray tracing methods. Supports
 * multiple incident and observation direction pairs for comprehensive RCS
 * pattern analysis. Uses BVH acceleration structures for efficient ray-triangle
 * intersection computation.
 *
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets - must not be NULL
 * @param[in] inc_dir_array Incident direction vectors array [x₁,y₁,z₁,
 * x₂,y₂,z₂, ...] - must not be NULL, unit vectors
 * @param[in] obs_dir_array Observation direction vectors array [x₁,y₁,z₁,
 * x₂,y₂,z₂, ...] - must not be NULL, unit vectors
 * @param[in] num_directions Number of direction pairs for RCS calculation -
 * must be > 0
 * @param[in] inc_polar_real Real part of incident polarization vector {x, y, z}
 * - must not be NULL, unit vector
 * @param[in] inc_polar_imag Imaginary part of incident polarization vector {x,
 * y, z} - must not be NULL
 * @param[in] obs_polar_real Real part of observation polarization vector {x, y,
 * z} - must not be NULL, unit vector
 * @param[in] obs_polar_imag Imaginary part of observation polarization vector
 * {x, y, z} - must not be NULL
 * @param[in] frequency Electromagnetic frequency (Hz) - must be > 0, typically
 * 1 GHz to 100 GHz
 * @param[in] density Ray density for Physical Optics (rays per square
 * wavelength) - must be > 0, typical: 5-20
 * @param[out] rcs_result Output array for RCS values (m²) - must be
 * pre-allocated, size ≥ num_directions
 *
 * @return int Status code: 0 for success, 1 for failure or invalid parameters
 *
 * @note Higher density values provide more accurate results but increase
 * computation time exponentially.
 * @note For monostatic RCS: incident and observation directions should be
 * identical but opposite.
 * @warning Direction vectors should be normalized unit vectors pointing away
 * from the target.
 */
EXPORTED int Run_RcsSimulator(t_Targets *ptr_targets_c, double *inc_dir_array,
                              double *obs_dir_array, int num_directions,
                              double *inc_polar_real, double *inc_polar_imag,
                              double *obs_polar_real, double *obs_polar_imag,
                              double frequency, double density,
                              double *rcs_result);

/**
 * @brief Execute LiDAR point cloud simulation using ray tracing
 *
 * @details Performs high-fidelity LiDAR point cloud generation using ray
 * tracing simulation. Shoots rays from the sensor position in specified angular
 * directions and records intersection points on target mesh surfaces. Uses BVH
 * acceleration structures for efficient ray-triangle intersection computation.
 *
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets - must not be NULL
 * @param[in] phi_array Azimuth angle array (rad) - must not be NULL, range: [0,
 * 2π]
 * @param[in] theta_array Elevation angle array (rad) - must not be NULL, range:
 * [0, π]
 * @param[in] num_rays Number of rays to shoot - must be > 0, equal to array
 * sizes
 * @param[in] sensor_location LiDAR sensor position {x, y, z} (m) - must not be
 * NULL
 * @param[out] cloud_points Output point cloud coordinates [x₁,y₁,z₁, x₂,y₂,z₂,
 * ...] - must be pre-allocated
 * @param[out] cloud_distances Output array for point distances from sensor (m)
 * - must be pre-allocated
 * @param[out] cloud_intensities Output array for point intensities (normalized)
 * - must be pre-allocated
 * @param[in] max_points Maximum number of points to return - must be > 0,
 * limits output size
 * @param[out] actual_points Actual number of points found and returned - must
 * not be NULL
 *
 * @return int Status code: 0 for success, 1 for failure or invalid parameters
 *
 * @note Spherical coordinate convention: φ=0 is +X axis, θ=0 is +Z axis
 * (elevation from XY-plane)
 * @note Only first-surface intersections are recorded (no multi-bounce or
 * penetration).
 * @note Output arrays must be allocated for at least max_points elements.
 * @warning Rays that miss all targets will not contribute to the output point
 * cloud.
 */
EXPORTED int Run_LidarSimulator(t_Targets *ptr_targets_c, double *phi_array,
                                double *theta_array, int num_rays,
                                double *sensor_location, double *cloud_points,
                                double *cloud_distances,
                                double *cloud_intensities, int max_points,
                                int *actual_points);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_H
