/*
 * @file radarsim.h
 * @brief C API header for RadarSimCpp library
 *
 * @details
 * C-compatible interface for RadarSimCpp library with opaque pointer pattern.
 * Provides radar simulation, target management, and automatic memory cleanup.
 *
 * Features:
 * - Transmitter/receiver configuration with antenna patterns
 * - Point and mesh target simulation
 * - RCS and LiDAR simulation
 * - Automatic memory management with cleanup registration
 * - Exception-safe resource handling
 *
 * @note Automatic cleanup provides safety net; manual Free_* calls recommended.
 * @warning Free tier has channel, target, and mesh complexity limits.
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

// #define RADARSIM_SIMPLE_CLEANUP

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
 * @param[out] version Pre-allocated array {major, minor, patch} - must have 3
 * elements
 */
EXPORTED void Get_Version(int version[3]);

/*********************************************
 *
 *  Memory Management Configuration
 *
 *********************************************/
/**
 * @brief Force cleanup of all automatically registered objects
 *
 * @note Called automatically on DLL unload. Use for testing or explicit cleanup
 * timing.
 * @warning All automatically managed objects become invalid after calling.
 */
EXPORTED void Force_Cleanup_All();

/**
 * @brief Check if automatic cleanup is currently in progress
 *
 * @return int 1 if cleanup is in progress, 0 otherwise
 */
EXPORTED int Is_Cleanup_In_Progress();

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
 * @param[in] freq Frequency vector (Hz)
 * @param[in] freq_time Timestamp vector for frequency samples (s)
 * @param[in] waveform_size Length of freq and freq_time arrays
 * @param[in] freq_offset Frequency offset per pulse (Hz)
 *            The length of this array must be equal to `num_pulses`.
 * @param[in] pulse_start_time Pulse start time vector (s)
 *            The length of this array must be equal to `num_pulses`.
 * @param[in] num_pulses Number of pulses
 * @param[in] tx_power Transmitter power (dBm)
 *
 * @return t_Transmitter* Pointer to Transmitter object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Transmitter() for manual
 * cleanup.
 */
EXPORTED t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                           int waveform_size,
                                           double *freq_offset,
                                           double *pulse_start_time,
                                           int num_pulses, float tx_power);

/**
 * @brief Add a transmitter channel with antenna pattern and modulation
 *
 * @param[in] location Channel location {x, y, z} (m)
 * @param[in] polar_real Real part of polarization vector {x, y, z}
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z}
 * @param[in] phi Azimuth angles for radiation pattern (rad)
 * @param[in] phi_ptn Normalized phi pattern values (dB)
 * @param[in] phi_length Length of phi and phi_ptn arrays
 * @param[in] theta Elevation angles for radiation pattern (rad)
 * @param[in] theta_ptn Normalized theta pattern values (dB)
 * @param[in] theta_length Length of theta and theta_ptn arrays
 * @param[in] antenna_gain Maximum antenna gain (dB)
 * @param[in] mod_t Modulation timestamps (s)
 * @param[in] mod_var_real Real part of modulation amplitude
 * @param[in] mod_var_imag Imaginary part of modulation amplitude
 * @param[in] mod_length Length of modulation arrays (0 for no modulation)
 * @param[in] pulse_mod_real Real part of pulse modulation
 * @param[in] pulse_mod_imag Imaginary part of pulse modulation
 * @param[in] delay Transmitting delay (s)
 * @param[in] grid Ray occupancy grid resolution (rad)
 * @param[in] ptr_tx_c Pointer to the Transmitter object
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 1 transmitter channel.
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
 * @param[in] ptr_tx_c Pointer to the Transmitter object
 *
 * @return int Number of configured transmitter channels
 */
EXPORTED int Get_Num_Txchannel(t_Transmitter *ptr_tx_c);

/**
 * @brief Safely release transmitter resources
 *
 * @param[in] ptr_tx_c Pointer to the Transmitter object to free (may be NULL)
 *
 * @note Automatically unregisters from cleanup system. Safe with NULL pointer.
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
 * @param[in] fs Sampling rate (Hz)
 * @param[in] rf_gain RF amplifier gain (dB)
 * @param[in] resistor Load resistor (Ohm)
 * @param[in] baseband_gain Baseband amplifier gain (dB)
 * @param[in] baseband_bw Baseband bandwidth (Hz)
 *
 * @return t_Receiver* Pointer to Receiver object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Receiver() for manual
 * cleanup.
 */
EXPORTED t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                                     float baseband_gain, float baseband_bw);

/**
 * @brief Add a receiver channel with antenna pattern configuration
 *
 * @param[in] location Channel location {x, y, z} (m)
 * @param[in] polar_real Real part of polarization vector {x, y, z}
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z}
 * @param[in] phi Azimuth angles for radiation pattern (rad)
 * @param[in] phi_ptn Normalized phi pattern values (dB)
 * @param[in] phi_length Length of phi and phi_ptn arrays
 * @param[in] theta Elevation angles for radiation pattern (rad)
 * @param[in] theta_ptn Normalized theta pattern values (dB)
 * @param[in] theta_length Length of theta and theta_ptn arrays
 * @param[in] antenna_gain Maximum antenna gain (dB)
 * @param[in] ptr_rx_c Pointer to the Receiver object
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 1 receiver channel.
 */
EXPORTED int Add_Rxchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain,
                           t_Receiver *ptr_rx_c);

/**
 * @brief Get the number of configured receiver channels
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object
 *
 * @return int Number of configured receiver channels
 */
EXPORTED int Get_Num_Rxchannel(t_Receiver *ptr_rx_c);

/**
 * @brief Safely release receiver resources
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object to free (may be NULL)
 *
 * @note Automatically unregisters from cleanup system. Safe with NULL pointer.
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
 * @param[in] ptr_tx_c Pointer to configured Transmitter object
 * @param[in] ptr_rx_c Pointer to configured Receiver object
 * @param[in] frame_start_time Frame start time vector (s)
 * @param[in] num_frames Number of radar frames
 * @param[in] location Initial radar platform location {x, y, z} (m)
 * @param[in] speed Radar platform velocity {x, y, z} (m/s)
 * @param[in] rotation Initial radar platform orientation {x, y, z} (rad)
 * @param[in] rotation_rate Radar platform angular velocity {x, y, z} (rad/s)
 *
 * @return t_Radar* Pointer to Radar system object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Radar() for manual
 * cleanup.
 * @warning TX/RX objects must remain valid for radar system's lifetime.
 */
EXPORTED t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c,
                               double *frame_start_time, int num_frames,
                               float *location, float *speed, float *rotation,
                               float *rotation_rate);

/**
 * @brief Get the required baseband buffer size for the given radar
 *
 * @details Returns the total number of samples required for baseband buffers
 * (real and imaginary) for the configured radar. Use this to pre-allocate
 * arrays passed to simulation functions such as `Run_RadarSimulator`.
 *
 * @param[in] ptr_radar_c Pointer to the radar system object
 * @return int Total baseband buffer size in samples, or 0 on invalid input
 */
EXPORTED int Get_BB_Size(t_Radar *ptr_radar_c);

/**
 * @brief Safely release radar system resources
 *
 * @param[in] ptr_radar_c Pointer to the Radar system object to free (may be
 * NULL)
 *
 * @note TX/RX objects are NOT freed - manage separately. Auto-unregisters from
 * cleanup.
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
 * @return t_Targets* Pointer to target management system, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Targets() for manual
 * cleanup.
 * @note Use Add_Point_Target() and Add_Mesh_Target() to populate targets.
 */
EXPORTED t_Targets *Init_Targets();

/**
 * @brief Add an ideal point scatterer to the simulation
 *
 * @param[in] location Target's initial location {x, y, z} (m)
 * @param[in] speed Target's velocity vector {x, y, z} (m/s)
 * @param[in] rcs Target's radar cross section (dBsm)
 * @param[in] phs Target's initial phase (rad)
 * @param[in] ptr_targets_c Pointer to the target management system
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 2 point targets.
 */
EXPORTED int Add_Point_Target(float *location, float *speed, float rcs,
                              float phs, t_Targets *ptr_targets_c);

/**
 * @brief Add a complex 3D mesh target to the simulation
 *
 * @param[in] points Mesh vertex coordinates [x₁,y₁,z₁, x₂,y₂,z₂, ...]
 * @param[in] cells Triangle connectivity array [v₁,v₂,v₃, ...] (0-indexed)
 * @param[in] cell_size Number of triangular mesh faces
 * @param[in] origin Target's local coordinate origin (m)
 * @param[in] location Target's initial location (m)
 * @param[in] speed Target's velocity vector (m/s)
 * @param[in] rotation Target's initial orientation (rad)
 * @param[in] rotation_rate Target's angular velocity (rad/s)
 * @param[in] ep_real Real part of relative permittivity εᵣ
 * @param[in] ep_imag Imaginary part of relative permittivity εᵣ
 * @param[in] mu_real Real part of relative permeability μᵣ
 * @param[in] mu_imag Imaginary part of relative permeability μᵣ
 * @param[in] is_ground Flag for ground surface (affects ray tracing)
 * @param[in] ptr_targets_c Pointer to the target management system
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limits: 2 mesh targets max, 8 triangles per mesh max.
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
 * @param[in] ptr_targets_c Pointer to the target management system to free (may
 * be NULL)
 *
 * @note Auto-unregisters from cleanup system. Safe with NULL pointer.
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
 * @param[in] ptr_radar_c Pointer to the radar system
 * @param[in] ptr_targets_c Pointer to the target management system
 * @param[in] level Ray tracing quality level for mesh targets (0-2)
 * @param[in] density Ray density for mesh simulation (rays per wavelength²)
 * @param[in] ray_filter Valid range for ray reflection indices [min, max]
 * @param[out] ptr_bb_real Real part of baseband signal buffer (pre-allocated)
 * @param[out] ptr_bb_imag Imaginary part of baseband signal buffer
 * (pre-allocated)
 *
 * @note Buffer size: [num_pulses × num_rx_channels × samples_per_pulse]
 * @warning Buffers must be pre-allocated to correct size.
 */
EXPORTED void Run_RadarSimulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                                 int level, float density, int *ray_filter,
                                 double *ptr_bb_real, double *ptr_bb_imag);

/**
 * @brief Execute radar-to-radar interference simulation
 *
 * @param[in] ptr_radar_c Pointer to the victim radar system
 * @param[in] ptr_interf_radar_c Pointer to the interfering radar system
 * @param[out] ptr_interf_real Real part of interference signal buffer
 * (pre-allocated)
 * @param[out] ptr_interf_imag Imaginary part of interference signal buffer
 * (pre-allocated)
 *
 * @note Buffer size: [num_pulses × num_rx_channels × samples_per_pulse]
 * @warning Buffers must match victim radar's baseband dimensions.
 */
EXPORTED void Run_InterferenceSimulator(t_Radar *ptr_radar_c,
                                        t_Radar *ptr_interf_radar_c,
                                        double *ptr_interf_real,
                                        double *ptr_interf_imag);

/**
 * @brief Execute Radar Cross Section (RCS) simulation using Physical Optics
 *
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets
 * @param[in] inc_dir_array Incident direction vectors [x₁,y₁,z₁, x₂,y₂,z₂, ...]
 * @param[in] obs_dir_array Observation direction vectors [x₁,y₁,z₁, x₂,y₂,z₂,
 * ...]
 * @param[in] num_directions Number of direction pairs for RCS calculation
 * @param[in] inc_polar_real Real part of incident polarization vector {x, y, z}
 * @param[in] inc_polar_imag Imaginary part of incident polarization vector {x,
 * y, z}
 * @param[in] obs_polar_real Real part of observation polarization vector {x, y,
 * z}
 * @param[in] obs_polar_imag Imaginary part of observation polarization vector
 * {x, y, z}
 * @param[in] frequency Electromagnetic frequency (Hz)
 * @param[in] density Ray density for Physical Optics (rays per wavelength²)
 * @param[out] rcs_result Output array for RCS values (m²) - pre-allocated
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Higher density = more accurate but slower computation.
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
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets
 * @param[in] phi_array Azimuth angle array (rad) - range: [0, 2π]
 * @param[in] theta_array Elevation angle array (rad) - range: [0, π]
 * @param[in] num_rays Number of rays to shoot
 * @param[in] sensor_location LiDAR sensor position {x, y, z} (m)
 * @param[out] cloud_points Output point cloud coordinates [x₁,y₁,z₁, x₂,y₂,z₂,
 * ...] (pre-allocated)
 * @param[out] cloud_distances Output distances from sensor (m) (pre-allocated)
 * @param[out] cloud_intensities Output point intensities (pre-allocated)
 * @param[in] max_points Maximum number of points to return
 * @param[out] actual_points Actual number of points found and returned
 *
 * @return int 0 for success, 1 for failure
 *
 * @note φ=0 is +X axis, θ=0 is +Z axis. Only first-surface intersections
 * recorded.
 * @warning Output arrays must be allocated for at least max_points elements.
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
