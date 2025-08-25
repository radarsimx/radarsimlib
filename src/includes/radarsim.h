/*
 *
 *    C wrapper of RadarSimCpp
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

<<<<<<< HEAD
#define VERSION_MAJOR 13
#define VERSION_MINOR 1
#define VERSION_PATCH 0

// Error codes
// Note: These values must match RadarSimErrorCode enum in type_def.hpp
#define RADARSIM_SUCCESS 0
#define RADARSIM_ERROR_NULL_POINTER 1
#define RADARSIM_ERROR_INVALID_PARAMETER 2
#define RADARSIM_ERROR_MEMORY_ALLOCATION 3
#define RADARSIM_ERROR_FREE_TIER_LIMIT 4
#define RADARSIM_ERROR_EXCEPTION 5
#define RADARSIM_ERROR_TOO_MANY_RAYS_PER_GRID 6
=======
#define VERSION_MAJOR 14
#define VERSION_MINOR 0
#define VERSION_PATCH 0
>>>>>>> 9f5e941c61813d8a71009bc77b57c8bb136c2601

/*********************************************
 *
 *  Version
 *
 *********************************************/
/**
 * @brief Get the version of RadarSimLib
 *
<<<<<<< HEAD
 * @param version Version numbers {major, minor}, must not be NULL
=======
 * @param version Version numbers {major, minor, patch}
>>>>>>> 9f5e941c61813d8a71009bc77b57c8bb136c2601
 */
EXPORTED void Get_Version(int version[3]);

/**
 * @brief Get error message string for error code
 *
 * @param error_code Error code from RadarSim functions
 * @return const char* Human-readable error message
 */
EXPORTED const char* Get_Error_Message(int error_code);

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
struct s_Transmitter;
typedef struct s_Transmitter t_Transmitter;

/**
 * @brief Create a Transmitter, return the pointer of the Transmitter
 *
 * @param freq Frequency vector (Hz), must not be NULL
 * @param freq_time Timestamp vector for the frequency vector (s), must not be NULL
 * @param waveform_size Length of the frequency and timestamp vector, must be > 0
 * @param freq_offset Frequency offset per pulse (Hz), length should equal to
 * the number of pulses, must not be NULL
 * @param pulse_start_time Pulse start time vector (s), length should equal to
<<<<<<< HEAD
 * the number of pulses, must not be NULL
 * @param num_pulses Number of pulses, must be > 0
 * @param tx_power Transmitter power (dBm)
 * @return t_Transmitter* Pointer to the Transmitter, NULL on failure
 * 
 * @note This function performs input validation and returns NULL if any
 *       parameter is invalid or memory allocation fails
=======
 * the number of pulses
 * @param num_pulses Number of pulses
 * @param tx_power Transmitter power (dBm)
 * @return t_Transmitter* Pointer to the Transmitter, NULL on failure
>>>>>>> 9f5e941c61813d8a71009bc77b57c8bb136c2601
 */
EXPORTED t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                           int waveform_size,
                                           double *freq_offset,
                                           double *pulse_start_time,
                                           int num_pulses, float tx_power);

/**
 * @brief Add a transmitter channel to Transmitter
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar_real Real part of the polarization vector [x, y, z]
 * @param polar_imag Imaginary part of the polarization vector [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad), angles must
 * be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad), angles
 * must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param mod_t Timestamp of the modulation data (s), mod_t must be equal-spaced
 * incremental array
 * @param mod_var_real Real part of modulation value vector
 * @param mod_var_imag Imaginary part of modulation value vector
 * @param mod_length Length of mod_t, mod_var_real and mod_var_imag
 * @param pulse_mod_real Real part of pulse modulation vector, the length should
 * be the same as the number of pulses defined in Transmitter
 * @param pulse_mod_imag Imaginary part of pulse modulation vector, the length
 * should be the same as the number of pulses defined in Transmitter
 * @param delay Transmitting delay (s)
 * @param grid Ray occupancy checking grid (rad)
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Status code (0 for success, 1 for failure)
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
 * @brief Get the number of Tx channels
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Number of Tx channels
 */
EXPORTED int Get_Num_Txchannel(t_Transmitter *ptr_tx_c);

/**
 * @brief Free the memory of Transmitter
 *
 * @param ptr_tx_c Pointer to the Transmitter
 */
EXPORTED void Free_Transmitter(t_Transmitter *ptr_tx_c);

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
struct s_Receiver;
typedef struct s_Receiver t_Receiver;

/**
 * @brief Create a Receiver, return the pointer of the Receiver
 *
 * @param fs Sampling rate (Hz)
 * @param rf_gain RF gain (dB)
 * @param resistor Load resistor (Ohm)
 * @param baseband_gain Baseband gain (dB)
 * @param baseband_bw Baseband bandwidth (Hz)
 * @return t_Receiver* Pointer to the Receiver, NULL on failure
 */
EXPORTED t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                                     float baseband_gain, float baseband_bw);

/**
 * @brief Add a receiver channel to Receiver
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar_real Real part of the polarization vector [x, y, z]
 * @param polar_imag Imaginary part of the polarization vector [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad), angles must
 * be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad), angles
 * must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Rxchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain,
                           t_Receiver *ptr_rx_c);

/**
 * @brief Get the number of Rx channels
 *
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Number of Rx channels
 */
EXPORTED int Get_Num_Rxchannel(t_Receiver *ptr_rx_c);

/**
 * @brief Free the memory of Receiver
 *
 * @param ptr_rx_c Pointer to the Receiver
 */
EXPORTED void Free_Receiver(t_Receiver *ptr_rx_c);

/*********************************************
 *
 *  Radar
 *
 *********************************************/
struct s_Radar;
typedef struct s_Radar t_Radar;

/**
 * @brief Create a Radar, return the pointer of the Radar
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @param ptr_rx_c Pointer to the Receiver
 * @param frame_start_time Frame start time vector (s)
 * @param num_frames Number of frames
 * @param location Radar's location {x, y, z} (m)
 * @param speed Radar's speed {x, y, z} (m/s)
 * @param rotation Radar's rotation {x, y, z} (rad)
 * @param rotation_rate Radar's rotation rate {x, y, z} (rad/s)
 * @return t_Radar* Pointer to the Radar, NULL on failure
 */
EXPORTED t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c,
                               double *frame_start_time, int num_frames,
                               float *location, float *speed, float *rotation,
                               float *rotation_rate);

/**
 * @brief Free the memory of Radar
 *
 * @param ptr_radar_c Pointer to the Radar
 */
EXPORTED void Free_Radar(t_Radar *ptr_radar_c);

/*********************************************
 *
 *  Targets
 *
 *********************************************/
struct s_Targets;
typedef struct s_Targets t_Targets;

/**
 * @brief Initialize the target list
 *
 * @return t_Targets* Pointer to the target list, NULL on failure
 */
EXPORTED t_Targets *Init_Targets();

/**
 * @brief Add an ideal point target to the target list
 *
 * @param location Target's location {x, y, z} (m)
 * @param speed Target's speed {x, y, z} (m/s)
 * @param rcs Target's RCS (dBsm)
 * @param phs Target's phase (rad)
 * @param ptr_targets_c Pointer to the target list
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Point_Target(float *location, float *speed, float rcs,
                              float phs, t_Targets *ptr_targets_c);

/**
 * @brief Add a 3D mesh target to the target list
 *
 * @param points Mesh coordinates
 * @param cells Mesh connections
 * @param cell_size Number of meshes
 * @param origin Target origin (m)
 * @param location Target location (m)
 * @param speed Target speed (m/s)
 * @param rotation Target rotation (rad)
 * @param rotation_rate Target rotation rate (rad/s)
 * @param ep_real Real part of Permittivity
 * @param ep_imag Imaginary part of Permittivity
 * @param mu_real Real part of Permeability
 * @param mu_imag Imaginary part of Permeability
 * @param is_ground Flag to identify if the target is ground
 * @param ptr_targets_c Pointer to the target list
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Mesh_Target(float *points, int *cells, int cell_size,
                             float *origin, float *location, float *speed,
                             float *rotation, float *rotation_rate,
                             float ep_real, float ep_imag, float mu_real,
                             float mu_imag, bool is_ground,
                             t_Targets *ptr_targets_c);

/**
 * @brief Free the memory of target list
 *
 * @param ptr_targets_c Pointer to the target list
 */
EXPORTED void Free_Targets(t_Targets *ptr_targets_c);

/**
 * @brief Complete the initialization of targets
 *
 * @param ptr_targets_c Pointer to the target list
 */
EXPORTED void Complete_Targets_Initialization(t_Targets *ptr_targets_c);

/*********************************************
 *
 *  Simulator
 *
 *********************************************/
/**
 * @brief Run simulator for the ideal and mesh targets
 *
 * @param ptr_radar_c Pointer to the radar
 * @param ptr_targets_c Pointer to the target list
 * @param level Fidelity level of ray tracing
 * @param density Ray density
 * @param ray_filter Ray filter parameters
 * @param ptr_bb_real Real part of baseband samples
 * @param ptr_bb_imag Imaginary part of baseband samples
 * @return int Error code (RADARSIM_SUCCESS on success, error code on failure)
 */
<<<<<<< HEAD
EXPORTED int Run_RadarSimulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                           int level, float density, int *ray_filter,
                           double *ptr_bb_real, double *ptr_bb_imag);
=======
EXPORTED void Run_RadarSimulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                                 int level, float density, int *ray_filter,
                                 double *ptr_bb_real, double *ptr_bb_imag);
>>>>>>> 9f5e941c61813d8a71009bc77b57c8bb136c2601

/**
 * @brief Run interference simulation
 *
 * @param ptr_radar_c Pointer to the victim radar
 * @param ptr_interf_radar_c Pointer to the interference radar
 * @param ptr_interf_real Real part of the interference baseband
 * @param ptr_interf_imag Imaginary part of the interference baseband
 */
EXPORTED void Run_InterferenceSimulator(t_Radar *ptr_radar_c,
<<<<<<< HEAD
                               t_Radar *ptr_interf_radar_c,
                               double *ptr_interf_real,
                               double *ptr_interf_imag);
=======
                                        t_Radar *ptr_interf_radar_c,
                                        double *ptr_interf_real,
                                        double *ptr_interf_imag);

/**
 * @brief Execute Radar Cross Section (RCS) simulation
 *
 * @param ptr_targets_c Pointer to the target management system
 * @param inc_dir_array Array of incident direction vectors {x, y, z}
 * @param obs_dir_array Array of observation direction vectors {x, y, z}
 * @param num_directions Number of direction pairs
 * @param inc_polar_real Real part of incident polarization vector {x, y, z}
 * @param inc_polar_imag Imaginary part of incident polarization vector {x, y,
 * z}
 * @param obs_polar_real Real part of observation polarization vector {x, y, z}
 * @param obs_polar_imag Imaginary part of observation polarization vector {x,
 * y, z}
 * @param frequency Electromagnetic frequency (Hz)
 * @param density Ray density for Physical Optics (rays per wavelength)
 * @param rcs_result Output array for RCS values (m²)
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Run_RcsSimulator(t_Targets *ptr_targets_c, double *inc_dir_array,
                              double *obs_dir_array, int num_directions,
                              double *inc_polar_real, double *inc_polar_imag,
                              double *obs_polar_real, double *obs_polar_imag,
                              double frequency, double density,
                              double *rcs_result);

/**
 * @brief Execute LiDAR point cloud simulation
 *
 * @param ptr_targets_c Pointer to the target management system
 * @param phi_array Array of azimuth angles (rad)
 * @param theta_array Array of elevation angles (rad)
 * @param num_rays Number of rays to shoot
 * @param sensor_location LiDAR sensor position {x, y, z} (m)
 * @param cloud_points Output array for point cloud coordinates {x, y, z}
 * @param cloud_distances Output array for point distances (m)
 * @param cloud_intensities Output array for point intensities
 * @param max_points Maximum number of points to return
 * @param actual_points Output: actual number of points found
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Run_LidarSimulator(t_Targets *ptr_targets_c, double *phi_array,
                                double *theta_array, int num_rays,
                                double *sensor_location, double *cloud_points,
                                double *cloud_distances,
                                double *cloud_intensities, int max_points,
                                int *actual_points);
>>>>>>> 9f5e941c61813d8a71009bc77b57c8bb136c2601

/*********************************************
 *
 *  RCS Simulator
 *
 *********************************************/
typedef struct s_RcsSimulator t_RcsSimulator;

/**
 * @brief Create an RCS Simulator, return the pointer to the RCS Simulator
 *
 * @return t_RcsSimulator* Pointer to the RCS Simulator
 */
EXPORTED t_RcsSimulator *Create_RcsSimulator();

/**
 * @brief Calculate target RCS
 *
 * @param ptr_rcs_c Pointer to the RCS Simulator
 * @param ptr_targets_c Pointer to the target list
 * @param inc_dir_x Array of incident direction x components
 * @param inc_dir_y Array of incident direction y components  
 * @param inc_dir_z Array of incident direction z components
 * @param obs_dir_x Array of observation direction x components
 * @param obs_dir_y Array of observation direction y components
 * @param obs_dir_z Array of observation direction z components
 * @param num_directions Number of direction pairs
 * @param inc_polar_real Real part of incident polarization vector [x, y, z]
 * @param inc_polar_imag Imaginary part of incident polarization vector [x, y, z]
 * @param obs_polar_real Real part of observation polarization vector [x, y, z]
 * @param obs_polar_imag Imaginary part of observation polarization vector [x, y, z]
 * @param frequency Frequency (Hz)
 * @param density Ray density, number of rays per wavelength
 * @param ptr_rcs_results Array to store RCS results (m^2), size should be num_directions
 * @return int Status code (0 for success, error code for failure)
 */
EXPORTED int Run_RcsSimulator(t_RcsSimulator *ptr_rcs_c, t_Targets *ptr_targets_c,
                     double *inc_dir_x, double *inc_dir_y, double *inc_dir_z,
                     double *obs_dir_x, double *obs_dir_y, double *obs_dir_z,
                     int num_directions,
                     double *inc_polar_real, double *inc_polar_imag,
                     double *obs_polar_real, double *obs_polar_imag,
                     double frequency, double density, double *ptr_rcs_results);

/**
 * @brief Free the memory of RCS Simulator
 *
 * @param ptr_rcs_c Pointer to the RCS Simulator
 */
EXPORTED void Free_RcsSimulator(t_RcsSimulator *ptr_rcs_c);

/*********************************************
 *
 *  LiDAR Simulator
 *
 *********************************************/
typedef struct s_LidarSimulator t_LidarSimulator;

/**
 * @brief Create a LiDAR Simulator, return the pointer to the LiDAR Simulator
 *
 * @return t_LidarSimulator* Pointer to the LiDAR Simulator
 */
EXPORTED t_LidarSimulator *Create_LidarSimulator();

/**
 * @brief Add a target to the LiDAR Simulator
 *
 * @param ptr_lidar_c Pointer to the LiDAR Simulator
 * @param ptr_targets_c Pointer to the target list
 * @return int Status code (0 for success, error code for failure)
 */
EXPORTED int Add_Target_To_LidarSimulator(t_LidarSimulator *ptr_lidar_c, t_Targets *ptr_targets_c);

/**
 * @brief Run LiDAR point cloud simulation
 *
 * @param ptr_lidar_c Pointer to the LiDAR Simulator
 * @param phi Array of azimuth angles (rad)
 * @param theta Array of elevation angles (rad)
 * @param num_rays Number of rays (length of phi and theta arrays)
 * @param location LiDAR location [x, y, z] (m)
 * @param ptr_points_x Array to store point cloud x coordinates (m), size should be >= num_rays
 * @param ptr_points_y Array to store point cloud y coordinates (m), size should be >= num_rays
 * @param ptr_points_z Array to store point cloud z coordinates (m), size should be >= num_rays
 * @param ptr_ranges Array to store point ranges (m), size should be >= num_rays
 * @param ptr_num_points Pointer to store the actual number of points generated
 * @return int Status code (0 for success, error code for failure)
 */
EXPORTED int Run_LidarSimulator(t_LidarSimulator *ptr_lidar_c,
                       double *phi, double *theta, int num_rays,
                       double *location,
                       double *ptr_points_x, double *ptr_points_y, double *ptr_points_z,
                       double *ptr_ranges, int *ptr_num_points);

/**
 * @brief Clear the point cloud in the LiDAR Simulator
 *
 * @param ptr_lidar_c Pointer to the LiDAR Simulator
 */
EXPORTED void Clear_LidarSimulator_Cloud(t_LidarSimulator *ptr_lidar_c);

/**
 * @brief Free the memory of LiDAR Simulator
 *
 * @param ptr_lidar_c Pointer to the LiDAR Simulator
 */
EXPORTED void Free_LidarSimulator(t_LidarSimulator *ptr_lidar_c);

/*********************************************
 *
 *  Utility Functions
 *
 *********************************************/
/**
 * @brief Check if a pointer is valid (non-NULL)
 *
 * @param ptr Pointer to check
 * @return int 1 if valid, 0 if NULL
 */
EXPORTED int Is_Valid_Pointer(void *ptr);

/**
 * @brief Get the number of available CPU cores for simulation
 *
 * @return int Number of CPU cores
 */
EXPORTED int Get_CPU_Core_Count();

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_H
