#ifndef ORB_SLAM3_WRAPPER_H
#define ORB_SLAM3_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef void* ORBSLAMSystemHandle;

// Sensor types (must match System::eSensor)
typedef enum {
    SENSOR_MONOCULAR = 0,
    SENSOR_STEREO = 1,
    SENSOR_RGBD = 2,
    SENSOR_IMU_MONOCULAR = 3,
    SENSOR_IMU_STEREO = 4,
    SENSOR_IMU_RGBD = 5
} SensorType;

// Tracking states
typedef enum {
    TRACKING_SYSTEM_NOT_READY = -1,
    TRACKING_NO_IMAGES_YET = 0,
    TRACKING_NOT_INITIALIZED = 1,
    TRACKING_OK = 2,
    TRACKING_LOST = 3
} TrackingState;

// Camera pose structure (4x4 transformation matrix)
typedef struct {
    float data[16];  // Row-major order: [R|t; 0|1]
    int valid;       // 1 if pose is valid, 0 otherwise
} CameraPose;

// 3D Point structure
typedef struct {
    float x, y, z;
} Point3D;

// 2D Keypoint structure
typedef struct {
    float x, y;
    float angle;
    float response;
    int octave;
} KeyPoint2D;

// Map points structure
typedef struct {
    Point3D* points;
    int num_points;
} MapPoints;

// Tracked keypoints structure
typedef struct {
    KeyPoint2D* keypoints;
    int num_keypoints;
} TrackedKeyPoints;

// 2D Occupancy Map structure
typedef struct {
    unsigned char* data;  // Occupancy map data (0=occupied, 255=free, 127=unknown)
    int width;           // Map width in pixels
    int height;          // Map height in pixels
    float resolution;    // Resolution in meters per pixel
    float origin_x;      // Origin X coordinate in meters
    float origin_y;      // Origin Y coordinate in meters
    int valid;          // 1 if map is valid, 0 otherwise
} OccupancyMap2D;

// System management functions
ORBSLAMSystemHandle* orb_slam3_create_system(
    const char* vocab_file,
    const char* settings_file,
    SensorType sensor_type,
    int use_viewer
);

void orb_slam3_destroy_system(ORBSLAMSystemHandle* system);

void orb_slam3_shutdown(ORBSLAMSystemHandle* system);
int orb_slam3_is_shutdown(ORBSLAMSystemHandle* system);

// Tracking functions
CameraPose orb_slam3_track_rgbd(
    ORBSLAMSystemHandle* system,
    unsigned char* rgb_data,
    float* depth_data,
    int width,
    int height,
    double timestamp
);

CameraPose orb_slam3_track_stereo(
    ORBSLAMSystemHandle* system,
    unsigned char* left_data,
    unsigned char* right_data,
    int width,
    int height,
    double timestamp
);

CameraPose orb_slam3_track_monocular(
    ORBSLAMSystemHandle* system,
    unsigned char* image_data,
    int width,
    int height,
    double timestamp
);

// State query functions
TrackingState orb_slam3_get_tracking_state(ORBSLAMSystemHandle* system);
int orb_slam3_is_lost(ORBSLAMSystemHandle* system);
int orb_slam3_map_changed(ORBSLAMSystemHandle* system);

// Map data access functions
MapPoints orb_slam3_get_tracked_map_points(ORBSLAMSystemHandle* system);
TrackedKeyPoints orb_slam3_get_tracked_keypoints(ORBSLAMSystemHandle* system);
MapPoints orb_slam3_get_all_map_points(ORBSLAMSystemHandle* system);

// Memory management for returned data
void orb_slam3_free_map_points(MapPoints* map_points);
void orb_slam3_free_keypoints(TrackedKeyPoints* keypoints);
void orb_slam3_free_occupancy_map(OccupancyMap2D* occupancy_map);

// Mode control functions
void orb_slam3_activate_localization_mode(ORBSLAMSystemHandle* system);
void orb_slam3_deactivate_localization_mode(ORBSLAMSystemHandle* system);
void orb_slam3_reset(ORBSLAMSystemHandle* system);

// Save functions
void orb_slam3_save_trajectory_tum(ORBSLAMSystemHandle* system, const char* filename);
void orb_slam3_save_trajectory_kitti(ORBSLAMSystemHandle* system, const char* filename);

// Occupancy map generation functions
OccupancyMap2D orb_slam3_generate_occupancy_map(
    ORBSLAMSystemHandle* system,
    float resolution,        // Resolution in meters per pixel
    float robot_radius,      // Robot radius in meters for inflation
    float height_min,        // Minimum height threshold in meters
    float height_max,        // Maximum height threshold in meters
    float map_extension      // Map boundary extension in meters
);

#ifdef __cplusplus
}
#endif

#endif // ORB_SLAM3_WRAPPER_H
