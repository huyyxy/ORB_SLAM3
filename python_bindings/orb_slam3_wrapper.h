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

// Mode control functions
void orb_slam3_activate_localization_mode(ORBSLAMSystemHandle* system);
void orb_slam3_deactivate_localization_mode(ORBSLAMSystemHandle* system);
void orb_slam3_reset(ORBSLAMSystemHandle* system);

// Save functions
void orb_slam3_save_trajectory_tum(ORBSLAMSystemHandle* system, const char* filename);
void orb_slam3_save_trajectory_kitti(ORBSLAMSystemHandle* system, const char* filename);

#ifdef __cplusplus
}
#endif

#endif // ORB_SLAM3_WRAPPER_H
