#include "orb_slam3_wrapper.h"
#include "../include/System.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

// Internal structure to hold the ORB-SLAM3 system
struct ORBSLAMSystemImpl {
    ORB_SLAM3::System* system;
    
    ORBSLAMSystemImpl(const string& vocab_file, const string& settings_file, 
                      ORB_SLAM3::System::eSensor sensor, bool use_viewer)
        : system(new ORB_SLAM3::System(vocab_file, settings_file, sensor, use_viewer)) {}
    
    ~ORBSLAMSystemImpl() {
        if (system) {
            system->Shutdown();
            delete system;
        }
    }
};

// Helper function to convert ORB-SLAM3 sensor type
ORB_SLAM3::System::eSensor convert_sensor_type(SensorType sensor_type) {
    switch (sensor_type) {
        case SENSOR_MONOCULAR: return ORB_SLAM3::System::MONOCULAR;
        case SENSOR_STEREO: return ORB_SLAM3::System::STEREO;
        case SENSOR_RGBD: return ORB_SLAM3::System::RGBD;
        case SENSOR_IMU_MONOCULAR: return ORB_SLAM3::System::IMU_MONOCULAR;
        case SENSOR_IMU_STEREO: return ORB_SLAM3::System::IMU_STEREO;
        case SENSOR_IMU_RGBD: return ORB_SLAM3::System::IMU_RGBD;
        default: return ORB_SLAM3::System::RGBD;
    }
}

// Helper function to convert Sophus::SE3f to CameraPose
CameraPose convert_pose(const Sophus::SE3f& pose) {
    CameraPose camera_pose;
    camera_pose.valid = 1;
    
    // Convert SE3 to 4x4 matrix
    Eigen::Matrix4f matrix = pose.matrix();
    for (int i = 0; i < 16; i++) {
        camera_pose.data[i] = matrix.data()[i];
    }
    
    return camera_pose;
}

// Helper function to create invalid pose
CameraPose create_invalid_pose() {
    CameraPose camera_pose;
    camera_pose.valid = 0;
    for (int i = 0; i < 16; i++) {
        camera_pose.data[i] = 0.0f;
    }
    return camera_pose;
}

extern "C" {

ORBSLAMSystemHandle* orb_slam3_create_system(
    const char* vocab_file,
    const char* settings_file,
    SensorType sensor_type,
    int use_viewer
) {
    try {
        string vocab_str(vocab_file);
        string settings_str(settings_file);
        ORB_SLAM3::System::eSensor sensor = convert_sensor_type(sensor_type);
        bool viewer = (use_viewer != 0);
        
        cout << "Creating ORB-SLAM3 system with:" << endl;
        cout << "  Vocabulary file: " << vocab_str << endl;
        cout << "  Settings file: " << settings_str << endl;
        cout << "  Sensor type: " << sensor << endl;
        cout << "  Use viewer: " << (viewer ? "true" : "false") << endl;
        
        // 检查文件是否存在
        ifstream vocab_check(vocab_str);
        if (!vocab_check.good()) {
            cerr << "Vocabulary file does not exist or cannot be read: " << vocab_str << endl;
            return nullptr;
        }
        vocab_check.close();
        
        ifstream settings_check(settings_str);
        if (!settings_check.good()) {
            cerr << "Settings file does not exist or cannot be read: " << settings_str << endl;
            return nullptr;
        }
        settings_check.close();
        
        cout << "Files verified, creating system..." << endl;
        ORBSLAMSystemImpl* impl = new ORBSLAMSystemImpl(vocab_str, settings_str, sensor, viewer);
        cout << "ORB-SLAM3 system created successfully!" << endl;
        return reinterpret_cast<ORBSLAMSystemHandle*>(impl);
    } catch (const exception& e) {
        cerr << "Error creating ORB-SLAM3 system: " << e.what() << endl;
        return nullptr;
    } catch (...) {
        cerr << "Unknown error creating ORB-SLAM3 system" << endl;
        return nullptr;
    }
}

void orb_slam3_destroy_system(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        delete impl;
    }
}

void orb_slam3_shutdown(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            impl->system->Shutdown();
        }
    }
}

int orb_slam3_is_shutdown(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            return impl->system->isShutDown() ? 1 : 0;
        }
    }
    return 1;
}

CameraPose orb_slam3_track_rgbd(
    ORBSLAMSystemHandle* system,
    unsigned char* rgb_data,
    float* depth_data,
    int width,
    int height,
    double timestamp
) {
    if (!system) {
        return create_invalid_pose();
    }
    
    try {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        
        // Create OpenCV matrices from raw data
        cv::Mat rgb_image(height, width, CV_8UC3, rgb_data);
        cv::Mat depth_image(height, width, CV_32F, depth_data);
        
        // Track the frame
        Sophus::SE3f pose = impl->system->TrackRGBD(rgb_image, depth_image, timestamp);
        
        return convert_pose(pose);
    } catch (const exception& e) {
        cerr << "Error in RGBD tracking: " << e.what() << endl;
        return create_invalid_pose();
    }
}

CameraPose orb_slam3_track_stereo(
    ORBSLAMSystemHandle* system,
    unsigned char* left_data,
    unsigned char* right_data,
    int width,
    int height,
    double timestamp
) {
    if (!system) {
        return create_invalid_pose();
    }
    
    try {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        
        // Create OpenCV matrices from raw data
        cv::Mat left_image(height, width, CV_8UC1, left_data);
        cv::Mat right_image(height, width, CV_8UC1, right_data);
        
        // Track the frame
        Sophus::SE3f pose = impl->system->TrackStereo(left_image, right_image, timestamp);
        
        return convert_pose(pose);
    } catch (const exception& e) {
        cerr << "Error in stereo tracking: " << e.what() << endl;
        return create_invalid_pose();
    }
}

CameraPose orb_slam3_track_monocular(
    ORBSLAMSystemHandle* system,
    unsigned char* image_data,
    int width,
    int height,
    double timestamp
) {
    if (!system) {
        return create_invalid_pose();
    }
    
    try {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        
        // Create OpenCV matrix from raw data
        cv::Mat image(height, width, CV_8UC1, image_data);
        
        // Track the frame
        Sophus::SE3f pose = impl->system->TrackMonocular(image, timestamp);
        
        return convert_pose(pose);
    } catch (const exception& e) {
        cerr << "Error in monocular tracking: " << e.what() << endl;
        return create_invalid_pose();
    }
}

TrackingState orb_slam3_get_tracking_state(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            int state = impl->system->GetTrackingState();
            return static_cast<TrackingState>(state);
        }
    }
    return TRACKING_SYSTEM_NOT_READY;
}

int orb_slam3_is_lost(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            return impl->system->isLost() ? 1 : 0;
        }
    }
    return 1;
}

int orb_slam3_map_changed(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            return impl->system->MapChanged() ? 1 : 0;
        }
    }
    return 0;
}

MapPoints orb_slam3_get_tracked_map_points(ORBSLAMSystemHandle* system) {
    MapPoints result = {nullptr, 0};
    
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            try {
                vector<ORB_SLAM3::MapPoint*> map_points = impl->system->GetTrackedMapPoints();
                
                if (!map_points.empty()) {
                    result.num_points = 0;
                    // Count valid points
                    for (auto* mp : map_points) {
                        if (mp && !mp->isBad()) {
                            result.num_points++;
                        }
                    }
                    
                    if (result.num_points > 0) {
                        result.points = new Point3D[result.num_points];
                        int idx = 0;
                        
                        for (auto* mp : map_points) {
                            if (mp && !mp->isBad()) {
                                Eigen::Vector3f pos = mp->GetWorldPos();
                                result.points[idx].x = pos[0];
                                result.points[idx].y = pos[1];
                                result.points[idx].z = pos[2];
                                idx++;
                            }
                        }
                    }
                }
            } catch (const exception& e) {
                cerr << "Error getting tracked map points: " << e.what() << endl;
            }
        }
    }
    
    return result;
}

TrackedKeyPoints orb_slam3_get_tracked_keypoints(ORBSLAMSystemHandle* system) {
    TrackedKeyPoints result = {nullptr, 0};
    
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            try {
                vector<cv::KeyPoint> keypoints = impl->system->GetTrackedKeyPointsUn();
                
                if (!keypoints.empty()) {
                    result.num_keypoints = keypoints.size();
                    result.keypoints = new KeyPoint2D[result.num_keypoints];
                    
                    for (size_t i = 0; i < keypoints.size(); i++) {
                        result.keypoints[i].x = keypoints[i].pt.x;
                        result.keypoints[i].y = keypoints[i].pt.y;
                        result.keypoints[i].angle = keypoints[i].angle;
                        result.keypoints[i].response = keypoints[i].response;
                        result.keypoints[i].octave = keypoints[i].octave;
                    }
                }
            } catch (const exception& e) {
                cerr << "Error getting tracked keypoints: " << e.what() << endl;
            }
        }
    }
    
    return result;
}

MapPoints orb_slam3_get_all_map_points(ORBSLAMSystemHandle* system) {
    MapPoints result = {nullptr, 0};
    
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            try {
                // Note: ORB-SLAM3 doesn't have a direct GetAllMapPoints method in System class
                // You would need to add this method to System class or access through Atlas
                // For now, return tracked map points
                return orb_slam3_get_tracked_map_points(system);
            } catch (const exception& e) {
                cerr << "Error getting all map points: " << e.what() << endl;
            }
        }
    }
    
    return result;
}

void orb_slam3_free_map_points(MapPoints* map_points) {
    if (map_points && map_points->points) {
        delete[] map_points->points;
        map_points->points = nullptr;
        map_points->num_points = 0;
    }
}

void orb_slam3_free_keypoints(TrackedKeyPoints* keypoints) {
    if (keypoints && keypoints->keypoints) {
        delete[] keypoints->keypoints;
        keypoints->keypoints = nullptr;
        keypoints->num_keypoints = 0;
    }
}

void orb_slam3_activate_localization_mode(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            impl->system->ActivateLocalizationMode();
        }
    }
}

void orb_slam3_deactivate_localization_mode(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            impl->system->DeactivateLocalizationMode();
        }
    }
}

void orb_slam3_reset(ORBSLAMSystemHandle* system) {
    if (system) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            impl->system->Reset();
        }
    }
}

void orb_slam3_save_trajectory_tum(ORBSLAMSystemHandle* system, const char* filename) {
    if (system && filename) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            string filename_str(filename);
            impl->system->SaveTrajectoryTUM(filename_str);
        }
    }
}

void orb_slam3_save_trajectory_kitti(ORBSLAMSystemHandle* system, const char* filename) {
    if (system && filename) {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (impl->system) {
            string filename_str(filename);
            impl->system->SaveTrajectoryKITTI(filename_str);
        }
    }
}

// Helper function to create invalid occupancy map
OccupancyMap2D create_invalid_occupancy_map() {
    OccupancyMap2D map;
    map.data = nullptr;
    map.width = 0;
    map.height = 0;
    map.resolution = 0.0f;
    map.origin_x = 0.0f;
    map.origin_y = 0.0f;
    map.valid = 0;
    return map;
}

OccupancyMap2D orb_slam3_generate_occupancy_map(
    ORBSLAMSystemHandle* system,
    float resolution,
    float robot_radius,
    float height_min,
    float height_max,
    float map_extension
) {
    if (!system) {
        return create_invalid_occupancy_map();
    }
    
    try {
        ORBSLAMSystemImpl* impl = reinterpret_cast<ORBSLAMSystemImpl*>(system);
        if (!impl->system) {
            return create_invalid_occupancy_map();
        }
        
        // Get all map points from the system
        vector<ORB_SLAM3::MapPoint*> all_map_points = impl->system->GetTrackedMapPoints();
        if (all_map_points.empty()) {
            cerr << "No map points available for occupancy map generation" << endl;
            return create_invalid_occupancy_map();
        }
        
        // Filter map points by height and collect 2D positions
        vector<Eigen::Vector2f> filtered_points;
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        
        for (auto* mp : all_map_points) {
            if (mp && !mp->isBad()) {
                Eigen::Vector3f pos = mp->GetWorldPos();
                
                // Filter by height
                if (pos.z() >= height_min && pos.z() <= height_max) {
                    filtered_points.emplace_back(pos.x(), pos.y());
                    
                    // Update bounds
                    min_x = min(min_x, pos.x());
                    max_x = max(max_x, pos.x());
                    min_y = min(min_y, pos.y());
                    max_y = max(max_y, pos.y());
                }
            }
        }
        
        if (filtered_points.empty()) {
            cerr << "No valid map points found in height range [" << height_min 
                 << ", " << height_max << "]" << endl;
            return create_invalid_occupancy_map();
        }
        
        // Extend map bounds
        min_x -= map_extension;
        max_x += map_extension;
        min_y -= map_extension;
        max_y += map_extension;
        
        // Calculate map dimensions
        int width = static_cast<int>(ceil((max_x - min_x) / resolution));
        int height = static_cast<int>(ceil((max_y - min_y) / resolution));
        
        if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
            cerr << "Invalid map dimensions: " << width << "x" << height << endl;
            return create_invalid_occupancy_map();
        }
        
        // Initialize occupancy map with unknown values (127)
        unsigned char* map_data = new unsigned char[width * height];
        memset(map_data, 127, width * height); // Unknown = 127
        
        // Mark observed points as free (255)
        for (const auto& point : filtered_points) {
            int px = static_cast<int>((point.x() - min_x) / resolution);
            int py = static_cast<int>((point.y() - min_y) / resolution);
            
            // Ensure within bounds
            if (px >= 0 && px < width && py >= 0 && py < height) {
                map_data[py * width + px] = 255; // Free space
            }
        }
        
        // Apply robot radius inflation (mark areas around obstacles)
        if (robot_radius > 0) {
            int inflation_pixels = static_cast<int>(ceil(robot_radius / resolution));
            unsigned char* temp_map = new unsigned char[width * height];
            memcpy(temp_map, map_data, width * height);
            
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (temp_map[y * width + x] == 0) { // If occupied
                        // Inflate around this pixel
                        for (int dy = -inflation_pixels; dy <= inflation_pixels; dy++) {
                            for (int dx = -inflation_pixels; dx <= inflation_pixels; dx++) {
                                int nx = x + dx;
                                int ny = y + dy;
                                
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                    float dist = sqrt(dx*dx + dy*dy) * resolution;
                                    if (dist <= robot_radius) {
                                        map_data[ny * width + nx] = 0; // Occupied
                                    }
                                }
                            }
                        }
                    }
                }
            }
            delete[] temp_map;
        }
        
        // Simple obstacle inference: areas between free spaces that weren't observed
        // This is a basic implementation - could be improved with ray tracing
        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                if (map_data[y * width + x] == 127) { // Unknown
                    // Check if surrounded by free space - might be obstacle
                    int free_neighbors = 0;
                    int total_neighbors = 0;
                    
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                total_neighbors++;
                                if (map_data[ny * width + nx] == 255) {
                                    free_neighbors++;
                                }
                            }
                        }
                    }
                    
                    // If mostly surrounded by free space, likely an obstacle
                    if (total_neighbors > 0 && free_neighbors >= total_neighbors * 0.6) {
                        map_data[y * width + x] = 0; // Mark as occupied
                    }
                }
            }
        }
        
        // Create result structure
        OccupancyMap2D result;
        result.data = map_data;
        result.width = width;
        result.height = height;
        result.resolution = resolution;
        result.origin_x = min_x;
        result.origin_y = min_y;
        result.valid = 1;
        
        cout << "Generated occupancy map: " << width << "x" << height 
             << " pixels, resolution: " << resolution << " m/pixel" << endl;
        
        return result;
        
    } catch (const exception& e) {
        cerr << "Error generating occupancy map: " << e.what() << endl;
        return create_invalid_occupancy_map();
    }
}

void orb_slam3_free_occupancy_map(OccupancyMap2D* occupancy_map) {
    if (occupancy_map && occupancy_map->data) {
        delete[] occupancy_map->data;
        occupancy_map->data = nullptr;
        occupancy_map->valid = 0;
    }
}

} // extern "C"
