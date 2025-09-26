#include "orb_slam3_wrapper.h"
#include "../include/System.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

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
        
        ORBSLAMSystemImpl* impl = new ORBSLAMSystemImpl(vocab_str, settings_str, sensor, viewer);
        return reinterpret_cast<ORBSLAMSystemHandle*>(impl);
    } catch (const exception& e) {
        cerr << "Error creating ORB-SLAM3 system: " << e.what() << endl;
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

} // extern "C"
