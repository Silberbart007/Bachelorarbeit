#pragma once
#include <cstdint>

#pragma pack(push, 1)

struct LaserScanData {
    double stamp;
    char frame_id[64];
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[541];
    float intensities[541];
};

struct TFData {
    double stamp;
    char frame_id[64];
    char child_frame_id[64];
    float trans[3];
    float rot[4];
};

struct CombinedData {
    LaserScanData laser_data;
    TFData tf_data;
};

struct VelocityData {
    double linear_x;
    double linear_y;
    double angular_z;
};

#pragma pack(pop)
