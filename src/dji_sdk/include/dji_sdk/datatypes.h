#ifndef DATATYPES_H
#define DATATYPES_H

//typedef struct box
//{
//    float x, y, w, h;
//} box;

typedef struct Wp
{
    std::string id           = "";
    double latitude		= 0;
    double longitude	= 0;
    float  altitude 	= 0;
    double phi 			= 0;
    double lamda 		= 0;
    double HFSL         = 0;
    bool   inspected    = false;
} Wp;

//typedef struct FlightCtrlData
//{
//    DJI::OSDK::float32_t x = 0.00;
//    DJI::OSDK::float32_t y = 0.00;
//    DJI::OSDK::float32_t z = 0.00;
//    DJI::OSDK::float32_t yaw;
//} FlightCtrlData;

typedef struct ImageInfo
{
    float initWidth;
    float initHeight;
    float sizeOfSqrtImg;
}ImageInfo;

//typedef struct TargetInfo
//{
//    long long int   frameNum;
//    box             location;
//    int             id;

//} TargetInfo;

enum GNSSModes
{
    GNSS_MODE_GPS,
    GNSS_MODE_GPS_FUSED,
    GNSS_MODE_RTK
};

enum DetectionMode
{
    POLE,
    TBAR
};

enum Instruction
{
    CONTINUEMISSION,
    RETURNHOME,
    LASTPOINT,
    SHUTDOWN
};

enum CameraState
{
    DEFAULT,
    DETECTOR,
    RECORDPOWERLINES,
    PHOTO
};

typedef struct DroneVelocities
{
    double x	= 0;
    double y	= 0;
    double z	= 0;
} DroneVelocities;


//#define EARTH_RADIUS (double)6378137.0  // also defined in dji_mission_type.hpp line 40
#define RAD_2_DEG 57.29577951
#define DEG_2_RAD 0.01745329252

#define M_PI 3.14159265358979323846

#endif // DATATYPES_H
