#ifndef SENSORDEVICE_H
#define SENSORDEVICE_H

#include <QObject>
#include <QThread>

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2/opencv.hpp>

#include <QQuaternion>
#include <QVector3D>
#include <QVector2D>

#include <QDebug>

#define SENSOR_WIDTH 640
#define SENSOR_HEIGHT 480

namespace IMAGE { enum Mode { DEPTH = 0, COLOR = 1, IR = 2, }; }
Q_DECLARE_METATYPE(IMAGE::Mode)

namespace POSE { enum Type { PSI = 0, ARMS_CROSSED = 1, }; }
Q_DECLARE_METATYPE(POSE::Type)

typedef struct { QVector3D minimum; QVector3D maximum; } RealWorldBoundingBox;
typedef struct { QVector2D minimum; QVector2D maximum; } ProjectiveBoundingBox;
typedef struct
{
    cv::Mat pixelmap;
    QVector2D projective_center_of_mass;
    QVector3D real_world_center_of_mass;
    ProjectiveBoundingBox projective_bounding_box;
    RealWorldBoundingBox real_world_bounding_box;
} UserMap;
Q_DECLARE_METATYPE(UserMap)

typedef struct { QVector2D position; } ProjectiveJoint;
typedef struct
{
    ProjectiveJoint head; ProjectiveJoint neck;
    ProjectiveJoint left_shoulder; ProjectiveJoint left_elbow; ProjectiveJoint left_hand;
    ProjectiveJoint right_shoulder; ProjectiveJoint right_elbow; ProjectiveJoint right_hand;
    ProjectiveJoint torso;
    ProjectiveJoint left_hip; ProjectiveJoint left_knee; ProjectiveJoint left_foot;
    ProjectiveJoint right_hip; ProjectiveJoint right_knee; ProjectiveJoint right_foot;
} ProjectiveSkeleton;
Q_DECLARE_METATYPE(ProjectiveSkeleton)

typedef struct { QQuaternion orientation; QVector3D position; } RealWorldJoint;
typedef struct
{
    RealWorldJoint head; RealWorldJoint neck;
    RealWorldJoint left_shoulder; RealWorldJoint left_elbow; RealWorldJoint left_hand;
    RealWorldJoint right_shoulder; RealWorldJoint right_elbow; RealWorldJoint right_hand;
    RealWorldJoint torso;
    RealWorldJoint left_hip; RealWorldJoint left_knee; RealWorldJoint left_foot;
    RealWorldJoint right_hip; RealWorldJoint right_knee; RealWorldJoint right_foot;
} RealWorldSkeleton;
Q_DECLARE_METATYPE(RealWorldSkeleton)

Q_DECLARE_METATYPE(cv::Mat)

class SensorDevice : public QThread
{
    Q_OBJECT
public:
    explicit SensorDevice(QObject *parent = 0, QString devicePath = QString());
    ~SensorDevice(void);

    bool initSensor(int depthXres = 640,
                    int depthYres = 480,
                    int depthFPS = 30,
                    int rgbXres = 640,
                    int rgbYres = 480,
                    int rgbFPS = 30,
                    int irXres = 640,
                    int irYres = 480,
                    int irFPS = 30);
    bool stopSensor(void);

private:
    QString m_devicePath;
    bool m_isActive,
            m_isDepth_running,
            m_isRGB_running,
            m_isIR_running;

    openni::Device m_device;
    openni::VideoStream m_videoDepth, m_videoColor, m_videoIR;
    openni::VideoStream** m_videoStreamDepth, **m_videoStreamColor, **m_videoStreamIR;
    nite::HandTracker m_handTracker;
    nite::UserTracker m_userTracker;

    RealWorldJoint convertSkeletonJointToRealWorld(const nite::SkeletonJoint);
    ProjectiveJoint convertSkeletonJointToProjective(const nite::SkeletonJoint);

public slots:
    void run(void);

signals:
    void hand_init(short);
    void hand_coordinate(short, QVector3D);
    void hand_stop(short);

    void user_init(short);
    void user_map(short, UserMap);
    void user_projective_skeleton(short, ProjectiveSkeleton);
    void user_real_world_skeleton(short, RealWorldSkeleton);
    void user_pose(short, POSE::Type);
    void user_stop(short);

    void image_data(IMAGE::Mode, cv::Mat);
};

#endif // SENSORDEVICE_H
