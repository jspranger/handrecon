#include "sensordevice.h"

/* Constructor and Destructor *************************************************************************/
SensorDevice::SensorDevice(QObject *parent, QString device_path)
    : QThread(parent)
{
    this->m_devicePath = device_path;
    this->m_isDepth_running = false;
    this->m_isRGB_running = false;
    this->m_isIR_running = false;
    this->m_isActive = false;
}

SensorDevice::~SensorDevice()
{
    ;
}
/******************************************************************************************************/

bool SensorDevice::initSensor(int depthXres, int depthYres, int depthFPS, int rgbXres, int rgbYres, int rgbFPS, int irXres, int irYres, int irFPS)
{
    bool status = false;

    if (!this->m_isActive)
        if ((!openni::OpenNI::initialize()) && (!nite::NiTE::initialize()))
        {
            this->m_device.open(m_devicePath.isEmpty()?openni::ANY_DEVICE:(const char *)this->m_devicePath.toUtf8());
            if (this->m_device.isValid())
            {
                this->m_videoStreamDepth = new openni::VideoStream*();
                this->m_videoStreamColor = new openni::VideoStream*();
                this->m_videoStreamIR = new openni::VideoStream*();
                this->m_videoStreamDepth[0] = &this->m_videoDepth;
                this->m_videoStreamColor[0] = &this->m_videoColor;
                this->m_videoStreamIR[0] = &this->m_videoIR;

                if (!this->m_device.setDepthColorSyncEnabled(true))
                {
                    // Initialize Depth Stream _
                    if (this->m_device.hasSensor(openni::SENSOR_DEPTH) && !this->m_videoDepth.isValid())
                    {
                        this->m_videoDepth.create(this->m_device, openni::SENSOR_DEPTH);
                        if (this->m_videoDepth.isValid())
                        {
                            openni::VideoMode videoDepthMode;
                            videoDepthMode = this->m_videoDepth.getVideoMode();
                            videoDepthMode.setFps(depthFPS);
                            videoDepthMode.setResolution(depthXres, depthYres);
                            this->m_videoDepth.setVideoMode(videoDepthMode);
                            this->m_videoDepth.setMirroringEnabled(false);

                            this->m_isDepth_running = !this->m_videoDepth.start();
                        }
                    }

                    // Initialize Color Stream _
                    if (this->m_device.hasSensor(openni::SENSOR_COLOR) && !this->m_videoColor.isValid())
                    {
                        this->m_videoColor.create(this->m_device, openni::SENSOR_COLOR);
                        if (this->m_videoColor.isValid())
                        {
                            openni::VideoMode videoColorMode;
                            videoColorMode = this->m_videoColor.getVideoMode();
                            videoColorMode.setFps(rgbFPS);
                            videoColorMode.setResolution(rgbXres, rgbYres);
                            this->m_videoColor.setVideoMode(videoColorMode);
                            this->m_videoColor.setMirroringEnabled(false);

                            this->m_isRGB_running = !this->m_videoColor.start();
                        }
                    }

                    // Initialize IR Stream _
                    if (this->m_device.hasSensor(openni::SENSOR_IR) && !this->m_videoIR.isValid())
                    {
                        this->m_videoIR.create(this->m_device, openni::SENSOR_IR);
                        if (this->m_videoIR.isValid())
                        {
                            openni::VideoMode videoIRMode;
                            videoIRMode = this->m_videoIR.getVideoMode();
                            videoIRMode.setFps(irFPS);
                            videoIRMode.setResolution(irXres, irYres);
                            this->m_videoIR.setVideoMode(videoIRMode);
                            this->m_videoIR.setMirroringEnabled(false);

                            this->m_isIR_running = !this->m_videoIR.start();
                        }
                    }

                    // Hand Tracker _
                    if (this->m_isDepth_running && !this->m_handTracker.isValid())
                    {
                        this->m_handTracker.create(&this->m_device);
                        if (this->m_handTracker.isValid())
                        {
                            this->m_handTracker.setSmoothingFactor(0.1f);
                            this->m_handTracker.startGestureDetection(nite::GESTURE_HAND_RAISE);
                        }
                    }

                    // User Tracker _
                    if (this->m_isDepth_running && !this->m_userTracker.isValid())
                        this->m_userTracker.create(&this->m_device);
                }
            }
            status = this->m_isDepth_running || this->m_isRGB_running || this->m_isIR_running;
            if (status)
            {
                this->m_isActive = true;
                this->start();
            }
        }

    return status;
}

bool SensorDevice::stopSensor(void)
{
    bool status = this->m_isActive;

    this->m_isActive = false;

    return status;
}

/******************************************************************************************************/

/* Auxiliary Functions ********************************************************************************/
RealWorldJoint SensorDevice::convertSkeletonJointToRealWorld(const nite::SkeletonJoint joint)
{
    RealWorldJoint real_world_joint =
    {
        QQuaternion(joint.getOrientation().w, joint.getOrientation().x, joint.getOrientation().y, joint.getOrientation().z),
        QVector3D(joint.getPosition().x, joint.getPosition().y, joint.getPosition().z)
    };
    return real_world_joint;
}
ProjectiveJoint SensorDevice::convertSkeletonJointToProjective(const nite::SkeletonJoint joint)
{
    float projective_coordinates[2];
    m_userTracker.convertJointCoordinatesToDepth(joint.getPosition().x,
                                                                                  joint.getPosition().y,
                                                                                  joint.getPosition().z,
                                                                                  &projective_coordinates[0],
                                                                                  &projective_coordinates[1]);
    ProjectiveJoint projective_joint =
    {
        QVector2D(projective_coordinates[0], projective_coordinates[1])
    };
    return projective_joint;
}
/******************************************************************************************************/

/* Acquisition Thread **********************************************************************************/
void SensorDevice::run()
{
    int cameraIndex;
    nite::HandTrackerFrameRef handTrackerFrame;
    nite::UserTrackerFrameRef userTrackerFrame;
    openni::VideoFrameRef videoDepthFrame, videoColorFrame, videoInfraredFrame;
    while (this->m_isActive)
    {
        // VIDEO STREAM _______________________________________________________________________________
        if (this->m_isDepth_running)
            if (!openni::OpenNI::waitForAnyStream(this->m_videoStreamDepth, 1, &cameraIndex))
                if (!this->m_videoDepth.readFrame(&videoDepthFrame))
                {
                    cv::Mat depthMat(videoDepthFrame.getHeight(), videoDepthFrame.getWidth(), CV_16UC1, (unsigned char *)videoDepthFrame.getData());
                    emit image_data(IMAGE::DEPTH, depthMat);
                }

        if (this->m_isRGB_running)
            if (!openni::OpenNI::waitForAnyStream(this->m_videoStreamColor, 1, &cameraIndex))
                if (!this->m_videoColor.readFrame(&videoColorFrame))
                {
                    cv::Mat colorMat(videoColorFrame.getHeight(), videoColorFrame.getWidth(), CV_8UC3, (unsigned char *)videoColorFrame.getData());
                    cv::cvtColor(colorMat, colorMat, CV_BGR2RGB);
                    emit image_data(IMAGE::COLOR, colorMat);
                }

        if (this->m_isIR_running)
            if (!openni::OpenNI::waitForAnyStream(this->m_videoStreamIR, 1, &cameraIndex))
                if (!this->m_videoIR.readFrame(&videoInfraredFrame))
                {
                    cv::Mat irMat(videoInfraredFrame.getHeight(), videoInfraredFrame.getWidth(), CV_16UC1, (unsigned char *)videoInfraredFrame.getData());
                    emit image_data(IMAGE::IR, irMat);
                }
        // _____________________________________________________________________________________________

        // HAND TRACKING _____________________________________________________________________________
        if (this->m_handTracker.isValid())
            if (!this->m_handTracker.readFrame(&handTrackerFrame))
            {
                const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
                for (int i = 0; i < gestures.getSize(); i++)
                    if (gestures[i].isComplete())
                    {
                        nite::HandId newId;
                        this->m_handTracker.startHandTracking(gestures[i].getCurrentPosition(), &newId);
                    }
                const nite::Array<nite::HandData>& hands = handTrackerFrame.getHands();
                for (int i = 0; i < hands.getSize(); ++i)
                {
                    const nite::HandData& hand = hands[i];
                    if (hand.isNew())
                        emit hand_init(hand.getId());
                    else
                    if (hand.isTracking())
                    {
                        float x, y;
                        this->m_handTracker.convertHandCoordinatesToDepth(hand.getPosition().x, hand.getPosition().y, hand.getPosition().z, &x, &y);
                        emit hand_coordinate(hand.getId(), QVector3D(x, y, hand.getPosition().z));
                    }
                    else
                    if (hand.isLost())
                        emit hand_stop(hand.getId());
                }
            }
        // _____________________________________________________________________________________________

        // USER TRACKING _____________________________________________________________________________
        if (this->m_userTracker.isValid())
            if (!this->m_userTracker.readFrame(&userTrackerFrame))
            {
                cv::Mat userPixels(userTrackerFrame.getDepthFrame().getHeight(),
                                              userTrackerFrame.getDepthFrame().getWidth(),
                                              CV_16UC1,
                                              (unsigned char *)userTrackerFrame.getUserMap().getPixels());

                const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
                for (int i = 0; i < users.getSize(); ++i)
                {
                    const nite::UserData& user = users[i];
                    if (user.isNew())
                    {
                        this->m_userTracker.startSkeletonTracking(user.getId());
                        this->m_userTracker.startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
                        this->m_userTracker.startPoseDetection(user.getId(), nite::POSE_PSI);
                        emit user_init(user.getId());
                    }
                    else
                    if (!user.isLost())
                    {
                        // User Map _
                        UserMap map;
                        map.real_world_center_of_mass = QVector3D(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z);
                        map.real_world_bounding_box.maximum = QVector3D(user.getBoundingBox().max.x, user.getBoundingBox().max.y, user.getBoundingBox().max.z);
                        map.real_world_bounding_box.minimum = QVector3D(user.getBoundingBox().min.x, user.getBoundingBox().min.y, user.getBoundingBox().min.z);
                        float com[2], bb_max[2], bb_min[2];
                        this->m_userTracker.convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &com[0], &com[1]);
                        this->m_userTracker.convertJointCoordinatesToDepth(user.getBoundingBox().max.x, user.getBoundingBox().max.y, user.getBoundingBox().max.z, &bb_max[0], &bb_max[1]);
                        this->m_userTracker.convertJointCoordinatesToDepth(user.getBoundingBox().min.x, user.getBoundingBox().min.y, user.getBoundingBox().min.z, &bb_min[0], &bb_min[1]);
                        map.projective_center_of_mass = QVector2D(com[0], com[1]);
                        map.projective_bounding_box.maximum = QVector2D(bb_max[0], bb_max[1]);
                        map.projective_bounding_box.minimum = QVector2D(bb_min[0], bb_min[1]);
                        map.pixelmap = (userPixels == user.getId());
                        emit user_map(user.getId(), map);

                        // User Skeleton _
                        if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
                        {
                            // VER VALORES DE CONFIDENCE DOS PONTOS!!! > 0.5!!!!!
                            const nite::SkeletonJoint head_joint = user.getSkeleton().getJoint(nite::JOINT_HEAD);
                            const nite::SkeletonJoint neck_joint = user.getSkeleton().getJoint(nite::JOINT_NECK);
                            const nite::SkeletonJoint left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
                            const nite::SkeletonJoint left_elbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
                            const nite::SkeletonJoint left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
                            const nite::SkeletonJoint right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
                            const nite::SkeletonJoint right_elbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
                            const nite::SkeletonJoint right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
                            const nite::SkeletonJoint torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
                            const nite::SkeletonJoint left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
                            const nite::SkeletonJoint left_knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
                            const nite::SkeletonJoint left_foot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
                            const nite::SkeletonJoint right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
                            const nite::SkeletonJoint right_knee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
                            const nite::SkeletonJoint right_foot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);

                            ProjectiveSkeleton projective_skeleton;
                            projective_skeleton.head = convertSkeletonJointToProjective(head_joint);
                            projective_skeleton.neck = convertSkeletonJointToProjective(neck_joint);
                            projective_skeleton.left_shoulder = convertSkeletonJointToProjective(left_shoulder);
                            projective_skeleton.left_elbow = convertSkeletonJointToProjective(left_elbow);
                            projective_skeleton.left_hand = convertSkeletonJointToProjective(left_hand);
                            projective_skeleton.right_shoulder = convertSkeletonJointToProjective(right_shoulder);
                            projective_skeleton.right_elbow = convertSkeletonJointToProjective(right_elbow);
                            projective_skeleton.right_hand = convertSkeletonJointToProjective(right_hand);
                            projective_skeleton.torso = convertSkeletonJointToProjective(torso);
                            projective_skeleton.left_hip = convertSkeletonJointToProjective(left_hip);
                            projective_skeleton.left_knee = convertSkeletonJointToProjective(left_knee);
                            projective_skeleton.left_foot = convertSkeletonJointToProjective(left_foot);
                            projective_skeleton.right_hip = convertSkeletonJointToProjective(right_hip);
                            projective_skeleton.right_knee = convertSkeletonJointToProjective(right_knee);
                            projective_skeleton.right_foot = convertSkeletonJointToProjective(right_foot);
                            emit user_projective_skeleton(user.getId(), projective_skeleton);

                            RealWorldSkeleton real_world_skeleton;
                            real_world_skeleton.head = convertSkeletonJointToRealWorld(head_joint);
                            real_world_skeleton.neck = convertSkeletonJointToRealWorld(neck_joint);
                            real_world_skeleton.left_shoulder = convertSkeletonJointToRealWorld(left_shoulder);
                            real_world_skeleton.left_elbow = convertSkeletonJointToRealWorld(left_elbow);
                            real_world_skeleton.left_hand = convertSkeletonJointToRealWorld(left_hand);
                            real_world_skeleton.right_shoulder = convertSkeletonJointToRealWorld(right_shoulder);
                            real_world_skeleton.right_elbow = convertSkeletonJointToRealWorld(right_elbow);
                            real_world_skeleton.right_hand = convertSkeletonJointToRealWorld(right_hand);
                            real_world_skeleton.torso = convertSkeletonJointToRealWorld(torso);
                            real_world_skeleton.left_hip = convertSkeletonJointToRealWorld(left_hip);
                            real_world_skeleton.left_knee = convertSkeletonJointToRealWorld(left_knee);
                            real_world_skeleton.left_foot = convertSkeletonJointToRealWorld(left_foot);
                            real_world_skeleton.right_hip = convertSkeletonJointToRealWorld(right_hip);
                            real_world_skeleton.right_knee = convertSkeletonJointToRealWorld(right_knee);
                            real_world_skeleton.right_foot = convertSkeletonJointToRealWorld(right_foot);
                            emit user_real_world_skeleton(user.getId(), real_world_skeleton);
                        }

                        // User Pose _
                        const nite::PoseData& pose_ch = user.getPose(nite::POSE_CROSSED_HANDS);
                        if (pose_ch.isEntered())
                            emit user_pose(user.getId(), POSE::ARMS_CROSSED);

                        const nite::PoseData& pose_ps = user.getPose(nite::POSE_PSI);
                        if (pose_ps.isEntered())
                            emit user_pose(user.getId(), POSE::PSI);
                    }
                    else
                    if (user.isLost())
                        emit user_stop(user.getId());
                }
            // _____________________________________________________________________________________________
        }
    }

    // Shutdown
    if (this->m_userTracker.isValid()) this->m_userTracker.destroy();
    if (this->m_handTracker.isValid()) this->m_handTracker.destroy();
    if (this->m_isIR_running) { this->m_videoIR.stop(); this->m_videoIR.destroy(); }
    if (this->m_isRGB_running) { this->m_videoColor.stop(); this->m_videoColor.destroy(); }
    if (this->m_isDepth_running) { this->m_videoDepth.stop(); this->m_videoDepth.destroy(); }
    delete this->m_videoStreamIR;
    delete this->m_videoStreamColor;
    delete this->m_videoStreamDepth;
    this->m_device.close();
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
}
/******************************************************************************************************/
