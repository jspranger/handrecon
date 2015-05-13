#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#include <QThread>

#include <QHash>
#include <QImage>

#include "../ogrewidget.h"
#include "hw/sensordevice.h"
#include "learn/handhypothesis.h"

#include <QDebug>

class User
{
public:
    User(cv::Scalar color = cv::Scalar(0, 0, 0));

    void setUserMap(UserMap);
    void setProjectiveSkeleton(ProjectiveSkeleton);
    void setRealWorldSkeleton(RealWorldSkeleton);

    cv::Scalar getUserColor(void);
    UserMap getUserMap(void);
    ProjectiveSkeleton getProjectiveSkeleton(void);
    RealWorldSkeleton getRealWorldSkeleton(void);

private:
    cv::Scalar userColor;
    UserMap currentMap;
    ProjectiveSkeleton currentProjectiveSkeleton;
    RealWorldSkeleton currentRealWorldSkeleton;
};

class Recognizer : public QThread
{
    Q_OBJECT
public:
    explicit Recognizer(QObject *parent = 0);

    void initRecognizer(OgreWidget *);

private:
    SensorDevice m_sensorDevice;
    HandHypothesis m_handHypothesis;

    QImage m_blankImage;
    cv::Mat m_originalImageDepth,
                 m_originalImageRGB,
                 m_originalImageIR,
                 m_imageHandSegm,
                 m_imageUserSkel,
                 m_imageHandSegmCloseUp;
    QMutex m_mutexDepth,
                 m_mutexColor,
                 m_mutexInfrared,
                 m_mutexHandSegm,
                 m_mutexUserSkel,
                 m_mutexHandCloseUp;
    QTimer m_timerDepth,
                 m_timerColor,
                 m_timerInfrared,
                 m_timerHandSegm,
                 m_timerUserSkel,
                 m_timerHandCloseUp;

    cv::Mat m_structuringElement;

    short m_lockedHand;
    QVector3D m_lockedCoordinate;
    float m_handSize, m_handDepth;

    bool m_isActive;
    QHash<short, User> users;

public slots:
    void run(void);

private slots:
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


    void renderDepth(void);
    void renderColor(void);
    void renderInfrared(void);
    void renderHandSegmentation(void);
    void renderUserSkeleton(void);
    void renderHandSegmentationCloseUp(void);

signals:
    void updateOriginalDepth(QImage);
    void updateOriginalRGB(QImage);
    void updateOriginalIR(QImage);
    void updateHandSegmented(QImage);
    void updateUserSkeleton(QImage);
    void updateHandSegmentedCloseUp(QImage);



    /////////////////// TEMP
    void updateRenderedHandSegmentationTEMP(QImage);
    void updateRenderedHandDepthTEMP(QImage);
};

#endif // RECOGNIZER_H
