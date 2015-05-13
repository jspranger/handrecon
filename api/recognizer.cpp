#include "recognizer.h"

cv::Scalar getColor(short i)
{
    cv::Scalar result = cv::Scalar(0, 0, 0);

    result = (i == 0)?cv::Scalar(0,0,255):result;
    result = (i == 1)?cv::Scalar(0,255,0):result;
    result = (i == 2)?cv::Scalar(255,0,0):result;
    result = (i == 3)?cv::Scalar(255,0,255):result;
    result = (i == 4)?cv::Scalar(0,255,255):result;
    result = (i == 5)?cv::Scalar(255,255,0):result;
    result = (i == 6)?cv::Scalar(0, 127, 255):result;
    result = (i == 7)?cv::Scalar(32, 64, 127):result;
    result = (i == 8)?cv::Scalar(127, 127, 127):result;
    result = (i == 9)?cv::Scalar(255,255,255):result;

    return result;
};

Recognizer::Recognizer(QObject *parent) :
    QThread(parent)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<UserMap>("UserMap");
    qRegisterMetaType<ProjectiveSkeleton>("ProjectiveSkeleton");
    qRegisterMetaType<RealWorldSkeleton>("RealWorldSkeleton");
    qRegisterMetaType<POSE::Type>("POSE::Type");
    qRegisterMetaType<IMAGE::Mode>("IMAGE::Mode");

    this->m_blankImage = QImage(SENSOR_WIDTH, SENSOR_HEIGHT, QImage::Format_RGB888);
    this->m_blankImage.fill(Qt::black);

    this->m_lockedHand = -1;
    this->m_handSize = 150.0f;
    this->m_handDepth = 200.0f;

    connect(&this->m_sensorDevice, SIGNAL(hand_init(short)), this, SLOT(hand_init(short)));
    connect(&this->m_sensorDevice, SIGNAL(hand_coordinate(short,QVector3D)), this, SLOT(hand_coordinate(short,QVector3D)));
    connect(&this->m_sensorDevice, SIGNAL(hand_stop(short)), this, SLOT(hand_stop(short)));
    connect(&this->m_sensorDevice, SIGNAL(user_init(short)), this, SLOT(user_init(short)));
    connect(&this->m_sensorDevice, SIGNAL(user_map(short,UserMap)), this, SLOT(user_map(short,UserMap)));
    connect(&this->m_sensorDevice, SIGNAL(user_projective_skeleton(short,ProjectiveSkeleton)), this, SLOT(user_projective_skeleton(short,ProjectiveSkeleton)));
    connect(&this->m_sensorDevice, SIGNAL(user_real_world_skeleton(short,RealWorldSkeleton)), this, SLOT(user_real_world_skeleton(short,RealWorldSkeleton)));
    connect(&this->m_sensorDevice, SIGNAL(user_pose(short,POSE::Type)), this, SLOT(user_pose(short,POSE::Type)));
    connect(&this->m_sensorDevice, SIGNAL(user_stop(short)), this, SLOT(user_stop(short)));
    connect(&this->m_sensorDevice, SIGNAL(image_data(IMAGE::Mode,cv::Mat)), this, SLOT(image_data(IMAGE::Mode,cv::Mat)));

    connect(&this->m_timerDepth, SIGNAL(timeout()), this, SLOT(renderDepth()));
    connect(&this->m_timerColor, SIGNAL(timeout()), this, SLOT(renderColor()));
    connect(&this->m_timerInfrared, SIGNAL(timeout()), this, SLOT(renderInfrared()));
    connect(&this->m_timerHandSegm, SIGNAL(timeout()), this, SLOT(renderHandSegmentation()));
    connect(&this->m_timerUserSkel, SIGNAL(timeout()), this, SLOT(renderUserSkeleton()));
    connect(&this->m_timerHandCloseUp, SIGNAL(timeout()), this, SLOT(renderHandSegmentationCloseUp()));

    this->m_isActive = false;
}

void Recognizer::initRecognizer(OgreWidget *ogreWidget)
{
    connect(ogreWidget, SIGNAL(ogreWidgetReady(Ogre::Root *, Ogre::RenderWindow *, Ogre::Camera *, Ogre::Viewport *, Ogre::SceneManager *)),
                   &this->m_handHypothesis, SLOT(renderReady(Ogre::Root *, Ogre::RenderWindow *, Ogre::Camera *, Ogre::Viewport *, Ogre::SceneManager *)), Qt::DirectConnection);

    this->m_timerDepth.start(33);
    this->m_timerColor.start(33);
    this->m_timerInfrared.start(33);
    this->m_timerHandSegm.start(33);
    this->m_timerUserSkel.start(33);
    this->m_timerHandCloseUp.start(33);
    ogreWidget->initRender();

    this->m_sensorDevice.initSensor();

    this->m_isActive = true;
    this->start();
}

void Recognizer::hand_init(short handId) { if (this->m_lockedHand == -1) this->m_lockedHand = handId; }
void Recognizer::hand_coordinate(short handId, QVector3D coordinate)
{
    if (this->m_lockedHand == handId)
    {
        this->m_lockedCoordinate = coordinate;

        cv::Rect m_roi;
        int roi_offset = (int)((m_handSize / (double)(this->m_lockedCoordinate.z() / 1000.0)) / 2.0);
        if (roi_offset % 2) roi_offset++;
        if (roi_offset < 40) roi_offset = 40;
        m_roi.x = this->m_lockedCoordinate.x() - roi_offset;
        m_roi.y = this->m_lockedCoordinate.y() - roi_offset;
        m_roi.width = m_roi.height = roi_offset * 2;

        double min, max;
        cv::minMaxIdx(this->m_originalImageDepth, &min, &max);

        if ((m_roi.x >= 0) && (m_roi.y >= 0) &&
            (m_roi.width > 0) && (m_roi.height > 0) &&
            ((m_roi.x + m_roi.width) <= this->m_originalImageDepth.cols - 1) &&
            ((m_roi.y + m_roi.height) <= this->m_originalImageDepth.rows - 1) &&
            ((this->m_lockedCoordinate.z() - this->m_handDepth >= 0) && (this->m_lockedCoordinate.z() + this->m_handDepth <= max)))
        {
            cv::Mat handSegm = cv::Mat(this->m_originalImageDepth, m_roi).clone();

            handSegm = (handSegm >= this->m_lockedCoordinate.z() - this->m_handDepth) & (handSegm <= this->m_lockedCoordinate.z() + this->m_handDepth);
            cv::GaussianBlur(handSegm, handSegm, cv::Size(3,3), 0);
            cv::erode(handSegm, handSegm, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7)));

            handSegm.copyTo(this->m_imageHandSegmCloseUp);

            cv::Mat segmentedDepth = cv::Mat::zeros(this->m_originalImageDepth.rows, this->m_originalImageDepth.cols, CV_16UC1);
            cv::Mat m_src = this->m_originalImageDepth(m_roi);
            m_src.copyTo(segmentedDepth(m_roi), handSegm);

            segmentedDepth.copyTo(this->m_imageHandSegm);

            cv::imshow("TEMP",m_src);

            this->m_handHypothesis.startOptimization(handSegm, m_src);
            qDebug() << "recognizer_startOptimization()";
        }
    }
}
void Recognizer::hand_stop(short handId)
{
    if (this->m_lockedHand == handId)
    {
        qDebug() << "Should stop!";
        this->m_handHypothesis.stopOptimization();
        this->m_lockedHand = -1;
    }
}

void Recognizer::user_init(short userId)
{
    if (!this->users.contains(userId))
    {
        User user(getColor(userId % 9));
        this->users.insert(userId, user);
    }
}

void Recognizer::user_map(short userId, UserMap userMap)
{
    if (this->users.contains(userId))
        this->users[userId].setUserMap(userMap);
}

void Recognizer::user_projective_skeleton(short userId, ProjectiveSkeleton userProjectiveSkeleton)
{
    if (this->users.contains(userId))
        this->users[userId].setProjectiveSkeleton(userProjectiveSkeleton);
}

void Recognizer::user_real_world_skeleton(short userId, RealWorldSkeleton userRealWorldSkeleton)
{
    if (this->users.contains(userId))
        this->users[userId].setRealWorldSkeleton(userRealWorldSkeleton);
}

void Recognizer::user_pose(short userId, POSE::Type userPose)
{
    // Not really useful for now
}

void Recognizer::user_stop(short userId)
{
    if (this->users.contains(userId))
        this->users.remove(userId);
}

void Recognizer::image_data(IMAGE::Mode imageMode, cv::Mat image)
{
    switch (imageMode)
    {
        case IMAGE::DEPTH:
            this->m_originalImageDepth = image;
        break;
        case IMAGE::COLOR:
            this->m_originalImageRGB = image;
            break;
        case IMAGE::IR:
            this->m_originalImageIR = image;
            break;
        default: break;
    }
}

void Recognizer::renderDepth(void)
{
    this->m_mutexDepth.lock();

    if (!this->m_originalImageDepth.empty())
    {
        double min, max;
        cv::minMaxIdx(this->m_originalImageDepth, &min, &max);
        cv::Mat viewableDepth;
        cv::convertScaleAbs(this->m_originalImageDepth, viewableDepth, 255/max);

        QImage m_image(viewableDepth.cols, viewableDepth.rows, QImage::Format_RGB888);
        unsigned char *m_data = (unsigned char *)viewableDepth.data;
        for (int i = 0; i < m_image.height(); i++)
        {
            uchar *m_pixel = (uchar *)m_image.scanLine(i);
            for (int j = 0; j < (m_image.width() * 3); j += 3)
            {
                m_pixel[j] = *m_data;
                m_pixel[j+1] = *m_data;
                m_pixel[j+2] = *m_data;

                m_data++;
            }
        }

        QImage result(m_image);
        result.detach();

        emit updateOriginalDepth(result);
    }

    this->m_mutexDepth.unlock();
}

void Recognizer::renderColor(void)
{
    this->m_mutexColor.lock();

//    if (!this->m_originalImageRGB.empty())
//    {
//        QImage m_image((unsigned char *)this->m_originalImageRGB.data,
//                       this->m_originalImageRGB.cols,
//                       this->m_originalImageRGB.rows,
//                       QImage::Format_RGB888);
//        QImage result(m_image);
//        result.detach();

//        emit updateOriginalRGB(result);
//    }

    //////////////////// TEST
    if (!this->m_originalImageRGB.empty())
    {
        double min, max;
        cv::minMaxIdx(this->m_originalImageRGB, &min, &max);
        cv::Mat viewableDepth;
        cv::convertScaleAbs(this->m_originalImageRGB, viewableDepth, 255/max);

        QImage m_image(viewableDepth.cols, viewableDepth.rows, QImage::Format_RGB888);
        unsigned char *m_data = (unsigned char *)viewableDepth.data;
        for (int i = 0; i < m_image.height(); i++)
        {
            uchar *m_pixel = (uchar *)m_image.scanLine(i);
            for (int j = 0; j < (m_image.width() * 3); j += 3)
            {
                m_pixel[j] = *m_data;
                m_pixel[j+1] = *m_data;
                m_pixel[j+2] = *m_data;

                m_data++;
            }
        }

        QImage result(m_image);
        result.detach();

        emit updateOriginalRGB(result);
    }

    this->m_mutexColor.unlock();
}

void Recognizer::renderInfrared(void)
{
    this->m_mutexInfrared.lock();
// qDebug() << "2_________";
    if (!this->m_originalImageDepth.empty())
    {
//        qDebug() << m_originalImageIR.at<uint16_t>(50,50);
//        qDebug() << m_originalImageIR.size().width << " , " << m_originalImageIR.size().height;
//        cv::imshow("aaaaa", m_originalImageIR);

         qDebug() << "3_________";
        double min, max;
        cv::minMaxIdx(this->m_originalImageIR, &min, &max);
        cv::Mat viewableIR;
        cv::convertScaleAbs(this->m_originalImageIR, viewableIR, 255/max);

        QImage m_image(viewableIR.cols, viewableIR.rows, QImage::Format_RGB888);
        unsigned char *m_data = (unsigned char *)viewableIR.data;
        for (int i = 0; i < m_image.height(); i++)
        {
            uchar *m_pixel = (uchar *)m_image.scanLine(i);
            for (int j = 0; j < (m_image.width() * 3); j += 3)
            {
                m_pixel[j] = *m_data;
                m_pixel[j+1] = *m_data;
                m_pixel[j+2] = *m_data;

                m_data++;
            }
        }

        QImage result(m_image);
        result.detach();

        emit updateOriginalIR(result);
    }

    this->m_mutexInfrared.unlock();
}

void Recognizer::renderHandSegmentation(void)
{
    this->m_mutexHandSegm.lock();

    if (!this->m_imageHandSegm.empty())
    {
        double min, max;
        cv::minMaxIdx(this->m_imageHandSegm, &min, &max);
        cv::Mat viewableDepth;
        cv::convertScaleAbs(this->m_imageHandSegm, viewableDepth, 255/max);

        QImage m_image(viewableDepth.cols, viewableDepth.rows, QImage::Format_RGB888);
        unsigned char *m_data = (unsigned char *)viewableDepth.data;
        for (int i = 0; i < m_image.height(); i++)
        {
            uchar *m_pixel = (uchar *)m_image.scanLine(i);
            for (int j = 0; j < (m_image.width() * 3); j += 3)
            {
                m_pixel[j] = *m_data;
                m_pixel[j+1] = *m_data;
                m_pixel[j+2] = *m_data;

                m_data++;
            }
        }

        QImage result(m_image);
        result.detach();

        emit updateHandSegmented(result);
    }

    this->m_mutexHandSegm.unlock();
}

void Recognizer::renderUserSkeleton(void)
{
    this->m_mutexUserSkel.lock();

    if (!this->m_imageUserSkel.empty())
    {
        QImage m_image((unsigned char *)this->m_imageUserSkel.data,
                       this->m_imageUserSkel.cols,
                       this->m_imageUserSkel.rows,
                       QImage::Format_RGB888);
        QImage result(m_image);
        result.detach();

        emit updateUserSkeleton(result);
    }

    this->m_mutexUserSkel.unlock();
}

void Recognizer::renderHandSegmentationCloseUp(void)
{
    this->m_mutexHandCloseUp.lock();

    if (!this->m_imageHandSegmCloseUp.empty())
    {
        double min, max;
        cv::minMaxIdx(this->m_imageHandSegmCloseUp, &min, &max);
        cv::Mat viewableHand;
        cv::convertScaleAbs(this->m_imageHandSegmCloseUp, viewableHand, 255/max);

        QImage m_image(viewableHand.cols, viewableHand.rows, QImage::Format_RGB888);
        unsigned char *m_data = (unsigned char *)viewableHand.data;
        for (int i = 0; i < m_image.height(); i++)
        {
            uchar *m_pixel = (uchar *)m_image.scanLine(i);
            for (int j = 0; j < (m_image.width() * 3); j += 3)
            {
                m_pixel[j] = *m_data;
                m_pixel[j+1] = *m_data;
                m_pixel[j+2] = *m_data;

                m_data++;
            }
        }

        QImage result(m_image);
        result.detach();

        emit updateHandSegmentedCloseUp(result);
    }

    this->m_mutexHandCloseUp.unlock();
}

void Recognizer::run(void)
{
    while (this->m_isActive)
    {
        this->m_mutexUserSkel.lock();

        if (!this->m_imageUserSkel.empty())
            this->m_imageUserSkel.release();

        this->m_imageUserSkel = cv::Mat::zeros(SENSOR_HEIGHT, SENSOR_WIDTH, CV_8UC3);

        for (int i = 0; i < this->users.size(); i++)
        {
            cv::circle(this->m_imageUserSkel, cv::Point(this->users[i].getProjectiveSkeleton().head.position.x(),
                            this->users[i].getProjectiveSkeleton().head.position.y()), 3, this->users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().neck.position.x(),
                            users[i].getProjectiveSkeleton().neck.position.y()), 3, users[i].getUserColor(), CV_FILLED);

            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_shoulder.position.x(),
                            users[i].getProjectiveSkeleton().left_shoulder.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_shoulder.position.x(),
                            users[i].getProjectiveSkeleton().right_shoulder.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_elbow.position.x(),
                            users[i].getProjectiveSkeleton().left_elbow.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_elbow.position.x(),
                            users[i].getProjectiveSkeleton().right_elbow.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_hand.position.x(),
                            users[i].getProjectiveSkeleton().left_hand.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_hand.position.x(),
                            users[i].getProjectiveSkeleton().right_hand.position.y()), 3, users[i].getUserColor(), CV_FILLED);

            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().torso.position.x(),
                            users[i].getProjectiveSkeleton().torso.position.y()), 3, users[i].getUserColor(), CV_FILLED);

            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_hip.position.x(),
                            users[i].getProjectiveSkeleton().left_hip.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_hip.position.x(),
                            users[i].getProjectiveSkeleton().right_hip.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_knee.position.x(),
                            users[i].getProjectiveSkeleton().left_knee.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_knee.position.x(),
                            users[i].getProjectiveSkeleton().right_knee.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().left_foot.position.x(),
                            users[i].getProjectiveSkeleton().left_foot.position.y()), 3, users[i].getUserColor(), CV_FILLED);
            cv::circle(this->m_imageUserSkel, cv::Point(users[i].getProjectiveSkeleton().right_foot.position.x(),
                            users[i].getProjectiveSkeleton().right_foot.position.y()), 3, users[i].getUserColor(), CV_FILLED);
        }

        this->m_mutexUserSkel.unlock();
    }
}

// _____________________________________
User::User(cv::Scalar color) { this->userColor = color; }

void User::setUserMap(UserMap map) { this->currentMap = map; }
void User::setProjectiveSkeleton(ProjectiveSkeleton skeleton) { this->currentProjectiveSkeleton = skeleton; }
void User::setRealWorldSkeleton(RealWorldSkeleton skeleton) { this->currentRealWorldSkeleton = skeleton; }

cv::Scalar User::getUserColor() { return this->userColor; }
UserMap User::getUserMap(void) { return this->currentMap; }
ProjectiveSkeleton User::getProjectiveSkeleton(void) { return this->currentProjectiveSkeleton; }
RealWorldSkeleton User::getRealWorldSkeleton(void) { return this->currentRealWorldSkeleton; }
