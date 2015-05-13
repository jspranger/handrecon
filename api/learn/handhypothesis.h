#ifndef HANDHYPOTHESIS_H
#define HANDHYPOTHESIS_H

#include <QThread>

#include <opencv2/opencv.hpp>
#include <math.h>
#include <random>
#include <time.h>

#include "../../ogrewidget.h"
#include "handmodel.h"

//////////////// TEMP
#include "./../hw/sensordevice.h"

#define M_PI        3.14159

#define NUMBER_OF_DIMENSIONS	23

class HandHypothesis : public QThread
{
    Q_OBJECT
public:
    explicit HandHypothesis(QObject *parent = 0, int numberOfGenerations = 50, int numberOfParticles = 20);
    ~HandHypothesis();

    void startOptimization(cv::Mat observedSegmentation, cv::Mat observedDepth);
    void stopOptimization(void);

private:
    void run(void);

    std::vector<cv::Point> largest_osContour;

    bool isActive,
            clearParticles;
    QMutex m_mutex;

    HandModel *m_handModel;
    double evaluate(int p);

    struct ParameterSpace
    {
        double minimum;
        double maximum;
    };
    struct Dimension
    {
        double currentPosition;
        double nextPosition;
        double bestPosition;
        double velocity;
    };
    struct Particle
    {
        Dimension dimension[NUMBER_OF_DIMENSIONS];
        double bestSoFar;
    };
    cv::Mat m_observedSegmentation,
            m_observedDepth;

    int m_numberOfGenerations,
        m_numberOfParticles;

    ParameterSpace limits[NUMBER_OF_DIMENSIONS];
    double bestPopulation[NUMBER_OF_DIMENSIONS];
    double bestSoFar;

    int randomizerCounter;
    int fingerJointsIndex[20];

    double c1, c2, w;
    Particle *particles;

public slots:
    void renderReady(Ogre::Root *, Ogre::RenderWindow *, Ogre::Camera *, Ogre::Viewport *, Ogre::SceneManager *);

signals:
    void bestHypothesis(void); // Change later to coordinates
    void updateImgTEMP(IMAGE::Mode, cv::Mat);
};

#endif // HANDHYPOTHESIS_H
