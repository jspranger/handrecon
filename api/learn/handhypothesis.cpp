#include "handhypothesis.h"

std::mt19937 Generator;
HandHypothesis::HandHypothesis(QObject *parent, int numberOfGenerations, int numberOfParticles) :
    QThread(parent)
{
    qRegisterMetaType<Ogre::Root *>("Ogre::Root *");
    qRegisterMetaType<Ogre::RenderWindow *>("Ogre::RenderWindow *");
    qRegisterMetaType<Ogre::Camera *>("Ogre::Camera *");
    qRegisterMetaType<Ogre::Viewport *>("Ogre::Viewport *");
    qRegisterMetaType<Ogre::SceneManager *>("Ogre::SceneManager *");

    ///////////// TEMP
    connect(this, SIGNAL(updateImgTEMP(IMAGE::Mode, cv::Mat)), parent, SLOT(image_data(IMAGE::Mode, cv::Mat)), Qt::DirectConnection);

    this->c1 = 2.8;
    this->c2 = 1.3;
    this->w = 2/abs(2-(c1+c2)-sqrt(pow(c1+c2,2)-(4*(c1+c2))));

    this->m_numberOfGenerations = numberOfGenerations;
    this->m_numberOfParticles = numberOfParticles;

    // All
    this->limits[0].minimum = 0.0; // Get from hand model
    this->limits[1].minimum = 0.0; // Get from hand model
    this->limits[2].minimum = 0.0; // Get from hand model

    // Thumb
    this->limits[3].minimum = 0.0; // Get from hand model
    this->limits[4].minimum = 0.0; // Get from hand model
    this->limits[5].minimum = 0.0; // Get from hand model
    this->limits[6].minimum = 0.0; // Get from hand model

    // Index
    this->limits[7].minimum = 0.0; // Get from hand model
    this->limits[8].minimum = -45.0; // Get from hand model
    this->limits[9].minimum = 0.0; // Get from hand model
    this->limits[10].minimum = 0.0; // Get from hand model

    // Middle
    this->limits[11].minimum = 0.0; // Get from hand model
    this->limits[12].minimum = -45.0; // Get from hand model
    this->limits[13].minimum = 0.0; // Get from hand model
    this->limits[14].minimum = 0.0; // Get from hand model

    // Third
    this->limits[15].minimum = 0.0; // Get from hand model
    this->limits[16].minimum = -45.0; // Get from hand model
    this->limits[17].minimum = 0.0; // Get from hand model
    this->limits[18].minimum = 0.0; // Get from hand model

    // Little
    this->limits[19].minimum = 0.0; // Get from hand model
    this->limits[20].minimum = -45.0; // Get from hand model
    this->limits[21].minimum = 0.0; // Get from hand model
    this->limits[22].minimum = 0.0; // Get from hand model

    // All
    this->limits[0].maximum = 359.9; // Get from hand model
    this->limits[1].maximum = 359.9; // Get from hand model
    this->limits[2].maximum = 359.9; // Get from hand model

    // Thumb
    this->limits[3].maximum = 30.0; // Get from hand model
    this->limits[4].maximum = 10.0; // Get from hand model
    this->limits[5].maximum = 90.0; // Get from hand model
    this->limits[6].maximum = 90.0; // Get from hand model

    // Index
    this->limits[7].maximum = 90.0; // Get from hand model
    this->limits[8].maximum = 45.0; // Get from hand model
    this->limits[9].maximum = 90.0; // Get from hand model
    this->limits[10].maximum = 90.0; // Get from hand model

    // Middle
    this->limits[11].maximum = 90.0; // Get from hand model
    this->limits[12].maximum = 45.0; // Get from hand model
    this->limits[13].maximum = 90.0; // Get from hand model
    this->limits[14].maximum = 90.0; // Get from hand model

    // Third
    this->limits[15].maximum = 90.0; // Get from hand model
    this->limits[16].maximum = 45.0; // Get from hand model
    this->limits[17].maximum = 90.0; // Get from hand model
    this->limits[18].maximum = 90.0; // Get from hand model

    this->limits[19].maximum = 90.0; // Get from hand model
    this->limits[20].maximum = 45.0; // Get from hand model
    this->limits[21].maximum = 90.0; // Get from hand model
    this->limits[22].maximum = 90.0; // Get from hand model

    this->clearParticles = false;
    this->particles = 0;
    this->m_handModel = 0;

    std::random_device seed;
    Generator.seed(seed());

    randomizerCounter = 0;

    this->isActive = false;
}

HandHypothesis::~HandHypothesis()
{
}

int getRandom(int min, int max){
    return std::uniform_int_distribution<int>(min, max)(Generator);
}

float getRandomFloat(float min, float max){
    return std::uniform_real_distribution<float>(min, max)(Generator);
}

void HandHypothesis::renderReady(Ogre::Root *ogreRoot, Ogre::RenderWindow *ogreWindow, Ogre::Camera *ogreCamera, Ogre::Viewport *ogreViewport, Ogre::SceneManager *ogreScreenMgr)
{
    this->m_handModel = new HandModel(ogreRoot, ogreWindow, ogreCamera, ogreViewport, ogreScreenMgr);
}
double hack = 1000;
void HandHypothesis::startOptimization(cv::Mat observedSegmentation, cv::Mat observedDepth)
{
    if (!this->isActive)
    {
        this->m_observedSegmentation = observedSegmentation.clone();
        cv::Mat observedSegmentationCopy;
        this->m_observedSegmentation.copyTo(observedSegmentationCopy);

        std::vector<std::vector<cv::Point> > osContours;
        cv::findContours(observedSegmentationCopy, osContours,  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        if (osContours.size())
        {
            this->largest_osContour = osContours.at(0);
            for (unsigned int c = 0; c < osContours.size(); c++)
                if (cv::contourArea(osContours.at(c)) > cv::contourArea(this->largest_osContour))
                    this->largest_osContour = osContours.at(c);
            observedDepth.copyTo(this->m_observedDepth, this->m_observedSegmentation);

            if (!this->m_observedSegmentation.empty() && !this->m_observedDepth.empty() && m_handModel)
            {
                this->isActive = true;
                hack = 1000;
                this->start();
            }
        }
    }
}

void HandHypothesis::stopOptimization(void)
{
    if (this->isActive)
        this->clearParticles = true;
    else
    {
        if (this->particles)
            delete this->particles;
        this->particles = 0;
        this->clearParticles= false;
    }
}

void HandHypothesis::run(void)
{
    if (this->isActive)
    {
        this->m_mutex.lock();

        if (!this->particles)
        {
            // Initialize particles at random
            this->particles = (Particle *)calloc(this->m_numberOfParticles, sizeof(Particle));
            this->bestSoFar = 1000;
            for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
//                this->bestPopulation[d] = (((rand() % 1000)/1000.0) * (this->limits[d].maximum - this->limits[d].minimum)) + this->limits[d].minimum;
                this->bestPopulation[d] = 0;
        }
        // Generate Initial Population _________________________________
        for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
        {
            // First member of initial population in frame t is the best
            // alternative obtained in frame t-1;
            this->particles[1].dimension[d].nextPosition = this->bestPopulation[d];
            this->particles[1].dimension[d].bestPosition = this->particles[1].dimension[d].nextPosition;
            this->particles[1].dimension[d].velocity = 0;
            for (int p = 1; p < this->m_numberOfParticles; p++)
            {
                // Since a number between [0..1] is being multiplied to
                // an already scaled value, its result will never
                // overflow the parameter space for the dimension;
                this->particles[p].dimension[d].nextPosition = getRandomFloat(this->limits[d].minimum,this->limits[d].maximum);
                this->particles[p].dimension[d].bestPosition = this->particles[p].dimension[d].nextPosition;
                this->particles[p].dimension[d].velocity = 0;
            }
        }
        for (int p = 0; p < this->m_numberOfParticles; p++)
            this->particles[p].bestSoFar = 1000;
        // _____________________________________________________________

        // Iterate through Generations _________________________________
        for (int g = 0; g < this->m_numberOfGenerations; g++)
        {
            randomizerCounter += 1;
QString mmm;
            for (int p = 0; p < this->m_numberOfParticles; p++)
            {
                for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
                {
                    particles[p].dimension[d].currentPosition = particles[p].dimension[d].nextPosition;
                    mmm.append(QString::number(particles[p].dimension[d].currentPosition));
                    mmm.append(" | ");
                }
                qDebug() << " _ ";



//                if (randomizerCounter == 3)
//                {

//                    for (int i = 0; i < 10; i++)
//                    {
//                        int index = getRandom(3,22);
//                        particles[p].dimension[index].currentPosition = getRandomFloat(this->limits[index].minimum,this->limits[index].maximum);
//                    }
//                }

                qDebug() <<  mmm;
                double fitness = evaluate(p);

                if (fitness < particles[p].bestSoFar)
                {
                    this->particles[p].bestSoFar = fitness;
                    for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
                        this->particles[p].dimension[d].bestPosition = this->particles[p].dimension[d].currentPosition;

                    if (this->particles[p].bestSoFar < this->bestSoFar)
                    {
                        this->bestSoFar = this->particles[p].bestSoFar;
                        for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
                            this->bestPopulation[d] = this->particles[p].dimension[d].bestPosition;
                    }
                }
            }
            // Update Position _________________________________
            for (int p = 0; p < this->m_numberOfParticles; p++)
                for (int d = 0; d < NUMBER_OF_DIMENSIONS; d++)
                {
                    particles[p].dimension[d].velocity = w * (
                                particles[p].dimension[d].velocity
                                                              +
                                                              (this->c1 * (getRandomFloat(particles[p].dimension[d].currentPosition, particles[p].dimension[d].bestPosition)))
                                                              +
                                                              (this->c2 * (getRandomFloat(particles[p].dimension[d].currentPosition, particles[p].bestSoFar)))
                    );

                    particles[p].dimension[d].nextPosition = particles[p].dimension[d].currentPosition + particles[p].dimension[d].velocity;
                    if (particles[p].dimension[d].nextPosition < this->limits[d].minimum)
                        particles[p].dimension[d].nextPosition = this->limits[d].minimum;
                    if (particles[p].dimension[d].nextPosition > this->limits[d].maximum)
                        particles[p].dimension[d].nextPosition = this->limits[d].maximum;
                }
            if (randomizerCounter >= 3 )
                randomizerCounter = 0;
            // _________________________________________________
        }
        // _____________________________________________________________

        emit bestHypothesis(); // Coordinates here
        // EMIT COORDINATES OF HAND POSITION!!!!!!

        if (this->clearParticles)
        {
            if (this->particles)
                delete this->particles;
            this->particles = 0;
            this->clearParticles= false;
        }

        this->isActive = false;

        this->m_mutex.unlock();
    }
}




double getRadianVector3DDiff(QVector3D v1, QVector3D v2)
{

    return acos((((v1.x() * v2.x())+(v1.y() * v2.y())+(v1.z() * v2.z())) /
                (double) sqrt(pow(v1.x(), 2.0) + pow(v1.y(), 2.0) + pow(v1.z(), 2.0)) +
                    sqrt(pow(v2.x(), 2.0) + pow(v2.y(), 2.0) + pow(v2.z(), 2.0))) * M_PI / 180);
}

double HandHypothesis::evaluate(int p)
{
    double result = 1000.0;

    // Render hypothesis and evaluate
    m_handModel->updateParameters(particles[p].dimension[0].currentPosition,
                                                              particles[p].dimension[1].currentPosition,
                                                              particles[p].dimension[2].currentPosition,
                                                              particles[p].dimension[3].currentPosition,
                                                              particles[p].dimension[4].currentPosition,
                                                              particles[p].dimension[5].currentPosition,
                                                              particles[p].dimension[6].currentPosition,
                                                              particles[p].dimension[7].currentPosition,
                                                              particles[p].dimension[8].currentPosition,
                                                              particles[p].dimension[9].currentPosition,
                                                              particles[p].dimension[10].currentPosition,
                                                              particles[p].dimension[11].currentPosition,
                                                              particles[p].dimension[12].currentPosition,
                                                              particles[p].dimension[13].currentPosition,
                                                              particles[p].dimension[14].currentPosition,
                                                              particles[p].dimension[15].currentPosition,
                                                              particles[p].dimension[16].currentPosition,
                                                              particles[p].dimension[17].currentPosition,
                                                              particles[p].dimension[18].currentPosition,
                                                              particles[p].dimension[19].currentPosition,
                                                              particles[p].dimension[20].currentPosition,
                                                              particles[p].dimension[21].currentPosition,
                                                              particles[p].dimension[22].currentPosition);

    // Get rendered image
    // Compare
    cv::Mat m_renderedImage = cv::Mat::zeros(400, 400, CV_8UC4);
    int status;
    do
    {
        status = m_handModel->getRenderedImage(m_renderedImage.data);
    } while (status < 0);

    if (!status)
    {
        cv::Mat m_grey,
                     m_bin,
                     m_binCopy;
        cv::cvtColor(m_renderedImage, m_grey, CV_BGRA2GRAY);
        m_grey.copyTo(m_bin);
        m_bin = m_bin > 0;
        m_bin.copyTo(m_binCopy);

        std::vector<std::vector<cv::Point> > hpContours;
        cv::findContours(m_binCopy, hpContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        if (largest_osContour.size() && hpContours.size())
        {
            cv::Rect bbRendered = cv::boundingRect(hpContours.at(0)),
                          bbSegmented = cv::boundingRect(largest_osContour);

            if ((bbSegmented.area() > 0) && (bbRendered.area() > 0))
            {
                // Normalize segmentation images
                cv::Mat hypothesisSegmentation = cv::Mat::zeros(m_observedSegmentation.rows, m_observedSegmentation.cols, CV_8UC1);

                cv::Mat hsTemp;
                m_bin(bbRendered).copyTo(hsTemp);
                cv::resize(hsTemp, hsTemp, bbSegmented.size(),0,0,CV_INTER_CUBIC);
                hsTemp.copyTo(hypothesisSegmentation(bbSegmented));

                // Normalize depth images
                cv::Mat hypothesisDepth = cv::Mat::zeros(m_observedDepth.rows, m_observedDepth.cols, CV_16UC1);

                cv::Mat hdTemp;
                m_grey(bbRendered).convertTo(hdTemp, CV_16UC1);
                cv::resize(hdTemp, hdTemp, bbSegmented.size(), 0, 0, CV_INTER_CUBIC);
                double min,max;
                cv::minMaxIdx(m_observedDepth, &min, &max);
                cv::normalize(hdTemp, hdTemp, min, max, CV_MINMAX);
                hdTemp.copyTo(hypothesisDepth(bbSegmented));

                // Article Evaluation formula
                cv::Mat diff;
                cv::absdiff(m_observedDepth, hypothesisDepth, diff);
                for (int i = 0; i < (diff.rows * diff.cols); i++)
                    diff.data[i] = (diff.data[i] > 4)?4:diff.data[i];

//                // - D(O,h,C)
                double intersectionCount = cv::countNonZero(m_observedSegmentation & hypothesisSegmentation),
                            reunionCount = cv::countNonZero(m_observedSegmentation | hypothesisSegmentation);

                double D =  (cv::countNonZero(diff) / (double)(reunionCount + 0.01)) + 20 * (1 - ((2 * intersectionCount) / (double)(intersectionCount + reunionCount)));
                // Obtain constrictions (finger interpenetration)
                QVector3D indexVector(m_handModel->getHandParameters().middleIndex.wPosition.x - m_handModel->getHandParameters().proximalIndex.wPosition.x,
                                      m_handModel->getHandParameters().middleIndex.wPosition.y - m_handModel->getHandParameters().proximalIndex.wPosition.y,
                                      m_handModel->getHandParameters().middleIndex.wPosition.z - m_handModel->getHandParameters().proximalIndex.wPosition.z),
                                  middleVector(m_handModel->getHandParameters().middleMiddle.wPosition.x - m_handModel->getHandParameters().proximalMiddle.wPosition.x,
                                               m_handModel->getHandParameters().middleMiddle.wPosition.y - m_handModel->getHandParameters().proximalMiddle.wPosition.y,
                                               m_handModel->getHandParameters().middleMiddle.wPosition.z - m_handModel->getHandParameters().proximalMiddle.wPosition.z),
                                  thirdVector(m_handModel->getHandParameters().middleThird.wPosition.x - m_handModel->getHandParameters().proximalThird.wPosition.x,
                                              m_handModel->getHandParameters().middleThird.wPosition.y - m_handModel->getHandParameters().proximalThird.wPosition.y,
                                              m_handModel->getHandParameters().middleThird.wPosition.z - m_handModel->getHandParameters().proximalThird.wPosition.z),
                                  littleVector(m_handModel->getHandParameters().middleLittle.wPosition.x - m_handModel->getHandParameters().proximalLittle.wPosition.x,
                                               m_handModel->getHandParameters().middleLittle.wPosition.y - m_handModel->getHandParameters().proximalLittle.wPosition.y,
                                               m_handModel->getHandParameters().middleLittle.wPosition.z - m_handModel->getHandParameters().proximalLittle.wPosition.z);



//                cos ang = v.w / mag(v)*mag(w);

                double Kc = (1 -
                             std::min(getRadianVector3DDiff(indexVector, middleVector), 0.0)
                             ) +
                        (1 - std::min(getRadianVector3DDiff(middleVector, thirdVector), 0.0)) +
                        (1 -
                         std::min(getRadianVector3DDiff(thirdVector, littleVector), 0.0)
                         );
                double E = D * Kc;

                result = E;

                qDebug() << E;


//                %%% - Obtain constrictions (finger interpenetration) kc(h)
//                kc = (1 - min(degtorad(data.index(1).abduction) - degtorad(data.middle(1).abduction),0)) + ...
//                     (1 - min(degtorad(data.middle(1).abduction) - degtorad(data.third(1).abduction),0)) + ...
//                     (1 - min(degtorad(data.third(1).abduction) - degtorad(data.little(1).abduction),0));


                // Evaluation
                double inverseSimilarity = 0;
                if (reunionCount != 0)
                    //inverseSimilarity = 1 - ((2 * intersectionCount) / (intersectionCount + reunionCount));
                    inverseSimilarity = 1 - (intersectionCount / reunionCount);

//                qDebug() << "Similarity Measure is: " << inverseSimilarity;
//                result = inverseSimilarity;
                if (result < hack)
                {
                    qDebug() << result;
                    qDebug() << "Writing!";
                    emit updateImgTEMP(IMAGE::IR, hypothesisSegmentation);
                    hack = result;
                    cv::imwrite("observedOriginal_S.png", m_observedSegmentation);
                    cv::imwrite("observedBest_S.png", hypothesisSegmentation);
                    cv::imwrite("observedOriginal_D.png", m_observedDepth);
                    cv::imwrite("observedBest_D.png", hypothesisDepth);
                }\
            }
        }
    }

    return result;
}
