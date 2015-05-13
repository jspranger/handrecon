#ifndef HANDMODEL_H
#define HANDMODEL_H

#include <QString>
#include <OGRE/Ogre.h>

#include <QDebug>

class HandModel : public Ogre::RenderTargetListener
{
public:
    explicit HandModel(Ogre::Root *ogreRoot = 0, Ogre::RenderWindow *ogreWindow = 0, Ogre::Camera *ogreCamera = 0, Ogre::Viewport *ogreViewport = 0, Ogre::SceneManager *ogreSceneMgr = 0);
    ~HandModel(void);

    typedef struct
    {
        Ogre::Bone *handle;
        Ogre::Vector3 wPosition;
    } Joint;

    typedef struct
    {
        Ogre::SceneNode *rootNode;

        Joint wrist;

        Joint rootThumb;
        Joint proximalThumb;
        Joint distalThumb;

        Joint rootIndex;
        Joint proximalIndex;
        Joint middleIndex;
        Joint distalIndex;

        Joint rootMiddle;
        Joint proximalMiddle;
        Joint middleMiddle;
        Joint distalMiddle;

        Joint rootThird;
        Joint proximalThird;
        Joint middleThird;
        Joint distalThird;

        Joint rootLittle;
        Joint proximalLittle;
        Joint middleLittle;
        Joint distalLittle;
    } HandJoints;

    void updateParameters(double p1, double p2, double p3,
                                           double p4, double p5,
                                           double p6,
                                           double p7,
                                           double p8, double p9,
                                           double p10,
                                           double p11,
                                           double p12, double p13,
                                           double p14,
                                           double p15,
                                           double p16, double p17,
                                           double p18,
                                           double p19,
                                           double p20, double p21,
                                           double p22,
                                           double p23);
    void resetParameters(void);

    int getRenderedImage(unsigned char *ptr);
    HandJoints getHandParameters(void);

private:
    Ogre::Root *m_ogreRoot;
    Ogre::RenderWindow *m_ogreWindow;
    Ogre::Camera *m_ogreCamera;
    Ogre::Viewport *m_ogreViewport;
    Ogre::SceneManager *m_ogreSceneMgr;
    Ogre::TexturePtr m_ogreTexturePtr;

    void postRenderTargetUpdate(const Ogre::RenderTargetEvent &evt);
    bool isHandUpdated, isCameraUpdated;

    unsigned char *data;
    int bufferSize;

    Ogre::Entity *m_hand;
    HandJoints m_handJoints;
};
#endif // HANDMODEL_H
