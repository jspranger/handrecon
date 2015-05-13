#include "handmodel.h"

HandModel::HandModel(Ogre::Root *ogreRoot, Ogre::RenderWindow *ogreWindow, Ogre::Camera *ogreCamera, Ogre::Viewport *ogreViewport, Ogre::SceneManager *ogreSceneMgr)
{
    m_ogreRoot = ogreRoot;
    m_ogreWindow = ogreWindow;
    m_ogreCamera = ogreCamera;
    m_ogreViewport = ogreViewport;
    m_ogreSceneMgr = ogreSceneMgr;

    m_ogreTexturePtr.setNull();

    m_ogreCamera->setPosition(Ogre::Vector3(0,0,0));
    m_ogreCamera->lookAt(Ogre::Vector3(0,0,-1));

    m_ogreSceneMgr->setAmbientLight(Ogre::ColourValue(1,1,1));

    Ogre::Light* light = m_ogreSceneMgr->createLight("MainLight");

    light->setPosition(Ogre::Vector3(0,0,0));
    light->setDirection(Ogre::Vector3(0,0,-1));
    m_ogreCamera->setNearClipDistance(1);

    m_ogreTexturePtr = Ogre::TextureManager::getSingleton().createManual("RTT",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 400, 400, 0, Ogre::PF_B8G8R8A8,
        Ogre::TU_RENDERTARGET | Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
    m_ogreTexturePtr->getBuffer()->getRenderTarget()->addViewport(m_ogreCamera);
    m_ogreTexturePtr->getBuffer()->getRenderTarget()->getViewport(0)->setClearEveryFrame(true);
    m_ogreTexturePtr->getBuffer()->getRenderTarget()->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
    m_ogreTexturePtr->getBuffer()->getRenderTarget()->getViewport(0)->setOverlaysEnabled(false);

    bufferSize = 400*400*Ogre::PixelUtil::getNumElemBytes(Ogre::PF_BYTE_BGRA);
    data = (unsigned char *)calloc(bufferSize, sizeof(unsigned char));
    m_ogreTexturePtr->getBuffer()->getRenderTarget()->addListener(this);

    // Create Hand Entity
    m_hand = m_ogreSceneMgr->createEntity("handRight", "handRight_mesh.mesh");

    // Create a SceneNode and attach the Entity to it
    m_handJoints.rootNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode("handRight_wrist");
    m_handJoints.rootNode->attachObject(m_hand);

    m_handJoints.wrist.handle = m_hand->getSkeleton()->getBone("handRight_wrist");
    m_handJoints.wrist.handle->setManuallyControlled(true);

    m_handJoints.rootThumb.handle = m_hand->getSkeleton()->getBone("handRight_rootThumb");
    m_handJoints.rootThumb.handle->setManuallyControlled(true);
    m_handJoints.proximalThumb.handle = m_hand->getSkeleton()->getBone("handRight_proximalThumb");
    m_handJoints.proximalThumb.handle->setManuallyControlled(true);
    m_handJoints.distalThumb.handle = m_hand->getSkeleton()->getBone("handRight_distalThumb");
    m_handJoints.distalThumb.handle->setManuallyControlled(true);

    m_handJoints.rootIndex.handle = m_hand->getSkeleton()->getBone("handRight_rootIndex");
    m_handJoints.rootIndex.handle->setManuallyControlled(true);
    m_handJoints.proximalIndex.handle = m_hand->getSkeleton()->getBone("handRight_proximalIndex");
    m_handJoints.proximalIndex.handle->setManuallyControlled(true);
    m_handJoints.middleIndex.handle = m_hand->getSkeleton()->getBone("handRight_middleIndex");
    m_handJoints.middleIndex.handle->setManuallyControlled(true);
    m_handJoints.distalIndex.handle = m_hand->getSkeleton()->getBone("handRight_distalIndex");
    m_handJoints.distalIndex.handle->setManuallyControlled(true);

    m_handJoints.rootMiddle.handle = m_hand->getSkeleton()->getBone("handRight_rootMiddle");
    m_handJoints.rootMiddle.handle->setManuallyControlled(true);
    m_handJoints.proximalMiddle.handle = m_hand->getSkeleton()->getBone("handRight_proximalMiddle");
    m_handJoints.proximalMiddle.handle->setManuallyControlled(true);
    m_handJoints.middleMiddle.handle = m_hand->getSkeleton()->getBone("handRight_middleMiddle");
    m_handJoints.middleMiddle.handle->setManuallyControlled(true);
    m_handJoints.distalMiddle.handle = m_hand->getSkeleton()->getBone("handRight_distalMiddle");
    m_handJoints.distalMiddle.handle->setManuallyControlled(true);

    m_handJoints.rootThird.handle = m_hand->getSkeleton()->getBone("handRight_rootThird");
    m_handJoints.rootThird.handle->setManuallyControlled(true);
    m_handJoints.proximalThird.handle = m_hand->getSkeleton()->getBone("handRight_proximalThird");
    m_handJoints.proximalThird.handle->setManuallyControlled(true);
    m_handJoints.middleThird.handle = m_hand->getSkeleton()->getBone("handRight_middleThird");
    m_handJoints.middleThird.handle->setManuallyControlled(true);
    m_handJoints.distalThird.handle = m_hand->getSkeleton()->getBone("handRight_distalThird");
    m_handJoints.distalThird.handle->setManuallyControlled(true);

    m_handJoints.rootLittle.handle = m_hand->getSkeleton()->getBone("handRight_rootLittle");
    m_handJoints.rootLittle.handle->setManuallyControlled(true);
    m_handJoints.proximalLittle.handle = m_hand->getSkeleton()->getBone("handRight_proximalLittle");
    m_handJoints.proximalLittle.handle->setManuallyControlled(true);
    m_handJoints.middleLittle.handle = m_hand->getSkeleton()->getBone("handRight_middleLittle");
    m_handJoints.middleLittle.handle->setManuallyControlled(true);
    m_handJoints.distalLittle.handle = m_hand->getSkeleton()->getBone("handRight_distalLittle");
    m_handJoints.distalLittle.handle->setManuallyControlled(true);

    m_handJoints.rootNode->translate(Ogre::Vector3(0,0,-40));

    this->updateParameters(0,0,0,
                           0,0,
                           0,
                           0,
                           0,0,
                           0,
                           0,
                           0,0,
                           0,
                           0,
                           0,0,
                           0,
                           0,
                           0,0,
                           0,
                           0);
}

HandModel::~HandModel(void)
{
    //// Shutdown Ogre!!!!! <- Later
    //if (!m_ogreTexturePtr.isNull())
    //{
    //	// Delete, destroy, whatever
    //	;
    //}
}

void HandModel::updateParameters(double p1, double p2, double p3,
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
                                          double p23)
{
    m_handJoints.wrist.handle->reset();
    m_handJoints.wrist.handle->roll(Ogre::Radian(Ogre::Degree(90+p3).valueRadians()));
    m_handJoints.wrist.handle->yaw(Ogre::Radian(Ogre::Degree(180-p2).valueRadians()));
    m_handJoints.wrist.handle->pitch(Ogre::Radian(Ogre::Degree(-p1).valueRadians()));

    m_handJoints.rootThumb.handle->reset();
    m_handJoints.proximalThumb.handle->reset();
    m_handJoints.distalThumb.handle->reset();
    m_handJoints.rootThumb.handle->pitch(Ogre::Radian(Ogre::Degree(-p4).valueRadians()));
    m_handJoints.rootThumb.handle->roll(Ogre::Radian(Ogre::Degree(-p5).valueRadians()));
    m_handJoints.proximalThumb.handle->roll(Ogre::Radian(Ogre::Degree(-p6).valueRadians()));
    m_handJoints.distalThumb.handle->roll(Ogre::Radian(Ogre::Degree(-p7).valueRadians()));

    m_handJoints.proximalIndex.handle->reset();
    m_handJoints.middleIndex.handle->reset();
    m_handJoints.distalIndex.handle->reset();
    m_handJoints.proximalIndex.handle->roll(Ogre::Radian(Ogre::Degree(-p8).valueRadians()));
    m_handJoints.proximalIndex.handle->pitch(Ogre::Radian(Ogre::Degree(-p9).valueRadians()));
    m_handJoints.middleIndex.handle->roll(Ogre::Radian(Ogre::Degree(-p10).valueRadians()));
    m_handJoints.distalIndex.handle->roll(Ogre::Radian(Ogre::Degree(-p11).valueRadians()));

    m_handJoints.proximalMiddle.handle->reset();
    m_handJoints.middleMiddle.handle->reset();
    m_handJoints.distalMiddle.handle->reset();
    m_handJoints.proximalMiddle.handle->roll(Ogre::Radian(Ogre::Degree(-p12).valueRadians()));
    m_handJoints.proximalMiddle.handle->pitch(Ogre::Radian(Ogre::Degree(-p13).valueRadians()));
    m_handJoints.middleMiddle.handle->roll(Ogre::Radian(Ogre::Degree(-p14).valueRadians()));
    m_handJoints.distalMiddle.handle->roll(Ogre::Radian(Ogre::Degree(-p15).valueRadians()));

    m_handJoints.proximalThird.handle->reset();
    m_handJoints.middleThird.handle->reset();
    m_handJoints.distalThird.handle->reset();
    m_handJoints.proximalThird.handle->roll(Ogre::Radian(Ogre::Degree(-p16).valueRadians()));
    m_handJoints.proximalThird.handle->pitch(Ogre::Radian(Ogre::Degree(-p17).valueRadians()));
    m_handJoints.middleThird.handle->roll(Ogre::Radian(Ogre::Degree(-p18).valueRadians()));
    m_handJoints.distalThird.handle->roll(Ogre::Radian(Ogre::Degree(-p19).valueRadians()));

    m_handJoints.proximalLittle.handle->reset();
    m_handJoints.middleLittle.handle->reset();
    m_handJoints.distalLittle.handle->reset();
    m_handJoints.proximalLittle.handle->roll(Ogre::Radian(Ogre::Degree(-p20).valueRadians()));
    m_handJoints.proximalLittle.handle->pitch(Ogre::Radian(Ogre::Degree(-p21).valueRadians()));
    m_handJoints.middleLittle.handle->roll(Ogre::Radian(Ogre::Degree(-p22).valueRadians()));
    m_handJoints.distalLittle.handle->roll(Ogre::Radian(Ogre::Degree(-p23).valueRadians()));

    isCameraUpdated = false;
    isHandUpdated = false;
}

int HandModel::getRenderedImage(unsigned char *buffer)
{
    int status = 0;

    if (m_ogreTexturePtr.isNull()) status = 1;
    else
    if (!isHandUpdated) status = -1;

    if (!status)
        memcpy(buffer, data, bufferSize);

    return status;
}

HandModel::HandJoints HandModel::getHandParameters(void)
{
    return m_handJoints;
}

void HandModel::postRenderTargetUpdate(const Ogre::RenderTargetEvent &evt)
{
    if (!m_ogreTexturePtr.isNull() && (evt.source == m_ogreTexturePtr->getBuffer()->getRenderTarget()) && !isHandUpdated)
    {
        if (!isCameraUpdated)
        {
            Ogre::Real lowestX = 0.0,
                lowestY = 0.0,
                highestX = 0.0,
                highestY = 0.0;

            // Positions update
            m_handJoints.wrist.wPosition = m_handJoints.wrist.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.wrist.wPosition.x < lowestX) lowestX = m_handJoints.wrist.wPosition.x;
            if (m_handJoints.wrist.wPosition.y < lowestY) lowestX = m_handJoints.wrist.wPosition.y;
            if (m_handJoints.wrist.wPosition.x > highestX) highestX = m_handJoints.wrist.wPosition.x;
            if (m_handJoints.wrist.wPosition.y > highestY) highestY = m_handJoints.wrist.wPosition.y;

            m_handJoints.rootThumb.wPosition = m_handJoints.rootThumb.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.proximalThumb.wPosition = m_handJoints.proximalThumb.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.distalThumb.wPosition = m_handJoints.distalThumb.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.rootThumb.wPosition.x < lowestX) lowestX = m_handJoints.rootThumb.wPosition.x;
            if (m_handJoints.rootThumb.wPosition.y < lowestY) lowestX = m_handJoints.rootThumb.wPosition.y;
            if (m_handJoints.rootThumb.wPosition.x > highestX) highestX = m_handJoints.rootThumb.wPosition.x;
            if (m_handJoints.rootThumb.wPosition.y > highestY) highestY = m_handJoints.rootThumb.wPosition.y;
            if (m_handJoints.proximalThumb.wPosition.x < lowestX) lowestX = m_handJoints.proximalThumb.wPosition.x;
            if (m_handJoints.proximalThumb.wPosition.y < lowestY) lowestX = m_handJoints.proximalThumb.wPosition.y;
            if (m_handJoints.proximalThumb.wPosition.x > highestX) highestX = m_handJoints.proximalThumb.wPosition.x;
            if (m_handJoints.proximalThumb.wPosition.y > highestY) highestY = m_handJoints.proximalThumb.wPosition.y;
            if (m_handJoints.distalThumb.wPosition.x < lowestX) lowestX = m_handJoints.distalThumb.wPosition.x;
            if (m_handJoints.distalThumb.wPosition.y < lowestY) lowestX = m_handJoints.distalThumb.wPosition.y;
            if (m_handJoints.distalThumb.wPosition.x > highestX) highestX = m_handJoints.distalThumb.wPosition.x;
            if (m_handJoints.distalThumb.wPosition.y > highestY) highestY = m_handJoints.distalThumb.wPosition.y;

            m_handJoints.rootIndex.wPosition = m_handJoints.rootIndex.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.proximalIndex.wPosition = m_handJoints.proximalIndex.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.middleIndex.wPosition = m_handJoints.middleIndex.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.distalIndex.wPosition = m_handJoints.distalIndex.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.proximalIndex.wPosition.x < lowestX) lowestX = m_handJoints.proximalIndex.wPosition.x;
            if (m_handJoints.proximalIndex.wPosition.y < lowestY) lowestX = m_handJoints.proximalIndex.wPosition.y;
            if (m_handJoints.proximalIndex.wPosition.x > highestX) highestX = m_handJoints.proximalIndex.wPosition.x;
            if (m_handJoints.proximalIndex.wPosition.y > highestY) highestY = m_handJoints.proximalIndex.wPosition.y;
            if (m_handJoints.middleIndex.wPosition.x < lowestX) lowestX = m_handJoints.middleIndex.wPosition.x;
            if (m_handJoints.middleIndex.wPosition.y < lowestY) lowestX = m_handJoints.middleIndex.wPosition.y;
            if (m_handJoints.middleIndex.wPosition.x > highestX) highestX = m_handJoints.middleIndex.wPosition.x;
            if (m_handJoints.middleIndex.wPosition.y > highestY) highestY = m_handJoints.middleIndex.wPosition.y;
            if (m_handJoints.distalIndex.wPosition.x < lowestX) lowestX = m_handJoints.distalIndex.wPosition.x;
            if (m_handJoints.distalIndex.wPosition.y < lowestY) lowestX = m_handJoints.distalIndex.wPosition.y;
            if (m_handJoints.distalIndex.wPosition.x > highestX) highestX = m_handJoints.distalIndex.wPosition.x;
            if (m_handJoints.distalIndex.wPosition.y > highestY) highestY = m_handJoints.distalIndex.wPosition.y;

            m_handJoints.rootMiddle.wPosition = m_handJoints.rootMiddle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.proximalMiddle.wPosition = m_handJoints.proximalMiddle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.middleMiddle.wPosition = m_handJoints.middleMiddle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.distalMiddle.wPosition = m_handJoints.distalMiddle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.proximalMiddle.wPosition.x < lowestX) lowestX = m_handJoints.proximalMiddle.wPosition.x;
            if (m_handJoints.proximalMiddle.wPosition.y < lowestY) lowestX = m_handJoints.proximalMiddle.wPosition.y;
            if (m_handJoints.proximalMiddle.wPosition.x > highestX) highestX = m_handJoints.proximalMiddle.wPosition.x;
            if (m_handJoints.proximalMiddle.wPosition.y > highestY) highestY = m_handJoints.proximalMiddle.wPosition.y;
            if (m_handJoints.middleMiddle.wPosition.x < lowestX) lowestX = m_handJoints.middleMiddle.wPosition.x;
            if (m_handJoints.middleMiddle.wPosition.y < lowestY) lowestX = m_handJoints.middleMiddle.wPosition.y;
            if (m_handJoints.middleMiddle.wPosition.x > highestX) highestX = m_handJoints.middleMiddle.wPosition.x;
            if (m_handJoints.middleMiddle.wPosition.y > highestY) highestY = m_handJoints.middleMiddle.wPosition.y;
            if (m_handJoints.distalMiddle.wPosition.x < lowestX) lowestX = m_handJoints.distalMiddle.wPosition.x;
            if (m_handJoints.distalMiddle.wPosition.y < lowestY) lowestX = m_handJoints.distalMiddle.wPosition.y;
            if (m_handJoints.distalMiddle.wPosition.x > highestX) highestX = m_handJoints.distalMiddle.wPosition.x;
            if (m_handJoints.distalMiddle.wPosition.y > highestY) highestY = m_handJoints.distalMiddle.wPosition.y;

            m_handJoints.rootThird.wPosition = m_handJoints.rootThird.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.proximalThird.wPosition = m_handJoints.proximalThird.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.middleThird.wPosition = m_handJoints.middleThird.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.distalThird.wPosition = m_handJoints.distalThird.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.proximalThird.wPosition.x < lowestX) lowestX = m_handJoints.proximalThird.wPosition.x;
            if (m_handJoints.proximalThird.wPosition.y < lowestY) lowestX = m_handJoints.proximalThird.wPosition.y;
            if (m_handJoints.proximalThird.wPosition.x > highestX) highestX = m_handJoints.proximalThird.wPosition.x;
            if (m_handJoints.proximalThird.wPosition.y > highestY) highestY = m_handJoints.proximalThird.wPosition.y;
            if (m_handJoints.middleThird.wPosition.x < lowestX) lowestX = m_handJoints.middleThird.wPosition.x;
            if (m_handJoints.middleThird.wPosition.y < lowestY) lowestX = m_handJoints.middleThird.wPosition.y;
            if (m_handJoints.middleThird.wPosition.x > highestX) highestX = m_handJoints.middleThird.wPosition.x;
            if (m_handJoints.middleThird.wPosition.y > highestY) highestY = m_handJoints.middleThird.wPosition.y;
            if (m_handJoints.distalThird.wPosition.x < lowestX) lowestX = m_handJoints.distalThird.wPosition.x;
            if (m_handJoints.distalThird.wPosition.y < lowestY) lowestX = m_handJoints.distalThird.wPosition.y;
            if (m_handJoints.distalThird.wPosition.x > highestX) highestX = m_handJoints.distalThird.wPosition.x;
            if (m_handJoints.distalThird.wPosition.y > highestY) highestY = m_handJoints.distalThird.wPosition.y;

            m_handJoints.rootLittle.wPosition = m_handJoints.rootLittle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.proximalLittle.wPosition = m_handJoints.proximalLittle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.middleLittle.wPosition = m_handJoints.middleLittle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            m_handJoints.distalLittle.wPosition = m_handJoints.distalLittle.handle->convertLocalToWorldPosition(Ogre::Vector3(0, 0, 0));
            if (m_handJoints.proximalLittle.wPosition.x < lowestX) lowestX = m_handJoints.proximalLittle.wPosition.x;
            if (m_handJoints.proximalLittle.wPosition.y < lowestY) lowestX = m_handJoints.proximalLittle.wPosition.y;
            if (m_handJoints.proximalLittle.wPosition.x > highestX) highestX = m_handJoints.proximalLittle.wPosition.x;
            if (m_handJoints.proximalLittle.wPosition.y > highestY) highestY = m_handJoints.proximalLittle.wPosition.y;
            if (m_handJoints.middleLittle.wPosition.x < lowestX) lowestX = m_handJoints.middleLittle.wPosition.x;
            if (m_handJoints.middleLittle.wPosition.y < lowestY) lowestX = m_handJoints.middleLittle.wPosition.y;
            if (m_handJoints.middleLittle.wPosition.x > highestX) highestX = m_handJoints.middleLittle.wPosition.x;
            if (m_handJoints.middleLittle.wPosition.y > highestY) highestY = m_handJoints.middleLittle.wPosition.y;
            if (m_handJoints.distalLittle.wPosition.x < lowestX) lowestX = m_handJoints.distalLittle.wPosition.x;
            if (m_handJoints.distalLittle.wPosition.y < lowestY) lowestX = m_handJoints.distalLittle.wPosition.y;
            if (m_handJoints.distalLittle.wPosition.x > highestX) highestX = m_handJoints.distalLittle.wPosition.x;
            if (m_handJoints.distalLittle.wPosition.y > highestY) highestY = m_handJoints.distalLittle.wPosition.y;
            // ____________

            m_ogreCamera->setPosition(Ogre::Vector3((lowestX + highestX)/2.0, (lowestY + highestY)/2.0, 0));
            isCameraUpdated = true;
        }
        else
        {
            memcpy(data, (unsigned char *)m_ogreTexturePtr->getBuffer()->lock(Ogre::HardwareBuffer::HBL_READ_ONLY), bufferSize);
            m_ogreTexturePtr->getBuffer()->unlock();
            isHandUpdated = true;
        }
    }
}
