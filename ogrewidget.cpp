#include "OgreWidget.h"


OgreWidget::OgreWidget(QObject *parent) : QGLWidget((QWidget *)parent), m_ogreWindow(NULL)
{
	this->m_isInit = false;
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(repaint()));
}

OgreWidget::~OgreWidget(void)
{
	destroy();
}

void OgreWidget::initRender(QString pluginPath,
							QString configPath,
							QString logPath,
							QString resPath,
							QString renderSys,
							QHash<QString, QString> renderSysConf)
{
	if (!this->m_isInit)
	{
		this->m_ogreRoot = new Ogre::Root(pluginPath.toStdString(), configPath.toStdString(), logPath.toStdString());

		// setup a renderer
		Ogre::RenderSystemList::const_iterator renderers = m_ogreRoot->getAvailableRenderers().begin();
		while (renderers != m_ogreRoot->getAvailableRenderers().end())
		{
			Ogre::String rName = (*renderers)->getName();
			if (rName == renderSys.toStdString())
				break;
			renderers++;
		}
		m_ogreRoot->setRenderSystem((Ogre::RenderSystem *)(*renderers));

		QHash<QString, QString> m_renderOptions = renderSysConf;
		if (m_renderOptions.empty())
		{
			m_renderOptions.insert(QString("Video Mode"), QString("%1x%2").arg(this->width()).arg(this->height()));
			m_renderOptions.insert(QString("Full Screen"), QString("No"));
		}

		QHashIterator<QString, QString> i(m_renderOptions);
		while (i.hasNext())
		{
			i.next();
			m_ogreRoot->getRenderSystem()->setConfigOption(i.key().toStdString(), i.value().toStdString());
		
		}
		m_ogreRoot->saveConfig();
		m_ogreRoot->initialise(false); // don't create a window

		// Load resource paths from config file
		Ogre::ConfigFile cf;
		cf.load(resPath.toStdString());
		// Go through all sections & settings in the file
		Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
		QString secName, typeName, archName;
		while (seci.hasMoreElements())
		{
			secName = QString::fromStdString((std::string)seci.peekNextKey());
			Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
			Ogre::ConfigFile::SettingsMultiMap::iterator i;
			for (i = settings->begin(); i != settings->end(); ++i)
			{
				typeName = QString::fromStdString((std::string)i->first);
				archName = QString::fromStdString((std::string)i->second);
				Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName.toStdString(), typeName.toStdString(), secName.toStdString());
			}
		}

		m_timer.start(10);

		this->m_isInit = true;
	}
}

void OgreWidget::stopRender()
{
	if (this->m_isInit)
	{
	if (m_timer.isActive())
		m_timer.stop();

		m_ogreRoot->shutdown();
		delete m_ogreRoot;

		this->m_isInit = false;
	}
}

void OgreWidget::initializeGL()
{
	if (this->m_isInit)
	{
		//== Creating and Acquiring Ogre Window ==//
		// Get the parameters of the window QT created
		Ogre::String winHandle;
		winHandle += Ogre::StringConverter::toString((unsigned long)(this->parentWidget()->winId()));

		Ogre::NameValuePairList params;
		// code for Windows and Linux
		params["parentWindowHandle"] = winHandle;
		m_ogreWindow = m_ogreRoot->createRenderWindow("QOgreWidget_RenderWindow",
													  this->width(),
													  this->height(),
													  false,
													  &params);
		m_ogreWindow->setActive(true);
		WId ogreWinId = 0x0;
		m_ogreWindow->getCustomAttribute("WINDOW", &ogreWinId);

		// bug fix, extract geometry
		QRect geo = this->frameGeometry();
		// create new window
		this->create(ogreWinId);
		this->setGeometry(geo);

		setAutoBufferSwap(false);
		setAttribute(Qt::WA_PaintOnScreen, true);
		setAttribute(Qt::WA_NoBackground);
	 
		//== Ogre Initialization ==//
		m_ogreSceneMgr = m_ogreRoot->createSceneManager(Ogre::ST_GENERIC);
		m_ogreCamera = m_ogreSceneMgr->createCamera("QOgreWidget_Cam");

		m_ogreViewport = m_ogreWindow->addViewport(m_ogreCamera);
		m_ogreViewport->setBackgroundColour(Ogre::ColourValue(0,0,0));
		m_ogreCamera->setAspectRatio(Ogre::Real(m_ogreViewport->getActualWidth()) / Ogre::Real(m_ogreViewport->getActualHeight()));

		Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

		resizeGL(this->width(), this->height());

		emit ogreWidgetReady(this->m_ogreRoot, this->m_ogreWindow, this->m_ogreCamera, this->m_ogreViewport, this->m_ogreSceneMgr);
	}
}

void OgreWidget::paintGL()
{
	if (this->m_isInit)
	{
		// Be sure to call "OgreWidget->repaint();"
		swapBuffers();
        Sleep(500);
		m_ogreRoot->renderOneFrame();
	}
}

void OgreWidget::resizeGL(int width, int height)
{
	if (this->m_isInit)
	{
        m_ogreWindow->reposition(this->pos().x(), this->pos().y());
		m_ogreWindow->resize(width, height);
		paintGL();
	}
}
