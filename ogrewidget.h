#ifndef __OGREWIDGET_H__
#define __OGREWIDGET_H__

#include <QTimer>
#include <QHash>

#include <QMetaType>
#include <OGRE/Ogre.h>
#include <QGLWidget>
#include <QDebug>

Q_DECLARE_METATYPE(Ogre::Root *);
Q_DECLARE_METATYPE(Ogre::RenderWindow *);
Q_DECLARE_METATYPE(Ogre::Camera *);
Q_DECLARE_METATYPE(Ogre::Viewport *);
Q_DECLARE_METATYPE(Ogre::SceneManager *);

class OgreWidget : public QGLWidget
{
    Q_OBJECT
public:
	OgreWidget(QObject *parent = 0);
	virtual ~OgreWidget(void);

	void initRender(QString pluginPath = QString("plugins.cfg"),
					QString configPath = QString(""),
					QString logPath = QString("log.cfg"),
					QString resPath = QString("resources.cfg"),
					QString renderSys = QString("OpenGL Rendering Subsystem"),
					QHash<QString, QString> renderSysConf = QHash<QString, QString>());
	void stopRender();

 protected:
  virtual void initializeGL();
  virtual void resizeGL(int, int);
  virtual void paintGL();
 
  void init(std::string, std::string, std::string);

  Ogre::Root *m_ogreRoot;
  Ogre::RenderWindow *m_ogreWindow;
  Ogre::Camera *m_ogreCamera;
  Ogre::Viewport *m_ogreViewport;
  Ogre::SceneManager *m_ogreSceneMgr;

private:
	bool m_isInit;
	QTimer m_timer;

signals:
  void ogreWidgetReady(Ogre::Root *, Ogre::RenderWindow *, Ogre::Camera *, Ogre::Viewport *, Ogre::SceneManager *);
};

#endif
