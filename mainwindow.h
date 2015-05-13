#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "ogrewidget.h"
#include "api/recognizer.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    OgreWidget *m_ogreWidget;
    Recognizer m_recognizer;

private slots:
    void updateOriginalDepth(QImage);
    void updateOriginalRGB(QImage);
    void updateOriginalIR(QImage);
    void updateHandSegmented(QImage);
    void updateUserSkeleton(QImage);
    void updateHandSegmentedCloseUp(QImage);

    void updateRenderedHandSegmentationTEMP(QImage);
    void updateRenderedHandDepthTEMP(QImage);
};

#endif // MAINWINDOW_H
