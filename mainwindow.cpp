#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QImage m_image(640, 480, QImage::Format_RGB888);
    m_image.fill(Qt::black);

    this->updateOriginalDepth(m_image);
    this->updateOriginalRGB(m_image);
    this->updateOriginalIR(m_image);
    this->updateUserSkeleton(m_image);
    this->updateHandSegmented(m_image);
    this->updateHandSegmentedCloseUp(m_image);

    connect(&m_recognizer, SIGNAL(updateOriginalDepth(QImage)), this, SLOT(updateOriginalDepth(QImage)));
    connect(&m_recognizer, SIGNAL(updateOriginalRGB(QImage)), this, SLOT(updateOriginalRGB(QImage)));
    connect(&m_recognizer, SIGNAL(updateUserSkeleton(QImage)), this, SLOT(updateUserSkeleton(QImage)));
    connect(&m_recognizer, SIGNAL(updateHandSegmented(QImage)), this, SLOT(updateHandSegmented(QImage)));
    connect(&m_recognizer, SIGNAL(updateHandSegmentedCloseUp(QImage)), this, SLOT(updateHandSegmentedCloseUp(QImage)));




    ///////////////// TEMP
    connect(&m_recognizer, SIGNAL(updateRenderedHandSegmentationTEMP(QImage)), this, SLOT(updateRenderedHandSegmentationTEMP(QImage)));
    connect(&m_recognizer, SIGNAL(updateRenderedHandDepthTEMP(QImage)), this, SLOT(updateRenderedHandDepthTEMP(QImage)));





    this->m_ogreWidget = new OgreWidget(this);
    ui->handHypothesisLayout->addWidget(m_ogreWidget);
    this->m_recognizer.initRecognizer(m_ogreWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateOriginalDepth(QImage image)
{
    ui->original_depth->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateOriginalRGB(QImage image)
{
    ui->original_rgb->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateOriginalIR(QImage image)
{
    ui->original_ir->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateHandSegmented(QImage image)
{
    ui->hand_segmented->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateUserSkeleton(QImage image)
{
    ui->user_skeleton->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateHandSegmentedCloseUp(QImage image)
{
    ui->hand_segmented_closeup->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}


///////////////////////// TEMP
void MainWindow::updateRenderedHandSegmentationTEMP(QImage image)
{
    ui->original_ir->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}

void MainWindow::updateRenderedHandDepthTEMP(QImage image)
{
    ui->original_rgb->setPixmap(QPixmap::fromImage(image.scaled(320,240)));
}
