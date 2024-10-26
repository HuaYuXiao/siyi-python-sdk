#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <QDateTime>
#include <QDir>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateFrame();
    void saveFrame();

private:
    Ui::MainWindow *ui;
    cv::VideoCapture cap;
    QTimer *timer;
    cv::Mat currentFrame;  // Store the current frame
};

#endif // MAINWINDOW_H
