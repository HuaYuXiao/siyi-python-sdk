#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QMessageBox>
#include <QLabel>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QImage>

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

private:
    Ui::MainWindow *ui;
    cv::VideoCapture cap;
    QTimer *timer;
    void displayImage(const cv::Mat &image);
};

#endif // MAINWINDOW_H
