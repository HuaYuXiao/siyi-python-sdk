#include "mainwindow.h"
#include "ui_mainwindow.h"


using namespace std;


MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow), timer(new QTimer(this)) {
    ui->setupUi(this);

    // Open the RTSP stream
    cap.open("rtsp://192.168.1.222:8554/main.264");
    if (!cap.isOpened()) {
        QMessageBox::critical(this, "Error", "Could not open RTSP stream.");
        return;
    }

    connect(timer, &QTimer::timeout, this, &MainWindow::updateFrame);
    connect(ui->saveButton, &QPushButton::clicked, this, &MainWindow::saveFrame);
    timer->start(30); // Update every 30 ms
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::updateFrame() {
    if (cap.read(currentFrame)) {
        cv::cvtColor(currentFrame, currentFrame, cv::COLOR_BGR2RGB);
        QImage img((const uchar*)currentFrame.data, currentFrame.cols, currentFrame.rows, currentFrame.step[0], QImage::Format_RGB888);
        ui->videoLabel->setPixmap(QPixmap::fromImage(img).scaled(ui->videoLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

// Store the current frame
void MainWindow::saveFrame() {
    if (currentFrame.empty()) {
        QMessageBox::warning(this, "Error", "No frame available to save.");
        return;
    }

    // Ensure the directory exists
    QDir dir(QDir::homePath() + "/easondrone_ws/vision/siyi-ros-sdk/img/");
    if (!dir.exists()) {
        dir.mkpath(".");  // Create the directory if it does not exist
    }

    // Generate a filename based on the current date and time
    QString currentTime = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString file = currentTime + ".png";
    QFileInfo fi(dir, file);
    QString fullPath = fi.absoluteFilePath();

    // Save the frame as an image
    cv::cvtColor(currentFrame, currentFrame, cv::COLOR_RGB2BGR);
    cv::imwrite(fullPath.toStdString(), currentFrame);
}
