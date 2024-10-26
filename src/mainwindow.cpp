#include "mainwindow.h"
#include "ui_mainwindow.h"

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
    timer->start(30); // Update every 30 ms
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::updateFrame() {
    cv::Mat frame;
    if (cap.read(frame)) {
        displayImage(frame);
    }
}

void MainWindow::displayImage(const cv::Mat &image) {
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    QImage img((const uchar*)image.data, image.cols, image.rows, image.step[0], QImage::Format_RGB888);
//    ui->label->setPixmap(QPixmap::fromImage(img));
    ui->videoLabel->setPixmap(QPixmap::fromImage(img).scaled(ui->videoLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

