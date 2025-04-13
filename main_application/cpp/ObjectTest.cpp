#include <QApplication>
#include <QThread>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QSpinBox>
#include <QFormLayout>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QPlainTextEdit>
#include <QCheckBox>
#include <QTextCursor>
#include <QFont>
#include <QMessageBox>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>
#include <mutex>
#include <future>
#include <iomanip>
#include <sstream>
#include <application_base_modules/Camera.h>
#include "DataProcessor.h"

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZ>)
Q_DECLARE_METATYPE(Eigen::Vector3d)


// --------- ObjectProcessor Thread ---------
class ObjectProcessor : public QThread
{
    Q_OBJECT
public:
    explicit ObjectProcessor(QObject* parent)
    : QThread(parent),
      camera(Camera::getInstance()),
      stop_event(false),
      eps(0.05),
      min_samples(15),
      delay(800),
      min_depth(0.530),
      max_depth(0.536),
      integration_time(1250),
      integration_time_color(6000),
      frames_to_average(50),
      dataProcessor(50)
{
    future_obj = exit_signal.get_future();
}

    ~ObjectProcessor()
    {
        stopWorker();
        camera.cleanup();
    }

    void startWorker()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!isRunning())
        {
            stop_event.store(false);
            exit_signal = std::promise<void>();
            future_obj = exit_signal.get_future();
            start();
        }
    }

    void stopWorker()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (isRunning())
        {
            stop_event.store(true);
            exit_signal.set_value();
            wait();
        }
    }

    Camera& getCamera()
    {
        return camera;
    }
    double getEps() const { return eps; }
    void setEps(double value) { eps = value; }

    int getMinSamples() const { return min_samples; }
    void setMinSamples(int value) { min_samples = value; }

    double getMinDepth() const { return min_depth; }
    void setMinDepth(double value) { min_depth = value; }

    double getMaxDepth() const { return max_depth; }
    void setMaxDepth(double value) { max_depth = value; }

    int getIntegrationTime() const { return integration_time; }
    void setIntegrationTime(int value) { integration_time = value; }

    int getIntegrationTimeColor() const { return integration_time_color; }
    void setIntegrationTimeColor(int value) { integration_time_color = value; }

    int getDelay() const { return delay; }
    void setDelay(int value) { delay = value; }

    int getFramesToAverage() const { return frames_to_average; }
    void setFramesToAverage(int value)
    {
        frames_to_average = value;
        dataProcessor.setFramesToAverage(value);
    }

signals:
    void updateConsole(QString message);
    void updateMeanCentroid(QString meanX, QString meanY, QString meanZ, QString stdDevX, QString stdDevY, QString stdDevZ);

protected:
    void run() override
    {
        worker(future_obj);
    }

private:
    void worker(std::shared_future<void>& future_obj)
    {
        camera.initializeStream();
        camera.setDepthRange(std::make_tuple(min_depth, max_depth));

        while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
        {
            auto start = std::chrono::steady_clock::now();

            camera.processFrame(false, false, false, false, false, false);
            auto [contours, centroid, point_color] = camera.getContours(false, eps, min_samples);

            auto end = std::chrono::steady_clock::now();
            double time_taken_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (centroid(2) > 0.53)
            {
                QString timestamp = getTimestamp();
                QString colorName = colorToString(getColor(point_color));

                QString message = QString("Centroid: (%1, %2, %3) | Color: %4 | TimeElapsed: %5 s | Timestamp: %6")
                    .arg(centroid(0)*100, 0, 'f', 3)
                    .arg(centroid(1)*100, 0, 'f', 3)
                    .arg(centroid(2)*100, 0, 'f', 3)
                    .arg(colorName)
                    .arg(time_taken_ms/1000, 0, 'f', 4)
                    .arg(timestamp);


                emit updateConsole(message);

                dataProcessor.addCentroid(centroid*100);

                Eigen::Vector3d mean = dataProcessor.getMean();
                Eigen::Vector3d stddev = dataProcessor.getStdDev();

                emit updateMeanCentroid(
                    QString::number(mean(0), 'f', 3),
                    QString::number(mean(1), 'f', 3),
                    QString::number(mean(2), 'f', 3),
                    QString::number(stddev(0), 'f', 3),
                    QString::number(stddev(1), 'f', 3),
                    QString::number(stddev(2), 'f', 3)
                );
            }
            else
            {
                emit updateConsole("No valid centroid found.");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    }

    int getColor(const Eigen::Vector3d& color)
    {
        int r = static_cast<int>(color(0));
        int g = static_cast<int>(color(1));
        int b = static_cast<int>(color(2));

        if (r > 100 && g < 100 && b < 100) return 1; // RED
        if (r < 100 && g > 100 && b < 100) return 2; // GREEN
        if (r < 100 && g < 100 && b > 100) return 3; // BLUE
        if (r > 100 && g > 100 && b < 100) return 4; // YELLOW
        return 0;
    }

    QString colorToString(int color)
    {
        switch (color)
        {
        case 1: return "RED";
        case 2: return "GREEN";
        case 3: return "BLUE";
        case 4: return "YELLOW";
        default: return "UNKNOWN";
        }
    }

    QString getTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setw(3) << std::setfill('0') << ms.count();

        return QString::fromStdString(ss.str());
    }

    Camera& camera;
    std::atomic<bool> stop_event;
    std::promise<void> exit_signal;
    std::shared_future<void> future_obj;
    std::mutex mutex;

    double eps;
    int min_samples;
    double min_depth;
    double max_depth;
    int integration_time;
    int integration_time_color;
    int delay;
    int frames_to_average;

    DataProcessor dataProcessor;
};

class ProcessFrameDialog : public QDialog
{
    Q_OBJECT

public:
    ProcessFrameDialog(ObjectProcessor* processor, QWidget* parent = nullptr)
        : QDialog(parent), processor(processor)  
    {
        QVBoxLayout* layout = new QVBoxLayout(this);

        imagePlotCheckBox = new QCheckBox("Image Plot", this);
        pointCloudPlotCheckBox = new QCheckBox("Point Cloud Plot", this);
        PCDCheckBox = new QCheckBox("PCD", this);
        PLYCheckBox = new QCheckBox("PLY", this);
        saveImageCheckBox = new QCheckBox("Save Image", this);
        savePointCloudCheckBox = new QCheckBox("Save Point Cloud", this);

        QPushButton* processButton = new QPushButton("Process Frame", this);

        layout->addWidget(imagePlotCheckBox);
        layout->addWidget(pointCloudPlotCheckBox);
        layout->addWidget(PCDCheckBox);
        layout->addWidget(PLYCheckBox);
        layout->addWidget(saveImageCheckBox);
        layout->addWidget(savePointCloudCheckBox);
        layout->addWidget(processButton);

        connect(processButton, &QPushButton::clicked, this, &ProcessFrameDialog::Frame);
    }

private slots:
    void Frame()
    {
        bool imagePlot = imagePlotCheckBox->isChecked();
        bool pointCloudPlot = pointCloudPlotCheckBox->isChecked();
        bool PCD = PCDCheckBox->isChecked();
        bool PLY = PLYCheckBox->isChecked();
        bool saveImage = saveImageCheckBox->isChecked();
        bool savePointCloud = savePointCloudCheckBox->isChecked();
        processor->getCamera().initializeStream();
        processor->getCamera().processFrame(imagePlot, pointCloudPlot, PCD, PLY, saveImage, savePointCloud);
    }

private:
	ObjectProcessor* processor;
    QCheckBox* imagePlotCheckBox;
    QCheckBox* pointCloudPlotCheckBox;
    QCheckBox* PCDCheckBox;
    QCheckBox* PLYCheckBox;
    QCheckBox* saveImageCheckBox;
    QCheckBox* savePointCloudCheckBox;
};


class SettingsDialog : public QDialog
{
    Q_OBJECT
public:
    SettingsDialog(ObjectProcessor* processor, QWidget* parent = nullptr)
        : QDialog(parent), processor(processor)
    {
        QVBoxLayout* layout = new QVBoxLayout(this);
        QFormLayout* formLayout = new QFormLayout();

        epsSpin = new QDoubleSpinBox(this);
        epsSpin->setRange(0.01, 1.0);
        epsSpin->setSingleStep(0.01);
        epsSpin->setValue(processor->getEps());

        minSamplesSpin = new QSpinBox(this);
        minSamplesSpin->setRange(1, 100);
        minSamplesSpin->setValue(processor->getMinSamples());

        integrationTimeSpin = new QSpinBox(this);
        integrationTimeSpin->setRange(1, 10000);
        integrationTimeSpin->setValue(processor->getIntegrationTime());

        integrationTimeColorSpin = new QSpinBox(this);
        integrationTimeColorSpin->setRange(1, 10000);
        integrationTimeColorSpin->setValue(processor->getIntegrationTimeColor());

        minDepthSpin = new QDoubleSpinBox(this);
        minDepthSpin->setDecimals(3);
        minDepthSpin->setRange(0.0, 10.0);
        minDepthSpin->setSingleStep(0.001);
        minDepthSpin->setValue(processor->getMinDepth());

        maxDepthSpin = new QDoubleSpinBox(this);
        maxDepthSpin->setDecimals(3);
        maxDepthSpin->setRange(0.0, 10.0);
        maxDepthSpin->setSingleStep(0.001);
        maxDepthSpin->setValue(processor->getMaxDepth());

        QPushButton* applySampleButton = new QPushButton("Apply Sample Settings", this);
        QPushButton* applyIntegrationButton = new QPushButton("Apply Integration Time", this);
        QPushButton* applyDepthButton = new QPushButton("Apply Depth Range", this);

        formLayout->addRow("Epsilon:", epsSpin);
        formLayout->addRow("Min Samples:", minSamplesSpin);
        layout->addLayout(formLayout);
        layout->addWidget(applySampleButton);

        formLayout = new QFormLayout();
        formLayout->addRow("Integration Time:", integrationTimeSpin);
        formLayout->addRow("Integration Time Color:", integrationTimeColorSpin);
        layout->addLayout(formLayout);
        layout->addWidget(applyIntegrationButton);

        formLayout = new QFormLayout();
        formLayout->addRow("Min Depth:", minDepthSpin);
        formLayout->addRow("Max Depth:", maxDepthSpin);
        layout->addLayout(formLayout);
        layout->addWidget(applyDepthButton);

        connect(applySampleButton, &QPushButton::clicked, this, &SettingsDialog::applySampleSettings);
        connect(applyIntegrationButton, &QPushButton::clicked, this, &SettingsDialog::applyIntegrationSettings);
        connect(applyDepthButton, &QPushButton::clicked, this, &SettingsDialog::applyDepthRange);
    }

private slots:
    void applySampleSettings()
    {
        processor->setEps(epsSpin->value());
        processor->setMinSamples(minSamplesSpin->value());
    }

    void applyIntegrationSettings()
    {
        processor->setIntegrationTime(integrationTimeSpin->value());
        processor->setIntegrationTimeColor(integrationTimeColorSpin->value());
    }

    void applyDepthRange()
    {
        processor->setMinDepth(minDepthSpin->value());
        processor->setMaxDepth(maxDepthSpin->value());
    }

private:
    ObjectProcessor* processor;

    QDoubleSpinBox* epsSpin;
    QSpinBox* minSamplesSpin;
    QSpinBox* integrationTimeSpin;
    QSpinBox* integrationTimeColorSpin;
    QDoubleSpinBox* minDepthSpin;
    QDoubleSpinBox* maxDepthSpin;
};

class MainWindow : public QWidget
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        QVBoxLayout* layout = new QVBoxLayout(this);

        processor = new ObjectProcessor(this);

        QFont font("Arial", 12);

        startButton = new QPushButton("Start Worker", this);
        stopButton = new QPushButton("Stop Worker", this);
        settingsButton = new QPushButton("Settings", this);
        processFrameButton = new QPushButton("Process Frame", this);

        QLabel* avgLabel = new QLabel("Average Centroid:", this);
        meanXLabel = new QLabel("X: 0.000", this);
        meanYLabel = new QLabel("Y: 0.000", this);
        meanZLabel = new QLabel("Z: 0.000", this);

        delayLabel = new QLabel("Delay (ms):", this);
        delaySpinBox = new QSpinBox(this);
        delaySpinBox->setRange(0, 5000);
        delaySpinBox->setValue(processor->getDelay());

        framesToAverageLabel = new QLabel("Frames to Average:", this);
        framesToAverageSpinBox = new QSpinBox(this);
        framesToAverageSpinBox->setRange(1, 1000);
        framesToAverageSpinBox->setValue(processor->getFramesToAverage());

        consoleOutput = new QPlainTextEdit(this);
        consoleOutput->setReadOnly(true);
        consoleOutput->setMaximumBlockCount(1); // Single line console output
        consoleOutput->setStyleSheet("background-color: black; color: white;");
        consoleOutput->setFont(QFont("Courier", 14));

        layout->addWidget(startButton);
        layout->addWidget(stopButton);
        layout->addWidget(settingsButton);
        layout->addWidget(processFrameButton);
        layout->addWidget(avgLabel);
        layout->addWidget(meanXLabel);
        layout->addWidget(meanYLabel);
        layout->addWidget(meanZLabel);
        layout->addWidget(delayLabel);
        layout->addWidget(delaySpinBox);
        layout->addWidget(framesToAverageLabel);
        layout->addWidget(framesToAverageSpinBox);
        layout->addWidget(consoleOutput);

        connect(startButton, &QPushButton::clicked, processor, &ObjectProcessor::startWorker);
        connect(stopButton, &QPushButton::clicked, processor, &ObjectProcessor::stopWorker);
        connect(settingsButton, &QPushButton::clicked, this, &MainWindow::openSettings);
        connect(processFrameButton, &QPushButton::clicked, this, &MainWindow::openProcessFrameDialog);
        connect(delaySpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onDelayChanged);
        connect(framesToAverageSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onFramesToAverageChanged);
        connect(processor, &ObjectProcessor::updateConsole, this, &MainWindow::updateConsole);
        connect(processor, &ObjectProcessor::updateMeanCentroid, this, &MainWindow::updateMeanCentroid);
    }

private slots:
    void updateConsole(const QString& message)
    {
        consoleOutput->setPlainText(message);
    }

    void updateMeanCentroid(QString meanX, QString meanY, QString meanZ, QString stdDevX, QString stdDevY, QString stdDevZ)
    {
        meanXLabel->setText("X: " + meanX + " | StdDev X: " + stdDevX);
        meanYLabel->setText("Y: " + meanY + " | StdDev Y: " + stdDevY);
        meanZLabel->setText("Z: " + meanZ + " | StdDev Z: " + stdDevZ);
    }

    void openSettings()
    {
        if (processor->isRunning())
        {
            QMessageBox::warning(this, "Running", "Stop the worker before changing settings.");
            return;
        }
        SettingsDialog dialog(processor, this);
        dialog.exec();
    }

    void openProcessFrameDialog()
    {
        ProcessFrameDialog dialog(processor, this);
        dialog.exec();
    }

    void onDelayChanged(int value)
    {
        processor->setDelay(value);
    }

    void onFramesToAverageChanged(int value)
    {
        processor->setFramesToAverage(value);
    }

private:
    ObjectProcessor* processor;
    QPushButton* startButton;
    QPushButton* stopButton;
    QPushButton* settingsButton;
    QPushButton* processFrameButton;
    QLabel* delayLabel;
    QSpinBox* delaySpinBox;
    QLabel* framesToAverageLabel;
    QSpinBox* framesToAverageSpinBox;
    QPlainTextEdit* consoleOutput;
    QLabel* meanXLabel;
    QLabel* meanYLabel;
    QLabel* meanZLabel;
};


int main(int argc, char* argv[])
{
    qRegisterMetaType<std::string>("std::string");
    QApplication app(argc, argv);

    MainWindow window;
    window.setWindowTitle("Object Processor");
    window.resize(500, 650);
    window.show();

    return app.exec();
}

#include "ObjectTest.moc"
