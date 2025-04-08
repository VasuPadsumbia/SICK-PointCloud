#include <application_base_modules/Camera.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <future>
#include <iostream>
#include <nlohmann/json.hpp>
#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QThread>
#include <QLineEdit>
#include <QFormLayout>
#include <QDialog>
#include <QTimer>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QtCore/QThread>
#include <CustomStreamBuffer.h>
//#include <application_base_modules/PLCData.h>
#define CLEAR_SCREEN "\033[2J\033[H"       // Clear screen and move cursor to top
#define MOVE_CURSOR_TOP "\033[H"           // Just move to top


namespace bip = boost::interprocess;
std::stringstream ss;

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZ>)
Q_DECLARE_METATYPE(Eigen::Vector3d)

class PositionData : public QThread
{
	Q_OBJECT
	Q_PROPERTY(QVariant contours READ getContours NOTIFY contoursChanged)
	Q_PROPERTY(QVariant centroid READ getCentroid NOTIFY centroidChanged)
	Q_PROPERTY(QVariant point_color READ getPointColor NOTIFY pointColorChanged)
public:
    pcl::PointCloud<pcl::PointXYZ> contours;
    Eigen::Vector3d centroid;
    Eigen::Vector3d point_color;

  explicit PositionData(QObject* parent = nullptr) : 
      QThread(parent), 
        camera(Camera::getInstance()), 
            current_id(0), stop_event(false), 
            op_connect_queue(bip::open_only, "op_connect_queue"),
            OPC_Connect(false), eps(0.05), min_samples(15) {
	  std::cout << "PositionData instance created." << std::endl;
      future_obj = exit_signal.get_future();
      camera.setDepthRange(std::make_tuple(0.530, 0.536));
    }

  ~PositionData() {
	  std::cout << "PositionData instance destroyed." << std::endl;
	  stopWorker();
	  camera.cleanup();
  }

  void startWorker()
  {
      std::lock_guard<std::mutex> lock(mutex);
      if (!isRunning()) {
          stop_event.store(false);
          exit_signal = std::promise<void>(); // Reinitialize exit_signal
          future_obj = exit_signal.get_future(); // Reinitialize future_obj
          start();
      }
      //stop_event.store(false);
      //exit_signal = std::promise<void>(); // Reinitialize exit_signal
      //future_obj = exit_signal.get_future(); // Reinitialize future_obj
      //start();
  }

  void stopWorker()
  {
      std::lock_guard<std::mutex> lock(mutex);
      if (isRunning()) {
          stop_event.store(true);
          exit_signal.set_value();
          wait();
      }
      //stop_event.store(true);
      //exit_signal.set_value();
      //wait();
  }

  QVariant getContours() const { return QVariant::fromValue(contours); }
  QVariant getCentroid() const { return QVariant::fromValue(centroid); }
  QVariant getPointColor() const { return QVariant::fromValue(point_color); }

  double getEps() const { return eps; }
  void setEps(double value) { eps = value; }

  int getMinSamples() const { return min_samples; }
  void setMinSamples(int value) { min_samples = value; }

  int getDelay() const { return delay; }
  void setDelay(int value) { delay = value; }

  Camera& getCamera() { return camera; }

signals:
    void updateCentroid(double x, double y, double z, int color);
    void updateCurrentId(int id);
    void updateTimestamp(std::string timestamp);
    void updateOPCConnect(bool connected);
    void contoursChanged();
    void centroidChanged();
    void pointColorChanged();

protected:
	void run() override {
        bip::message_queue mq(bip::open_only, "message_queue");
        worker(mq, op_connect_queue, future_obj);
	}

private:
  void worker(bip::message_queue& mq,
              bip::message_queue& op_connect_queue,
              //std::promise<void>& exit_signal,
              std::shared_future<void>& future_obj)
  {
    std::cout << "PositionData worker started." << "\n" << std::flush;
    std::cout << CLEAR_SCREEN;
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
       std::cout << MOVE_CURSOR_TOP;
      //std::cout << "PositionData worker running." << std::endl;
      try
      {
        bip::message_queue::size_type recvd_size;
        unsigned int                  priority;
        bool previous_OPC_Connect = OPC_Connect;
        op_connect_queue.try_receive(&OPC_Connect, sizeof(OPC_Connect), recvd_size, priority);
        if (previous_OPC_Connect != OPC_Connect) {
            std::cout << "OPC_Connect: " << OPC_Connect << "\n" << std::flush;
            emit updateOPCConnect(OPC_Connect);
            std::cout << CLEAR_SCREEN;
        }

        if (OPC_Connect)
        {
          if (&camera != nullptr)
          {
			auto time_start = std::chrono::high_resolution_clock::now();
            //std::cout << "Processing frame..." << "\n";
            camera.initializeStream();
            camera.processFrame(false, false, false, false, false, false);
            //std::cout << "Setting depth range..." << "\n";
            //camera.setDepthRange(std::make_tuple(0.554, 0.569));
            //camera.setDepthRange(std::make_tuple(0.530, 0.536));
            std::tie(contours, centroid, point_color) = camera.getContours(false, eps, min_samples);
            if (centroid(2) > 0.53)
            {
                current_id = current_id + 1;
                nlohmann::json json_obj;
                json_obj["x"] = centroid(0) * 1000;
                json_obj["y"] = centroid(1) * 1000;
                json_obj["z"] = centroid(2) * 1000;
                json_obj["color"] = getColor(point_color);
                json_obj["id"] = current_id;
                std::cout << "\rtime elapsed: " << (camera.getTimestampMS() - timestamp) << "ms \n" << std::flush;
                timestamp = camera.getTimestampMS();
                json_obj["timestamp"] = timestamp;// timestamp_value;
                //std::cout << "timestamp: " << timestamp << "\n";

                std::string json_data = json_obj.dump();
                std::cout << "\rjson_data: " << json_data << "\n" << std::flush;
                //std::cout << "Json data size: " << json_data.size() << "\n";
                if (json_data.size() > 4096)
                {
                    std::cerr << "Error: JSON data size exceeds message queue limit." << "\n" << std::flush;
                    continue;
                }
                mq.send(json_data.c_str(), json_data.size(), 0);
                std::string timestamp_str = printTimestamp();
				std::cout << "\rTimestamp: " << timestamp_str << "\n" << std::flush;
                if (current_id >= 1000)
                {
                    current_id = 0;
                }
                auto time_end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_seconds = time_end - time_start;
                //std::cout << "Position Data processing time: " << elapsed_seconds.count() << "s\n";

                emit updateCentroid(centroid(0) * 100, centroid(1) * 100, centroid(2) * 100, getColor(point_color));
                emit updateCurrentId(current_id);
                emit updateTimestamp(timestamp_str);
                emit contoursChanged();
                emit centroidChanged();
                emit pointColorChanged();

            }
            else
			{
				std::cout << "Centroid Z value is less than 0.5. i.e. No centroid found" << "\n" << std::flush;
			}
          }
          else
          {
            std::cerr << "Camera instance is null." << "\n" << std::flush;
          }
		  
          std::this_thread::sleep_for(std::chrono::milliseconds(delay));

        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "An error occurred: " << e.what() << "\n" << std::flush;
        break;
      }
      std::cout << std::flush;
    }
    std::cout << "PositionData worker stopped." << "\n" << std::flush;
  }

  int getColor(Eigen::Vector3d pointcolor)
  {
	  //std::cout << "Point color: " << pointcolor(0) << ", " << pointcolor(1) << ", " << pointcolor(2) << "\n" << std::flush;
	  int r = static_cast<int>(pointcolor(0));
	  int g = static_cast<int>(pointcolor(1));
	  int b = static_cast<int>(pointcolor(2));
	  if (r > 100 && g < 100 && b < 100)
	  {
		  return 1; // RED
	  }
	  else if (r < 100 && g > 100 && b < 100)
	  {
		  return 2; // GREEN
	  }
	  else if (r < 100 && g < 100 && b > 100)
	  {
		  return 3; // BLUE
	  }
	  else if (r > 100 && g > 100 && b < 100)
	  {
		  return 4; // YELLOW
	  }
      else
	  {
		  return 0; // UNKNOWN
	  }
  }

  Camera& camera = Camera::getInstance();
  int current_id = 0;
  std::atomic<bool> stop_event;
  bool OPC_Connect;
  uint64_t timestamp;
  double            eps         = 0.05;
  int               min_samples = 15;
  int32_t delay = 800;
  std::promise<void> exit_signal;
  std::shared_future<void> future_obj;
  bip::message_queue op_connect_queue;
  std::mutex mutex;

  std::string printTimestamp()
  {
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

      std::stringstream ss;
      ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
      ss << '.' << std::setw(3) << std::setfill('0') << ms.count();
      //std::cout << ss.str() << std::endl;
	  return ss.str();
  }
};

class ProcessFrameDialog : public QDialog
{
    Q_OBJECT

public:
    ProcessFrameDialog(Camera& camera, QWidget* parent = nullptr)
        : QDialog(parent), camera(camera)
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
        camera.initializeStream();
        camera.processFrame(imagePlot, pointCloudPlot, PCD, PLY, saveImage, savePointCloud);
    }

private:
    Camera& camera;
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
    SettingsDialog(PositionData* positionData, QWidget* parent = nullptr)
        : QDialog(parent), positionData(positionData)
    {
        QFormLayout* formLayout = new QFormLayout(this);

        QFont font;
        font.setPointSize(12); // Set the desired font size

        epsSpinBox = new QDoubleSpinBox(this);
        epsSpinBox->setRange(0.01, 1.0);
        epsSpinBox->setValue(positionData->getEps());
        epsSpinBox->setFont(font);
        formLayout->addRow("Epsilon:", epsSpinBox);

        minSamplesSpinBox = new QSpinBox(this);
        minSamplesSpinBox->setRange(1, 100);
        minSamplesSpinBox->setValue(positionData->getMinSamples());
        minSamplesSpinBox->setFont(font);
        formLayout->addRow("Min Samples:", minSamplesSpinBox);

        QPushButton* samplebutton = new QPushButton("Apply Sample Settings", this);
        samplebutton->setFont(font);
        formLayout->addWidget(samplebutton);

        connect(samplebutton, &QPushButton::clicked, this, &SettingsDialog::applySampleSettings);

        integrationTimeSpinBox = new QSpinBox(this);
        integrationTimeSpinBox->setRange(100, 10000);
        integrationTimeSpinBox->setValue(1250);
        integrationTimeSpinBox->setFont(font);
        formLayout->addRow("Integration Time:", integrationTimeSpinBox);

        integrationTimeColorSpinBox = new QSpinBox(this);
        integrationTimeColorSpinBox->setRange(100, 10000);
        integrationTimeColorSpinBox->setValue(6000);
        integrationTimeColorSpinBox->setFont(font);
        formLayout->addRow("Integration Time Color:", integrationTimeColorSpinBox);
       
        QPushButton* ITButton = new QPushButton("Apply Integration Time", this);
        ITButton->setFont(font);
        formLayout->addWidget(ITButton);

        connect(ITButton, &QPushButton::clicked, this, &SettingsDialog::applyIntegrationSettings);

        minDepthSpinBox = new QDoubleSpinBox(this);
        minDepthSpinBox->setRange(0.0000, 10.0000);
        minDepthSpinBox->setDecimals(4); // Allow four decimal place
        minDepthSpinBox->setValue(0.5300);
        minDepthSpinBox->setFont(font);
        formLayout->addRow("Min Depth:", minDepthSpinBox);

        maxDepthSpinBox = new QDoubleSpinBox(this);
        maxDepthSpinBox->setRange(0.0000, 10.0000);
        maxDepthSpinBox->setDecimals(4); // Allow four decimal place
        maxDepthSpinBox->setValue(0.5360);
        maxDepthSpinBox->setFont(font);
        formLayout->addRow("Max Depth:", maxDepthSpinBox);

        QPushButton* applyButton = new QPushButton("Apply Depth Range", this);
        applyButton->setFont(font);
        formLayout->addWidget(applyButton);

        connect(applyButton, &QPushButton::clicked, this, &SettingsDialog::applyDepthSettings);

        QPushButton* processFrameButton = new QPushButton("Process Frame", this);
        processFrameButton->setFont(font);
        formLayout->addWidget(processFrameButton);

        connect(processFrameButton, &QPushButton::clicked, this, &SettingsDialog::openProcessFrameDialog);
    }

private slots:
    void applySettings()
    {
        positionData->setEps(epsSpinBox->value());
        positionData->setMinSamples(minSamplesSpinBox->value());
        positionData->getCamera().setIntegrationTime(integrationTimeSpinBox->value(), integrationTimeColorSpinBox->value());
        positionData->getCamera().setDepthRange(std::make_tuple(minDepthSpinBox->value(), maxDepthSpinBox->value()));
        QMessageBox::information(this, "Settings Applied", "Settings have been applied successfully.");
    }

	void applySampleSettings()
	{
        positionData->setEps(epsSpinBox->value());
        positionData->setMinSamples(minSamplesSpinBox->value());
		QMessageBox::information(this, "Settings Applied", "Sampling Settings have been applied successfully.");
	}

	void applyIntegrationSettings()
	{
		positionData->getCamera().setIntegrationTime(integrationTimeSpinBox->value(), integrationTimeColorSpinBox->value());
		QMessageBox::information(this, "Settings Applied", "Integration Settings have been applied successfully.");
	}

	void applyDepthSettings()
	{
		positionData->getCamera().setDepthRange(std::make_tuple(minDepthSpinBox->value(), maxDepthSpinBox->value()));
		QMessageBox::information(this, "Settings Applied", "Depth Settings have been applied successfully.");
	}

    void openProcessFrameDialog()
    {
        ProcessFrameDialog processFrameDialog(positionData->getCamera(), this);
        processFrameDialog.exec();
    }

private:
    PositionData* positionData;
    QDoubleSpinBox* epsSpinBox;
    QSpinBox* minSamplesSpinBox;
    QSpinBox* integrationTimeSpinBox;
    QSpinBox* integrationTimeColorSpinBox;
    QDoubleSpinBox* minDepthSpinBox;
    QDoubleSpinBox* maxDepthSpinBox;
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr)
        : QWidget(parent), positionData(new PositionData(this))
    {
        //QHBoxLayout* mainLayout = new QHBoxLayout(this); // Main horizontal layout
        QVBoxLayout* leftLayout = new QVBoxLayout(this); // Vertical layout for buttons and other UI elements

        QFont font;
        font.setPointSize(12); // Set the desired font size

        QPushButton* startButton = new QPushButton("Start Worker", this);
        startButton->setFont(font);
        QPushButton* stopButton = new QPushButton("Stop Worker", this);
        stopButton->setFont(font);
        QPushButton* settingsButton = new QPushButton("Settings", this);
        settingsButton->setFont(font);
        centroidLabel = new QLabel("Centroid: (0, 0, 0)", this);
        centroidLabel->setFont(font);
        colorLabel = new QLabel("Color: UNKNOWN", this);
        colorLabel->setFont(font);
        currentIdLabel = new QLabel("Current ID: 0", this);
        currentIdLabel->setFont(font);
        timestampLabel = new QLabel("Timestamp: 0", this);
        timestampLabel->setFont(font);
        opcConnectLamp = new QLabel(this);
        opcConnectLamp->setFixedSize(20, 20);
        opcConnectLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        opcConnectLamp->setFont(font);

        QLabel* opcConnectStatusLabel = new QLabel("OPC Connect Status:", this);
        opcConnectStatusLabel->setFont(font);

        delaySpinBox = new QSpinBox(this);
        delaySpinBox->setRange(0, 5000); // Set the range for delay in milliseconds
        delaySpinBox->setValue(800); // Set the default value
        delaySpinBox->setFont(font);
        QLabel* delayLabel = new QLabel("Delay (ms):", this);
        delayLabel->setFont(font);

        //consoleOutput = new QTextEdit(this);
        //consoleOutput->setFont(font);
        //consoleOutput->setReadOnly(true);
        //consoleOutput->setStyleSheet("background-color: black; color: white;"); // Set background and text color
        //consoleOutput->setTextInteractionFlags(Qt::TextBrowserInteraction);
        //consoleOutput->setAcceptRichText(true); // Enable rich text formatting

        leftLayout->addWidget(startButton);
        leftLayout->addWidget(stopButton);
        leftLayout->addWidget(settingsButton);
        leftLayout->addWidget(centroidLabel);
        leftLayout->addWidget(colorLabel);
        leftLayout->addWidget(currentIdLabel);
        leftLayout->addWidget(timestampLabel);
        leftLayout->addWidget(opcConnectStatusLabel);
        leftLayout->addWidget(opcConnectLamp);
        leftLayout->addWidget(delayLabel);
        leftLayout->addWidget(delaySpinBox);

        // Add the left layout and console output to the main layout
        //mainLayout->addLayout(leftLayout);
        //mainLayout->addWidget(consoleOutput);

        connect(startButton, &QPushButton::clicked, positionData, &PositionData::startWorker);
        connect(stopButton, &QPushButton::clicked, positionData, &PositionData::stopWorker);
        connect(settingsButton, &QPushButton::clicked, this, &MainWindow::openSettingsDialog);
        connect(delaySpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onDelayChanged);
        connect(positionData, &PositionData::updateCentroid, this, &MainWindow::onUpdateCentroid);
        connect(positionData, &PositionData::updateCurrentId, this, &MainWindow::onUpdateCurrentId);
        connect(positionData, &PositionData::updateTimestamp, this, &MainWindow::onUpdateTimestamp);
        connect(positionData, &PositionData::updateOPCConnect, this, &MainWindow::onUpdateOPCConnect);

        // Redirect std::cout to the QTextEdit widget
        //customStreamBuffer = new CustomStreamBuffer(consoleOutput);
        //std::cout.rdbuf(customStreamBuffer);
    }

    //~MainWindow()
    //{
    //    delete customStreamBuffer;
    //}

private slots:
    void onUpdateCentroid(double x, double y, double z, int color)
    {
        centroidLabel->setText(QString("Centroid: (%1, %2, %3)").arg(x).arg(y).arg(z));
        QString colorStr;
        switch (color)
        {
        case 1:
            colorStr = "RED";
            break;
        case 2:
            colorStr = "GREEN";
            break;
        case 3:
            colorStr = "BLUE";
            break;
        case 4:
            colorStr = "YELLOW";
            break;
        default:
            colorStr = "UNKNOWN";
            break;
        }
        colorLabel->setText(QString("Color: %1").arg(colorStr));
    }

    void onUpdateCurrentId(int id)
    {
        currentIdLabel->setText(QString("Current ID: %1").arg(id));
    }

    void onUpdateTimestamp(std::string timestamp)
    {
		timestampLabel->setText(QString("Timestamp: %1").arg(QString::fromStdString(timestamp)));
        //QMetaObject::invokeMethod(this, [this, timestamp]() {
        //    timestampLabel->setText(QString("Timestamp: %1").arg(QString::fromStdString(timestamp)));
        //    std::cout << "Updated timestampLabel with value: " << timestampLabel->text().toStdString() << std::endl; // Debug statement
        //    }, Qt::QueuedConnection);
    }

    void onUpdateOPCConnect(bool connected)
    {
        if (connected)
        {
            opcConnectLamp->setStyleSheet("background-color: green; border-radius: 15px;");
        }
        else
        {
            opcConnectLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        }
    }

    void openSettingsDialog()
    {
        if (positionData->isRunning())
        {
            QMessageBox::warning(this, "Worker Running", "Please stop the worker before changing settings.");
            return;
        }

        SettingsDialog settingsDialog(positionData, this);
        settingsDialog.exec();
    }

    void onDelayChanged(int value)
    {
        positionData->setDelay(value);
    }

private:
    PositionData* positionData;
    QLabel* centroidLabel;
    QLabel* colorLabel;
    QLabel* currentIdLabel;
    QLabel* timestampLabel;
    QLabel* opcConnectLamp;
    QSpinBox* delaySpinBox;
    //QTextEdit* consoleOutput;
    //CustomStreamBuffer* customStreamBuffer;
};

int main(int argc, char* argv[])
{
    // Register std::string with Qt's meta-object system
    qRegisterMetaType<std::string>("std::string");

	QApplication app(argc, argv);
	MainWindow window;
	window.setWindowTitle("Position Data");
	window.resize(400, 300);
	window.show();
	// Create a message queue for communication
	//bip::message_queue mq(bip::open_or_create, "message_queue", 100, sizeof(std::string));
	// Create a message queue for OPC connection status

  //std::promise<void> exit_signal;
  //std::shared_future<void> future_obj = exit_signal.get_future();
  //bip::message_queue mq(bip::open_only, "message_queue");
  //bip::message_queue       op_connect_queue(bip::open_only, "op_connect_queue");
  //
  //PositionData pos;
  //pos.worker(mq, op_connect_queue, exit_signal, future_obj);

  return app.exec();
}

#include "PositionData.moc"