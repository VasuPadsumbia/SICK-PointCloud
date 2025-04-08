//#include <application_base_modules/Camera.h>
#include <application_base_modules/PLCData.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <future>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QThread>
#include <QMessageBox>
#include <QFormLayout>
#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QtCore/QThread>
#include <CustomStreamBuffer.h>
#define CLEAR_SCREEN "\033[2J\033[H"       // Clear screen and move cursor to top
#define MOVE_CURSOR_TOP "\033[H"           // Just move to top

// Existing code
std::stringstream ss;
namespace bip = boost::interprocess;

class Communication : public QThread
{
    Q_OBJECT
		
public:
  explicit Communication(QObject* parent = nullptr) : 
      QThread(parent), 
      stop_event(false), 
      opc(deviceIpAddr, port),
      op_connect_queue(bip::open_only, "op_connect_queue")
  {
      std::cout << "Communication instance created." << "\n";
  }

  ~Communication()
  {
      std::cout << "Communication instance destroyed." << "\n";
      stopWorker();
      opc.disconnect();
  }

  void startWorker()
  {
      std::lock_guard<std::mutex> lock(mutex);
      if (!isRunning()) {
           stop_event.store(false);
           exit_signal = std::promise<void>(); // Reinitialize exit_signal
           future_obj = exit_signal.get_future(); // Reinitialize future_obj
           start();
           std::cout << "Communication worker started." << "\n";
        }
  }

  void stopWorker()
  {
      std::lock_guard<std::mutex> lock(mutex);
      if (isRunning()) {
          stop_event.store(true);
          opc.disconnect();
          emit updateConnectionStatus(opc.isConnected);
          exit_signal.set_value();
          wait();
          // Set OPC_Connect to false and send the message to the op_connect_queue
          OPC_Connect = false;
          op_connect_queue.send(&OPC_Connect, sizeof(OPC_Connect), 0);
          emit updateOPCConnectStatus(OPC_Connect);
          std::cout << "Communication worker stopped." << "\n";
      }
  }
  
  void setConfig(const std::string& ip, int port, const std::string& baseNodeId)
  {
      deviceIpAddr = ip;
      this->port = port;
      base_node_id = baseNodeId;
      connect_node_id = base_node_id + ".GVL.OPC_Connect";
      newpos_node_id = base_node_id + ".OPC_UA.newPos";
      time_node_id = base_node_id + ".OPC_UA.TimeStamp";
      opc = PLCData(deviceIpAddr, this->port);
  }

signals:
    void updateConnectionStatus(bool isConnected);
    void updateOPCConnectStatus(bool OPC_Connect);
    void errorOccurred(const QString& error);

protected:
    void run() override
    {
        bip::message_queue mq(bip::open_only, "message_queue");
        
        worker(mq, op_connect_queue, future_obj);
    }

private:
  void worker(bip::message_queue& mq,
              bip::message_queue& op_connect_queue,
              //std::promise<void>& exit_signal,
              std::shared_future<void>& future_obj)
  {
    opc.connect();
    std::cout << CLEAR_SCREEN;
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        std::cout << MOVE_CURSOR_TOP;
      try
      {
		emit updateConnectionStatus(opc.isConnected);
        if (opc.isConnected)
        {
          auto time_start = std::chrono::high_resolution_clock::now();
          std::vector<char> buffer(4096); // Buffer to receive data
          std::string json_data;
          unsigned int priority;
          bip::message_queue::size_type recvd_size;
		 
          if (mq.try_receive(buffer.data(), buffer.size(), recvd_size, priority))
          {
              std::string json_data(buffer.data(), recvd_size);
			  std::cout << "Received data: " << json_data << "\n" << std::flush;
              opc.dataProcessing(json_data);
			  printTimestamp();
              auto time_end = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> elapsed_seconds = time_end - time_start;
              std::cout << "Communication processing time: " << elapsed_seconds.count() << "s\n" << std::flush;
           }
          bool new_OPC_Connect = opc.readData(connect_node_id);
          if (new_OPC_Connect != OPC_Connect) {
              OPC_Connect = new_OPC_Connect;
              std::cout << "OPC_Connect: " << OPC_Connect << "\n" << std::flush;
              emit updateOPCConnectStatus(OPC_Connect);
              op_connect_queue.send(&OPC_Connect, sizeof(OPC_Connect), 0);
          }
          //if (OPC_Connect)
          //{
          //  std::cout << "\rCODESYS is ready to receive data." << "\n" << std::flush;
          //}
          //else
          //{
          //  std::cout << "\rCODESYS not ready to receive data." << "\n" << std::flush;
          //}
          
        }
        else
        {
          try
          {
            opc.connect();
            emit updateConnectionStatus(opc.isConnected);
          }
          catch (const std::exception& e)
          {
            std::cerr << "OPC UA Connection failed: " << e.what() << "\n" << std::flush;
            emit errorOccurred(QString::fromStdString(e.what()));
          }
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "An error occurred: " << e.what() << "\n" << std::flush;
        emit errorOccurred(QString::fromStdString(e.what()));
        opc.disconnect();
        emit updateConnectionStatus(opc.isConnected);
        break;
      }
    }
  }

  void printTimestamp()
  {
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

      std::stringstream ss;
      ss << std::put_time(std::localtime(&in_time_t), "[%Y-%m-%d %H:%M:%S");
      ss << '.' << std::setw(3) << std::setfill('0') << ms.count() << ']';
      std::cout << "\r" << ss.str() << "\n" << std::flush;
  }

  std::atomic<bool> stop_event;
  std::string deviceIpAddr = "10.11.36.166";
  int port = 4841;
  std::string base_node_id = "|var|CODESYS Control RTE x64 .Application";
  std::string connect_node_id = base_node_id + ".GVL.OPC_Connect";
  std::string newpos_node_id = base_node_id + ".OPC_UA.newPos";
  std::string time_node_id = base_node_id + ".OPC_UA.TimeStamp";
  PLCData opc;
  bool OPC_Connect = false;
  std::promise<void> exit_signal;
  std::shared_future<void> future_obj;
  bip::message_queue op_connect_queue;
  std::mutex mutex;
};

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    SettingsDialog(Communication* communication, QWidget* parent = nullptr)
        : QDialog(parent), communication(communication)
    {
        QFormLayout* formLayout = new QFormLayout(this);

        QFont font;
        font.setPointSize(12); // Set the desired font size

        ipLineEdit = new QLineEdit(this);
        ipLineEdit->setText("10.11.36.166");
        ipLineEdit->setFont(font);
        formLayout->addRow("IP Address:", ipLineEdit);

        portSpinBox = new QSpinBox(this);
        portSpinBox->setRange(1, 65535);
        portSpinBox->setValue(4841);
        portSpinBox->setFont(font);
        formLayout->addRow("Port:", portSpinBox);

        baseNodeIdLineEdit = new QLineEdit(this);
        baseNodeIdLineEdit->setText("|var|CODESYS Control RTE x64 .Application");
        baseNodeIdLineEdit->setFont(font);
        formLayout->addRow("Base Node ID:", baseNodeIdLineEdit);

        QPushButton* applyButton = new QPushButton("Apply", this);
        applyButton->setFont(font);
        formLayout->addWidget(applyButton);

        connect(applyButton, &QPushButton::clicked, this, &SettingsDialog::applySettings);
    }

private slots:
    void applySettings()
    {
        communication->setConfig(ipLineEdit->text().toStdString(), portSpinBox->value(), baseNodeIdLineEdit->text().toStdString());
        QMessageBox::information(this, "Settings Applied", "Settings have been applied successfully.");
    }

private:
    Communication* communication;
    QLineEdit* ipLineEdit;
    QSpinBox* portSpinBox;
    QLineEdit* baseNodeIdLineEdit;
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr)
        : QWidget(parent), communication(new Communication(this))
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
        statusLabel = new QLabel("Status: Stopped", this);
        statusLabel->setFont(font);
        connectionLamp = new QLabel(this);
        connectionLamp->setFixedSize(20, 20);
        connectionLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        connectionLamp->setFont(font);
        opcConnectLamp = new QLabel(this);
        opcConnectLamp->setFixedSize(20, 20);
        opcConnectLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        opcConnectLamp->setFont(font);
        errorLabel = new QLabel(this);
        errorLabel->setFont(font);

        QLabel* connectionStatusLabel = new QLabel("Connection Status:", this);
        connectionStatusLabel->setFont(font);
        QLabel* opcConnectStatusLabel = new QLabel("OPC Connect Status:", this);
        opcConnectStatusLabel->setFont(font);
        QLabel* errorsLabel = new QLabel("Errors:", this);
        errorsLabel->setFont(font);

        //consoleOutput = new QTextEdit(this);
        //consoleOutput->setFont(font);
        //consoleOutput->setReadOnly(true);
        //consoleOutput->setStyleSheet("background-color: black; color: white;"); // Set background and text color
        //consoleOutput->setTextInteractionFlags(Qt::TextBrowserInteraction);
        //consoleOutput->setAcceptRichText(true); // Enable rich text formatting

        leftLayout->addWidget(startButton);
        leftLayout->addWidget(stopButton);
        leftLayout->addWidget(settingsButton);
        leftLayout->addWidget(statusLabel);
        leftLayout->addWidget(connectionStatusLabel);
        leftLayout->addWidget(connectionLamp);
        leftLayout->addWidget(opcConnectStatusLabel);
        leftLayout->addWidget(opcConnectLamp);
        leftLayout->addWidget(errorsLabel);
        leftLayout->addWidget(errorLabel);

        // Add the left layout and console output to the main layout
        //mainLayout->addLayout(leftLayout);
        //mainLayout->addWidget(consoleOutput);

        connect(startButton, &QPushButton::clicked, this, &MainWindow::startWorker);
        connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopWorker);
        connect(settingsButton, &QPushButton::clicked, this, &MainWindow::openSettingsDialog);
        connect(communication, &Communication::started, this, &MainWindow::onWorkerStarted);
        connect(communication, &Communication::finished, this, &MainWindow::onWorkerStopped);
        connect(communication, &Communication::updateConnectionStatus, this, &MainWindow::onUpdateConnectionStatus);
        connect(communication, &Communication::updateOPCConnectStatus, this, &MainWindow::onUpdateOPCConnectStatus);
        connect(communication, &Communication::errorOccurred, this, &MainWindow::onErrorOccurred);

        // Redirect std::cout to the QTextEdit widget
        //customStreamBuffer = new CustomStreamBuffer(consoleOutput);
        //std::cout.rdbuf(customStreamBuffer);
    }

    //~MainWindow()
    //{
    //    delete customStreamBuffer;
    //}

private slots:
    void startWorker()
    {
        communication->startWorker();
    }

    void stopWorker()
    {
        communication->stopWorker();
    }

    void onWorkerStarted()
    {
        statusLabel->setText("Status: Running");
    }

    void onWorkerStopped()
    {
        statusLabel->setText("Status: Stopped");
    }

    void onUpdateConnectionStatus(bool isConnected)
    {
        if (isConnected)
        {
            connectionLamp->setStyleSheet("background-color: green; border-radius: 15px;");
        }
        else
        {
            connectionLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        }
    }

    void onUpdateOPCConnectStatus(bool OPC_Connect)
    {
        if (OPC_Connect)
        {
            opcConnectLamp->setStyleSheet("background-color: green; border-radius: 15px;");
        }
        else
        {
            opcConnectLamp->setStyleSheet("background-color: red; border-radius: 15px;");
        }
    }

    void onErrorOccurred(const QString& error)
    {
        errorLabel->setText(error);
    }

    void openSettingsDialog()
    {
        if (communication->isRunning())
        {
            QMessageBox::warning(this, "Worker Running", "Please stop the worker before changing settings.");
            return;
        }

        SettingsDialog settingsDialog(communication, this);
        settingsDialog.exec();
    }

private:
    Communication* communication;
    QLabel* statusLabel;
    QLabel* connectionLamp;
    QLabel* opcConnectLamp;
    QLabel* errorLabel;
    //QTextEdit* consoleOutput;
    //CustomStreamBuffer* customStreamBuffer;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.setWindowTitle("Communication");
    window.resize(400, 300);
    window.show();
    return app.exec();
  //std::promise<void> exit_signal;
  //std::shared_future<void> future_obj = exit_signal.get_future();
  //bip::message_queue mq(bip::open_only, "message_queue");
  //bip::message_queue       op_connect_queue(bip::open_only, "op_connect_queue");
  //
  //Communication comm;
  //comm.worker(mq, op_connect_queue, exit_signal, future_obj);

  return 0;
}

#include "Communication.moc"