#include <future>
#include <iostream>
#include <thread>
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

namespace bp  = boost::process;
namespace bip = boost::interprocess;

/*void read_and_print_output(bp::ipstream& stream, const std::string& process_name) {
    std::string line;
    while (std::getline(stream, line)) {
        std::cout << "[" << process_name << "] " << line << std::endl;
    }
}*/

int main(int argc, char* argv[])
{
  std::promise<void>        exit_signal;
  std::shared_future<void>  future_obj = exit_signal.get_future();

  bip::message_queue::remove("message_queue");
  bip::message_queue mq(bip::create_only, "message_queue", 100, 4096);

  bip::message_queue::remove("op_connect_queue");
  bip::message_queue op_connect_queue(bip::create_only, "op_connect_queue", 100, sizeof(bool));

  try
  {
    auto comm_worker_path = bp::search_path("comm_worker");
    auto pos_worker_path  = bp::search_path("pos_worker");

    if (comm_worker_path.empty() || pos_worker_path.empty())
    {
      std::cerr << "Error: comm_worker or pos_worker not found in PATH." << std::endl;
      return 1;
    }

    //bp::child comm_process(bp::search_path("comm_worker"));
    //bp::child pos_process(bp::search_path("pos_worker"));
    // Launch Communication process in a new command prompt window
    bp::child comm_process("cmd.exe /c start cmd.exe /k " + comm_worker_path.string());

    // Launch PositionData process in a new command prompt window
    bp::child pos_process("cmd.exe /c start cmd.exe /k " + pos_worker_path.string());

    //std::this_thread::sleep_for(std::chrono::seconds(10)); // Run for 10 seconds

    std::thread input_thread([&exit_signal, &comm_process, &pos_process]() {
        char input;
        while (true) {
            std::cin >> input;
            if (input == 'q') {
                exit_signal.set_value();
                comm_process.terminate();
                pos_process.terminate();
                break;
            }
        }
        });
    
    comm_process.wait();
    pos_process.wait();
	input_thread.join();
  }
  catch (const boost::process::process_error& e)
  {
    std::cerr << "Process error: " << e.what() << std::endl;
    return 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << "An error occurred: " << e.what() << std::endl;
    return 1;
  }

  bip::message_queue::remove("message_queue");
  bip::message_queue::remove("op_connect_queue");
   
  std::cout << "Processes stopped." << std::endl;

  return 0;
}
