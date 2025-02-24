from app import Communication, PositionData
from queue import Queue
from threading import Thread, Event
import time
class Worker:
    def __init__(self) -> None:
        self.Communication = Communication()
        self.position_data = PositionData()
        # Create the shared queue and launch all threads
        self.q1 = Queue() # for app command
        self.q2 = Queue() # for navigator target position
        self.input_event = Event()
        self.output_event = Event()
        self.start_event = Event()

    def main(self):
        communication_thread = Thread(
            target=self.Communication.worker,
            args=(
                self.q1,
                self.q2,
                self.input_event,
                self.output_event,
                self.start_event
                )
            )
        position_data_thread = Thread(
            target=self.position_data.worker, 
            args=(
                self.q1, 
                self.q2,
                self.input_event,
                self.output_event,
                self.start_event
                )
            )

        communication_thread.start()
        position_data_thread.start()
    
        # Wait for user input to stop the threads
        input("Press Enter to stop the workers...\n")

        # Signal the workers to stop
        self.Communication.stop_worker()
        time.sleep(1)
        self.position_data.stop_worker()

        # Wait for the worker threads to finish
        communication_thread.join()
        position_data_thread.join()
        print("Main thread finished.")

if __name__ == "__main__":
    worker = Worker()
    worker.main()
