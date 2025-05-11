/*
 * File:    main.cpp
 * Author:  psherman-uva
 * Date:    Jan 2023
 */

#include <libs/threadsafe_queue/ThreadsafeQueue.hpp>

#include <iostream>
#include <atomic>
#include <thread>
#include <random>
#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

/// Read all the lines from a file and insert into c++ container
/// @param  filename Name of file to read
/// @return Container filled with each line from file, in order read from file.
std::vector<std::string> read_text_from_file(const std::string& filename);

/// Function running inside threads to asynchronously pop items off queue and print
void queue_popping(int count, int id, int mint, int maxt);


static amrl::ThreadsafeQueue<std::string> queue;
static std::atomic<bool> quit(false);
std::mutex write_mtx; // For printing with std::cout

// ------------ //
//     MAIN    //
// ------------ //

int main(int argc, char* argv[])
{
  std::string filename = "/home/patrick/uva/ros/catkin_ws/src/amrl_logging/libs/threadsafe_queue/test/wordlist.txt";
  std::vector<std::string> lines = read_text_from_file(filename);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(0,10);

  // Begin popping threads
  std::thread t1 = std::thread(queue_popping, lines.size(), 1, 5, 20);
  std::thread t2 = std::thread(queue_popping, lines.size(), 2, 19, 40);
  std::thread t3 = std::thread(queue_popping, lines.size(), 3, 3, 45);

  // Push words onto queue
  unsigned int pushed = 0;
  for (const std::string &word : lines) {
    queue.push(word);
    ++pushed;

    {
      std::lock_guard<std::mutex> lck(write_mtx);
      std::cout << "Pushed: " << word << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)*100));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20000));

  // Ending
  quit = true;
  std::cout << "...Ending" << std::endl;
  if(t1.joinable())
    t1.join();
  if(t2.joinable())
    t2.join();
  if(t3.joinable())
    t3.join();
  return 0;
}


std::vector<std::string> read_text_from_file(const std::string& filename)
{
  std::vector<std::string> text;
  std::ifstream ifs;
  ifs.open(filename, std::ifstream::in);

  std::string line;
  while(std::getline(ifs, line)) {
    text.push_back(line);
  }

  ifs.close();
  return text;
}

void queue_popping(int count, int id, int mint, int maxt)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(mint, maxt);
  std::string tabs;
  for(int i = 0; i < id; ++i)
    tabs += "\t\t\t";

  int popped = 0;
  while(popped < count/2 && !quit.load()) {
    std::shared_ptr<std::string> ptr = queue.pop();
    if(ptr) {
      {
        std::lock_guard<std::mutex> lck(write_mtx);
        std::cout << tabs <<std::to_string(id) <<" Popped: " << *ptr << std::endl;
      }
      ++popped;
    }
    int number = distribution(generator);
    std::this_thread::sleep_for(std::chrono::milliseconds(number*100));
  }
}
