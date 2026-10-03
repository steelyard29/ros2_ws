#include "px4_interface/process_manager.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

ProcessManager::ProcessManager(const std::string &executable_name,
                               const std::vector<std::string> &args)
    : executable_name_(executable_name), args_(args), pid_{-1} {
  if (executable_name_.empty()) {
    std::cerr << "Executable name cannot be empty." << std::endl;
  }
  if (args_.empty()) {
    std::cerr << "Warning: No arguments provided for the executable."
              << std::endl;
  }
}

ProcessManager::~ProcessManager() {
  if (is_running()) {
    stop();
  }
}

bool ProcessManager::start() {
  if (is_running()) {
    std::cerr << "Process is already running with PID: " << pid_ << std::endl;
    return false;
  }
  pid_ = fork();
  if (pid_ < 0) {
    std::cerr << "Failed to fork process." << std::endl;
    return false;
  }
  if (pid_ == 0) {
    // Child process
    std::vector<char *> c_args;
    c_args.push_back(const_cast<char *>(executable_name_.c_str()));
    for (const auto &arg : args_) {
      c_args.push_back(const_cast<char *>(arg.c_str()));
    }
    c_args.push_back(nullptr);
    if (execvp(executable_name_.c_str(), c_args.data()) < 0) {
      std::cerr << "Failed to execute: " << executable_name_ << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  // Parent process
  return true;
}

bool ProcessManager::stop() {
  if (!is_running()) {
    std::cerr << "No running process to stop." << std::endl;
    return false;
  }

  if (kill(pid_, SIGTERM) < 0) {
    std::cerr << "Failed to send SIGTERM to process with PID: " << pid_
              << std::endl;
    return false;
  }

  int waited_ms = 0;
  const int wait_interval_ms = 100;

  while (is_running() && waited_ms < max_wait_ms_) {
    int status;
    pid_t result = waitpid(pid_, &status, WNOHANG);
    if (result == pid_) {
      pid_ = -1;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_interval_ms));
    waited_ms += wait_interval_ms;
  }

  std::cerr << "Process did not terminate in time, sending SIGKILL to PID: "
            << pid_ << std::endl;
  if (kill(pid_, SIGKILL) < 0) {
    std::cerr << "Failed to send SIGKILL to process with PID: " << pid_
              << std::endl;
    return false;
  }

  int status;
  waitpid(pid_, &status, 0);
  pid_ = -1;
  return true;
}

bool ProcessManager::is_running() const {
  return (pid_ > 0 && kill(pid_, 0) == 0);
}
