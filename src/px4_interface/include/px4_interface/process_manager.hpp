
/**
 * @file process_manager.hpp
 * @brief Header file for the ProcessManager class, which manages the lifecycle
 * of external processes.
 *
 * This file defines the ProcessManager class, providing methods to start, stop,
 * and monitor external executable processes using fork and exec system calls.
 * It is part of the px4_interface package in a ROS2 workspace.
 *
 * @author Wang WeiMing
 * @date 2025-10-7
 * @version 1.0
 *
 * @note This class is designed for Unix-like systems and uses POSIX process
 * management functions. Ensure proper permissions and environment setup for
 * executing external processes.
 *
 * @see ProcessManager
 */
#pragma once

#include <sys/types.h>

#include <csignal>
#include <string>
#include <vector>

/**
 * @class ProcessManager
 * @brief Manages the lifecycle of an external process.
 *
 * The ProcessManager class provides functionality to start, stop, and monitor
 * an external executable process. It handles process creation using fork and
 * exec, and tracks the process ID for status checks and termination.
 */
class ProcessManager {
 public:
  /**
   * @brief Constructs a ProcessManager with the specified executable and
   * arguments.
   *
   * @param executable_name The name or path of the executable to run.
   * @param args A vector of command-line arguments to pass to the executable.
   */
  ProcessManager(const std::string &executable_name,
                 const std::vector<std::string> &args);

  /**
   * @brief Destructor for ProcessManager.
   *
   * Ensures that any running process is stopped before destruction.
   */
  ~ProcessManager();

  /**
   * @brief Starts the external process.
   *
   * Forks a new process and executes the specified executable with the given
   * arguments.
   *
   * @return true if the process was started successfully, false otherwise.
   */
  bool start();

  /**
   * @brief Stops the running process.
   *
   * Send a SIGTERM signal to the process if it is currently running.
   *
   * @return true if the process was stopped successfully, false otherwise.
   */
  bool stop();

  /**
   * @brief Checks if the managed process is currently running.
   *
   * @return true if the process is running, false otherwise.
   */
  bool is_running() const;

 private:
  std::string executable_name_;
  std::vector<std::string> args_;
  pid_t pid_{-1};

  const int max_wait_ms_ = 2000;  // Max wait time in milliseconds
};
