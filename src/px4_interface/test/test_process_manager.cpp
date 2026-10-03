#include <gtest/gtest.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#define private public
#define protected public
#include "px4_interface/process_manager.hpp"
#undef private
#undef protected

namespace {

constexpr std::chrono::milliseconds kShortWait{100};

void wait_for_state(
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
  const auto start_time = std::chrono::steady_clock::now();
  while (!predicate()) {
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      break;
    }
    std::this_thread::sleep_for(kShortWait);
  }
}

}  // namespace

class ProcessManagerTest : public ::testing::Test {
 protected:
  static ProcessManager make_sleeping_manager(
      const std::string& duration_seconds = "2") {
    return ProcessManager("/bin/sleep", {duration_seconds});
  }
};

TEST_F(ProcessManagerTest, ConstructorInitializesState) {
  auto manager = ProcessManager("/bin/true", {"--help"});

  EXPECT_EQ(manager.pid_, -1);
  EXPECT_FALSE(manager.is_running());
}

TEST_F(ProcessManagerTest, StartLaunchesAndStopTerminatesProcess) {
  auto manager = make_sleeping_manager("5");

  ASSERT_TRUE(manager.start());
  ASSERT_GT(manager.pid_, 0);

  wait_for_state([&]() { return manager.is_running(); });
  EXPECT_TRUE(manager.is_running());

  EXPECT_TRUE(manager.stop());
  EXPECT_EQ(manager.pid_, -1);
  EXPECT_FALSE(manager.is_running());
}

TEST_F(ProcessManagerTest, StartTwiceReturnsFalseAndKeepsOriginalPid) {
  auto manager = make_sleeping_manager("3");

  ASSERT_TRUE(manager.start());
  const pid_t first_pid = manager.pid_;

  EXPECT_FALSE(manager.start());
  EXPECT_EQ(manager.pid_, first_pid);

  EXPECT_TRUE(manager.stop());
}

TEST_F(ProcessManagerTest, StopWithoutRunningProcessReturnsFalse) {
  auto manager = make_sleeping_manager("1");

  EXPECT_FALSE(manager.stop());
  EXPECT_EQ(manager.pid_, -1);
}

TEST_F(ProcessManagerTest, StartWithInvalidExecutableExitsImmediately) {
  ProcessManager manager("/nonexistent_executable_px4_interface", {});

  ASSERT_TRUE(manager.start());
  ASSERT_GT(manager.pid_, 0);

  int status = 0;
  waitpid(manager.pid_, &status, 0);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_NE(0, WEXITSTATUS(status));

  manager.pid_ = -1;
  EXPECT_LT(manager.pid_, 0);
  EXPECT_FALSE(manager.is_running());
}

TEST_F(ProcessManagerTest, DestructorStopsRunningProcess) {
  pid_t child_pid = -1;

  {
    auto manager = make_sleeping_manager("10");
    ASSERT_TRUE(manager.start());
    ASSERT_GT(manager.pid_, 0);
    child_pid = manager.pid_;

    wait_for_state([&]() { return manager.is_running(); });
    EXPECT_TRUE(manager.is_running());
  }

  wait_for_state([&]() { return kill(child_pid, 0) != 0; });
  EXPECT_EQ(-1, kill(child_pid, 0));
  EXPECT_EQ(ESRCH, errno);
}

TEST_F(ProcessManagerTest, IsRunningReflectsPidState) {
  auto manager = make_sleeping_manager("2");

  EXPECT_FALSE(manager.is_running());
  ASSERT_TRUE(manager.start());
  ASSERT_GT(manager.pid_, 0);

  wait_for_state([&]() { return manager.is_running(); });
  EXPECT_TRUE(manager.is_running());

  kill(manager.pid_, SIGTERM);
  int status = 0;
  waitpid(manager.pid_, &status, 0);
  manager.pid_ = -1;
  EXPECT_LT(manager.pid_, 0);
  EXPECT_FALSE(manager.is_running());
}
