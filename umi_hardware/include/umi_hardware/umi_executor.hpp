
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace umi_hardware {

class UmiExecutor : public rclcpp::executors::MultiThreadedExecutor {
 public:
    // Create an instance and start the internal thread
    UmiExecutor();
    UmiExecutor(const UmiExecutor&) = delete;
    UmiExecutor(UmiExecutor&&) = delete;

    UmiExecutor& operator=(const UmiExecutor&) = delete;
    UmiExecutor& operator=(UmiExecutor&&) = delete;

    ~UmiExecutor() override;

 private:
    std::thread executor_spin_;
    void run();
    void shutdown();
};
}  // namespace umi_hardware
