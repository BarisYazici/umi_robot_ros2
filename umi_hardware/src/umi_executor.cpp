#include "umi_hardware/umi_executor.hpp"

namespace umi_hardware {
using namespace std::chrono_literals;

UmiExecutor::UmiExecutor() : executor_spin_([this] { run(); }) {
  while (!this->spinning) {
    std::this_thread::sleep_for(100ms);
  }
}

UmiExecutor::~UmiExecutor() {
  this->shutdown();
  executor_spin_.join();
}

void UmiExecutor::run() {
  spin();
}

void UmiExecutor::shutdown() {
  if (this->spinning) {
    this->cancel();
  }
}

}  // namespace umi_hardware
