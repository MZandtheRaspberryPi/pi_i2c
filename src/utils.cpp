#include "utils.hpp"

void delay(int64_t sleep_ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}

void log_msg(const std::string &msg) { std::cout << msg << std::endl; }

uint64_t millis() {
  std::chrono::milliseconds ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  return ms.count();
}
