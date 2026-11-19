#ifndef RS485MASTER_H
#define RS485MASTER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

class RS485Master {
 public:
  using ReceiveCallback =
      std::function<void(const uint8_t* data, size_t size)>;

  RS485Master();
  ~RS485Master();

  std::vector<std::string> scanDevices();

  bool connect(int deviceIndex, int baudrate = 500000);
  bool disconnect();
  bool attemptReconnect();

  bool sendData(const unsigned char* data, unsigned int size);
  void setReceiveCallback(ReceiveCallback callback);

  bool isConnected() const;
  int getCurrentDeviceIndex() const;
  int getBaudrate() const;

 private:
  bool openPort(const std::string& port_name, int baudrate);
  void closePort();
  void receiveThreadFunc();
  bool platformWrite(const uint8_t* data, size_t size);
  int platformRead(uint8_t* data, int size);

 private:
  int dev_index_;
  int baudrate_;
  std::string current_port_;
  std::atomic<bool> is_connected_;
  std::atomic<bool> receive_thread_running_;
  std::thread receive_thread_;
  ReceiveCallback receive_callback_;
  std::mutex write_mutex_;

#ifdef _WIN32
  HANDLE hcom_;
#else
  int fd_;
#endif
};

#endif  // RS485MASTER_H
