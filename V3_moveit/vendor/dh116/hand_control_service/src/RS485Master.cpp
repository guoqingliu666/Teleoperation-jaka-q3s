#include "RS485Master.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <iostream>
#include <utility>

#ifdef _WIN32

RS485Master::RS485Master()
    : dev_index_(-1),
      baudrate_(500000),
      is_connected_(false),
      receive_thread_running_(false),
      hcom_(INVALID_HANDLE_VALUE) {}

RS485Master::~RS485Master() { disconnect(); }

std::vector<std::string> RS485Master::scanDevices() {
  std::vector<std::string> ports;

  HKEY hkey;
  if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                    KEY_READ, &hkey) == ERROR_SUCCESS) {
    char value_name[256];
    BYTE value_data[256];
    DWORD index = 0;
    DWORD name_len = 0;
    DWORD data_len = 0;
    DWORD data_type = 0;

    while (true) {
      name_len = sizeof(value_name);
      data_len = sizeof(value_data);
      LONG ret = RegEnumValueA(hkey, index++, value_name, &name_len, nullptr,
                               &data_type, value_data, &data_len);
      if (ret != ERROR_SUCCESS) {
        break;
      }
      if (data_type == REG_SZ) {
        std::string port(reinterpret_cast<char*>(value_data));
        if (!port.empty()) {
          ports.push_back(port);
        }
      }
    }
    RegCloseKey(hkey);
  }

  if (ports.empty()) {
    for (int i = 1; i <= 256; ++i) {
      std::string port = "COM" + std::to_string(i);
      std::string full_port = "\\\\.\\" + port;
      HANDLE h = CreateFileA(full_port.c_str(), 0, 0, nullptr, OPEN_EXISTING,
                             0, nullptr);
      if (h != INVALID_HANDLE_VALUE) {
        ports.push_back(port);
        CloseHandle(h);
      }
    }
  }

  return ports;
}

bool RS485Master::openPort(const std::string& port_name, int baudrate) {
  std::string full_port = "\\\\.\\" + port_name;
  hcom_ = CreateFileA(full_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                      nullptr, OPEN_EXISTING, 0, nullptr);
  if (hcom_ == INVALID_HANDLE_VALUE) {
    return false;
  }

  DCB dcb = {};
  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(hcom_, &dcb)) {
    CloseHandle(hcom_);
    hcom_ = INVALID_HANDLE_VALUE;
    return false;
  }

  dcb.BaudRate = static_cast<DWORD>(baudrate);
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;
  dcb.Parity = NOPARITY;

  if (!SetCommState(hcom_, &dcb)) {
    CloseHandle(hcom_);
    hcom_ = INVALID_HANDLE_VALUE;
    return false;
  }

  COMMTIMEOUTS timeouts = {};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  SetCommTimeouts(hcom_, &timeouts);

  return true;
}

void RS485Master::closePort() {
  if (hcom_ != INVALID_HANDLE_VALUE) {
    CloseHandle(hcom_);
    hcom_ = INVALID_HANDLE_VALUE;
  }
}

bool RS485Master::platformWrite(const uint8_t* data, size_t size) {
  DWORD written = 0;
  return WriteFile(hcom_, data, static_cast<DWORD>(size), &written, nullptr) &&
         written == static_cast<DWORD>(size);
}

int RS485Master::platformRead(uint8_t* data, int size) {
  DWORD bytes_read = 0;
  if (ReadFile(hcom_, data, static_cast<DWORD>(size), &bytes_read, nullptr)) {
    return static_cast<int>(bytes_read);
  }
  return -1;
}

#else

#include <fcntl.h>
#include <glob.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

bool baudrateToSpeed(int baudrate, speed_t* speed) {
  switch (baudrate) {
    case 9600:
      *speed = B9600;
      return true;
    case 19200:
      *speed = B19200;
      return true;
    case 38400:
      *speed = B38400;
      return true;
    case 57600:
      *speed = B57600;
      return true;
    case 115200:
      *speed = B115200;
      return true;
#ifdef B230400
    case 230400:
      *speed = B230400;
      return true;
#endif
#ifdef B460800
    case 460800:
      *speed = B460800;
      return true;
#endif
#ifdef B500000
    case 500000:
      *speed = B500000;
      return true;
#endif
#ifdef B576000
    case 576000:
      *speed = B576000;
      return true;
#endif
#ifdef B921600
    case 921600:
      *speed = B921600;
      return true;
#endif
#ifdef B1000000
    case 1000000:
      *speed = B1000000;
      return true;
#endif
#ifdef B1500000
    case 1500000:
      *speed = B1500000;
      return true;
#endif
#ifdef B2000000
    case 2000000:
      *speed = B2000000;
      return true;
#endif
#ifdef B3000000
    case 3000000:
      *speed = B3000000;
      return true;
#endif
    default:
      return false;
  }
}

void appendPorts(const char* pattern, std::vector<std::string>* ports) {
  glob_t glob_result = {};
  if (glob(pattern, 0, nullptr, &glob_result) != 0) {
    globfree(&glob_result);
    return;
  }

  for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
    std::string port = glob_result.gl_pathv[i];
    if (std::find(ports->begin(), ports->end(), port) == ports->end()) {
      ports->push_back(port);
    }
  }
  globfree(&glob_result);
}

bool isVirtualConsoleTty(const std::string& port) {
  const auto pos = port.find_last_of('/');
  const std::string name =
      pos == std::string::npos ? port : port.substr(pos + 1);
  if (name == "tty" || name == "ttyprintk") {
    return true;
  }
  if (name.size() <= 3 || name.rfind("tty", 0) != 0) {
    return false;
  }
  return std::all_of(name.begin() + 3, name.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool isStandardTtyS(const std::string& port) {
  const auto pos = port.find_last_of('/');
  const std::string name =
      pos == std::string::npos ? port : port.substr(pos + 1);
  return name.size() > 4 && name.rfind("ttyS", 0) == 0 &&
         std::isdigit(static_cast<unsigned char>(name[4])) != 0;
}

void appendFallbackTtyPorts(std::vector<std::string>* ports) {
  glob_t glob_result = {};
  if (glob("/dev/tty*", 0, nullptr, &glob_result) != 0) {
    globfree(&glob_result);
    return;
  }

  for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
    std::string port = glob_result.gl_pathv[i];
    if (isVirtualConsoleTty(port) || isStandardTtyS(port)) {
      continue;
    }
    if (std::find(ports->begin(), ports->end(), port) == ports->end()) {
      ports->push_back(port);
    }
  }
  globfree(&glob_result);
}

}  // namespace

RS485Master::RS485Master()
    : dev_index_(-1),
      baudrate_(500000),
      is_connected_(false),
      receive_thread_running_(false),
      fd_(-1) {}

RS485Master::~RS485Master() { disconnect(); }

std::vector<std::string> RS485Master::scanDevices() {
  std::vector<std::string> ports;
  appendPorts("/dev/ttyXR*", &ports);
  appendPorts("/dev/ttyUSB*", &ports);
  appendPorts("/dev/ttyACM*", &ports);
  appendPorts("/dev/ttyAMA*", &ports);
  appendPorts("/dev/ttyTHS*", &ports);
  appendPorts("/dev/ttyO*", &ports);
  appendPorts("/dev/ttySAC*", &ports);
  appendPorts("/dev/ttySC*", &ports);
  appendPorts("/dev/ttymxc*", &ports);
  appendPorts("/dev/ttyUL*", &ports);
  appendPorts("/dev/ttyPS*", &ports);
  appendPorts("/dev/ttyMSM*", &ports);
  appendPorts("/dev/ttyHS*", &ports);
  appendFallbackTtyPorts(&ports);
  appendPorts("/dev/ttyS*", &ports);
  return ports;
}

bool RS485Master::openPort(const std::string& port_name, int baudrate) {
  speed_t speed = B0;
  if (!baudrateToSpeed(baudrate, &speed)) {
    std::cerr << "Unsupported RS485 baudrate: " << baudrate << std::endl;
    return false;
  }

  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void RS485Master::closePort() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool RS485Master::platformWrite(const uint8_t* data, size_t size) {
  return ::write(fd_, data, size) == static_cast<ssize_t>(size);
}

int RS485Master::platformRead(uint8_t* data, int size) {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);
  struct timeval tv = {0, 1000};

  int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret > 0 && FD_ISSET(fd_, &read_fds)) {
    return static_cast<int>(::read(fd_, data, static_cast<size_t>(size)));
  }
  return ret < 0 ? -1 : 0;
}

#endif  // _WIN32

bool RS485Master::connect(int deviceIndex, int baudrate) {
  if (is_connected_) {
    disconnect();
  }

  std::vector<std::string> ports = scanDevices();
  if (deviceIndex < 0 || deviceIndex >= static_cast<int>(ports.size())) {
    return false;
  }

  if (!openPort(ports[deviceIndex], baudrate)) {
    return false;
  }

  dev_index_ = deviceIndex;
  baudrate_ = baudrate;
  current_port_ = ports[deviceIndex];
  is_connected_ = true;
  receive_thread_running_ = true;
  receive_thread_ = std::thread(&RS485Master::receiveThreadFunc, this);
  return true;
}

bool RS485Master::disconnect() {
  if (!is_connected_) {
    return true;
  }

  receive_thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  closePort();
  dev_index_ = -1;
  current_port_.clear();
  is_connected_ = false;
  return true;
}

bool RS485Master::attemptReconnect() {
  const int device_index = dev_index_;
  const int baudrate = baudrate_;
  disconnect();
  return connect(device_index, baudrate);
}

bool RS485Master::sendData(const unsigned char* data, unsigned int size) {
  if (!is_connected_ || data == nullptr || size == 0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  return platformWrite(reinterpret_cast<const uint8_t*>(data),
                       static_cast<size_t>(size));
}

void RS485Master::setReceiveCallback(ReceiveCallback callback) {
  receive_callback_ = std::move(callback);
}

bool RS485Master::isConnected() const { return is_connected_; }

int RS485Master::getCurrentDeviceIndex() const { return dev_index_; }

int RS485Master::getBaudrate() const { return baudrate_; }

void RS485Master::receiveThreadFunc() {
  uint8_t buffer[1024];

  while (receive_thread_running_) {
    if (!is_connected_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    int bytes_read = platformRead(buffer, static_cast<int>(sizeof(buffer)));
    if (bytes_read <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (receive_callback_) {
      receive_callback_(buffer, static_cast<size_t>(bytes_read));
    }
  }
}
