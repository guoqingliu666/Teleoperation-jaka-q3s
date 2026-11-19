// Bounded Linux adaptation of manufacturer main.cpp; originals under vendor/dh116/motion_example.
// ============================================================================
// 文件说明:
// File overview:
// 该示例演示如何在 EtherCAT 主站下连接、初始化并控制单个从站。
// This sample shows how to connect, initialize, and control one EtherCAT slave
// under a single EtherCAT master.
//
// 主要流程包括:
// Main flow includes:
// 1. 扫描并选择网口
// 1. Scan and select a network adapter
// 2. 扫描并选择要控制的从站
// 2. Scan and select the target slave
// 3. 创建 LHandProLib 实例并绑定到当前从站
// 3. Create one LHandProLib instance and bind it to the selected slave
// 4. 后台循环接收并解析当前从站的 TPDO 数据
// 4. Receive and decode TPDO data for the selected slave in a background loop
// 5. 执行单从站运动测试或传感器读取测试
// 5. Run the single-slave motion test or sensor read test
// ============================================================================

#include <algorithm>
#include <csignal>
#include <sys/file.h>
#include <fcntl.h>
#include <pwd.h>
#include <grp.h>
#include <cstring>
#include <atomic>
#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "EthercatMaster.h"
#ifndef SDK_CPP_HEADER
#define SDK_CPP_HEADER "LHandProLib.hpp"
#endif
#include SDK_CPP_HEADER

#ifndef SDK_NS
#define SDK_NS lhplib
#endif
#ifndef SDK_CLASS
#define SDK_CLASS LHandProLib
#endif
#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {
volatile sig_atomic_t test_running=1;
void request_stop(int){test_running=0;}
bool test_passed=false;


// 使能后等待设备进入稳定状态的时间，单位为毫秒。
// Time to wait after enable so the device can reach a stable state, in ms.
constexpr int kEnableWaitMs = 1000;

// 回零命令下发后等待回零完成的时间，单位为毫秒。
// Time to wait after issuing homing, in ms.
constexpr int kHomeWaitMs = 10000;

// 监控线程启动后的首次等待时间，给 EtherCAT 循环和设备初始化留出缓冲。
// Initial delay before the monitor thread starts polling TPDO data, in ms.
constexpr int kMonitorStartupDelayMs = 1000;

// 监控线程轮询当前从站 TPDO 数据的周期，单位为毫秒。
// Polling period of the monitor thread when reading TPDO data, in ms.
constexpr int kMonitorLoopDelayMs = 10;

// 每组测试位姿之间的停顿时间，单位为毫秒。
// Delay between motion steps in the test sequence, in ms.
constexpr int kMotionStepDelayMs = 6000;

// 位置模式示例中使用的默认目标速度。
// Default position velocity used in the motion example.
constexpr int kPositionVelocity = 2000;

// 位置模式示例中使用的默认最大电流限制。
// Default max current limit used in the motion example.
constexpr int kMaxCurrent = 200;

// 仅在 Windows 控制台中检测 Esc，便于快速停止循环运动示例。
// Only check Esc on Windows console so the motion loop can be stopped quickly.
bool is_escape_pressed() {
#ifdef _WIN32
  if (_kbhit()) {
    return _getch() == 27;
  }
#endif
  return !test_running;
}

// 退出前等待用户确认，便于查看终端输出信息。
// Wait before exit so the user can read the terminal output.
void wait_before_exit() {
#ifdef _WIN32
  system("pause");
#else
  std::cin.get();
#endif
}

// 读取一个指定范围内的数字输入。
// Read a numeric input within the specified range.
int select_number_in_range(const std::string& prompt, int min_value,
                           int max_value) {
  while (true) {
    std::cout << prompt << " [" << min_value << " - " << max_value << "]"
              << std::endl;

    int value = 0;
    std::cin >> value;
    if (value < min_value || value > max_value) {
      std::cout << "Input out of range, please try again." << std::endl;
      continue;
    }

    return value;
  }
}

// 打印一组目标位置，便于观察当前步的控制目标。
// Print one position set so the current command step is easy to inspect.
void print_positions(int line_index, const std::vector<int>& positions,
                     int return_code) {
  std::cout << "line: " << line_index << " positions: ";
  for (int value : positions) {
    std::cout << value << " ";
  }
  std::cout << "retn: " << return_code << std::endl;
}

struct FingerForceData {
  std::vector<float> normal_force;
  std::vector<float> tangential_force;
  std::vector<float> force_direction;
};

using FingerForceGetter = int (SDK_NS::SDK_CLASS::*)(int, float*, int*);

bool read_finger_force_array(SDK_NS::SDK_CLASS& lib, int sensor_id,
                             FingerForceGetter getter,
                             std::vector<float>& values) {
  int count = 0;
  int result = (lib.*getter)(sensor_id, nullptr, &count);
  if (result != SDK_NS::LER_NONE || count <= 0) {
    return false;
  }

  values.assign(static_cast<size_t>(count), 0.0f);
  result = (lib.*getter)(sensor_id, values.data(), &count);
  if (result != SDK_NS::LER_NONE || count < 0) {
    return false;
  }

  values.resize(static_cast<size_t>(count));
  return true;
}

bool read_finger_force_data(SDK_NS::SDK_CLASS& lib, int sensor_id,
                            FingerForceData& data) {
  return read_finger_force_array(
             lib, sensor_id, &SDK_NS::SDK_CLASS::get_finger_normal_force_ex,
             data.normal_force) &&
         read_finger_force_array(
             lib, sensor_id,
             &SDK_NS::SDK_CLASS::get_finger_tangential_force_ex,
             data.tangential_force) &&
         read_finger_force_array(
             lib, sensor_id, &SDK_NS::SDK_CLASS::get_finger_force_direction_ex,
             data.force_direction);
}

std::string format_finger_force_sample(const FingerForceData& data,
                                       size_t index) {
  std::ostringstream sample;
  if (index >= data.normal_force.size() || index >= data.tangential_force.size() ||
      index >= data.force_direction.size()) {
    sample << "force" << (index + 1) << "(n/a)";
    return sample.str();
  }

  sample << "force" << (index + 1) << "(nf:" << data.normal_force[index]
         << ",tf:" << data.tangential_force[index]
         << ",dir:" << data.force_direction[index] << ")";
  return sample.str();
}

std::string format_palm_pressure(SDK_NS::SDK_CLASS& lib,
                                 int palm_sensor_id) {
  std::ostringstream palm;
  palm << "palm:" << palm_sensor_id << " pressure:";

  int count = 0;
  int result = lib.get_finger_pressure(palm_sensor_id, nullptr, &count);
  if (result != SDK_NS::LER_NONE || count <= 0) {
    palm << "n/a";
    return palm.str();
  }

  std::vector<float> pressure_list(static_cast<size_t>(count), 0.0f);
  result = lib.get_finger_pressure(palm_sensor_id, pressure_list.data(), &count);
  if (result != SDK_NS::LER_NONE || count < 0) {
    palm << "n/a";
    return palm.str();
  }

  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      palm << ",";
    }
    palm << pressure_list[static_cast<size_t>(i)];
  }
  return palm.str();
}

// 打印单个传感器的两组三维力数据，便于观察原始反馈。
void print_one_sensor_force_line(SDK_NS::SDK_CLASS& lib, int sensor_id) {
  constexpr size_t kForceSamplesToPrint = 2;
  FingerForceData data;
  read_finger_force_data(lib, sensor_id, data);

  std::ostringstream line;
  line << "\033[2K  id:" << sensor_id << " ";
  for (size_t i = 0; i < kForceSamplesToPrint; ++i) {
    line << format_finger_force_sample(data, i);
    if (i + 1 < kForceSamplesToPrint) {
      line << " ";
    }
  }
  std::cout << line.str() << std::endl;
}

// 打印手心压力数据。
void print_palm_pressure_line(SDK_NS::SDK_CLASS& lib, int palm_sensor_id) {
  std::cout << "\033[2K  " << format_palm_pressure(lib, palm_sensor_id)
            << std::endl;
}

// 打印所有传感器数据（原地刷新）。
void print_sensor_force_data(
    SDK_NS::SDK_CLASS& lib, const std::vector<int>& finger_sensor_id_list,
    int palm_sensor_id, bool first_frame) {
  // 非首帧：光标上移覆盖上一帧（1行分隔线 + 5行手指 + 1行掌心 = 7行）
  if (!first_frame) {
    std::cout << "\033[7A";
  }
  std::cout << "\033[2K--- sensor force data ---" << std::endl;
  for (int sensor_id : finger_sensor_id_list) {
    print_one_sensor_force_line(lib, sensor_id);
  }
  print_palm_pressure_line(lib, palm_sensor_id);
  std::cout << std::flush;
}

// 为当前从站批量下发本步运动参数，包括目标位置、速度和电流限制。
// Apply one motion step to the selected slave, including position, velocity,
// and current.
void configure_motion(SDK_NS::SDK_CLASS& lib,
                      const std::vector<int>& positions, int dof_active) {
  for (int axis = 0; axis < dof_active; ++axis) {
    const int joint_id = axis + 1;
    lib.set_target_position(joint_id, positions[axis]);
    lib.set_position_velocity(joint_id, kPositionVelocity);
    lib.set_max_current(joint_id, kMaxCurrent);
  }
}

// 从当前从站读取一帧 TPDO 输入数据，并交给 SDK 进行解析。
// Read one TPDO input frame from the selected slave and decode it in the SDK.
void refresh_tpdo(EthercatMaster& master, int slave_id,
                  SDK_NS::SDK_CLASS& lib) {
  const int input_size = master.getSlaveInputSize(slave_id);
  if (input_size <= 0) {
    return;
  }

  std::vector<uint8_t> input_buffer(input_size);
  if (master.getSlaveInputs(slave_id, input_buffer.data(), input_size)) {
    lib.set_tpdo_data_decode(input_buffer.data(), input_size);
  }
}

}  // namespace

int main(int argc,char** argv) {
  if(argc!=2 || std::strcmp(argv[1],"--hardware")){std::cerr<<"Explicit --hardware required\n";return 2;}
  signal(SIGINT,request_stop);signal(SIGTERM,request_stop);signal(SIGPIPE,SIG_IGN);
  int lockfd=open(".cache/operator/hand.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);
  if(lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)){std::cerr<<"DH116 owner lock unavailable\n";return 2;}
  if(geteuid()==0 && getenv("PKEXEC_UID"))fchown(lockfd,std::stoi(getenv("PKEXEC_UID")),-1);

  auto sdk_handle = std::make_shared<SDK_NS::SDK_CLASS>();
  sdk_handle->set_log_callback([](const char* text){std::cerr<<"SDK "<<text<<std::endl;});
  auto ec_master = std::make_shared<EthercatMaster>();

  std::atomic<bool> stop_monitor{false};
  std::thread monitor_thread;
  int return_code = 0;

  // 运动测试使用的目标位置表。
  // Motion table used by the single-slave motion example.
  const std::vector<std::vector<int>> positions = {
      {1500, 0, 0, 0, 0, 0}, {0, 1500, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
      {0, 0, 1500, 0, 0, 0}, {0, 0, 0, 1500, 0, 0}, {0, 0, 0, 0, 1500, 0},
      {0, 0, 0, 0, 0, 1500}, {0, 0, 0, 0, 0, 0},
  };

  // ---------------------------------------------------------------------------
  // 1. 初始化 EtherCAT 主站并选择网口
  // 1. Initialize EtherCAT master and let the user choose a network adapter.
  // ---------------------------------------------------------------------------
  std::cout << "Using EtherCAT communication (100M)." << std::endl << std::endl;

  const std::vector<std::string> interface_names =
      ec_master->scanNetworkInterfaces();
  std::cout << "Detected network adapters: " << interface_names.size()
            << std::endl
            << std::endl;

  if (interface_names.empty()) {
    wait_before_exit();
    return 0;
  }

  for (size_t i = 0; i < interface_names.size(); ++i) {
    std::cout << "Input " << i << " ---- adapter " << interface_names[i]
              << std::endl;
  }

  const int channel_index =
      select_number_in_range("\nPlease select the EtherCAT adapter", 0,
                             static_cast<int>(interface_names.size()) - 1);

  if (!ec_master->init(channel_index)) {
    std::cout << "EtherCAT init failed." << std::endl;
    wait_before_exit();
    return -1;
  }

  std::cout << "EtherCAT init succeeded." << std::endl;

  if (!ec_master->start()) {
    std::cout << "Failed to start EtherCAT master." << std::endl;
    return -1;
  }

  // ---------------------------------------------------------------------------
  // 2. 扫描从站并选择目标从站
  // 2. Scan slaves and choose the target slave.
  // ---------------------------------------------------------------------------
  const int slave_count = ec_master->getSlaveCount();
  std::cout << "\nDetected slave count: " << slave_count << std::endl;

  const auto slave_list = ec_master->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "  Slave " << slave.index << " : " << slave.name << std::endl;
  }

  if (slave_count < 1) {
    std::cerr << "No EtherCAT slave was found." << std::endl;
    return -1;
  }

  int slave_id = 1;
  if (slave_count > 1) {
    slave_id = select_number_in_range("\nPlease select the target slave id", 1,
                                      slave_count);
  }
  std::cout << "Selected slave: " << slave_id << std::endl;

  // 启动 EtherCAT 后台交换循环，后续 SDK 的发送和接收都依赖这条循环。
  // Start the EtherCAT background exchange loop. The SDK send/receive callbacks
  // below depend on this loop to keep PDO data flowing.
  ec_master->run();
  if(geteuid()==0 && getenv("PKEXEC_UID")){
    auto uid=uid_t(std::stoul(getenv("PKEXEC_UID")));auto* pw=getpwuid(uid);
    if(!pw || setgroups(0,nullptr) || setgid(pw->pw_gid) || setuid(uid)){ec_master->stop();return 2;}
  }

  // ---------------------------------------------------------------------------
  // 3. 绑定 LHandProLib 实例到当前从站
  // 3. Bind one LHandProLib instance to the selected slave.
  // ---------------------------------------------------------------------------
  auto send_func = [ec_master, slave_id](const unsigned char* data,
                                         unsigned int size) {
    return ec_master->setSlaveOutputs(slave_id, data, size);
  };

  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  sdk_handle->set_send_rpdo_callback_ex(&func);

  // SDO（CoE 邮箱）：注册后 initial() 会通过 SDO 读取手型/左右手/固件/SN，
  // get/set/save_sdo_drive_param() 也经由该回调（邮箱归主站，SDK 不直接访问）。
  // SDO (CoE mailbox): with this callback, initial() auto-detects hand type /
  // side / firmware / SN and the SDO parameter APIs are routed to the master.
  int identified_model=-1,identified_side=-1,optional_absent=0;bool unexpected_sdo_failure=false;
  std::function<bool(unsigned int, unsigned char, int, unsigned int,
                     unsigned int*)>
      sdo_func = [ec_master, slave_id, &identified_model, &identified_side, &optional_absent, &unexpected_sdo_failure](unsigned int index,
                                       unsigned char subindex, int is_write,
                                       unsigned int value,
                                       unsigned int* out_value) {
        bool ok = is_write
                   ? ec_master->sdoWrite(static_cast<uint16_t>(index), subindex,
                                         value, slave_id)
                   : ec_master->sdoRead(static_cast<uint16_t>(index), subindex,
                                        out_value, slave_id);
        if(ok && !is_write && index==0x200e && subindex==0x1d)identified_model=*out_value;
        if(ok && !is_write && index==0x200e && subindex==0x1e)identified_side=*out_value;
        if(!ok){
          if(!is_write && index==0x2030 && (subindex==5 || subindex==6) && ec_master->getLastError().find("06020000")!=std::string::npos)optional_absent|=(subindex==5?1:2);
          else unexpected_sdo_failure=true;
        }
        std::cerr<<"SDO "<<std::hex<<index<<":"<<unsigned(subindex)<<std::dec<<" write="<<is_write<<" ok="<<ok;
        if(ok && !is_write && out_value)std::cerr<<" value="<<*out_value;
        if(!ok)std::cerr<<" error="<<ec_master->getLastError();
        std::cerr<<std::endl;return ok;
      };
  sdk_handle->set_ecat_sdo_callback_ex(&sdo_func);

  // 监控线程持续从当前从站获取 TPDO 数据，并交给 SDK 实例解析。
  // This monitor thread continuously fetches TPDO data from the selected slave
  // and forwards it to the SDK instance for decoding.
  monitor_thread = std::thread([=, &stop_monitor]() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kMonitorStartupDelayMs));

    while (!stop_monitor) {
      refresh_tpdo(*ec_master, slave_id, *sdk_handle);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMonitorLoopDelayMs));
    }
  });

  int dof_total = 0;
  int dof_active = 0;
  float version = 0.0;

  {int result=sdk_handle->initial(SDK_NS::LCN_ECAT);
  std::cerr<<"SDK_INITIAL_RESULT="<<result<<std::endl;
  const bool known_optional_metadata_absent=result==SDK_NS::LER_COMM_RECV && identified_model==0 && (identified_side==0 || identified_side==1) && optional_absent==3 && !unexpected_sdo_failure;
  if(known_optional_metadata_absent)std::cout<<"COMPAT: DH116 model/side verified; only optional firmware/SN objects absent; SDK remains initialized per manufacturer contract."<<std::endl;
  if (result != SDK_NS::LER_NONE && !known_optional_metadata_absent) {
    std::cerr << "LHandProLib init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }}

  // 从这里开始，sdk_handle 就代表当前选中的 EtherCAT 从站。
  // From here on, sdk_handle represents the selected EtherCAT slave.

  // ---------------------------------------------------------------------------
  // 4. 读取从站能力，完成使能，然后执行回零
  // 4. Read slave capability, enable the device, then home it.
  // ---------------------------------------------------------------------------
  sdk_handle->get_firmware_version(&version);

  sdk_handle->get_dof(&dof_total, &dof_active);
  std::cout << "DOF: total " << dof_total << ", active " << dof_active
            << std::endl;

  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    std::vector<uint8_t> raw(ec_master->getSlaveInputSize(1));
    if(!test_running || !ec_master->isConnected() || !ec_master->getSlaveInputs(1,raw.data(),raw.size()) || raw.size()!=192 || raw[0]!=0 || raw[1]!=27){std::cerr<<"No verified joint feedback before enable\n";return_code=5;goto Cleanup;}
    for(int axis=0;axis<6;axis++)if(!raw[164+axis] || (raw[164+axis]>>5)>1){std::cerr<<"Abnormal axis feedback before enable\n";return_code=6;goto Cleanup;}
  }
  if(dof_active!=6){std::cerr<<"Expected six active axes\n";return_code=2;goto Cleanup;}
  // This DH116 reports 0x06020000 for 0x2022:15. Keep the manufacturer's
  // existing homing parameters; the 200-per-mille cap applies to position moves.
  std::cout<<"Homing uses existing device parameters; position current cap=200 per-mille."<<std::endl;
  if(sdk_handle->set_max_current(0,kMaxCurrent)!=0 || sdk_handle->set_enable(0, 1)!=0){return_code=3;goto Cleanup;}

  std::cout << "Waiting for enable..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

  std::cout << "Homing slave..." << std::endl;
  if(!test_running || sdk_handle->home_motors(0)!=0){return_code=4;goto Cleanup;}
  for(int wait=0;wait<kHomeWaitMs && test_running;wait+=20)std::this_thread::sleep_for(std::chrono::milliseconds(20));
  {
    std::vector<uint8_t> raw(ec_master->getSlaveInputSize(1));
    if(!ec_master->getSlaveInputs(1,raw.data(),raw.size()) || raw.size()!=192 || raw[0]!=0 || raw[1]!=27){std::cerr<<"No verified joint frame after home\n";return_code=5;goto Cleanup;}
    for(int axis=0;axis<6;axis++)if(!(raw[164+axis]&16)){std::cerr<<"Homing not confirmed on axis "<<axis+1<<" status="<<int(raw[164+axis])<<"\n";return_code=6;goto Cleanup;}
  }

#if 1  // motion test
  // ---------------------------------------------------------------------------
  // 5. 运动测试
  // 5. Single-slave motion example.
  // ---------------------------------------------------------------------------
  std::cout << "Starting cyclic motion test..." << std::endl;

  for (int cycle=0;cycle<1 && test_running;++cycle) {
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
      if (is_escape_pressed()) {
        std::cout << "Esc pressed, exiting..." << std::endl;
        goto Cleanup;
      }

      if(!ec_master->isConnected()){return_code=10;goto Cleanup;}
      configure_motion(*sdk_handle, positions[i], dof_active);

      const int motion_result = sdk_handle->move_motors();
      print_positions(i, positions[i], motion_result);

      if (motion_result != SDK_NS::LER_NONE) {
        return_code = motion_result;
        goto Cleanup;
      }

      bool reached=false;
      for(int wait=0;wait<kMotionStepDelayMs && test_running;wait+=100){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(!ec_master->isConnected()){return_code=10;goto Cleanup;}
        reached=true;
        if(wait%500==0)std::cout<<"SAMPLE step="<<i<<" elapsed_ms="<<wait<<" ";
        for(int axis=0;axis<6;axis++){
          int pos=-9999,current=-9999,alarm=-9999;int rc=sdk_handle->get_now_position(axis+1,&pos);
          sdk_handle->get_now_current(axis+1,&current);sdk_handle->get_now_alarm(axis+1,&alarm);
          if(wait%500==0)std::cout<<"["<<axis+1<<","<<pos<<","<<current<<","<<alarm<<"]";
          if(rc || std::abs(pos-positions[i][axis])>200)reached=false;
        }
        if(wait%500==0)std::cout<<std::endl;
        if(reached)break;
      }
      if(!test_running || !reached){return_code=8;std::cout<<"Target not reached within bounded wait"<<std::endl;goto Cleanup;}

    }
  }

  test_passed=test_running;
#else  // sensor test
  // ---------------------------------------------------------------------------
  // 5. 传感器测试
  // 5. Sensor read example.
  // ---------------------------------------------------------------------------
  std::cout << "Reading sensor data..." << std::endl;
  sdk_handle->set_sensor_enable(true);

  std::vector<int> finger_sensor_id_list = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1};
  const int palm_sensor_id = SDK_NS::LSS_HAND_PALM;

  {
    bool first_frame = true;
    while (true) {
      if (is_escape_pressed()) {
        std::cout << "\nEsc pressed, exiting..." << std::endl;
        goto Cleanup;
      }

      print_sensor_force_data(*sdk_handle, finger_sensor_id_list, palm_sensor_id,
                              first_frame);
      first_frame = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
#endif

Cleanup:
  // 使用统一清理路径，确保线程退出、SDK 关闭和主站停止都能一致执行。
  // Use a single cleanup path so thread shutdown and SDK close happen
  // consistently on normal exit and on early error.
  int stop_rc=sdk_handle->stop_motors(0);
  int disable_rc=sdk_handle->set_enable(0,0);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  refresh_tpdo(*ec_master,slave_id,*sdk_handle);
  bool disabled=ec_master->isConnected();
  for(int axis=1;axis<=6;axis++){int enabled=-1;if(sdk_handle->get_enable(axis,&enabled)!=0 || enabled!=0)disabled=false;}
  if(ec_master->isConnected() && disabled){
    int sensor_rc=sdk_handle->set_sensor_enable(1);
    std::cout<<"SENSOR_ENABLE rc="<<sensor_rc<<std::endl;
    const std::vector<int> ids={1,3,5,7,9};
    for(int k=0;k<100 && test_running;k++){
      refresh_tpdo(*ec_master,slave_id,*sdk_handle);
      std::vector<uint8_t> raw(ec_master->getSlaveInputSize(slave_id));
      if(ec_master->getSlaveInputs(slave_id,raw.data(),raw.size())){
        std::cout<<"SENSOR_RAW ";for(auto byte:raw)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);std::cout<<std::dec<<std::endl;
      }
      for(int id:ids){FingerForceData force;bool ok=read_finger_force_data(*sdk_handle,id,force);std::cout<<"SENSOR id="<<id<<" ok="<<ok<<" ";for(size_t t=0;t<force.normal_force.size();t++)std::cout<<format_finger_force_sample(force,t)<<" ";std::cout<<std::endl;}
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sdk_handle->set_sensor_enable(0);
  }
  std::cout<<"BOUNDED_TEST passed="<<(test_passed && disabled && stop_rc==0 && disable_rc==0)<<" stop_rc="<<stop_rc<<" disable_rc="<<disable_rc<<" disabled="<<disabled<<std::endl;
  if(!test_passed || !disabled || stop_rc || disable_rc)return_code=9;
  stop_monitor = true;
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  sdk_handle->close();
  ec_master->stop();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
