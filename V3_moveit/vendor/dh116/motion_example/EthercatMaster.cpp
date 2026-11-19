// EthercatMaster.cpp
#include "EthercatMaster.h"

#include "EthercatIoMapBounds.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#if defined(__APPLE__)
#include <CoreFoundation/CoreFoundation.h>
#include <SystemConfiguration/SystemConfiguration.h>
#endif

namespace {
constexpr std::size_t kIoMapCapacity =
    static_cast<std::size_t>(EC_MAXIOSEGMENTS) * EC_MAXLRWDATA;

constexpr int kProcessCycleUs = 1000;

const char* ecStateToString(uint16_t state) {
  switch (state & 0x0F) {
    case EC_STATE_INIT:
      return "INIT";
    case EC_STATE_PRE_OP:
      return "PRE_OP";
    case EC_STATE_SAFE_OP:
      return "SAFE_OP";
    case EC_STATE_OPERATIONAL:
      return "OP";
    default:
      return "UNKNOWN";
  }
}

std::string hexValue(uint32_t value, int width) {
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(width)
      << std::setfill('0') << value;
  return oss.str();
}

std::string sdoName(uint16_t index, uint8_t subindex) {
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(4)
      << std::setfill('0') << index << ":" << std::setw(2)
      << static_cast<int>(subindex);
  return oss.str();
}

// 来自 LSLQ_DH116.xml: <Vendor><Id>#x0017185</Id><Name>Leadshine</Name>
constexpr uint32_t kLslqDh116VendorId = 0x0017185;

bool isLslqDh116Slave(const ec_slavet* slave) {
  return slave && slave->eep_id == 0x00660000 &&
         slave->eep_rev == 0x00000001;
}

#if defined(__APPLE__)
struct MacInterfaceInfo {
  std::string display_name;  // 友好名,如 "USB 10/100/1000 LAN"
  bool is_wireless = false;
};

// 通过 SystemConfiguration 枚举 macOS 网络接口,得到 BSD 名(enX)-> 友好名/类型。
// SOEM 在 macOS 上只返回接口名,这里补充可读名称,便于用户识别物理网口。
std::map<std::string, MacInterfaceInfo> macInterfaceInfo() {
  std::map<std::string, MacInterfaceInfo> result;

  CFArrayRef interfaces = SCNetworkInterfaceCopyAll();
  if (!interfaces)
    return result;

  const CFIndex count = CFArrayGetCount(interfaces);
  for (CFIndex i = 0; i < count; ++i) {
    auto* iface = static_cast<SCNetworkInterfaceRef>(
        const_cast<void*>(CFArrayGetValueAtIndex(interfaces, i)));
    if (!iface)
      continue;

    CFStringRef bsd = SCNetworkInterfaceGetBSDName(iface);
    if (!bsd)
      continue;
    char bsd_buf[256] = {0};
    if (!CFStringGetCString(bsd, bsd_buf, sizeof(bsd_buf),
                            kCFStringEncodingUTF8))
      continue;

    MacInterfaceInfo info;
    CFStringRef display = SCNetworkInterfaceGetLocalizedDisplayName(iface);
    if (display) {
      char name_buf[256] = {0};
      if (CFStringGetCString(display, name_buf, sizeof(name_buf),
                             kCFStringEncodingUTF8))
        info.display_name = name_buf;
    }

    CFStringRef type = SCNetworkInterfaceGetInterfaceType(iface);
    if (type) {
      info.is_wireless =
          CFStringCompare(type, kSCNetworkInterfaceTypeIEEE80211, 0) ==
          kCFCompareEqualTo;
    }

    result[bsd_buf] = info;
  }

  CFRelease(interfaces);
  return result;
}
#endif
}  // namespace

#if ETHERCAT_DEBUG
#define ECOUT std::cout
#else
#define ECOUT \
  if (0)      \
  std::cout
#endif

// 平台特定的网口过滤关键词
#ifdef _WIN32
// Windows特定关键词
static const std::vector<std::string> EXCLUDE_KEYWORDS = {
    "wan miniport",  // Windows WAN Miniport
    "wi-fi",         // Windows WiFi
    "wifi",          // WiFi接口
    "wireless",      // 无线
    "bluetooth",     // 蓝牙
    "vmware",        // VMware虚拟网卡
    "virtual",       // 虚拟设备
    "loopback",      // 回环(Windows)
    "tap-",          // TAP虚拟设备
    "vpn",           // VPN接口
    "wintun",        // Windows TUN设备
    "teredo",        // Teredo隧道
    "isatap"         // ISATAP隧道
};
#elif defined(__APPLE__)
// macOS 特定关键词:排除虚拟/无线隧道接口。en* 可能是有线也可能是 Wi-Fi,
// 不能靠名字排除,这里再配合描述过滤(见 scanNetworkInterfaces)。
static const std::vector<std::string> EXCLUDE_KEYWORDS = {
    "lo",       // 回环(lo0)
    "awdl",     // Apple Wireless Direct Link
    "llw",      // 低延迟 WLAN
    "utun",     // 隧道
    "bridge",   // 网桥
    "anpi",     // Apple Network Proxy Interface
    "vmenet",   // 虚拟机虚拟网卡
    "gif",      // GIF 隧道
    "stf",      // 6to4 隧道
    "p2p",      // 点对点
    "ap1",      // 热点接口
    "wi-fi",    // Wi-Fi(描述)
    "iphone"    // iPhone USB 网络共享
};
#else
// Linux特定关键词
static const std::vector<std::string> EXCLUDE_KEYWORDS = {
    "lo",       // Linux 回环接口
    "docker",   // Docker虚拟网卡
    "veth",     // 虚拟以太网设备
    "br-",      // 网桥接口
    "virbr",    // 虚拟网桥
    "vmnet",    // VMware虚拟网卡
    "tap",      // TAP虚拟设备
    "tun",      // TUN虚拟设备
    "wlan",     // 无线网卡
    "wlp",      // 无线网卡(新命名)
    "wlx",      // 无线网卡
    "wwan",     // 无线广域网
    "vboxnet",  // VirtualBox虚拟网卡
    "p2p",      // P2P连接
    "teredo",   // Teredo隧道
    "isatap"    // ISATAP隧道
};
#endif

std::string SlaveInfo::toString() const {
  // 状态字符串映射
  const char* state_str = ecStateToString(state);

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Slave %2d [%-16s] State: %8s (0x%02X) AL: 0x%04X (%s)%s", index,
           name.substr(0, 16).c_str(), state_str, state, al_status,
           al_status_str.c_str(), is_lost ? " [LOST]" : "");

  return std::string(buffer);
}

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info) {
  return os << info.toString();
}

namespace {
// ---- CoE 数据类型码（ETG.1000，ETG.1000.6 表） ----
// 本项目自带的旧版 SOEM 头里没有 ECT_* 枚举，这里按规范值补齐常用项。
constexpr uint16_t kEctBoolean = 0x0001;
constexpr uint16_t kEctInteger8 = 0x0002;
constexpr uint16_t kEctInteger16 = 0x0003;
constexpr uint16_t kEctInteger32 = 0x0004;
constexpr uint16_t kEctUnsigned8 = 0x0005;
constexpr uint16_t kEctUnsigned16 = 0x0006;
constexpr uint16_t kEctUnsigned32 = 0x0007;
constexpr uint16_t kEctReal32 = 0x0008;
constexpr uint16_t kEctVisibleString = 0x0009;
constexpr uint16_t kEctOctetString = 0x000A;
constexpr uint16_t kEctUnicodeString = 0x000B;
constexpr uint16_t kEctTimeOfDay = 0x000C;
constexpr uint16_t kEctTimeDifference = 0x000D;
constexpr uint16_t kEctDomain = 0x000F;
constexpr uint16_t kEctInteger24 = 0x0010;
constexpr uint16_t kEctReal64 = 0x0011;
constexpr uint16_t kEctInteger64 = 0x0015;
constexpr uint16_t kEctUnsigned24 = 0x0016;
constexpr uint16_t kEctUnsigned64 = 0x001B;

// ---- SDO Information「对象访问权限」位（按 AL 状态分段）----
constexpr uint16_t kAccessReadPre = 0x01;
constexpr uint16_t kAccessReadSafe = 0x02;
constexpr uint16_t kAccessReadOp = 0x04;
constexpr uint16_t kAccessWritePre = 0x08;
constexpr uint16_t kAccessWriteSafe = 0x10;
constexpr uint16_t kAccessWriteOp = 0x20;

// 单次 SDO 读写字节上限 / 拿不到 SDO Info 时的试探缓冲
constexpr int kSdoMaxBytes = 1024;
constexpr int kSdoProbeBytes = 64;

std::string ectTypeName(uint16_t type) {
  switch (type) {
    case kEctBoolean:
      return "BOOLEAN";
    case kEctInteger8:
      return "INTEGER8";
    case kEctInteger16:
      return "INTEGER16";
    case kEctInteger24:
      return "INTEGER24";
    case kEctInteger32:
      return "INTEGER32";
    case kEctInteger64:
      return "INTEGER64";
    case kEctUnsigned8:
      return "UNSIGNED8";
    case kEctUnsigned16:
      return "UNSIGNED16";
    case kEctUnsigned24:
      return "UNSIGNED24";
    case kEctUnsigned32:
      return "UNSIGNED32";
    case kEctUnsigned64:
      return "UNSIGNED64";
    case kEctReal32:
      return "REAL32";
    case kEctReal64:
      return "REAL64";
    case kEctVisibleString:
      return "VISIBLE_STRING";
    case kEctOctetString:
      return "OCTET_STRING";
    case kEctUnicodeString:
      return "UNICODE_STRING";
    case kEctTimeOfDay:
      return "TIME_OF_DAY";
    case kEctTimeDifference:
      return "TIME_DIFFERENCE";
    case kEctDomain:
      return "DOMAIN";
    default:
      break;
  }
  return "TYPE_" + hexValue(type, 4);
}
}  // namespace

bool EthercatMaster::SdoObjectInfo::readableNow() const {
  switch (al_state & 0x0F) {
    case EC_STATE_OPERATIONAL:
      return (obj_access & kAccessReadOp) != 0;
    case EC_STATE_SAFE_OP:
      return (obj_access & kAccessReadSafe) != 0;
    default:
      return (obj_access & kAccessReadPre) != 0;
  }
}

bool EthercatMaster::SdoObjectInfo::writableNow() const {
  switch (al_state & 0x0F) {
    case EC_STATE_OPERATIONAL:
      return (obj_access & kAccessWriteOp) != 0;
    case EC_STATE_SAFE_OP:
      return (obj_access & kAccessWriteSafe) != 0;
    default:
      return (obj_access & kAccessWritePre) != 0;
  }
}

std::string EthercatMaster::SdoObjectInfo::toString() const {
  if (!valid) {
    return "对象信息: 不可用（从站未提供 SDO Info 条目描述）";
  }
  std::ostringstream oss;
  oss << "对象信息: 0x" << std::uppercase << std::hex << std::setw(4)
      << std::setfill('0') << index << ":" << std::setw(2)
      << static_cast<int>(subindex) << std::dec << std::setfill(' ') << "  "
      << bit_length << " 位 / " << byte_length << " 字节  "
      << ectTypeName(data_type) << "  名称=\""
      << (name.empty() ? std::string("?") : name) << "\"  权限 读["
      << (((obj_access & kAccessReadPre) != 0) ? "P" : "-")
      << (((obj_access & kAccessReadSafe) != 0) ? "S" : "-")
      << (((obj_access & kAccessReadOp) != 0) ? "O" : "-") << "] 写["
      << (((obj_access & kAccessWritePre) != 0) ? "P" : "-")
      << (((obj_access & kAccessWriteSafe) != 0) ? "S" : "-")
      << (((obj_access & kAccessWriteOp) != 0) ? "O" : "-") << "]"
      << "  (当前 " << ecStateToString(al_state) << ": "
      << (readableNow() ? "可读" : "不可读") << " / "
      << (writableNow() ? "可写" : "不可写") << ")";
  return oss.str();
}

std::string EthercatMaster::getLastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void EthercatMaster::clearLastError() {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_.clear();
}

void EthercatMaster::setLastError(const std::string& message) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = message;
}

void EthercatMaster::setStatusCallback(StatusUpdateCallback callback) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_callback_ = std::move(callback);
}

void EthercatMaster::clearStatusCallback() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_callback_ = nullptr;
}

std::string EthercatMaster::getLastStatus() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return last_status_;
}

void EthercatMaster::reportStatus(const std::string& message) {
  StatusUpdateCallback callback;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_ = message;
    callback = status_callback_;
  }
  if (callback) {
    callback(message);
  }
}

int EthercatMaster::getExpectedWkc() const {
  const ec_groupt* grp = context_.grouplist + group_;
  return grp->outputsWKC * 2 + grp->inputsWKC;
}

std::string EthercatMaster::describeSlaveState(int slave_index) const {
  if (slave_index < 1 || slave_index > context_.slavecount) {
    return {};
  }

  const ec_slavet* slave = context_.slavelist + slave_index;
  std::ostringstream oss;
  oss << "S" << slave_index << "=" << ecStateToString(slave->state) << "(0x"
      << std::hex << std::setw(4) << std::setfill('0') << slave->state << ")";

  if (slave->ALstatuscode != 0) {
    oss << " AL=0x" << std::setw(4) << slave->ALstatuscode << " "
        << ec_ALstatuscode2string(slave->ALstatuscode);
  }

  oss << std::dec << std::setfill(' ');
  return oss.str();
}

std::string EthercatMaster::describeAllSlaveStates() const {
  std::ostringstream oss;
  for (int i = 1; i <= context_.slavecount; ++i) {
    if (i > 1) {
      oss << " | ";
    }
    oss << describeSlaveState(i);
  }
  return oss.str();
}

// 取出（并清空）SOEM 错误列表中的内容，返回可读文本。
// ecx_poperror 出队并在列表为空时清掉 ecaterror 标志，因此循环必然收敛。
std::string EthercatMaster::takeSoemErrors(const std::string& phase) {
  std::ostringstream oss;
  ec_errort err;
  while (ecx_poperror(&context_, &err)) {
    const char* text = ecx_err2string(err);
    oss << "[" << phase << "] " << (text ? text : "");
    if (err.Etype == EC_ERR_TYPE_SDO_ERROR ||
        err.Etype == EC_ERR_TYPE_SDOINFO_ERROR) {
      oss << "    说明: " << describeSdoAbort(err.AbortCode) << "\n";
    }
  }
  return oss.str();
}

// SDO abort code 的中文提示（EtherCAT/CoE 标准码，只列常见项）
std::string EthercatMaster::describeSdoAbort(int32_t code) {
  switch (static_cast<uint32_t>(code)) {
    case 0x05030000:
      return "分段传输 toggle 位未改变（通信异常）";
    case 0x05040000:
      return "SDO 协议超时";
    case 0x06010000:
      return "不支持对该对象的访问";
    case 0x06010001:
      return "试图读取只写对象（该对象不可读）";
    case 0x06010002:
      return "试图写入只读对象（该对象不可写）";
    case 0x06020000:
      return "对象不存在：请检查 Index / SubIndex（按十六进制填写）";
    case 0x06040041:
      return "对象不能映射到 PDO";
    case 0x06040042:
      return "PDO 长度超限";
    case 0x06060000:
      return "硬件错误导致访问失败";
    case 0x06070010:
      return "数据类型不匹配 / 数据长度不合适（例如按 4 字节写 2 字节对象）";
    case 0x06070012:
      return "数据长度过长";
    case 0x06070013:
      return "数据长度过短";
    case 0x06090011:
      return "SubIndex 不存在";
    case 0x06090030:
      return "参数数值超出允许范围";
    case 0x08000000:
      return "一般性错误";
    case 0x08000020:
      return "数据无法传输或保存到应用";
    case 0x08000022:
      return "当前设备状态不允许访问该对象（例如未使能 / 不在 OP）";
    default:
      break;
  }
  return "未知或厂商自定义 abort code";
}

void EthercatMaster::dumpSoemErrors(const std::string& phase) {
  const std::string text = takeSoemErrors(phase);
  if (!text.empty()) {
    ECOUT << text;
  }
}

void EthercatMaster::dumpSlaveDiagnostics(const std::string& phase) {
  ecx_readstate(&context_);
  ECOUT << "[" << phase << "] slave diagnostics:\n";

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    ECOUT << "  " << describeSlaveState(i) << " Name='" << slave->name
          << "' Obits=" << slave->Obits << " Ibits=" << slave->Ibits
          << " Obytes=" << slave->Obytes << " Ibytes=" << slave->Ibytes
          << " hasdc=" << static_cast<int>(slave->hasdc)
          << " CoE=0x" << std::hex << static_cast<int>(slave->CoEdetails)
          << " SM0=" << slave->SM[0].StartAddr << "/" << slave->SM[0].SMlength
          << " SM1=" << slave->SM[1].StartAddr << "/" << slave->SM[1].SMlength
          << " SM2=" << slave->SM[2].StartAddr << "/" << slave->SM[2].SMlength
          << " SM3=" << slave->SM[3].StartAddr << "/" << slave->SM[3].SMlength
          << std::dec << "\n";
  }

  dumpSoemErrors(phase);
}

void EthercatMaster::dumpMappingDiagnostics() const {
  const ec_groupt* grp = context_.grouplist + group_;
  ECOUT << "Group " << static_cast<int>(group_) << " mapping diagnostics:\n";
  ECOUT << "  Obytes=" << grp->Obytes << " Ibytes=" << grp->Ibytes
        << " outputsWKC=" << grp->outputsWKC
        << " inputsWKC=" << grp->inputsWKC
        << " expectedWKC=" << getExpectedWkc()
        << " nsegments=" << grp->nsegments << "\n";

  ECOUT << "  IO segments:";
  for (int i = 0; i < grp->nsegments; ++i) {
    ECOUT << " " << grp->IOsegment[i];
  }
  ECOUT << "\n";

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    ECOUT << "  Slave " << i << " '" << slave->name << "'"
          << " Obits=" << slave->Obits << " Ibits=" << slave->Ibits
          << " Obytes=" << slave->Obytes << " Ibytes=" << slave->Ibytes
          << " Ostartbit=" << static_cast<int>(slave->Ostartbit)
          << " Istartbit=" << static_cast<int>(slave->Istartbit)
          << " FMMUunused=" << static_cast<int>(slave->FMMUunused) << "\n";
  }
}

void EthercatMaster::applyLslqDh116EsiDefaults() {
  if (context_.slavecount < 1) {
    return;
  }

  ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  // 厂商 ID 核对（LSLQ_DH116.xml: Vendor/@Id = #x0017185, Leadshine）。
  // 产品码 + 版本号已经唯一确定型号，这里只做核对与告警、不阻断兜底：同一片
  // E2 数据损坏时厂商字段往往也不可靠，硬性拦截会让现场失去唯一的恢复手段。
  // 若现场要求严格匹配，把下面的 reportStatus 换成 return 即可。
  if (slave->eep_man != 0 && slave->eep_man != kLslqDh116VendorId) {
    reportStatus("ESI后备: 厂商ID不一致 eep_man=" +
                 hexValue(slave->eep_man, 8) + " 期望=" +
                 hexValue(kLslqDh116VendorId, 8) +
                 "（产品码/版本号匹配，仍按 LSLQ_DH116 补齐）");
  }

  const bool sm2_incomplete =
      slave->SM[2].StartAddr == 0 || slave->SM[2].SMlength == 0 ||
      slave->SM[2].SMflags == 0 || slave->SMtype[2] == 0;
  const bool sm3_incomplete =
      slave->SM[3].StartAddr == 0 || slave->SM[3].SMlength == 0 ||
      slave->SM[3].SMflags == 0 || slave->SMtype[3] == 0;

  if (!sm2_incomplete && !sm3_incomplete) {
    ECOUT << "ESI fallback: SM2/SM3 already complete\n";
    reportStatus("ESI后备: SII已读取到完整SM2/SM3配置，无需补齐");
    return;
  }

  if (sm2_incomplete) {
    if (slave->SM[2].StartAddr == 0) {
      slave->SM[2].StartAddr = htoes(0x1100);
    }
    if (slave->SM[2].SMlength == 0) {
      slave->SM[2].SMlength = htoes(64);
    }
    if (slave->SM[2].SMflags == 0) {
      slave->SM[2].SMflags = htoel(0x00010064);
    }
    if (slave->SMtype[2] == 0) {
      slave->SMtype[2] = 3;
    }
  }

  if (sm3_incomplete) {
    if (slave->SM[3].StartAddr == 0) {
      slave->SM[3].StartAddr = htoes(0x1400);
    }
    if (slave->SM[3].SMlength == 0) {
      slave->SM[3].SMlength = htoes(192);
    }
    if (slave->SM[3].SMflags == 0) {
      slave->SM[3].SMflags = htoel(0x00010020);
    }
    if (slave->SMtype[3] == 0) {
      slave->SMtype[3] = 4;
    }
  }

  ECOUT << "ESI fallback: filled missing LSLQ DH116 SM2/SM3 fields\n";
  reportStatus("ESI后备: SII读取SM2/SM3不完整，已按内置配置补齐缺失字段");
}

void EthercatMaster::dumpSiiPdoDiagnostics() {
  if (context_.slavecount < 1) {
    return;
  }

  for (uint8_t type = 0; type <= 1; ++type) {
    ec_eepromPDOt pdo{};
    uint32_t bits = ecx_siiPDO(&context_, 1, &pdo, type);
    std::ostringstream oss;

    oss << "SII PDO type" << static_cast<int>(type) << ": bits=" << bits
        << " entries=" << pdo.nPDO << " indexes=";

    for (uint16_t i = 1; i <= pdo.nPDO; ++i) {
      if (i > 1) {
        oss << ",";
      }
      oss << hexValue(pdo.Index[i], 4) << "(" << pdo.BitSize[i] << "b"
          << ",SM" << static_cast<int>(pdo.SyncM[i]) << ")";
    }

    oss << " SMbits=[";
    for (int sm = 0; sm < EC_MAXSM; ++sm) {
      if (sm > 0) {
        oss << ",";
      }
      oss << "SM" << sm << ":" << pdo.SMbitsize[sm];
    }
    oss << "]";

    ECOUT << oss.str() << "\n";
  }
}

bool EthercatMaster::readSdoValue(uint16_t slave, uint16_t index,
                                  uint8_t subindex, int max_size,
                                  uint32_t* value) {
  if (!value || max_size <= 0 || max_size > 4) {
    return false;
  }

  uint8_t buffer[4] = {};
  int size = max_size;
  int result = ecx_SDOread(&context_, slave, index, subindex, FALSE, &size,
                           buffer, EC_TIMEOUTRXM * 3);

  if (result <= 0 || size <= 0) {
    ECOUT << "ESI verify: CoE read " << sdoName(index, subindex)
          << " unavailable ret=" << result << "\n";
    dumpSoemErrors("ESI-SDO");
    return false;
  }

  uint32_t parsed = 0;
  for (int i = 0; i < size && i < 4; ++i) {
    parsed |= static_cast<uint32_t>(buffer[i]) << (8 * i);
  }

  *value = parsed;
  return true;
}

void EthercatMaster::verifySdoValue(const std::string& name, uint16_t index,
                                    uint8_t subindex, uint32_t expected,
                                    int max_size, int* mismatch_count) {
  uint32_t actual = 0;
  if (!readSdoValue(1, index, subindex, max_size, &actual)) {
    return;
  }

  if (actual != expected) {
    if (mismatch_count) {
      ++(*mismatch_count);
    }
    ECOUT << "ESI mismatch: " << name << " " << sdoName(index, subindex)
          << " actual=" << hexValue(actual, max_size * 2)
          << " expected=" << hexValue(expected, max_size * 2) << "\n";
  } else {
    ECOUT << "ESI OK: " << name << " " << sdoName(index, subindex) << " = "
          << hexValue(actual, max_size * 2) << "\n";
  }
}

void EthercatMaster::verifyPdoMapping(const std::string& name,
                                      uint16_t pdo_index,
                                      uint16_t object_index,
                                      uint8_t expected_entries,
                                      int* mismatch_count) {
  uint32_t actual_count = 0;
  int local_mismatch_count = 0;

  if (!readSdoValue(1, pdo_index, 0, 1, &actual_count)) {
    ECOUT << "ESI verify: skip " << name << " " << hexValue(pdo_index, 4)
          << " CoE mapping read\n";
    return;
  }

  if (actual_count != expected_entries) {
    ++local_mismatch_count;
    ECOUT << "ESI mismatch: " << name << " " << sdoName(pdo_index, 0)
          << " actual=" << actual_count
          << " expected=" << static_cast<int>(expected_entries) << "\n";
  }

  uint8_t count_to_check =
      actual_count < expected_entries ? static_cast<uint8_t>(actual_count)
                                      : expected_entries;

  for (uint8_t subindex = 1; subindex <= count_to_check; ++subindex) {
    uint32_t actual = 0;
    uint32_t expected = (static_cast<uint32_t>(object_index) << 16) |
                        (static_cast<uint32_t>(subindex) << 8) | 0x10;

    if (!readSdoValue(1, pdo_index, subindex, 4, &actual)) {
      ++local_mismatch_count;
      continue;
    }

    if (actual != expected) {
      ++local_mismatch_count;
      ECOUT << "ESI mismatch: " << name << " " << sdoName(pdo_index, subindex)
            << " actual=" << hexValue(actual, 8)
            << " expected=" << hexValue(expected, 8) << "\n";
    }
  }

  if (actual_count > expected_entries) {
    local_mismatch_count +=
        static_cast<int>(actual_count - expected_entries);
    ECOUT << "ESI mismatch: " << name << " extra PDO entries "
          << (actual_count - expected_entries) << "\n";
  }

  if (local_mismatch_count == 0) {
    ECOUT << "ESI OK: " << name << " " << hexValue(pdo_index, 4)
          << " entries=" << static_cast<int>(expected_entries) << "\n";
  } else if (mismatch_count) {
    *mismatch_count += local_mismatch_count;
  }
}

void EthercatMaster::verifyEsiConfiguration() {
  if (context_.slavecount < 1) {
    return;
  }

  const ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  int mismatch_count = 0;

  verifySdoValue("SM2 RxPDO count", 0x1C12, 0, 0x01, 1, &mismatch_count);
  verifySdoValue("SM2 RxPDO assignment", 0x1C12, 1, 0x1600, 2,
                 &mismatch_count);
  verifySdoValue("SM3 TxPDO count", 0x1C13, 0, 0x01, 1, &mismatch_count);
  verifySdoValue("SM3 TxPDO assignment", 0x1C13, 1, 0x1A00, 2,
                 &mismatch_count);

  verifyPdoMapping("RxPDO", 0x1600, 0x9000, 32, &mismatch_count);
  verifyPdoMapping("TxPDO", 0x1A00, 0x9001, 96, &mismatch_count);
  dumpSiiPdoDiagnostics();

  ECOUT << "ESI verify complete: mismatches=" << mismatch_count << "\n";
  // 参考实现只用 ECOUT，而 ETHERCAT_DEBUG 默认未定义 -> 校验等于静默执行。
  // 这里通过状态回调上报，保证现场在状态栏就能看到 ESI 是否一致。
  reportStatus("ESI校验: CoE对象检查完成, 不一致项=" +
               std::to_string(mismatch_count));
}

void EthercatMaster::verifyMappedEsiConfiguration() {
  if (context_.slavecount < 1) {
    return;
  }

  int mismatch_count = 0;
  const ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  const bool sm2_ok =
      etohs(slave->SM[2].StartAddr) == 0x1100 &&
      etohs(slave->SM[2].SMlength) == 64;
  const bool sm3_ok =
      etohs(slave->SM[3].StartAddr) == 0x1400 &&
      etohs(slave->SM[3].SMlength) == 192;

  if (!sm2_ok) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: SM2 actual start="
          << hexValue(etohs(slave->SM[2].StartAddr), 4)
          << " len=" << etohs(slave->SM[2].SMlength)
          << " expected start=0x1100 len=64\n";
  }

  if (!sm3_ok) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: SM3 actual start="
          << hexValue(etohs(slave->SM[3].StartAddr), 4)
          << " len=" << etohs(slave->SM[3].SMlength)
          << " expected start=0x1400 len=192\n";
  }

  if (slave->Obytes != 64 || slave->Ibytes != 192) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: slave IO size actual=" << slave->Obytes << "O+"
          << slave->Ibytes << "I expected=64O+192I\n";
  }

  const ec_groupt* grp = context_.grouplist + group_;
  if (grp->Obytes != 64 || grp->Ibytes != 192) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: group IO size actual=" << grp->Obytes << "O+"
          << grp->Ibytes << "I expected=64O+192I\n";
  }

  ECOUT << "ESI mapped verify complete: mismatches=" << mismatch_count << "\n";
  reportStatus("ESI校验: 映射后SM/IO大小检查完成, 不一致项=" +
               std::to_string(mismatch_count));
}

EthercatMaster::EthercatMaster()
    : group_(0),
      roundtrip_time_(0),
      initialized_(false),
      started_(false),
      connected_index_(-1),
      running_(false),
      outputs_(nullptr),
      inputs_(nullptr),
      output_bytes_(0),
      input_bytes_(0) {
  memset(&context_, 0, sizeof(context_));
}

EthercatMaster::~EthercatMaster() {
  stop();
}

std::vector<std::string> EthercatMaster::scanNetworkInterfaces() {
  interfaces_.clear();
  std::vector<std::string> descrptions;
  ec_adaptert* adapter = ec_find_adapters();
  ec_adaptert* head = adapter;

#if defined(__APPLE__)
  const std::map<std::string, MacInterfaceInfo> mac_info = macInterfaceInfo();
#endif

  ECOUT << "Available adapters:\n";
  while (adapter != nullptr) {
    // 检查是否需要排除
    bool should_exclude = false;
    std::string adapter_name(adapter->name);
    std::string adapter_desc(adapter->desc);
    std::string adapter_name_lower = adapter_name;
    std::string adapter_desc_lower = adapter_desc;

    // 转换为小写便于比较
    for (char& c : adapter_name_lower) {
      c = std::tolower(static_cast<unsigned char>(c));
    }
    for (char& c : adapter_desc_lower) {
      c = std::tolower(static_cast<unsigned char>(c));
    }

#if defined(__APPLE__)
    // macOS:SOEM 的 desc 就是接口名,不能用于区分有线/Wi-Fi。
    // 改用 SystemConfiguration 的接口类型排除无线接口;并把友好名作为描述。
    const auto mac_it = mac_info.find(adapter_name);
    if (mac_it != mac_info.end()) {
      if (mac_it->second.is_wireless)
        should_exclude = true;
      if (!mac_it->second.display_name.empty() &&
          mac_it->second.display_name != adapter_name) {
        adapter_desc = adapter_name + " (" + mac_it->second.display_name + ")";
      }
    }
#endif

    for (const std::string& keyword : EXCLUDE_KEYWORDS) {
      // 检查名称
      if (adapter_name_lower.find(keyword) != std::string::npos) {
        should_exclude = true;
        break;
      }
#if defined(_WIN32) || defined(__APPLE__)
      // 检查描述
      if (adapter_desc_lower.find(keyword) != std::string::npos) {
        should_exclude = true;
        break;
      }
#endif
    }

    if (!should_exclude) {
      ECOUT << "    - " << adapter_name << "  (" << adapter_desc << ")\n";
      interfaces_.push_back(adapter_name);
      descrptions.push_back(adapter_desc);
    }

    adapter = adapter->next;
  }
  ec_free_adapters(head);

  return descrptions;
}

bool EthercatMaster::probeSlaves(int index, std::vector<SlaveInfo>& slaves) {
  clearLastError();
  slaves.clear();

  if (running_.load() || initialized_.load()) {
    setLastError("Already connected");
    return false;
  }

  if (index < 0 || index >= static_cast<int>(interfaces_.size())) {
    setLastError("Network interface selection");
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  ecx_contextt probe_context;
  std::memset(&probe_context, 0, sizeof(probe_context));

  const std::string iface = interfaces_.at(index);
  if (!ecx_init(&probe_context, iface.c_str())) {
    setLastError("Open network interface");
    return false;
  }

  bool ok = false;
  if (ecx_config_init(&probe_context) <= 0) {
    setLastError("Scan EtherCAT slaves");
  } else {
    ecx_readstate(&probe_context);
    for (int i = 1; i <= probe_context.slavecount; ++i) {
      const ec_slavet* slave = probe_context.slavelist + i;
      SlaveInfo info;
      info.index = i;
      info.name = std::string(slave->name);
      info.state = slave->state;
      info.al_status = slave->ALstatuscode;
      info.al_status_str =
          std::string(ec_ALstatuscode2string(slave->ALstatuscode));
      info.is_lost = slave->islost;
      slaves.push_back(info);
    }
    ok = !slaves.empty();
    if (!ok) {
      setLastError("Scan EtherCAT slaves");
    }
  }

  ecx_close(&probe_context);
  if (ok) {
    clearLastError();
  }
  return ok;
}

bool EthercatMaster::init(int index) { return initInternal(index, false); }

bool EthercatMaster::initInternal(int index, bool for_reconnect) {
  clearLastError();

  // 重连路径由通信线程自己调用：此时 running_ 为 true、initialized_ 已被
  // attemptReconnect() 关闭，因此不能再按「已连接」拒绝。
  if (!for_reconnect && (running_.load() || initialized_.load())) {
    setLastError("Already connected");
    return false;
  }

  if (index < 0 || index >= interfaces_.size()) {
    setLastError("Network interface selection");
    ECOUT << "Index not available\n";
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  iface_ = interfaces_.at(index);

  ECOUT << "Initializing SOEM on '" << iface_ << "'... " << std::flush;
  if (!ecx_init(&context_, iface_.c_str())) {
    setLastError("Open network interface");
    ECOUT << "no socket connection\n";
    return false;
  }
  ECOUT << "done\n";

  ECOUT << "Finding autoconfig slaves... " << std::flush;
  if (ecx_config_init(&context_) <= 0) {
    dumpSoemErrors("config_init");
    setLastError("Scan EtherCAT slaves");
    ECOUT << "no slaves found\n";
    ecx_close(&context_);
    return false;
  }
  dumpSoemErrors("config_init");
  ECOUT << context_.slavecount << " slaves found\n";
  applyLslqDh116EsiDefaults();

  ECOUT << "Sequential mapping of I/O... " << std::flush;
  io_map_.assign(kIoMapCapacity, 0);
  int mapped_bytes = ecx_config_map_group(&context_, io_map_.data(), group_);
  ec_groupt* grp = context_.grouplist + group_;

  if (grp->Obytes <= 0 || grp->Ibytes <= 0 || grp->outputs == nullptr ||
      grp->inputs == nullptr) {
    setLastError("I/O mapping");
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
    io_map_.clear();
    ecx_close(&context_);
    return false;
  }

  if (mapped_bytes < 0 ||
      !ethercat_iomap::layoutWithinMapBounds(
          io_map_.data(), io_map_.size(), static_cast<std::size_t>(mapped_bytes),
          grp->outputs, static_cast<std::size_t>(grp->Obytes), grp->inputs,
          static_cast<std::size_t>(grp->Ibytes))) {
    setLastError("I/O mapping");
    ECOUT << "I/O map exceeds local buffer bounds: mapped " << mapped_bytes
          << " bytes, outputs " << grp->Obytes << " bytes, inputs "
          << grp->Ibytes << " bytes, capacity " << io_map_.size() << "\n";
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
    io_map_.clear();
    ecx_close(&context_);
    return false;
  }

  output_bytes_ = static_cast<int>(grp->Obytes);
  input_bytes_ = static_cast<int>(grp->Ibytes);
  outputs_ = grp->outputs;
  inputs_ = grp->inputs;

  //  初始化缓冲区大小
  output_buffer_.resize(output_bytes_);
  if (outputs_ && output_bytes_ > 0) {
    std::memset(outputs_, 0, output_bytes_);
  }

  // 填充每从站 PDO 布局信息
  slave_io_info_.clear();
  for (int i = 1; i <= context_.slavecount; ++i) {
    ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_) continue;
    SlaveIOInfo info;
    info.slave_id      = i;
    info.name          = std::string(slave->name);
    info.output_bytes  = (slave->Obytes > 0 && slave->outputs) ? (int)slave->Obytes : 0;
    info.output_offset = (info.output_bytes > 0 && outputs_)
                             ? static_cast<int>(slave->outputs - outputs_)
                             : -1;
    info.input_bytes   = (slave->Ibytes > 0 && slave->inputs) ? (int)slave->Ibytes : 0;
    info.input_offset  = (info.input_bytes > 0 && inputs_)
                             ? static_cast<int>(slave->inputs - inputs_)
                             : -1;
    slave_io_info_.push_back(info);
  }

  ECOUT << "mapped " << grp->Obytes << "O+" << grp->Ibytes << "I bytes from "
        << grp->nsegments << " segments";

  if (grp->nsegments > 1) {
    ECOUT << " (";
    for (int i = 0; i < grp->nsegments; ++i) {
      if (i > 0)
        ECOUT << "+";
      ECOUT << grp->IOsegment[i];
    }
    ECOUT << " slaves)";
  }
  ECOUT << "\n";
  ECOUT << "Mapped logical bytes: " << mapped_bytes << "\n";
  dumpMappingDiagnostics();
  dumpSoemErrors("config_map");
  verifyMappedEsiConfiguration();
#if ETHERCAT_DEBUG
  verifyEsiConfiguration();
#endif

  ECOUT << "Configuring distributed clock... " << std::flush;
  ecx_configdc(&context_);
  dumpSoemErrors("configdc");
  ECOUT << "done\n";

  connected_index_ = index;
  sdo_info_cache_.clear();  // 新 context，条目描述缓存作废
  initialized_.store(true);
  connection_lost_.store(false);
  clearLastError();
  return true;
}

bool EthercatMaster::start() {
  clearLastError();

  if (!initialized_.load()) {
    setLastError("SOEM init");
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (context_.slavecount < 1) {
    setLastError("Scan EtherCAT slaves");
    return false;
  }

  // ec_groupt* grp = context_.grouplist + group_;
  ec_slavet* slave = context_.slavelist;

  ECOUT << "Waiting for all slaves in safe operational... " << std::flush;
  uint16 safe_state =
      ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ecx_readstate(&context_);
  if (safe_state != EC_STATE_SAFE_OP ||
      context_.slavelist[0].state != EC_STATE_SAFE_OP) {
    dumpSlaveDiagnostics("SAFE_OP");
    setLastError("SAFE_OP");
    ECOUT << "failed to reach SAFE_OP, " << describeAllSlaveStates() << "\n";
    return false;
  }
  ECOUT << "done\n";

  ECOUT << "Send a roundtrip to make outputs in slaves happy... " << std::flush;
  ec_timet start = osal_current_time();
  ecx_send_processdata(&context_);
  int first_wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
  ec_timet end = osal_current_time();
  ec_timet diff;
  osal_time_diff(&start, &end, &diff);
  roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);
  ECOUT << " first WKC " << first_wkc << " expected " << getExpectedWkc();
  if (first_wkc < getExpectedWkc()) {
    ECOUT << " warning";
  }
  ECOUT << "done\n";

  ECOUT << "Setting operational state.." << std::flush;

  slave->state = EC_STATE_OPERATIONAL;
  ecx_writestate(&context_, 0);

  constexpr int max_op_attempts = 2000;
  for (int i = 0; i < max_op_attempts; ++i) {
    if ((i % 100) == 0) {
      ECOUT << "." << std::flush;
    }
    start = osal_current_time();
    ecx_send_processdata(&context_);
    int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
    end = osal_current_time();
    osal_time_diff(&start, &end, &diff);
    roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

    if ((i % 10) == 0) {
      ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
    }
    if (slave->state == EC_STATE_OPERATIONAL) {
      ECOUT << " all slaves are now operational\n";
      started_.store(true);
      connection_lost_.store(false);
      resetLostFrames();
      reportStatus("运行: 通讯正常(OPERATIONAL), 往返 " +
                   std::to_string(roundtrip_time_) + "us");
      return true;
    }
    if ((i % 100) == 0) {
      ECOUT << "(wkc=" << wkc << "/" << getExpectedWkc()
            << ", state=0x" << std::hex << slave->state << std::dec << ")";
    }
    osal_usleep(kProcessCycleUs);
  }

  ECOUT << " failed,";
  ecx_readstate(&context_);
  dumpSlaveDiagnostics("OP");
  setLastError("OPERATIONAL");
  for (int i = 1; i <= context_.slavecount; ++i) {
    slave = context_.slavelist + i;
    if (slave->state != EC_STATE_OPERATIONAL) {
      ECOUT << " slave " << i << " is 0x" << std::hex << std::setfill('0')
            << std::setw(4) << slave->state << " (AL-status=0x" << std::setw(4)
            << slave->ALstatuscode << std::dec << " "
            << ec_ALstatuscode2string(slave->ALstatuscode) << ")";
    }
  }
  ECOUT << "\n";

  return false;
}

void EthercatMaster::run() {
  if (!started_.load() || running_.load()) {
    return;
  }

  running_.store(true);

  worker_thread_ = std::thread([this]() {
    reportStatus("运行: 工作线程已启动");
    int min_time = 0, max_time = 0;
    int iteration = 1;
    int consecutive_lost_frames = 0;     // 连续丢帧计数
    int reconnect_counter = 0;           // 重连节拍计数
    constexpr int max_lost_frames = 10;  // 最大断线丢帧判断数
    // 每 200ms 尝试重连一次（每周期 1ms）
    constexpr int reconnect_interval_cycles = 200000 / kProcessCycleUs;

    while (running_.load()) {
      // FoE / AL 状态切换期间暂停周期过程数据：两者与过程数据共用同一个网卡
      // socket 和 mailbox，并发会导致收发错乱。
      if (io_pause_requested_.load()) {
        io_pause_ack_.store(true);
        osal_usleep(kProcessCycleUs);
        continue;
      }
      io_pause_ack_.store(false);

      // ---- 自动重连模式 ----
      if (is_reconnecting_.load()) {
        // Bounded motion validation never resumes/replays after a link fault.
        connection_lost_.store(true);running_.store(false);break;
        lost_frames_.fetch_add(1);  // 重连期间按丢帧累计
        if (++reconnect_counter >= reconnect_interval_cycles) {
          reconnect_counter = 0;
          reportStatus("运行: 尝试重连");
          ECOUT << "Attempting to reconnect... ";
          if (attemptReconnect()) {
            consecutive_lost_frames = 0;
            iteration = 1;
            ECOUT << "Success!\n";
            reportStatus("运行: 重连成功");
          } else {
            ECOUT << "Failed\n";
          }
        } else {
          osal_usleep(kProcessCycleUs);
        }
        continue;
      }

      ECOUT << "Iteration " << std::setw(4) << std::setfill(' ') << iteration
            << ":";

      int wkc = 0;
      int expected_wkc = 0;
      int64 dc_time = 0;
      std::vector<uint8_t> output_dump;
      std::vector<uint8_t> input_dump;

      // 先将新提交的输出同步到 SOEM 输出缓冲区:
      // 纯内存拷贝,不涉及 SOEM 上下文,置于 soem_mutex_ 之外,
      // 避免阻塞 SDO 读写,也避免被长 SDO 拖慢发送周期
      {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        std::vector<uint8_t> frame;
        if (output_buffer_.consume(frame)) {
          // (std::min) 括号规避 Windows min 宏污染(SOEM 头链入 windows.h)
          const size_t copy_len = (std::min)(
              frame.size(), static_cast<size_t>(output_bytes_));
          std::memcpy(outputs_, frame.data(), copy_len);
        }
      }

      {
        std::lock_guard<std::mutex> soem_lock(soem_mutex_);

        if (!started_.load() || connection_lost_.load() || outputs_ == nullptr ||
            inputs_ == nullptr || output_bytes_ <= 0 || input_bytes_ <= 0) {
          setLastError("Process data");
          reportStatus("运行: 过程数据不可用，进入自动重连");
          is_reconnecting_.store(true);
          reconnect_counter = 0;
          continue;
        }

        ec_timet start = osal_current_time();
        ecx_send_processdata(&context_);
        wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
        ec_timet end = osal_current_time();
        ec_timet diff;
        osal_time_diff(&start, &end, &diff);
        roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

        expected_wkc = getExpectedWkc();
        if (expected_wkc > 0 && wkc == expected_wkc) {
          verified_inputs_.assign(inputs_, inputs_ + input_bytes_);
          ++verified_sequence_;
          verified_stamp_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }
        dc_time = context_.DCtime;
#if ETHERCAT_DEBUG
        ec_groupt* grp = context_.grouplist + group_;
        output_dump.assign(grp->outputs, grp->outputs + grp->Obytes);
        input_dump.assign(grp->inputs, grp->inputs + grp->Ibytes);
#endif
      }

      if (expected_wkc <= 0) {
        setLastError("Process data");
        reportStatus("运行: 期望 WKC 无效，进入自动重连");
        is_reconnecting_.store(true);
        reconnect_counter = 0;
        continue;
      }

      // 统计丢帧：WKC 小于期望值就认为是丢帧。
      if (wkc < expected_wkc) {
        // 非实时系统上偶发的单次 WKC 不匹配(网卡中断/调度时序抖动)很常见,
        // 不为此上报"疑似丢帧"状态(避免刷日志),仅累计丢帧数。
        lost_frames_.fetch_add(1);   // 增加丢帧计数
        consecutive_lost_frames++;  // 增加连续丢帧计数

        ECOUT << std::setw(6) << roundtrip_time_ << " usec  WKC " << wkc;
        ECOUT << " wrong (expected " << expected_wkc
              << "), consecutive: " << consecutive_lost_frames << "\n";

        // 连续丢帧超过阈值，判定链路断开，由上层决定是否重新连接。
        if (consecutive_lost_frames >= max_lost_frames) {
          ECOUT << "Connection lost!\n";
          setLastError("Process data");
          reportStatus("运行: 连续丢帧超过阈值，开始自动重连");
          is_reconnecting_.store(true);
          reconnect_counter = 0;
          continue;
        }
      } else {
        // WKC正常，重置连续丢帧计数
        consecutive_lost_frames = 0;

        ECOUT << std::setw(6) << roundtrip_time_ << " usec  WKC " << wkc;
        ECOUT << "  O:";
        for (uint8_t value : output_dump) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value) << std::dec;
        }
        ECOUT << "  I:";
        for (uint8_t value : input_dump) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value) << std::dec;
        }
        ECOUT << "  T: " << std::dec << dc_time << "\r" << std::flush;
      }

      if (iteration == 1) {
        min_time = max_time = roundtrip_time_;
      } else {
        if (roundtrip_time_ < min_time)
          min_time = roundtrip_time_;
        if (roundtrip_time_ > max_time)
          max_time = roundtrip_time_;
      }

      iteration++;
      osal_usleep(kProcessCycleUs);
    }

    ECOUT << "\nRoundtrip time (usec): min " << min_time << " max " << max_time
          << "\n";
  });
}

EthercatMaster::EthercatState EthercatMaster::getState() const {
  // 重连中 context 正在被销毁/重建，必须先于 initialized_ 判断返回
  if (is_reconnecting_.load()) {
    return EthercatState::Reconnecting;
  }

  if (!running_.load() || !initialized_.load() || connection_lost_.load()) {
    return EthercatState::Disconnected;
  }

  // 与重连/长事务（FoE、AL 状态切换、SDO）互斥。
  // 关键：通信线程每个 1ms 周期都会短暂持有 soem_mutex_ 收发过程数据，而 UI
  // 每 100ms 轮询一次，撞上它的概率不低——如果"一次 try_lock 失败"就上报
  // Reconnecting，状态栏就会在 OP 与 Reconnecting 之间来回闪。
  // 因此先重试约 5ms 等它让出锁，仍拿不到才沿用上一次的状态。
  std::unique_lock<std::mutex> soem_lock(soem_mutex_, std::try_to_lock);
  if (!soem_lock.owns_lock() && !lockSoemBriefly(soem_lock)) {
    return static_cast<EthercatState>(cached_state_.load());
  }
  if (context_.slavecount < 1) {
    return EthercatState::Disconnected;
  }

  EthercatMaster* mutable_this = const_cast<EthercatMaster*>(this);
  ecx_readstate(&mutable_this->context_);

  // 将单个从站状态映射到 EthercatState（优先级从低到高）
  auto slave_state_to_ethercat = [](uint16_t slave_state) -> EthercatState {
    if (slave_state & EC_STATE_ERROR)
      return EthercatState::Error;
    if (slave_state == EC_STATE_NONE)
      return EthercatState::Disconnected;
    switch (slave_state) {
      case EC_STATE_INIT:
      case EC_STATE_PRE_OP:
        return EthercatState::Initializing;
      case EC_STATE_SAFE_OP:
        return EthercatState::SafeOperational;
      case EC_STATE_OPERATIONAL:
        return EthercatState::Operational;
      default:
        return EthercatState::Initializing;
    }
  };

  // 优先级顺序（数值越小越差），用于取"最差"状态
  auto state_priority = [](EthercatState s) -> int {
    switch (s) {
      case EthercatState::Error:           return 0;
      case EthercatState::Disconnected:    return 1;
      case EthercatState::Initializing:    return 2;
      case EthercatState::SafeOperational: return 3;
      case EthercatState::Operational:     return 4;
      default:                             return 0;
    }
  };

  // 遍历所有属于本 group 的从站，取最差状态
  bool found_any = false;
  EthercatState worst = EthercatState::Operational;

  constexpr int MAX_NONE_COUNT = 3;

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_)
      continue;

    found_any = true;
    EthercatState s = slave_state_to_ethercat(slave->state);
    if (state_priority(s) < state_priority(worst))
      worst = s;
  }

  if (!found_any) {
    cached_state_.store(static_cast<int>(EthercatState::Disconnected));
    return EthercatState::Disconnected;
  }

  // NONE 状态防抖：连续 MAX_NONE_COUNT 次才确认断开
  if (worst == EthercatState::Disconnected) {
    none_state_count_++;
    if (none_state_count_ >= MAX_NONE_COUNT) {
      none_state_count_ = 0;
      cached_state_.store(static_cast<int>(EthercatState::Disconnected));
      return EthercatState::Disconnected;
    }
    cached_state_.store(static_cast<int>(EthercatState::Initializing));
    return EthercatState::Initializing;
  } else {
    none_state_count_ = 0;
  }

  cached_state_.store(static_cast<int>(worst));
  return worst;
}

bool EthercatMaster::isConnected() const {
  return running_.load() && initialized_.load() && started_.load() &&
         !connection_lost_.load();
}

std::vector<SlaveInfo> EthercatMaster::getSlaveInfoList() const {
  std::vector<SlaveInfo> result;

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_)
      continue;

    SlaveInfo info;
    info.index = i;
    info.name = std::string(slave->name);
    info.state = slave->state;
    info.al_status = slave->ALstatuscode;
    info.al_status_str =
        std::string(ec_ALstatuscode2string(slave->ALstatuscode));
    info.is_lost = slave->islost;
    result.push_back(info);
  }

  return result;
}

bool EthercatMaster::verifyDh116Identity(int slave_id) const {
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (!initialized_.load() || slave_id < 1 || slave_id > context_.slavecount) return false;
  const auto* slave = context_.slavelist + slave_id;
  return isLslqDh116Slave(slave) && slave->eep_man == kLslqDh116VendorId;
}

uint64_t EthercatMaster::getLostFrames() const {
  return lost_frames_.load();
}

void EthercatMaster::resetLostFrames() {
  lost_frames_.store(0);
}

void EthercatMaster::stop() {
  // 标记停止中：重连流程不得再拉起新的 context
  stopping_.store(true);
  io_pause_requested_.store(false);

  if (running_.load()) {
    ECOUT << "Stopping EtherCAT thread... " << std::flush;
    running_.store(false);
    reportStatus("停止: 正在停止通信线程");
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
    ECOUT << "done\n";

    // 显示最终的丢帧统计
    ECOUT << "Total lost frames: " << lost_frames_.load() << "\n";
  }

  {
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);
    if (started_.load()) {
      if (!connection_lost_.load() && context_.slavecount > 0) {
        ec_slavet* slave = context_.slavelist;
        ECOUT << "Requesting init state on all slaves... " << std::flush;
        slave->state = EC_STATE_INIT;
        ecx_writestate(&context_, 0);
        ECOUT << "done\n";
      }
      started_.store(false);
    }

    if (initialized_.load()) {
      ECOUT << "Close socket... " << std::flush;
      ecx_close(&context_);
      ECOUT << "done\n";
      initialized_.store(false);
    }
    sdo_info_cache_.clear();
  }

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
  }
  output_buffer_.resize(0);
  io_map_.clear();
  slave_io_info_.clear();
  connected_index_ = -1;
  none_state_count_ = 0;
  resetLostFrames();
  connection_lost_.store(false);
  is_reconnecting_.store(false);
  io_pause_ack_.store(false);
  stopping_.store(false);
  saved_mailbox_slave_ = -1;  // 新 context，旧的邮箱保存值作废
}

void EthercatMaster::setOutput(int slave_id, int index, uint8_t value) {
  // 通信线程必须实际在运行,否则数据只写入缓冲区而永不下发,
  // 上层会误以为发送成功
  if (!started_.load() || !running_.load() || connection_lost_.load()) return;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) return;
  if (index < 0 || index >= info->output_bytes) return;
  output_buffer_.write_byte(static_cast<size_t>(info->output_offset + index), value);
}

uint8_t EthercatMaster::getInput(int slave_id, int index) {
  if (!started_.load() || connection_lost_.load() || inputs_ == nullptr) return 0;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->input_bytes == 0 || info->input_offset < 0) return 0;
  if (index < 0 || index >= info->input_bytes) return 0;
  std::lock_guard<std::mutex> lock(io_mutex_);
  return inputs_[info->input_offset + index];
}

// ---------------------------------------------------------------------------
// 按从站粒度的 PDO 操作
// ---------------------------------------------------------------------------

const SlaveIOInfo* EthercatMaster::findSlaveInfo(int slave_id) const {
  for (const auto& info : slave_io_info_) {
    if (info.slave_id == slave_id) return &info;
  }
  return nullptr;
}

int EthercatMaster::getSlaveCount() const {
  return static_cast<int>(slave_io_info_.size());
}

int EthercatMaster::getSlaveOutputSize(int slave_id) const {
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  return info ? info->output_bytes : 0;
}

int EthercatMaster::getSlaveInputSize(int slave_id) const {
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  return info ? info->input_bytes : 0;
}

bool EthercatMaster::setSlaveOutputs(int slave_id, const uint8_t* data,
                                     unsigned int len) {
  // 通信线程必须实际在运行,否则数据只写入缓冲区而永不下发,
  // 上层(SDK 发送回调)会误以为发送成功
  if (!started_.load() || !running_.load() || connection_lost_.load() ||
      !data || len == 0)
    return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) {
    ECOUT << "setSlaveOutputs: slave " << slave_id << " has no outputs\n";
    return false;
  }
  if (len > (unsigned int)info->output_bytes) {
    ECOUT << "setSlaveOutputs: len " << len << " exceeds slave " << slave_id
          << " output bytes " << info->output_bytes << "\n";
    return false;
  }
  return output_buffer_.write_slice(
      static_cast<size_t>(info->output_offset), data, len);
}

bool EthercatMaster::getSlaveInputs(int slave_id, uint8_t* buffer,
                                    unsigned int len) const {
  if (!started_.load() || connection_lost_.load() || !buffer || len == 0) return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->input_bytes == 0 || info->input_offset < 0) {
    ECOUT << "getSlaveInputs: slave " << slave_id << " has no inputs\n";
    return false;
  }
  if (len > (unsigned int)info->input_bytes) {
    ECOUT << "getSlaveInputs: len " << len << " exceeds slave " << slave_id
          << " input bytes " << info->input_bytes << "\n";
    return false;
  }
  std::lock_guard<std::mutex> lock(io_mutex_);
  std::memcpy(buffer, inputs_ + info->input_offset, len);
  return true;
}

// Snapshot and timestamp share the process-data mutex: failed WKC never refreshes a lease.
bool EthercatMaster::getVerifiedInputs(int slave_id, uint8_t* buffer, unsigned int len,
                                      uint64_t& sequence, double& stamp) const {
  std::lock_guard<std::mutex> lock(soem_mutex_);
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!started_.load() || connection_lost_.load() || !buffer || !info ||
      info->input_offset < 0 || len != unsigned(info->input_bytes) ||
      verified_inputs_.size() < size_t(info->input_offset) + len || !verified_sequence_) return false;
  std::memcpy(buffer, verified_inputs_.data() + info->input_offset, len);
  sequence = verified_sequence_; stamp = verified_stamp_; return true;
}

bool EthercatMaster::getSlaveOutputs(int slave_id, uint8_t* buffer,
                                     unsigned int len) const {
  if (!started_.load() || connection_lost_.load() || !buffer || len == 0) return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) {
    ECOUT << "getSlaveOutputs: slave " << slave_id << " has no outputs\n";
    return false;
  }
  if (len > (unsigned int)info->output_bytes) {
    ECOUT << "getSlaveOutputs: len " << len << " exceeds slave " << slave_id
          << " output bytes " << info->output_bytes << "\n";
    return false;
  }
  return output_buffer_.read_slice(
      static_cast<size_t>(info->output_offset), buffer, len);
}

// ---------------------------------------------------------------------------
// SDO 读写
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SDO 辅助：锁重试、目标校验、对象条目描述（SDO Information 服务）
// ---------------------------------------------------------------------------

// 通信线程每个 1ms 周期都会短暂持有 soem_mutex_。UI 每 100ms 轮询状态，
// 一次 try_lock 失败并不代表出问题，先重试 ~10ms（让出 CPU 给通信线程）。
bool EthercatMaster::lockSoemBriefly(std::unique_lock<std::mutex>& lock) const {
  for (int i = 0; i < 5 && !lock.owns_lock(); ++i) {
    osal_usleep(1000);  // 1ms，最多等 5ms
    if (lock.try_lock()) {
      break;
    }
  }
  return lock.owns_lock();
}

bool EthercatMaster::checkSdoTargetLocked(int slave, std::string* error) const {
  if (!initialized_.load()) {
    if (error) {
      *error = "SOEM 未初始化，请先「连接」";
    }
    return false;
  }
  if (slave < 1 || slave > context_.slavecount) {
    if (error) {
      *error = "从站号无效: " + std::to_string(slave) + "（当前从站数 " +
               std::to_string(context_.slavecount) + "）";
    }
    return false;
  }
  return true;
}

// 查询单个对象的条目描述（要求已持有 soem_mutex_）。
// ecx_readOEsingle 只用到 pODlist 的 Slave 与 Index[Item]，不需要先把整份对象
// 字典列表读回来，所以这里手工填一个最小 OD 列表，代价只有一次邮箱往返。
bool EthercatMaster::querySdoObjectInfoLocked(int slave, uint16_t index,
                                              uint8_t subindex,
                                              SdoObjectInfo* info,
                                              std::string* error) {
  if (!info) {
    return false;
  }
  *info = SdoObjectInfo{};
  info->index = index;
  info->subindex = subindex;

  const uint32_t key = (static_cast<uint32_t>(slave) << 24) |
                       (static_cast<uint32_t>(index) << 8) | subindex;
  const auto cached = sdo_info_cache_.find(key);
  if (cached != sdo_info_cache_.end()) {
    *info = cached->second;
    return info->valid;
  }

  if (slave < 1 || slave > context_.slavecount) {
    if (error) {
      *error = "从站号无效: " + std::to_string(slave);
    }
    return false;
  }

  // 结构较大（od 列表约 48KB、oe 列表约 12KB），放堆上避免爆栈
  auto od_list = std::make_unique<ec_ODlistt>();
  auto oe_list = std::make_unique<ec_OElistt>();
  std::memset(od_list.get(), 0, sizeof(ec_ODlistt));
  std::memset(oe_list.get(), 0, sizeof(ec_OElistt));
  od_list->Slave = static_cast<uint16>(slave);
  od_list->Index[0] = index;

  const int wkc =
      ecx_readOEsingle(&context_, 0, subindex, od_list.get(), oe_list.get());
  if (wkc <= 0) {
    // 失败原因（SDO Info 未实现 / 对象不存在 / 邮箱超时）先取干净，
    // 免得污染随后真正读写操作的错误信息。失败结果不缓存，下次仍可重试。
    const std::string detail = takeSoemErrors("SDO Info");
    if (error) {
      *error = "从站未提供该对象的条目描述（SDO Info 可能未实现）" +
               (detail.empty() ? std::string() : "：\n" + detail);
    }
    return false;
  }

  info->valid = true;
  info->data_type = oe_list->DataType[subindex];
  info->obj_access = oe_list->ObjAccess[subindex];
  info->bit_length = static_cast<int>(oe_list->BitLength[subindex]);
  info->byte_length = (info->bit_length + 7) / 8;
  if (info->byte_length < 1) {
    info->byte_length = 1;
  }
  info->name = std::string(oe_list->Name[subindex]);
  info->al_state = static_cast<uint16>(context_.slavelist[slave].state & 0x0F);

  sdo_info_cache_[key] = *info;
  return true;
}

bool EthercatMaster::querySdoObjectInfo(int slave, uint16_t index,
                                        uint8_t subindex, SdoObjectInfo* info,
                                        std::string* error) {
  if (!info) {
    return false;
  }
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  std::string err;
  if (!checkSdoTargetLocked(slave, &err)) {
    *info = SdoObjectInfo{};
    if (error) {
      *error = err;
    }
    return false;
  }
  return querySdoObjectInfoLocked(slave, index, subindex, info, error);
}

// ---------------------------------------------------------------------------
// SDO 读写（按对象真实长度）
// ---------------------------------------------------------------------------

bool EthercatMaster::sdoReadBytes(int slave, uint16_t index, uint8_t subindex,
                                  std::vector<uint8_t>* data,
                                  std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    setLastError(msg);
    return false;
  };

  if (!data) {
    return fail("输出缓冲区为空");
  }
  data->clear();

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  clearLastError();

  std::string err;
  if (!checkSdoTargetLocked(slave, &err)) {
    return fail(err);
  }

  // 1) 先向 SDO Info 要真实长度；拿不到就退回 64 字节试探缓冲
  //    （≤64 字节的对象一次就能读全，不必知道确切长度）。
  SdoObjectInfo info;
  std::string info_err;
  const bool have_info =
      querySdoObjectInfoLocked(slave, index, subindex, &info, &info_err);
  int want = kSdoProbeBytes;
  if (have_info) {
    if (info.byte_length > kSdoMaxBytes) {
      return fail("读取失败 " + sdoName(index, subindex) + "：对象长度 " +
                  std::to_string(info.byte_length) + " 字节，超出上限 " +
                  std::to_string(kSdoMaxBytes) + " 字节");
    }
    want = std::clamp(info.byte_length, 1, kSdoMaxBytes);
  }

  std::vector<uint8_t> buffer(static_cast<size_t>(want), 0);
  int size = want;
  const int wkc = ecx_SDOread(&context_, slave, index, subindex, false, &size,
                              buffer.data(), EC_TIMEOUTRXM * 3);

  if (wkc <= 0 || size <= 0 || size > want) {
    // SOEM 会把从站返回的 SDO abort 码经 ecx_SDOerror 推进错误列表，
    // 这里取出来给上层显示（对象不存在 / 只写对象 / 长度不匹配...）。
    const std::string detail = takeSoemErrors("SDO读");
    std::string hint;
    if (!have_info) {
      hint = "；从站未提供 SDO Info，已按 " + std::to_string(kSdoProbeBytes) +
             " 字节试探，对象可能更长";
    }
    return fail("读取失败 " + sdoName(index, subindex) +
                (detail.empty() ? "：从站无响应（邮箱超时 / WKC=0）"
                                : "：\n" + detail) +
                hint);
  }

  data->assign(buffer.begin(), buffer.begin() + size);
  return true;
}

bool EthercatMaster::sdoWriteBytes(int slave, uint16_t index, uint8_t subindex,
                                   const std::vector<uint8_t>& data,
                                   std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    setLastError(msg);
    return false;
  };

  if (data.empty()) {
    return fail("待写入数据为空");
  }
  if (data.size() > static_cast<size_t>(kSdoMaxBytes)) {
    return fail("待写入数据 " + std::to_string(data.size()) + " 字节，超出上限 " +
                std::to_string(kSdoMaxBytes) + " 字节");
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  clearLastError();

  std::string err;
  if (!checkSdoTargetLocked(slave, &err)) {
    return fail(err);
  }

  const int wkc =
      ecx_SDOwrite(&context_, slave, index, subindex, FALSE,
                   static_cast<int>(data.size()), data.data(), EC_TIMEOUTRXM * 3);

  // ecx_SDOwrite 返回 1 表示从站确认了这次下载
  if (wkc != 1) {
    const std::string detail = takeSoemErrors("SDO写");
    return fail("写入失败 " + sdoName(index, subindex) + "（本次下发 " +
                std::to_string(data.size()) + " 字节）" +
                (detail.empty() ? "：从站无响应（邮箱超时 / WKC=0）"
                                : "：\n" + detail));
  }

  reportStatus("[SDO] 写入 " + sdoName(index, subindex) + " " +
               std::to_string(data.size()) + " 字节成功");
  return true;
}

// ---- 数值 <-> 小端字节序（读、写共用，保证两边大小端逻辑完全一致）----

std::vector<uint8_t> EthercatMaster::valueToLittleEndian(uint64_t value,
                                                         int byte_count) {
  std::vector<uint8_t> out;
  if (byte_count <= 0) {
    return out;
  }
  out.resize(static_cast<size_t>(byte_count), 0);
  for (int i = 0; i < byte_count && i < 8; ++i) {
    out[static_cast<size_t>(i)] =
        static_cast<uint8_t>((value >> (8 * i)) & 0xFF);  // byte[0] = 最低字节
  }
  return out;
}

uint64_t EthercatMaster::littleEndianToValue(const uint8_t* data,
                                             size_t byte_count) {
  uint64_t value = 0;
  if (!data) {
    return 0;
  }
  for (size_t i = 0; i < byte_count && i < sizeof(uint64_t); ++i) {
    value |= static_cast<uint64_t>(data[i]) << (8 * i);  // byte[0] = 最低字节
  }
  return value;
}

// ---- 兼容封装：定长 uint32 版本，内部转发到按真实长度的实现 ----

bool EthercatMaster::sdoRead(uint16_t index, uint8_t subindex, uint32_t* value,
                             int slave_id, int* out_size) {
  if (out_size) {
    *out_size = 0;
  }
  if (!value) {
    return false;
  }

  std::vector<uint8_t> data;
  if (!sdoReadBytes(slave_id, index, subindex, &data, nullptr)) {
    return false;  // last_error 已由 sdoReadBytes 设置
  }
  if (data.size() > sizeof(uint32_t)) {
    setLastError("读取成功但对象长度 " + std::to_string(data.size()) +
                 " 字节，超过 4 字节；请改用按字节读取接口");
    return false;
  }

  *value = static_cast<uint32_t>(littleEndianToValue(data.data(), data.size()));
  if (out_size) {
    *out_size = static_cast<int>(data.size());
  }
  return true;
}

bool EthercatMaster::sdoWrite(uint16_t index, uint8_t subindex, uint32_t value,
                              int slave_id) {
  // 长度优先取对象真实长度（SDO Info），拿不到就按 4 字节
  int byte_length = static_cast<int>(sizeof(uint32_t));
  {
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);
    SdoObjectInfo info;
    std::string info_err;
    if (querySdoObjectInfoLocked(slave_id, index, subindex, &info, &info_err) &&
        info.byte_length > 0) {
      if (info.byte_length > static_cast<int>(sizeof(uint32_t))) {
        setLastError("对象长度 " + std::to_string(info.byte_length) +
                     " 字节，超过 4 字节；请改用按字节写入接口");
        return false;
      }
      byte_length = info.byte_length;
    }
  }

  return sdoWriteBytes(slave_id, index, subindex,
                       valueToLittleEndian(value, byte_length), nullptr);
}

// ========================================================================
//                              自动重连
// ========================================================================

bool EthercatMaster::isReconnecting() const { return is_reconnecting_.load(); }

uint32_t EthercatMaster::getReconnectAttempts() const {
  return reconnect_attempts_.load();
}

bool EthercatMaster::attemptReconnect() {
  if (stopping_.load()) {
    return false;
  }

  if (!is_reconnecting_.exchange(true)) {
    reportStatus("重连: 开始执行重连流程");
  }
  reconnect_attempts_.fetch_add(1);
  ECOUT << "Starting reconnection process\n";

  // ---- 1. 关闭旧 socket（本函数由通信线程自身调用，线程保持存活） ----
  {
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);

    if (started_.load() && context_.slavecount > 0) {
      ECOUT << "Requesting init state on all slaves... " << std::flush;
      context_.slavelist[0].state = EC_STATE_INIT;
      ecx_writestate(&context_, 0);
      ECOUT << "done\n";
      started_.store(false);
    }

    if (initialized_.load()) {
      ECOUT << "Closing socket... " << std::flush;
      ecx_close(&context_);
      ECOUT << "done\n";
      initialized_.store(false);
    }

    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      outputs_ = nullptr;
      inputs_ = nullptr;
      output_bytes_ = 0;
      input_bytes_ = 0;
    }
    output_buffer_.resize(0);
    slave_io_info_.clear();
  }

  osal_usleep(500000);  // 500ms，等从站完成 INIT 复位

  // stop() 可能在这段时间里被调用：此时绝不能重新打开 socket
  if (stopping_.load() || !running_.load()) {
    is_reconnecting_.store(false);
    return false;
  }

  // ---- 2. 重建 context 并重新走到 OP ----
  if (!initInternal(connected_index_, true)) {
    reportStatus("重连失败: init失败 | " + getLastError());
    is_reconnecting_.store(false);
    return false;
  }

  if (!start()) {
    reportStatus("重连失败: start失败 | " + getLastError());
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);
    if (initialized_.load()) {
      ecx_close(&context_);
      initialized_.store(false);
    }
    is_reconnecting_.store(false);
    return false;
  }

  resetLostFrames();
  clearLastError();
  is_reconnecting_.store(false);
  return true;
}

// ========================================================================
//              过程数据暂停 / 恢复（FoE、AL 状态切换专用）
// ========================================================================

void EthercatMaster::pauseProcessData() {
  io_pause_requested_.store(true);
  if (!running_.load()) {
    return;
  }

  // 等待工作线程确认已停止过程数据（最多 500ms）
  for (int i = 0; i < 500; ++i) {
    if (io_pause_ack_.load()) {
      break;
    }
    osal_usleep(1000);
  }
  if (!io_pause_ack_.load()) {
    reportStatus("警告: 过程数据暂停确认超时，仍将尝试独占 socket");
  }
}

void EthercatMaster::resumeProcessData() { io_pause_requested_.store(false); }

bool EthercatMaster::isProcessDataPaused() const {
  return io_pause_requested_.load() && io_pause_ack_.load();
}

// ========================================================================
//            从站 AL 状态控制（INIT / PRE_OP / SAFE_OP / OP / BOOT）
// ========================================================================

uint16_t EthercatMaster::getSlaveState(int slave) {
  if (!initialized_.load() || is_reconnecting_.load()) {
    return 0;
  }
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (slave < 1 || slave > context_.slavecount) {
    return 0;
  }
  ecx_readstate(&context_);
  return context_.slavelist[slave].state;
}

bool EthercatMaster::readSlaveAlState(int slave, uint16_t* state,
                                      uint16_t* al_status) {
  if (!initialized_.load() || is_reconnecting_.load()) {
    return false;
  }
  if (slave < 1 || slave > context_.slavecount) {
    return false;
  }
  // 与重连的 context 重建互斥。同样先重试 ~10ms：拿不到锁只说明工作线程正在
  // 收发过程数据，不能因此让界面把从站状态显示成 "--" 而闪烁。
  std::unique_lock<std::mutex> soem_lock(soem_mutex_, std::try_to_lock);
  if (!soem_lock.owns_lock() && !lockSoemBriefly(soem_lock)) {
    std::lock_guard<std::mutex> cache_lock(al_cache_mutex_);
    if (cached_al_slave_ != slave) {
      return false;
    }
    if (state) {
      *state = cached_al_state_;
    }
    if (al_status) {
      *al_status = cached_al_status_;
    }
    return true;
  }
  ecx_readstate(&context_);
  const ec_slavet* s = context_.slavelist + slave;
  if (state) {
    *state = s->state;
  }
  if (al_status) {
    *al_status = s->ALstatuscode;
  }
  {
    std::lock_guard<std::mutex> cache_lock(al_cache_mutex_);
    cached_al_slave_ = slave;
    cached_al_state_ = s->state;
    cached_al_status_ = s->ALstatuscode;
  }
  return true;
}

bool EthercatMaster::setSlaveState(int slave, uint16_t state, int timeout_ms,
                                   std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    ECOUT << "[State] setSlaveState failed: " << msg << "\n";
    return false;
  };

  if (!initialized_.load()) {
    return fail("SOEM 未初始化");
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    return fail("SOEM 未初始化");
  }
  if (slave < 1 || slave > context_.slavecount) {
    return fail("从站号无效: " + std::to_string(slave));
  }

  const uint16_t req = static_cast<uint16_t>(state & 0x0F);
  ec_slavet* s = context_.slavelist + slave;

  // 清掉上一次的 AL 状态错误（写 ACK 位），否则状态切换可能被拒
  ecx_readstate(&context_);
  if (s->state & EC_STATE_ERROR) {
    s->state = EC_STATE_ACK;
    ecx_writestate(&context_, static_cast<uint16>(slave));
    ECOUT << "[State] cleared AL error bit on slave " << slave << "\n";
  }

  s->state = req;
  ecx_writestate(&context_, static_cast<uint16>(slave));

  const int timeout_us = (timeout_ms > 0 ? timeout_ms : 5000) * 1000;
  const uint16_t got =
      ecx_statecheck(&context_, static_cast<uint16>(slave), req, timeout_us);

  if (got != req) {
    ecx_readstate(&context_);
    std::ostringstream oss;
    oss << "从站 " << slave << " 未能进入状态 0x" << std::hex << std::uppercase
        << req << "（当前 0x" << s->state << ", AL=0x" << s->ALstatuscode
        << "）";
    return fail(oss.str());
  }

  reportStatus("[State] 从站 " + std::to_string(slave) + " 已进入状态 " +
               hexValue(req, 4));
  return true;
}

bool EthercatMaster::enterBootState(int slave, int timeout_ms,
                                    std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    ECOUT << "[State] enterBootState failed: " << msg << "\n";
    return false;
  };

  if (!initialized_.load()) {
    return fail("SOEM 未初始化");
  }
  if (slave < 1 || slave > context_.slavecount) {
    return fail("从站号无效: " + std::to_string(slave));
  }

  // 1) BOOT 只能从 INIT 进入
  if (!setSlaveState(slave, EC_STATE_INIT, timeout_ms, error)) {
    return false;
  }

  // 2) 按 SII 的 Bootstrap 邮箱重写 SM0/SM1（SOEM firm_update 示例做法）。
  //    BOOT 下邮箱地址/长度可能与 PRE_OP/OP 的应用邮箱不同，不重配会导致
  //    FoE 请求进错邮箱，从站返回 FoE error(0x8000) 或直接超时。
  {
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);
    ec_slavet* s = context_.slavelist + slave;
    const uint32 boot_rx =
        ecx_readeeprom(&context_, static_cast<uint16>(slave),
                       ECT_SII_BOOTRXMBX, EC_TIMEOUTEEP);
    const uint32 boot_tx =
        ecx_readeeprom(&context_, static_cast<uint16>(slave),
                       ECT_SII_BOOTTXMBX, EC_TIMEOUTEEP);
    if (LO_WORD(boot_rx) == 0 || HI_WORD(boot_rx) == 0 ||
        LO_WORD(boot_tx) == 0 || HI_WORD(boot_tx) == 0) {
      return fail("读取 Bootstrap 邮箱配置失败（SII 0x0014/0x0016 为空）");
    }

    // 只在第一次进 BOOT 时保存应用邮箱；重复进 BOOT 时当前 SM 已经是
    // Bootstrap 配置，不能再覆盖保存值。
    if (saved_mailbox_slave_ != slave) {
      saved_sm0_ = s->SM[0];
      saved_sm1_ = s->SM[1];
      saved_mbx_wo_ = s->mbx_wo;
      saved_mbx_l_ = s->mbx_l;
      saved_mbx_ro_ = s->mbx_ro;
      saved_mbx_rl_ = s->mbx_rl;
      saved_mailbox_slave_ = slave;
    }

    s->SM[0].StartAddr = LO_WORD(boot_rx);
    s->SM[0].SMlength = HI_WORD(boot_rx);
    s->mbx_wo = LO_WORD(boot_rx);
    s->mbx_l = HI_WORD(boot_rx);
    s->SM[1].StartAddr = LO_WORD(boot_tx);
    s->SM[1].SMlength = HI_WORD(boot_tx);
    s->mbx_ro = LO_WORD(boot_tx);
    s->mbx_rl = HI_WORD(boot_tx);

    ecx_FPWR(&context_.port, s->configadr, ECT_REG_SM0, sizeof(ec_smt),
             &s->SM[0], EC_TIMEOUTRET);
    ecx_FPWR(&context_.port, s->configadr, ECT_REG_SM1, sizeof(ec_smt),
             &s->SM[1], EC_TIMEOUTRET);

    ECOUT << "[State] bootstrap mailbox slave " << slave << " SM0 A:0x"
          << std::hex << s->SM[0].StartAddr << " L:" << std::dec
          << s->SM[0].SMlength << " SM1 A:0x" << std::hex
          << s->SM[1].StartAddr << " L:" << std::dec << s->SM[1].SMlength
          << "\n";
  }

  // 3) 进入 BOOT
  return setSlaveState(slave, EC_STATE_BOOT, timeout_ms, error);
}

bool EthercatMaster::restoreMailboxConfig(int slave, std::string* error) {
  if (!initialized_.load()) {
    if (error) {
      *error = "SOEM 未初始化";
    }
    return false;
  }
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (saved_mailbox_slave_ != slave) {
    return true;  // 没有保存过（没进过 BOOT），无需恢复
  }
  ec_slavet* s = context_.slavelist + slave;
  s->SM[0] = saved_sm0_;
  s->SM[1] = saved_sm1_;
  s->mbx_wo = saved_mbx_wo_;
  s->mbx_l = saved_mbx_l_;
  s->mbx_ro = saved_mbx_ro_;
  s->mbx_rl = saved_mbx_rl_;
  ecx_FPWR(&context_.port, s->configadr, ECT_REG_SM0, sizeof(ec_smt),
           &s->SM[0], EC_TIMEOUTRET);
  ecx_FPWR(&context_.port, s->configadr, ECT_REG_SM1, sizeof(ec_smt),
           &s->SM[1], EC_TIMEOUTRET);
  saved_mailbox_slave_ = -1;
  return true;
}

bool EthercatMaster::requestSlaveState(int slave, uint16_t state,
                                       std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!initialized_.load()) {
    return fail("SOEM 未初始化");
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    return fail("SOEM 未初始化");
  }
  if (slave < 1 || slave > context_.slavecount) {
    return fail("从站号无效: " + std::to_string(slave));
  }

  ec_slavet* s = context_.slavelist + slave;
  s->state = static_cast<uint16>(state & 0x0F);
  const int wkc = ecx_writestate(&context_, static_cast<uint16>(slave));
  if (wkc <= 0) {
    return fail("AL 控制写请求失败（wkc=" + std::to_string(wkc) + "）");
  }
  return true;
}

// ========================================================================
//                        FoE (File over EtherCAT)
// ========================================================================

EthercatMaster* EthercatMaster::foe_owner_ = nullptr;

void EthercatMaster::setFoeProgressCallback(FoeProgressCallback callback) {
  std::lock_guard<std::mutex> lock(foe_cb_mutex_);
  foe_progress_callback_ = std::move(callback);
}

void EthercatMaster::clearFoeProgressCallback() {
  std::lock_guard<std::mutex> lock(foe_cb_mutex_);
  foe_progress_callback_ = nullptr;
}

void EthercatMaster::reportFoeProgress(uint64_t transferred, uint64_t total) {
  FoeProgressCallback cb;
  {
    std::lock_guard<std::mutex> lock(foe_cb_mutex_);
    cb = foe_progress_callback_;
  }
  if (cb) {
    cb(transferred, total);
  }
}

// SOEM 的 FoE hook：int (*)(uint16 slave, int packetnumber, int datasize)
//   - 写模式: datasize = 剩余待发送字节数
//   - 读模式: datasize = 已读取字节数
// 该 hook 的返回值会被 SOEM 忽略，因此只能用于上报进度 / 触发打断。
int EthercatMaster::foeHookTrampoline(uint16 slave, int packetnumber,
                                      int datasize) {
  EthercatMaster* self = foe_owner_;
  if (!self) {
    return 1;
  }

  const uint64_t total = self->foe_total_.load();
  uint64_t done = 0;
  if (self->foe_read_mode_.load()) {
    done = static_cast<uint64_t>(datasize);
  } else {
    done = (total > static_cast<uint64_t>(datasize))
               ? total - static_cast<uint64_t>(datasize)
               : total;
  }
  if (done > total) {
    done = total;
  }
  self->foe_done_.store(done);
  self->reportFoeProgress(done, total);

  // 软取消：向从站请求 INIT，从站会离开 BOOT/PRE_OP，FoE 会话随之失效，
  // 下一个分段等待 ACK 超时后 ecx_FOEwrite/read 即返回。
  if (self->foe_cancel_.load() && !self->foe_cancel_handled_.exchange(true)) {
    if (slave > 0 && slave <= self->context_.slavecount) {
      ec_slavet* s = self->context_.slavelist + slave;
      s->state = EC_STATE_INIT;
      ecx_writestate(&self->context_, slave);
      ECOUT << "[FoE] cancel requested, slave " << slave
            << " switched to INIT\n";
    }
  }
  return 1;
}

void EthercatMaster::requestFoeCancel() { foe_cancel_.store(true); }

std::string EthercatMaster::describeFoeResult(int wkc) {
  switch (wkc) {
    case -EC_ERR_TYPE_FOE_ERROR:
      return "FoE 错误：从站返回错误码（可能是文件格式/固件不匹配）";
    case -EC_ERR_TYPE_FOE_BUF2SMALL:
      return "FoE 错误：缓冲区太小";
    case -EC_ERR_TYPE_FOE_PACKETNUMBER:
      return "FoE 错误：分包序号不同步";
    case -EC_ERR_TYPE_FOE_FILE_NOTFOUND:
      return "FoE 错误：从站上未找到该文件名";
    case -EC_ERR_TYPE_PACKET_ERROR:
      return "FoE 错误：收到非预期的 mailbox 报文";
    default:
      break;
  }
  if (wkc <= 0) {
    return "FoE 失败：从站未响应（超时）。请确认从站状态、"
           "邮箱配置以及网线连接（wkc=" +
           std::to_string(wkc) + "）";
  }
  return "OK";
}

bool EthercatMaster::foeWrite(int slave, const std::string& filename,
                              uint32_t password,
                              const std::vector<uint8_t>& data, int timeout_ms,
                              std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    ECOUT << "[FoE] write failed: " << msg << "\n";
    return false;
  };

  if (!initialized_.load()) {
    return fail("SOEM 未初始化，请先「连接」");
  }
  // FoE 与重连/状态读取共用 pcap 句柄，必须串行化
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    return fail("SOEM 未初始化，请先「连接」");
  }
  if (slave < 1 || slave > context_.slavecount) {
    return fail("从站号无效: " + std::to_string(slave) + "（当前从站数 " +
                std::to_string(context_.slavecount) + "）");
  }
  if (filename.empty()) {
    return fail("远端文件名为空");
  }
  if (filename.size() > 255) {
    return fail("远端文件名过长（>255）");
  }
  if (data.empty()) {
    return fail("待下发文件内容为空");
  }

  // 初始化本次传输状态
  foe_owner_ = this;
  foe_total_.store(data.size());
  foe_done_.store(0);
  foe_read_mode_.store(false);
  foe_cancel_.store(false);
  foe_cancel_handled_.store(false);
  context_.FOEhook = foeHookTrampoline;

  const int timeout_us = (timeout_ms > 0 ? timeout_ms : 5000) * 1000;
  ECOUT << "[FoE] write '" << filename << "' size=" << data.size()
        << " slave=" << slave << " password=" << password << "\n";

  const int wkc = ecx_FOEwrite(
      &context_, static_cast<uint16>(slave),
      const_cast<char*>(filename.c_str()), password,
      static_cast<int>(data.size()), const_cast<uint8_t*>(data.data()),
      timeout_us);

  context_.FOEhook = nullptr;
  foe_owner_ = nullptr;

  const uint64_t done = foe_done_.load();
  foe_done_.store(data.size());
  reportFoeProgress(data.size(), data.size());

  if (foe_cancel_.load()) {
    return fail("传输已被取消（已下发 " + std::to_string(done) + "/" +
                std::to_string(data.size()) + " 字节）");
  }

  if (wkc <= 0) {
    return fail(describeFoeResult(wkc) + "（wkc=" + std::to_string(wkc) + "）");
  }

  ECOUT << "[FoE] write done, wkc=" << wkc << "\n";
  reportStatus("[FoE] 下发完成: " + filename + " (" +
               std::to_string(data.size()) + " 字节)");
  return true;
}

bool EthercatMaster::foeRead(int slave, const std::string& filename,
                             uint32_t password, std::vector<uint8_t>* data,
                             int timeout_ms, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    ECOUT << "[FoE] read failed: " << msg << "\n";
    return false;
  };

  if (!data) {
    return fail("输出缓冲区为空");
  }
  if (!initialized_.load()) {
    return fail("SOEM 未初始化，请先「连接」");
  }
  // FoE 与重连/状态读取共用 pcap 句柄，必须串行化
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    return fail("SOEM 未初始化，请先「连接」");
  }
  if (slave < 1 || slave > context_.slavecount) {
    return fail("从站号无效: " + std::to_string(slave));
  }
  if (filename.empty()) {
    return fail("远端文件名为空");
  }

  // 上传大小未知，按足够大的缓冲一次性申请（由调用方按需截断）
  const int kMaxReadSize = 16 * 1024 * 1024;  // 16 MB
  data->assign(kMaxReadSize, 0);
  int size = kMaxReadSize;

  foe_owner_ = this;
  foe_total_.store(0);  // 读模式总长未知，仅上报已读字节
  foe_done_.store(0);
  foe_read_mode_.store(true);
  foe_cancel_.store(false);
  foe_cancel_handled_.store(false);
  context_.FOEhook = foeHookTrampoline;

  const int timeout_us = (timeout_ms > 0 ? timeout_ms : 5000) * 1000;
  ECOUT << "[FoE] read '" << filename << "' slave=" << slave << "\n";

  const int wkc = ecx_FOEread(&context_, static_cast<uint16>(slave),
                              const_cast<char*>(filename.c_str()), password,
                              &size, data->data(), timeout_us);

  context_.FOEhook = nullptr;
  foe_owner_ = nullptr;

  if (foe_cancel_.load()) {
    data->clear();
    return fail("传输已被取消（已读取 " + std::to_string(size) + " 字节）");
  }

  if (wkc <= 0 || size <= 0) {
    data->clear();
    return fail(describeFoeResult(wkc) + "（wkc=" + std::to_string(wkc) + "）");
  }

  data->resize(size);
  reportFoeProgress(size, size);
  ECOUT << "[FoE] read done, " << size << " bytes\n";
  reportStatus("[FoE] 上传完成: " + filename + " (" + std::to_string(size) +
               " 字节)");
  return true;
}
