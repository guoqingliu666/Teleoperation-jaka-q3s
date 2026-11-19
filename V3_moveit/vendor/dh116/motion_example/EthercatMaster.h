// EthercatMaster.h
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstring>

extern "C" {
#include <soem/soem.h>
}

struct SlaveInfo {
  int index;
  std::string name;
  uint16_t state;
  uint16_t al_status;
  std::string al_status_str;
  bool is_lost;

  std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info);

// FIFO 输出帧队列:
// SDK 每次写入(速度/电流/位置帧)以"完整输出快照"入队,comm thread 每周期
// 取一帧发送。相比覆盖式双缓冲,SDO 持锁阻塞期间积压的帧在恢复后
// 按序发出——03/04/02 等中间帧不再被下一帧覆盖而从未上总线。
struct OutputBuffer {
  static constexpr size_t kMaxFrames = 16;  // 防积压(丢最旧)

  std::deque<std::vector<uint8_t>> frames_;  // 待发送帧(完整快照)
  std::vector<uint8_t> frame_base_;          // 累积视图:各次写入的合并结果
  mutable std::mutex mtx_;

  void resize(size_t size) {
    std::lock_guard<std::mutex> lock(mtx_);
    frame_base_.assign(size, 0);
    frames_.clear();
  }

  // 整帧写入(覆盖 frame_base_ 前 len 字节后入队完整快照)
  bool write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data || len == 0 || len > frame_base_.size()) return false;
    std::memcpy(frame_base_.data(), data, len);
    push_locked();
    return true;
  }

  // 分片写入(更新 frame_base_ [off, off+len) 后入队完整快照)
  bool write_slice(size_t off, const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data || len == 0 || off + len > frame_base_.size()) return false;
    std::memcpy(frame_base_.data() + off, data, len);
    push_locked();
    return true;
  }

  // 单字节写入
  bool write_byte(size_t idx, uint8_t val) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (idx >= frame_base_.size()) return false;
    frame_base_[idx] = val;
    push_locked();
    return true;
  }

  // 全量读最新累积视图
  bool read_all(uint8_t* out, size_t len) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out || len == 0 || len > frame_base_.size()) return false;
    std::memcpy(out, frame_base_.data(), len);
    return true;
  }

  // 分片读最新累积视图
  bool read_slice(size_t off, uint8_t* out, size_t len) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out || len == 0 || off + len > frame_base_.size()) return false;
    std::memcpy(out, frame_base_.data() + off, len);
    return true;
  }

  // comm thread 专用:取出一帧(队列空返回 false)
  bool consume(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (frames_.empty())
      return false;
    out = std::move(frames_.front());
    frames_.pop_front();
    return true;
  }

 private:
  void push_locked() {
    if (frames_.size() >= kMaxFrames)
      frames_.pop_front();  // 防积压:丢最旧
    frames_.push_back(frame_base_);
  }
};

struct SlaveIOInfo {
  int         slave_id;       // 1-based SOEM 从站编号
  std::string name;
  int         output_offset;  // 在 grp->outputs 中的字节偏移（无输出则 -1）
  int         output_bytes;   // 0 = 无输出
  int         input_offset;   // 在 grp->inputs 中的字节偏移（无输入则 -1）
  int         input_bytes;    // 0 = 无输入
};

class EthercatMaster {
 public:
  enum class EthercatState {
    Disconnected,     // 未初始化或已停止
    Reconnecting,     // 自动重连中（context 正在销毁/重建，禁止并发访问）
    Initializing,     // init 成功，但未 start
    SafeOperational,  // 进入 SAFE_OP
    Operational,      // 正常运行（OPERATIONAL）
    Error             // 有从站报错
  };

  EthercatMaster();
  ~EthercatMaster();

  std::vector<std::string> scanNetworkInterfaces();
  bool probeSlaves(int index, std::vector<SlaveInfo>& slaves);
  std::string getLastError() const;
  bool init(int index);
  bool start();
  void stop();
  void run();  // 启动子线程（非阻塞）
  bool isConnected() const;

  // 是否正在执行自动重连（重连期间会重建 SOEM context，禁止发起新的连接）
  bool isReconnecting() const;

  // 累计发起过的重连次数。用于区分"状态栏显示 Reconnecting"到底是真重连，
  // 还是只是没抢到 soem_mutex_（修复后后者不会再报 Reconnecting）。
  uint32_t getReconnectAttempts() const;

  // 手动触发一次重连（正常流程由通信线程在连续丢帧后自动调用）
  bool attemptReconnect();

  // 获取从站状态
  EthercatState getState() const;

  // 获取所有从站信息列表
  std::vector<SlaveInfo> getSlaveInfoList() const;

  // 获取丢帧统计
  uint64_t getLostFrames() const;

  // 重置丢帧统计
  void resetLostFrames();

  // 单字节操作
  void    setOutput(int slave_id, int index, uint8_t value);
  uint8_t getInput(int slave_id, int index);

  // PDO 批量操作
  int  getSlaveCount() const;
  bool verifyDh116Identity(int slave_id) const;
  int  getSlaveOutputSize(int slave_id) const;
  int  getSlaveInputSize(int slave_id) const;
  bool setSlaveOutputs(int slave_id, const uint8_t* data, unsigned int len);
  bool getSlaveInputs(int slave_id, uint8_t* buffer, unsigned int len) const;
  bool getVerifiedInputs(int slave_id, uint8_t* buffer, unsigned int len, uint64_t& sequence, double& stamp) const;
  bool getSlaveOutputs(int slave_id, uint8_t* buffer, unsigned int len) const;

  // SDO 读写: 对象字典 (index, subindex)
  // out_size 可选：回传从站实际给出的数据长度。
  // 注意：CoE 对 ≤4 字节的对象用「加速响应(expedited)」，长度可能是 1/2/3/4，
  // 不能假定为 4；>4 字节的对象按 4 字节缓冲去读会直接被从站拒绝。
  bool sdoRead(uint16_t index, uint8_t subindex, uint32_t* value,
               int slave_id, int* out_size = nullptr);
  bool sdoWrite(uint16_t index, uint8_t subindex, uint32_t value,
                int slave_id);

  // ============ 对象字典条目信息（CoE SDO Information 服务） ============
  // 该服务能给出对象的真实位长、数据类型与访问权限。有了它就不用猜
  // "这个对象到底是 1/2/4 字节"，读写都能按真实长度进行（也是避免
  // 0x06070010 长度不匹配的前提）。注意：SDO Info 是可选服务，
  // 从站不支持时 valid=false，调用方需退回按输入长度处理。
  struct SdoObjectInfo {
    bool valid{false};       // 是否成功取到条目描述
    uint16_t index{0};
    uint8_t subindex{0};
    uint16_t data_type{0};   // CoE 数据类型码（ECT_*）
    uint16_t obj_access{0};  // 访问权限位（ATYPE_*，按 AL 状态分段）
    uint16_t al_state{0};    // 查询时的从站 AL 状态
    int bit_length{0};       // 位长
    int byte_length{0};      // 字节长（位长向上取整，最小 1）
    std::string name;        // 对象名

    // 结合查询时的 AL 状态判断"现在能不能读 / 写"
    bool readableNow() const;
    bool writableNow() const;
    // 单行摘要，供界面直接显示
    std::string toString() const;
  };

  // 查询单个对象的条目描述（成功结果会缓存）。失败时 info->valid=false，
  // 并通过 error 给出原因（从站不支持 SDO Info / 对象不存在 / 邮箱超时）。
  bool querySdoObjectInfo(int slave, uint16_t index, uint8_t subindex,
                          SdoObjectInfo* info, std::string* error);

  // 按对象真实长度读取原始字节（小端）。长度取自 SDO Info；从站不支持时
  // 退回 kSdoProbeBytes 字节的试探缓冲，可覆盖绝大多数对象。
  bool sdoReadBytes(int slave, uint16_t index, uint8_t subindex,
                    std::vector<uint8_t>* data, std::string* error);

  // 按给定字节数下发（data.size() 即下发长度），避免拿 4 字节去写 1/2 字节
  // 对象导致从站返回 0x06070010。
  bool sdoWriteBytes(int slave, uint16_t index, uint8_t subindex,
                     const std::vector<uint8_t>& data, std::string* error);

  // ---- 数值 <-> 字节序：读、写共用同一对函数，保证大小端逻辑只有一份 ----
  // EtherCAT/CoE 的对象数据按小端存放：byte[0] 是最低字节。
  //   valueToLittleEndian(0x00000064, 4)      -> 64 00 00 00
  //   littleEndianToValue(64 00 00 00, 4)     -> 0x00000064
  // 读用后者、写用前者；两边对称，才不会出现"读出来是 64 00 00 00，
  // 写进去变成 0x64000000"。
  static std::vector<uint8_t> valueToLittleEndian(uint64_t value,
                                                  int byte_count);
  static uint64_t littleEndianToValue(const uint8_t* data, size_t byte_count);

  // ==================== 过程数据暂停/恢复 ====================
  // FoE 与周期过程数据共用同一个网卡 socket 和 mailbox，两者不能并发，
  // 因此 FoE / AL 状态切换前后必须暂停/恢复工作线程的周期通信。
  // pauseProcessData() 会等待工作线程确认已停止发送过程数据后返回。
  void pauseProcessData();
  void resumeProcessData();
  bool isProcessDataPaused() const;

  // 将指定从站切换到目标 AL 状态（EC_STATE_INIT / PRE_OP / BOOT / ...）
  bool setSlaveState(int slave, uint16_t state, int timeout_ms,
                     std::string* error);

  // 只下发一次 AL 控制请求，不等待状态到位。
  // 用于「恢复 OP」：进入 OP 需要周期过程数据，必须在恢复周期通信之前
  // 把请求写下去，否则会与工作线程争用同一个网卡 socket。
  bool requestSlaveState(int slave, uint16_t state, std::string* error);

  // 进入 BOOT：先切 INIT，再按 SII 里的 Bootstrap 邮箱配置重写 SM0/SM1，
  // 最后切 BOOT（与 SOEM firm_update 示例一致）。调用前请先 pauseProcessData()。
  // 返回失败原因写入 error。
  bool enterBootState(int slave, int timeout_ms, std::string* error);

  // 恢复进入 BOOT 之前保存的 SM0/SM1 与邮箱参数（回到应用邮箱配置）。
  bool restoreMailboxConfig(int slave, std::string* error);

  // 读取从站当前 AL 状态（0 表示读取失败）
  uint16_t getSlaveState(int slave);

  // 一次性读取从站 AL 状态字与 AL Status Code。
  // 未初始化或从站号无效时返回 false，成功时写入 state / al_status。
  bool readSlaveAlState(int slave, uint16_t* state, uint16_t* al_status);

  // ==================== FoE (File over EtherCAT) ====================

  // FoE 进度回调: (已传输字节, 总字节)
  using FoeProgressCallback =
      std::function<void(uint64_t transferred, uint64_t total)>;

  void setFoeProgressCallback(FoeProgressCallback callback);
  void clearFoeProgressCallback();

  // 请求取消当前 FoE 传输（软取消）。
  // SOEM 的 ecx_FOEread/FoEwrite 是阻塞调用且不支持中断，此处通过向从站请求
  // INIT 状态来打断 FoE 会话，使传输在下一个分段超时后退出。
  void requestFoeCancel();

  // FoE 写文件（下发）。阻塞调用，必须在独立线程中执行。
  // data 为待下发的完整文件内容，由 SOEM 内部负责分包。
  bool foeWrite(int slave, const std::string& filename, uint32_t password,
                const std::vector<uint8_t>& data, int timeout_ms,
                std::string* error);

  // FoE 读文件（上传）。阻塞调用，必须在独立线程中执行。
  bool foeRead(int slave, const std::string& filename, uint32_t password,
               std::vector<uint8_t>* data, int timeout_ms,
               std::string* error);

  // 把 FoE 返回码翻译成可读文本
  static std::string describeFoeResult(int wkc);

  // 状态回调:主动上报运行状态(启动/异常 WKC/连接丢失/停止等),
  // 由上层注册;线程安全,回调在产生线程(worker/调用方)同步执行
  using StatusUpdateCallback = std::function<void(const std::string&)>;
  void setStatusCallback(StatusUpdateCallback callback);
  void clearStatusCallback();
  std::string getLastStatus() const;
  void reportStatus(const std::string& message);

 private:
  ecx_contextt context_;                 // SOEM context
  std::vector<std::string> interfaces_;  // 扫描到的网卡名
  std::string iface_;                    // 网卡名
  uint8 group_;                          // 组号
  int roundtrip_time_;                   // 往返时间
  uint64_t verified_sequence_{0};
  double verified_stamp_{0};
  std::vector<uint8_t> verified_inputs_;
  std::vector<uint8_t> io_map_;          // I/O 映射缓冲区
  std::atomic<bool> initialized_;
  std::atomic<bool> started_;
  int connected_index_;

  // 多线程控制
  std::atomic<bool> running_;
  std::thread worker_thread_;
  mutable std::mutex soem_mutex_;  // 保护 SOEM context 访问
  mutable std::mutex io_mutex_;  // 保护 outputs/inputs 访问

  // 缓存指针
  OutputBuffer output_buffer_;  // 双缓冲实现写入
  uint8* outputs_;
  uint8* inputs_;
  int output_bytes_;
  int input_bytes_;

  std::atomic<uint64_t> lost_frames_{0};  // 丢帧计数器
  std::atomic<bool> connection_lost_{false};
  std::atomic<bool> stopping_{false};         // 正在 stop()，重连不得再拉起通信
  std::atomic<bool> is_reconnecting_{false};  // 自动重连中
  std::atomic<uint32_t> reconnect_attempts_{0};  // 累计重连发起次数

  // getState() / readSlaveAlState() 抢不到 soem_mutex_ 时的回退缓存。
  // 通信线程每个 1ms 周期都会短暂持锁收发过程数据，UI 每 100ms 轮询一次，
  // 若把"没抢到锁"当成重连上报，状态栏就会在 OP / Reconnecting 之间闪烁。
  mutable std::atomic<int> cached_state_{
      static_cast<int>(EthercatState::Initializing)};
  mutable std::mutex al_cache_mutex_;
  // 进入 BOOT 前保存的应用邮箱配置（SM0/SM1 + mbx），用于恢复
  ec_smt saved_sm0_{};
  ec_smt saved_sm1_{};
  uint16_t saved_mbx_wo_ = 0;
  uint16_t saved_mbx_l_ = 0;
  uint16_t saved_mbx_ro_ = 0;
  uint16_t saved_mbx_rl_ = 0;
  int saved_mailbox_slave_ = -1;
  mutable int cached_al_slave_{-1};
  mutable uint16_t cached_al_state_{0};
  mutable uint16_t cached_al_status_{0};

  // 对象条目描述缓存，键 = slave<<24 | index<<8 | subindex。只在持
  // soem_mutex_ 时访问（init/stop 里清空）。
  std::map<uint32_t, SdoObjectInfo> sdo_info_cache_;

  // ---- 过程数据暂停（FoE / AL 状态切换期间独占 socket 与 mailbox） ----
  std::atomic<bool> io_pause_requested_{false};  // 请求暂停周期通信
  std::atomic<bool> io_pause_ack_{false};        // 工作线程已确认暂停

  // ---- FoE 传输状态（原子，避免 hook 与调用线程间加锁） ----
  std::atomic<uint64_t> foe_total_{0};
  std::atomic<uint64_t> foe_done_{0};
  std::atomic<bool> foe_read_mode_{false};
  std::atomic<bool> foe_cancel_{false};
  std::atomic<bool> foe_cancel_handled_{false};
  std::mutex foe_cb_mutex_;                    // 保护 foe_progress_callback_
  FoeProgressCallback foe_progress_callback_;  // 进度回调
  mutable int none_state_count_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;

  // 状态回调
  mutable std::mutex status_mutex_;
  StatusUpdateCallback status_callback_;
  std::string last_status_;

  // 每从站 PDO 布局信息（init() 后只读，无需锁）
  std::vector<SlaveIOInfo> slave_io_info_;
  const SlaveIOInfo* findSlaveInfo(int slave_id) const;

  void clearLastError();
  void setLastError(const std::string& message);

  // 取出（并清空）SOEM 错误列表中的内容。SDO 失败时里面含 abort code 与说明，
  // 用于让上层显示"为什么读写失败"。
  std::string takeSoemErrors(const std::string& phase);
  // SDO abort code → 中文提示（仅列常见标准码）
  static std::string describeSdoAbort(int32_t code);

  // for_reconnect = true 时允许在通信线程存活期间重建 context（重连路径专用）
  bool initInternal(int index, bool for_reconnect);

  // 抢 soem_mutex_：先 try_lock，失败后最多重试约 10ms（通信线程每周期只
  // 短暂持锁，正常一次就成功；长期抢不到说明 FoE / 状态切换在独占）。
  bool lockSoemBriefly(std::unique_lock<std::mutex>& lock) const;

  // 以下 *Locked 版本要求调用方已持有 soem_mutex_
  bool checkSdoTargetLocked(int slave, std::string* error) const;
  bool querySdoObjectInfoLocked(int slave, uint16_t index, uint8_t subindex,
                                SdoObjectInfo* info, std::string* error);
  int getExpectedWkc() const;
  std::string describeSlaveState(int slave_index) const;
  std::string describeAllSlaveStates() const;
  void dumpSoemErrors(const std::string& phase);
  void dumpSlaveDiagnostics(const std::string& phase);
  void dumpMappingDiagnostics() const;
  void applyLslqDh116EsiDefaults();
  void dumpSiiPdoDiagnostics();
  bool readSdoValue(uint16_t slave, uint16_t index, uint8_t subindex,
                    int max_size, uint32_t* value);
  void verifySdoValue(const std::string& name, uint16_t index,
                      uint8_t subindex, uint32_t expected, int max_size,
                      int* mismatch_count);
  void verifyPdoMapping(const std::string& name, uint16_t pdo_index,
                        uint16_t object_index, uint8_t expected_entries,
                        int* mismatch_count);
  void verifyEsiConfiguration();
  void verifyMappedEsiConfiguration();

  // SOEM 的 FoE hook 是裸函数指针（无 userdata），用静态指针把回调转发到实例
  static EthercatMaster* foe_owner_;
  static int foeHookTrampoline(uint16 slave, int packetnumber, int datasize);
  void reportFoeProgress(uint64_t transferred, uint64_t total);
};
