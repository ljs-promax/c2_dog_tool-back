#pragma once

#include <string>
#include <vector>
#include "Protocol.h"
#include "StatusReporter.h"
#include "ProdTestProtocol.h"
namespace board_test {

// LED 状态枚举
enum class LedFsmState {
    IDLE_WHITE,    // 空闲：白灯
    TESTING_BLINK, // 测试中：红色闪烁
    ESTOP_WAITING, // 等待急停按下：蓝色闪烁
    PASS_GREEN,    // 最终成功：绿色
    FAIL_RED       // 最终失败：红色
};

// LED 全局状态变量 (在 BoardBasicTester.cpp 中定义)
extern volatile LedFsmState g_led_state;
extern volatile bool        g_led_running;
extern WS2812*             global_strip;

// 单项测试的统一返回结构。
// name    : 当前测试项名称，例如 sn / usb / serial
// passed  : 当前测试项是否通过
// message : 适合给上层界面直接展示的简要说明
// detail  : 更详细的诊断信息，便于现场排查
struct TestResult {
    std::string name;
    bool        passed{false};
    std::string message;
    std::string detail;
};

// BoardBasicTester 是当前主板基础功能检测的核心类。
// 设计目标：
// 1. 把每一种硬件/功能检测拆成独立函数，便于单独调试。
// 2. 统一返回 TestResult，方便以后接入 UDP 服务端或界面。
// 3. 检测逻辑优先围绕“功能是否可用”来判断，而不是只看命令有没有输出。
class BoardBasicTester {
public:
    BoardBasicTester();

    // 根据外部输入的命令分发到对应测试项。
    // 例如：
    //   run("sn")
    //   run("usb")
    //   run("serial /dev/ttyS0")
    TestResult run(const std::string &command) const;

    // 顺序执行全部测试项。
    std::vector<TestResult> runAll() const;

    // 网络测试，返回详细的网卡结果（用于协议响应）
    NetworkResponse getNetworkResponse() const;

    // RTC测试，返回详细的RTC数据（用于协议响应）
    // cmd: 0=只读，1=写入并读回；systemTime 为空时写入当前系统时间
    RtcResponse getRtcResponse(int8_t cmd, const std::string &systemTime = {}) const;

    // BMS测试，返回详细的电池数据（用于协议响应）
    BmsResponse getBmsResponse() const;

    // MCU状态测试，返回详细的状态数据（用于协议响应）
    McuStatusResponse getMcuStatusResponse() const;

    // 返回当前 CLI 支持的命令列表，用于 help 展示。
    std::vector<std::string> supportedCommands() const;
    TestResult               Led_init() const;

private:
    // 统一封装 shell 命令执行结果。
    // exitCode : 命令退出码
    // output   : 标准输出/错误输出拼接后的文本
    struct CommandResult {
        int         exitCode{-1};
        std::string output;
    };

    // 各测试项的具体实现。
    TestResult testSn() const;
    TestResult testHardwareClock() const;
    TestResult testBluetooth() const;
    TestResult testNetwork() const;
    TestResult testUsb() const;
    TestResult testSerial() const;
    TestResult testImu() const;
    TestResult testImu(const std::string &ip, int port) const;
    TestResult testBms() const;
    TestResult testBms(const std::string &ip, int port) const;
    TestResult testCan() const;
    TestResult testCan(const std::string &ip, int port) const;
    TestResult testmcu() const;
    TestResult testmcu(const std::string &ip, int port) const;
    // TestResult testLed() const;
    // TestResult testLed(const std::string &port) const;
    TestResult stop_bpx_motion() const;
    // 对指定串口做一次主动收发测试。
    // 这个函数适合“已知端口路径”的场景，例如 serial /dev/ttyS0。
    TestResult testSerialLoopback(const std::string &port) const;
    TestResult testSBUS(const std::string &port) const;
    TestResult testSBUSWithUART(const std::string &uartPort, const std::string &sbusPort) const;

    // 执行一个 shell 命令并抓取输出。
    static CommandResult execCommand(const std::string &command);

    // 读取一个文本文件的全部内容。
    // 用在 /sys/class/net、/sys/bus/usb 等 sysfs 节点读取上。
    static std::string readTextFile(const std::string &path);

    // 通用字符串工具函数。
    static std::string trim(const std::string &value);
    static std::string toLower(std::string value);
    static bool        contains(const std::string &haystack, const std::string &needle);
    static bool        looksLikeSn(const std::string &value);
    static std::string firstLine(const std::string &value);

    // 预期需要关注的网口名称。
    // 后续如果主板型号变化，只需要调整这里即可。
    std::vector<std::string> expectedNetInterfaces_;

    // 常见串口设备名前缀。
    // 这里只是用于枚举候选节点，不代表节点存在就等于串口通讯正常。
    std::vector<std::string> serialDevicePrefixes_;
    const std::string        m_targetIp   = DefaultConfig::GATEWAY_IP;
    const int                m_targetPort = DefaultConfig::REMOTE_PORT;
    StatusReporter           m_reporter;
};

} // namespace board_test
