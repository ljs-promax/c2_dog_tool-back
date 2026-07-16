#include "BoardBasicTester.h"
#include "TestConfig.h"
#include "McuUpdater.h"
#include "JointUpdater.h"
#include "JointControl.h"
#include "device_creator.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <sstream>
#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <iostream>
#include "ws2812_control.hpp"
#include <string>
#include <cstdint>
#include <unistd.h>
#include <sys/ioctl.h>

// LED 状态机枚举
enum class LedFsmState {
    IDLE_WHITE,    // 空闲：白灯 
    TESTING_BLINK, // 测试中：红色闪烁
    ESTOP_WAITING, // 等待急停按下：蓝色闪烁
    PASS_GREEN,    // 最终成功：绿色
    FAIL_RED       // 最终失败：红色
};
namespace board_test {

namespace {

/**
 * MCU 查询时复用的协议常量。
 *
 * 这里直接沿用 ota_tool / miog 里的协议约定：
 * 1. magic = "RD"
 * 2. version = 1
 * 3. IMU 查询指令 = 0xE1
 * 4. BMS 查询指令 = 0xE2
 *
 * broad_test 保持自包含，不直接依赖 ota_tool 的类，
 * 这样编译这个小工具时不会把 OTA 升级逻辑一并拉进来。
 */
constexpr uint16_t    kMcuMagic            = 0x5244;
constexpr uint8_t     kMcuProtocolVersion  = 1;
constexpr uint8_t     kCmdGetImu           = 0xE1;
constexpr uint8_t     kCmdGetBms           = 0xE2;
constexpr const char *kDefaultGatewayIp    = "10.21.32.121";
constexpr int         kDefaultGatewayPort  = 43893;
constexpr int         kMcuReceiveTimeoutMs = 1500;

struct McuQueryResponse {
    bool                 ok{false};
    std::string          error;
    std::vector<uint8_t> payload;
};

/**
 * 把单个字节格式化成 0x55 这种形式。
 *
 * 串口测试里最重要的就是直观看到 TX/RX 的字节值，
 * 所以这里统一用大写十六进制格式输出。
 */
std::string formatHexByte(unsigned char value) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(value) << std::dec;
    return oss.str();
}

/**
 * 把一批字节统一格式化成：
 * 0x55 0xAA 0x01
 *
 * 这样 detail 里会非常直观，方便测试人员现场确认。
 */
std::string formatHexBytes(const std::vector<unsigned char> &values) {
    if (values.empty()) {
        return "empty";
    }

    std::ostringstream oss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            oss << " ";
        }
        oss << formatHexByte(values[ i ]);
    }
    return oss.str();
}

/**
 * 判断字符串是否是一个合法端口号。
 *
 * CLI 里现在支持：
 * - imu
 * - imu 10.21.32.121
 * - imu 10.21.32.121 43893
 * - bms 10.21.32.121 43893
 *
 * 所以这里需要把第三个参数从字符串安全地转成整数。
 */
bool parsePortNumber(const std::string &text, int &outPort) {
    if (text.empty()) {
        return false;
    }

    char      *end   = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') {
        return false;
    }
    if (value <= 0 || value > 65535) {
        return false;
    }

    outPort = static_cast<int>(value);
    return true;
}

/**
 * 将协议里的定长字符数组安全转成 C++ 字符串。
 *
 * 某些 MCU 返回的 char[16]/char[32] 字段不一定主动补 '\0'，
 * 如果直接按 C 字符串去读，可能把后面的脏内容一并带出来。
 * 所以这里按固定长度扫描，到 '\0' 为止，没有 '\0' 就按满长度截断。
 */
std::string fixedFieldToString(const char *data, std::size_t len) {
    std::size_t actualLen = 0;
    while (actualLen < len && data[ actualLen ] != '\0') {
        ++actualLen;
    }
    return std::string(data, actualLen);
}

/**
 * 收集当前连续到达的一批串口 RX 字节。
 *
 * 处理思路：
 * 1. select 已经确认串口可读，说明至少已经有完整字节到达。
 * 2. 先 read 当前驱动缓冲区里的数据。
 * 3. 再给一个很短的空闲时间窗口，例如 20ms。
 * 4. 如果这 20ms 内又来了新字节，就继续读。
 * 5. 如果这 20ms 内没有新字节，就认为这一批数据已经收完。
 *
 * 这样比“只 read 一次”更稳，因为同一批连续字节不容易被拆成多次读取。
 */
bool collectSerialBytes(int fd, std::vector<unsigned char> &out, int idleGapMs) {
    out.clear();

    while (true) {
        unsigned char buffer[ 256 ]{};
        const ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            out.insert(out.end(), buffer, buffer + n);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return false;
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = idleGapMs * 1000;

        const int ready = select(fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }

        if (ready == 0) {
            break;
        }
    }

    return true;
}

} // namespace

// 全局 LED & 急停 状态 (在 board_test 命名空间内，可被 extern 访问)
WS2812              *global_strip      = nullptr;
volatile LedFsmState g_led_state       = LedFsmState::IDLE_WHITE;
volatile bool        g_led_running     = false;
volatile uint8_t     g_stop_key_status = 1;

BoardBasicTester::BoardBasicTester()
    : expectedNetInterfaces_({"enp1s0", "eth0", "wlan0", "wlan1"})


    , serialDevicePrefixes_({"/dev/ttyS", "/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyAMA"}) {
}

std::vector<std::string> BoardBasicTester::supportedCommands() const {
    return {"all", "sn",  "hwclock", "bluetooth", "net",  "usb", "serial", "imu",
            "bms", "can", "all",     "mcu",       "sbus", "led", "help"};
}
void printTestResult(int index, const std::string &name, const std::string &result,
                     const std::string &msg) {
    std::cout << "=======================<<" << index << ">>======================" << std::endl;
    std::cout << "\033[1;32m" << name << ": " << result << " | " << msg << "\033[0m" << std::endl;
    std::cout << std::endl;
}
TestResult BoardBasicTester::run(const std::string &command) const {
    /**
     * 命令分发逻辑：
     * 1. 先去掉前后空白。
     * 2. 再按“命令字 + 参数”拆分。
     * 3. 例如：
     *    - run("sn")
     *    - run("usb")
     *    - run("serial /dev/ttyS1")
     */
    const std::string        cleaned = trim(command);
    std::stringstream        ss(cleaned);
    std::string              action;
    std::vector<std::string> args;
    ss >> action;
    for (std::string token; ss >> token;) {
        args.push_back(token);
    }
    std::cout << "[RunCommand] " << command << std::endl;
    const std::string cmd = toLower(action.empty() ? cleaned : action);

    if (cmd == "sn")
        return testSn();
    if (cmd == "hwclock")
        return testHardwareClock();
    if (cmd == "bluetooth")
        return testBluetooth();
    if (cmd == "net")
        return testNetwork();
    if (cmd == "usb")
        return testUsb();
    if (cmd == "serial") {
        if (!args.empty()) {
            return testSerialLoopback(args.front());
        }
        return testSerial();
    }
    if (cmd == "sbus") {
        return testSBUS("/dev/ttyS2");
    }
    if (cmd == "uart2sbus")
        return testSBUSWithUART("/dev/ttyS0", "/dev/ttyS2");
    if (cmd == "imu") {
        if (args.empty()) {
            return testImu();
        }

        if (args.size() == 1) {
            return testImu(args[ 0 ], kDefaultGatewayPort);
        }

        int port = 0;
        if (!parsePortNumber(args[ 1 ], port)) {
            return {"imu", false, "invalid imu port", args[ 1 ]};
        }
        return testImu(args[ 0 ], port);
    }
    if (cmd == "bms") {
        if (args.empty()) {
            return testBms();
        }

        if (args.size() == 1) {
            return testBms(args[ 0 ], kDefaultGatewayPort);
        }

        int port = 0;
        if (!parsePortNumber(args[ 1 ], port)) {
            return {"bms", false, "invalid bms port", args[ 1 ]};
        }
        return testBms(args[ 0 ], port);
    }
    if (cmd == "can") {
        if (args.empty()) {
            return testCan();
        }
    }
    if (cmd == "led") {
        if (args.empty()) {
            return Led_init();
        }
        // 带颜色参数：led red/green/white/orange/blink
        std::string color = args[0];
        if (color == "white") {
            g_led_state = LedFsmState::IDLE_WHITE;
            std::cout << "Set LED to WHITE" << std::endl;
            return {"led", true, "set white", ""};
        } else if (color == "red") {
            g_led_state = LedFsmState::FAIL_RED;
            std::cout << "Set LED to RED" << std::endl;
            return {"led", true, "set red", ""};
        } else if (color == "green") {
            g_led_state = LedFsmState::PASS_GREEN;
            std::cout << "Set LED to GREEN" << std::endl;
            return {"led", true, "set green", ""};
        } else if (color == "orange") {
            g_led_state = LedFsmState::ESTOP_WAITING;
            std::cout << "Set LED to ORANGE" << std::endl;
            return {"led", true, "set orange", ""};
        } else if (color == "blink") {
            g_led_state = LedFsmState::TESTING_BLINK;
            std::cout << "Set LED to BLINK" << std::endl;
            return {"led", true, "set blink", ""};
        } else {
            return {"led", false, "unknown color: white/red/green/orange/blink", ""};
        }
    }
    if (cmd == "mcu") {
        if (args.empty()) {
            return testmcu();
        }
    }

    if (cmd == "all") {
        const auto         results = runAll();
        bool               allOk   = true;
        int                count   = 0;
        std::ostringstream detail;
        detail << "\n";
        int idx = 1;
        for (const auto &res : results) {
            std::string result = res.passed ? "PASS" : "FAIL";
            if (!res.passed) {
                allOk = false;
            }
            printTestResult(idx++, res.name, result, res.message);
        }
        detail << "------------------------------------------------------------\n\n";
        std::ostringstream final_msg;
        if (allOk) {
            // 成功：手指标志 + PASS
            final_msg << "👉 [all] PASS";
        } else {
            // 失败
            final_msg << "👉 [all] FAIL";
        }

        // 返回结果，name 字段直接显示加粗样式
        return {final_msg.str(), allOk, allOk ? "all tests passed" : "one or more tests failed",
                trim(detail.str())};
    }

    return {"unknown", false, "unsupported command", "type help to list supported commands"};
}

std::vector<TestResult> BoardBasicTester::runAll() const {
    std::vector<TestResult> results;
    // ============================
    // 步骤1：初始白灯
    // ============================
    g_led_state = LedFsmState::IDLE_WHITE;

    // ============================
    // 步骤2：先读 MCU → 检查按键必须=1
    // ============================
    auto mcu_check = testmcu();
    results.push_back(mcu_check);
    if (!mcu_check.passed || g_stop_key_status != 1) {
        g_led_state = LedFsmState::FAIL_RED;
        std::cout << "mcu: " << mcu_check.passed << " stop: " << g_stop_key_status << "\n"
                  << std::endl;
        results.emplace_back(TestResult{"急停    ", false, "请将急停按键恢复为松开状态", ""});
        return results;
    }
    g_led_state = LedFsmState::TESTING_BLINK;
    stop_bpx_motion();
    std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    // 产测序列（已优化顺序）

    results.push_back(testHardwareClock());
    results.push_back(testBluetooth());
    results.push_back(testNetwork());
    results.push_back(testUsb());
    // results.push_back(testSerial());
    results.push_back(testImu());
    results.push_back(testBms());
    results.push_back(testCan());
    // results.push_back(testSBUS("/dev/ttyS2"));

    // ============================
    // 步骤4：最后测试 MCU → 等待按下急停  (急停功能暂不测试)
    // ============================
    // g_led_state = LedFsmState::ESTOP_WAITING;
    // printf("请按下急停按键...\n");

    // bool estop_ok = false;
    // auto start    = std::chrono::system_clock::now();

    // while (true) {
    //     auto now = std::chrono::system_clock::now();
    //     auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

    //     testmcu(); // 刷新按键

    //     if (g_stop_key_status == 0) {
    //         estop_ok = true;
    //         break;
    //     }
    //     if (ms > 8000) { // 8秒超时
    //         break;
    //     }
    //     printf("请按下急停按键...\n");
    //     std::this_thread::sleep_for(std::chrono::milliseconds{100});
    // }

    // if (estop_ok) {
    //     results.emplace_back(TestResult{"急停    ", true, "急停按下，测试通过", ""});
    // } else {
    //     results.emplace_back(TestResult{"急停    ", false, "急停超时未按下", ""});
    // }
    results.push_back(testSn());
    // ============================
    // 步骤5：全部结束 → 最终灯色
    // ============================
    bool all_pass = true;
    for (auto &r : results) {
        if (!r.passed)
            all_pass = false;
    }

    if (all_pass) {
        g_led_state = LedFsmState::PASS_GREEN;
    } else {
        g_led_state = LedFsmState::FAIL_RED;
    }

    return results;
}

TestResult BoardBasicTester::stop_bpx_motion() const {
    const auto cmdResult  = execCommand("systemctl stop bpx_motion.service");
    const auto writeText  = trim(cmdResult.output);
    const auto writeLower = toLower(writeText);
    if (contains(writeLower, "not found") || contains(writeLower, "no such file") ||
        contains(writeLower, "error") || contains(writeLower, "fail")) {
        return {"stop_bpx_motion", false, "stop_bpx_motion", writeText};
    }
    printf("do systemctl stop bpx_motion.service\r\n");
    return {"stop_bpx_motion", true, "success", "success"};
}
bool check_sn(const std::string &sn) {
    // 1. 长度必须 18 位
    if (sn.size() != 18) {
        return false;
    }

    // 截取各字段（和 shell 完全对应）
    std::string hw_ver    = sn.substr(2, 2);  // 第3-4位
    std::string factory   = sn.substr(4, 2);  // 第5-6位
    std::string prod_date = sn.substr(6, 6);  // 第7-12位
    std::string serial    = sn.substr(13, 5); // 第14-18位

    // ==========================================
    // 2. 硬件版本：2位数字，01~99
    // ==========================================
    if (!isdigit(hw_ver[ 0 ]) || !isdigit(hw_ver[ 1 ])) {
        return false;
    }
    int ver = std::stoi(hw_ver);
    if (ver < 1 || ver > 99) {
        return false;
    }

    // ==========================================
    // 3. 工厂代码：2位大写字母 A-Z
    // ==========================================
    if (!isupper(factory[ 0 ]) || !isupper(factory[ 1 ])) {
        return false;
    }

    // ==========================================
    // 4. 生产日期：6位数字
    // ==========================================
    for (char c : prod_date) {
        if (!isdigit(c)) {
            return false;
        }
    }

    // ==========================================
    // 5. 流水号：5位数字，1~99999
    // ==========================================
    for (char c : serial) {
        if (!isdigit(c)) {
            return false;
        }
    }
    int seq = std::stoi(serial);
    if (seq < 1 || seq > 99999) {
        return false;
    }

    // 全部校验通过
    return true;
}
TestResult BoardBasicTester::testSn() const {
    /**
     * SN 检测逻辑：
     * 1. 先执行 sn write 12341234。
     * 2. 再执行 sn read。
     * 3. 只要回读值与写入值一致，就说明这一功能正常。
     */
    auto checkCmd = execCommand("which sn");
    if (checkCmd.exitCode != 0 || trim(checkCmd.output).empty()) {
        return {"序列号  ", false, "SKIP (sn command not found)", "SN test skipped"};
    }
    auto        readResult = execCommand("sn read");
    std::string currentSn  = trim(readResult.output);
    if (readResult.exitCode != 0) {
        return {"序列号", false, "sn read 执行失败", "读取指令返回错误"};
    }
    std::cout << "\n✅ 当前 SN: [" << currentSn << "]\n";
    std::cout << "请输入新序列号（直接回车使用当前值）: ";
    std::string userInput;
    std::getline(std::cin, userInput);
    std::string finalSn;
    if (userInput.empty()) {
        finalSn = currentSn;
    }
    // 用户输入了内容 → 必须校验格式
    else {
        finalSn = trim(userInput);
    }
    if (!check_sn(finalSn)) {
        return {"序列号", false, "SN格式不合法", "必须18位符合工厂规则"};
    }
    auto writeResult = execCommand("sn write " + finalSn);
    if (writeResult.exitCode != 0) {
        return {"序列号", false, "sn write 写入失败", finalSn};
    }
    auto        verifyResult = execCommand("sn read");
    std::string verifySn     = trim(verifyResult.output);

    if (verifyResult.exitCode != 0 || verifySn != finalSn) {
        return {"序列号", false, "写入后校验失败", "期望:" + finalSn + " 实际:" + verifySn};
    }

    // std::cout << "\n==========================================" << std::endl;
    // std::cout << "是否上传 SN 到远程服务器？" << std::endl;
    // std::cout << "SN: " << finalSn << std::endl;
    // std::cout << "请确认 [Y/回车=确认，N=取消]: ";

    // std::string confirm;
    // std::getline(std::cin, confirm);

    // // 判断：空(回车) 或 Y/y 才上传
    // bool needUpload = false;
    // if (confirm.empty() || confirm == "Y" || confirm == "y") {
    //     needUpload = true;
    // }

    // if (needUpload) {
    //     std::cout << "✅ 开始上传 SN 到服务器..." << std::endl;
    //     DeviceCreator creator;
    //     std::string   resp;
    //     bool          ok = creator.create(finalSn, resp);

    //     std::cout << "SN to server 返回结果:\n" << resp << std::endl;

    //     // ======================
    //     // 直接接收返回值
    //     // ======================
    //     bool parseOk = creator.parseResponse(resp);

    //     // 失败直接 return 给测试框架
    //     if (!parseOk) {
    //         return {"序列号", false, "服务器上传失败", finalSn};
    //     }
    // }

    return {"序列号", true, "sn write/read verification passed", "最终SN: " + finalSn};
}

// TestResult BoardBasicTester::testSn() const {
//     /**
//      * SN 检测逻辑：
//      * 1. 先执行 sn write 12341234。
//      * 2. 再执行 sn read。
//      * 3. 只要回读值与写入值一致，就说明这一功能正常。
//      */
//     static const std::string kExpectedSn = "12341234";
//     auto                     checkCmd    = execCommand("which sn");
//     if (checkCmd.exitCode != 0 || trim(checkCmd.output).empty()) {
//         return {"序列号  ", false, "SKIP (sn command not found)", "SN test skipped"};
//     }

//     const auto writeResult = execCommand("sn write " + kExpectedSn);
//     const auto writeText   = trim(writeResult.output);
//     const auto writeLower  = toLower(writeText);
//     if (contains(writeLower, "not found") || contains(writeLower, "no such file") ||
//         contains(writeLower, "error") || contains(writeLower, "fail")) {
//         return {"序列号  ", false, "sn write failed", writeText};
//     }
//     const auto readResult = execCommand("sn read");
//     const auto readText   = trim(readResult.output);
//     const auto readLower  = toLower(readText);
//     if (readText.empty()) {
//         return {"序列号  ", false, "sn read returned empty output", "write value: " +
//         kExpectedSn};
//     }
//     if (contains(readLower, "not found") || contains(readLower, "no such file") ||
//         contains(readLower, "error") || contains(readLower, "fail")) {
//         return {"序列号  ", false, "sn read failed", readText};
//     }

//     if (readText == kExpectedSn || contains(readText, kExpectedSn)) {
//         std::ostringstream detail;
//         detail << "write=" << kExpectedSn << "\nread=" << firstLine(readText);
//         if (!writeText.empty()) {
//             detail << "\nwrite_output=" << writeText;
//         }
//         return {"序列号  ", true, "sn write/read verification passed", detail.str()};
//     }

//     std::ostringstream detail;
//     detail << "expected=" << kExpectedSn << "\nactual=" << readText;
//     if (!writeText.empty()) {
//         detail << "\nwrite_output=" << writeText;
//     }
//     return {"序列号  ", false, "sn write/read mismatch", detail.str()};
// }

TestResult BoardBasicTester::testHardwareClock() const {
    /**
     * RTC 测试：
     * 1. 检测 /dev/rtc0 是否存在
     * 2. 写入时间到 RTC (hwclock -w)
     * 3. 读取时间验证 (hwclock -r)
     */
    std::ostringstream detail;

    // 1. 检测 rtc0 是否存在
    bool rtc0Exists = (access("/dev/rtc0", F_OK) == 0);
    detail << "rtc0 exists: " << (rtc0Exists ? "yes" : "no") << "\n";

    // 2. 写入时间 (使用系统时间同步到 RTC)
    auto writeResult = execCommand("hwclock -w 2>/dev/null");
    std::string writeTime;
    if (writeResult.exitCode == 0) {
        // 写入成功后立即读取
        auto readResult = execCommand("hwclock -r 2>/dev/null");
        writeTime = trim(readResult.output);
        detail << "write: OK\n";
        detail << "read: " << writeTime << "\n";

        if (!writeTime.empty()) {
            return {"实时时钟", true, "rtc test passed", trim(detail.str())};
        }
        return {"实时时钟", false, "read after write returned empty", trim(detail.str())};
    }

    // hwclock 不可用，退化到 date
    detail << "hwclock unavailable, using date\n";
    auto dateResult = execCommand("date '+%Y-%m-%d %H:%M:%S %z'");
    std::string dateTime = trim(dateResult.output);
    if (!dateTime.empty()) {
        detail << "system time: " << dateTime << "\n";
        return {"实时时钟", true, "date read OK (hwclock unavailable)", trim(detail.str())};
    }

    return {"实时时钟", false, "failed to read clock", trim(detail.str())};
}

TestResult BoardBasicTester::testBluetooth() const {
    /**
     * 蓝牙检测目标：
     * 1. 找到蓝牙控制器存在的证据。
     * 2. 尽量确认控制器处于 active / powered 状态。
     */
    const std::array<std::string, 4> candidates = {
        "hciconfig 2>/dev/null", "bluetoothctl show 2>/dev/null",
        "rfkill list bluetooth 2>/dev/null", "ls /sys/class/bluetooth 2>/dev/null"};

    std::ostringstream detail;
    bool               controllerFound  = false;
    bool               controllerActive = false;

    auto checkCmd = execCommand("which hciconfig");
    if (checkCmd.exitCode != 0 || trim(checkCmd.output).empty()) {
        return {"蓝牙    ", false, "SKIP (hciconfig command not found)", "Bluetooth test skipped"};
    }
    checkCmd = execCommand("which bluetoothctl");
    if (checkCmd.exitCode != 0 || trim(checkCmd.output).empty()) {
        return {"蓝牙    ", false, "SKIP (bluetoothctl command not found)",
                "Bluetooth test skipped"};
    }

    checkCmd = execCommand("which rfkill");
    if (checkCmd.exitCode != 0 || trim(checkCmd.output).empty()) {
        return {"蓝牙    ", false, "SKIP (rfkill command not found)", "Bluetooth test skipped"};
    }
    for (const auto &cmd : candidates) {
        const auto result = execCommand(cmd);
        if (!result.output.empty()) {
            detail << result.output;
        }

        const auto lower = toLower(result.output);
        if (contains(lower, "hci")) {
            controllerFound = true;
        }
        if (contains(lower, "up running") || contains(lower, "powered: yes") ||
            contains(lower, "powered yes")) {
            controllerActive = true;
        }
        if (contains(lower, "soft blocked: no") && contains(lower, "hard blocked: no")) {
            controllerFound = true;
        }
    }

    if (controllerFound && controllerActive) {
        return {"蓝牙    ", true, "bluetooth controller is active", trim(detail.str())};
    }

    return {"蓝牙    ", false, "bluetooth controller not active", trim(detail.str())};
}

NetworkResponse BoardBasicTester::getNetworkResponse() const {
    NetworkResponse resp{};
    resp.res = 0;  // 整体结果
    resp.wlan0_result = -2;  // -2=不存在
    resp.wlan1_result = -2;
    resp.enp1s0_result = -2;
    resp.eth0_result = -2;

    std::map<std::string, int8_t*> resultMap = {
        {"wlan0", &resp.wlan0_result},
        {"wlan1", &resp.wlan1_result},
        {"enp1s0", &resp.enp1s0_result},
        {"eth0", &resp.eth0_result},
    };

    std::map<std::string, std::string> pingTargets = {
        {"wlan0", TestConfigManager::get().networkWlan0Ip},
        {"wlan1", TestConfigManager::get().networkWlan1Ip},
        {"enp1s0", TestConfigManager::get().networkEnp1s0Ip},
        {"eth0", TestConfigManager::get().networkEth0Ip},
    };

    bool anySuccess = false;
    bool anyFailure = false;
    std::ostringstream detail;
    auto getIfconfigState = [this](const std::string &iface, std::string *summary) -> int8_t {
        const auto result = execCommand("ifconfig " + iface + " 2>/dev/null");
        const std::string text = trim(result.output);
        const std::string line = firstLine(text);
        if (summary) {
            *summary = line;
        }
        if (result.exitCode != 0 || text.empty()) {
            return -2;
        }

        std::string normalized = toLower(result.output);
        for (char &c : normalized) {
            if (!std::isalnum(static_cast<unsigned char>(c))) {
                c = ' ';
            }
        }

        return contains(" " + normalized + " ", " up ") ? 0 : -1;
    };

    for (const auto &iface : expectedNetInterfaces_) {
        auto itResult = resultMap.find(iface);
        auto itTarget = pingTargets.find(iface);
        std::string linkSummary;

        // 检查 ifconfig 是否显示 UP 标志
        const int8_t ifconfigState = getIfconfigState(iface, &linkSummary);
        if (ifconfigState != 0) {
            detail << iface << (ifconfigState == -2 ? ": not present" : ": down");
            if (!linkSummary.empty()) {
                detail << " (" << linkSummary << ")";
            }
            detail << "\n";
            if (itResult != resultMap.end()) {
                *itResult->second = -2;
            }
            continue;
        }

        // 检查是否有 ping 目标
        if (itTarget == pingTargets.end() || itTarget->second.empty()) {
            detail << iface << ": up, no ping target\n";
            if (itResult != resultMap.end()) {
                *itResult->second = -1;  // 未配置
            }
            continue;
        }

        // 执行 ping 测试
        std::string cmd = "ping -I " + iface + " " + itTarget->second + " -c 1 -W 1 2>&1";
        FILE* fp = popen(cmd.c_str(), "r");
        int ret = -1;
        char buf[256] = {0};

        if (fp) {
            while (fgets(buf, sizeof(buf), fp)) {
                detail << buf;
            }
            int rc = pclose(fp);
            if (rc != -1 && WIFEXITED(rc) && WEXITSTATUS(rc) == 0) {
                ret = 0;
            }
        }

        if (ret == 0) {
            detail << iface << ": " << itTarget->second << " reachable\n";
            if (itResult != resultMap.end()) {
                *itResult->second = 0;  // 成功
            }
            anySuccess = true;
        } else {
            detail << iface << ": " << itTarget->second << " unreachable\n";
            if (itResult != resultMap.end()) {
                *itResult->second = -1;  // 失败
            }
            anyFailure = true;
        }
    }

    // 设置整体结果
    if (anyFailure) {
        resp.res = -1;
    } else if (anySuccess) {
        resp.res = 0;
    } else {
        resp.res = -1;
    }

    return resp;
}

RtcResponse BoardBasicTester::getRtcResponse(int8_t cmd, const std::string &systemTime) const {
    RtcResponse resp{};
    resp.res = 0;
    resp.rtc0_exists = 0;
    memset(resp.write_time, 0, sizeof(resp.write_time));
    memset(resp.read_time, 0, sizeof(resp.read_time));

    auto isValidDateTime = [](const std::string &value) {
        if (value.size() != 19) {
            return false;
        }
        const int digitPositions[] = {0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18};
        for (int pos : digitPositions) {
            if (!std::isdigit(static_cast<unsigned char>(value[pos]))) {
                return false;
            }
        }
        return value[4] == '-' && value[7] == '-' && value[10] == ' ' &&
               value[13] == ':' && value[16] == ':';
    };

    // 1. 检测 rtc0 是否存在
    bool rtc0Exists = (access("/dev/rtc0", F_OK) == 0);
    resp.rtc0_exists = rtc0Exists ? 1 : 0;

    // 2. 写入时间（仅当 cmd=1 时）
    if (cmd == 1) {
        if (!systemTime.empty()) {
            if (!isValidDateTime(systemTime)) {
                resp.res = -1;
                return resp;
            }

            auto setSystemResult = execCommand("date -s '" + systemTime + "' 2>/dev/null");
            if (setSystemResult.exitCode != 0) {
                resp.res = -1;
                return resp;
            }

            auto currentSystemTime = execCommand("date '+%Y-%m-%d %H:%M:%S' 2>/dev/null");
            std::string writtenTime = trim(currentSystemTime.output);
            if (!writtenTime.empty()) {
                snprintf(resp.write_time, sizeof(resp.write_time), "%s", writtenTime.c_str());
            } else {
                snprintf(resp.write_time, sizeof(resp.write_time), "%s", systemTime.c_str());
            }
        }

        auto writeResult = execCommand("hwclock -w -f /dev/rtc0 2>/dev/null");
        if (writeResult.exitCode == 0) {
            // 写入成功后立即读取
            auto readResult = execCommand("hwclock -r -f /dev/rtc0 2>/dev/null");
            std::string readTime = trim(readResult.output);
            if (!readTime.empty()) {
                if (readTime.size() > 19) {
                    readTime = readTime.substr(0, 19);
                }
                if (systemTime.empty()) {
                    snprintf(resp.write_time, sizeof(resp.write_time), "%s", readTime.c_str());
                }
                snprintf(resp.read_time, sizeof(resp.read_time), "%s", readTime.c_str());
                if (!systemTime.empty() && readTime.rfind(systemTime, 0) != 0) {
                    resp.res = -1;
                    return resp;
                }
                resp.res = 0;
                return resp;
            }
        }
        resp.res = -1;
        return resp;
    }

    // 3. 读取时间
    auto readResult = execCommand("hwclock -r -f /dev/rtc0 2>/dev/null");
    std::string readTime = trim(readResult.output);
    if (!readTime.empty()) {
        if (readTime.size() > 19) {
            readTime = readTime.substr(0, 19);
        }
        if (cmd == 0) {
            // 只读模式，不写入
            snprintf(resp.read_time, sizeof(resp.read_time), "%s", readTime.c_str());
        }
        // cmd=1 时已在上面处理
        resp.res = 0;
        return resp;
    }

    // hwclock 不可用，退化到 date
    auto dateResult = execCommand("date '+%Y-%m-%d %H:%M:%S %z'");
    std::string dateTime = trim(dateResult.output);
    if (!dateTime.empty()) {
        snprintf(resp.read_time, sizeof(resp.read_time), "%s", dateTime.c_str());
        resp.res = 0;
        return resp;
    }

    resp.res = -1;
    return resp;
}

BmsResponse BoardBasicTester::getBmsResponse() const {
    BmsResponse resp{};
    resp.res = -1;
    resp.voltage_V = 0;
    resp.current_A = 0;
    resp.battery_level_percent = 0;
    resp.remaining_capacity_mAh = 0;
    resp.cycles = 0;
    resp.temp_fet_C = 0;
    resp.charger_in1 = 0;
    resp.charger_in2 = 0;

    McuUpdater mcu(kDefaultGatewayIp, kDefaultGatewayPort, const_cast<StatusReporter&>(m_reporter));
    if (mcu.init()) {
        BmsFeedback bat = mcu.getBattery();
        if (bat.voltage_V > 0.1f) {
            resp.res = 0;
            resp.voltage_V = bat.voltage_V;
            resp.current_A = bat.current_A;
            resp.battery_level_percent = bat.battery_level_percent;
            resp.remaining_capacity_mAh = bat.remaining_capacity_mAh;
            resp.cycles = bat.cycles;
            resp.temp_fet_C = bat.temp_fet_C;
            resp.charger_in1 = bat.charger_in1;
            resp.charger_in2 = bat.charger_in2;
            return resp;
        }
    }

    return resp;
}

McuStatusResponse BoardBasicTester::getMcuStatusResponse() const {
    McuStatusResponse resp{};
    resp.res = -1;
    resp.sensors_status = 0;
    resp.can0_status = 0;
    resp.can1_status = 0;
    resp.can2_status = 0;
    resp.can3_status = 0;

    McuUpdater mcu(kDefaultGatewayIp, kDefaultGatewayPort, const_cast<StatusReporter&>(m_reporter));
    if (mcu.init()) {
        McuStatus status = mcu.getmcuStatus();
        resp.res = 0;
        resp.sensors_status = status.sensors_status;
        resp.can0_status = status.can_status[0];
        resp.can1_status = status.can_status[1];
        resp.can2_status = status.can_status[2];
        resp.can3_status = status.can_status[3];
        return resp;
    }

    return resp;
}

TestResult BoardBasicTester::testNetwork() const {
    /**
     * 网络检测：
     * 1. 检查 ifconfig 是否显示 UP 标志
     * 2. ping 各网段的指定IP，验证通信
     */
    std::ostringstream detail;
    bool allPassed = true;
    std::map<std::string, std::string> pingTargets = {
        {"wlan0", TestConfigManager::get().networkWlan0Ip},
        {"wlan1", TestConfigManager::get().networkWlan1Ip},
        {"enp1s0", TestConfigManager::get().networkEnp1s0Ip},
        {"eth0", TestConfigManager::get().networkEth0Ip},
    };
    auto getIfconfigState = [this](const std::string &iface, std::string *summary) -> int8_t {
        const auto result = execCommand("ifconfig " + iface + " 2>/dev/null");
        const std::string text = trim(result.output);
        const std::string line = firstLine(text);
        if (summary) {
            *summary = line;
        }
        if (result.exitCode != 0 || text.empty()) {
            return -2;
        }

        std::string normalized = toLower(result.output);
        for (char &c : normalized) {
            if (!std::isalnum(static_cast<unsigned char>(c))) {
                c = ' ';
            }
        }

        return contains(" " + normalized + " ", " up ") ? 0 : -1;
    };

    for (const auto &iface : expectedNetInterfaces_) {
        std::string linkSummary;

        // 检查 ifconfig 是否显示 UP 标志
        const int8_t ifconfigState = getIfconfigState(iface, &linkSummary);
        if (ifconfigState != 0) {
            detail << iface << (ifconfigState == -2 ? ": not present" : ": down");
            if (!linkSummary.empty()) {
                detail << " (" << linkSummary << ")";
            }
            detail << "\n";
            allPassed = false;
            continue;
        }

        // 获取 ping 目标
        auto it = pingTargets.find(iface);
        if (it == pingTargets.end() || it->second.empty()) {
            detail << iface << ": up, no ping target configured\n";
            continue;
        }
        const std::string &pingIp = it->second;

        // 执行 ping 测试
        std::string cmd = "ping -I " + iface + " " + pingIp + " -c 1 -W 1 2>&1";
        FILE* fp = popen(cmd.c_str(), "r");
        int ret = -1;
        char buf[256] = {0};

        if (fp) {
            while (fgets(buf, sizeof(buf), fp)) {
                detail << buf;
            }
            int rc = pclose(fp);
            if (rc != -1 && WIFEXITED(rc) && WEXITSTATUS(rc) == 0) {
                ret = 0;
            }
        }

        if (ret == 0) {
            detail << iface << ": " << pingIp << " reachable\n";
        } else {
            detail << iface << ": " << pingIp << " unreachable\n";
            allPassed = false;
        }
    }

    if (allPassed) {
        return {"网络  ", true, "all network interfaces ping OK", trim(detail.str())};
    }
    return {"网络  ", false, "some network interfaces failed", trim(detail.str())};
}
TestResult BoardBasicTester::testUsb() const {
    /**
     * USB 检测规则：仅 /sys/bus/usb/devices/1-1 存在有效外接设备才算测试通过
     * 对应内核日志 usb 1-1
     */
    std::ostringstream detail;

    DIR *dir = opendir("/sys/bus/usb/devices");
    if (!dir) {
        const auto fallback = execCommand("dmesg | grep -i usb | tail -n 20");
        const auto text     = trim(fallback.output);
        if (!text.empty()) {
            return {"USB   ", false, "usb sysfs unavailable, only log evidence found", text};
        }
        return {"USB   ", false, "cannot open /sys/bus/usb/devices",
                "usb sysfs unavailable and dmesg fallback empty"};
    }

    bool targetPortHasDevice = false; // 仅标记usb 1-1是否有设备
    while (auto *ent = readdir(dir)) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") {
            continue;
        }

        const std::string base         = std::string("/sys/bus/usb/devices/") + name;
        const std::string idVendor     = trim(readTextFile(base + "/idVendor"));
        const std::string idProduct    = trim(readTextFile(base + "/idProduct"));
        const std::string product      = trim(readTextFile(base + "/product"));
        const std::string manufacturer = trim(readTextFile(base + "/manufacturer"));
        const std::string serial       = trim(readTextFile(base + "/serial"));

        // 无VID/PID，不是有效USB设备，跳过
        if (idVendor.empty() && idProduct.empty()) {
            continue;
        }

        // 过滤1：Linux Foundation 根Hub，全部忽略
        if (idVendor == "1d6b") {
            detail << "[PORT:usb " << name << "] ignored root hub VID=1d6b\n";
            continue;
        }

        // 过滤2：板载USB WIFI，忽略
        if (idVendor == "0bda" && idProduct == "f72b") {
            detail << "[PORT:usb " << name << "] ignored onboard wlan: " << manufacturer << " "
                   << product << " VID=" << idVendor << " PID=" << idProduct << "\n";
            continue;
        }

        // 当前是合法外接USB外设
        if (name == "1-1") {
            // 目标端口usb 1-1，标记测试通过
            targetPortHasDevice = true;
            detail << "[MATCH TARGET PORT:usb 1-1] ";
        } else {
            // 其他端口外设，仅记录日志，不判定测试通过
            detail << "[IGNORE PORT:usb " << name << "] ";
        }

        // 打印设备通用信息
        if (!manufacturer.empty())
            detail << "Mfr=" << manufacturer << " ";
        if (!product.empty())
            detail << "Prod=" << product << " ";
        if (!idVendor.empty())
            detail << "VID=" << idVendor << " ";
        if (!idProduct.empty())
            detail << "PID=" << idProduct << " ";
        if (!serial.empty())
            detail << "SN=" << serial;
        detail << "\n";
    }

    closedir(dir);

    // 只有usb 1-1存在有效外设才返回PASS
    if (targetPortHasDevice) {
        return {"USB   ", true, "target port usb 1-1 detected valid usb device",
                trim(detail.str())};
    }

    return {"USB   ", false, "no valid usb device on target port usb 1-1", trim(detail.str())};
}
#if 0
TestResult BoardBasicTester::testUsb() const {
    /**
     * USB 检测目标是找到真实 USB 设备的枚举信息，
     * 而不是只看命令有没有输出。
     *
     * 因此这里直接遍历 /sys/bus/usb/devices 读取：
     * - idVendor
     * - idProduct
     * - product
     * - manufacturer
     * - serial
     */
    std::ostringstream detail;

    DIR *dir = opendir("/sys/bus/usb/devices");
    if (!dir) {
        const auto fallback = execCommand("dmesg | grep -i usb | tail -n 20");
        const auto text     = trim(fallback.output);
        if (!text.empty()) {
            return {"USB   ", false, "usb sysfs unavailable, only log evidence found", text};
        }
        return {"USB   ", false, "cannot open /sys/bus/usb/devices",
                "usb sysfs unavailable and dmesg fallback empty"};
    }

    bool foundUsbDevice = false;
    while (auto *ent = readdir(dir)) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") {
            continue;
        }

        const std::string base         = std::string("/sys/bus/usb/devices/") + name;
        const std::string idVendor     = trim(readTextFile(base + "/idVendor"));
        const std::string idProduct    = trim(readTextFile(base + "/idProduct"));
        const std::string product      = trim(readTextFile(base + "/product"));
        const std::string manufacturer = trim(readTextFile(base + "/manufacturer"));
        const std::string serial       = trim(readTextFile(base + "/serial"));

        if (idVendor.empty() && idProduct.empty() && product.empty() && manufacturer.empty() &&
            serial.empty()) {
            continue;
        }

        /**
         * 过滤 Linux Foundation Root Hub。
         * 它是控制器自带根设备，不代表外部 USB 口插入了设备。
         */
        if (idVendor == "1d6b") {
            continue;
        }

        /**
         * 过滤板载 USB WLAN 模块。
         * 否则在没有插外部 USB 时也可能误判 PASS。
         */
        if (idVendor == "0bda" && idProduct == "f72b") {
            detail << "[" << name << "] ignored onboard usb wlan device: " << manufacturer << " "
                   << product << " VID=" << idVendor << " PID=" << idProduct << "\n";
            continue;
        }

        foundUsbDevice = true;
        detail << "[" << name << "] ";
        if (!manufacturer.empty())
            detail << "Manufacturer=" << manufacturer << " ";
        if (!product.empty())
            detail << "Product=" << product << " ";
        if (!idVendor.empty())
            detail << "VID=" << idVendor << " ";
        if (!idProduct.empty())
            detail << "PID=" << idProduct << " ";
        if (!serial.empty())
            detail << "SN=" << serial << " ";
        detail << "\n";
    }

    closedir(dir);

    if (foundUsbDevice) {
        return {"USB   ", true, "usb device information found", trim(detail.str())};
    }

    return {"USB   ", false, "no usb device evidence found",
            "no readable usb device entries under /sys/bus/usb/devices"};
}
#endif
#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif
#ifndef BOTHER
#define BOTHER 0x1000
#endif
void sbus_parse(const uint8_t *buf, uint16_t *ch) {
    ch[ 0 ]  = ((buf[ 1 ] | buf[ 2 ] << 8) & 0x07FF);
    ch[ 1 ]  = ((buf[ 2 ] >> 3 | buf[ 3 ] << 5) & 0x07FF);
    ch[ 2 ]  = ((buf[ 3 ] >> 6 | buf[ 4 ] << 2 | buf[ 5 ] << 10) & 0x07FF);
    ch[ 3 ]  = ((buf[ 5 ] >> 1 | buf[ 6 ] << 7) & 0x07FF);
    ch[ 4 ]  = ((buf[ 6 ] >> 4 | buf[ 7 ] << 4) & 0x07FF);
    ch[ 5 ]  = ((buf[ 7 ] >> 7 | buf[ 8 ] << 1 | buf[ 9 ] << 9) & 0x07FF);
    ch[ 6 ]  = ((buf[ 9 ] >> 2 | buf[ 10 ] << 6) & 0x07FF);
    ch[ 7 ]  = ((buf[ 10 ] >> 5 | buf[ 11 ] << 3) & 0x07FF);
    ch[ 8 ]  = ((buf[ 12 ] | buf[ 13 ] << 8) & 0x07FF);
    ch[ 9 ]  = ((buf[ 13 ] >> 3 | buf[ 14 ] << 5) & 0x07FF);
    ch[ 10 ] = ((buf[ 14 ] >> 6 | buf[ 15 ] << 2 | buf[ 16 ] << 10) & 0x07FF);
    ch[ 11 ] = ((buf[ 16 ] >> 1 | buf[ 17 ] << 7) & 0x07FF);
    ch[ 12 ] = ((buf[ 17 ] >> 4 | buf[ 18 ] << 4) & 0x07FF);
    ch[ 13 ] = ((buf[ 18 ] >> 7 | buf[ 19 ] << 1 | buf[ 20 ] << 9) & 0x07FF);
    ch[ 14 ] = ((buf[ 20 ] >> 2 | buf[ 21 ] << 6) & 0x07FF);
    ch[ 15 ] = ((buf[ 21 ] >> 5 | buf[ 22 ] << 3) & 0x07FF);
}
TestResult BoardBasicTester::testSBUS(const std::string &port) const {
    if (port.empty()) {
        return {"SBUS", false, "port empty", "need tty device path"};
    }

    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return {"SBUS", false, "open failed", port};
    }
    struct termios2 {
        tcflag_t c_iflag;
        tcflag_t c_oflag;
        tcflag_t c_cflag;
        tcflag_t c_lflag;
        cc_t     c_cc[ 19 ];
        speed_t  c_ispeed;
        speed_t  c_ospeed;
    };

    termios2 tty{};
    if (ioctl(fd, TCGETS2, &tty) != 0) {
        close(fd);
        return {"SBUS", false, "TCGETS2 failed", port};
    }

    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = 100000;
    tty.c_ospeed = 100000;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | PARENB | CLOCAL | CREAD;
    tty.c_cc[ VMIN ]  = 1;
    tty.c_cc[ VTIME ] = 1;

    if (ioctl(fd, TCSETS2, &tty) != 0) {
        close(fd);
        return {"SBUS", false, "TCSETS2 failed", port};
    }

    tcflush(fd, TCIOFLUSH);
    usleep(100000);

    // ==============================
    // 线程共享变量
    // ==============================
    std::atomic<bool> test_success{false};
    std::atomic<bool> stop_thread{false};
    uint16_t          channels[ 16 ] = {0};

    // ==============================
    // 接收线程
    // ==============================
    std::thread rxThread([ & ]() {
        uint8_t buf[ 25 ];
        int     idx = 0;

        while (!stop_thread) {
            uint8_t ch;
            int     n = read(fd, &ch, 1);

            if (n != 1) {
                usleep(50);
                continue;
            }
            // printf("get ch: %02X\n", ch);
            //   帧同步：必须从 0x0F 开始
            if (idx == 0 && ch != 0x0F) {
                idx = 0;
                continue;
            }
            // printf("get ch: %02X\n", ch);
            buf[ idx++ ] = ch;

            // 收满一帧 → 解析
            if (idx >= 25) {
                // printf("[RX] ====== Full SBUS frame received ======\n");
                // // 打印整帧25字节数据
                // for (int i = 0; i < 25; i++) {
                //     printf("0x%02X ", buf[ i ]);
                //     if ((i + 1) % 10 == 0)
                //         printf("\n"); // 每10个换行，方便查看
                // }
                // printf("\n==========================================\n");
                sbus_parse(buf, channels);

                // 检查通道是否合法
                test_success = true;
                // break;

                idx = 0;
            }
        }
    });

    // ==============================
    // 主线程等待 1 秒
    // ==============================
    auto start = std::chrono::steady_clock::now();
    while (true) {
        if (test_success)
            break;

        auto now = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (ms >= 1000)
            break;

        usleep(1000);
    }

    // 停止线程
    stop_thread = true;
    if (rxThread.joinable()) {
        rxThread.join();
    }

    close(fd);

    std::ostringstream detail;
    detail << "port=" << port << "\n100000 8E1\n";

    if (test_success) {
        detail << "parse OK\nchannels:";
        for (int i = 0; i < 16; i++) {
            detail << " " << channels[ i ];
        }
        return {"SBUS", true, "thread test passed", trim(detail.str())};
    }

    return {"SBUS", false, "no valid sbus frame in 1s", trim(detail.str())};
}

TestResult BoardBasicTester::testSBUSWithUART(const std::string &uartPort,
                                              const std::string &sbusPort) const {
    /**
     * 测试逻辑：
     * 1. ttyS0 不断发送数据
     * 2. ttyS2 (SBUS) 接收
     * 3. 1 秒内收到任意数据 → 成功
     * 4. 不校验数据内容，只验证 RX 通路正常
     */

    // 打开 UART0 发送
    int fd_uart = open(uartPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_uart < 0) {
        return {"SBUS通路", false, "打开发送串口失败", uartPort};
    }

    // 打开 SBUS 接收
    int fd_sbus = open(sbusPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_sbus < 0) {
        close(fd_uart);
        return {"SBUS通路", false, "打开SBUS串口失败", sbusPort};
    }

    // ======================
    // 配置串口：统一 100000 8E1（和你SBUS一致）
    // ======================
#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif
#ifndef BOTHER
#define BOTHER 0x1000
#endif

    struct termios2 {
        tcflag_t c_iflag;
        tcflag_t c_oflag;
        tcflag_t c_cflag;
        tcflag_t c_lflag;
        cc_t     c_cc[ 19 ];
        speed_t  c_ispeed;
        speed_t  c_ospeed;
    };

    auto configPort = [](int fd) -> bool {
        termios2 tty{};
        if (ioctl(fd, TCGETS2, &tty) != 0)
            return false;

        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= BOTHER;
        tty.c_ispeed = 100000;
        tty.c_ospeed = 100000;

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARODD | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | PARENB | CLOCAL | CREAD;
        tty.c_cc[ VMIN ]  = 1;
        tty.c_cc[ VTIME ] = 1;

        return ioctl(fd, TCSETS2, &tty) == 0;
    };

    if (!configPort(fd_uart) || !configPort(fd_sbus)) {
        close(fd_uart);
        close(fd_sbus);
        return {"SBUS通路", false, "串口配置失败", "100000 8E1"};
    }

    tcflush(fd_uart, TCIOFLUSH);
    tcflush(fd_sbus, TCIOFLUSH);
    const uint8_t test_bytes[ 5 ] = {0x00, 0x00, 0x55, 0x55, 0x55};
    // ======================
    // 线程共享变量
    // ======================
    std::atomic<bool> uart_loop_ok{false};
    std::atomic<bool> sbus_rx_ok{false};
    std::atomic<bool> stop{false};
    uint8_t           test_data = 0xA5;
    // ======================
    // 发送线程：ttyS0 不断发 0x55
    // ======================
    std::thread txThread([ & ]() {
        while (!stop) {
            // write(fd_uart, &test_data, 1);
            write(fd_uart, test_bytes, 5);
            usleep(5000);
        }
    });
    // 线程2：ttyS0 自发自收测试
    // std::thread uartRxThread([ & ]() {
    //     while (!stop && !uart_loop_ok) {
    //         uint8_t ch;
    //         int     n = read(fd_uart, &ch, 1);
    //         if (n == 1) {
    //             if (ch == test_data)
    //                 uart_loop_ok = true;
    //             break;
    //         }
    //         usleep(100);
    //     }
    // });
    std::thread uartRxThread([ & ]() {
        uint8_t buf[ 5 ] = {0};
        while (!stop && !uart_loop_ok) {
            int n = read(fd_uart, buf, 5);
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    if (buf[ i ] == test_bytes[ 0 ]) {
                        uart_loop_ok = true;
                        break;
                    }
                }
            }
            usleep(100);
        }
    });
    // ======================
    // 接收线程：ttyS2 只要收到就标记成功
    // ======================
    // std::thread sbusRxThread([ & ]() {
    //     while (!stop && !sbus_rx_ok) {
    //         uint8_t ch;
    //         int     n = read(fd_sbus, &ch, 1);
    //         if (n == 1) {
    //             printf("sbus rx: %02x\n", ch);
    //             sbus_rx_ok = true;
    //             break;
    //         }
    //         usleep(100);
    //     }
    // });
    uint8_t     ch;
    static int  count = 0;
    std::thread sbusRxThread([ & ]() {
        while (!stop && !sbus_rx_ok) {

            int n = read(fd_sbus, &ch, 1);
            if (n == 1) {
                count++;

                sbus_rx_ok = true;
                if (count >= 5)
                    break;
            }
            usleep(100);
        }
    });
    printf("sbus1 rx byte[%d]: 0x%02X\n", count, ch);
    // ======================
    // 等待 1 秒
    // ======================
    auto start = std::chrono::steady_clock::now();
    while (!uart_loop_ok || !sbus_rx_ok) {
        auto now = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (ms >= 1000)
            break;
        usleep(1000);
    }

    // ======================
    // 停止线程
    // ======================
    stop = true;
    if (txThread.joinable())
        txThread.join();
    if (uartRxThread.joinable())
        uartRxThread.join();
    if (sbusRxThread.joinable())
        sbusRxThread.join();

    close(fd_uart);
    close(fd_sbus);

    // 两个都成功才算通过
    bool pass = uart_loop_ok && sbus_rx_ok;

    std::ostringstream detail;
    detail << "uart_loop=" << (uart_loop_ok ? "OK" : "FAIL")
           << "  sbus_rx=" << (sbus_rx_ok ? "OK" : "FAIL");

    if (pass) {
        return {"SBUS_TEST", true, "UART自收发 & SBUS接收 全部正常", detail.str()};
    } else {
        return {"SBUS_TEST", false, "UART或SBUS通路测试失败", detail.str()};
    }
}
TestResult BoardBasicTester::testSerial() const {
    /**
     * 默认串口测试直接测 /dev/ttyS0。
     *
     * 使用前提：
     * 1. 测试人员先把目标串口 TX 和 RX 短接。
     * 2. broad_test_cli 中执行 serial。
     * 3. 程序发送固定测试字节 0x55。
     * 4. 如果 RX 回读的第一个字节也是 0x55，就判定串口基础收发正常。
     */
    return testSerialLoopback("/dev/ttyS0");
}

TestResult BoardBasicTester::testSerialLoopback(const std::string &port) const {
    /**
     * 串口自发自收检测流程：
     * 1. 打开指定串口。
     * 2. 配置 raw + 8N1 + 115200。
     * 3. 清空旧缓存。
     * 4. 发送 1 个固定字节 0x55。
     * 5. 等待发送完成。
     * 6. 等待 RX 有数据。
     * 7. 收集这一批连续到达的字节。
     * 8. 如果收到的第一个字节与发送值相同，则 PASS。
     *
     * 当前判定标准很明确：
     * - TX 能发出去
     * - RX 能收到
     * - 收到的第一个字节与 TX 相同
     * 满足这三个条件，就说明串口接口基础功能正常。
     */
    if (port.empty()) {
        return {"serial", false, "serial port is empty", "need a real tty device path"};
    }

    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return {"serial", false, "open serial port failed", port};
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return {"serial", false, "tcgetattr failed", port};
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[ VMIN ]  = 0;
    tty.c_cc[ VTIME ] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return {"serial", false, "tcsetattr failed", port};
    }

    /**
     * 清空旧缓存，避免上一次测试残留数据影响本次判断。
     */
    tcflush(fd, TCIOFLUSH);

    /**
     * 多轮检测策略：
     * 1. 不是只发 1 次，而是连续做多次尝试。
     * 2. 每一轮都发固定字节 0x55。
     * 3. 每一轮都等待 RX 回读。
     * 4. 只要其中任意一轮收到的第一个字节与 0x55 一致，就判定 PASS。
     *
     * 这样做的原因是：
     * 1. 现场串口自环测试偶尔可能因为时序或缓存原因出现单次异常。
     * 2. 用户的最终需求是“多次发送，只要有一次发送接收正确即可说明接口正常”。
     */
    const unsigned char txByte               = 0x55;
    const int           maxAttempts          = 5;
    const int           perAttemptTimeoutSec = 1;

    std::ostringstream detail;
    detail << "port=" << port << "\nbaud=115200"
           << "\ntx=" << formatHexByte(txByte) << "\nattempts=" << maxAttempts << "\n";

    bool        anyPass = false;
    std::string successSummary;

    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        tcflush(fd, TCIOFLUSH);

        const ssize_t written = write(fd, &txByte, 1);
        if (written != 1) {
            detail << "attempt_" << attempt << "=write_failed"
                   << " written=" << written << "\n";
            continue;
        }

        /**
         * 等待本轮发送真正完成，再进入接收等待。
         */
        tcdrain(fd);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        timeval tv{};
        tv.tv_sec  = perAttemptTimeoutSec;
        tv.tv_usec = 0;

        const int ready = select(fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ready <= 0) {
            detail << "attempt_" << attempt << "=rx_timeout\n";
            continue;
        }

        std::vector<unsigned char> rxBytes;
        const bool                 readOk = collectSerialBytes(fd, rxBytes, 20);
        if (!readOk) {
            detail << "attempt_" << attempt << "=rx_read_error\n";
            continue;
        }

        if (rxBytes.empty()) {
            detail << "attempt_" << attempt << "=rx_empty\n";
            continue;
        }

        detail << "attempt_" << attempt << "=rx_count:" << rxBytes.size()
               << " rx:" << formatHexBytes(rxBytes) << "\n";

        if (rxBytes.front() == txByte) {
            anyPass = true;
            std::ostringstream success;
            success << "success_attempt=" << attempt << "\nrx_count=" << rxBytes.size()
                    << "\nrx=" << formatHexBytes(rxBytes);
            successSummary = success.str();
            break;
        }
    }

    close(fd);

    if (anyPass) {
        detail << successSummary;
        return {"串口  ", true, "serial tx/rx loopback passed", trim(detail.str())};
    }

    return {"串口  ", false, "serial tx/rx loopback failed", trim(detail.str())};
}
void led_fsm_thread() {
    g_led_running = true;
    int      n    = global_strip->getNumLeds();
    LedState s{};

    while (g_led_running) {
        switch (g_led_state) {
            case LedFsmState::IDLE_WHITE:
                s.mode       = LedMode::STATIC;
                s.brightness = 0.4f;
                s.colors.assign(n, {255, 255, 255});
                global_strip->setState(s);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

            case LedFsmState::TESTING_BLINK:
                s.mode       = LedMode::BLINK;
                s.period_ms  = 500;
                s.brightness = 1.0f;
                s.colors.assign(n, {255, 0, 0});
                global_strip->setState(s);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

            case LedFsmState::ESTOP_WAITING:
                s.mode      = LedMode::BLINK;
                s.period_ms = 300;
                s.colors.assign(n, {0, 0, 255});
                global_strip->setState(s);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

            case LedFsmState::PASS_GREEN:
                s.mode = LedMode::STATIC;
                s.colors.assign(n, {0, 255, 0});
                global_strip->setState(s);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

            case LedFsmState::FAIL_RED:
                s.mode = LedMode::STATIC;
                s.colors.assign(n, {255, 0, 0});
                global_strip->setState(s);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
        }
    }
    global_strip->stop();
}
TestResult BoardBasicTester::Led_init() const {
    // 如果线程已在运行，直接返回
    if (g_led_running) {
        return {"led", true, "led already running", ""};
    }
    static WS2812 strip("/dev/spidev1.0");
    global_strip = &strip;
    int num_leds = strip.getNumLeds();
    if (!strip.start()) {
        std::cerr << "错误：无法启动 SPI 控制器，请检查权限或路径！" << std::endl;
        return {"led", false, "无法启动 SPI 控制器，请检查权限或路径", "SPI 控制器 打开异常"};
    }
    g_led_state = LedFsmState::IDLE_WHITE; // 白灯
    g_led_running = true;  // 启动线程前设置为 true，避免竞态
    std::thread(led_fsm_thread).detach();
    return {"led", true, "led init ok", ""};
}
TestResult BoardBasicTester::testImu() const {
    /**
     * 默认 MCU/IMU 查询入口。
     *
     * 为了让测试人员直接输入 imu 就能测通，
     * 这里默认走项目里已经固定使用的网关地址和端口。
     */
    return testImu(kDefaultGatewayIp, kDefaultGatewayPort);
}

TestResult BoardBasicTester::testImu(const std::string &ip, int port) const {

    // 用来存储线程执行结果
    bool               res = false;
    std::string        info;
    std::ostringstream detail;
    // 执行查询
    std::thread t([ & ]() {
        try {
            McuUpdater mcu(ip, port, const_cast<StatusReporter &>(this->m_reporter));

            if (mcu.init()) {
                ImuFeedback imu = mcu.getIMU();
                bool        is_valid =
                    !(imu.qx == 0.0f && imu.qy == 0.0f && imu.qz == 0.0f && imu.qw == 0.0f);

                if (is_valid) {
                    detail << "qx: " << imu.qx << " qy: " << imu.qy << " qz: " << imu.qz
                           << " qw: " << imu.qw << "\n";
                    detail << "wx: " << imu.wx << " wy: " << imu.wy << " wz: " << imu.wz << "\n";
                    detail << "ax: " << imu.ax << " ay: " << imu.ay << " az: " << imu.az << "\n";
                    res  = true;
                    info = "IMU 查询成功[有效数据]";
                } else {
                    printf("[IMU] 无响应或数据无效\n");

                    res  = false;
                    info = "IMU 查询超时/无效";
                }
            } else {
                res  = false;
                info = "IMU 查询失败";
            }
        } catch (...) {
            res  = false;
            info = "IMU 查询异常";
        }
    });

    // 等待线程执行完毕
    t.join();

    // 线程结束后，再返回最终结果
    return {"陀螺仪  ", res, info, trim(detail.str())};
}
TestResult BoardBasicTester::testmcu() const {
    /**
     * 默认 MCU/IMU 查询入口。
     *
     * 为了让测试人员直接输入 imu 就能测通，
     * 这里默认走项目里已经固定使用的网关地址和端口。
     */
    return testmcu(kDefaultGatewayIp, kDefaultGatewayPort);
}

TestResult BoardBasicTester::testmcu(const std::string &ip, int port) const {

    // 用来存储线程执行结果
    bool               res = false;
    std::string        info;
    std::ostringstream detail;
    // 执行查询
    std::thread t([ & ]() {
        try {
            McuUpdater mcu(ip, port, const_cast<StatusReporter &>(this->m_reporter));

            if (mcu.init()) {
                McuStatus mcustatus = mcu.getmcuStatus();
                bool      is_valid  = !(mcustatus.reserverd == 0xA5);

                if (is_valid) {
                    g_stop_key_status = mcustatus.sensors_status & SENS_BIT_ESTOP;
                    // printf("[IMU] qx=%.2f qy=%.2f qz=%.2f qw=%.2f\n", imu.qx, imu.qy, imu.qz,
                    //        imu.qw);
                    std::cout << "can0: " << mcustatus.can_status[ 0 ]
                              << " can1: " << mcustatus.can_status[ 1 ]
                              << " can2: " << mcustatus.can_status[ 2 ]
                              << " can3: " << mcustatus.can_status[ 3 ]
                              << " key: " << static_cast<int>(mcustatus.sensors_status) << " stop "
                              << (int)g_stop_key_status << "\n"
                              << std::endl;
                    detail << "can0: " << mcustatus.can_status[ 0 ]
                           << " can1: " << mcustatus.can_status[ 1 ]
                           << " can2: " << mcustatus.can_status[ 2 ]
                           << " can3: " << mcustatus.can_status[ 3 ]
                           << " key: " << (int)mcustatus.sensors_status << "\n";
                    res  = true;
                    info = "mcustatus 查询成功[有效数据]";
                } else {
                    printf("[mcustatus] 无响应或数据无效\n");

                    res  = false;
                    info = "mcustatus 查询超时/无效";
                }
            } else {
                res  = false;
                info = "mcustatus 查询失败";
            }
        } catch (...) {
            res  = false;
            info = "mcustatus 查询异常";
        }
    });

    // 等待线程执行完毕
    t.join();

    // 线程结束后，再返回最终结果
    return {"单片机  ", res, info, trim(detail.str())};
}
TestResult BoardBasicTester::testCan() const {
    /**
     * 默认 BMS 查询入口，和 imu 一样直接使用项目默认 MCU 网关地址。
     */
    return testCan(kDefaultGatewayIp, kDefaultGatewayPort);
}

TestResult BoardBasicTester::testCan(const std::string &ip, int port) const {
    bool               res = true; // 先假设成功
    std::string        info;
    std::ostringstream detail;

    std::thread t([ & ]() {
        // 遍历 4 路 CAN：0、1、2、3
        for (int ch = 0; ch < 4; ++ch) {
            // for (int id = 1; id <= 3; ++id) {
            try {
                JointUpdater joint(ip, port, 1, ch, const_cast<StatusReporter &>(this->m_reporter));

                if (joint.init()) {
                    std::string ver = joint.getVersion();
                    printf("-[CAN%d] version: %s\n", ch, ver.c_str());
                    // 判断版本是否有效（非空 且 不是默认0.0.0）
                    bool valid = !ver.empty() && (ver.find("Timeout") == std::string::npos);
                    if (valid) {
                        // printf("[CAN%d] version: %s\n", ch, ver.c_str());
                        detail << "[CAN" << ch << "] OK: " << ver << "\n";
                    } else {
                        printf("[CAN%d] version invalid\n", ch);
                        detail << "[CAN" << ch << "] FAIL: invalid version\n";
                        res = false; // 任意一路失败 → 整体失败
                    }
                } else {
                    printf("[CAN%d] init failed\n", ch);
                    detail << "[CAN" << ch << "] FAIL: init failed\n";
                    res = false;
                }
            } catch (...) {
                printf("[CAN%d] exception\n", ch);
                detail << "[CAN" << ch << "] FAIL: exception\n";
                res = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //  }
        }
    });
    t.join();
    // 最终结果
    if (res) {
        info = "All 4-CH CAN version received successfully";
    } else {
        info = "Some CAN channels abnormal";
    }

    return {"电机  ", res, info, trim(detail.str())};
}

TestResult BoardBasicTester::testBms() const {
    /**
     * 默认 BMS 查询入口，和 imu 一样直接使用项目默认 MCU 网关地址。
     */
    return testBms(kDefaultGatewayIp, kDefaultGatewayPort);
}

TestResult BoardBasicTester::testBms(const std::string &ip, int port) const {
    // 用来存储线程执行结果
    bool               success = false;
    std::string        info;
    std::ostringstream detail;
    // 执行查询
    std::thread t([ & ]() {
        try {
            McuUpdater mcu(ip, port, const_cast<StatusReporter &>(this->m_reporter));

            if (mcu.init()) {
                BmsFeedback bat = mcu.getBattery();

                if (bat.voltage_V > 0.1f) { // 有效电池电压一定 > 0.1V
                    // printf("[Battery] %.2fV %.2fA %d%%\n", bat.voltage_V, bat.current_A,
                    //        bat.battery_level_percent);
                    detail << "[Battery] " << bat.voltage_V << "V " << bat.current_A << "A "
                           << (int)bat.battery_level_percent << "%\n";
                    success = true;
                    info    = "Battery 查询成功[有效数据]";
                } else {
                    printf("[Battery] 无响应或数据无效\n");

                    success = false;
                    info    = "Battery 查询超时/无效";
                }
            } else {
                success = false;
                info    = "Battery 查询失败";
            }
        } catch (...) {
            success = false;
            info    = "Battery 查询异常";
        }
    });

    // 等待线程执行完毕
    t.join();

    // 线程结束后，再返回最终结果
    return {"电池  ", success, info, trim(detail.str())};
}

BoardBasicTester::CommandResult BoardBasicTester::execCommand(const std::string &command) {
    /**
     * 统一通过 popen 执行 shell 命令并抓取输出。
     * 这个函数会被 sn / hwclock / bluetooth / usb 等多个测试项复用。
     */
    CommandResult         out{};
    std::array<char, 256> buffer{};

    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
        out.output = "popen failed";
        return out;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        out.output += buffer.data();
    }

    out.exitCode = pclose(pipe);
    return out;
}

std::string BoardBasicTester::readTextFile(const std::string &path) {
    /**
     * 用于读取 sysfs 等文本节点的完整内容。
     * 这些文件通常很短，顺序读完整个文件即可。
     */
    FILE *fp = fopen(path.c_str(), "r");
    if (!fp) {
        return {};
    }

    std::array<char, 256> buffer{};
    std::string           output;
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), fp) != nullptr) {
        output += buffer.data();
    }
    fclose(fp);
    return output;
}

std::string BoardBasicTester::firstLine(const std::string &value) {
    /**
     * 某些命令可能返回多行，这里只取第一行做摘要显示。
     */
    const auto pos = value.find_first_of("\r\n");
    if (pos == std::string::npos) {
        return value;
    }
    return value.substr(0, pos);
}

std::string BoardBasicTester::trim(const std::string &value) {
    /**
     * 去掉前后空白，避免命令输出中的多余换行影响判断。
     */
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }

    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string BoardBasicTester::toLower(std::string value) {
    /**
     * 用于做不区分大小写的关键字匹配。
     */
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool BoardBasicTester::contains(const std::string &haystack, const std::string &needle) {
    return haystack.find(needle) != std::string::npos;
}

bool BoardBasicTester::looksLikeSn(const std::string &value) {
    /**
     * 保留的通用 SN 形态判断：
     * 1. 长度不能太短。
     * 2. 同时包含字母和数字。
     *
     * 当前 SN 检测主要依赖 write/read 一致性，
     * 这个函数更多是作为通用工具保留。
     */
    if (value.size() < 4) {
        return false;
    }

    bool hasDigit = false;
    bool hasAlpha = false;
    for (char ch : value) {
        if (std::isdigit(static_cast<unsigned char>(ch))) {
            hasDigit = true;
        } else if (std::isalpha(static_cast<unsigned char>(ch))) {
            hasAlpha = true;
        }
    }

    return hasDigit && hasAlpha;
}

} // namespace board_test
