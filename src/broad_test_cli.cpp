#include "board_test/BoardBasicTester.h"

#include <iostream>
#include <sstream>
#include <string>

// 去掉用户输入前后的空白字符，避免因为多打了空格导致命令识别失败。
static std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

// 兼容两种输入习惯：
// 1. 直接输入命令：usb / sn / serial /dev/ttyS0 / imu / bms
// 2. 带前缀输入：check usb / test sn
static std::string normalizeCommand(const std::string& input) {
    std::stringstream ss(input);
    std::string first;
    ss >> first;
    if (first == "check" || first == "test") {
        std::string rest;
        std::getline(ss, rest);
        return trim(rest);
    }
    return input;
}

// 打印 help 信息时，直接复用 BoardBasicTester 里注册的命令列表。
static void printHelp(const board_test::BoardBasicTester& tester) {
    std::cout << "Supported commands:\n";
    for (const auto& cmd : tester.supportedCommands()) {
        std::cout << "  - " << cmd << "\n";
    }
    std::cout << "\nExamples:\n";
    std::cout << "  serial /dev/ttyS0\n";
    std::cout << "  imu\n";
    std::cout << "  imu 10.21.32.121 43893\n";
    std::cout << "  bms\n";
    std::cout << "  bms 10.21.32.121 43893\n";
}

// 所有测试项统一按 TestResult 格式打印，方便以后改成日志或网络回包。
static void printResult(const board_test::TestResult& result) {
    std::cout << "[" << result.name << "] " << (result.passed ? "PASS" : "FAIL") << "\n";
    std::cout << "  message: " << result.message << "\n";
    if (!result.detail.empty()) {
        std::cout << "  detail:\n" << result.detail << "\n";
    }
}

int main() {
    // CLI 只负责和用户交互，真正的检测逻辑全部下放到 BoardBasicTester。
    board_test::BoardBasicTester tester;

    std::cout << "broad_test console\n";
    std::cout << "Type help for command list, exit to quit.\n";

    std::string line;
    while (true) {
        std::cout << "> ";
        if (!std::getline(std::cin, line)) {
            break;
        }

        // exit / quit 直接退出程序。
        const auto cmd = trim(line);
        if (cmd == "exit" || cmd == "quit") {
            break;
        }

        // 空行或 help 不执行测试，而是打印帮助信息。
        if (cmd == "help" || cmd.empty()) {
            printHelp(tester);
            continue;
        }

        // 这里把输入统一归一化后再交给 tester，
        // 保证 CLI 层尽量薄，后续接 UDP 服务时可以复用 tester.run()。
        printResult(tester.run(normalizeCommand(cmd)));
    }

    return 0;
}
