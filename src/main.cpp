#include "BoardBasicTester.h"
#include "StatusReporter.h"
#include "ProdTestServer.h"
#include "TestLogger.h"
#include "TestConfig.h"
#include "JointUpdater.h"
#include "JointControl.h"
#include "Protocol.h"
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

// 交互式 CLI 调试模式
int dog_test_cli() {
    signal(SIGPIPE, SIG_IGN);

    TestLogger::instance().stopConflictingServices();

    board_test::BoardBasicTester tester;
    board_test::TestResult led_ret = tester.Led_init();
    if (!led_ret.passed) {
        printf("LED 初始化失败：%s\n", led_ret.message.c_str());
    }

    StatusReporter reporter;

    std::cout << "Bpx 主控盒&驱动器 功能测试！！\n";
    std::cout << "输入 all 可测试所有功能项\n";
    std::cout << "输入 回车 可查看分项功能测试指令\n";

    std::string line;
    while (true) {
        printf("> ");
        fflush(stdout);

        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        line.clear();
        char c;
        while (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\n' || c == '\r') {
                break;
            }
            if (c == 127 || c == '\b') {
                if (!line.empty()) {
                    line.pop_back();
                    printf("\b \b");
                    fflush(stdout);
                }
            } else {
                line += c;
                printf("%c", c);
                fflush(stdout);
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        printf("\n");

        auto cmd = line;
        if (cmd == "exit" || cmd == "quit") {
            break;
        }
        if (cmd == "help" || cmd.empty()) {
            std::cout << "=== 主控盒测试指令 ===\n";
            for (const auto& c : tester.supportedCommands()) {
                std::cout << "  - " << c << "\n";
            }
            std::cout << "\n=== 驱动器测试指令 ===\n";
            std::cout << "  driver_connect <ch> <id>\n";
            std::cout << "  driver_version <ch> <id>\n";
            std::cout << "  driver_sn <ch> <id> [sn]\n";
            std::cout << "  driver_ntc <ch> <id>\n";
            std::cout << "  driver_encoder_cal <ch> <id>\n";
            std::cout << "  driver_encoder_loss <ch> <id>\n";
            std::cout << "  motor_enable <ch> <id>\n";
            std::cout << "  motor_set <ch> <id> <kp> <kd> <pos> <vel> <tor>\n";
            std::cout << "  motor_disable <ch> <id>\n";
            continue;
        }

        // 解析驱动器命令
        std::stringstream ss(cmd);
        std::string subcmd;
        ss >> subcmd;

        if (subcmd == "driver_connect") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: driver_connect <ch> <id>\n";
                continue;
            }
            JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                               id, ch, reporter);
            if (joint.init()) {
                std::cout << "[PASS] 关节连接成功 (ch=" << ch << ", id=" << id << ")\n";
                TestLogger::instance().logTestResult("driver_connect", true,
                    "ch=" + std::to_string(ch) + " id=" + std::to_string(id));
            } else {
                std::cout << "[FAIL] 关节连接失败\n";
                TestLogger::instance().logTestResult("driver_connect", false, "init failed");
            }
        }
        else if (subcmd == "driver_version") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: driver_version <ch> <id>\n";
                continue;
            }
            JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                               id, ch, reporter);
            if (joint.init()) {
                std::string ver = joint.getVersion();
                std::cout << "[PASS] 版本: " << ver << "\n";
                TestLogger::instance().logTestResult("driver_version", true, ver);
            } else {
                std::cout << "[FAIL] 版本读取失败\n";
                TestLogger::instance().logTestResult("driver_version", false, "init failed");
            }
        }
        else if (subcmd == "driver_sn") {
            int ch, id;
            std::string sn;
            ss >> ch >> id >> sn;
            if (ch < 0 || id < 0) {
                std::cout << "用法: driver_sn <ch> <id> [sn]\n";
                continue;
            }
            JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                               id, ch, reporter);
            if (!joint.init()) {
                std::cout << "[FAIL] 关节初始化失败\n";
                continue;
            }
            if (sn.empty()) {
                // 读SN
                std::string read_sn = joint.getSn();
                std::cout << "[PASS] SN: " << read_sn << "\n";
                TestLogger::instance().logTestResult("driver_sn_read", true, read_sn);
            } else {
                // 写SN
                bool ok = joint.setSn(sn);
                if (ok) {
                    std::cout << "[PASS] SN写入成功: " << sn << "\n";
                    TestLogger::instance().logTestResult("driver_sn_write", true, sn);
                } else {
                    std::cout << "[FAIL] SN写入失败\n";
                    TestLogger::instance().logTestResult("driver_sn_write", false, sn);
                }
            }
        }
        else if (subcmd == "driver_ntc") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: driver_ntc <ch> <id>\n";
                continue;
            }
            std::cout << "[INFO] NTC读取 (ch=" << ch << ", id=" << id << ")\n";
            std::cout << "[INFO] 暂未实现完整NTC读取\n";
            // TODO: 实现完整的NTC读取
            TestLogger::instance().logTestResult("driver_ntc", false, "not implemented");
        }
        else if (subcmd == "driver_encoder_cal") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: driver_encoder_cal <ch> <id>\n";
                continue;
            }
            std::cout << "[INFO] 编码器校准 (ch=" << ch << ", id=" << id << ")\n";
            std::cout << "[INFO] 暂未实现完整编码器校准\n";
            // TODO: 实现编码器校准
            TestLogger::instance().logTestResult("driver_encoder_cal", false, "not implemented");
        }
        else if (subcmd == "driver_encoder_loss") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: driver_encoder_loss <ch> <id>\n";
                continue;
            }
            std::cout << "[INFO] 副编码器丢失检测 (ch=" << ch << ", id=" << id << ")\n";
            std::cout << "[INFO] 暂未实现副编码器检测\n";
            // TODO: 实现副编码器丢失检测
            TestLogger::instance().logTestResult("driver_encoder_loss", false, "not implemented");
        }
        else if (subcmd == "motor_enable") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: motor_enable <ch> <id>\n";
                continue;
            }
            JointId jid;
            jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
            jid.can_ch = ch;
            jid.can_id = id;

            JointCfg cfg;
            cfg.canfd = 1;
            cfg.extend_id = 0;
            cfg.bitrate_switch = 1;

            JointControl ctrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
            if (ctrl.init()) {
                if (!ctrl.enable(true)) {
                    std::cout << "[FAIL] 电机使能发送失败\n";
                    TestLogger::instance().logTestResult("motor_enable", false, "mode+enable send failed");
                    continue;
                }
                std::cout << "[PASS] 电机使能已发送 (ch=" << ch << ", id=" << id << ")\n";
                TestLogger::instance().logTestResult("motor_enable", true,
                    "ch=" + std::to_string(ch) + " id=" + std::to_string(id) + " sent");
            } else {
                std::cout << "[FAIL] 电机使能失败\n";
                TestLogger::instance().logTestResult("motor_enable", false, "init failed");
            }
        }
        else if (subcmd == "motor_set") {
            int ch, id;
            float kp, kd, pos, vel, tor;
            if (!(ss >> ch >> id >> kp >> kd >> pos >> vel >> tor)) {
                std::cout << "用法: motor_set <ch> <id> <kp> <kd> <pos> <vel> <tor>\n";
                continue;
            }
            JointId jid;
            jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
            jid.can_ch = ch;
            jid.can_id = id;

            JointCfg cfg;
            cfg.canfd = 1;
            cfg.extend_id = 0;
            cfg.bitrate_switch = 1;

            JointControl ctrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
            if (ctrl.init()) {
                if (!ctrl.enable(true)) {
                    std::cout << "[FAIL] 电机使能发送失败\n";
                    TestLogger::instance().logTestResult("motor_set", false, "mode+enable send failed");
                    continue;
                }
                ctrl.command().kp = kp;
                ctrl.command().kd = kd;
                ctrl.command().position = pos;
                ctrl.command().velocity = vel;
                ctrl.command().torque = tor;
                ctrl.sendControl();
                if (!ctrl.receiveAndDecodeFeedback()) {
                    std::cout << "[FAIL] 未收到电机反馈 0xA0/RPDO (ch=" << ch << ", id=" << id << ")\n";
                    TestLogger::instance().logTestResult("motor_set", false, "feedback timeout");
                    continue;
                }
                auto& fb = ctrl.getFeedback();
                std::cout << "[PASS] 电机参数设置成功\n";
                std::cout << "  pos=" << fb.position.load() << " vel=" << fb.velocity.load()
                          << " tor=" << fb.torque.load() << "\n";
                TestLogger::instance().logTestResult("motor_set", true,
                    "kp=" + std::to_string(kp) + " kd=" + std::to_string(kd));
            } else {
                std::cout << "[FAIL] 电机控制失败\n";
                TestLogger::instance().logTestResult("motor_set", false, "init failed");
            }
        }
        else if (subcmd == "motor_disable") {
            int ch, id;
            if (!(ss >> ch >> id)) {
                std::cout << "用法: motor_disable <ch> <id>\n";
                continue;
            }
            JointId jid;
            jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
            jid.can_ch = ch;
            jid.can_id = id;

            JointCfg cfg;
            cfg.canfd = 1;
            cfg.extend_id = 0;
            cfg.bitrate_switch = 1;

            JointControl ctrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
            if (ctrl.init()) {
                ctrl.enable(false);
                std::cout << "[PASS] 电机禁用成功\n";
                TestLogger::instance().logTestResult("motor_disable", true,
                    "ch=" + std::to_string(ch) + " id=" + std::to_string(id));
            } else {
                std::cout << "[FAIL] 电机禁用失败\n";
                TestLogger::instance().logTestResult("motor_disable", false, "init failed");
            }
        }
        else {
            // 尝试作为主控盒测试命令处理
            auto result = tester.run(cmd);
            std::cout << "[" << result.name << "] " << (result.passed ? "PASS" : "FAIL") << "\n";
            std::cout << "  message: " << result.message << "\n";
            if (!result.detail.empty()) {
                std::cout << "  detail:\n" << result.detail << "\n";
            }

            TestLogger::instance().logTestResult(result.name, result.passed,
                                                  result.message, result.detail);
        }
    }

    TestLogger::instance().restartConflictingServices();
    return 0;
}

// Daemon 模式：启动产测服务器
int dog_daemon() {
    signal(SIGPIPE, SIG_IGN);

    StatusReporter reporter;
    reporter.setEnabled(false);
    ProdTestServer server(9878, reporter);

    // 让StatusReporter使用ProdTestServer的socket，这样所有响应都从9878端口发出
    reporter.setExternalSocket(server.getSocketFd());

    std::cout << "========================================" << std::endl;
    std::cout << "  Dog Tool ProdTest Server v3.0        " << std::endl;
    std::cout << "  - Server Listen: UDP 9878            " << std::endl;
    std::cout << "  - All responses sent from 9878       " << std::endl;
    std::cout << "========================================" << std::endl;

    TestLogger::instance().logTestResult("system", true, "Server started");

    server.start();
    return 0;
}

int main(int argc, char* argv[]) {
    // 只保留 daemon 和 CLI 两种运行模式；默认进入 CLI。
    bool daemonMode = false;
    bool cliMode = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--daemon") == 0) {
            daemonMode = true;
        } else if (strcmp(argv[i], "--cli") == 0) {
            cliMode = true;
        }
    }

    if (daemonMode) {
        return dog_daemon();
    }

    if (!cliMode) {
        std::cout << "Dog Tool - Production Test Server" << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  --daemon         Start as daemon (controlled by client)" << std::endl;
        std::cout << "  --cli            Interactive CLI mode" << std::endl;
        std::cout << std::endl;
        std::cout << "Starting in CLI mode..." << std::endl;
    }

    return dog_test_cli();
}
