#pragma once
#include "Protocol.h"
#include "StatusReporter.h" // 必须包含
#include "UdpClient.h"
#include "mcu_soc_com_protocol.h"
#include <functional>
#include <memory>
#include <string>

class McuUpdater {
public:
    // 构造函数增加 reporter
    McuUpdater(const std::string &ip, int port, StatusReporter &reporter);
    bool        init();
    std::string getVersion();
    bool        upgrade(const std::string &filePath);
    ImuFeedback getIMU();
    BmsFeedback getBattery();
    McuStatus   getmcuStatus();
    McuStatus   getmcuStatus_wait();
    void        setProgressCallback(std::function<void(int, const std::string &)> cb);

private:
    bool sendAndWaitAck(uint8_t cmd, const std::vector<uint8_t> &payload, uint8_t frameId,
                        uint32_t &out_write_pos, int timeout_ms);

    std::unique_ptr<UdpClient> m_network;
    uint8_t                    m_frameCounter;
    StatusReporter            &m_reporter; // 引用成员
    std::function<void(int, const std::string &)> m_progressCallback;
};
