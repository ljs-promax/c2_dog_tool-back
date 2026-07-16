#pragma once
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include "UdpClient.h"
#include "Protocol.h"
#include "StatusReporter.h" // 必须包含
#include "mcu_soc_com_protocol.h"
class JointUpdater {
public:
    // 构造函数增加 reporter
    JointUpdater(const std::string &ip, int port, int nodeID, int channel,
                 StatusReporter &reporter);
    bool        init();
    std::string getVersion();
    std::string getSn();
    bool        setSn(const std::string &sn);
    bool        upgrade(const std::string &filePath);
    void        setProgressCallback(std::function<void(int, const std::string &)> cb);

private:
    bool    packAndSendCan(uint32_t canId, const std::vector<uint8_t> &payload);
    bool    waitForCanResponse(uint32_t expectedCanId, const std::vector<uint8_t> &expectedPayload);
    uint8_t bytesToDlc(size_t bytes);
    bool    readSnField(uint32_t reqId, uint32_t respId, uint16_t index, std::string &out4chars);

    bool writeSnField(uint32_t reqId, uint32_t respId, uint16_t index, const std::string &in4chars);

    std::unique_ptr<UdpClient> m_network;
    int                        m_nodeID;
    int                        m_channel;
    uint8_t                    m_frameCounter;
    std::string                m_deviceName; // 存储 joint_chX_idY
    StatusReporter            &m_reporter;   // 引用成员
    std::function<void(int, const std::string &)> m_progressCallback;
};
