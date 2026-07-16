#ifndef JOINT_CONTROL_HPP
#define JOINT_CONTROL_HPP

#include "UdpClient.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include "mcu_soc_com_protocol.h"
#include "Protocol.h"
#pragma pack(push, 1)
// 你最新的结构体 【完全按你提供】
// typedef struct {
//     uint16_t header;     // 替代原 magic (0x4248)
//     uint8_t  frame_id;   // 本地累加
//     uint8_t  version;    // 协议版本
//     uint8_t  cmd;        // 命令码 0xA0
//     uint8_t  packet_num; // 本包帧数
//     uint16_t data_len;   // buffer[0..data_len-1] 有效
//     uint16_t crc_sum;    // 对 buffer 段做 16-bit 累加和
// } DataHeader;

// typedef struct {
//     DataHeader header;
//     uint8_t    buffer[ 1024 ];
// } CanFdNetBuffer;

// typedef union CANHeader {
//     uint8_t buffer[ 10 ];
//     struct {
//         uint8_t board_id; // 转接板 IP 最后一位
//         uint8_t can_ch;   // CAN 通道
//         struct {
//             uint32_t id : 29;
//             uint32_t : 2;
//             uint32_t transmit_timestamp_enable : 1;
//         };
//         struct {
//             uint32_t dlc : 4; // 0..15
//             uint32_t bitrate_switch : 1;
//             uint32_t canfd_frame : 1;
//             uint32_t remote_frame : 1;
//             uint32_t extend_id : 1;
//             uint32_t : 24;
//         };
//     };
// } CANHeader;

// struct Net2CanFrame {
//     CANHeader               header;
//     std::array<uint8_t, 64> data{};
// };
#pragma pack(pop)

// 配置与数据结构
struct JointId {
    uint8_t  board_id = DefaultConfig::GATEWAY_BOARD_ID;
    uint8_t  can_ch   = 0;
    uint32_t can_id   = 0;
};

struct JointCfg {
    bool extend_id      = false;
    bool canfd          = false;
    bool bitrate_switch = false;
    bool remote_frame   = false;
};

struct JointCmd {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque   = 0.0f;
    float kp       = 0.0f;
    float kd       = 0.0f;
};
struct Feedback {
    std::atomic<float>    position{0.0f};
    std::atomic<float>    velocity{0.0f};
    std::atomic<float>    torque{0.0f};
    std::atomic<float>    motor_temperature{0.0f};
    std::atomic<float>    driver_temperature{0.0f};
    std::atomic<uint16_t> error_code{0};
};

struct JointReceiveResult {
    bool feedback = false;
    bool sdo_ack = false;
};

class JointControl {
public:
    JointControl(const std::string &ip, int port, JointId id, JointCfg cfg);
    bool init();

    void         decodeFeedback(const Net2CanFrameWrapper &f);
    bool         receiveAndDecodeFeedback(int timeoutMs = 1500);
    JointReceiveResult receiveAndDecodeFeedbackOrSdoAck(uint16_t ackIndex,
                                                        uint8_t ackSubIndex,
                                                        bool waitForSdoAck,
                                                        int timeoutMs = 1500,
                                                        uint32_t expectedRespCanId = 0,
                                                        uint32_t altExpectedRespCanId = 0);
    int          drainPendingPackets(int maxPackets = 64, int quietTimeoutMs = 0);
    bool         receiveAndDecodeErrorCode(uint16_t &error_code, int timeoutMs = 1500);
    bool         receiveSdoAck(uint16_t index,
                               uint8_t subIndex = 0x00,
                               int timeoutMs = 1500,
                               uint32_t expectedRespCanId = 0,
                               uint32_t altExpectedRespCanId = 0);
    void         decodeErrorCode(const Net2CanFrameWrapper &f, uint16_t &error_code);
    Net2CanFrameWrapper makeReadErrorCodeFrame() const;
    Net2CanFrameWrapper makeReadU8Frame(uint16_t index, uint8_t subIndex) const;
    Net2CanFrameWrapper makeEncoderCalcFrame();
    Net2CanFrameWrapper makeEncoderZeroFrame();
    bool         writeSdoU8(uint16_t index, uint8_t subIndex, uint8_t value, int timeoutMs = 1500);
    bool         readSdoU8(uint16_t index, uint8_t subIndex, uint8_t &value, int timeoutMs = 1500);
    bool         readSdoU16(uint16_t index, uint8_t subIndex, uint16_t &value, int timeoutMs = 1500);
    bool         changeNodeId(uint8_t newCanId, int timeoutMs = 1500);
    bool         sendPacket(const Net2CanFrameWrapper &f);
    void         printNet2CanFrame(const Net2CanFrameWrapper &f);
    // 获取反馈数据
    const Feedback &getFeedback() const {
        return fb_;
    }
    // 你原来的接口 【完全保留】
    JointCmd &command() noexcept {
        return cmd_;
    }
    const JointCmd &command() const noexcept {
        return cmd_;
    }

    const JointId &id() const noexcept {
        return id_;
    }
    const JointCfg &cfg() const noexcept {
        return cfg_;
    }

    // 功能接口
    void enable(bool on);
    void setControlMode(uint8_t mode);
    bool setControlModeAndWait(uint8_t mode, int timeoutMs = 1500);
    bool enableAndWait(bool on, int timeoutMs = 1500);
    void setControlPdoBase(uint32_t base);
    uint32_t controlPdoBase() const;
    bool sendControl();

    // 批量编码（多帧）
    std::vector<uint8_t> encodeBatch(const std::vector<Net2CanFrameWrapper> &frames);

private:
    std::vector<Net2CanFrameWrapper> decodeCanFrames(const uint8_t *buffer, size_t size) const;
    uint8_t dlcToLen(uint8_t dlc) const;
    uint8_t lenToDlc(uint8_t dlc) const;
    bool isSdoResponseFrame(const Net2CanFrameWrapper &f,
                            uint32_t expectedRespCanId = 0,
                            uint32_t altExpectedRespCanId = 0) const;
    bool isSdoAckFrame(const Net2CanFrameWrapper &f,
                       uint16_t index,
                       uint8_t subIndex,
                       uint32_t expectedRespCanId = 0,
                       uint32_t altExpectedRespCanId = 0) const;
    void cacheSdoResponse(const Net2CanFrameWrapper &f);
    bool takeCachedSdoAck(uint16_t index,
                          uint8_t subIndex,
                          uint32_t expectedRespCanId = 0,
                          uint32_t altExpectedRespCanId = 0);

    uint16_t  calcCrc(const uint8_t *buf, size_t len);
    CANHeader buildCanHeader(uint32_t can_id, uint8_t dlc) const;

    Net2CanFrameWrapper makeControlFrame();
    Net2CanFrameWrapper makeEnableFrame(bool on);
    Net2CanFrameWrapper makeModeFrame(uint8_t mode);
    Net2CanFrameWrapper makeWriteU8Frame(uint16_t index, uint8_t subIndex, uint8_t value) const;

private:
    JointId                    id_;
    JointCfg                   cfg_;
    JointCmd                   cmd_;
    Feedback                   fb_;
    std::unique_ptr<UdpClient> m_network;
    std::atomic<uint8_t>       frame_seq_{0};
    std::atomic<uint32_t>      control_pdo_base_{0x100};
    std::mutex                 m_sdoMutex;
    std::deque<Net2CanFrameWrapper> m_pendingSdoResponses;
};

#endif
