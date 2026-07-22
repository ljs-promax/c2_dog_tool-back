#include "JointControl.h"
#include <algorithm>
#include <chrono>
#include <thread>
JointControl::JointControl(const std::string &ip, int port, JointId id, JointCfg cfg)
    : id_(id)
    , cfg_(cfg) {
    m_network = std::make_unique<UdpClient>(ip, port, 8086);
}

bool JointControl::init() {
    return m_network && m_network->init();
}

uint8_t JointControl::dlcToLen(uint8_t dlc) const {
    if (dlc <= 8)
        return dlc;
    switch (dlc) {
        case 9:
            return 12;
        case 10:
            return 16;
        case 11:
            return 20;
        case 12:
            return 24;
        case 13:
            return 32;
        case 14:
            return 48;
        case 15:
            return 64;
        default:
            return 0;
    }
}

uint16_t JointControl::calcCrc(const uint8_t *buf, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc += buf[ i ];
    return crc;
}

std::vector<Net2CanFrameWrapper> JointControl::decodeCanFrames(const uint8_t *buffer,
                                                               size_t size) const {
    std::vector<Net2CanFrameWrapper> frames;

    if (size >= sizeof(Net2CanFrame) && size % sizeof(Net2CanFrame) == 0) {
        for (size_t offset = 0; offset + sizeof(Net2CanFrame) <= size;
             offset += sizeof(Net2CanFrame)) {
            Net2CanFrame raw{};
            std::memcpy(raw.raw_buffer, buffer + offset, sizeof(Net2CanFrame));
            frames.emplace_back(raw);
        }
        return frames;
    }

    size_t offset = 0;
    while (offset + sizeof(CANHeader) <= size) {
        Net2CanFrameWrapper f{};
        std::memcpy(f.header.buffer, buffer + offset, sizeof(CANHeader));
        offset += sizeof(CANHeader);

        uint8_t len = dlcToLen(f.header.dlc);
        if (offset + len > size) {
            break;
        }

        if (len > 0) {
            std::memcpy(f.data.data(), buffer + offset, len);
        }
        offset += len;
        frames.push_back(f);
    }

    return frames;
}

uint8_t JointControl::lenToDlc(uint8_t len) const {
    if (len <= 8)
        return len;
    if (len <= 12)
        return 9;
    if (len <= 16)
        return 10;
    if (len <= 20)
        return 11;
    if (len <= 24)
        return 12;
    if (len <= 32)
        return 13;
    if (len <= 48)
        return 14;
    return 15;
}
void JointControl::printNet2CanFrame(const Net2CanFrameWrapper &f) {
    // 打印 CAN 头 10 字节
    printf("[Net2CanFrameWrapper] Header 10bytes: ");
    for (int i = 0; i < 10; i++) {
        printf("%02X ", f.header.buffer[ i ]);
    }
    printf("\n");

    // 打印数据长度
    uint8_t L = dlcToLen(f.header.dlc);
    printf("[Net2CanFrameWrapper] Data len: %u bytes\n", L);

    // 打印数据内容
    printf("[Net2CanFrameWrapper] Data: ");
    for (int i = 0; i < L; i++) {
        printf("%02X ", f.data[ i ]);
    }
    printf("\n");
}
// ==============================
// 对照官方：构建控制帧（16字节）
// ==============================
Net2CanFrameWrapper JointControl::makeControlFrame() {
    Net2CanFrameWrapper f{};
    f.header.board_id                  = id_.board_id;
    f.header.can_ch                    = id_.can_ch;
    f.header.id                        = id_.can_id + control_pdo_base_.load();
    f.header.transmit_timestamp_enable = 0;
    f.header.bitrate_switch            = cfg_.bitrate_switch;
    f.header.canfd_frame               = cfg_.canfd;
    f.header.remote_frame              = 0;
    f.header.extend_id                 = cfg_.extend_id;

    const uint8_t len = 16;
    f.header.dlc      = lenToDlc(len);

    uint16_t kp = cmd_.kp * 100;
    uint16_t kd = cmd_.kd * 100;

    memcpy(&f.data[ 0 ], &cmd_.position, 4);
    memcpy(&f.data[ 4 ], &cmd_.velocity, 4);
    memcpy(&f.data[ 8 ], &cmd_.torque, 4);
    memcpy(&f.data[ 12 ], &kp, 2);
    memcpy(&f.data[ 14 ], &kd, 2);

    return f;
}

// ==============================
// 对照官方：模式帧（0x600 偏移）
// ==============================
Net2CanFrameWrapper JointControl::makeModeFrame(uint8_t mode) {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600; // ✅ 官方 SDO 偏移
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = 0;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    f.data[ 0 ] = 0x2F;
    f.data[ 1 ] = 0x03;
    f.data[ 2 ] = 0x20;
    f.data[ 3 ] = 0x00;
    f.data[ 4 ] = mode;
    f.data[ 5 ] = 0x00;
    f.data[ 6 ] = 0x00;
    f.data[ 7 ] = 0x00;

    return f;
}
Net2CanFrameWrapper JointControl::makeReadErrorCodeFrame() const {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600;
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = false;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    // 读取 0x2000 错误码（2字节）
    f.data[ 0 ] = 0x40;
    f.data[ 1 ] = 0x00;
    f.data[ 2 ] = 0x20;
    f.data[ 3 ] = 0x00;

    return f;
}

Net2CanFrameWrapper JointControl::makeReadU8Frame(uint16_t index, uint8_t subIndex) const {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600;
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = false;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    f.data[0] = 0x40;
    f.data[1] = static_cast<uint8_t>(index & 0xFF);
    f.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    f.data[3] = subIndex;
    f.data[4] = 0x00;
    f.data[5] = 0x00;
    f.data[6] = 0x00;
    f.data[7] = 0x00;

    return f;
}
void JointControl::decodeErrorCode(const Net2CanFrameWrapper &f, uint16_t &error_code) {
    const uint8_t L = dlcToLen(f.header.dlc);

    // 回包长度 >=6 才包含错误码（4字节指令 + 2字节数据）
    if (L >= 6) {
        std::memcpy(&error_code, &f.data[ 4 ], 2);
    } else {
        error_code = 0xFFFF; // 无效值
    }
}
Net2CanFrameWrapper JointControl::makeControlWordFrame(uint16_t controlWord) {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600;
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = 0;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    f.data[ 0 ] = 0x2B;
    f.data[ 1 ] = 0x02;
    f.data[ 2 ] = 0x20;
    f.data[ 3 ] = 0x00;
    f.data[ 4 ] = static_cast<uint8_t>(controlWord & 0xFF);
    f.data[ 5 ] = static_cast<uint8_t>((controlWord >> 8) & 0xFF);
    f.data[ 6 ] = 0x00;
    f.data[ 7 ] = 0x00;

    return f;
}

// ==============================
// 对照官方：使能帧
// ==============================
Net2CanFrameWrapper JointControl::makeEnableFrame(bool on) {
    return makeControlWordFrame(on ? 0x0001 : 0x0002);
}

Net2CanFrameWrapper JointControl::makeEncoderCalcFrame() {
    return makeControlWordFrame(0x00F1);
}

Net2CanFrameWrapper JointControl::makeEncoderZeroFrame() {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600;
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = 0;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    f.data[ 0 ] = 0x2B;
    f.data[ 1 ] = 0x70;
    f.data[ 2 ] = 0x20;
    f.data[ 3 ] = 0x00;
    f.data[ 4 ] = 0x00;
    f.data[ 5 ] = 0x00;
    f.data[ 6 ] = 0x00;
    f.data[ 7 ] = 0x00;

    return f;
}

Net2CanFrameWrapper JointControl::makeWriteU8Frame(uint16_t index, uint8_t subIndex,
                                                   uint8_t value) const {
    Net2CanFrameWrapper f{};
    f.header.board_id       = id_.board_id;
    f.header.can_ch         = id_.can_ch;
    f.header.id             = id_.can_id + 0x600;
    f.header.bitrate_switch = cfg_.bitrate_switch;
    f.header.canfd_frame    = cfg_.canfd;
    f.header.remote_frame   = 0;
    f.header.extend_id      = cfg_.extend_id;

    const uint8_t len = 8;
    f.header.dlc      = lenToDlc(len);

    f.data[0] = 0x2F;
    f.data[1] = static_cast<uint8_t>(index & 0xFF);
    f.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    f.data[3] = subIndex;
    f.data[4] = value;
    f.data[5] = 0x00;
    f.data[6] = 0x00;
    f.data[7] = 0x00;

    return f;
}

// ==============================
// 正确打包：一帧接一帧
// ==============================
std::vector<uint8_t> JointControl::encodeBatch(const std::vector<Net2CanFrameWrapper> &frames) {
    CanFdNetBuffer pkt{};
    size_t         offset = 0;

    for (const auto &f : frames) {
        memcpy(pkt.buffer + offset, f.header.buffer, 10);
        offset += 10;

        uint8_t len = dlcToLen(f.header.dlc);
        if (len > 0) {
            memcpy(pkt.buffer + offset, f.data.data(), len);
            offset += len;
        }
    }

    pkt.header.header     = htole16(Magic::BH_JOINT);
    pkt.header.frame_id   = frame_seq_++;
    pkt.header.version    = 1;
    pkt.header.cmd        = 0xA0;
    pkt.header.packet_num = frames.size();
    pkt.header.data_len   = htole16(offset);
    pkt.header.crc_sum    = htole16(calcCrc(pkt.buffer, offset));

    std::vector<uint8_t> out(sizeof(DataHeaderMM) + offset);
    memcpy(out.data(), &pkt.header, sizeof(DataHeaderMM));
    memcpy(out.data() + sizeof(DataHeaderMM), pkt.buffer, offset);

    return out;
}
// ============================================================
// 参照你官方代码 1:1 实现的回包解析
// ============================================================
void JointControl::decodeFeedback(const Net2CanFrameWrapper &f) {
    const uint8_t L = dlcToLen(f.header.dlc);

    if (L >= 16) {
        float   pf, vf, tf;
        int16_t motor_temp, driver_temp;

        std::memcpy(&pf, &f.data[ 0 ], 4);
        std::memcpy(&vf, &f.data[ 4 ], 4);
        std::memcpy(&tf, &f.data[ 8 ], 4);
        std::memcpy(&motor_temp, &f.data[ 12 ], 2);
        std::memcpy(&driver_temp, &f.data[ 14 ], 2);

        float mt = static_cast<float>(motor_temp) / 10.0f;
        float dt = static_cast<float>(driver_temp) / 10.0f;

        fb_.position.store(pf, std::memory_order_relaxed);
        fb_.velocity.store(vf, std::memory_order_relaxed);
        fb_.torque.store(tf, std::memory_order_relaxed);
        fb_.motor_temperature.store(mt, std::memory_order_relaxed);
        fb_.driver_temperature.store(dt, std::memory_order_relaxed);
    }

    if (L >= 20) {
        uint16_t error_code;
        std::memcpy(&error_code, &f.data[ 16 ], 2);
        fb_.error_code.store(error_code, std::memory_order_relaxed);
    } else if (L >= 16) {
        fb_.error_code.store(0, std::memory_order_relaxed);
    }
}

bool JointControl::isSdoResponseFrame(const Net2CanFrameWrapper &f,
                                      uint32_t expectedRespCanId,
                                      uint32_t altExpectedRespCanId) const {
    const uint8_t L = dlcToLen(f.header.dlc);
    const uint32_t primaryRespCanId =
        expectedRespCanId != 0 ? expectedRespCanId : id_.can_id + 0x580;
    const bool matchedRespCanId =
        f.header.id == primaryRespCanId ||
        (altExpectedRespCanId != 0 && f.header.id == altExpectedRespCanId);
    return f.header.can_ch == id_.can_ch && matchedRespCanId && L >= 4;
}

bool JointControl::isSdoAckFrame(const Net2CanFrameWrapper &f,
                                 uint16_t index,
                                 uint8_t subIndex,
                                 uint32_t expectedRespCanId,
                                 uint32_t altExpectedRespCanId) const {
    if (!isSdoResponseFrame(f, expectedRespCanId, altExpectedRespCanId)) {
        return false;
    }

    uint16_t respIndex = static_cast<uint16_t>(f.data[1]) |
                         (static_cast<uint16_t>(f.data[2]) << 8);
    return f.data[0] == 0x60 && respIndex == index && f.data[3] == subIndex;
}

void JointControl::cacheSdoResponse(const Net2CanFrameWrapper &f) {
    std::lock_guard<std::mutex> lock(m_sdoMutex);
    if (m_pendingSdoResponses.size() >= 32) {
        m_pendingSdoResponses.pop_front();
    }
    m_pendingSdoResponses.push_back(f);
}

bool JointControl::takeCachedSdoAck(uint16_t index,
                                    uint8_t subIndex,
                                    uint32_t expectedRespCanId,
                                    uint32_t altExpectedRespCanId) {
    std::lock_guard<std::mutex> lock(m_sdoMutex);
    for (auto it = m_pendingSdoResponses.begin(); it != m_pendingSdoResponses.end(); ++it) {
        if (isSdoAckFrame(*it, index, subIndex, expectedRespCanId, altExpectedRespCanId)) {
            m_pendingSdoResponses.erase(it);
            return true;
        }
    }
    return false;
}

JointReceiveResult JointControl::receiveAndDecodeFeedbackOrSdoAck(uint16_t ackIndex,
                                                                  uint8_t ackSubIndex,
                                                                  bool waitForSdoAck,
                                                                  int timeoutMs /*=1500*/,
                                                                  uint32_t expectedRespCanId,
                                                                  uint32_t altExpectedRespCanId) {
    auto startTime = std::chrono::steady_clock::now();
    JointReceiveResult result{};

    if (waitForSdoAck &&
        takeCachedSdoAck(ackIndex, ackSubIndex, expectedRespCanId, altExpectedRespCanId)) {
        result.sdo_ack = true;
        return result;
    }

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs) {
            break; // 超时退出
        }

        std::vector<uint8_t> resp;
        int                  receiveTimeoutMs = std::max(1, std::min(500, timeoutMs - elapsed));
        int                  n = m_network->receive(resp, receiveTimeoutMs);

        // 长度不够包头，跳过
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        //  校验命令
        if (rHdr->cmd == 0xA0) {
            uint16_t data_len = le16toh(rHdr->data_len);
            if (data_len == 0 || n < (int)(sizeof(DataHeaderMM) + data_len)) {
                continue;
            }

            uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
            for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
                uint8_t L = dlcToLen(f.header.dlc);
                if (f.header.can_ch == id_.can_ch && f.header.id == id_.can_id + 0x190 && L >= 16) {
                    decodeFeedback(f);
                    result.feedback = true;
                    continue;
                }

                if (isSdoResponseFrame(f, expectedRespCanId, altExpectedRespCanId)) {
                    if (waitForSdoAck &&
                        isSdoAckFrame(f, ackIndex, ackSubIndex, expectedRespCanId,
                                      altExpectedRespCanId)) {
                        result.sdo_ack = true;
                    } else {
                        cacheSdoResponse(f);
                    }
                    continue;
                }

            }

            if ((!waitForSdoAck && result.feedback) || (waitForSdoAck && result.sdo_ack)) {
                return result;
            }
        } else if (rHdr->cmd != 0xE1) {
            printf("[JointControl] ignore packet cmd=0x%02X len=%d\n", rHdr->cmd, n);
        }
    }

    return result;
}

// 接收回包并解析反馈（超时 1500ms）
bool JointControl::receiveAndDecodeFeedback(int timeoutMs /*=1500*/) {
    return receiveAndDecodeFeedbackOrSdoAck(0, 0, false, timeoutMs).feedback;
}

int JointControl::drainPendingPackets(int maxPackets, int quietTimeoutMs) {
    if (!m_network || maxPackets <= 0) {
        return 0;
    }

    int drained = 0;
    while (drained < maxPackets) {
        std::vector<uint8_t> resp;
        int n = m_network->receive(resp, quietTimeoutMs);
        if (n <= 0) {
            break;
        }
        ++drained;

        if (n < static_cast<int>(sizeof(DataHeaderMM))) {
            continue;
        }

        DataHeaderMM *rHdr = reinterpret_cast<DataHeaderMM *>(resp.data());
        if (rHdr->cmd != 0xA0) {
            continue;
        }

        uint16_t data_len = le16toh(rHdr->data_len);
        if (data_len == 0 || n < static_cast<int>(sizeof(DataHeaderMM) + data_len)) {
            continue;
        }

        uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
        for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
            uint8_t L = dlcToLen(f.header.dlc);
            if (f.header.can_ch == id_.can_ch && f.header.id == id_.can_id + 0x190 && L >= 16) {
                decodeFeedback(f);
            }
        }
    }

    return drained;
}

bool JointControl::receiveAndDecodeErrorCode(uint16_t &error_code, int timeoutMs /*=1500*/) {
    auto startTime = std::chrono::steady_clock::now();
    error_code     = 0xFFFF; // 默认无效

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs) {
            break; // 超时退出
        }

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 500);

        // 长度不够包头，跳过
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();

        //  校验 命令
        if (rHdr->cmd == 0xA0) {
            uint16_t data_len = le16toh(rHdr->data_len);
            if (data_len == 0 || n < (int)(sizeof(DataHeaderMM) + data_len)) {
                continue;
            }

            uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
            for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
                decodeErrorCode(f, error_code);
            }

            return true;
        }
    }

    return false;
}

bool JointControl::receiveSdoAck(uint16_t index,
                                 uint8_t subIndex,
                                 int timeoutMs /*=1500*/,
                                 uint32_t expectedRespCanId,
                                 uint32_t altExpectedRespCanId) {
    auto startTime = std::chrono::steady_clock::now();

    if (takeCachedSdoAck(index, subIndex, expectedRespCanId, altExpectedRespCanId)) {
        return true;
    }

    while (true) {
        if (takeCachedSdoAck(index, subIndex, expectedRespCanId, altExpectedRespCanId)) {
            return true;
        }

        auto now = std::chrono::steady_clock::now();
        int elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs) {
            break;
        }

        std::vector<uint8_t> resp;
        int n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM)) {
            continue;
        }

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd != 0xA0) {
            continue;
        }

        uint16_t data_len = le16toh(rHdr->data_len);
        if (data_len == 0 || n < (int)(sizeof(DataHeaderMM) + data_len)) {
            continue;
        }

        uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
        for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
            uint8_t L = dlcToLen(f.header.dlc);
            if (f.header.can_ch == id_.can_ch && f.header.id == id_.can_id + 0x190 && L >= 16) {
                decodeFeedback(f);
                continue;
            }

            if (isSdoResponseFrame(f, expectedRespCanId, altExpectedRespCanId)) {
                if (isSdoAckFrame(f, index, subIndex, expectedRespCanId, altExpectedRespCanId)) {
                    return true;
                }
                uint16_t respIndex = static_cast<uint16_t>(f.data[1]) |
                                     (static_cast<uint16_t>(f.data[2]) << 8);
                if (f.data[0] == 0x80 || respIndex == index) {
                    std::printf("[SDO RX] wait index=0x%04X sub=0x%02X got can_id=0x%03X "
                                "cs=0x%02X index=0x%04X sub=0x%02X data=%02X %02X %02X %02X\n",
                                index,
                                subIndex,
                                static_cast<unsigned>(f.header.id),
                                f.data[0],
                                respIndex,
                                f.data[3],
                                f.data[4],
                                f.data[5],
                                f.data[6],
                                f.data[7]);
                }
                cacheSdoResponse(f);
            }
        }
    }

    return false;
}
// ==============================
// 发送接口
// ==============================
bool JointControl::sendControl() {
    if (!m_network)
        return false;

    auto pkt = encodeBatch({makeControlFrame()});
    return m_network->send(pkt.data(), pkt.size());
}

bool JointControl::writeSdoU8(uint16_t index, uint8_t subIndex, uint8_t value, int timeoutMs) {
    if (!m_network) {
        return false;
    }

    auto pkt = encodeBatch({makeWriteU8Frame(index, subIndex, value)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }

    return receiveSdoAck(index, subIndex, timeoutMs);
}

bool JointControl::readSdoU8(uint16_t index, uint8_t subIndex, uint8_t &value, int timeoutMs) {
    if (!m_network) {
        return false;
    }

    auto pkt = encodeBatch({makeReadU8Frame(index, subIndex)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }

    auto startTime = std::chrono::steady_clock::now();
    const uint32_t expectedRespCanId = id_.can_id + 0x580;
    while (true) {
        auto now = std::chrono::steady_clock::now();
        int elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs) {
            break;
        }

        std::vector<uint8_t> resp;
        int n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM)) {
            continue;
        }

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd != 0xA0) {
            continue;
        }

        uint16_t data_len = le16toh(rHdr->data_len);
        if (data_len == 0 || n < (int)(sizeof(DataHeaderMM) + data_len)) {
            continue;
        }

        uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
        for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
            uint8_t L = dlcToLen(f.header.dlc);
            if (f.header.can_ch != id_.can_ch || f.header.id != expectedRespCanId || L < 5) {
                continue;
            }

            uint16_t respIndex = static_cast<uint16_t>(f.data[1]) |
                                 (static_cast<uint16_t>(f.data[2]) << 8);
            if (respIndex != index || f.data[3] != subIndex) {
                continue;
            }

            if (f.data[0] == 0x4F) {
                value = f.data[4];
                return true;
            }
        }
    }

    return false;
}

bool JointControl::readSdoU16(uint16_t index, uint8_t subIndex, uint16_t &value, int timeoutMs) {
    if (!m_network) {
        return false;
    }

    auto pkt = encodeBatch({makeReadU8Frame(index, subIndex)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }

    auto startTime = std::chrono::steady_clock::now();
    const uint32_t expectedRespCanId = id_.can_id + 0x580;
    while (true) {
        auto now = std::chrono::steady_clock::now();
        int elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs) {
            break;
        }

        std::vector<uint8_t> resp;
        int n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM)) {
            continue;
        }

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd != 0xA0) {
            continue;
        }

        uint16_t data_len = le16toh(rHdr->data_len);
        if (data_len == 0 || n < (int)(sizeof(DataHeaderMM) + data_len)) {
            continue;
        }

        uint8_t *pBuffer = resp.data() + sizeof(DataHeaderMM);
        for (const auto &f : decodeCanFrames(pBuffer, data_len)) {
            uint8_t L = dlcToLen(f.header.dlc);
            if (f.header.can_ch == id_.can_ch && f.header.id == id_.can_id + 0x190 && L >= 16) {
                decodeFeedback(f);
                continue;
            }

            if (f.header.can_ch != id_.can_ch || f.header.id != expectedRespCanId || L < 6) {
                continue;
            }

            uint16_t respIndex = static_cast<uint16_t>(f.data[1]) |
                                 (static_cast<uint16_t>(f.data[2]) << 8);
            if (respIndex != index || f.data[3] != subIndex) {
                continue;
            }

            if (f.data[0] == 0x4B) {
                value = static_cast<uint16_t>(f.data[4]) |
                        (static_cast<uint16_t>(f.data[5]) << 8);
                return true;
            }
        }
    }

    return false;
}

bool JointControl::changeNodeId(uint8_t newCanId, int timeoutMs) {
    if (!m_network) {
        return false;
    }

    const uint32_t oldCanId = id_.can_id;
    std::printf("[NodeIdChange] write old=%u new=%u\n",
                static_cast<unsigned>(oldCanId),
                static_cast<unsigned>(newCanId));
    auto pkt = encodeBatch({makeWriteU8Frame(0x2040, 0x00, newCanId)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        std::printf("[NodeIdChange] send failed old=%u new=%u\n",
                    static_cast<unsigned>(oldCanId),
                    static_cast<unsigned>(newCanId));
        return false;
    }

    const uint32_t oldRespCanId = oldCanId + 0x580;
    const uint32_t newRespCanId = static_cast<uint32_t>(newCanId) + 0x580;
    const bool ackReceived = receiveSdoAck(0x2040, 0x00, timeoutMs, oldRespCanId, newRespCanId);
    if (!ackReceived) {
        std::printf("[NodeIdChange] write ack timeout old=%u new=%u\n",
                    static_cast<unsigned>(oldCanId),
                    static_cast<unsigned>(newCanId));
        id_.can_id = oldCanId;
        return false;
    }
    std::printf("[NodeIdChange] write ack ok old=%u new=%u\n",
                static_cast<unsigned>(oldCanId),
                static_cast<unsigned>(newCanId));

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    id_.can_id = newCanId;
    uint8_t readBackId = 0;
    if (readSdoU8(0x2040, 0x00, readBackId, timeoutMs)) {
        std::printf("[NodeIdChange] verify new-id read value=%u expected=%u\n",
                    static_cast<unsigned>(readBackId),
                    static_cast<unsigned>(newCanId));
        if (readBackId == newCanId) {
            return true;
        }
    } else {
        std::printf("[NodeIdChange] verify new-id read timeout id=%u\n",
                    static_cast<unsigned>(newCanId));
    }

    id_.can_id = oldCanId;
    uint8_t oldReadBackId = 0;
    if (readSdoU8(0x2040, 0x00, oldReadBackId, timeoutMs)) {
        std::printf("[NodeIdChange] old-id read value=%u old_id=%u\n",
                    static_cast<unsigned>(oldReadBackId),
                    static_cast<unsigned>(oldCanId));
        if (oldReadBackId == newCanId) {
            std::printf("[NodeIdChange] value stored, runtime id still old old=%u new=%u\n",
                        static_cast<unsigned>(oldCanId),
                        static_cast<unsigned>(newCanId));
            id_.can_id = oldCanId;
            return true;
        }
    } else {
        std::printf("[NodeIdChange] old-id read timeout old_id=%u\n",
                    static_cast<unsigned>(oldCanId));
    }

    id_.can_id = oldCanId;
    return false;
}

bool JointControl::enable(bool on) {
    if (!m_network)
        return false;

    auto modePkt = encodeBatch({makeModeFrame(3)});
    if (!m_network->send(modePkt.data(), modePkt.size())) {
        return false;
    }

    auto enablePkt = encodeBatch({makeEnableFrame(on)});
    return m_network->send(enablePkt.data(), enablePkt.size());
}

void JointControl::setControlMode(uint8_t mode) {
    if (!m_network)
        return;
    auto pkt = encodeBatch({makeModeFrame(mode)});
    m_network->send(pkt.data(), pkt.size());
}

bool JointControl::setControlModeAndWait(uint8_t mode, int timeoutMs) {
    if (!m_network) {
        return false;
    }
    auto pkt = encodeBatch({makeModeFrame(mode)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }
    return receiveSdoAck(0x2003, 0x00, timeoutMs);
}

bool JointControl::enableAndWait(bool on, int timeoutMs) {
    if (!m_network) {
        return false;
    }
    auto pkt = encodeBatch({makeEnableFrame(on)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }
    return receiveSdoAck(0x2002, 0x00, timeoutMs);
}

bool JointControl::clearErrorAndWait(int timeoutMs) {
    if (!m_network) {
        return false;
    }

    while (takeCachedSdoAck(0x2002, 0x00)) {
    }
    drainPendingPackets(32, 0);
    while (takeCachedSdoAck(0x2002, 0x00)) {
    }

    auto pkt = encodeBatch({makeControlWordFrame(0x00FF)});
    if (!m_network->send(pkt.data(), pkt.size())) {
        return false;
    }
    return receiveSdoAck(0x2002, 0x00, timeoutMs);
}

void JointControl::setControlPdoBase(uint32_t base) {
    control_pdo_base_.store(base);
}

uint32_t JointControl::controlPdoBase() const {
    return control_pdo_base_.load();
}

bool JointControl::sendPacket(const Net2CanFrameWrapper &f) {
    if (!m_network) {
        return false;
    }
    auto pkt = encodeBatch({f});
    return m_network->send(pkt.data(), pkt.size());
}
