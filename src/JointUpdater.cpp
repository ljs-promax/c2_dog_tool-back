#include "JointUpdater.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

// 添加这个函数定义
static void printProgress(int percentage) {
    std::cout << "\r[Joint] Progress: [" << percentage << "%] " << std::flush;
    if (percentage >= 100)
        std::cout << std::endl;
}

// 内部静态辅助：计算/校验 CRC
static bool validateCrc(const uint8_t *buf, size_t totalLen) {
    if (totalLen < sizeof(DataHeaderMM))
        return false;
    const DataHeaderMM *dh          = (const DataHeaderMM *)buf;
    uint16_t            expectedCrc = le16toh(dh->crc_sum);
    uint16_t            actualCrc   = 0;
    for (size_t i = sizeof(DataHeaderMM); i < totalLen; ++i) {
        actualCrc += buf[ i ];
    }
    return actualCrc == expectedCrc;
}

namespace {
constexpr size_t   kSnTotalLen     = 26;
constexpr uint16_t kSnStartIndex   = 0x2004;
constexpr uint16_t kSnEndIndex     = 0x200A;
constexpr size_t   kSnSegmentCount = 7;
constexpr size_t   kSnRawLen       = kSnSegmentCount * 4;
constexpr uint16_t kVersionIndex   = 0x2100;

static void trimSnPadding(std::string &sn) {
    if (sn.size() >= 2 && static_cast<uint8_t>(sn[ sn.size() - 2 ]) == 0 &&
        static_cast<uint8_t>(sn[ sn.size() - 1 ]) == 0) {
        sn.resize(sn.size() - 2);
    }
}
} // namespace

// 构造函数：增加 StatusReporter 引用
JointUpdater::JointUpdater(const std::string &ip, int port, int nodeID, int channel,
                           StatusReporter &reporter)
    : m_nodeID(nodeID)
    , m_channel(channel)
    , m_frameCounter(0)
    , m_reporter(reporter) {
    // 本地端口设为 8086，避免与 MCU 升级冲突
    m_network = std::make_unique<UdpClient>(ip, port, 8086);

    // 生成云端要求的设备名称：joint_chX_idY
    m_deviceName = "joint_ch" + std::to_string(channel) + "_id" + std::to_string(nodeID);
}

bool JointUpdater::init() {
    return m_network->init();
}

void JointUpdater::setProgressCallback(std::function<void(int, const std::string &)> cb) {
    m_progressCallback = std::move(cb);
}

/**
 * 查看版本号
 */
std::string JointUpdater::getVersion() {
    uint32_t reqId  = 0x600 + (uint32_t)m_nodeID;
    uint32_t respId = 0x580 + (uint32_t)m_nodeID;

    std::vector<uint8_t> stale;
    for (int i = 0; i < 64 && m_network->receive(stale, 0) > 0; ++i) {
    }

    std::vector<uint8_t> queryCmd = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (!packAndSendCan(reqId, queryCmd)) {
        m_reporter.sendStatus(m_deviceName, "send_failed", -2);
        return "Error: Send Failed";
    }

    auto startTime = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 startTime)
               .count() < 1500) {

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 100);
        if (n < (int)(sizeof(DataHeaderMM)))
            continue;

        DataHeaderMM *dh = (DataHeaderMM *)resp.data();
        if (dh->cmd == 0xE1)
            continue;
        printf("Received %d  cmd %x %x\n", n, dh->cmd, dh->header);
        if (!validateCrc(resp.data(), n))
            continue;
        if (dh->cmd != Cmd::JOINT_CAN_TRANS)
            continue;
        if (n < (int)(sizeof(DataHeaderMM) + sizeof(CANHeaderPacked)))
            continue;

        CANHeaderPacked *ch            = (CANHeaderPacked *)(resp.data() + sizeof(DataHeaderMM));
        uint32_t         receivedCanId = le32toh(ch->id_word) & 0x1FFFFFFF;
        if (ch->can_ch != static_cast<uint8_t>(m_channel))
            continue;

        if (receivedCanId == respId) {
            uint8_t *p          = resp.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked);
            size_t   payloadLen = n - (sizeof(DataHeaderMM) + sizeof(CANHeaderPacked));

            if (payloadLen >= 8) {
                uint16_t respIndex = static_cast<uint16_t>(p[ 1 ]) |
                                     (static_cast<uint16_t>(p[ 2 ]) << 8);
                if (respIndex != kVersionIndex) {
                    continue;
                }
                if (p[ 0 ] == 0x80) {
                    uint32_t abortCode = static_cast<uint32_t>(p[ 4 ]) |
                                         (static_cast<uint32_t>(p[ 5 ]) << 8) |
                                         (static_cast<uint32_t>(p[ 6 ]) << 16) |
                                         (static_cast<uint32_t>(p[ 7 ]) << 24);
                    printf("[Version] abort index=0x%04X code=0x%08X\n", respIndex, abortCode);
                    continue;
                }
                if (p[ 0 ] != 0x4B && p[ 0 ] != 0x43) {
                    continue;
                }
                uint32_t v = (uint32_t)p[ 4 ] | ((uint32_t)p[ 5 ] << 8) | ((uint32_t)p[ 6 ] << 16) |
                             ((uint32_t)p[ 7 ] << 24);
                std::stringstream ss;
                ss << (v / 100) << "." << ((v / 10) % 10) << "." << (v % 10);
                std::string verStr = ss.str();

                // --- 上报：版本查询成功 ---
                m_reporter.sendStatus(m_deviceName, verStr, -1);
                return verStr;
            }
        }
    }

    // --- 上报：查询超时 ---
    m_reporter.sendStatus(m_deviceName, "unknown", -2);
    return "Error: Timeout";
}

// 读取单个 SDO 对象，解析出 4 字符（回包 0x4B，数据在 p[4..7]）
bool JointUpdater::readSnField(uint32_t reqId, uint32_t respId, uint16_t index,
                               std::string &out4chars) {
    uint8_t              idxLo    = static_cast<uint8_t>(index & 0xFF);
    uint8_t              idxHi    = static_cast<uint8_t>((index >> 8) & 0xFF);
    std::vector<uint8_t> queryCmd = {0x40, idxLo, idxHi, 0x00, 0x00, 0x00, 0x00, 0x00};
    // printf("[SN] send index=0x%04X, reqCanId=0x%X, cmd=",
    //        index, reqId);
    // for (uint8_t b : queryCmd)
    //   printf("%02X ", b);
    // printf("\n");
    if (!packAndSendCan(reqId, queryCmd))
        return false;
    auto startTime = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 startTime)
               .count() < 1500) {
        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 100);
        if (n < (int)(sizeof(DataHeaderMM) + sizeof(CANHeaderPacked) + 8))
            continue;
        DataHeaderMM *dh = (DataHeaderMM *)resp.data();
        if (dh->cmd == 0xE1)
            continue;
        if (!validateCrc(resp.data(), n))
            continue;
        if (dh->cmd != Cmd::JOINT_CAN_TRANS)
            continue;
        CANHeaderPacked *ch            = (CANHeaderPacked *)(resp.data() + sizeof(DataHeaderMM));
        uint32_t         receivedCanId = le32toh(ch->id_word) & 0x1FFFFFFF;
        if (ch->can_ch != static_cast<uint8_t>(m_channel)) {
            printf("[SN] skip: can_ch=%u != expect=%d\n", ch->can_ch, m_channel);
            continue;
        }
        if (receivedCanId != respId)
            continue;
        uint8_t *p          = resp.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked);
        size_t   payloadLen = n - (sizeof(DataHeaderMM) + sizeof(CANHeaderPacked));
        // 打印整包原始数据（hex）
        // printf("[SN] recv index=0x%04X, respCanId=0x%X, totalLen=%d, raw=",
        //        index, receivedCanId, n);
        // for (int i = 0; i < n; ++i)
        //   printf("%02X ", resp[i]);
        // printf("\n");
        // 打印 SDO 载荷 8 字节
        // printf("[SN] recv index=0x%04X, sdo payload(%zu)=", index, payloadLen);
        // for (size_t i = 0; i < payloadLen && i < 8; ++i)
        //     printf("%02X ", p[ i ]);
        // printf("\n");
        uint16_t respIndex = static_cast<uint16_t>(p[ 1 ]) | (static_cast<uint16_t>(p[ 2 ]) << 8);
        if (respIndex != index) {
            printf("[SN] skip: respIndex=0x%04X != expect=0x%04X\n", respIndex, index);
            continue;
        }
        if (p[ 0 ] == 0x80) {
            uint32_t abortCode = static_cast<uint32_t>(p[ 4 ]) |
                                 (static_cast<uint32_t>(p[ 5 ]) << 8) |
                                 (static_cast<uint32_t>(p[ 6 ]) << 16) |
                                 (static_cast<uint32_t>(p[ 7 ]) << 24);
            printf("[SN] abort index=0x%04X code=0x%08X\n", index, abortCode);
            continue;
        }
        if (p[ 0 ] != 0x4B && p[ 0 ] != 0x43) {
            printf("[SN] skip: index=0x%04X cs=0x%02X, expect 0x4B/0x43\n", index, p[ 0 ]);
            continue;
        }
        out4chars.assign(reinterpret_cast<char *>(&p[ 4 ]), 4);
        // 打印解析结果：hex + 可见字符
        // printf("[SN] parsed index=0x%04X, data_hex=", index);
        // for (size_t i = 0; i < 4; ++i)
        //     printf("%02X ", static_cast<uint8_t>(out4chars[ i ]));
        // printf(", data_str=[");
        // for (size_t i = 0; i < 4; ++i) {
        //     char c = out4chars[ i ];
        //     if (c >= 0x20 && c <= 0x7E)
        //         printf("%c", c);
        //     else
        //         printf("\\x%02X", static_cast<uint8_t>(c));
        // }
        // printf("]\n");
        return true;
    }
    printf("[SN] timeout index=0x%04X\n", index);
    return false;
}
// /**
//  * 查看sn
//  */
// std::string JointUpdater::getSn() {
//   uint32_t reqId = 0x600 + static_cast<uint32_t>(m_nodeID);
//   uint32_t respId = 0x580 + static_cast<uint32_t>(m_nodeID);
//   printf("[SN] start read, ch=%d, nodeID=%d, reqId=0x%X, respId=0x%X\n",
//          m_channel, m_nodeID, reqId, respId);
//   std::string sn;
//   sn.reserve(28);
//   for (uint16_t index = 0x2004; index <= 0x200A; ++index) {
//     std::string part;
//     if (!readSnField(reqId, respId, index, part)) {
//       printf("[SN] failed at index=0x%04X\n", index);
//       m_reporter.sendStatus(m_deviceName, "read_failed", -2);
//       return "Error: Timeout at 0x" + std::to_string(index);
//     }
//     printf("[SN] part[%u]: [%s]\n", index, part.c_str());
//     sn += part;
//   }
//   printf("[SN] final SN(%zu chars): [%s]\n", sn.size(), sn.c_str());
//   std::cout << "[Joint] SN: " << sn << std::endl;
//   m_reporter.sendStatus(m_deviceName, sn, -1);
//   return sn;
// }

// bool JointUpdater::writeSnField(uint32_t reqId, uint32_t respId, uint16_t index, const
// std::string &in4chars) {
//   if(in4chars.size() != 4)
//     return false;
//   uint8_t idxLo = static_cast<uint8_t>(index & 0xFF);
//   uint8_t idxHi = static_cast<uint8_t>((index >> 8) & 0xFF);

//   std::vector<uint8_t> writeCmd = {
//     0x23, idxLo, idxHi, 0x00,
//     static_cast<uint8_t>(in4chars[0]),
//     static_cast<uint8_t>(in4chars[1]),
//     static_cast<uint8_t>(in4chars[2]),
//     static_cast<uint8_t>(in4chars[3])};

//   if (!packAndSendCan(reqId, writeCmd))
//     return false;

//   auto startTime = std::chrono::steady_clock::now();
//   while (std::chrono::duration_cast<std::chrono::milliseconds>(
//              std::chrono::steady_clock::now() - startTime)
//              .count() < 1500) {
//     std::vector<uint8_t> resp;
//     int n = m_network->receive(resp, 100);
//     if (n < (int)(sizeof(DataHeaderMM) + sizeof(CANHeaderPacked) + 8))
//       continue;

//     uint8_t *p = resp.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked);

//     uint16_t respIndex = static_cast<uint16_t>(p[1]) | (static_cast<uint16_t>(p[2]) << 8);
//     if(respIndex != index)
//       continue;
//     if(p[0] == 0x60)
//       return true;
//       // 部分设备写失败会回 0x80 + abort code
//       if (p[0] == 0x80) {
//         uint32_t abort = (uint32_t)p[4] | ((uint32_t)p[5] << 8) |
//                          ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
//         printf("[SN] write abort index=0x%04X, code=0x%08X\n", index, abort);
//         return false;
//       }
//   }
//   return false;
// }

// bool JointUpdater::setSn(const std::string &sn) {
//   if(sn.size() != 24){
//     m_reporter.sendStatus(m_deviceName, "sn_error", -2);
//     return false;
//   }
//   uint32_t reqId = 0x600 + static_cast<uint32_t>(m_nodeID);
//   uint32_t respId = 0x580 + static_cast<uint32_t>(m_nodeID);
//   for(uint16_t i = 0; i <6 ;i++){
//     uint16_t index = 0x2004 + i;
//     std::string part = sn.substr(i * 4, 4);
//     if(!writeSnField(reqId, respId, index, part)){
//       m_reporter.sendStatus(m_deviceName, "write_failed_at_0x" + std::to_string(index), -2);
//       return false;
//     }
//   }

//   // 可选：写完后回读校验
//   std::string readBack = getSn();
//   if (readBack != sn) {
//     m_reporter.sendStatus(m_deviceName, "verify_mismatch", -2);
//     return false;
//   }
//   m_reporter.sendStatus(m_deviceName, sn, -1);
//   return true;
// }

std::string JointUpdater::getSn() {
    uint32_t reqId  = 0x600 + static_cast<uint32_t>(m_nodeID);
    uint32_t respId = 0x580 + static_cast<uint32_t>(m_nodeID);
    printf("[SN] start read, ch=%d, nodeID=%d, reqId=0x%X, respId=0x%X\n", m_channel, m_nodeID,
           reqId, respId);

    std::vector<uint8_t> stale;
    for (int i = 0; i < 64 && m_network->receive(stale, 0) > 0; ++i) {
    }

    std::string sn;
    sn.reserve(kSnRawLen);
    for (uint16_t index = kSnStartIndex; index <= kSnEndIndex; ++index) {
        std::string part;
        if (!readSnField(reqId, respId, index, part)) {
            printf("[SN] failed at index=0x%04X\n", index);
            m_reporter.sendStatus(m_deviceName, "read_failed", -2);
            return "Error: Timeout at 0x" + std::to_string(index);
        }
        // printf("[SN] part[%u]: [%s]\n", index, part.c_str());
        sn += part;
    }

    trimSnPadding(sn);

    printf("[SN] final SN(%zu chars): [%s]\n", sn.size(), sn.c_str());
    std::cout << "[Joint] SN: " << sn << std::endl;
    m_reporter.sendStatus(m_deviceName, sn, -1);
    return sn;
}

bool JointUpdater::writeSnField(uint32_t reqId, uint32_t respId, uint16_t index,
                                const std::string &in4chars) {
    if (in4chars.size() != 4)
        return false;

    uint8_t idxLo = static_cast<uint8_t>(index & 0xFF);
    uint8_t idxHi = static_cast<uint8_t>((index >> 8) & 0xFF);

    std::vector<uint8_t> writeCmd = {0x23,
                                     idxLo,
                                     idxHi,
                                     0x00,
                                     static_cast<uint8_t>(in4chars[ 0 ]),
                                     static_cast<uint8_t>(in4chars[ 1 ]),
                                     static_cast<uint8_t>(in4chars[ 2 ]),
                                     static_cast<uint8_t>(in4chars[ 3 ])};

    // printf("[SN] write index=0x%04X, reqCanId=0x%X, data=", index, reqId);
    // for (uint8_t b : writeCmd)
    //     printf("%02X ", b);
    // printf("\n");

    if (!packAndSendCan(reqId, writeCmd))
        return false;

    auto startTime = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 startTime)
               .count() < 1500) {
        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 100);
        if (n < (int)(sizeof(DataHeaderMM) + sizeof(CANHeaderPacked) + 8))
            continue;

        DataHeaderMM *dh = (DataHeaderMM *)resp.data();
        if (dh->cmd == 0xE1)
            continue;
        if (!validateCrc(resp.data(), n))
            continue;
        if (dh->cmd != Cmd::JOINT_CAN_TRANS)
            continue;

        CANHeaderPacked *ch            = (CANHeaderPacked *)(resp.data() + sizeof(DataHeaderMM));
        uint32_t         receivedCanId = le32toh(ch->id_word) & 0x1FFFFFFF;
        if (ch->can_ch != static_cast<uint8_t>(m_channel)) {
            printf("[SN] write skip: can_ch=%u != expect=%d\n", ch->can_ch, m_channel);
            continue;
        }
        if (receivedCanId != respId)
            continue;

        uint8_t *p         = resp.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked);
        uint16_t respIndex = static_cast<uint16_t>(p[ 1 ]) | (static_cast<uint16_t>(p[ 2 ]) << 8);
        if (respIndex != index)
            continue;

        if (p[ 0 ] == 0x60)
            return true;

        if (p[ 0 ] == 0x80) {
            uint32_t abort = (uint32_t)p[ 4 ] | ((uint32_t)p[ 5 ] << 8) | ((uint32_t)p[ 6 ] << 16) |
                             ((uint32_t)p[ 7 ] << 24);
            printf("[SN] write abort index=0x%04X, code=0x%08X\n", index, abort);
            return false;
        }
    }

    printf("[SN] write timeout index=0x%04X\n", index);
    return false;
}

bool JointUpdater::setSn(const std::string &sn) {
    if (sn.size() != kSnTotalLen) {
        printf("[SN] invalid length: %zu, expect %zu\n", sn.size(), kSnTotalLen);
        m_reporter.sendStatus(m_deviceName, "sn_error", -2);
        return false;
    }

    uint32_t reqId  = 0x600 + static_cast<uint32_t>(m_nodeID);
    uint32_t respId = 0x580 + static_cast<uint32_t>(m_nodeID);

    printf("[SN] start write, ch=%d, nodeID=%d, sn=[%s]\n", m_channel, m_nodeID, sn.c_str());

    for (size_t i = 0; i < kSnSegmentCount; ++i) {
        uint16_t    index = static_cast<uint16_t>(kSnStartIndex + i);
        std::string part;

        if (i < kSnSegmentCount - 1) {
            part = sn.substr(i * 4, 4);
        } else {
            part = sn.substr(24, 2);
            part.push_back('\0');
            part.push_back('\0');
        }

        if (!writeSnField(reqId, respId, index, part)) {
            m_reporter.sendStatus(m_deviceName, "write_failed_at_0x" + std::to_string(index), -2);
            return false;
        }
    }

    std::string readBack = getSn();
    if (readBack != sn) {
        printf("[SN] verify mismatch: expect=[%s], read=[%s]\n", sn.c_str(), readBack.c_str());
        m_reporter.sendStatus(m_deviceName, "verify_mismatch", -2);
        return false;
    }

    m_reporter.sendStatus(m_deviceName, sn, -1);
    return true;
}

/**
 * 执行升级逻辑
 */
bool JointUpdater::upgrade(const std::string &filePath) {
    std::ifstream is(filePath, std::ios::binary | std::ios::ate);
    if (!is.is_open()) {
        std::cerr << "Failed to open: " << filePath << std::endl;
        m_reporter.sendStatus(m_deviceName, "file_error", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "file_error");
        }
        return false;
    }

    std::streamsize size = is.tellg();
    if (size <= 0) {
        std::cerr << "Invalid firmware size: " << filePath << std::endl;
        m_reporter.sendStatus(m_deviceName, "file_error", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "file_error");
        }
        return false;
    }
    is.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(size);
    is.read((char *)data.data(), size);

    uint32_t upgradeCanId = 0x680 + m_nodeID;

    // 1. 握手 (Handshake)
    std::cout << "[Joint] Sending Handshake..." << std::endl;
    std::vector<uint8_t> handshake = {0xDD, 0xDD, 0xDD, 0xDD};
    if (!packAndSendCan(upgradeCanId, handshake) || !waitForCanResponse(upgradeCanId, handshake)) {
        std::cerr << "Handshake failed!" << std::endl;
        m_reporter.sendStatus(m_deviceName, "handshake_error", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "handshake_error");
        }
        return false;
    }
    if (m_progressCallback) {
        m_progressCallback(0, "upgrading");
    }

    // 2. 数据传输
    std::cout << "[Joint] Transferring data..." << std::endl;
    for (size_t i = 0; i < data.size(); i += 8) {
        size_t               len = std::min((size_t)8, data.size() - i);
        std::vector<uint8_t> chunk(data.begin() + i, data.begin() + i + len);

        if (chunk.size() < 8)
            chunk.resize(8, 0xFF);

        if (!packAndSendCan(upgradeCanId, chunk) || !waitForCanResponse(upgradeCanId, {})) {
            std::cerr << "\nData transfer error at offset: " << i << std::endl;
            m_reporter.sendStatus(m_deviceName, "transfer_error", -2);
            if (m_progressCallback) {
                m_progressCallback((i * 100) / data.size(), "transfer_error");
            }
            return false;
        }

        int progress = ((i + len) * 100) / data.size();
        if (i % 64 == 0) {
            printProgress(progress);
            // --- 上报：升级进度 ---
            m_reporter.sendStatus(m_deviceName, "upgrading", progress);
            if (m_progressCallback) {
                m_progressCallback(progress, "upgrading");
            }
        }
    }
    printProgress(100);
    if (m_progressCallback) {
        m_progressCallback(100, "finalizing");
    }

    // 3. 结束指令 (Finalization)
    std::cout << "[Joint] Finalizing..." << std::endl;
    std::vector<uint8_t> finalCmd = {0xFF, 0xFF, 0xFF, 0xFF};
    if (!packAndSendCan(upgradeCanId, finalCmd) || !waitForCanResponse(upgradeCanId, finalCmd)) {
        std::cerr << "Finalization failed!" << std::endl;
        m_reporter.sendStatus(m_deviceName, "error_end", -2);
        if (m_progressCallback) {
            m_progressCallback(99, "error_end");
        }
        return false;
    }

    // --- 上报：升级完成 ---
    m_reporter.sendStatus(m_deviceName, "done", 100);
    if (m_progressCallback) {
        m_progressCallback(100, "done");
    }
    return true;
}

/**
 * 封装并发送：DataHeaderMM + CANHeaderPacked + Payload
 */
bool JointUpdater::packAndSendCan(uint32_t canId, const std::vector<uint8_t> &payload) {
    size_t               totalLen = sizeof(DataHeaderMM) + sizeof(CANHeaderPacked) + payload.size();
    std::vector<uint8_t> pkt(totalLen);

    // 1. DataHeaderMM
    DataHeaderMM *dh = (DataHeaderMM *)pkt.data();
    dh->header       = htole16(Magic::BH_JOINT);
    dh->frame_id     = m_frameCounter++;
    dh->version      = 1;
    dh->cmd          = Cmd::JOINT_CAN_TRANS;
    dh->packet_num   = 1;
    dh->data_len     = htole16(sizeof(CANHeaderPacked) + payload.size());

    // 2. CANHeaderPacked (已修正：使用 pkt.data() 而不是 resp.data())
    CANHeaderPacked *ch = (CANHeaderPacked *)(pkt.data() + sizeof(DataHeaderMM));
    ch->board_id        = DefaultConfig::GATEWAY_BOARD_ID;
    ch->can_ch          = (uint8_t)m_channel;
    ch->id_word         = htole32(canId & 0x1FFFFFFF);
    ch->props_word      = htole32(bytesToDlc(payload.size()) & 0x0F);

    // 3. Payload
    if (!payload.empty()) {
        memcpy(pkt.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked), payload.data(),
               payload.size());
    }

    // 4. CRC 计算
    uint16_t crc = 0;
    for (size_t i = sizeof(DataHeaderMM); i < pkt.size(); ++i) {
        crc += pkt[ i ];
    }
    dh->crc_sum = htole16(crc);

    return m_network->send(pkt.data(), pkt.size());
}

/**
 * 等待响应并校验
 */
bool JointUpdater::waitForCanResponse(uint32_t                    expectedCanId,
                                      const std::vector<uint8_t> &expectedPayload) {
    auto      startTime  = std::chrono::steady_clock::now();
    const int kTimeoutMs = 3000;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() >=
            kTimeoutMs)
            break;
        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 200);

        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *dh = (DataHeaderMM *)resp.data();
        if (dh->cmd == 0xE1)
            continue;

        if (!validateCrc(resp.data(), n))
            continue;
        if (n < (int)(sizeof(DataHeaderMM) + sizeof(CANHeaderPacked)))
            continue;
        CANHeaderPacked *ch            = (CANHeaderPacked *)(resp.data() + sizeof(DataHeaderMM));
        uint32_t         receivedCanId = le32toh(ch->id_word) & 0x1FFFFFFF;
        if (ch->can_ch != static_cast<uint8_t>(m_channel))
            continue;
        if (receivedCanId != expectedCanId)
            continue;
        if (!expectedPayload.empty()) {
            uint8_t *p          = resp.data() + sizeof(DataHeaderMM) + sizeof(CANHeaderPacked);
            size_t   payloadLen = n - (sizeof(DataHeaderMM) + sizeof(CANHeaderPacked));
            if (payloadLen < expectedPayload.size())
                continue;
            if (memcmp(p, expectedPayload.data(), expectedPayload.size()) != 0)
                continue;
        }
        return true;
    }
    return false;
}

uint8_t JointUpdater::bytesToDlc(size_t bytes) {
    if (bytes <= 8)
        return (uint8_t)bytes;
    return 8;
}
