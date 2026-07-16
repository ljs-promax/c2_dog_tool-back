#include "McuUpdater.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <utility>

// 添加这个函数定义
static void printProgress(int percentage) {
    std::cout << "\r[MCU] Progress: [" << percentage << "%] " << std::flush;
    if (percentage >= 100)
        std::cout << std::endl;
}

// 构造函数：接收 StatusReporter 引用
McuUpdater::McuUpdater(const std::string &ip, int port, StatusReporter &reporter)
    : m_frameCounter(0)
    , m_reporter(reporter) {
    // 为 MCU 升级分配独立的 UdpClient，本地端口 8085
    m_network = std::make_unique<UdpClient>(ip, port, 8085);
}

bool McuUpdater::init() {
    return m_network->init();
}

void McuUpdater::setProgressCallback(std::function<void(int, const std::string &)> cb) {
    m_progressCallback = std::move(cb);
}

McuStatus McuUpdater::getmcuStatus() {
    uint8_t   cmdQuery = 0xF5;
    McuStatus status{};
    status.reserverd = 0xA5;
    // 1. 构造请求包
    std::vector<uint8_t> pkt(sizeof(DataHeaderMM));
    DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    h->header              = htole16(Magic::RD_MCU);
    h->frame_id            = m_frameCounter++;
    h->version             = 1;
    h->cmd                 = cmdQuery;
    h->packet_num          = 1;
    h->data_len            = 0;
    h->crc_sum             = 0;

    m_network->send(pkt.data(), pkt.size());

    // 2. 等待回包
    auto      startTime = std::chrono::steady_clock::now();
    const int timeoutMs = 1500;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs)
            break;

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd == cmdQuery) {
            uint16_t len = le16toh(rHdr->data_len);

            if (len >= sizeof(McuStatus) && n >= (int)(sizeof(DataHeaderMM) + len)) {
                const uint8_t *data = resp.data() + sizeof(DataHeaderMM);
                memcpy(&status, data, sizeof(McuStatus));
                m_reporter.sendRes("status", -1);
                return status;
            }
        }
    }
    m_reporter.sendRes("status", -2);
    return status;
}

McuStatus McuUpdater::getmcuStatus_wait() {
    uint8_t   cmdQuery = 0xF5;
    McuStatus status{};
    status.reserverd = 0xA5;
    // 1. 构造请求包
    // std::vector<uint8_t> pkt(sizeof(DataHeaderMM));
    // DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    // h->header               = htole16(Magic::RD_MCU);
    // h->frame_id            = m_frameCounter++;
    // h->version             = 1;
    // h->cmd                 = 0x11;
    // h->packet_num          = 1;
    // h->data_len            = 0;
    // h->crc_sum             = 0;

    // m_network->send(pkt.data(), pkt.size());

    // 2. 等待回包
    auto      startTime = std::chrono::steady_clock::now();
    const int timeoutMs = 2500;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs)
            break;

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd == cmdQuery) {
            uint16_t len = le16toh(rHdr->data_len);

            if (len >= sizeof(McuStatus) && n >= (int)(sizeof(DataHeaderMM) + len)) {
                const uint8_t *data = resp.data() + sizeof(DataHeaderMM);
                memcpy(&status, data, sizeof(McuStatus));
                m_reporter.sendRes("status", -1);
                return status;
            }
        }
    }
    m_reporter.sendRes("status", -2);
    return status;
}
ImuFeedback McuUpdater::getIMU() {
    uint8_t     cmdQuery = 0xE1;
    ImuFeedback imu{};

    // 1. 构造请求包
    std::vector<uint8_t> pkt(sizeof(DataHeaderMM));
    DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    h->header              = htole16(Magic::RD_MCU);
    h->frame_id            = m_frameCounter++;
    h->version             = 1;
    h->cmd                 = cmdQuery;
    h->packet_num          = 1;
    h->data_len            = 0;
    h->crc_sum             = 0;

    m_network->send(pkt.data(), pkt.size());

    // 2. 等待回包
    auto      startTime = std::chrono::steady_clock::now();
    const int timeoutMs = 1500;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs)
            break;

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();

        if (rHdr->cmd == cmdQuery) {
            uint16_t len = le16toh(rHdr->data_len);

            if (len >= sizeof(ImuFeedback) && n >= (int)(sizeof(DataHeaderMM) + len)) {
                const uint8_t *data = resp.data() + sizeof(DataHeaderMM);
                memcpy(&imu, data, sizeof(ImuFeedback));

                m_reporter.sendRes("imu", -1);
                return imu;
            }
        }
    }
    m_reporter.sendRes("imu", -2);
    return imu;
}

/**
 * 查询 MCU 版本：指令码 0xF4
 */
std::string McuUpdater::getVersion() {
    uint8_t cmdQuery = 0xF4;

    // 1. 构造请求包头
    std::vector<uint8_t> pkt(sizeof(DataHeaderMM));
    DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    h->header              = htole16(Magic::RD_MCU);
    h->frame_id            = m_frameCounter++;
    h->version             = 1;
    h->cmd                 = cmdQuery;
    h->packet_num          = 1;
    h->data_len            = 0;
    h->crc_sum             = 0;

    // 2. 发送请求
    m_network->send(pkt.data(), pkt.size());

    // 3. 等待并解析响应
    auto      startTime = std::chrono::steady_clock::now();
    const int timeoutMs = 1500;
    printf("getVersion------ \n");
    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs)
            break;

        std::vector<uint8_t> resp;
        int                  n    = m_network->receive(resp, 500);
        DataHeaderMM        *rHdr = (DataHeaderMM *)resp.data();

        if (n < (int)sizeof(DataHeaderMM))
            continue;

        if (rHdr->cmd == cmdQuery) {
            uint16_t len = le16toh(rHdr->data_len);

            if (len > 0 && n >= (int)(sizeof(DataHeaderMM) + len)) {
                const char *versionData = (const char *)(resp.data() + sizeof(DataHeaderMM));
                std::string verStr(versionData, len);

                // --- 上报：查询版本成功 ---
                m_reporter.sendStatus("mcu", verStr, -1);

                return verStr;
            }
        }
    }

    // --- 上报：查询版本超时 ---
    m_reporter.sendStatus("mcu", "unknown", -2);
    return "Timeout";
}
BmsFeedback McuUpdater::getBattery() {
    uint8_t     cmdQuery = 0xE2;
    BmsFeedback bms{};

    // 1. 构造请求包
    std::vector<uint8_t> pkt(sizeof(DataHeaderMM));
    DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    h->header              = htole16(Magic::RD_MCU);
    h->frame_id            = m_frameCounter++;
    h->version             = 1;
    h->cmd                 = cmdQuery;
    h->packet_num          = 1;
    h->data_len            = 0;
    h->crc_sum             = 0;

    m_network->send(pkt.data(), pkt.size());

    // 2. 等待回包
    auto      startTime = std::chrono::steady_clock::now();
    const int timeoutMs = 4500;
    while (true) {
        auto now = std::chrono::steady_clock::now();
        int  elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        if (elapsed > timeoutMs)
            break;

        std::vector<uint8_t> resp;
        int                  n = m_network->receive(resp, 500);
        if (n < (int)sizeof(DataHeaderMM))
            continue;

        DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();
        if (rHdr->cmd == cmdQuery) {
            uint16_t len = le16toh(rHdr->data_len);
            if (len >= sizeof(BmsFeedback) && n >= (int)(sizeof(DataHeaderMM) + len)) {
                const uint8_t *data = resp.data() + sizeof(DataHeaderMM);
                memcpy(&bms, data, sizeof(BmsFeedback));
                m_reporter.sendRes("battery", -1);
                return bms;
            }
        }
    }
    printf("bms timeout\n");
    m_reporter.sendRes("battery", -2);
    return bms;
}
/**
 * 执行 MCU 升级流程
 */

bool McuUpdater::upgrade(const std::string &filePath) {
    // 1. 读取固件
    std::ifstream is(filePath, std::ios::binary | std::ios::ate);
    if (!is.is_open()) {
        std::cerr << "Failed to open firmware: " << filePath << std::endl;
        // 文件打不开也属于异常
        m_reporter.sendStatus("mcu", "file_error", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "file_error");
        }
        return false;
    }
    std::streamsize size = is.tellg();
    if (size <= 0) {
        std::cerr << "Invalid firmware size: " << filePath << std::endl;
        m_reporter.sendStatus("mcu", "file_error", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "file_error");
        }
        return false;
    }
    is.seekg(0, std::ios::beg);
    std::vector<uint8_t> fwData(size);
    is.read((char *)fwData.data(), size);

    uint32_t totalChecksum = 0;
    for (uint8_t b : fwData)
        totalChecksum += (uint8_t)b;

    uint32_t deviceWritePos = 0;
    uint8_t  fid            = 1;

    // -------- Step 1: OTA Start (0xE3) --------
    std::cout << "[MCU] Sending OTA Start (Erase Flash)..." << std::endl;
    uint32_t             fileSize32 = htole32((uint32_t)size);
    std::vector<uint8_t> startPayload(4);
    memcpy(startPayload.data(), &fileSize32, 4);

    if (!sendAndWaitAck(Cmd::MCU_OTA_START, startPayload, fid++, deviceWritePos, 30000)) {
        std::cerr << "OTA Start failed." << std::endl;
        m_reporter.sendStatus("mcu", "error_start", -2);
        if (m_progressCallback) {
            m_progressCallback(0, "error_start");
        }
        return false;
    }
    m_reporter.sendStatus("mcu", "upgrading", 0);
    if (m_progressCallback) {
        m_progressCallback(0, "upgrading");
    }

    // -------- Step 2: OTA Data (0xE4) --------
    std::cout << "[MCU] Transferring Data..." << std::endl;
    size_t       offset    = 0;
    const size_t chunkSize = 512;

    while (offset < (size_t)size) {
        size_t               len = std::min(chunkSize, (size_t)size - offset);
        std::vector<uint8_t> chunk(fwData.begin() + offset, fwData.begin() + offset + len);

        uint32_t currentPos = 0;
        if (!sendAndWaitAck(Cmd::MCU_OTA_DATA, chunk, fid++, currentPos, 5000)) {
            std::cerr << "\nData transfer failed at offset " << offset << std::endl;
            m_reporter.sendStatus("mcu", "error_data", -2);
            if (m_progressCallback) {
                m_progressCallback((offset * 100) / size, "error_data");
            }
            return false;
        }

        if (currentPos != (uint32_t)(offset + len)) {
            std::cerr << "\nAddress Mismatch!" << std::endl;
            m_reporter.sendStatus("mcu", "error_address", -2);
            if (m_progressCallback) {
                m_progressCallback((offset * 100) / size, "error_address");
            }
            return false;
        }

        offset       = currentPos;
        int progress = (offset * 100) / size;
        printProgress(progress);

        // --- 上报：升级进度 ---
        m_reporter.sendStatus("mcu", "upgrading", progress);
        if (m_progressCallback) {
            m_progressCallback(progress, "upgrading");
        }
    }
    std::cout << std::endl;

    // -------- Step 3: OTA End (0xE5) --------
    std::cout << "[MCU] Finalizing (Checksum Verify)..." << std::endl;
    uint32_t             check32 = htole32(totalChecksum);
    std::vector<uint8_t> endPayload(4);
    memcpy(endPayload.data(), &check32, 4);

    if (!sendAndWaitAck(Cmd::MCU_OTA_END, endPayload, fid++, deviceWritePos, 10000)) {
        std::cerr << "OTA End failed." << std::endl;
        m_reporter.sendStatus("mcu", "error_end", -2);
        if (m_progressCallback) {
            m_progressCallback(99, "error_end");
        }
        return false;
    }

    // --- 上报：最终成功 ---
    std::cout << "[MCU] Upgrade Success!" << std::endl;
    m_reporter.sendStatus("mcu", "done", 100);
    if (m_progressCallback) {
        m_progressCallback(100, "done");
    }

    return true;
}

/**
 * 执行 MCU 升级流程
 */
/*bool McuUpdater::upgrade(const std::string &filePath) {
  // 1. 读取固件
  std::ifstream is(filePath, std::ios::binary | std::ios::ate);
  if (!is.is_open()) {
    std::cerr << "Failed to open firmware: " << filePath << std::endl;
    // 文件打不开也属于异常
    m_reporter.sendStatus("mcu", "file_error", -2);
    return false;
  }

  std::streamsize originalSize = is.tellg();
  if (originalSize < 4) {
    std::cerr << "Firmware file too small: " << filePath << std::endl;
    m_reporter.sendStatus("mcu", "file_error", -2);
    return false;
  }

  std::streamsize dataSize = originalSize - 4;
  is.seekg(0, std::ios::beg);
  std::vector<uint8_t> fwData(dataSize);
  is.read((char *)fwData.data(), dataSize);

  uint32_t originalChecksum;
  is.read((char *)&originalChecksum, 4);
  // originalChecksum = le32toh(originalChecksum); // 转换为小端格式
  std::cerr << "[DEBUG] Read CRC32: 0x" << std::hex << originalChecksum
            << std::dec << ", File size: " << originalSize
            << " (data: " << dataSize << ")" << std::endl;

  // 计算原始数据的校验和（用于验证）
  uint32_t calculatedChecksum = 0;
  for (uint8_t b : fwData)
    calculatedChecksum += (uint8_t)b;

  uint32_t deviceWritePos = 0;
  uint8_t fid = 1;

  // -------- Step 1: OTA Start (0xE3) --------
  std::cout << "[MCU] Sending OTA Start (Erase Flash)..." << std::endl;
  uint32_t dataFileSize32 = htole32((uint32_t)dataSize);
  std::vector<uint8_t> startPayload(4);
  memcpy(startPayload.data(), &dataFileSize32, 4);

  if (!sendAndWaitAck(Cmd::MCU_OTA_START, startPayload, fid++, deviceWritePos,
                      30000)) {
    std::cerr << "OTA Start failed." << std::endl;
    m_reporter.sendStatus("mcu", "error_start", -2);
    return false;
  }
  m_reporter.sendStatus("mcu", "upgrading", 0);

  // -------- Step 2: OTA Data (0xE4) --------
  std::cout << "[MCU] Transferring Data..." << std::endl;
  size_t offset = 0;
  const size_t chunkSize = 512;

  while (offset < (size_t)dataSize) {
    size_t len = std::min(chunkSize, (size_t)dataSize - offset);
    std::vector<uint8_t> chunk(fwData.begin() + offset,
                               fwData.begin() + offset + len);

    uint32_t currentPos = 0;
    if (!sendAndWaitAck(Cmd::MCU_OTA_DATA, chunk, fid++, currentPos, 5000)) {
      std::cerr << "\nData transfer failed at offset " << offset << std::endl;
      m_reporter.sendStatus("mcu", "error_data", -2);
      return false;
    }

    if (currentPos != (uint32_t)(offset + len)) {
      std::cerr << "\nAddress Mismatch!" << std::endl;
      m_reporter.sendStatus("mcu", "error_address", -2);
      return false;
    }

    offset = currentPos;
    int progress = (offset * 100) / dataSize;
    printProgress(progress);

    // --- 上报：升级进度 ---
    m_reporter.sendStatus("mcu", "upgrading", progress);
  }
  std::cout << std::endl;

  // -------- Step 3: OTA End (0xE5) --------
  std::cout << "[MCU] Finalizing (Checksum Verify)..." << std::endl;
  uint32_t checksum32 = htole32(originalChecksum);
  std::vector<uint8_t> checksumPayload(4);
  memcpy(checksumPayload.data(), &checksum32, 4);
  uint32_t checksumWritePos = 0;
  if (!sendAndWaitAck(Cmd::MCU_OTA_END, checksumPayload, fid++,
                      checksumWritePos, 10000)) {
    std::cerr << "OTA End failed." << std::endl;
    m_reporter.sendStatus("mcu", "error_end", -2);
    return false;
  }

  // --- 上报：最终成功 ---
  std::cout << "[MCU] Upgrade Success!" << std::endl;
  m_reporter.sendStatus("mcu", "done", 100);

  return true;
}
*/
/**
 * 通用 ACK 接收逻辑
 */
bool McuUpdater::sendAndWaitAck(uint8_t cmd, const std::vector<uint8_t> &payload, uint8_t frameId,
                                uint32_t &out_write_pos, int timeout_ms) {
    std::vector<uint8_t> pkt(sizeof(DataHeaderMM) + payload.size());
    DataHeaderMM        *h = (DataHeaderMM *)pkt.data();
    h->header              = htole16(Magic::RD_MCU);
    h->frame_id            = frameId;
    h->version             = 1;
    h->cmd                 = cmd;
    h->packet_num          = 1;
    h->data_len            = htole16((uint16_t)payload.size());

    uint16_t crc = 0;
    for (uint8_t b : payload)
        crc += b;
    h->crc_sum = htole16(crc);

    if (!payload.empty()) {
        memcpy(pkt.data() + sizeof(DataHeaderMM), payload.data(), payload.size());
    }

    for (int retry = 0; retry < 3; ++retry) {
        m_network->send(pkt.data(), pkt.size());

        auto startTime = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            int  elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
            if (elapsed >= timeout_ms)
                break;

            std::vector<uint8_t> resp;
            int                  n = m_network->receive(resp, 500);

            if (n < (int)(sizeof(DataHeaderMM) + 5))
                continue;

            DataHeaderMM *rHdr = (DataHeaderMM *)resp.data();

            if (rHdr->cmd != cmd) {
                continue;
            }

            uint8_t result = resp[ sizeof(DataHeaderMM) ];
            if (result != 0x00) {
                std::cerr << "Device Error Code: 0x" << std::hex << (int)result << std::dec
                          << std::endl;
                return false;
            }

            uint32_t devPos = 0;
            memcpy(&devPos, resp.data() + sizeof(DataHeaderMM) + 1, 4);
            out_write_pos = le32toh(devPos);

            return true;
        }
    }

    return false;
}
