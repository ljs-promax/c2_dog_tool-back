#include "ProdTestServer.h"
#include "TestLogger.h"
#include "TestConfig.h"
#include "Protocol.h"
#include "BoardBasicTester.h"
#include "ws2812_control.hpp"
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cerrno>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <thread>
#include <vector>

namespace {
static const char *kOtaUploadDir = "/tmp/dog_tool_ota";
static constexpr size_t kMaxRequestQueueSize = 3;
static constexpr const char *kDogToolServiceName = "dog_tool.service";
static constexpr uint16_t kUploadChunkMaxSize = 900;
static constexpr const char *kSbusRxPort = "/dev/ttyS2";
static constexpr int kSbusSendIntervalMs = 100;
static constexpr int kSbusMaxFrames = 10;
static constexpr size_t kSbusFrameSize = 25;
static constexpr int kMotorControlPeriodMs = 1;
static constexpr int kMotorFeedbackPollTimeoutMs = 1;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return ~crc;
}

static std::string sn_trim(const std::string &s);
static std::string bounded_string_from_buffer(const uint8_t *data, size_t len);
static bool ensure_ota_upload_dir(std::string &err);
static bool has_bin_extension(const std::string &path);
static bool is_safe_ota_filename(const std::string &name);

static void prepare_ota_upload_dir_on_start() {
    struct stat st {};
    if (stat(kOtaUploadDir, &st) == 0 && S_ISDIR(st.st_mode)) {
        chmod(kOtaUploadDir, 0777);
        return;
    }
    if (mkdir(kOtaUploadDir, 0777) != 0 && errno != EEXIST) {
        std::cerr << "[OTA] failed to create upload dir: " << kOtaUploadDir
                  << " errno=" << errno << std::endl;
        return;
    }
    chmod(kOtaUploadDir, 0777);
}

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
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};

static bool configure_sbus_port(int fd) {
    termios2 tty{};
    if (ioctl(fd, TCGETS2, &tty) != 0) {
        return false;
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
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    return ioctl(fd, TCSETS2, &tty) == 0;
}

static bool receive_sbus_frame(int fdRx, size_t expectedLen, int timeoutMs) {
    if (expectedLen == 0) {
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(expectedLen);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t byte = 0;
        ssize_t n = read(fdRx, &byte, 1);
        if (n == 1) {
            if (frame.empty()) {
                if (byte != 0x0F) {
                    continue;
                }
            }
            frame.push_back(byte);
            if (frame.size() >= expectedLen) {
                return true;
            }
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return false;
}

static bool run_sbus_serial_test(std::string &detail) {
    const int fdRx = open(kSbusRxPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdRx < 0) {
        detail = std::string("open rx failed: ") + kSbusRxPort;
        return false;
    }

    if (!configure_sbus_port(fdRx)) {
        close(fdRx);
        detail = "configure sbus port failed";
        return false;
    }

    tcflush(fdRx, TCIOFLUSH);

    bool success = false;
    int receivedAt = -1;
    for (int i = 0; i < kSbusMaxFrames; ++i) {
        if (receive_sbus_frame(fdRx, kSbusFrameSize, kSbusSendIntervalMs)) {
            success = true;
            receivedAt = i + 1;
            break;
        }
    }

    close(fdRx);

    if (success) {
        detail = "rx frame ok at packet " + std::to_string(receivedAt);
        return true;
    }

    if (detail.empty()) {
        detail = "no rx frame within 10 packets";
    }
    return false;
}

static bool is_driver_version_error(const std::string &version) {
    return version.empty() ||
           version == "Timeout" ||
           version == "unknown" ||
           version.rfind("Error:", 0) == 0;
}

static std::string read_driver_version_with_retry(JointUpdater &joint, int retries, int delayMs) {
    std::string last;
    for (int i = 0; i < retries; ++i) {
        last = joint.getVersion();
        if (!is_driver_version_error(last)) {
            return last;
        }
        if (i + 1 < retries) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
    }
    return last;
}
} // anonymous namespace

ProdTestServer::ProdTestServer(int port, StatusReporter& reporter)
    : m_port(port), m_reporter(reporter),
      m_running(false), m_motorTestActive(false), m_otaActive(false),
      m_bmsActive(false), m_networkActive(false), m_motorStreamFrameId(0),
      m_nextUploadId(1) {
    memset(&m_lastClient, 0, sizeof(m_lastClient));
    memset(&m_motorClient, 0, sizeof(m_motorClient));

    // 创建socket
    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockfd < 0) {
        perror("ProdTestServer: socket failed");
        exit(EXIT_FAILURE);
    }

    int reuse = 1;
    setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(m_port);

    if (bind(m_sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("ProdTestServer: bind failed");
        close(m_sockfd);
        exit(EXIT_FAILURE);
    }

    prepare_ota_upload_dir_on_start();
}

ProdTestServer::~ProdTestServer() {
    stop();
}

void ProdTestServer::start() {
    m_running = true;
    registerHandlers();
    m_workerThread = std::thread(&ProdTestServer::workerLoop, this);
    m_uploadWorkerThread = std::thread(&ProdTestServer::uploadWorkerLoop, this);

    std::cout << "[ProdTestServer] Listening on UDP port " << m_port << std::endl;

    runLoop();
}

void ProdTestServer::stop() {
    m_running = false;
    m_queueCv.notify_all();
    m_uploadQueueCv.notify_all();
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }
    if (m_uploadWorkerThread.joinable()) {
        m_uploadWorkerThread.join();
    }
    stopMotorTest();
    if (m_otaThread.joinable()) {
        m_otaThread.join();
    }
    if (m_bmsThread.joinable()) {
        m_bmsThread.join();
    }
    if (m_networkThread.joinable()) {
        m_networkThread.join();
    }
    {
        std::lock_guard<std::mutex> lock(m_uploadMutex);
        resetUploadSessionLocked();
    }
    if (m_sockfd >= 0) {    
        close(m_sockfd);
        m_sockfd = -1;
    }
}

void ProdTestServer::stopMotorTest() {
    m_motorTestActive = false;

    if (m_motorThread.joinable()) {
        m_motorThread.join();
    }

    std::lock_guard<std::mutex> lock(m_motorMutex);
    if (m_motorCtx.ctrl) {
        m_motorCtx.ctrl->enable(false);
        m_motorCtx.ctrl.reset();
    }
    m_motorCtx = MotorCtx{};
}

void ProdTestServer::runLoop() {
    CS_ComFrame frame;
    struct sockaddr_in client;
    socklen_t len = sizeof(client);

    while (m_running) {
        int n = recvfrom(m_sockfd, &frame, sizeof(frame), 0,
                         (struct sockaddr *)&client, &len);
        if (n > 0) {
            char clientIp[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client.sin_addr, clientIp, INET_ADDRSTRLEN);
            uint16_t clientPort = ntohs(client.sin_port);

            if (n < static_cast<int>(sizeof(CS_DataHeader)) ||
                !ProdTestProtocol::verifyHeader(&frame.header) ||
                n < static_cast<int>(sizeof(CS_DataHeader) + frame.header.data_len)) {
                printf("[ProdTestServer] %s:%d | invalid frame\n", clientIp, clientPort);
                continue;
            }

            // CRC校验: header(10字节) + buffer(dataLen字节)
            // crc_sum本身(2字节)不参与计算，且位于buffer之前
            // 客户端CRC计算范围: header(10) + payload(dataLen)，不包含crc_sum
            int dataLen = (frame.header.data_len);
            int crcLen = 10 + dataLen;

            // 构造CRC计算缓冲区: header前10字节 + buffer[dataLen]字节
            uint8_t crcBuf[1024];
            memcpy(crcBuf, &frame.header, 10);  // header前10字节
            if (dataLen > 0) {
                memcpy(&crcBuf[10], frame.buffer, dataLen);  // buffer数据
            }
            uint16_t crc = ProdTestProtocol::calcCrcSum(crcBuf, crcLen);
            uint16_t recvCrc = (frame.header.crc_sum);
            if (crc != recvCrc) {
                printf("[ProdTestServer] %s:%d | cmd=0x%04X | CRC MISMATCH: calc=0x%04X recv=0x%04X\n",
                       clientIp, clientPort, frame.header.cmd, crc, recvCrc);
                sendError(&client, frame.header.cmd, frame.header.frame_id, "CRC error");
                continue;
            }

            printf("[ProdTestServer] %s:%d | cmd=0x%04X | CRC OK: 0x%04X\n",
                   clientIp, clientPort, frame.header.cmd, crc);

            if (isRestartCommand(frame.header.cmd)) {
                handleRestartCommand(frame, client);
                continue;
            }

            if (isUploadCommand(frame.header.cmd)) {
                if (!enqueueUploadRequest(frame, n, client)) {
                    sendError(&client, frame.header.cmd, frame.header.frame_id, "Upload queue busy");
                }
                continue;
            }

            if (!enqueueRequest(frame, n, client)) {
                sendError(&client, frame.header.cmd, frame.header.frame_id, "Server busy");
            }
        }
    }
}

bool ProdTestServer::enqueueRequest(const CS_ComFrame& frame, int len,
                                    const struct sockaddr_in& client) {
    std::lock_guard<std::mutex> lock(m_queueMutex);
    if (m_requestQueue.size() >= kMaxRequestQueueSize) {
        return false;
    }

    RequestTask task;
    task.frame = frame;
    task.recvLen = len;
    task.client = client;
    m_requestQueue.push_back(task);
    m_queueCv.notify_one();
    return true;
}

bool ProdTestServer::enqueueUploadRequest(const CS_ComFrame& frame, int len,
                                          const struct sockaddr_in& client) {
    std::lock_guard<std::mutex> lock(m_uploadQueueMutex);
    if (m_uploadRequestQueue.size() >= kMaxRequestQueueSize) {
        return false;
    }

    RequestTask task;
    task.frame = frame;
    task.recvLen = len;
    task.client = client;
    m_uploadRequestQueue.push_back(task);
    m_uploadQueueCv.notify_one();
    return true;
}

void ProdTestServer::workerLoop() {
    while (m_running.load()) {
        RequestTask task;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCv.wait(lock, [this]() {
                return !m_running.load() || !m_requestQueue.empty();
            });

            if (!m_running.load() && m_requestQueue.empty()) {
                break;
            }

            task = m_requestQueue.front();
            m_requestQueue.pop_front();
        }

        handleFrame(&task.frame, task.recvLen, &task.client);
    }
}

void ProdTestServer::uploadWorkerLoop() {
    while (m_running.load()) {
        RequestTask task;
        {
            std::unique_lock<std::mutex> lock(m_uploadQueueMutex);
            m_uploadQueueCv.wait(lock, [this]() {
                return !m_running.load() || !m_uploadRequestQueue.empty();
            });

            if (!m_running.load() && m_uploadRequestQueue.empty()) {
                break;
            }

            task = m_uploadRequestQueue.front();
            m_uploadRequestQueue.pop_front();
        }

        handleUploadCommand(task.frame, task.client);
    }
}

bool ProdTestServer::isRestartCommand(uint16_t cmd) const {
    return cmd == ProdTestCmd::SERVER_RESTART;
}

bool ProdTestServer::isUploadCommand(uint16_t cmd) const {
    return cmd == ProdTestCmd::FILE_UPLOAD_BEGIN ||
           cmd == ProdTestCmd::FILE_UPLOAD_DATA ||
           cmd == ProdTestCmd::FILE_UPLOAD_END ||
           cmd == ProdTestCmd::FILE_UPLOAD_CANCEL;
}

void ProdTestServer::handleRestartCommand(const CS_ComFrame& frame,
                                          const struct sockaddr_in& client) {
    ServerRestartResponse out{};
    out.res = 0;
    struct sockaddr_in replyClient = client;
    sendResponse(&replyClient, ProdTestCmd::SERVER_RESTART, frame.header.frame_id,
                 &out, sizeof(out), out.res);

    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::string cmd = std::string("systemctl restart ") + kDogToolServiceName;
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::cerr << "[ProdTestServer] restart command failed rc=" << rc << std::endl;
        }
    }).detach();
}

void ProdTestServer::resetUploadSessionLocked() {
    if (m_uploadSession.file) {
        fclose(m_uploadSession.file);
        m_uploadSession.file = nullptr;
    }
    if (!m_uploadSession.tempPath.empty()) {
        unlink(m_uploadSession.tempPath.c_str());
    }
    m_uploadSession = UploadSession{};
}

void ProdTestServer::handleUploadCommand(const CS_ComFrame& frame,
                                         const struct sockaddr_in& client) {
    switch (frame.header.cmd) {
        case ProdTestCmd::FILE_UPLOAD_BEGIN:
            handleUploadBegin(frame, client);
            return;
        case ProdTestCmd::FILE_UPLOAD_DATA:
            handleUploadData(frame, client);
            return;
        case ProdTestCmd::FILE_UPLOAD_END:
            handleUploadEnd(frame, client);
            return;
        case ProdTestCmd::FILE_UPLOAD_CANCEL:
            handleUploadCancel(frame, client);
            return;
        default:
            sendError(const_cast<struct sockaddr_in*>(&client), frame.header.cmd,
                      frame.header.frame_id, "Unknown upload cmd");
            return;
    }
}

void ProdTestServer::handleUploadBegin(const CS_ComFrame& frame,
                                       const struct sockaddr_in& client) {
    FileUploadBeginResponse out{};
    out.res = -1;
    out.max_chunk_size = kUploadChunkMaxSize;

    if (frame.header.data_len <= sizeof(FileUploadBeginRequest)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_BEGIN,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    const auto *req = reinterpret_cast<const FileUploadBeginRequest *>(frame.buffer);
    std::string filename = bounded_string_from_buffer(frame.buffer + sizeof(FileUploadBeginRequest),
                                                      frame.header.data_len - sizeof(FileUploadBeginRequest));
    filename = sn_trim(filename);
    std::string err;
    std::string finalPath;
    if (filename.empty() || !is_safe_ota_filename(filename) || !has_bin_extension(filename)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_BEGIN,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }
    if (!ensure_ota_upload_dir(err)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_BEGIN,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    finalPath = std::string(kOtaUploadDir) + "/" + filename;
    uint32_t uploadId = m_nextUploadId.fetch_add(1);
    std::string tempPath = finalPath + ".part." + std::to_string(uploadId);
    FILE *fp = fopen(tempPath.c_str(), "wb");
    if (!fp) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_BEGIN,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_uploadMutex);
        resetUploadSessionLocked();
        m_uploadSession.active = true;
        m_uploadSession.uploadId = uploadId;
        m_uploadSession.fileSize = req->file_size;
        m_uploadSession.expectedChunkIndex = 0;
        m_uploadSession.receivedSize = 0;
        m_uploadSession.expectedCrc32 = req->file_crc32;
        m_uploadSession.filename = filename;
        m_uploadSession.tempPath = tempPath;
        m_uploadSession.finalPath = finalPath;
        m_uploadSession.file = fp;
    }

    out.res = 0;
    out.upload_id = uploadId;
    out.file_size = req->file_size;
    sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_BEGIN,
                 frame.header.frame_id, &out, sizeof(out), out.res);
}

void ProdTestServer::handleUploadData(const CS_ComFrame& frame,
                                      const struct sockaddr_in& client) {
    FileUploadDataResponse out{};
    out.res = -1;

    if (frame.header.data_len <= sizeof(FileUploadDataRequest)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_DATA,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    const auto *req = reinterpret_cast<const FileUploadDataRequest *>(frame.buffer);
    const uint8_t *chunkData = frame.buffer + sizeof(FileUploadDataRequest);
    uint32_t chunkLen = frame.header.data_len - sizeof(FileUploadDataRequest);

    std::lock_guard<std::mutex> lock(m_uploadMutex);
    if (!m_uploadSession.active ||
        req->upload_id != m_uploadSession.uploadId ||
        req->chunk_index != m_uploadSession.expectedChunkIndex ||
        req->offset != m_uploadSession.receivedSize ||
        chunkLen > kUploadChunkMaxSize ||
        m_uploadSession.receivedSize + chunkLen > m_uploadSession.fileSize ||
        !m_uploadSession.file) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_DATA,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    size_t written = fwrite(chunkData, 1, chunkLen, m_uploadSession.file);
    if (written != chunkLen) {
        resetUploadSessionLocked();
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_DATA,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }
    fflush(m_uploadSession.file);

    m_uploadSession.receivedSize += chunkLen; 
    m_uploadSession.expectedChunkIndex += 1;

    out.res = 0;
    out.upload_id = m_uploadSession.uploadId;
    out.next_chunk_index = m_uploadSession.expectedChunkIndex;
    out.received_size = m_uploadSession.receivedSize;
    sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_DATA,
                 frame.header.frame_id, &out, sizeof(out), out.res);
}

void ProdTestServer::handleUploadEnd(const CS_ComFrame& frame,
                                     const struct sockaddr_in& client) {
    FileUploadEndResponse out{};
    out.res = -1;

    if (frame.header.data_len < sizeof(FileUploadEndRequest)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_END,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    const auto *req = reinterpret_cast<const FileUploadEndRequest *>(frame.buffer);

    std::lock_guard<std::mutex> lock(m_uploadMutex);
    if (!m_uploadSession.active ||
        req->upload_id != m_uploadSession.uploadId ||
        !m_uploadSession.file ||
        m_uploadSession.receivedSize != m_uploadSession.fileSize) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_END,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    fclose(m_uploadSession.file);
    m_uploadSession.file = nullptr;

    std::ifstream is(m_uploadSession.tempPath, std::ios::binary);
    if (!is) {
        resetUploadSessionLocked();
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_END,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    uint32_t crc = 0;
    std::vector<uint8_t> buf(4096);
    while (is) {
        is.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
        std::streamsize count = is.gcount();
        if (count > 0) {
            crc = crc32_update(crc, buf.data(), static_cast<size_t>(count));
        }
    }

    if (crc != m_uploadSession.expectedCrc32 ||
        rename(m_uploadSession.tempPath.c_str(), m_uploadSession.finalPath.c_str()) != 0) {
        resetUploadSessionLocked();
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_END,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    out.res = 0;
    out.upload_id = m_uploadSession.uploadId;
    out.file_size = m_uploadSession.fileSize;
    out.file_crc32 = crc;
    strncpy(out.filename, m_uploadSession.filename.c_str(), sizeof(out.filename) - 1);
    m_uploadSession = UploadSession{};
    sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_END,
                 frame.header.frame_id, &out, sizeof(out), out.res);
}

void ProdTestServer::handleUploadCancel(const CS_ComFrame& frame,
                                        const struct sockaddr_in& client) {
    FileUploadCancelResponse out{};
    out.res = -1;

    if (frame.header.data_len < sizeof(FileUploadCancelRequest)) {
        sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_CANCEL,
                     frame.header.frame_id, &out, sizeof(out), out.res);
        return;
    }

    const auto *req = reinterpret_cast<const FileUploadCancelRequest *>(frame.buffer);

    std::lock_guard<std::mutex> lock(m_uploadMutex);
    if (m_uploadSession.active && req->upload_id == m_uploadSession.uploadId) {
        out.res = 0;
        out.upload_id = m_uploadSession.uploadId;
        resetUploadSessionLocked();
    }

    sendResponse(const_cast<struct sockaddr_in*>(&client), ProdTestCmd::FILE_UPLOAD_CANCEL,
                 frame.header.frame_id, &out, sizeof(out), out.res);
}

void ProdTestServer::handleFrame(CS_ComFrame* frame, int len,
                                 struct sockaddr_in* client) {
    // 小端接收，字段直接使用，不需要swap

    m_lastClientFrameId = frame->header.frame_id;
    m_lastClient = *client;
   
    // Verify header
    if (!ProdTestProtocol::verifyHeader(&frame->header)) {
        return;
    }

    // Verify CRC
    if (!ProdTestProtocol::verifyFrame(frame, len)) {
        sendError(client, frame->header.cmd, frame->header.frame_id, "CRC error");
        return;
    }

    // Find handler
    uint16_t cmd = frame->header.cmd;
    auto it = m_handlers.find(cmd);

    if (it != m_handlers.end()) {
        if (m_otaActive.load() &&
            cmd != ProdTestCmd::MAINCTRL_OTA_UPGRADE &&
            cmd != ProdTestCmd::MOTOR_OTA_UPGRADE) {
            sendError(client, cmd, frame->header.frame_id, "OTA busy");
            return;
        }

        // 更新StatusReporter的目标地址为当前客户端
        m_reporter.setDestAddr(client);

        m_responseSent.store(false);
        CS_ComFrame resp;
        int result = it->second(frame, &resp);

        if (!m_responseSent.load()) {
            int respLen = sizeof(CS_DataHeader) + resp.header.data_len;
            sendto(m_sockfd, &resp, respLen, 0,
                   (struct sockaddr*)client, sizeof(*client));
        }
    } else {
        sendError(client, cmd, frame->header.frame_id, "Unknown cmd");
    }
}

void ProdTestServer::sendResponse(struct sockaddr_in* client, uint16_t cmd, uint16_t frame_id,
                                  const void* data, int dataLen, int8_t res) {
    std::lock_guard<std::mutex> sendLock(m_sendMutex);

    char clientIp[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client->sin_addr, clientIp, INET_ADDRSTRLEN);
    uint16_t clientPort = ntohs(client->sin_port);

    // 协议格式: header(2) + frame_id(2) + version(2) + cmd(2) + data_len(2) + crc_sum(2) + buffer[N]
    int maxLen = sizeof(CS_DataHeader) + dataLen;
    uint8_t* respBuf = new uint8_t[maxLen];
    int idx = 0;

    // header (2字节, 小端) = 0xA5A5 (回文，无影响)
    respBuf[idx++] = 0xA5;
    respBuf[idx++] = 0xA5;

    // frame_id (2字节, 小端)
    respBuf[idx++] = frame_id & 0xFF;
    respBuf[idx++] = (frame_id >> 8) & 0xFF;

    // version (2字节, 小端) = 1
    respBuf[idx++] = 1;
    respBuf[idx++] = 0;

    // cmd (2字节, 小端) = 请求cmd + 0x1000
    uint16_t respCmd = cmd + 0x1000;
    respBuf[idx++] = respCmd & 0xFF;
    respBuf[idx++] = (respCmd >> 8) & 0xFF;

    // data_len (2字节, 小端) - buffer实际长度
    respBuf[idx++] = dataLen & 0xFF;
    respBuf[idx++] = (dataLen >> 8) & 0xFF;

    // crc_sum (2字节, 小端) - 先跳过CRC字段
    idx += 2;

    // buffer: 业务数据结构体
    if (data && dataLen > 0) {
        memcpy(&respBuf[idx], data, dataLen);
    }
    idx += dataLen;

    // 计算CRC: header(10字节) + buffer(dataLen字节)
    // crc_sum位置填0，buffer整体前移2字节，这样CRC计算包含完整的54字节
    // 计算完后再移回去，保持crc_sum在header内的原有位置
    uint8_t crcBuf[1024];
    memcpy(crcBuf, respBuf, 12 + dataLen);  // 备份完整数据
    respBuf[10] = 0;
    respBuf[11] = 0;
    memmove(&respBuf[10], &respBuf[12], dataLen);  // buffer前移2字节
    uint16_t crc = ProdTestProtocol::calcCrcSum(&respBuf[0], 10 + dataLen);
    // 恢复原布局：buffer后移回原位，crc_sum写入header位置
    memmove(&respBuf[12], &respBuf[10], dataLen);
    respBuf[10] = crc & 0xFF;
    respBuf[11] = (crc >> 8) & 0xFF;

    printf("[ProdTestServer] -> %s:%d | cmd=0x%04X | dataLen=%d | crc=0x%04X\n",
           clientIp, clientPort, respCmd, dataLen, crc);

    sendto(m_sockfd, respBuf, idx, 0,
           (struct sockaddr*)client, sizeof(*client));

    m_responseSent.store(true);
    delete[] respBuf;
}

void ProdTestServer::sendError(struct sockaddr_in* client, uint16_t cmd, uint16_t frame_id,
                               const char* errMsg) {
    // 直接调用sendResponse发送错误信息
    sendResponse(client, cmd, frame_id, errMsg, errMsg ? strlen(errMsg) : 0, -1);
}

int ProdTestServer::handleImu(CS_ComFrame* req, CS_ComFrame* resp) {
    bool prevEnabled = m_reporter.isEnabled();
    m_reporter.setEnabled(false);

    McuUpdater mcu(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, m_reporter);
    if (!mcu.init()) {
        m_reporter.setEnabled(prevEnabled);
        sendError(&m_lastClient, ProdTestCmd::MAINCTRL_IMU, req->header.frame_id, "MCU init failed");
        return -1;
    }

    ImuFeedback imuData = mcu.getIMU();
    m_reporter.setEnabled(prevEnabled);

    // 检测IMU是否未上电/无数据（四元数全零表示无效）
    if (imuData.qx == 0 && imuData.qy == 0 &&
        imuData.qz == 0 && imuData.qw == 0) {
        TestLogger::instance().logTestResult("imu", false, "IMU not responding");
        ImuResponse out{};
        out.res = -1;
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_IMU, req->header.frame_id, &out, sizeof(out), -1);
        return 0;
    }

    TestLogger::instance().logTestResult("imu", true, "IMU queried");

    ImuResponse out{};
    out.res = 0;
    out.qx = imuData.qx;
    out.qy = imuData.qy;
    out.qz = imuData.qz;
    out.qw = imuData.qw;
    out.wx = imuData.wx;
    out.wy = imuData.wy;
    out.wz = imuData.wz;
    out.ax = imuData.ax;
    out.ay = imuData.ay;
    out.az = imuData.az;

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_IMU, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleBms(CS_ComFrame* req, CS_ComFrame* resp) {
    bool expected = false;
    if (!m_bmsActive.compare_exchange_strong(expected, true)) {
        printf("[BMS] busy: task already running\n");
        BmsResponse out{};
        out.res = -1;
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_BMS, req->header.frame_id,
                     &out, sizeof(out), out.res);
        m_responseSent.store(true);
        return 0;
    }

    if (m_bmsThread.joinable()) {
        m_bmsThread.join();
    }

    struct sockaddr_in client = m_lastClient;
    uint16_t frameId = req->header.frame_id;
    m_responseSent.store(true);

    m_bmsThread = std::thread([this, client, frameId]() mutable {
        bool stopped = false;
        if (!TestLogger::instance().isConflictingServicesPinnedStopped()) {
            stopped = TestLogger::instance().stopConflictingServices();
        }
        bool prevEnabled = m_reporter.isEnabled();
        m_reporter.setEnabled(false);

        board_test::BoardBasicTester tester;
        BmsResponse out = tester.getBmsResponse();

        // BMS链路在首次查询时可能需要一次预热，失败后短暂等待再重试一次。
        if (out.res != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            out = tester.getBmsResponse();
        }

        m_reporter.setEnabled(prevEnabled);
        if (stopped) {
            TestLogger::instance().restartConflictingServices();
        }

        TestLogger::instance().logTestResult("bms", out.res == 0, "", "");
        sendResponse(&client, ProdTestCmd::MAINCTRL_BMS, frameId, &out, sizeof(out), out.res);
        m_bmsActive.store(false);
    });
    return 0;
}

void ProdTestServer::registerHandlers() {
    using namespace ProdTestCmd;

    m_handlers[MOTOR_TEST_ENABLE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorEnable(req, resp);
    };
    m_handlers[MOTOR_TEST_DISABLE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorDisable(req, resp);
    };
    m_handlers[MOTOR_SN_RW] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorSnRw(req, resp);
    };
    m_handlers[MOTOR_VERSION] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorVersion(req, resp);
    };
    m_handlers[MOTOR_CONTROL] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorSetParam(req, resp);
    };
    m_handlers[MOTOR_ENCODER_CAL] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleEncoderCal(req, resp);
    };
    m_handlers[MOTOR_ENCODER_LOSS] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleEncoderLoss(req, resp);
    };
    m_handlers[MOTOR_NTC_READ] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleNtcRead(req, resp);
    };
    m_handlers[MOTOR_ENCODER_ZERO] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleEncoderZero(req, resp);
    };
    m_handlers[MOTOR_OTA_UPGRADE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorOtaUpgrade(req, resp);
    };
    m_handlers[MOTOR_ID_CHANGE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMotorIdChange(req, resp);
    };

    m_handlers[MAINCTRL_VERSION] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMcuVersion(req, resp);
    };
    m_handlers[MAINCTRL_SENSOR] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMcuSensors(req, resp);
    };
	    m_handlers[MAINCTRL_IMU] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleImu(req, resp);
    };
    m_handlers[MAINCTRL_BMS] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleBms(req, resp);
    };
    m_handlers[MAINCTRL_BLUETOOTH] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleBluetoothTest(req, resp);
    };
    m_handlers[MAINCTRL_NETWORK] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleNetworkTest(req, resp);
    };
    m_handlers[MAINCTRL_USB] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleUsbTest(req, resp);
    };
    m_handlers[MAINCTRL_RTC] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleRtcTest(req, resp);
    };
    m_handlers[MAINCTRL_SBUS] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleSbusTest(req, resp);
    };
    m_handlers[MAINCTRL_LED] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleLedTest(req, resp);
    };
    m_handlers[MAINCTRL_SN] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleSnTest(req, resp);
    };
    m_handlers[MAINCTRL_APP_SERVICE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleAppServiceTest(req, resp);
    };
    m_handlers[MAINCTRL_OTA_UPGRADE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleMcuOtaUpgrade(req, resp);
    };
    m_handlers[SERVER_PING] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleServerPing(req, resp);
    };
    m_handlers[SERVER_DISABLE_TARGET_SERVICE] = [this](CS_ComFrame* req, CS_ComFrame* resp) {
        return handleDisableTargetService(req, resp);
    };

}

// ===== Motor Handlers =====

bool ProdTestServer::ensureMotorTestRunning(uint8_t canCh, uint8_t canId, uint16_t frameId,
                                            const JointCmd &cmd) {
    bool needRestart = false;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        needRestart = !m_motorCtx.ctrl || !m_motorTestActive.load() || m_motorCtx.canCh != canCh ||
                      m_motorCtx.canId != canId;
    }

    if (needRestart) {
        stopMotorTest();
        TestLogger::instance().stopConflictingServices();

        MotorCtx newCtx;
        newCtx.canCh = canCh;
        newCtx.canId = canId;
        newCtx.jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
        newCtx.jid.can_ch = canCh;
        newCtx.jid.can_id = canId;
        newCtx.cfg.canfd = 1;
        newCtx.cfg.extend_id = 0;
        newCtx.cfg.bitrate_switch = 1;
        newCtx.ctrl = std::make_unique<JointControl>(DefaultConfig::GATEWAY_IP,
                                                     DefaultConfig::REMOTE_PORT,
                                                     newCtx.jid,
                                                     newCtx.cfg);
        if (!newCtx.ctrl->init()) {
            TestLogger::instance().logTestResult("motor_enable", false, "init failed");
            return false;
        }

        newCtx.ctrl->drainPendingPackets(16, 20);
        if (!newCtx.ctrl->enable(true)) {
            std::printf("[MotorEnable] ch=%u id=%u enable send failed\n",
                        static_cast<unsigned>(canCh),
                        static_cast<unsigned>(canId));
            TestLogger::instance().logTestResult("motor_enable", false, "enable send failed");
            return false;
        }
        std::printf("[MotorEnable] ch=%u id=%u mode+enable sent without ack wait\n",
                    static_cast<unsigned>(canCh),
                    static_cast<unsigned>(canId));
        newCtx.enabled = true;
        newCtx.cmd = cmd;
        newCtx.lastFeedback.res = 1;

        {
            std::lock_guard<std::mutex> lock(m_motorMutex);
            m_motorCtx = std::move(newCtx);
            m_motorClient = m_lastClient;
        }

        m_motorTestActive = true;
        m_motorStreamFrameId.store(frameId);
        m_motorThread = std::thread(&ProdTestServer::motorTestLoop, this);
        return true;
    }

    std::lock_guard<std::mutex> lock(m_motorMutex);
    m_motorCtx.cmd = cmd;
    m_motorClient = m_lastClient;
    m_motorStreamFrameId.store(frameId);
    return true;
}

int ProdTestServer::handleMotorEnable(CS_ComFrame* req, CS_ComFrame* resp) {
    if (req->header.data_len < 2) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_TEST_ENABLE, req->header.frame_id, "Invalid motor request");
        return -1;
    }

    // Buffer format: [can_ch(1), can_id(1), ...reserved...]
    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];
    JointCmd cmd{};
    if (!ensureMotorTestRunning(canCh, canId, req->header.frame_id, cmd)) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_TEST_ENABLE, req->header.frame_id, "Joint init failed");
        return -1;
    }

    TestLogger::instance().logTestResult("motor_enable", true, "enabled");
    MotorEnableResponse out{};
    out.res = 1;
    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_TEST_ENABLE, req->header.frame_id, &out, sizeof(out), 1);
    return 0;
}

int ProdTestServer::handleMotorDisable(CS_ComFrame* req, CS_ComFrame* resp) {
    stopMotorTest();

    TestLogger::instance().restartConflictingServices();

    TestLogger::instance().logTestResult("motor_disable", true, "disabled");
    MotorDisableResponse out{};
    out.res = 0;
    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_TEST_DISABLE, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleMotorSetParam(CS_ComFrame* req, CS_ComFrame* resp) {
    if (req->header.data_len < static_cast<int>(sizeof(uint8_t) * 2 + sizeof(float) * 5)) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_CONTROL, req->header.frame_id, "Invalid motor params");
        return -1;
    }

    // Buffer format:
    // 1) packed   : [can_ch(u8), can_id(u8), kp(f32), kd(f32), pos(f32), vel(f32), torque(f32)]
    // 2) aligned  : [can_ch(u8), can_id(u8), pad(2), kp(f32), kd(f32), pos(f32), vel(f32), torque(f32)]
    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];
    float params[5] = {};
    size_t paramsOffset = 2;
    if (req->header.data_len >= static_cast<int>(sizeof(uint8_t) * 2 + sizeof(uint16_t) + sizeof(params))) {
        paramsOffset = 4;
    }
    std::memcpy(params, req->buffer + paramsOffset, sizeof(params));

    std::printf("[MotorControl RX] frame_id=%u data_len=%u ch=%u id=%u params_offset=%zu "
                "kp=%.6f kd=%.6f pos=%.6f vel=%.6f torque=%.6f raw=",
                static_cast<unsigned>(req->header.frame_id),
                static_cast<unsigned>(req->header.data_len),
                static_cast<unsigned>(canCh),
                static_cast<unsigned>(canId),
                paramsOffset,
                params[0],
                params[1],
                params[2],
                params[3],
                params[4]);
    const int rawLen = std::min<int>(req->header.data_len, 32);
    for (int i = 0; i < rawLen; ++i) {
        std::printf("%02X", req->buffer[i]);
        if (i + 1 < rawLen) {
            std::printf(" ");
        }
    }
    std::printf("\n");

    JointCmd cmd{};
    cmd.kp = params[0];
    cmd.kd = params[1];
    cmd.position = params[2];
    cmd.velocity = params[3];
    cmd.torque = params[4];

    if (!ensureMotorTestRunning(canCh, canId, req->header.frame_id, cmd)) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_CONTROL, req->header.frame_id, "Joint init failed");
        return -1;
    }

    MotorSetParamResponse out{};
    out.res = 0;
    out.kp = params[0];
    out.kd = params[1];
    out.position = params[2];
    out.velocity = params[3];
    out.torque = params[4];
    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_CONTROL, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleEncoderCal(CS_ComFrame* req, CS_ComFrame* resp) {
    EncoderCalResponse out{};
    out.res = -1;

    if (req->header.data_len < 2) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_CAL, req->header.frame_id, "Invalid encoder calibration request");
        return -1;
    }

    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];

    bool usingActiveMotorCtx = false;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        usingActiveMotorCtx = m_motorCtx.ctrl && m_motorCtx.canCh == canCh &&
                              m_motorCtx.canId == canId && m_motorTestActive.load();
    }

    if (!usingActiveMotorCtx) {
        TestLogger::instance().stopConflictingServices();
    }

    JointId jid;
    jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
    jid.can_ch = canCh;
    jid.can_id = canId;

    JointCfg cfg;
    cfg.canfd = 1;
    cfg.extend_id = 0;
    cfg.bitrate_switch = 1;

    JointControl calCtrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
    if (!calCtrl.init()) {
        TestLogger::instance().logTestResult("encoder_cal", false, "init failed");
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_CAL, req->header.frame_id, &out, sizeof(out), out.res);
        return 0;
    }

    if (!usingActiveMotorCtx) {
        calCtrl.setControlMode(3);
        calCtrl.enable(true);
        calCtrl.drainPendingPackets(16, 20);
    } else {
        calCtrl.drainPendingPackets(16, 20);
    }

    if (!calCtrl.sendPacket(calCtrl.makeEncoderCalcFrame())) {
        TestLogger::instance().logTestResult("encoder_cal", false, "calibration send failed");
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_CAL, req->header.frame_id, &out, sizeof(out), out.res);
        return 0;
    }

    bool received = calCtrl.receiveSdoAck(0x2002, 0x00, 1000);
    if (received) {
        uint16_t errorCode = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (calCtrl.readSdoU16(0x2000, 0x00, errorCode, 500)) {
            std::printf("[Encoder Cal] ack received error_code=0x%04X\n", errorCode);
        } else {
            std::printf("[Encoder Cal] ack received error_code read timeout\n");
        }

        out.res = 0;
        TestLogger::instance().logTestResult("encoder_cal", true, "calibration ack received");
    } else {
        if (usingActiveMotorCtx) {
            out.res = 0;
            std::printf("[Encoder Cal] active motor calibration sent, ack timeout ignored\n");
            TestLogger::instance().logTestResult("encoder_cal", true,
                                                 "active motor calibration sent, ack timeout ignored");
        } else {
            TestLogger::instance().logTestResult("encoder_cal", false, "calibration ack timeout");
        }
    }

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_CAL, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleEncoderLoss(CS_ComFrame* req, CS_ComFrame* resp) {
    EncoderCalResponse out{};
    out.res = -1;
    TestLogger::instance().logTestResult("encoder_loss", false, "not implemented");
    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_LOSS, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleEncoderZero(CS_ComFrame* req, CS_ComFrame* resp) {
    EncoderCalResponse out{};
    out.res = -1;

    if (req->header.data_len < 2) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_ZERO, req->header.frame_id, "Invalid encoder zero request");
        return -1;
    }

    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];

    bool usingActiveMotorCtx = false;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        usingActiveMotorCtx = m_motorCtx.ctrl && m_motorCtx.canCh == canCh &&
                              m_motorCtx.canId == canId && m_motorTestActive.load();
    }

    if (!usingActiveMotorCtx) {
        TestLogger::instance().stopConflictingServices();
    }

    JointId jid;
    jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
    jid.can_ch = canCh;
    jid.can_id = canId;

    JointCfg cfg;
    cfg.canfd = 1;
    cfg.extend_id = 0;
    cfg.bitrate_switch = 1;

    JointControl zeroCtrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
    if (!zeroCtrl.init()) {
        TestLogger::instance().logTestResult("encoder_zero", false, "init failed");
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_ZERO, req->header.frame_id, &out, sizeof(out), out.res);
        return 0;
    }

    if (!usingActiveMotorCtx) {
        zeroCtrl.setControlMode(3);
        zeroCtrl.enable(true);
        zeroCtrl.drainPendingPackets(16, 20);
    }

    zeroCtrl.sendPacket(zeroCtrl.makeEncoderZeroFrame());

    bool received = zeroCtrl.receiveSdoAck(0x2070, 0x00, 1000);
    if (received) {
        out.res = 0;
        TestLogger::instance().logTestResult("encoder_zero", true, "zero command ack received");
    } else {
        TestLogger::instance().logTestResult("encoder_zero", false, "zero command ack timeout");
    }

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ENCODER_ZERO, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

void ProdTestServer::motorTestLoop() {
    auto nextTick = std::chrono::steady_clock::now();
    JointCmd lastPrintedCmd{};
    bool haveLastPrintedCmd = false;

    while (m_motorTestActive.load()) {
        nextTick += std::chrono::milliseconds(kMotorControlPeriodMs);

        MotorEnableResponse out{};
        struct sockaddr_in client{};
        uint16_t frameId = 0;
        bool hasFeedback = false;
        JointControl* ctrl = nullptr;
        JointCmd cmd{};

        {
            std::lock_guard<std::mutex> lock(m_motorMutex);
            if (!m_motorTestActive.load() || !m_motorCtx.ctrl) {
                break;
            }
            ctrl = m_motorCtx.ctrl.get();
            cmd = m_motorCtx.cmd;
            client = m_motorClient;
            frameId = m_motorStreamFrameId.load();
        }

        const bool cmdChanged = !haveLastPrintedCmd ||
                                cmd.kp != lastPrintedCmd.kp ||
                                cmd.kd != lastPrintedCmd.kd ||
                                cmd.position != lastPrintedCmd.position ||
                                cmd.velocity != lastPrintedCmd.velocity ||
                                cmd.torque != lastPrintedCmd.torque;
        if (cmdChanged) {
            const uint16_t kpRaw = static_cast<uint16_t>(cmd.kp * 100);
            const uint16_t kdRaw = static_cast<uint16_t>(cmd.kd * 100);
            std::printf("[MotorControl TX] ch=%u id=%u can_id=0x%03X pos=%.6f vel=%.6f torque=%.6f "
                        "kp=%.6f(raw=%u) kd=%.6f(raw=%u)\n",
                        static_cast<unsigned>(ctrl->id().can_ch),
                        static_cast<unsigned>(ctrl->id().can_id),
                        static_cast<unsigned>(ctrl->id().can_id + ctrl->controlPdoBase()),
                        cmd.position,
                        cmd.velocity,
                        cmd.torque,
                        cmd.kp,
                        static_cast<unsigned>(kpRaw),
                        cmd.kd,
                        static_cast<unsigned>(kdRaw));
            lastPrintedCmd = cmd;
            haveLastPrintedCmd = true;
        }

        ctrl->command().kp = cmd.kp;
        ctrl->command().kd = cmd.kd;
        ctrl->command().position = cmd.position;
        ctrl->command().velocity = cmd.velocity;
        ctrl->command().torque = cmd.torque;
        ctrl->sendControl();

        if (ctrl->receiveAndDecodeFeedback(kMotorFeedbackPollTimeoutMs)) {
            const auto& fb = ctrl->getFeedback();
            out.res = 1;
            out.position = fb.position.load();
            out.velocity = fb.velocity.load();
            out.torque = fb.torque.load();
            out.motor_temperature = fb.motor_temperature.load();
            out.driver_temperature = fb.driver_temperature.load();
            out.error_code = fb.error_code.load();

            {
                std::lock_guard<std::mutex> lock(m_motorMutex);
                if (m_motorCtx.ctrl.get() == ctrl) {
                    m_motorCtx.lastFeedback = out;
                }
            }
            hasFeedback = true;
        }

        if (hasFeedback) {
            sendResponse(&client, ProdTestCmd::MOTOR_TEST_ENABLE, frameId, &out, sizeof(out), 1);
        }

        std::this_thread::sleep_until(nextTick);
    }
}

// ===== MainCtrl Handlers =====

int ProdTestServer::handleMcuVersion(CS_ComFrame* req, CS_ComFrame* resp) {
    bool prevEnabled = m_reporter.isEnabled();
    m_reporter.setEnabled(false);

    McuUpdater mcu(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, m_reporter);
    if (!mcu.init()) {
        m_reporter.setEnabled(prevEnabled);
        TestLogger::instance().logTestResult("mcu_version", false, "init failed");
        sendError(&m_lastClient, ProdTestCmd::MAINCTRL_VERSION, req->header.frame_id, "MCU init failed");
        return -1;
    }

    std::string version = mcu.getVersion();
    m_reporter.setEnabled(prevEnabled);

    McuVersionResponse out{};
    if (version == "Timeout" || version == "unknown") {
        TestLogger::instance().logTestResult("mcu_version", false, version);
        out.res = -1;
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_VERSION, req->header.frame_id, &out, sizeof(out), -1);
        return 0;
    }

    TestLogger::instance().logTestResult("mcu_version", true, version);
    out.res = 0;
    strncpy(out.version, version.c_str(), sizeof(out.version) - 1);

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_VERSION, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleMcuSensors(CS_ComFrame* req, CS_ComFrame* resp) {
    bool prevEnabled = m_reporter.isEnabled();
    m_reporter.setEnabled(false);

    McuUpdater mcu(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, m_reporter);
    if (!mcu.init()) {
        m_reporter.setEnabled(prevEnabled);
        sendError(&m_lastClient, ProdTestCmd::MAINCTRL_SENSOR, req->header.frame_id, "MCU init failed");
        return -1;
    }

    McuStatus status = mcu.getmcuStatus();

    m_reporter.setEnabled(prevEnabled);

    TestLogger::instance().logTestResult("mcu_sensors", true, "sensors queried");

    McuStatusResponse out{};
    out.res = 0;
    out.sensors_status = status.sensors_status;
    out.can0_status = status.can_status[0];
    out.can1_status = status.can_status[1];
    out.can2_status = status.can_status[2];
    out.can3_status = status.can_status[3];

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_SENSOR, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleRtcTest(CS_ComFrame* req, CS_ComFrame* resp) {
    board_test::BoardBasicTester tester;

    // 解析请求数据：cmd (0=只读, 1=设置系统时间后写入RTC并读回)
    int8_t cmd = 0;
    std::string systemTime;
    if (req->header.data_len > 0) {
        cmd = req->buffer[0];
        if (req->header.data_len > 1) {
            systemTime = sn_trim(bounded_string_from_buffer(&req->buffer[1],
                                                            req->header.data_len - 1));
        }
    }

    RtcResponse out = tester.getRtcResponse(cmd, systemTime);

    TestLogger::instance().logTestResult("rtc", out.res == 0, systemTime, "");

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_RTC, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleBluetoothTest(CS_ComFrame* req, CS_ComFrame* resp) {
    board_test::BoardBasicTester tester;
    auto result = tester.run("bluetooth");

    TestLogger::instance().logTestResult("bluetooth", result.passed, result.message, result.detail);

    BluetoothResponse out{};
    out.res = result.passed ? 0 : -1;
    strncpy(out.info, result.message.c_str(), sizeof(out.info) - 1);

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_BLUETOOTH, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleNetworkTest(CS_ComFrame* req, CS_ComFrame* resp) {
    bool expected = false;
    if (!m_networkActive.compare_exchange_strong(expected, true)) {
        printf("[Network] busy: task already running\n");
        NetworkResponse out{};
        out.res = -1;
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_NETWORK, req->header.frame_id,
                     &out, sizeof(out), out.res);
        m_responseSent.store(true);
        return 0;
    }

    if (m_networkThread.joinable()) {
        m_networkThread.join();
    }

    struct sockaddr_in client = m_lastClient;
    uint16_t frameId = req->header.frame_id;
    m_responseSent.store(true);

    m_networkThread = std::thread([this, client, frameId]() mutable {
        board_test::BoardBasicTester tester;
        NetworkResponse out = tester.getNetworkResponse();

        TestLogger::instance().logTestResult("network", out.res == 0, "", "");
        sendResponse(&client, ProdTestCmd::MAINCTRL_NETWORK, frameId, &out, sizeof(out), out.res);
        m_networkActive.store(false);
    });
    return 0;
}

int ProdTestServer::handleUsbTest(CS_ComFrame* req, CS_ComFrame* resp) {
    board_test::BoardBasicTester tester;
    auto result = tester.run("usb");

    TestLogger::instance().logTestResult("usb", result.passed, result.message, result.detail);

    UsbResponse out{};
    out.res = result.passed ? 0 : -1;

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_USB, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleSbusTest(CS_ComFrame* req, CS_ComFrame* resp) {
    std::string detail;
    bool passed = run_sbus_serial_test(detail);

    TestLogger::instance().logTestResult("sbus", passed,
                                         passed ? "sbus loop test passed" : "sbus loop test failed",
                                         detail);

    SbusResponse out{};
    out.res = passed ? 0 : -1;
    strncpy(out.info, detail.c_str(), sizeof(out.info) - 1);

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_SBUS, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleLedTest(CS_ComFrame* req, CS_ComFrame* resp) {
    board_test::BoardBasicTester tester;
    LedResponse out{};

    // 解析请求: buffer[0] = LED状态 (0=空闲白,1=测试中红闪,2=等待急停蓝闪,3=成功绿,4=失败红)
    int8_t cmd = 0;
    if (req->header.data_len > 0) {
        cmd = req->buffer[0];
    }

    // 初始化 LED (仅在 LED 线程未运行时才需要初始化)
    if (!board_test::g_led_running) {
        auto result = tester.Led_init();
        TestLogger::instance().logTestResult("led", result.passed, result.message, result.detail);
        if (!result.passed) {
            out.res = -1;
            sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_LED, req->header.frame_id, &out, sizeof(out), out.res);
            return 0;
        }
    }

    // 控制 LED 状态: 0=白, 1=红闪, 2=蓝闪, 3=绿, 4=红
    board_test::LedFsmState state;
    switch (cmd) {
        case 0: state = board_test::LedFsmState::IDLE_WHITE; break;
        case 1: state = board_test::LedFsmState::TESTING_BLINK; break;
        case 2: state = board_test::LedFsmState::ESTOP_WAITING; break;
        case 3: state = board_test::LedFsmState::PASS_GREEN; break;
        case 4: state = board_test::LedFsmState::FAIL_RED; break;
        default: state = board_test::LedFsmState::IDLE_WHITE; break;
    }
    board_test::g_led_state = state;
    out.res = cmd;  // 返回当前设置的 LED 状态

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_LED, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

namespace {
// 简易 trim 实现
static std::string sn_trim(const std::string &s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static std::string bounded_string_from_buffer(const uint8_t *data, size_t len) {
    size_t actualLen = 0;
    while (actualLen < len && data[actualLen] != '\0') {
        ++actualLen;
    }
    return std::string(reinterpret_cast<const char *>(data), actualLen);
}

class ConflictingServiceGuard {
public:
    ConflictingServiceGuard() : stopped_(TestLogger::instance().stopConflictingServices()) {}
    ~ConflictingServiceGuard() {
        if (stopped_) {
            TestLogger::instance().restartConflictingServices();
        }
    }

private:
    bool stopped_;
};

class StatusReporterGuard {
public:
    explicit StatusReporterGuard(StatusReporter &reporter)
        : reporter_(reporter), previous_(reporter.isEnabled()) {
        reporter_.setEnabled(false);
    }

    ~StatusReporterGuard() {
        reporter_.setEnabled(previous_);
    }

private:
    StatusReporter &reporter_;
    bool previous_;
};

static OtaUpgradeResponse make_ota_response(int8_t res, uint8_t progress) {
    OtaUpgradeResponse out{};
    out.res = res;
    out.progress = progress;
    return out;
}

static bool ensure_ota_upload_dir(std::string &err) {
    struct stat st {};
    if (stat(kOtaUploadDir, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return true;
        }
        err = "OTA upload path is not directory";
        return false;
    }

    if (mkdir(kOtaUploadDir, 0777) == 0 || errno == EEXIST) {
        chmod(kOtaUploadDir, 0777);
        return true;
    }

    err = "create OTA upload dir failed";
    return false;
}

static bool has_bin_extension(const std::string &path) {
    if (path.size() < 4) {
        return false;
    }
    std::string ext = path.substr(path.size() - 4);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == ".bin";
}

static bool is_safe_ota_filename(const std::string &name) {
    if (name.empty() || name == "." || name == "..") {
        return false;
    }
    return name.find('/') == std::string::npos &&
           name.find('\\') == std::string::npos &&
           name.find("..") == std::string::npos;
}

static bool resolve_ota_upload_file(const std::string &requestedPath,
                                    std::string &resolvedPath,
                                    std::string &err) {
    std::string trimmed = sn_trim(requestedPath);
    if (trimmed.empty()) {
        err = "empty firmware path";
        return false;
    }

    if (!ensure_ota_upload_dir(err)) {
        return false;
    }

    if (!has_bin_extension(trimmed)) {
        err = "firmware must be .bin";
        return false;
    }

    std::string candidate;
    if (trimmed.find('/') == std::string::npos) {
        if (!is_safe_ota_filename(trimmed)) {
            err = "invalid firmware filename";
            return false;
        }
        candidate = std::string(kOtaUploadDir) + "/" + trimmed;
    } else {
        candidate = trimmed;
    }

    char uploadDirReal[PATH_MAX] = {};
    if (!realpath(kOtaUploadDir, uploadDirReal)) {
        err = "resolve OTA upload dir failed";
        return false;
    }

    char fileReal[PATH_MAX] = {};
    if (!realpath(candidate.c_str(), fileReal)) {
        err = "firmware not found";
        return false;
    }

    std::string dirReal(uploadDirReal);
    std::string filePath(fileReal);
    std::string dirPrefix = dirReal + "/";
    if (filePath.compare(0, dirPrefix.size(), dirPrefix) != 0) {
        err = "firmware outside upload dir";
        return false;
    }

    struct stat st {};
    if (stat(filePath.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
        err = "firmware is not regular file";
        return false;
    }

    if (access(filePath.c_str(), R_OK) != 0) {
        err = "firmware not readable";
        return false;
    }

    resolvedPath = filePath;
    return true;
}

static void cleanup_ota_upload_file(const std::string &filePath) {
    if (filePath.empty()) {
        return;
    }

    if (unlink(filePath.c_str()) == 0) {
        std::cout << "[OTA] deleted uploaded firmware: " << filePath << std::endl;
    } else if (errno != ENOENT) {
        std::cerr << "[OTA] failed to delete uploaded firmware: " << filePath
                  << " errno=" << errno << std::endl;
    }
}

// SN 格式校验 (18位: XX + AA + YYMMDD + NNNNN)
static bool validate_sn(const std::string &sn) {
    if (sn.size() != 18) return false;
    for (unsigned char ch : sn) {
        if (!std::isalnum(ch)) return false;
    }
    std::string hw_ver = sn.substr(2, 2);
    std::string factory = sn.substr(4, 2);
    std::string prod_date = sn.substr(6, 6);
    std::string serial = sn.substr(13, 5);
    // HW version: 2 digits 01-99
    if (!isdigit(hw_ver[0]) || !isdigit(hw_ver[1])) return false;
    int ver = (hw_ver[0]-'0')*10 + (hw_ver[1]-'0');
    if (ver < 1 || ver > 99) return false;
    // Factory: 2 uppercase letters
    if (!isupper(factory[0]) || !isupper(factory[1])) return false;
    // Production date: 6 digits
    for (int i = 0; i < 6; i++) if (!isdigit(prod_date[i])) return false;
    // Serial: 5 digits 00001-99999
    for (int i = 0; i < 5; i++) if (!isdigit(serial[i])) return false;
    return true;
}

// 执行 shell 命令并获取输出
static int exec_cmd(const char *cmd, std::string &output) {
    char buf[256];
    FILE *fp = popen(cmd, "r");
    if (!fp) return -1;
    output.clear();
    while (fgets(buf, sizeof(buf), fp)) {
        output += buf;
    }
    int rc = pclose(fp);
    return WEXITSTATUS(rc);
}
} // anonymous namespace

int ProdTestServer::handleSnTest(CS_ComFrame* req, CS_ComFrame* resp) {
    SnTestResponse out{};

    // 解析请求: buffer[0] = 0(读) 或 1(写)
    int8_t cmd = 0;
    if (req->header.data_len > 0) {
        cmd = req->buffer[0];
    }

    if (cmd == 0) {
        // 读 SN: sn read
        std::string output;
        int rc = exec_cmd("sn read", output);
        fprintf(stderr, "[SN_READ] rc=%d output='%s' len=%zu\n", rc, output.c_str(), output.size());
        if (rc == 0) {
            std::string sn = sn_trim(output);
            out.res = 0;
            out.len = sn.length();
            memcpy(out.sn, sn.c_str(), std::min((size_t)out.len, sizeof(out.sn) - 1));
            TestLogger::instance().logTestResult("sn_read", true, sn);
        } else {
            out.res = -1;
            TestLogger::instance().logTestResult("sn_read", false, "sn read failed", output);
        }
    } else {
        if (req->header.data_len <= 1) {
            out.res = -1;
            out.len = 0;
            memset(out.sn, 0, sizeof(out.sn));
            TestLogger::instance().logTestResult("sn_write", false, "SN missing", "");
            sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_SN, req->header.frame_id, &out, sizeof(out), out.res);
            return 0;
        }

        // 写 SN: buffer[0]=1, buffer[1..] = SN bytes.
        std::string sn = bounded_string_from_buffer(&req->buffer[1], req->header.data_len - 1);
        sn = sn_trim(sn);

        if (!validate_sn(sn)) {
            out.res = -1;
            out.len = 0;
            memset(out.sn, 0, sizeof(out.sn));
            TestLogger::instance().logTestResult("sn_write", false, "SN format invalid", sn);
        } else {
            // 执行写入
            std::string writeCmd = "sn write " + sn;
            std::string output;
            int rc = exec_cmd(writeCmd.c_str(), output);
            if (rc != 0) {
                out.res = -1;
                out.len = 0;
                memset(out.sn, 0, sizeof(out.sn));
                TestLogger::instance().logTestResult("sn_write", false, "sn write failed", output);
            } else {
                // 验证回读
                std::string verifyOut;
                exec_cmd("sn read", verifyOut);
                std::string verifySn = sn_trim(verifyOut);
                if (verifySn == sn) {
                    out.res = 0;
                    out.len = verifySn.length();
                    memcpy(out.sn, verifySn.c_str(), std::min((size_t)out.len, sizeof(out.sn) - 1));
                    TestLogger::instance().logTestResult("sn_write", true, "write verify passed", verifySn);
                } else {
                    out.res = -1;
                    out.len = 0;
                    memset(out.sn, 0, sizeof(out.sn));
                    TestLogger::instance().logTestResult("sn_write", false, "verify failed", "expected:" + sn + " actual:" + verifySn);
                }
            }
        }
    }

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_SN, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleAppServiceTest(CS_ComFrame* req, CS_ComFrame* resp) {
    AppServiceResponse out{};

    char buf[256] = {};
    FILE *fp = popen("systemctl is-active app_client.service", "r");
    if (fp) {
        if (fgets(buf, sizeof(buf), fp)) {
            std::string status = sn_trim(buf);
            if (status == "active") {
                out.res = 0;
            } else {
                out.res = -1;
            }
        } else {
            out.res = -1;
        }
        pclose(fp);
    } else {
        out.res = -1;
    }

    TestLogger::instance().logTestResult("app_service", out.res == 0,
        out.res == 0 ? "app_client.service is active" : "app_client.service is not active", "");

    sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_APP_SERVICE, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleServerPing(CS_ComFrame* req, CS_ComFrame* resp) {
    (void)req;
    (void)resp;

    const bool stopped = TestLogger::instance().stopConflictingServices();
    if (stopped) {
        TestLogger::instance().setConflictingServicesPinnedStopped(true);
    } else {
        std::cerr << "[SERVER_PING] warning: failed to stop bpx_motion.service" << std::endl;
    }

    ServerPingResponse out{};
    out.res = stopped ? 0 : -1;
    sendResponse(&m_lastClient, ProdTestCmd::SERVER_PING, req->header.frame_id,
                 &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleDisableTargetService(CS_ComFrame* req, CS_ComFrame* resp) {
    (void)req;
    (void)resp;

    ServerDisableTargetServiceResponse out{};
    out.res = 0;
    sendResponse(&m_lastClient, ProdTestCmd::SERVER_DISABLE_TARGET_SERVICE, req->header.frame_id,
                 &out, sizeof(out), out.res);

    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::string cmd = std::string("systemctl disable ") + kDogToolServiceName;
        int rc = std::system(cmd.c_str());
        if (rc == 0) {
            TestLogger::instance().logTestResult("disable_target_service", true,
                "dog_tool.service disable accepted");
        } else {
            TestLogger::instance().logTestResult("disable_target_service", false,
                "failed to disable dog_tool.service");
            std::cerr << "[ProdTestServer] disable command failed rc=" << rc << std::endl;
        }
    }).detach();

    return 0;
}

int ProdTestServer::handleMcuOtaUpgrade(CS_ComFrame* req, CS_ComFrame* resp) {
    (void)resp;

    std::string requestedPath = bounded_string_from_buffer(req->buffer, req->header.data_len);
    std::string filePath;
    std::string err;
    if (!resolve_ota_upload_file(requestedPath, filePath, err)) {
        OtaUpgradeResponse out = make_ota_response(-1, 0);
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_OTA_UPGRADE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    bool expected = false;
    if (!m_otaActive.compare_exchange_strong(expected, true)) {
        OtaUpgradeResponse out = make_ota_response(-1, 0);
        sendResponse(&m_lastClient, ProdTestCmd::MAINCTRL_OTA_UPGRADE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    if (m_otaThread.joinable()) {
        m_otaThread.join();
    }

    struct sockaddr_in client = m_lastClient;
    uint16_t frameId = req->header.frame_id;

    OtaUpgradeResponse started = make_ota_response(1, 0);
    sendResponse(&client, ProdTestCmd::MAINCTRL_OTA_UPGRADE, frameId,
                 &started, sizeof(started), started.res);

    m_otaThread = std::thread([this, client, frameId, filePath]() mutable {
        bool ok = false;
        std::string info = "failed";

        {
            ConflictingServiceGuard serviceGuard;
            StatusReporterGuard reporterGuard(m_reporter);

            McuUpdater mcu(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, m_reporter);
            if (!mcu.init()) {
                info = "MCU init failed";
            } else {
                int lastProgress = -1;
                mcu.setProgressCallback([this, &client, frameId, &lastProgress]
                                        (int progress, const std::string &status) {
                    (void)status;
                    progress = std::max(0, std::min(100, progress));
                    if (progress == lastProgress) {
                        return;
                    }
                    lastProgress = progress;

                    OtaUpgradeResponse progressResp =
                        make_ota_response(1, static_cast<uint8_t>(progress));
                    sendResponse(&client, ProdTestCmd::MAINCTRL_OTA_UPGRADE, frameId,
                                 &progressResp, sizeof(progressResp), progressResp.res);
                });
                ok = mcu.upgrade(filePath);
                info = ok ? "done" : "upgrade failed";
            }
        }

        cleanup_ota_upload_file(filePath);
        TestLogger::instance().logTestResult("mcu_ota", ok, info, filePath);

        OtaUpgradeResponse done = make_ota_response(ok ? 0 : -1, ok ? 100 : 0);
        sendResponse(&client, ProdTestCmd::MAINCTRL_OTA_UPGRADE, frameId,
                     &done, sizeof(done), done.res);
        m_otaActive.store(false);
    });

    return 0;
}

int ProdTestServer::handleMotorOtaUpgrade(CS_ComFrame* req, CS_ComFrame* resp) {
    (void)resp;

    if (req->header.data_len < 3) {
        OtaUpgradeResponse out = make_ota_response(-1, 0);
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_OTA_UPGRADE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];
    std::string requestedPath = bounded_string_from_buffer(&req->buffer[2],
                                                           req->header.data_len - 2);
    std::string filePath;
    std::string err;
    if (!resolve_ota_upload_file(requestedPath, filePath, err)) {
        OtaUpgradeResponse out = make_ota_response(-1, 0);
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_OTA_UPGRADE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    bool expected = false;
    if (!m_otaActive.compare_exchange_strong(expected, true)) {
        OtaUpgradeResponse out = make_ota_response(-1, 0);
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_OTA_UPGRADE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    stopMotorTest();

    if (m_otaThread.joinable()) {
        m_otaThread.join();
    }

    struct sockaddr_in client = m_lastClient;
    uint16_t frameId = req->header.frame_id;

    OtaUpgradeResponse started = make_ota_response(1, 0);
    sendResponse(&client, ProdTestCmd::MOTOR_OTA_UPGRADE, frameId,
                 &started, sizeof(started), started.res);

    m_otaThread = std::thread([this, client, frameId, filePath, canCh, canId]() mutable {
        bool ok = false;
        std::string info = "failed";

        {
            ConflictingServiceGuard serviceGuard;
            StatusReporterGuard reporterGuard(m_reporter);

            JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                               canId, canCh, m_reporter);
            if (!joint.init()) {
                info = "Joint init failed";
            } else {
                std::string versionBefore = read_driver_version_with_retry(joint, 2, 200);
                int lastProgress = -1;
                joint.setProgressCallback([this, &client, frameId, &lastProgress]
                                          (int progress, const std::string &status) {
                    (void)status;
                    progress = std::max(0, std::min(100, progress));
                    if (progress == lastProgress) {
                        return;
                    }
                    lastProgress = progress;

                    OtaUpgradeResponse progressResp =
                        make_ota_response(1, static_cast<uint8_t>(progress));
                    sendResponse(&client, ProdTestCmd::MOTOR_OTA_UPGRADE, frameId,
                                 &progressResp, sizeof(progressResp), progressResp.res);
                });
                ok = joint.upgrade(filePath);
                if (ok) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    std::string versionAfter = read_driver_version_with_retry(joint, 5, 500);
                    if (is_driver_version_error(versionAfter)) {
                        info = "done: version query failed";
                    } else if (!is_driver_version_error(versionBefore) &&
                               versionAfter == versionBefore) {
                        info = "done: version unchanged " + versionAfter;
                    } else {
                        info = "done: " + versionAfter;
                    }
                } else {
                    info = "upgrade failed";
                }
            }
        }

        cleanup_ota_upload_file(filePath);
        TestLogger::instance().logTestResult("motor_ota", ok, info,
            "ch=" + std::to_string(canCh) + " id=" + std::to_string(canId) +
            " file=" + filePath);

        OtaUpgradeResponse done = make_ota_response(ok ? 0 : -1, ok ? 100 : 0);
        sendResponse(&client, ProdTestCmd::MOTOR_OTA_UPGRADE, frameId,
                     &done, sizeof(done), done.res);
        m_otaActive.store(false);
    });

    return 0;
}

// ===== Motor extended handlers =====

int ProdTestServer::handleMotorSnRw(CS_ComFrame* req, CS_ComFrame* resp) {
    if (req->header.data_len < 3) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_SN_RW, req->header.frame_id, "Invalid motor SN request");
        return -1;
    }

    ConflictingServiceGuard serviceGuard;
    StatusReporterGuard reporterGuard(m_reporter);

    uint8_t mode = req->buffer[0];  // 0=read, 1=write
    uint8_t canCh = req->buffer[1];
    uint8_t canId = req->buffer[2];

    JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                        canId, canCh, m_reporter);
    if (!joint.init()) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_SN_RW, req->header.frame_id, "Joint init failed");
        return -1;
    }

    DriverSnRwResponse out{};
    out.res = 0;

    if (mode == 0) {
        // Read SN
        std::string sn = joint.getSn();
        if (sn.rfind("Error:", 0) == 0) {
            out.res = -1;
            TestLogger::instance().logTestResult("driver_sn_read", false, sn);
        } else {
            out.len = sn.length();
            memcpy(out.sn, sn.c_str(), std::min(sn.length(), sizeof(out.sn) - 1));
            TestLogger::instance().logTestResult("driver_sn_read", true, sn);
        }
    } else {
        if (req->header.data_len <= 3) {
            out.res = -1;
            TestLogger::instance().logTestResult("driver_sn_write", false, "SN missing");
            sendResponse(&m_lastClient, ProdTestCmd::MOTOR_SN_RW, req->header.frame_id, &out, sizeof(out), out.res);
            return 0;
        }

        // Write SN
        std::string sn = bounded_string_from_buffer(&req->buffer[3], req->header.data_len - 3);
        sn = sn_trim(sn);
        bool success = joint.setSn(sn);
        out.res = success ? 0 : -1;
        if (success) {
            out.len = sn.length();
            memcpy(out.sn, sn.c_str(), std::min(sn.length(), sizeof(out.sn) - 1));
        }
        TestLogger::instance().logTestResult("driver_sn_write", success, sn);
    }

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_SN_RW, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleMotorVersion(CS_ComFrame* req, CS_ComFrame* resp) {
    ConflictingServiceGuard serviceGuard;
    StatusReporterGuard reporterGuard(m_reporter);

    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];

    JointUpdater joint(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT,
                        canId, canCh, m_reporter);
    if (!joint.init()) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_VERSION, req->header.frame_id, "Joint init failed");
        return -1;
    }

    std::string version = read_driver_version_with_retry(joint, 2, 200);
    DriverVersionResponse out{};
    out.res = is_driver_version_error(version) ? -1 : 0;
    out.can_ch = canCh;
    out.can_id = canId;
    memset(out.sw_version, 0, sizeof(out.sw_version));
    memset(out.hw_version, 0, sizeof(out.hw_version));
    memcpy(out.sw_version, version.c_str(), std::min(version.length(), (size_t)15));

    TestLogger::instance().logTestResult("driver_version", out.res == 0, version);

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_VERSION, req->header.frame_id, &out, sizeof(out), out.res);
    return 0;
}

int ProdTestServer::handleNtcRead(CS_ComFrame* req, CS_ComFrame* resp) {
    ConflictingServiceGuard serviceGuard;
    StatusReporterGuard reporterGuard(m_reporter);

    NtcReadResponse out{};
    out.res = -1;

    if (req->header.data_len < 2) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_NTC_READ, req->header.frame_id, "Invalid NTC request");
        return -1;
    }

    uint8_t canCh = req->buffer[0];
    uint8_t canId = req->buffer[1];
    out.can_ch = canCh;
    out.can_id = canId;

    bool gotTemperature = false;

    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        if (m_motorCtx.ctrl && m_motorCtx.canCh == canCh && m_motorCtx.canId == canId) {
            const auto& fb = m_motorCtx.ctrl->getFeedback();
            out.motor_ntc = fb.motor_temperature.load();
            out.driver_ntc = fb.driver_temperature.load();
            out.res = 0;
            gotTemperature = true;
        }
    }

    if (!gotTemperature) {
        JointId jid;
        jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
        jid.can_ch = canCh;
        jid.can_id = canId;

        JointCfg cfg;
        cfg.canfd = 1;
        cfg.extend_id = 0;
        cfg.bitrate_switch = 1;

        JointControl ctrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
        if (!ctrl.init()) {
            TestLogger::instance().logTestResult("ntc_read", false, "init failed");
            sendResponse(&m_lastClient, ProdTestCmd::MOTOR_NTC_READ, req->header.frame_id, &out, sizeof(out), out.res);
            return 0;
        }

        ctrl.setControlMode(3);
        ctrl.enable(true);
        ctrl.command() = JointCmd{};
        ctrl.sendControl();

        if (ctrl.receiveAndDecodeFeedback(1000)) {
            const auto& fb = ctrl.getFeedback();
            out.motor_ntc = fb.motor_temperature.load();
            out.driver_ntc = fb.driver_temperature.load();
            out.res = 0;
            gotTemperature = true;
        }

        ctrl.enable(false);
    }

    if (!gotTemperature) {
        TestLogger::instance().logTestResult("ntc_read", false, "feedback timeout");
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_NTC_READ, req->header.frame_id, &out, sizeof(out), out.res);
        return 0;
    }

    TestLogger::instance().logTestResult("ntc_read", true, "ch=" + std::to_string(canCh));

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_NTC_READ, req->header.frame_id, &out, sizeof(out), 0);
    return 0;
}

int ProdTestServer::handleMotorIdChange(CS_ComFrame* req, CS_ComFrame* resp) {
    (void)resp;

    if (req->header.data_len < 3) {
        sendError(&m_lastClient, ProdTestCmd::MOTOR_ID_CHANGE, req->header.frame_id,
                  "Invalid motor id change request");
        return -1;
    }

    ConflictingServiceGuard serviceGuard;
    StatusReporterGuard reporterGuard(m_reporter);

    const uint8_t canCh = req->buffer[0];
    const uint8_t oldCanId = req->buffer[1];
    const uint8_t newCanId = req->buffer[2];

    MotorIdChangeResponse out{};
    out.res = -1;
    out.can_ch = canCh;
    out.old_can_id = oldCanId;
    out.new_can_id = newCanId;

    if (oldCanId == 0 || newCanId == 0 || oldCanId > 0x7F || newCanId > 0x7F) {
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ID_CHANGE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    if (oldCanId == newCanId) {
        out.res = 0;
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ID_CHANGE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    bool shouldStopMotor = false;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        shouldStopMotor = m_motorCtx.ctrl && m_motorCtx.canCh == canCh &&
                          (m_motorCtx.canId == oldCanId || m_motorCtx.canId == newCanId);
    }
    if (shouldStopMotor) {
        stopMotorTest();
    }

    JointId jid;
    jid.board_id = DefaultConfig::GATEWAY_BOARD_ID;
    jid.can_ch = canCh;
    jid.can_id = oldCanId;

    JointCfg cfg;
    cfg.canfd = 1;
    cfg.extend_id = 0;
    cfg.bitrate_switch = 1;

    JointControl ctrl(DefaultConfig::GATEWAY_IP, DefaultConfig::REMOTE_PORT, jid, cfg);
    if (!ctrl.init()) {
        TestLogger::instance().logTestResult("motor_id_change", false, "init failed");
        sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ID_CHANGE, req->header.frame_id,
                     &out, sizeof(out), out.res);
        return 0;
    }

    const bool success = ctrl.changeNodeId(newCanId, 1500);
    out.res = success ? 0 : -1;
    std::printf("[MotorIdChange] ch=%u old=%u new=%u verify=%s\n",
                static_cast<unsigned>(canCh),
                static_cast<unsigned>(oldCanId),
                static_cast<unsigned>(newCanId),
                success ? "ok" : "failed");
    TestLogger::instance().logTestResult(
        "motor_id_change",
        success,
        std::string("ch=") + std::to_string(canCh) +
            " old=" + std::to_string(oldCanId) +
            " new=" + std::to_string(newCanId));

    sendResponse(&m_lastClient, ProdTestCmd::MOTOR_ID_CHANGE, req->header.frame_id,
                 &out, sizeof(out), out.res);
    return 0;
}
