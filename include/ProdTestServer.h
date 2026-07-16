#ifndef PROD_TEST_SERVER_H
#define PROD_TEST_SERVER_H

#include <atomic>
#include <functional>
#include <map>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstdio>
#include <string>
#include <netinet/in.h>
#include "ProdTestProtocol.h"
#include "StatusReporter.h"
#include "JointControl.h"
#include "JointUpdater.h"
#include "McuUpdater.h"
#include "BoardBasicTester.h"

// Function pointer type for command handlers
// Returns: 0=success, -1=error
using CmdHandler = std::function<int(CS_ComFrame* req, CS_ComFrame* resp)>;

class ProdTestServer {
public:
    ProdTestServer(int port, StatusReporter& reporter);
    ~ProdTestServer();

    // Start the server (blocking)
    void start();

    // Stop server
    void stop();

    // Get socket fd for StatusReporter
    int getSocketFd() { return m_sockfd; }

private:
    // Core loop
    void runLoop();
    void workerLoop();
    void uploadWorkerLoop();
    bool enqueueRequest(const CS_ComFrame& frame, int len, const struct sockaddr_in& client);
    bool enqueueUploadRequest(const CS_ComFrame& frame, int len, const struct sockaddr_in& client);
    void handleRestartCommand(const CS_ComFrame& frame, const struct sockaddr_in& client);
    bool isRestartCommand(uint16_t cmd) const;
    bool isUploadCommand(uint16_t cmd) const;
    void handleUploadCommand(const CS_ComFrame& frame, const struct sockaddr_in& client);
    void handleUploadBegin(const CS_ComFrame& frame, const struct sockaddr_in& client);
    void handleUploadData(const CS_ComFrame& frame, const struct sockaddr_in& client);
    void handleUploadEnd(const CS_ComFrame& frame, const struct sockaddr_in& client);
    void handleUploadCancel(const CS_ComFrame& frame, const struct sockaddr_in& client);
    void resetUploadSessionLocked();

    // Handle incoming frame
    void handleFrame(CS_ComFrame* frame, int len, struct sockaddr_in* client);

    // Build and send response
    void sendResponse(struct sockaddr_in* client, uint16_t cmd, uint16_t frame_id,
                     const void* data, int dataLen, int8_t res);

    // Build error response
    void sendError(struct sockaddr_in* client, uint16_t cmd, uint16_t frame_id, const char* errMsg);

    // Motor test worker
    void motorTestLoop();
    void stopMotorTest();
    bool ensureMotorTestRunning(uint8_t canCh, uint8_t canId, uint16_t frameId, const JointCmd &cmd);

    // ===== Command Handlers =====
    int handleMotorEnable(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMotorDisable(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMotorSetParam(CS_ComFrame* req, CS_ComFrame* resp);
    int handleEncoderCal(CS_ComFrame* req, CS_ComFrame* resp);
    int handleEncoderLoss(CS_ComFrame* req, CS_ComFrame* resp);
    int handleEncoderZero(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMotorOtaUpgrade(CS_ComFrame* req, CS_ComFrame* resp);

    int handleMcuVersion(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMcuSensors(CS_ComFrame* req, CS_ComFrame* resp);
    int handleImu(CS_ComFrame* req, CS_ComFrame* resp);
    int handleBms(CS_ComFrame* req, CS_ComFrame* resp);
    int handleRtcTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleBluetoothTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleNetworkTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleUsbTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleSbusTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleLedTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleSnTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleAppServiceTest(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMcuOtaUpgrade(CS_ComFrame* req, CS_ComFrame* resp);
    int handleServerPing(CS_ComFrame* req, CS_ComFrame* resp);
    int handleDisableTargetService(CS_ComFrame* req, CS_ComFrame* resp);

    int handleMotorSnRw(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMotorVersion(CS_ComFrame* req, CS_ComFrame* resp);
    int handleNtcRead(CS_ComFrame* req, CS_ComFrame* resp);
    int handleMotorIdChange(CS_ComFrame* req, CS_ComFrame* resp);

    // Register all handlers
    void registerHandlers();

    // Motor Control Context
    struct MotorCtx {
        JointId jid;
        JointCfg cfg;
        std::unique_ptr<JointControl> ctrl;
        bool enabled = false;
        JointCmd cmd;
        MotorEnableResponse lastFeedback{};
        int canCh = 0;
        int canId = 0;
    };

    struct RequestTask {
        CS_ComFrame frame{};
        int recvLen = 0;
        struct sockaddr_in client{};
    };

    struct UploadSession {
        bool active = false;
        uint32_t uploadId = 0;
        uint32_t fileSize = 0;
        uint32_t expectedChunkIndex = 0;
        uint32_t receivedSize = 0;
        uint32_t expectedCrc32 = 0;
        std::string filename;
        std::string tempPath;
        std::string finalPath;
        FILE* file = nullptr;
    };

    int m_sockfd;
    int m_port;
    StatusReporter& m_reporter;
    std::atomic<bool> m_running;
    std::atomic<bool> m_motorTestActive;
    std::atomic<bool> m_otaActive;
    std::atomic<bool> m_bmsActive;
    std::atomic<bool> m_networkActive;
    std::map<uint16_t, CmdHandler> m_handlers;
    std::mutex m_motorMutex;
    std::mutex m_sendMutex;
    std::mutex m_queueMutex;
    std::mutex m_uploadQueueMutex;
    std::mutex m_uploadMutex;
    std::condition_variable m_queueCv;
    std::condition_variable m_uploadQueueCv;
    std::deque<RequestTask> m_requestQueue;
    std::deque<RequestTask> m_uploadRequestQueue;
    MotorCtx m_motorCtx;
    UploadSession m_uploadSession;
    std::thread m_workerThread;
    std::thread m_uploadWorkerThread;
    std::thread m_motorThread;
    std::thread m_otaThread;
    std::thread m_bmsThread;
    std::thread m_networkThread;
    std::atomic<uint16_t> m_lastClientFrameId;
    std::atomic<uint16_t> m_motorStreamFrameId;
    std::atomic<uint32_t> m_nextUploadId;
    std::atomic<bool> m_responseSent;
    struct sockaddr_in m_lastClient;
    struct sockaddr_in m_motorClient;
};

#endif // PROD_TEST_SERVER_H
