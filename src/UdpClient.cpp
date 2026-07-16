#include "UdpClient.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <unordered_map>

namespace {
constexpr size_t kMaxQueuedPacketsPerSubscriber = 256;

struct SharedSocketKey {
    std::string ip;
    int remotePort = 0;
    int localPort = 0;

    bool operator==(const SharedSocketKey &other) const {
        return ip == other.ip && remotePort == other.remotePort && localPort == other.localPort;
    }
};

struct SharedSocketKeyHash {
    size_t operator()(const SharedSocketKey &key) const {
        size_t h1 = std::hash<std::string>{}(key.ip);
        size_t h2 = std::hash<int>{}(key.remotePort);
        size_t h3 = std::hash<int>{}(key.localPort);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

std::mutex g_sharedSocketsMutex;
std::unordered_map<SharedSocketKey,
                   std::shared_ptr<UdpClient::SharedSocket>,
                   SharedSocketKeyHash>
    g_sharedSockets;
} // namespace

struct UdpClient::SharedSocket {
    int sockfd = -1;
    sockaddr_in remoteAddr{};
    std::mutex sendMutex;
    std::mutex subscribersMutex;
    std::vector<std::weak_ptr<SubscriberState>> subscribers;
    std::thread recvThread;
    bool running = false;

    void start() {
        running = true;
        recvThread = std::thread([this]() { recvLoop(); });
    }

    void stop() {
        running = false;
        if (recvThread.joinable()) {
            recvThread.join();
        }
        if (sockfd >= 0) {
            close(sockfd);
            sockfd = -1;
        }
    }

    void addSubscriber(const std::shared_ptr<SubscriberState> &state) {
        std::lock_guard<std::mutex> lock(subscribersMutex);
        subscribers.push_back(state);
    }

    void removeSubscriber(const std::shared_ptr<SubscriberState> &state) {
        std::lock_guard<std::mutex> lock(subscribersMutex);
        subscribers.erase(std::remove_if(subscribers.begin(),
                                         subscribers.end(),
                                         [&state](const std::weak_ptr<SubscriberState> &weak) {
                                             auto shared = weak.lock();
                                             return !shared || shared == state;
                                         }),
                          subscribers.end());
    }

    bool hasSubscribers() {
        std::lock_guard<std::mutex> lock(subscribersMutex);
        subscribers.erase(std::remove_if(subscribers.begin(),
                                         subscribers.end(),
                                         [](const std::weak_ptr<SubscriberState> &weak) {
                                             return weak.expired();
                                         }),
                          subscribers.end());
        return !subscribers.empty();
    }

    void recvLoop() {
        while (running) {
            fd_set readFds;
            FD_ZERO(&readFds);
            FD_SET(sockfd, &readFds);

            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100 * 1000;

            int ret = select(sockfd + 1, &readFds, nullptr, nullptr, &tv);
            if (ret <= 0) {
                continue;
            }

            uint8_t temp[2048];
            ssize_t n = recvfrom(sockfd, temp, sizeof(temp), 0, nullptr, nullptr);
            if (n <= 0) {
                continue;
            }

            std::vector<std::shared_ptr<SubscriberState>> targets;
            {
                std::lock_guard<std::mutex> lock(subscribersMutex);
                subscribers.erase(std::remove_if(subscribers.begin(),
                                                 subscribers.end(),
                                                 [&targets](const std::weak_ptr<SubscriberState> &weak) {
                                                     auto shared = weak.lock();
                                                     if (!shared) {
                                                         return true;
                                                     }
                                                     targets.push_back(shared);
                                                     return false;
                                                 }),
                                  subscribers.end());
            }

            for (const auto &state : targets) {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (state->packets.size() >= kMaxQueuedPacketsPerSubscriber) {
                    state->packets.pop_front();
                }
                state->packets.emplace_back(temp, temp + n);
                state->cv.notify_one();
            }
        }
    }
};

UdpClient::UdpClient(const std::string &ip, int remotePort, int localPort)
    : m_ip(ip)
    , m_remotePort(remotePort)
    , m_localPort(localPort)
    , m_sockfd(-1) {
}

UdpClient::~UdpClient() {
    closeSocket();
}

bool UdpClient::init() {
    SharedSocketKey key{m_ip, m_remotePort, m_localPort};

    std::lock_guard<std::mutex> globalLock(g_sharedSocketsMutex);
    auto it = g_sharedSockets.find(key);
    if (it == g_sharedSockets.end()) {
        auto sharedSocket = std::make_shared<SharedSocket>();
        sharedSocket->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sharedSocket->sockfd < 0) {
            return false;
        }

        int nRecvBuf = 4 * 1024 * 1024;
        setsockopt(sharedSocket->sockfd, SOL_SOCKET, SO_RCVBUF,
                   (const char *)&nRecvBuf, sizeof(int));

        int reuse = 1;
        setsockopt(sharedSocket->sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        if (m_localPort > 0) {
            struct sockaddr_in localAddr;
            memset(&localAddr, 0, sizeof(localAddr));
            localAddr.sin_family = AF_INET;
            localAddr.sin_addr.s_addr = INADDR_ANY;
            localAddr.sin_port = htons(m_localPort);
            if (bind(sharedSocket->sockfd, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0) {
                close(sharedSocket->sockfd);
                return false;
            }
        }

        memset(&sharedSocket->remoteAddr, 0, sizeof(sharedSocket->remoteAddr));
        sharedSocket->remoteAddr.sin_family = AF_INET;
        sharedSocket->remoteAddr.sin_port = htons(m_remotePort);
        inet_pton(AF_INET, m_ip.c_str(), &sharedSocket->remoteAddr.sin_addr);
        sharedSocket->start();
        it = g_sharedSockets.emplace(key, sharedSocket).first;
    }

    m_sharedSocket = it->second;
    m_sockfd = m_sharedSocket->sockfd;
    m_remoteAddr = m_sharedSocket->remoteAddr;
    m_subscriberState = std::make_shared<SubscriberState>();
    m_sharedSocket->addSubscriber(m_subscriberState);
    return true;
}

bool UdpClient::send(const uint8_t *data, size_t len) {
    if (!m_sharedSocket || m_sharedSocket->sockfd < 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_sharedSocket->sendMutex);
    ssize_t sent = sendto(m_sharedSocket->sockfd,
                          data,
                          len,
                          0,
                          (struct sockaddr *)&m_sharedSocket->remoteAddr,
                          sizeof(m_sharedSocket->remoteAddr));
    return sent == (ssize_t)len;
}

int UdpClient::receive(std::vector<uint8_t> &out_buffer, int timeout_ms) {
    if (!m_subscriberState) {
        return -1;
    }

    std::unique_lock<std::mutex> lock(m_subscriberState->mutex);
    if (m_subscriberState->packets.empty()) {
        if (!m_subscriberState->cv.wait_for(
                lock,
                std::chrono::milliseconds(timeout_ms),
                [this]() {
                    return !m_subscriberState->packets.empty() || m_subscriberState->closed;
                })) {
            return 0;
        }
    }

    if (m_subscriberState->closed) {
        return -1;
    }
    if (m_subscriberState->packets.empty()) {
        return 0;
    }

    out_buffer = std::move(m_subscriberState->packets.front());
    m_subscriberState->packets.pop_front();
    return static_cast<int>(out_buffer.size());
}

void UdpClient::closeSocket() {
    auto sharedSocket = m_sharedSocket;
    auto subscriberState = m_subscriberState;
    if (subscriberState) {
        {
            std::lock_guard<std::mutex> lock(subscriberState->mutex);
            subscriberState->closed = true;
            subscriberState->packets.clear();
        }
        subscriberState->cv.notify_all();
    }

    if (sharedSocket && subscriberState) {
        sharedSocket->removeSubscriber(subscriberState);
    }

    if (sharedSocket) {
        std::lock_guard<std::mutex> globalLock(g_sharedSocketsMutex);
        SharedSocketKey key{m_ip, m_remotePort, m_localPort};
        auto it = g_sharedSockets.find(key);
        if (it != g_sharedSockets.end() && it->second == sharedSocket && !sharedSocket->hasSubscribers()) {
            sharedSocket->stop();
            g_sharedSockets.erase(it);
        }
    }

    m_subscriberState.reset();
    m_sharedSocket.reset();
    m_sockfd = -1;
}
