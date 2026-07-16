#pragma once
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

class StatusReporter {
public:
    StatusReporter() {
        m_useExternalSocket = false;
        m_enabled = true;  // 默认启用
        memset(&m_destaddr, 0, sizeof(m_destaddr));
        m_destaddr.sin_family = AF_INET;
        m_destaddr.sin_port = htons(9876);
        inet_pton(AF_INET, "127.0.0.1", &m_destaddr.sin_addr);
    }

    ~StatusReporter() {
        // 不再关闭socket，由外部管理
    }

    void setExternalSocket(int sockfd) {
        m_sockfd = sockfd;
        m_useExternalSocket = true;
    }

    void setDestAddr(struct sockaddr_in* client) {
        m_destaddr = *client;
    }

    // 启用/禁用所有状态上报
    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

    void sendStatus(const std::string &device, const std::string &version, int progress) {
        if (!m_enabled) return;  // 禁用时不发送
        if (progress >= 0 && progress <= 100) {
            if (m_lastProgress.count(device) && m_lastProgress[device] == progress) {
                return;
            }
        }
        m_lastProgress[device] = progress;

        std::string json = "{\"device\": \"" + device + "\", \"version\": \"" +
                           version + "\", \"progress\": " + std::to_string(progress) + "}\n";
        sendto(m_sockfd, json.c_str(), json.length(), 0,
               (struct sockaddr *)&m_destaddr, sizeof(m_destaddr));
    }

    void sendRes(const std::string &device, int progress) {
        if (!m_enabled) return;  // 禁用时不发送
        std::string json = "{\"device\": \"" + device +
                           "\", \"res\": " + std::to_string(progress) +
                           ", \"progress\": " + std::to_string(progress) + "}\n";
        sendto(m_sockfd, json.c_str(), json.length(), 0,
               (struct sockaddr *)&m_destaddr, sizeof(m_destaddr));
    }

private:
    int m_sockfd;
    bool m_useExternalSocket;
    bool m_enabled;
    struct sockaddr_in m_destaddr;
    std::map<std::string, int> m_lastProgress;
};
