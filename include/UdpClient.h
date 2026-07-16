#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

class UdpClient {
public:
    struct SharedSocket;
    struct SubscriberState {
        std::mutex mutex;
        std::condition_variable cv;
        std::deque<std::vector<uint8_t>> packets;
        bool closed = false;
    };

    UdpClient(const std::string& ip, int remotePort, int localPort = 0);
    ~UdpClient();

    bool init();
    
    // 发送原始数据
    bool send(const uint8_t* data, size_t len);
    
    // 接收数据（带超时机制）
    // timeout_ms: 等待时间
    // out_buffer: 接收缓冲区
    // returns: 接收到的字节数，失败返回 -1
    int receive(std::vector<uint8_t>& out_buffer, int timeout_ms = 1000);

    void closeSocket();

private:
    std::string m_ip;
    int m_remotePort;
    int m_localPort;
    int m_sockfd;
    struct sockaddr_in m_remoteAddr;
    std::shared_ptr<SharedSocket> m_sharedSocket;
    std::shared_ptr<SubscriberState> m_subscriberState;
};

#endif
