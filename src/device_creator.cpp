#include "device_creator.hpp"
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <iostream>
using namespace std;

DeviceCreator::DeviceCreator() {
}

int DeviceCreator::createSocket() {
    // 域名解析
    struct hostent *hostent = gethostbyname(m_host);
    if (!hostent)
        return -1;

    // 创建 TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return -1;

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(m_port);
    memcpy(&addr.sin_addr.s_addr, hostent->h_addr, hostent->h_length);

    // 连接服务器
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }

    return sock;
}

bool DeviceCreator::sendHttpPost(int sock, const string &sn, string &resp) {
    // 手动拼接 JSON（不依赖任何库）
    string json = "{";
    json += "\"sn\":\"" + sn + "\",";
    json += "\"scannerId\":\"SCAN-001\",";
    json += "\"operatorId\":\"OP001\",";
    json += "\"workstationId\":\"WS001\",";
    json += "\"snType\":\"MASS\"";
    // json += "\"firmwareVer\":\"{\\\"ver\\\":\\\"1.0.0\\\"}\","; //
    // json += "\"hardwareVer\":\"{\\\"ver\\\":\\\"01\\\"}\",";    //
    // json += "\"softwareVer\":\"{\\\"ver\\\":\\\"1.2.3\\\"}\"";  //
    json += "}";

    // 拼接 HTTP 请求
    string req;
#if 0
    req += "POST " + string(m_path) + "?sn=" + sn + " HTTP/1.1\r\n";
#else

    req += "POST " + string(m_path) + " HTTP/1.1\r\n";
#endif
    req += "Host: " + string(m_host) + ":" + to_string(m_port) + "\r\n";
    req += "Authorization: " + string(m_token) + "\r\n";
    req += "Content-Type: application/json\r\n";
    req += "Content-Length: " + to_string(json.size()) + "\r\n";
    req += "Connection: close\r\n\r\n";
    req += json;

    // std::cout << "\n========================================" << std::endl;
    // std::cout << "【发送到服务器的完整请求】" << std::endl;
    // std::cout << "----------------------------------------" << std::endl;
    // std::cout << req << std::endl;
    // std::cout << "========================================\n" << std::endl;

    // 发送
    send(sock, req.c_str(), req.size(), 0);

    // 接收响应
    char buf[ 4096 ];
    resp.clear();
    ssize_t n;
    while ((n = recv(sock, buf, sizeof(buf) - 1, 0)) > 0) {
        buf[ n ] = 0;
        resp += buf;
    }

    return true;
}
bool DeviceCreator::parseResponse(const std::string &resp) {
    bool        uploadSuccess   = false;
    bool        snNotFound      = false;
    bool        snAlreadyExists = false;
    std::string serverMsg;

    // 1. 空回复 = 超时/网络失败
    if (resp.empty()) {
        serverMsg = "网络异常或连接超时";
    } else {
        // 2. 非 200
        if (resp.find("HTTP/1.1 200") == std::string::npos) {
            serverMsg = "服务器响应异常";
        } else {
            // 成功
            if (resp.find("\"status\":\"success\"") != std::string::npos) {
                uploadSuccess = true;
            }
            // SN 未找到
            else if (resp.find("模组SN未在生产系统中找到") != std::string::npos) {
                snNotFound = true;
            }
            // 已入库
            else if (resp.find("该模组已入库") != std::string::npos) {
                snAlreadyExists = true;
            }

            // 提取错误信息
            size_t msgPos = resp.find("\"message\":\"");
            if (msgPos != std::string::npos) {
                msgPos += 11;
                size_t endPos = resp.find("\"", msgPos);
                if (endPos != std::string::npos) {
                    serverMsg = resp.substr(msgPos, endPos - msgPos);
                }
            }
        }
    }

    // ========== 输出提示 ==========
    std::cout << "\n========================================" << std::endl;
    if (uploadSuccess) {
        std::cout << "✅ 服务器入库成功！" << std::endl;
        std::cout << "========================================\n" << std::endl;
        return true;
    } else if (snAlreadyExists) {
        std::cout << "⚠️  设备已入库，无需重复操作" << std::endl;
        return true;
    } else if (snNotFound) {
        std::cout << "❌ SN 未在生产系统中找到，请检查SN是否正确" << std::endl;
    } else if (!serverMsg.empty()) {
        std::cout << "❌ 服务器返回：" << serverMsg << std::endl;
    } else {
        std::cout << "❌ 上传失败，请检查网络或重试" << std::endl;
    }
    std::cout << "========================================\n" << std::endl;

    // 失败统一 return false
    return false;
}
bool DeviceCreator::create(const string &sn, string &response) {
    int sock = createSocket();
    if (sock < 0) {
        response = "connect failed";
        return false;
    }

    bool ok = sendHttpPost(sock, sn, response);
    close(sock);
    return ok;
}