#ifndef DEVICE_CREATOR_H
#define DEVICE_CREATOR_H

#include <string>

class DeviceCreator {
public:
    DeviceCreator();
    bool create(const std::string &sn, std::string &response);
    bool parseResponse(const std::string &resp);

private:
    // 服务器配置

    int         m_port = 8080;
    const char *m_host = "www.mirrormetech.com";
#if 1

    const char *m_path = "/corm/api/v1/device/create"; // for dev sn

#else
    const char *m_path = "/corm/api/v2/module-inventory/check-in"; // for module sn

#endif
    const char *m_token = "Bearer "
                          "eyJhbGciOiJIUzI1NiJ9."
                          "eyJ1c2VySWQiOjEsInN1YiI6InpodWp1bmppYW4iLCJpYXQiOjE3NDU3NTg5NjYsImV4cCI6"
                          "MzkyMzAzODk2Nn0.TwvqwY_yb_PF88YGB-7C7BUUi5LDASj2cQt_Za6qjMQ";
    // 内部接口
    int  createSocket();
    bool sendHttpPost(int sock, const std::string &sn, std::string &resp);
};

#endif