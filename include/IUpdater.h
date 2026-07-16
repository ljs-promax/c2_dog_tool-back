#ifndef I_UPDATER_H
#define I_UPDATER_H

#include <string>
#include <vector>
#include <iostream>

class IUpdater {
public:
    virtual ~IUpdater() = default;

    // 初始化（如建立 Socket）
    virtual bool init() = 0;

    // 获取版本号（返回字符串，如 "v1.0.2"）
    virtual std::string getVersion() = 0;

    // 执行升级流程
    virtual bool upgrade(const std::string& filePath) = 0;

protected:
    // 通用辅助工具：计算校验和 (RD协议使用)
    uint16_t calculateCrc16(const std::vector<uint8_t>& data) {
        uint32_t sum = 0;
        for (uint8_t b : data) sum += b;
        return static_cast<uint16_t>(sum & 0xFFFF);
    }

    // 打印进度条
    void printProgress(int percentage) {
        int width = 50;
        std::cout << "\r[" << std::string(percentage / (100 / width), '=') 
                  << std::string(width - percentage / (100 / width), ' ') 
                  << "] " << percentage << "%" << std::flush;
        if (percentage >= 100) std::cout << std::endl;
    }
};

#endif