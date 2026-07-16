#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#include <string>
#include <map>

struct TestConfig {
    // SN validation
    bool snStrictMode;
    int snMinLen;
    int snMaxLen;

    // SBUS thresholds
    int sbusMinChannel;
    int sbusMaxChannel;
    int sbusTimeoutMs;
    std::string sbusTestData;  // hex string for test data

    // IMU thresholds
    float imuTempMin;
    float imuTempMax;

    // BMS thresholds
    float bmsVoltageMin;
    float bmsVoltageMax;
    int bmsMinCapacityMah;

    // Motor thresholds
    float motorTempMax;
    float driverTempMax;

    // Network test
    std::string networkWlan0Ip;
    std::string networkWlan1Ip;
    std::string networkEnp1s0Ip;
    std::string networkEth0Ip;

    // Log settings
    std::string logPath;
    int logRetentionDays;

    // Gateway
    std::string gatewayIp;
    int gatewayPort;
};

class TestConfigManager {
public:
    static TestConfig& get();
    static bool loadFromFile(const std::string& path);
    static bool saveToFile(const std::string& path);
    static std::string getDefaultConfigPath();

private:
    static TestConfig s_config;
    static bool s_loaded;
};

#endif // TEST_CONFIG_H
