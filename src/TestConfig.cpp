#include "TestConfig.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>

TestConfig TestConfigManager::s_config;
bool TestConfigManager::s_loaded = false;

TestConfig& TestConfigManager::get() {
    if (!s_loaded) {
        loadFromFile(getDefaultConfigPath());
    }
    return s_config;
}

std::string TestConfigManager::getDefaultConfigPath() {
    return "/etc/dog_tool/config.ini";
}

bool TestConfigManager::loadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        // Try current directory
        file.open("test_config.ini");
        if (!file.is_open()) {
            std::cerr << "TestConfig: cannot open " << path << ", using defaults" << std::endl;
            s_loaded = true;
            return false;
        }
    }

    std::string line;
    while (std::getline(file, line)) {
        // Trim whitespace
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        size_t end = line.find_last_not_of(" \t\r\n");
        line = line.substr(start, end - start + 1);

        // Skip comments
        if (line[0] == '#' || line[0] == ';') continue;

        // Parse key=value
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Trim
        start = key.find_first_not_of(" \t");
        end = key.find_last_not_of(" \t");
        if (start != std::string::npos) key = key.substr(start, end - start + 1);
        start = val.find_first_not_of(" \t");
        end = val.find_last_not_of(" \t");
        if (start != std::string::npos) val = val.substr(start, end - start + 1);

        // Parse key-value pairs
        if (key == "snStrictMode") s_config.snStrictMode = (val == "1" || val == "true");
        else if (key == "snMinLen") s_config.snMinLen = std::stoi(val);
        else if (key == "snMaxLen") s_config.snMaxLen = std::stoi(val);
        else if (key == "sbusMinChannel") s_config.sbusMinChannel = std::stoi(val);
        else if (key == "sbusMaxChannel") s_config.sbusMaxChannel = std::stoi(val);
        else if (key == "sbusTimeoutMs") s_config.sbusTimeoutMs = std::stoi(val);
        else if (key == "sbusTestData") s_config.sbusTestData = val;
        else if (key == "imuTempMin") s_config.imuTempMin = std::stof(val);
        else if (key == "imuTempMax") s_config.imuTempMax = std::stof(val);
        else if (key == "bmsVoltageMin") s_config.bmsVoltageMin = std::stof(val);
        else if (key == "bmsVoltageMax") s_config.bmsVoltageMax = std::stof(val);
        else if (key == "bmsMinCapacityMah") s_config.bmsMinCapacityMah = std::stoi(val);
        else if (key == "motorTempMax") s_config.motorTempMax = std::stof(val);
        else if (key == "driverTempMax") s_config.driverTempMax = std::stof(val);
        else if (key == "networkWlan0Ip") s_config.networkWlan0Ip = val;
        else if (key == "networkWlan1Ip") s_config.networkWlan1Ip = val;
        else if (key == "networkEnp1s0Ip") s_config.networkEnp1s0Ip = val;
        else if (key == "networkEth0Ip") s_config.networkEth0Ip = val;
        else if (key == "logPath") s_config.logPath = val;
        else if (key == "logRetentionDays") s_config.logRetentionDays = std::stoi(val);
        else if (key == "gatewayIp") s_config.gatewayIp = val;
        else if (key == "gatewayPort") s_config.gatewayPort = std::stoi(val);
    }

    s_loaded = true;
    return true;
}

bool TestConfigManager::saveToFile(const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }

    file << "# Dog Tool Production Test Configuration\n\n";
    file << "[SN]\n";
    file << "snStrictMode=" << (s_config.snStrictMode ? "1" : "0") << "\n";
    file << "snMinLen=" << s_config.snMinLen << "\n";
    file << "snMaxLen=" << s_config.snMaxLen << "\n\n";

    file << "[SBUS]\n";
    file << "sbusMinChannel=" << s_config.sbusMinChannel << "\n";
    file << "sbusMaxChannel=" << s_config.sbusMaxChannel << "\n";
    file << "sbusTimeoutMs=" << s_config.sbusTimeoutMs << "\n";
    file << "sbusTestData=" << s_config.sbusTestData << "\n\n";

    file << "[IMU]\n";
    file << "imuTempMin=" << s_config.imuTempMin << "\n";
    file << "imuTempMax=" << s_config.imuTempMax << "\n\n";

    file << "[BMS]\n";
    file << "bmsVoltageMin=" << s_config.bmsVoltageMin << "\n";
    file << "bmsVoltageMax=" << s_config.bmsVoltageMax << "\n";
    file << "bmsMinCapacityMah=" << s_config.bmsMinCapacityMah << "\n\n";

    file << "[Motor]\n";
    file << "motorTempMax=" << s_config.motorTempMax << "\n";
    file << "driverTempMax=" << s_config.driverTempMax << "\n\n";

    file << "[Network]\n";
    file << "networkWlan0Ip=" << s_config.networkWlan0Ip << "\n";
    file << "networkWlan1Ip=" << s_config.networkWlan1Ip << "\n";
    file << "networkEnp1s0Ip=" << s_config.networkEnp1s0Ip << "\n";
    file << "networkEth0Ip=" << s_config.networkEth0Ip << "\n\n";

    file << "[Gateway]\n";
    file << "gatewayIp=" << s_config.gatewayIp << "\n";
    file << "gatewayPort=" << s_config.gatewayPort << "\n\n";

    file << "[Log]\n";
    file << "logPath=" << s_config.logPath << "\n";
    file << "logRetentionDays=" << s_config.logRetentionDays << "\n";

    return true;
}
