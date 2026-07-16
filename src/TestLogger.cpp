#include "TestLogger.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <unistd.h>
#include <cstring>

TestLogger& TestLogger::instance() {
    static TestLogger logger;
    return logger;
}

TestLogger::TestLogger()
    : m_initialized(false), m_conflictingServicesPinnedStopped(false) {
}

TestLogger::~TestLogger() {
    if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
    }
}

bool TestLogger::ensureLogDir() {
    if (m_initialized) return true;

    // /var/log/dog_tool/
    const char* logDir = "/var/log/dog_tool";
    mkdir(logDir, 0755);

    std::lock_guard<std::mutex> lock(m_mutex);

    m_file.open(getLogFilePath(), std::ios::app);
    if (m_file.is_open()) {
        m_initialized = true;
        return true;
    }

    // Fallback: try current directory
    m_file.open("dog_tool_test.log", std::ios::app);
    if (m_file.is_open()) {
        m_initialized = true;
        return true;
    }

    return false;
}

std::string TestLogger::getLogFilePath() {
    char buf[256];
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::strftime(buf, sizeof(buf), "/var/log/dog_tool/prod_test_%Y%m%d.log", &tm_buf);
    return std::string(buf);
}

void TestLogger::log(const TestLogEntry& entry) {
    if (!ensureLogDir()) {
        std::cerr << "TestLogger: failed to open log file" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    auto t = std::chrono::system_clock::to_time_t(entry.timestamp);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif

    m_file << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
           << " | " << entry.testName
           << " | " << (entry.passed ? "PASS" : "FAIL")
           << " | " << entry.message;

    if (!entry.detail.empty()) {
        m_file << " | " << entry.detail;
    }
    m_file << "\n";
    m_file.flush();
}

void TestLogger::logTestResult(const std::string& name, bool passed,
                                const std::string& msg, const std::string& detail) {
    TestLogEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.testName = name;
    entry.passed = passed;
    entry.message = msg;
    entry.detail = detail;
    log(entry);
}

void TestLogger::flush() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file.is_open()) {
        m_file.flush();
    }
}

bool TestLogger::stopConflictingServices() {
    if (m_conflictingServicesPinnedStopped) {
        return true;
    }

    int ret = system("systemctl stop bpx_motion.service 2>/dev/null");
    return (ret == 0);
}

bool TestLogger::restartConflictingServices() {
    if (m_conflictingServicesPinnedStopped) {
        return true;
    }

    int ret = system("systemctl start bpx_motion.service 2>/dev/null");
    return (ret == 0);
}

void TestLogger::setConflictingServicesPinnedStopped(bool pinned) {
    m_conflictingServicesPinnedStopped = pinned;
}

bool TestLogger::isConflictingServicesPinnedStopped() const {
    return m_conflictingServicesPinnedStopped;
}
