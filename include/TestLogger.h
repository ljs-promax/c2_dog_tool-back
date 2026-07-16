#ifndef TEST_LOGGER_H
#define TEST_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>

struct TestLogEntry {
    std::chrono::system_clock::time_point timestamp;
    std::string testName;
    bool passed;
    std::string message;
    std::string detail;
};

class TestLogger {
public:
    static TestLogger& instance();

    void log(const TestLogEntry& entry);
    void logTestResult(const std::string& name, bool passed,
                       const std::string& msg, const std::string& detail = "");

    // Auto-disable conflicting programs
    bool stopConflictingServices();
    bool restartConflictingServices();
    void setConflictingServicesPinnedStopped(bool pinned);
    bool isConflictingServicesPinnedStopped() const;

    // Flush and close
    void flush();

private:
    TestLogger();
    ~TestLogger();
    std::string getLogFilePath();
    bool ensureLogDir();

    std::ofstream m_file;
    std::mutex m_mutex;
    bool m_initialized;
    bool m_conflictingServicesPinnedStopped;
};

#endif // TEST_LOGGER_H
