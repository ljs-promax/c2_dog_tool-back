#ifndef WS2812_CONTROL_HPP
#define WS2812_CONTROL_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

enum class LedMode {
    OFF,  //关闭
    STATIC,  //静态
    BREATHE,  //呼吸
    CYCLE,   //流水
    BLINK   //闪烁
};

struct RgbColor { uint8_t r, g, b; };
struct HsvColor { float h, s, v; };

struct LedState {
    LedMode mode = LedMode::OFF;
    std::vector<RgbColor> colors; // 用户定义的每个灯珠的颜色
    float brightness = 1.0f;      // 全局亮度 (0.0 - 1.0)
    int period_ms = 1000;         // 动画周期 (毫秒)，用于 BREATHE 和 CYCLE
};

class WS2812 {
public:
    WS2812(const std::string& device);
    ~WS2812();

    static const int MAX_LEDS = 64;

    // 启动后台渲染线程，必须在开始时调用
    bool start();

    // 停止后台线程并清理资源
    void stop();
    
    // --- 用户调用的唯一核心接口 ---
    void setState(const LedState& newState);

    // 测试 SPI 写入是否成功（同步调用）
    bool testWrite();

    //获取当前状态，方便读取
    LedState getState();
    
    // 获取LED数量
    int getNumLeds() const;

private:
    // 禁止拷贝构造和赋值
    WS2812(const WS2812&) = delete;
    WS2812& operator=(const WS2812&) = delete;

    // 后台渲染线程的循环函数
    void renderLoop();

    // --- 内部状态变量 ---
    std::mutex mtx; // 互斥锁，保护整个状态结构体
    LedState currentState;
    
    // --- 硬件和线程管理 ---
    std::thread renderer_thread;
    std::atomic<bool> is_running;
    int timer_fd;   // timerfd 文件描述符
    int epoll_fd;   // epoll 文件描述符
    int event_fd;   // 用于通知退出的 eventfd
    
    int spi_fd;
    const int num_leds;
    const std::string device_path;
    std::vector<uint8_t> led_buffer;

    bool initializeSpi();
    void setPixel(int index, RgbColor color);
    void clear();
    bool show();
    void colorToSpi(uint8_t g, uint8_t r, uint8_t b, std::vector<uint8_t>& buffer);
};

RgbColor hsvToRgb(HsvColor hsv);
HsvColor rgbToHsv(RgbColor rgb);

#endif

