#include "ws2812_control.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

const uint32_t SPI_SPEED_HZ = 6400000;
const uint8_t BITS_PER_WORD = 8;
const uint8_t WS2812_CODE_0 = 0xC0;
const uint8_t WS2812_CODE_1 = 0xF0;
const int WS2812::MAX_LEDS;


HsvColor rgbToHsv(RgbColor rgb) {
    HsvColor hsv;
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;

    float max_val = std::max({r, g, b});
    float min_val = std::min({r, g, b});
    float delta = max_val - min_val;

    hsv.v = max_val;

    if (max_val == 0) { // 如果是黑色
        hsv.s = 0;
        hsv.h = 0;
    } else {
        hsv.s = delta / max_val;
        if (delta == 0) { // 如果是灰度色
            hsv.h = 0;
        } else {
            if (r == max_val) hsv.h = 60 * fmod(((g - b) / delta), 6);
            else if (g == max_val) hsv.h = 60 * (((b - r) / delta) + 2);
            else hsv.h = 60 * (((r - g) / delta) + 4);
        }
    }
    if (hsv.h < 0) hsv.h += 360;
    
    return hsv;
}

RgbColor hsvToRgb(HsvColor hsv) {
    RgbColor rgb;
    float r, g, b;
    int i = static_cast<int>(hsv.h / 60) % 6;
    float f = hsv.h / 60.0f - i;
    float p = hsv.v * (1 - hsv.s);
    float q = hsv.v * (1 - f * hsv.s);
    float t = hsv.v * (1 - (1 - f) * hsv.s);

    switch (i) {
        case 0: r = hsv.v; g = t; b = p; break;
        case 1: r = q; g = hsv.v; b = p; break;
        case 2: r = p; g = hsv.v; b = t; break;
        case 3: r = p; g = q; b = hsv.v; break;
        case 4: r = t; g = p; b = hsv.v; break;
        default: r = hsv.v; g = p; b = q; break;
    }
    rgb.r = static_cast<uint8_t>(r * 255);
    rgb.g = static_cast<uint8_t>(g * 255);
    rgb.b = static_cast<uint8_t>(b * 255);
    return rgb;
}

WS2812::WS2812(const std::string& device)
    : spi_fd(-1),
      num_leds(MAX_LEDS),
      device_path(device),
      is_running(false),
      timer_fd(-1),
      epoll_fd(-1),
      event_fd(-1) {
      led_buffer.resize(num_leds * 3, 0);
}

WS2812::~WS2812(){
    stop();
}

bool WS2812::start(){
    if(!initializeSpi())
        return false;
    timer_fd = timerfd_create(CLOCK_MONOTONIC ,TFD_NONBLOCK);//使用系统启动以来的单调时钟,非阻塞
    if(timer_fd == -1){
        perror("timerfd_create");
        return false;
    }
    struct itimerspec its;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 16666667; // ~60fps  周期性触发间隔
    its.it_value = its.it_interval;
    //启动定时器
    if(timerfd_settime(timer_fd, 0, &its, NULL) == -1) {
        perror("timerfd_settime"); 
        close(timer_fd); 
        return false; 
    }
    //轻量级事件通知
    event_fd = eventfd(0, EFD_NONBLOCK);
    if (event_fd == -1) { 
        perror("eventfd"); 
        close(timer_fd); 
        return false; 
    }
    //实例化  用于同时监听 timer_fd 和 event_fd
    epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) { 
        perror("epoll_create1"); 
        close(timer_fd); 
        close(event_fd); 
        return false; 
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = timer_fd;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &ev);
    ev.data.fd = event_fd;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, event_fd, &ev);
    is_running = true;
    renderer_thread = std::thread(&WS2812::renderLoop, this);
    return true;
}

void WS2812::stop(){
     if (is_running.exchange(false)) {//把值设为false，并返回原来的值
        if(event_fd != -1) {
            uint64_t u = 1;
            write(event_fd, &u, sizeof(uint64_t));
        }
        if (renderer_thread.joinable()) {
            renderer_thread.join();
        }
        if (epoll_fd != -1) { close(epoll_fd); epoll_fd = -1; }
        if (timer_fd != -1) { close(timer_fd); timer_fd = -1; }
        if (event_fd != -1) { close(event_fd); event_fd = -1; }
        if (spi_fd >= 0) {
            clear(); show(); usleep(10000); close(spi_fd);
            spi_fd = -1;
        }
    }
}

void WS2812::setState(const LedState& newState) {
    std::lock_guard<std::mutex> lock(mtx);
    currentState = newState;
    currentState.brightness = std::max(0.0f, std::min(1.0f, currentState.brightness));
    currentState.period_ms = std::max(100, currentState.period_ms);
}

LedState WS2812::getState(){
    std::lock_guard<std::mutex> lock(mtx);
    return currentState;
}

int WS2812::getNumLeds() const {
    return num_leds;
}

void WS2812::renderLoop(){
    struct epoll_event events[1]; //接收 epoll 监听到的事件
    uint64_t timer_expirations; //存放 timerfd 的触发次数。
    auto start_time = std::chrono::steady_clock::now();//记录起始时间，用于计算呼吸灯、彩虹循环等的进度

    while(is_running){
        int nfds = epoll_wait(epoll_fd, events, 1, -1);
        if (nfds <= 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (events[0].data.fd == event_fd) break;
        if (events[0].data.fd == timer_fd){
            read(timer_fd, &timer_expirations, sizeof(uint64_t));
            LedState state = getState();
            int num_colors_to_set = std::min(num_leds, (int)state.colors.size());
            clear();
            if (state.mode != LedMode::OFF && num_colors_to_set > 0) {
                switch (state.mode){
                    case LedMode::OFF: clear();break;
                    case LedMode::STATIC:{
                        for (int i = 0; i < num_colors_to_set; ++i) {
                            RgbColor c = state.colors[i];
                            HsvColor hsv = rgbToHsv(c);
                            hsv.v *= state.brightness;
                            setPixel(i, hsvToRgb(hsv)); // 2. 只设置用户指定的那些灯
                        }
                        break;
                    }
                    case LedMode::BREATHE:{
                        float elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
                        float wave = (sin((elapsed_ms / state.period_ms) * 2.0f * M_PI) + 1.0f) / 2.0f;
                        float final_brightness = wave * state.brightness;
                        for (int i = 0; i < num_colors_to_set; ++i) {
                            RgbColor c = state.colors[i];
                            HsvColor hsv = rgbToHsv(c);
                            hsv.v *= final_brightness;
                            setPixel(i, hsvToRgb(hsv));
                        }
                        break;
                    }
                    case LedMode::CYCLE:{
                        float progress = fmod(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count(), state.period_ms);
                        float cycle_offset_float = progress / state.period_ms;
                        int color_offset = static_cast<int>(cycle_offset_float * num_colors_to_set);
                        for (int i = 0; i < num_colors_to_set; i++) {
                            int color_index = (i + color_offset) % num_colors_to_set;
                            RgbColor c = state.colors[color_index];
                            HsvColor hsv = rgbToHsv(c);
                            hsv.v *= state.brightness;
                            // 将流动后的颜色设置到对应的灯珠上
                            setPixel(i, hsvToRgb(hsv));
                        }
                        break;
                    }
                    case LedMode::BLINK: { 
                        // 1. 计算当前时间点在周期内的位置
                        float elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
                        
                        // 2. 一个周期分为两半：一半时间亮，一半时间灭
                        float progress_in_period = fmod(elapsed_ms, state.period_ms);

                        // 3. 如果当前时间在周期的前半段，则点亮
                        if (progress_in_period < (state.period_ms / 2.0f)) {
                            for (int i = 0; i < num_colors_to_set; ++i) {
                                RgbColor c = state.colors[i];
                                HsvColor hsv = rgbToHsv(c);
                                hsv.v *= state.brightness; // 应用全局亮度
                                setPixel(i, hsvToRgb(hsv));
                            }
                        }
                        break;
                    }
                }
            }
            show();
        }
    }
}

// === 底层私有函数 ===
bool WS2812::initializeSpi() {
    spi_fd = open(device_path.c_str(), O_RDWR);
    if (spi_fd < 0) { 
        perror(("无法打开SPI: " + device_path).c_str()); 
        return false;
    }
    uint8_t mode = SPI_MODE_0;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) { 
        perror("无法设置SPI模式"); close(spi_fd); 
        return false; 
    }
    uint8_t bits = BITS_PER_WORD;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) { 
        perror("无法设置字长"); close(spi_fd); 
        return false; 
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ) == -1) { 
        perror("无法设置SPI速度"); close(spi_fd); 
        return false; 
    }
    std::cout << "SPI初始化成功: " << device_path << std::endl;

    std::vector<uint8_t> reset_buf(400, 0); // 500µs reset
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)reset_buf.data();
    tr.len = reset_buf.size();
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = BITS_PER_WORD;
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    usleep(1000);
    return true;
}

void WS2812::setPixel(int index, RgbColor color) {
    if (index < 0 || index >= num_leds) return;
    int base = index * 3;
    led_buffer[base] = color.g;
    led_buffer[base + 1] = color.r;
    led_buffer[base + 2] = color.b;
}

void WS2812::clear() {
    std::fill(led_buffer.begin(), led_buffer.end(), 0);
}

bool WS2812::testWrite() {
    // 尝试向 SPI 写入一个最小的测试帧
    // 如果 LED 断开或 SPI 出错，ioctl 会返回 -1
    std::vector<uint8_t> spi_data;
    spi_data.reserve(256);
    spi_data.insert(spi_data.begin(), 50, 0);  // 起始脉冲
    spi_data.insert(spi_data.end(), 50, 0);       // 结束脉冲
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)spi_data.data();
    tr.len = spi_data.size();
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = BITS_PER_WORD;
    return (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 1);
}

bool WS2812::show() {
    std::vector<uint8_t> spi_data;
    spi_data.reserve(num_leds * 24 + 400);
    spi_data.insert(spi_data.begin(), 250, 0);
    for (int i = 0; i < num_leds; ++i) {
        colorToSpi(led_buffer[i * 3], led_buffer[i * 3 + 1], led_buffer[i * 3 + 2], spi_data);
    }
    spi_data.insert(spi_data.end(), 280, 0);
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)spi_data.data();
    tr.len = spi_data.size();
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = BITS_PER_WORD;
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) { 
        return false; 
    }
    return true;
}

void WS2812::colorToSpi(uint8_t g, uint8_t r, uint8_t b, std::vector<uint8_t>& buffer) {
    uint32_t color = (static_cast<uint32_t>(g) << 16) | (static_cast<uint32_t>(r) << 8) | b;
    for (int i = 23; i >= 0; --i) {
        if ((color >> i) & 1) buffer.push_back(WS2812_CODE_1);
        else buffer.push_back(WS2812_CODE_0);
    }
}



