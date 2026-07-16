#include <cstdint>
#include <cstring>
#include "mcu_soc_com_protocol.h"
// 协议头定义
typedef struct {
    uint16_t header;                     // 帧头固定0xAA55
    uint8_t  frame_id;                   // 帧ID（递增）
    uint8_t  version;                    // 协议版本，如0x01
    uint8_t  cmd;                        // 指令类型（见下文枚举）
    uint8_t  packet_num;                 // 分包号（单包为0）
    uint16_t data_len;                   // buffer数据长度
    uint16_t crc_sum;                    // 校验和（header+data）
} __attribute__((packed)) CS_DataHeader; // 取消字节对齐

// 完整通信帧
typedef struct {
    CS_DataHeader header;
    uint8_t       buffer[ 1024 ];
} __attribute__((packed)) ComFrame;

// 指令类型枚举（cmd字段值）
enum TestCmd : uint8_t {
    CMD_CONNECT             = 0x01, // 连接测试
    CMD_DISCONNECT          = 0x02, // 断开连接
    CMD_TEST_ALL_BASE       = 0x03, // 主控盒基础全测
    CMD_TEST_RTC            = 0x04, // RTC测试
    CMD_TEST_BLUETOOTH      = 0x05, // 蓝牙测试
    CMD_TEST_MCU            = 0x06, // MCU测试
    CMD_TEST_IMU            = 0x07, // IMU测试
    CMD_TEST_BMS            = 0x08, // BMS测试
    CMD_TEST_NETWORK        = 0x09, // 网络测试
    CMD_TEST_USB            = 0x0A, // USB测试
    CMD_TEST_LED            = 0x0B, // LED测试
    CMD_TEST_SBUS           = 0x0C, // SBUS测试
    CMD_TEST_SN             = 0x0D, // SN码读写
    CMD_TEST_MOTOR_CONN    = 0x0E, // 电机连接
    CMD_TEST_MOTOR_SN      = 0x0F, // 电机SN读写
    CMD_TEST_MOTOR_VERSION = 0x10, // 电机版本读取
    CMD_TEST_MOTOR_CONTROL = 0x11, // 电机控制
    CMD_TEST_MOTOR_ENCODER = 0x12, // 编码器校准
    CMD_TEST_MOTOR_NTC     = 0x13, // NTC读取
    CMD_DISABLE_AUTOSTART   = 0xFE, // 禁用服务自启
    CMD_RESPONSE            = 0xFF  // 响应指令（服务端回传）
};

// CRC16校验函数（用于协议校验）
static uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[ i ] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 封装通信帧
ComFrame CS_pack_frame(uint8_t cmd, uint8_t frame_id, const uint8_t *data, uint16_t data_len) {
    ComFrame frame;
    memset(&frame, 0, sizeof(ComFrame));
    // 填充协议头
    frame.header.header     = 0xAA55;
    frame.header.frame_id   = frame_id;
    frame.header.version    = 0x01;
    frame.header.cmd        = cmd;
    frame.header.packet_num = 0;
    frame.header.data_len   = data_len;
    // 填充数据
    if (data && data_len > 0) {
        memcpy(frame.buffer, data, data_len);
    }
    // 计算CRC（头+数据）
    uint8_t crc_data[ sizeof(DataHeaderMM) + data_len ];
    memcpy(crc_data, &frame.header, sizeof(DataHeaderMM));
    memcpy(crc_data + sizeof(DataHeaderMM), frame.buffer, data_len);
    frame.header.crc_sum = crc16(crc_data, sizeof(DataHeaderMM) + data_len);
    return frame;
}

// 解析通信帧（校验+解包）
bool CS_unpack_frame(const ComFrame &frame, DataHeaderMM &header, uint8_t *buffer,
                     uint16_t &buf_len) {
    // 校验帧头
    if (frame.header.header != 0xAA55)
        return false;
    // 校验CRC
    uint8_t crc_data[ sizeof(DataHeaderMM) + frame.header.data_len ];
    memcpy(crc_data, &frame.header, sizeof(DataHeaderMM));
    memcpy(crc_data + sizeof(DataHeaderMM), frame.buffer, frame.header.data_len);
    uint16_t calc_crc = crc16(crc_data, sizeof(DataHeaderMM) + frame.header.data_len);
    if (calc_crc != frame.header.crc_sum)
        return false;
    // 解包
    header  = frame.header;
    buf_len = frame.header.data_len;
    if (buf_len > 0) {
        memcpy(buffer, frame.buffer, buf_len);
    }
    return true;
}
