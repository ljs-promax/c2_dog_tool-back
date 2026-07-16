#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <endian.h>

#pragma pack(push, 1)

// 基础协议头 (RD 和 BH 共用结构)
// struct DataHeaderMM {
//   uint16_t magic; // 0x4248 (BH-关节) 或 0x5244 (RD-MCU)
//   uint8_t frame_id;
//   uint8_t version;
//   uint8_t cmd;
//   uint8_t packet_num;
//   uint16_t data_len;
//   uint16_t crc_sum;
// };

// 关节专用：CAN 头部封装
struct CANHeaderPacked {
    uint8_t  board_id;
    uint8_t  can_ch;
    uint32_t id_word;    // 包含 ID 和时间戳标志
    uint32_t props_word; // 包含 DLC 和各种标志
};

// MCU 专用：OTA 响应载荷
struct McuResponsePayload {
    uint8_t  result;    // 0x00 为成功
    uint32_t write_pos; // 设备已写入的偏移位置
};

#pragma pack(pop)

// 命令码定义
namespace Cmd {
// MCU (RD协议)
static constexpr uint8_t MCU_GET_VER   = 0xF4;
static constexpr uint8_t MCU_OTA_START = 0xE3;
static constexpr uint8_t MCU_OTA_DATA  = 0xE4;
static constexpr uint8_t MCU_OTA_END   = 0xE5;

// 关节 (BH协议)
static constexpr uint8_t JOINT_CAN_TRANS = 0xA0;
} // namespace Cmd

// 魔数定义
namespace Magic {
static constexpr uint16_t BH_JOINT = 0x4248; // "BH"
static constexpr uint16_t RD_MCU   = 0x5244; // "RD"
} // namespace Magic

// --- 新增这个部分 ---
namespace DefaultConfig {
static const char *GATEWAY_IP = "10.21.32.121";
static constexpr uint8_t GATEWAY_BOARD_ID = 121; // Gateway IP last octet.
// static const char *GATEWAY_IP = "10.21.20.97";
static constexpr int REMOTE_PORT = 43893;
} // namespace DefaultConfig

#endif
