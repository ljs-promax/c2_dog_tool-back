/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-16     17511       the first version
 */
/*
    *本协议说明 soc 和MCU 之间以及驱动器之间的通信协议说明
    *MCU 支持被SOC 查询一些状态，并会转发can 有关数据到驱动器中。
    *如下是通信协议主体结构 DataHeaderMM + buffer[N 个 Net2CanFrame{board_id+can_ch+can_headr+data}
或直接给到MCU的数据内容 ] ┌─────────────────────────────────────────────────────────────
CanFdNetBuffer 顶层网络大包 │  ┌──────────────────────────────────── DataHeaderMM 网络包头（控制域）
│  │  [uint16] header      帧同步魔数
│  │  [uint8]  frame_id    请求应答匹配ID
│  │  [uint8]  version     协议版本
│  │  [uint8]  cmd         业务指令码
│  │  [uint8]  packet_num  分片序号
│  │  [uint16] data_len    负载buffer有效总字节
│  │  [uint16] crc_sum     整包CRC16校验
│  └────────────────────────────────────
│
│  ┌──────────────────────────────────── buffer[1024] 负载区
│  │  连续存放 N 条 Net2CanFrame 单CAN子帧
│  │  ┌───────────────────────────────── Net2CanFrame 单条CANFD子帧(固定70Byte)
│  │  │  ┌────────────────────────── CANHeader CAN底层帧头(固定10Byte)
│  │  │  │ [uint8] board_id      转接板设备ID(IP末位)
│  │  │  │ [uint8] can_ch        CAN硬件通道号
│  │  │  │  位域组1：29bit CANID + 保留位 + 时间戳使能标记
│  │  │  │  位域组2：DLC编码、CANFD/RTR/扩展ID/波特率切换标志
│  │  │  └──────────────────────────
│  │  │  [uint8] data[64]        CAN报文数据负载，有效长度由DLC查表转换
│  │  └─────────────────────────────────
│  └────────────────────────────────────
└─────────────────────────────────────────────────────────────

*/

#ifndef MCU_SOC_COM_PROTOCOL_H_
#define MCU_SOC_COM_PROTOCOL_H_
#include <stdint.h>
#pragma pack(2)

enum Version {
    kProtocolVersion = 1,
    kEtherCANVersion = 1,
    kHardwareVersion = 1,

};

enum UDPCommandIndex {

    // to Drivers
    kBpx2Driver = 0xA0, // used: Bpx2Drivers 命令字 关节协议命令 由MCU 收到到解析分发到驱动器

    // to MCU
    kMCUReBoot = 0xF1, // unused: 重启MCU 的指令

    // 即可以主动上报，也可查询后上报
    kGetAppVersion = 0xF4, // used: 获取MCU 版本号
    kGetMcustatus  = 0xF5, // used: 获取MCU 状态
    KMcuSensorImu  = 0xE1, // used: MCU 传感器IMU数据
    kMcuSensorBat  = 0xE2, // used: MCU 传感器电池数据

    /* 新增 OTA 命令 */
    kOTAStart = 0xE3, // used: 准备升级：包含总大小和总校验和
    kOTAData  = 0xE4, // used: 传输数据：buffer 里的固件切片
    kOTAEnd   = 0xE5, // used: 结束升级：触发重启
};
// 未解析，未匹配，发什么都可以
enum CommandHeader {
    kCommandHeader     = 0x5244, // MCU2SOC   ota_ack  kGetAppVersion  kGetMcustatus
    kKeyData2BpxHeader = 0x4248, // IMU2Bpx  can2net

    kBpx2DriverHeader = 0x3344 // battery2soc  bpx2mcu
};

#define SENS_BIT_ESTOP (1U << 0)  // bit0 急停按键 1 正常 0 被按下
#define SENS_BIT_IMU_OK (1U << 1) // bit1 IMU正常 1 正常 0 异常
#define SENS_BIT_BAT_OK (1U << 2) // bit2 电池正常 1 正常 0 异常

typedef struct {
    uint32_t can_status[ 4 ]; // can0-3 总线异常恢复的次数
    uint16_t
        version; // /* 修改版本号只需修改此宏 0xabcd  a: 协议版本  b: 主版本号  cd: 修订版本号   */
    uint8_t sensors_status; // bit0 stop键状态 bit1:imu是否正常 bit2：battery是否正常
    uint8_t reserverd;
} McuStatus;

typedef struct {
    float    voltage_V;              ///< 总电压 (V, float32)
    float    current_A;              ///< 总电流 (A, 正值充电/负值放电)
    uint16_t remaining_capacity_mAh; ///< 剩余容量 (mAh)
    uint16_t nominal_capacity_mAh;   ///< 额定容量 (mAh)
    uint16_t cycles;                 ///< 循环次数
    uint16_t _reserved_padding;      ///< 保留: 使 protected_state 4字节对齐，禁止赋值
    uint32_t protected_state;        ///< 保护状态位掩码
    float    temp_soc_C;             ///< SOC 温度 (℃)
    float    temp_ntc1_C;            ///< NTC1 温度 (℃)
    float    temp_ntc2_C;            ///< NTC2 温度 (℃)
    float    temp_air_C;             ///< 环境温度 (℃)
    float    temp_fet_C;             ///< MOS管温度 (℃)
    uint8_t  battery_level_percent;  ///< SOC 电量百分比 (0-100)
    char     software_version[ 8 ];  ///< 软件版本字符串
    uint8_t  battery_quantity;       ///< 电芯数量
    char     serial_number[ 48 ];    ///< 设备序列号
    char     hardware_version[ 8 ];  ///< 硬件版本字符串
    uint16_t protect_counts[ 14 ];   ///< 各保护项触发计数
    uint8_t  charger_in1;            ///< 充电口1状态 (0=未插/1=已插)
    uint8_t  charger_in2;            ///< 充电口2状态 (0=未插/1=已插)
} BmsFeedback;

typedef struct {
    float    qx, qy, qz, qw;
    float    wx, wy, wz; // rad/s
    float    ax, ay, az; // m/s^2
    uint32_t timestamp;
} ImuFeedback;

/**
 * @brief 网络大包头部控制域（CanFdNetBuffer 帧头，固定长度）
 * 用于上位机 ↔ CAN转接板MCU 之间的网络数据包校验、分包、指令区分、版本兼容
 */
typedef struct {
    uint16_t header;     /**< 帧头魔数，用于帧同步，区分合法数据包 */
    uint8_t  frame_id;   /**< 帧流水ID，应答帧需回填相同ID，实现请求-应答匹配 */
    uint8_t  version;    /**< 通信协议版本号，用于多版本上位机兼容 */
    uint8_t  cmd;        /**< 业务指令码：下发CAN发送/上报CAN接收/设备查询/配置等 */
    uint8_t  packet_num; /**< 分包序号，大数据包多段拆分传输时标记分片编号 */
    uint16_t data_len;   /**< 后续buffer有效数据总字节长度，代表内部存放多少条Net2CanFrame */
    uint16_t crc_sum;    /**< CRC16校验，校验「header+buffer」全部数据，防止传输错误 */
} DataHeaderMM;
/**
 * @brief 网络完整传输大包（上位机 <--> CAN转接板MCU 顶层帧）
 * 完整一包 = 控制头DataHeaderMM + 承载多条CANFD子帧的缓冲区
 * 用途：上位机批量下发多组CAN报文、转接板批量上报收到的CAN报文
 */
typedef struct {
    DataHeaderMM header;    /**< 网络包控制头部，含校验、指令、长度、版本 */
    uint8_t buffer[ 1024 ]; /**< 负载缓冲区，内部连续存放 N 条 Net2CanFrame CAN子帧；
                                 有效长度由上层header.data_len指定，超出部分丢弃 */
} CanFdNetBuffer;
/**
 * @brief CANFD单路子帧头部联合体（固定10字节）
 * 描述单条CAN报文的硬件通道、ID、帧类型、长度、时间戳使能等底层属性
 * 联合体buffer[10]用于直接二进制序列化收发，内部结构体方便按字段读写
 */
typedef union can_header {
    uint8_t buffer[ 10 ]; /**< 原始字节数组，可直接拷贝到串口/CAN发送缓冲区 */
    struct {
        uint8_t board_id; /**< CAN转接板设备标识，对应设备IP最后1字节，多转接板组网区分 */
        uint8_t can_ch;   /**< CAN通道号：0/1/2... 区分转接板多路CAN硬件通道 */
        struct {
            uint32_t id : 29; /**< 29bit CAN扩展ID，标准CAN2.0/CANFD共用 ！！！！！！！！！！！！*/
            uint32_t : 2;     /**< 保留占位bit，无实际用途，填充对齐32bit */
            uint32_t transmit_timestamp_enable
                : 1; /**< 发送时间戳使能：1=硬件记录帧收发时间戳附加上报 */
        };
        struct {
            uint32_t dlc
                : 4; /**< CAN
                        DLC长度编码(0~15)，通过dlc_to_len查表转为真实数据字节0/1/2/3/4/5/6/7/8/12/16/20/24/32/48/64
                        ！！！！！！！！！*/
            uint32_t bitrate_switch
                : 1; /**< CANFD波特率切换标志：1=数据段使用高速波特率，0=全程基础波特率 */
            uint32_t canfd_frame
                : 1; /**< CANFD帧标记：1=CANFD帧(支持64字节数据)，0=标准CAN2.0(最多8字节) */
            uint32_t remote_frame : 1; /**< 远程请求帧RTR：1=远程帧无数据段，0=数据帧带有效数据 */
            uint32_t extend_id : 1;    /**< 扩展ID标记：1=29bit扩展ID，0=11bit标准ID */
            uint32_t : 24;             /**< 保留占位bit，填充对齐32bit */
        };
    };
} CANHeader;

/**
 * @brief 单条CANFD完整子帧（70字节固定长度，存放在CanFdNetBuffer.buffer内）
 * 一条结构体对应总线上一帧CAN/CANFD报文；
 * 联合体raw_buffer用于整块二进制收发，内部结构体分离CAN头与数据区，便于字段操作
 * ===================== CAN 业务协议完整说明（SDO/PDO/升级/心跳）=====================
 * 本协议基于 CANopen 深度定制，不支持标准CiA 402对象字典；分为三大业务类型：
 * 【1】SDO 服务数据对象（参数配置、读写，带应答确认）
 *     用途：驱动器参数读写、设备配置，一问一答保证可靠传输
 *     1.1 主站 -> 从节点(SDO请求)
 *         CAN ID  ：0x600 + NodeID  | DLC：8
 *         数据域 ：data[0]=CS命令符 | data[1~2]=主索引 | data[3]=子索引(固定0x00) |
 * data[4~7]=读写数据 1.2 从节点 -> 主站(SDO应答) CAN ID  ：0x580 + NodeID  | DLC：8 数据域
 * ：同请求帧格式 1.3 CS命令符定义 读请求：0x40
 *         读响应：0x4F(1字节)/0x4B(2字节)/0x47(3字节)/0x43(4字节)
 *         写请求：0x2F(1字节)/0x2B(2字节)/0x27(3字节)/0x23(4字节)
 *         写成功应答：0x60
 *         异常响应：0x80
 *     1.4 常用主索引(参数)
 *         0x2000 : error_code 故障码(u16, RO)
 *         0x2002 : control_word 控制字(u16, RW) 0x0001使能/0x0002失能/0x00FF清故障/0x00F1编码器校准
 *         0x2040 : node_id 节点ID(u8, RW)
 *         0x2043/0x2044 : 心跳开关(u16, RW)
 *         0x205B : torque_limit 力矩限制(f32)
 *         0x2060/0x2061 : 过温阈值(f32)
 *         0x2070 : in_encoder_offset 编码器标零(u16)
 *         0x2100 : firmware_version 固件版本(u16, RO)
 *         0x2104-210A : sn 序列号(u8[4], RO)
 *
 * 【2】PDO 过程数据对象（实时运动控制、状态上报，高速高频传输）
 *     用途：电机实时控制、运行状态上报，时效性优先
 *     2.1 PDO1 MIT模式控制
 *         下发(TPDO)：CAN ID=0x400+NodeID  DLC=16
 *         数据域：目标位置(f32) + 目标速度(f32) + 前馈力矩(f32) + KP(u16) + KD(u16)
 *         上报(RPDO)：CAN ID=0x190+NodeID
 *           - 无故障：DLC=16  当前位置/速度/力矩/电机温度/驱动器温度
 *           - 有故障：DLC=20  额外增加 error_code 故障码
 *     2.2 PDO2 直接运动控制（自动使能电机）
 *         帧格式同 PDO1
 *
 * 【3】固件升级（统一ID：0x680 + NodeID）
 *     阶段1：发送升级开始命令(DLC=4, 0xDDDDDDDD)
 *     阶段2：循环传输固件数据(每帧8字节，不足8字节用0xFF补齐)
 *     阶段3：发送升级完成命令(DLC=4, 0xFFFFFFFF)
 *
 * 【4】心跳机制（默认开启，周期1s）
 *     - 主站发包：CAN ID=0x700  DLC=1  data[0]=0x05
 *     - 从机应答：CAN ID=0x700+NodeID  DLC=1  data[0]=0x05
 *     - 断线判定：2.5s未收到心跳 → 驱动器失能，需重新上电恢复
 *
 * ===================== 常用测试指令示例(默认NodeID=1) =====================
 * 1. 设置节点ID为2
 *    下发：ID=0x601 DLC=8  DATA:2F 40 20 00 02 00 00 00
 *    应答：ID=0x581 DLC=8  DATA:60 40 20 00 00 00 00 00
 * 2. 电机使能
 *    下发：ID=0x601 DLC=8  DATA:2B 02 20 00 01 00 00 00
 *     应答：ID=0x581 DLC=8  DATA:60 02 20 00 00 00 00 00
 * 3. 电机失能
 *    下发：ID=0x601 DLC=8  DATA:2B 02 20 00 02 00 00 00
 *     应答：ID=0x581 DLC=8  DATA:60 02 20 00 00 00 00 00
 * 4. 编码器校准
 *    下发：ID=0x601 DLC=8  DATA:2B 02 20 00 F1 00 00 00
 *     应答：ID=0x581 DLC=8  DATA:60 02 20 00 00 00 00 00
 * 5. 标零
 *    下发：ID=0x601 DLC=8  DATA:2B 70 20 00 00 00 00 00
 *     应答：ID=0x581 DLC=8  DATA:60 70 20 00 00 00 00 00
 * 5. 运动控制下发(目标位置1.0rad，速度0，力矩0，KP=10000，KD=100)
 *    下发：ID=0x401 DLC=16  DATA:00 00 80 3F 00 00 00 00 00 00 00 00 10 27 64 00
 *     正常应答：ID=0x191 DLC=16  DATA:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *     有异常应答：ID=0x191 DLC=20  DATA:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 00 00 00
 * ==============================================================================
 */
typedef union {
    uint8_t raw_buffer[ sizeof(CANHeader) +
                        64 ]; /**< 70字节完整CAN帧原始字节缓冲区，可直接整体发送/拷贝 */
    struct {
        CANHeader header;   /**< 10字节CAN属性头，通道、ID、帧类型、DLC等配置 */
        uint8_t data[ 64 ]; /**< CAN数据负载区，最大64字节；仅前 dlc_to_len(header.dlc)
                               字节为有效数据 */
    };
} Net2CanFrame;

#pragma pack()

// ===================== 仅C++编译生效：上层std::array业务封装（MCU直接跳过） =====================
#ifdef __cplusplus
#include <array>
#include <algorithm>
#include <cstring>

// C++业务层包装器，仅用于上层逻辑处理，底层收发依旧使用上面的Net2CanFrame联合体
struct Net2CanFrameWrapper {
    CANHeader               header;
    std::array<uint8_t, 64> data{};

    // 从底层协议帧拷贝构造
    Net2CanFrameWrapper() = default;
    Net2CanFrameWrapper(const Net2CanFrame &raw_frame) {
        header = raw_frame.header;
        std::copy(raw_frame.data, raw_frame.data + 64, data.begin());
    }

    // 转换回底层联合体，用于下发发送
    Net2CanFrame to_raw_frame() const {
        Net2CanFrame raw{};
        raw.header = this->header;
        std::copy(data.begin(), data.end(), raw.data);
        return raw;
    }

    // 获取有效数据长度
    size_t get_valid_data_len() const {
        return header.dlc > 64 ? 64 : header.dlc;
    }

    // 清空整帧
    void clear() {
        memset(&header, 0, sizeof(header));
        data.fill(0);
    }
};
#endif

static inline uint8_t dlc_to_len(uint8_t dlc) {
    static const uint8_t dlc_table[ 16 ] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return (dlc < 16) ? dlc_table[ dlc ] : 0;
}

static inline uint8_t len_to_dlc(uint8_t len) {
    if (len <= 8)
        return len;
    if (len <= 12)
        return 9;
    if (len <= 16)
        return 10;
    if (len <= 20)
        return 11;
    if (len <= 24)
        return 12;
    if (len <= 32)
        return 13;
    if (len <= 48)
        return 14;
    if (len <= 64)
        return 15;
    return 0;
}

#endif
