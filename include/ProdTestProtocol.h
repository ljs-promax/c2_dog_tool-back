#ifndef PROD_TEST_PROTOCOL_H
#define PROD_TEST_PROTOCOL_H

#include <cstdint>
#include <cstring>
#include "JointControl.h"         // JointCmd, Feedback, JointId, JointCfg
#include "mcu_soc_com_protocol.h" // ImuFeedback, BmsFeedback, McuStatus
#include "ws2812_control.hpp"     // RgbColor

#pragma pack(push, 1)

// ============================================
// 通信帧头
// ============================================
typedef struct {
    uint16_t header;    // 帧头标志，固定 0xA5A5
    uint16_t frame_id;  // 帧序号
    uint16_t version;   // 协议版本，固定 1
    uint16_t cmd;       // 命令码
    uint16_t data_len;  // 数据长度
    uint16_t crc_sum;   // CRC校验
} CS_DataHeader;

// ============================================
// 通信帧（完整数据帧）
// ============================================
typedef struct {
    CS_DataHeader header;
    uint8_t       buffer[1024];  // 数据区
} CS_ComFrame;

// ============================================
// 测试结果
// ============================================
typedef struct {
    int8_t  res;                 // 0:测试结束  1:测试中  -1:测试错误
    uint8_t buffer[63];          // 测试数据内容(不同的测试对象，返回的不同，可自定义)
} ProdTestResult;

#pragma pack(pop)

// ============================================
// 帧头默认值
// ============================================
namespace ProdTestHeader {
static constexpr uint16_t FRAME_HEADER     = 0xA5A5;
static constexpr uint16_t PROTOCOL_VERSION = 1;
} // namespace ProdTestHeader

// ============================================
// 命令码定义 (Client -> Server)
// ============================================
namespace ProdTestCmd {
         
// ---------- 电机相关 (0x1000-0x10FF) ----------
static constexpr uint16_t MOTOR_TEST_ENABLE   = 0x1000; // 开启电机测试
static constexpr uint16_t MOTOR_TEST_DISABLE  = 0x1001; // 关闭电机测试
static constexpr uint16_t MOTOR_SN_RW         = 0x1002; // SN码读写
static constexpr uint16_t MOTOR_VERSION       = 0x1003; // 读取软件/硬件版本号
static constexpr uint16_t MOTOR_CONTROL       = 0x1004; // 电机控制
static constexpr uint16_t MOTOR_ENCODER_CAL   = 0x1005; // 编码器校准指令
static constexpr uint16_t MOTOR_ENCODER_LOSS  = 0x1006; // 副编码器丢失检测
static constexpr uint16_t MOTOR_NTC_READ      = 0x1007; // NTC温度读取
static constexpr uint16_t MOTOR_ENCODER_ZERO  = 0x1008; // 编码器当前位置标零
static constexpr uint16_t MOTOR_OTA_UPGRADE   = 0x1009; // 关节驱动器OTA升级
static constexpr uint16_t MOTOR_ID_CHANGE     = 0x100A; // 关节驱动器ID修改
static constexpr uint16_t MOTOR_ERROR_CLEAR   = 0x100B; // 清除关节驱动器错误
 
// ---------- 主控盒相关 (0x1100-0x11FF) ----------
static constexpr uint16_t MAINCTRL_VERSION    = 0x1100;  // 查询主控盒MCU版本
static constexpr uint16_t MAINCTRL_SENSOR     = 0x1101;  // 查询主控盒传感器状态
static constexpr uint16_t MAINCTRL_IMU       = 0x1102;  // 查询IMU数据
static constexpr uint16_t MAINCTRL_BMS        = 0x1103;  // 查询电池信息
static constexpr uint16_t MAINCTRL_RTC        = 0x1104;  // RTC测试(读写时间)
static constexpr uint16_t MAINCTRL_BLUETOOTH  = 0x1105;  // 蓝牙测试
static constexpr uint16_t MAINCTRL_NETWORK   = 0x1106;  // 网络测试
static constexpr uint16_t MAINCTRL_USB        = 0x1107;  // USB测试
static constexpr uint16_t MAINCTRL_SBUS      = 0x1108;  // SBUS测试
static constexpr uint16_t MAINCTRL_LED        = 0x1109;  // LED测试
static constexpr uint16_t MAINCTRL_SN         = 0x110A;  // 主控盒SN读写
static constexpr uint16_t MAINCTRL_APP_SERVICE = 0x110B;  // APP服务状态检测
static constexpr uint16_t MAINCTRL_OTA_UPGRADE = 0x110C;  // 主控盒MCU OTA升级

// ---------- 服务端控制相关 (0x1F00-0x1FFF) ----------
static constexpr uint16_t SERVER_RESTART      = 0x1F00;  // 重启daemon服务
static constexpr uint16_t SERVER_PING         = 0x1F01;  // 服务端通信测试
static constexpr uint16_t SERVER_DISABLE_TARGET_SERVICE = 0x1F02;  // 禁用daemon服务
static constexpr uint16_t FILE_UPLOAD_BEGIN   = 0x1F10;  // OTA文件上传开始
static constexpr uint16_t FILE_UPLOAD_DATA    = 0x1F11;  // OTA文件上传分片
static constexpr uint16_t FILE_UPLOAD_END     = 0x1F12;  // OTA文件上传结束
static constexpr uint16_t FILE_UPLOAD_CANCEL  = 0x1F13;  // OTA文件上传取消

} // namespace ProdTestCmd
 
// ============================================
// 响应码定义 (Server -> Client)
// 响应码 = 命令码 + 0x1000
// ========================================
namespace ProdTestResp {
static constexpr uint16_t MOTOR_DATA_STREAM   = 0x2000;  // 电机数据(周期推送)
static constexpr uint16_t MOTOR_DISABLE_ACK     = 0x2001;  // 电机测试关闭确认
static constexpr uint16_t MOTOR_SN_ACK          = 0x2002;  // SN码结果
static constexpr uint16_t MOTOR_VERSION_ACK     = 0x2003;  // 版本读取结果
static constexpr uint16_t MOTOR_CONTROL_ACK     = 0x2004;  // 电机控制结果
static constexpr uint16_t MOTOR_ENCODER_CAL_ACK = 0x2005;  // 编码器校准结果
static constexpr uint16_t MOTOR_ENCODER_LOSS_ACK = 0x2006; // 副编码器检测结果
static constexpr uint16_t MOTOR_NTC_ACK         = 0x2007;  // NTC读取结果
static constexpr uint16_t MOTOR_ENCODER_ZERO_ACK = 0x2008; // 编码器标零结果
static constexpr uint16_t MOTOR_OTA_ACK          = 0x2009; // 关节驱动器OTA结果
static constexpr uint16_t MOTOR_ID_CHANGE_ACK    = 0x200A; // 关节驱动器ID修改结果
static constexpr uint16_t MOTOR_ERROR_CLEAR_ACK  = 0x200B; // 驱动器清错结果

static constexpr uint16_t MAINCTRL_VERSION_ACK = 0x2100;  // MCU版本查询结果
static constexpr uint16_t MAINCTRL_SENSOR_ACK  = 0x2101;  // 传感器状态结果
static constexpr uint16_t MAINCTRL_IMU_ACK     = 0x2102;  // IMU数据结果
static constexpr uint16_t MAINCTRL_BMS_ACK     = 0x2103;  // BMS电池信息结果
static constexpr uint16_t MAINCTRL_RTC_ACK     = 0x2104;  // RTC测试结果
static constexpr uint16_t MAINCTRL_BT_ACK      = 0x2105;  // 蓝牙测试结果
static constexpr uint16_t MAINCTRL_NET_ACK     = 0x2106;  // 网络测试结果
static constexpr uint16_t MAINCTRL_USB_ACK     = 0x2107;  // USB测试结果
static constexpr uint16_t MAINCTRL_SBUS_ACK    = 0x2108;  // SBUS测试结果
static constexpr uint16_t MAINCTRL_LED_ACK     = 0x2109;  // LED测试结果
static constexpr uint16_t MAINCTRL_SN_ACK      = 0x210A;  // 主控盒SN结果
static constexpr uint16_t MAINCTRL_APP_ACK     = 0x210B;  // APP服务状态结果
static constexpr uint16_t MAINCTRL_OTA_ACK     = 0x210C;  // 主控盒MCU OTA结果
static constexpr uint16_t SERVER_RESTART_ACK   = 0x2F00;  // daemon重启确认
static constexpr uint16_t SERVER_PING_ACK      = 0x2F01;  // 服务端通信测试确认
static constexpr uint16_t SERVER_DISABLE_TARGET_SERVICE_ACK = 0x2F02;  // 禁用daemon服务确认
static constexpr uint16_t FILE_UPLOAD_BEGIN_ACK  = 0x2F10; // OTA文件上传开始确认
static constexpr uint16_t FILE_UPLOAD_DATA_ACK   = 0x2F11; // OTA文件上传分片确认
static constexpr uint16_t FILE_UPLOAD_END_ACK    = 0x2F12; // OTA文件上传结束确认
static constexpr uint16_t FILE_UPLOAD_CANCEL_ACK = 0x2F13; // OTA文件上传取消确认

// 系统状态
static constexpr uint16_t SYSTEM_BUSY   = 0xF001;  // 系统忙
static constexpr uint16_t SYSTEM_ERROR = 0xF002;  // 系统错误
static constexpr uint16_t SYSTEM_OK    = 0xF003;  // 确认成功
} // namespace ProdTestResp

// ============================================
// 数据结构定义 (复用已有结构)
// ============================================
// JointCmd         -> JointControl.h (电机控制参数)
// JointId          -> JointControl.h (关节连接参数: can_ch, can_id)
// JointCfg         -> JointControl.h (关节配置)
// Feedback         -> JointControl.h (电机反馈数据)
// ImuFeedback      -> mcu_soc_com_protocol.h (IMU数据)
// BmsFeedback      -> mcu_soc_com_protocol.h (电池数据)
// McuStatus        -> mcu_soc_com_protocol.h (MCU状态)
// RgbColor         -> ws2812_control.hpp (LED颜色, 复用 LedColor)

// ============================================
// 新增数据结构
// ============================================
#pragma pack(push, 1)

// SN码 (主控盒18字节 / 关节26字节)
typedef struct {
    uint8_t sn[32];  // 预留足够空间
    uint8_t len;     // 实际长度
} SnData;

// 关节版本信息
typedef struct {
    uint8_t can_ch;
    uint8_t can_id;
    char    sw_version[16];  // 软件版本
    char    hw_version[16];  // 硬件版本
} DriverVersionInfo;

// NTC温度
typedef struct {
    uint8_t can_ch;
    uint8_t can_id;
    float   motor_ntc;
    float   driver_ntc;
} NtcData;

// ============================================
// 各Handler响应结构体 (res + 数据)
// ============================================

// IMU响应
typedef struct {
    int8_t  res;
    float   qx, qy, qz, qw;
    float   wx, wy, wz;
    float   ax, ay, az;
} ImuResponse;
 
// MCU版本响应
typedef struct {
    int8_t  res;
    char    version[32];
} McuVersionResponse;

// MCU状态响应
typedef struct {
    int8_t   res;              // 0=成功，-1=失败
    uint8_t  sensors_status;    // bit0: 急停 1正常0被按下  bit1: IMU 1正常0异常  bit2: 电池 1正常0异常
    uint32_t can0_status;      // CAN0 总线异常恢复次数
    uint32_t can1_status;      // CAN1 总线异常恢复次数
    uint32_t can2_status;      // CAN2 总线异常恢复次数
    uint32_t can3_status;      // CAN3 总线异常恢复次数
} McuStatusResponse;

// 传感器响应 (IMU + BMS + MCUStatus)
typedef struct {
    int8_t  res;
    uint8_t data[204];  // SensorDataPack内容
} SensorResponse;

// RTC响应
typedef struct {
    int8_t  res;              // 0=成功，-1=失败
    int8_t  rtc0_exists;      // 0=不存在，1=存在
    char    write_time[32];    // 写入的时间
    char    read_time[32];     // 读出的时间
} RtcResponse;

// 蓝牙响应
typedef struct {
    int8_t  res;
    char    info[64];
} BluetoothResponse;

// 网络响应
typedef struct {
    int8_t  res;              // 整体结果：0=全部成功，-1=有失败
    // 每个网卡的ping结果
    int8_t  wlan0_result;     // 0=成功，-1=失败，-2=不存在
    int8_t  wlan1_result;
    int8_t  enp1s0_result;
    int8_t  eth0_result;
} NetworkResponse;

// USB响应
typedef struct {
    int8_t  res;      // 0=成功，-1=失败
} UsbResponse;

// BMS响应
typedef struct {
    int8_t   res;                   // 0=成功，-1=失败
    float    voltage_V;              // 总电压 (V)
    float    current_A;             // 总电流 (A)
    uint8_t  battery_level_percent; // 电量百分比 (0-100)
    uint16_t remaining_capacity_mAh;// 剩余容量 (mAh)
    uint16_t cycles;               // 循环次数
    float    temp_fet_C;           // MOS管温度 (℃)
    uint8_t  charger_in1;         // 充电口1状态 (0=未插/1=已插)
    uint8_t  charger_in2;         // 充电口2状态 (0=未插/1=已插)
} BmsResponse;

// SBUS响应
typedef struct {
    int8_t  res;
    char    info[64];
} SbusResponse;

// LED响应
typedef struct {
    int8_t  res;
} LedResponse;

// APP服务状态响应  app_client.service
typedef struct {
    int8_t  res;      // 0=服务开启，-1=服务未开启
} AppServiceResponse;

// SN响应
typedef struct {
    int8_t  res;
    uint8_t sn[32];
    uint8_t len;
} SnTestResponse;

// 关节连接响应
typedef struct {
    int8_t  res;
} DriverConnectResponse;

// 关节断开响应
typedef struct {
    int8_t  res;
} DriverDisconnectResponse;

// 关节SN读写响应
typedef struct {
    int8_t  res;
    uint8_t sn[32];
    uint8_t len;
} DriverSnRwResponse;

// 关节版本响应
typedef struct {
    int8_t  res;
    uint8_t can_ch;
    uint8_t can_id;
    char    sw_version[16];
    char    hw_version[16];
} DriverVersionResponse;

// NTC读取响应
typedef struct {
    int8_t  res;
    uint8_t can_ch;
    uint8_t can_id;
    float   motor_ntc;
    float   driver_ntc;
} NtcReadResponse;

// 驱动器ID修改响应
typedef struct {
    int8_t  res;
    uint8_t can_ch;
    uint8_t old_can_id;
    uint8_t new_can_id;
} MotorIdChangeResponse;

// 电机使能响应 (成功带反馈数据)
typedef struct {
    int8_t  res;
    float   position;
    float   velocity;
    float   torque;
    float   motor_temperature;
    float   driver_temperature;
    uint16_t error_code;
} MotorEnableResponse;

// 电机失能响应
typedef struct {
    int8_t  res;
} MotorDisableResponse;

// 驱动器清错响应
typedef struct {
    int8_t   res;
    uint16_t error_code;
} MotorErrorClearResponse;

// 电机参数响应
typedef struct {
    int8_t  res;
    float   kp;
    float   kd;
    float   position;
    float   velocity;
    float   torque;
} MotorSetParamResponse;

// 编码器校准响应
typedef struct {
    int8_t  res;
} EncoderCalResponse;

// OTA升级响应
typedef struct {
    int8_t  res;          // 1=已开始/进行中，0=成功，-1=失败
    uint8_t progress;     // 0-100，升级过程中持续推送
} OtaUpgradeResponse;

// 服务端重启响应
typedef struct {
    int8_t  res;          // 0=已接受重启命令
} ServerRestartResponse;

// 服务端通信测试响应
typedef struct {
    int8_t  res;          // 0=成功, -1=失败
} ServerPingResponse;

// 禁用daemon服务响应
typedef struct {
    int8_t  res;          // 0=成功, -1=失败
} ServerDisableTargetServiceResponse;

// OTA文件上传开始请求头，后续紧跟 filename 字符串
typedef struct {
    uint32_t file_size;
    uint32_t file_crc32;
} FileUploadBeginRequest;

// OTA文件上传分片请求头，后续紧跟 chunk 数据
typedef struct {
    uint32_t upload_id;
    uint32_t chunk_index;
    uint32_t offset;
} FileUploadDataRequest;

// OTA文件上传结束请求
typedef struct {
    uint32_t upload_id;
} FileUploadEndRequest;

// OTA文件上传取消请求
typedef struct {
    uint32_t upload_id;
} FileUploadCancelRequest;

// OTA文件上传开始响应
typedef struct {
    int8_t   res;
    uint32_t upload_id;//cmd
    uint32_t file_size;
    uint16_t max_chunk_size;
} FileUploadBeginResponse;

// OTA文件上传分片响应
typedef struct {
    int8_t   res;
    uint32_t upload_id;
    uint32_t next_chunk_index;
    uint32_t received_size;
} FileUploadDataResponse;

// OTA文件上传结束响应
typedef struct {
    int8_t   res;
    uint32_t upload_id;
    uint32_t file_size;
    uint32_t file_crc32;
    char     filename[128];
} FileUploadEndResponse;

// OTA文件上传取消响应
typedef struct {
    int8_t   res;
    uint32_t upload_id;
} FileUploadCancelResponse;

#pragma pack(pop)

// ============================================
// 协议工具类
// ============================================
class ProdTestProtocol {
public:
    // 计算CRC16（CRC-CCITT, 多项式0x1021）
    static uint16_t calcCrcSum(const uint8_t* data, int len) {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    // 校验帧头
    static bool verifyHeader(const CS_DataHeader* hdr) {
        if (hdr->header != ProdTestHeader::FRAME_HEADER) return false;
        if (hdr->version != ProdTestHeader::PROTOCOL_VERSION) return false;
        if (hdr->data_len > 1024) return false;
        return true;
    }

    // 校验整帧
    static bool verifyFrame(const CS_ComFrame* frame, int recvLen) {
        int totalLen = sizeof(CS_DataHeader) + frame->header.data_len;
        if (recvLen < totalLen) return false;

        // CRC覆盖范围: header起始的10字节(header+frame_id+version+cmd+data_len)
        //           + buffer(data_len字节)
        // 不包括crc_sum(2字节)本身
        // 与发送端保持一致，重新拼CRC数据
        int crcLen = 10 + frame->header.data_len;
        std::vector<uint8_t> crcBuf;
        crcBuf.insert(crcBuf.end(),
                      (uint8_t*)&frame->header,
                      (uint8_t*)&frame->header + 8);              // header+frame_id+version+cmd (8字节)
        crcBuf.insert(crcBuf.end(),
                      (uint8_t*)&frame->header.data_len,
                      (uint8_t*)&frame->header.data_len + 2);    // data_len (2字节)
        crcBuf.insert(crcBuf.end(),
                      frame->buffer,
                      frame->buffer + frame->header.data_len);

        uint16_t crc = calcCrcSum(crcBuf.data(), crcBuf.size());
        if (crc != frame->header.crc_sum) return false;
        return true;
    }

    // 构建响应帧
    static void buildResponse(CS_ComFrame* frame, uint16_t cmd,
                              const void* data, int dataLen) {
        frame->header.header = ProdTestHeader::FRAME_HEADER;
        frame->header.frame_id = 0;  // TODO: 递增帧ID
        frame->header.version = ProdTestHeader::PROTOCOL_VERSION;
        frame->header.cmd = cmd;
        frame->header.data_len = dataLen;

        if (data && dataLen > 0) {
            memcpy(frame->buffer, data, dataLen);
        }

        // CRC范围: header起始的10字节(header+frame_id+version+cmd+data_len)
        //           + buffer(dataLen字节)
        // 不包括crc_sum(2字节)本身
        // buffer前移2字节
        std::vector<uint8_t> crcBuf;
        crcBuf.insert(crcBuf.end(),
                      (uint8_t*)&frame->header,
                      (uint8_t*)&frame->header + 8);             // header+frame_id+version+cmd (8字节)
        crcBuf.insert(crcBuf.end(),
                      (uint8_t*)&frame->header.data_len,
                      (uint8_t*)&frame->header.data_len + 2);   // data_len (2字节)
        crcBuf.insert(crcBuf.end(),
                      frame->buffer,
                      frame->buffer + dataLen);

        // 打印CRC原始数据
        printf("[buildResponse] CRC raw data (%d bytes):\n", (int)crcBuf.size());
        for (size_t i = 0; i < crcBuf.size(); i++) {
            printf("  [%02zu] 0x%02X\n", i, crcBuf[i]);
        }

        uint16_t crc = calcCrcSum(crcBuf.data(), crcBuf.size());
        frame->header.crc_sum = crc;
    }

    // 构建测试结果帧
    static void buildTestResult(CS_ComFrame* frame, uint16_t cmd,
                                int8_t res, const void* data, int dataLen) {
        ProdTestResult result;
        result.res = res;
        if (data && dataLen > 0 && dataLen <= 63) {
            memcpy(result.buffer, data, dataLen);
        }
        buildResponse(frame, cmd, &result, sizeof(ProdTestResult));
    }
};

#endif // PROD_TEST_PROTOCOL_H
