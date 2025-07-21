#ifndef CALIBRATION_ALGORITHM_H
#define CALIBRATION_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

// 校准参数结构体
typedef struct {
    // 软硬铁校准参数
    float center_x;
    float center_y;
    float scale_x;
    float scale_y;
    
    // 校准质量指标
    float calibration_quality;  // 0.0-1.0，1.0为最佳
    uint32_t data_points;       // 校准数据点数
    
    // 参数有效性标志
    bool is_valid;
} factory_calibration_t;

// 用户校准参数结构体
typedef struct {
    // 环境磁场补偿
    float env_offset_x;
    float env_offset_y;
    
    // 参数有效性标志
    bool is_valid;
} user_calibration_t;

// 传感器数据结构体
typedef struct {
    float mag_x, mag_y, mag_z;  // 磁力计三轴数据
    float acc_x, acc_y, acc_z;  // 加速度计三轴数据
    float gyro_x, gyro_y, gyro_z; // 陀螺仪三轴数据
    float pitch, roll;          // 欧拉角（度）
} sensor_data_t;

// 校准结果结构体
typedef struct {
    float heading;              // 方位角（度）
    float pitch, roll;          // 姿态角（度）
    float calibration_quality;  // 当前校准质量
} calibration_result_t;

// 校准状态枚举
typedef enum {
    CALIB_IDLE,                 // 空闲状态
    CALIB_HORIZONTAL,          // 水平校准
    CALIB_PITCH_UP,            // 船头垫高校准
    CALIB_ROLL_TILT,           // 船侧垫高校准
    CALIB_PROCESSING,          // 数据处理中
    CALIB_COMPLETE,            // 校准完成
    CALIB_ERROR                // 校准错误
} calibration_state_t;

// 函数声明
bool factory_calibration_start(void);
bool factory_calibration_add_data(const sensor_data_t* data);
calibration_state_t factory_calibration_get_state(void);
bool factory_calibration_finish(factory_calibration_t* result);
bool factory_calibration_save(const factory_calibration_t* calib);
bool factory_calibration_load(factory_calibration_t* calib);

bool user_calibration_start(void);
bool user_calibration_add_data(const sensor_data_t* data);
bool user_calibration_finish(user_calibration_t* result);
bool user_calibration_save(const user_calibration_t* calib);
bool user_calibration_load(user_calibration_t* calib);

bool apply_calibration(const sensor_data_t* input, 
                      const factory_calibration_t* factory_calib,
                      const user_calibration_t* user_calib,
                      calibration_result_t* output);

// 校准质量评估函数
float evaluate_calibration_quality(const sensor_data_t* data, uint32_t count);

// 数据有效性检查
bool validate_sensor_data(const sensor_data_t* data);

#endif // CALIBRATION_ALGORITHM_H 