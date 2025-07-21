#ifndef CALIBRATION_ALGORITHM_V2_H
#define CALIBRATION_ALGORITHM_V2_H

#include <stdint.h>
#include <stdbool.h>

// 传感器数据结构体
typedef struct {
    float mag_x, mag_y, mag_z;  // 三轴磁力计数据
    float acc_x, acc_y, acc_z;  // 三轴加速度计数据
    float gyro_x, gyro_y, gyro_z; // 三轴陀螺仪数据
    float pitch, roll;          // 欧拉角（度）
} sensor_data_t;

// 水平面校准参数（倾角补偿后的数据）
typedef struct {
    // 硬铁偏移（水平面）
    float center_x;
    float center_y;
    
    // 软铁缩放（水平面）
    float scale_x;
    float scale_y;
    
    // 校准质量指标
    float calibration_quality;
    uint32_t data_points;
    bool is_valid;
} horizontal_calibration_t;

// 用户校准参数
typedef struct {
    float env_offset_x;
    float env_offset_y;
    bool is_valid;
} user_calibration_t;

// 校准结果
typedef struct {
    float heading;              // 方位角（度）
    float pitch, roll;          // 姿态角（度）
    float calibration_quality;
} calibration_result_t;

// 校准状态
typedef enum {
    CALIB_IDLE,
    CALIB_HORIZONTAL,          // 水平校准
    CALIB_PITCH_UP,            // 船头垫高校准
    CALIB_ROLL_TILT,           // 船侧垫高校准
    CALIB_PROCESSING,
    CALIB_COMPLETE,
    CALIB_ERROR
} calibration_state_t;

// 函数声明
bool calibration_start(void);
bool calibration_add_data(const sensor_data_t* data);
calibration_state_t calibration_get_state(void);
bool calibration_finish(horizontal_calibration_t* result);

bool user_calibration_start(void);
bool user_calibration_add_data(const sensor_data_t* data);
bool user_calibration_finish(user_calibration_t* result);

bool apply_calibration(const sensor_data_t* input, 
                      const horizontal_calibration_t* calib,
                      const user_calibration_t* user_calib,
                      calibration_result_t* output);

// 倾角补偿函数
void tilt_compensate(float mx, float my, float mz, float pitch_deg, float roll_deg,
                    float* mx_horizontal, float* my_horizontal);

// 数据有效性检查
bool validate_sensor_data(const sensor_data_t* data);

#endif // CALIBRATION_ALGORITHM_V2_H 