#include "calibration_algorithm_v2.h"
#include <math.h>
#include <string.h>

// 校准数据缓冲区
#define MAX_CALIB_DATA 1000
static sensor_data_t calib_data_buffer[MAX_CALIB_DATA];
static uint32_t calib_data_count = 0;
static calibration_state_t calib_state = CALIB_IDLE;

// 数学常量
#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 倾角补偿：将三轴数据从倾斜坐标系转换到水平坐标系
void tilt_compensate(float mx, float my, float mz, float pitch_deg, float roll_deg,
                    float* mx_horizontal, float* my_horizontal) {
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad = roll_deg * DEG_TO_RAD;
    
    // 旋转矩阵：从倾斜坐标系到水平坐标系
    *mx_horizontal = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
    *my_horizontal = mx * sinf(roll_rad) * sinf(pitch_rad) + 
                    my * cosf(roll_rad) - 
                    mz * sinf(roll_rad) * cosf(pitch_rad);
}

// 计算方位角
static float calculate_heading(float mx, float my) {
    float heading_rad = atan2f(-my, mx);
    float heading_deg = heading_rad * RAD_TO_DEG;
    if (heading_deg < 0) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

// 椭圆拟合算法（对水平面数据进行拟合）
static bool fit_ellipse(const float* x, const float* y, uint32_t count,
                       float* center_x, float* center_y, float* scale_x, float* scale_y) {
    if (count < 10) return false;
    
    // 计算数据范围
    float x_min = x[0], x_max = x[0];
    float y_min = y[0], y_max = y[0];
    
    for (uint32_t i = 1; i < count; i++) {
        if (x[i] < x_min) x_min = x[i];
        if (x[i] > x_max) x_max = x[i];
        if (y[i] < y_min) y_min = y[i];
        if (y[i] > y_max) y_max = y[i];
    }
    
    // 计算中心点
    *center_x = (x_min + x_max) / 2.0f;
    *center_y = (y_min + y_max) / 2.0f;
    
    // 计算缩放因子
    float range_x = x_max - x_min;
    float range_y = y_max - y_min;
    
    if (range_y > 0.001f) {
        *scale_x = 1.0f;
        *scale_y = range_x / range_y;
    } else {
        *scale_x = 1.0f;
        *scale_y = 1.0f;
    }
    
    return true;
}

// 数据有效性检查
bool validate_sensor_data(const sensor_data_t* data) {
    if (!data) return false;
    
    // 检查磁力计数据范围
    float mag_magnitude = sqrtf(data->mag_x * data->mag_x + 
                               data->mag_y * data->mag_y + 
                               data->mag_z * data->mag_z);
    
    // 地磁场强度约为25-65 uT
    if (mag_magnitude < 10.0f || mag_magnitude > 100.0f) {
        return false;
    }
    
    // 检查加速度计数据
    float acc_magnitude = sqrtf(data->acc_x * data->acc_x + 
                               data->acc_y * data->acc_y + 
                               data->acc_z * data->acc_z);
    
    // 重力加速度约为9.8 m/s²
    if (acc_magnitude < 5.0f || acc_magnitude > 15.0f) {
        return false;
    }
    
    return true;
}

// 校准质量评估
float evaluate_calibration_quality(const sensor_data_t* data, uint32_t count) {
    if (count < 50) return 0.0f;
    
    // 计算数据覆盖度
    float heading_coverage = 0.0f;
    float pitch_coverage = 0.0f;
    float roll_coverage = 0.0f;
    
    // 检查方位角覆盖（应该接近360度）
    float min_heading = 360.0f, max_heading = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        float mx_horizontal, my_horizontal;
        tilt_compensate(data[i].mag_x, data[i].mag_y, data[i].mag_z,
                       data[i].pitch, data[i].roll, &mx_horizontal, &my_horizontal);
        float heading = calculate_heading(mx_horizontal, my_horizontal);
        
        if (heading < min_heading) min_heading = heading;
        if (heading > max_heading) max_heading = heading;
    }
    
    heading_coverage = (max_heading - min_heading) / 360.0f;
    
    // 检查姿态角覆盖
    float min_pitch = 90.0f, max_pitch = -90.0f;
    float min_roll = 90.0f, max_roll = -90.0f;
    
    for (uint32_t i = 0; i < count; i++) {
        if (data[i].pitch < min_pitch) min_pitch = data[i].pitch;
        if (data[i].pitch > max_pitch) max_pitch = data[i].pitch;
        if (data[i].roll < min_roll) min_roll = data[i].roll;
        if (data[i].roll > max_roll) max_roll = data[i].roll;
    }
    
    pitch_coverage = (max_pitch - min_pitch) / 40.0f; // 期望20度范围
    roll_coverage = (max_roll - min_roll) / 40.0f;
    
    // 综合质量评分
    float quality = (heading_coverage * 0.6f + 
                    pitch_coverage * 0.2f + 
                    roll_coverage * 0.2f);
    
    return (quality > 1.0f) ? 1.0f : quality;
}

// 校准开始
bool calibration_start(void) {
    calib_data_count = 0;
    calib_state = CALIB_HORIZONTAL;
    memset(calib_data_buffer, 0, sizeof(calib_data_buffer));
    return true;
}

// 添加校准数据
bool calibration_add_data(const sensor_data_t* data) {
    if (!data || !validate_sensor_data(data)) {
        return false;
    }
    
    if (calib_data_count >= MAX_CALIB_DATA) {
        return false;
    }
    
    calib_data_buffer[calib_data_count] = *data;
    calib_data_count++;
    
    return true;
}

// 获取校准状态
calibration_state_t calibration_get_state(void) {
    return calib_state;
}

// 完成校准
bool calibration_finish(horizontal_calibration_t* result) {
    if (!result || calib_data_count < 100) {
        calib_state = CALIB_ERROR;
        return false;
    }
    
    // 1. 对所有数据进行倾角补偿，得到水平面数据
    float* mx_horizontal = malloc(calib_data_count * sizeof(float));
    float* my_horizontal = malloc(calib_data_count * sizeof(float));
    
    if (!mx_horizontal || !my_horizontal) {
        free(mx_horizontal);
        free(my_horizontal);
        calib_state = CALIB_ERROR;
        return false;
    }
    
    for (uint32_t i = 0; i < calib_data_count; i++) {
        tilt_compensate(calib_data_buffer[i].mag_x, 
                       calib_data_buffer[i].mag_y, 
                       calib_data_buffer[i].mag_z,
                       calib_data_buffer[i].pitch, 
                       calib_data_buffer[i].roll,
                       &mx_horizontal[i], &my_horizontal[i]);
    }
    
    // 2. 对水平面数据进行椭圆拟合
    if (!fit_ellipse(mx_horizontal, my_horizontal, calib_data_count,
                    &result->center_x, &result->center_y,
                    &result->scale_x, &result->scale_y)) {
        free(mx_horizontal);
        free(my_horizontal);
        calib_state = CALIB_ERROR;
        return false;
    }
    
    // 3. 评估校准质量
    result->calibration_quality = evaluate_calibration_quality(calib_data_buffer, calib_data_count);
    result->data_points = calib_data_count;
    result->is_valid = (result->calibration_quality > 0.7f);
    
    free(mx_horizontal);
    free(my_horizontal);
    
    calib_state = CALIB_COMPLETE;
    return result->is_valid;
}

// 用户校准开始
bool user_calibration_start(void) {
    calib_data_count = 0;
    calib_state = CALIB_HORIZONTAL;
    memset(calib_data_buffer, 0, sizeof(calib_data_buffer));
    return true;
}

// 用户校准添加数据
bool user_calibration_add_data(const sensor_data_t* data) {
    return calibration_add_data(data);
}

// 完成用户校准
bool user_calibration_finish(user_calibration_t* result) {
    if (!result || calib_data_count < 50) {
        calib_state = CALIB_ERROR;
        return false;
    }
    
    // 计算环境磁场偏移
    float mx_sum = 0.0f, my_sum = 0.0f;
    
    for (uint32_t i = 0; i < calib_data_count; i++) {
        float mx_horizontal, my_horizontal;
        tilt_compensate(calib_data_buffer[i].mag_x, 
                       calib_data_buffer[i].mag_y, 
                       calib_data_buffer[i].mag_z,
                       calib_data_buffer[i].pitch, 
                       calib_data_buffer[i].roll,
                       &mx_horizontal, &my_horizontal);
        
        mx_sum += mx_horizontal;
        my_sum += my_horizontal;
    }
    
    result->env_offset_x = mx_sum / calib_data_count;
    result->env_offset_y = my_sum / calib_data_count;
    result->is_valid = true;
    
    calib_state = CALIB_COMPLETE;
    return true;
}

// 应用校准
bool apply_calibration(const sensor_data_t* input, 
                      const horizontal_calibration_t* calib,
                      const user_calibration_t* user_calib,
                      calibration_result_t* output) {
    if (!input || !calib || !output) {
        return false;
    }
    
    // 1. 倾角补偿
    float mx_horizontal, my_horizontal;
    tilt_compensate(input->mag_x, input->mag_y, input->mag_z,
                   input->pitch, input->roll, &mx_horizontal, &my_horizontal);
    
    // 2. 硬铁偏移补偿
    mx_horizontal -= calib->center_x;
    my_horizontal -= calib->center_y;
    
    // 3. 软铁缩放补偿
    mx_horizontal *= calib->scale_x;
    my_horizontal *= calib->scale_y;
    
    // 4. 用户环境补偿（如果有效）
    if (user_calib && user_calib->is_valid) {
        mx_horizontal -= user_calib->env_offset_x;
        my_horizontal -= user_calib->env_offset_y;
    }
    
    // 5. 计算方位角
    output->heading = calculate_heading(mx_horizontal, my_horizontal);
    output->pitch = input->pitch;
    output->roll = input->roll;
    output->calibration_quality = calib->calibration_quality;
    
    return true;
} 