
/**
 * 磁力计校准算法 - 自动生成
 * 包含倾斜补偿和软铁/硬铁校准
 */

#include <math.h>

// 校准参数
#define HARD_IRON_OFFSET_X -72.976264f
#define HARD_IRON_OFFSET_Y -10.188739f
#define SOFT_IRON_SCALE 1.035606f

// 坐标系映射类型
typedef enum {
    MAPPING_TYPE_1 = 1,  // X→Y, Y→X, Z→Z
    MAPPING_TYPE_2 = 2,  // X→X, Y→Y, Z→Z (原始映射)
    MAPPING_TYPE_3 = 3   // X→-X, Y→Y, Z→Z (X轴取负)
} mapping_type_t;

// 输入数据结构
typedef struct {
    float mag_x, mag_y, mag_z;  // 磁力计原始数据
    float pitch, roll, yaw;     // 欧拉角(度)
    mapping_type_t mapping;     // 坐标系映射类型
} tilt_input_t;

// 输出数据结构
typedef struct {
    float mx_comp, my_comp;     // 倾斜补偿后的水平磁场
    float mx_cal, my_cal;       // 校准后的磁场
} tilt_output_t;

/**
 * 坐标系映射函数
 */
void map_coordinates(float mag_x, float mag_y, float mag_z, 
                    mapping_type_t mapping,
                    float *mx_out, float *my_out, float *mz_out) {
    switch(mapping) {
        case MAPPING_TYPE_1:  // X→Y, Y→X, Z→Z
            *mx_out = mag_y;
            *my_out = mag_x;
            *mz_out = mag_z;
            break;
            
        case MAPPING_TYPE_2:  // X→X, Y→Y, Z→Z (原始映射)
            *mx_out = mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
            
        case MAPPING_TYPE_3:  // X→-X, Y→Y, Z→Z (X轴取负)
            *mx_out = -mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
            
        default:
            *mx_out = mag_x;
            *my_out = mag_y;
            *mz_out = mag_z;
            break;
    }
}

/**
 * 倾斜补偿算法
 */
void tilt_compensation(tilt_input_t *input, tilt_output_t *output) {
    // 1. 坐标系映射
    float mx, my, mz;
    map_coordinates(input->mag_x, input->mag_y, input->mag_z, 
                   input->mapping, &mx, &my, &mz);
    
    // 2. 欧拉角转弧度
    float pitch_rad = input->pitch * M_PI / 180.0f;
    float roll_rad = input->roll * M_PI / 180.0f;
    
    // 3. 计算三角函数
    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);
    
    // 4. 倾斜补偿公式
    // 先绕X轴旋转（pitch），再绕Y轴旋转（roll）
    output->mx_comp = mx * cos_p + mz * sin_p;
    output->my_comp = mx * sin_p * sin_r + my * cos_r - mz * cos_p * sin_r;
}

/**
 * 软铁/硬铁校准算法
 */
void calibrate_magnetometer(float *mx_comp, float *my_comp) {
    // 硬铁校准：减去偏移
    *mx_comp -= HARD_IRON_OFFSET_X;
    *my_comp -= HARD_IRON_OFFSET_Y;
    
    // 软铁校准：缩放
    *my_comp *= SOFT_IRON_SCALE;
}

/**
 * 完整的磁力计处理函数
 */
void process_magnetometer_data(tilt_input_t *input, tilt_output_t *output) {
    // 1. 倾斜补偿
    tilt_compensation(input, output);
    
    // 2. 软铁/硬铁校准
    calibrate_magnetometer(&output->mx_comp, &output->my_comp);
    
    // 3. 保存校准后的结果
    output->mx_cal = output->mx_comp;
    output->my_cal = output->my_comp;
}

/**
 * 计算磁偏角（指南针方向）
 */
float calculate_heading(float mx_cal, float my_cal) {
    float heading = atan2f(my_cal, mx_cal) * 180.0f / M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

/**
 * 使用示例
 */
void example_usage() {
    // 示例数据
    tilt_input_t input = {
        .mag_x = 247.0f,
        .mag_y = 137.0f, 
        .mag_z = -57.0f,
        .pitch = 62.46f,
        .roll = -1.01f,
        .yaw = -90.37f,
        .mapping = MAPPING_TYPE_1
    };
    
    tilt_output_t output;
    
    // 执行完整的磁力计处理
    process_magnetometer_data(&input, &output);
    
    // 计算指南针方向
    float heading = calculate_heading(output.mx_cal, output.my_cal);
    
    // 输出结果
    printf("倾斜补偿结果: mx=%.2f, my=%.2f\n", output.mx_comp, output.my_comp);
    printf("校准后结果: mx=%.2f, my=%.2f\n", output.mx_cal, output.my_cal);
    printf("指南针方向: %.1f°\n", heading);
}
