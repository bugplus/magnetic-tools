/**
 * 倾斜补偿算法头文件
 */

#ifndef TILT_COMPENSATION_ALGORITHM_H
#define TILT_COMPENSATION_ALGORITHM_H

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
} tilt_output_t;

// 函数声明
void map_coordinates(float mag_x, float mag_y, float mag_z, 
                    mapping_type_t mapping,
                    float *mx_out, float *my_out, float *mz_out);

void tilt_compensation(tilt_input_t *input, tilt_output_t *output);

void batch_tilt_compensation(tilt_input_t *inputs, tilt_output_t *outputs, 
                           int count, mapping_type_t mapping);

void example_usage(void);

#endif // TILT_COMPENSATION_ALGORITHM_H 