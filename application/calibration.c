#include <stdio.h>

// 磁力计数据结构
typedef struct {
    float raw_x;
    float raw_y;
} MagnetometerData;

// 校准参数结构
typedef struct {
    float scale_x;
    float scale_y;
    float center_x;
    float center_y;
} CalibrationParams;

// 单点校准函数
void calibrate_data_point(MagnetometerData *raw, CalibrationParams *params, MagnetometerData *calibrated) {
    // 应用缩放因子
    float scaled_x = raw->raw_x * params->scale_x;
    float scaled_y = raw->raw_y * params->scale_y;
    
    // 应用中心偏移
    calibrated->raw_x = scaled_x - params->center_x;
    calibrated->raw_y = scaled_y - params->center_y;
}

// 批量校准函数
void calibrate_data_batch(MagnetometerData raw_data[], 
                         MagnetometerData calibrated_data[],
                         CalibrationParams *params,
                         int num_points) {
    for (int i = 0; i < num_points; i++) {
        // 应用缩放因子
        float scaled_x = raw_data[i].raw_x * params->scale_x;
        float scaled_y = raw_data[i].raw_y * params->scale_y;
        
        // 应用中心偏移
        calibrated_data[i].raw_x = scaled_x - params->center_x;
        calibrated_data[i].raw_y = scaled_y - params->center_y;
    }
}

// 打印单个数据点
void print_data_point(const char* label, MagnetometerData *data) {
    printf("%s: X=%.4f, Y=%.4f\n", label, data->raw_x, data->raw_y);
}

// 打印数据集
void print_data_set(const char* label, MagnetometerData data[], int num_points) {
    printf("\n%s:\n", label);
    printf("Index\tX\t\tY\n");
    printf("----\t----------\t----------\n");
    
    for (int i = 0; i < num_points; i++) {
        printf("%4d\t%8.4f\t%8.4f\n", i+1, data[i].raw_x, data[i].raw_y);
    }
}

int main() {
    // 示例数据 - 类似图片中显示的圆形分布
    MagnetometerData raw_data[] = {
        {100.0, 200.0},
        {120.0, 180.0},
        {90.0,  220.0},
        {110.0, 190.0},
        {95.0,  210.0}
    };
    
    const int num_points = sizeof(raw_data) / sizeof(raw_data[0]);
    
    // 校准参数 - 类似图片中提到的缩放因子和圆心偏移
    CalibrationParams params = {
        .scale_x = 0.85f,    // X轴缩放因子
        .scale_y = 1.15f,    // Y轴缩放因子
        .center_x = 50.0f,   // X轴偏移量
        .center_y = 100.0f   // Y轴偏移量
    };
    
    // 创建存储校准数据的数组
    MagnetometerData calibrated_data[num_points];
    
    // 执行批量校准
    calibrate_data_batch(raw_data, calibrated_data, &params, num_points);
    
    // 打印原始数据 - 类似图片中的图1
    print_data_set("Raw Data (Fig1)", raw_data, num_points);
    
    // 打印校准参数
    printf("\nCalibration Parameters:\n");
    printf("Scale X: %.4f\n", params.scale_x);
    printf("Scale Y: %.4f\n", params.scale_y);
    printf("Center X: %.4f\n", params.center_x);
    printf("Center Y: %.4f\n", params.center_y);
    
    // 打印校准数据 - 类似图片中的图3
    print_data_set("Calibrated Data (Fig3)", calibrated_data, num_points);
    
    return 0;
}