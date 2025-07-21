#include "calibration_algorithm.h"
#include <stdio.h>

// 示例：出厂校准流程
void factory_calibration_example(void) {
    printf("=== 出厂校准流程 ===\n");
    
    // 1. 开始校准
    if (!factory_calibration_start()) {
        printf("校准启动失败\n");
        return;
    }
    
    // 2. 模拟数据采集
    sensor_data_t sensor_data;
    uint32_t data_count = 0;
    
    printf("请按以下步骤进行校准：\n");
    printf("1. 水平旋转360°（2-3圈）\n");
    printf("2. 船头垫高20°旋转360°（2-3圈）\n");
    printf("3. 船侧垫高20°旋转360°（2-3圈）\n");
    
    // 模拟数据采集过程
    while (data_count < 300) {  // 采集300个数据点
        // 这里应该从实际传感器读取数据
        // 示例数据
        sensor_data.mag_x = 25.0f + (data_count % 100) * 0.1f;
        sensor_data.mag_y = 15.0f + (data_count % 100) * 0.1f;
        sensor_data.mag_z = -45.0f;
        sensor_data.acc_x = 0.0f;
        sensor_data.acc_y = 0.0f;
        sensor_data.acc_z = 9.8f;
        sensor_data.gyro_x = 0.0f;
        sensor_data.gyro_y = 0.0f;
        sensor_data.gyro_z = 0.0f;
        
        // 模拟不同姿态
        if (data_count < 100) {
            // 水平姿态
            sensor_data.pitch = 0.0f;
            sensor_data.roll = 0.0f;
        } else if (data_count < 200) {
            // 船头垫高20°
            sensor_data.pitch = 20.0f;
            sensor_data.roll = 0.0f;
        } else {
            // 船侧垫高20°
            sensor_data.pitch = 0.0f;
            sensor_data.roll = 20.0f;
        }
        
        // 添加校准数据
        if (factory_calibration_add_data(&sensor_data)) {
            data_count++;
        }
        
        // 显示进度
        if (data_count % 50 == 0) {
            printf("已采集 %d 个数据点\n", data_count);
        }
    }
    
    // 3. 完成校准
    factory_calibration_t factory_result;
    if (factory_calibration_finish(&factory_result)) {
        printf("出厂校准完成！\n");
        printf("校准质量: %.2f\n", factory_result.calibration_quality);
        printf("数据点数: %d\n", factory_result.data_points);
        printf("中心偏移: (%.2f, %.2f)\n", factory_result.center_x, factory_result.center_y);
        printf("缩放因子: (%.2f, %.2f)\n", factory_result.scale_x, factory_result.scale_y);
        
        // 保存校准参数
        if (factory_calibration_save(&factory_result)) {
            printf("校准参数已保存\n");
        }
    } else {
        printf("出厂校准失败\n");
    }
}

// 示例：用户校准流程
void user_calibration_example(void) {
    printf("=== 用户校准流程 ===\n");
    
    // 1. 加载出厂校准参数
    factory_calibration_t factory_calib;
    if (!factory_calibration_load(&factory_calib)) {
        printf("未找到出厂校准参数，请先进行出厂校准\n");
        return;
    }
    
    // 2. 开始用户校准
    if (!user_calibration_start()) {
        printf("用户校准启动失败\n");
        return;
    }
    
    printf("请水平旋转360°（1圈）\n");
    
    // 3. 模拟数据采集
    sensor_data_t sensor_data;
    uint32_t data_count = 0;
    
    while (data_count < 100) {  // 采集100个数据点
        // 模拟传感器数据
        sensor_data.mag_x = 25.0f + (data_count % 100) * 0.1f;
        sensor_data.mag_y = 15.0f + (data_count % 100) * 0.1f;
        sensor_data.mag_z = -45.0f;
        sensor_data.acc_x = 0.0f;
        sensor_data.acc_y = 0.0f;
        sensor_data.acc_z = 9.8f;
        sensor_data.gyro_x = 0.0f;
        sensor_data.gyro_y = 0.0f;
        sensor_data.gyro_z = 0.0f;
        sensor_data.pitch = 0.0f;  // 水平姿态
        sensor_data.roll = 0.0f;
        
        if (user_calibration_add_data(&sensor_data)) {
            data_count++;
        }
    }
    
    // 4. 完成用户校准
    user_calibration_t user_result;
    if (user_calibration_finish(&user_result)) {
        printf("用户校准完成！\n");
        printf("环境偏移: (%.2f, %.2f)\n", user_result.env_offset_x, user_result.env_offset_y);
        
        // 保存用户校准参数
        if (user_calibration_save(&user_result)) {
            printf("用户校准参数已保存\n");
        }
    } else {
        printf("用户校准失败\n");
    }
}

// 示例：实时应用校准
void real_time_application_example(void) {
    printf("=== 实时应用示例 ===\n");
    
    // 1. 加载校准参数
    factory_calibration_t factory_calib;
    user_calibration_t user_calib;
    
    if (!factory_calibration_load(&factory_calib)) {
        printf("未找到出厂校准参数\n");
        return;
    }
    
    user_calibration_load(&user_calib);  // 用户校准可选
    
    // 2. 模拟实时数据
    sensor_data_t real_time_data;
    calibration_result_t result;
    
    // 模拟不同姿态的实时数据
    for (int i = 0; i < 10; i++) {
        // 模拟传感器数据
        real_time_data.mag_x = 25.0f + i * 2.0f;
        real_time_data.mag_y = 15.0f + i * 1.5f;
        real_time_data.mag_z = -45.0f;
        real_time_data.acc_x = 0.0f;
        real_time_data.acc_y = 0.0f;
        real_time_data.acc_z = 9.8f;
        real_time_data.gyro_x = 0.0f;
        real_time_data.gyro_y = 0.0f;
        real_time_data.gyro_z = 0.0f;
        real_time_data.pitch = i * 5.0f;  // 模拟不同姿态
        real_time_data.roll = i * 3.0f;
        
        // 应用校准
        if (apply_calibration(&real_time_data, &factory_calib, &user_calib, &result)) {
            printf("数据 %d: 方位角=%.1f°, 俯仰=%.1f°, 横滚=%.1f°, 质量=%.2f\n",
                   i, result.heading, result.pitch, result.roll, result.calibration_quality);
        }
    }
}

// 主函数示例
int main(void) {
    printf("船模地磁校准系统\n");
    printf("================\n");
    
    // 1. 出厂校准（生产时执行）
    factory_calibration_example();
    
    printf("\n");
    
    // 2. 用户校准（用户使用时执行）
    user_calibration_example();
    
    printf("\n");
    
    // 3. 实时应用（正常使用时执行）
    real_time_application_example();
    
    return 0;
} 