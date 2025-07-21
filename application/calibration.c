
/*********************************************************************
 * 磁力计校准参数与接口 —— 2×2 非对角软铁矩阵版
 * 由上位机自动生成，勿手动修改数值
 *********************************************************************/
#include <math.h>
#include <stdio.h>

/* ---------- 1. 校准参数 ---------- */
#define HARD_IRON_OFFSET_X  (-74.774555f)
#define HARD_IRON_OFFSET_Y  (7.917627f)

static const float SOFT_IRON_MATRIX[2][2] = {
    {1.000000f, 0.000000f},
    {0.000000f, 1.019503f}
};

/* ---------- 2. 数据结构 ---------- */
typedef enum {
    MAPPING_TYPE_1 = 1,  /* X→Y, Y→X, Z→Z */
    MAPPING_TYPE_2 = 2,  /* X→X, Y→Y, Z→Z */
    MAPPING_TYPE_3 = 3   /* X→-X, Y→Y, Z→Z */
} mapping_type_t;

typedef struct {
    float mag_x, mag_y, mag_z;
    float pitch, roll, yaw;   /* 角度，单位：度 */
    mapping_type_t mapping;
} tilt_input_t;

typedef struct {
    float mx_comp, my_comp; /* 倾斜补偿后水平分量 */
    float mx_cal,  my_cal;  /* 校准后最终分量 */
} tilt_output_t;

/* ---------- 3. 坐标系映射 ---------- */
static inline void map_coordinates(float mx, float my, float mz,
                                   mapping_type_t m,
                                   float *ox, float *oy, float *oz)
{
    switch (m) {
        case MAPPING_TYPE_1: *ox = my; *oy = mx; *oz = mz; break;
        case MAPPING_TYPE_2: *ox = mx; *oy = my; *oz = mz; break;
        case MAPPING_TYPE_3: *ox = -mx; *oy = my; *oz = mz; break;
        default:             *ox = mx; *oy = my; *oz = mz; break;
    }
}

/* ---------- 4. 倾斜补偿 ---------- */
static inline void tilt_compensation(const tilt_input_t *in, tilt_output_t *out)
{
    float mx, my, mz;
    map_coordinates(in->mag_x, in->mag_y, in->mag_z, in->mapping, &mx, &my, &mz);

    float pr = in->pitch * (float)M_PI / 180.0f;
    float rr = in->roll  * (float)M_PI / 180.0f;

    float cp = cosf(pr), sp = sinf(pr);
    float cr = cosf(rr), sr = sinf(rr);

    out->mx_comp = mx * cp + mz * sp;
    out->my_comp = mx * sp * sr + my * cr - mz * cp * sr;
}

/* ---------- 5. 软铁/硬铁校准 ---------- */
static inline void calibrate_magnetometer(float *mx, float *my)
{
    /* 1. 硬铁偏移 */
    float hx = *mx - HARD_IRON_OFFSET_X;
    float hy = *my - HARD_IRON_OFFSET_Y;

    /* 2. 2×2 非对角软铁矩阵乘法 */
    *mx = SOFT_IRON_MATRIX[0][0] * hx + SOFT_IRON_MATRIX[0][1] * hy;
    *my = SOFT_IRON_MATRIX[1][0] * hx + SOFT_IRON_MATRIX[1][1] * hy;
}

/* ---------- 6. 完整处理 ---------- */
static inline void process_magnetometer_data(const tilt_input_t *in,
                                             tilt_output_t *out)
{
    tilt_compensation(in, out);
    calibrate_magnetometer(&out->mx_comp, &out->my_comp);
    out->mx_cal = out->mx_comp;
    out->my_cal = out->my_comp;
}

/* ---------- 7. 使用示例 ---------- */
void example_usage(void)
{
    tilt_input_t in = {
        .mag_x = 247.0f, .mag_y = 137.0f, .mag_z = -57.0f,
        .pitch = 62.46f, .roll = -1.01f, .yaw = -90.37f,
        .mapping = MAPPING_TYPE_1
    };
    tilt_output_t out;

    process_magnetometer_data(&in, &out);

    float heading = atan2f(out.my_cal, out.mx_cal) * 180.0f / (float)M_PI;
    if (heading < 0.0f) heading += 360.0f;

    printf("Compensated: mx=%.2f, my=%.2f\n", out.mx_comp, out.my_comp);
    printf("Calibrated : mx=%.2f, my=%.2f\n", out.mx_cal,  out.my_cal);
    printf("Heading    : %.1f°\n", heading);
}

/* 文件结束 */
