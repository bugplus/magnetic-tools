
// Auto-generated calibration code
void calibrate(float* x, float* y) {
    // Hard iron offset
    *x -= 32.788931f;
    *y -= 6.571429f;
    // Soft iron scale
    *y *= 0.980909f;
}
