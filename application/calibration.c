/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {-12.450149f, -36.017094f, 12.623722f};
const float SOFT_IRON[3][3] = {
  {0.004486f, 0.000099f, -0.000152f},
  {0.000099f, 0.004638f, 0.000552f},
  {-0.000152f, 0.000552f, 0.006112f}
};