/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {-6.417485f, -38.929059f, 16.072515f};
const float SOFT_IRON[3][3] = {
  {0.004365f, -0.000007f, -0.000442f},
  {-0.000007f, 0.004543f, 0.000397f},
  {-0.000442f, 0.000397f, 0.005450f}
};