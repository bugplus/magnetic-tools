/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {6.596202f, 8.887894f, 14.068686f};
const float SOFT_IRON[3][3] = {
  {0.004671f, 0.000107f, 0.000296f},
  {0.000107f, 0.004282f, -0.000050f},
  {0.000296f, -0.000050f, 0.006423f}
};