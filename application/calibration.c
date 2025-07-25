/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {-24.341703f, 167.147329f, 628.967929f};
const float SOFT_IRON[3][3] = {
  {0.006180f, 0.000088f, 0.000098f},
  {0.000088f, 0.005822f, -0.000647f},
  {0.000098f, -0.000647f, 0.002537f}
};