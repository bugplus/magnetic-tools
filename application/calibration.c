/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {-62.249286f, 114.401537f, 593.369935f};
const float SOFT_IRON[3][3] = {
  {0.006603f, 0.000073f, 0.000455f},
  {0.000073f, 0.006348f, -0.000269f},
  {0.000455f, -0.000269f, 0.002986f}
};