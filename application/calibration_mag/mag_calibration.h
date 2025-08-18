/* 3-step mag calibration (fixed XY center, optimize z & radius) */
const float HARD_IRON[3] = {-331.95855668f, 198.31643286f, 132.36440436f};
const float SOFT_IRON[3][3] = {
  {0.00370687f, 0.0f, 0.0f},
  {0.0f, 0.00370687f, 0.0f},
  {0.0f, 0.0f, 0.00370687f}
};