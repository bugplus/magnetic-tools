/* 3D mag calibration (auto) */
const float HARD_IRON[3] = {-11.099350f, 17.791209f, 34.538109f};
const float SOFT_IRON[3][3] = {
  {0.004377f, 0.000070f, -0.000177f},
  {0.000070f, 0.004763f, -0.000592f},
  {-0.000177f, -0.000592f, 0.004310f}
};