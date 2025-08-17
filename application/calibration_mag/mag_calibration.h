/* 3D mag calibration (forced concentric circle) */
const float HARD_IRON[3] = {-271.75742898f, 184.17725684f, 142.29753013f};
const float SOFT_IRON[3][3] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};