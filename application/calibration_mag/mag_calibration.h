/* 3D mag calibration (forced concentric circle) */
const float HARD_IRON[3] = {-333.85866968f, 200.51857584f, -296.02064918f};
const float SOFT_IRON[3][3] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};