/* 3-step mag calibration (Step-0 forced circle, sphere axis through circle center) */
const float HARD_IRON[3] = {14.49048302f, -2479.70468560f, -6655.35127283f};
const float SOFT_IRON[3][3] = {
  {0.00096206f, 0.0f, 0.0f},
  {0.0f, 0.00096206f, 0.0f},
  {0.0f, 0.0f, 0.00096206f}
};