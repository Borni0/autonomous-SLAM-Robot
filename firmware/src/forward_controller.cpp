#include "forward_controller.h"

ForwardController::ForwardController() {
    _cfg.forwardPct = 55;
    _cfg.pivotPct = 0;
    _cfg.alignThresholdRad = 0.20f;
    _cfg.headingGain = 80.0f;
    _cfg.maxSteerOffset = 35.0f;
}

void ForwardController::begin(const ForwardControllerConfig& cfg) {
    _cfg = cfg;
}

MotorCommand ForwardController::compute(float headingErrRad, float distanceM) const {
    MotorCommand cmd;

    if (distanceM <= 0.0f) {
        cmd.left = 0;
        cmd.right = 0;
        return cmd;
    }

    // Steering: a proportional controller in [percent / rad].
    float steer = _cfg.headingGain * headingErrRad;

    if (steer >  _cfg.maxSteerOffset) steer =  _cfg.maxSteerOffset;
    if (steer < -_cfg.maxSteerOffset) steer = -_cfg.maxSteerOffset;

    if (fabsf(headingErrRad) > _cfg.alignThresholdRad) {
        // Mostly turning in place.
        float pivot = _cfg.pivotPct;
        if (pivot > 0) {
            cmd.left  = (int8_t)(-pivot);
            cmd.right = (int8_t)( pivot);
        } else {
            // Fall back to a slow creep with strong steering.
            cmd.left  = (int8_t)(_cfg.forwardPct - steer);
            cmd.right = (int8_t)(_cfg.forwardPct + steer);
        }
    } else {
        cmd.left  = (int8_t)(_cfg.forwardPct - steer);
        cmd.right = (int8_t)(_cfg.forwardPct + steer);
    }

    if (cmd.left  >  100) cmd.left  =  100;
    if (cmd.left  < -100) cmd.left  = -100;
    if (cmd.right >  100) cmd.right =  100;
    if (cmd.right < -100) cmd.right = -100;

    return cmd;
}