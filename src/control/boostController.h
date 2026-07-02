#ifndef BoostController_h
#define BoostController_h

class BoostController {
    public:
        static void Initialize();
        static void update();
        // Tuning (bprTarget/kp/ki) is compile-time only — edit boostController.cpp
        // and reflash. No runtime setters: live config changes are a driving hazard.
        static float getBprTarget();
        // Diagnostic accessors: expose the BPR controller's hidden internal state
        // for telemetry so we can see WHY boost sticks at low load (region the
        // controller is in, and the integral windup that precedes the snap).
        static bool getInPiRegion();
        static float getIntegralTerm();
};

#endif
