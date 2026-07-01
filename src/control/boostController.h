#ifndef BoostController_h
#define BoostController_h

class BoostController {
    public:
        static void Initialize();
        static void update();
        // Runtime tuning via serial. These mutate the BPR controller's config so
        // the target/gains can be swept on the truck without reflashing. They are
        // harmless no-effect setters when built in map mode.
        static void setBprTarget(float value);
        static void setKp(float value);
        static void setKi(float value);
        static float getBprTarget();
        // Diagnostic accessors: expose the BPR controller's hidden internal state
        // for telemetry so we can see WHY boost sticks at low load (region the
        // controller is in, and the integral windup that precedes the snap).
        static bool getInPiRegion();
        static float getIntegralTerm();
        static void printParams();
};

#endif
