#ifndef exhaustBrakeController_h
#define exhaustBrakeController_h

class ExhaustBrakeController {
    public:
        static void Initialize();
        // Returns true if the brake is active and has written
        // appData.actuatorDemandedPosition this cycle. manualMode is passed in
        // because it is private to the ovgt orchestrator.
        static bool update(bool manualMode);
};

#endif
