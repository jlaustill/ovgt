class ovgt
{
    private:
        static unsigned long count;
        static unsigned long lastMillis;
        static unsigned long thisMillis;
        static unsigned long thisDuration;
        static unsigned long loopCountLastMillis;

    public:
        static void setup();
        static void loop();
};
