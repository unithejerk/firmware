// Preflight observers receive null for light sleep and non-null for deep sleep; nonzero vetoes.
#include "Arduino.h"
#include "TestUtil.h"
#include "gps/GPS.h"
#include "sleep.h"
#include <unity.h>

class GPSPreflightTestShim : public GPS
{
  public:
    GPSPreflightTestShim() : GPS() {}

    void configure(bool threadEnabled, bool initFinished, GPSPowerState state)
    {
        enabled = threadEnabled;
        GPSInitFinished = initFinished;
        powerState = state;
    }

    void registerPreflight() { preflightSleepObserver.observe(&::preflightSleep); }
};

class PreflightRecorder : public Observer<void *>
{
  public:
    int calls = 0;
    void *lastArgument = nullptr;
    int result = 0;

  protected:
    int onNotify(void *argument) override
    {
        calls++;
        lastArgument = argument;
        return result;
    }
};

void setUp(void) {}
void tearDown(void) {}

void test_light_sleep_passes_null_to_observers()
{
    PreflightRecorder recorder;
    recorder.observe(&preflightSleep);

    TEST_ASSERT_TRUE(doPreflightSleep(false));
    TEST_ASSERT_EQUAL(1, recorder.calls);
    TEST_ASSERT_NULL(recorder.lastArgument);
}

void test_deep_sleep_passes_nonnull_to_observers()
{
    PreflightRecorder recorder;
    recorder.observe(&preflightSleep);

    TEST_ASSERT_TRUE(doPreflightSleep(true));
    TEST_ASSERT_EQUAL(1, recorder.calls);
    TEST_ASSERT_NOT_NULL(recorder.lastArgument);
}

void test_nonzero_observer_result_vetoes_sleep_and_can_clear()
{
    PreflightRecorder recorder;
    recorder.result = 1;
    recorder.observe(&preflightSleep);

    TEST_ASSERT_FALSE(doPreflightSleep(false));
    TEST_ASSERT_EQUAL(1, recorder.calls);

    recorder.result = 0;
    TEST_ASSERT_TRUE(doPreflightSleep(false));
    TEST_ASSERT_EQUAL(2, recorder.calls);
}

void test_veto_short_circuits_later_observers()
{
    PreflightRecorder veto;
    PreflightRecorder after;
    veto.result = 1;
    veto.observe(&preflightSleep);
    after.observe(&preflightSleep);

    TEST_ASSERT_FALSE(doPreflightSleep(false));
    TEST_ASSERT_EQUAL(1, veto.calls);
    TEST_ASSERT_EQUAL(0, after.calls);
}

void test_gps_vetoes_light_sleep_during_initialization()
{
    GPSPreflightTestShim gps;
    gps.configure(true, false, GPS_IDLE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;

    gps.registerPreflight();
    TEST_ASSERT_FALSE(doPreflightSleep(false));
}

void test_gps_vetoes_light_sleep_while_active()
{
    GPSPreflightTestShim gps;
    gps.configure(true, true, GPS_ACTIVE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;

    gps.registerPreflight();
    TEST_ASSERT_FALSE(doPreflightSleep(false));
}

void test_gps_allows_deep_sleep_while_active()
{
    GPSPreflightTestShim gps;
    gps.configure(true, true, GPS_ACTIVE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;

    gps.registerPreflight();
    TEST_ASSERT_TRUE(doPreflightSleep(true));
}

void test_gps_allows_light_sleep_when_disabled()
{
    GPSPreflightTestShim gps;
    gps.configure(false, true, GPS_ACTIVE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
    gps.registerPreflight();
    TEST_ASSERT_TRUE(doPreflightSleep(false));
}

void test_gps_allows_light_sleep_when_mode_is_disabled()
{
    GPSPreflightTestShim gps;
    gps.configure(true, true, GPS_ACTIVE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;
    gps.registerPreflight();
    TEST_ASSERT_TRUE(doPreflightSleep(false));
}

void test_gps_allows_light_sleep_when_inactive()
{
    GPSPreflightTestShim gps;
    gps.configure(true, true, GPS_IDLE);
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
    gps.registerPreflight();
    TEST_ASSERT_TRUE(doPreflightSleep(false));
}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_light_sleep_passes_null_to_observers);
    RUN_TEST(test_deep_sleep_passes_nonnull_to_observers);
    RUN_TEST(test_nonzero_observer_result_vetoes_sleep_and_can_clear);
    RUN_TEST(test_veto_short_circuits_later_observers);
    RUN_TEST(test_gps_vetoes_light_sleep_during_initialization);
    RUN_TEST(test_gps_vetoes_light_sleep_while_active);
    RUN_TEST(test_gps_allows_deep_sleep_while_active);
    RUN_TEST(test_gps_allows_light_sleep_when_disabled);
    RUN_TEST(test_gps_allows_light_sleep_when_mode_is_disabled);
    RUN_TEST(test_gps_allows_light_sleep_when_inactive);
    exit(UNITY_END());
}

void loop() {}
