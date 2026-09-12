#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPS
#include "GPS.h"
#endif

#include "Default.h"
#include "MeshRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerMon.h"
#include "TransmitHistory.h"
#include "detect/LoRaRadioType.h"
#include "error.h"
#include "main.h"
#include "meshUtils.h"
#include "modules/StatusLEDModule.h"
#include "sleep.h"
#include "target_specific.h"

#if HAS_WIFI
#include "mesh/wifi/WiFiAPClient.h"
#endif

#ifdef ARCH_ESP32
#include "concurrency/Lock.h"
#include "esp_pm.h"
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
#include <atomic>
#include <freertos/portmacro.h>
#endif
#include "rom/rtc.h"
#include <RadioLib.h>
#include <driver/rtc_io.h>
#include <driver/uart.h>
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
#include <hal/rtc_io_hal.h>
#endif

esp_sleep_source_t wakeCause; // the reason we booted this time
#endif
#include "Throttle.h"

#ifdef USE_PCA95X5
#include PCA95X5_INC
extern PCA95X5_CLS io;
#endif

#ifdef HAS_PPM
#include <XPowersLib.h>
extern XPowersPPM *PPM;
#endif

#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend 0
#endif

/// Called to ask any observers if they want to veto sleep. Return 1 to veto or 0 to allow sleep to happen
Observable<void *> preflightSleep;

/// Called to tell observers we are now entering (deep) sleep and you should prepare.  Must return 0
Observable<void *> notifyDeepSleep;

/// Called to tell observers we are rebooting ASAP.  Must return 0
Observable<void *> notifyReboot;

/// True when the current role/config make PowerFSM's stateLS reachable at all. See sleep.h.
bool isAutoLightSleepEligible()
{
#if defined(ARCH_ESP32) && HAS_WIFI && !defined(MESHTASTIC_EXCLUDE_WIFI)
    const bool powerSavingCandidate =
        config.power.is_power_saving || IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_ROUTER,
                                                  meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
    const bool isTrackerOrSensor =
        IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_TRACKER,
                  meshtastic_Config_DeviceConfig_Role_TAK_TRACKER, meshtastic_Config_DeviceConfig_Role_SENSOR);
    return powerSavingCandidate && !isWifiAvailable() && !isTrackerOrSensor;
#else
    return false;
#endif
}

#ifdef ARCH_ESP32
/// Called to tell observers that light sleep is about to begin
Observable<void *> notifyLightSleep;

/// Called to tell observers that light sleep has just ended, and why it ended
Observable<esp_sleep_wakeup_cause_t> notifyLightSleepEnd;

#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
static bool dynamicLightSleepReady;        // capability, decided once in initLightSleep()
static uint8_t autoLightSleepFailureCount; // bounded runtime-retry counter; see isAutoLightSleepAvailable()
static constexpr uint8_t MAX_AUTO_LIGHT_SLEEP_FAILURES = 3;
static portMUX_TYPE autoLightSleepWakeMux = portMUX_INITIALIZER_UNLOCKED;
static bool autoLightSleepButtonWakePending; // GPIO wake w/ active input; drives the stop/reattach cycle
static esp_sleep_wakeup_cause_t autoLightSleepLastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
static std::atomic<bool> autoLightSleepLoraWakeConfigured{false};
static std::atomic<bool> autoLightSleepPowerServicePending{
    false}; // GPIO or EXT1 wake; see consumeAutoLightSleepPowerServiceWake()
static gpio_num_t autoLightSleepLoraWakePinCached;
static bool autoLightSleepLoraWakePinValid;
#endif
#endif

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;

// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

/**
 * Control CPU core speed (80MHz vs 240MHz)
 *
 * We leave CPU at full speed during init, but once loop is called switch to low speed (for a 50% power savings)
 *
 * On PM builds this drives an ESP_PM_CPU_FREQ_MAX lock instead of poking the clock
 * directly: setCpuFrequencyMhz() bypasses esp_pm and desyncs its bookkeeping.
 */
void setCPUFast(bool on)
{
#if defined(ARCH_ESP32) && HAS_ESP32_PM_SUPPORT
    // PM builds: an ESP_PM_CPU_FREQ_MAX lock pins the scheduler's top tier.
    // setCpuFrequencyMhz() would bypass esp_pm and desync its bookkeeping.
    static esp_pm_lock_handle_t cpuFreqMaxLock;
    static bool lockHeld; // acquire/release are counting; callers are not balanced

    // WiFi is unstable below 240 MHz, so pin max whenever WiFi is in use
    bool wantFast = on;
#if HAS_WIFI
    wantFast = wantFast || isWifiAvailable();
#endif

    if (wantFast) {
        if (!lockHeld) {
            if (!cpuFreqMaxLock && esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_fast", &cpuFreqMaxLock) != ESP_OK)
                return;
            esp_err_t result = esp_pm_lock_acquire(cpuFreqMaxLock);
            if (result != ESP_OK) {
                LOG_ERROR("esp_pm_lock_acquire(cpu_fast) result %d", result);
                return;
            }
            lockHeld = true;
        }
    } else if (lockHeld) {
        esp_err_t result = esp_pm_lock_release(cpuFreqMaxLock);
        if (result == ESP_OK)
            lockHeld = false;
        else
            LOG_ERROR("esp_pm_lock_release(cpu_fast) result %d", result);
    }
#elif defined(ARCH_ESP32) && HAS_WIFI && !HAS_TFT && !defined(T_LORA_PAGER) && !defined(T_DECK)

    if (isWifiAvailable()) {
        /*
         *
         * There's a newly introduced bug in the espressif framework where WiFi is
         *   unstable when the frequency is less than 240MHz.
         *
         *   This mostly impacts WiFi AP mode but we'll bump the frequency for
         *     all WiFi use cases.
         * (Added: Dec 23, 2021 by Jm Casler)
         */
#ifndef CONFIG_IDF_TARGET_ESP32C3
        LOG_DEBUG("Set CPU to 240MHz because WiFi is in use");
        setCpuFrequencyMhz(240);
#endif
        return;
    }

// The Heltec LORA32 V1 runs at 26 MHz base frequency and doesn't react well to switching to 80 MHz...
#if !defined(ARDUINO_HELTEC_WIFI_LORA_32) && !defined(CONFIG_IDF_TARGET_ESP32C3)
    setCpuFrequencyMhz(on ? 240 : 80);
#endif

#endif
}

// Perform power on init that we do on each wake from deep sleep
void initDeepSleep()
{
#ifdef ARCH_ESP32
    bootCount++;
    const char *reason;
    wakeCause = esp_sleep_get_wakeup_cause();

    switch (wakeCause) {
    case ESP_SLEEP_WAKEUP_EXT0:
        reason = "ext0 RTC_IO";
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        reason = "ext1 RTC_CNTL";
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        reason = "timer";
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        reason = "touchpad";
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        reason = "ULP program";
        break;
    default:
        reason = "reset";
        break;
    }
    /*
      Not using yet because we are using wake on all buttons being low

      wakeButtons = esp_sleep_get_ext1_wakeup_status();       // If one of these buttons is set it was the reason we woke
      if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons) // we must have been using the 'all buttons rule for waking' to
      support busted boards, assume button one was pressed wakeButtons = ((uint64_t)1) << buttons.gpios[0];
      */

#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
    // If we booted because our timer ran out or the user pressed reset, send those as fake events
    RESET_REASON hwReason = rtc_get_reset_reason(0);

#ifdef CONFIG_IDF_TARGET_ESP32P4
    if (hwReason == BROWN_OUT_RESET)
        reason = "brownout";
    else if (hwReason == HP_CORE_HP_WDT_RESET)
        reason = "taskWatchdog";
    else if (hwReason == HP_CORE_LP_WDT_RESET)
        reason = "intWatchdog";
    else if (hwReason == CHIP_LP_WDT_RESET)
        reason = "chipWatchdog";
    else if (hwReason == SUPER_WDT_RESET)
        reason = "superWatchdog";
    else if (hwReason == HP_SYS_HP_WDT_RESET)
        reason = "systemWatchdog";
    else if (hwReason == HP_SYS_LP_WDT_RESET)
        reason = "systemLowPowerWatchdog";
#else
    if (hwReason == RTCWDT_BROWN_OUT_RESET)
        reason = "brownout";
    else if (hwReason == RTCWDT_RTC_RESET)
        reason = "rtcWatchdog";
    else if (hwReason == TG0WDT_SYS_RESET)
        reason = "taskWatchdog";
    else if (hwReason == TG1WDT_SYS_RESET)
        reason = "intWatchdog";
#endif
    LOG_INFO("Booted, wake cause %d (boot count %d), reset_reason=%s", wakeCause, bootCount, reason);
#endif

#if SOC_RTCIO_HOLD_SUPPORTED
    // If waking from sleep, release any and all RTC GPIOs
    if (wakeCause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        LOG_DEBUG("Disable any holds on RTC IO pads");
        for (uint8_t i = 0; i <= GPIO_NUM_MAX; i++) {
            if (rtc_gpio_is_valid_gpio((gpio_num_t)i))
                rtc_gpio_hold_dis((gpio_num_t)i);

            // ESP32 (original)
            else if (GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)i))
                gpio_hold_dis((gpio_num_t)i);
        }
    }
#endif

#endif
}

bool doPreflightSleep(bool deepSleep)
{
    // Observers only get a void*: non-NULL means the hardware (radio) is about to be powered
    // down (deep sleep / shutdown), NULL means a light sleep where the radio keeps running
    static const bool deepSleepFlag = true;
    if (preflightSleep.notifyObservers(deepSleep ? (void *)&deepSleepFlag : NULL) != 0)
        return false; // vetoed
    else
        return true;
}

/// Tell devices we are going to sleep and wait for them to handle things
static void waitEnterSleep(bool skipPreflight, bool deepSleep)
{
    if (!skipPreflight) {
        uint32_t now = millis();
        while (!doPreflightSleep(deepSleep)) {
            delay(100); // Kinda yucky - wait until radio says say we can shutdown (finished in process sends/receives)

            if (!Throttle::isWithinTimespanMs(now,
                                              THIRTY_SECONDS_MS)) { // If we wait too long just report an error and go to sleep
                RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_SLEEP_ENTER_WAIT);
                assert(0); // FIXME - for now we just restart, need to fix bug #167
                break;
            }
        }
    }

    // Code that still needs to be moved into notifyObservers
    console->flush();          // send all our characters before we stop cpu clock
    setBluetoothEnable(false); // has to be off before calling light sleep
}

void doDeepSleep(uint32_t msecToWake, bool skipPreflight = false, bool skipSaveNodeDb = false)
{
    if (INCLUDE_vTaskSuspend && (msecToWake == portMAX_DELAY)) {
        LOG_INFO("Enter deep sleep forever");
    } else {
        LOG_INFO("Enter deep sleep for %u seconds", msecToWake / 1000);
    }

    // not using wifi yet, but once we are this is needed to shutoff the radio hw
    // esp_wifi_stop();
    waitEnterSleep(skipPreflight, true);

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_BLUETOOTH
    // Full shutdown of bluetooth hardware
    if (nimbleBluetooth)
        nimbleBluetooth->deinit();
#endif

#ifdef ARCH_ESP32
    if (!shouldLoraWake(msecToWake))
        notifyDeepSleep.notifyObservers(NULL);
#else
    notifyDeepSleep.notifyObservers(NULL);
#endif

    powerMon->setState(meshtastic_PowerMon_State_CPU_DeepSleep);
    if (screen)
        screen->doDeepSleep(); // datasheet says this will draw only 10ua

    if (!skipSaveNodeDb) {
        nodeDB->saveToDisk();
    }

    // Persist broadcast transmit times so throttle survives reboot
    if (transmitHistory)
        transmitHistory->saveToDisk();

#ifdef PIN_POWER_EN
    digitalWrite(PIN_POWER_EN, LOW);
    pinMode(PIN_POWER_EN, INPUT); // power off peripherals
#endif

#ifdef RAK_WISMESH_TAP_V2
    digitalWrite(SDCARD_CS, LOW);
#endif

#if defined(TRACKER_T1000_E) || defined(MESH_TRACKER_X1)
#ifdef GNSS_AIROHA
    digitalWrite(GPS_VRTC_EN, LOW);
    digitalWrite(PIN_GPS_RESET, LOW);
    digitalWrite(GPS_SLEEP_INT, LOW);
    digitalWrite(GPS_RTC_INT, LOW);
#ifdef GPS_RESETB_OUT
    pinMode(GPS_RESETB_OUT, OUTPUT);
    digitalWrite(GPS_RESETB_OUT, LOW);
#endif
#endif

#ifdef BUZZER_EN_PIN
    digitalWrite(BUZZER_EN_PIN, LOW);
#endif

#ifdef PIN_DRV_EN
    digitalWrite(PIN_DRV_EN, LOW);
#endif

#ifdef PIN_3V3_EN
    digitalWrite(PIN_3V3_EN, LOW);
#endif
#ifdef PIN_WD_EN
    digitalWrite(PIN_WD_EN, LOW);
#endif
#endif
    statusLEDModule->setPowerLED(false);
#ifdef RESET_OLED
    digitalWrite(RESET_OLED, 1); // put the display in reset before killing its power
#endif

#if defined(VEXT_ENABLE)
    digitalWrite(VEXT_ENABLE, !VEXT_ON_VALUE); // turn on the display power
#endif

#ifdef ARCH_ESP32
    if (shouldLoraWake(msecToWake)) {
        enableLoraInterrupt();
    }
#ifdef BUTTON_PIN
    // Avoid leakage through button pin
    if (GPIO_IS_VALID_OUTPUT_GPIO(BUTTON_PIN)) {
#ifdef BUTTON_NEED_PULLUP
        pinMode(BUTTON_PIN, INPUT_PULLUP);
#else
        pinMode(BUTTON_PIN, INPUT);
#endif
        // A held pad ignores ext1_wakeup_prepare()'s re-route to RTC, so never hold the pin we wake on.
        if (config.device.button_gpio && config.device.button_gpio != BUTTON_PIN)
            gpio_hold_en((gpio_num_t)BUTTON_PIN);
    }
#endif
#ifdef SENSECAP_INDICATOR
    // Portexpander definition does not pass GPIO_IS_VALID_OUTPUT_GPIO
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    gpio_hold_en((gpio_num_t)LORA_CS);
#elif defined(ELECROW_PANEL)
    // Elecrow panels do not use LORA_CS, do nothing
#else
    if (GPIO_IS_VALID_OUTPUT_GPIO(LORA_CS)) {
        // LoRa CS (RADIO_NSS) needs to stay HIGH, even during deep sleep
        pinMode(LORA_CS, OUTPUT);
        digitalWrite(LORA_CS, HIGH);
        gpio_hold_en((gpio_num_t)LORA_CS);
    }
#endif
#endif

#ifdef HAS_PPM
    if (PPM) {
        // BQ25896 PMIC shutdown is a hard power-off state.
        // Only use it for "sleep forever" / explicit shutdown, because timed deep sleep
        // must remain wakeable by RTC timer.
        if (msecToWake == portMAX_DELAY) {
            LOG_INFO("PPM shutdown");
            console->flush();
            PPM->shutdown();
        }
    }
#endif

#ifdef HAS_PMU
    if (pmu_found && PMU) {
        // Obsolete comment: from back when we we used to receive lora packets while CPU was in deep sleep.
        // We no longer do that, because our light-sleep current draws are low enough and it provides fast start/low cost
        // wake.  We currently use deep sleep only for 'we want our device to actually be off - because our battery is
        // critically low'.  So in deep sleep we DO shut down power to LORA (and when we boot later we completely reinit it)
        //
        // No need to turn this off if the power draw in sleep mode really is just 0.2uA and turning it off would
        // leave floating input for the IRQ line
        // If we want to leave the radio receiving in would be 11.5mA current draw, but most of the time it is just waiting
        // in its sequencer (true?) so the average power draw should be much lower even if we were listening for packets
        // all the time.
        PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);

        uint8_t model = PMU->getChipModel();
        if (model == XPOWERS_AXP2101) {
            if (HW_VENDOR == meshtastic_HardwareModel_TBEAM) {
                // t-beam v1.2 radio power channel
                PMU->disablePowerOutput(XPOWERS_ALDO2); // lora radio power channel
            } else if (HW_VENDOR == meshtastic_HardwareModel_LILYGO_TBEAM_S3_CORE ||
                       HW_VENDOR == meshtastic_HardwareModel_T_WATCH_S3 || HW_VENDOR == meshtastic_HardwareModel_T_WATCH_ULTRA) {
                PMU->disablePowerOutput(XPOWERS_ALDO3); // lora radio power channel
            }
        } else if (model == XPOWERS_AXP192) {
            // t-beam v1.1 radio power channel
            PMU->disablePowerOutput(XPOWERS_LDO2); // lora radio power channel
        }
        if (msecToWake == portMAX_DELAY) {
            LOG_INFO("PMU shutdown");
            console->flush();
            PMU->shutdown();
        }
    }
#endif

#if !MESHTASTIC_EXCLUDE_I2C && defined(ARCH_ESP32) && defined(I2C_SDA)
    // Added by https://github.com/meshtastic/firmware/pull/4418
    // Possibly to support Heltec Capsule Sensor?
    Wire.end();
    pinMode(I2C_SDA, ANALOG);
    pinMode(I2C_SCL, ANALOG);
#endif

    console->flush();
    cpuDeepSleep(msecToWake);
}

#ifdef ARCH_ESP32
#if HAS_ESP32_PM_SUPPORT
static esp_pm_lock_handle_t pmLightSleepLock;
static bool pmLightSleepLockHeld; // true while the "meshtastic" lock blocks auto sleep
#endif
static concurrency::Lock *lightSleepLock;

// Wake cause of the most recent *light* sleep. Distinct from the global wakeCause,
// which holds the deep-sleep/reset boot reason and is read at render time (Screen.cpp).
static esp_sleep_wakeup_cause_t lightSleepWakeCause;

#ifdef BUTTON_PIN
#ifndef BUTTON_ACTIVE_LOW
#define BUTTON_ACTIVE_LOW true
#endif
#endif
#ifdef CANCEL_BUTTON_PIN
#ifndef CANCEL_BUTTON_ACTIVE_LOW
#define CANCEL_BUTTON_ACTIVE_LOW true
#endif
#endif
#ifdef DOWN_BUTTON_PIN
#ifndef DOWN_BUTTON_ACTIVE_LOW
#define DOWN_BUTTON_ACTIVE_LOW true
#endif
#endif

static bool IRAM_ATTR inputLevelActive(gpio_num_t pin, bool activeLow)
{
    return GPIO_IS_VALID_GPIO(pin) && (gpio_get_level(pin) == (activeLow ? 0 : 1));
}

static bool IRAM_ATTR autoLightSleepInputLevelActive()
{
#ifdef BUTTON_PIN
    if (inputLevelActive((gpio_num_t)(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN), BUTTON_ACTIVE_LOW))
        return true;
#endif
#ifdef ALT_BUTTON_PIN
    if (inputLevelActive((gpio_num_t)ALT_BUTTON_PIN, ALT_BUTTON_ACTIVE_LOW))
        return true;
#endif
#ifdef CANCEL_BUTTON_PIN
    if (inputLevelActive((gpio_num_t)CANCEL_BUTTON_PIN, CANCEL_BUTTON_ACTIVE_LOW))
        return true;
#endif
#ifdef DOWN_BUTTON_PIN
    if (inputLevelActive((gpio_num_t)DOWN_BUTTON_PIN, DOWN_BUTTON_ACTIVE_LOW))
        return true;
#endif
#ifdef ROTARY_PRESS
    if (inputLevelActive((gpio_num_t)ROTARY_PRESS, true))
        return true;
#endif
#ifdef KB_INT
#if KB_INT_WAKE_ON_HIGH
    if (inputLevelActive((gpio_num_t)KB_INT, false))
        return true;
#else
    if (inputLevelActive((gpio_num_t)KB_INT, true))
        return true;
#endif
#endif
#ifdef BOARD_PCA9535_INT
    if (inputLevelActive((gpio_num_t)BOARD_PCA9535_INT, true))
        return true;
#endif
#if defined(WAKE_ON_TOUCH) && defined(SCREEN_TOUCH_INT)
    if (inputLevelActive((gpio_num_t)SCREEN_TOUCH_INT, true))
        return true;
#endif
#if defined(INPUTDRIVER_TWO_WAY_ROCKER_BTN)
    if (inputLevelActive((gpio_num_t)INPUTDRIVER_TWO_WAY_ROCKER_BTN, true))
        return true;
#elif defined(INPUTDRIVER_ENCODER_BTN)
    if (inputLevelActive((gpio_num_t)INPUTDRIVER_ENCODER_BTN, true))
        return true;
#endif
    return false;
}

/// Tear down everything enableButtonInterrupt()/enableLoraInterrupt() armed.
static void disableWakeInterrupts(bool gpioWakeArmed);
static void configureLoraSleepHardware();

/**
 * enter light sleep (preserves ram but stops everything about CPU) for msecToWake ms.
 *
 * For the esp_pm auto-sleep controls see startAutoLightSleep()/stopAutoLightSleep().
 *
 * Returns the wake cause of the most recent light sleep.
 */
esp_sleep_wakeup_cause_t doLightSleep(uint32_t msecToWake)
{
    if (!lightSleepLock)
        return ESP_SLEEP_WAKEUP_UNDEFINED;
    lightSleepLock->lock();

    // LORA_DIO1 is an extended IO pin (on an I/O expander). Setting it as a wake-up pin will cause problems,
    // such as the device not entering light sleep. Boards opt in with LORA_DIO1_EXTENDED_IO in their variant.
#if defined(LORA_DIO1_EXTENDED_IO)
    lightSleepWakeCause = ESP_SLEEP_WAKEUP_TIMER;
    lightSleepLock->unlock();
    return lightSleepWakeCause;
#endif

    // Explicit timed sleep (the legacy stateLS loop). On dynamic builds this is
    // only reachable when called directly with a timeout while the lock is held,
    // so hold the NO_LIGHT_SLEEP lock across it too: PM must not race us.
    waitEnterSleep(false, false);
    notifyLightSleep.notifyObservers(NULL); // Button interrupts are detached here

    // NOTE! ESP docs say we must disable bluetooth and wifi before light sleep
#if SOC_PM_SUPPORT_RTC_PERIPH_PD
    // We want RTC peripherals to stay on
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#endif

    // Arm wake sources on every entry; torn down on every exit below.
    enableLoraInterrupt();
    enableButtonInterrupt();

    auto res = esp_sleep_enable_gpio_wakeup();
    if (res != ESP_OK) {
        LOG_ERROR("esp_sleep_enable_gpio_wakeup result %d", res);
    }
    res = esp_sleep_enable_timer_wakeup((uint64_t)msecToWake * 1000LL);
    if (res != ESP_OK) {
        LOG_ERROR("esp_sleep_enable_timer_wakeup result %d", res);
    }

    console->flush();
    res = esp_light_sleep_start();
    if (res != ESP_OK) {
        LOG_ERROR("esp_light_sleep_start result %d", res);
    }
    // Tear down everything armed above so the PM auto path does not fire on stale sources.
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    disableWakeInterrupts(true);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    notifyLightSleepEnd.notifyObservers(cause); // Button interrupts are reattached here

    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        LOG_INFO("Exit light sleep gpio");
        // If we woke because of a GPIO, it's possible power needs to run to handle.
        power->setIntervalFromNow(0);
        runASAP = true;
    } else {
        LOG_INFO("Exit light sleep cause: %d", cause);
    }
    lightSleepWakeCause = cause;
    lightSleepLock->unlock();
    return cause;
}

#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
static void IRAM_ATTR clearAutoLightSleepButtonWake()
{
    portENTER_CRITICAL_SAFE(&autoLightSleepWakeMux);
    autoLightSleepButtonWakePending = false;
    autoLightSleepLastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
    portEXIT_CRITICAL_SAFE(&autoLightSleepWakeMux);
}

// Record each wake cause and latch active GPIO input separately from EXT1 radio wakes.
static void IRAM_ATTR recordAutoLightSleepWake(esp_sleep_wakeup_cause_t cause, bool buttonWake)
{
    portENTER_CRITICAL_SAFE(&autoLightSleepWakeMux);
    autoLightSleepLastWakeCause = cause;
    if (buttonWake)
        autoLightSleepButtonWakePending = true;
    portEXIT_CRITICAL_SAFE(&autoLightSleepWakeMux);
}

static bool autoLightSleepLoraWakePin(gpio_num_t &pin)
{
#if SOC_PM_SUPPORT_EXT1_WAKEUP && SOC_RTCIO_PIN_COUNT > 0
#if defined(LORA_DIO1) && (LORA_DIO1 != RADIOLIB_NC) && !defined(LORA_DIO1_EXTENDED_IO)
    if (radioType != RF95_RADIO) {
        pin = (gpio_num_t)LORA_DIO1;
        return esp_sleep_is_valid_wakeup_gpio(pin);
    }
#endif
#if defined(RF95_IRQ) && (RF95_IRQ != RADIOLIB_NC)
    if (radioType == RF95_RADIO) {
        pin = (gpio_num_t)RF95_IRQ;
        return esp_sleep_is_valid_wakeup_gpio(pin);
    }
#endif
#else
    (void)pin;
#endif
    return false;
}

static esp_err_t autoLightSleepArmLoraWake()
{
#if SOC_PM_SUPPORT_EXT1_WAKEUP && SOC_RTCIO_PIN_COUNT > 0
    if (!autoLightSleepLoraWakePinValid)
        return ESP_ERR_NOT_SUPPORTED;

    configureLoraSleepHardware();
    esp_err_t res = esp_sleep_enable_ext1_wakeup_io(1ULL << (uint32_t)autoLightSleepLoraWakePinCached, ESP_EXT1_WAKEUP_ANY_HIGH);
    if (res == ESP_OK)
        autoLightSleepLoraWakeConfigured.store(true, std::memory_order_release);
    return res;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

// Restore the cached LoRa pin from RTC to digital mode in the PM exit callback.
static void IRAM_ATTR autoLightSleepRestoreLoraWakePin()
{
#if SOC_PM_SUPPORT_EXT1_WAKEUP && SOC_RTCIO_PIN_COUNT > 0
    if (autoLightSleepLoraWakeConfigured.load(std::memory_order_acquire) && autoLightSleepLoraWakePinValid) {
#if SOC_RTCIO_HOLD_SUPPORTED
        rtcio_hal_hold_disable(rtc_io_number_get(autoLightSleepLoraWakePinCached));
#endif
        rtcio_hal_function_select(rtc_io_number_get(autoLightSleepLoraWakePinCached), RTCIO_LL_FUNC_DIGITAL);
    }
#endif
}

// IRAM: registered as esp_pm's light-sleep exit callback, invoked from vApplicationSleep() inside
// portENTER_CRITICAL(&s_switch_lock) with interrupts disabled, on every light-sleep exit.
static esp_err_t IRAM_ATTR autoLightSleepExit(int64_t sleepTimeUsec, void *unused)
{
    (void)unused;

    if (sleepTimeUsec <= 0)
        return ESP_OK; // PM invokes exit callbacks even when an entry was vetoed: nothing was remuxed

    autoLightSleepRestoreLoraWakePin();

    // PM invokes this after esp_light_sleep_start() restores the flash cache.
    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    recordAutoLightSleepWake(cause, cause == ESP_SLEEP_WAKEUP_GPIO && autoLightSleepInputLevelActive());

    // Defer power servicing because power and runASAP are unsafe in this callback.
    if (cause == ESP_SLEEP_WAKEUP_GPIO || cause == ESP_SLEEP_WAKEUP_EXT1)
        autoLightSleepPowerServicePending.store(true, std::memory_order_release);

    return ESP_OK;
}

bool consumeAutoLightSleepButtonWake()
{
    portENTER_CRITICAL_SAFE(&autoLightSleepWakeMux);
    const bool pending = autoLightSleepButtonWakePending;
    autoLightSleepButtonWakePending = false;
    portEXIT_CRITICAL_SAFE(&autoLightSleepWakeMux);
    return pending;
}

bool consumeAutoLightSleepPowerServiceWake()
{
    return autoLightSleepPowerServicePending.exchange(false, std::memory_order_acq_rel);
}

bool startAutoLightSleep()
{
    if (!lightSleepLock || !pmLightSleepLock || !dynamicLightSleepReady ||
        autoLightSleepFailureCount >= MAX_AUTO_LIGHT_SLEEP_FAILURES)
        return false;
    lightSleepLock->lock();
    if (pmLightSleepLockHeld) {
        clearAutoLightSleepButtonWake();
        autoLightSleepPowerServicePending.store(false, std::memory_order_release);
        // First opt-in after init or a prior stopAutoLightSleep(): arm wake sources for
        // the PM auto path (not torn down by us afterward - esp_pm owns sleeping).
        notifyLightSleep.notifyObservers(NULL);
        enableButtonInterrupt();
        auto res = esp_sleep_enable_gpio_wakeup();
        if (res != ESP_OK) {
            LOG_ERROR("esp_sleep_enable_gpio_wakeup result %d", res);
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            disableWakeInterrupts(false);
            notifyLightSleepEnd.notifyObservers(ESP_SLEEP_WAKEUP_UNDEFINED); // no sleep happened
            autoLightSleepFailureCount++;
            lightSleepLock->unlock();
            return false;
        }
        res = autoLightSleepArmLoraWake();
        if (res != ESP_OK) {
            LOG_ERROR("LoRa EXT1 wake setup result %d", res);
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            autoLightSleepLoraWakeConfigured.store(false, std::memory_order_release);
            disableWakeInterrupts(false);
            notifyLightSleepEnd.notifyObservers(ESP_SLEEP_WAKEUP_UNDEFINED); // no sleep happened
            autoLightSleepFailureCount++;
            lightSleepLock->unlock();
            return false;
        }
        res = esp_pm_lock_release(pmLightSleepLock);
        if (res != ESP_OK) {
            LOG_ERROR("esp_pm_lock_release(meshtastic) result %d", res);
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            autoLightSleepRestoreLoraWakePin();
            autoLightSleepLoraWakeConfigured.store(false, std::memory_order_release);
            disableWakeInterrupts(false);
            notifyLightSleepEnd.notifyObservers(ESP_SLEEP_WAKEUP_UNDEFINED); // no sleep happened
            autoLightSleepFailureCount++;
            lightSleepLock->unlock();
            return false;
        }
        pmLightSleepLockHeld = false;
        autoLightSleepFailureCount = 0; // bounded-retry counter resets on success
        LOG_INFO("PM dynamic light sleep enabled");
    }
    lightSleepLock->unlock();
    return true;
}

bool stopAutoLightSleep()
{
    if (!lightSleepLock || !pmLightSleepLock)
        return false;
    lightSleepLock->lock();
    if (!pmLightSleepLockHeld) {
        // Block PM entry before changing the wake configuration.
        auto res = esp_pm_lock_acquire(pmLightSleepLock);
        if (res != ESP_OK) {
            LOG_ERROR("esp_pm_lock_acquire(meshtastic) result %d", res);
            lightSleepLock->unlock();
            return false;
        }
        pmLightSleepLockHeld = true;
        res = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        if (res != ESP_OK)
            LOG_ERROR("esp_sleep_disable_wakeup_source(ALL) result %d", res);
        autoLightSleepRestoreLoraWakePin();
        autoLightSleepLoraWakeConfigured.store(false, std::memory_order_release);
        disableWakeInterrupts(false);

        portENTER_CRITICAL_SAFE(&autoLightSleepWakeMux);
        const esp_sleep_wakeup_cause_t lastWakeCause = autoLightSleepLastWakeCause;
        autoLightSleepLastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
        portEXIT_CRITICAL_SAFE(&autoLightSleepWakeMux);
        notifyLightSleepEnd.notifyObservers(lastWakeCause); // cause of the sleep that actually woke us, if tracked

        clearAutoLightSleepButtonWake();
        autoLightSleepPowerServicePending.store(false, std::memory_order_release);
        LOG_INFO("PM dynamic light sleep disabled");
    }
    lightSleepLock->unlock();
    return true;
}
#else
bool startAutoLightSleep()
{
    return false;
}
bool stopAutoLightSleep()
{
    return true;
}
bool consumeAutoLightSleepButtonWake()
{
    return false;
}
bool consumeAutoLightSleepPowerServiceWake()
{
    return false;
}
#endif

void initLightSleep()
{
    // Every ESP32 build needs the serialization lock: the legacy explicit-sleep
    // loop in PowerFSM stateLS calls doLightSleep() even without PM support.
    if (!lightSleepLock)
        lightSleepLock = new concurrency::Lock();

#if HAS_ESP32_PM_SUPPORT
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
    dynamicLightSleepReady = false;
    autoLightSleepFailureCount = 0;
    autoLightSleepLastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
    autoLightSleepPowerServicePending.store(false, std::memory_order_release);
    autoLightSleepLoraWakeConfigured.store(false, std::memory_order_release);
    clearAutoLightSleepButtonWake();
#endif
    // Prepare PM only for roles/settings that can request PowerFSM light sleep. Shared with
    // PowerFSM_setup() so the two can't disagree about when stateLS applies.
    const bool autoSleepCandidate = isAutoLightSleepEligible();

    if (autoSleepCandidate) {
        esp_err_t lockResult = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "meshtastic", &pmLightSleepLock);
        if (lockResult != ESP_OK) {
            LOG_ERROR("esp_pm_lock_create(meshtastic) result %d", lockResult);
        } else {
            lockResult = esp_pm_lock_acquire(pmLightSleepLock);
            if (lockResult != ESP_OK) {
                LOG_ERROR("esp_pm_lock_acquire(meshtastic) result %d", lockResult);
                esp_pm_lock_delete(pmLightSleepLock);
                pmLightSleepLock = nullptr;
            } else {
                pmLightSleepLockHeld = true;
            }
        }
    }

    // IDF's USB Serial JTAG connection monitor holds its own NO_LIGHT_SLEEP lock
    // ("usb_serial_jtag") under CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION.

    static esp_pm_config_t esp32_config; // filled with zeros because bss
#if CONFIG_IDF_TARGET_ESP32S3
    esp32_config.max_freq_mhz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ;
#elif CONFIG_IDF_TARGET_ESP32S2
    esp32_config.max_freq_mhz = CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ;
#elif CONFIG_IDF_TARGET_ESP32C3
    esp32_config.max_freq_mhz = CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ;
#elif CONFIG_IDF_TARGET_ESP32C6
    esp32_config.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
#elif CONFIG_IDF_TARGET_ESP32P4
#if CONFIG_ESP32P4_REV_MIN_FULL < 300
    esp32_config.max_freq_mhz = 360;
#else
    esp32_config.max_freq_mhz = 400;
#endif
#else
    esp32_config.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
#endif
    esp32_config.min_freq_mhz = 20; // 10MHz is minimum recommended
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
    esp32_config.light_sleep_enable = autoSleepCandidate && pmLightSleepLockHeld;
#else
    // esp_pm_configure() rejects light_sleep_enable without tickless idle, and fails wholesale
    esp32_config.light_sleep_enable = false;
#endif
    // Configure on every PM build so the ESP_PM_CPU_FREQ_MAX lock in setCPUFast()
    // has a DFS range to work against - without this the CPU would stay at max.
    int rv = esp_pm_configure(&esp32_config);
    LOG_INFO("PM config: min=%d max=%d light_sleep=%d (rv=%x)", esp32_config.min_freq_mhz, esp32_config.max_freq_mhz,
             esp32_config.light_sleep_enable, rv);
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
    dynamicLightSleepReady = (rv == ESP_OK) && esp32_config.light_sleep_enable;
    // Resolve the LoRa wake pin once here and cache it, so the exit callback does not repeat the
    // esp_sleep_is_valid_wakeup_gpio() lookup inside a critical section on every wake.
    gpio_num_t loraWakePin;
    autoLightSleepLoraWakePinValid = autoLightSleepLoraWakePin(loraWakePin);
    autoLightSleepLoraWakePinCached = autoLightSleepLoraWakePinValid ? loraWakePin : GPIO_NUM_NC;
    if (dynamicLightSleepReady && !autoLightSleepLoraWakePinValid) {
        LOG_ERROR("PM dynamic light sleep requires an RTC-capable LoRa wake pin");
        dynamicLightSleepReady = false;
    }
    if (dynamicLightSleepReady) {
        static esp_pm_sleep_cbs_register_config_t sleepCallbacks = {};
        sleepCallbacks.exit_cb = autoLightSleepExit;
        esp_err_t callbackResult = esp_pm_light_sleep_register_cbs(&sleepCallbacks);
        if (callbackResult != ESP_OK) {
            LOG_ERROR("esp_pm_light_sleep_register_cbs result %d", callbackResult);
            dynamicLightSleepReady = false;
        }
    }
#endif

#endif
}

bool isDynamicLightSleepReady()
{
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
    return dynamicLightSleepReady;
#else
    return false;
#endif
}

bool isAutoLightSleepAvailable()
{
#if HAS_ESP32_DYNAMIC_LIGHT_SLEEP
    return dynamicLightSleepReady && autoLightSleepFailureCount < MAX_AUTO_LIGHT_SLEEP_FAILURES;
#else
    return false;
#endif
}

bool shouldLoraWake(uint32_t msecToWake)
{
    return msecToWake < portMAX_DELAY && (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
                                          config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
}

#if defined(INPUTDRIVER_TWO_WAY_ROCKER_BTN) || defined(INPUTDRIVER_ENCODER_BTN)
/// The wake-capable input-driver button pin, independent of textual macro ordering
static gpio_num_t inputDriverWakeBtnPin()
{
#if defined(INPUTDRIVER_TWO_WAY_ROCKER_BTN)
    return (gpio_num_t)INPUTDRIVER_TWO_WAY_ROCKER_BTN;
#else
    return (gpio_num_t)INPUTDRIVER_ENCODER_BTN;
#endif
}
#endif

/// Arm button (and other user input) GPIO wake sources. The pull-up matters: a
/// BUTTON_NEED_PULLUP board floats the pin otherwise, blocking sleep or spinning it.
void enableButtonInterrupt()
{
#if defined(BUTTON_PIN)
    gpio_num_t pin = (gpio_num_t)(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN);
#if defined(BUTTON_NEED_PULLUP)
    gpio_pullup_en(pin);
#endif
    gpio_wakeup_enable(pin, BUTTON_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
#endif
#if defined(ALT_BUTTON_PIN)
    gpio_wakeup_enable((gpio_num_t)ALT_BUTTON_PIN, ALT_BUTTON_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
#endif
#if defined(CANCEL_BUTTON_PIN)
    gpio_wakeup_enable((gpio_num_t)CANCEL_BUTTON_PIN, CANCEL_BUTTON_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
#endif
#if defined(DOWN_BUTTON_PIN)
    gpio_wakeup_enable((gpio_num_t)DOWN_BUTTON_PIN, DOWN_BUTTON_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
#endif
#if defined(ROTARY_PRESS)
    gpio_wakeup_enable((gpio_num_t)ROTARY_PRESS, GPIO_INTR_LOW_LEVEL);
#endif
#if defined(KB_INT)
#if KB_INT_WAKE_ON_HIGH
    gpio_wakeup_enable((gpio_num_t)KB_INT, GPIO_INTR_HIGH_LEVEL);
#else
    gpio_wakeup_enable((gpio_num_t)KB_INT, GPIO_INTR_LOW_LEVEL);
#endif
#endif
#if defined(BOARD_PCA9535_INT)
    // Side-key interrupt line from PCA9535 expander (active low).
    gpio_wakeup_enable((gpio_num_t)BOARD_PCA9535_INT, GPIO_INTR_LOW_LEVEL);
#endif
#if defined(INPUTDRIVER_TWO_WAY_ROCKER_BTN) || defined(INPUTDRIVER_ENCODER_BTN)
    gpio_wakeup_enable(inputDriverWakeBtnPin(), GPIO_INTR_LOW_LEVEL);
#endif
#if defined(WAKE_ON_TOUCH)
    gpio_wakeup_enable((gpio_num_t)SCREEN_TOUCH_INT, GPIO_INTR_LOW_LEVEL);
#endif
#ifdef MOTION_WAKE_INT_PIN
    if (config.display.wake_on_tap_or_motion)
        gpio_wakeup_enable((gpio_num_t)MOTION_WAKE_INT_PIN,
                           MOTION_WAKE_INT_ACTIVE_HIGH ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);
#endif
#ifdef PMU_IRQ
    // wake due to PMU can happen repeatedly if there is no battery installed or the battery fills
    if (pmu_found)
        gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL); // pmu irq
#endif
}

// Restore the radio interrupt type only if GPIO wake changed it to level-triggered.
static void disableWakeInterrupts(bool gpioWakeArmed)
{
#if defined(BUTTON_PIN)
    gpio_num_t buttonPin = (gpio_num_t)(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN);
    gpio_wakeup_disable(buttonPin);
#endif
#if defined(ALT_BUTTON_PIN)
    gpio_wakeup_disable((gpio_num_t)ALT_BUTTON_PIN);
#endif
#if defined(CANCEL_BUTTON_PIN)
    gpio_wakeup_disable((gpio_num_t)CANCEL_BUTTON_PIN);
#endif
#if defined(DOWN_BUTTON_PIN)
    gpio_wakeup_disable((gpio_num_t)DOWN_BUTTON_PIN);
#endif
#if defined(ROTARY_PRESS)
    gpio_wakeup_disable((gpio_num_t)ROTARY_PRESS);
#endif
#if defined(KB_INT)
    gpio_wakeup_disable((gpio_num_t)KB_INT);
#endif
#if defined(BOARD_PCA9535_INT)
    gpio_wakeup_disable((gpio_num_t)BOARD_PCA9535_INT);
#endif
#if defined(INPUTDRIVER_TWO_WAY_ROCKER_BTN) || defined(INPUTDRIVER_ENCODER_BTN)
    gpio_wakeup_disable(inputDriverWakeBtnPin());
#endif
#if defined(WAKE_ON_TOUCH)
    gpio_wakeup_disable((gpio_num_t)SCREEN_TOUCH_INT);
#endif
#ifdef MOTION_WAKE_INT_PIN
    gpio_wakeup_disable((gpio_num_t)MOTION_WAKE_INT_PIN);
#endif
#if defined(PMU_IRQ)
    if (pmu_found)
        gpio_wakeup_disable((gpio_num_t)PMU_IRQ);
#endif
#if defined(LORA_DIO1) && (LORA_DIO1 != RADIOLIB_NC) && !defined(LORA_DIO1_EXTENDED_IO)
    if (radioType != RF95_RADIO) {
        gpio_wakeup_disable((gpio_num_t)LORA_DIO1);
        if (gpioWakeArmed)
            gpio_set_intr_type((gpio_num_t)LORA_DIO1, GPIO_INTR_POSEDGE);
#if SOC_PM_SUPPORT_EXT_WAKEUP
        // Undo the pull-down enableLoraInterrupt() armed; the radio drives DIO1
        // while awake and a latched pull-down can hold the last level.
        gpio_pulldown_dis((gpio_num_t)LORA_DIO1);
#endif
    }
#endif
#if defined(RF95_IRQ) && (RF95_IRQ != RADIOLIB_NC)
    if (radioType == RF95_RADIO) {
        gpio_wakeup_disable((gpio_num_t)RF95_IRQ);
        if (gpioWakeArmed)
            gpio_set_intr_type((gpio_num_t)RF95_IRQ, GPIO_INTR_POSEDGE);
    }
#endif
#if HAS_LORA_FEM
    loraFEMInterface.releaseSleepHolds();
#endif
}

static void configureLoraSleepHardware()
{
#if !defined(LORA_DIO1_EXTENDED_IO) && SOC_PM_SUPPORT_EXT_WAKEUP && defined(LORA_DIO1) && (LORA_DIO1 != RADIOLIB_NC)
    esp_err_t res = gpio_pulldown_en((gpio_num_t)LORA_DIO1);
    if (res != ESP_OK) {
        LOG_ERROR("gpio_pulldown_en(LORA_DIO1) result %d", res);
    }
#if defined(LORA_RESET) && (LORA_RESET != RADIOLIB_NC)
    res = gpio_pullup_en((gpio_num_t)LORA_RESET);
    if (res != ESP_OK) {
        LOG_ERROR("gpio_pullup_en(LORA_RESET) result %d", res);
    }
#endif
#if defined(LORA_CS) && (LORA_CS != RADIOLIB_NC) && !defined(ELECROW_PANEL)
    gpio_pullup_en((gpio_num_t)LORA_CS);
#endif

#if HAS_LORA_FEM
    loraFEMInterface.setRxModeEnableWhenMCUSleep();
#endif
#endif
}

void enableLoraInterrupt()
{
#if defined(LORA_DIO1_EXTENDED_IO)
    // DIO1 is a virtual pin on an I/O expander - it cannot be a GPIO wakeup source
#elif SOC_PM_SUPPORT_EXT_WAKEUP && defined(LORA_DIO1) && (LORA_DIO1 != RADIOLIB_NC)
    configureLoraSleepHardware();

    LOG_INFO("Wake on LORA_DIO1 (GPIO%02d) gpio interrupt", LORA_DIO1);
    gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);

#elif defined(LORA_DIO1) && (LORA_DIO1 != RADIOLIB_NC)
    if (radioType != RF95_RADIO) {
        LOG_INFO("Wake on LORA_DIO1 (GPIO%02d) gpio interrupt", LORA_DIO1);
        gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL); // SX126x/SX128x interrupt, active high
    }
#endif
#if defined(RF95_IRQ) && (RF95_IRQ != RADIOLIB_NC)
    if (radioType == RF95_RADIO) {
        LOG_INFO("Wake on RF95_IRQ (GPIO%02d) gpio interrupt", RF95_IRQ);
        gpio_wakeup_enable((gpio_num_t)RF95_IRQ, GPIO_INTR_HIGH_LEVEL); // RF95 interrupt, active high
    }
#endif
}
#endif

#ifndef ARCH_ESP32
bool startAutoLightSleep()
{
    return false;
}

bool stopAutoLightSleep()
{
    return true;
}

bool consumeAutoLightSleepButtonWake()
{
    return false;
}

bool consumeAutoLightSleepPowerServiceWake()
{
    return false;
}

bool isDynamicLightSleepReady()
{
    return false;
}

bool isAutoLightSleepAvailable()
{
    return false;
}
#endif
