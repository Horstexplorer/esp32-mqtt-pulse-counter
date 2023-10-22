#include <Arduino.h>
#include <sstream>
#include <Preferences.h>
#include <WiFi.h>
#include "PubSubClient.h"

#define DEFAULT_SERIAL_BAUD 115200

#define IO_SENSOR_PIN 23
#define IO_SENSOR_DEBOUNCE 50
#define IO_RESET_PREFERENCES_PIN 22

class GPIO_DEBOUNCED {
public:
    const uint8_t pin;
    uint32_t steady_state;
    ulong last_debounce_time;
    uint16_t debounce_duration;
    std::function<void(GPIO_DEBOUNCED)> callable;
};

void IRAM_ATTR ioInterruptDigitalReadISR(void* args) {
    auto gpio_debounced = (GPIO_DEBOUNCED*) args;
    auto current_state = digitalRead(gpio_debounced->pin);
    if ((millis() - gpio_debounced->last_debounce_time) > gpio_debounced->debounce_duration) {
        gpio_debounced->last_debounce_time = millis();
        gpio_debounced->steady_state = current_state;
        if (gpio_debounced->callable != nullptr){
            gpio_debounced->callable(*gpio_debounced);
        }
    }
}

GPIO_DEBOUNCED* ioInterruptDigitalRead(uint8_t pin, int mode, uint16_t debounce_duration = 0, uint32_t initial_state = LOW, std::function<void(GPIO_DEBOUNCED)> callable = nullptr) {
    Serial.println(("<IO> Attached interrupt to digital read of pin " + std::to_string(pin)).c_str());

    auto* gpio_debounced = new GPIO_DEBOUNCED{pin, initial_state, 0, debounce_duration, callable};

    attachInterruptArg(pin, ioInterruptDigitalReadISR, gpio_debounced, mode);

    return gpio_debounced;
}

void ioInterruptDetach(const GPIO_DEBOUNCED& gpioDebounced) {
    detachInterrupt(gpioDebounced.pin);
}


#define PREFERENCES_STORAGE_SPACE "gm-readouts"
#define PREFERENCES_TOTAL_PULSES_COUNT_KEY "total-pulses"
#define PREFERENCES_TOTAL_PULSES_COUNT_TOPIC "gas/total/pulses"
#define PREFERENCES_TOTAL_PULSES_COUNT_OVERRIDE 0
#define PREFERENCES_TOTAL_VOLUME_KEY "total-volume"
#define PREFERENCES_TOTAL_VOLUME_TOPIC "gas/total/volume"
#define PREFERENCES_TOTAL_VOLUME_OVERRIDE 0.0
#define PREFERENCES_VOLUME_PER_IMPULSE 0.01

Preferences preferences;

void initialize_preference_l64(char* key, int64_t initial, int64_t test = -1) {
    if (preferences.getLong64(key, test) == test)
        preferences.putLong64(key, initial);
}

void initialize_preference_d(char* key, double_t initial, double_t test = -1.0) {
    if (preferences.getDouble(key, test) == test)
        preferences.putDouble(key, initial);
}

void increment_preference_l64(char* key, int64_t increment) {
    preferences.putLong64(key, preferences.getLong64(key) + increment);
}

void increment_preference_d(char* key, double increment) {
    preferences.putDouble(key, preferences.getDouble(key) + increment);
}

void preference_setup(char* storage_space, bool read_only = false) {
    Serial.println("<Preferences> Setup");
    preferences.begin(storage_space, read_only);
    Serial.println("<Preferences> Enabled");
}

void reset_preferences() {
    preferences.clear();

    Serial.println("<Preferences> Reset. Restarting...");

    delay(2500);
    ESP.restart();
}

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

void wifi_client_setup_dhcp(char* wifi_ssid, char* wifi_password, bool auto_reconnect = true) {
    Serial.println("<WiFi> Setup");

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(auto_reconnect);

    Serial.println("<WiFi> Connecting...");

    WiFi.begin(wifi_ssid, wifi_password);

    auto start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) <= 30000) {
        delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("<WiFi> Connection established!");
        Serial.println(WiFi.localIP());
        Serial.println(WiFi.localIPv6());
        Serial.println("<WiFi> Enabled");
    } else {
        Serial.println("<WiFi> Connection failed!");
        Serial.println(WiFi.status());
        Serial.println("<WiFi> Failed. Restarting...");

        delay(2500);
        ESP.restart();
    }
}

#define PUBSUB_SERVER_ADDRESS IPAddress(0, 0, 0, 0)
#define PUBSUB_SERVER_PORT 1883
#define PUBSUB_USERNAME "gasmeter"
#define PUBSUB_PASSWORD ""
#define PUBSUB_CLIENT_ID "gasmeter"

WiFiClient wifi_client;
PubSubClient pub_sub_client(wifi_client);

void mqtt_setup(IPAddress server_ip, int server_port, char* username, char* password, char* client_id) {
    Serial.println("<MQTT> Setup");

    Serial.println("<MQTT> Connecting...");
    pub_sub_client.setServer(server_ip, server_port);
    boolean connected = pub_sub_client.connect(client_id, username, password);

    if (connected) {
        Serial.println("<MQTT> Connection established!");
    } else {
        Serial.println("<MQTT> Connection failed!");
        Serial.println(pub_sub_client.state());
        Serial.println("<MQTT> Failed. Restarting...");

        delay(2500);
        ESP.restart();
    }
}

bool mqtt_publish(char* topic, char* content) {
    if (!pub_sub_client.connected()) {
        Serial.println("<MQTT> Not Connected. Restarting...");

        delay(2500);
        ESP.restart();
    }

    return pub_sub_client.publish(topic, content);
}

//############################

GPIO_DEBOUNCED* sensor_gpio;

void setup() {
    delay(2500);
    // setup serial out
    Serial.begin(DEFAULT_SERIAL_BAUD);
    while (!Serial)
        delay(100);
    Serial.println("<Serial> Enabled");
    // setup io
    Serial.println("<IO> Setup");
    pinMode(IO_SENSOR_PIN, INPUT_PULLDOWN);
    pinMode(IO_RESET_PREFERENCES_PIN, INPUT_PULLDOWN);
    sensor_gpio = ioInterruptDigitalRead(IO_SENSOR_PIN, CHANGE, IO_SENSOR_DEBOUNCE, LOW, nullptr);
    Serial.println("<IO> Enabled");
    // setup preferences
    preference_setup((char*) PREFERENCES_STORAGE_SPACE);
    if (digitalRead(IO_RESET_PREFERENCES_PIN) == HIGH) {
        reset_preferences();
    }
    initialize_preference_l64((char*) PREFERENCES_TOTAL_PULSES_COUNT_KEY, PREFERENCES_TOTAL_PULSES_COUNT_OVERRIDE);
    initialize_preference_d((char*) PREFERENCES_TOTAL_VOLUME_KEY, PREFERENCES_TOTAL_VOLUME_OVERRIDE);
    // setup wifi
    wifi_client_setup_dhcp((char*) WIFI_SSID, (char*) WIFI_PASSWORD);
    // setup mqtt
    mqtt_setup(PUBSUB_SERVER_ADDRESS, PUBSUB_SERVER_PORT, (char*) PUBSUB_USERNAME, (char*) PUBSUB_PASSWORD, (char*) PUBSUB_CLIENT_ID);
}

uint previous_state;
ulong last_state_switch;

uint p;
void loop() {
    if (sensor_gpio->steady_state == HIGH && previous_state != HIGH) {
        // closed
        Serial.println("<> New impulse counted");
        // update local storage
        increment_preference_l64((char*) PREFERENCES_TOTAL_PULSES_COUNT_KEY, 1);
        increment_preference_d((char*) PREFERENCES_TOTAL_VOLUME_KEY, PREFERENCES_VOLUME_PER_IMPULSE);
        // send pub sub message
        mqtt_publish((char*) PREFERENCES_TOTAL_PULSES_COUNT_TOPIC,(char*) std::to_string(preferences.getLong64(PREFERENCES_TOTAL_PULSES_COUNT_KEY)).c_str());
        mqtt_publish((char*) PREFERENCES_TOTAL_VOLUME_TOPIC,(char*) std::to_string(preferences.getDouble(PREFERENCES_TOTAL_VOLUME_KEY)).c_str());
        // output
        Serial.println(("<> Values: " + std::to_string(preferences.getLong64(PREFERENCES_TOTAL_PULSES_COUNT_KEY)) + " pulses, "
                        + std::to_string(preferences.getDouble(PREFERENCES_TOTAL_VOLUME_KEY)) + " m3 volume").c_str());
    } else if (sensor_gpio->steady_state == LOW && previous_state != LOW) {
        Serial.println("<> Impulse reset");
    }
    previous_state = sensor_gpio->steady_state;
    // mqtt client
    pub_sub_client.loop();
}