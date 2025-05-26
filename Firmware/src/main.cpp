#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NeoPixelBus.h>
#include <NeoPixelBusLg.h>
#include <WiFi.h>
#include <WiFiMulti.h>

#include "secrets.h"

WiFiMulti wifiMulti;

// Array of server URLs for failover
const char* serverURLs[] = {
	"http://170.64.133.152:3000/ledmap100.json",
	"http://192.168.86.31:3000/ledmap100.json",
};
const int numServers = sizeof(serverURLs) / sizeof(serverURLs[0]);
int currentServerIndex = 0;

// Pins and pixel counts defined in the board file (./boards/Auckland_Rail_Network_V1_0_0.json)

NeoPixelBusLg<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod> strandMNK(STRAND_MNK_PIXELS, STRAND_MNK);
NeoPixelBusLg<NeoGrbFeature, NeoEsp32Rmt1800KbpsMethod> nalNIMT(NAL_NIMT_PIXELS, NAL_NIMT);

RgbColor black(0, 0, 0);

enum statusLedCommand {
	LED_OFF,
	LED_ON_GREEN,
	LED_ON_RED,
	LED_BLINK_GREEN_SLOW,  // 1Hz
	LED_BLINK_GREEN_FAST,  // 5Hz
	LED_BLINK_RED_SLOW,	   // 1Hz
	LED_BLINK_RED_FAST	   // 5Hz
};

enum charlieplexedLedState { GREEN, RED, OFF };

typedef struct {
	uint8_t pin;
	statusLedCommand command;
	bool currentState;
	unsigned long lastToggle;
} statusLed;

TaskHandle_t statusLedTaskHandle;

static unsigned long lastUpdate = 0;
int16_t brightness = 20;
String jsonInput = String();
bool brightnessChanged = false;

struct Button {
	uint8_t pin;
	volatile unsigned long lastChangeTime;
	volatile bool pendingCheck;
	bool lastState;
};

// Initialize button structures
Button brightnessDownButton = { BRIGHTNESS_DOWN_BUTTON, 0, false, HIGH };
Button brightnessUpButton = { BRIGHTNESS_UP_BUTTON, 0, false, HIGH };
Button powerButton = { POWER_BUTTON, 0, false, HIGH };

// IRAM_ATTR ensures the ISR is placed in IRAM (critical for ESP32)
void IRAM_ATTR buttonISR(void* arg) {
	Button* button = (Button*)arg;
	button->lastChangeTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
	button->pendingCheck = true;
}

void checkButton(Button* button) {
	if (button->pendingCheck && (millis() - button->lastChangeTime) >= DEBOUNCE_DELAY) {
		bool currentState = digitalRead(button->pin);
		if (currentState != button->lastState) {
			button->lastState = currentState;
			if (currentState == LOW) {	// Assuming active-low configuration
				uint8_t oldBrightness = brightness;
				// Handle button press
				switch (button->pin) {
					case BRIGHTNESS_DOWN_BUTTON:
						Serial.print("Brightness Down pressed");
						brightness -= 20;
						break;
					case BRIGHTNESS_UP_BUTTON:
						Serial.print("Brightness Up pressed");
						brightness += 20;
						break;
					case POWER_BUTTON:
						Serial.print("Power button pressed");
						brightness = (brightness == 0) ? 250 : 0;  // Toggle brightness
						break;
				}
				// Ensure brightness stays within bounds
				brightness = (brightness > 0) ? constrain(brightness, 20, 250) : 0;
				strandMNK.SetLuminance(brightness);
				nalNIMT.SetLuminance(brightness);
				Serial.printf(" brightness: %i/255\n", brightness);

				brightnessChanged = (oldBrightness != brightness);
			}
		}
		button->pendingCheck = false;
	}
}

void setCharlieplexedLED(uint8_t pin, charlieplexedLedState state) {
	switch (state) {
		case GREEN:
			pinMode(pin, OUTPUT);
			digitalWrite(pin, HIGH);
			break;

		case RED:
			pinMode(pin, OUTPUT);
			digitalWrite(pin, LOW);
			break;

		case OFF:
			// Set as input (High Resistance) to disable output driver
			pinMode(pin, INPUT);
			break;
	}
}

// LED Task function
void statusLedManagerTask(void* pvParameters) {
	statusLed leds[] = { { WIFI_LED_PIN, LED_OFF, false, 0 }, { CONFIG_LED_PIN, LED_OFF, false, 0 } };
	const int numLeds = sizeof(leds) / sizeof(leds[0]);

	while (1) {
		// Check for notifications
		uint32_t notification;
		if (xTaskNotifyWait(0, ULONG_MAX, &notification, 0) == pdTRUE) {
			uint8_t pin = notification >> 24;
			statusLedCommand cmd = statusLedCommand((notification >> 16) & 0xFF);

			for (int i = 0; i < numLeds; i++) {
				if (leds[i].pin == pin) {
					leds[i].command = cmd;
					// Immediate response for non-blinking states
					if (cmd == LED_ON_GREEN || cmd == LED_ON_RED || cmd == LED_OFF) {
						setCharlieplexedLED(pin, (cmd == LED_ON_GREEN) ? GREEN : (cmd == LED_ON_RED) ? RED : OFF);
					}
					break;
				}
			}
		}

		// Handle blinking
		unsigned long now = millis();
		for (int i = 0; i < numLeds; i++) {
			if (leds[i].command >= LED_BLINK_GREEN_SLOW) {	// All blink commands
				// Extract blink parameters from command
				const bool isGreen = (leds[i].command == LED_BLINK_GREEN_SLOW || leds[i].command == LED_BLINK_GREEN_FAST);
				const bool isRed = (leds[i].command == LED_BLINK_RED_SLOW || leds[i].command == LED_BLINK_RED_FAST);
				const bool isSlow = (leds[i].command == LED_BLINK_GREEN_SLOW || leds[i].command == LED_BLINK_RED_SLOW);

				if (isGreen || isRed) {
					const int interval = isSlow ? 500 : 100;  // 1Hz or 5Hz
					const charlieplexedLedState color = isGreen ? GREEN : RED;

					if (now - leds[i].lastToggle >= interval) {
						leds[i].currentState = !leds[i].currentState;
						setCharlieplexedLED(leds[i].pin, leds[i].currentState ? color : OFF);
						leds[i].lastToggle = now;
					}
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(25));
	}
}

// Replace all setCharlieplexedLED() calls with notifications:
void setStatusLedState(uint8_t pin, statusLedCommand command) {
	uint32_t notification = (pin << 24) | (command << 16);
	xTaskNotify(statusLedTaskHandle, notification, eSetValueWithOverwrite);
}

const char* getSystemInfo() {
	static char buffer[255];
	FlashMode_t mode = (FlashMode_t)ESP.getFlashChipMode();
	const char* flash_mode_str;

	// Convert flash mode to human-readable string
	switch (mode) {
		case FM_QIO: flash_mode_str = "Quad I/O (QIO)"; break;
		case FM_QOUT: flash_mode_str = "Quad Output (QOUT)"; break;
		case FM_DIO: flash_mode_str = "Dual I/O (DIO)"; break;
		case FM_DOUT: flash_mode_str = "Dual Output (DOUT)"; break;
		case FM_FAST_READ: flash_mode_str = "Fast Read"; break;
		case FM_SLOW_READ: flash_mode_str = "Slow Read"; break;
		case FM_UNKNOWN:
		default: flash_mode_str = "Unknown"; break;
	}

	snprintf(
		buffer,
		sizeof(buffer),
		"\n%s\n"
		"%s-Rev%d\n"
		"%d Core @ %dMHz\n"
		"%dMiB Flash @ %dMHz in %s Mode\n"
		"RAM Heap: %dkiB\n"
		"IDF SDK: %s\n",
		ARDUINO_BOARD,
		ESP.getChipModel(),
		ESP.getChipRevision(),
		ESP.getChipCores(),
		ESP.getCpuFreqMHz(),
		ESP.getFlashChipSize() / (1024 * 1024),
		ESP.getFlashChipSpeed() / (1000 * 1000),
		flash_mode_str,
		ESP.getHeapSize() / 1024,
		ESP.getSdkVersion());

	return buffer;
}

String downloadJSON() {
	HTTPClient http;
	String payload;

	for (int i = 0; i < numServers; i++) {
		int serverIndex = (currentServerIndex + i) % numServers;
		const char* url = serverURLs[serverIndex];
		http.setTimeout(500);  // Set timeout to 500ms seconds per server
		http.begin(url);

		int httpCode = http.GET();
		if (httpCode == HTTP_CODE_OK) {
			payload = http.getString();
			http.end();
			currentServerIndex = serverIndex;  // Update to the successful server
			return payload;
		} else {
			Serial.printf("Fetch from %s returned: %i (%s)\n", url, httpCode, http.errorToString(httpCode).c_str());
			http.end();
		}
	}
	return String();
}

RgbColor hexStringToRgbColor(const char* hexStr) {
	if (!hexStr || strlen(hexStr) < 3)
		return RgbColor(0, 0, 0);

	// Skip '#' if present
	const char* start = hexStr;
	if (hexStr[0] == '#')
		start++;

	// Convert to 32-bit RGB value
	uint32_t rgb = strtoul(start, NULL, 16);

	// Handle shorthand 3-digit format
	if (strlen(start) == 3) {
		return RgbColor(((rgb >> 8) & 0xF) * 0x11,	// Expand 4-bit to 8-bit
						((rgb >> 4) & 0xF) * 0x11,
						(rgb & 0xF) * 0x11);
	}

	// Standard 6-digit format
	return RgbColor((rgb >> 16) & 0xFF,	 // Red component
					(rgb >> 8) & 0xFF,	 // Green component
					rgb & 0xFF			 // Blue component
	);
}

void parseLEDMap(const String& jsonInput) {
	JsonDocument doc;
	DeserializationError error = deserializeJson(doc, jsonInput);

	if (error) {
		Serial.printf("JSON parse error: %s\n", error.c_str());
		return;
	}

	// Parse version
	const char* version = doc["version"];

	// Parse line colors
	JsonObject colors = doc["lineColors"];

	// Parse busses
	JsonArray busses = doc["busses"];
	for (JsonObject bus : busses) {
		String busId = bus["busId"];
		JsonObject leds = bus["leds"];
		for (JsonPair led : leds) {
			int ledIndex = atoi(led.key().c_str());
			int colorId = led.value().as<int>();

			// Get color from line_colors
			char colorKey[4];
			snprintf(colorKey, sizeof(colorKey), "%d", colorId);
			const char* hexColor = colors[colorKey];
			RgbColor pixelColor = hexStringToRgbColor(hexColor);
			// pixelColor = pixelColor.Dim(uint8_t(brightness));

			if (busId == "STRAND_MNK") {
				strandMNK.SetPixelColor(ledIndex, pixelColor);
			} else if (busId == "NAL_NIMT") {
				nalNIMT.SetPixelColor(ledIndex, pixelColor);
			} else {
				Serial.printf("Unknown bus ID: %s\n", busId);
			}
		}
	}
}

void setup() {
	xTaskCreate(statusLedManagerTask, "Status LED Manager", 1024, NULL, 1, &statusLedTaskHandle);

	// Hardware Serial
	Serial0.begin(115200);
	Serial0.setDebugOutput(true);

	// USB Serial
	Serial.begin();
	Serial.setDebugOutput(true);

	pinMode(LVL_Shifter_EN, OUTPUT);
	digitalWrite(LVL_Shifter_EN, HIGH);	 //Disable LVL Shifter
	pinMode(LED_5V_EN, OUTPUT);
	digitalWrite(LED_5V_EN, LOW);  //Disable 5V Power

	strandMNK.Begin();
	strandMNK.ClearTo(black);
	nalNIMT.Begin();
	nalNIMT.ClearTo(black);

	digitalWrite(LVL_Shifter_EN, LOW);	//Enable LVL Shifter
	digitalWrite(LED_5V_EN, HIGH);		//Enable 5V Power

	nalNIMT.Show();
	strandMNK.Show();

	// Set initial brightness
	strandMNK.SetLuminance(brightness);
	nalNIMT.SetLuminance(brightness);

	// Button initialization
	pinMode(brightnessDownButton.pin, INPUT_PULLUP);
	pinMode(brightnessUpButton.pin, INPUT_PULLUP);
	pinMode(powerButton.pin, INPUT_PULLUP);

	// Attach interrupts with debouncing
	attachInterruptArg(digitalPinToInterrupt(BRIGHTNESS_DOWN_BUTTON), buttonISR, &brightnessDownButton, CHANGE);
	attachInterruptArg(digitalPinToInterrupt(BRIGHTNESS_UP_BUTTON), buttonISR, &brightnessUpButton, CHANGE);
	attachInterruptArg(digitalPinToInterrupt(POWER_BUTTON), buttonISR, &powerButton, CHANGE);

	Serial.println(getSystemInfo());

	// --- WiFi Setup ---
	setStatusLedState(WIFI_LED_PIN, LED_BLINK_GREEN_FAST);
	WiFi.mode(WIFI_STA);
	const int numWiFiNetworks = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);
	for (int i = 0; i < numWiFiNetworks; i++) {
		wifiMulti.addAP(wifiNetworks[i].ssid, wifiNetworks[i].password);
		Serial.printf("Added WiFi Network %d: %s\n", i, wifiNetworks[i].ssid);
	}

	Serial.print("Connecting to WiFi ");
	uint16_t wifi_retries = 0;
	// Use WiFiMulti's run() to handle connection attempts
	while (wifiMulti.run(5 * 1000) != WL_CONNECTED && wifi_retries < 6) {  // ~30 second timeout
		wifi_retries++;
		Serial.print("-");
	}

	if (WiFi.status() == WL_CONNECTED) {
		Serial.printf("> Connected! IP Address:%s in %ims\n", WiFi.localIP().toString(), millis());
	} else {
		Serial.println("X Connection Failed!");
		setStatusLedState(WIFI_LED_PIN, LED_OFF);
		setStatusLedState(CONFIG_LED_PIN, LED_BLINK_RED_SLOW);
	}

	Serial.println(getSystemInfo());
}

void loop() {
	if (WiFi.status() == WL_CONNECTED) {

		if (millis() - lastUpdate >= 6000) {
			lastUpdate = millis();

			jsonInput = downloadJSON();
			if (jsonInput.length() > 0) {
				setStatusLedState(WIFI_LED_PIN, LED_ON_GREEN);

				strandMNK.ClearTo(black);
				nalNIMT.ClearTo(black);
				parseLEDMap(jsonInput);
				strandMNK.Show();
				nalNIMT.Show();

			} else {
				Serial.println("All servers failed");
				setStatusLedState(WIFI_LED_PIN, LED_BLINK_RED_SLOW);
			}

			Serial.printf("MCU:%2.1f°C  WiFi:%idBm\n", temperatureRead(), WiFi.RSSI());
		}

		// Check button states
		checkButton(&brightnessDownButton);
		checkButton(&brightnessUpButton);
		checkButton(&powerButton);

		if (brightnessChanged) {
			brightnessChanged = false;
			parseLEDMap(jsonInput);
			strandMNK.Show();
			nalNIMT.Show();
		}

	} else {
		wifiMulti.run(6000);  // Attempt to reconnect if WiFi is disconnected
		Serial.println("WiFi disconnected, attempting to reconnect...");
		setStatusLedState(WIFI_LED_PIN, LED_ON_RED);
	}
	vTaskDelay(pdMS_TO_TICKS(100));
}
