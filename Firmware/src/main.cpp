#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NeoPixelBus.h>
#include <WiFi.h>

#include "secrets.h"

const char* serverURL = "http://170.64.133.152:3000/ledmap100.json";

// pins and pixel counts defined in the board file (./boards/Auckland_Rail_Network_V1_0_0.json)

NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> strandMNK(STRAND_MNK_PIXELS, STRAND_MNK);
NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> nalNIMT(NAL_NIMT_PIXELS, NAL_NIMT);

RgbColor black(0, 0, 0);
RgbColor red(255, 0, 0);
RgbColor green(0, 255, 0);
RgbColor blue(0, 0, 255);

enum charlieplexedLedState { GREEN, RED, OFF };

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

const char* getSystemInfo() {
	static char buffer[512];
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
		"\nBoard: %s\n"
		"Chip: %s-Rev%d\n"
		"%d Core @ %dMHz\n"
		"%dMiB Flash @ %dMHz in %s Mode\n"
		"RAM Heap: %dkiB\n"
		"SDK: %s\n",
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

	String payload = String();

	http.begin(serverURL);
	int httpCode = http.GET();

	if (httpCode == HTTP_CODE_OK) {
		payload = http.getString();
	} else {
		Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
	}

	http.end();
	return payload;
}

uint32_t hexStringToRgb(const char* hexStr) {
	if (hexStr == nullptr)
		return 0;
	if (hexStr[0] == '#')
		hexStr++;  // Skip #
	return strtoul(hexStr, NULL, 16);
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
	//Serial.printf("Configuration Version: %s\n", version);

	// Parse line colors
	JsonObject colors = doc["lineColors"];
	//Serial.println("\nLine Colors:");
	for (JsonPair color : colors) {
		uint32_t rgb = hexStringToRgb(color.value().as<const char*>());
		//Serial.printf("  Color %s: 0x%06X\n", color.key().c_str(), rgb);
	}

	// Parse busses
	JsonArray busses = doc["busses"];
	//Serial.println("\nBus Configurations:");
	for (JsonObject bus : busses) {
		String busId = bus["busId"];
		//Serial.printf("Bus ID: %s\n", busId);

		JsonObject leds = bus["leds"];
		for (JsonPair led : leds) {
			int ledIndex = atoi(led.key().c_str());
			int colorId = led.value().as<int>();

			// Get color from line_colors
			char colorKey[4];
			snprintf(colorKey, sizeof(colorKey), "%d", colorId);
			const char* hexColor = colors[colorKey];
			uint32_t rgb = hexStringToRgb(hexColor);

			//Serial.printf("  LED %02d -> Color ID %d (0x%06X)\n", ledIndex, colorId, rgb);

			if (busId == "STRAND_MNK") {
				strandMNK.SetPixelColor(ledIndex, HtmlColor(rgb));
			} else if (busId == "NAL_NIMT") {
				nalNIMT.SetPixelColor(ledIndex, HtmlColor(rgb));
			} else {
				Serial.printf("Unknown bus ID: %s\n", busId);
			}
		}
	}
}

void setup() {
	setCharlieplexedLED(WIFI_LED_PIN, GREEN);
	setCharlieplexedLED(CONFIG_LED_PIN, OFF);

	// Hardware Serial
	Serial0.begin(115200);
	Serial0.setDebugOutput(true);

	// USB Serial
	Serial.begin();

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

	vTaskDelay(pdMS_TO_TICKS(5000));

	Serial.println(getSystemInfo());

	// --- WiFi Setup ---
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	Serial.print("Connecting to WiFi ");
	uint16_t wifi_retries = 0;
	while (WiFi.status() != WL_CONNECTED && wifi_retries < 300) {  // ~30 seconds timeout
		Serial.print("-");
		if (wifi_retries % 2 == 0) {
			setCharlieplexedLED(WIFI_LED_PIN, GREEN);
		} else {
			setCharlieplexedLED(WIFI_LED_PIN, OFF);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
		wifi_retries++;
	}

	if (WiFi.status() == WL_CONNECTED) {
		Serial.printf("> Connected! IP Address:%s in %is\n", WiFi.localIP().toString(), millis() / 1000);
		setCharlieplexedLED(WIFI_LED_PIN, GREEN);
	} else {
		Serial.println("X Connection Failed!");
		setCharlieplexedLED(WIFI_LED_PIN, RED);
	}
}

void loop() {
	if (WiFi.status() == WL_CONNECTED) {
		strandMNK.ClearTo(black);
		nalNIMT.ClearTo(black);

		String jsonInput = downloadJSON();
		// Serial.println(jsonInput);

		parseLEDMap(jsonInput);

		strandMNK.Show();
		nalNIMT.Show();

		Serial.printf("MCU:%2.0f°C	WiFi:%idBm\n", temperatureRead(), WiFi.RSSI());

		vTaskDelay(pdMS_TO_TICKS(6000));

	} else {
		strandMNK.ClearTo(red);
		strandMNK.Show();
		nalNIMT.ClearTo(red);
		nalNIMT.Show();
		pinMode(WIFI_LED_PIN, OUTPUT);
		digitalWrite(WIFI_LED_PIN, LOW);
		pinMode(CONFIG_LED_PIN, OUTPUT);
		digitalWrite(CONFIG_LED_PIN, LOW);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		strandMNK.ClearTo(green);
		strandMNK.Show();
		nalNIMT.ClearTo(green);
		nalNIMT.Show();
		digitalWrite(WIFI_LED_PIN, HIGH);
		digitalWrite(CONFIG_LED_PIN, HIGH);
		vTaskDelay(pdMS_TO_TICKS(1000));

		strandMNK.ClearTo(blue);
		strandMNK.Show();
		nalNIMT.ClearTo(blue);
		nalNIMT.Show();
		pinMode(WIFI_LED_PIN, INPUT);
		pinMode(CONFIG_LED_PIN, INPUT);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
