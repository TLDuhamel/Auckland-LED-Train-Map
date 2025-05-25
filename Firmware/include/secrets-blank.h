// Rename to secrets.h for the code to use your WiFi Network
struct wifiNetwork {
	const char* ssid;
	const char* password;
};

wifiNetwork wifiNetworks[] = {
	{ "SSID1", "password1" },
	{ "SSID2", "password2" },
	{ "SSID3", "password3" },
};