#ifndef WEB_UI_H
#define WEB_UI_H

#include <WiFi.h>
#include <WebServer.h>
#include "led_functions.h"

class WebUI
{
public:
    // invoked from the /set handler with the raw 0-255 control values
    typedef void (*SetCallback)(uint8_t mode, uint8_t bpm, uint8_t dim, uint8_t dimmer,
                                 uint8_t red, uint8_t green, uint8_t blue,
                                 uint8_t extra1, uint8_t extra2);

    WebUI(Pixels &led_, const char *apSSID_, const char *apPassword_);

    // starts the AP + HTTP routes, call once from a dedicated task
    void begin();

    // pumps the HTTP server, call repeatedly from a task loop
    void handle();

    void onSet(SetCallback cb);

private:
    Pixels &led;
    WebServer server;
    const char *apSSID;
    const char *apPassword;
    SetCallback setCallback = nullptr;

    void handleRoot();
    void handlePositions();
    void handleColors();
    void handleSet();
};

#endif
