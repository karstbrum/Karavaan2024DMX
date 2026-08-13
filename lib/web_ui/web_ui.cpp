#include "web_ui.h"
#include "web_page.h"
#include <cstdarg>
#include <cstring>
#include <cstdio>

// Accumulates small pieces into a fixed buffer and flushes as one TCP
// write, instead of one sendContent() call per pixel (positions/colors
// responses iterate over every pixel, and /colors is polled repeatedly).
namespace
{
    class ChunkWriter
    {
    public:
        explicit ChunkWriter(WebServer &server_) : server(server_), len(0) {}

        void add(const char *text)
        {
            size_t textLen = strlen(text);
            if (len + textLen >= sizeof(buf))
            {
                flush();
            }
            memcpy(buf + len, text, textLen);
            len += textLen;
        }

        void addf(const char *fmt, ...)
        {
            char tmp[32];
            va_list args;
            va_start(args, fmt);
            vsnprintf(tmp, sizeof(tmp), fmt, args);
            va_end(args);
            add(tmp);
        }

        void flush()
        {
            if (len > 0)
            {
                server.sendContent(buf, len);
                len = 0;
            }
        }

        // lets callers stop iterating once the client has gone away (e.g.
        // the browser refreshed and reset the connection mid-response),
        // instead of wasting time formatting/writing the remaining pixels
        bool connected()
        {
            return server.client().connected();
        }

    private:
        WebServer &server;
        char buf[1400];
        size_t len;
    };
}

WebUI::WebUI(Pixels &led_, const char *apSSID_, const char *apPassword_)
    : led(led_), server(80), apSSID(apSSID_), apPassword(apPassword_)
{
}

void WebUI::begin()
{
    WiFi.softAP(apSSID, apPassword);

    server.on("/", HTTP_GET, [this]()
              { handleRoot(); });
    server.on("/positions", HTTP_GET, [this]()
              { handlePositions(); });
    server.on("/colors", HTTP_GET, [this]()
              { handleColors(); });
    server.on("/set", HTTP_POST, [this]()
              { handleSet(); });

    server.begin();
}

void WebUI::handle()
{
    server.handleClient();
}

void WebUI::onSet(SetCallback cb)
{
    setCallback = cb;
}

void WebUI::handleRoot()
{
    server.send_P(200, "text/html", WEB_UI_PAGE);
}

void WebUI::handlePositions()
{
    uint16_t n = led.getTotalPixels();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

    ChunkWriter out(server);
    out.addf("{\"n\":%u,\"x\":[", n);
    for (uint16_t i = 0; i < n && out.connected(); i++)
    {
        out.addf(i ? ",%.4f" : "%.4f", led.getPixelX(i));
    }
    out.add("],\"y\":[");
    for (uint16_t i = 0; i < n && out.connected(); i++)
    {
        out.addf(i ? ",%.4f" : "%.4f", led.getPixelY(i));
    }
    out.add("]}");
    if (out.connected())
    {
        out.flush();
    }
}

void WebUI::handleColors()
{
    uint16_t n = led.getTotalPixels();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

    ChunkWriter out(server);
    out.add("[");
    for (uint16_t i = 0; i < n && out.connected(); i++)
    {
        uint8_t w, r, g, b;
        led.getPixelColor(i, w, r, g, b);
        out.addf(i ? ",[%u,%u,%u,%u]" : "[%u,%u,%u,%u]", w, r, g, b);
    }
    out.add("]");
    if (out.connected())
    {
        out.flush();
    }
}

void WebUI::handleSet()
{
    if (!setCallback)
    {
        server.send(400, "text/plain", "no handler registered");
        return;
    }

    auto argU8 = [this](const char *name, uint8_t def) -> uint8_t
    {
        return server.hasArg(name) ? static_cast<uint8_t>(server.arg(name).toInt()) : def;
    };

    setCallback(
        argU8("mode", 0), argU8("bpm", 1), argU8("dim", 255), argU8("dimmer", 0),
        argU8("red", 255), argU8("green", 255), argU8("blue", 255),
        argU8("extra1", 0), argU8("extra2", 0));

    server.send(200, "text/plain", "ok");
}
