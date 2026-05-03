#include <WiFi.h>
#include <WebServer.h>
#include <U8g2lib.h>

// === OLED setup (SH1106 on I2C) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// === WiFi credentials ===
const char* ssid = "CARDAK";
const char* password = "OSMAN1450";

// === Web server on port 80 ===
WebServer server(80);

// Message buffer
String message = "Hello!";

void handleRoot() {
  String html = "<html><body>"
                "<h2>Send a message to ESP32</h2>"
                "<form action='/send' method='POST'>"
                "Message: <input type='text' name='msg'>"
                "<input type='submit' value='Send'>"
                "</form>"
                "<p>Current message: " + message + "</p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleSend() {
  if (server.hasArg("msg")) {
    message = server.arg("msg");
    Serial.println("New message: " + message);

    // Update OLED
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 20, message.c_str());
    u8g2.sendBuffer();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/send", HTTP_POST, handleSend);
  server.begin();
}

void loop() {
  server.handleClient();
}
