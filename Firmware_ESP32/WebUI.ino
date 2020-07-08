AsyncWebServer server =  AsyncWebServer(80);
WebSocketsServer webSocket = WebSocketsServer(1337);

const char *ssid = "LED";
const char *password =  "";
const char *msg_toggle_led = "toggleLED";
const char *msg_get_led = "getLEDState";
char msg_buf[10];
const int refresh_ms = 50;
unsigned long  wait_ms = 0UL;
String JSONtxt;

void web_ui_init()
{
  // Make sure we can read the file system
  if (!SPIFFS.begin())
  {
    Serial.println("Error mounting SPIFFS");
    while (1);
  }
  // Start access point
  WiFi.softAP(ssid, password);

  // Print our IP address
  //Serial.println();
  //Serial.println("AP running");
  Serial.print("My IP address: ");
  Serial.println(WiFi.softAPIP());

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, web_ui_onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  server.on("/style.css", HTTP_GET, web_ui_onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(web_ui_onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(web_ui_onWebSocketEvent);
}

void web_ui_eval()
{
  webSocket.loop();
  if (millis() > wait_ms)
  {
    // Build the collision points string from the matrix
    String pointstr;
    pointstr += "[";
    for (uint8_t i = 0; i < points_cnt; i++)
    {
      pointstr += "[";
      pointstr += String(points[i][0] * 1.0E3F);
      pointstr += ",";
      pointstr += String(points[i][1] * 1.0E3F);
      pointstr += ",";
      pointstr += String(points[i][2] * 1.0E3F);
      pointstr += ",";
      pointstr += String(points[i][3] * 1.0E3F);
      pointstr += ",";
      pointstr += String(points[i][4] * 1.0E3F);
      pointstr += "]";

      if (i < points_cnt - 1)
        pointstr += ",";
    }
    pointstr += "]";

    // JSON requires double quotes
    JSONtxt = "{\"x\":" + String(coord[0] * 1.0E3F) + "," +
              "\"y\":" + String(coord[1] * 1.0E3F) + "," +
              "\"z\":" + String(coord[2] * 1.0E3F) + "," +
              "\"mx\":" + String(coord[3] * 1.0E3F) + "," +
              "\"my\":" + String(coord[4] * 1.0E3F) + "," +
              "\"mz\":" + String(coord[5] * 1.0E3F) + "," +
              "\"ttx\":" + String(tooltip[0] * 1.0E3F) + "," +
              "\"tty\":" + String(tooltip[1] * 1.0E3F) + "," +
              "\"ttz\":" + String(tooltip[2] * 1.0E3F) + "," +
              "\"pts\":" + pointstr + "," +
              "\"pt1\":" + String(point_1_idx) + "," +
              "\"pt2\":" + String(point_2_idx) + "," +
              "\"d\":" + String(distance * 1.0E3F) + "," +
              "\"tc\":" + String(cycle_time_ms) + "," +
              "\"bw\":" + String(si72_param[1]) + "}";

    webSocket.broadcastTXT(JSONtxt);
    wait_ms = millis() + refresh_ms;
  }
}

// Callback: receiving any WebSocket message
void web_ui_onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length)
{

  // Figure out the type of WebSocket event
  switch (type)
  {
    // Client has disconnected
    case WStype_DISCONNECTED:
      //Serial.printf("[%u] Disconnected!\n", client_num);
      break;
    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        //Serial.printf("[%u] Connection from ", client_num);
        //Serial.println(ip.toString());
      }
      break;
    // Handle text messages from client
    case WStype_TEXT:
      // Print out raw message
      //Serial.printf("[%u] Received text: %s\n", client_num, payload);

      // Button "Teach point" was clicked
      if (strcmp((char *)payload, "teach_point") == 0)
      {
        if (points_cnt < POINTS_LEN)
        {
          points[points_cnt][0] = tooltip[0];
          points[points_cnt][1] = tooltip[1];
          points[points_cnt][2] = tooltip[2];
          if (points_cnt > 0) {
            float dist[3];
            dist[0] = points[points_cnt][0] - points[points_cnt - 1][0];
            dist[1] = points[points_cnt][1] - points[points_cnt - 1][1];
            dist[2] = points[points_cnt][2] - points[points_cnt - 1][2];
            // TODO: prevent 0.00
            points[points_cnt][4] = b_norm(dist);
          }
          points_cnt++;
        }
      }

      // Button "Delete Last" was clicked
      else if (strcmp((char *)payload, "delete_point") == 0)
      {
        if (points_cnt > 0)
          points_cnt--;
      }

      // Button "Save" was clicked
      else if (strcmp((char *)payload, "write_eeprom") == 0)
      {
        collision_points_save();
      }

      // Button "Restore" was clicked
      else if (strcmp((char *)payload, "restore_eeprom") == 0)
      {
        collision_points_restore();
      }

      // Bust width of SI72 was changed, Button "1" ... "4096"
      else if (strcmp((char *)payload, "0") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "1") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "2") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "3") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "4") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "5") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "6") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "7") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "8") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "9") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "10") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "11") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      else if (strcmp((char *)payload, "12") == 0)
      {
        si72_param[1] = atoi((const char*)payload);
        restart = true;
      }
      break;
      
    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

// Callback: send homepage
void web_ui_onIndexRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  //Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void web_ui_onCSSRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  //Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/style.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void web_ui_onPageNotFound(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  //Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}
