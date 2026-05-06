
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h> 

// Wifi
#define WIFI_SSID       "Hoang Quan_2.4G"
#define WIFI_PASSWORD   "hoangquan123"

// Pin 
#define PIN_SENSOR_IN   4    // Sensor ngõ vào 
#define PIN_SENSOR_OUT  5    // Sensor ngõ ra  
#define PIN_SERVO       6    // Servo motor 

// LCD I2C
#define LCD_I2C_ADDR    0x27
#define LCD_COLS        16
#define LCD_ROWS        2
#define LCD_MSG_HOLD_MS 3000   // Thời gian hiển thị thông báo xe ra/vào (ms) 

// Parking information
#define MAX_SPACES      5           // Tổng số chỗ đỗ
#define BARRIER_OPEN_MS 3000        // Thời gian barrier mở (ms)
#define SERVO_OPEN_DEG  0           // Góc servo mở
#define SERVO_CLOSE_DEG 180         // Góc servo đóng

// Task
#define STACK_WEB       8192
#define STACK_SENSOR    2048
#define STACK_ENTRY     3072
#define STACK_EXIT      3072
#define STACK_NTP       3072
#define STACK_LCD       2048   

#define PRIO_SENSOR     4 
#define PRIO_ENTRY      3
#define PRIO_EXIT       3
#define PRIO_WEB        2
#define PRIO_NTP        1    
#define PRIO_LCD        2    

// Event Types (gửi qua Queue) 
typedef enum {
    EVT_VEHICLE_ENTRY = 0,
    EVT_VEHICLE_EXIT  = 1,
} ParkingEvent_t;

// LCD Message (gửi qua xLcdQueue) 
typedef enum {
    LCD_EVT_ENTRY = 0,   // Xe vào
    LCD_EVT_EXIT  = 1,   // Xe ra
    LCD_EVT_FULL  = 2,   // Bãi đầy
    LCD_EVT_IDLE  = 3,   // Trở về idle
} LcdEventType_t;

struct LcdMessage_t {
    LcdEventType_t type;
    int  vehicleCount;     // Số xe hiện tại sau sự kiện
    int  availableSpaces;  // Tổng số chỗ
    char timeStr[32];      // Thời điểm sự kiện
};

// Shared Data (bảo vệ bởi mutex)
struct SharedData {
    int  vehicleCount;
    int  availableSpaces;
    char currentStatus[64];
    char currentTime[32];
    char plateHistory[20][32];   
    int  historyCount;
};

// Global RTOS Objects 
static QueueHandle_t     xEventQueue;
static QueueHandle_t     xLcdQueue;         // Queue gửi thông báo → Task LCD
static SemaphoreHandle_t xServoMutex;       // bảo vệ servo
static SemaphoreHandle_t xDataMutex;        // bảo vệ SharedData
static SemaphoreHandle_t xSerialMutex;      // bảo vệ Serial print
static SemaphoreHandle_t xLcdMutex;         // bảo vệ LCD (I2C bus)
static SemaphoreHandle_t xEntrySemaphore;   // ISR → EntryTask
static SemaphoreHandle_t xExitSemaphore;    // ISR → ExitTask
static TimerHandle_t     xBarrierTimer;     // sw timer đóng barrier

// Global Objects 
static SharedData   g_data;
static Servo        g_servo;
static WebServer    g_server(80);
static WiFiUDP      g_ntpUDP;
static NTPClient    g_timeClient(g_ntpUDP, "pool.ntp.org", 25200); 
static LiquidCrystal_I2C g_lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);  

// Task Handles (để có thể suspend/resume nếu cần) 
static TaskHandle_t hTaskEntry  = NULL;
static TaskHandle_t hTaskExit   = NULL;
static TaskHandle_t hTaskWeb    = NULL;
static TaskHandle_t hTaskSensor = NULL;
static TaskHandle_t hTaskNTP    = NULL;
static TaskHandle_t hTaskLCD    = NULL;   

#define ISR_DEBOUNCE_MS     5000UL   // Sau 5s cảm biến sensor mới kích hoạt lại - tránh 1 xe đếm thành 2 xe 
static volatile uint32_t lastEntryTriggerMs = 0;
static volatile uint32_t lastExitTriggerMs  = 0;

/**
 * @brief ISR cho cảm biến ngõ vào (FALLING edge = xe đến)
 *        Debounce bằng millis() trước khi gửi event.
 *        Gửi event vào queue và give semaphore cho EntryTask.
 */
void IRAM_ATTR ISR_SensorIn() {
    uint32_t now = millis();
    if (now - lastEntryTriggerMs < ISR_DEBOUNCE_MS) return; 
    lastEntryTriggerMs = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ParkingEvent_t evt = EVT_VEHICLE_ENTRY;

    // Gửi vào queue (không block)
    xQueueSendFromISR(xEventQueue, &evt, &xHigherPriorityTaskWoken);
    // Give semaphore để đánh thức EntryTask
    xSemaphoreGiveFromISR(xEntrySemaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief ISR cho cảm biến ngõ ra (FALLING edge = xe ra)
 *        Debounce bằng millis() trước khi gửi event.
 */
void IRAM_ATTR ISR_SensorOut() {
    uint32_t now = millis();
    if (now - lastExitTriggerMs < ISR_DEBOUNCE_MS) return; 
    lastExitTriggerMs = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ParkingEvent_t evt = EVT_VEHICLE_EXIT;

    xQueueSendFromISR(xEventQueue, &evt, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(xExitSemaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


// HELPER — Thread-safe Serial print
static void safeLog(const char* msg) {
    if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println(msg);
        xSemaphoreGive(xSerialMutex);
    }
}

// HELPER — Thread-safe status update
static void setStatus(const char* status) {
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        strncpy(g_data.currentStatus, status, sizeof(g_data.currentStatus) - 1);
        xSemaphoreGive(xDataMutex);
    }
}

// HELPER — Thêm bản ghi lịch sử (thread-safe)
static void addHistory(const char* label, const char* time) {
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (g_data.historyCount < 20) {
            snprintf(g_data.plateHistory[g_data.historyCount],
                     32, "%s @ %s", label, time);
            g_data.historyCount++;
        }
        xSemaphoreGive(xDataMutex);
    }
}

// SOFTWARE TIMER CALLBACK — Đóng barrier sau BARRIER_OPEN_MS
static void vBarrierTimerCB(TimerHandle_t xTimer) {
    // Lấy mutex servo để đóng barrier
    if (xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        g_servo.write(SERVO_CLOSE_DEG);
        xSemaphoreGive(xServoMutex);
    }
    setStatus("Idle");
    safeLog("[TIMER] Barrier closed by timer");
}

// HELPER — Mở barrier + khởi động timer tự đóng
static void openBarrier(const char* reason) {
    // Lấy mutex trước khi điều khiển servo
    if (xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        g_servo.write(SERVO_OPEN_DEG);
        xSemaphoreGive(xServoMutex);
    }

    setStatus(reason);
    safeLog(reason);

    // (Re)start software timer — barrier sẽ tự đóng sau BARRIER_OPEN_MS
    xTimerStop(xBarrierTimer, 0);
    xTimerChangePeriod(xBarrierTimer, pdMS_TO_TICKS(BARRIER_OPEN_MS), 0);
    xTimerStart(xBarrierTimer, 0);
}

//  TASK: Entry Control
//  - Chờ semaphore từ ISR (block hoàn toàn khi không có xe)
//  - Kiểm tra slot, cập nhật dữ liệu, mở barrier
static void vTaskEntryCtrl(void* pvParam) {
    safeLog("[ENTRY] Task started");

    for (;;) {
        // Block vô thời hạn cho đến khi ISR give semaphore
        if (xSemaphoreTake(xEntrySemaphore, portMAX_DELAY) == pdTRUE) {

            // Debounce đã được xử lý trong ISR bằng millis().
            vTaskDelay(pdMS_TO_TICKS(200));

            // Lấy mutex để đọc/ghi shared data
            if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                int spaces = g_data.availableSpaces - g_data.vehicleCount;

                if (spaces <= 0) {
                    // Bãi đầy
                    strncpy(g_data.currentStatus, "Lot Full - Entry Denied",
                            sizeof(g_data.currentStatus) - 1);
                    xSemaphoreGive(xDataMutex);
                    safeLog("[ENTRY] Parking lot full");
                    sendLcdMessage(LCD_EVT_FULL);
                    continue;
                }

                // Cho xe vào
                g_data.vehicleCount++;
                char timeSnap[32];
                strncpy(timeSnap, g_data.currentTime, sizeof(timeSnap) - 1);
                xSemaphoreGive(xDataMutex);

                // Ghi lịch sử
                addHistory("ENTER", timeSnap);

                // Gửi thông báo tới LCD
                sendLcdMessage(LCD_EVT_ENTRY);

                // Mở barrier (timer tự đóng)
                openBarrier("[ENTRY] Barrier opening");

                safeLog("[ENTRY] Vehicle entered");
            }
        }
    }
    vTaskDelete(NULL);
}

//  TASK: Exit Control
//  - Chờ semaphore từ ISR
//  - Cập nhật dữ liệu, mở barrier ngõ ra
static void vTaskExitCtrl(void* pvParam) {
    safeLog("[EXIT] Task started");

    for (;;) {
        if (xSemaphoreTake(xExitSemaphore, portMAX_DELAY) == pdTRUE) {

            // Debounce đã được xử lý trong ISR bằng millis().
            vTaskDelay(pdMS_TO_TICKS(200));

            if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                if (g_data.vehicleCount <= 0) {
                    xSemaphoreGive(xDataMutex);
                    safeLog("[EXIT] No vehicle to exit");
                    continue;
                }

                g_data.vehicleCount--;
                char timeSnap[32];
                strncpy(timeSnap, g_data.currentTime, sizeof(timeSnap) - 1);
                xSemaphoreGive(xDataMutex);

                addHistory("EXIT", timeSnap);

                // Gửi thông báo tới LCD
                sendLcdMessage(LCD_EVT_EXIT);

                openBarrier("[EXIT] Barrier opening");

                safeLog("[EXIT] Vehicle exited");
            }
        }
    }
    vTaskDelete(NULL);
}

//  TASK: Sensor Monitor 
//  Đọc queue events và log — giúp trace luồng sự kiện
static void vTaskSensorMonitor(void* pvParam) {
    safeLog("[SENSOR] Monitor task started");
    ParkingEvent_t evt;

    for (;;) {
        if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(5000)) == pdTRUE) {
            if (evt == EVT_VEHICLE_ENTRY) {
                safeLog("[SENSOR] Queue: ENTRY event");
            } else if (evt == EVT_VEHICLE_EXIT) {
                safeLog("[SENSOR] Queue: EXIT event");
            }
        }
        // Chỉ log, xử lý ở EntryCtrl/ExitCtrl
        taskYIELD();
    }
    vTaskDelete(NULL);
}

// TASK: NTP Time Sync
// Cập nhật thời gian mỗi 10 giây
static void vTaskNTP(void* pvParam) {
    safeLog("[NTP] Task started");

    for (;;) {
        g_timeClient.update();
        String t = g_timeClient.getFormattedTime();

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            strncpy(g_data.currentTime, t.c_str(),
                    sizeof(g_data.currentTime) - 1);
            xSemaphoreGive(xDataMutex);
        }

        // Chạy mỗi 10 giây (non-blocking với vTaskDelay)
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    vTaskDelete(NULL);
}

// WEB SERVER — HTML local
static String buildHTML() {
    // Snapshot dữ liệu dưới mutex
    SharedData snap;
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        memcpy(&snap, &g_data, sizeof(SharedData));
        xSemaphoreGive(xDataMutex);
    } else {
        return "<html><body>Data unavailable</body></html>";
    }

    int freeSpaces = snap.availableSpaces - snap.vehicleCount;

    String html;
    html.reserve(2048);
    html += "<!DOCTYPE html><html lang='vi'><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='2'>";
    html += "<title>Smart Parking — ESP32-C3 RTOS</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;background:#f4f4f9;margin:0;color:#333}";
    html += "header{background:#0e3d79;color:#fff;padding:15px;text-align:center}";
    html += ".container{max-width:900px;margin:0 auto;padding:20px}";
    html += ".card{background:#fff;border-radius:8px;padding:16px;margin:12px 0;"
            "box-shadow:0 2px 6px rgba(0,0,0,.12)}";
    html += ".badge{display:inline-block;padding:4px 10px;border-radius:12px;"
            "font-weight:bold;font-size:.85em}";
    html += ".green{background:#d4edda;color:#155724}";
    html += ".red{background:#f8d7da;color:#721c24}";
    html += ".yellow{background:#fff3cd;color:#856404}";
    html += "table{width:100%;border-collapse:collapse;margin-top:8px}";
    html += "th,td{padding:8px 12px;border:1px solid #ddd;text-align:left}";
    html += "th{background:#0e3d79;color:#fff}";
    html += "tr:nth-child(even){background:#f9f9f9}";
    html += "</style></head><body>";
    html += "<header><h1>Smart Parking System</h1>";
    html += "<p>ESP32-C3 · FreeRTOS · RTOS Edition</p></header>";
    html += "<div class='container'>";

    // Status card
    html += "<div class='card'>";
    html += "<h2>Trạng thái hệ thống</h2>";
    html += "<p><strong>Thời gian:</strong> ";
    html += snap.currentTime;
    html += "</p><p><strong>Trạng thái:</strong> <span class='badge yellow'>";
    html += snap.currentStatus;
    html += "</span></p></div>";

    // Parking slots card
    html += "<div class='card'><h2>Bãi đỗ xe</h2>";
    html += "<p><strong>Tổng chỗ:</strong> ";
    html += snap.availableSpaces;
    html += "</p><p><strong>Đang đỗ:</strong> ";
    html += snap.vehicleCount;
    html += "</p><p><strong>Còn trống:</strong> <span class='badge ";
    html += (freeSpaces > 0) ? "green" : "red";
    html += "'>";
    html += freeSpaces;
    html += " chỗ</span></p></div>";

    // History
    html += "<div class='card'><h2>Lịch sử vào/ra</h2>";
    if (snap.historyCount == 0) {
        html += "<p><em>Chưa có bản ghi nào.</em></p>";
    } else {
        html += "<table><tr><th>#</th><th>Sự kiện</th></tr>";
        for (int i = snap.historyCount - 1; i >= 0; i--) {
            html += "<tr><td>";
            html += (snap.historyCount - i);
            html += "</td><td>";
            html += snap.plateHistory[i];
            html += "</td></tr>";
        }
        html += "</table>";
    }
    html += "</div>";

    html += "</div></body></html>";
    return html;
}

//  TASK: Web Server
//  Chạy trong task riêng — không block các task khác
static void vTaskWebServer(void* pvParam) {
    safeLog("[WEB] Task started");

    g_server.on("/", HTTP_GET, []() {
        String page = buildHTML();
        g_server.send(200, "text/html", page);
    });

    g_server.on("/api/status", HTTP_GET, []() {
        SharedData snap;
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            memcpy(&snap, &g_data, sizeof(SharedData));
            xSemaphoreGive(xDataMutex);
        }
        String json = "{\"vehicleCount\":";
        json += snap.vehicleCount;
        json += ",\"available\":";
        json += (snap.availableSpaces - snap.vehicleCount);
        json += ",\"status\":\"";
        json += snap.currentStatus;
        json += "\",\"time\":\"";
        json += snap.currentTime;
        json += "\"}";
        g_server.send(200, "application/json", json);
    });

    g_server.begin();
    safeLog("[WEB] Server listening on port 80");

    for (;;) {
        g_server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

//  HELPER — Gửi thông báo tới Task LCD (thread-safe, non-block)
//  Gọi từ EntryCtrl, ExitCtrl
static void sendLcdMessage(LcdEventType_t type) {
    LcdMessage_t msg;
    msg.type = type;

    // Snapshot dữ liệu hiện tại - dưới mutex
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        msg.vehicleCount    = g_data.vehicleCount;
        msg.availableSpaces = g_data.availableSpaces;
        strncpy(msg.timeStr, g_data.currentTime, sizeof(msg.timeStr) - 1);
        xSemaphoreGive(xDataMutex);
    } else {
        msg.vehicleCount    = 0;
        msg.availableSpaces = MAX_SPACES;
        strncpy(msg.timeStr, "--:--:--", sizeof(msg.timeStr) - 1);
    }

    // Gửi vào queue LCD, không block — bỏ qua nếu queue đầy
    xQueueSend(xLcdQueue, &msg, 0);
}

//  TASK: LCD I2C Display
//  - Chờ thông báo từ xLcdQueue
//  - Hiển thị 2 dòng thông tin xe ra/vào
//  - Trở về màn hình Idle sau LCD_MSG_HOLD_MS
static void vTaskLCD(void* pvParam) {
    safeLog("[LCD] Task started");

    if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        g_lcd.init();
        g_lcd.backlight();
        g_lcd.clear();
        g_lcd.setCursor(2, 0);
        g_lcd.print("Smart Parking");
        g_lcd.setCursor(3, 1);
        g_lcd.print("Khoi dong...");
        xSemaphoreGive(xLcdMutex);
    }

    // Delay hiện màn hình khởi động
    vTaskDelay(pdMS_TO_TICKS(2000));

    LcdMessage_t msg;
    bool         inEvent = false;  // Hiển thị sự kiện - chưa về Idle

    for (;;) {
        // Chờ thông báo tối đa LCD_MSG_HOLD_MS, hết timeout => cập nhật Idle
        BaseType_t got = xQueueReceive(xLcdQueue, &msg,
                                       pdMS_TO_TICKS(LCD_MSG_HOLD_MS));

        if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;  // Không lấy được mutex => bỏ qua 
        }

        g_lcd.clear();

        if (got == pdTRUE) {
            // Hiển thị sự kiện 
            int freeSpaces = msg.availableSpaces - msg.vehicleCount;

            // Rút gọn thời gian: chỉ lấy HH:MM (5 ký tự)
            char hhmm[6] = "??:??";
            if (strlen(msg.timeStr) >= 5) {
                strncpy(hhmm, msg.timeStr, 5);
                hhmm[5] = '\0';
            }

            char line1[17];
            char line2[17];

            switch (msg.type) {
                case LCD_EVT_ENTRY:
                    snprintf(line1, sizeof(line1), ">> Xe vao %s", hhmm);
                    snprintf(line2, sizeof(line2), "Dang do:%2d/%d  ",
                             msg.vehicleCount, msg.availableSpaces);
                    break;

                case LCD_EVT_EXIT:
                    snprintf(line1, sizeof(line1), "<< Xe ra  %s", hhmm);
                    snprintf(line2, sizeof(line2), "Dang do:%2d/%d  ",
                             msg.vehicleCount, msg.availableSpaces);
                    break;

                case LCD_EVT_FULL:
                    snprintf(line1, sizeof(line1), "!! BAI DAY !!   ");
                    snprintf(line2, sizeof(line2), "Toi da: %d xe   ",
                             msg.availableSpaces);
                    break;

                default:
                    snprintf(line1, sizeof(line1), "  Smart Parking ");
                    snprintf(line2, sizeof(line2), "Cho:%d Con:%d   ",
                             msg.vehicleCount,
                             freeSpaces < 0 ? 0 : freeSpaces);
                    break;
            }

            g_lcd.setCursor(0, 0);
            g_lcd.print(line1);
            g_lcd.setCursor(0, 1);
            g_lcd.print(line2);
            inEvent = true;

        } else {
            // Timeout => màn hình Idle
            // Snapshot nhanh dữ liệu hiện tại
            int curVeh  = 0;
            int totSlot = MAX_SPACES;
            char timeNow[32] = "--:--:--";

            if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                curVeh  = g_data.vehicleCount;
                totSlot = g_data.availableSpaces;
                strncpy(timeNow, g_data.currentTime, sizeof(timeNow) - 1);
                xSemaphoreGive(xDataMutex);
            }

            int freeNow = totSlot - curVeh;
            if (freeNow < 0) freeNow = 0;

            // Dòng 1
            g_lcd.setCursor(0, 0);
            g_lcd.print("  Smart Parking ");

            // Dòng 2: đang đỗ / còn trống
            char line2[17];
            snprintf(line2, sizeof(line2), "Do:%d Con:%d %s",
                     curVeh, freeNow,
                     freeNow == 0 ? "DAY" : "   ");
            g_lcd.setCursor(0, 1);
            g_lcd.print(line2);

            inEvent = false;
        }

        xSemaphoreGive(xLcdMutex);
    }
    vTaskDelete(NULL);
}

// SETUP
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n Smart Parking RTOS ");

    // GPIO setup 
    pinMode(PIN_SENSOR_IN,  INPUT_PULLUP);
    pinMode(PIN_SENSOR_OUT, INPUT_PULLUP);

    // LCD setup
    Wire.begin(1, 0);   // SDA = GPIO1 ; SCL = GPIO0

    // Servo setup 
    ESP32PWM::allocateTimer(0);
    g_servo.setPeriodHertz(50);
    g_servo.attach(PIN_SERVO, 1000, 2000);
    g_servo.write(SERVO_CLOSE_DEG); 

    // Shared data init 
    memset(&g_data, 0, sizeof(SharedData));
    g_data.availableSpaces = MAX_SPACES;
    strncpy(g_data.currentStatus, "Initializing...", 63);
    strncpy(g_data.currentTime, "00:00:00", 31);

    //  WiFi 
    Serial.print("[WiFi] Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[WiFi] Connected — IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n[WiFi] FAILED — continuing offline");
    }

    // NTP init 
    g_timeClient.begin();
    g_timeClient.update();

    // Queue: 8 event, mỗi event = 1 byte enum
    xEventQueue = xQueueCreate(8, sizeof(ParkingEvent_t));
    configASSERT(xEventQueue != NULL);

    // Queue LCD: 4 tin nhắn hiển thị
    xLcdQueue = xQueueCreate(4, sizeof(LcdMessage_t));
    configASSERT(xLcdQueue != NULL);

    // Mutex
    xServoMutex  = xSemaphoreCreateMutex();
    xDataMutex   = xSemaphoreCreateMutex();
    xSerialMutex = xSemaphoreCreateMutex();
    xLcdMutex    = xSemaphoreCreateMutex();   // Bảo vệ LCD
    configASSERT(xServoMutex  != NULL);
    configASSERT(xDataMutex   != NULL);
    configASSERT(xSerialMutex != NULL);
    configASSERT(xLcdMutex    != NULL);

    // Binary semaphore: ISR => EntryTask / ExitTask
    xEntrySemaphore = xSemaphoreCreateBinary();
    xExitSemaphore  = xSemaphoreCreateBinary();
    configASSERT(xEntrySemaphore != NULL);
    configASSERT(xExitSemaphore  != NULL);

    // Software timer: tự đóng barrier
    xBarrierTimer = xTimerCreate(
        "BarrierTimer",            
        pdMS_TO_TICKS(BARRIER_OPEN_MS),
        pdFALSE,                  
        NULL,
        vBarrierTimerCB
    );
    configASSERT(xBarrierTimer != NULL);

    //  TẠO TASKS
    xTaskCreate(vTaskEntryCtrl,    "EntryCtrl",    STACK_ENTRY,  NULL, PRIO_ENTRY,  &hTaskEntry);
    xTaskCreate(vTaskExitCtrl,     "ExitCtrl",     STACK_EXIT,   NULL, PRIO_EXIT,   &hTaskExit);
    xTaskCreate(vTaskSensorMonitor,"SensorMon",    STACK_SENSOR, NULL, PRIO_SENSOR, &hTaskSensor);
    xTaskCreate(vTaskWebServer,    "WebServer",    STACK_WEB,    NULL, PRIO_WEB,    &hTaskWeb);
    xTaskCreate(vTaskNTP,          "NTPSync",      STACK_NTP,    NULL, PRIO_NTP,    &hTaskNTP);
    xTaskCreate(vTaskLCD,          "LCDDisplay",   STACK_LCD,    NULL, PRIO_LCD,    &hTaskLCD);

    // INTERRUPT 
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_IN), ISR_SensorIn, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_OUT), ISR_SensorOut, FALLING);

    setStatus("Idle");
    Serial.println("[SETUP] All tasks created — RTOS scheduler running");
    Serial.printf("[SETUP] Free heap: %u bytes\n", esp_get_free_heap_size());
    Serial.printf("[SETUP] Task count: %u\n", uxTaskGetNumberOfTasks());
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
