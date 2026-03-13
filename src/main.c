#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "quirc.h"

static const char *TAG = "QR_SCAN";

/* ─── Camera pins (AI-Thinker ESP32-CAM) ──────────────────── */
#define CAM_PWDN   32
#define CAM_RESET  (-1)
#define CAM_XCLK   0
#define CAM_SIOD   26
#define CAM_SIOC   27
#define CAM_D7     35
#define CAM_D6     34
#define CAM_D5     39
#define CAM_D4     36
#define CAM_D3     21
#define CAM_D2     19
#define CAM_D1     18
#define CAM_D0     5
#define CAM_VSYNC  25
#define CAM_HREF   23
#define CAM_PCLK   22

#define FRAME_W    640
#define FRAME_H    480

/* ─── QR result ───────────────────────────────────────────── */
static SemaphoreHandle_t s_qr_lock;
static char  s_qr_text[1024] = "";
static bool  s_qr_found = false;
static int   s_qr_corners[8] = {0};
static int64_t s_qr_found_time = 0;   /* hold result for 2s */
#define QR_HOLD_MS 2000

/* ─── WiFi AP ─────────────────────────────────────────────── */
static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    wifi_config_t ap = {
        .ap = {
            .ssid = "ESP32-CAM",
            .ssid_len = 9,
            .password = "12345678",
            .channel = 1,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP started: SSID=\"ESP32-CAM\" PASS=\"12345678\"");
    ESP_LOGI(TAG, "Connect and open http://192.168.4.1");
}

/* ─── Camera init ─────────────────────────────────────────── */
static esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "--- Camera init start ---");
    ESP_LOGI(TAG, "  PWDN=%d RESET=%d XCLK=%d", CAM_PWDN, CAM_RESET, CAM_XCLK);
    ESP_LOGI(TAG, "  SIOD=%d SIOC=%d", CAM_SIOD, CAM_SIOC);
    ESP_LOGI(TAG, "  D7=%d D6=%d D5=%d D4=%d D3=%d D2=%d D1=%d D0=%d",
             CAM_D7, CAM_D6, CAM_D5, CAM_D4, CAM_D3, CAM_D2, CAM_D1, CAM_D0);
    ESP_LOGI(TAG, "  VSYNC=%d HREF=%d PCLK=%d", CAM_VSYNC, CAM_HREF, CAM_PCLK);
    ESP_LOGI(TAG, "  xclk=20MHz format=JPEG size=QVGA(320x240) quality=12 fb=2");
    ESP_LOGI(TAG, "  Free heap: %lu  Free PSRAM: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    camera_config_t cfg = {
        .pin_pwdn     = CAM_PWDN,   .pin_reset   = CAM_RESET,
        .pin_xclk     = CAM_XCLK,
        .pin_sccb_sda = CAM_SIOD,   .pin_sccb_scl = CAM_SIOC,
        .pin_d7 = CAM_D7, .pin_d6 = CAM_D6, .pin_d5 = CAM_D5, .pin_d4 = CAM_D4,
        .pin_d3 = CAM_D3, .pin_d2 = CAM_D2, .pin_d1 = CAM_D1, .pin_d0 = CAM_D0,
        .pin_vsync    = CAM_VSYNC,
        .pin_href     = CAM_HREF,
        .pin_pclk     = CAM_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA,     /* 640x480 — better for QR */
        .jpeg_quality = 6,                 /* lower = better quality */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST, /* always freshest frame */
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: 0x%x (%s)", err, esp_err_to_name(err));
    } else {
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "Camera sensor PID=0x%02X VER=0x%02X",
                     s->id.PID, s->id.VER);
            /* Improve image quality for QR detection */
            s->set_quality(s, 6);
            s->set_contrast(s, 1);       /* +1 contrast */
            s->set_sharpness(s, 1);      /* +1 sharpness */
            s->set_denoise(s, 1);        /* denoise on */
        }
        ESP_LOGI(TAG, "Camera init OK");
    }
    ESP_LOGI(TAG, "  Free heap after: %lu  Free PSRAM after: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return err;
}

/* ─── QR scan task ────────────────────────────────────────── */
static void qr_task(void *arg)
{
    ESP_LOGI(TAG, "QR task started on core %d", xPortGetCoreID());
    vTaskDelay(pdMS_TO_TICKS(1000));

    struct quirc *qr = quirc_new();
    if (!qr || quirc_resize(qr, FRAME_W, FRAME_H) < 0) {
        ESP_LOGE(TAG, "quirc init FAILED");
        vTaskDelete(NULL);
        return;
    }

    uint8_t *rgb  = heap_caps_malloc(FRAME_W * FRAME_H * 3, MALLOC_CAP_SPIRAM);
    struct quirc_code *code = heap_caps_malloc(sizeof(struct quirc_code), MALLOC_CAP_SPIRAM);
    struct quirc_data *data = heap_caps_malloc(sizeof(struct quirc_data), MALLOC_CAP_SPIRAM);
    if (!rgb || !code || !data) {
        ESP_LOGE(TAG, "PSRAM alloc FAILED");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "QR scanner ready (%dx%d)", FRAME_W, FRAME_H);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(300));

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;

        bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
        esp_camera_fb_return(fb);
        if (!ok) continue;

        int w, h;
        uint8_t *qbuf = quirc_begin(qr, &w, &h);
        for (int i = 0; i < w * h; i++) {
            int ri = i * 3;
            qbuf[i] = (uint8_t)((rgb[ri]*77 + rgb[ri+1]*150 + rgb[ri+2]*29) >> 8);
        }
        quirc_end(qr);

        bool found = false;
        int n = quirc_count(qr);
        for (int i = 0; i < n; i++) {
            quirc_extract(qr, i, code);
            if (quirc_decode(code, data) == QUIRC_SUCCESS) {
                xSemaphoreTake(s_qr_lock, portMAX_DELAY);
                int plen = data->payload_len;
                if (plen > (int)sizeof(s_qr_text) - 1) plen = sizeof(s_qr_text) - 1;
                memcpy(s_qr_text, data->payload, plen);
                s_qr_text[plen] = '\0';
                s_qr_found = true;
                s_qr_found_time = esp_timer_get_time() / 1000; /* ms */
                for (int j = 0; j < 4; j++) {
                    s_qr_corners[j*2]   = code->corners[j].x;
                    s_qr_corners[j*2+1] = code->corners[j].y;
                }
                xSemaphoreGive(s_qr_lock);
                found = true;
                ESP_LOGI(TAG, ">>> QR DECODED: %.128s", s_qr_text);
                ESP_LOGI(TAG, "    corners: (%d,%d) (%d,%d) (%d,%d) (%d,%d)",
                    s_qr_corners[0],s_qr_corners[1],s_qr_corners[2],s_qr_corners[3],
                    s_qr_corners[4],s_qr_corners[5],s_qr_corners[6],s_qr_corners[7]);
                break;
            }
        }
        if (!found) {
            /* Only clear after QR_HOLD_MS so JS can pick it up */
            int64_t now = esp_timer_get_time() / 1000;
            if (s_qr_found && (now - s_qr_found_time > QR_HOLD_MS)) {
                xSemaphoreTake(s_qr_lock, portMAX_DELAY);
                s_qr_found = false;
                xSemaphoreGive(s_qr_lock);
            }
        }
    }
}

/* ─── HTTP: index ─────────────────────────────────────────── */
static const char INDEX_HTML[] =
"<!DOCTYPE html><html><head>"
"<meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>ESP32 QR Scanner</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{background:#0f0f1a;color:#e0e0e0;font-family:'Courier New',monospace;"
"display:flex;flex-direction:column;align-items:center;padding:20px}"
"h1{color:#e94560;margin-bottom:16px;font-size:1.5em}"
".wrap{position:relative;display:inline-block}"
"#cam{border:2px solid #e94560;border-radius:8px;width:640px;height:480px;"
"object-fit:contain;background:#000;display:block}"
"#ov{position:absolute;top:2px;left:2px;pointer-events:none}"
"#qr{background:#16213e;padding:14px 20px;margin-top:14px;border-radius:8px;"
"width:640px;min-height:48px;font-size:1.1em;word-break:break-all}"
".found{color:#0f0}.no{color:#555}"
"</style></head><body>"
"<h1>ESP32-CAM QR Scanner</h1>"
"<div class=\"wrap\">"
"<img id=\"cam\"/>"
"<canvas id=\"ov\" width=\"636\" height=\"476\"></canvas>"
"</div>"
"<div id=\"qr\"><span class=\"no\">scanning...</span></div>"
"<script>\n"
"document.getElementById('cam').src='/stream';\n"
"const cv=document.getElementById('ov');\n"
"const ctx=cv.getContext('2d');\n"
"const sx=636/640,sy=476/480;\n"
"setInterval(()=>{\n"
"fetch('/qr').then(r=>r.json()).then(d=>{\n"
"  ctx.clearRect(0,0,636,476);\n"
"  const el=document.getElementById('qr');\n"
"  if(d.found){\n"
"    el.innerHTML='<span class=\"found\">'+d.data.replace(/&/g,'&amp;')"
".replace(/</g,'&lt;').replace(/>/g,'&gt;')+'</span>';\n"
"    if(d.c){\n"
"      ctx.strokeStyle='#0f0';ctx.lineWidth=3;ctx.beginPath();\n"
"      ctx.moveTo(d.c[0]*sx,d.c[1]*sy);\n"
"      ctx.lineTo(d.c[2]*sx,d.c[3]*sy);\n"
"      ctx.lineTo(d.c[4]*sx,d.c[5]*sy);\n"
"      ctx.lineTo(d.c[6]*sx,d.c[7]*sy);\n"
"      ctx.closePath();ctx.stroke();\n"
"    }\n"
"  }else{\n"
"    el.innerHTML='<span class=\"no\">scanning...</span>';\n"
"  }\n"
"}).catch(()=>{});\n"
"},500);\n"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

/* ─── HTTP: QR JSON ───────────────────────────────────────── */
static esp_err_t qr_handler(httpd_req_t *req)
{
    char buf[1300];
    xSemaphoreTake(s_qr_lock, portMAX_DELAY);
    if (s_qr_found) {
        char esc[1024]; int j = 0;
        for (int i = 0; s_qr_text[i] && j < (int)sizeof(esc)-6; i++) {
            char c = s_qr_text[i];
            if (c=='"'||c=='\\') esc[j++]='\\';
            else if (c=='\n'){esc[j++]='\\';c='n';}
            else if (c=='\r'){esc[j++]='\\';c='r';}
            esc[j++]=c;
        }
        esc[j]='\0';
        snprintf(buf, sizeof(buf),
            "{\"found\":true,\"data\":\"%s\","
            "\"c\":[%d,%d,%d,%d,%d,%d,%d,%d]}", esc,
            s_qr_corners[0],s_qr_corners[1],s_qr_corners[2],s_qr_corners[3],
            s_qr_corners[4],s_qr_corners[5],s_qr_corners[6],s_qr_corners[7]);
    } else {
        strcpy(buf, "{\"found\":false,\"data\":\"\"}");
    }
    xSemaphoreGive(s_qr_lock);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ─── HTTP: MJPEG stream ──────────────────────────────────── */
#define BOUNDARY "frame"

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res;
    char hdr[80];

    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    if (res != ESP_OK) return res;

    ESP_LOGI(TAG, "Stream client connected");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "stream: fb_get fail");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int hlen = snprintf(hdr, sizeof(hdr),
            "--" BOUNDARY "\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n", (unsigned)fb->len);

        res = httpd_resp_send_chunk(req, hdr, hlen);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, "\r\n", 2);

        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return res;
}

/* ─── Web server ──────────────────────────────────────────── */
static void start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 4;
    cfg.stack_size = 16384;
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    httpd_uri_t u1 = {.uri="/",      .method=HTTP_GET, .handler=index_handler};
    httpd_uri_t u2 = {.uri="/qr",    .method=HTTP_GET, .handler=qr_handler};
    httpd_uri_t u3 = {.uri="/stream", .method=HTTP_GET, .handler=stream_handler};
    httpd_register_uri_handler(srv, &u1);
    httpd_register_uri_handler(srv, &u2);
    httpd_register_uri_handler(srv, &u3);
    ESP_LOGI(TAG, "HTTP server on port 80");
}


/* ─── app_main ────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== ESP32-CAM QR Scanner (diag mode) ===");
    ESP_LOGI(TAG, "========================================");

    /* Suppress noisy components — only show warnings and errors */
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);  /* our logs: init + QR decoded */

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_qr_lock = xSemaphoreCreateMutex();

    /* Camera FIRST */
    ESP_LOGI(TAG, ">>> Attempting camera init...");
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "!!! CAMERA FAILED !!!");
        ESP_LOGE(TAG, "Starting WiFi AP anyway for diagnostics...");

        wifi_init_ap();
        ESP_LOGE(TAG, "Halting — camera not available");
        while (1) vTaskDelay(pdMS_TO_TICKS(10000));
    }

    /* WiFi AP */
    wifi_init_ap();

    /* Web server */
    start_webserver();

    /* QR task (8KB stack — heavy allocs are on PSRAM heap) */
    ESP_LOGI(TAG, "Free internal heap: %lu", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    BaseType_t rc = xTaskCreatePinnedToCore(qr_task, "qr", 16 * 1024, NULL, 3, NULL, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "!!! QR task create FAILED (rc=%d) !!!", rc);
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== All systems GO ===");
    ESP_LOGI(TAG, "========================================");
}
