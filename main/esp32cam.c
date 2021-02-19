
/* test esp32-cam camera
ov2640

Nov 2020, Thomas Krueger, Germany (all rights reserved)
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_camera.h"
#include "../components/esp32-camera-master/driver/private_include/sccb.h"
#include "../components/esp32-camera-master/sensors/private_include/ov2640.h"
//camera pin config
//esp32-cam ai-thinker PIN Map
#define CAM_PIN_PWDN    32 //on esp32-cam, this is the control line for the 2 voltage regulators, 1.2v and 2.8v. they are active low!!
#define CAM_PIN_RESET   -1 //software reset will be performed. not connected!
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// map pins to ascii names
gpio_num_t pintab[]= {CAM_PIN_D2,CAM_PIN_D1,CAM_PIN_D3,CAM_PIN_D0,CAM_PIN_D4,CAM_PIN_PCLK,CAM_PIN_D5,CAM_PIN_D6,CAM_PIN_XCLK,CAM_PIN_D7,CAM_PIN_HREF,CAM_PIN_VSYNC,CAM_PIN_SIOC,CAM_PIN_SIOD};
char *nametab[]= {"D2","D1","D3","D0","D4","PCLK","D5","D6","XCLK","D7","HSYNC","VSYNC","SDC","SDA"};

static camera_config_t camera_config =
{
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,//640x480 standard. Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 10, //0-63 lower number means higher quality
    .fb_count = 2 //if more than one, i2s runs in continuous mode. Use only with JPEG
};


const char *TAG = "camtest";
uint8_t wifi_retrys = 0;
uint8_t wifi_status=0; // 1=started,0=stopped
uint8_t server_status=0; // 1=started,0=stopped
sensor_t sensor;

//protos:
esp_err_t event_handler(void *ctx, system_event_t *event);
void wifi_start(void);
void wifi_stop(void);
void camserver(void);
void server_start(void);
void cam_reset(void);
void probe_pclk(void);
uint8_t probe_adr(void);
uint8_t SCCB_Probe(void);
int SCCB_Init(int pin_sda, int pin_scl);
esp_err_t camera_enable_out_clock(camera_config_t* config);
int camprobe(const camera_config_t* config);
void testpins(void);


typedef enum
{
    CAMERA_NONE = 0,
    CAMERA_UNKNOWN = 1,
    CAMERA_OV7725 = 7725,
    CAMERA_OV2640 = 2640,
    CAMERA_OV3660 = 3660,
    CAMERA_OV5640 = 5640,
} camera_model_t;
esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model);

void app_main()
{
    uint8_t ch;


// init:
    ESP_ERROR_CHECK(nvs_flash_init()); //needed for wifi and PHY
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

//init basic stuff

	ESP_LOGI(TAG, "\n\n\n\n+++Camera Test Start+++....\n");
	


	
// give it a more verbose debug print !!	
	ESP_LOGI(TAG, "\n\nDoing Camera driver init....watch...\n\n");
        esp_err_t err = esp_camera_init(&camera_config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        }
		else
		 ESP_LOGI(TAG, " Cam seems OK (asto driver)");



    const char *menue="\n***Menue***\n"
                      "1-start wifi\n"
                      "2-stop wifi\n"
                      "3-start server, menue gone...needs wifi!\n"
                      "4-Reset processor\n"
                      "5-Reset Camera\n"
                      "6-cam probe\n"
                      "7-VSYNC dutyclyle\n"
                      "8-pintest\n"
                      ;
    const char *prompt="\ncamtest>";

    puts(menue);
    puts(prompt);
    while(1) // menue loop
    {
        vTaskDelay(10/portTICK_PERIOD_MS); //min 10 for watchdog not to trigger!
        //	esp_task_wdt_reset();
        ch = fgetc(stdin);
        if (ch!=0xFF)
        {
            fputc(ch, stdout); //echo
            switch(ch)
            {
            case '1':
                wifi_start();
                break;
            case '2':
//nosupport:
                wifi_stop();
                break;
            case '3':
                if (server_status==1) break;  // warning: the connected serial cables may cause very slow wifi transmission!
                camserver();
                server_status=1;
                break;
            case '4':
			esp_restart();
                break;
            case '5':
              cam_reset();
                break;
            case '6':
                probe_adr();
                break;
            case '7':
                probe_pclk();
                break;
            case '8':
                testpins();
                //probe_adr();
                break;
            default:
                puts(menue);
            }
            puts(prompt);
        }
    }
}

//return 1=OK,else 0=fail
int camprobe(const camera_config_t* config)
{
    ESP_LOGI(TAG, "CAMPROBE: Enabling XCLK output");
    camera_enable_out_clock(&camera_config);

    ESP_LOGI(TAG, "CAMPROBE: Initializing SSCB");
    SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);

    ESP_LOGI(TAG, "CAMPROBE: resetting power");
   
    gpio_set_direction(32, GPIO_MODE_OUTPUT); // set portpin to output
    gpio_set_level(32, 1); // turn power on
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(32, 0); // turn power on
    vTaskDelay(100/portTICK_PERIOD_MS);




    ESP_LOGI(TAG, "CAMPROBE: Searching for camera address");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t slv_addr = SCCB_Probe();

    ESP_LOGI(TAG, "Detected camera-address=0x%02x (0=none)", slv_addr);
    if (slv_addr == 0)
    {
        ESP_LOGI(TAG, "CAMPROBE: sccb failed!!!");
        return 0;
    }
    ESP_LOGI(TAG, "CAMPROBE: detected: 0x%02x",slv_addr);
    return 1;
}

// returns 1 if output, else 0
int IsOutput(gpio_num_t pin)
{
    int state;
    if (pin < 32)
    {
        state = GPIO_REG_READ(GPIO_ENABLE_REG) & BIT(pin);
    }
    else
    {
        pin -= 31;
        state = GPIO_REG_READ(GPIO_ENABLE1_REG) & BIT(pin);
    }
    return state;
}

// returns current value of in or out pin as 1 or 0
int GetPinVal(gpio_num_t pin)
{
    int state;

    if (IsOutput(pin))
    {
        //pin is output - read the GPIO_OUT_REG register
        if (pin < 32)
        {
            state = (GPIO_REG_READ(GPIO_OUT_REG)  >> pin) & 1U;
        }
        else
        {
            pin -= 31;
            state = (GPIO_REG_READ(GPIO_OUT1_REG)  >> pin) & 1U;
        }
    }
    else
    {
        //pin is input - read the GPIO_IN_REG register
        if (pin < 32)
        {
            state = (GPIO_REG_READ(GPIO_IN_REG)  >> pin) & 1U;
        }
        else
        {
            pin -= 31;
            state = (GPIO_REG_READ(GPIO_IN1_REG)  >> pin) & 1U;
        }
    }
    return state;
}



void testpins(void)
{
    int i,state,in,cnt,time;
    char c;
    volatile gpio_num_t pin;
	printf("Testing activity on defined camera pins.\nD0-D7 may toggle depending on picture data\nSDA,SDC,XCLK no activity is OK\nVSYNC,HSYNC,PCLK ,must toggle, else problem on that pin\n");
    printf("PinTesting displays toggle events for each pin.\n Just one value means -no activity-:\n");
    for (i=0; i<sizeof(pintab)/sizeof(gpio_num_t); i++)
    {
        pin=pintab[i];
        printf("\n\nCamPin: %d  IO-%d %s ",i+2, pin,nametab[i]);
        if (IsOutput(pin))
            puts("Output:");
        else
            puts("Input:");

        state=0xff;
        cnt=0;
		
		time=0xffff;

        while (time--)
        {
            c = fgetc(stdin);
            if (c!=0xFF) break;
            in = GetPinVal(pin);

            if (in != state)
            {
                cnt++;
                state=in;
                if (state) putchar('1');
                else putchar('0');

            }
            if (cnt > 200) break; // limit the output
            // vTaskDelay(5/portTICK_PERIOD_MS);
        }
    }

    printf("\n\n\nfinished BYE....\n");

}



// pin 25 vsync duty cycle
void probe_pclk(void)
{
    uint32_t h,l;
    int state,cnt;
    h=l=cnt=0;
    state=gpio_get_level(CAM_PIN_VSYNC);
    if (state) h++;
    else l++;
    while(1)
    {
        if (gpio_get_level(CAM_PIN_VSYNC) != state)
        {
            if (state) l++;
            else h++;
            state=!state;
        }
        cnt++; // count the changes
        if (cnt > 1000000) break;
    }
    printf("VSYNC duty cycle:Hight: %u Low: %u\n",h,l);

}

void cam_reset(void)
{
	uint8_t adr,ret=0;
	printf ("toggle cam powersupply to reset\n");
    gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT); // set portpin to output
    gpio_set_level(CAM_PIN_PWDN, 1); // turn power off
    vTaskDelay(1000/portTICK_PERIOD_MS);
    gpio_set_level(CAM_PIN_PWDN, 0); // turn power on
    vTaskDelay(1000/portTICK_PERIOD_MS);
	printf("intialize camera...\n");
	
	adr = probe_adr();
printf("writings init regs...\n");

// do minimal ov2640 init to produce output signals	
ret+=SCCB_Write(adr,0xff, 0x00);
ret+=SCCB_Write(adr,0x2c, 0xff); // start DSP DATAport D0-D7 output

ret+=SCCB_Write(adr,0x2e, 0xdf); // start DSP HSYNC,VSYNC PCLK output


	if (ret) printf("I2C write failed\n");
		

}

uint8_t probe_adr(void)
{
    uint8_t adr,ch;
    adr=SCCB_Probe();
    printf("CamAdr: 0x%02x\n",adr);
	SCCB_Write(adr,0xff,0x01); // select bank 1
	ch = SCCB_Read(adr,0x0a);
    printf("Reading Camera PID register, got: 0x%02x\n",ch);
	
	if (ch != OV2640_PID)
	{
		ESP_LOGI(TAG, "The CamId doesnt match OV2640.\n That hints to assume that there is a trouble in the I2C communication-lines");
	}
	else
	ESP_LOGI(TAG, "Found OV2640 Camera ID, and I2C is working!");	
	return adr;
}


// wifi conect to router with credentials supplied in config
void wifi_start(void)
{
    if (wifi_status) return; // already started!
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    snprintf((char*)wifi_config.sta.ssid, 32, "%s", CONFIG_ESP_WIFI_SSID);
    snprintf((char*)wifi_config.sta.password, 64, "%s", CONFIG_ESP_WIFI_PASSWORD);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_LOGI(TAG, "...connecting to ap SSID:%s password:%s",CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); // disable powersave. this will make ping go much faster: DEFAULT POWERMODE IS.WIFI_PS_MIN_MODEM
    wifi_status=1;
}


void wifi_stop(void)
{
    if (!wifi_status) return; // already stopped!
    wifi_retrys=5; //dont reconnect!
    esp_wifi_disconnect(); // disconnect the Wi-Fi connectivity.
    esp_wifi_stop(); // stop the Wi-Fi driver.
    esp_wifi_deinit(); // unload the Wi-Fi driver.
//btStop();
    wifi_status=0;

}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id)
    {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        wifi_retrys = 0;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED: // try to reconnect 5 times
    {
        if (wifi_retrys < 5)
        {
            esp_wifi_connect();
            wifi_retrys++;
            ESP_LOGI(TAG,"wifi disconnect, try to reconnect...");
        }
        ESP_LOGI(TAG,"wifi is disconnected");
        break;
    }
    default:
        ESP_LOGI(TAG,"got unknown wifi event:%d!!",event->event_id);
        break;
    }
    return ESP_OK;
}

/*
http://ojisanseiuchi.com/2018/04/29/serving-sensor-data-via-esp32/
https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/lwip.html
We’ll use the Netconn API from the lwIP stack to serve our page. Esentially, this is a sequential API that handles the protocol and keeps us out of the messy implementation details. Mostly.

//  http server task
static void http_server(void *pvParameters) {
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}
*/