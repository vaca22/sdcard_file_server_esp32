#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "soc/soc_caps.h"
#include "driver/sdmmc_host.h"
#include <freertos/event_groups.h>
#include <soc/i2s_reg.h>
#include <soc/rtc.h>
#include <driver/periph_ctrl.h>
#include "lwip/sockets.h"
#include "lwip/opt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOUNT_POINT "/sdcard"
static const char *TAG = "example";
int haveSD=false;
int sdStatus=false;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0


static sdmmc_card_t *mount_card = NULL;
static char *mount_base_path = MOUNT_POINT;


esp_err_t start_file_server();

void sdcard_mount2(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        ESP_ERROR_CHECK(ret);
    }
    sdmmc_card_print_info(stdout, card);

}


#define GPIO_OUTPUT_IO_0    32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) )

void setIo32() {
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
}


/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *) event_data;

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;
        default:
            break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}

void initEthernet() {
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create new default instance of esp-netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    esp_netif_set_hostname(eth_netif, "lgh");
    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = 33;
    mac_config.smi_mdc_gpio_num = 23;
    mac_config.smi_mdio_gpio_num = 18;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}



int TCP_PORT = 13207;








static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = (int) pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *) &dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(TCP_PORT);
        ip_protocol = IPPROTO_IP;
    }


    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));


    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *) &source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);


        shutdown(sock, 0);
        close(sock);
    }

    CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void restart_timer_callback(void* arg)
{
    ESP_LOGI(TAG, "Restarting now...");
    sdcard_mount2();
}
esp_timer_create_args_t restart_timer_args = {
        .callback = &restart_timer_callback,
        /* argument specified here will be passed to timer callback function */
        .arg = (void*) 0,
        .name = "restart_timer"
};
esp_timer_handle_t restart_timer;




#define GPIO_INPUT_IO_0     14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) )
#define ESP_INTR_FLAG_DEFAULT 0


static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(gpio_get_level(io_num)==0){
                haveSD=1;
                if(sdStatus== false){
                    sdStatus=true;
                    esp_timer_start_once(restart_timer, 1000000);
                    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
                }
            }
        }
    }
}


void initDetectSD(){
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type =GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    esp_timer_create(&restart_timer_args, &restart_timer);
}





void app_main(void) {
    s_wifi_event_group = xEventGroupCreate();
    setIo32();
    initDetectSD();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initEthernet();
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);



    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
    if(gpio_get_level(GPIO_INPUT_IO_0)==0){
        haveSD=1;
        gpio_isr_handler_remove(GPIO_INPUT_IO_0);
        sdcard_mount2();
    }else{
        haveSD=0;
    }

    start_file_server();
    vEventGroupDelete(s_wifi_event_group);
}
