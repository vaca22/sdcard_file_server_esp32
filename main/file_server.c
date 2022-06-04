#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <audio_event_iface.h>

#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include <string.h>
#include <soc/rtc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "esp_peripherals.h"
#include "periph_touch.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "board.h"
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
#include "CJC8988_Reg.h"

#include <freertos/event_groups.h>
#include <soc/i2s_reg.h>
#include <soc/rtc.h>
#include <driver/periph_ctrl.h>
#include <cJSON.h>
extern int haveSD;
static TaskHandle_t chem1_task_h;
FILE *playFile = NULL;

#define FILE_PATH_MAX (256)

audio_element_handle_t i2s_stream_writer, mp3_decoder;

audio_pipeline_handle_t pipeline;

#define SCRATCH_BUFSIZE  4096
#define SD_Fragment 4096

struct file_server_data {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
};

static const char *TAG = "file_server";










#define I2C_MASTER_SCL_IO           12      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           13      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */



static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}





static char card_buf[SD_Fragment];


#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

void preprocess_string(char *str) {
    char *p, *q;

    for (p = q = str; *p != 0; p++) {
        if (*(p) == '%' && *(p + 1) != 0 && *(p + 2) != 0) {
            // quoted hex
            uint8_t a;
            p++;
            if (*p <= '9')
                a = *p - '0';
            else
                a = toupper((unsigned char) *p) - 'A' + 10;
            a <<= 4;
            p++;
            if (*p <= '9')
                a += *p - '0';
            else
                a += toupper((unsigned char) *p) - 'A' + 10;
            *q++ = a;
        } else if (*(p) == '+') {
            *q++ = ' ';
        } else {
            *q++ = *p;
        }
    }
    *q = '\0';
}

static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename) {
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }

    return httpd_resp_set_type(req, "text/plain");
}


static const char *get_path_from_uri(char *dest, const char *base_path, const char *uri2, size_t destsize) {
    char uri[260];
    strcpy(uri, uri2);
    preprocess_string(uri);
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        return NULL;
    }


    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    return dest + base_pathlen;
}


static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath) {

    if(haveSD){
        char entrypath[FILE_PATH_MAX];
        char entrysize[16];
        const char *entrytype;

        struct dirent *entry;
        struct stat entry_stat;

        DIR *dir = opendir(dirpath);
        const size_t dirpath_len = strlen(dirpath);

        strlcpy(entrypath, dirpath, sizeof(entrypath));

        if (!dir) {
            ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
            return ESP_FAIL;
        }
        cJSON *files = cJSON_CreateArray();
        while ((entry = readdir(dir)) != NULL) {
            entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
            if (entry->d_type == DT_DIR) {
                continue;
            }
            strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
            if (stat(entrypath, &entry_stat) == -1) {
                ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
                continue;
            }
            cJSON_AddItemToArray(files, cJSON_CreateString(entry->d_name));
        }


        httpd_resp_sendstr_chunk(req, cJSON_Print(files));
        httpd_resp_sendstr_chunk(req, NULL);
        cJSON_Delete(files);
    }else{
        httpd_resp_sendstr_chunk(req, NULL);
    }




    return ESP_OK;
}


static esp_err_t download_get_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *) req->user_ctx)->base_path,
                                             req->uri, sizeof(filepath));
    if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if (filename[strlen(filename) - 1] == '/') {
        return http_resp_dir_html(req, filepath);
    }
    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Filexxx does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    char *chunk = ((struct file_server_data *) req->user_ctx)->scratch;
    size_t chunksize;
    do {
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);
        if (chunksize > 0) {
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (chunksize != 0);

    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}









const int total=15000;
char *play_ring_buffer;
const int safeArea=6000;
static int downloadIndex=0;
static int playIndex=0;
const int update_mtu=1500;


int isSafe(){
    if(downloadIndex>=playIndex){
        if(downloadIndex-playIndex<safeArea){
            return 0;
        }else{
            return 1;
        }
    }else{
        if(downloadIndex+total-playIndex<safeArea){
            return 0;
        }else{
            return 1;
        }
    }

}
int isSafe2(int x){
    if(downloadIndex>=playIndex){
        if(downloadIndex-playIndex<x){
            return 0;
        }else{
            return 1;
        }
    }else{
        if(downloadIndex+total-playIndex<x){
            return 0;
        }else{
            return 1;
        }
    }

}






static esp_err_t uploadPlay_post_handler(httpd_req_t *req) {
    audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
    if(el_state!=AEL_STATE_RUNNING){
        audio_pipeline_run(pipeline);
    }
    char buf[update_mtu] ;
    int received=0;
    int remaining = req->content_len;
    while (remaining > 0) {
        if(isSafe()==0){
            if ((received = httpd_req_recv(req, buf, MIN(remaining, update_mtu))) <= 0) {
                continue;
            }else{
                for(int k=0;k<received;k++){
                    if(downloadIndex>=total){
                        downloadIndex=0;
                    }
                    play_ring_buffer[downloadIndex]=buf[k];
                    downloadIndex++;
                }
            }
        }else{
            vTaskDelay(1);
            continue;
        }
        remaining -= received;
    }
    httpd_resp_sendstr(req, "File uploaded successfully");
    return ESP_OK;
}







static esp_err_t upload_post_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    int x = xPortGetCoreID();
    ESP_LOGI(TAG, "Corefuckx  %d", x);
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *) req->user_ctx)->base_path,
                                             req->uri + sizeof("/upload") - 1, sizeof(filepath));
    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == 0) {
        ESP_LOGE(TAG, "File already exists : %s", filepath);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");
        return ESP_FAIL;
    }


    fd = fopen(filepath, "w");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to create file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Receiving file : %s...", filename);


    char *buf = ((struct file_server_data *) req->user_ctx)->scratch;
    int received;


    int remaining = req->content_len;
    int index = 0;

    while (remaining > 0) {


        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            fclose(fd);
            unlink(filepath);

            ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }

        if (received && (received != fwrite(buf, 1, received, fd))) {
            /* Couldn't write everything to file!
             * Storage may be full? */
            fclose(fd);
            unlink(filepath);

            ESP_LOGE(TAG, "File write failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");
            return ESP_FAIL;
        }


        remaining -= received;
    }


    fclose(fd);
    ESP_LOGI(TAG, "File reception complete");


    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File uploaded successfully");
    return ESP_OK;
}


static esp_err_t delete_post_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;


    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *) req->user_ctx)->base_path,
                                             req->uri + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "File does not exist : %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist x55");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting file : %s", filename);

    unlink(filepath);


    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
}

char lastSong[200];

static esp_err_t play_post_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *) req->user_ctx)->base_path,
                                             req->uri + sizeof("/play") - 1, sizeof(filepath));
    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "File does not exist x1: %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist x23");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Play file : %s", filename);


    audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
    if (el_state == AEL_STATE_PAUSED) {
        if (strcmp(lastSong, filename) == 0) {
            audio_pipeline_resume(pipeline);
        } else {
            if (playFile != NULL) {
                fclose(playFile);
                playFile = NULL;
            }
            playFile = fopen(filepath, "rb");
            setvbuf(playFile, card_buf, _IOFBF, SD_Fragment);
            audio_pipeline_resume(pipeline);
        }

    } else {
        if (playFile != NULL) {
            fclose(playFile);
            playFile = NULL;
        }
        playFile = fopen(filepath, "rb");
        setvbuf(playFile, card_buf, _IOFBF,SD_Fragment);
        if (el_state == AEL_STATE_FINISHED) {
            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
            audio_pipeline_run(pipeline);
        } else if (el_state == AEL_STATE_INIT) {
            audio_pipeline_run(pipeline);
        }

    }

    strcpy(lastSong, filename);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File play successfully");
    return ESP_OK;
}


static esp_err_t pause_post_handler(httpd_req_t *req) {
    audio_pipeline_pause(pipeline);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File play successfully");
    return ESP_OK;
}


static esp_err_t volume_post_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *) req->user_ctx)->base_path,
                                             req->uri + sizeof("/volume") - 1, sizeof(filepath));
    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    int duck= atoi(filename+1);
    ESP_LOGE("asdf","%d      %s",duck, filename);
    if(duck==0){
        CJC8988_SET_Volume(0);
    }else{
        ESP_LOGE("asdf","%d",duck);
        CJC8988_SET_Volume(255+duck-100);
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File play successfully");
    return ESP_OK;
}


int mp3_music_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx) {

    if(haveSD==0){
        while (isSafe2(len)==0){
            vTaskDelay(1);
        }

        for(int k=0;k<len;k++){
            if(playIndex>=total){
                playIndex=0;
            }
            buf[k]=play_ring_buffer[playIndex];
            playIndex++;
        }

        return len;
    }

    int a = fread(buf, len, 1, playFile);
    if (a == 0) {
        fclose(playFile);
        playFile = NULL;
        return AEL_IO_DONE;
    }
    return len;
}

static void chem1_task(void *pvParameters) {
    const char *base_path = "/sdcard";
    static struct file_server_data *server_data = NULL;


    if (server_data) {
        ESP_LOGE(TAG, "File server already started");
    }

    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
    }
    strlcpy(server_data->base_path, base_path,
            sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
    }


    httpd_uri_t file_download = {
            .uri       = "/*",
            .method    = HTTP_GET,
            .handler   = download_get_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_download);


    httpd_uri_t file_upload = {
            .uri       = "/upload/*",
            .method    = HTTP_POST,
            .handler   = upload_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_upload);


    httpd_uri_t file_uploadPlay = {
            .uri       = "/uploadPlay/*",
            .method    = HTTP_POST,
            .handler   = uploadPlay_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_uploadPlay);


    httpd_uri_t file_delete = {
            .uri       = "/delete/*",
            .method    = HTTP_POST,
            .handler   = delete_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_delete);
    httpd_uri_t file_play = {
            .uri       = "/play/*",
            .method    = HTTP_POST,
            .handler   = play_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_play);
    httpd_uri_t file_pause = {
            .uri       = "/pause/*",
            .method    = HTTP_POST,
            .handler   = pause_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_pause);

    httpd_uri_t file_volume = {
            .uri       = "/volume/*",
            .method    = HTTP_POST,
            .handler   = volume_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_volume);
    while (1) {

        vTaskDelay(1000);
    }
}

esp_err_t start_file_server() {
    if(haveSD==0){
        play_ring_buffer= malloc(total);
    }
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_cfg.out_rb_size = 16 * 1024;
    mp3_decoder = mp3_decoder_init(&mp3_cfg);
    audio_element_set_read_cb(mp3_decoder, mp3_music_read_cb, NULL);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.i2s_config.use_apll = false;
    i2s_cfg.i2s_config.sample_rate=44100;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);


    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    const char *link_tag[2] = {"mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);


    int x = xPortGetCoreID();
    ESP_LOGI(TAG, "Corefuckxy  %d", x);
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    vTaskDelay(10);
    CJC8988_DAC_TO_LOUT1();

    CJC8988_SET_Volume(200);
    vTaskDelay(100);

    xTaskCreatePinnedToCore(chem1_task, "chem1", 4096, NULL, configMAX_PRIORITIES, &chem1_task_h, 1);
    return ESP_OK;
}
