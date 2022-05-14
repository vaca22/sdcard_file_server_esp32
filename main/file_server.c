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

#include <freertos/event_groups.h>
#include <soc/i2s_reg.h>
#include <soc/rtc.h>
#include <driver/periph_ctrl.h>
#include <cJSON.h>

FILE *playFile=NULL;

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

audio_element_handle_t i2s_stream_writer, mp3_decoder;

audio_pipeline_handle_t pipeline;

#define SCRATCH_BUFSIZE  1500

struct file_server_data {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
};

static const char *TAG = "file_server";





static char  rec_buf[64*1024];


#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)


static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
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


static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
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


static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath)
{
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
    cJSON *files= cJSON_CreateArray();
    while ((entry = readdir(dir)) != NULL) {
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
        if(entry->d_type==DT_DIR){
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
    return ESP_OK;
}




static esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
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

    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
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


static esp_err_t upload_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;


    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
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


    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
    int received;


    int remaining = req->content_len;
    int index=0;

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

        if(received>0){
            for(int k=0;k<received;k++){
                if(index<65536){
                    rec_buf[index]=buf[k];
                    index++;
                }else{
                    index=0;
                    fwrite(rec_buf,65536,1,fd);
                    rec_buf[index]=buf[k];
                    index++;
                }
            }
        }

        if(remaining==received){
            if(received<65536){
                fwrite(buf,received,1,fd);
            }
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


static esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;


    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/delete") - 1, sizeof(filepath));
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

static esp_err_t play_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/play") - 1, sizeof(filepath));
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
    if(el_state==AEL_STATE_PAUSED){
        if(strcmp(lastSong,filename)==0){
            audio_pipeline_resume(pipeline);
        }else{
            if(playFile!=NULL){
                fclose(playFile);
                playFile=NULL;
            }
            playFile= fopen(filepath,"rb");
            audio_pipeline_resume(pipeline);
        }

    }else{
        if(playFile!=NULL){
            fclose(playFile);
            playFile=NULL;
        }
        playFile= fopen(filepath,"rb");
        if(el_state==AEL_STATE_FINISHED){
            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
            audio_pipeline_run(pipeline);
        }else if(el_state==AEL_STATE_INIT){
            audio_pipeline_run(pipeline);
        }

    }

    strcpy(lastSong,filename);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File play successfully");
    return ESP_OK;
}


static esp_err_t pause_post_handler(httpd_req_t *req)
{
    audio_pipeline_pause(pipeline);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File play successfully");
    return ESP_OK;
}




int mp3_music_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
  int a=fread(buf,len,1,playFile);
  if(a==0){
      fclose(playFile);
      playFile=NULL;
      return  AEL_IO_DONE;
  }
    return len;
}



esp_err_t start_file_server(const char *base_path)
{

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);
    audio_element_set_read_cb(mp3_decoder, mp3_music_read_cb, NULL);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.i2s_config.use_apll=false;
    i2s_cfg.use_alc=false;

    i2s_stream_writer = i2s_stream_init(&i2s_cfg);



    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    const char *link_tag[2] = {"mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);
    static struct file_server_data *server_data = NULL;


    if (server_data) {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path,
            sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
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


    httpd_uri_t file_delete = {
        .uri       = "/delete/*",
        .method    = HTTP_POST,
        .handler   = delete_post_handler,
        .user_ctx  = server_data
    };

    httpd_uri_t file_play = {
            .uri       = "/play/*",
            .method    = HTTP_POST,
            .handler   = play_post_handler,
            .user_ctx  = server_data
    };
    httpd_uri_t file_pause = {
            .uri       = "/pause/*",
            .method    = HTTP_POST,
            .handler   = pause_post_handler,
            .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_delete);
    httpd_register_uri_handler(server, &file_play);
    httpd_register_uri_handler(server, &file_pause);
    return ESP_OK;
}
