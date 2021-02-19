set(COMPONENT_SRCS "esp32cam.c" "tcpserver.c")

set(COMPONENT_REQUIRES
    esp32-camera-master
    nvs_flash
    esp_http_server
    )

set(COMPONENT_EMBED_FILES
        "www/index_ov2640.html.gz"
        "www/index_ov3660.html.gz"
        "www/index_ov5640.html.gz")

register_component()
