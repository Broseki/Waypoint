idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_event nvs_flash driver)
set(COMPONENT_EMBED_TXTFILES certs/whitetail.pem)
