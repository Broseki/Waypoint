idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_event nvs_flash driver
                       EMBED_TXTFILES ${project_dir}/certs/whitetail.pem)
