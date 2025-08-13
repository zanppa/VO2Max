/*
 * files.cpp - SPIFFS file system helper functions
 * Copyright (C) 2025 Lauri Peltonen
 * GPL V3
 * 
 **/
 
#include "FS.h"
#include "SPIFFS.H"

#include "files.h"

#include "config.h"

static const char *TAG_FILES = "FS";

static bool fs_ok = false;  // Flag to see if filesystem is ok, if not, skip all operations on files

static const char *storeFileName = "/data_log.dat";  // File path & name where to save the storage data (always root dir)


// Initialize the filesystem
bool init_filesystem()
{
    ESP_LOGI(TAG_FILES, "Initializing filesystem. First time format (if mount fails) may take a minute.");
    if(!SPIFFS.begin(true)) {       // Format filesystem if mount fails (should only happen on first use)
        ESP_LOGE(TAG_FILES, "Could not initialize filesystem");
        fs_ok = false;
        return false;
    }

    ESP_LOGI(TAG_FILES, "Total space %d, used space %d", SPIFFS.totalBytes(), SPIFFS.usedBytes());

    fs_ok = true;
    return true;
}

// Write a buffer to the predefined file
bool write_log_buffer_to_file(const char *buffer, size_t length, size_t block_size, unsigned int position)
{
    File fp;
    bool ok = true;

    if(!fs_ok) return false;

    ESP_LOGI(TAG_FILES, "Starting to write to %s", storeFileName);
    fp = SPIFFS.open(storeFileName, "wb", true);   // Open & clear file for writing binary, create if not exists
    if(!fp) {
        ESP_LOGE(TAG_FILES, "Could not open file %s", storeFileName);
        return false;
    }

    ok &= fp.write(DATA_FILE_MAGIC) == 1;
    ok &= fp.write(DATA_FILE_VERSION) == 1;
    ok &= fp.write((const uint8_t *)&length, sizeof(size_t)) == sizeof(size_t);
    ok &= fp.write((const uint8_t *)&global_settings.storeDataRate, sizeof(int)) == sizeof(int);
    ok &= fp.write((const uint8_t *)&global_settings.integrationTime, sizeof(int)) == sizeof(int);

    // Write first the oldest data (current write position is the oldest one, from that to end of buffer)
    size_t to_write = length - position;
    if(to_write > 0)
        ok &= fp.write((const uint8_t *)&buffer[position * block_size], to_write * block_size) == to_write * block_size;

    // Then write from beginning to the newest data
    to_write = length - to_write;
    if(to_write > 0)
        ok &= fp.write((const uint8_t *)&buffer[0], to_write * block_size) == to_write * block_size;

    if(!ok) {
        ESP_LOGE(TAG_FILES, "File write truncated, not all bytes written");
        fp.close();
        return false;
    }

    ESP_LOGI(TAG_FILES, "File written");
    fp.close();
    return true;
}


static File read_file;
bool open_data_file()
{
    read_file = SPIFFS.open(storeFileName, "rb", false);   // Read binary, do not create if not exists
    if(!read_file) return false;
    return true;
}

void close_data_file()
{
    read_file.close();
}

bool read_data_file(char *buffer, size_t bytes)
{
    return read_file.readBytes(buffer, bytes) == bytes;
}
