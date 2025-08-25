/*
 * Copyright (C) 2025 Lauri Peltonen
 * GPL V3
 */

#ifndef __FILES_H__
#define __FILES_H__

bool init_filesystem();
bool write_log_buffer_to_file(const char *buffer, size_t length, size_t block_size, unsigned int position);
bool open_data_file();
void close_data_file();
bool read_data_file(char *buffer, size_t bytes);

#define DATA_FILE_MAGIC 0xEB     // Magic identifier for data files
#define DATA_FILE_VERSION 1      // Data file version

#endif
