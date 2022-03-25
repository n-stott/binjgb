/*
 * Copyright (C) 2017 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char* replace_extension(const char* filename, const char* extension) {
  size_t length = strlen(filename) + strlen(extension) + 1; /* +1 for \0. */
  char* result = reinterpret_cast<char*>(xmalloc(length));
  const char* last_dot = strrchr(filename, '.');
  if (last_dot == NULL) {
    snprintf(result, length, "%s%s", filename, extension);
  } else {
    snprintf(result, length, "%.*s%s", (int)(last_dot - filename), filename,
             extension);
  }
  return result;
}

static Result get_file_size(FILE* f, long* out_size) {
  if(!(fseek(f, 0, SEEK_END) >= 0)) return ERROR; // fseek to end failed
  long size = ftell(f);
  if(!(size >= 0)) return ERROR; // ftell failed
  if(!(fseek(f, 0, SEEK_SET) >= 0)) return ERROR; // fseek to beginning failed
  *out_size = size;
  return OK;
}

Result file_read(const char* filename, FileData* out_file_data) {
  FILE* f = fopen(filename, "rb");
  auto onError = [&]() {
    if (f) { fclose(f); } 
    return ERROR;
  };
  if(!(f)) return onError(); // unable to open file filename
  long size;
  if(!(SUCCESS(get_file_size(f, &size)))) return onError();
  u8* data = reinterpret_cast<u8*>(xmalloc(size));
  if(!(data)) return ERROR; // allocation failed
  if (!(fread(data, size, 1, f) == 1)) return onError(); // fread failed
  fclose(f);
  out_file_data->data = data;
  out_file_data->size = size;
  return OK;
}

Result file_write(const char* filename, const FileData* file_data) {
  FILE* f = fopen(filename, "wb");
  if(!(f)) return ERROR; // unable to open file filename
  if(!(fwrite(file_data->data, file_data->size, 1, f) == 1)) if (f) { fclose(f); } return ERROR;; // fwrite failed
  fclose(f);
  return OK;
}

void file_data_delete(FileData* file_data) {
  xfree(file_data->data);
  file_data->size = 0;
  file_data->data = NULL;
}
