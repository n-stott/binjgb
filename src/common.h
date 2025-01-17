/*
 * Copyright (C) 2017 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef BINJGB_COMMON_H_
#define BINJGB_COMMON_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "memory.h"

#if defined(__clang__) || defined(__GNUC__)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define LIKELY(x) __builtin_expect(!!(x), 1)
#else
#define UNLIKELY(x) (x)
#define LIKELY(x) (x)
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ZERO_MEMORY(x) memset(&(x), 0, sizeof(x))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define CLAMP(x, min, max) MIN(max, MAX(min, x))
#define NEXT_MODULO(value, mod) ((mod) - (value) % (mod))
#define KILOBYTES(x) ((size_t)(x) * 1024)
#define MEGABYTES(x) ((size_t)(x) * 1024 * 1024)
#define GIGABYTES(x) ((size_t)(x) * 1024 * 1024 * 1024)
#define INVALID_TICKS (~0ULL)
#define ALIGN_UP(x, align) (((x) + (align) - 1) & ~((align) - 1))
#define ALIGN_DOWN(x, align) ((x) & ~((align) - 1))
#define IS_ALIGNED(x, align) (((x) & ((align) - 1)) == 0)

#define SUCCESS(x) ((x) == OK)
#define PRINT_ERROR(...) fprintf(stderr, __VA_ARGS__)
#define CHECK_MSG(x, ...)                       \
  if (!(x)) {                                   \
    PRINT_ERROR("%s:%d: ", __FILE__, __LINE__); \
    PRINT_ERROR(__VA_ARGS__);                   \
    goto error;                                 \
  }
#define CHECK(x) do if (!(x)) { goto error; } while(0)
#define ON_ERROR_RETURN \
  error:                \
  return ERROR
#define ON_ERROR_CLOSE_FILE_AND_RETURN \
  error:                               \
  if (f) {                             \
    fclose(f);                         \
  }                                    \
  return ERROR
#define UNREACHABLE(...) PRINT_ERROR(__VA_ARGS__), exit(1)

#define MAKE_RGBA(r, g, b, a)                                          \
  (((u32)(u8)(a) << 24) | ((u32)(u8)(b) << 16) | ((u32)(u8)(g) << 8) | \
   ((u32)(u8)(r)))

#define LOWER_BOUND(Type, var, init_begin, init_end, to_find, GET, CMP) \
  Type* var = NULL;                                                     \
  if (init_end - init_begin != 0) {                                     \
    Type* begin_ = init_begin; /* Inclusive. */                         \
    Type* end_ = init_end;     /* Exclusive. */                         \
    while (end_ - begin_ > 1) {                                         \
      Type* mid_ = begin_ + ((end_ - begin_) / 2);                      \
      if (to_find == GET(*mid_)) {                                      \
        begin_ = mid_;                                                  \
        break;                                                          \
      } else if (CMP(to_find, GET(*mid_))) {                            \
        end_ = mid_;                                                    \
      } else {                                                          \
        begin_ = mid_;                                                  \
      }                                                                 \
    }                                                                   \
    var = begin_;                                                       \
  }

using s8 = int8_t;
using s32 = int32_t;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using f32 = float;
using f64 = double;
using Address = u16;
using MaskedAddress = u16;
using RGBA = u32;
using Ticks = u64;
enum Result { OK = 0, ERROR = 1 };

struct FileData {
  u8* data;
  size_t size;
};

struct JoypadButtons {
  bool down, up, left, right;
  bool start, select, B, A;
};

const char* replace_extension(const char* filename, const char* extension);
Result file_read(const char* filename, FileData* out);
Result file_write(const char* filename, const FileData*);
void file_data_delete(FileData*);

#endif /*  BINJGB_COMMON_H_ */
