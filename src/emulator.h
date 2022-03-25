/*
 * Copyright (C) 2017 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef BINJGB_EMULATOR_H_
#define BINJGB_EMULATOR_H_

#include "common.h"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144
#define SCREEN_HEIGHT_WITH_VBLANK 154
#define SGB_SCREEN_WIDTH 256
#define SGB_SCREEN_HEIGHT 224
#define SGB_SCREEN_LEFT ((SGB_SCREEN_WIDTH - SCREEN_WIDTH) / 2)
#define SGB_SCREEN_RIGHT ((SGB_SCREEN_WIDTH + SCREEN_WIDTH) / 2)
#define SGB_SCREEN_TOP ((SGB_SCREEN_HEIGHT - SCREEN_HEIGHT) / 2)
#define SGB_SCREEN_BOTTOM ((SGB_SCREEN_HEIGHT + SCREEN_HEIGHT) / 2)

#define CPU_TICKS_PER_SECOND 4194304
#define APU_TICKS_PER_SECOND 2097152
#define PPU_LINE_TICKS 456
#define PPU_VBLANK_TICKS (PPU_LINE_TICKS * 10)
#define PPU_FRAME_TICKS (PPU_LINE_TICKS * SCREEN_HEIGHT_WITH_VBLANK)

#define SOUND_OUTPUT_COUNT 2
#define PALETTE_COLOR_COUNT 4
#define OBJ_COUNT 40

#define OBJ_X_OFFSET 8
#define OBJ_Y_OFFSET 16

#define BUILTIN_PALETTE_COUNT 84

#define RGBA_WHITE 0xffffffffu
#define RGBA_LIGHT_GRAY 0xffaaaaaau
#define RGBA_DARK_GRAY 0xff555555u
#define RGBA_BLACK 0xff000000u

#define MAX_APU_LOG_FRAME_WRITES 1024

struct Emulator;

enum {
  APU_CHANNEL1,
  APU_CHANNEL2,
  APU_CHANNEL3,
  APU_CHANNEL4,
  APU_CHANNEL_COUNT,
};

using JoypadCallback = void (*)(JoypadButtons* joyp, void* user_data);

struct JoypadCallbackInfo {
  JoypadCallback callback;
  void* user_data;
};

using FrameBuffer = RGBA[SCREEN_WIDTH * SCREEN_HEIGHT];
using SgbFrameBuffer = RGBA[SGB_SCREEN_WIDTH * SGB_SCREEN_HEIGHT];

enum Color {
  COLOR_WHITE = 0,
  COLOR_LIGHT_GRAY = 1,
  COLOR_DARK_GRAY = 2,
  COLOR_BLACK = 3,
};

enum PaletteType {
  PALETTE_TYPE_BGP,
  PALETTE_TYPE_OBP0,
  PALETTE_TYPE_OBP1,
  PALETTE_TYPE_COUNT,
};

enum TileDataSelect {
  TILE_DATA_8800_97FF = 0,
  TILE_DATA_8000_8FFF = 1,
};

enum TileMapSelect {
  TILE_MAP_9800_9BFF = 0,
  TILE_MAP_9C00_9FFF = 1,
};

enum ObjSize {
  OBJ_SIZE_8X8 = 0,
  OBJ_SIZE_8X16 = 1,
};

enum ObjPriority {
  OBJ_PRIORITY_ABOVE_BG = 0,
  OBJ_PRIORITY_BEHIND_BG = 1,
};

enum TimerClock {
  TIMER_CLOCK_4096_HZ = 0,
  TIMER_CLOCK_262144_HZ = 1,
  TIMER_CLOCK_65536_HZ = 2,
  TIMER_CLOCK_16384_HZ = 3,
};

/* TODO(binji): endianness */
#define REGISTER_PAIR(X, Y) \
  union {                   \
    struct { u8 Y; u8 X; }; \
    u16 X##Y;               \
  }

struct Registers {
  u8 A;
  REGISTER_PAIR(B, C);
  REGISTER_PAIR(D, E);
  REGISTER_PAIR(H, L);
  u16 SP;
  u16 PC;
  struct { bool Z, N, H, C; } F;
};

struct Obj {
  u8 y;
  u8 x;
  u8 tile;
  u8 byte3;
  ObjPriority priority;
  bool yflip;
  bool xflip;
  u8 palette;
  u8 bank;
  u8 cgb_palette;
};

struct Palette {
  Color color[PALETTE_COLOR_COUNT];
};

struct PaletteRGBA {
  RGBA color[PALETTE_COLOR_COUNT];
};

struct AudioBuffer {
  u32 frequency;    /* Sample frequency, as N samples per second */
  u32 freq_counter; /* Used for resampling; [0..APU_TICKS_PER_SECOND). */
  u32 divisor;
  u32 frames; /* Number of frames to generate per call to emulator_run. */
  u8* data;   /* Unsigned 8-bit 2-channel samples @ |frequency| */
  u8* end;
  u8* position;
};

enum CgbColorCurve {
  CGB_COLOR_CURVE_NONE,
  CGB_COLOR_CURVE_SAMEBOY_EMULATE_HARDWARE,
  CGB_COLOR_CURVE_GAMBATTE,
};

struct EmulatorInit {
  FileData rom;
  int audio_frequency;
  int audio_frames;
  u32 random_seed;
  u32 builtin_palette;
  bool force_dmg;
  CgbColorCurve cgb_color_curve;
};

struct EmulatorConfig {
  bool disable_sound[APU_CHANNEL_COUNT];
  bool disable_bg;
  bool disable_window;
  bool disable_obj;
  bool allow_simulataneous_dpad_opposites;
  bool log_apu_writes;
};

struct ApuWrite {
  u8 addr;
  u8 value;
};

struct ApuLog {
  ApuWrite writes[MAX_APU_LOG_FRAME_WRITES];
  size_t write_count;
};

using EmulatorEvent = u32;

enum {
  EMULATOR_EVENT_NEW_FRAME = 0x1,
  EMULATOR_EVENT_AUDIO_BUFFER_FULL = 0x2,
  EMULATOR_EVENT_UNTIL_TICKS = 0x4,
  EMULATOR_EVENT_BREAKPOINT = 0x8,
  EMULATOR_EVENT_INVALID_OPCODE = 0x10,
};

extern const size_t s_emulator_state_size;

Emulator* emulator_new(const EmulatorInit*);
void emulator_delete(Emulator*);

void emulator_set_joypad_buttons(Emulator*, JoypadButtons*);
void emulator_set_joypad_callback(Emulator*, JoypadCallback, void* user_data);
JoypadCallbackInfo emulator_get_joypad_callback(Emulator*);
void emulator_set_config(Emulator*, const EmulatorConfig*);
EmulatorConfig emulator_get_config(Emulator*);
FrameBuffer* emulator_get_frame_buffer(Emulator*);
SgbFrameBuffer* emulator_get_sgb_frame_buffer(Emulator*);
AudioBuffer* emulator_get_audio_buffer(Emulator*);
Ticks emulator_get_ticks(Emulator*);
u32 emulator_get_ppu_frame(Emulator*);
u32 audio_buffer_get_frames(AudioBuffer*);
void emulator_set_builtin_palette(Emulator*, u32 index);
void emulator_set_bw_palette(Emulator*, PaletteType, const PaletteRGBA*);
void emulator_set_all_bw_palettes(Emulator*, const PaletteRGBA*);

void emulator_ticks_to_time(Ticks, u32* day, u32* hr, u32* min, u32* sec,
                            u32* ms);

bool emulator_was_ext_ram_updated(Emulator*);

void emulator_init_state_file_data(FileData*);
void emulator_init_ext_ram_file_data(Emulator*, FileData*);
Result emulator_read_state(Emulator*, const FileData*);
Result emulator_write_state(Emulator*, FileData*);
Result emulator_read_ext_ram(Emulator*, const FileData*);
Result emulator_write_ext_ram(Emulator*, FileData*);

Result emulator_read_state_from_file(Emulator*, const char* filename);
Result emulator_write_state_to_file(Emulator*, const char* filename);
Result emulator_read_ext_ram_from_file(Emulator*, const char* filename);
Result emulator_write_ext_ram_to_file(Emulator*, const char* filename);

EmulatorEvent emulator_step(Emulator*);
EmulatorEvent emulator_run_until(Emulator*, Ticks until_ticks);

ApuLog* emulator_get_apu_log(Emulator*);
void emulator_reset_apu_log(Emulator*);

#endif /* BINJGB_EMULATOR_H_ */
