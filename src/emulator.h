/*
 * Copyright (C) 2017 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef BINJGB_EMULATOR_H_
#define BINJGB_EMULATOR_H_

#include "common.h"
#include <memory>
#include <functional>

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

#define MAXIMUM_ROM_SIZE MEGABYTES(8)
#define MINIMUM_ROM_SIZE KILOBYTES(32)
#define MAX_CART_INFOS (MAXIMUM_ROM_SIZE / MINIMUM_ROM_SIZE)
#define VIDEO_RAM_SIZE KILOBYTES(16)
#define WORK_RAM_SIZE KILOBYTES(32)
#define EXT_RAM_MAX_SIZE KILOBYTES(128)
#define WAVE_RAM_SIZE 16
#define HIGH_RAM_SIZE 127

#define OBJ_PER_LINE_COUNT 10

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


/* Addresses are relative to IO_START_ADDR (0xff00). */
#define FOREACH_IO_REG(V)                           \
  V(JOYP, 0x00)  /* Joypad */                       \
  V(SB, 0x01)    /* Serial transfer data */         \
  V(SC, 0x02)    /* Serial transfer control */      \
  V(DIV, 0x04)   /* Divider */                      \
  V(TIMA, 0x05)  /* Timer counter */                \
  V(TMA, 0x06)   /* Timer modulo */                 \
  V(TAC, 0x07)   /* Timer control */                \
  V(IF, 0x0f)    /* Interrupt request */            \
  V(LCDC, 0x40)  /* LCD control */                  \
  V(STAT, 0x41)  /* LCD status */                   \
  V(SCY, 0x42)   /* Screen Y */                     \
  V(SCX, 0x43)   /* Screen X */                     \
  V(LY, 0x44)    /* Y Line */                       \
  V(LYC, 0x45)   /* Y Line compare */               \
  V(DMA, 0x46)   /* DMA transfer to OAM */          \
  V(BGP, 0x47)   /* BG palette */                   \
  V(OBP0, 0x48)  /* OBJ palette 0 */                \
  V(OBP1, 0x49)  /* OBJ palette 1 */                \
  V(WY, 0x4a)    /* Window Y */                     \
  V(WX, 0x4b)    /* Window X */                     \
  V(KEY1, 0x4d)  /* Prepare speed switch X */       \
  V(VBK, 0x4f)   /* VRAM bank */                    \
  V(HDMA1, 0x51) /* HDMA 1 */                       \
  V(HDMA2, 0x52) /* HDMA 2 */                       \
  V(HDMA3, 0x53) /* HDMA 3 */                       \
  V(HDMA4, 0x54) /* HDMA 4 */                       \
  V(HDMA5, 0x55) /* HDMA 5 */                       \
  V(RP, 0x56)    /* Infrared communications port */ \
  V(BCPS, 0x68)  /* Background palette index */     \
  V(BCPD, 0x69)  /* Background palette data */      \
  V(OCPS, 0x6a)  /* Obj palette index */            \
  V(OCPD, 0x6b)  /* Obj palette data */             \
  V(SVBK, 0x70)  /* WRAM bank */                    \
  V(IE, 0xff)    /* Interrupt enable */

/* Addresses are relative to APU_START_ADDR (0xff10). */
#define FOREACH_APU_REG(V)                                   \
  V(NR10, 0x0)  /* Channel 1 sweep */                        \
  V(NR11, 0x1)  /* Channel 1 sound length/wave pattern */    \
  V(NR12, 0x2)  /* Channel 1 volume envelope */              \
  V(NR13, 0x3)  /* Channel 1 frequency lo */                 \
  V(NR14, 0x4)  /* Channel 1 frequency hi */                 \
  V(NR21, 0x6)  /* Channel 2 sound length/wave pattern */    \
  V(NR22, 0x7)  /* Channel 2 volume envelope */              \
  V(NR23, 0x8)  /* Channel 2 frequency lo */                 \
  V(NR24, 0x9)  /* Channel 2 frequency hi */                 \
  V(NR30, 0xa)  /* Channel 3 DAC enabled */                  \
  V(NR31, 0xb)  /* Channel 3 sound length */                 \
  V(NR32, 0xc)  /* Channel 3 select output level */          \
  V(NR33, 0xd)  /* Channel 3 frequency lo */                 \
  V(NR34, 0xe)  /* Channel 3 frequency hi */                 \
  V(NR41, 0x10) /* Channel 4 sound length */                 \
  V(NR42, 0x11) /* Channel 4 volume envelope */              \
  V(NR43, 0x12) /* Channel 4 polynomial counter */           \
  V(NR44, 0x13) /* Channel 4 counter/consecutive; trigger */ \
  V(NR50, 0x14) /* Sound volume */                           \
  V(NR51, 0x15) /* Sound output select */                    \
  V(NR52, 0x16) /* Sound enabled */

#define FOREACH_CGB_FLAG(V)   \
  V(CGB_FLAG_NONE, 0)         \
  V(CGB_FLAG_SUPPORTED, 0x80) \
  V(CGB_FLAG_REQUIRED, 0xC0)

#define FOREACH_SGB_FLAG(V) \
  V(SGB_FLAG_NONE, 0)       \
  V(SGB_FLAG_SUPPORTED, 3)

#define FOREACH_CART_TYPE(V)                                                   \
  V(CART_TYPE_ROM_ONLY, 0x0, NO_MBC, NO_RAM, NO_BATTERY, NO_TIMER)             \
  V(CART_TYPE_MBC1, 0x1, MBC1, NO_RAM, NO_BATTERY, NO_TIMER)                   \
  V(CART_TYPE_MBC1_RAM, 0x2, MBC1, WITH_RAM, NO_BATTERY, NO_TIMER)             \
  V(CART_TYPE_MBC1_RAM_BATTERY, 0x3, MBC1, WITH_RAM, WITH_BATTERY, NO_TIMER)   \
  V(CART_TYPE_MBC2, 0x5, MBC2, NO_RAM, NO_BATTERY, NO_TIMER)                   \
  V(CART_TYPE_MBC2_BATTERY, 0x6, MBC2, NO_RAM, WITH_BATTERY, NO_TIMER)         \
  V(CART_TYPE_ROM_RAM, 0x8, NO_MBC, WITH_RAM, NO_BATTERY, NO_TIMER)            \
  V(CART_TYPE_ROM_RAM_BATTERY, 0x9, NO_MBC, WITH_RAM, WITH_BATTERY, NO_TIMER)  \
  V(CART_TYPE_MMM01, 0xb, MMM01, NO_RAM, NO_BATTERY, NO_TIMER)                 \
  V(CART_TYPE_MMM01_RAM, 0xc, MMM01, WITH_RAM, NO_BATTERY, NO_TIMER)           \
  V(CART_TYPE_MMM01_RAM_BATTERY, 0xd, MMM01, WITH_RAM, WITH_BATTERY, NO_TIMER) \
  V(CART_TYPE_MBC3_TIMER_BATTERY, 0xf, MBC3, NO_RAM, WITH_BATTERY, WITH_TIMER) \
  V(CART_TYPE_MBC3_TIMER_RAM_BATTERY, 0x10, MBC3, WITH_RAM, WITH_BATTERY,      \
    WITH_TIMER)                                                                \
  V(CART_TYPE_MBC3, 0x11, MBC3, NO_RAM, NO_BATTERY, NO_TIMER)                  \
  V(CART_TYPE_MBC3_RAM, 0x12, MBC3, WITH_RAM, NO_BATTERY, NO_TIMER)            \
  V(CART_TYPE_MBC3_RAM_BATTERY, 0x13, MBC3, WITH_RAM, WITH_BATTERY, NO_TIMER)  \
  V(CART_TYPE_MBC5, 0x19, MBC5, NO_RAM, NO_BATTERY, NO_TIMER)                  \
  V(CART_TYPE_MBC5_RAM, 0x1a, MBC5, WITH_RAM, NO_BATTERY, NO_TIMER)            \
  V(CART_TYPE_MBC5_RAM_BATTERY, 0x1b, MBC5, WITH_RAM, WITH_BATTERY, NO_TIMER)  \
  V(CART_TYPE_MBC5_RUMBLE, 0x1c, MBC5, NO_RAM, NO_BATTERY, NO_TIMER)           \
  V(CART_TYPE_MBC5_RUMBLE_RAM, 0x1d, MBC5, WITH_RAM, NO_BATTERY, NO_TIMER)     \
  V(CART_TYPE_MBC5_RUMBLE_RAM_BATTERY, 0x1e, MBC5, WITH_RAM, WITH_BATTERY,     \
    NO_TIMER)                                                                  \
  V(CART_TYPE_POCKET_CAMERA, 0xfc, NO_MBC, NO_RAM, NO_BATTERY, NO_TIMER)       \
  V(CART_TYPE_BANDAI_TAMA5, 0xfd, TAMA5, NO_RAM, NO_BATTERY, NO_TIMER)         \
  V(CART_TYPE_HUC3, 0xfe, HUC3, NO_RAM, NO_BATTERY, NO_TIMER)                  \
  V(CART_TYPE_HUC1_RAM_BATTERY, 0xff, HUC1, WITH_RAM, WITH_BATTERY, NO_TIMER)

#define FOREACH_ROM_SIZE(V) \
  V(ROM_SIZE_32K, 0, 2)     \
  V(ROM_SIZE_64K, 1, 4)     \
  V(ROM_SIZE_128K, 2, 8)    \
  V(ROM_SIZE_256K, 3, 16)   \
  V(ROM_SIZE_512K, 4, 32)   \
  V(ROM_SIZE_1M, 5, 64)     \
  V(ROM_SIZE_2M, 6, 128)    \
  V(ROM_SIZE_4M, 7, 256)    \
  V(ROM_SIZE_8M, 8, 512)

#define FOREACH_EXT_RAM_SIZE(V)           \
  V(EXT_RAM_SIZE_NONE, 0, 0)              \
  V(EXT_RAM_SIZE_2K, 1, KILOBYTES(2))     \
  V(EXT_RAM_SIZE_8K, 2, KILOBYTES(8))     \
  V(EXT_RAM_SIZE_32K, 3, KILOBYTES(32))   \
  V(EXT_RAM_SIZE_128K, 4, KILOBYTES(128)) \
  V(EXT_RAM_SIZE_64K, 5, KILOBYTES(64))

#define FOREACH_PPU_MODE(V) \
  V(PPU_MODE_HBLANK, 0)     \
  V(PPU_MODE_VBLANK, 1)     \
  V(PPU_MODE_MODE2, 2)      \
  V(PPU_MODE_MODE3, 3)

#define FOREACH_PPU_STATE(V)          \
  V(PPU_STATE_HBLANK, 0)              \
  V(PPU_STATE_HBLANK_PLUS_4, 1)       \
  V(PPU_STATE_VBLANK, 2)              \
  V(PPU_STATE_VBLANK_PLUS_4, 3)       \
  V(PPU_STATE_VBLANK_LY_0, 4)         \
  V(PPU_STATE_VBLANK_LY_0_PLUS_4, 5)  \
  V(PPU_STATE_VBLANK_LINE_Y_0, 6)     \
  V(PPU_STATE_LCD_ON_MODE2, 7)        \
  V(PPU_STATE_MODE2, 8)               \
  V(PPU_STATE_MODE3_EARLY_TRIGGER, 9) \
  V(PPU_STATE_MODE3, 10)              \
  V(PPU_STATE_MODE3_COMMON, 11)

#define DEFINE_ENUM(name, code, ...) name = code,
#define DEFINE_IO_REG_ENUM(name, code, ...) IO_##name##_ADDR = code,
#define DEFINE_APU_REG_ENUM(name, code, ...) APU_##name##_ADDR = code,
#define DEFINE_STRING(name, code, ...) [code] = #name,

static inline const char* get_enum_string(const char** strings,
                                          size_t string_count, size_t value) {
  const char* result = value < string_count ? strings[value] : "unknown";
  return result ? result : "unknown";
}

#define DEFINE_NAMED_ENUM(NAME, Name, name, foreach, enum_def)       \
  typedef enum { foreach (enum_def) NAME##_COUNT } Name;             \
  [[maybe_unused]] static inline bool is_##name##_valid(Name value) {                 \
    return value < NAME##_COUNT;                                     \
  }                                                                  \
  [[maybe_unused]] static inline const char* get_##name##_string(Name value) {        \
    static const char* s_strings[] = {foreach (DEFINE_STRING)};      \
    return get_enum_string(s_strings, ARRAY_SIZE(s_strings), value); \
  }

DEFINE_NAMED_ENUM(CGB_FLAG, CgbFlag, cgb_flag, FOREACH_CGB_FLAG, DEFINE_ENUM)
DEFINE_NAMED_ENUM(SGB_FLAG, SgbFlag, sgb_flag, FOREACH_SGB_FLAG, DEFINE_ENUM)
DEFINE_NAMED_ENUM(CART_TYPE, CartType, cart_type, FOREACH_CART_TYPE,
                  DEFINE_ENUM)
DEFINE_NAMED_ENUM(ROM_SIZE, RomSize, rom_size, FOREACH_ROM_SIZE, DEFINE_ENUM)
DEFINE_NAMED_ENUM(EXT_RAM_SIZE, ExtRamSize, ext_ram_size, FOREACH_EXT_RAM_SIZE,
                  DEFINE_ENUM)
DEFINE_NAMED_ENUM(IO_REG, IOReg, io_reg, FOREACH_IO_REG, DEFINE_IO_REG_ENUM)
DEFINE_NAMED_ENUM(APU_REG, APUReg, apu_reg, FOREACH_APU_REG,
                  DEFINE_APU_REG_ENUM)
DEFINE_NAMED_ENUM(PPU_MODE, PPUMode, ppu_mode, FOREACH_PPU_MODE, DEFINE_ENUM)
DEFINE_NAMED_ENUM(PPU_STATE, PPUState, ppu_state, FOREACH_PPU_STATE,
                  DEFINE_ENUM)

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


enum MbcType {
  MBC_TYPE_NO_MBC,
  MBC_TYPE_MBC1,
  MBC_TYPE_MBC2,
  MBC_TYPE_MBC3,
  MBC_TYPE_MBC5,
  MBC_TYPE_MMM01,
  MBC_TYPE_TAMA5,
  MBC_TYPE_HUC3,
  MBC_TYPE_HUC1,
};

enum ExtRamType {
  EXT_RAM_TYPE_NO_RAM,
  EXT_RAM_TYPE_WITH_RAM,
};

enum BatteryType {
  BATTERY_TYPE_NO_BATTERY,
  BATTERY_TYPE_WITH_BATTERY,
};

enum TimerType {
  TIMER_TYPE_NO_TIMER,
  TIMER_TYPE_WITH_TIMER,
};

struct CartTypeInfo {
  MbcType mbc_type;
  ExtRamType ext_ram_type;
  BatteryType battery_type;
  TimerType timer_type;
};

enum MemoryMapType {
  MEMORY_MAP_ROM0,
  MEMORY_MAP_ROM1,
  MEMORY_MAP_VRAM,
  MEMORY_MAP_EXT_RAM,
  MEMORY_MAP_WORK_RAM0,
  MEMORY_MAP_WORK_RAM1,
  MEMORY_MAP_OAM,
  MEMORY_MAP_UNUSED,
  MEMORY_MAP_IO,
  MEMORY_MAP_APU,
  MEMORY_MAP_WAVE_RAM,
  MEMORY_MAP_HIGH_RAM,
};

enum BankMode {
  BANK_MODE_ROM = 0,
  BANK_MODE_RAM = 1,
};

enum JoypadSelect {
  JOYPAD_SELECT_BOTH = 0,
  JOYPAD_SELECT_BUTTONS = 1,
  JOYPAD_SELECT_DPAD = 2,
  JOYPAD_SELECT_NONE = 3,

  JOYPAD_SGB_BOTH_LOW = 0,
  JOYPAD_SGB_P15_LOW = 1,
  JOYPAD_SGB_P14_LOW = 2,
  JOYPAD_SGB_BOTH_HIGH = 3,
};

enum TimaState {
  TIMA_STATE_NORMAL,
  TIMA_STATE_OVERFLOW,
  TIMA_STATE_RESET,
};

enum SerialClock {
  SERIAL_CLOCK_EXTERNAL = 0,
  SERIAL_CLOCK_INTERNAL = 1,
};

enum DataReadEnable {
  DATA_READ_DISABLE = 0,
  DATA_READ_ENABLE = 3,
};

enum {
  SOUND1,
  SOUND2,
  SOUND3,
  SOUND4,
  VIN,
  SOUND_COUNT,
};

enum SweepDirection {
  SWEEP_DIRECTION_ADDITION = 0,
  SWEEP_DIRECTION_SUBTRACTION = 1,
};

enum EnvelopeDirection {
  ENVELOPE_ATTENUATE = 0,
  ENVELOPE_AMPLIFY = 1,
};

enum WaveDuty {
  WAVE_DUTY_12_5 = 0,
  WAVE_DUTY_25 = 1,
  WAVE_DUTY_50 = 2,
  WAVE_DUTY_75 = 3,
  WAVE_DUTY_COUNT,
};

enum WaveVolume {
  WAVE_VOLUME_MUTE = 0,
  WAVE_VOLUME_100 = 1,
  WAVE_VOLUME_50 = 2,
  WAVE_VOLUME_25 = 3,
  WAVE_VOLUME_COUNT,
};

enum LfsrWidth {
  LFSR_WIDTH_15 = 0, /* 15-bit LFSR */
  LFSR_WIDTH_7 = 1,  /* 7-bit LFSR */
};

enum DmaState {
  DMA_INACTIVE = 0,
  DMA_TRIGGERED = 1,
  DMA_ACTIVE = 2,
};

enum HdmaTransferMode {
  HDMA_TRANSFER_MODE_GDMA = 0,
  HDMA_TRANSFER_MODE_HDMA = 1,
};

enum Speed {
  SPEED_NORMAL = 0,
  SPEED_DOUBLE = 1,
};

enum SgbMask {
  SGB_MASK_CANCEL = 0,
  SGB_MASK_FREEZE = 1,
  SGB_MASK_BLACK = 2,
  SGB_MASK_COLOR0 = 3,
};

enum SgbState {
  SGB_STATE_IDLE,
  SGB_STATE_WAIT_BIT,
  SGB_STATE_READ_BIT,
  SGB_STATE_STOP_BIT,
  SGB_STATE_STOP_WAIT,
};

struct ExtRam {
  u8 data[EXT_RAM_MAX_SIZE];
  size_t size;
  BatteryType battery_type;
};

struct CartInfo {
  size_t offset; /* Offset of cart in FileData. */
  u8* data;      /* == FileData.data + offset */
  size_t size;
  CgbFlag cgb_flag;
  SgbFlag sgb_flag;
  CartType cart_type;
  RomSize rom_size;
  ExtRamSize ext_ram_size;
};

typedef struct {
  u8 byte_2000_3fff;
  u8 byte_4000_5fff;
  BankMode bank_mode;
} Mbc1, Huc1, Mmm01;

struct Mbc3 {
  u8 sec, min, hour;
  u16 day;
  bool day_carry;
  Ticks latch_ticks;
  u8 rtc_reg;
  bool rtc_halt;
  bool latched;
};

struct Mbc5 {
  u8 byte_2000_2fff;
  u8 byte_3000_3fff;
};

struct MemoryMap {
  std::function<u8(MaskedAddress)> read_ext_ram;
  std::function<void(MaskedAddress, u8)> write_rom;
  std::function<void(MaskedAddress, u8)> write_ext_ram;
};

struct MemoryMapState {
  u32 rom_base[2];
  u32 ext_ram_base;
  bool ext_ram_enabled;
  union {
    Mbc1 mbc1;
    Mmm01 mmm01;
    Mbc3 mbc3;
    Huc1 huc1;
    Mbc5 mbc5;
  };
};

struct MemoryTypeAddressPair {
  MemoryMapType type;
  MaskedAddress addr;
};

struct Joypad {
  JoypadButtons buttons;
  JoypadSelect joypad_select;
  u8 last_p10_p13;
  Ticks last_callback; /* The last time joypad callback was called. */
};

struct SGB {
  u8 chr_ram[8192];
  u8 pal_ram[4096];
  u8 attr_ram[4050];
  u8 attr_map[90];
  PaletteRGBA screen_pal[4];
  RGBA border_pal[4][16];
  SgbMask mask;
  SgbState state;
  u8 data[16 * 7];
  u8 bits_read;
  u8 current_packet;
  u8 packet_count;
  u8 current_player;
  u8 player_mask;
  bool player_incremented;
};

enum CpuState {
  CPU_STATE_NORMAL = 0,
  CPU_STATE_STOP = 1,
  CPU_STATE_ENABLE_IME = 2,
  CPU_STATE_HALT_BUG = 3,
  CPU_STATE_HALT = 4,
  CPU_STATE_HALT_DI = 5,
};

struct Interrupt {
  bool ime;      /* Interrupt Master Enable */
  u8 ie;         /* Interrupt Enable */
  u8 if_;        /* Interrupt Request, delayed by 1 tick for some IRQs. */
  u8 new_if;     /* The new value of IF, updated in 1 tick. */
  CpuState state;
};

struct Timer {
  Ticks sync_ticks;        /* Current synchronization ticks. */
  Ticks next_intr_ticks;   /* Tick when the next timer intr will occur. */
  TimerClock clock_select; /* Select the rate of TIMA */
  TimaState tima_state;    /* Used to implement TIMA overflow delay. */
  u16 div_counter; /* Internal clock counter, upper 8 bits are DIV. */
  u8 tima;         /* Incremented at rate defined by clock_select */
  u8 tma;          /* When TIMA overflows, it is set to this value */
  bool on;
};

struct Serial {
  Ticks sync_ticks;       /* Current synchronization ticks. */
  Ticks tick_count;       /* 0..SERIAL_TICKS */
  Ticks next_intr_ticks;  /* Tick when the next intr will occur. */
  SerialClock clock;
  bool transferring;
  u8 sb; /* Serial transfer data. */
  u8 transferred_bits;
};

struct Infrared {
  bool write;
  bool read;
  DataReadEnable enabled;
};

struct Sweep {
  u8 period;
  SweepDirection direction;
  u8 shift;
  u16 frequency;
  u8 timer; /* 0..period */
  bool enabled;
  bool calculated_subtract;
};

struct Envelope {
  u8 initial_volume;
  EnvelopeDirection direction;
  u8 period;
  u8 volume;      /* 0..15 */
  u32 timer;      /* 0..period */
  bool automatic; /* true when MAX/MIN has not yet been reached. */
};

/* Channel 1 and 2 */
struct SquareWave {
  WaveDuty duty;
  u8 sample;   /* Last sample generated, 0..1 */
  u32 period;  /* Calculated from the frequency. */
  u8 position; /* Position in the duty tick, 0..7 */
  u32 ticks;   /* 0..period */
};

/* Channel 3 */
struct Wave {
  WaveVolume volume;
  u8 volume_shift;
  u8 ram[WAVE_RAM_SIZE];
  Ticks sample_time; /* Time (in ticks) the sample was read. */
  u8 sample_data;    /* Last sample generated, 0..1 */
  u32 period;        /* Calculated from the frequency. */
  u8 position;       /* 0..31 */
  u32 ticks;         /* 0..period */
  bool playing;      /* true if the channel has been triggered but the DAC not
                             disabled. */
};

/* Channel 4 */
struct Noise {
  u8 clock_shift;
  LfsrWidth lfsr_width;
  u8 divisor; /* 0..NOISE_DIVISOR_COUNT */
  u8 sample;  /* Last sample generated, 0..1 */
  u16 lfsr;   /* Linear feedback shift register, 15- or 7-bit. */
  u32 period; /* Calculated from the clock_shift and divisor. */
  u32 ticks;  /* 0..period */
};

struct Channel {
  SquareWave square_wave; /* Channel 1, 2 */
  Envelope envelope;      /* Channel 1, 2, 4 */
  u16 frequency;          /* Channel 1, 2, 3 */
  u16 length;             /* All channels */
  bool length_enabled;    /* All channels */
  bool dac_enabled;
  bool status;     /* Status bit for NR52 */
  u32 accumulator; /* Accumulates samples for resampling. */
};

struct Apu {
  u8 so_volume[SOUND_OUTPUT_COUNT];
  bool so_output[SOUND_COUNT][SOUND_OUTPUT_COUNT];
  bool enabled;
  Sweep sweep;
  Wave wave;
  Noise noise;
  Channel channel[APU_CHANNEL_COUNT];
  u8 frame;         /* 0..FRAME_SEQUENCER_COUNT */
  Ticks sync_ticks; /* Raw tick counter */
  bool initialized;
};

struct Lcdc {
  bool display;
  TileMapSelect window_tile_map_select;
  bool window_display;
  TileDataSelect bg_tile_data_select;
  TileMapSelect bg_tile_map_select;
  ObjSize obj_size;
  bool obj_display;
  bool bg_display;
};

struct StatInterrupt {
  bool irq;
  bool trigger;
};

struct Stat {
  StatInterrupt y_compare;
  StatInterrupt mode2;
  StatInterrupt vblank;
  StatInterrupt hblank;
  bool ly_eq_lyc;       /* true if ly=lyc, delayed by 1 tick. */
  PPUMode mode;         /* The current PPU mode. */
  bool if_;             /* Internal interrupt flag for STAT interrupts. */
  PPUMode trigger_mode; /* This mode is used for checking STAT IRQ triggers. */
  bool new_ly_eq_lyc;   /* The new value for ly_eq_lyc, updated in 1 tick. */
};

struct ColorPalettes {
  PaletteRGBA palettes[8];
  u8 data[64];
  u8 index;
  bool auto_increment;
};

struct Ppu {
  Ticks sync_ticks;                 /* Current synchronization tick. */
  Ticks next_intr_ticks;            /* Tick when the next intr will occur. */
  Lcdc lcdc;                        /* LCD control */
  Stat stat;                        /* LCD status */
  u8 scy;                           /* Screen Y */
  u8 scx;                           /* Screen X */
  u8 ly;                            /* Line Y */
  u8 lyc;                           /* Line Y Compare */
  u8 wy;                            /* Window Y */
  u8 wx;                            /* Window X */
  Palette pal[PALETTE_TYPE_COUNT];  /* BGP, OBP0, OBP1 Palettes */
  ColorPalettes bgcp;               /* BG Color Palettes */
  ColorPalettes obcp;               /* OBJ Color Palettes */
  PPUState state;
  Ticks mode3_render_ticks; /* Ticks at last mode3 synchronization. */
  Ticks line_start_ticks; /* Ticks at the start of this line_y. */
  u32 state_ticks;
  u32 frame;      /* The currently rendering frame. */
  u8 last_ly;     /* LY from the previous tick. */
  u8 render_x;    /* Currently rendering X coordinate. */
  u8 line_y;      /* The currently rendering line. Can be different than LY. */
  u8 win_y;       /* The window Y is only incremented when rendered. */
  Obj line_obj[OBJ_PER_LINE_COUNT]; /* Cached from OAM during mode2. */
  u8 line_obj_count;     /* Number of sprites to draw on this line. */
  bool rendering_window; /* true when this line is rendering the window. */
  u8 display_delay_frames; /* Wait this many frames before displaying. */
};

struct Dma {
  Ticks sync_ticks;       /* Current synchronization tick. */
  Ticks tick_count;       /* 0..DMA_TICKS */
  DmaState state;         /* Used to implement DMA delay. */
  Address source;         /* Source address; dest is calculated from this. */
};

struct CpuSpeed {
  Speed speed;
  bool switching;
};

struct Vram {
  u8 data[VIDEO_RAM_SIZE];
  Address offset;
  u8 bank;
};

struct Wram {
  u8 data[WORK_RAM_SIZE];
  Address offset;
  u8 bank;
};

struct Hdma {
  DmaState state;
  Address source;
  Address dest;
  HdmaTransferMode mode;
  u8 blocks;
  u8 block_bytes;
};

struct EmulatorState {
  u32 header; /* Set to SAVE_STATE_HEADER; makes it easier to save state. */
  u32 random_seed;
  u8 cart_info_index;
  MemoryMapState memory_map_state;
  Registers reg;
  Vram vram;
  ExtRam ext_ram;
  Wram wram;
  Interrupt interrupt;
  Obj oam[OBJ_COUNT];
  Joypad joyp;
  SGB sgb;
  Serial serial;
  Infrared infrared;
  Timer timer;
  Apu apu;
  Ppu ppu;
  Dma dma;
  Hdma hdma;
  CpuSpeed cpu_speed;
  u8 hram[HIGH_RAM_SIZE];
  Ticks ticks;
  Ticks cpu_tick;
  Ticks next_intr_ticks; /* For Timer, Serial, or PPU interrupts. */
  bool is_cgb;
  bool is_sgb;
  bool ext_ram_updated;
  EmulatorEvent event;
};

const size_t s_emulator_state_size = sizeof(EmulatorState);

struct Emulator {
  EmulatorConfig config;
  FileData file_data;
  CartInfo cart_infos[MAX_CART_INFOS];
  u32 cart_info_count;
  CartInfo* cart_info; /* Cached for convenience. */
  MemoryMap memory_map;
  EmulatorState state;
  FrameBuffer frame_buffer;
  SgbFrameBuffer sgb_frame_buffer;
  AudioBuffer audio_buffer;
  JoypadCallbackInfo joypad_info;
  /* color_to_rgba stores mappings from 4 DMG colors to RGBA colors. pal is a
   * cached copy of the current DMG palette (e.g. could be all COLOR_WHITE). */
  PaletteRGBA color_to_rgba[PALETTE_TYPE_COUNT];
  PaletteRGBA pal[PALETTE_TYPE_COUNT];
  PaletteRGBA sgb_pal[4];
  CgbColorCurve cgb_color_curve;
  ApuLog apu_log;

  Emulator();
  ~Emulator();

  static std::unique_ptr<Emulator> try_create(const EmulatorInit*);

  void emulator_set_joypad_buttons(JoypadButtons*);
  void emulator_set_joypad_callback(JoypadCallback, void* user_data);
  JoypadCallbackInfo emulator_get_joypad_callback();
  void emulator_set_config(const EmulatorConfig*);
  EmulatorConfig emulator_get_config();

  FrameBuffer* emulator_get_frame_buffer();
  SgbFrameBuffer* emulator_get_sgb_frame_buffer();
  AudioBuffer* emulator_get_audio_buffer();

  Ticks emulator_get_ticks();
  u32 emulator_get_ppu_frame();

  void emulator_set_builtin_palette(u32 index);
  void emulator_set_bw_palette(PaletteType, const PaletteRGBA*);
  void emulator_set_all_bw_palettes(const PaletteRGBA*);

  bool emulator_was_ext_ram_updated();

  void emulator_init_ext_ram_file_data(FileData*);

  Result emulator_read_state(const FileData*);
  Result emulator_write_state(FileData*);
  Result emulator_read_ext_ram(const FileData*);
  Result emulator_write_ext_ram(FileData*);


  Result emulator_read_state_from_file(const char* filename);
  Result emulator_write_state_to_file(const char* filename);
  Result emulator_read_ext_ram_from_file(const char* filename);
  Result emulator_write_ext_ram_to_file(const char* filename);

  EmulatorEvent emulator_step();
  EmulatorEvent emulator_run_until(Ticks until_ticks);
  void emulator_step_internal();
  void execute_instruction();
  void dispatch_interrupt();

  ApuLog* emulator_get_apu_log();
  void emulator_reset_apu_log();

  void tick();

  u8 read_u8_tick(Address addr);
  u16 read_u16_tick(Address addr);
  void write_u8_tick(Address addr, u8 value);
  void write_u16_tick(Address addr, u16 value);

  // TODO
  void set_rom_bank(int index, u16 bank);
  void set_ext_ram_bank(u8 bank);
  u8 gb_read_ext_ram(MaskedAddress addr);
  void gb_write_ext_ram(MaskedAddress addr, u8 value);

  void mbc1_write_rom_shared(u16 bank_lo_mask, int bank_hi_shift, MaskedAddress addr, u8 value);
  void mbc1_write_rom(MaskedAddress addr, u8 value);
  void mbc1m_write_rom(MaskedAddress addr, u8 value);
  void mbc2_write_rom(MaskedAddress addr, u8 value);
  u8 mbc2_read_ram(MaskedAddress addr);
  void mbc2_write_ram(MaskedAddress addr, u8 value);
  void mbc3_write_rom(MaskedAddress addr, u8 value);
  u8 mbc3_read_ext_ram(MaskedAddress addr);
  void mbc3_write_ext_ram(MaskedAddress addr, u8 value);
  void mbc5_write_rom(MaskedAddress addr, u8 value);
  void huc1_write_rom(MaskedAddress addr, u8 value);
  void mmm01_write_rom(MaskedAddress addr, u8 value);
  Result init_memory_map();
  bool is_almost_mode3();
  bool is_using_vram(bool write);
  bool is_using_oam(bool write);
  u8 read_vram(MaskedAddress addr);
  u8 read_oam(MaskedAddress addr);
  u8 read_joyp_p10_p13();
  void call_joyp_callback(bool wait);
  u8 read_io(MaskedAddress addr);
  u8 read_apu(MaskedAddress addr);
  u8 read_wave_ram(MaskedAddress addr);
  bool is_dma_access_ok(Address addr);

  u8 read_u8_pair(MemoryTypeAddressPair pair, bool raw);
  u8 read_u8_raw(Address addr);
  u8 read_u8(Address addr);
  void write_vram(MaskedAddress addr, u8 value);
  void write_oam_no_mode_check(MaskedAddress addr, u8 value);
  void write_oam(MaskedAddress addr, u8 value);

  void calculate_next_intr();
  bool is_div_falling_edge(u16 old_div_counter, u16 div_counter);
  void increment_tima();
  void timer_synchronize();
  void calculate_next_timer_intr();
  void do_timer_interrupt();
  void clear_div();
  void check_stat();
  void check_ly_eq_lyc(bool write);
  void check_joyp_intr();
  // void update_bw_palette_rgba(PaletteType type);
  // RGBA unpack_cgb_color(u16 color);
  // RGBA unpack_cgb_color8(u8 lo, u8 hi);
  // void set_sgb_palette(int pal, u8 lo0, u8 hi0, u8 lo1,
  //                      u8 hi1, u8 lo2, u8 hi2, u8 lo3, u8 hi3);
  // void unpack_sgb_palette_ram(int pal, u8 idx_lo, u8 idx_hi);
  // void clear_frame_buffer(RGBA color);
  // void update_sgb_mask();
  // void set_sgb_attr(u8 byte);
  // void set_sgb_attr_block(int x0, int y0, int x1, int y1, u8 pal);

  // void do_sgb();
  void write_io(MaskedAddress addr, u8 value);

  // void write_nrx1_reg(Channel* channel, Address addr, u8 value);
  // void write_nrx2_reg(Channel* channel, Address addr, u8 value);
  // void write_nrx3_reg(Channel* channel, u8 value);
  // bool write_nrx4_reg(Channel* channel, Address addr, u8 value, u16 max_length);
  // void trigger_nrx4_envelope(Envelope* envelope, Address addr);
  // u16 calculate_sweep_frequency();
  // void trigger_nr14_reg(Channel* channel);
  // void write_wave_period(Channel* channel);
  // void write_noise_period();
  // void write_apu(MaskedAddress addr, u8 value);
  // void write_wave_ram(MaskedAddress addr, u8 value);
  // void write_u8_pair(MemoryTypeAddressPair pair, u8 value);
  // void write_u8_raw(Address addr, u8 value);
  // void write_u8(Address addr, u8 value);
  // void do_ppu_mode2();
  // u32 mode3_tick_count();
  // void ppu_mode3_synchronize();
  // void ppu_synchronize();
  // void calculate_next_ppu_intr();
  // void update_sweep();
  // void update_lengths();
  // void update_envelopes();
  // void update_wave(u32 apu_ticks, u32 total_frames);
  // void update_noise(u32 total_frames);

  // u32 get_gb_frames_until_next_resampled_frame();
  // void write_audio_frame(u32 gb_frames);
  // void apu_update_channels(u32 total_frames);
  // void apu_update(u32 total_ticks);
  // void intr_synchronize();
  // void apu_synchronize();
  // void dma_synchronize();
  // void hdma_copy_byte();
  // void calculate_next_serial_intr();
  // void serial_synchronize();
  // u16 get_af_reg();
  // void set_af_reg(u16 af);
  // Result init_audio_buffer(u32 frequency, u32 frames);
  // Result init_emulator(const EmulatorInit* init);
};

extern const size_t s_emulator_state_size;

u32 audio_buffer_get_frames(AudioBuffer*);

void emulator_ticks_to_time(Ticks, u32* day, u32* hr, u32* min, u32* sec,
                            u32* ms);


void emulator_init_state_file_data(FileData*);

#endif /* BINJGB_EMULATOR_H_ */
