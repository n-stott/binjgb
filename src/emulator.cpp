/*
 * Copyright (C) 2016 Ben Smith
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "emulator.h"

/* Abbreviations of commonly accessed values. */

#define APU state_.apu
#define CHANNEL1 CHANNEL(1)
#define CHANNEL2 CHANNEL(2)
#define CHANNEL3 CHANNEL(3)
#define CHANNEL4 CHANNEL(4)
#define CHANNEL(i) APU.channel[APU_CHANNEL##i]
#define CPU_SPEED state_.cpu_speed
#define TICKS state_.ticks
#define DMA state_.dma
#define EXT_RAM state_.ext_ram
#define HRAM state_.hram
#define HDMA state_.hdma
#define INFRARED state_.infrared
#define INTR state_.interrupt
#define IS_CGB state_.is_cgb
#define IS_SGB state_.is_sgb
#define JOYP state_.joyp
#define SGB state_.sgb
#define LCDC PPU.lcdc
#define MMAP_STATE state_.memory_map_state
#define NOISE APU.noise
#define OAM state_.oam
#define PPU state_.ppu
#define REG state_.reg
#define SERIAL state_.serial
#define STAT PPU.stat
#define SWEEP APU.sweep
#define TIMER state_.timer
#define VRAM state_.vram
#define WAVE APU.wave
#define WRAM state_.wram


#define DIV_CEIL(numer, denom) (((numer) + (denom) - 1) / (denom))
#define VALUE_WRAPPED(X, MAX) \
  (UNLIKELY((X) >= (MAX) ? ((X) -= (MAX), true) : false))

#define SAVE_STATE_VERSION (2)
#define SAVE_STATE_HEADER (u32)(0x6b57a7e0 + SAVE_STATE_VERSION)

#ifndef HOOK0
#define HOOK0(name)
#define HOOK0(name)
#endif

#ifndef HOOK
#define HOOK(name, ...)
#define HOOK(name, ...)
#endif

#ifndef HOOK0_false
#define HOOK0_false(name) false
#define HOOK0_false(name) false
#endif

/* ROM header stuff */
#define LOGO_START_ADDR 0x104
#define LOGO_END_ADDR 0x133
#define TITLE_START_ADDR 0x134
#define TITLE_MAX_LENGTH 0x10
#define CGB_FLAG_ADDR 0x143
#define SGB_FLAG_ADDR 0x146
#define CART_TYPE_ADDR 0x147
#define ROM_SIZE_ADDR 0x148
#define EXT_RAM_SIZE_ADDR 0x149
#define HEADER_CHECKSUM_ADDR 0x14d
#define GLOBAL_CHECKSUM_START_ADDR 0x14e
#define HEADER_CHECKSUM_RANGE_START 0x134
#define HEADER_CHECKSUM_RANGE_END 0x14c

/* Memory map */
#define ADDR_MASK_4K 0x0fff
#define ADDR_MASK_8K 0x1fff
#define ADDR_MASK_16K 0x3fff

#define MBC_RAM_ENABLED_MASK 0xf
#define MBC_RAM_ENABLED_VALUE 0xa
#define MBC1_ROM_BANK_LO_SELECT_MASK 0x1f
#define MBC1_BANK_HI_SELECT_MASK 0x3
#define MBC1_BANK_HI_SHIFT 5
#define MBC1M_ROM_BANK_LO_SELECT_MASK 0xf
#define MBC1M_BANK_HI_SHIFT 4
/* MBC2 has built-in RAM, 512 4-bit values. It's not external, but it maps to
 * the same address space. */
#define MBC2_RAM_SIZE 0x200
#define MBC2_RAM_ADDR_MASK 0x1ff
#define MBC2_RAM_VALUE_MASK 0xf
#define MBC2_ADDR_SELECT_BIT_MASK 0x100
#define MBC2_ROM_BANK_SELECT_MASK 0xf
#define MBC3_ROM_BANK_SELECT_MASK 0x7f
#define MBC3_RAM_BANK_SELECT_MASK 0x7
#define MBC5_RAM_BANK_SELECT_MASK 0xf
#define HUC1_ROM_BANK_LO_SELECT_MASK 0x3f
#define HUC1_BANK_HI_SELECT_MASK 0x3
#define HUC1_BANK_HI_SHIFT 6

#define OAM_START_ADDR 0xfe00
#define OAM_END_ADDR 0xfe9f
#define IO_START_ADDR 0xff00
#define APU_START_ADDR 0xff10
#define WAVE_RAM_START_ADDR 0xff30
#define HIGH_RAM_START_ADDR 0xff80
#define IE_ADDR 0xffff

#define OAM_TRANSFER_SIZE (OAM_END_ADDR - OAM_START_ADDR + 1)

#define CART_INFO_SHIFT 15
#define ROM_BANK_SHIFT 14
#define EXT_RAM_BANK_SHIFT 13

/* Tick counts */
#define CPU_TICK 4
#define CPU_2X_TICK 2
#define APU_TICKS 2
#define PPU_ENABLE_DISPLAY_DELAY_FRAMES 4
#define PPU_MODE2_TICKS 80
#define PPU_MODE3_MIN_TICKS 172
#define DMA_TICKS 648
#define DMA_DELAY_TICKS 8
#define SERIAL_TICKS (CPU_TICKS_PER_SECOND / 8192)
#define JOYP_INTERRUPT_WAIT_TICKS 10000 /* Arbitrary. */

/* Video */
#define TILE_WIDTH 8
#define TILE_HEIGHT 8
#define TILE_ROW_BYTES 2
#define TILE_MAP_WIDTH 32
#define WINDOW_MAX_X 166
#define WINDOW_X_OFFSET 7

/* Audio */
#define NRX1_MAX_LENGTH 64
#define NR31_MAX_LENGTH 256
#define SWEEP_MAX_PERIOD 8
#define SOUND_MAX_FREQUENCY 2047
#define WAVE_SAMPLE_COUNT 32
#define NOISE_MAX_CLOCK_SHIFT 13
#define NOISE_DIVISOR_COUNT 8
#define ENVELOPE_MAX_PERIOD 8
#define ENVELOPE_MAX_VOLUME 15
#define DUTY_CYCLE_COUNT 8
#define SOUND_OUTPUT_MAX_VOLUME 7

/* Additional samples so the AudioBuffer doesn't overflow. This could happen
 * because the audio buffer is updated at the granularity of an instruction, so
 * the most extra frames that could be added is equal to the Apu tick count
 * of the slowest instruction. */
#define AUDIO_BUFFER_EXTRA_FRAMES 256

#define WAVE_TRIGGER_CORRUPTION_OFFSET_TICKS APU_TICKS
#define WAVE_TRIGGER_DELAY_TICKS (3 * APU_TICKS)

#define FRAME_SEQUENCER_COUNT 8
#define FRAME_SEQUENCER_TICKS 8192 /* 512Hz */
#define FRAME_SEQUENCER_UPDATE_ENVELOPE_FRAME 7

#define INVALID_READ_BYTE 0xff

static auto get_lo = []([[maybe_unused]] auto hi, auto lo) {
  return lo;
};

static auto get_bitmask = [](auto hi, auto lo) {
  return (1 << (hi-lo+1)) - 1;
};

template<typename Value, typename Functor>
static auto unpack(Value&& v, Functor&& f) {
  return (((v) >> f(get_lo)) & f(get_bitmask));
}

template<typename Value, typename Functor>
static auto pack(Value&& v, Functor&& f) {
  return (((v) & f(get_bitmask)) << f(get_lo));
}

template<typename Int>
static auto bits(Int hi, Int lo) {
  return [=](auto f) { return f(hi, lo); };
}

template<typename Value>
static auto bit(Value&& v) {
  return [=](auto f) { return f(v, v); };
}

static auto CPU_FLAG_Z = bit(7);
static auto CPU_FLAG_N = bit(6);
static auto CPU_FLAG_H = bit(5);
static auto CPU_FLAG_C = bit(4);

#define JOYP_UNUSED 0xc0
#define JOYP_RESULT_MASK 0x0f
static auto JOYP_JOYPAD_SELECT = bits(5, 4);
static auto JOYP_DPAD_DOWN = bit(3);
static auto JOYP_DPAD_UP = bit(2);
static auto JOYP_DPAD_LEFT = bit(1);
static auto JOYP_DPAD_RIGHT = bit(0);
static auto JOYP_BUTTON_START = bit(3);
static auto JOYP_BUTTON_SELECT = bit(2);
static auto JOYP_BUTTON_B = bit(1);
static auto JOYP_BUTTON_A = bit(0);
#define SC_UNUSED 0x7e
static auto SC_TRANSFER_START = bit(7);
static auto SC_SHIFT_CLOCK = bit(0);
#define TAC_UNUSED 0xf8
static auto TAC_TIMER_ON = bit(2);
static auto TAC_CLOCK_SELECT = bits(1, 0);
#define IF_UNUSED 0xe0
#define IF_ALL 0x1f
#define IF_JOYPAD 0x10
#define IF_SERIAL 0x08
#define IF_TIMER 0x04
#define IF_STAT 0x02
#define IF_VBLANK 0x01
static auto LCDC_DISPLAY = bit(7);
static auto LCDC_WINDOW_TILE_MAP_SELECT = bit(6);
static auto LCDC_WINDOW_DISPLAY = bit(5);
static auto LCDC_BG_TILE_DATA_SELECT = bit(4);
static auto LCDC_BG_TILE_MAP_SELECT = bit(3);
static auto LCDC_OBJ_SIZE = bit(2);
static auto LCDC_OBJ_DISPLAY = bit(1);
static auto LCDC_BG_DISPLAY = bit(0);
#define STAT_UNUSED 0x80
static auto STAT_YCOMPARE_INTR = bit(6);
static auto STAT_MODE2_INTR = bit(5);
static auto STAT_VBLANK_INTR = bit(4);
static auto STAT_HBLANK_INTR = bit(3);
static auto STAT_YCOMPARE = bit(2);
static auto STAT_MODE = bits(1, 0);
static auto PALETTE_COLOR3 = bits(7, 6);
static auto PALETTE_COLOR2 = bits(5, 4);
static auto PALETTE_COLOR1 = bits(3, 2);
static auto PALETTE_COLOR0 = bits(1, 0);
#define NR10_UNUSED 0x80
static auto NR10_SWEEP_PERIOD = bits(6, 4);
static auto NR10_SWEEP_DIRECTION = bit(3);
static auto NR10_SWEEP_SHIFT = bits(2, 0);
#define NRX1_UNUSED 0x3f
static auto NRX1_WAVE_DUTY = bits(7, 6);
static auto NRX1_LENGTH = bits(5, 0);
static auto NRX2_INITIAL_VOLUME = bits(7, 4);
static auto NRX2_DAC_ENABLED = bits(7, 3);
static auto NRX2_ENVELOPE_DIRECTION = bit(3);
static auto NRX2_ENVELOPE_PERIOD = bits(2, 0);
#define NRX4_UNUSED 0xbf
static auto NRX4_INITIAL = bit(7);
static auto NRX4_LENGTH_ENABLED = bit(6);
static auto NRX4_FREQUENCY_HI = bits(2, 0);
#define NR30_UNUSED 0x7f
static auto NR30_DAC_ENABLED = bit(7);
#define NR32_UNUSED 0x9f
static auto NR32_SELECT_WAVE_VOLUME = bits(6, 5);
static auto NR43_CLOCK_SHIFT = bits(7, 4);
static auto NR43_LFSR_WIDTH = bit(3);
static auto NR43_DIVISOR = bits(2, 0);
static auto NR50_VIN_SO2 = bit(7);
static auto NR50_SO2_VOLUME = bits(6, 4);
static auto NR50_VIN_SO1 = bit(3);
static auto NR50_SO1_VOLUME = bits(2, 0);
static auto NR51_SOUND4_SO2 = bit(7);
static auto NR51_SOUND3_SO2 = bit(6);
static auto NR51_SOUND2_SO2 = bit(5);
static auto NR51_SOUND1_SO2 = bit(4);
static auto NR51_SOUND4_SO1 = bit(3);
static auto NR51_SOUND3_SO1 = bit(2);
static auto NR51_SOUND2_SO1 = bit(1);
static auto NR51_SOUND1_SO1 = bit(0);
#define NR52_UNUSED 0x70
static auto NR52_ALL_SOUND_ENABLED = bit(7);
static auto NR52_SOUND4_ON = bit(3);
static auto NR52_SOUND3_ON = bit(2);
static auto NR52_SOUND2_ON = bit(1);
static auto NR52_SOUND1_ON = bit(0);

#define KEY1_UNUSED 0x7e
static auto KEY1_CURRENT_SPEED = bit(7);
static auto KEY1_PREPARE_SPEED_SWITCH = bit(0);
#define RP_UNUSED 0x3c
static auto RP_DATA_READ_ENABLE = bits(7, 6);
static auto RP_READ_DATA = bit(1);
static auto RP_WRITE_DATA = bit(0);
#define VBK_UNUSED 0xfe
static auto VBK_VRAM_BANK = bit(0);
static auto HDMA5_TRANSFER_MODE = bit(7);
static auto HDMA5_BLOCKS = bits(6, 0);
#define XCPS_UNUSED 0x40
static auto XCPS_AUTO_INCREMENT = bit(7);
static auto XCPS_INDEX = bits(5, 0);
static auto XCPD_BLUE_INTENSITY = bits(14, 10);
static auto XCPD_GREEN_INTENSITY = bits(9, 5);
static auto XCPD_RED_INTENSITY = bits(4, 0);
#define SVBK_UNUSED 0xf8
static auto SVBK_WRAM_BANK = bits(2, 0);

static auto OBJ_PRIORITY = bit(7);
static auto OBJ_YFLIP = bit(6);
static auto OBJ_XFLIP = bit(5);
static auto OBJ_PALETTE = bit(4);
static auto OBJ_BANK = bit(3);
static auto OBJ_CGB_PALETTE = bits(2, 0);

static auto MBC3_RTC_DAY_CARRY = bit(7);
static auto MBC3_RTC_HALT = bit(6);
static auto MBC3_RTC_DAY_HI = bit(0);

static u32 s_rom_bank_count[] = {
#define V(name, code, bank_count) [code] = bank_count,
    FOREACH_ROM_SIZE(V)
#undef V
};
#define ROM_BANK_COUNT(e) s_rom_bank_count[(e)->cart_info_->rom_size]
#define ROM_BANK_MASK(e) (ROM_BANK_COUNT(e) - 1)

static u32 s_ext_ram_byte_size[] = {
#define V(name, code, byte_size) [code] = byte_size,
    FOREACH_EXT_RAM_SIZE(V)
#undef V
};
#define EXT_RAM_BYTE_SIZE(e) s_ext_ram_byte_size[(e)->cart_info_->ext_ram_size]
#define EXT_RAM_BYTE_SIZE_MASK(e) (EXT_RAM_BYTE_SIZE(e) - 1)

static CartTypeInfo s_cart_type_info[] = {
#define V(name, code, mbc, ram, battery, timer)                         \
  [code] = {MBC_TYPE_##mbc, EXT_RAM_TYPE_##ram, BATTERY_TYPE_##battery, \
            TIMER_TYPE_##timer},
    FOREACH_CART_TYPE(V)
#undef V
};

/* TIMA is incremented when the given bit of DIV_counter changes from 1 to 0. */
static const u16 s_tima_mask[] = {1 << 9, 1 << 3, 1 << 5, 1 << 7};
static u8 s_wave_volume_shift[WAVE_VOLUME_COUNT] = {4, 0, 1, 2};
static u8 s_obj_size_to_height[] = {[OBJ_SIZE_8X8] = 8, [OBJ_SIZE_8X16] = 16};

static MemoryTypeAddressPair map_address(Address addr) {
  switch (addr >> 12) {
    case 0x0: case 0x1: case 0x2: case 0x3:
      return {MEMORY_MAP_ROM0, addr & ADDR_MASK_16K};
    case 0x4: case 0x5: case 0x6: case 0x7:
      return {MEMORY_MAP_ROM1, addr & ADDR_MASK_16K};
    case 0x8: case 0x9:
      return {MEMORY_MAP_VRAM, addr & ADDR_MASK_8K};
    case 0xA: case 0xB:
      return {MEMORY_MAP_EXT_RAM, addr & ADDR_MASK_8K};
    case 0xC: case 0xE: /* mirror of 0xc000..0xcfff */
      return {MEMORY_MAP_WORK_RAM0, addr & ADDR_MASK_4K};
    case 0xD:
      return {MEMORY_MAP_WORK_RAM1, addr & ADDR_MASK_4K};
    default: case 0xF:
      switch ((addr >> 8) & 0xf) {
        default: /* 0xf000 - 0xfdff: mirror of 0xd000-0xddff */
          return {MEMORY_MAP_WORK_RAM1, addr & ADDR_MASK_4K};
        case 0xe:
          if (addr <= OAM_END_ADDR) { /* 0xfe00 - 0xfe9f */
            return {MEMORY_MAP_OAM, addr - OAM_START_ADDR};
          } else { /* 0xfea0 - 0xfeff */
            return {MEMORY_MAP_UNUSED, addr};
          }
          break;
        case 0xf:
          switch ((addr >> 4) & 0xf) {
            case 0: case 4: case 5: case 6: case 7:
              /* 0xff00 - 0xff0f, 0xff40 - 0xff7f */
              return {MEMORY_MAP_IO, addr - IO_START_ADDR};
            case 1: case 2: /* 0xff10 - 0xff2f */
              return {MEMORY_MAP_APU, addr - APU_START_ADDR};
            case 3: /* 0xff30 - 0xff3f */
              return {MEMORY_MAP_WAVE_RAM, addr - WAVE_RAM_START_ADDR};
            case 0xf:
              if (addr == IE_ADDR) {
                return {MEMORY_MAP_IO, addr - IO_START_ADDR};
              }
              /* fallthrough */
            default: /* 0xff80 - 0xfffe */
              return {MEMORY_MAP_HIGH_RAM, addr - HIGH_RAM_START_ADDR};
          }
      }
  }
}

static MemoryTypeAddressPair map_hdma_source_address(Address addr) {
  switch (addr >> 12) {
    case 0x0: case 0x1: case 0x2: case 0x3:
      return {MEMORY_MAP_ROM0, addr & ADDR_MASK_16K};
    case 0x4: case 0x5: case 0x6: case 0x7:
      return {MEMORY_MAP_ROM1, addr & ADDR_MASK_16K};
    case 0x8: case 0x9:
      return {MEMORY_MAP_VRAM, addr & ADDR_MASK_8K};
    default: case 0xA: case 0xB: case 0xE: case 0xF:
      return {MEMORY_MAP_EXT_RAM, addr & ADDR_MASK_8K};
    case 0xC:
      return {MEMORY_MAP_WORK_RAM0, addr & ADDR_MASK_4K};
    case 0xD:
      return {MEMORY_MAP_WORK_RAM1, addr & ADDR_MASK_4K};
  }
}

void Emulator::set_cart_info(u8 index) {
  state_.cart_info_index = index;
  cart_info_ = &cart_infos_[index];
  if (!(cart_info_->data && SUCCESS(init_memory_map()))) {
    UNREACHABLE("Unable to switch cart (%d).\n", index);
  }
}

static Result get_cart_info(FileData* file_data, size_t offset, CartInfo* cart_info) {
  /* Simple checksum on logo data so we don't have to include it here. :) */
  u8* data = file_data->data + offset;
  size_t i;
  u32 logo_checksum = 0;
  for (i = LOGO_START_ADDR; i <= LOGO_END_ADDR; ++i) {
    logo_checksum = (logo_checksum << 1) ^ data[i];
  }
  if(!(logo_checksum == 0xe06c8834)) return ERROR;
  cart_info->offset = offset;
  cart_info->data = data;
  cart_info->rom_size = static_cast<RomSize>(data[ROM_SIZE_ADDR]);
  /* HACK(binji): The mooneye-gb multicart test doesn't set any of the header
   * bits, even though multicart games all seem to. Just force the values in
   * reasonable defaults in that case. */
  if (offset != 0 && !is_rom_size_valid(cart_info->rom_size)) {
    cart_info->rom_size = ROM_SIZE_32K;
    cart_info->cgb_flag = CGB_FLAG_NONE;
    cart_info->sgb_flag = SGB_FLAG_NONE;
    cart_info->cart_type = CART_TYPE_MBC1;
    cart_info->ext_ram_size = EXT_RAM_SIZE_NONE;
  } else {
    if(!(is_rom_size_valid(cart_info->rom_size))) return ERROR; // "Invalid ROM size code: %u\n", cart_info->rom_size);

    cart_info->cgb_flag = static_cast<CgbFlag>(data[CGB_FLAG_ADDR]);
    cart_info->sgb_flag = static_cast<SgbFlag>(data[SGB_FLAG_ADDR]);
    cart_info->cart_type = static_cast<CartType>(data[CART_TYPE_ADDR]);
    if(!(is_cart_type_valid(cart_info->cart_type))) return ERROR; // "Invalid cart type: %u\n", cart_info->cart_type);
    cart_info->ext_ram_size = static_cast<ExtRamSize>(data[EXT_RAM_SIZE_ADDR]);
    if(!(is_ext_ram_size_valid(cart_info->ext_ram_size))) return ERROR; // "Invalid ext ram size: %u\n", cart_info->ext_ram_size);
  }

  u32 rom_byte_size = s_rom_bank_count[cart_info->rom_size] << ROM_BANK_SHIFT;
  cart_info->size = rom_byte_size;
  if(!(file_data->size >= offset + rom_byte_size)) return ERROR;
            // "File size too small (required %ld, got %ld)\n",
            // (long)(offset + rom_byte_size), (long)file_data->size);

  return OK;
}

Result Emulator::get_cart_infos() {
  u32 i;
  for (i = 0; i < MAX_CART_INFOS; ++i) {
    size_t offset = i << CART_INFO_SHIFT;
    if (offset + MINIMUM_ROM_SIZE > file_data_.size) break;
    if (SUCCESS(get_cart_info(&file_data_, offset, &cart_infos_[i]))) {
      if (s_cart_type_info[cart_infos_[i].cart_type].mbc_type ==
          MBC_TYPE_MMM01) {
        /* MMM01 has the cart header at the end. */
        set_cart_info(i);
        return OK;
      }
      cart_info_count_++;
    }
  }
  CHECK_MSG(cart_info_count_ != 0, "Invalid ROM.\n");
  set_cart_info(0);
  return OK;
  ON_ERROR_RETURN;
}

void Emulator::dummy_write([[maybe_unused]] MaskedAddress addr, [[maybe_unused]] u8 value) {}

u8 Emulator::dummy_read([[maybe_unused]] MaskedAddress addr) {
  return INVALID_READ_BYTE;
}

void Emulator::set_rom_bank(int index, u16 bank) {
  u32 new_base = (bank & ROM_BANK_MASK(this)) << ROM_BANK_SHIFT;
  u32* base = &MMAP_STATE.rom_base[index];
  if (new_base != *base) {
    HOOK(set_rom_bank_ihi, index, bank, new_base);
  }
  *base = new_base;
}

void Emulator::set_ext_ram_bank(u8 bank) {
  u32 new_base = (bank << EXT_RAM_BANK_SHIFT) & EXT_RAM_BYTE_SIZE_MASK(this);
  u32* base = &MMAP_STATE.ext_ram_base;
  if (new_base != *base) {
    HOOK(set_ext_ram_bank_bi, bank, new_base);
  }
  *base = new_base;
}

u8 Emulator::gb_read_ext_ram(MaskedAddress addr) {
  if (MMAP_STATE.ext_ram_enabled) {
    assert(addr <= ADDR_MASK_8K);
    return EXT_RAM.data[MMAP_STATE.ext_ram_base | addr];
  } else {
    HOOK(read_ram_disabled_a, addr);
    return INVALID_READ_BYTE;
  }
}

void Emulator::gb_write_ext_ram(MaskedAddress addr, u8 value) {
  if (MMAP_STATE.ext_ram_enabled) {
    assert(addr <= ADDR_MASK_8K);
    EXT_RAM.data[MMAP_STATE.ext_ram_base | addr] = value;
    state_.ext_ram_updated = true;
  } else {
    HOOK(write_ram_disabled_ab, addr, value);
  }
}

void Emulator::mbc1_write_rom_shared(u16 bank_lo_mask,
                                  int bank_hi_shift, MaskedAddress addr,
                                  u8 value) {
  Mbc1* mbc1 = &MMAP_STATE.mbc1;
  switch (addr >> 13) {
    case 0: /* 0000-1fff */
      MMAP_STATE.ext_ram_enabled =
          (value & MBC_RAM_ENABLED_MASK) == MBC_RAM_ENABLED_VALUE;
      break;
    case 1: /* 2000-3fff */
      mbc1->byte_2000_3fff = value & MBC1_ROM_BANK_LO_SELECT_MASK;
      break;
    case 2: /* 4000-5fff */
      mbc1->byte_4000_5fff = value & MBC1_BANK_HI_SELECT_MASK;
      break;
    case 3: /* 6000-7fff */
      mbc1->bank_mode = (BankMode)(value & 1);
      break;
  }

  u16 hi_bank = mbc1->byte_4000_5fff << bank_hi_shift;

  u16 rom1_bank = mbc1->byte_2000_3fff;
  if (rom1_bank == 0) {
    rom1_bank++;
  }
  rom1_bank = (rom1_bank & bank_lo_mask) | hi_bank;

  u16 rom0_bank = 0;
  u8 ext_ram_bank = 0;
  if (mbc1->bank_mode == BANK_MODE_RAM) {
    rom0_bank |= hi_bank;
    ext_ram_bank = mbc1->byte_4000_5fff;
  }

  set_rom_bank(0, rom0_bank);
  set_rom_bank(1, rom1_bank);
  set_ext_ram_bank(ext_ram_bank);
}

void Emulator::mbc1_write_rom(MaskedAddress addr, u8 value) {
  mbc1_write_rom_shared(MBC1_ROM_BANK_LO_SELECT_MASK, MBC1_BANK_HI_SHIFT,
                        addr, value);
}

void Emulator::mbc1m_write_rom(MaskedAddress addr, u8 value) {
  mbc1_write_rom_shared(MBC1M_ROM_BANK_LO_SELECT_MASK, MBC1M_BANK_HI_SHIFT,
                        addr, value);
}

void Emulator::mbc2_write_rom(MaskedAddress addr, u8 value) {
  if (addr < 0x4000) {
    if ((addr & MBC2_ADDR_SELECT_BIT_MASK) != 0) {
      u16 rom1_bank = value & MBC2_ROM_BANK_SELECT_MASK & ROM_BANK_MASK(this);
      if (rom1_bank == 0) {
        rom1_bank++;
      }
      set_rom_bank(1, rom1_bank);
    } else {
      MMAP_STATE.ext_ram_enabled =
          (value & MBC_RAM_ENABLED_MASK) == MBC_RAM_ENABLED_VALUE;
    }
  }
}

u8 Emulator::mbc2_read_ram(MaskedAddress addr) {
  if (MMAP_STATE.ext_ram_enabled) {
    return EXT_RAM.data[addr & MBC2_RAM_ADDR_MASK];
  } else {
    HOOK(read_ram_disabled_a, addr);
    return INVALID_READ_BYTE;
  }
}

void Emulator::mbc2_write_ram(MaskedAddress addr, u8 value) {
  if (MMAP_STATE.ext_ram_enabled) {
    EXT_RAM.data[addr & MBC2_RAM_ADDR_MASK] = value & MBC2_RAM_VALUE_MASK;
  } else {
    HOOK(write_ram_disabled_ab, addr, value);
  }
}

void Emulator::mbc3_write_rom(MaskedAddress addr, u8 value) {
  switch (addr >> 13) {
    case 0: /* 0000-1fff */
      MMAP_STATE.ext_ram_enabled =
          (value & MBC_RAM_ENABLED_MASK) == MBC_RAM_ENABLED_VALUE;
      break;
    case 1: { /* 2000-3fff */
      u16 rom1_bank = value & MBC3_ROM_BANK_SELECT_MASK & ROM_BANK_MASK(this);
      if (rom1_bank == 0) {
        rom1_bank++;
      }
      set_rom_bank(1, rom1_bank);
      break;
    }
    case 2: /* 4000-5fff */
      MMAP_STATE.mbc3.rtc_reg = value;
      if (value < 8) {
        set_ext_ram_bank(value & MBC3_RAM_BANK_SELECT_MASK);
      }
      break;
    case 3: { /* 6000-7fff */
      Mbc3* mbc3 = &MMAP_STATE.mbc3;
      bool was_latched = mbc3->latched;
      bool latched = value == 1;
      if (!was_latched && latched && !mbc3->rtc_halt) {
        // Update the clock by how much time has passed since it was last
        // latched.
        Ticks delta = TICKS - mbc3->latch_ticks;
        // RTC ticks every second, so don't update unless at least a second
        // has passed.
        if (delta >= CPU_TICKS_PER_SECOND) {
          u32 ms, sec, min, hour, day;
          emulator_ticks_to_time(delta, &day, &hour, &min, &sec, &ms);

          bool secovf = false;
          if (mbc3->sec >= 60) {
            mbc3->sec += sec;
            if (mbc3->sec >= 64) {
              mbc3->sec -= 64;
              if (mbc3->sec >= 60) { mbc3->sec -= 60; ++min; secovf = true; }
            }
          } else {
            mbc3->sec += sec;
            if (mbc3->sec >= 60) { mbc3->sec -= 60; ++min; secovf = true; }
          }

          bool minovf = false;
          if (min > 0 || secovf) {
            if (mbc3->min >= 60) {
              mbc3->min += min;
              if (mbc3->min >= 64) {
                mbc3->min -= 64;
                if (mbc3->min >= 60) { mbc3->min -= 60; ++hour; minovf = true; }
              }
            } else {
              mbc3->min += min;
              if (mbc3->min >= 60) { mbc3->min -= 60; ++hour; minovf = true; }
            }
          }

          bool hourovf = false;
          if (hour > 0 || minovf) {
            if (mbc3->hour >= 24) {
              mbc3->hour += hour;
              if (mbc3->hour >= 32) {
                mbc3->hour -= 32;
                if (mbc3->hour >= 24) { mbc3->hour -= 24; ++day; hourovf = true; }
              }
            } else {
              mbc3->hour += hour;
              if (mbc3->hour >= 24) { mbc3->hour -= 24; ++day; hourovf = true; }
            }
          }

          if (day > 0 || hourovf) {
            mbc3->day += day;
            if (mbc3->day >= 512) {
              mbc3->day_carry = true;
            }
          }

          mbc3->latch_ticks = TICKS;
        }
      }
      mbc3->latched = latched;
      break;
    }
    default:
      break;
  }
}

u8 Emulator::mbc3_read_ext_ram(MaskedAddress addr) {
  if (!MMAP_STATE.ext_ram_enabled) {
    return INVALID_READ_BYTE;
  }

  Mbc3* mbc3 = &MMAP_STATE.mbc3;
  if (mbc3->rtc_reg <= 3) {
    return gb_read_ext_ram(addr);
  }

  if (!mbc3->latched) {
    return INVALID_READ_BYTE;
  }

  u8 result = INVALID_READ_BYTE;
  switch (mbc3->rtc_reg) {
    case 8: result = mbc3->sec; break;
    case 9: result = mbc3->min; break;
    case 10: result = mbc3->hour; break;
    case 11: result = mbc3->day; break;
    case 12:
      result = pack(mbc3->day_carry, MBC3_RTC_DAY_CARRY) |
               pack(mbc3->rtc_halt, MBC3_RTC_HALT) |
               pack((mbc3->day >> 8) & 1, MBC3_RTC_DAY_HI);
      break;
  }

  return result;
}

void Emulator::mbc3_write_ext_ram(MaskedAddress addr, u8 value) {
  if (!MMAP_STATE.ext_ram_enabled) {
    return;
  }

  Mbc3* mbc3 = &MMAP_STATE.mbc3;
  if (mbc3->rtc_reg <= 3) {
    gb_write_ext_ram(addr, value);
    return;
  }

  if (!mbc3->latched) {
    return;
  }

  switch (mbc3->rtc_reg) {
    case 8:
      mbc3->sec = value & 63;
      /* Reset the tick timer. Note that if the RTC timer is halted then
       * latch_ticks is a previously stored delta, not an absolute tick timer.
       * Once the timer is restarted then latch_ticks is an absolute timer
       * again. */
      mbc3->latch_ticks = mbc3->rtc_halt ? 0 : TICKS;
      break;
    case 9: mbc3->min = value & 63; break;
    case 10: mbc3->hour = value & 31; break;
    case 11: mbc3->day = (mbc3->day & 0x100) | value; break;
    case 12: {
      mbc3->day = (unpack(value, MBC3_RTC_DAY_HI) << 8) | (mbc3->day & 0xff);
      mbc3->day_carry = unpack(value, MBC3_RTC_DAY_CARRY);
      bool old_rtc_halt = mbc3->rtc_halt;
      mbc3->rtc_halt = unpack(value, MBC3_RTC_HALT);
      if (mbc3->rtc_halt != old_rtc_halt) {
        // Update the tick timer; if the clock is halted, then store the
        // previous delta before the clock was stopped. If the clock is
        // restarted, then subtract that delta from the current tick timer to
        // "add" in the delta that is not yet accounted for in the RTC
        // registers.
        mbc3->latch_ticks = TICKS - mbc3->latch_ticks;
      }
      break;
    }
    default:
      break;
  }
}

void Emulator::mbc5_write_rom(MaskedAddress addr, u8 value) {
  switch (addr >> 12) {
    case 0: case 1: /* 0000-1fff */
      MMAP_STATE.ext_ram_enabled =
          (value & MBC_RAM_ENABLED_MASK) == MBC_RAM_ENABLED_VALUE;
      break;
    case 2: /* 2000-2fff */
      MMAP_STATE.mbc5.byte_2000_2fff = value;
      break;
    case 3: /* 3000-3fff */
      MMAP_STATE.mbc5.byte_3000_3fff = value;
      break;
    case 4: case 5: /* 4000-5fff */
      set_ext_ram_bank(value & MBC5_RAM_BANK_SELECT_MASK);
      break;
    default:
      break;
  }

  set_rom_bank(1,
               ((MMAP_STATE.mbc5.byte_3000_3fff & 1) << 8) |
                   MMAP_STATE.mbc5.byte_2000_2fff);
}

void Emulator::huc1_write_rom(MaskedAddress addr, u8 value) {
  Huc1* huc1 = &MMAP_STATE.huc1;
  switch (addr >> 13) {
    case 0: /* 0000-1fff */
      MMAP_STATE.ext_ram_enabled =
          (value & MBC_RAM_ENABLED_MASK) == MBC_RAM_ENABLED_VALUE;
      break;
    case 1: /* 2000-3fff */
      huc1->byte_2000_3fff = value;
      break;
    case 2: /* 4000-5fff */
      huc1->byte_4000_5fff = value;
      break;
    case 3: /* 6000-7fff */
      huc1->bank_mode = (BankMode)(value & 1);
      break;
  }

  u16 rom1_bank = huc1->byte_2000_3fff & HUC1_ROM_BANK_LO_SELECT_MASK;
  if (rom1_bank == 0) {
    rom1_bank++;
  }

  u8 ext_ram_bank;
  if (huc1->bank_mode == BANK_MODE_ROM) {
    rom1_bank |= (huc1->byte_4000_5fff & HUC1_BANK_HI_SELECT_MASK)
                 << HUC1_BANK_HI_SHIFT;
    ext_ram_bank = 0;
  } else {
    ext_ram_bank = huc1->byte_4000_5fff & HUC1_BANK_HI_SELECT_MASK;
  }
  set_rom_bank(1, rom1_bank);
  set_ext_ram_bank(ext_ram_bank);
}

void Emulator::mmm01_write_rom(MaskedAddress addr, u8 value) {
  Mmm01* mmm01 = &MMAP_STATE.mmm01;
  switch (addr >> 13) {
    case 0: { /* 0000-1fff */
      /* ROM size should be power-of-two. */
      assert((cart_info_->size & (cart_info_->size - 1)) == 0);
      u32 rom_offset =
          (mmm01->byte_2000_3fff << ROM_BANK_SHIFT) & (cart_info_->size - 1);
      set_cart_info(rom_offset >> CART_INFO_SHIFT);
      break;
    }
    case 1: /* 2000-3fff */
      mmm01->byte_2000_3fff = value;
      break;
  }
}

Result Emulator::init_memory_map() {
  CartTypeInfo* cart_type_info = &s_cart_type_info[cart_info_->cart_type];

  switch (cart_type_info->ext_ram_type) {
    case EXT_RAM_TYPE_WITH_RAM:
      assert(is_ext_ram_size_valid(cart_info_->ext_ram_size));
      memory_map_.read_ext_ram = [=](MaskedAddress addr) -> u8 { return gb_read_ext_ram(addr); };
      memory_map_.write_ext_ram = [=](MaskedAddress addr, u8 value) -> void { return gb_write_ext_ram(addr, value); };
      EXT_RAM.size = EXT_RAM_BYTE_SIZE(this);
      break;
    default:
    case EXT_RAM_TYPE_NO_RAM:
      memory_map_.read_ext_ram = [=](MaskedAddress addr) -> u8 { return dummy_read(addr); };
      memory_map_.write_ext_ram = [=](MaskedAddress addr, u8 value) -> void { return dummy_write(addr, value); };
      EXT_RAM.size = 0;
      break;
  }

  switch (cart_type_info->mbc_type) {
    case MBC_TYPE_NO_MBC:
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return dummy_write(addr, value); };
      break;
    case MBC_TYPE_MBC1: {
      bool is_mbc1m = cart_info_count_ > 1;
      if(is_mbc1m) {
        memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mbc1m_write_rom(addr, value); };
      } else {
        memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mbc1_write_rom(addr, value); };
      }
      break;
    }
    case MBC_TYPE_MBC2:
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mbc2_write_rom(addr, value); };
      memory_map_.read_ext_ram = [=](MaskedAddress addr) -> u8 { return mbc2_read_ram(addr); };
      memory_map_.write_ext_ram = [=](MaskedAddress addr, u8 value) -> void { return mbc2_write_ram(addr, value); };
      EXT_RAM.size = MBC2_RAM_SIZE;
      break;
    case MBC_TYPE_MMM01:
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mmm01_write_rom(addr, value); };
      break;
    case MBC_TYPE_MBC3: {
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mbc3_write_rom(addr, value); };
      if (cart_type_info->timer_type == TIMER_TYPE_WITH_TIMER) {
        memory_map_.read_ext_ram = [=](MaskedAddress addr) -> u8 { return mbc3_read_ext_ram(addr); };
        memory_map_.write_ext_ram = [=](MaskedAddress addr, u8 value) -> void { return mbc3_write_ext_ram(addr, value); };
      }
      break;
    }
    case MBC_TYPE_MBC5:
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return mbc5_write_rom(addr, value); };
      MMAP_STATE.mbc5.byte_2000_2fff = 1;
      break;
    case MBC_TYPE_HUC1:
      memory_map_.write_rom = [=](MaskedAddress addr, u8 value) -> void { return huc1_write_rom(addr, value); };
      break;
    default:
      PRINT_ERROR("memory map for %s not implemented.\n",
                  get_cart_type_string(cart_info_->cart_type));
      return ERROR;
  }

  EXT_RAM.battery_type = cart_type_info->battery_type;
  return OK;
}

bool Emulator::is_almost_mode3() {
  return PPU.state_ticks == CPU_TICK && STAT.mode == PPU_MODE_MODE2;
}

bool Emulator::is_using_vram(bool write) {
  if (write) {
    return STAT.mode == PPU_MODE_MODE3;
  } else {
    return STAT.mode == PPU_MODE_MODE3 || is_almost_mode3();
  }
}

bool Emulator::is_using_oam(bool write) {
  if (write) {
    return (STAT.mode == PPU_MODE_MODE2 && !is_almost_mode3()) ||
           STAT.mode == PPU_MODE_MODE3;
  } else {
    return STAT.mode2.trigger || STAT.mode == PPU_MODE_MODE2 ||
           STAT.mode == PPU_MODE_MODE3;
  }
}

u8 Emulator::read_vram(MaskedAddress addr) {
  ppu_synchronize();
  if (is_using_vram(false)) {
    HOOK(read_vram_in_use_a, addr);
    return INVALID_READ_BYTE;
  } else {
    assert(addr <= ADDR_MASK_8K);
    return VRAM.data[VRAM.offset + addr];
  }
}

u8 Emulator::read_oam(MaskedAddress addr) {
  ppu_synchronize();
  if (is_using_oam(false)) {
    HOOK(read_oam_in_use_a, addr);
    return INVALID_READ_BYTE;
  }

  u8 obj_index = addr >> 2;
  Obj* obj = &OAM[obj_index];
  switch (addr & 3) {
    case 0: return obj->y + OBJ_Y_OFFSET;
    case 1: return obj->x + OBJ_X_OFFSET;
    case 2: return obj->tile;
    case 3: return obj->byte3;
  }
  UNREACHABLE("invalid OAM address: 0x%04x\n", addr);
}

u8 Emulator::read_joyp_p10_p13() {
  if (JOYP.joypad_select == JOYPAD_SELECT_NONE) {
    return ~(SGB.current_player & 3);
  }
  if (SGB.current_player != 0) { return ~0; }  // Ignore other controllers.

  u8 result = 0;
  if (JOYP.joypad_select == JOYPAD_SELECT_BUTTONS ||
      JOYP.joypad_select == JOYPAD_SELECT_BOTH) {
    result |= pack(JOYP.buttons.start, JOYP_BUTTON_START) |
              pack(JOYP.buttons.select, JOYP_BUTTON_SELECT) |
              pack(JOYP.buttons.B, JOYP_BUTTON_B) |
              pack(JOYP.buttons.A, JOYP_BUTTON_A);
  }

  bool left = JOYP.buttons.left;
  bool right = JOYP.buttons.right;
  bool up = JOYP.buttons.up;
  bool down = JOYP.buttons.down;
  if (!config_.allow_simulataneous_dpad_opposites) {
    if (left && right) {
      left = false;
    } else if (up && down) {
      up = false;
    }
  }

  if (JOYP.joypad_select == JOYPAD_SELECT_DPAD ||
      JOYP.joypad_select == JOYPAD_SELECT_BOTH) {
    result |= pack(down, JOYP_DPAD_DOWN) | pack(up, JOYP_DPAD_UP) |
              pack(left, JOYP_DPAD_LEFT) | pack(right, JOYP_DPAD_RIGHT);
  }
  /* The bits are low when the buttons are pressed. */
  return ~result;
}

void Emulator::call_joyp_callback(bool wait) {
  if (joypad_info_.callback &&
      (!wait || TICKS - JOYP.last_callback >= JOYP_INTERRUPT_WAIT_TICKS)) {
    joypad_info_.callback(&JOYP.buttons, joypad_info_.user_data);
    JOYP.last_callback = TICKS;
  }
}

u8 Emulator::read_io(MaskedAddress addr) {
  switch (addr) {
    case IO_JOYP_ADDR:
      call_joyp_callback(false);
      return JOYP_UNUSED | pack(JOYP.joypad_select, JOYP_JOYPAD_SELECT) |
             (read_joyp_p10_p13() & JOYP_RESULT_MASK);
    case IO_SB_ADDR:
      serial_synchronize();
      return SERIAL.sb;
    case IO_SC_ADDR:
      serial_synchronize();
      return SC_UNUSED | pack(SERIAL.transferring, SC_TRANSFER_START) |
             pack(SERIAL.clock, SC_SHIFT_CLOCK);
    case IO_DIV_ADDR:
      timer_synchronize();
      return TIMER.div_counter >> 8;
    case IO_TIMA_ADDR:
      timer_synchronize();
      return TIMER.tima;
    case IO_TMA_ADDR:
      timer_synchronize();
      return TIMER.tma;
    case IO_TAC_ADDR:
      return TAC_UNUSED | pack(TIMER.on, TAC_TIMER_ON) |
             pack(TIMER.clock_select, TAC_CLOCK_SELECT);
    case IO_IF_ADDR:
      intr_synchronize();
      return IF_UNUSED | INTR.if_;
    case IO_LCDC_ADDR:
      return pack(LCDC.display, LCDC_DISPLAY) |
             pack(LCDC.window_tile_map_select,
                  LCDC_WINDOW_TILE_MAP_SELECT) |
             pack(LCDC.window_display, LCDC_WINDOW_DISPLAY) |
             pack(LCDC.bg_tile_data_select, LCDC_BG_TILE_DATA_SELECT) |
             pack(LCDC.bg_tile_map_select, LCDC_BG_TILE_MAP_SELECT) |
             pack(LCDC.obj_size, LCDC_OBJ_SIZE) |
             pack(LCDC.obj_display, LCDC_OBJ_DISPLAY) |
             pack(LCDC.bg_display, LCDC_BG_DISPLAY);
    case IO_STAT_ADDR:
      ppu_synchronize();
      return STAT_UNUSED | pack(STAT.y_compare.irq, STAT_YCOMPARE_INTR) |
             pack(STAT.mode2.irq, STAT_MODE2_INTR) |
             pack(STAT.vblank.irq, STAT_VBLANK_INTR) |
             pack(STAT.hblank.irq, STAT_HBLANK_INTR) |
             pack(STAT.ly_eq_lyc, STAT_YCOMPARE) |
             pack(STAT.mode, STAT_MODE);
    case IO_SCY_ADDR:
      return PPU.scy;
    case IO_SCX_ADDR:
      return PPU.scx;
    case IO_LY_ADDR:
      ppu_synchronize();
      return PPU.ly;
    case IO_LYC_ADDR:
      return PPU.lyc;
    case IO_DMA_ADDR:
      return INVALID_READ_BYTE; /* Write only. */
    case IO_BGP_ADDR:
    case IO_OBP0_ADDR:
    case IO_OBP1_ADDR: {
      Palette* pal = &PPU.pal[addr - IO_BGP_ADDR];
      return pack(pal->color[3], PALETTE_COLOR3) |
             pack(pal->color[2], PALETTE_COLOR2) |
             pack(pal->color[1], PALETTE_COLOR1) |
             pack(pal->color[0], PALETTE_COLOR0);
    }
    case IO_WY_ADDR:
      return PPU.wy;
    case IO_WX_ADDR:
      return PPU.wx;
    case IO_KEY1_ADDR:
      return IS_CGB ? (KEY1_UNUSED | pack(CPU_SPEED.speed, KEY1_CURRENT_SPEED) |
                       pack(CPU_SPEED.switching, KEY1_PREPARE_SPEED_SWITCH))
                    : INVALID_READ_BYTE;
    case IO_VBK_ADDR:
      return IS_CGB ? (VBK_UNUSED | pack(VRAM.bank, VBK_VRAM_BANK))
                    : INVALID_READ_BYTE;
    case IO_HDMA5_ADDR:
      return IS_CGB ? HDMA.blocks : INVALID_READ_BYTE;
    case IO_RP_ADDR:
      return IS_CGB ? (RP_UNUSED | pack(INFRARED.enabled, RP_DATA_READ_ENABLE) |
                       pack(INFRARED.read, RP_READ_DATA) |
                       pack(INFRARED.write, RP_WRITE_DATA))
                    : INVALID_READ_BYTE;
    case IO_BCPS_ADDR:
    case IO_OCPS_ADDR:
      if (IS_CGB) {
        ColorPalettes* cp = addr == IO_BCPS_ADDR ? &PPU.bgcp : &PPU.obcp;
        return XCPS_UNUSED | pack(cp->index, XCPS_INDEX) |
               pack(cp->auto_increment, XCPS_AUTO_INCREMENT);
      } else {
        return INVALID_READ_BYTE;
      }
    case IO_BCPD_ADDR:
    case IO_OCPD_ADDR:
      if (IS_CGB) {
        ColorPalettes* cp = addr == IO_BCPD_ADDR ? &PPU.bgcp : &PPU.obcp;
        return cp->data[cp->index];
      } else {
        return INVALID_READ_BYTE;
      }
    case IO_SVBK_ADDR:
      return IS_CGB ? (SVBK_UNUSED | pack(WRAM.bank, SVBK_WRAM_BANK))
                    : INVALID_READ_BYTE;
    case IO_IE_ADDR:
      return INTR.ie;
    default:
      HOOK(read_io_ignored_as, addr, get_io_reg_string(static_cast<IOReg>(addr)));
      return INVALID_READ_BYTE;
  }
}

static u8 read_nrx1_reg(Channel* channel) {
  return pack(channel->square_wave.duty, NRX1_WAVE_DUTY);
}

static u8 read_nrx2_reg(Channel* channel) {
  return pack(channel->envelope.initial_volume, NRX2_INITIAL_VOLUME) |
         pack(channel->envelope.direction, NRX2_ENVELOPE_DIRECTION) |
         pack(channel->envelope.period, NRX2_ENVELOPE_PERIOD);
}

static u8 read_nrx4_reg(Channel* channel) {
  return pack(channel->length_enabled, NRX4_LENGTH_ENABLED);
}

u8 Emulator::read_apu(MaskedAddress addr) {
  apu_synchronize();
  switch (addr) {
    case APU_NR10_ADDR:
      return NR10_UNUSED | pack(SWEEP.period, NR10_SWEEP_PERIOD) |
             pack(SWEEP.direction, NR10_SWEEP_DIRECTION) |
             pack(SWEEP.shift, NR10_SWEEP_SHIFT);
    case APU_NR11_ADDR:
      return NRX1_UNUSED | read_nrx1_reg(&CHANNEL1);
    case APU_NR12_ADDR:
      return read_nrx2_reg(&CHANNEL1);
    case APU_NR14_ADDR:
      return NRX4_UNUSED | read_nrx4_reg(&CHANNEL1);
    case APU_NR21_ADDR:
      return NRX1_UNUSED | read_nrx1_reg(&CHANNEL2);
    case APU_NR22_ADDR:
      return read_nrx2_reg(&CHANNEL2);
    case APU_NR24_ADDR:
      return NRX4_UNUSED | read_nrx4_reg(&CHANNEL2);
    case APU_NR30_ADDR:
      return NR30_UNUSED |
             pack(CHANNEL3.dac_enabled, NR30_DAC_ENABLED);
    case APU_NR32_ADDR:
      return NR32_UNUSED | pack(WAVE.volume, NR32_SELECT_WAVE_VOLUME);
    case APU_NR34_ADDR:
      return NRX4_UNUSED | read_nrx4_reg(&CHANNEL3);
    case APU_NR42_ADDR:
      return read_nrx2_reg(&CHANNEL4);
    case APU_NR43_ADDR:
      return pack(NOISE.clock_shift, NR43_CLOCK_SHIFT) |
             pack(NOISE.lfsr_width, NR43_LFSR_WIDTH) |
             pack(NOISE.divisor, NR43_DIVISOR);
    case APU_NR44_ADDR:
      return NRX4_UNUSED | read_nrx4_reg(&CHANNEL4);
    case APU_NR50_ADDR:
      return pack(APU.so_output[VIN][1], NR50_VIN_SO2) |
             pack(APU.so_volume[1], NR50_SO2_VOLUME) |
             pack(APU.so_output[VIN][0], NR50_VIN_SO1) |
             pack(APU.so_volume[0], NR50_SO1_VOLUME);
    case APU_NR51_ADDR:
      return pack(APU.so_output[SOUND4][1], NR51_SOUND4_SO2) |
             pack(APU.so_output[SOUND3][1], NR51_SOUND3_SO2) |
             pack(APU.so_output[SOUND2][1], NR51_SOUND2_SO2) |
             pack(APU.so_output[SOUND1][1], NR51_SOUND1_SO2) |
             pack(APU.so_output[SOUND4][0], NR51_SOUND4_SO1) |
             pack(APU.so_output[SOUND3][0], NR51_SOUND3_SO1) |
             pack(APU.so_output[SOUND2][0], NR51_SOUND2_SO1) |
             pack(APU.so_output[SOUND1][0], NR51_SOUND1_SO1);
    case APU_NR52_ADDR:
      return NR52_UNUSED | pack(APU.enabled, NR52_ALL_SOUND_ENABLED) |
             pack(CHANNEL4.status, NR52_SOUND4_ON) |
             pack(CHANNEL3.status, NR52_SOUND3_ON) |
             pack(CHANNEL2.status, NR52_SOUND2_ON) |
             pack(CHANNEL1.status, NR52_SOUND1_ON);
    default:
      return INVALID_READ_BYTE;
  }
}

u8 Emulator::read_wave_ram(MaskedAddress addr) {
  apu_synchronize();
  if (CHANNEL3.status) {
    /* If the wave channel is playing, the byte is read from the sample
     * position. On DMG, this is only allowed if the read occurs exactly when
     * it is being accessed by the Wave channel. */
    u8 result;
    if (IS_CGB || TICKS == WAVE.sample_time) {
      result = WAVE.ram[WAVE.position >> 1];
      HOOK(read_wave_ram_while_playing_ab, addr, result);
    } else {
      result = INVALID_READ_BYTE;
      HOOK(read_wave_ram_while_playing_invalid_a, addr);
    }
    return result;
  } else {
    return WAVE.ram[addr];
  }
}

bool Emulator::is_dma_access_ok(Address addr) {
  /* TODO: need to figure out bus conflicts during DMA for non-OAM accesses. */
  return DMA.state != DMA_ACTIVE || (addr & 0xff00) != 0xfe00;
}

u8 Emulator::read_u8_pair(MemoryTypeAddressPair pair, bool raw) {
  switch (pair.type) {
    /* Take advantage of the fact that MEMORY_MAP_ROM9 is 0, and ROM1 is 1 when
     * indexing into rom_base. */
    case MEMORY_MAP_ROM0:
    case MEMORY_MAP_ROM1: {
      u32 rom_addr = MMAP_STATE.rom_base[pair.type] | pair.addr;
      assert(rom_addr < cart_info_->size);
      u8 value = cart_info_->data[rom_addr];
      if (!raw) {
        HOOK(read_rom_ib, rom_addr, value);
      }
      return value;
    }
    case MEMORY_MAP_VRAM:
      return read_vram(pair.addr);
    case MEMORY_MAP_EXT_RAM:
      return memory_map_.read_ext_ram(pair.addr);
    case MEMORY_MAP_WORK_RAM0:
      return WRAM.data[pair.addr];
    case MEMORY_MAP_WORK_RAM1:
      return WRAM.data[WRAM.offset + pair.addr];
    case MEMORY_MAP_OAM:
      return read_oam(pair.addr);
    case MEMORY_MAP_UNUSED:
      return INVALID_READ_BYTE;
    case MEMORY_MAP_IO: {
      u8 value = read_io(pair.addr);
      HOOK(read_io_asb, pair.addr, get_io_reg_string(static_cast<IOReg>(pair.addr)), value);
      return value;
    }
    case MEMORY_MAP_APU:
      return read_apu(pair.addr);
    case MEMORY_MAP_WAVE_RAM:
      return read_wave_ram(pair.addr);
    case MEMORY_MAP_HIGH_RAM:
      return HRAM[pair.addr];
    default:
      UNREACHABLE("invalid address: %u 0x%04x.\n", pair.type, pair.addr);
  }
}

[[maybe_unused]] u8 Emulator::read_u8_raw(Address addr) {
  return read_u8_pair(map_address(addr), true);
}

u8 Emulator::read_u8(Address addr) {
  dma_synchronize();
  if (UNLIKELY(!is_dma_access_ok(addr))) {
    HOOK(read_during_dma_a, addr);
    return INVALID_READ_BYTE;
  }
  if (LIKELY(addr < 0x8000)) {
    u32 bank = addr >> ROM_BANK_SHIFT;
    u32 rom_addr = MMAP_STATE.rom_base[bank] | (addr & ADDR_MASK_16K);
    u8 value = cart_info_->data[rom_addr];
    HOOK(read_rom_ib, rom_addr, value);
    return value;
  } else {
    return read_u8_pair(map_address(addr), false);
  }
}

void Emulator::write_vram(MaskedAddress addr, u8 value) {
  ppu_synchronize();
  if (UNLIKELY(is_using_vram(true))) {
    HOOK(write_vram_in_use_ab, addr, value);
    return;
  }

  assert(addr <= ADDR_MASK_8K);
  VRAM.data[VRAM.offset + addr] = value;
}

void Emulator::write_oam_no_mode_check(MaskedAddress addr, u8 value) {
  Obj* obj = &OAM[addr >> 2];
  switch (addr & 3) {
    case 0: obj->y = value - OBJ_Y_OFFSET; break;
    case 1: obj->x = value - OBJ_X_OFFSET; break;
    case 2: obj->tile = value; break;
    case 3:
      obj->byte3 = value;
      obj->priority = static_cast<ObjPriority>(unpack(value, OBJ_PRIORITY));
      obj->yflip = unpack(value, OBJ_YFLIP);
      obj->xflip = unpack(value, OBJ_XFLIP);
      obj->palette = unpack(value, OBJ_PALETTE);
      obj->bank = unpack(value, OBJ_BANK);
      obj->cgb_palette = unpack(value, OBJ_CGB_PALETTE);
      break;
  }
}

void Emulator::write_oam(MaskedAddress addr, u8 value) {
  ppu_synchronize();
  if (UNLIKELY(is_using_oam(true))) {
    HOOK(write_oam_in_use_ab, addr, value);
    return;
  }

  write_oam_no_mode_check(addr, value);
}

void Emulator::calculate_next_intr() {
  state_.next_intr_ticks = MIN(
      MIN(SERIAL.next_intr_ticks, TIMER.next_intr_ticks), PPU.next_intr_ticks);
}

bool Emulator::is_div_falling_edge(u16 old_div_counter,
                                u16 div_counter) {
  u16 falling_edge = ((old_div_counter ^ div_counter) & ~div_counter);
  return falling_edge & s_tima_mask[TIMER.clock_select];
}

void Emulator::timer_synchronize() {
  if (TICKS > TIMER.sync_ticks) {
    Ticks delta_ticks = TICKS - TIMER.sync_ticks;
    TIMER.sync_ticks = TICKS;

    if (TIMER.on) {
      Ticks cpu_tick = state_.cpu_tick;
      for (; delta_ticks > 0; delta_ticks -= cpu_tick) {
        if (TIMER.tima_state == TIMA_STATE_OVERFLOW) {
          INTR.if_ |= (INTR.new_if & IF_TIMER);
          TIMER.tima = TIMER.tma;
          TIMER.tima_state = TIMA_STATE_RESET;
        } else if (TIMER.tima_state == TIMA_STATE_RESET) {
          TIMER.tima_state = TIMA_STATE_NORMAL;
        }
        u16 old_div_counter = TIMER.div_counter;
        TIMER.div_counter += CPU_TICK;
        if (is_div_falling_edge(old_div_counter, TIMER.div_counter)) {
          increment_tima();
        }
      }
    } else {
      TIMER.div_counter += delta_ticks;
    }
  }
}

void Emulator::calculate_next_timer_intr() {
  if (TIMER.on) {
    Ticks ticks = TIMER.sync_ticks;
    Ticks cpu_tick = state_.cpu_tick;
    u16 div_counter = TIMER.div_counter;
    u8 tima = TIMER.tima;
    if (TIMER.tima_state == TIMA_STATE_OVERFLOW) {
      tima = TIMER.tma;
      div_counter += CPU_TICK;
      ticks += cpu_tick;
    }

    while (1) {
      u16 old_div_counter = div_counter;
      div_counter += CPU_TICK;
      if (is_div_falling_edge(old_div_counter, div_counter) && ++tima == 0) {
        break;
      }
      ticks += cpu_tick;
    }
    TIMER.next_intr_ticks = ticks;
  } else {
    TIMER.next_intr_ticks = INVALID_TICKS;
  }
  calculate_next_intr();
}

void Emulator::do_timer_interrupt() {
  Ticks cpu_tick = state_.cpu_tick;
  HOOK(trigger_timer_i, TICKS + cpu_tick);
  TIMER.tima_state = TIMA_STATE_OVERFLOW;
  TIMER.div_counter += TICKS + CPU_TICK - TIMER.sync_ticks;
  TIMER.sync_ticks = TICKS + cpu_tick;
  TIMER.tima = 0;
  INTR.new_if |= IF_TIMER;
  calculate_next_timer_intr();
}

void Emulator::increment_tima() {
  if (++TIMER.tima == 0) {
    do_timer_interrupt();
  }
}

void Emulator::clear_div() {
  if (TIMER.on && is_div_falling_edge(TIMER.div_counter, 0)) {
    increment_tima();
  }
  TIMER.div_counter = 0;
}

/* Trigger is only true on the tick where it transitioned to the new state;
 * "check" is true as long as at continues to be in that state. This is
 * necessary because the internal STAT IF flag is set when "triggered", and
 * cleared only when the "check" returns false for all STAT IF bits. HBLANK and
 * VBLANK don't have a special trigger, so "trigger" and "check" are equal for
 * those modes. */
#define TRIGGER_MODE_IS(X) (STAT.trigger_mode == PPU_MODE_##X)
#define TRIGGER_HBLANK (TRIGGER_MODE_IS(HBLANK) && STAT.hblank.irq)
#define TRIGGER_VBLANK (TRIGGER_MODE_IS(VBLANK) && STAT.vblank.irq)
#define TRIGGER_MODE2 (STAT.mode2.trigger && STAT.mode2.irq)
#define CHECK_MODE2 (TRIGGER_MODE_IS(MODE2) && STAT.mode2.irq)
#define TRIGGER_Y_COMPARE (STAT.y_compare.trigger && STAT.y_compare.irq)
#define CHECK_Y_COMPARE (STAT.new_ly_eq_lyc && STAT.y_compare.irq)
#define SHOULD_TRIGGER_STAT \
  (TRIGGER_HBLANK || TRIGGER_VBLANK || TRIGGER_MODE2 || TRIGGER_Y_COMPARE)

void Emulator::check_stat() {
  if (!STAT.if_ && SHOULD_TRIGGER_STAT) {
    HOOK(trigger_stat_ii, PPU.ly, TICKS + CPU_TICK);
    INTR.new_if |= IF_STAT;
    if (!(TRIGGER_VBLANK || TRIGGER_Y_COMPARE)) {
      INTR.if_ |= IF_STAT;
    }
    STAT.if_ = true;
  } else if (!(TRIGGER_HBLANK || TRIGGER_VBLANK || CHECK_MODE2 ||
               CHECK_Y_COMPARE)) {
    STAT.if_ = false;
  }
}

void Emulator::check_ly_eq_lyc(bool write) {
  if (PPU.ly == PPU.lyc ||
      (write && PPU.last_ly == SCREEN_HEIGHT_WITH_VBLANK - 1 &&
       PPU.last_ly == PPU.lyc)) {
    HOOK(trigger_y_compare_ii, PPU.ly, TICKS + CPU_TICK);
    STAT.y_compare.trigger = true;
    STAT.new_ly_eq_lyc = true;
  } else {
    STAT.y_compare.trigger = false;
    STAT.ly_eq_lyc = STAT.new_ly_eq_lyc = false;
    if (write) {
      /* If stat was triggered this frame due to Y compare, cancel it.
       * There's probably a nicer way to do this. */
      if ((INTR.new_if ^ INTR.if_) & INTR.new_if & IF_STAT) {
        if (!SHOULD_TRIGGER_STAT) {
          INTR.new_if &= ~IF_STAT;
        }
      }
    }
  }
}

void Emulator::check_joyp_intr() {
  call_joyp_callback(true);
  u8 p10_p13 = read_joyp_p10_p13();
  /* joyp interrupt only triggers on p10-p13 going from high to low (i.e. not
   * pressed to pressed). */
  if ((p10_p13 ^ JOYP.last_p10_p13) & ~p10_p13) {
    INTR.new_if |= IF_JOYPAD;
  }
  JOYP.last_p10_p13 = p10_p13;
}

void Emulator::update_bw_palette_rgba(PaletteType type) {
  for (int i = 0; i < 4; ++i) {
    pal_[type].color[i] =
        color_to_rgba_[type].color[PPU.pal[type].color[i]];
  }
  if (type == PALETTE_TYPE_BGP) {
    for (int pal = 0; pal < 4; ++pal) {
      for (int i = 0; i < 4; ++i) {
        sgb_pal_[pal].color[i] =
            SGB.screen_pal[pal].color[PPU.pal[PALETTE_TYPE_BGP].color[i]];
      }
    }
  }
}

RGBA Emulator::unpack_cgb_color(u16 color) {
  u8 r = unpack(color, XCPD_RED_INTENSITY);
  u8 g = unpack(color, XCPD_GREEN_INTENSITY);
  u8 b = unpack(color, XCPD_BLUE_INTENSITY);

  switch (cgb_color_curve_) {
    default:
    case CGB_COLOR_CURVE_NONE:
      return MAKE_RGBA(r << 3, g << 3, b << 3, 255);

    case CGB_COLOR_CURVE_SAMEBOY_EMULATE_HARDWARE: {
      // Using Sameboy's color curves, see
      // https://github.com/LIJI32/SameBoy/blob/345e51647f2a7ce1ea39f21497f5a6dc75a587c8/Core/display.c#L239
      static const u8 curve[] = {
          0,   6,   12,  20,  28,  36,  45,  56,  66,  76,  88,
          100, 113, 125, 137, 149, 161, 172, 182, 192, 202, 210,
          218, 225, 232, 238, 243, 247, 250, 252, 254, 255,
      };
      r = curve[r];
      g = curve[g];
      b = curve[b];
      g = (g * 3 + b) / 4;
      return MAKE_RGBA(r, g, b, 255);
    }

    case CGB_COLOR_CURVE_GAMBATTE:
      // Using gambatte's color curves, according to Gameboy Online, see
      // https://github.com/taisel/GameBoy-Online/blob/47f9f638a8a9445aaa75050f634e437baa34aae0/js/GameBoyCore.js#L6453
      return MAKE_RGBA((r * 13 + g * 2 + b) >> 1, (g * 3 + b) << 1,
                       (r * 3 + g * 2 + b * 11) >> 1, 255);
  }
}

RGBA Emulator::unpack_cgb_color8(u8 lo, u8 hi) {
  return unpack_cgb_color((hi << 8) | lo);
}

void Emulator::set_sgb_palette(int pal, u8 lo0, u8 hi0, u8 lo1,
                               u8 hi1, u8 lo2, u8 hi2, u8 lo3, u8 hi3) {
  for (int i = 0; i < 4; ++i) {
    SGB.screen_pal[i].color[0] = unpack_cgb_color8(lo0, hi0);
  }
  SGB.screen_pal[pal].color[1] = unpack_cgb_color8(lo1, hi1);
  SGB.screen_pal[pal].color[2] = unpack_cgb_color8(lo2, hi2);
  SGB.screen_pal[pal].color[3] = unpack_cgb_color8(lo3, hi3);
  if (pal == 0) {
    set_bw_palette(PALETTE_TYPE_OBP0, &SGB.screen_pal[0]);
    set_bw_palette(PALETTE_TYPE_OBP1, &SGB.screen_pal[0]);
  }
  update_bw_palette_rgba(PALETTE_TYPE_BGP);
}

void Emulator::unpack_sgb_palette_ram(int pal, u8 idx_lo, u8 idx_hi) {
  u16 idx = (idx_hi << 8) | idx_lo;
  u8* data = SGB.pal_ram + 8 * (idx & 0x1ff);
  set_sgb_palette(pal, data[0], data[1], data[2], data[3], data[4], data[5],
                  data[6], data[7]);
}

void Emulator::clear_frame_buffer(RGBA color) {
  for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i) {
    frame_buffer_[i] = color;
  }
}

void Emulator::update_sgb_mask() {
  RGBA color = RGBA_BLACK;
  bool should_clear = true;
  switch (SGB.mask) {
    case SGB_MASK_CANCEL: should_clear = false; break;
    case SGB_MASK_FREEZE: should_clear = false; break;
    case SGB_MASK_BLACK:  color = RGBA_BLACK; break;
    case SGB_MASK_COLOR0: color = SGB.screen_pal[0].color[0]; break;
  }
  if (should_clear) {
    clear_frame_buffer(color);
  }
}

void Emulator::set_sgb_attr(u8 byte) {
  u8 file = byte & 0x3f;
  if (file < 0x2D) {
    memcpy(SGB.attr_map, SGB.attr_ram + file * 90, sizeof(SGB.attr_map));
  }
  if (byte & 0x40) {
    SGB.mask = SGB_MASK_CANCEL;
    update_sgb_mask();
  }
}

void Emulator::set_sgb_attr_block(int x0, int y0, int x1, int y1, u8 pal) {
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      int index = y * 20 + x;
      u8 *byte = &SGB.attr_map[index >> 2];
      u8 mask = ~(0xc0 >> (2 * (x & 3)));
      *byte = (*byte & mask) | (pal << (2 * (3 - (x & 3))));
    }
  }
}

static u8 reverse_bits_u8(u8 x) {
  x = ((x << 4) & 0xf0) | ((x >> 4) & 0x0f);
  x = ((x << 2) & 0xcc) | ((x >> 2) & 0x33);
  x = ((x << 1) & 0xaa) | ((x >> 1) & 0x55);
  return x;
}

static u16 map_select_to_address(TileMapSelect map_select) {
  return map_select == TILE_MAP_9800_9BFF ? 0x1800 : 0x1c00;
}

void Emulator::do_sgb() {
  if (!IS_SGB) { return; }

  bool do_command = false;

  switch (SGB.state) {
    case SGB_STATE_IDLE:
      if (JOYP.joypad_select == JOYPAD_SGB_BOTH_LOW) {
        SGB.bits_read = 0;
        if (++SGB.current_packet >= SGB.packet_count) {
          SGB.current_packet = 0;
          SGB.packet_count = 0;
          ZERO_MEMORY(SGB.data);
        }
        SGB.state = SGB_STATE_WAIT_BIT;
      }
      break;
    case SGB_STATE_WAIT_BIT:
      if (JOYP.joypad_select == JOYPAD_SGB_BOTH_HIGH) {
        SGB.state =
            SGB.bits_read >= 128 ? SGB_STATE_STOP_BIT : SGB_STATE_READ_BIT;
      } else {
        SGB.state = SGB_STATE_IDLE;
      }
      break;
    case SGB_STATE_READ_BIT:
      if (JOYP.joypad_select == JOYPAD_SGB_P15_LOW) {
        int curbyte = (SGB.current_packet << 4) | (SGB.bits_read >> 3);
        u8 curbit = SGB.bits_read & 7;
        SGB.data[curbyte] |= 1 << curbit;
        SGB.bits_read++;
        SGB.state = SGB_STATE_WAIT_BIT;
      } else if (JOYP.joypad_select == JOYPAD_SGB_P14_LOW) {
        SGB.bits_read++;
        SGB.state = SGB_STATE_WAIT_BIT;
      }
      break;
    case SGB_STATE_STOP_BIT:
      if (JOYP.joypad_select == JOYPAD_SGB_P14_LOW) {
        SGB.state = SGB_STATE_STOP_WAIT;
      } else {
        SGB.state = SGB_STATE_IDLE;
      }
      break;
    case SGB_STATE_STOP_WAIT:
      if (JOYP.joypad_select == JOYPAD_SGB_BOTH_HIGH) {
        do_command = true;
        SGB.state = SGB_STATE_IDLE;
      }
      break;
  }

  if ((JOYP.joypad_select == JOYPAD_SGB_BOTH_LOW ||
       JOYP.joypad_select == JOYPAD_SGB_P15_LOW) &&
      !SGB.player_incremented) {
    SGB.player_incremented = true;
  } else if (JOYP.joypad_select == JOYPAD_SGB_BOTH_HIGH) {
    if (SGB.player_incremented) {
      SGB.current_player = (SGB.current_player + 1) & SGB.player_mask;
    }
    SGB.player_incremented = false;
  }

  if (do_command) {
    if (SGB.current_packet == 0) {
      SGB.packet_count = SGB.data[0] & 7;
    }

    if (SGB.current_packet == SGB.packet_count - 1) {
      // Assume we can just read the data directly from VRAM. Cheat by reading
      // the upper-left tile and assuming that the rest of the data is in
      // order.
      int code = SGB.data[0] >> 3;
      u8* xfer_src = NULL;
      if (code == 0x0b || code == 0x13 || code == 0x14 || code == 0x15) {
        u16 map_base = map_select_to_address(LCDC.bg_tile_map_select);
        u16 tile_index = VRAM.data[map_base];
        if (LCDC.bg_tile_data_select == TILE_DATA_8800_97FF) {
          // Copy the data into the temporary buffer so it can be used
          // contiguously.
          static u8 s_temp_xfer_buffer[4096];
          u16 start_offset = (256 + (s8)tile_index) * 16;
          u16 len = 0x1800 - start_offset;
          memcpy(s_temp_xfer_buffer, VRAM.data + start_offset, len);
          memcpy(s_temp_xfer_buffer + len, VRAM.data + 0x800, 0x1000 - len);
          xfer_src = s_temp_xfer_buffer;
        } else {
          xfer_src = VRAM.data + tile_index * 16;
        }
      }

      switch (code) {
        case 0x00:   // PAL01
        case 0x01:   // PAL23
        case 0x02:   // PAL03
        case 0x03: { // PAL12
          static struct {
            int pal0, pal1;
          } s_pals[] = {{0, 1}, {2, 3}, {0, 3}, {1, 2}};
          set_sgb_palette(s_pals[code].pal0, SGB.data[1], SGB.data[2],
                          SGB.data[3], SGB.data[4], SGB.data[5], SGB.data[6],
                          SGB.data[7], SGB.data[8]);
          set_sgb_palette(s_pals[code].pal1, SGB.data[1], SGB.data[2],
                          SGB.data[9], SGB.data[10], SGB.data[11], SGB.data[12],
                          SGB.data[13], SGB.data[14]);
          break;
        }
        case 0x04: { // ATTR_BLK
          int datasets = MIN(SGB.data[1], (SGB.packet_count * 16 - 2) / 6);
          for (int i = 0; i < datasets; ++i) {
            u8 info = SGB.data[2 + i * 6];
            u8 pal = SGB.data[3 + i * 6];
            u8 palin = pal & 3, palon = (pal >> 2) & 3, palout = (pal >> 4) & 3;
            u8 l = SGB.data[4 + i * 6], t = SGB.data[5 + i * 6],
               r = SGB.data[6 + i * 6], b = SGB.data[7 + i * 6];
            if (info & 1) {  // colors inside region
              set_sgb_attr_block(l, t, r, b, palin);
            }
            if (info & 2) { // colors on region border
              set_sgb_attr_block(l, t, r, t, palon);  // top
              set_sgb_attr_block(l, t, l, b, palon);  // left
              set_sgb_attr_block(l, b, r, b, palon);  // bottom
              set_sgb_attr_block(r, t, r, b, palon);  // right
            }
            if (info & 4) { // colors outside region
              set_sgb_attr_block(0, 0, 19, t - 1, palout);   // top
              set_sgb_attr_block(0, t, l - 1, b, palout);    // left
              set_sgb_attr_block(0, b + 1, 19, 17, palout);  // bottom
              set_sgb_attr_block(r + 1, t, 19, b, palout);   // right
            }
          }
          break;
        }
        case 0x05: { // ATTR_LIN
          int datasets = MIN(SGB.data[1], SGB.packet_count * 16 - 2);
          for (int i = 0; i < datasets; ++i) {
            u8 info = SGB.data[2 + i];
            u8 line = info & 0x1f;
            u8 pal = (info >> 5) & 3;
            if (info & 0x80) {  // horizontal
              set_sgb_attr_block(0, line, 19, line, pal);
            } else { // vertical
              set_sgb_attr_block(line, 0, line, 17, pal);
            }
          }
          break;
        }
        case 0x06: { // ATTR_DIV
          u8 pal = SGB.data[1];
          u8 pallo = pal & 3, palon = (pal >> 2) & 3, palhi = (pal >> 4) & 3;
          u8 line = SGB.data[2];
          if (pal & 0x40) {  // above/below
            set_sgb_attr_block(0, 0, 19, line - 1, palhi);   // top
            set_sgb_attr_block(0, line, 19, line, palon);    // on
            set_sgb_attr_block(0, line + 1, 19, 17, pallo);  // bottom
          } else { // left/right
            set_sgb_attr_block(0, 0, line - 1, 17, palhi);   // left
            set_sgb_attr_block(line, 0, line, 17, palon);    // on
            set_sgb_attr_block(line + 1, 0, 19, 17, pallo);  // right
          }
          break;
        }
        case 0x07: { // ATTR_CHR
          u8 x = SGB.data[1], y = SGB.data[2];
          u8 dx = 0, dy = 0;
          if (SGB.data[5] == 0) { dx = 1; } else { dy = 1; }
          int datasets = MIN(MIN((SGB.data[4] << 8) | SGB.data[3],
                                 (SGB.packet_count * 16 - 6) * 4),
                             360);
          for (int i = 0; i < datasets; i += 4) {
            u8 byte = SGB.data[6 + (i >> 2)];
            for (int j = 0; j < MIN(datasets, 4); ++j) {
              set_sgb_attr_block(x, y, x, y, byte >> ((3 - j) * 2));
              x += dx;
              y += dy;
              if (x >= 20) { x = 0; y++; }
              if (y >= 18) { y = 0; x++; if (x >= 20) { x = 0; } }
            }
          }
          break;
        }
        case 0x0a: // PAL_SET
          unpack_sgb_palette_ram(3, SGB.data[7], SGB.data[8]);
          unpack_sgb_palette_ram(2, SGB.data[5], SGB.data[6]);
          unpack_sgb_palette_ram(1, SGB.data[3], SGB.data[4]);
          unpack_sgb_palette_ram(0, SGB.data[1], SGB.data[2]);
          if (SGB.data[9] & 0x80) {  // Use attr file
            set_sgb_attr(SGB.data[9] & 0x7f);
          }
          break;
        case 0x0b: // PAL_TRN
          memcpy(SGB.pal_ram, xfer_src, sizeof(SGB.pal_ram));
          break;
        case 0x11: // MLT_REQ
          SGB.player_mask = SGB.data[1] & 3;
          break;
        case 0x13: // CHR_TRN
          memcpy(SGB.chr_ram + ((SGB.data[1] & 1) << 12), xfer_src, 4096);
          break;
        case 0x14: { // PCT_TRN
          for (int pal = 0; pal < 4; ++pal) {
            SGB.border_pal[pal][0] = 0;
            for (int col = 1; col < 16; ++col) {
              int idx = 0x800 + (pal * 16 + col) * 2;
              u8 lo = xfer_src[idx], hi = xfer_src[idx + 1];
              SGB.border_pal[pal][col] = unpack_cgb_color8(lo, hi);
            }
          }
          RGBA* dst = sgb_frame_buffer_;
          for (int col = 0; col < 28; ++col) {
            for (int row = 0; row < 32; ++row) {
              int idx = (col * 32 + row) * 2;
              u8 tile = xfer_src[idx];
              u8 info = xfer_src[idx + 1];
              u8 pal = (info >> 2) & 3;
              u8* src = SGB.chr_ram + tile * 32;
              int dsrc = 2;
              if (info & 0x80) {
                dsrc = -2;
                src += 14;
              }
              for (int y = 0; y < 8; ++y, src += dsrc) {
                u8 p0 = src[0], p1 = src[1], p2 = src[16], p3 = src[17];
                if (!(info & 0x40)) {
                  p0 = reverse_bits_u8(p0);
                  p1 = reverse_bits_u8(p1);
                  p2 = reverse_bits_u8(p2);
                  p3 = reverse_bits_u8(p3);
                }
                for (int x = 0; x < 8; ++x) {
                  int palidx = ((p3 & 1) << 3) | ((p2 & 1) << 2) |
                               ((p1 & 1) << 1) | (p0 & 1);
                  dst[(col * 8 + y) * SGB_SCREEN_WIDTH + (row * 8 + x)] =
                      SGB.border_pal[pal][palidx];
                  p0 >>= 1;
                  p1 >>= 1;
                  p2 >>= 1;
                  p3 >>= 1;
                }
              }
            }
          }
          // Update the mask in case we overwrote the center area.
          update_sgb_mask();
          break;
        }
        case 0x15: // ATTR_TRN
          memcpy(SGB.attr_ram, xfer_src, sizeof(SGB.attr_ram));
          break;
        case 0x16: // ATTR_SET
          set_sgb_attr(SGB.data[1]);
          break;
        case 0x17: // MASK_EN
          if (SGB.data[1] <= 3) {
            SGB.mask = (SgbMask)(SGB.data[1]);
            update_sgb_mask();
          }
          break;
        case 0x1e:
        case 0x1f:
          return;  // Invalid
      }
    }
  }
}

void Emulator::write_io(MaskedAddress addr, u8 value) {
  HOOK(write_io_asb, addr, get_io_reg_string(static_cast<IOReg>(addr)), value);
  switch (addr) {
    case IO_JOYP_ADDR:
      JOYP.joypad_select = static_cast<JoypadSelect>(unpack(value, JOYP_JOYPAD_SELECT));
      do_sgb();
      check_joyp_intr();
      break;
    case IO_SB_ADDR:
      serial_synchronize();
      SERIAL.sb = value;
      break;
    case IO_SC_ADDR:
      serial_synchronize();
      SERIAL.transferring = unpack(value, SC_TRANSFER_START);
      SERIAL.clock = static_cast<SerialClock>(unpack(value, SC_SHIFT_CLOCK));
      if (SERIAL.transferring) {
        SERIAL.tick_count = 0;
        SERIAL.transferred_bits = 0;
      }
      calculate_next_serial_intr();
      break;
    case IO_DIV_ADDR:
      timer_synchronize();
      clear_div();
      calculate_next_timer_intr();
      break;
    case IO_TIMA_ADDR:
      timer_synchronize();
      if (TIMER.on) {
        if (UNLIKELY(TIMER.tima_state == TIMA_STATE_OVERFLOW)) {
          /* Cancel the overflow and interrupt if written on the same tick. */
          TIMER.tima_state = TIMA_STATE_NORMAL;
          INTR.new_if &= ~IF_TIMER;
          TIMER.tima = value;
        } else if (TIMER.tima_state != TIMA_STATE_RESET) {
          /* Only update tima if it wasn't reset this tick. */
          TIMER.tima = value;
        }
        calculate_next_timer_intr();
      } else {
        TIMER.tima = value;
      }
      break;
    case IO_TMA_ADDR:
      timer_synchronize();
      TIMER.tma = value;
      if (UNLIKELY(TIMER.on && TIMER.tima_state == TIMA_STATE_RESET)) {
        TIMER.tima = value;
      }
      calculate_next_timer_intr();
      break;
    case IO_TAC_ADDR: {
      timer_synchronize();
      bool old_timer_on = TIMER.on;
      u16 old_tima_mask = s_tima_mask[TIMER.clock_select];
      TIMER.clock_select = static_cast<TimerClock>(unpack(value, TAC_CLOCK_SELECT));
      TIMER.on = unpack(value, TAC_TIMER_ON);
      /* tima is incremented when a specific bit of div_counter transitions
       * from 1 to 0. This can happen as a result of writing to DIV, or in this
       * case modifying which bit we're looking at. */
      bool tima_tick = false;
      if (!old_timer_on) {
        u16 tima_mask = s_tima_mask[TIMER.clock_select];
        if (TIMER.on) {
          tima_tick = (TIMER.div_counter & old_tima_mask) != 0;
        } else {
          tima_tick = (TIMER.div_counter & old_tima_mask) != 0 &&
                      (TIMER.div_counter & tima_mask) == 0;
        }
        if (tima_tick) {
          increment_tima();
        }
      }
      calculate_next_timer_intr();
      break;
    }
    case IO_IF_ADDR:
      intr_synchronize();
      INTR.new_if = INTR.if_ = value & IF_ALL;
      break;
    case IO_LCDC_ADDR: {
      ppu_synchronize();
      ppu_mode3_synchronize();
      bool was_enabled = LCDC.display;
      LCDC.display = unpack(value, LCDC_DISPLAY);
      LCDC.window_tile_map_select = static_cast<TileMapSelect>(unpack(value, LCDC_WINDOW_TILE_MAP_SELECT));
      LCDC.window_display = unpack(value, LCDC_WINDOW_DISPLAY);
      LCDC.bg_tile_data_select = static_cast<TileDataSelect>(unpack(value, LCDC_BG_TILE_DATA_SELECT));
      LCDC.bg_tile_map_select = static_cast<TileMapSelect>(unpack(value, LCDC_BG_TILE_MAP_SELECT));
      LCDC.obj_size = static_cast<ObjSize>(unpack(value, LCDC_OBJ_SIZE));
      LCDC.obj_display = unpack(value, LCDC_OBJ_DISPLAY);
      LCDC.bg_display = unpack(value, LCDC_BG_DISPLAY);
      if (was_enabled ^ LCDC.display) {
        STAT.mode = PPU_MODE_HBLANK;
        PPU.ly = PPU.line_y = 0;
        if (LCDC.display) {
          check_ly_eq_lyc(false);
          HOOK0(enable_display_v);
          PPU.state = PPU_STATE_LCD_ON_MODE2;
          PPU.state_ticks = PPU_MODE2_TICKS;
          PPU.line_start_ticks =
              ALIGN_UP(TICKS - CPU_TICK - CPU_TICK, CPU_TICK);
          PPU.display_delay_frames = PPU_ENABLE_DISPLAY_DELAY_FRAMES;
          STAT.trigger_mode = PPU_MODE_MODE2;
        } else {
          HOOK0(disable_display_v);
          /* Clear the framebuffer. */
          if (IS_SGB) {
            update_sgb_mask();
          } else {
            clear_frame_buffer(RGBA_WHITE);
          }
          state_.event |= EMULATOR_EVENT_NEW_FRAME;
        }
        calculate_next_ppu_intr();
      }
      break;
    }
    case IO_STAT_ADDR: {
      ppu_synchronize();
      bool new_vblank_irq = unpack(value, STAT_VBLANK_INTR);
      bool new_hblank_irq = unpack(value, STAT_HBLANK_INTR);
      if (LCDC.display) {
        bool hblank = TRIGGER_MODE_IS(HBLANK) && !STAT.hblank.irq;
        bool vblank = TRIGGER_MODE_IS(VBLANK) && !STAT.vblank.irq;
        bool y_compare = STAT.new_ly_eq_lyc && !STAT.y_compare.irq;
        if (IS_CGB) {
          /* CGB only triggers on STAT write if the value being written
           * actually sets that IRQ */
          hblank = hblank && new_hblank_irq;
          vblank = vblank && new_vblank_irq;
        }
        if (!STAT.if_ && (hblank || vblank || y_compare)) {
          HOOK(trigger_stat_from_write_cccii, y_compare ? 'Y' : '.',
               vblank ? 'V' : '.', hblank ? 'H' : '.', PPU.ly,
               TICKS + CPU_TICK);
          INTR.new_if |= IF_STAT;
          INTR.if_ |= IF_STAT;
          STAT.if_ = true;
        }
      }
      STAT.y_compare.irq = unpack(value, STAT_YCOMPARE_INTR);
      STAT.mode2.irq = unpack(value, STAT_MODE2_INTR);
      STAT.vblank.irq = new_vblank_irq;
      STAT.hblank.irq = new_hblank_irq;
      calculate_next_ppu_intr();
      break;
    }
    case IO_SCY_ADDR:
      ppu_mode3_synchronize();
      PPU.scy = value;
      break;
    case IO_SCX_ADDR:
      ppu_synchronize();
      ppu_mode3_synchronize();
      PPU.scx = value;
      break;
    case IO_LY_ADDR:
      break;
    case IO_LYC_ADDR:
      ppu_synchronize();
      PPU.lyc = value;
      if (LCDC.display) {
        check_ly_eq_lyc(true);
        check_stat();
      }
      calculate_next_ppu_intr();
      break;
    case IO_DMA_ADDR:
      /* DMA can be restarted. */
      dma_synchronize();
      DMA.sync_ticks = TICKS;
      DMA.tick_count = 0;
      DMA.state = (DMA.state != DMA_INACTIVE ? DMA.state : DMA_TRIGGERED);
      DMA.source = value << 8;
      break;
    case IO_BGP_ADDR:
    case IO_OBP0_ADDR:
    case IO_OBP1_ADDR: {
      PaletteType type = static_cast<PaletteType>(addr - IO_BGP_ADDR);
      Palette* pal = &PPU.pal[type];
      ppu_mode3_synchronize();
      pal->color[3] = static_cast<Color>(unpack(value, PALETTE_COLOR3));
      pal->color[2] = static_cast<Color>(unpack(value, PALETTE_COLOR2));
      pal->color[1] = static_cast<Color>(unpack(value, PALETTE_COLOR1));
      pal->color[0] = static_cast<Color>(unpack(value, PALETTE_COLOR0));
      update_bw_palette_rgba(type);
      break;
    }
    case IO_WY_ADDR:
      ppu_synchronize();
      ppu_mode3_synchronize();
      PPU.wy = value;
      break;
    case IO_WX_ADDR:
      ppu_mode3_synchronize();
      PPU.wx = value;
      break;
    case IO_KEY1_ADDR:
      if (IS_CGB) {
        CPU_SPEED.switching = unpack(value, KEY1_PREPARE_SPEED_SWITCH);
      }
      break;
    case IO_VBK_ADDR:
      if (IS_CGB) {
        VRAM.bank = unpack(value, VBK_VRAM_BANK);
        VRAM.offset = VRAM.bank << 13;
      }
      break;
    case IO_HDMA1_ADDR:
      if (IS_CGB) {
        HDMA.source = (HDMA.source & 0x00ff) | (value << 8);
      }
      break;
    case IO_HDMA2_ADDR:
      if (IS_CGB) {
        HDMA.source = (HDMA.source & 0xff00) | (value & 0xf0);
      }
      break;
    case IO_HDMA3_ADDR:
      if (IS_CGB) {
        HDMA.dest = (HDMA.dest & 0x00ff) | (value << 8);
      }
      break;
    case IO_HDMA4_ADDR:
      if (IS_CGB) {
        HDMA.dest = (HDMA.dest & 0xff00) | (value & 0xf0);
      }
      break;
    case IO_HDMA5_ADDR:
      if (IS_CGB) {
        HdmaTransferMode new_mode = static_cast<HdmaTransferMode>(unpack(value, HDMA5_TRANSFER_MODE));
        u8 new_blocks = unpack(value, HDMA5_BLOCKS);
        if (HDMA.mode == HDMA_TRANSFER_MODE_HDMA &&
            (HDMA.blocks & 0x80) == 0) { /* HDMA Active */
          if (new_mode == HDMA_TRANSFER_MODE_GDMA) {
            /* Stop HDMA copy. */
            HDMA.blocks |= 0x80 | new_blocks;
          } else {
            HDMA.blocks = new_blocks;
            HDMA.mode = new_mode;
          }
        } else {
          HDMA.mode = new_mode;
          HDMA.blocks = new_blocks;
        }
        if (HDMA.mode == HDMA_TRANSFER_MODE_GDMA) {
          HDMA.state = DMA_ACTIVE;
        }
      }
      break;
    case IO_RP_ADDR:
      if (IS_CGB) {
        INFRARED.write = unpack(value, RP_WRITE_DATA);
        INFRARED.enabled = static_cast<DataReadEnable>(unpack(value, RP_DATA_READ_ENABLE));
      }
      break;
    case IO_BCPS_ADDR:
    case IO_OCPS_ADDR:
      if (IS_CGB) {
        ppu_mode3_synchronize();
        ColorPalettes* cp = addr == IO_BCPS_ADDR ? &PPU.bgcp : &PPU.obcp;
        cp->index = unpack(value, XCPS_INDEX);
        cp->auto_increment = unpack(value, XCPS_AUTO_INCREMENT);
      }
      break;
    case IO_BCPD_ADDR:
    case IO_OCPD_ADDR:
      if (IS_CGB) {
        ppu_mode3_synchronize();
        ColorPalettes* cp = addr == IO_BCPD_ADDR ? &PPU.bgcp : &PPU.obcp;
        cp->data[cp->index] = value;
        u8 palette_index = (cp->index >> 3) & 7;
        u8 color_index = (cp->index >> 1) & 3;
        u16 color16 = (cp->data[cp->index | 1] << 8) | cp->data[cp->index & ~1];
        RGBA color = unpack_cgb_color(color16);
        cp->palettes[palette_index].color[color_index] = color;
        if (cp->auto_increment) {
          cp->index = (cp->index + 1) & 0x3f;
        }
      }
      break;
    case IO_SVBK_ADDR:
      if (IS_CGB) {
        WRAM.bank = unpack(value, SVBK_WRAM_BANK);
        WRAM.offset = WRAM.bank == 0 ? 0x1000 : (WRAM.bank << 12);
      }
      break;
    case IO_IE_ADDR:
      INTR.ie = value;
      break;
    default:
      HOOK(write_io_ignored_as, addr, get_io_reg_string((IOReg)addr), value);
      break;
  }
}

void Emulator::write_nrx1_reg(Channel* channel, [[maybe_unused]] Address addr,
                           u8 value) {
  if (APU.enabled) {
    channel->square_wave.duty = static_cast<WaveDuty>(unpack(value, NRX1_WAVE_DUTY));
  }
  channel->length = NRX1_MAX_LENGTH - unpack(value, NRX1_LENGTH);
  HOOK(write_nrx1_abi, addr, value, channel->length);
}

void Emulator::write_nrx2_reg(Channel* channel, [[maybe_unused]] Address addr,
                           u8 value) {
  channel->envelope.initial_volume = unpack(value, NRX2_INITIAL_VOLUME);
  channel->dac_enabled = unpack(value, NRX2_DAC_ENABLED) != 0;
  if (!channel->dac_enabled) {
    channel->status = false;
    HOOK(write_nrx2_disable_dac_ab, addr, value);
  }
  if (channel->status) {
    if (UNLIKELY(channel->envelope.period == 0 &&
                 channel->envelope.automatic)) {
      u8 new_volume = (channel->envelope.volume + 1) & ENVELOPE_MAX_VOLUME;
      HOOK(write_nrx2_zombie_mode_abii, addr, value, channel->envelope.volume,
           new_volume);
      channel->envelope.volume = new_volume;
    }
  }
  channel->envelope.direction = static_cast<EnvelopeDirection>(unpack(value, NRX2_ENVELOPE_DIRECTION));
  channel->envelope.period = unpack(value, NRX2_ENVELOPE_PERIOD);
  HOOK(write_nrx2_initial_volume_abi, addr, value,
       channel->envelope.initial_volume);
}

void Emulator::write_nrx3_reg(Channel* channel, u8 value) {
  channel->frequency = (channel->frequency & ~0xff) | value;
}

/* Returns true if this channel was triggered. */
bool Emulator::write_nrx4_reg(Channel* channel, [[maybe_unused]] Address addr,
                           u8 value, u16 max_length) {
  bool trigger = unpack(value, NRX4_INITIAL);
  bool was_length_enabled = channel->length_enabled;
  channel->length_enabled = unpack(value, NRX4_LENGTH_ENABLED);
  channel->frequency &= 0xff;
  channel->frequency |= unpack(value, NRX4_FREQUENCY_HI) << 8;

  /* Extra length clocking occurs on NRX4 writes if the next APU frame isn't a
   * length counter frame. This only occurs on transition from disabled to
   * enabled. */
  bool next_frame_is_length = (APU.frame & 1) == 1;
  if (UNLIKELY(!was_length_enabled && channel->length_enabled &&
               !next_frame_is_length && channel->length > 0)) {
    channel->length--;
    HOOK(write_nrx4_extra_length_clock_abi, addr, value, channel->length);
    if (!trigger && channel->length == 0) {
      HOOK(write_nrx4_disable_channel_ab, addr, value);
      channel->status = false;
    }
  }

  if (trigger) {
    if (channel->length == 0) {
      channel->length = max_length;
      if (channel->length_enabled && !next_frame_is_length) {
        channel->length--;
      }
      HOOK(write_nrx4_trigger_new_length_abi, addr, value, channel->length);
    }
    if (channel->dac_enabled) {
      channel->status = true;
    }
  }

  HOOK(write_nrx4_info_abii, addr, value, trigger, channel->length_enabled);
  return trigger;
}

void Emulator::trigger_nrx4_envelope(Envelope* envelope, [[maybe_unused]] Address addr) {
  envelope->volume = envelope->initial_volume;
  envelope->timer = envelope->period ? envelope->period : ENVELOPE_MAX_PERIOD;
  envelope->automatic = true;
  /* If the next APU frame will update the envelope, increment the timer. */
  if (UNLIKELY(APU.frame + 1 == FRAME_SEQUENCER_UPDATE_ENVELOPE_FRAME)) {
    envelope->timer++;
  }
  HOOK(trigger_nrx4_info_asii, addr, get_apu_reg_string((APUReg)addr), envelope->volume,
       envelope->timer);
}

u16 Emulator::calculate_sweep_frequency() {
  u16 f = SWEEP.frequency;
  if (SWEEP.direction == SWEEP_DIRECTION_ADDITION) {
    return f + (f >> SWEEP.shift);
  } else {
    SWEEP.calculated_subtract = true;
    return f - (f >> SWEEP.shift);
  }
}

void Emulator::trigger_nr14_reg(Channel* channel) {
  SWEEP.enabled = SWEEP.period || SWEEP.shift;
  SWEEP.frequency = channel->frequency;
  SWEEP.timer = SWEEP.period ? SWEEP.period : SWEEP_MAX_PERIOD;
  SWEEP.calculated_subtract = false;
  if (UNLIKELY(SWEEP.shift &&
               calculate_sweep_frequency() > SOUND_MAX_FREQUENCY)) {
    channel->status = false;
    HOOK0(trigger_nr14_sweep_overflow_v);
  } else {
    HOOK(trigger_nr14_info_i, SWEEP.frequency);
  }
}

void Emulator::write_wave_period(Channel* channel) {
  WAVE.period = ((SOUND_MAX_FREQUENCY + 1) - channel->frequency) * 2;
  HOOK(write_wave_period_info_iii, channel->frequency, WAVE.ticks, WAVE.period);
}

void Emulator::write_square_wave_period(Channel* channel,
                                     SquareWave* square) {
  square->period = ((SOUND_MAX_FREQUENCY + 1) - channel->frequency) * 4;
  HOOK(write_square_wave_period_info_iii, channel->frequency, square->ticks,
       square->period);
}

void Emulator::write_noise_period() {
  static const u8 s_divisors[NOISE_DIVISOR_COUNT] = {8,  16, 32, 48,
                                                     64, 80, 96, 112};
  u8 divisor = s_divisors[NOISE.divisor];
  assert(NOISE.divisor < NOISE_DIVISOR_COUNT);
  NOISE.period = divisor << NOISE.clock_shift;
  HOOK(write_noise_period_info_iii, divisor, NOISE.clock_shift, NOISE.period);
}

void Emulator::write_apu(MaskedAddress addr, u8 value) {
  if (config_.log_apu_writes || !APU.initialized) {
    if (apu_log_.write_count < MAX_APU_LOG_FRAME_WRITES) {
      ApuWrite* write = &apu_log_.writes[apu_log_.write_count++];
      write->addr = addr;
      write->value = value;
    }
  }

  if (!APU.enabled) {
    if (!IS_CGB && (addr == APU_NR11_ADDR || addr == APU_NR21_ADDR ||
                    addr == APU_NR31_ADDR || addr == APU_NR41_ADDR)) {
      /* DMG allows writes to the length counters when power is disabled. */
    } else if (addr == APU_NR52_ADDR) {
      /* Always can write to NR52; it's necessary to re-enable power to APU. */
    } else {
      /* Ignore all other writes. */
      HOOK(write_apu_disabled_asb, addr, get_apu_reg_string((APUReg)addr), value);
      return;
    }
  }

  if (APU.initialized) {
    apu_synchronize();
  }

  HOOK(write_apu_asb, addr, get_apu_reg_string((APUReg)addr), value);
  switch (addr) {
    case APU_NR10_ADDR: {
      SweepDirection old_direction = SWEEP.direction;
      SWEEP.period = unpack(value, NR10_SWEEP_PERIOD);
      SWEEP.direction = static_cast<SweepDirection>(unpack(value, NR10_SWEEP_DIRECTION));
      SWEEP.shift = unpack(value, NR10_SWEEP_SHIFT);
      if (old_direction == SWEEP_DIRECTION_SUBTRACTION &&
          SWEEP.direction == SWEEP_DIRECTION_ADDITION &&
          SWEEP.calculated_subtract) {
        CHANNEL1.status = false;
      }
      break;
    }
    case APU_NR11_ADDR:
      write_nrx1_reg(&CHANNEL1, addr, value);
      break;
    case APU_NR12_ADDR:
      write_nrx2_reg(&CHANNEL1, addr, value);
      break;
    case APU_NR13_ADDR:
      write_nrx3_reg(&CHANNEL1, value);
      write_square_wave_period(&CHANNEL1, &CHANNEL1.square_wave);
      break;
    case APU_NR14_ADDR: {
      bool trigger = write_nrx4_reg(&CHANNEL1, addr, value, NRX1_MAX_LENGTH);
      write_square_wave_period(&CHANNEL1, &CHANNEL1.square_wave);
      if (trigger) {
        trigger_nrx4_envelope(&CHANNEL1.envelope, addr);
        trigger_nr14_reg(&CHANNEL1);
        CHANNEL1.square_wave.ticks = CHANNEL1.square_wave.period;
      }
      break;
    }
    case APU_NR21_ADDR:
      write_nrx1_reg(&CHANNEL2, addr, value);
      break;
    case APU_NR22_ADDR:
      write_nrx2_reg(&CHANNEL2, addr, value);
      break;
    case APU_NR23_ADDR:
      write_nrx3_reg(&CHANNEL2, value);
      write_square_wave_period(&CHANNEL2, &CHANNEL2.square_wave);
      break;
    case APU_NR24_ADDR: {
      bool trigger = write_nrx4_reg(&CHANNEL2, addr, value, NRX1_MAX_LENGTH);
      write_square_wave_period(&CHANNEL2, &CHANNEL2.square_wave);
      if (trigger) {
        trigger_nrx4_envelope(&CHANNEL2.envelope, addr);
        CHANNEL2.square_wave.ticks = CHANNEL2.square_wave.period;
      }
      break;
    }
    case APU_NR30_ADDR:
      CHANNEL3.dac_enabled = unpack(value, NR30_DAC_ENABLED);
      if (!CHANNEL3.dac_enabled) {
        CHANNEL3.status = false;
        WAVE.playing = false;
      }
      break;
    case APU_NR31_ADDR:
      CHANNEL3.length = NR31_MAX_LENGTH - value;
      break;
    case APU_NR32_ADDR:
      WAVE.volume = static_cast<WaveVolume>(unpack(value, NR32_SELECT_WAVE_VOLUME));
      assert(WAVE.volume < WAVE_VOLUME_COUNT);
      WAVE.volume_shift = s_wave_volume_shift[WAVE.volume];
      break;
    case APU_NR33_ADDR:
      write_nrx3_reg(&CHANNEL3, value);
      write_wave_period(&CHANNEL3);
      break;
    case APU_NR34_ADDR: {
      bool trigger = write_nrx4_reg(&CHANNEL3, addr, value, NR31_MAX_LENGTH);
      write_wave_period(&CHANNEL3);
      if (trigger) {
        if (!IS_CGB && WAVE.playing) {
          /* Triggering the wave channel while it is already playing will
           * corrupt the wave RAM on DMG. */
          if (WAVE.ticks == WAVE_TRIGGER_CORRUPTION_OFFSET_TICKS) {
            assert(WAVE.position < 32);
            u8 position = (WAVE.position + 1) & 31;
            u8 byte = WAVE.ram[position >> 1];
            switch (position >> 3) {
              case 0:
                WAVE.ram[0] = byte;
                break;
              case 1:
              case 2:
              case 3:
                memcpy(&WAVE.ram[0], &WAVE.ram[(position >> 1) & 12], 4);
                break;
            }
            HOOK(corrupt_wave_ram_i, position);
          }
        }

        WAVE.position = 0;
        WAVE.ticks = WAVE.period + WAVE_TRIGGER_DELAY_TICKS;
        WAVE.playing = true;
      }
      break;
    }
    case APU_NR41_ADDR:
      write_nrx1_reg(&CHANNEL4, addr, value);
      break;
    case APU_NR42_ADDR:
      write_nrx2_reg(&CHANNEL4, addr, value);
      break;
    case APU_NR43_ADDR: {
      NOISE.clock_shift = unpack(value, NR43_CLOCK_SHIFT);
      NOISE.lfsr_width = static_cast<LfsrWidth>(unpack(value, NR43_LFSR_WIDTH));
      NOISE.divisor = unpack(value, NR43_DIVISOR);
      write_noise_period();
      break;
    }
    case APU_NR44_ADDR: {
      bool trigger = write_nrx4_reg(&CHANNEL4, addr, value, NRX1_MAX_LENGTH);
      if (trigger) {
        write_noise_period();
        trigger_nrx4_envelope(&CHANNEL4.envelope, addr);
        NOISE.lfsr = 0x7fff;
        NOISE.sample = 1;
        NOISE.ticks = NOISE.period;
      }
      break;
    }
    case APU_NR50_ADDR:
      APU.so_output[VIN][1] = unpack(value, NR50_VIN_SO2);
      APU.so_volume[1] = unpack(value, NR50_SO2_VOLUME);
      APU.so_output[VIN][0] = unpack(value, NR50_VIN_SO1);
      APU.so_volume[0] = unpack(value, NR50_SO1_VOLUME);
      break;
    case APU_NR51_ADDR:
      APU.so_output[SOUND4][1] = unpack(value, NR51_SOUND4_SO2);
      APU.so_output[SOUND3][1] = unpack(value, NR51_SOUND3_SO2);
      APU.so_output[SOUND2][1] = unpack(value, NR51_SOUND2_SO2);
      APU.so_output[SOUND1][1] = unpack(value, NR51_SOUND1_SO2);
      APU.so_output[SOUND4][0] = unpack(value, NR51_SOUND4_SO1);
      APU.so_output[SOUND3][0] = unpack(value, NR51_SOUND3_SO1);
      APU.so_output[SOUND2][0] = unpack(value, NR51_SOUND2_SO1);
      APU.so_output[SOUND1][0] = unpack(value, NR51_SOUND1_SO1);
      break;
    case APU_NR52_ADDR: {
      bool was_enabled = APU.enabled;
      bool is_enabled = unpack(value, NR52_ALL_SOUND_ENABLED);
      if (was_enabled && !is_enabled) {
        HOOK0(apu_power_down_v);
        int i;
        for (i = 0; i < APU_REG_COUNT; ++i) {
          if (i != APU_NR52_ADDR) {
            write_apu(i, 0);
          }
        }
      } else if (!was_enabled && is_enabled) {
        HOOK0(apu_power_up_v);
        APU.frame = 7;
      }
      APU.enabled = is_enabled;
      break;
    }
  }
}

void Emulator::write_wave_ram(MaskedAddress addr, u8 value) {
  apu_synchronize();
  if (CHANNEL3.status) {
    /* If the wave channel is playing, the byte is written to the sample
     * position. On DMG, this is only allowed if the write occurs exactly when
     * it is being accessed by the Wave channel. */
    if (UNLIKELY(IS_CGB || TICKS == WAVE.sample_time)) {
      WAVE.ram[WAVE.position >> 1] = value;
      HOOK(write_wave_ram_while_playing_ab, addr, value);
    }
  } else {
    WAVE.ram[addr] = value;
    HOOK(write_wave_ram_ab, addr, value);
  }
}

void Emulator::write_u8_pair(MemoryTypeAddressPair pair, u8 value) {
  switch (pair.type) {
    case MEMORY_MAP_ROM0:
      memory_map_.write_rom(pair.addr, value);
      break;
    case MEMORY_MAP_ROM1:
      memory_map_.write_rom(pair.addr + 0x4000, value);
      break;
    case MEMORY_MAP_VRAM:
      write_vram(pair.addr, value);
      break;
    case MEMORY_MAP_EXT_RAM:
      memory_map_.write_ext_ram(pair.addr, value);
      break;
    case MEMORY_MAP_WORK_RAM0:
      WRAM.data[pair.addr] = value;
      break;
    case MEMORY_MAP_WORK_RAM1:
      WRAM.data[WRAM.offset + pair.addr] = value;
      break;
    case MEMORY_MAP_OAM:
      write_oam(pair.addr, value);
      break;
    case MEMORY_MAP_UNUSED:
      break;
    case MEMORY_MAP_IO:
      write_io(pair.addr, value);
      break;
    case MEMORY_MAP_APU:
      write_apu(pair.addr, value);
      break;
    case MEMORY_MAP_WAVE_RAM:
      write_wave_ram(pair.addr, value);
      break;
    case MEMORY_MAP_HIGH_RAM:
      HRAM[pair.addr] = value;
      break;
  }
}

[[maybe_unused]] void Emulator::write_u8_raw(Address addr, u8 value) {
  write_u8_pair(map_address(addr), value);
}

void Emulator::write_u8(Address addr, u8 value) {
  dma_synchronize();
  if (UNLIKELY(!is_dma_access_ok(addr))) {
    HOOK(write_during_dma_ab, addr, value);
    return;
  }
  write_u8_pair(map_address(addr), value);
}

void Emulator::do_ppu_mode2() {
  dma_synchronize();
  if (!LCDC.obj_display || config_.disable_obj) {
    return;
  }

  int line_obj_count = 0;
  int i;
  u8 obj_height = s_obj_size_to_height[LCDC.obj_size];
  u8 y = PPU.line_y;
  for (i = 0; i < OBJ_COUNT; ++i) {
    /* Put the visible sprites into line_obj. Insert them so sprites with
     * smaller X-coordinates are earlier, but only on DMG. On CGB, they are
     * always ordered by obj index. */
    Obj* o = &OAM[i];
    u8 rel_y = y - o->y;
    if (rel_y < obj_height) {
      int j = line_obj_count;
      if (!IS_CGB) {
        while (j > 0 && o->x < PPU.line_obj[j - 1].x) {
          PPU.line_obj[j] = PPU.line_obj[j - 1];
          j--;
        }
      }
      PPU.line_obj[j] = *o;
      if (++line_obj_count == OBJ_PER_LINE_COUNT) {
        break;
      }
    }
  }
  PPU.line_obj_count = line_obj_count;
}

u32 Emulator::mode3_tick_count() {
  s32 buckets[SCREEN_WIDTH / 8 + 2];
  ZERO_MEMORY(buckets);
  u8 scx_fine = PPU.scx & 7;
  u32 ticks = PPU_MODE3_MIN_TICKS + scx_fine;
  bool has_zero = false;
  int i;
  for (i = 0; i < PPU.line_obj_count; ++i) {
    Obj* o = &PPU.line_obj[i];
    u8 x = o->x + OBJ_X_OFFSET;
    if (x >= SCREEN_WIDTH + OBJ_X_OFFSET) {
      continue;
    }
    if (!has_zero && x == 0) {
      has_zero = true;
      ticks += scx_fine;
    }
    x += scx_fine;
    int bucket = x >> 3;
    buckets[bucket] = MAX(buckets[bucket], 5 - (x & 7));
    ticks += 6;
  }
  for (i = 0; i < (int)ARRAY_SIZE(buckets); ++i) {
    ticks += buckets[i];
  }
  return ticks;
}

void Emulator::ppu_mode3_synchronize() {
  u8 x = PPU.render_x;
  const u8 y = PPU.line_y;
  if (STAT.mode != PPU_MODE_MODE3 || x >= SCREEN_WIDTH) return;

  bool display_bg = (IS_CGB || LCDC.bg_display) && !config_.disable_bg;
  const bool display_obj = LCDC.obj_display && !config_.disable_obj;
  bool rendering_window = PPU.rendering_window;
  int window_counter = rendering_window ? 0 : 255;
  if (!rendering_window && LCDC.window_display && !config_.disable_window &&
      PPU.wx <= WINDOW_MAX_X && y >= PPU.wy) {
    window_counter = MAX(0, PPU.wx - (x + WINDOW_X_OFFSET));
  }

  const TileDataSelect data_select = LCDC.bg_tile_data_select;
  u8 mx = PPU.scx + x;
  u8 my = PPU.scy + y;
  u16 map_base = map_select_to_address(LCDC.bg_tile_map_select) |
                 ((my >> 3) * TILE_MAP_WIDTH);
  RGBA* pixel;
  if (SGB.mask != SGB_MASK_CANCEL) {
    static RGBA s_dummy_frame_buffer_line[SCREEN_WIDTH];
    pixel = s_dummy_frame_buffer_line;
  } else {
    pixel = &frame_buffer_[y * SCREEN_WIDTH + x];
  }

  /* Cache map_addr info. */
  u16 map_addr = 0;
  PaletteRGBA* pal = NULL;
  u8 lo = 0, hi = 0;

  bool priority = false;
  int i;
  for (; PPU.mode3_render_ticks < TICKS && x < SCREEN_WIDTH;
       PPU.mode3_render_ticks += CPU_TICK, pixel += 4, x += 4) {
    bool bg_is_zero[4] = {true, true, true, true},
         bg_priority[4] = {false, false, false, false};
    for (i = 0; i < 4; ++i, ++mx) {
      if (UNLIKELY(window_counter-- == 0)) {
        PPU.rendering_window = rendering_window = display_bg = true;
        mx = x + i + WINDOW_X_OFFSET - PPU.wx;
        my = PPU.win_y;
        map_base = map_select_to_address(LCDC.window_tile_map_select) |
                  ((my >> 3) * TILE_MAP_WIDTH);
        map_addr = 0;
      }
      if (display_bg) {
        u16 new_map_addr = map_base | (mx >> 3);
        if (map_addr == new_map_addr) {
          lo <<= 1;
          hi <<= 1;
        } else {
          map_addr = new_map_addr;
          u16 tile_index = VRAM.data[map_addr];
          u8 my7 = my & 7;
          if (data_select == TILE_DATA_8800_97FF) {
            tile_index = 256 + (s8)tile_index;
          }
          if (IS_CGB) {
            u8 attr = VRAM.data[0x2000 + map_addr];
            pal = &PPU.bgcp.palettes[attr & 0x7];
            if (attr & 0x08) { tile_index += 0x200; }
            if (attr & 0x40) { my7 = 7 - my7; }
            priority = (attr & 0x80) != 0;
            u16 tile_addr = (tile_index * TILE_HEIGHT + my7) * TILE_ROW_BYTES;
            lo = VRAM.data[tile_addr];
            hi = VRAM.data[tile_addr + 1];
            if (attr & 0x20) {
              lo = reverse_bits_u8(lo);
              hi = reverse_bits_u8(hi);
            }
          } else {
            if (IS_SGB) {
              int idx = (y >> 3) * (SCREEN_WIDTH >> 3) + (x >> 3);
              u8 palidx = (SGB.attr_map[idx >> 2] >> (2 * (3 - (idx & 3)))) & 3;
              pal = &sgb_pal_[palidx];
            } else {
              pal = &this->pal_[PALETTE_TYPE_BGP];
            }
            priority = false;
            u16 tile_addr = (tile_index * TILE_HEIGHT + my7) * TILE_ROW_BYTES;
            lo = VRAM.data[tile_addr];
            hi = VRAM.data[tile_addr + 1];
          }
          u8 shift = mx & 7;
          lo <<= shift;
          hi <<= shift;
        }
        u8 palette_index = ((hi >> 6) & 2) | (lo >> 7);
        pixel[i] = pal->color[palette_index];
        bg_is_zero[i] = palette_index == 0;
        bg_priority[i] = priority;
      } else {
        if (IS_CGB) {
          pixel[i] = PPU.bgcp.palettes[0].color[0];
        } else if (IS_SGB) {
          pixel[i] = sgb_pal_[0].color[0];
        } else {
          pixel[i] = color_to_rgba_[0].color[0];
        }
      }
    }

    /* LCDC bit 0 works differently on cgb; when it's cleared OBJ will always
     * have priority over bg and window. */
    if (IS_CGB && !LCDC.bg_display) {
      memset(&bg_is_zero, true, sizeof(bg_is_zero));
      memset(&bg_priority, false, sizeof(bg_priority));
    }

    if (display_obj) {
      u8 obj_height = s_obj_size_to_height[LCDC.obj_size];
      int n;
      for (n = PPU.line_obj_count - 1; n >= 0; --n) {
        Obj* o = &PPU.line_obj[n];
        /* Does [x, x + 4) intersect [o->x, o->x + 8)? Note that the sums must
         * wrap at 256 (i.e. arithmetic is 8-bit). */
        s8 ox_start = o->x - x;
        s8 ox_end = ox_start + 7; /* ox_end is inclusive. */
        u8 oy = y - o->y;
        if (((u8)ox_start >= 4 && (u8)ox_end >= 8) || oy >= obj_height) {
          continue;
        }

        if (o->yflip) {
          oy = obj_height - 1 - oy;
        }

        u16 tile_index = o->tile;
        if (obj_height == 16) {
          if (oy < 8) {
            /* Top tile of 8x16 sprite. */
            tile_index &= 0xfe;
          } else {
            /* Bottom tile of 8x16 sprite. */
            tile_index |= 0x01;
            oy -= 8;
          }
        }
        PaletteRGBA* pal2 = NULL;
        if (IS_CGB) {
          pal2 = &PPU.obcp.palettes[o->cgb_palette & 0x7];
          if (o->bank) { tile_index += 0x200; }
        } else {
          pal2 = &this->pal_[o->palette + 1];
        }
        u16 tile_addr = (tile_index * TILE_HEIGHT + (oy & 7)) * TILE_ROW_BYTES;
        u8 lo2 = VRAM.data[tile_addr];
        u8 hi2 = VRAM.data[tile_addr + 1];
        if (!o->xflip) {
          lo2 = reverse_bits_u8(lo2);
          hi2 = reverse_bits_u8(hi2);
        }

        int tile_data_offset = MAX(0, -ox_start);
        assert(tile_data_offset >= 0 && tile_data_offset < 8);
        lo2 >>= tile_data_offset;
        hi2 >>= tile_data_offset;

        int start = MAX(0, ox_start);
        assert(start >= 0 && start < 4);
        int end = MIN(3, ox_end); /* end is inclusive. */
        assert(end >= 0 && end < 4);
        for (i = start; i <= end; ++i, lo2 >>= 1, hi2 >>= 1) {
          u8 palette_index = ((hi2 & 1) << 1) | (lo2 & 1);
          if (palette_index != 0 && (!bg_priority[i] || bg_is_zero[i]) &&
              (o->priority == OBJ_PRIORITY_ABOVE_BG || bg_is_zero[i])) {
            pixel[i] = pal2->color[palette_index];
          }
        }
      }
    }
  }
  PPU.render_x = x;
}

void Emulator::ppu_synchronize() {
  assert(IS_ALIGNED(PPU.sync_ticks, CPU_TICK));
  Ticks aligned_ticks = ALIGN_DOWN(TICKS, CPU_TICK);
  if (aligned_ticks > PPU.sync_ticks) {
    Ticks delta_ticks = aligned_ticks - PPU.sync_ticks;

    if (LCDC.display) {
      for (; delta_ticks > 0; delta_ticks -= CPU_TICK) {
        INTR.if_ |= (INTR.new_if & (IF_VBLANK | IF_STAT));
        STAT.mode2.trigger = false;
        STAT.y_compare.trigger = false;
        STAT.ly_eq_lyc = STAT.new_ly_eq_lyc;
        PPU.last_ly = PPU.ly;

        PPU.state_ticks -= CPU_TICK;
        if (LIKELY(PPU.state_ticks != 0)) {
          continue;
        }

        Ticks ticks = aligned_ticks - delta_ticks;

        switch (PPU.state) {
          case PPU_STATE_HBLANK:
          case PPU_STATE_VBLANK_PLUS_4:
            PPU.line_y++;
            PPU.ly++;
            PPU.line_start_ticks = ticks;
            check_ly_eq_lyc(false);
            PPU.state_ticks = CPU_TICK;

            if (PPU.state == PPU_STATE_HBLANK) {
              STAT.mode2.trigger = true;
              if (PPU.ly == SCREEN_HEIGHT) {
                PPU.state = PPU_STATE_VBLANK;
                STAT.trigger_mode = PPU_MODE_VBLANK;
                PPU.frame++;
                INTR.new_if |= IF_VBLANK;
                if (LIKELY(PPU.display_delay_frames == 0)) {
                  state_.event |= EMULATOR_EVENT_NEW_FRAME;
                } else {
                  PPU.display_delay_frames--;
                }
              } else {
                PPU.state = PPU_STATE_HBLANK_PLUS_4;
                STAT.trigger_mode = PPU_MODE_MODE2;
                if (PPU.rendering_window) {
                  PPU.win_y++;
                }
                if (UNLIKELY(HDMA.mode == HDMA_TRANSFER_MODE_HDMA &&
                             (HDMA.blocks & 0x80) == 0)) {
                  HDMA.state = DMA_ACTIVE;
                }
              }
            } else {
              assert(PPU.state == PPU_STATE_VBLANK_PLUS_4);
              if (PPU.ly == SCREEN_HEIGHT_WITH_VBLANK - 1) {
                PPU.state = PPU_STATE_VBLANK_LY_0;
              } else {
                PPU.state_ticks = PPU_LINE_TICKS;
              }
            }
            check_stat();
            break;

          case PPU_STATE_HBLANK_PLUS_4:
            PPU.state = PPU_STATE_MODE2;
            PPU.state_ticks = PPU_MODE2_TICKS;
            STAT.mode = PPU_MODE_MODE2;
            do_ppu_mode2();
            break;

          case PPU_STATE_VBLANK:
            PPU.state = PPU_STATE_VBLANK_PLUS_4;
            PPU.state_ticks = PPU_LINE_TICKS - CPU_TICK;
            STAT.mode = PPU_MODE_VBLANK;
            check_stat();
            break;

          case PPU_STATE_VBLANK_LY_0:
            PPU.state = PPU_STATE_VBLANK_LY_0_PLUS_4;
            PPU.state_ticks = CPU_TICK;
            PPU.ly = 0;
            break;

          case PPU_STATE_VBLANK_LY_0_PLUS_4:
            PPU.state = PPU_STATE_VBLANK_LINE_Y_0;
            PPU.state_ticks = PPU_LINE_TICKS - CPU_TICK - CPU_TICK;
            check_ly_eq_lyc(false);
            check_stat();
            break;

          case PPU_STATE_VBLANK_LINE_Y_0:
            PPU.state = PPU_STATE_HBLANK_PLUS_4;
            PPU.state_ticks = CPU_TICK;
            PPU.line_start_ticks = ticks;
            PPU.line_y = 0;
            PPU.win_y = 0;
            STAT.mode2.trigger = true;
            STAT.mode = PPU_MODE_HBLANK;
            STAT.trigger_mode = PPU_MODE_MODE2;
            check_stat();
            break;

          case PPU_STATE_LCD_ON_MODE2:
          case PPU_STATE_MODE2:
            PPU.state_ticks = mode3_tick_count();
            if (PPU.state == PPU_STATE_LCD_ON_MODE2 ||
                (PPU.state_ticks & 3) != 0) {
              PPU.state = PPU_STATE_MODE3;
            } else {
              PPU.state = PPU_STATE_MODE3_EARLY_TRIGGER;
              PPU.state_ticks--;
            }
            PPU.state_ticks &= ~3;
            STAT.mode = STAT.trigger_mode = PPU_MODE_MODE3;
            PPU.mode3_render_ticks = ticks;
            PPU.render_x = 0;
            PPU.rendering_window = false;
            check_stat();
            break;

          case PPU_STATE_MODE3_EARLY_TRIGGER:
            PPU.state = PPU_STATE_MODE3_COMMON;
            PPU.state_ticks = CPU_TICK;
            STAT.trigger_mode = PPU_MODE_HBLANK;
            check_stat();
            break;

          case PPU_STATE_MODE3:
            STAT.trigger_mode = PPU_MODE_HBLANK;
            /* fallthrough */

          case PPU_STATE_MODE3_COMMON:
            ppu_mode3_synchronize();
            PPU.state = PPU_STATE_HBLANK;
            PPU.state_ticks = PPU_LINE_TICKS + PPU.line_start_ticks - ticks;
            STAT.mode = PPU_MODE_HBLANK;
            check_stat();
            break;

          case PPU_STATE_COUNT:
            assert(0);
            break;
        }

        PPU.sync_ticks = ticks + CPU_TICK;
        calculate_next_ppu_intr();
      }
    }
    PPU.sync_ticks = aligned_ticks;
  }
}

void Emulator::calculate_next_ppu_intr() {
  if (LCDC.display) {
    /* TODO: Looser bounds on sync points. This syncs at every state
     * transition, even though we often won't need to sync that often. */
    PPU.next_intr_ticks = PPU.sync_ticks + PPU.state_ticks;
  } else {
    PPU.next_intr_ticks = INVALID_TICKS;
  }
  calculate_next_intr();
}

void Emulator::update_sweep() {
  if (!(CHANNEL1.status && SWEEP.enabled)) {
    return;
  }

  u8 period = SWEEP.period;
  if (--SWEEP.timer == 0) {
    if (period) {
      SWEEP.timer = period;
      u16 new_frequency = calculate_sweep_frequency();
      if (new_frequency > SOUND_MAX_FREQUENCY) {
        HOOK0(sweep_overflow_v);
        CHANNEL1.status = false;
      } else {
        if (SWEEP.shift) {
          HOOK(sweep_update_frequency_i, new_frequency);
          SWEEP.frequency = CHANNEL1.frequency = new_frequency;
          write_square_wave_period(&CHANNEL1, &CHANNEL1.square_wave);
        }

        /* Perform another overflow check. */
        if (UNLIKELY(calculate_sweep_frequency() > SOUND_MAX_FREQUENCY)) {
          HOOK0(sweep_overflow_2nd_v);
          CHANNEL1.status = false;
        }
      }
    } else {
      SWEEP.timer = SWEEP_MAX_PERIOD;
    }
  }
}

void Emulator::update_lengths() {
  int i;
  for (i = 0; i < APU_CHANNEL_COUNT; ++i) {
    Channel* channel = &APU.channel[i];
    if (channel->length_enabled && channel->length > 0) {
      if (--channel->length == 0) {
        channel->status = false;
      }
    }
  }
}

void Emulator::update_envelopes() {
  int i;
  for (i = 0; i < APU_CHANNEL_COUNT; ++i) {
    Envelope* envelope = &APU.channel[i].envelope;
    if (envelope->period) {
      if (envelope->automatic && --envelope->timer == 0) {
        envelope->timer = envelope->period;
        u8 delta = envelope->direction == ENVELOPE_ATTENUATE ? -1 : 1;
        u8 volume = envelope->volume + delta;
        if (volume < ENVELOPE_MAX_VOLUME) {
          envelope->volume = volume;
        } else {
          envelope->automatic = false;
        }
      }
    } else {
      envelope->timer = ENVELOPE_MAX_PERIOD;
    }
  }
}

/* Convert from 1-bit sample to 4-bit sample. */
#define CHANNELX_SAMPLE(channel, sample) \
  (-(sample) & (channel)->envelope.volume)

static void update_square_wave(Channel* channel, u32 total_frames) {
  static u8 duty[WAVE_DUTY_COUNT][DUTY_CYCLE_COUNT] =
      {[WAVE_DUTY_12_5] = {0, 0, 0, 0, 0, 0, 0, 1},
       [WAVE_DUTY_25] = {1, 0, 0, 0, 0, 0, 0, 1},
       [WAVE_DUTY_50] = {1, 0, 0, 0, 0, 1, 1, 1},
       [WAVE_DUTY_75] = {0, 1, 1, 1, 1, 1, 1, 0}};
  SquareWave* square = &channel->square_wave;
  if (channel->status) {
    while (total_frames) {
      u32 frames = square->ticks / APU_TICKS;
      u8 sample = CHANNELX_SAMPLE(channel, square->sample);
      if (frames <= total_frames) {
        square->ticks = square->period;
        square->position = (square->position + 1) % DUTY_CYCLE_COUNT;
        square->sample = duty[square->duty][square->position];
      } else {
        frames = total_frames;
        square->ticks -= frames * APU_TICKS;
      }
      channel->accumulator += sample * frames;
      total_frames -= frames;
    }
  }
}

void Emulator::update_wave(u32 apu_ticks, u32 total_frames) {
  if (CHANNEL3.status) {
    while (total_frames) {
      u32 frames = WAVE.ticks / APU_TICKS;
      /* Modulate 4-bit sample by wave volume. */
      u8 sample = WAVE.sample_data >> WAVE.volume_shift;
      if (frames <= total_frames) {
        WAVE.position = (WAVE.position + 1) % WAVE_SAMPLE_COUNT;
        WAVE.sample_time = apu_ticks + WAVE.ticks;
        u8 byte = WAVE.ram[WAVE.position >> 1];
        if ((WAVE.position & 1) == 0) {
          WAVE.sample_data = byte >> 4; /* High nybble. */
        } else {
          WAVE.sample_data = byte & 0x0f; /* Low nybble. */
        }
        WAVE.ticks = WAVE.period;
        HOOK(wave_update_position_iii, WAVE.position, WAVE.sample_data,
             WAVE.sample_time);
      } else {
        frames = total_frames;
        WAVE.ticks -= frames * APU_TICKS;
      }
      apu_ticks += frames * APU_TICKS;
      CHANNEL3.accumulator += sample * frames;
      total_frames -= frames;
    }
  }
}

void Emulator::update_noise(u32 total_frames) {
  if (CHANNEL4.status) {
    while (total_frames) {
      u32 frames = NOISE.ticks / APU_TICKS;
      u8 sample = CHANNELX_SAMPLE(&CHANNEL4, NOISE.sample);
      if (NOISE.clock_shift <= NOISE_MAX_CLOCK_SHIFT) {
        if (frames <= total_frames) {
          u16 bit = (NOISE.lfsr ^ (NOISE.lfsr >> 1)) & 1;
          if (NOISE.lfsr_width == LFSR_WIDTH_7) {
            NOISE.lfsr = ((NOISE.lfsr >> 1) & ~0x40) | (bit << 6);
          } else {
            NOISE.lfsr = ((NOISE.lfsr >> 1) & ~0x4000) | (bit << 14);
          }
          NOISE.sample = ~NOISE.lfsr & 1;
          NOISE.ticks = NOISE.period;
        } else {
          frames = total_frames;
          NOISE.ticks -= frames * APU_TICKS;
        }
      } else {
        frames = total_frames;
      }
      CHANNEL4.accumulator += sample * frames;
      total_frames -= frames;
    }
  }
}

u32 Emulator::get_gb_frames_until_next_resampled_frame() {
  u32 result = 0;
  u32 counter = audio_buffer_.freq_counter;
  while (!VALUE_WRAPPED(counter, APU_TICKS_PER_SECOND)) {
    counter += audio_buffer_.frequency;
    result++;
  }
  return result;
}

void Emulator::write_audio_frame(u32 gb_frames) {
  int i, j;
  AudioBuffer* buffer = &audio_buffer_;
  buffer->divisor += gb_frames;
  buffer->freq_counter += buffer->frequency * gb_frames;
  if (VALUE_WRAPPED(buffer->freq_counter, APU_TICKS_PER_SECOND)) {
    for (i = 0; i < SOUND_OUTPUT_COUNT; ++i) {
      u32 accumulator = 0;
      for (j = 0; j < APU_CHANNEL_COUNT; ++j) {
        if (!config_.disable_sound[j]) {
          accumulator += APU.channel[j].accumulator * APU.so_output[j][i];
        }
      }
      accumulator *= (APU.so_volume[i] + 1) * 16; /* 4bit -> 8bit samples. */
      accumulator /= ((SOUND_OUTPUT_MAX_VOLUME + 1) * APU_CHANNEL_COUNT);
      *buffer->position++ = accumulator / buffer->divisor;
    }
    for (j = 0; j < APU_CHANNEL_COUNT; ++j) {
      APU.channel[j].accumulator = 0;
    }
    buffer->divisor = 0;
  }
  assert(buffer->position <= buffer->end);
}

void Emulator::apu_update_channels(u32 total_frames) {
  while (total_frames) {
    u32 frames = get_gb_frames_until_next_resampled_frame();
    frames = MIN(frames, total_frames);
    update_square_wave(&CHANNEL1, frames);
    update_square_wave(&CHANNEL2, frames);
    update_wave(APU.sync_ticks, frames);
    update_noise(frames);
    write_audio_frame(frames);
    APU.sync_ticks += frames * APU_TICKS;
    total_frames -= frames;
  }
}

void Emulator::apu_update(u32 total_ticks) {
  while (total_ticks) {
    Ticks next_seq_ticks = NEXT_MODULO(APU.sync_ticks, FRAME_SEQUENCER_TICKS);
    if (next_seq_ticks == FRAME_SEQUENCER_TICKS) {
      APU.frame = (APU.frame + 1) % FRAME_SEQUENCER_COUNT;
      switch (APU.frame) {
        case 2: case 6: update_sweep(); /* Fallthrough. */
        case 0: case 4: update_lengths(); break;
        case 7: update_envelopes(); break;
      }
    }
    Ticks ticks = MIN(next_seq_ticks, total_ticks);
    apu_update_channels(ticks / APU_TICKS);
    total_ticks -= ticks;
  }
}

void Emulator::intr_synchronize() {
  dma_synchronize();
  serial_synchronize();
  ppu_synchronize();
  timer_synchronize();
}

void Emulator::apu_synchronize() {
  if (TICKS > APU.sync_ticks) {
    u32 ticks = TICKS - APU.sync_ticks;
    if (APU.enabled) {
      apu_update(ticks);
      assert(APU.sync_ticks == TICKS);
    } else {
      for (; ticks; ticks -= APU_TICKS) {
        write_audio_frame(1);
      }
      APU.sync_ticks = TICKS;
    }
  }
}

void Emulator::dma_synchronize() {
  if (UNLIKELY(DMA.state != DMA_INACTIVE)) {
    if (TICKS > DMA.sync_ticks) {
      Ticks delta_ticks = TICKS - DMA.sync_ticks;
      DMA.sync_ticks = TICKS;

      Ticks cpu_tick = state_.cpu_tick;
      for (; delta_ticks > 0; delta_ticks -= cpu_tick) {
        if (DMA.tick_count < DMA_DELAY_TICKS) {
          DMA.tick_count += CPU_TICK;
          if (DMA.tick_count >= DMA_DELAY_TICKS) {
            DMA.tick_count = DMA_DELAY_TICKS;
            DMA.state = DMA_ACTIVE;
          }
          continue;
        }

        u8 addr_offset = (DMA.tick_count - DMA_DELAY_TICKS) >> 2;
        assert(addr_offset < OAM_TRANSFER_SIZE);
        u8 value =
            read_u8_pair(map_address(DMA.source + addr_offset), false);
        write_oam_no_mode_check(addr_offset, value);
        DMA.tick_count += CPU_TICK;
        if (VALUE_WRAPPED(DMA.tick_count, DMA_TICKS)) {
          DMA.state = DMA_INACTIVE;
          break;
        }
      }
    }
  }
}

void Emulator::hdma_copy_byte() {
  MemoryTypeAddressPair source_pair = map_hdma_source_address(HDMA.source++);
  u8 value;
  if (UNLIKELY(source_pair.type == MEMORY_MAP_VRAM)) {
    /* TODO(binji): According to TCAGBD this should read "two unknown bytes",
     * then 0xff for the rest. */
    value = INVALID_READ_BYTE;
  } else {
    value = read_u8_pair(source_pair, false);
  }
  write_vram(HDMA.dest++ & ADDR_MASK_8K, value);
  HDMA.block_bytes++;
  if (VALUE_WRAPPED(HDMA.block_bytes, 16)) {
    --HDMA.blocks;
    if (HDMA.mode == HDMA_TRANSFER_MODE_GDMA) {
      if (HDMA.blocks == 0xff) {
        HDMA.state = DMA_INACTIVE;
      }
    } else {
      HDMA.state = DMA_INACTIVE;
    }
  }
}

void Emulator::calculate_next_serial_intr() {
  if (!SERIAL.transferring || SERIAL.clock != SERIAL_CLOCK_INTERNAL) {
    SERIAL.next_intr_ticks = INVALID_TICKS;
    calculate_next_intr();
    return;
  }

  /* Should only be called when receiving a new byte. */
  assert(SERIAL.tick_count == 0);
  assert(SERIAL.transferred_bits == 0);
  SERIAL.next_intr_ticks =
      SERIAL.sync_ticks +
      SERIAL_TICKS * (CPU_SPEED.speed == SPEED_NORMAL ? 8 : 4);
  calculate_next_intr();
}

void Emulator::serial_synchronize() {
  if (TICKS > SERIAL.sync_ticks) {
    Ticks delta_ticks = TICKS - SERIAL.sync_ticks;

    if (UNLIKELY(SERIAL.transferring &&
                 SERIAL.clock == SERIAL_CLOCK_INTERNAL)) {
      Ticks cpu_tick = state_.cpu_tick;
      for (; delta_ticks > 0; delta_ticks -= cpu_tick) {
        SERIAL.tick_count += cpu_tick;
        if (VALUE_WRAPPED(SERIAL.tick_count, SERIAL_TICKS)) {
          /* Since we're never connected to another device, always shift in
           * 0xff. */
          SERIAL.sb = (SERIAL.sb << 1) | 1;
          SERIAL.transferred_bits++;
          if (VALUE_WRAPPED(SERIAL.transferred_bits, 8)) {
            SERIAL.transferring = 0;
            INTR.new_if |= IF_SERIAL;
            SERIAL.sync_ticks = TICKS - delta_ticks;
            calculate_next_serial_intr();
          }
        } else if (UNLIKELY(SERIAL.tick_count == 0 &&
                            SERIAL.transferred_bits == 0)) {
          INTR.if_ |= (INTR.new_if & IF_SERIAL);
        }
      }
    }
    SERIAL.sync_ticks = TICKS;
  }
}

void Emulator::tick() {
  INTR.if_ = INTR.new_if;
  TICKS += state_.cpu_tick;
}

u8 Emulator::read_u8_tick(Address addr) {
  tick();
  return read_u8(addr);
}

u16 Emulator::read_u16_tick(Address addr) {
  u8 lo = read_u8_tick(addr);
  u8 hi = read_u8_tick(addr + 1);
  return (hi << 8) | lo;
}

void Emulator::write_u8_tick(Address addr, u8 value) {
  tick();
  write_u8(addr, value);
}

void Emulator::write_u16_tick(Address addr, u16 value) {
  write_u8_tick(addr + 1, value >> 8);
  write_u8_tick(addr, (u8)value);
}

u16 Emulator::get_af_reg() {
  return (REG.A << 8) | pack(REG.F.Z, CPU_FLAG_Z) | pack(REG.F.N, CPU_FLAG_N) |
         pack(REG.F.H, CPU_FLAG_H) | pack(REG.F.C, CPU_FLAG_C);
}

void Emulator::set_af_reg(u16 af) {
  REG.A = af >> 8;
  REG.F.Z = unpack(af, CPU_FLAG_Z);
  REG.F.N = unpack(af, CPU_FLAG_N);
  REG.F.H = unpack(af, CPU_FLAG_H);
  REG.F.C = unpack(af, CPU_FLAG_C);
}

#define TICK tick()
#define RA REG.A
#define RSP REG.SP
#define FZ REG.F.Z
#define FC REG.F.C
#define FH REG.F.H
#define FN REG.F.N
#define FZ_EQ0(X) FZ = (u8)(X) == 0
#define SHIFT_FLAGS FZ_EQ0(u); FN = FH = 0
#define MASK8(X) ((X) & 0xf)
#define MASK16(X) ((X) & 0xfff)
#define READ8(X) read_u8_tick(X)
#define READ16(X) read_u16_tick(X)
#define WRITE8(X, V) write_u8_tick(X, V)
#define WRITE16(X, V) write_u16_tick(X, V)
#define READ_N (new_pc += 1, READ8(REG.PC))
#define READ_NN (new_pc += 2, READ16(REG.PC))
#define READMR(MR) READ8(REG.MR)
#define WRITEMR(MR, V) WRITE8(REG.MR, V)
#define BASIC_OP_R(R, OP) u = REG.R; OP; REG.R = u
#define BASIC_OP_MR(MR, OP) u = READMR(MR); OP; WRITEMR(MR, u)
#define FC_ADD(X, Y) FC = ((X) + (Y) > 0xff)
#define FH_ADD(X, Y) FH = (MASK8(X) + MASK8(Y) > 0xf)
#define FCH_ADD(X, Y) FC_ADD(X, Y); FH_ADD(X, Y)
#define FC_ADD16(X, Y) FC = ((X) + (Y) > 0xffff)
#define FH_ADD16(X, Y) FH = (MASK16(X) + MASK16(Y) > 0xfff)
#define FCH_ADD16(X, Y) FC_ADD16(X, Y); FH_ADD16(X, Y)
#define ADD_FLAGS(X, Y) FZ_EQ0((X) + (Y)); FN = 0; FCH_ADD(X, Y)
#define ADD_FLAGS16(X, Y) FN = 0; FCH_ADD16(X, Y)
#define ADD_SP_FLAGS(Y) FZ = FN = 0; FCH_ADD((u8)RSP, (u8)(Y))
#define ADD_R(R) ADD_FLAGS(RA, REG.R); RA += REG.R
#define ADD_MR(MR) u = READMR(MR); ADD_FLAGS(RA, u); RA += u
#define ADD_N u = READ_N; ADD_FLAGS(RA, u); RA += u
#define ADD_HL_RR(RR) TICK; ADD_FLAGS16(REG.HL, REG.RR); REG.HL += REG.RR
#define ADD_SP_N s = (s8)READ_N; ADD_SP_FLAGS(s); RSP += s; TICK; TICK
#define FC_ADC(X, Y, C) FC = ((X) + (Y) + (C) > 0xff)
#define FH_ADC(X, Y, C) FH = (MASK8(X) + MASK8(Y) + C > 0xf)
#define FCH_ADC(X, Y, C) FC_ADC(X, Y, C); FH_ADC(X, Y, C)
#define ADC_FLAGS(X, Y, C) FZ_EQ0((X) + (Y) + (C)); FN = 0; FCH_ADC(X, Y, C)
#define ADC_R(R) u = REG.R; c = FC; ADC_FLAGS(RA, u, c); RA += u + c
#define ADC_MR(MR) u = READMR(MR); c = FC; ADC_FLAGS(RA, u, c); RA += u + c
#define ADC_N u = READ_N; c = FC; ADC_FLAGS(RA, u, c); RA += u + c
#define AND_FLAGS FZ_EQ0(RA); FH = 1; FN = FC = 0
#define AND_R(R) RA &= REG.R; AND_FLAGS
#define AND_MR(MR) RA &= READMR(MR); AND_FLAGS
#define AND_N RA &= READ_N; AND_FLAGS
#define BIT_FLAGS(BIT, X) FZ_EQ0((X) & (1 << (BIT))); FN = 0; FH = 1
#define BIT_R(BIT, R) u = REG.R; BIT_FLAGS(BIT, u)
#define BIT_MR(BIT, MR) u = READMR(MR); BIT_FLAGS(BIT, u)
#define CALL(X) TICK; RSP -= 2; WRITE16(RSP, new_pc); new_pc = X
#define CALL_NN u16 = READ_NN; CALL(u16)
#define CALL_F_NN(COND) u16 = READ_NN; if (COND) { CALL(u16); }
#define CCF FC ^= 1; FN = FH = 0
#define CP_FLAGS(X, Y) FZ_EQ0((X) - (Y)); FN = 1; FCH_SUB(X, Y)
#define CP_R(R) CP_FLAGS(RA, REG.R)
#define CP_N u = READ_N; CP_FLAGS(RA, u)
#define CP_MR(MR) u = READMR(MR); CP_FLAGS(RA, u)
#define CPL RA = ~RA; FN = FH = 1
#define DAA                              \
  do {                                   \
    u = 0;                               \
    if (FH || (!FN && (RA & 0xf) > 9)) { \
      u = 6;                             \
    }                                    \
    if (FC || (!FN && RA > 0x99)) {      \
      u |= 0x60;                         \
      FC = 1;                            \
    }                                    \
    RA += FN ? -u : u;                   \
    FZ_EQ0(RA);                          \
    FH = 0;                              \
  } while (0)
#define DEC u--
#define DEC_FLAGS FZ_EQ0(u); FN = 1; FH = MASK8(u) == 0xf
#define DEC_R(R) BASIC_OP_R(R, DEC); DEC_FLAGS
#define DEC_RR(RR) REG.RR--; TICK
#define DEC_MR(MR) BASIC_OP_MR(MR, DEC); DEC_FLAGS
#define DI INTR.state = CPU_STATE_NORMAL; INTR.ime = false;
#define EI INTR.state = CPU_STATE_ENABLE_IME;
#define HALT                                   \
  if (INTR.ime) {                              \
    INTR.state = CPU_STATE_HALT;               \
  } else if (INTR.ie & INTR.new_if & IF_ALL) { \
    INTR.state = CPU_STATE_HALT_BUG;           \
  } else {                                     \
    INTR.state = CPU_STATE_HALT_DI;            \
  }
#define INC u++
#define INC_FLAGS FZ_EQ0(u); FN = 0; FH = MASK8(u) == 0
#define INC_R(R) BASIC_OP_R(R, INC); INC_FLAGS
#define INC_RR(RR) REG.RR++; TICK
#define INC_MR(MR) BASIC_OP_MR(MR, INC); INC_FLAGS
#define JP_F_NN(COND) u16 = READ_NN; if (COND) { new_pc = u16; TICK; }
#define JP_RR(RR) new_pc = REG.RR
#define JP_NN new_pc = READ_NN; TICK
#define JR new_pc += s; TICK
#define JR_F_N(COND) s = READ_N; if (COND) { JR; }
#define JR_N s = READ_N; JR
#define LD_R_R(RD, RS) REG.RD = REG.RS
#define LD_R_N(R) REG.R = READ_N
#define LD_RR_RR(RRD, RRS) REG.RRD = REG.RRS; TICK
#define LD_RR_NN(RR) REG.RR = READ_NN
#define LD_R_MR(R, MR) REG.R = READMR(MR)
#define LD_R_MN(R) REG.R = READ8(READ_NN)
#define LD_MR_R(MR, R) WRITEMR(MR, REG.R)
#define LD_MR_N(MR) WRITEMR(MR, READ_N)
#define LD_MN_R(R) WRITE8(READ_NN, REG.R)
#define LD_MFF00_N_R(R) WRITE8(0xFF00 + READ_N, RA)
#define LD_MFF00_R_R(R1, R2) WRITE8(0xFF00 + REG.R1, REG.R2)
#define LD_R_MFF00_N(R) REG.R = READ8(0xFF00 + READ_N)
#define LD_R_MFF00_R(R1, R2) REG.R1 = READ8(0xFF00 + REG.R2)
#define LD_MNN_SP u16 = READ_NN; WRITE16(u16, RSP)
#define LD_HL_SP_N s = (s8)READ_N; ADD_SP_FLAGS(s); REG.HL = RSP + s; TICK
#define OR_FLAGS FZ_EQ0(RA); FN = FH = FC = 0
#define OR_R(R) RA |= REG.R; OR_FLAGS
#define OR_MR(MR) RA |= READMR(MR); OR_FLAGS
#define OR_N RA |= READ_N; OR_FLAGS
#define POP_RR(RR) REG.RR = READ16(RSP); RSP += 2
#define POP_AF set_af_reg(READ16(RSP)); RSP += 2
#define PUSH_RR(RR) TICK; RSP -= 2; WRITE16(RSP, REG.RR)
#define PUSH_AF TICK; RSP -= 2; WRITE16(RSP, get_af_reg())
#define RES(BIT) u &= ~(1 << (BIT))
#define RES_R(BIT, R) BASIC_OP_R(R, RES(BIT))
#define RES_MR(BIT, MR) BASIC_OP_MR(MR, RES(BIT))
#define RET new_pc = READ16(RSP); RSP += 2; TICK
#define RET_F(COND) TICK; if (COND) { RET; }
#define RETI INTR.state = CPU_STATE_NORMAL; INTR.ime = true; RET
#define RL c = (u >> 7) & 1; u = (u << 1) | FC; FC = c
#define RLA BASIC_OP_R(A, RL); FZ = FN = FH = 0
#define RL_R(R) BASIC_OP_R(R, RL); SHIFT_FLAGS
#define RL_MR(MR) BASIC_OP_MR(MR, RL); SHIFT_FLAGS
#define RLC c = (u >> 7) & 1; u = (u << 1) | c; FC = c
#define RLCA BASIC_OP_R(A, RLC); FZ = FN = FH = 0
#define RLC_R(R) BASIC_OP_R(R, RLC); SHIFT_FLAGS
#define RLC_MR(MR) BASIC_OP_MR(MR, RLC); SHIFT_FLAGS
#define RR c = u & 1; u = (FC << 7) | (u >> 1); FC = c
#define RRA BASIC_OP_R(A, RR); FZ = FN = FH = 0
#define RR_R(R) BASIC_OP_R(R, RR); SHIFT_FLAGS
#define RR_MR(MR) BASIC_OP_MR(MR, RR); SHIFT_FLAGS
#define RRC c = u & 1; u = (c << 7) | (u >> 1); FC = c
#define RRCA BASIC_OP_R(A, RRC); FZ = FN = FH = 0
#define RRC_R(R) BASIC_OP_R(R, RRC); SHIFT_FLAGS
#define RRC_MR(MR) BASIC_OP_MR(MR, RRC); SHIFT_FLAGS
#define SCF FC = 1; FN = FH = 0
#define SET(BIT) u |= (1 << BIT)
#define SET_R(BIT, R) BASIC_OP_R(R, SET(BIT))
#define SET_MR(BIT, MR) BASIC_OP_MR(MR, SET(BIT))
#define SLA FC = (u >> 7) & 1; u <<= 1
#define SLA_R(R) BASIC_OP_R(R, SLA); SHIFT_FLAGS
#define SLA_MR(MR) BASIC_OP_MR(MR, SLA); SHIFT_FLAGS
#define SRA FC = u & 1; u = (s8)u >> 1
#define SRA_R(R) BASIC_OP_R(R, SRA); SHIFT_FLAGS
#define SRA_MR(MR) BASIC_OP_MR(MR, SRA); SHIFT_FLAGS
#define SRL FC = u & 1; u >>= 1
#define SRL_R(R) BASIC_OP_R(R, SRL); SHIFT_FLAGS
#define SRL_MR(MR) BASIC_OP_MR(MR, SRL); SHIFT_FLAGS
#define STOP INTR.state = CPU_STATE_STOP;
#define FC_SUB(X, Y) FC = ((int)(X) - (int)(Y) < 0)
#define FH_SUB(X, Y) FH = ((int)MASK8(X) - (int)MASK8(Y) < 0)
#define FCH_SUB(X, Y) FC_SUB(X, Y); FH_SUB(X, Y)
#define SUB_FLAGS(X, Y) FZ_EQ0((X) - (Y)); FN = 1; FCH_SUB(X, Y)
#define SUB_R(R) SUB_FLAGS(RA, REG.R); RA -= REG.R
#define SUB_MR(MR) u = READMR(MR); SUB_FLAGS(RA, u); RA -= u
#define SUB_N u = READ_N; SUB_FLAGS(RA, u); RA -= u
#define FC_SBC(X, Y, C) FC = ((int)(X) - (int)(Y) - (int)(C) < 0)
#define FH_SBC(X, Y, C) FH = ((int)MASK8(X) - (int)MASK8(Y) - (int)C < 0)
#define FCH_SBC(X, Y, C) FC_SBC(X, Y, C); FH_SBC(X, Y, C)
#define SBC_FLAGS(X, Y, C) FZ_EQ0((X) - (Y) - (C)); FN = 1; FCH_SBC(X, Y, C)
#define SBC_R(R) u = REG.R; c = FC; SBC_FLAGS(RA, u, c); RA -= u + c
#define SBC_MR(MR) u = READMR(MR); c = FC; SBC_FLAGS(RA, u, c); RA -= u + c
#define SBC_N u = READ_N; c = FC; SBC_FLAGS(RA, u, c); RA -= u + c
#define SWAP u = (u << 4) | (u >> 4)
#define SWAP_FLAGS FZ_EQ0(u); FN = FH = FC = 0
#define SWAP_R(R) BASIC_OP_R(R, SWAP); SWAP_FLAGS
#define SWAP_MR(MR) BASIC_OP_MR(MR, SWAP); SWAP_FLAGS
#define XOR_FLAGS FZ_EQ0(RA); FN = FH = FC = 0
#define XOR_R(R) RA ^= REG.R; XOR_FLAGS
#define XOR_MR(MR) RA ^= READMR(MR); XOR_FLAGS
#define XOR_N RA ^= READ_N; XOR_FLAGS

void Emulator::dispatch_interrupt() {
  bool was_halt = INTR.state >= CPU_STATE_HALT;
  if (!(INTR.ime || was_halt)) {
    return;
  }

  INTR.ime = false;
  INTR.state = CPU_STATE_NORMAL;

  /* Write MSB of PC. */
  RSP--; WRITE8(RSP, REG.PC >> 8);

  /* Now check which interrupt to raise, after having written the MSB of PC.
   * This behavior is needed to pass the ie_push mooneye-gb test. */
  u8 interrupt = INTR.new_if & INTR.ie;

  bool delay = false;
  u8 mask = 0;
  Address vector = 0;
  if (interrupt & IF_VBLANK) {
    HOOK(vblank_interrupt_i, PPU.frame);
    vector = 0x40;
    mask = IF_VBLANK;
  } else if (interrupt & IF_STAT) {
    HOOK(stat_interrupt_cccc, STAT.y_compare.irq ? 'Y' : '.',
         STAT.mode2.irq ? 'O' : '.', STAT.vblank.irq ? 'V' : '.',
         STAT.hblank.irq ? 'H' : '.');
    vector = 0x48;
    mask = IF_STAT;
  } else if (interrupt & IF_TIMER) {
    HOOK0(timer_interrupt_v);
    vector = 0x50;
    mask = IF_TIMER;
    delay = was_halt;
  } else if (interrupt & IF_SERIAL) {
    HOOK0(serial_interrupt_v);
    vector = 0x58;
    mask = IF_SERIAL;
  } else if (interrupt & IF_JOYPAD) {
    HOOK0(joypad_interrupt_v);
    vector = 0x60;
    mask = IF_JOYPAD;
  } else {
    /* Interrupt was canceled. */
    vector = 0;
    mask = 0;
  }

  INTR.new_if &= ~mask;

  /* Now write the LSB of PC. */
  RSP--; WRITE8(RSP, REG.PC);
  REG.PC = vector;

  if (delay) {
    tick();
  }
  tick();
  tick();
}

void Emulator::execute_instruction() {
  u8 opcode = 0;
  s8 s;
  u8 u, c;
  u16 u16;
  Address new_pc;

  if (UNLIKELY(TICKS >= state_.next_intr_ticks)) {
    if (TICKS >= TIMER.next_intr_ticks) {
      timer_synchronize();
    }
    if (TICKS >= SERIAL.next_intr_ticks) {
      serial_synchronize();
    }
    if (TICKS >= PPU.next_intr_ticks) {
      ppu_synchronize();
    }
  }

  bool should_dispatch = false;

  if (LIKELY(INTR.state == CPU_STATE_NORMAL)) {
    should_dispatch = INTR.ime && (INTR.new_if & INTR.ie) != 0;
    opcode = read_u8_tick(REG.PC);
  } else {
    switch (INTR.state) {
      case CPU_STATE_NORMAL:
        assert(0);

      case CPU_STATE_STOP:
        should_dispatch = INTR.ime && (INTR.new_if & INTR.ie) != 0;
        if (UNLIKELY(!should_dispatch)) {
          // TODO(binji): proper timing of speed switching.
          if (CPU_SPEED.switching) {
            intr_synchronize();
            CPU_SPEED.switching = false;
            CPU_SPEED.speed = static_cast<Speed>(static_cast<int>(CPU_SPEED.speed) ^ 1);
            INTR.state = CPU_STATE_NORMAL;
            if (CPU_SPEED.speed == SPEED_NORMAL) {
              state_.cpu_tick = CPU_TICK;
              HOOK(speed_switch_i, 1);
            } else {
              state_.cpu_tick = CPU_2X_TICK;
              HOOK(speed_switch_i, 2);
            }
          } else {
            TICKS += CPU_TICK;
            return;
          }
        }
        opcode = read_u8_tick(REG.PC);
        break;

      case CPU_STATE_ENABLE_IME:
        should_dispatch = INTR.ime && (INTR.new_if & INTR.ie) != 0;
        INTR.ime = true;
        INTR.state = CPU_STATE_NORMAL;
        opcode = read_u8_tick(REG.PC);
        break;

      case CPU_STATE_HALT_BUG:
        /* When interrupts are disabled during a HALT, the following byte will
         * be duplicated when decoding. */
        should_dispatch = INTR.ime && (INTR.new_if & INTR.ie) != 0;
        opcode = read_u8(REG.PC);
        REG.PC--;
        INTR.state = CPU_STATE_NORMAL;
        break;

      case CPU_STATE_HALT:
        should_dispatch = (INTR.new_if & INTR.ie) != 0;
        tick();
        if (UNLIKELY(should_dispatch)) {
          intr_synchronize();
          dispatch_interrupt();
        }
        return;

      case CPU_STATE_HALT_DI:
        should_dispatch = (INTR.new_if & INTR.ie) != 0;
        opcode = read_u8_tick(REG.PC);
        if (UNLIKELY(should_dispatch)) {
          HOOK0(interrupt_during_halt_di_v);
          INTR.state = CPU_STATE_NORMAL;
          should_dispatch = false;
          break;
        }
        return;
    }
  }

  if (UNLIKELY(should_dispatch)) {
    intr_synchronize();
    dispatch_interrupt();
    return;
  }

#define REG_OPS(code, name)            \
  case code + 0: name##_R(B); break;   \
  case code + 1: name##_R(C); break;   \
  case code + 2: name##_R(D); break;   \
  case code + 3: name##_R(E); break;   \
  case code + 4: name##_R(H); break;   \
  case code + 5: name##_R(L); break;   \
  case code + 6: name##_MR(HL); break; \
  case code + 7: name##_R(A); break;
#define REG_OPS_N(code, name, N)          \
  case code + 0: name##_R(N, B); break;   \
  case code + 1: name##_R(N, C); break;   \
  case code + 2: name##_R(N, D); break;   \
  case code + 3: name##_R(N, E); break;   \
  case code + 4: name##_R(N, H); break;   \
  case code + 5: name##_R(N, L); break;   \
  case code + 6: name##_MR(N, HL); break; \
  case code + 7: name##_R(N, A); break;
#define LD_R_OPS(code, R) REG_OPS_N(code, LD_R, R)

  HOOK(exec_op_ai, REG.PC, opcode);
  new_pc = ++REG.PC;

  switch (opcode) {
    case 0x00: break;
    case 0x01: LD_RR_NN(BC); break;
    case 0x02: LD_MR_R(BC, A); break;
    case 0x03: INC_RR(BC); break;
    case 0x04: INC_R(B); break;
    case 0x05: DEC_R(B); break;
    case 0x06: LD_R_N(B); break;
    case 0x07: RLCA; break;
    case 0x08: LD_MNN_SP; break;
    case 0x09: ADD_HL_RR(BC); break;
    case 0x0a: LD_R_MR(A, BC); break;
    case 0x0b: DEC_RR(BC); break;
    case 0x0c: INC_R(C); break;
    case 0x0d: DEC_R(C); break;
    case 0x0e: LD_R_N(C); break;
    case 0x0f: RRCA; break;
    case 0x10: STOP; break;
    case 0x11: LD_RR_NN(DE); break;
    case 0x12: LD_MR_R(DE, A); break;
    case 0x13: INC_RR(DE); break;
    case 0x14: INC_R(D); break;
    case 0x15: DEC_R(D); break;
    case 0x16: LD_R_N(D); break;
    case 0x17: RLA; break;
    case 0x18: JR_N; break;
    case 0x19: ADD_HL_RR(DE); break;
    case 0x1a: LD_R_MR(A, DE); break;
    case 0x1b: DEC_RR(DE); break;
    case 0x1c: INC_R(E); break;
    case 0x1d: DEC_R(E); break;
    case 0x1e: LD_R_N(E); break;
    case 0x1f: RRA; break;
    case 0x20: JR_F_N(!FZ); break;
    case 0x21: LD_RR_NN(HL); break;
    case 0x22: LD_MR_R(HL, A); REG.HL++; break;
    case 0x23: INC_RR(HL); break;
    case 0x24: INC_R(H); break;
    case 0x25: DEC_R(H); break;
    case 0x26: LD_R_N(H); break;
    case 0x27: DAA; break;
    case 0x28: JR_F_N(FZ); break;
    case 0x29: ADD_HL_RR(HL); break;
    case 0x2a: LD_R_MR(A, HL); REG.HL++; break;
    case 0x2b: DEC_RR(HL); break;
    case 0x2c: INC_R(L); break;
    case 0x2d: DEC_R(L); break;
    case 0x2e: LD_R_N(L); break;
    case 0x2f: CPL; break;
    case 0x30: JR_F_N(!FC); break;
    case 0x31: LD_RR_NN(SP); break;
    case 0x32: LD_MR_R(HL, A); REG.HL--; break;
    case 0x33: INC_RR(SP); break;
    case 0x34: INC_MR(HL); break;
    case 0x35: DEC_MR(HL); break;
    case 0x36: LD_MR_N(HL); break;
    case 0x37: SCF; break;
    case 0x38: JR_F_N(FC); break;
    case 0x39: ADD_HL_RR(SP); break;
    case 0x3a: LD_R_MR(A, HL); REG.HL--; break;
    case 0x3b: DEC_RR(SP); break;
    case 0x3c: INC_R(A); break;
    case 0x3d: DEC_R(A); break;
    case 0x3e: LD_R_N(A); break;
    case 0x3f: CCF; break;
    LD_R_OPS(0x40, B)
    LD_R_OPS(0x48, C)
    LD_R_OPS(0x50, D)
    LD_R_OPS(0x58, E)
    LD_R_OPS(0x60, H)
    LD_R_OPS(0x68, L)
    case 0x70: LD_MR_R(HL, B); break;
    case 0x71: LD_MR_R(HL, C); break;
    case 0x72: LD_MR_R(HL, D); break;
    case 0x73: LD_MR_R(HL, E); break;
    case 0x74: LD_MR_R(HL, H); break;
    case 0x75: LD_MR_R(HL, L); break;
    case 0x76: HALT; break;
    case 0x77: LD_MR_R(HL, A); break;
    LD_R_OPS(0x78, A)
    REG_OPS(0x80, ADD)
    REG_OPS(0x88, ADC)
    REG_OPS(0x90, SUB)
    REG_OPS(0x98, SBC)
    REG_OPS(0xa0, AND)
    REG_OPS(0xa8, XOR)
    REG_OPS(0xb0, OR)
    REG_OPS(0xb8, CP)
    case 0xc0: RET_F(!FZ); break;
    case 0xc1: POP_RR(BC); break;
    case 0xc2: JP_F_NN(!FZ); break;
    case 0xc3: JP_NN; break;
    case 0xc4: CALL_F_NN(!FZ); break;
    case 0xc5: PUSH_RR(BC); break;
    case 0xc6: ADD_N; break;
    case 0xc7: CALL(0x00); break;
    case 0xc8: RET_F(FZ); break;
    case 0xc9: RET; break;
    case 0xca: JP_F_NN(FZ); break;
    case 0xcb: {
      new_pc += 1;
      u8 cb = read_u8_tick(REG.PC);
      HOOK(exec_cb_op_i, cb);
      switch (cb) {
        REG_OPS(0x00, RLC)
        REG_OPS(0x08, RRC)
        REG_OPS(0x10, RL)
        REG_OPS(0x18, RR)
        REG_OPS(0x20, SLA)
        REG_OPS(0x28, SRA)
        REG_OPS(0x30, SWAP)
        REG_OPS(0x38, SRL)
        REG_OPS_N(0x40, BIT, 0)
        REG_OPS_N(0x48, BIT, 1)
        REG_OPS_N(0x50, BIT, 2)
        REG_OPS_N(0x58, BIT, 3)
        REG_OPS_N(0x60, BIT, 4)
        REG_OPS_N(0x68, BIT, 5)
        REG_OPS_N(0x70, BIT, 6)
        REG_OPS_N(0x78, BIT, 7)
        REG_OPS_N(0x80, RES, 0)
        REG_OPS_N(0x88, RES, 1)
        REG_OPS_N(0x90, RES, 2)
        REG_OPS_N(0x98, RES, 3)
        REG_OPS_N(0xa0, RES, 4)
        REG_OPS_N(0xa8, RES, 5)
        REG_OPS_N(0xb0, RES, 6)
        REG_OPS_N(0xb8, RES, 7)
        REG_OPS_N(0xc0, SET, 0)
        REG_OPS_N(0xc8, SET, 1)
        REG_OPS_N(0xd0, SET, 2)
        REG_OPS_N(0xd8, SET, 3)
        REG_OPS_N(0xe0, SET, 4)
        REG_OPS_N(0xe8, SET, 5)
        REG_OPS_N(0xf0, SET, 6)
        REG_OPS_N(0xf8, SET, 7)
      }
      break;
    }
    case 0xcc: CALL_F_NN(FZ); break;
    case 0xcd: CALL_NN; break;
    case 0xce: ADC_N; break;
    case 0xcf: CALL(0x08); break;
    case 0xd0: RET_F(!FC); break;
    case 0xd1: POP_RR(DE); break;
    case 0xd2: JP_F_NN(!FC); break;
    case 0xd4: CALL_F_NN(!FC); break;
    case 0xd5: PUSH_RR(DE); break;
    case 0xd6: SUB_N; break;
    case 0xd7: CALL(0x10); break;
    case 0xd8: RET_F(FC); break;
    case 0xd9: RETI; break;
    case 0xda: JP_F_NN(FC); break;
    case 0xdc: CALL_F_NN(FC); break;
    case 0xde: SBC_N; break;
    case 0xdf: CALL(0x18); break;
    case 0xe0: LD_MFF00_N_R(A); break;
    case 0xe1: POP_RR(HL); break;
    case 0xe2: LD_MFF00_R_R(C, A); break;
    case 0xe5: PUSH_RR(HL); break;
    case 0xe6: AND_N; break;
    case 0xe7: CALL(0x20); break;
    case 0xe8: ADD_SP_N; break;
    case 0xe9: JP_RR(HL); break;
    case 0xea: LD_MN_R(A); break;
    case 0xee: XOR_N; break;
    case 0xef: CALL(0x28); break;
    case 0xf0: LD_R_MFF00_N(A); break;
    case 0xf1: POP_AF; break;
    case 0xf2: LD_R_MFF00_R(A, C); break;
    case 0xf3: DI; break;
    case 0xf5: PUSH_AF; break;
    case 0xf6: OR_N; break;
    case 0xf7: CALL(0x30); break;
    case 0xf8: LD_HL_SP_N; break;
    case 0xf9: LD_RR_RR(SP, HL); break;
    case 0xfa: LD_R_MN(A); break;
    case 0xfb: EI; break;
    case 0xfe: CP_N; break;
    case 0xff: CALL(0x38); break;
    default:
      state_.event |= EMULATOR_EVENT_INVALID_OPCODE;
      break;
  }
  REG.PC = new_pc;
}

void Emulator::step_internal() {
  if (HDMA.state == DMA_INACTIVE) {
    if (HOOK0_false(emulator_step)) {
      return;
    }
    execute_instruction();
  } else {
    tick();
    hdma_copy_byte();
    hdma_copy_byte();
  }
}

EmulatorEvent Emulator::run_until(Ticks until_ticks) {
  AudioBuffer* ab = &audio_buffer_;
  if (state_.event & EMULATOR_EVENT_AUDIO_BUFFER_FULL) {
    ab->position = ab->data;
  }
  check_joyp_intr();
  state_.event = 0;

  u64 frames_left = ab->frames - ab->audio_buffer_get_frames();
  Ticks max_audio_ticks =
      APU.sync_ticks +
      (u32)DIV_CEIL(frames_left * CPU_TICKS_PER_SECOND, ab->frequency);
  Ticks check_ticks = MIN(until_ticks, max_audio_ticks);
  while (state_.event == 0 && TICKS < check_ticks) {
    step_internal();
  }
  if (TICKS >= max_audio_ticks) {
    state_.event |= EMULATOR_EVENT_AUDIO_BUFFER_FULL;
  }
  if (TICKS >= until_ticks) {
    state_.event |= EMULATOR_EVENT_UNTIL_TICKS;
  }
  apu_synchronize();
  return state_.event;
}

EmulatorEvent Emulator::step() {
  return run_until(TICKS + 1);
}

static Result validate_header_checksum(CartInfo* cart_info) {
  u8 checksum = 0;
  size_t i = 0;
  for (i = HEADER_CHECKSUM_RANGE_START; i <= HEADER_CHECKSUM_RANGE_END; ++i) {
    checksum = checksum - cart_info->data[i] - 1;
  }
  return checksum == cart_info->data[HEADER_CHECKSUM_ADDR] ? OK : ERROR;
}

static const char* get_result_string(Result value) {
  static const char* s_strings[] = {[OK] = "OK", [ERROR] = "ERROR"};
  return get_enum_string(s_strings, ARRAY_SIZE(s_strings), value);
}

static void log_cart_info(CartInfo* cart_info) {
  char* title_start = (char*)cart_info->data + TITLE_START_ADDR;
  char* title_end = reinterpret_cast<char*>(memchr(title_start, '\0', TITLE_MAX_LENGTH));
  int title_length =
      (int)(title_end ? title_end - title_start : TITLE_MAX_LENGTH);
  printf("title: \"%.*s\"\n", title_length, title_start);
  printf("cgb flag: %s\n", get_cgb_flag_string(cart_info->cgb_flag));
  printf("sgb flag: %s\n", get_sgb_flag_string(cart_info->sgb_flag));
  printf("cart type: %s\n", get_cart_type_string(cart_info->cart_type));
  printf("rom size: %s\n", get_rom_size_string(cart_info->rom_size));
  printf("ext ram size: %s\n",
         get_ext_ram_size_string(cart_info->ext_ram_size));
  printf("header checksum: 0x%02x [%s]\n",
         cart_info->data[HEADER_CHECKSUM_ADDR],
         get_result_string(validate_header_checksum(cart_info)));
}

Result Emulator::init_audio_buffer(u32 frequency, u32 frames) {
  AudioBuffer* audio_buffer = &this->audio_buffer_;
  audio_buffer->frames = frames;
  size_t buffer_size =
      (frames + AUDIO_BUFFER_EXTRA_FRAMES) * SOUND_OUTPUT_COUNT;
  audio_buffer->data = reinterpret_cast<u8*>(xmalloc(buffer_size));
  CHECK_MSG(audio_buffer->data != NULL, "Audio buffer allocation failed.\n");
  audio_buffer->end = audio_buffer->data + buffer_size;
  audio_buffer->position = audio_buffer->data;
  audio_buffer->frequency = frequency;
  return OK;
  ON_ERROR_RETURN;
}

static u32 random_u32(u32* state) {
  /* xorshift32: https://en.wikipedia.org/wiki/Xorshift */
  u32 x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static void randomize_buffer(u32* seed, u8* buffer, u32 size) {
  while (size >= sizeof(u32)) {
    u32 x = random_u32(seed);
    memcpy(buffer, &x, sizeof(x));
    buffer += sizeof(u32);
    size -= sizeof(u32);
  }
  if (size > 0) {
    u32 x = random_u32(seed);
    switch (size) {
      case 3: *buffer++ = x & 0xff; x >>= 8; break;
      case 2: *buffer++ = x & 0xff; x >>= 8; break;
      case 1: *buffer++ = x & 0xff; x >>= 8; break;
    }
  }
}

Result Emulator::init_emulator(const EmulatorInit* init) {
  static u8 s_initial_wave_ram[WAVE_RAM_SIZE] = {
      0x60, 0x0d, 0xda, 0xdd, 0x50, 0x0f, 0xad, 0xed,
      0xc0, 0xde, 0xf0, 0x0d, 0xbe, 0xef, 0xfe, 0xed,
  };
  if(!(SUCCESS(get_cart_infos()))) return ERROR;
  log_cart_info(cart_info_);
  MMAP_STATE.rom_base[0] = 0;
  MMAP_STATE.rom_base[1] = 1 << ROM_BANK_SHIFT;
  IS_CGB = !init->force_dmg && (cart_info_->cgb_flag == CGB_FLAG_SUPPORTED ||
                                cart_info_->cgb_flag == CGB_FLAG_REQUIRED);
  IS_SGB = !init->force_dmg && !IS_CGB &&
           cart_info_->sgb_flag == SGB_FLAG_SUPPORTED;
  set_af_reg(0xb0);
  REG.A = IS_CGB ? 0x11 : 0x01;
  REG.BC = 0x0013;
  REG.DE = 0x00d8;
  REG.HL = 0x014d;
  REG.SP = 0xfffe;
  REG.PC = 0x0100;
  INTR.ime = false;
  TIMER.div_counter = 0xAC00;
  TIMER.next_intr_ticks = SERIAL.next_intr_ticks = state_.next_intr_ticks =
      INVALID_TICKS;
  WRAM.offset = 0x1000;
  /* Enable apu first, so subsequent writes succeed. */
  write_apu(APU_NR52_ADDR, 0xf1);
  write_apu(APU_NR11_ADDR, 0x80);
  write_apu(APU_NR12_ADDR, 0xf3);
  write_apu(APU_NR14_ADDR, 0x80);
  write_apu(APU_NR50_ADDR, 0x77);
  write_apu(APU_NR51_ADDR, 0xf3);
  APU.initialized = true;
  memcpy(&WAVE.ram, s_initial_wave_ram, WAVE_RAM_SIZE);
  /* Turn down the volume on channel1, it is playing by default (because of the
   * GB startup sound), but we don't want to hear it when starting the
   * emulator. */
  CHANNEL1.envelope.volume = 0;
  write_io(IO_LCDC_ADDR, 0x91);
  write_io(IO_SCY_ADDR, 0x00);
  write_io(IO_SCX_ADDR, 0x00);
  write_io(IO_LYC_ADDR, 0x00);
  write_io(IO_BGP_ADDR, 0xfc);
  write_io(IO_OBP0_ADDR, 0xff);
  write_io(IO_OBP1_ADDR, 0xff);
  write_io(IO_IF_ADDR, 0x1);
  write_io(IO_IE_ADDR, 0x0);
  HDMA.blocks = 0xff;

  /* Set initial DMG/SGB palettes */
  set_builtin_palette(init->builtin_palette);

  /* Set up cgb color curve */
  cgb_color_curve_ = init->cgb_color_curve;

  /* Set initial CGB palettes to white. */
  int pal_index;
  for (pal_index = 0; pal_index < 2; ++pal_index) {
    ColorPalettes* palette = pal_index == 0 ? &PPU.bgcp : &PPU.obcp;
    int i;
    for (i = 0; i < 32; ++i) {
      palette->palettes[i >> 2].color[i & 3] = RGBA_WHITE;
      palette->data[i * 2] = 0xff;
      palette->data[i * 2 + 1] = 0x7f;
    }
  }

  /* Randomize RAM */
  u32 random_seed = init->random_seed;
  state_.random_seed = random_seed;
  randomize_buffer(&random_seed, state_.ext_ram.data, EXT_RAM_MAX_SIZE);
  randomize_buffer(&random_seed, state_.wram.data, WORK_RAM_SIZE);
  randomize_buffer(&random_seed, state_.hram, HIGH_RAM_SIZE);

  state_.cpu_tick = CPU_TICK;
  calculate_next_ppu_intr();
  return OK;
}

void Emulator::set_joypad_buttons(JoypadButtons* buttons) {
  JOYP.buttons = *buttons;
}

void Emulator::set_joypad_callback(JoypadCallback callback, void* user_data) {
  joypad_info_.callback = callback;
  joypad_info_.user_data = user_data;
}

JoypadCallbackInfo Emulator::get_joypad_callback() {
  return joypad_info_;
}

void Emulator::set_config(const EmulatorConfig* config) {
  this->config_ = *config;
}

EmulatorConfig Emulator::get_config() {
  return config_;
}

FrameBuffer* Emulator::get_frame_buffer() {
  return &frame_buffer_;
}

SgbFrameBuffer* Emulator::get_sgb_frame_buffer() {
  return &sgb_frame_buffer_;
}

AudioBuffer* Emulator::get_audio_buffer() {
  return &audio_buffer_;
}

Ticks Emulator::get_ticks() {
  return TICKS;
}

u32 Emulator::get_ppu_frame() {
  return PPU.frame;
}

u32 AudioBuffer::audio_buffer_get_frames() {
  return (position - data) / SOUND_OUTPUT_COUNT;
}

void Emulator::set_bw_palette(PaletteType type,
                             const PaletteRGBA* palette) {
  color_to_rgba_[type] = *palette;
  update_bw_palette_rgba(type);
}

void Emulator::set_all_bw_palettes(const PaletteRGBA* palette) {
  color_to_rgba_[PALETTE_TYPE_BGP] = *palette;
  color_to_rgba_[PALETTE_TYPE_OBP0] = *palette;
  color_to_rgba_[PALETTE_TYPE_OBP1] = *palette;
}

Result Emulator::set_rom_file_data(const FileData* file_data) {
  CHECK_MSG(file_data->size > 0, "File is empty.\n");
  CHECK_MSG((file_data->size & (MINIMUM_ROM_SIZE - 1)) == 0,
            "File size (%ld) should be a multiple of minimum rom size (%ld).\n",
            (long)file_data->size, (long)MINIMUM_ROM_SIZE);
  this->file_data_ = *file_data;
  return OK;
  ON_ERROR_RETURN;
}

bool Emulator::was_ext_ram_updated() {
  bool result = state_.ext_ram_updated;
  state_.ext_ram_updated = false;
  return result;
}

void emulator_init_state_file_data(FileData* file_data) {
  file_data->size = sizeof(EmulatorState);
  file_data->data = reinterpret_cast<u8*>(xmalloc(file_data->size));
}

void Emulator::init_ext_ram_file_data(FileData* file_data) {
  file_data->size = EXT_RAM.size;
  file_data->data = reinterpret_cast<u8*>(xmalloc(file_data->size));
}

Result Emulator::read_state(const FileData* file_data) {
  if(!(file_data->size == sizeof(EmulatorState))) return ERROR;
            // "save state file is wrong size: %ld, expected %ld.\n",
            // (long)file_data->size, (long)sizeof(EmulatorState));
  EmulatorState* new_state = (EmulatorState*)file_data->data;
  if(!(new_state->header == SAVE_STATE_HEADER)) return ERROR;
            // "header mismatch: %u, expected %u.\n", new_state->header,
            // SAVE_STATE_HEADER);
  memcpy(&state_, new_state, sizeof(EmulatorState));
  set_cart_info(state_.cart_info_index);

  if (IS_SGB) {
    set_bw_palette(PALETTE_TYPE_OBP0, &SGB.screen_pal[0]);
    set_bw_palette(PALETTE_TYPE_OBP1, &SGB.screen_pal[0]);
  }
  update_bw_palette_rgba(PALETTE_TYPE_BGP);
  update_bw_palette_rgba(PALETTE_TYPE_OBP0);
  update_bw_palette_rgba(PALETTE_TYPE_OBP1);
  return OK;
}

Result Emulator::write_state(FileData* file_data) {
  CHECK(file_data->size >= sizeof(EmulatorState));
  state_.header = SAVE_STATE_HEADER;
  memcpy(file_data->data, &state_, file_data->size);
  return OK;
  ON_ERROR_RETURN;
}

Result Emulator::read_ext_ram(const FileData* file_data) {
  if (EXT_RAM.battery_type != BATTERY_TYPE_WITH_BATTERY)
    return OK;

  CHECK_MSG(file_data->size == EXT_RAM.size,
            "save file is wrong size: %ld, expected %ld.\n",
            (long)file_data->size, (long)EXT_RAM.size);
  memcpy(EXT_RAM.data, file_data->data, file_data->size);
  return OK;
  ON_ERROR_RETURN;
}

Result Emulator::write_ext_ram(FileData* file_data) {
  if (EXT_RAM.battery_type != BATTERY_TYPE_WITH_BATTERY)
    return OK;

  CHECK(file_data->size >= EXT_RAM.size);
  memcpy(file_data->data, EXT_RAM.data, file_data->size);
  return OK;
  ON_ERROR_RETURN;
}

Result Emulator::read_ext_ram_from_file(const char* filename) {
  if (EXT_RAM.battery_type != BATTERY_TYPE_WITH_BATTERY)
    return OK;
  Result result = ERROR;
  FileData file_data;
  ZERO_MEMORY(file_data);
  CHECK(SUCCESS(file_read(filename, &file_data)));
  CHECK(SUCCESS(read_ext_ram(&file_data)));
  result = OK;
error:
  file_data_delete(&file_data);
  return result;
}

Result Emulator::write_ext_ram_to_file(const char* filename) {
  if (EXT_RAM.battery_type != BATTERY_TYPE_WITH_BATTERY)
    return OK;

  Result result = ERROR;
  FileData file_data;
  file_data.size = EXT_RAM.size;
  file_data.data = reinterpret_cast<u8*>(xmalloc(file_data.size));
  CHECK(SUCCESS(write_ext_ram(&file_data)));
  CHECK(SUCCESS(file_write(filename, &file_data)));
  result = OK;
error:
  file_data_delete(&file_data);
  return result;
}

Result Emulator::read_state_from_file(const char* filename) {
  Result result = ERROR;
  FileData file_data;
  ZERO_MEMORY(file_data);
  CHECK(SUCCESS(file_read(filename, &file_data)));
  CHECK(SUCCESS(read_state(&file_data)));
  result = OK;
error:
  file_data_delete(&file_data);
  return result;
}

Result Emulator::write_state_to_file(const char* filename) {
  Result result = ERROR;
  FileData file_data;
  emulator_init_state_file_data(&file_data);
  CHECK(SUCCESS(write_state(&file_data)));
  CHECK(SUCCESS(file_write(filename, &file_data)));
  result = OK;
error:
  file_data_delete(&file_data);
  return result;
}

std::unique_ptr<Emulator> Emulator::try_create(const EmulatorInit* init) {
  std::unique_ptr<Emulator> e = std::make_unique<Emulator>();
  CHECK(SUCCESS(e->set_rom_file_data(&init->rom)));
  CHECK(SUCCESS(e->init_emulator(init)));
  CHECK(
      SUCCESS(e->init_audio_buffer(init->audio_frequency, init->audio_frames)));
  return e;
error:
  return NULL;
}

Emulator::Emulator() = default;

Emulator::~Emulator() {
  xfree(audio_buffer_.data);
}

void emulator_ticks_to_time(Ticks ticks, u32* day, u32* hr, u32* min, u32* sec,
                            u32* ms) {
  u64 secs = ticks / CPU_TICKS_PER_SECOND;
  *ms = (secs / 1000) % 1000;
  *sec = secs % 60;
  *min = (secs / 60) % 60;
  *hr = (secs / (60 * 60)) % 24;
  *day = secs / (60 * 60 * 24);
}

void Emulator::set_builtin_palette(u32 index) {
  static const PaletteRGBA pals[][3] = {
#define PAL(b0, b1, b2, b3, o00, o01, o02, o03, o10, o11, o12, o13) \
  {{{b0, b1, b2, b3}}, {{o00, o01, o02, o03}}, {{o10, o11, o12, o13}}},
#define PAL3(c0, c1, c2, c3) \
  {{{c0, c1, c2, c3}}, {{c0, c1, c2, c3}}, {{c0, c1, c2, c3}}},
#include "builtin-palettes.def"
#undef PAL
#undef PAL3
  };
  size_t count = sizeof(pals) / sizeof(*pals);
  if (index >= count) { return; }
  set_bw_palette(static_cast<PaletteType>(0), &pals[index][0]);
  set_bw_palette(static_cast<PaletteType>(1), &pals[index][1]);
  set_bw_palette(static_cast<PaletteType>(2), &pals[index][2]);
  for (int i = 0; i < 4; ++i) {
    SGB.screen_pal[i] = pals[index][0];
  }
  update_bw_palette_rgba(PALETTE_TYPE_BGP);
}

ApuLog* Emulator::get_apu_log() {
  return &apu_log_;
}

void Emulator::reset_apu_log() {
  apu_log_.write_count = 0;
}
