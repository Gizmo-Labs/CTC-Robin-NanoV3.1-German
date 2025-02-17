/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfigPre.h"

#if BOTH(HAS_TFT_LVGL_UI, HAS_STEALTHCHOP)

#include "draw_ui.h"
#include <lv_conf.h>

#include "../../../module/stepper/indirection.h"
#include "../../../feature/tmc_util.h"
#include "../../../inc/MarlinConfig.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../module/settings.h"
#endif

extern lv_group_t *g;
static lv_obj_t *scr;

enum {
  ID_TMC_MODE_RETURN = 1,
  ID_TMC_MODE_X,
  ID_TMC_MODE_Y,
  ID_TMC_MODE_Z,
  ID_TMC_MODE_E0,
  ID_TMC_MODE_DOWN,
  ID_TMC_MODE_UP,
  ID_TMC_NONE,
};

static lv_obj_t *buttonXState = nullptr, *buttonYState = nullptr, *buttonZState = nullptr, *buttonE0State = nullptr;
static lv_obj_t *labelXState = nullptr, *labelYState = nullptr, *labelZState = nullptr, *labelE0State = nullptr;

//static lv_obj_t *buttonE1State = nullptr;
//static lv_obj_t *labelE1State = nullptr;

static void draw_onoff_btn_update(lv_obj_t *btn, lv_obj_t *label, bool isena);

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;

  auto toggle_chop = [&](auto &stepper, auto &button, auto &label) {
    const bool isena = stepper.toggle_stepping_mode();
    // lv_screen_menu_item_onoff_update(button, isena);
    draw_onoff_btn_update(button, label, isena);
    TERN_(EEPROM_SETTINGS, (void)settings.save());
  };

  switch (obj->mks_obj_id) {
    case ID_TMC_MODE_RETURN:
      uiCfg.para_ui_page = false;
      lv_clear_tmc_step_mode_settings();
      draw_return_ui();
      break;

    #if X_HAS_STEALTHCHOP
      case ID_TMC_MODE_X:  toggle_chop(stepperX,  buttonXState, labelXState);  break;
    #endif
    #if Y_HAS_STEALTHCHOP
      case ID_TMC_MODE_Y:  toggle_chop(stepperY,  buttonYState, labelYState);  break;
    #endif
    #if Z_HAS_STEALTHCHOP
      case ID_TMC_MODE_Z:  toggle_chop(stepperZ,  buttonZState, labelZState);  break;
    #endif
    #if E0_HAS_STEALTHCHOP
      case ID_TMC_MODE_E0: toggle_chop(stepperE0, buttonE0State, labelE0State); break;
    #endif
    

    case ID_TMC_MODE_UP:
      uiCfg.para_ui_page = false;
      lv_clear_tmc_step_mode_settings();
      lv_draw_tmc_step_mode_settings();
      break;
    case ID_TMC_MODE_DOWN:
      uiCfg.para_ui_page = true;
      lv_clear_tmc_step_mode_settings();
      lv_draw_tmc_step_mode_settings();
      break;

    case ID_TMC_NONE:
      // draw_onoff_btn_update();
    break;
  }
}

static lv_obj_t* set_on_off_label(lv_obj_t *labelValue, lv_obj_t *btn, bool curValue) {
  labelValue = lv_label_create_empty(btn);
  lv_label_set_text(labelValue, curValue ? machine_menu.enable : machine_menu.disable);
  lv_obj_align(labelValue, btn, LV_ALIGN_CENTER, 0, 0);
  return labelValue;
}

void lv_draw_tmc_step_mode_settings() {
  buttonXState = buttonYState = buttonZState = buttonE0State = nullptr;

  scr = lv_screen_create(TMC_MODE_UI, machine_menu.TmcStepModeConfTitle);

  bool stealth_X = false, stealth_Y = false, stealth_Z = false, stealth_E0 = false;
  
  TERN_(X_HAS_STEALTHCHOP,  stealth_X  = stepperX.get_stealthChop());
  TERN_(Y_HAS_STEALTHCHOP,  stealth_Y  = stepperY.get_stealthChop());
  TERN_(Z_HAS_STEALTHCHOP,  stealth_Z  = stepperZ.get_stealthChop());
  TERN_(E0_HAS_STEALTHCHOP, stealth_E0 = stepperE0.get_stealthChop());

  if (!uiCfg.para_ui_page) {
    lv_screen_menu_item_w(scr, machine_menu.X_StepMode, PARA_UI_POS_X, PARA_UI_POS_Y, event_handler, ID_TMC_NONE, 0, false);
    
    lv_screen_menu_item_w(scr, machine_menu.Y_StepMode, PARA_UI_POS_X, PARA_UI_POS_Y * 2, event_handler, ID_TMC_NONE, 1, false);
    lv_screen_menu_item_w(scr, machine_menu.Z_StepMode, PARA_UI_POS_X, PARA_UI_POS_Y * 3, event_handler, ID_TMC_NONE, 2, false);
    lv_screen_menu_item_w(scr, machine_menu.E0_StepMode, PARA_UI_POS_X, PARA_UI_POS_Y * 4, event_handler, ID_TMC_NONE, 2, false);
    lv_big_button_create(scr, "F:/bmp_back70x40.bin", machine_menu.next, PARA_UI_TURN_PAGE_POS_X, PARA_UI_TURN_PAGE_POS_Y, event_handler, ID_TMC_MODE_DOWN, true);

    buttonXState = lv_imgbtn_create(scr, stealth_X ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin", PARA_UI_STATE_POS_X, PARA_UI_POS_Y + PARA_UI_STATE_V, event_handler, ID_TMC_MODE_X);
    buttonYState = lv_imgbtn_create(scr, stealth_Y ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin", PARA_UI_STATE_POS_X, PARA_UI_POS_Y * 2 + PARA_UI_STATE_V, event_handler, ID_TMC_MODE_Y);
    buttonZState = lv_imgbtn_create(scr, stealth_Z ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin", PARA_UI_STATE_POS_X, PARA_UI_POS_Y * 3 + PARA_UI_STATE_V, event_handler, ID_TMC_MODE_Z);
    buttonE0State = lv_imgbtn_create(scr, stealth_E0 ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin", PARA_UI_STATE_POS_X, PARA_UI_POS_Y * 4 + PARA_UI_STATE_V, event_handler, ID_TMC_MODE_E0);

    labelXState = set_on_off_label(labelXState, buttonXState, stealth_X);
    labelYState = set_on_off_label(labelYState, buttonYState, stealth_Y);
    labelZState = set_on_off_label(labelZState, buttonZState, stealth_Z);
    labelE0State = set_on_off_label(labelE0State, buttonE0State, stealth_E0);
  }  
  lv_big_button_create(scr, "F:/bmp_back70x40.bin", common_menu.text_back, PARA_UI_BACL_POS_X, PARA_UI_BACL_POS_Y, event_handler, ID_TMC_MODE_RETURN, true);
}

static void draw_onoff_btn_update(lv_obj_t *btn, lv_obj_t *label, bool isena) {
  lv_imgbtn_set_src_both(btn, isena ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin");
  lv_label_set_text(label, isena ? machine_menu.enable : machine_menu.disable);
}


void lv_clear_tmc_step_mode_settings() {
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // HAS_TFT_LVGL_UI && HAS_STEALTHCHOP
