#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  HSV_19_255_255,
  HSV_229_255_255,
  HSV_122_255_255,
  HSV_0_0_255,
  MAC_MISSION_CONTROL,
  MAC_LOCK,
  DRAG_SCROLL,
  TOGGLE_SCROLL,
  NAVIGATOR_INC_CPI,
  NAVIGATOR_DEC_CPI,
  NAVIGATOR_TURBO,
  NAVIGATOR_AIM
};



#define DUAL_FUNC_0 LT(9, KC_2)
#define DUAL_FUNC_1 LT(5, KC_F13)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    DUAL_FUNC_0,    KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           TO(6),          
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,        
    MT(MOD_LCTL, KC_GRAVE),KC_A,           LT(3, KC_S),    KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           LT(3, KC_L),    LT(1, KC_SCLN), MT(MOD_RCTL, KC_QUOTE),
    KC_LEFT_SHIFT,  KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       MO(2),          
                                                    MT(MOD_RALT, KC_SPACE),MT(MOD_LGUI, KC_ESCAPE),                                LT(1, KC_ENTER),MT(MOD_LALT, KC_BSPC)
  ),
  [1] = LAYOUT_voyager(
    TO(0),          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_CIRC,        KC_LABK,        KC_RABK,        KC_DLR,         KC_PERC,                                        KC_AT,          KC_AMPR,        KC_ASTR,        KC_QUOTE,       KC_GRAVE,       KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_LCBR,        KC_LPRN,        KC_RPRN,        KC_RCBR,        KC_EQUAL,                                       KC_BSLS,        KC_MINUS,       KC_PLUS,        KC_SLASH,       KC_DQUO,        KC_TRANSPARENT, 
    CW_TOGG,        KC_TILD,        KC_LBRC,        KC_RBRC,        KC_UNDS,        KC_HASH,                                        KC_PIPE,        KC_EXLM,        KC_SCLN,        KC_COLN,        KC_QUES,        KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_DELETE
  ),
  [2] = LAYOUT_voyager(
    TO(0),          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    TO(0),          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(7),          
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LCTL(KC_LEFT),  RCTL(KC_RIGHT), KC_TRANSPARENT,                                 KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LCTL(LSFT(KC_TAB)),RCTL(KC_TAB),   KC_TRANSPARENT,                                 KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RGUI(LSFT(KC_TAB)),LGUI(KC_TAB),   KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    TO(0),          KC_BRIGHTNESS_DOWN,KC_F15,         MAC_MISSION_CONTROL,KC_F11,         KC_F12,                                         KC_MEDIA_PREV_TRACK,KC_MEDIA_PLAY_PAUSE,KC_MEDIA_NEXT_TRACK,KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, MAC_LOCK,       
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    TO(0),          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_KP_ASTERISK, KC_7,           KC_8,           KC_9,           KC_KP_PLUS,     KC_BSPC,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_KP_SLASH,    KC_4,           KC_5,           KC_6,           KC_KP_MINUS,    KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_KP_EQUAL,    KC_1,           KC_2,           KC_3,           KC_KP_DOT,      KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_0
  ),
  [6] = LAYOUT_voyager(
    TO(0),          RGB_VAD,        RGB_VAI,        KC_TRANSPARENT, RGB_TOG,        TOGGLE_LAYER_COLOR,                                KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,        
    KC_TRANSPARENT, RGB_SPD,        RGB_SPI,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, RGB_HUD,        RGB_HUI,        KC_TRANSPARENT, RGB_SLD,        RGB_MODE_FORWARD,                                HSV_19_255_255, HSV_229_255_255,HSV_122_255_255,HSV_0_0_255,    KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, RGB_SAD,        RGB_SAI,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [7] = LAYOUT_voyager(
    TO(0),          KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_F12,         
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, TO(0),                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [8] = LAYOUT_voyager(
    NAVIGATOR_DEC_CPI,NAVIGATOR_INC_CPI,KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_LLCK,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN3,     KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN2,     KC_MS_BTN1,     KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, DUAL_FUNC_1,    KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] PROGMEM =
    LAYOUT(
        '*', 'L', 'L', 'L', 'L', 'L',  'R', 'R', 'R', 'R', 'R', '*',
        '*', 'L', 'L', 'L', 'L', 'L',  'R', 'R', 'R', 'R', 'R', '*',
        '*', 'L', 'L', 'L', 'L', 'L',  'R', 'R', 'R', 'R', 'R', '*',
        '*', 'L', 'L', 'L', 'L', 'L',  'R', 'R', 'R', 'R', 'R', '*',
                            '*', '*',  '*', '*'
    );

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        // Whether or to immediately select the hold action when another key is pressed.
        case MT(MOD_RALT, KC_SPACE):
            return false;
        // case MT(MOD_LCTL, KC_GRAVE):
        //     return true;
        // case MT(MOD_RCTL, KC_QUOTE):
        //     return true;
        // case LT(1, KC_SCLN):
        //     return true;
        // case MT(MOD_LGUI, KC_ESCAPE):
        //     return true;
        // case MT(MOD_RGUI, KC_ENTER):
        //     return true;
        // case MT(MOD_LALT, KC_BSPC):
        //     return true;

        default:
            return true;
    }
}




extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [1] = { {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {0,0,0}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {163,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {0,0,0}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {0,0,0}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {219,255,255}, {0,0,0}, {0,0,0}, {152,255,255} },

    [2] = { {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {115,255,255}, {115,255,255}, {115,255,255}, {115,255,255}, {115,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {115,255,255}, {115,255,255}, {115,255,255}, {115,255,255}, {115,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {31,255,255}, {31,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {31,255,255}, {31,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {31,255,255}, {31,255,255}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {0,0,0}, {0,0,0}, {134,255,255}, {134,255,255}, {134,255,255}, {134,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,255}, {143,255,255}, {143,255,255}, {143,255,255}, {143,255,255}, {143,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {143,255,255}, {143,255,255}, {143,255,255}, {143,255,255}, {143,255,255}, {143,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0} },

    [5] = { {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {19,255,255}, {142,255,255}, {142,255,255}, {142,255,255}, {19,255,255}, {0,245,245}, {19,255,255}, {142,255,255}, {142,255,255}, {142,255,255}, {19,255,255}, {0,0,0}, {19,255,255}, {142,255,255}, {142,255,255}, {142,255,255}, {19,255,255}, {0,0,0}, {0,0,0}, {142,255,255} },

    [6] = { {0,0,255}, {45,255,255}, {45,255,255}, {0,0,0}, {45,255,255}, {45,255,255}, {0,0,0}, {45,255,255}, {45,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {45,255,255}, {45,255,255}, {0,0,0}, {45,255,255}, {45,255,255}, {0,0,0}, {45,255,255}, {45,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {19,255,255}, {229,255,255}, {122,255,255}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [7] = { {0,0,255}, {64,255,255}, {64,255,255}, {64,255,255}, {64,255,255}, {64,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {64,255,255}, {64,255,255}, {64,255,255}, {64,255,255}, {64,255,255}, {64,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {64,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [8] = { {143,255,255}, {143,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {223,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (!keyboard_config.disable_layer_led) { 
    switch (biton32(layer_state)) {
      case 1:
        set_layer_color(1);
        break;
      case 2:
        set_layer_color(2);
        break;
      case 3:
        set_layer_color(3);
        break;
      case 4:
        set_layer_color(4);
        break;
      case 5:
        set_layer_color(5);
        break;
      case 6:
        set_layer_color(6);
        break;
      case 7:
        set_layer_color(7);
        break;
      case 8:
        set_layer_color(8);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  return true;
}

extern bool set_scrolling;
extern bool navigator_turbo;
extern bool navigator_aim;
void pointing_device_init_user(void) {
    set_auto_mouse_enable(true);
}
bool is_mouse_record_kb(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case NAVIGATOR_INC_CPI ... NAVIGATOR_AIM:
    case DRAG_SCROLL:
    case TOGGLE_SCROLL:
      return true;
  }
  return is_mouse_record_user(keycode, record);
}




bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
  case QK_MODS ... QK_MODS_MAX: 
    // Mouse keys with modifiers work inconsistently across operating systems, this makes sure that modifiers are always
    // applied to the mouse key that was pressed.
    if (IS_MOUSE_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode))) {
    if (record->event.pressed) {
        add_mods(QK_MODS_GET_MODS(keycode));
        send_keyboard_report();
        wait_ms(2);
        register_code(QK_MODS_GET_BASIC_KEYCODE(keycode));
        return false;
      } else {
        wait_ms(2);
        del_mods(QK_MODS_GET_MODS(keycode));
      }
    }
    break;
    case MAC_MISSION_CONTROL:
      HCS(0x29F);
    case MAC_LOCK:
      HCS(0x19E);

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          layer_move(5);
        } else {
          layer_move(5);
        }
      } else {
        if (record->event.pressed) {
          layer_on(4);
        } else {
          layer_off(4);
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(TOGGLE_SCROLL);
        } else {
          unregister_code16(TOGGLE_SCROLL);
        }
      } else {
        if (record->event.pressed) {
          register_code16(DRAG_SCROLL);
        } else {
          unregister_code16(DRAG_SCROLL);
        }  
      }  
      return false;
    case DRAG_SCROLL:
      if (record->event.pressed) {
        set_scrolling = true;
      } else {
        set_scrolling = false;
      }
      return false;
    case TOGGLE_SCROLL:
      if (record->event.pressed) {
        set_scrolling = !set_scrolling;
      }
      return false;
    break;
  case NAVIGATOR_TURBO:
    if (record->event.pressed) {
      navigator_turbo = true;
    } else {
      navigator_turbo = false;
    }
    return false;
  case NAVIGATOR_AIM:
    if (record->event.pressed) {
      navigator_aim = true;
    } else {
      navigator_aim = false;
    }
    return false;
  case NAVIGATOR_INC_CPI:
    if (record->event.pressed) {
        pointing_device_set_cpi(1);
    }
    return false;
  case NAVIGATOR_DEC_CPI:
    if (record->event.pressed) {
        pointing_device_set_cpi(0);
    }
    return false;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_19_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(19,255,255);
      }
      return false;
    case HSV_229_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(229,255,255);
      }
      return false;
    case HSV_122_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(122,255,255);
      }
      return false;
    case HSV_0_0_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,0,255);
      }
      return false;
  }
  return true;
}

