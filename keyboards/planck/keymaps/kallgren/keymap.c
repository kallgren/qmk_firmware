#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"
#include "keymap_nordic.h"
#include "keymap_swedish.h"
#include "keymap_br_abnt2.h"
#include "keymap_bepo.h"
#include "keymap_us_international.h"

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)
#define SE_SECT_MAC ALGR(KC_6)

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  SE_LSPO,
  SE_RSPC,
};

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
  DANCE_7,
  DANCE_8,
  DANCE_9,
  DANCE_10,
  DANCE_11,
  DANCE_12,
  DANCE_13,
  DANCE_14,
};

enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
  _LAYER4,
  _LAYER5,
  _LAYER6,
  _LAYER7,
  _LAYER8,
  _LAYER9,
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT_planck_grid(
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSPACE,
    KC_ESCAPE,      KC_A,           KC_S,           KC_D,           KC_F,           KC_G,           KC_H,           KC_J,           KC_K,           KC_L,           SE_ODIA,        KC_ENTER,
    KC_LSHIFT,      KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,           KC_N,           KC_M,           SE_ARNG,          SE_ADIA,        KC_COMMA,       MT(MOD_RSFT, KC_DOT),
    TT(7),          KC_LCTRL,       KC_LALT,        KC_LGUI,        LOWER,          ALL_T(KC_SPACE),KC_NO,          RAISE,          MT(MOD_RGUI, KC_LEFT),MT(MOD_RALT, KC_DOWN),MT(MOD_RCTL, KC_UP),KC_RIGHT
  ),

  [_LOWER] = LAYOUT_planck_grid(
    KC_TILD,        KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           SE_PLUS,
    KC_DELETE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_F6,          KC_KP_4,        KC_KP_5,        KC_KP_6,        KC_RCBR,        KC_PIPE,
    KC_TRANSPARENT, KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         KC_F12,         KC_KP_1,        KC_KP_2,        KC_KP_3,        KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_HOME,        KC_PGDOWN,      KC_PGUP,        KC_END
  ),

  [_RAISE] = LAYOUT_planck_grid(
    KC_GRAVE,       KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,        KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RPRN,        SE_QUES,
    KC_DELETE,      KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_RBRACKET,    KC_BSLASH,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NONUS_HASH,  KC_COMMA,       KC_DOT,         SE_MINS,        KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_ADJUST] = LAYOUT_planck_grid(
    KC_MEDIA_PLAY_PAUSE,KC_MEDIA_PREV_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, KC_TRANSPARENT, RGB_HUD,        RGB_VAD,        RGB_VAI,        RGB_HUI,        LGUI(LCTL(KC_Q)),
    KC_BRIGHTNESS_DOWN,KC_BRIGHTNESS_UP,AU_ON,          AU_OFF,         AU_TOG,         KC_TRANSPARENT, KC_TRANSPARENT, RGB_TOG,        RGB_MOD,        KC_TRANSPARENT, KC_TRANSPARENT, RESET,
    KC_TRANSPARENT, KC_TRANSPARENT, MU_ON,          MU_OFF,         MU_TOG,         KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    WEBUSB_PAIR,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(4),          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER4] = LAYOUT_planck_grid(
    KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,           KC_NO,          KC_NO,          KC_J,           KC_L,           KC_U,           KC_Y,           SE_ODIA,
    MT(MOD_LSFT, KC_A),MT(MOD_LCTL, KC_R),MT(MOD_LALT, KC_S),MT(MOD_LGUI, KC_T),KC_G,           KC_NO,          KC_NO,          KC_M,           MT(MOD_RGUI, KC_N),MT(MOD_RALT, KC_E),MT(MOD_RCTL, KC_I),MT(MOD_RSFT, KC_O),
    LT(7,KC_Z),     KC_X,           KC_C,           KC_D,           KC_V,           KC_NO,          KC_NO,          KC_K,           KC_H,           SE_ARNG,          SE_ADIA,        TD(DANCE_0),
    KC_NO,          KC_NO,          KC_NO,          ALL_T(KC_SPACE),LT(5,KC_TAB),   TO(0),          KC_NO,          MO(6),          KC_BSPACE,      KC_NO,          KC_NO,          KC_NO
  ),

  [_LAYER5] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, LGUI(LSFT(KC_TAB)),LGUI(KC_TAB),   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LSFT(KC_TAB),   KC_TAB,         KC_TRANSPARENT, KC_TRANSPARENT,
    KC_ESCAPE,      KC_TRANSPARENT, LALT(LSFT(KC_TAB)),LALT(KC_TAB),   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, LCTL(LSFT(KC_TAB)),LCTL(KC_TAB),   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER6] = LAYOUT_planck_grid(
    KC_TAB,         SE_QUOT,        SE_LPRN,        SE_RPRN,        SE_RABK,    KC_TRANSPARENT, KC_TRANSPARENT, SE_MINS,        KC_7,           KC_8,           KC_9,           TD(DANCE_1),
    TD(DANCE_2),    TD(DANCE_3),    TD(DANCE_4),    TD(DANCE_5),    KC_EQUAL,       KC_TRANSPARENT, KC_TRANSPARENT, SE_PLUS,        KC_4,           KC_5,           KC_6,           KC_ENTER,
    SE_ACUT,        SE_CIRC,        SE_LBRC,        SE_RBRC,        SE_SLSH,        KC_TRANSPARENT, KC_TRANSPARENT, KC_0,           KC_1,           KC_2,           KC_3,           KC_COMMA,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER7] = LAYOUT_planck_grid(
    KC_TRANSPARENT, TD(DANCE_6),    KC_MS_UP,       TD(DANCE_7),    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_UP,    KC_MS_WH_DOWN,  KC_TRANSPARENT, KC_MS_BTN1,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_UP,    KC_MS_WH_DOWN,  KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TD(DANCE_8),    TD(DANCE_9),    KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, LCTL(KC_LEFT),  LCTL(KC_DOWN),  LCTL(KC_UP),    LCTL(KC_RIGHT)
  ),

  [_LAYER8] = LAYOUT_planck_grid(
    KC_AMPR,        SE_AT,          TD(DANCE_10),   SE_SLSH,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, SE_BSLS,    SE_QUOT,        SE_CIRC,        TD(DANCE_11),
    KC_ESCAPE,      SE_LBRC,        KC_LCBR,        SE_LPRN,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, SE_RPRN,        KC_RCBR,        SE_RBRC,        KC_ENTER,
    SE_UNDS,        KC_TRANSPARENT, SE_GRV,         KC_LABK,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_RABK,        SE_ACUT,        SE_TILD,        TD(DANCE_12),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER9] = LAYOUT_planck_grid(
    KC_0,           KC_7,           KC_8,           KC_9,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_7,           KC_8,           KC_9,           TD(DANCE_13),
    KC_LSHIFT,      MT(MOD_LCTL, KC_4),MT(MOD_LALT, KC_5),MT(MOD_LGUI, KC_6),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_4,           KC_5,           KC_6,           KC_ENTER,
    KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           TD(DANCE_14),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [1] = { {0,0,0}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {74,255,255}, {74,255,255}, {74,255,255}, {74,255,255} },

    [2] = { {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {74,255,255}, {74,255,255}, {74,255,255}, {74,255,255}, {74,255,255}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {188,255,255}, {188,255,255}, {188,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {219,255,255}, {0,0,0}, {0,0,0}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {219,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {219,255,255}, {219,255,255}, {0,0,0}, {219,255,255}, {219,255,255}, {0,0,0}, {0,0,0}, {0,0,0} },

    [5] = { {0,0,0}, {0,0,0}, {139,220,207}, {139,220,207}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {139,220,207}, {139,220,207}, {0,0,0}, {0,0,0}, {30,255,255}, {0,0,0}, {139,220,207}, {139,220,207}, {0,0,0}, {0,0,0}, {0,0,0}, {140,98,230}, {140,98,230}, {140,98,230}, {140,98,230}, {0,0,0}, {0,0,0}, {0,0,0}, {139,220,207}, {139,220,207}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [6] = { {154,170,255}, {117,155,255}, {84,155,255}, {84,155,255}, {9,198,247}, {0,0,0}, {0,0,0}, {27,171,255}, {140,64,230}, {140,64,230}, {140,64,230}, {9,198,247}, {9,179,241}, {31,149,189}, {129,104,255}, {129,104,255}, {154,170,255}, {0,0,0}, {0,0,0}, {27,171,255}, {140,64,230}, {140,64,230}, {140,64,230}, {30,255,255}, {117,155,255}, {63,199,247}, {154,170,255}, {154,170,255}, {200,188,255}, {0,0,0}, {0,0,0}, {139,45,241}, {140,64,230}, {140,64,230}, {140,64,230}, {63,199,247}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [7] = { {0,0,0}, {30,247,242}, {30,247,242}, {30,247,242}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {30,247,242}, {30,247,242}, {30,247,242}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {30,247,242}, {30,247,242}, {0,0,0}, {30,247,242}, {0,0,0}, {0,0,0}, {30,247,242}, {30,247,242}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {30,247,242}, {30,247,242}, {0,0,0}, {0,0,0}, {30,247,242}, {30,247,242}, {30,247,242}, {30,247,242} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < DRIVER_LED_TOTAL; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

void rgb_matrix_indicators_user(void) {
  if (keyboard_config.disable_layer_led) { return; }
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
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case SE_LSPO:
      perform_space_cadet(record, keycode, KC_LSFT, KC_LSFT, KC_8);
      return false;
    case SE_RSPC:
      perform_space_cadet(record, keycode, KC_LSFT, KC_LSFT, KC_9);
      return false;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}

#ifdef AUDIO_ENABLE
bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
    if (muse_mode) {
        if (IS_LAYER_ON(_RAISE)) {
            if (clockwise) {
                muse_offset++;
            } else {
                muse_offset--;
            }
        } else {
            if (clockwise) {
                muse_tempo+=1;
            } else {
                muse_tempo-=1;
            }
        }
    } else {
        if (clockwise) {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_DOWN);
            unregister_code(KC_MS_WH_DOWN);
        #else
            register_code(KC_PGDN);
            unregister_code(KC_PGDN);
        #endif
        } else {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_UP);
            unregister_code(KC_MS_WH_UP);
        #else
            register_code(KC_PGUP);
            unregister_code(KC_PGUP);
        #endif
        }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
    switch (keycode) {
    case RAISE:
    case LOWER:
        return false;
    default:
        return true;
    }
}
#endif

uint32_t layer_state_set_user(uint32_t state) {
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[15];

uint8_t dance_step(qk_tap_dance_state_t *state);

uint8_t dance_step(qk_tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(qk_tap_dance_state_t *state, void *user_data);
void dance_0_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_0_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_0(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DOT);
        tap_code16(KC_DOT);
        tap_code16(KC_DOT);
    }
    if(state->count > 3) {
        tap_code16(KC_DOT);
    }
}

void dance_0_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_DOT); break;
        case DOUBLE_TAP: register_code16(SE_COLN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DOT); register_code16(KC_DOT);
    }
}

void dance_0_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_DOT); break;
        case DOUBLE_TAP: unregister_code16(SE_COLN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DOT); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(qk_tap_dance_state_t *state, void *user_data);
void dance_1_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_1_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_1(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
    }
    if(state->count > 3) {
        tap_code16(SE_QUES);
    }
}

void dance_1_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(SE_QUES); break;
        case DOUBLE_TAP: register_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_QUES); register_code16(SE_QUES);
    }
}

void dance_1_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(SE_QUES); break;
        case DOUBLE_TAP: unregister_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_QUES); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(qk_tap_dance_state_t *state, void *user_data);
void dance_2_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_2_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_2(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_AMPR);
        tap_code16(SE_AMPR);
        tap_code16(SE_AMPR);
    }
    if(state->count > 3) {
        tap_code16(SE_AMPR);
    }
}

void dance_2_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(SE_AMPR); break;
        case SINGLE_HOLD: register_code16(KC_LSHIFT); break;
        case DOUBLE_TAP: register_code16(SE_AMPR); register_code16(SE_AMPR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_AMPR); register_code16(SE_AMPR);
    }
}

void dance_2_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(SE_AMPR); break;
        case SINGLE_HOLD: unregister_code16(KC_LSHIFT); break;
        case DOUBLE_TAP: unregister_code16(SE_AMPR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_AMPR); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(qk_tap_dance_state_t *state, void *user_data);
void dance_3_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_3_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_3(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_AT);
        tap_code16(SE_AT);
        tap_code16(SE_AT);
    }
    if(state->count > 3) {
        tap_code16(SE_AT);
    }
}

void dance_3_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(SE_AT); break;
        case SINGLE_HOLD: register_code16(KC_LCTRL); break;
        case DOUBLE_TAP: register_code16(SE_AT); register_code16(SE_AT); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_AT); register_code16(SE_AT);
    }
}

void dance_3_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(SE_AT); break;
        case SINGLE_HOLD: unregister_code16(KC_LCTRL); break;
        case DOUBLE_TAP: unregister_code16(SE_AT); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_AT); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(qk_tap_dance_state_t *state, void *user_data);
void dance_4_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_4_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_4(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_LCBR);
        tap_code16(SE_LCBR);
        tap_code16(SE_LCBR);
    }
    if(state->count > 3) {
        tap_code16(SE_LCBR);
    }
}

void dance_4_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(SE_LCBR); break;
        case SINGLE_HOLD: register_code16(KC_LALT); break;
        case DOUBLE_TAP: register_code16(SE_LCBR); register_code16(SE_LCBR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_LCBR); register_code16(SE_LCBR);
    }
}

void dance_4_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(SE_LCBR); break;
        case SINGLE_HOLD: unregister_code16(KC_LALT); break;
        case DOUBLE_TAP: unregister_code16(SE_LCBR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_LCBR); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(qk_tap_dance_state_t *state, void *user_data);
void dance_5_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_5_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_5(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_RCBR);
        tap_code16(SE_RCBR);
        tap_code16(SE_RCBR);
    }
    if(state->count > 3) {
        tap_code16(SE_RCBR);
    }
}

void dance_5_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(SE_RCBR); break;
        case SINGLE_HOLD: register_code16(KC_LGUI); break;
        case DOUBLE_TAP: register_code16(SE_RCBR); register_code16(SE_RCBR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_RCBR); register_code16(SE_RCBR);
    }
}

void dance_5_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(SE_RCBR); break;
        case SINGLE_HOLD: unregister_code16(KC_LGUI); break;
        case DOUBLE_TAP: unregister_code16(SE_RCBR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_RCBR); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(qk_tap_dance_state_t *state, void *user_data);
void dance_6_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_6_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_6(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN2);
    }
}

void dance_6_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_RIGHT); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2);
    }
}

void dance_6_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_RIGHT); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
    }
    dance_state[6].step = 0;
}
void on_dance_7(qk_tap_dance_state_t *state, void *user_data);
void dance_7_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_7_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_7(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN1);
    }
}

void dance_7_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[7].step = dance_step(state);
    switch (dance_state[7].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_LEFT); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1);
    }
}

void dance_7_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[7].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_LEFT); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
    }
    dance_state[7].step = 0;
}
void on_dance_8(qk_tap_dance_state_t *state, void *user_data);
void dance_8_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_8_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_8(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN2);
    }
}

void dance_8_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[8].step = dance_step(state);
    switch (dance_state[8].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_DOWN); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2);
    }
}

void dance_8_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[8].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_DOWN); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
    }
    dance_state[8].step = 0;
}
void on_dance_9(qk_tap_dance_state_t *state, void *user_data);
void dance_9_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_9_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_9(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN1);
    }
}

void dance_9_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[9].step = dance_step(state);
    switch (dance_state[9].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_UP); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1);
    }
}

void dance_9_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[9].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_UP); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
    }
    dance_state[9].step = 0;
}
void on_dance_10(qk_tap_dance_state_t *state, void *user_data);
void dance_10_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_10_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_10(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_DQUO);
        tap_code16(SE_DQUO);
        tap_code16(SE_DQUO);
    }
    if(state->count > 3) {
        tap_code16(SE_DQUO);
    }
}

void dance_10_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[10].step = dance_step(state);
    switch (dance_state[10].step) {
        case SINGLE_TAP: register_code16(SE_DQUO); break;
        case DOUBLE_TAP: register_code16(SE_QUOT); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_DQUO); register_code16(SE_DQUO);
    }
}

void dance_10_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[10].step) {
        case SINGLE_TAP: unregister_code16(SE_DQUO); break;
        case DOUBLE_TAP: unregister_code16(SE_QUOT); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_DQUO); break;
    }
    dance_state[10].step = 0;
}
void on_dance_11(qk_tap_dance_state_t *state, void *user_data);
void dance_11_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_11_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_11(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_QUES);
        tap_code16(KC_QUES);
        tap_code16(KC_QUES);
    }
    if(state->count > 3) {
        tap_code16(KC_QUES);
    }
}

void dance_11_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[11].step = dance_step(state);
    switch (dance_state[11].step) {
        case SINGLE_TAP: register_code16(KC_QUES); break;
        case DOUBLE_TAP: register_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_QUES); register_code16(KC_QUES);
    }
}

void dance_11_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[11].step) {
        case SINGLE_TAP: unregister_code16(KC_QUES); break;
        case DOUBLE_TAP: unregister_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_QUES); break;
    }
    dance_state[11].step = 0;
}
void on_dance_12(qk_tap_dance_state_t *state, void *user_data);
void dance_12_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_12_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_12(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_COMMA);
        tap_code16(KC_COMMA);
        tap_code16(KC_COMMA);
    }
    if(state->count > 3) {
        tap_code16(KC_COMMA);
    }
}

void dance_12_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[12].step = dance_step(state);
    switch (dance_state[12].step) {
        case SINGLE_TAP: register_code16(KC_COMMA); break;
        case DOUBLE_TAP: register_code16(KC_SCOLON); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_COMMA); register_code16(KC_COMMA);
    }
}

void dance_12_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[12].step) {
        case SINGLE_TAP: unregister_code16(KC_COMMA); break;
        case DOUBLE_TAP: unregister_code16(KC_SCOLON); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_COMMA); break;
    }
    dance_state[12].step = 0;
}
void on_dance_13(qk_tap_dance_state_t *state, void *user_data);
void dance_13_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_13_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_13(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_0);
        tap_code16(KC_0);
        tap_code16(KC_0);
    }
    if(state->count > 3) {
        tap_code16(KC_0);
    }
}

void dance_13_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[13].step = dance_step(state);
    switch (dance_state[13].step) {
        case SINGLE_TAP: register_code16(KC_0); break;
        case SINGLE_HOLD: register_code16(SE_EQL); break;
        case DOUBLE_TAP: register_code16(KC_0); register_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_0); register_code16(KC_0);
    }
}

void dance_13_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[13].step) {
        case SINGLE_TAP: unregister_code16(KC_0); break;
        case SINGLE_HOLD: unregister_code16(SE_EQL); break;
        case DOUBLE_TAP: unregister_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_0); break;
    }
    dance_state[13].step = 0;
}
void on_dance_14(qk_tap_dance_state_t *state, void *user_data);
void dance_14_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_14_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_14(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_MINS);
        tap_code16(SE_MINS);
        tap_code16(SE_MINS);
    }
    if(state->count > 3) {
        tap_code16(SE_MINS);
    }
}

void dance_14_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[14].step = dance_step(state);
    switch (dance_state[14].step) {
        case SINGLE_TAP: register_code16(SE_MINS); break;
        case DOUBLE_TAP: register_code16(SE_PLUS); break;
        case DOUBLE_HOLD: register_code16(SE_EQL); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_MINS); register_code16(SE_MINS);
    }
}

void dance_14_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[14].step) {
        case SINGLE_TAP: unregister_code16(SE_MINS); break;
        case DOUBLE_TAP: unregister_code16(SE_PLUS); break;
        case DOUBLE_HOLD: unregister_code16(SE_EQL); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_MINS); break;
    }
    dance_state[14].step = 0;
}

qk_tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
        [DANCE_7] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_7, dance_7_finished, dance_7_reset),
        [DANCE_8] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_8, dance_8_finished, dance_8_reset),
        [DANCE_9] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_9, dance_9_finished, dance_9_reset),
        [DANCE_10] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_10, dance_10_finished, dance_10_reset),
        [DANCE_11] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_11, dance_11_finished, dance_11_reset),
        [DANCE_12] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_12, dance_12_finished, dance_12_reset),
        [DANCE_13] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_13, dance_13_finished, dance_13_reset),
        [DANCE_14] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_14, dance_14_finished, dance_14_reset),
};

