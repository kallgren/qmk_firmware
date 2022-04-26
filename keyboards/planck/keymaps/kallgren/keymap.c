#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"
#include "keymap_swedish.h"
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

#define SE_BSLS_FIXED S(LALT(SE_7))
#define SE_LCBR_FIXED S(LALT(SE_8))
#define SE_RCBR_FIXED S(LALT(SE_9))
#define SE_PIPE_FIXED LALT(SE_7)

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
};

enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
  _LAYER4,
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT_planck_grid(
    KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,           KC_TRANSPARENT, KC_TRANSPARENT, KC_J,           KC_L,           KC_U,           KC_Y,           SE_ODIA,
    MT(MOD_LSFT, KC_A),MT(MOD_LCTL, KC_R),MT(MOD_LALT, KC_S),MT(MOD_LGUI, KC_T),KC_G,           KC_TRANSPARENT, KC_TRANSPARENT, KC_M,           MT(MOD_RGUI, KC_N),MT(MOD_RALT, KC_E),MT(MOD_RCTL, KC_I),MT(MOD_RSFT, KC_O),
    LT(4,KC_Z),     KC_X,           KC_C,           KC_D,           KC_V,           KC_TRANSPARENT, KC_TRANSPARENT, KC_K,           KC_H,           SE_ARNG,          SE_ADIA,        TD(DANCE_0),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, ALL_T(KC_SPACE),LT(_LOWER, KC_TAB),KC_TRANSPARENT, KC_NO,          RAISE,          KC_BSPACE,      KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LOWER] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_7,           KC_8,           KC_9,           KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, MT(MOD_RGUI, KC_4),MT(MOD_RALT, KC_5),MT(MOD_RCTL, KC_6),KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_0,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_RAISE] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TD(DANCE_3),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_ADJUST] = LAYOUT_planck_grid(
    KC_MEDIA_PLAY_PAUSE,KC_MEDIA_PREV_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, KC_TRANSPARENT, RGB_HUD,        RGB_VAD,        RGB_VAI,        RGB_HUI,        LGUI(LCTL(KC_Q)),
    KC_BRIGHTNESS_DOWN,KC_BRIGHTNESS_UP,AU_ON,          AU_OFF,         AU_TOG,         KC_TRANSPARENT, KC_TRANSPARENT, RGB_TOG,        RGB_MOD,        KC_TRANSPARENT, KC_TRANSPARENT, RESET,
    TG(4),          KC_TRANSPARENT, MU_ON,          MU_OFF,         MU_TOG,         KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    WEBUSB_PAIR,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER4] = LAYOUT_planck_grid(
    KC_TRANSPARENT, TD(DANCE_5),    KC_MS_UP,       TD(DANCE_6),    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LCTL(KC_LEFT),  LCTL(KC_DOWN),  TD(DANCE_7),    LCTL(KC_RIGHT),
    KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_MS_BTN1,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TD(DANCE_8),    TD(DANCE_9),    KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [3] = { {74,255,255}, {74,255,255}, {74,255,255}, {74,255,255}, {74,255,255}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {188,255,255}, {188,255,255}, {188,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {219,255,255}, {28,185,255}, {0,0,0}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {219,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,0}, {28,185,255}, {28,185,255}, {28,185,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {202,211,154}, {202,211,154}, {202,211,154}, {202,211,154}, {0,0,0}, {28,185,255}, {28,185,255}, {28,185,255}, {0,0,0}, {0,0,0}, {0,0,0}, {139,108,230}, {139,108,230}, {139,108,230}, {139,108,230}, {28,185,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {28,185,255}, {28,204,241}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

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
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
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

static tap dance_state[10];

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
        tap_code16(SE_EQL);
        tap_code16(SE_EQL);
        tap_code16(SE_EQL);
    }
    if(state->count > 3) {
        tap_code16(SE_EQL);
    }
}

void dance_1_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(SE_EQL); break;
        case SINGLE_HOLD: register_code16(KC_LGUI); break;
        case DOUBLE_TAP: register_code16(SE_EQL); register_code16(SE_EQL); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_EQL); register_code16(SE_EQL);
    }
}

void dance_1_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(SE_EQL); break;
        case SINGLE_HOLD: unregister_code16(KC_LGUI); break;
        case DOUBLE_TAP: unregister_code16(SE_EQL); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_EQL); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(qk_tap_dance_state_t *state, void *user_data);
void dance_2_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_2_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_2(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_DLR);
        tap_code16(SE_DLR);
        tap_code16(SE_DLR);
    }
    if(state->count > 3) {
        tap_code16(SE_DLR);
    }
}

void dance_2_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(SE_DLR); break;
        case DOUBLE_TAP: register_code16(SE_PND); break;
        case DOUBLE_HOLD: register_code16(SE_EURO); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_DLR); register_code16(SE_DLR);
    }
}

void dance_2_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(SE_DLR); break;
        case DOUBLE_TAP: unregister_code16(SE_PND); break;
        case DOUBLE_HOLD: unregister_code16(SE_EURO); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_DLR); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(qk_tap_dance_state_t *state, void *user_data);
void dance_3_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_3_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_3(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
    }
    if(state->count > 3) {
        tap_code16(SE_QUES);
    }
}

void dance_3_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(SE_QUES); break;
        case DOUBLE_TAP: register_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_QUES); register_code16(SE_QUES);
    }
}

void dance_3_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(SE_QUES); break;
        case DOUBLE_TAP: unregister_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_QUES); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(qk_tap_dance_state_t *state, void *user_data);
void dance_4_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_4_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_4(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_COMMA);
        tap_code16(KC_COMMA);
        tap_code16(KC_COMMA);
    }
    if(state->count > 3) {
        tap_code16(KC_COMMA);
    }
}

void dance_4_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(KC_COMMA); break;
        case DOUBLE_TAP: register_code16(SE_SCLN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_COMMA); register_code16(KC_COMMA);
    }
}

void dance_4_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(KC_COMMA); break;
        case DOUBLE_TAP: unregister_code16(SE_SCLN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_COMMA); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(qk_tap_dance_state_t *state, void *user_data);
void dance_5_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_5_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_5(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
        tap_code16(KC_MS_BTN2);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN2);
    }
}

void dance_5_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_RIGHT); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN2); register_code16(KC_MS_BTN2);
    }
}

void dance_5_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_RIGHT); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN2); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN2); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(qk_tap_dance_state_t *state, void *user_data);
void dance_6_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_6_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_6(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
        tap_code16(KC_MS_BTN1);
    }
    if(state->count > 3) {
        tap_code16(KC_MS_BTN1);
    }
}

void dance_6_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: register_code16(KC_MS_WH_LEFT); break;
        case DOUBLE_TAP: register_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MS_BTN1); register_code16(KC_MS_BTN1);
    }
}

void dance_6_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case SINGLE_HOLD: unregister_code16(KC_MS_WH_LEFT); break;
        case DOUBLE_TAP: unregister_code16(KC_MS_BTN1); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MS_BTN1); break;
    }
    dance_state[6].step = 0;
}
void on_dance_7(qk_tap_dance_state_t *state, void *user_data);
void dance_7_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_7_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_7(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_UP));
        tap_code16(LCTL(KC_UP));
        tap_code16(LCTL(KC_UP));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_UP));
    }
}

void dance_7_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[7].step = dance_step(state);
    switch (dance_state[7].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_UP)); break;
        case SINGLE_HOLD: register_code16(KC_LSHIFT); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_UP)); register_code16(LCTL(KC_UP)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_UP)); register_code16(LCTL(KC_UP));
    }
}

void dance_7_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[7].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_UP)); break;
        case SINGLE_HOLD: unregister_code16(KC_LSHIFT); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_UP)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_UP)); break;
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
};

/* COMBOS: */
// Big keys
const uint16_t PROGMEM rst_combo[] = {MT(MOD_LCTL, KC_R), MT(MOD_LALT, KC_S), MT(MOD_LGUI, KC_T), COMBO_END};
const uint16_t PROGMEM nei_combo[] = {MT(MOD_RGUI, KC_N), MT(MOD_RALT, KC_E), MT(MOD_RCTL, KC_I), COMBO_END};

// Arrows
const uint16_t PROGMEM mn_combo[] = {KC_M, MT(MOD_RGUI, KC_N), COMBO_END};
const uint16_t PROGMEM ne_combo[] = {MT(MOD_RGUI, KC_N), MT(MOD_RALT, KC_E), COMBO_END};
const uint16_t PROGMEM ei_combo[] = {MT(MOD_RALT, KC_E), MT(MOD_RCTL, KC_I), COMBO_END};
const uint16_t PROGMEM nu_combo[] = {MT(MOD_RGUI, KC_N), KC_U, COMBO_END};

/* Symbols */
// Left hand
const uint16_t PROGMEM wf_combo[] = {KC_W, KC_F, COMBO_END};
const uint16_t PROGMEM fp_combo[] = {KC_F, KC_P, COMBO_END};

const uint16_t PROGMEM qa_combo[] = {KC_Q, MT(MOD_LSFT, KC_A), COMBO_END};
const uint16_t PROGMEM wr_combo[] = {KC_W, MT(MOD_LCTL, KC_R), COMBO_END};
const uint16_t PROGMEM rf_combo[] = {MT(MOD_LCTL, KC_R), KC_F, COMBO_END};
const uint16_t PROGMEM fs_combo[] = {KC_F, MT(MOD_LALT, KC_S), COMBO_END};
const uint16_t PROGMEM ft_combo[] = {KC_F, MT(MOD_LGUI, KC_T), COMBO_END};
const uint16_t PROGMEM pt_combo[] = {KC_P, MT(MOD_LGUI, KC_T), COMBO_END};

const uint16_t PROGMEM ar_combo[] = {MT(MOD_LSFT, KC_A), MT(MOD_LCTL, KC_R), COMBO_END};
const uint16_t PROGMEM rs_combo[] = {MT(MOD_LCTL, KC_R), MT(MOD_LALT, KC_S), COMBO_END};
const uint16_t PROGMEM st_combo[] = {MT(MOD_LALT, KC_S), MT(MOD_LGUI, KC_T), COMBO_END};
const uint16_t PROGMEM tg_combo[] = {MT(MOD_LGUI, KC_T), KC_G, COMBO_END};

const uint16_t PROGMEM rt_combo[] = {MT(MOD_LCTL, KC_R), MT(MOD_LGUI, KC_T), COMBO_END};

const uint16_t PROGMEM sd_combo[] = {MT(MOD_LALT, KC_S), KC_D, COMBO_END};
const uint16_t PROGMEM td_combo[] = {MT(MOD_LGUI, KC_T), KC_D, COMBO_END};

const uint16_t PROGMEM xc_combo[] = {KC_X, KC_C, COMBO_END};
const uint16_t PROGMEM cd_combo[] = {KC_C, KC_D, COMBO_END};

// Right hand
const uint16_t PROGMEM lu_combo[] = {KC_L, KC_U, COMBO_END};
const uint16_t PROGMEM uy_combo[] = {KC_U, KC_Y, COMBO_END};

const uint16_t PROGMEM jm_combo[] = {KC_J, KC_M, COMBO_END};
const uint16_t PROGMEM ln_combo[] = {KC_L, MT(MOD_RGUI, KC_N), COMBO_END};
const uint16_t PROGMEM ue_combo[] = {KC_U, MT(MOD_RALT, KC_E), COMBO_END};
const uint16_t PROGMEM ui_combo[] = {KC_U, MT(MOD_RCTL, KC_I), COMBO_END};
const uint16_t PROGMEM yi_combo[] = {KC_Y, MT(MOD_RCTL, KC_I), COMBO_END};
const uint16_t PROGMEM oo_combo[] = {SE_ODIA, MT(MOD_RSFT, KC_O), COMBO_END};

const uint16_t PROGMEM ny_combo[] = {MT(MOD_RGUI, KC_N), SE_Y, COMBO_END};
const uint16_t PROGMEM no_combo[] = {MT(MOD_RGUI, KC_N), SE_ODIA, COMBO_END};
const uint16_t PROGMEM ni_combo[] = {MT(MOD_RGUI, KC_N), MT(MOD_RCTL, KC_I), COMBO_END};

const uint16_t PROGMEM io_combo[] = {MT(MOD_RCTL, KC_I), MT(MOD_RSFT, KC_O), COMBO_END};

const uint16_t PROGMEM nh_combo[] = {MT(MOD_RGUI, KC_N), KC_H, COMBO_END};
const uint16_t PROGMEM he_combo[] = {KC_H, MT(MOD_RALT, KC_E), COMBO_END};

const uint16_t PROGMEM ha_combo[] = {KC_H, SE_ARNG, COMBO_END};
const uint16_t PROGMEM aa_combo[] = {SE_ARNG, SE_ADIA, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(rst_combo, KC_ESC),
    COMBO(nei_combo, KC_ENTER),

    COMBO(mn_combo, KC_LEFT),
    COMBO(ne_combo, KC_DOWN),
    COMBO(ei_combo, KC_RIGHT),
    COMBO(nu_combo, KC_UP),

    // Symbols
    COMBO(wf_combo, SE_LPRN),
    COMBO(fp_combo, SE_LCBR_FIXED),

    COMBO(qa_combo, SE_TILD),
    COMBO(wr_combo, SE_AT),
    COMBO(rf_combo, SE_SLSH),
    COMBO(fs_combo, KC_HASH),
    COMBO(ft_combo, SE_DQUO),
    COMBO(pt_combo, SE_UNDS),

    COMBO(ar_combo, SE_LABK),
    COMBO(rs_combo, SE_MINS),
    COMBO(st_combo, SE_EQL),
    COMBO(tg_combo, SE_PLUS),

    COMBO(rt_combo, SE_DQUO),

    COMBO(sd_combo, SE_QUOT),
    COMBO(td_combo, SE_PIPE_FIXED),

    COMBO(xc_combo, SE_LBRC),
    COMBO(cd_combo, SE_GRV),

    COMBO(lu_combo, SE_RCBR_FIXED),
    COMBO(uy_combo, SE_RPRN),

    COMBO(jm_combo, SE_CIRC),
    COMBO(ln_combo, SE_DLR),
    COMBO(ue_combo, SE_ASTR),
    COMBO(ui_combo, SE_BSLS_FIXED),
    COMBO(yi_combo, SE_AMPR),
    COMBO(oo_combo, SE_COLN),

    COMBO(ny_combo, SE_QUES),
    COMBO(no_combo, KC_EXLM),
    COMBO(ni_combo, SE_QUOT),

    COMBO(io_combo, SE_RABK),

    COMBO(nh_combo, KC_PERC),
    COMBO(he_combo, KC_COMMA),

    COMBO(ha_combo, SE_ACUT),
    COMBO(aa_combo, SE_RBRC),
};
