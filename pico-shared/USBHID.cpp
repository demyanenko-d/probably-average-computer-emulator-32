#include <array>

#include "tusb.h"

#include "Scancode.h"

static const ATScancode scancodeMap[]{
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,

    ATScancode::A,
    ATScancode::B,
    ATScancode::C,
    ATScancode::D,
    ATScancode::E,
    ATScancode::F,
    ATScancode::G,
    ATScancode::H,
    ATScancode::I,
    ATScancode::J,
    ATScancode::K,
    ATScancode::L,
    ATScancode::M,
    ATScancode::N,
    ATScancode::O,
    ATScancode::P,
    ATScancode::Q,
    ATScancode::R,
    ATScancode::S,
    ATScancode::T,
    ATScancode::U,
    ATScancode::V,
    ATScancode::W,
    ATScancode::X,
    ATScancode::Y,
    ATScancode::Z,
    
    ATScancode::_1,
    ATScancode::_2,
    ATScancode::_3,
    ATScancode::_4,
    ATScancode::_5,
    ATScancode::_6,
    ATScancode::_7,
    ATScancode::_8,
    ATScancode::_9,
    ATScancode::_0,

    ATScancode::Return,
    ATScancode::Escape,
    ATScancode::Backspace,
    ATScancode::Tab,
    ATScancode::Space,

    ATScancode::Minus,
    ATScancode::Equals,
    ATScancode::LeftBracket,
    ATScancode::RightBracket,
    ATScancode::Backslash,
    ATScancode::Backslash, // same key
    ATScancode::Semicolon,
    ATScancode::Apostrophe,
    ATScancode::Grave,
    ATScancode::Comma,
    ATScancode::Period,
    ATScancode::Slash,

    ATScancode::CapsLock,

    ATScancode::F1,
    ATScancode::F2,
    ATScancode::F3,
    ATScancode::F4,
    ATScancode::F5,
    ATScancode::F6,
    ATScancode::F7,
    ATScancode::F8,
    ATScancode::F9,
    ATScancode::F10,
    ATScancode::F11,
    ATScancode::F12,

    ATScancode::Invalid, // PrintScreen
    ATScancode::ScrollLock,
    ATScancode::Invalid, // Pause
    ATScancode::Insert,
    
    ATScancode::Home,
    ATScancode::PageUp,
    ATScancode::Delete,
    ATScancode::End,
    ATScancode::PageDown,
    ATScancode::Right,
    ATScancode::Left,
    ATScancode::Down,
    ATScancode::Up,

    ATScancode::NumLock,

    ATScancode::KPDivide,
    ATScancode::KPMultiply,
    ATScancode::KPMinus,
    ATScancode::KPPlus,
    ATScancode::KPEnter,
    ATScancode::KP1,
    ATScancode::KP2,
    ATScancode::KP3,
    ATScancode::KP4,
    ATScancode::KP5,
    ATScancode::KP6,
    ATScancode::KP7,
    ATScancode::KP8,
    ATScancode::KP9,
    ATScancode::KP0,
    ATScancode::KPPeriod,

    ATScancode::NonUSBackslash,

    ATScancode::Application,
    ATScancode::Power,

    ATScancode::KPEquals,

    // F13-F24
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,

    // no mapping
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,
    ATScancode::Invalid,

    ATScancode::KPComma,
    ATScancode::Invalid,

    ATScancode::International1,
    ATScancode::International2,
    ATScancode::International3,
    ATScancode::International4,
    ATScancode::International5,
    ATScancode::International6,
    ATScancode::Invalid, // ...7
    ATScancode::Invalid, // ...8
    ATScancode::Invalid, // ...9
    ATScancode::Lang1,
    ATScancode::Lang2,
    ATScancode::Lang3,
    ATScancode::Lang4,
    ATScancode::Lang5,

    // ... some media keys
};

static const ATScancode modMap[]
{
    ATScancode::LeftCtrl,
    ATScancode::LeftShift,
    ATScancode::LeftAlt,
    ATScancode::LeftGUI,
    ATScancode::RightCtrl,
    ATScancode::RightShift,
    ATScancode::RightAlt,
    ATScancode::RightGUI,
};

static uint8_t lastKeys[6]{0, 0, 0, 0, 0};
static uint8_t lastKeyMod = 0;

void update_key_state(ATScancode code, bool state);
void update_mouse_state(int8_t x, int8_t y, bool left, bool right);

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
    // request report if it's a keyboard/mouse
    auto protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if(protocol == HID_ITF_PROTOCOL_KEYBOARD || protocol == HID_ITF_PROTOCOL_MOUSE)
        tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    auto protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if(protocol == HID_ITF_PROTOCOL_KEYBOARD)
    {
        auto keyboardReport = (hid_keyboard_report_t const*) report;

        // check for new keys down
        for(int i = 0; i < 6 && keyboardReport->keycode[i]; i++)
        {
            auto key = keyboardReport->keycode[i];
            bool found = false;
            for(int j = 0; j < 6 && lastKeys[j] && !found; j++)
                found = lastKeys[j] == key;

            if(found)
                continue;

            if(key < std::size(scancodeMap) && scancodeMap[key] != ATScancode::Invalid)
                update_key_state(scancodeMap[key], true);
            else
                printf("key down %i %i\n", i, key);
        }

        // do the reverse and check for released keys
        for(int i = 0; i < 6 && lastKeys[i]; i++)
        {
            auto key = lastKeys[i];
            bool found = false;
            for(int j = 0; j < 6 && keyboardReport->keycode[j] && !found; j++)
                found = keyboardReport->keycode[j] == key;

            if(found)
                continue;

            if(key < std::size(scancodeMap) && scancodeMap[key] != ATScancode::Invalid)
                update_key_state(scancodeMap[key], false);
            else
                printf("key up %i %i\n", i, key);
        }

        // ...and mods
        auto changedMods = lastKeyMod ^ keyboardReport->modifier;
        auto pressedMods = changedMods & keyboardReport->modifier;
        auto releasedMods = changedMods ^ pressedMods;
        
        for(int i = 0; i < 8; i++)
        {
            if(modMap[i] == ATScancode::Invalid)
                continue;

            if(pressedMods & (1 << i))
                update_key_state(modMap[i], true);
            else if(releasedMods & (1 << i))
                update_key_state(modMap[i], false);
        }

        memcpy(lastKeys, keyboardReport->keycode, 6);
        lastKeyMod = keyboardReport->modifier;

        tuh_hid_receive_report(dev_addr, instance);
    }
    else if(protocol == HID_ITF_PROTOCOL_MOUSE)
    {
        auto mouseReport = (hid_mouse_report_t const*) report;

        update_mouse_state(mouseReport->x, mouseReport->y, mouseReport->buttons & MOUSE_BUTTON_LEFT, mouseReport->buttons & MOUSE_BUTTON_RIGHT);

        tuh_hid_receive_report(dev_addr, instance);
    }
}