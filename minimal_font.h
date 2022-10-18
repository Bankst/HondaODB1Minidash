#ifndef MINIMAL_FONT_H
#define MINIMAL_FONT_H

// a 5x7 font table
const unsigned char BasicFont[] PROGMEM = {
// This appears to be using Code Page 437
// Hex codes provided because you probably aren't
  0x00, 0x00, 0x00, 0x00, 0x00, //    \x00
  0x3E, 0x5B, 0x4F, 0x5B, 0x3E, // ☺  \x01
  0x3E, 0x6B, 0x4F, 0x6B, 0x3E, // ☻  \x02
  0x1C, 0x3E, 0x7C, 0x3E, 0x1C, // ♥  \x03
  0x18, 0x3C, 0x7E, 0x3C, 0x18, // ♦  \x04
  0x1C, 0x57, 0x7D, 0x57, 0x1C, // ♣  \x05
  0x1C, 0x5E, 0x7F, 0x5E, 0x1C, // ♠  \x06
  0x00, 0x18, 0x3C, 0x18, 0x00, // •  \x07
  0xFF, 0xE7, 0xC3, 0xE7, 0xFF, // ◘  \x08
  0x00, 0x18, 0x24, 0x18, 0x00, // ○  \x09
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF, // ◙  \x0a
  0x30, 0x48, 0x3A, 0x06, 0x0E, // ♂  \x0b
  0x26, 0x29, 0x79, 0x29, 0x26, // ♀  \x0c
  0x40, 0x7F, 0x05, 0x05, 0x07, // ♪  \x0d
  0x40, 0x7F, 0x05, 0x25, 0x3F, // ♫  \x0e
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A, // ☼  \x0f
  0x7F, 0x3E, 0x1C, 0x1C, 0x08, // ►  \x10
  0x08, 0x1C, 0x1C, 0x3E, 0x7F, // ◀  \x11
  0x14, 0x22, 0x7F, 0x22, 0x14, // ↕  \x12
  0x5F, 0x5F, 0x00, 0x5F, 0x5F, // ‼  \x13
  0x06, 0x09, 0x7F, 0x01, 0x7F, // ¶  \x14
  0x00, 0x66, 0x89, 0x95, 0x6A, // §  \x15
  0x60, 0x60, 0x60, 0x60, 0x60, // ▬  \x16
  0x94, 0xA2, 0xFF, 0xA2, 0x94, // ↨  \x17
  0x08, 0x04, 0x7E, 0x04, 0x08, // ↑  \x18
  0x10, 0x20, 0x7E, 0x20, 0x10, // ↓  \x19
  0x08, 0x08, 0x2A, 0x1C, 0x08, // →  \x1a
  0x08, 0x1C, 0x2A, 0x08, 0x08, // ←  \x1b
  0x1E, 0x10, 0x10, 0x10, 0x10, // ∟  \x1c
  0x0C, 0x1E, 0x0C, 0x1E, 0x0C, // ↔  \x1d
  0x30, 0x38, 0x3E, 0x38, 0x30, // ▲  \x1e
  0x06, 0x0E, 0x3E, 0x0E, 0x06, // ▼  \x1f
  0x00, 0x00, 0x00, 0x00, 0x00, // SP  \x20
  0x00, 0x00, 0x5F, 0x00, 0x00, // !  \x21
  0x00, 0x07, 0x00, 0x07, 0x00, // "  \x22
  0x14, 0x7F, 0x14, 0x7F, 0x14, // #  \x23
  0x24, 0x2A, 0x7F, 0x2A, 0x12, // $  \x24
  0x23, 0x13, 0x08, 0x64, 0x62, // %  \x25
  0x36, 0x49, 0x56, 0x20, 0x50, // &  \x26
  0x00, 0x08, 0x07, 0x03, 0x00, // '  \x27
  0x00, 0x1C, 0x22, 0x41, 0x00, // (  \x28
  0x00, 0x41, 0x22, 0x1C, 0x00, // )  \x29
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A, // *  \x2a
  0x08, 0x08, 0x3E, 0x08, 0x08, // +  \x2b
  0x00, 0x80, 0x70, 0x30, 0x00, // ,  \x2c
  0x08, 0x08, 0x08, 0x08, 0x08, // -  \x2d
  0x00, 0x00, 0x60, 0x60, 0x00, // .  \x2e
  0x20, 0x10, 0x08, 0x04, 0x02, // /  \x2f
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 0  \x30
  0x00, 0x42, 0x7F, 0x40, 0x00, // 1  \x31
  0x72, 0x49, 0x49, 0x49, 0x46, // 2  \x32
  0x21, 0x41, 0x49, 0x4D, 0x33, // 3  \x33
  0x18, 0x14, 0x12, 0x7F, 0x10, // 4  \x34
  0x27, 0x45, 0x45, 0x45, 0x39, // 5  \x35
  0x3C, 0x4A, 0x49, 0x49, 0x31, // 6  \x36
  0x41, 0x21, 0x11, 0x09, 0x07, // 7  \x37
  0x36, 0x49, 0x49, 0x49, 0x36, // 8  \x38
  0x46, 0x49, 0x49, 0x29, 0x1E, // 9  \x39
  0x00, 0x00, 0x14, 0x00, 0x00, // :  \x3a
  0x00, 0x40, 0x34, 0x00, 0x00, // ;  \x3b
  0x00, 0x08, 0x14, 0x22, 0x41, // <  \x3c
  0x14, 0x14, 0x14, 0x14, 0x14, // =  \x3d
  0x00, 0x41, 0x22, 0x14, 0x08, // >  \x3e
  0x02, 0x01, 0x59, 0x09, 0x06, // ?  \x3f
  0x3E, 0x41, 0x5D, 0x59, 0x4E, // @  \x40
  0x7C, 0x12, 0x11, 0x12, 0x7C, // A  \x41
  0x7F, 0x49, 0x49, 0x49, 0x36, // B  \x42
  0x3E, 0x41, 0x41, 0x41, 0x22, // C  \x43
  0x7F, 0x41, 0x41, 0x41, 0x3E, // D  \x44
  0x7F, 0x49, 0x49, 0x49, 0x41, // E  \x45
  0x7F, 0x09, 0x09, 0x09, 0x01, // F  \x46
  0x3E, 0x41, 0x41, 0x51, 0x73, // G  \x47
  0x7F, 0x08, 0x08, 0x08, 0x7F, // H  \x48
  0x00, 0x41, 0x7F, 0x41, 0x00, // I  \x49
  0x20, 0x40, 0x41, 0x3F, 0x01, // J  \x4a
  0x7F, 0x08, 0x14, 0x22, 0x41, // K  \x4b
  0x7F, 0x40, 0x40, 0x40, 0x40, // L  \x4c
  0x7F, 0x02, 0x1C, 0x02, 0x7F, // M  \x4d
  0x7F, 0x04, 0x08, 0x10, 0x7F, // N  \x4e
  0x3E, 0x41, 0x41, 0x41, 0x3E, // O  \x4f
  0x7F, 0x09, 0x09, 0x09, 0x06, // P  \x50
  0x3E, 0x41, 0x51, 0x21, 0x5E, // Q  \x51
  0x7F, 0x09, 0x19, 0x29, 0x46, // R  \x52
  0x26, 0x49, 0x49, 0x49, 0x32, // S  \x53
  0x03, 0x01, 0x7F, 0x01, 0x03, // T  \x54
  0x3F, 0x40, 0x40, 0x40, 0x3F, // U  \x55
  0x1F, 0x20, 0x40, 0x20, 0x1F, // V  \x56
  0x3F, 0x40, 0x38, 0x40, 0x3F, // W  \x57
  0x63, 0x14, 0x08, 0x14, 0x63, // X  \x58
  0x03, 0x04, 0x78, 0x04, 0x03, // Y  \x59
  0x61, 0x59, 0x49, 0x4D, 0x43, // Z  \x5a
  0x00, 0x7F, 0x41, 0x41, 0x41, // [  \x5b
  0x02, 0x04, 0x08, 0x10, 0x20, // \  \x5c
  0x00, 0x41, 0x41, 0x41, 0x7F, // ]  \x5d
  0x04, 0x02, 0x01, 0x02, 0x04, // ^  \x5e
  0x40, 0x40, 0x40, 0x40, 0x40, // _  \x5f
  0x00, 0x03, 0x07, 0x08, 0x00, // `  \x60
  0x20, 0x54, 0x54, 0x78, 0x40, // a  \x61
  0x7F, 0x28, 0x44, 0x44, 0x38, // b  \x62
  0x38, 0x44, 0x44, 0x44, 0x28, // c  \x63
  0x38, 0x44, 0x44, 0x28, 0x7F, // d  \x64
  0x38, 0x54, 0x54, 0x54, 0x18, // e  \x65
  0x00, 0x08, 0x7E, 0x09, 0x02, // f  \x66
  0x18, 0xA4, 0xA4, 0x9C, 0x78, // g  \x67
  0x7F, 0x08, 0x04, 0x04, 0x78, // h  \x68
  0x00, 0x44, 0x7D, 0x40, 0x00, // i  \x69
  0x20, 0x40, 0x40, 0x3D, 0x00, // j  \x6a
  0x7F, 0x10, 0x28, 0x44, 0x00, // k  \x6b
  0x00, 0x41, 0x7F, 0x40, 0x00, // l  \x6c
  0x7C, 0x04, 0x78, 0x04, 0x78, // m  \x6d
  0x7C, 0x08, 0x04, 0x04, 0x78, // n  \x6e
  0x38, 0x44, 0x44, 0x44, 0x38, // o  \x6f
  0xFC, 0x18, 0x24, 0x24, 0x18, // p  \x70
  0x18, 0x24, 0x24, 0x18, 0xFC, // q  \x71
  0x7C, 0x08, 0x04, 0x04, 0x08, // r  \x72
  0x48, 0x54, 0x54, 0x54, 0x24, // s  \x73
  0x04, 0x04, 0x3F, 0x44, 0x24, // t  \x74
  0x3C, 0x40, 0x40, 0x20, 0x7C, // u  \x75
  0x1C, 0x20, 0x40, 0x20, 0x1C, // v  \x76
  0x3C, 0x40, 0x30, 0x40, 0x3C, // w  \x77
  0x44, 0x28, 0x10, 0x28, 0x44, // x  \x78
  0x4C, 0x90, 0x90, 0x90, 0x7C, // y  \x79
  0x44, 0x64, 0x54, 0x4C, 0x44, // z  \x7a
  0x00, 0x08, 0x36, 0x41, 0x00, // {  \x7b
  0x00, 0x00, 0x77, 0x00, 0x00, // |  \x7c
  0x00, 0x41, 0x36, 0x08, 0x00, // }  \x7d
  0x02, 0x01, 0x02, 0x04, 0x02, // ~  \x7e
  0x3C, 0x26, 0x23, 0x26, 0x3C, // ⌂  \x7f
  0x1E, 0xA1, 0xA1, 0x61, 0x12, // Ç  \x80
  0x3A, 0x40, 0x40, 0x20, 0x7A, // ü  \x81
  0x38, 0x54, 0x54, 0x55, 0x59, // é  \x82
  0x21, 0x55, 0x55, 0x79, 0x41, // â  \x83
  0x21, 0x54, 0x54, 0x78, 0x41, // ä  \x84
  0x21, 0x55, 0x54, 0x78, 0x40, // à  \x85
  0x20, 0x54, 0x55, 0x79, 0x40, // å  \x86
  0x0C, 0x1E, 0x52, 0x72, 0x12, // ç  \x87
  0x39, 0x55, 0x55, 0x55, 0x59, // ê  \x88
  0x39, 0x54, 0x54, 0x54, 0x59, // ë  \x89
  0x39, 0x55, 0x54, 0x54, 0x58, // è  \x8a
  0x00, 0x00, 0x45, 0x7C, 0x41, // ï  \x8b
  0x00, 0x02, 0x45, 0x7D, 0x42, // î  \x8c
  0x00, 0x01, 0x45, 0x7C, 0x40, // ì  \x8d
  0xF0, 0x29, 0x24, 0x29, 0xF0, // Ä  \x8e
  0xF0, 0x28, 0x25, 0x28, 0xF0, // Å  \x8f
  0x7C, 0x54, 0x55, 0x45, 0x00, // É  \x90
  0x20, 0x54, 0x54, 0x7C, 0x54, // æ  \x91
  0x7C, 0x0A, 0x09, 0x7F, 0x49, // Æ  \x92
  0x32, 0x49, 0x49, 0x49, 0x32, // ô  \x93
  0x32, 0x48, 0x48, 0x48, 0x32, // ö  \x94
  0x32, 0x4A, 0x48, 0x48, 0x30, // ò  \x95
  0x3A, 0x41, 0x41, 0x21, 0x7A, // û  \x96
  0x3A, 0x42, 0x40, 0x20, 0x78, // ù  \x97
  0x00, 0x9D, 0xA0, 0xA0, 0x7D, // ÿ  \x98
  0x39, 0x44, 0x44, 0x44, 0x39, // Ö  \x99
  0x3D, 0x40, 0x40, 0x40, 0x3D, // Ü  \x9a
  0x3C, 0x24, 0xFF, 0x24, 0x24, // ¢  \x9b
  0x48, 0x7E, 0x49, 0x43, 0x66, // £  \x9c
  0x2B, 0x2F, 0xFC, 0x2F, 0x2B, // ¥  \x9d
  0xFF, 0x09, 0x29, 0xF6, 0x20, // ₧  \x9e
  0xC0, 0x88, 0x7E, 0x09, 0x03, // ƒ  \x9f
  0x20, 0x54, 0x54, 0x79, 0x41, // á  \xa0
  0x00, 0x00, 0x44, 0x7D, 0x41, // í  \xa1
  0x30, 0x48, 0x48, 0x4A, 0x32, // ó  \xa2
  0x38, 0x40, 0x40, 0x22, 0x7A, // ú  \xa3
  0x00, 0x7A, 0x0A, 0x0A, 0x72, // ñ  \xa4
  0x7D, 0x0D, 0x19, 0x31, 0x7D, // Ñ  \xa5
  0x26, 0x29, 0x29, 0x2F, 0x28, // ª  \xa6
  0x26, 0x29, 0x29, 0x29, 0x26, // º  \xa7
  0x30, 0x48, 0x4D, 0x40, 0x20, // ¿  \xa8
  0x38, 0x08, 0x08, 0x08, 0x08, // ⌐  \xa9
  0x08, 0x08, 0x08, 0x08, 0x38, // ¬  \xaa
  0x2F, 0x10, 0xC8, 0xAC, 0xBA, // ½  \xab
  0x2F, 0x10, 0x28, 0x34, 0xFA, // ¼  \xac
  0x00, 0x00, 0x7B, 0x00, 0x00, // ¡  \xad
  0x08, 0x14, 0x2A, 0x14, 0x22, // «  \xae
  0x22, 0x14, 0x2A, 0x14, 0x08, // »  \xaf
  0xAA, 0x00, 0x55, 0x00, 0xAA, // ░  \xb0
  0xAA, 0x55, 0xAA, 0x55, 0xAA, // ▒  \xb1
  0xAA, 0xFF, 0xAA, 0xFF, 0xAA, // ▓  \xb2
  0x00, 0x00, 0x00, 0xFF, 0x00, // │  \xb3
  0x10, 0x10, 0x10, 0xFF, 0x00, // ┤  \xb4
  0x14, 0x14, 0x14, 0xFF, 0x00, // ╡  \xb5
  0x10, 0x10, 0xFF, 0x00, 0xFF, // ╢  \xb6
  0x10, 0x10, 0xF0, 0x10, 0xF0, // ╖  \xb7
  0x14, 0x14, 0x14, 0xFC, 0x00, // ╕  \xb8
  0x14, 0x14, 0xF7, 0x00, 0xFF, // ╣  \xb9
  0x00, 0x00, 0xFF, 0x00, 0xFF, // ║  \xba
  0x14, 0x14, 0xF4, 0x04, 0xFC, // ╗  \xbb
  0x14, 0x14, 0x17, 0x10, 0x1F, // ╝  \xbc
  0x10, 0x10, 0x1F, 0x10, 0x1F, // ╜  \xbd
  0x14, 0x14, 0x14, 0x1F, 0x00, // ╛  \xbe
  0x10, 0x10, 0x10, 0xF0, 0x00, // ┐  \xbf
  0x00, 0x00, 0x00, 0x1F, 0x10, // └  \xc0
  0x10, 0x10, 0x10, 0x1F, 0x10, // ┴  \xc1
  0x10, 0x10, 0x10, 0xF0, 0x10, // ┬  \xc2
  0x00, 0x00, 0x00, 0xFF, 0x10, // ├  \xc3
  0x10, 0x10, 0x10, 0x10, 0x10, // ─  \xc4
  0x10, 0x10, 0x10, 0xFF, 0x10, // ┼  \xc5
  0x00, 0x00, 0x00, 0xFF, 0x14, // ╞  \xc6
  0x00, 0x00, 0xFF, 0x00, 0xFF, // ╟  \xc7
  0x00, 0x00, 0x1F, 0x10, 0x17, // ╚  \xc8
  0x00, 0x00, 0xFC, 0x04, 0xF4, // ╔  \xc9
  0x14, 0x14, 0x17, 0x10, 0x17, // ╩  \xca
  0x14, 0x14, 0xF4, 0x04, 0xF4, // ╦  \xcb
  0x00, 0x00, 0xFF, 0x00, 0xF7, // ╠  \xcc
  0x14, 0x14, 0x14, 0x14, 0x14, // ═  \xcd
  0x14, 0x14, 0xF7, 0x00, 0xF7, // ╬  \xce
  0x14, 0x14, 0x14, 0x17, 0x14, // ╧  \xcf
  0x10, 0x10, 0x1F, 0x10, 0x1F, // ╨  \xd0
  0x14, 0x14, 0x14, 0xF4, 0x14, // ╤  \xd1
  0x10, 0x10, 0xF0, 0x10, 0xF0, // ╥  \xd2
  0x00, 0x00, 0x1F, 0x10, 0x1F, // ╙  \xd3
  0x00, 0x00, 0x00, 0x1F, 0x14, // ╘  \xd4
  0x00, 0x00, 0x00, 0xFC, 0x14, // ╒  \xd5
  0x00, 0x00, 0xF0, 0x10, 0xF0, // ╓  \xd6
  0x10, 0x10, 0xFF, 0x10, 0xFF, // ╫  \xd7
  0x14, 0x14, 0x14, 0xFF, 0x14, // ╪  \xd8
  0x10, 0x10, 0x10, 0x1F, 0x00, // ┘  \xd9
  0x00, 0x00, 0x00, 0xF0, 0x10, // ┌  \xda
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // █  \xdb
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0, // ▄  \xdc
  0xFF, 0xFF, 0xFF, 0x00, 0x00, // ▌  \xdd
  0x00, 0x00, 0x00, 0xFF, 0xFF, // ▐  \xde
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // ▀  \xdf
  0x38, 0x44, 0x44, 0x38, 0x44, // α  \xe0
  0x7C, 0x2A, 0x2A, 0x3E, 0x14, // ß  \xe1
  0x7E, 0x02, 0x02, 0x06, 0x06, // Γ  \xe2
  0x02, 0x7E, 0x02, 0x7E, 0x02, // π  \xe3
  0x63, 0x55, 0x49, 0x41, 0x63, // Σ  \xe4
  0x38, 0x44, 0x44, 0x3C, 0x04, // σ  \xe5
  0x40, 0x7E, 0x20, 0x1E, 0x20, // µ  \xe6
  0x06, 0x02, 0x7E, 0x02, 0x02, // τ  \xe7
  0x99, 0xA5, 0xE7, 0xA5, 0x99, // Φ  \xe8
  0x1C, 0x2A, 0x49, 0x2A, 0x1C, // Θ  \xe9
  0x4C, 0x72, 0x01, 0x72, 0x4C, // Ω  \xea
  0x30, 0x4A, 0x4D, 0x4D, 0x30, // δ  \xeb
  0x30, 0x48, 0x78, 0x48, 0x30, // ∞  \xec
  0xBC, 0x62, 0x5A, 0x46, 0x3D, // φ  \xed
  0x3E, 0x49, 0x49, 0x49, 0x00, // ε  \xee
  0x7E, 0x01, 0x01, 0x01, 0x7E, // ∩  \xef
  0x2A, 0x2A, 0x2A, 0x2A, 0x2A, // ≡  \xf0
  0x44, 0x44, 0x5F, 0x44, 0x44, // ±  \xf1
  0x40, 0x51, 0x4A, 0x44, 0x40, // ≥  \xf2
  0x40, 0x44, 0x4A, 0x51, 0x40, // ≤  \xf3
  0x00, 0x00, 0xFF, 0x01, 0x03, // ⌠  \xf4
  0xE0, 0x80, 0xFF, 0x00, 0x00, // ⌡  \xf5
  0x08, 0x08, 0x6B, 0x6B, 0x08, // ÷  \xf6
  0x36, 0x12, 0x36, 0x24, 0x36, // ≈  \xf7
  0x06, 0x0F, 0x09, 0x0F, 0x06, // °  \xf8
  0x00, 0x00, 0x18, 0x18, 0x00, // ∙  \xf9
  0x00, 0x00, 0x10, 0x10, 0x00, // ·  \xfa
  0x30, 0x40, 0xFF, 0x01, 0x01, // √  \xfb
  0x00, 0x1F, 0x01, 0x01, 0x1E, // ⁿ  \xfc
  0x00, 0x19, 0x1D, 0x17, 0x12, // ²  \xfd
  0x00, 0x3C, 0x3C, 0x3C, 0x3C, // ■  \xfe
  0x00, 0x00, 0x00, 0x00, 0x00, // NBSP  \xff
};

/*

typedef uint32_t u8chr_t;
static const uint8_t u8_length[] = {
	// 0 1 2 3 4 5 6 7 8 9 A B C D E F
		 1,1,1,1,1,1,1,1,0,0,0,0,2,2,3,4
};
#define u8length(s) u8_length[(((uint8_t *)(s))[0] & 0xFF) >> 4];

int u8chrisvalid(u8chr_t c)
{
  if (c <= 0x7F) return 1;                    // [1]

  if (0xC280 <= c && c <= 0xDFBF)             // [2]
     return ((c & 0xE0C0) == 0xC080);

  if (0xEDA080 <= c && c <= 0xEDBFBF)         // [3]
     return 0; // Reject UTF-16 surrogates

  if (0xE0A080 <= c && c <= 0xEFBFBF)         // [4]
     return ((c & 0xF0C0C0) == 0xE08080);

  if (0xF0908080 <= c && c <= 0xF48FBFBF)     // [5]
     return ((c & 0xF8C0C0C0) == 0xF0808080);

  return 0;
}

int u8next(char *txt, u8chr_t *ch) {
	u8chr_t encoding = 0;

	int len = u8length(txt);

	for (int i=0; i<len && txt[i] != '\0'; i++) {
		encoding = (encoding << 8) | txt[i];
	}

	// errno = 0;
	if (len == 0 || !u8chrisvalid(encoding)) {
		encoding = txt[0];
		len = 1;
		// errno = -1;
	}

	if (ch) *ch = encoding;

	return encoding ? len : 0;
}

unsigned char specialCharTable(u8chr_t chr) {
  if (chr == ' ') return 0x00;
  if (chr == 0x263A) return 0x01; // ☺
  if (chr == 0x2634) return 0x02; // ☻
  if (chr == 0x2665) return 0x03; // ♥
  if (chr == 0x2666) return 0x04; // ♦
  if (chr == 0x2663) return 0x05; // ♣
  if (chr == 0x2660) return 0x06; // ♠
  if (chr == 0x2022) return 0x07; // •
  if (chr == 0x25D8) return 0x08; // ◘
  if (chr == 0x25CB) return 0x09; // ○
  if (chr == 0x25D9) return 0x0a; // ◙
  if (chr == 0x2642) return 0x0b; // ♂
  if (chr == 0x2640) return 0x0c; // ♀
  if (chr == 0x266A) return 0x0d; // ♪
  if (chr == 0x266B) return 0x0e; // ♫
  if (chr == 0x263C) return 0x0f; // ☼
  if (chr == 0x25BA) return 0x10; // ►
  if (chr == 0x25C0) return 0x11; // ◀
  if (chr == 0x2195) return 0x12; // ↕
  if (chr == 0x203C) return 0x13; // ‼
  if (chr == 0x00B6) return 0x14; // ¶
  if (chr == 0x00A7) return 0x15; // §
  if (chr == 0x25AC) return 0x16; // ▬
  if (chr == 0x21A8) return 0x17; // ↨
  if (chr == 0x2191) return 0x18; // ↑
  if (chr == 0x2193) return 0x19; // ↓
  if (chr == 0x2192) return 0x1a; // →
  if (chr == 0x2190) return 0x1b; // ←
  if (chr == 0x221F) return 0x1c; // ∟
  if (chr == 0x2194) return 0x1d; // ↔
  if (chr == 0x25B2) return 0x1e; // ▲
  if (chr == 0x25BC) return 0x1f; // ▼
	/*
  if (chr == 0x0) return 0x7f; // ⌂
  if (chr == 0x0) return 0x80; // Ç
  if (chr == 0x0) return 0x81; // ü
  if (chr == 0x0) return 0x82; // é
  if (chr == 0x0) return 0x83; // â
  if (chr == 0x0) return 0x84; // ä
  if (chr == 0x0) return 0x85; // à
  if (chr == 0x0) return 0x86; // å
  if (chr == 0x0) return 0x87; // ç
  if (chr == 0x0) return 0x88; // ê
  if (chr == 0x0) return 0x89; // ë
  if (chr == 0x0) return 0x8a; // è
  if (chr == 0x0) return 0x8b; // ï
  if (chr == 0x0) return 0x8c; // î
  if (chr == 0x0) return 0x8d; // ì
  if (chr == 0x0) return 0x8e; // Ä
  if (chr == 0x0) return 0x8f; // Å
  if (chr == 0x0) return 0x90; // É
  if (chr == 0x0) return 0x91; // æ
  if (chr == 0x0) return 0x92; // Æ
  if (chr == 0x0) return 0x93; // ô
  if (chr == 0x0) return 0x94; // ö
  if (chr == 0x0) return 0x95; // ò
  if (chr == 0x0) return 0x96; // û
  if (chr == 0x0) return 0x97; // ù
  if (chr == 0x0) return 0x98; // ÿ
  if (chr == 0x0) return 0x99; // Ö
  if (chr == 0x0) return 0x9a; // Ü
  if (chr == 0x0) return 0x9b; // ¢
  if (chr == 0x0) return 0x9c; // £
  if (chr == 0x0) return 0x9d; // ¥
  if (chr == 0x0) return 0x9e; // ₧
  if (chr == 0x0) return 0x9f; // ƒ
  if (chr == 0x0) return 0xa0; // á
  if (chr == 0x0) return 0xa1; // í
  if (chr == 0x0) return 0xa2; // ó
  if (chr == 0x0) return 0xa3; // ú
  if (chr == 0x0) return 0xa4; // ñ
  if (chr == 0x0) return 0xa5; // Ñ
  if (chr == 0x0) return 0xa6; // ª
  if (chr == 0x0) return 0xa7; // º
  if (chr == 0x0) return 0xa8; // ¿
  if (chr == 0x0) return 0xa9; // ⌐
  if (chr == 0x0) return 0xaa; // ¬
  if (chr == 0x0) return 0xab; // ½
  if (chr == 0x0) return 0xac; // ¼
  if (chr == 0x0) return 0xad; // ¡
  if (chr == 0x0) return 0xae; // «
  if (chr == 0x0) return 0xaf; // »
  if (chr == 0x0) return 0xb0; // ░
  if (chr == 0x0) return 0xb1; // ▒
  if (chr == 0x0) return 0xb2; // ▓
  if (chr == 0x0) return 0xb3; // │
  if (chr == 0x0) return 0xb4; // ┤
  if (chr == 0x0) return 0xb5; // ╡
  if (chr == 0x0) return 0xb6; // ╢
  if (chr == 0x0) return 0xb7; // ╖
  if (chr == 0x0) return 0xb8; // ╕
  if (chr == 0x0) return 0xb9; // ╣
  if (chr == 0x0) return 0xba; // ║
  if (chr == 0x0) return 0xbb; // ╗
  if (chr == 0x0) return 0xbc; // ╝
  if (chr == 0x0) return 0xbd; // ╜
  if (chr == 0x0) return 0xbe; // ╛
  if (chr == 0x0) return 0xbf; // ┐
  if (chr == 0x0) return 0xc0; // └
  if (chr == 0x0) return 0xc1; // ┴
  if (chr == 0x0) return 0xc2; // ┬
  if (chr == 0x0) return 0xc3; // ├
  if (chr == 0x0) return 0xc4; // ─
  if (chr == 0x0) return 0xc5; // ┼
  if (chr == 0x0) return 0xc6; // ╞
  if (chr == 0x0) return 0xc7; // ╟
  if (chr == 0x0) return 0xc8; // ╚
  if (chr == 0x0) return 0xc9; // ╔
  if (chr == 0x0) return 0xca; // ╩
  if (chr == 0x0) return 0xcb; // ╦
  if (chr == 0x0) return 0xcc; // ╠
  if (chr == 0x0) return 0xcd; // ═
  if (chr == 0x0) return 0xce; // ╬
  if (chr == 0x0) return 0xcf; // ╧
  if (chr == 0x0) return 0xd0; // ╨
  if (chr == 0x0) return 0xd1; // ╤
  if (chr == 0x0) return 0xd2; // ╥
  if (chr == 0x0) return 0xd3; // ╙
  if (chr == 0x0) return 0xd4; // ╘
  if (chr == 0x0) return 0xd5; // ╒
  if (chr == 0x0) return 0xd6; // ╓
  if (chr == 0x0) return 0xd7; // ╫
  if (chr == 0x0) return 0xd8; // ╪
  if (chr == 0x0) return 0xd9; // ┘
  if (chr == 0x0) return 0xda; // ┌
  if (chr == 0x0) return 0xdb; // █
  if (chr == 0x0) return 0xdc; // ▄
  if (chr == 0x0) return 0xdd; // ▌
  if (chr == 0x0) return 0xde; // ▐
  if (chr == 0x0) return 0xdf; // ▀
  if (chr == 0x0) return 0xe0; // α
  if (chr == 0x0) return 0xe1; // ß
  if (chr == 0x0) return 0xe2; // Γ
  if (chr == 0x0) return 0xe3; // π
  if (chr == 0x0) return 0xe4; // Σ
  if (chr == 0x0) return 0xe5; // σ
  if (chr == 0x0) return 0xe6; // µ
  if (chr == 0x0) return 0xe7; // τ
  if (chr == 0x0) return 0xe8; // Φ
  if (chr == 0x0) return 0xe9; // Θ
  if (chr == 0x0) return 0xea; // Ω
  if (chr == 0x0) return 0xeb; // δ
  if (chr == 0x0) return 0xec; // ∞
  if (chr == 0x0) return 0xed; // φ
  if (chr == 0x0) return 0xee; // ε
  if (chr == 0x0) return 0xef; // ∩
  if (chr == 0x0) return 0xf0; // ≡
  if (chr == 0x0) return 0xf1; // ±
  if (chr == 0x0) return 0xf2; // ≥
  if (chr == 0x0) return 0xf3; // ≤
  if (chr == 0x0) return 0xf4; // ⌠
  if (chr == 0x0) return 0xf5; // ⌡
  if (chr == 0x0) return 0xf6; // ÷
  if (chr == 0x0) return 0xf7; // ≈
  if (chr == 0x00B0) return 0xf8; // °
  if (chr == 0x0) return 0xf9; // ∙
  if (chr == 0x0) return 0xfa; // ·
  if (chr == 0x0) return 0xfb; // √
  if (chr == 0x0) return 0xfc; // ⁿ
  if (chr == 0x0) return 0xfd; // ²
  if (chr == 0x0) return 0xfe; // ■
}
*/

#endif // MINIMAL_FONT_H