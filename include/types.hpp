#ifndef TYPES_HPP_
#define TYPES_HPP_
namespace ros2_custom_controller_sample
{

    typedef union ControlWord
    {
        uint16_t word;
        struct
        {
            bool enable : 1;          // ビット0: 有効化フラグ (true/false)
            unsigned int reserved_1 : 3; // ビット1-3: 予約領域（将来の拡張用）
            unsigned int reserved_2 : 4; // ビット4-7: 予約領域（将来の拡張用）
            unsigned int reserved_3 : 4; // ビット8-11: 予約領域（将来の拡張用）
            unsigned int reserved_4 : 4; // ビット12-15: 予約領域（将来の拡張用）
        } bits;

    } ControlWord;
    
}

#endif