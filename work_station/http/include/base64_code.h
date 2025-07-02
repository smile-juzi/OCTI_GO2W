#include <string>
class Base64Encoder {
public:
    static std::string encode(const unsigned char* data, size_t length) {
        const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string encoded;
        int val = 0;
        unsigned char char_array_3[3], char_array_4[4];
        
        for (size_t i = 0; i < length; ) {
            char_array_3[0] = data[i++];
            char_array_3[1] = (i < length) ? data[i++] : 0;
            char_array_3[2] = (i < length) ? data[i++] : 0;

            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) | ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) | ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (int j = 0; j < 4; j++) {
                encoded += base64_chars[char_array_4[j]];
            }
        }

        // 处理填充字符
        switch (length % 3) {
            case 1: encoded.replace(encoded.end()-2, encoded.end(), "=="); break;
            case 2: encoded.replace(encoded.end()-1, encoded.end(), "="); break;
        }
        return encoded;
    }
};