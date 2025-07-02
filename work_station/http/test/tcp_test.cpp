#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdint.h>

#pragma pack(push, 1)
struct PGMHeader {
    uint32_t magic;      
    uint32_t width;      
    uint32_t height;     
    uint32_t max_gray;   
    uint32_t data_size;  
    uint32_t map_number; 
    uint32_t datalen;    
    float    resolution; 
};

struct PGMData {
    struct PGMHeader header;
    uint8_t pixel_data[]; 
};
#pragma pack(pop)

#define SERVER_IP    "127.0.0.1"
#define SERVER_PORT  1234

struct PGMData* create_pgm_data(
    uint32_t width, 
    uint32_t height, 
    uint8_t max_gray,
    const uint8_t* pixels)
{
    const int bytes_per_pixel = (max_gray > 255) ? 2 : 1;
    const uint32_t data_size = width * height * bytes_per_pixel;
    
    // 显式类型转换
    auto* pgm = static_cast<PGMData*>(std::malloc(sizeof(struct PGMHeader) + data_size));

   // struct PGMData* pgm = (struct PGMData*)malloc(sizeof(struct PGMHeader) + data_size);
    if (!pgm) return NULL;

    pgm->header = {
        .magic      = htonl(0x5035),    
        .width      = htonl(width),
        .height     = htonl(height),
        .max_gray   = htonl(max_gray),
        .data_size  = htonl(data_size),
        .map_number = 0,                    
        .datalen    = htonl(sizeof(struct PGMHeader) + data_size),                    
        .resolution = 0.1f                  
    };

    memcpy(pgm->pixel_data, pixels, data_size);
    return pgm;
}

int main() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        return EXIT_FAILURE;
    }

    struct sockaddr_in serv_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(SERVER_PORT)
    };
    
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("invalid address");
        close(sockfd);
        return EXIT_FAILURE;
    }

    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("connection failed");
        close(sockfd);
        return EXIT_FAILURE;
    }

    uint8_t sample_pixels[] = { 0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF,
    				0x00, 0x7F,0xCD,0xCD,0xFF,0x00, 0x7F,0xCD,0xCD,0xFF
    				}; 
    struct PGMData* pgm = create_pgm_data(10, 8, 255, sample_pixels);
    if (!pgm) {
        fprintf(stderr, "Failed to create PGM data\n");
        close(sockfd);
        return EXIT_FAILURE;
    }

    const size_t total_size = sizeof(struct PGMHeader) + ntohl(pgm->header.data_size);

    // 正确声明变量
    size_t sent = 0;
    while (sent < total_size) {
        ssize_t n = send(sockfd, 
                        (char*)pgm + sent, 
                        total_size - sent, 
                        MSG_NOSIGNAL);
        if (n < 0) {
            perror("send failed");
            break;
        }
        sent += n;
    }

    printf("Sent %zu/%zu bytes\n", sent, total_size);

    std::free(pgm);
    close(sockfd);
    return EXIT_SUCCESS;
}
