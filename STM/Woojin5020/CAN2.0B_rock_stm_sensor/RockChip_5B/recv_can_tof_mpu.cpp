/* rock칩에서 stm32와 CAN으로 연결하고
   수신 내용을 디버깅하는 코드 (0x102/0x103 전용 해석) */
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

static inline uint16_t be16(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);   // big-endian -> host
}

int main() {
    // can0 up (이미 up이면 첫 두 줄은 생략 가능)
    system("sudo ip link set can0 down");
    system("sudo ip link set can0 type can bitrate 100000");
    system("sudo ip link set can0 up");

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }

    sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }

    // ---- (선택) 0x102, 0x103만 받도록 필터링 ----
    struct can_filter filt[2];
    filt[0].can_id   = 0x102;
    filt[0].can_mask = CAN_SFF_MASK;
    filt[1].can_id   = 0x103;
    filt[1].can_mask = CAN_SFF_MASK;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0) {
        perror("setsockopt(CAN_RAW_FILTER)");
        // 필터 실패해도 계속 진행은 가능
    }

    printf("Listening on can0 ...\n");
    while (true) {
        struct can_frame fr {};
        ssize_t n = read(s, &fr, sizeof(fr));
        if (n < 0) { perror("read"); break; }
        if ((size_t)n < sizeof(fr)) continue;

        const uint16_t id = fr.can_id & CAN_SFF_MASK;

        // 공통 덤프
        printf("ID=0x%03X DLC=%d DATA=", id, fr.can_dlc);
        for (int i = 0; i < fr.can_dlc; ++i) printf("%02X ", fr.data[i]);
        printf("\n");

        // 0x102: VL53L0X 거리(mm) 2바이트 (BE)
        if (id == 0x102 && fr.can_dlc >= 2) {
            uint16_t dist_mm = be16(fr.data);
            printf("전방 물체 감지 = %u mm\n", dist_mm);
        }

        // 0x103: 자력 |B| (µT) 2바이트 (BE)
        if (id == 0x103 && fr.can_dlc >= 2) {
            uint16_t B_uT = be16(fr.data);
            printf("키즈존 입장 = %u uT\n", B_uT);
        }
    }

    close(s);
    return 0;
}
