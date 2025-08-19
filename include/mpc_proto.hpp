#pragma once
#include <cstdint>
#include <cstring>
#include <chrono>

// Sizes must match solver's expectations.
#ifndef N_TARGET
#define N_TARGET 8
#endif

#ifndef N_P
#define N_P 7
#endif

#ifndef N_X
#define N_X 6
#endif

#ifndef N_U
#define N_U 4
#endif

#ifndef N
#define N 7
#endif
// ================================================

static inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

#pragma pack(push, 1)
struct MsgHeader {
    uint32_t magic;     // 'SOLV' = 0x564C4F53 little-endian
    uint16_t version;   // protocol version
    uint16_t type;      // 1=request, 2=response
    uint32_t seq;       // sequence number
    uint64_t t_ns;      // sender monotonic timestamp (ns)
};

struct MpcRequest {
    MsgHeader h;
    double target[N_TARGET];
    double x0[N_X];
    double u_last[N_U];
    double p[N_P];
};

struct MpcResponse {
    MsgHeader h;
    uint64_t state_t;
    double u_out[N_U*N];
};
#pragma pack(pop)

static constexpr uint32_t MAGIC_SOLV = 0x564C4F53u; // 'SOLV'
static constexpr uint16_t PROTO_VER  = 1;
static constexpr uint16_t TYPE_REQ   = 1;
static constexpr uint16_t TYPE_RESP  = 2;

// helpers
static inline void fill_header(MsgHeader &h, uint16_t type, uint32_t seq) {
    h.magic   = MAGIC_SOLV;
    h.version = PROTO_VER;
    h.type    = type;
    h.seq     = seq;
    h.t_ns    = now_ns();
}

static inline bool header_ok(const MsgHeader &h, uint16_t expect_type) {
    return h.magic == MAGIC_SOLV && h.version == PROTO_VER && h.type == expect_type;
}
