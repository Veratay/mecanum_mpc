#include <arpa/inet.h>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <chrono>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>


#include <sys/resource.h>

#include "mpc_proto.hpp"
#include "solver.hpp"

struct SolverCtx {
    int sockfd;
};

static void set_low_latency(int fd) {
    int prio = 1;
    setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &prio, sizeof(prio));
}

// Log a double array with a label
void log_array(const char* name, const double* arr, size_t n) {
    std::cout << name << ": [";
    for (size_t i = 0; i < n; i++) {
        std::cout << std::fixed << std::setprecision(6) << arr[i];
        if (i + 1 < n) std::cout << ", ";
    }
    std::cout << "]\n";
}

// Log the request
void log_request(const MpcRequest& req) {
    std::cout << "===== MPC Request =====\n";
    std::cout << "magic=" << req.h.magic << ", version=" << req.h.version << ", type=" << req.h.type <<"\n";
    std::cout << "seq=" << req.h.seq << ", t_ns=" << req.h.t_ns << "\n";
    log_array("target", req.target, sizeof(req.target)/sizeof(double));
    log_array("x0",     req.x0,     sizeof(req.x0)/sizeof(double));
    log_array("u_last", req.u_last, sizeof(req.u_last)/sizeof(double));
    log_array("p",      req.p,      sizeof(req.p)/sizeof(double));
}

// Log the response
void log_response(const MpcResponse& resp) {
    std::cout << "===== MPC Response =====\n";
    std::cout << "seq=" << resp.h.seq << ", t_ns=" << resp.h.t_ns << "\n";
    log_array("u_out", resp.u_out, sizeof(resp.u_out)/sizeof(double));
}


long get_rss_kb() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss; // KB on Linux
}

int main(int argc, char** argv) {
    // std::printf("Attempting rockit solver.");
    //
    // std::ifstream rfile;
    // rfile.open("./codegen/casadi_codegen.json");
    // std::printf("loaded file\n");
    //
    // RockitSolver rockit_solver("./build/libcodegen.so",rfile);
    //
    // std::printf("initialized solver\n");
    //
    // double r_target[] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 100.0};
    // double r_x0[] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // double r_p[] = {
    //     96.664389,
    //     7.8,
    //     0.27,
    //     0.27,
    //     0.1016,
    //     68.0,
    //     6.0
    // };
    // double r_u_last[] = {0.0, 0.0, 0.0, 0.0};
    //
    // rockit_solver.solve(r_target, r_x0, r_u_last, r_p,  0, 0);

    if (argc != 2) {
        std::fprintf(stderr, "Usage: %s <listen_port>\n", argv[0]);
        return 1;
    }
    int listen_port = std::atoi(argv[1]);

    // UDP socket
    int sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); return 1; }
    set_low_latency(sockfd);

    // Bind
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(listen_port);
    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    std::printf("[solver] listening on UDP %d\n", listen_port);

    // Instantiate solver once, reuse memory
    Solver solver;

    // Reusable buffers
    alignas(8) MpcRequest req{};
    alignas(8) MpcResponse resp{};

    while (true) {
        sockaddr_in client{};
        socklen_t clen = sizeof(client);

        ssize_t n;
        ssize_t tmp_n;
        bool got_packet = false;

        // fetch latest state update to solve
        // if >1 state update has been sent since the last solve started, all are dropped except the last
        do {
            tmp_n = recvfrom(sockfd, &req, sizeof(req), MSG_DONTWAIT, (sockaddr*)&client, &clen);
            if (tmp_n > 0) {
                got_packet = true;
                n = tmp_n;
            }
        } while (tmp_n > 0);

        if (!got_packet) {
            // nothing new, avoid busy spin
            usleep(20); 
            continue;
        }

        // log_request(req);
        
        if ((size_t)n < sizeof(MpcRequest)) {
            std::fprintf(stderr, "[solver] bad packet size=%zd. Expected %zd \n", n, sizeof(MpcRequest));
            continue;
        }

        if (!header_ok(req.h, TYPE_REQ)) {
            std::fprintf(stderr, "[solver] bad header\n");
            continue;
        }

        // Prepare output buffers
        double x_out[N_X*(N+1)]{};
        double u_out[N_U*N]{};

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        long mem_before = get_rss_kb();

        solver.solve(
            (double*)req.target,
            (double*)req.x0,
            (double*)req.u_last,
            (double*)req.p,
            (double*)x_out,
            (double*)u_out
        );
        long mem_after = get_rss_kb();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Solved in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "µs" << std::endl;
        std::cout << "RSS before: " << mem_before << " kB, after: " << mem_after << " kB\n";

        // Fill response
        std::memset(&resp, 0, sizeof(resp));
        fill_header(resp.h, TYPE_RESP, req.h.seq);
        resp.state_t = req.h.t_ns;
        std::memcpy(resp.u_out, u_out, sizeof(u_out));

        //log_response(resp);

        // Send back to sender
        ssize_t sent = sendto(sockfd, &resp, sizeof(resp), 0, (sockaddr*)&client, clen);
        if (sent < 0) perror("sendto");
    }

    close(sockfd);
    return 0;
}
