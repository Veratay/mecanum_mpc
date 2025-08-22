#include "fatrop/common/options.hpp"
#include "fatrop/ip_algorithm/ip_alg_builder.hpp"
#include "fatrop/ocp/nlp_ocp.hpp"
#include "fatrop/ocp/ocp_abstract.hpp"
#include "fatrop/ocp/type.hpp"
#include "json.h"
#include "StageOCP.hpp"
#include <iostream>
#include <problem_information.h>
#include <cstdio>
#include <cstring>
#include <memory>
#include <rockit_solver.hpp>
#include <fatrop/fatrop.hpp>
#include <fstream>
#include <string>

using namespace std;

RockitSolver::RockitSolver(string lib, std::ifstream& info) {
    std::stringstream buffer;
    buffer << info.rdbuf();
    json::jobject json_spec = json::jobject::parse(buffer.str());

    fatrop::OptionRegistry options;

    shared_ptr<DLHandler> handle = make_shared<DLHandler>(lib);

    ocp = std::dynamic_pointer_cast<StageOCPRockit>(StageOCPBuilder::FromRockitInterface(handle, json_spec));

    // fatrop::IpAlgBuilder<fatrop::OcpType> builder(std::make_shared<fatrop::NlpOcp>(ocp));
    builder = std::make_unique<fatrop::IpAlgBuilder<fatrop::OcpType>>(fatrop::IpAlgBuilder<fatrop::OcpType>(std::make_shared<fatrop::NlpOcp>(ocp)));
    ip_algorithm = builder->with_options_registry(&options).build();
}

std::vector<double> to_std_vector(const blasfeo_dvec& v)
{
    return std::vector<double>(v.pa, v.pa + v.m);
}

int RockitSolver::solve(casadi_real target[], casadi_real x0[], casadi_real u_last[], casadi_real p[], casadi_real x_out[], casadi_real u_out[]) {
    auto dst = ocp->global_params.data();
    dst = std::copy(x0, x0 + 6, dst);
    dst = std::copy(p, p + 7, dst);
    dst = std::copy(target, target + 8, dst);
    dst = std::copy(u_last, u_last + 4, dst);

    for(int i=0; i<ocp->initial_u.size(); i++) {
        ocp->initial_u[i] = 2.0;
    }

    Timer timer;
    timer.start();
    IpSolverReturnFlag ret = ip_algorithm->optimize();

    std::cout << "Elapsed time: " << timer.stop() << std::endl;

    auto solved_us = to_std_vector(ip_algorithm->solution_dual().vec());

    for (auto u: solved_us)
        std::cout << u << ' ';
    std::cout << std::endl;

    auto data = builder->get_ipdata();
    std::cout << "Return flag: " << int(ret) << std::endl;
    std::cout << "Return flag == success: " << (ret == IpSolverReturnFlag::Success)
              << std::endl;
    std::cout << data->timing_statistics() << std::endl;
    return 0;
}
