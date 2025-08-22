#ifndef MPCC_ROCKIT_SOLVER_H
#define MPCC_ROCKIT_SOLVER_H

#include "CasadiCodegen.hpp"
#include "StageOCP.hpp"
#include "fatrop/ocp/type.hpp"
#include <fatrop/ip_algorithm/ip_algorithm.hpp>
#include <fatrop/ocp/ocp_abstract.hpp>
#include <casadi_codegen.h>
#include <map>
#include <memory>
#include <string>

#define casadi_real double

using namespace fatrop;
class RockitSolver {
private:
    std::shared_ptr<fatrop::IpAlgorithm<fatrop::OcpType>> ip_algorithm;
    std::shared_ptr<StageOCPRockit> ocp;
    std::unique_ptr<fatrop::IpAlgBuilder<fatrop::OcpType>> builder;
public:
    RockitSolver(std::string lib, std::ifstream& t);

    int solve(casadi_real target[], casadi_real x0[], casadi_real u_last[], casadi_real p[], casadi_real x_out[], casadi_real u_out[]);
};

#endif // SOLVER_H
