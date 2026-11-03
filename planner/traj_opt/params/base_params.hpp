#pragma once

#include "param_validation.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace traj_opt
{

struct SolverConfig
{
    int mem_size = 16;        // 历史向量对数量
    int max_iterations = 200; // 每轮 L-BFGS 的迭代上限；重试时重新计数
    int past = 3;             // 代价收敛的回看步数
    int max_linesearch = 40;  // 单次线搜索求值上限
    double g_epsilon = 1e-5;  // 梯度收敛阈值
    double delta = 1e-2;      // 相对代价变化阈值
    double min_step = 1e-32;
    double max_step = 1e20;
    double f_dec_coeff = 1e-4; // 充分下降条件系数
    double s_curv_coeff = 0.9; // 曲率条件系数
    double xtol = 1e-16;
    int abs_curv_cond = 1;

};

class OptimizerParams
{
public:
    SolverConfig solver;
    int integration_intervals = 5;
    int max_attempts = 3;

    virtual ~OptimizerParams() = default;

    virtual void validate() const
    {
        param_validation::nonnegative({solver.g_epsilon, solver.delta});
        param_validation::positive({solver.min_step, solver.max_step, solver.f_dec_coeff, solver.s_curv_coeff, solver.xtol});
        if (solver.mem_size < 1 || solver.max_iterations < 1 || solver.past < 0 || solver.max_linesearch < 1 ||
            solver.max_step <= solver.min_step || solver.s_curv_coeff <= solver.f_dec_coeff ||
            solver.s_curv_coeff >= 1 || (solver.abs_curv_cond != 0 && solver.abs_curv_cond != 1))
        {
            throw std::invalid_argument("Invalid L-BFGS configuration");
        }
        if (integration_intervals < 1 || integration_intervals > 10000 || max_attempts < 1)
            throw std::invalid_argument("Invalid integration intervals or attempt count");
    }

    static nlohmann::json readJson(const std::string &path)
    {
        std::ifstream stream(path);
        if (!stream)
            throw std::invalid_argument("Cannot open config: " + path);
        nlohmann::json json;
        stream >> json;
        return json;
    }


protected:
    void readCommon(const nlohmann::json &json)
    {
        integration_intervals = json.value("integration_intervals", integration_intervals);
        max_attempts = json.value("max_attempts", max_attempts);
        if (json.contains("solver"))
        {
            const auto &v = json.at("solver");
            solver.mem_size = v.value("mem_size", solver.mem_size);
            solver.max_iterations = v.value("max_iterations", solver.max_iterations);
            solver.past = v.value("past", solver.past);
            solver.max_linesearch = v.value("max_linesearch", solver.max_linesearch);
            solver.g_epsilon = v.value("g_epsilon", solver.g_epsilon);
            solver.delta = v.value("delta", solver.delta);
            solver.min_step = v.value("min_step", solver.min_step);
            solver.max_step = v.value("max_step", solver.max_step);
            solver.f_dec_coeff = v.value("f_dec_coeff", solver.f_dec_coeff);
            solver.s_curv_coeff = v.value("s_curv_coeff", solver.s_curv_coeff);
            solver.xtol = v.value("xtol", solver.xtol);
            solver.abs_curv_cond = v.value("abs_curv_cond", solver.abs_curv_cond);
        }
    }


};
} // namespace traj_opt
