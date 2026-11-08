#include <diff/diff_optimizer.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <perception/esdf/esdf_map_2d.hpp>
#include <tools/log.hpp>
#include <utils/lbfgs.hpp>

namespace navapex::traj_opt
{
static constexpr char OPTIMIZER_TAG[] = "optimizer";

namespace
{

Eigen::Matrix<double, 6, 1> basis(double t, int d = 0)
{
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    for (int k = d; k < 6; ++k)
    {
        double f = 1;
        for (int j = 0; j < d; ++j)
            f *= k - j;
        b[k] = f * std::pow(t, k - d);
    }
    return b;
}

double realTime(double x)
{
    return x > 0 ? (0.5 * x + 1) * x + 1 : 1 / ((0.5 * x - 1) * x + 1);
}

double virtualTime(double t)
{
    return t > 1 ? std::sqrt(2 * t - 1) - 1 : 1 - std::sqrt(2 / t - 1);
}

double timeDerivative(double x)
{
    double d = (0.5 * x - 1) * x + 1;
    return x > 0 ? x + 1 : (1 - x) / (d * d);
}

void neighborState(const NeighborTrajectory &n, double t, Eigen::Vector3d &p, Eigen::Vector3d &v)
{
    double duration = n.trajectory.getTotalDuration();
    if (t < 0)
    {
        p = n.trajectory.getPos(0);
        v.setZero();
    }
    else if (t <= duration)
    {
        p = n.trajectory.getPos(t);
        v = n.trajectory.getVel(t);
    }
    else
    {
        //匀速外推
        v = n.trajectory.getVel(duration);
        p = n.trajectory.getPos(duration) + (t - duration) * v;
    }
}

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

} // namespace

Diff_Optimizer::Diff_Optimizer(DiffParams c,
                               std::shared_ptr<const OptimizerInput> input) 
                               : param_(std::move(c))
{
    state_.input = std::move(input);
    LOGD(OPTIMIZER_TAG) << "create diff_optimizer success";
}

const DiffParams &Diff_Optimizer::param() const
{
    return param_;
}

OptimizerResult Diff_Optimizer::optimize()
{
    auto start = std::chrono::steady_clock::now();
    OptimizerResult result;
    LOGD(OPTIMIZER_TAG) << "optimize start robot_id=" << state_.input->robot_id;
    if (initializePlanning() && runOptimization())
    {
        result.trajectory = minco_.getTraj();
        result.success = true;
        LOGI(OPTIMIZER_TAG) << "optimize success robot_id=" << state_.input->robot_id
                            << " cost=" << state_.evaluation_cost.total()
                            << " elapsed_ms=" << elapsedMs(start);
        LOG_TRAJECTORY(OPTIMIZER_TAG) << "trajectory robot_id=" << state_.input->robot_id
                                      << " start_time=" << state_.input->start_time << ' '
                                      << result.trajectory;
    }else{
        LOGW(OPTIMIZER_TAG) << "optimize failed robot_id=" << state_.input->robot_id
                            << " elapsed_ms=" << elapsedMs(start);
    }
    return result;
}

bool Diff_Optimizer::initializePlanning()
{
    const auto &input = *state_.input;
    state_.swarm_weight = param_.weight_swarm;
    state_.piece_count = static_cast<int>(input.durations.size());
    state_.min_swarm_dist_sq.resize(input.neighbors.size());
    state_.sample_points.resize(3, state_.piece_count * param_.integration_intervals + 1);

    // double clearance = 0;
    // // 检查起点终点是否位于障碍物内
    // Pose2D start{input.start.position.x(), input.start.position.y(), input.start.position.z()};
    // input.robot_->clearance(start, *input.esdf, clearance);
    // if (clearance < 0)
    // {
    //     LOGW(OPTIMIZER_TAG) << "start in collision or outside esdf";
    //     return false;
    // }
    // Pose2D goal{input.goal.position.x(), input.goal.position.y(), input.goal.position.z()};
    // input.robot_->clearance(goal, *input.esdf, clearance);
    // if (clearance < 0)
    // {
    //     LOGW(OPTIMIZER_TAG) << "goal in collision or outside esdf";
    //     return false;
    // }

    // 1.1 生成初始轨迹
    minco_.reset(input.start.matrix(), input.goal.matrix(), state_.piece_count);
    minco_.generate(input.inner_points, input.durations);
    if (!minco_.get_b().allFinite())
    {
        LOGE(OPTIMIZER_TAG) << "nonfinite initial coefficients";
        return false;
    }

    return true;
}

bool Diff_Optimizer::runOptimization()
{
    // 2.1 初始化变量
    Eigen::VectorXd variables = Eigen::VectorXd::Zero(4 * state_.piece_count - 3);
    for (int k = 0; k < state_.piece_count - 1; ++k)
        variables.segment<3>(3 * k) = state_.input->inner_points.col(k);
    for (int i = 0; i < state_.piece_count; ++i){
        variables.tail(state_.piece_count)[i] = virtualTime(state_.input->durations[i]);
    }

    for (int attempt = 0; attempt < param_.max_attempts; ++attempt)
    {
        // 2.2 迭代求解lbfgs
        if (!solveLbfgs(variables))
            return false;
        LOGD(OPTIMIZER_TAG) << "solver cost"
                            << " attempts=" << attempt + 1
                            << " total=" << state_.evaluation_cost.total()
                            << " smoothness=" << state_.evaluation_cost.smoothness
                            << " time=" << state_.evaluation_cost.time
                            << " obstacle=" << state_.evaluation_cost.obstacle
                            << " obstacle_soft=" << state_.evaluation_cost.obstacle_soft
                            << " velocity=" << state_.evaluation_cost.velocity
                            << " acceleration=" << state_.evaluation_cost.acceleration
                            << " angular_velocity=" << state_.evaluation_cost.angular_velocity
                            << " angular_acceleration=" << state_.evaluation_cost.angular_acceleration
                            << " kinematics=" << state_.evaluation_cost.kinematics
                            << " spacing=" << state_.evaluation_cost.spacing
                            << " swarm=" << state_.evaluation_cost.swarm;

        // 2.3 优化结束检查轨迹
        bool swarm_safe = true;
        for (std::size_t i = 0; i < state_.input->neighbors.size(); ++i)
        {
            const auto &neighbor = state_.input->neighbors[i];
            if (neighbor.robot_id < 0 || neighbor.robot_id == state_.input->robot_id)
                continue;
            double clearance = param_.swarm_validation_scale * 
                (state_.input->robot_->radius() + neighbor.clearance + param_.swarm_clearance);
            if (state_.min_swarm_dist_sq[i] < clearance * clearance)
            {
                LOGW(OPTIMIZER_TAG) << "swarm collision robot_id=" << neighbor.robot_id;
                swarm_safe = false;
                break;
            }
        }
        if (!swarm_safe)
            state_.swarm_weight *= 2;
        else if (CheckTrajectory())     // check dynamic feasible and static collision
        {
            return true;
        }

        // 2.4 检查失败尝试重启
        if (attempt + 1 < param_.max_attempts)
            LOGD(OPTIMIZER_TAG) << "solver retry attempt=" << attempt + 1
                                << " swarm_weight=" << state_.swarm_weight;
    }
    return false;
}

// 调用lbfgs求解
bool Diff_Optimizer::solveLbfgs(Eigen::VectorXd &variables)
{
    planner_lbfgs::lbfgs_parameter_t p;
    planner_lbfgs::lbfgs_load_default_parameters(&p);
    p.mem_size = param_.solver_mem_size;
    p.max_iterations = param_.solver_max_iterations;
    p.past = param_.solver_past;
    p.max_linesearch = param_.solver_max_linesearch;
    p.g_epsilon = param_.solver_g_epsilon;
    p.delta = param_.solver_delta;
    p.min_step = param_.solver_min_step;
    p.max_step = param_.solver_max_step;
    p.f_dec_coeff = param_.solver_f_dec_coeff;
    p.s_curv_coeff = param_.solver_s_curv_coeff;
    p.xtol = param_.solver_xtol;
    p.abs_curv_cond = param_.solver_abs_curv_cond;
    state_.stop = false;
    int code = planner_lbfgs::lbfgs_optimize(variables.size(), variables.data(), nullptr,
                                             costFunctionCallback, nullptr,
                                             progressCallback, this, &p);
    if (state_.stop)
    {
        LOGW(OPTIMIZER_TAG) << "solver error stop";
        return false;
    }
    bool accepted = code == planner_lbfgs::LBFGS_CONVERGENCE ||
                    code == planner_lbfgs::LBFGSERR_MAXIMUMITERATION ||
                    code == planner_lbfgs::LBFGS_ALREADY_MINIMIZED ||
                    code == planner_lbfgs::LBFGS_STOP;
    if (!accepted)
    {
        LOGW(OPTIMIZER_TAG) << "solver_failed code=" << code
                            << " reason=" << planner_lbfgs::lbfgs_strerror(code);
        return false;
    }
    LOGD(OPTIMIZER_TAG) << "solver stop code=" << code;
    return true;
}

double Diff_Optimizer::costFunctionCallback(void *instance, const double *raw, double *grad, int n)
{
    auto &planner = *static_cast<Diff_Optimizer *>(instance);
    Eigen::Map<Eigen::VectorXd> out(grad, n);
    if (!planner.evaluateCost(Eigen::Map<const Eigen::VectorXd>(raw, n), out))
    {
        planner.state_.stop = true;
        out.setZero();
        return 1e100;
    }
    return planner.state_.evaluation_cost.total();
}

int Diff_Optimizer::progressCallback(void *instance,const double *,const double *,double,double,double,double,int,int,int)
{
    auto &planner = *static_cast<Diff_Optimizer *>(instance);
    return planner.state_.stop ? 1 : 0;
}

bool Diff_Optimizer::evaluateCost(Eigen::Ref<const Eigen::VectorXd> variables,
                                  Eigen::Ref<Eigen::VectorXd> gradient)
{
    state_.evaluation_cost = {};
    std::fill(state_.min_swarm_dist_sq.begin(), state_.min_swarm_dist_sq.end(),
              std::numeric_limits<double>::infinity());

    // 2.2.1 新变量更新minco轨迹
    if (!decodeVariables(variables))
    {
        LOGE(OPTIMIZER_TAG) << "invalid decoded trajectory";
        return false;
    }
    Eigen::VectorXd time_gradient(state_.piece_count);

    // 2.2.2 更新代价和梯度 (ci梯度更新在minco_内)
    addJerkCost(time_gradient);                                                // jerk 代价函数
    addPenaltyCosts(time_gradient);                                            // 时间积分罚函数
    addSpacingCost(time_gradient);                                             // 间距惩罚函数
    state_.evaluation_cost.time = param_.weight_time * minco_.get_T1().sum();  // 时间项函数
    propagateGradient(variables, time_gradient, gradient);                     // 回传梯度

    if (!std::isfinite(state_.evaluation_cost.total()) || !gradient.allFinite())
    {
        LOGE(OPTIMIZER_TAG) << "nonfinite cost or gradient";
        return false;
    }
    return true;
}

bool Diff_Optimizer::decodeVariables(Eigen::Ref<const Eigen::VectorXd> variables)
{
    Eigen::VectorXd times(state_.piece_count);
    Eigen::Matrix3Xd inner_points(3, state_.piece_count - 1);
    for (int i = 0; i < state_.piece_count; ++i){
        times[i] = realTime(variables.tail(state_.piece_count)[i]);
    }
    for (int k = 0; k < state_.piece_count - 1; ++k){
        inner_points.col(k) = variables.segment<3>(3 * k);
    }
    if (!times.allFinite() || (times.array() <= 0).any())
        return false;
    minco_.generate(inner_points, times);
    return minco_.get_b().allFinite();
}

void Diff_Optimizer::addJerkCost(Eigen::VectorXd &time_gradient)
{
    minco_.initGradCost(time_gradient, state_.evaluation_cost.smoothness);
    state_.evaluation_cost.smoothness *= param_.weight_smoothness;
    minco_.get_gdC() *= param_.weight_smoothness;
    time_gradient *= param_.weight_smoothness;
}

void Diff_Optimizer::addPenaltyCosts(Eigen::VectorXd &time_gradient)
{
    const auto &coefficients = minco_.get_b();
    int K = param_.integration_intervals;
    int check_end = checkEnd();
    double elapsed = 0;

    for (int i = 0; i < state_.piece_count; ++i)
    {
        auto c = coefficients.block<6, 3>(6 * i, 0);
        double h = minco_.get_T1()[i] / K;
        for (int j = 0; j <= K; ++j)
        {
            double alpha = double(j) / K;
            double t = alpha * minco_.get_T1()[i];
            double w = (j == 0 || j == K) ? 0.5 : 1.0;

            auto b0 = basis(t);
            auto b1 = basis(t, 1);
            auto b2 = basis(t, 2);

            Eigen::Vector3d p = c.transpose() * b0;
            Eigen::Vector3d v = c.transpose() * b1;
            Eigen::Vector3d a = c.transpose() * b2;
            Eigen::Vector3d jer = c.transpose() * basis(t, 3);

            state_.sample_points.col(i * K + j) = p;
            Eigen::Vector3d gp = Eigen::Vector3d::Zero();
            Eigen::Vector3d gv = Eigen::Vector3d::Zero();
            Eigen::Vector3d ga = Eigen::Vector3d::Zero();
            double integrand = 0;
            double explicit_time = 0;

            //静态避障
            if (i * K + j > 0)
            {
                double clearance = 0;
                Eigen::Vector3d gradient;
                state_.input->robot_->clearanceSmooth(
                    {p.x(), p.y(), p.z()}, *state_.input->esdf, clearance, gradient);
                double hard = param_.collision_clearance - clearance;
                double soft = param_.collision_soft_clearance - clearance;
                if (hard > 0)
                {
                    double f = param_.weight_obstacle * hard * hard * hard;
                    state_.evaluation_cost.obstacle += w * h * f;
                    integrand += f;
                    gp -= 3 * param_.weight_obstacle * hard * hard * gradient;
                }
                if (soft > 0)
                {
                    double r = param_.collision_soft_scale;
                    double q = std::sqrt(1 + soft * soft / (r * r));
                    double f = param_.weight_obstacle_soft * r * r * (q - 1);
                    state_.evaluation_cost.obstacle_soft += w * h * f;
                    integrand += f;
                    gp -= param_.weight_obstacle_soft * soft / q * gradient;
                }
            }

            //集群避障
            if (i * K + j > 0)
            {
                for(std::size_t neighbor_index = 0; neighbor_index < state_.input->neighbors.size(); ++neighbor_index)
                {
                    const auto &n = state_.input->neighbors[neighbor_index];
                    if(n.robot_id < 0 || n.robot_id == state_.input->robot_id)
                        continue;

                    Eigen::Vector3d np;
                    Eigen::Vector3d nv;
                    neighborState(n, (state_.input->start_time - n.start_time) + elapsed + t, np, nv);
                    Eigen::Vector2d diff = (p - np).head<2>();
                    double safe = param_.swarm_penalty_scale * (state_.input->robot_->radius() + n.clearance + param_.swarm_clearance);
                    double distance_squared = diff.squaredNorm();
                    if (i * K + j <= check_end)
                        state_.min_swarm_dist_sq[neighbor_index] = std::min(state_.min_swarm_dist_sq[neighbor_index], distance_squared);
                    double violation = safe * safe - distance_squared;
                    if (violation > 0)
                    {
                        double f = state_.swarm_weight * violation * violation * violation;
                        Eigen::Vector2d g = -6 * state_.swarm_weight * violation * violation * diff;
                        state_.evaluation_cost.swarm += w * h * f;
                        integrand += f;
                        gp.head<2>() += g;
                        double previous = -g.dot(nv.head<2>());
                        explicit_time += previous;
                        if (i > 0)
                            time_gradient.head(i).array() += w * h * previous;
                    }
                }
            }

            // 运动学一致性
            {
                Eigen::Vector2d heading(std::cos(p.z()), std::sin(p.z()));
                Eigen::Vector2d normal(-heading.y(), heading.x());
                double alignment = normal.dot(v.head<2>());           //横向速度
                double forward = heading.dot(v.head<2>());            //前进速度
                double backward = std::min(forward, 0.0);             //不允许倒车
                double kinematic_cost = param_.weight_kinematics * (alignment * alignment + backward * backward);
                state_.evaluation_cost.kinematics += w * h * kinematic_cost;
                integrand += kinematic_cost;
                gp.z() += 2 * param_.weight_kinematics * alignment * (backward - forward);
                gv.head<2>() += 2 * param_.weight_kinematics * (alignment * normal + backward * heading);
            }
            
            // 运动可行性
            auto feasibility = [&](double squared_value, double limit, double &part)
            {
                double violation = squared_value - limit * limit;
                if (violation <= 0) return 0.0;
                double f = param_.weight_feasibility * violation * violation * violation;
                part += w * h * f;
                integrand += f;
                return 6 * param_.weight_feasibility * violation * violation;
            };
            gv.head<2>() += feasibility(v.head<2>().squaredNorm(), param_.max_velocity,
                                        state_.evaluation_cost.velocity) * v.head<2>();
            ga.head<2>() += feasibility(a.head<2>().squaredNorm(), param_.max_acceleration,
                                        state_.evaluation_cost.acceleration) * a.head<2>();
            gv.z() += feasibility(v.z() * v.z(), param_.max_angular_velocity,
                                  state_.evaluation_cost.angular_velocity) * v.z();
            ga.z() += feasibility(a.z() * a.z(), param_.max_angular_acceleration,
                                  state_.evaluation_cost.angular_acceleration) * a.z();

            minco_.get_gdC().block<6, 3>(6 * i, 0) += w * h *
                (b0 * gp.transpose() + b1 * gv.transpose() + b2 * ga.transpose());
            time_gradient[i] += w * (integrand / K +
                    h * alpha * (gp.dot(v) + gv.dot(a) + ga.dot(jer) + explicit_time));
        }
        elapsed += minco_.get_T1()[i];
    }
}

void Diff_Optimizer::addSpacingCost(Eigen::VectorXd &time_gradient)
{
    const auto &coefficients = minco_.get_b();
    int K = param_.integration_intervals;
    Eigen::Matrix3Xd sample_gradient = Eigen::Matrix3Xd::Zero(3, state_.sample_points.cols());
    int edges = state_.sample_points.cols() - 1;
    for (int k = 0; k < edges; ++k)
    {
        Eigen::Vector2d d = (state_.sample_points.col(k + 1) - state_.sample_points.col(k)).head<2>();
        double d2 = d.squaredNorm();
        state_.evaluation_cost.spacing += param_.weight_sqrvariance * d2 * d2 / edges;
        Eigen::Vector2d g = 4 * param_.weight_sqrvariance * d2 / edges * d;
        sample_gradient.col(k).head<2>() -= g;
        sample_gradient.col(k + 1).head<2>() += g;
    }
    for (int i = 0; i < state_.piece_count; ++i)
    {
        for (int j = 0; j <= K; ++j)
        {
            double alpha = double(j) / K;
            double t = alpha * minco_.get_T1()[i];
            double w = (j == 0 || j == K) ? 0.5 : 1;
            Eigen::Vector3d g = sample_gradient.col(i * K + j);
            Eigen::Vector3d v = coefficients.block<6, 3>(6 * i, 0).transpose() * basis(t, 1);
            minco_.get_gdC().block<6, 3>(6 * i, 0) += w * basis(t) * g.transpose();
            time_gradient[i] += w * alpha * g.dot(v);
        }
    }
}

void Diff_Optimizer::propagateGradient(Eigen::Ref<const Eigen::VectorXd> variables,
                                       Eigen::VectorXd &time_gradient,
                                       Eigen::Ref<Eigen::VectorXd> gradient)
{
    Eigen::Matrix3Xd point_gradient(3, state_.piece_count - 1);
    minco_.getGrad2TP(time_gradient, point_gradient);
    for (int k = 0; k < state_.piece_count - 1; ++k){
        gradient.segment<3>(3 * k) = point_gradient.col(k);
    }
    for (int i = 0; i < state_.piece_count; ++i){
        gradient.tail(state_.piece_count)[i] = (time_gradient[i] + param_.weight_time) *
                                                timeDerivative(variables.tail(state_.piece_count)[i]);
    }
}

bool Diff_Optimizer::CheckTrajectory()
{
    auto tr = minco_.getTraj();
    auto durations = tr.getDurations();
    double resolution = state_.input->esdf->resolution();
    double duration = tr.getTotalDuration();
    const auto &robot = state_.input->robot_;
    int end = checkEnd();
    int end_piece = std::min(end / param_.integration_intervals, state_.piece_count - 1);
    double check_duration = durations.head(end_piece).sum() + durations[end_piece] *
        (end - end_piece * param_.integration_intervals) / param_.integration_intervals;

    double step = std::min(resolution / (2 * (param_.max_velocity + param_.velocity_tolerance)),
                           durations.minCoeff() / param_.integration_intervals / 1.5);
    if (!std::isfinite(step) || step <= 0)
    {
        LOGE(OPTIMIZER_TAG) << "invalid validation step";
        return false;
    }

    for (double time = 0;;)
    {
        double clearance = 0;
        Eigen::Vector3d position = tr.getPos(time);
        robot->clearance({position.x(), position.y(), position.z()}, *state_.input->esdf, clearance);
        if (clearance < 0)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory collision"
                                << " time=" << time << " duration=" << duration
                                << " check_duration=" << check_duration
                                << " position=" << position.transpose()
                                << " clearance=" << clearance;
            return false;
        }
        Eigen::Vector3d v = tr.getVel(time);
        Eigen::Vector3d a = tr.getAcc(time);
        double velocity = v.head<2>().norm();
        if (!std::isfinite(velocity) || velocity > param_.max_velocity + param_.velocity_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory velocity violation time=" << time << " velocity=" << velocity;
            return false;
        }
        double acceleration = a.head<2>().norm();
        if (!std::isfinite(acceleration) || acceleration > param_.max_acceleration + param_.acceleration_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory acceleration violation time=" << time << " acceleration=" << acceleration;
            return false;
        }
        double angular_velocity = std::abs(v.z());
        if (!std::isfinite(angular_velocity) || angular_velocity > param_.max_angular_velocity + param_.angular_velocity_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory angular velocity violation time=" << time << " angular_velocity=" << angular_velocity;
            return false;
        }
        double angular_acceleration = std::abs(a.z());
        if (!std::isfinite(angular_acceleration) || angular_acceleration > param_.max_angular_acceleration + param_.angular_acceleration_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory angular acceleration violation time=" << time << " angular_acceleration=" << angular_acceleration;
            return false;
        }
        if (time == check_duration)
            break;
        if (time + step <= time)
        {
            LOGE(OPTIMIZER_TAG) << "validation time stalled time=" << time;
            return false;
        }
        time = std::min(time + step, check_duration);
    }
    return true;
}

int Diff_Optimizer::checkEnd() const
{
    int last = state_.piece_count * param_.integration_intervals;
    return state_.input->touches_goal ? last : last - (last - 1) / 2;
}

} // namespace navapex::traj_opt
