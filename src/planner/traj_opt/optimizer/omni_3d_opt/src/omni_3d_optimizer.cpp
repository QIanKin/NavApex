#include <omni3d/omni_3d_optimizer.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

bool intersectGuide(const std::vector<Eigen::Vector3d> &path, 
                    const Eigen::Vector3d &point,
                    const Eigen::Vector3d &tangent, 
                    Eigen::Vector3d &intersection)
{
    if (path.size() < 2 || tangent.squaredNorm() < 1e-16) 
        return false;
    
    int index = static_cast<int>(path.size() / 2);
    double value = (path[index] - point).dot(tangent);
    double initial = value;
    for (std::size_t count = 0; count < path.size(); ++count)
    {
        int previous = index;
        index += value >= 0 ? 1 : -1;
        if (index < 0 || index >= static_cast<int>(path.size())) break;
        value = (path[index] - point).dot(tangent);
        if (value * initial <= 0 && (value != 0 || initial != 0))
        {
            Eigen::Vector3d delta = path[index] - path[previous];
            double denominator = tangent.dot(delta);
            if (std::abs(denominator) < 1e-12) return false;
            intersection = path[index] + delta * tangent.dot(point - path[index]) / denominator;
            return intersection.allFinite();
        }
    }
    return false;
}

void removeSegmentOverlap(std::vector<std::pair<int, int>> &segments)
{
    for (std::size_t i = 1; i < segments.size(); ++i)
    {
        if (segments[i - 1].second >= segments[i].first)
        {
            double middle = (segments[i - 1].second + segments[i].first) / 2.0;
            segments[i - 1].second = static_cast<int>(middle - 0.1);
            segments[i].first = static_cast<int>(middle + 1.1);
        }
    }
}

} // namespace


Omni_3d_Optimizer::Omni_3d_Optimizer(Omni3DParams c,
                                     std::shared_ptr<const OptimizerInput> input) 
                                     : param_(std::move(c))
{
    state_.input = std::move(input);
    if (state_.input->robot_)
    {
        const auto &limits = state_.input->robot_->limits();
        param_.max_velocity = std::min(param_.max_velocity, limits.max_linear_velocity);
        param_.max_acceleration = std::min(param_.max_acceleration, limits.max_linear_acceleration);
        param_.max_jerk = std::min(param_.max_jerk, limits.max_jerk);
    }
    LOGD(OPTIMIZER_TAG) << "create omni_3d_optimizer success";
}

const Omni3DParams &Omni_3d_Optimizer::param() const
{
    return param_;
}

OptimizerResult Omni_3d_Optimizer::optimize()
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
                                      << " start_time=" << state_.input->start_time << " trajectory_axes=x,y,z "
                                      << result.trajectory;
    }
    else
        LOGW(OPTIMIZER_TAG) << "optimize failed robot_id=" << state_.input->robot_id
                            << " elapsed_ms=" << elapsedMs(start);
    return result;
}

bool Omni_3d_Optimizer::initializePlanning()
{   
    state_.swarm_weight = param_.weight_swarm;
    state_.rebounds = 0;
    state_.piece_count = static_cast<int>(state_.input->durations.size());
    state_.min_swarm_dist_sq.resize(state_.input->neighbors.size());
    state_.sample_points.resize(3, state_.piece_count * param_.integration_intervals + 1);
    state_.normal_planes.clear();

    // 1.1 q T生成初始轨迹
    minco_.reset(state_.input->start.matrix(), state_.input->goal.matrix(), state_.piece_count);
    minco_.generate(state_.input->inner_points, state_.input->durations);
    if (!minco_.get_b().allFinite())
    {
        LOGE(OPTIMIZER_TAG) << "nonfinite initial coefficients";
        return false;
    }

    // 1.2 初始化法平面
    auto initial = minco_.getTraj();
    state_.normal_planes.reserve(state_.input->inner_points.cols());
    Eigen::Matrix3Xd guide(3, state_.input->inner_points.cols() + 2);
    guide.col(0) = state_.input->start.q;
    guide.middleCols(1, state_.input->inner_points.cols()) = state_.input->inner_points;
    guide.col(guide.cols() - 1) = state_.input->goal.q;
    for (int k = 0; k < state_.input->inner_points.cols(); ++k)
    {
        Eigen::Vector3d t = initial[k].getVel(initial[k].getDuration());
        if (t.squaredNorm() < 1e-12)
        {
            t = guide.col(k + 2) - guide.col(k);  // p-1  p+1
            if (t.squaredNorm() < 1e-12)
                t = guide.col(k + 2) - guide.col(k + 1); // p p+1
            if (t.squaredNorm() < 1e-12)
                t = guide.col(k + 1) - guide.col(k); // p-1 p
            if (t.squaredNorm() < 1e-12)
            {
                LOGW(OPTIMIZER_TAG) << "normal plane fixed axis" << " index=" << k;
                t = Eigen::Vector3d::UnitX();
            }
            else
            {
                LOGW(OPTIMIZER_TAG) << "normal plane fallback" << " index=" << k;
            }
        }
        t.normalize();
        Eigen::Index axis;
        t.cwiseAbs().minCoeff(&axis);
        NormalPlane plane;
        plane.anchor = state_.input->inner_points.col(k);
        plane.basis.col(0) = t.cross(Eigen::Vector3d::Unit(axis)).normalized();
        plane.basis.col(1) = t.cross(plane.basis.col(0)).normalized();
        state_.normal_planes.push_back(plane);
    }

    // 1.3 构建采样点避障约束
    for (auto &constraints : state_.obstacle_constraints){
        constraints.clear();
    }
    state_.obstacle_constraints.resize(state_.sample_points.cols());
    auto points = minco_.getInitConstraintPoints(param_.integration_intervals);
    std::vector<std::pair<int, int>> initial_segments;
    if (!collectCollisionSegments(initial, points, initial_segments, checkEnd()))
        return false;
    if (!finelyBuildObstacleConstraints(points, std::move(initial_segments))) //A*构造障碍物边界约束
        return false;

    return true;
}

bool Omni_3d_Optimizer::runOptimization()
{
    // 2.1 初始化变量 (法平面偏移量初始化为0)
    Eigen::VectorXd variables = Eigen::VectorXd::Zero(3 * state_.piece_count - 2);
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
                            << " jerk=" << state_.evaluation_cost.jerk
                            << " spacing=" << state_.evaluation_cost.spacing
                            << " swarm=" << state_.evaluation_cost.swarm;

        // 2.3 优化结束检查轨迹
        bool swarm_safe = true;
        for (std::size_t i = 0; i < state_.input->neighbors.size(); ++i)
        {
            const auto &neighbor = state_.input->neighbors[i];
            double clearance = param_.swarm_validation_scale * (param_.swarm_clearance + neighbor.clearance);
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
bool Omni_3d_Optimizer::solveLbfgs(Eigen::VectorXd &variables)
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
    for (;;)
    {
        state_.stop_reason = StopReason::None;
        state_.evaluations = 0;
        int code = planner_lbfgs::lbfgs_optimize(variables.size(), variables.data(), nullptr,
                                                 costFunctionCallback, nullptr,
                                                 progressCallback, this, &p);
        if (state_.stop_reason == StopReason::Error)
        {
            LOGW(OPTIMIZER_TAG) << "solver error stop";
            return false;
        }
        // 2.2.4 更新约束后重启优化
        if (state_.stop_reason == StopReason::Rebound)
        {
            if (state_.rebounds >= param_.max_rebounds)
            {
                LOGW(OPTIMIZER_TAG) << "rebound limit count " << state_.rebounds;
                return false;
            }
            ++state_.rebounds;
            LOGD(OPTIMIZER_TAG) << "solver rebound count " << state_.rebounds;
            continue;
        }
        bool accepted = code == planner_lbfgs::LBFGS_CONVERGENCE || 
                        code == planner_lbfgs::LBFGSERR_MAXIMUMITERATION ||
                        code == planner_lbfgs::LBFGS_ALREADY_MINIMIZED || 
                        code == planner_lbfgs::LBFGS_STOP;
        if (!accepted){
            LOGW(OPTIMIZER_TAG) << "solver_failed code=" << code
                                << " reason=" << planner_lbfgs::lbfgs_strerror(code);
        }else{
            LOGD(OPTIMIZER_TAG) << "solver stop code=" << code;
        }
        return accepted;
    }
}

double Omni_3d_Optimizer::costFunctionCallback(void *instance, const double *raw, double *grad, int n)
{
    auto &planner = *static_cast<Omni_3d_Optimizer *>(instance);
    Eigen::Map<Eigen::VectorXd> out(grad, n);
    ++planner.state_.evaluations;
    if (!planner.evaluateCost(Eigen::Map<const Eigen::VectorXd>(raw, n), out))
    {
        planner.state_.stop_reason = StopReason::Error;
        out.setZero();
        return 1e100;
    }
    // 2.2.3 碰撞检查补充约束
    if (planner.state_.stop_reason == StopReason::None && planner.allowRebound())
        planner.roughlyCheckConstraintPoints();

    return planner.state_.evaluation_cost.total();
}

int Omni_3d_Optimizer::progressCallback(void *instance,const double *,const double *,double,double,double,double,int,int,int)
{
    auto &planner = *static_cast<Omni_3d_Optimizer *>(instance);
    return planner.state_.stop_reason != StopReason::None ? 1 : 0;
}

bool Omni_3d_Optimizer::evaluateCost(Eigen::Ref<const Eigen::VectorXd> variables, 
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

bool Omni_3d_Optimizer::decodeVariables(Eigen::Ref<const Eigen::VectorXd> variables)
{
    Eigen::VectorXd times(state_.piece_count);
    Eigen::Matrix3Xd inner_points(3, state_.piece_count - 1);
    for (int i = 0; i < state_.piece_count; ++i){
        times[i] = realTime(variables.tail(state_.piece_count)[i]);
    }
    for (int k = 0; k < state_.piece_count - 1; ++k){
        inner_points.col(k) = state_.normal_planes[k].anchor + 
                              state_.normal_planes[k].basis * variables.segment<2>(2 * k);
    }
    if (!times.allFinite() || (times.array() <= 0).any())
        return false;
    minco_.generate(inner_points, times);
    return minco_.get_b().allFinite();
}

void Omni_3d_Optimizer::addJerkCost(Eigen::VectorXd &time_gradient)
{
    minco_.initGradCost(time_gradient, state_.evaluation_cost.smoothness);
    state_.evaluation_cost.smoothness *= param_.weight_smoothness;
    minco_.get_gdC() *= param_.weight_smoothness;
    time_gradient *= param_.weight_smoothness;
}

void Omni_3d_Optimizer::addPenaltyCosts(Eigen::VectorXd &time_gradient)
{
    const auto &coefficients = minco_.get_b();
    int K = param_.integration_intervals;
    int check_end = checkEnd();
    Eigen::Vector3d metric(1 / std::pow(param_.swarm_horizontal_scale, 2),
                           1 / std::pow(param_.swarm_horizontal_scale, 2),
                           1 / std::pow(param_.swarm_vertical_scale,   2));
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
            auto b3 = basis(t, 3);
            
            Eigen::Vector3d p = c.transpose() * b0;
            Eigen::Vector3d v = c.transpose() * b1;
            Eigen::Vector3d a = c.transpose() * b2;
            Eigen::Vector3d jer = c.transpose() * b3;
            Eigen::Vector3d snap = c.transpose() * basis(t, 4);
            
            state_.sample_points.col(i * K + j) = p;
            Eigen::Vector3d gp = Eigen::Vector3d::Zero();
            Eigen::Vector3d gv = Eigen::Vector3d::Zero();
            Eigen::Vector3d ga = Eigen::Vector3d::Zero();
            Eigen::Vector3d gj = Eigen::Vector3d::Zero();
            double integrand = 0;
            double explicit_time = 0;

            //静态避障
            if (i * K + j > 0 && i * K + j <= check_end)
            {
                for (const auto &constraint : state_.obstacle_constraints[i * K + j])
                {
                    double projection = (p - constraint.base).dot(constraint.direction);
                    double hard = param_.collision_clearance - projection;
                    double soft = param_.collision_soft_clearance - projection;
                    if (hard > 0)
                    {
                        double f = param_.weight_obstacle * hard * hard * hard;
                        state_.evaluation_cost.obstacle += w * h * f;
                        integrand += f;
                        gp -= 3 * param_.weight_obstacle * hard * hard * constraint.direction;
                    }
                    if (soft > 0)
                    {
                        double r = param_.collision_soft_scale;
                        double q = std::sqrt(1 + soft * soft / (r * r));
                        double f = param_.weight_obstacle_soft * r * r * (q - 1);
                        state_.evaluation_cost.obstacle_soft += w * h * f;
                        integrand += f;
                        gp -= param_.weight_obstacle_soft * soft / q * constraint.direction;
                    }
                }
            }

            //集群避障
            if (i * K + j > 0 && i * K + j <= check_end)
            {
                for(std::size_t neighbor_index = 0; neighbor_index < state_.input->neighbors.size(); ++neighbor_index)
                {
                    const auto &n = state_.input->neighbors[neighbor_index];
                    if(n.robot_id < 0 || n.robot_id == state_.input->robot_id)
                        continue;
                    
                    Eigen::Vector3d np;
                    Eigen::Vector3d nv;
                    neighborState(n, (state_.input->start_time - n.start_time) + elapsed + t, np, nv);
                    Eigen::Vector3d diff = p - np;
                    double safe = param_.swarm_penalty_scale * (param_.swarm_clearance + n.clearance);
                    double distance_squared = diff.dot(metric.cwiseProduct(diff));
                    state_.min_swarm_dist_sq[neighbor_index] = std::min(state_.min_swarm_dist_sq[neighbor_index], distance_squared);
                    double violation = safe * safe - distance_squared;
                    if (violation > 0)
                    {
                        double f = state_.swarm_weight * violation * violation * violation;
                        Eigen::Vector3d g = -6 * state_.swarm_weight * violation * violation *
                                            metric.cwiseProduct(diff);
                        state_.evaluation_cost.swarm += w * h * f;
                        integrand += f;
                        gp += g;
                        double previous = -g.dot(nv);
                        explicit_time += previous;
                        if (i > 0)
                            time_gradient.head(i).array() += w * h * previous;
                    }
                }
            }

            //运动可行性
            auto feasibility = [&](const Eigen::Vector3d &value, double limit, Eigen::Vector3d &g, double &part)
            {
                double violation = value.squaredNorm() - limit * limit;
                if (violation > 0)
                {
                    double f = param_.weight_feasibility * violation * violation * violation;
                    g = 6 * param_.weight_feasibility * violation * violation * value;
                    part += w * h * f;
                    integrand += f;
                }
            };
            feasibility(v, param_.max_velocity, gv, state_.evaluation_cost.velocity);
            feasibility(a, param_.max_acceleration, ga, state_.evaluation_cost.acceleration);
            feasibility(jer, param_.max_jerk, gj, state_.evaluation_cost.jerk);

            minco_.get_gdC().block<6, 3>(6 * i, 0) += w * h *
                (b0 * gp.transpose() + b1 * gv.transpose() + b2 * ga.transpose() + b3 * gj.transpose());
            time_gradient[i] += w * (integrand / K +
                    h * alpha * (gp.dot(v) + gv.dot(a) + ga.dot(jer) + gj.dot(snap) + explicit_time));
        }
        elapsed += minco_.get_T1()[i];
    }
}

void Omni_3d_Optimizer::addSpacingCost(Eigen::VectorXd &time_gradient)
{
    const auto &coefficients = minco_.get_b();
    int K = param_.integration_intervals;
    Eigen::Matrix3Xd sample_gradient = Eigen::Matrix3Xd::Zero(3, state_.sample_points.cols());
    int edges = state_.sample_points.cols() - 1;
    for (int k = 0; k < edges; ++k)
    {
        Eigen::Vector3d d = state_.sample_points.col(k + 1) - state_.sample_points.col(k);
        double d2 = d.squaredNorm();
        state_.evaluation_cost.spacing += param_.weight_sqrvariance * d2 * d2 / edges;
        Eigen::Vector3d g = 4 * param_.weight_sqrvariance * d2 / edges * d;
        sample_gradient.col(k) -= g;
        sample_gradient.col(k + 1) += g;
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

void Omni_3d_Optimizer::propagateGradient(Eigen::Ref<const Eigen::VectorXd> variables,
                                          Eigen::VectorXd &time_gradient, 
                                          Eigen::Ref<Eigen::VectorXd> gradient)
{
    Eigen::Matrix3Xd point_gradient(3, state_.piece_count - 1);
    minco_.getGrad2TP(time_gradient, point_gradient);
    for (int k = 0; k < state_.piece_count - 1; ++k){
        gradient.segment<2>(2 * k) = state_.normal_planes[k].basis.transpose() * point_gradient.col(k);
    }
    for (int i = 0; i < state_.piece_count; ++i){
        gradient.tail(state_.piece_count)[i] = (time_gradient[i] + param_.weight_time) * 
                                                timeDerivative(variables.tail(state_.piece_count)[i]);
    } 
}

bool Omni_3d_Optimizer::CheckTrajectory()
{
    auto tr = minco_.getTraj();
    auto durations = tr.getDurations();
    double resolution = state_.input->map->resolution();
    int end = checkEnd();
    int end_piece = std::min(end / param_.integration_intervals, state_.piece_count - 1);
    double duration = durations.head(end_piece).sum() + durations[end_piece] *
        (end - end_piece * param_.integration_intervals) / param_.integration_intervals;

    double step = std::min(resolution / param_.max_velocity, 
                           durations.minCoeff() / param_.integration_intervals / 1.5);
    if (!std::isfinite(step) || step <= 0)
    {
        LOGE(OPTIMIZER_TAG) << "invalid validation step";
        return false;
    }

    for (double time = 0;;)
    {
        double velocity = tr.getVel(time).norm();
        if (!std::isfinite(velocity) || velocity > param_.max_velocity + param_.velocity_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory velocity violation time=" << time << " velocity=" << velocity;
            return false;
        }
        double acceleration = tr.getAcc(time).norm();
        if (!std::isfinite(acceleration) || acceleration > param_.max_acceleration + param_.acceleration_tolerance)
        {
            LOGW(OPTIMIZER_TAG) << "trajectory acceleration violation time=" << time << " acceleration=" << acceleration;
            return false;
        }
        if (time == duration)
            break;
        if (time + step <= time)
        {
            LOGE(OPTIMIZER_TAG) << "validation time stalled time=" << time;
            return false;
        }
        time = std::min(time + step, duration);
    }

    // 轨迹精查发现碰撞后，新增方向约束再重试
    std::vector<std::pair<int, int>> segments;
    auto points = minco_.getInitConstraintPoints(param_.integration_intervals);
    if (!collectCollisionSegments(tr, points, segments, end))
        return false;
    if (segments.empty()) return true;
    LOGW(OPTIMIZER_TAG) << "trajectory collision segments=" << segments.size();
    finelyBuildObstacleConstraints(points, std::move(segments));
    return false;
}

int Omni_3d_Optimizer::checkEnd() const
{
    int last = state_.piece_count * param_.integration_intervals;
    return state_.input->touches_goal ? last : last - (last - 1) / 3;  // 非终点取前约 2/3 
}

bool Omni_3d_Optimizer::collectCollisionSegments(const poly_traj::Trajectory<3> &trajectory,
                                                 const Eigen::Matrix3Xd &points, 
                                                 std::vector<std::pair<int, int>> &segments, int end)
{
    segments.clear();
    auto durations = trajectory.getDurations();
    int K = param_.integration_intervals;
    int end_piece = std::min(end / K, state_.piece_count - 1);
    double end_time = durations.head(end_piece).sum() + durations[end_piece] * (end - end_piece * K) / K;
    double resolution = state_.input->map->resolution();
    double step = std::min(resolution / param_.max_velocity, durations.minCoeff() / K / 1.5);
    if (!std::isfinite(step) || step <= 0)
    {
        LOGE(OPTIMIZER_TAG) << "invalid collision sampling step";
        return false;
    }
    Eigen::Vector3d start_position = trajectory.getPos(0);
    if (state_.input->map->occupied({start_position.x(), start_position.y(), start_position.z()}))
    {
        LOGW(OPTIMIZER_TAG) << "start in collision position=" << start_position.transpose();
        return false;
    }
    if (state_.input->touches_goal)
    {
        Eigen::Vector3d goal_position = trajectory.getPos(durations.sum());
        if (state_.input->map->occupied({goal_position.x(), goal_position.y(), goal_position.z()}))
        {
            LOGW(OPTIMIZER_TAG) << "goal in collision position=" << goal_position.transpose();
            return false;
        }
    }

    int interval = 0, piece = 0, last_interval = -1;
    double piece_start = 0;
    Eigen::Vector3d last_position = start_position;
    constexpr int enough = 2;
    int same = enough + 1, in = -1, out = -1;
    bool previous = false, have_start = false, maybe_end = false;
    for (double time = 0;;)
    {
        //find time in which index
        while (interval < end - 1 && time >= piece_start + durations[piece] / K *
                (interval + 1 - K * piece))
        {
            if (interval + 1 >= K * (piece + 1))
                piece_start += durations[piece++];
            ++interval;
        }
        Eigen::Vector3d position = trajectory.getPos(time);
        if (time < 1e-5 || time == end_time || interval != last_interval ||
            (position - last_position).cwiseAbs().maxCoeff() > resolution / 2)
        {
            bool occupied = state_.input->map->occupied({position.x(), position.y(), position.z()});
            if (occupied && !previous)
            {
                if (interval == 0 || !have_start)
                {
                    in = interval;
                    have_start = true;
                }
                same = 0;
                maybe_end = false;
            }
            else if (!occupied && previous)
            {
                out = interval + 1;
                maybe_end = true;
                same = 0;
            }
            else ++same;
            if (maybe_end && (same > enough || interval == end - 1))
            {
                if (have_start) segments.emplace_back(in, out);
                have_start = false;
                maybe_end = false;
            }
            previous = occupied;
            last_position = position;
            last_interval = interval;
        }
        if (time == end_time) break;
        if (time + step <= time)
        {
            LOGE(OPTIMIZER_TAG) << "collision sampling stalled";
            return false;
        }
        time = std::min(time + step, end_time);
    }

    if (have_start)
    {
        out = maybe_end ? out : end;
        while (out < points.cols() &&
            state_.input->map->occupied({points(0, out), points(1, out), points(2, out)}))
            ++out;
        if (out >= points.cols())
        {
            LOGW(OPTIMIZER_TAG) << "collision segment no exit begin" << in;
            return false;
        }
        segments.emplace_back(in, out);
    }
    for (const auto &segment : segments)
        LOGD(OPTIMIZER_TAG) << "collision segment begin=" << segment.first << " end=" << segment.second;
    return true;
}

bool Omni_3d_Optimizer::finelyBuildObstacleConstraints(const Eigen::Matrix3Xd &points,
                                                       std::vector<std::pair<int, int>> segments)
{
    if (segments.empty()) return true;

    std::vector<std::vector<Eigen::Vector3d>> paths;
    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        std::vector<Eigen::Vector3d> path;
        while (true)
        {
            LocalSearchResult result = local_search_.search(*state_.input->map,
                points.col(segments[i].second), points.col(segments[i].first),
                param_.search_cells, param_.search_timeout_ms, path);
            if (result == LocalSearchResult::Success) break;
            if (result != LocalSearchResult::SearchError || i + 1 >= segments.size())
            {
                LOGW(OPTIMIZER_TAG) << "obstacle guide search failed begin="
                                    << segments[i].first << " end=" << segments[i].second;
                return false;
            }
            segments[i].second = segments[i + 1].second;
            segments.erase(segments.begin() + i + 1);
        }
        paths.push_back(std::move(path));
    }
    auto adjusted = segments;
    removeSegmentOverlap(adjusted);
    for (std::size_t i = 0; i < segments.size(); ++i)
        appendObstacleDirections(points, paths[i], segments[i], adjusted[i], true);
    LOGD(OPTIMIZER_TAG) << "obstacle constraints updated segments=" << segments.size();
    return true;
}

void Omni_3d_Optimizer::appendObstacleDirections(const Eigen::Matrix3Xd &points,
                                                 const std::vector<Eigen::Vector3d> &path, 
                                                 const std::pair<int, int> &segment,
                                                 const std::pair<int, int> &propagation_segment, 
                                                 bool handle_short_segment)
{
    double resolution = state_.input->map->resolution();
    std::vector<bool> generated(points.cols(), false);
    int last = -1;
    for (int j = segment.first + 1; j < segment.second; ++j)
    {
        Eigen::Vector3d intersection;
        Eigen::Vector3d p = points.col(j);
        if (!intersectGuide(path, p, points.col(j + 1) - points.col(j - 1), intersection))
            continue;
        double length = (intersection - p).norm();
        if (length <= 1e-5) continue;
        Eigen::Vector3d direction = (intersection - p) / length;
        for (double distance = length; distance >= 0; distance -= resolution)
        {
            Eigen::Vector3d position = p + distance * direction;
            bool occupied = state_.input->map->occupied({position.x(), position.y(), position.z()});
            if (occupied || distance < resolution)
            {
                if (occupied) distance += resolution;
                state_.obstacle_constraints[j].push_back({p + distance * direction, direction});
                generated[j] = true;
                last = j;
                break;
            }
        }
    }

    if (handle_short_segment && segment.second - segment.first == 1)
    {
        int first = segment.first;
        int j = first == 0 ? 1 : first;
        Eigen::Vector3d middle = (points.col(first) + points.col(first + 1)) * 0.5;
        Eigen::Vector3d intersection;
        if (intersectGuide(path, middle, points.col(first + 1) - points.col(first), intersection) &&
            (intersection - middle).norm() > 0.01)
        {
            state_.obstacle_constraints[j].push_back({points.col(j), (intersection - middle).normalized()});
            generated[j] = true;
            last = j;
        }
    }
    if (last < 0)
    {
        LOGW(OPTIMIZER_TAG) << "obstacle direction unavailable begin "
                            << segment.first << " to" << segment.second;
        return;
    }
    for (int j = last + 1; j <= propagation_segment.second; ++j)
        if (!generated[j]) state_.obstacle_constraints[j].push_back(state_.obstacle_constraints[j - 1].back());
    for (int j = last - 1; j >= propagation_segment.first; --j)
        if (!generated[j]) state_.obstacle_constraints[j].push_back(state_.obstacle_constraints[j + 1].back());
}

bool Omni_3d_Optimizer::allowRebound() const
{
    if (state_.evaluations <= 3) return false;
    for (int i = 3; i <= state_.sample_points.cols() - 4; ++i)
    {
        Eigen::Vector3d a = state_.sample_points.col(i) - state_.sample_points.col(i - 1);
        Eigen::Vector3d b = state_.sample_points.col(i + 1) - state_.sample_points.col(i);
        if (a.norm() < 1e-12 || b.norm() < 1e-12 || a.normalized().dot(b.normalized()) < 0.87)   //大尖角构造的法平面不适合rebound
            return false;
    }
    return true;
}

void Omni_3d_Optimizer::roughlyCheckConstraintPoints()
{
    std::vector<std::pair<int, int>> segments;
    auto occupied = [&](int index)
    {
        return state_.input->map->occupied({state_.sample_points(0, index),
            state_.sample_points(1, index), state_.sample_points(2, index)});
    };

    //寻找需要新加约束的碰撞区间
    int end = checkEnd();
    for (int i = 1; i <= end; ++i)
    {
        if (!occupied(i)) continue;
        bool covered = false;
        for (const auto &constraint : state_.obstacle_constraints[i])
        {
            if ((state_.sample_points.col(i) - constraint.base).dot(constraint.direction) <
                state_.input->map->resolution())
            {
                covered = true;
                break;
            }
        }
        if (covered) continue;

        int first = i - 1, last = i + 1;
        while (first >= 0 && occupied(first)) 
            --first;
        while (last < state_.sample_points.cols() && occupied(last)) 
            ++last;
        if (first < 0)
        {
            LOGW(OPTIMIZER_TAG) << "rebound segment no free entry index=" << i;
            first = 0;
        }
        if (last >= state_.sample_points.cols())
        {
            LOGW(OPTIMIZER_TAG) << "rebound segment no free endpoint index=" << i;
            state_.stop_reason = StopReason::Error;
            return;
        }
        segments.emplace_back(first, last);
        i = last + 1;
    }
    if (segments.empty()) 
        return;

    // a*搜索
    std::vector<std::vector<Eigen::Vector3d>> paths;
    for (std::size_t i = 0; i < segments.size();)
    {
        std::vector<Eigen::Vector3d> path;
        LocalSearchResult result = local_search_.search(*state_.input->map,
                                                         state_.sample_points.col(segments[i].second),
                                                         state_.sample_points.col(segments[i].first),
                                                         param_.search_cells, param_.search_timeout_ms, path);
        if (result == LocalSearchResult::Success)
        {
            paths.push_back(std::move(path));
            ++i;
        }
        else if (result == LocalSearchResult::SearchError && i + 1 < segments.size())
        {
            segments[i].second = segments[i + 1].second;
            segments.erase(segments.begin() + i + 1);
        }
        else
        {
            LOGD(OPTIMIZER_TAG) << "rebound guide segment skipped begin="
                                << segments[i].first << " end=" << segments[i].second;
            segments.erase(segments.begin() + i);
        }
    }

    // 构造约束
    removeSegmentOverlap(segments);
    for (std::size_t i = 0; i < segments.size(); ++i)
        appendObstacleDirections(state_.sample_points, paths[i], segments[i], segments[i], false);
    state_.stop_reason = StopReason::Rebound;
}
} // namespace navapex::traj_opt
