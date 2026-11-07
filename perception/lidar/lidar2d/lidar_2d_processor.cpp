#include "lidar_2d_processor.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tools/log.hpp>
#include <utility>

namespace navapex::perception
{

Lidar2DProcessor::Lidar2DProcessor(Lidar2DParams param, std::shared_ptr<const robot_model::RobotModel> robot)
                                  : param_(std::move(param)), robot_(std::move(robot))
{
    reset();
}

const Lidar2DParams& Lidar2DProcessor::param() const
{
    return param_; 
}

void Lidar2DProcessor::reset()
{
    raw_.assign(static_cast<std::size_t>(param_.width) * param_.height, Unknown);
    initialized_ = false;
    last_stamp_ = 0;
    origin_ = {};
}

GridGeometry2D Lidar2DProcessor::geometry() const
{
    GridGeometry2D result;
    result.resolution = param_.resolution;
    result.width = param_.width;
    result.height = param_.height;
    result.origin_x = origin_.x;
    result.origin_y = origin_.y;
    return result;
}

bool Lidar2DProcessor::cell(Position2D point, int &x, int &y) const
{
    return geometry().worldToCell(point.x, point.y, x, y);
}

void Lidar2DProcessor::moveWindow(Position2D position)
{    
    // 计算新的origin_
    Position2D next{(std::floor(position.x / param_.resolution) - param_.width / 2) * param_.resolution,
                    (std::floor(position.y / param_.resolution) - param_.height / 2) * param_.resolution};
    if (!initialized_)
    {
        origin_ = next;
        return;
    }
    double shift_x = std::round((next.x - origin_.x) / param_.resolution);
    double shift_y = std::round((next.y - origin_.y) / param_.resolution);
    if (shift_x == 0 && shift_y == 0) return;
    std::vector<std::uint8_t> moved(raw_.size(), Unknown);
    if (std::abs(shift_x) < param_.width && std::abs(shift_y) < param_.height)
    {
        int dx = static_cast<int>(shift_x), dy = static_cast<int>(shift_y);
        for (int y = std::max(0, -dy); y < std::min(param_.height, param_.height - dy); ++y)
        {
            for (int x = std::max(0, -dx); x < std::min(param_.width, param_.width - dx); ++x)
            {
                moved[static_cast<std::size_t>(y) * param_.width + x] =
                    raw_[static_cast<std::size_t>(y + dy) * param_.width + x + dx];
            }
        }   
    }
    raw_.swap(moved);
    origin_ = next;
}

void Lidar2DProcessor::clearRay(Position2D start, Position2D end)
{
    double ox = std::round(origin_.x / param_.resolution), oy = std::round(origin_.y / param_.resolution);
    double ax = start.x / param_.resolution - ox;
    double ay = start.y / param_.resolution - oy;
    double dx = end.x / param_.resolution - ox - ax;
    double dy = end.y / param_.resolution - oy - ay;
    
    // 0 <  a + u * direction < W
    // u = -a / direction
    // u = W - a / direction
    double enter = 0, leave = 1;
    auto clip = [&](double a, double d, double limit)
    {
        if (d == 0) return a >= 0 && a < limit;
        double near = -a / d, far = (limit - a) / d;
        if (near > far) std::swap(near, far);
        enter = std::max(enter, near);
        leave = std::min(leave, far);
        return enter <= leave;
    };
    if (!clip(ax, dx, param_.width) || !clip(ay, dy, param_.height)) return;
    double max_x = std::nextafter(static_cast<double>(param_.width), 0.0);
    double max_y = std::nextafter(static_cast<double>(param_.height), 0.0);
    double sx = std::clamp(ax + enter * dx, 0.0, max_x);
    double sy = std::clamp(ay + enter * dy, 0.0, max_y);
    double ex = std::clamp(ax + leave * dx, 0.0, max_x);
    double ey = std::clamp(ay + leave * dy, 0.0, max_y);
    
    // DDA
    // x(t) = sx + vx * t
    // y(t) = sy + vy * t
    int x = static_cast<int>(std::floor(sx));
    int y = static_cast<int>(std::floor(sy));
    int end_x = static_cast<int>(std::floor(ex));
    int end_y = static_cast<int>(std::floor(ey));  
    double vx = ex - sx, vy = ey - sy;
    int step_x = vx > 0 ? 1 : -1, step_y = vy > 0 ? 1 : -1;
    constexpr double inf = std::numeric_limits<double>::infinity();
    double delta_x = vx == 0 ? inf : 1 / std::abs(vx);
    double delta_y = vy == 0 ? inf : 1 / std::abs(vy);

    // if cross_x > cross_y ,  y + 1 faster 
    double cross_x = vx == 0 ? inf : (vx > 0 ? x + 1 - sx : sx - x) / std::abs(vx);
    double cross_y = vy == 0 ? inf : (vy > 0 ? y + 1 - sy : sy - y) / std::abs(vy);
    while (x >= 0 && y >= 0 && x < param_.width && y < param_.height)
    {
        raw_[static_cast<std::size_t>(y) * param_.width + x] = Free;
        if (x == end_x && y == end_y) 
            break;
        if (cross_x < cross_y){ 
            x += step_x; 
            cross_x += delta_x; 
        }else if (cross_y < cross_x){ 
            y += step_y; 
            cross_y += delta_y;
        }else{ 
            x += step_x; 
            y += step_y; 
            cross_x += delta_x; 
            cross_y += delta_y; 
        }
    }
}

std::vector<std::uint8_t> Lidar2DProcessor::inflatedCosts() const
{
    auto costs = raw_;
    if (param_.inflation_radius == 0) return costs;

    cv::Mat seeds(param_.height, param_.width, CV_8UC1, cv::Scalar(255));
    bool any = false;
    for (int y = 0; y < param_.height; ++y)
    {
        for (int x = 0; x < param_.width; ++x)
        {
            if (raw_[static_cast<std::size_t>(y) * param_.width + x] == Lethal)
            {
                seeds.at<unsigned char>(y, x) = 0;
                any = true;
            }
        }
    }
    if (!any) return costs;   //无障碍物点 return
    
    cv::Mat distances;
    cv::distanceTransform(seeds, distances, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    double cell_padding = std::sqrt(0.5) * param_.resolution;
    for (int y = 0; y < param_.height; ++y)
    {
        for (int x = 0; x < param_.width; ++x)
        {
            auto index = static_cast<std::size_t>(y) * param_.width + x;
            if (raw_[index] != Free) 
                continue;
            double distance = std::max(0.0, distances.at<float>(y, x) * param_.resolution - cell_padding);
            if (param_.forbidden_radius > 0 && distance <= param_.forbidden_radius)
            {
                costs[index] = Forbidden;
            }else if (distance <= param_.inflation_radius){
                costs[index] = static_cast<std::uint8_t>(std::clamp(
                    252.0 * std::exp(-param_.cost_scaling *
                        (distance - param_.forbidden_radius)), 1.0, 252.0));
            }
        }
    }  
    return costs;
}

bool Lidar2DProcessor::validateTiming(const LaserScan2D &scan, const RobotState &state,
                                      double now) const
{
    if (!std::isfinite(scan.stamp) || !std::isfinite(state.stamp) || !std::isfinite(now))
    {
        LOGW(PERCEPTION_TAG) << "Scan, pose and now timestamps must be finite";
        return false;
    }
    if (scan.stamp > now || state.stamp > now || 
        now > scan.stamp + param_.max_input_age || now > state.stamp + param_.max_input_age)
    {
        LOGW(PERCEPTION_TAG) << "Scan or pose is outside the allowed time window: scan="
                             << scan.stamp << " pose=" << state.stamp << " now=" << now;
        return false;
    }
    if (state.stamp > scan.stamp + param_.max_pose_scan_dt || scan.stamp > state.stamp + param_.max_pose_scan_dt)
    {
        LOGW(PERCEPTION_TAG) << "Pose/scan time difference exceeds max_pose_scan_dt";
        return false;
    }
    if (initialized_ && scan.stamp <= last_stamp_)
    {
        LOGW(PERCEPTION_TAG) << "Scan timestamps must be strictly increasing; reset after a clock restart";
        return false;
    }
    return true;
}

PerceptionResult Lidar2DProcessor::update(const PerceptionInput &input)
{
    auto frame = std::get_if<std::shared_ptr<const LaserScan2D>>(&input.sensor);
    
    const auto &scan = **frame;
    if (!validateTiming(scan, input.state, input.now)) return {};
    
    // 1.移动窗口
    moveWindow(toPosition2D(input.state.pose));
    auto pose = toPose2D(input.state.pose);
    Position2D laser_origin = transformPoint(pose, toPosition2D(param_.base_from_laser));
    
    std::vector<std::size_t> hits;
    hits.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        // 2.判段是否有回波
        double range = scan.ranges[i];
        bool no_return = (range == scan.range_max) || (param_.inf_is_free && std::isinf(range) && range > 0);
        if (!no_return && (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)) // 清除无效回波
        {
            continue;
        }
        // 3.射线转到base_link
        double angle = scan.angle_min + i * scan.angle_increment;
        double c = std::cos(angle);
        double s = std::sin(angle);
        double ray_range = no_return ? scan.range_max : range;
        Position2D base_endpoint = transformPoint(param_.base_from_laser, {ray_range * c, ray_range * s});
        if (!no_return && robot_->contains(base_endpoint, param_.self_filter_padding))  // 清除打到自身的回波
        {
            continue;
        }

        // 4.射线转到map
        Position2D endpoint = transformPoint(pose, base_endpoint);

        // 5.射线遍历网格
        clearRay(laser_origin, endpoint);
        // 6.有回波的标记
        if (!no_return)
        {
            int x, y;
            if (cell(endpoint, x, y))
            {
                hits.push_back(static_cast<std::size_t>(y) * param_.width + x);
            }
        }
    }

    // 7.标记设为占据
    for (auto index : hits){
        raw_[index] = Lethal;
    }
    
    // 8.膨胀
    auto costs = inflatedCosts();
    auto snapshot = std::make_shared<const GridMap2D>(geometry(), std::move(costs));
    initialized_ = true;
    last_stamp_ = scan.stamp;

    PerceptionResult result;
    result.success = true;
    result.stamp = input.now;
    result.map = std::move(snapshot);
    return result;
}

} // namespace navapex::perception
