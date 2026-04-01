/**
 * @file motion_controller.hpp
 * Integrates LTV and RAMSETE controllers with EZ-Template.
 *
 * Provides blocking moveTo / followPath functions that generate a
 * trapezoidal-velocity trajectory and track it with the selected
 * feedback controller, outputting directly to chassis.drive_set().
 *
 * Usage in autons:
 *   MotionController mc(chassis);                        // default RAMSETE
 *   mc.moveTo(24, 48, 45, 60);                           // x,y,θ°,speed
 *   mc.followPath({{12,24,NAN,70},{36,48,90,50}});        // multi-waypoint
 */
#pragma once

#include "controllers/ltv_controller.hpp"
#include "controllers/ramsete_controller.hpp"
#include "robot_config.hpp"
#include "EZ-Template/drive/drive.hpp"

#include <cmath>
#include <vector>
#include <algorithm>

class MotionController {
public:
    enum class Type { LTV, RAMSETE };

    struct Waypoint {
        float x, y;    // inches (EZ frame)
        float theta;   // degrees (EZ convention). NAN → auto heading toward point
        float speed;   // in/s  (0 → default)
    };

    explicit MotionController(ez::Drive& chassis, Type type = Type::RAMSETE)
        : m_chassis(chassis), m_type(type) {}

    // ── Single move ─────────────────────────────────────────────────────────

    /**
     * Drive to a pose using a trapezoidal velocity profile + feedback.
     * Blocks until settled or timeout.
     *
     * @param tx/ty/ttheta  target pose — inches / degrees
     * @param maxSpeed      max linear speed (in/s), 0 → default
     * @param timeout_ms    safety timeout
     */
    void moveTo(float tx, float ty, float ttheta,
                float maxSpeed = 0.0f, int timeout_ms = 5000) {
        if (maxSpeed <= 0.0f) maxSpeed = kDefaultSpeed;

        float cx = f(m_chassis.odom_x_get());
        float cy = f(m_chassis.odom_y_get());
        float ct = f(m_chassis.odom_theta_get());

        auto traj = buildLinearTraj(cx, cy, ct, tx, ty, ttheta, maxSpeed);
        runTrajectory(traj, tx, ty, ttheta, timeout_ms);
    }

    // ── Multi-waypoint path ─────────────────────────────────────────────────

    /**
     * Follow a sequence of waypoints.
     * Set waypoint theta to NAN to automatically face the next point.
     */
    void followPath(const std::vector<Waypoint>& path,
                    float defaultSpeed = 0.0f, int timeout_ms = 10000) {
        if (path.empty()) return;
        if (defaultSpeed <= 0.0f) defaultSpeed = kDefaultSpeed;

        std::vector<TrajPoint> fullTraj;
        float cx = f(m_chassis.odom_x_get());
        float cy = f(m_chassis.odom_y_get());
        float ct = f(m_chassis.odom_theta_get());

        for (size_t i = 0; i < path.size(); ++i) {
            const auto& wp = path[i];
            float spd   = wp.speed > 0.0f ? wp.speed : defaultSpeed;
            float theta = wp.theta;
            if (std::isnan(theta))
                theta = headingTo(cx, cy, wp.x, wp.y);

            auto seg = buildLinearTraj(cx, cy, ct, wp.x, wp.y, theta, spd);

            // Keep speed up between intermediate waypoints
            if (i < path.size() - 1 && !seg.empty()) {
                float nextSpd = path[i + 1].speed > 0 ? path[i + 1].speed : defaultSpeed;
                float floor   = std::min(spd, nextSpd) * 0.4f;
                for (auto it = seg.rbegin(); it != seg.rend(); ++it) {
                    if (it->v < floor) it->v = floor;
                    else break;
                }
            }

            fullTraj.insert(fullTraj.end(), seg.begin(), seg.end());
            cx = wp.x;
            cy = wp.y;
            ct = theta;
        }

        const auto& last = path.back();
        float lt = std::isnan(last.theta) ? ct : last.theta;
        runTrajectory(fullTraj, last.x, last.y, lt, timeout_ms);
    }

    // ── Tuning ──────────────────────────────────────────────────────────────
    void setType(Type t)                             { m_type = t; }
    void setSettleTolerance(float pos, float head)   { kSettlePos = pos; kSettleHead = head; }
    void setMaxAccel(float a)                        { kMaxAccel = a; }
    void setDefaultSpeed(float s)                    { kDefaultSpeed = s; }

private:
    // ── Constants ───────────────────────────────────────────────────────────
    static constexpr float kPi      = 3.14159265f;
    static constexpr float kDeg2Rad = kPi / 180.0f;
    static constexpr float kDt      = 0.01f;      // trajectory sample period (s)
    static constexpr int   kLoopMs  = 10;          // control loop period (ms)

    float kDefaultSpeed = RobotConfig::MAX_SPEED_INPS * 0.65f;
    float kMaxAccel     = RobotConfig::MAX_ACCELERATION_INPS2 * 0.6f;
    float kSettlePos    = 1.0f;   // inches
    float kSettleHead   = 3.0f;   // degrees
    static constexpr int kSettleMs  = 300;

    ez::Drive&        m_chassis;
    Type              m_type;
    LtvController     m_ltv;
    RamseteController m_ramsete;

    // ── Helpers ─────────────────────────────────────────────────────────────
    static float f(double d) { return static_cast<float>(d); }

    static float headingTo(float cx, float cy, float tx, float ty) {
        return std::atan2(tx - cx, ty - cy) * (180.0f / kPi);
    }

    static float wrapDeg(float d) {
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return d;
    }

    // ── Trajectory point ────────────────────────────────────────────────────
    struct TrajPoint {
        float x, y, theta;  // inches, inches, degrees
        float v, omega;     // in/s, deg/s
    };

    // ── Straight-line trajectory with trapezoidal velocity ──────────────────
    std::vector<TrajPoint> buildLinearTraj(
        float cx, float cy, float ct,
        float tx, float ty, float tt,
        float maxSpeed)
    {
        float dx   = tx - cx, dy = ty - cy;
        float dist = std::hypot(dx, dy);
        if (dist < 0.25f) return {};

        float headDeg = headingTo(cx, cy, tx, ty);

        // Trapezoidal profile
        float accelDist = maxSpeed * maxSpeed / (2.0f * kMaxAccel);
        if (2.0f * accelDist > dist) {
            accelDist = dist / 2.0f;
            maxSpeed  = std::sqrt(2.0f * kMaxAccel * accelDist);
        }
        float cruiseDist = dist - 2.0f * accelDist;
        float tAccel     = maxSpeed / kMaxAccel;
        float tCruise    = maxSpeed > 1e-3f ? cruiseDist / maxSpeed : 0.0f;
        float tTotal     = tAccel + tCruise + tAccel;

        int n = std::max(1, static_cast<int>(std::ceil(tTotal / kDt)));
        std::vector<TrajPoint> out;
        out.reserve(n + 1);

        float ux = dx / dist, uy = dy / dist;

        for (int i = 0; i <= n; ++i) {
            float t = std::min(static_cast<float>(i) * kDt, tTotal);
            float s, v;

            if (t <= tAccel) {                          // accelerate
                v = kMaxAccel * t;
                s = 0.5f * kMaxAccel * t * t;
            } else if (t <= tAccel + tCruise) {         // cruise
                float dt2 = t - tAccel;
                v = maxSpeed;
                s = accelDist + maxSpeed * dt2;
            } else {                                    // decelerate
                float dt2 = t - tAccel - tCruise;
                v = maxSpeed - kMaxAccel * dt2;
                if (v < 0.0f) v = 0.0f;
                s = accelDist + cruiseDist
                    + maxSpeed * dt2 - 0.5f * kMaxAccel * dt2 * dt2;
            }
            s = std::min(s, dist);

            // Heading: track toward target, blend to final θ in last 15 %
            float frac = s / dist;
            float theta;
            if (frac < 0.85f) {
                theta = headDeg;
            } else {
                float blend = (frac - 0.85f) / 0.15f;
                theta = headDeg + wrapDeg(tt - headDeg) * blend;
            }

            out.push_back({cx + ux * s, cy + uy * s, theta, v, 0.0f});
        }

        // Compute ω from finite-difference of heading
        for (size_t i = 1; i < out.size(); ++i)
            out[i].omega = wrapDeg(out[i].theta - out[i - 1].theta) / kDt;
        if (!out.empty())
            out[0].omega = out.size() > 1 ? out[1].omega : 0.0f;

        return out;
    }

    // ── Controller dispatch ─────────────────────────────────────────────────
    std::pair<int, int> computeCmd(float rx, float ry, float rtheta,
                                   float rv, float romega) {
        float cx = f(m_chassis.odom_x_get());
        float cy = f(m_chassis.odom_y_get());
        float ct = f(m_chassis.odom_theta_get());

        if (m_type == Type::LTV) {
            auto s = m_ltv.calculate(cx, cy, ct, rx, ry, rtheta, rv, romega);
            return {s.left, s.right};
        }
        auto s = m_ramsete.calculate(cx, cy, ct, rx, ry, rtheta, rv, romega);
        return {s.left, s.right};
    }

    // ── Trajectory executor ─────────────────────────────────────────────────
    void runTrajectory(const std::vector<TrajPoint>& traj,
                       float finalX, float finalY, float finalTheta,
                       int timeout_ms) {
        if (traj.empty()) { m_chassis.drive_set(0, 0); return; }

        uint32_t t0 = pros::millis();

        // Phase 1 — follow time-referenced trajectory
        for (size_t i = 0; i < traj.size(); ++i) {
            if (elapsed(t0) > timeout_ms) break;
            const auto& ref = traj[i];
            auto [l, r] = computeCmd(ref.x, ref.y, ref.theta,
                                     ref.v, ref.omega);
            m_chassis.drive_set(l, r);
            pros::delay(kLoopMs);
        }

        // Phase 2 — settle onto final pose
        //   Use a small creep velocity so RAMSETE gain stays nonzero.
        uint32_t t1 = pros::millis();
        while (elapsed(t1) < kSettleMs && elapsed(t0) < timeout_ms) {
            float cx = f(m_chassis.odom_x_get());
            float cy = f(m_chassis.odom_y_get());
            float ct = f(m_chassis.odom_theta_get());

            if (isSettled(cx, cy, ct, finalX, finalY, finalTheta)) break;

            float rem   = std::hypot(finalX - cx, finalY - cy);
            float creep = std::min(8.0f, rem * 4.0f);
            auto [l, r] = computeCmd(finalX, finalY, finalTheta, creep,
                                     wrapDeg(finalTheta - ct) * 2.0f);
            m_chassis.drive_set(l, r);
            pros::delay(kLoopMs);
        }

        m_chassis.drive_set(0, 0);
    }

    int elapsed(uint32_t since) const {
        return static_cast<int>(pros::millis() - since);
    }

    bool isSettled(float cx, float cy, float ct,
                   float tx, float ty, float tt) const {
        return std::hypot(tx - cx, ty - cy) < kSettlePos
            && std::fabs(wrapDeg(tt - ct)) < kSettleHead;
    }
};
