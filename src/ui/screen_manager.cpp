/**
 * @file screen_manager.cpp
 * Minimal two-page brain UI.
 *
 * Page 1 — SELECT:  scrollable auton list with prev/next buttons
 * Page 2 — INFO:    minimap + odom readout + motor temps + battery
 */
#include "ui/screen_manager.hpp"
#include "ui/field_display.hpp"
#include "ui/theme.hpp"
#include "EZ-Template/sdcard.hpp"

#include "pros/screen.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ScreenManager {

namespace {

// ── Pages ────────────────────────────────────────────────────────────────────
enum class Page { SELECT, INFO };
static Page s_page     = Page::SELECT;
static Page s_pagePrev = Page::SELECT;
static bool s_dirty    = true;
static uint32_t s_lastTouch = 0;

// ── Touch regions ────────────────────────────────────────────────────────────
// Header tabs
static constexpr UITheme::Rect TAB_SELECT = {140, 2, 260, 20};
static constexpr UITheme::Rect TAB_INFO   = {268, 2, 388, 20};

// SELECT page buttons
static constexpr UITheme::Rect BTN_PREV   = {340, 38,  408, 62};
static constexpr UITheme::Rect BTN_NEXT   = {340, 70,  408, 94};
static constexpr UITheme::Rect BTN_SKILLS = {340, 102, 460, 126};

// INFO page buttons
static constexpr UITheme::Rect BTN_ZERO_REL = {16, 178, 146, 202};

// Auton list visible rows (up to 7 rows shown, row height = 26)
static constexpr int LIST_X0      = 12;
static constexpr int LIST_X1      = 326;
static constexpr int LIST_Y0      = UITheme::kBodyY + 10;
static constexpr int LIST_ROW_H   = 26;
static constexpr int LIST_VISIBLE = 7;

inline bool inside(const UITheme::Rect& r, int x, int y) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

// ── Scroll offset for list ───────────────────────────────────────────────────
static int s_scrollOffset = 0;
static bool s_relativeFrameActive = false;
static float s_relativeOriginX = 0.0f;
static float s_relativeOriginY = 0.0f;
static float s_relativeOriginTheta = 0.0f;
static float s_lastAbsX = 0.0f;
static float s_lastAbsY = 0.0f;
static float s_lastAbsTheta = 0.0f;

struct DisplayPose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

double wrapDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

DisplayPose toDisplayPose(float absX, float absY, float absTheta) {
    if (!s_relativeFrameActive) return {absX, absY, absTheta};

    const double dx = static_cast<double>(absX) - s_relativeOriginX;
    const double dy = static_cast<double>(absY) - s_relativeOriginY;
    const double originRad = static_cast<double>(s_relativeOriginTheta) * 3.14159265358979323846 / 180.0;
    const double cosA = std::cos(originRad);
    const double sinA = std::sin(originRad);

    return {
        static_cast<float>((dx * cosA) - (dy * sinA)),
        static_cast<float>((dx * sinA) + (dy * cosA)),
        static_cast<float>(wrapDeg(static_cast<double>(absTheta) - s_relativeOriginTheta)),
    };
}

void selectAutonPage(int index) {
    int count = ez::as::auton_selector.auton_count;
    if (count <= 0) return;

    if (index < 0) index = count - 1;
    if (index >= count) index = 0;

    ez::as::auton_selector.last_auton_page_current = ez::as::auton_selector.auton_page_current;
    ez::as::auton_selector.auton_page_current = index;
    ez::as::auto_sd_update();
}

void ensureVisible(int index, int count) {
    if (index < s_scrollOffset) s_scrollOffset = index;
    if (index >= s_scrollOffset + LIST_VISIBLE) s_scrollOffset = index - LIST_VISIBLE + 1;
    if (s_scrollOffset < 0) s_scrollOffset = 0;
    int maxOff = std::max(0, count - LIST_VISIBLE);
    if (s_scrollOffset > maxOff) s_scrollOffset = maxOff;
}

// ── Touch handler ────────────────────────────────────────────────────────────
void handleTouch() {
    auto t = pros::screen::touch_status();
    bool pressed = (t.touch_status == pros::E_TOUCH_PRESSED);
    if (!pressed) return;

    uint32_t now = pros::millis();
    if (now - s_lastTouch < 180) return;
    s_lastTouch = now;

    int x = t.x, y = t.y;

    // Tab bar
    if (y <= UITheme::kHeaderH) {
        if (inside(TAB_SELECT, x, y)) { s_page = Page::SELECT; return; }
        if (inside(TAB_INFO,   x, y)) { s_page = Page::INFO;   return; }
        return;
    }

    // SELECT page buttons
    if (s_page == Page::SELECT) {
        if (inside(BTN_PREV, x, y)) {
            selectAutonPage(ez::as::auton_selector.auton_page_current - 1);
            return;
        }
        if (inside(BTN_NEXT, x, y)) {
            selectAutonPage(ez::as::auton_selector.auton_page_current + 1);
            return;
        }

        // Skills shortcut
        if (inside(BTN_SKILLS, x, y)) {
            // Find first auton with "skills" in the name
            if (ez::as::auton_selector.auton_count > 0) {
                for (size_t i = 0; i < ez::as::auton_selector.Autons.size(); ++i) {
                    std::string name = ez::as::auton_selector.Autons[i].Name;
                    for (char& c : name)
                        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (name.find("skills") != std::string::npos) {
                        selectAutonPage(static_cast<int>(i));
                        return;
                    }
                }
            }
            return;
        }

        // Tap on list row to select directly
        for (int row = 0; row < LIST_VISIBLE; ++row) {
            int ry = LIST_Y0 + row * LIST_ROW_H;
            if (x >= LIST_X0 && x <= LIST_X1 && y >= ry && y < ry + LIST_ROW_H) {
                int autonIdx = s_scrollOffset + row;
                if (autonIdx >= 0 && autonIdx < ez::as::auton_selector.auton_count) {
                    selectAutonPage(autonIdx);
                }
                return;
            }
        }
    }

    if (s_page == Page::INFO) {
        if (inside(BTN_ZERO_REL, x, y)) {
            s_relativeFrameActive = true;
            s_relativeOriginX = s_lastAbsX;
            s_relativeOriginY = s_lastAbsY;
            s_relativeOriginTheta = s_lastAbsTheta;
            FieldDisplay::clearTrail();
            return;
        }
    }
}

// ── Draw helpers ─────────────────────────────────────────────────────────────

void drawTabs(Page active) {
    // Brand
    UITheme::text(pros::E_TEXT_SMALL, 8, 6, UITheme::kWhite, UITheme::kBgHeader,
                  "BIGGA 5");

    // Tab buttons
    UITheme::drawBtn(TAB_SELECT, "SELECT", active == Page::SELECT);
    UITheme::drawBtn(TAB_INFO,   "INFO",   active == Page::INFO);
}

void drawSelectPage(const ViewModel& vm) {
    // Clear content area
    UITheme::fill(UITheme::makeRect(0, UITheme::kBodyY, UITheme::kScreenW,
                                    UITheme::kScreenH - UITheme::kBodyY), UITheme::kBg);

    int count = ez::as::auton_selector.auton_count;
    int current = ez::as::auton_selector.auton_page_current;

    // Ensure selected is visible
    ensureVisible(current, count);

    // Draw list rows
    for (int row = 0; row < LIST_VISIBLE; ++row) {
        int idx = s_scrollOffset + row;
        int ry  = LIST_Y0 + row * LIST_ROW_H;

        if (idx >= count) {
            // Empty row
            UITheme::fill(UITheme::makeRect(LIST_X0, ry, LIST_X1 - LIST_X0, LIST_ROW_H - 1),
                          UITheme::kBg);
            continue;
        }

        bool selected = (idx == current);
        uint32_t rowBg   = selected ? UITheme::kHighlight : UITheme::kBg;
        uint32_t rowText = selected ? UITheme::kWhite : UITheme::kGray;

        // Row background
        UITheme::fill(UITheme::makeRect(LIST_X0, ry, LIST_X1 - LIST_X0, LIST_ROW_H - 1), rowBg);

        // Selection indicator: simple ">" marker
        if (selected) {
            UITheme::text(pros::E_TEXT_MEDIUM, LIST_X0 + 4, ry + 5,
                          UITheme::kWhite, rowBg, ">");
        }

        // Auton name
        const char* name = (idx < static_cast<int>(ez::as::auton_selector.Autons.size()))
                               ? ez::as::auton_selector.Autons[idx].Name.c_str()
                               : "???";
        UITheme::text(pros::E_TEXT_MEDIUM, LIST_X0 + 22, ry + 5, rowText, rowBg, "%.28s", name);

        // Row divider
        UITheme::hline(LIST_X0, LIST_X1, ry + LIST_ROW_H - 1, UITheme::kDivider);
    }

    // Scroll indicator (dots on right side of list)
    if (count > LIST_VISIBLE) {
        int barH = UITheme::kScreenH - LIST_Y0 - 10;
        int thumbH = std::max(8, barH * LIST_VISIBLE / count);
        int thumbY = LIST_Y0 + (barH - thumbH) * s_scrollOffset / std::max(1, count - LIST_VISIBLE);
        UITheme::fill(UITheme::makeRect(LIST_X1 + 2, LIST_Y0, 3, barH), UITheme::kBgAlt);
        UITheme::fill(UITheme::makeRect(LIST_X1 + 2, thumbY, 3, thumbH), UITheme::kDarkGray);
    }

    // Right side — buttons + info
    UITheme::drawBtn(BTN_PREV, "PREV");
    UITheme::drawBtn(BTN_NEXT, "NEXT");
    UITheme::drawBtn(BTN_SKILLS, "SKILLS", false);

    // Selected auton name (large, below buttons)
    UITheme::fill(UITheme::makeRect(340, 140, 130, 36), UITheme::kBg);
    UITheme::text(pros::E_TEXT_SMALL, 340, 140, UITheme::kDimGray, UITheme::kBg, "SELECTED");
    UITheme::text(pros::E_TEXT_MEDIUM, 340, 156, UITheme::kWhite, UITheme::kBg,
                  "%.14s", vm.autonName.c_str());

    // Index display
    UITheme::text(pros::E_TEXT_SMALL, 340, 186, UITheme::kDimGray, UITheme::kBg,
                  "%d / %d", current + 1, count);

    // Bottom status bar
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kScreenH - 16, UITheme::kBorderLite);
    UITheme::fill(UITheme::makeRect(0, UITheme::kScreenH - 15, UITheme::kScreenW, 15), UITheme::kBgAlt);
    UITheme::text(pros::E_TEXT_SMALL, 8, UITheme::kScreenH - 12,
                  UITheme::kDimGray, UITheme::kBgAlt, "%.46s", vm.status.c_str());

    // Battery in bottom right
    char batBuf[24];
    std::snprintf(batBuf, sizeof(batBuf), "%.1fV  %.0f%%", vm.batteryVolts, vm.batteryPct);
    uint32_t batCol = vm.batteryPct > 30.f ? UITheme::kDimGray
                    : vm.batteryPct > 15.f ? UITheme::kWarn
                                           : UITheme::kDanger;
    int batW = UITheme::textW(pros::E_TEXT_SMALL, batBuf);
    UITheme::text(pros::E_TEXT_SMALL, UITheme::kScreenW - batW - 8, UITheme::kScreenH - 12,
                  batCol, UITheme::kBgAlt, "%s", batBuf);
}

void drawInfoPage(const ViewModel& vm) {
    const DisplayPose pose = toDisplayPose(vm.odomX, vm.odomY, vm.odomTheta);

    UITheme::fill(UITheme::makeRect(0, UITheme::kBodyY, UITheme::kScreenW,
                                    UITheme::kScreenH - UITheme::kBodyY), UITheme::kBg);

    // ── Left: Minimap ────────────────────────────────────────────────────
    constexpr int MAP_SIZE = 130;
    constexpr int MAP_X = 16;
    constexpr int MAP_Y = UITheme::kBodyY + 8;

    UITheme::text(pros::E_TEXT_SMALL, MAP_X, MAP_Y - 2, UITheme::kDimGray, UITheme::kBg,
                  s_relativeFrameActive ? "FIELD (REL)" : "FIELD (ABS)");
    FieldDisplay::draw(MAP_X, MAP_Y + 12, MAP_SIZE, pose.x, pose.y, pose.theta);
    UITheme::drawBtn(BTN_ZERO_REL, "ZERO HERE");
    UITheme::text(pros::E_TEXT_SMALL, MAP_X, 208, UITheme::kDimGray, UITheme::kBg,
                  s_relativeFrameActive ? "Frame: relative 0,0,0" : "Frame: absolute");

    // ── Right column: Telemetry ──────────────────────────────────────────
    constexpr int COL_X = 164;
    constexpr int COL_W = 306;
    int yy = UITheme::kBodyY + 6;

    // Odom section
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "ODOMETRY");
    yy += 16;

    char xBuf[20], yBuf[20], hBuf[20];
    std::snprintf(xBuf, sizeof(xBuf), "%+.1f", pose.x);
    std::snprintf(yBuf, sizeof(yBuf), "%+.1f", pose.y);
    std::snprintf(hBuf, sizeof(hBuf), "%.1f", pose.theta);

    // X / Y / H in a clean row layout
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "X");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 20, yy - 2, UITheme::kWhite, UITheme::kBg, "%s in", xBuf);

    UITheme::text(pros::E_TEXT_SMALL, COL_X + 120, yy, UITheme::kDimGray, UITheme::kBg, "Y");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 140, yy - 2, UITheme::kWhite, UITheme::kBg, "%s in", yBuf);

    yy += 22;
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "HDG");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2, UITheme::kWhite, UITheme::kBg, "%s deg", hBuf);

    // Divider
    yy += 24;
    UITheme::hline(COL_X, COL_X + COL_W - 1, yy, UITheme::kDivider);
    yy += 8;

    // Diagnostics section
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "DIAGNOSTICS");
    yy += 16;

    // Battery
    char batBuf[32];
    std::snprintf(batBuf, sizeof(batBuf), "%.1fV  (%.0f%%)", vm.batteryVolts, vm.batteryPct);
    uint32_t batCol = vm.batteryPct > 30.f ? UITheme::kGray
                    : vm.batteryPct > 15.f ? UITheme::kWarn
                                           : UITheme::kDanger;
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "BAT");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2, batCol, UITheme::kBg, "%s", batBuf);
    yy += 22;

    // Motor temps
    char tempBuf[48];
    uint32_t tempCol = vm.motorTempMax < 45.f ? UITheme::kGray
                     : vm.motorTempMax < 55.f ? UITheme::kWarn
                                              : UITheme::kDanger;
    std::snprintf(tempBuf, sizeof(tempBuf), "%.0f C", vm.motorTempMax);
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "TEMP");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 40, yy - 2, tempCol, UITheme::kBg, "%s", tempBuf);
    if (!vm.hotMotor.empty()) {
        UITheme::text(pros::E_TEXT_SMALL, COL_X + 120, yy, UITheme::kDarkGray, UITheme::kBg,
                      "(%s)", vm.hotMotor.c_str());
    }
    yy += 22;

    // IMU status
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "IMU");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2,
                  vm.imuCalibrated ? UITheme::kGray : UITheme::kWarn, UITheme::kBg,
                  vm.imuCalibrated ? "OK" : "Calibrating...");
    yy += 22;

    // Competition status
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "COMP");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 40, yy - 2,
                  vm.compConnected ? UITheme::kWhite : UITheme::kDarkGray, UITheme::kBg,
                  vm.compConnected ? "Connected" : "Not connected");

    // Bottom: selected auton reminder
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kScreenH - 16, UITheme::kBorderLite);
    UITheme::fill(UITheme::makeRect(0, UITheme::kScreenH - 15, UITheme::kScreenW, 15), UITheme::kBgAlt);
    UITheme::text(pros::E_TEXT_SMALL, 8, UITheme::kScreenH - 12,
                  UITheme::kDimGray, UITheme::kBgAlt, "Auton: %s", vm.autonName.c_str());
}

}  // anonymous namespace

// ── Public API ───────────────────────────────────────────────────────────────

void init() {
    UITheme::clearScreen();
    FieldDisplay::init();
    s_page = Page::SELECT;
    s_pagePrev = Page::SELECT;
    s_dirty = true;
    s_scrollOffset = 0;
    s_relativeFrameActive = false;
    s_relativeOriginX = 0.0f;
    s_relativeOriginY = 0.0f;
    s_relativeOriginTheta = 0.0f;
}

void render(const ViewModel& vm) {
    s_lastAbsX = vm.odomX;
    s_lastAbsY = vm.odomY;
    s_lastAbsTheta = vm.odomTheta;
    handleTouch();

    if (s_page != s_pagePrev) {
        s_pagePrev = s_page;
        s_dirty = true;
    }

    if (s_dirty) {
        UITheme::clearScreen();
        s_dirty = false;
    }

    // Header
    UITheme::fill(UITheme::makeRect(0, 0, UITheme::kScreenW, UITheme::kHeaderH), UITheme::kBgHeader);
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kHeaderH, UITheme::kBorder);
    drawTabs(s_page);

    // Page content
    switch (s_page) {
        case Page::SELECT: drawSelectPage(vm); break;
        case Page::INFO:   drawInfoPage(vm);   break;
    }
}

void renderBoot(float progress, const char* label) {
    if (progress < 0.f) progress = 0.f;
    if (progress > 1.f) progress = 1.f;

    UITheme::clearScreen();

    // Centered boot card
    constexpr int cardW = 280;
    constexpr int cardH = 80;
    int cx = (UITheme::kScreenW - cardW) / 2;
    int cy = (UITheme::kScreenH - cardH) / 2;

    UITheme::fill(UITheme::makeRect(cx, cy, cardW, cardH), UITheme::kBgAlt);
    UITheme::outline(UITheme::makeRect(cx, cy, cardW, cardH), UITheme::kBorder);

    UITheme::textCenter(pros::E_TEXT_LARGE,
                        UITheme::makeRect(cx, cy, cardW, 30), cy + 10,
                        UITheme::kWhite, UITheme::kBgAlt, "BIGGA 5");

    // Label
    UITheme::textCenter(pros::E_TEXT_SMALL,
                        UITheme::makeRect(cx, cy, cardW, 30), cy + 36,
                        UITheme::kDimGray, UITheme::kBgAlt, "%s", label ? label : "");

    // Progress bar
    UITheme::drawProgressBar(UITheme::makeRect(cx + 20, cy + 54, cardW - 40, 10), progress);
}

bool isInfoPageActive() {
    return s_page == Page::INFO;
}

}  // namespace ScreenManager
