#ifndef CONSOLE_UI_H
#define CONSOLE_UI_H

#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include "BridgeVicon.h"   // ObjectStatus

/**
 * Fixed-position terminal dashboard for vicon_bridge.
 *
 * The bridge publishes at 200 Hz and VRPN narrates every client that comes
 * and goes, so a plain log scrolls the terminal continuously and the one
 * thing you actually want — "is it streaming right now?" — is never in the
 * same place twice. This repaints a small frame in place instead: nothing
 * scrolls, and the answer is always on the same line.
 *
 * It also takes over stdout and stderr for the lifetime of the UI, so VRPN's
 * own printf/fprintf land in a bounded event pane rather than tearing through
 * the frame. Identical lines collapse into one with a repeat count, and the
 * notes VRPN itself labels harmless ("this is normal when a connection is
 * dropped") are muted down to a counter. Run with --plain to see the raw
 * stream instead; that is also what happens automatically when stdout is not
 * a terminal, so pipes, log files and systemd units are unaffected.
 */
class ConsoleUI {
public:
    ~ConsoleUI();

    /// True when stdout is a terminal that can be drawn on.
    static bool terminalAvailable();

    /// Takes over the terminal and redirects stdout/stderr. False if it could
    /// not, in which case the caller should keep logging plainly.
    bool begin();
    /// Restores the terminal and the original stdout/stderr. Idempotent.
    void end();

    /// Moves whatever stdout/stderr produced since the last call into the
    /// event pane. Cheap; call it every frame.
    void pump();

    void render(const std::vector<ObjectStatus>& objects,
                const std::string& config_path, int output_hz);

    bool active() const { return active_; }

private:
    struct Event {
        std::string when;
        std::string text;
        unsigned    repeats = 1;
    };

    void emit(const std::string& line);
    void write_out(const std::string& s) const;
    int  width() const;
    int  height() const;

    bool active_      = false;
    int  ui_fd_       = -1;   // dup of the real stdout, where the frame goes
    int  saved_out_   = -1;
    int  saved_err_   = -1;
    int  pipe_r_      = -1;
    int  pipe_w_      = -1;
    std::string partial_;     // trailing bytes of a line still being written
    std::deque<Event> events_;
    unsigned long muted_ = 0;
    std::chrono::steady_clock::time_point started_;
};

#endif
