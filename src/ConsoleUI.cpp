#include "ConsoleUI.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr size_t kMaxEvents   = 8;    // most event lines ever shown
constexpr size_t kMinEvents   = 2;    // ...and the fewest, before dropping the pane
constexpr int    kMinWidth    = 56;
constexpr int    kMaxWidth    = 140;

/* ANSI, suppressed when NO_COLOR is set (https://no-color.org). */
bool colorEnabled()
{
    static const bool on = (::getenv("NO_COLOR") == nullptr);
    return on;
}
std::string sgr(const char* code)
{
    return colorEnabled() ? std::string("\033[").append(code).append("m") : std::string();
}
const std::string& reset()  { static const std::string s = sgr("0");    return s; }
const std::string& dim()    { static const std::string s = sgr("2");    return s; }
const std::string& bold()   { static const std::string s = sgr("1");    return s; }
const std::string& green()  { static const std::string s = sgr("32");   return s; }
const std::string& yellow() { static const std::string s = sgr("33");   return s; }
const std::string& red()    { static const std::string s = sgr("31");   return s; }
const std::string& cyan()   { static const std::string s = sgr("36");   return s; }

/* Number of printable columns, treating a UTF-8 sequence as one column. */
size_t columns(const std::string& s)
{
    size_t n = 0;
    for (unsigned char c : s)
        if ((c & 0xC0) != 0x80) n++;      // count lead bytes only
    return n;
}

/* Truncate to `w` columns without splitting a UTF-8 sequence. */
std::string fit(const std::string& s, size_t w)
{
    if (columns(s) <= w) return s;
    if (w == 0) return "";
    size_t cols = 0, i = 0;
    for (; i < s.size(); ++i) {
        if ((static_cast<unsigned char>(s[i]) & 0xC0) != 0x80) {
            if (cols == w - 1) break;
            cols++;
        }
    }
    return s.substr(0, i) + "…";     // ellipsis
}

std::string pad(const std::string& s, size_t w)
{
    const std::string t = fit(s, w);
    return t + std::string(w - columns(t), ' ');
}

/* A table cell: like pad(), but always keeps one blank column at the end so
 * neighbouring columns cannot run together once the terminal gets narrow
 * enough that the elastic columns are squeezed to their share. */
std::string cell(const std::string& s, size_t w)
{
    return w > 1 ? pad(fit(s, w - 1), w) : pad(s, w);
}

std::string clockNow()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    ::localtime_r(&t, &tm);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

std::string uptime(std::chrono::steady_clock::time_point since)
{
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::steady_clock::now() - since).count();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld:%02lld:%02lld",
                  static_cast<long long>(secs / 3600),
                  static_cast<long long>((secs / 60) % 60),
                  static_cast<long long>(secs % 60));
    return buf;
}

/* Lines VRPN itself describes as normal. They arrive on every client connect
 * and disconnect, which is exactly the traffic that made the terminal roll. */
bool isMutedNote(const std::string& line)
{
    static const char* const patterns[] = {
        "check_vrpn_cookie(): VRPN Note:",
        "this is normal when a connection is dropped",
        "vrpn: TCP connection request received",
    };
    for (const char* p : patterns)
        if (line.find(p) != std::string::npos) return true;
    return false;
}

} // namespace

ConsoleUI::~ConsoleUI() { end(); }

bool ConsoleUI::terminalAvailable()
{
    if (!::isatty(STDOUT_FILENO)) return false;
    const char* term = ::getenv("TERM");
    return term && *term && std::strcmp(term, "dumb") != 0;
}

bool ConsoleUI::begin()
{
    if (active_) return true;
    if (!terminalAvailable()) return false;

    ui_fd_ = ::dup(STDOUT_FILENO);
    if (ui_fd_ < 0) return false;

    int fds[2];
    if (::pipe(fds) != 0) { ::close(ui_fd_); ui_fd_ = -1; return false; }
    pipe_r_ = fds[0];
    pipe_w_ = fds[1];
    ::fcntl(pipe_r_, F_SETFL, O_NONBLOCK);

    saved_out_ = ::dup(STDOUT_FILENO);
    saved_err_ = ::dup(STDERR_FILENO);
    ::fflush(stdout);
    ::fflush(stderr);
    ::dup2(pipe_w_, STDOUT_FILENO);
    ::dup2(pipe_w_, STDERR_FILENO);
    /* Line buffering on the captured stdout so messages reach the pane as
     * whole lines rather than in 4 KB blocks. */
    ::setvbuf(stdout, nullptr, _IOLBF, 0);

    started_ = std::chrono::steady_clock::now();
    active_  = true;
    write_out("\033[2J\033[H\033[?25l");   // clear, home, hide cursor
    return true;
}

void ConsoleUI::end()
{
    if (!active_) return;
    active_ = false;

    ::fflush(stdout);
    ::fflush(stderr);
    if (saved_out_ >= 0) { ::dup2(saved_out_, STDOUT_FILENO); ::close(saved_out_); saved_out_ = -1; }
    if (saved_err_ >= 0) { ::dup2(saved_err_, STDERR_FILENO); ::close(saved_err_); saved_err_ = -1; }
    if (pipe_w_ >= 0) { ::close(pipe_w_); pipe_w_ = -1; }
    if (pipe_r_ >= 0) { ::close(pipe_r_); pipe_r_ = -1; }

    write_out("\033[?25h\n");              // show the cursor again
    if (ui_fd_ >= 0) { ::close(ui_fd_); ui_fd_ = -1; }
}

void ConsoleUI::write_out(const std::string& s) const
{
    if (ui_fd_ < 0) return;
    size_t off = 0;
    while (off < s.size()) {
        const ssize_t n = ::write(ui_fd_, s.data() + off, s.size() - off);
        if (n <= 0) break;
        off += static_cast<size_t>(n);
    }
}

int ConsoleUI::width() const
{
    struct winsize ws{};
    int w = 80;
    if (ui_fd_ >= 0 && ::ioctl(ui_fd_, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0)
        w = ws.ws_col;
    if (w < kMinWidth) w = kMinWidth;
    if (w > kMaxWidth) w = kMaxWidth;
    return w;
}

int ConsoleUI::height() const
{
    struct winsize ws{};
    if (ui_fd_ >= 0 && ::ioctl(ui_fd_, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0)
        return ws.ws_row;
    return 24;
}

void ConsoleUI::emit(const std::string& line)
{
    if (line.empty()) return;
    if (isMutedNote(line)) { muted_++; return; }

    /* Collapse a repeat of the newest line instead of pushing a duplicate. */
    if (!events_.empty() && events_.back().text == line) {
        events_.back().repeats++;
        events_.back().when = clockNow();
        return;
    }
    events_.push_back({clockNow(), line, 1});
    while (events_.size() > kMaxEvents) events_.pop_front();
}

void ConsoleUI::pump()
{
    if (!active_) return;
    char buf[4096];
    for (;;) {
        const ssize_t n = ::read(pipe_r_, buf, sizeof(buf));
        if (n <= 0) break;
        partial_.append(buf, static_cast<size_t>(n));
        size_t nl;
        while ((nl = partial_.find('\n')) != std::string::npos) {
            std::string line = partial_.substr(0, nl);
            partial_.erase(0, nl + 1);
            while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
                line.pop_back();
            emit(line);
        }
        /* A writer that never emits a newline must not grow this forever. */
        if (partial_.size() > 8192) { emit(partial_); partial_.clear(); }
    }
}

void ConsoleUI::render(const std::vector<ObjectStatus>& objects,
                       const std::string& config_path, int output_hz)
{
    if (!active_) return;

    const int W = width();
    const int H = height();
    const std::string rule(static_cast<size_t>(W), '-');
    std::ostringstream f;

    f << "\033[H";                          // home, then overwrite line by line
    auto line = [&](const std::string& s) { f << pad(s, W) << "\033[K\n"; };
    auto raw  = [&](const std::string& s) { f << s << "\033[K\n"; };

    /* --- column widths, sized to the actual objects ---------------------- *
     * With several objects the names and endpoints vary a lot, so the table
     * is measured rather than fixed: the two elastic columns split whatever
     * the fixed ones leave, and each gets at least enough to be recognisable. */
    constexpr size_t kInW = 8, kOutW = 8, kClientW = 9, kGap = 1;
    size_t name_w = 6;
    for (const auto& o : objects) name_w = std::max(name_w, columns(o.name));
    name_w = std::min(name_w, static_cast<size_t>(20)) + kGap;

    size_t src_w = 8, tgt_w = 8;
    for (const auto& o : objects) {
        src_w = std::max(src_w, columns(o.input)  + kGap);
        tgt_w = std::max(tgt_w, columns(o.output) + kGap);
    }
    const size_t fixed = name_w + kInW + kOutW + kClientW;
    if (static_cast<size_t>(W) > fixed + 20) {
        const size_t elastic = static_cast<size_t>(W) - fixed;
        if (src_w + tgt_w > elastic) {          // share what is left, fairly
            const size_t half = elastic / 2;
            if (src_w > half && tgt_w > half) { src_w = half; tgt_w = elastic - half; }
            else if (src_w > half)             { src_w = elastic - tgt_w; }
            else                               { tgt_w = elastic - src_w; }
        }
    } else {
        src_w = tgt_w = 10;                     // very narrow terminal
    }

    /* --- header ---------------------------------------------------------- */
    {
        std::ostringstream l;
        l << "vicon_bridge  " << objects.size()
          << (objects.size() == 1 ? " object" : " objects");
        const std::string right = "up " + uptime(started_);
        const size_t used = columns(l.str()) + columns(right);
        const size_t gap  = static_cast<size_t>(W) > used ? W - used : 1;
        raw(bold() + l.str() + reset() + std::string(gap, ' ') + dim() + right + reset());
    }
    line(rule);

    /* --- one row per object ---------------------------------------------- */
    raw(dim() + cell("OBJECT", name_w) + cell("SOURCE", src_w) + cell("OUTPUT", tgt_w)
        + cell("IN", kInW) + cell("OUT", kOutW) + "CLIENT" + reset());

    unsigned long long total_in = 0, total_out = 0;
    size_t clients = 0, down = 0;
    for (const auto& o : objects) {
        std::ostringstream in, out;
        in  << static_cast<int>(o.in_hz  + 0.5) << " Hz";
        out << static_cast<int>(o.out_hz + 0.5) << " Hz";

        const char* state;
        const std::string* col;
        if (!o.link_up)              { state = "PORT DOWN"; col = &red();    down++; }
        else if (o.client_connected) { state = "connected"; col = &green();  clients++; }
        else                         { state = "waiting";   col = &yellow(); }
        total_in  += o.in_total;
        total_out += o.out_total;

        raw(cell(o.name, name_w)
            + dim() + cell(o.input, src_w) + cell(o.output, tgt_w) + reset()
            + (o.in_hz  > 1 ? green() : red())    + cell(in.str(),  kInW)  + reset()
            + (o.out_hz > 1 ? green() : yellow()) + cell(out.str(), kOutW) + reset()
            + *col + state + reset());
    }

    /* One explanation per broken port, however many objects ride on it. */
    std::vector<std::string> explained;
    for (const auto& o : objects) {
        if (o.link_up || o.down_reason.empty()) continue;
        if (std::find(explained.begin(), explained.end(), o.link_key) != explained.end())
            continue;
        explained.push_back(o.link_key);
        raw(red() + fit("  ! " + o.link_key + ": " + o.down_reason,
                        static_cast<size_t>(W)) + reset());
    }
    line(rule);

    /* --- totals ---------------------------------------------------------- */
    {
        std::ostringstream l;
        l << "publishing at " << output_hz << " Hz   ·   "
          << clients << "/" << objects.size() << " with a client";
        if (down) l << "   ·   " << down << " on a port that is down";
        l << "   ·   " << total_in << " in / " << total_out << " out";
        raw(cyan() + fit(l.str(), static_cast<size_t>(W)) + reset());
        raw(dim() + fit("config " + config_path, static_cast<size_t>(W)) + reset());
    }
    line(rule);

    /* --- event pane, sized to whatever rows are left --------------------- */
    const int used_rows = 2                                   // header + rule
                        + 1                                   // table header
                        + static_cast<int>(objects.size())
                        + static_cast<int>(explained.size())
                        + 1 + 2 + 1                           // rule + totals + rule
                        + 2;                                  // pane title + footer
    int room = H - used_rows - 2;
    size_t shown = 0;
    if (room >= static_cast<int>(kMinEvents)) {
        shown = std::min(kMaxEvents, static_cast<size_t>(room));
        std::ostringstream h;
        h << "events";
        if (muted_) h << "   (" << muted_ << " routine VRPN notes muted)";
        raw(dim() + fit(h.str(), static_cast<size_t>(W)) + reset());

        /* Newest last, so the pane reads like a log without ever scrolling. */
        const size_t first = events_.size() > shown ? events_.size() - shown : 0;
        for (size_t i = 0; i < shown; ++i) {
            const size_t idx = first + i;
            if (idx < events_.size()) {
                const Event& e = events_[idx];
                std::string text = e.text;
                if (e.repeats > 1) text += "  (x" + std::to_string(e.repeats) + ")";
                raw(dim() + e.when + reset() + " " +
                    fit(text, static_cast<size_t>(W) - 9));
            } else {
                line("");
            }
        }
        line(rule);
    }
    raw(dim() + fit("Ctrl-C to quit   ·   --plain for scrolling log output",
                    static_cast<size_t>(W)) + reset());
    f << "\033[J";                          // wipe anything below the frame

    write_out(f.str());
}
