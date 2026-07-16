#ifndef KEYINPUT_HPP
#define KEYINPUT_HPP

// -------------------------------------------------------------------------
// KeyInput — non-blocking terminal keypress detection (Linux/POSIX).
//
// Lets a headless loop exit on a keystroke typed into the launching terminal
// (e.g. 'q'), instead of depending on an OpenCV GUI window + cv::waitKey().
//
// Usage:
//     KeyInput keys;                 // puts stdin in non-canonical mode
//     while (running) {
//         ...
//         if (keys.quitPressed()) break;   // true if 'q' or 'Q' or ESC hit
//     }
//     // stdin restored automatically when 'keys' goes out of scope
//
// Notes:
//  - RAII: the terminal is restored in the destructor, even on early return.
//  - Reading is non-blocking, so it never stalls the acquisition loop.
//  - If stdin is not a TTY (e.g. piped/redirected), it degrades gracefully:
//    quitPressed() simply always returns false.
// -------------------------------------------------------------------------

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyInput {
public:
    KeyInput() {
        is_tty_ = isatty(STDIN_FILENO);
        if (!is_tty_) { return; }

        // Save current terminal settings so we can restore them later.
        tcgetattr(STDIN_FILENO, &orig_);
        termios raw = orig_;

        // Disable canonical mode (line buffering) and local echo so we read
        // keystrokes immediately without waiting for Enter.
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        // Make reads non-blocking.
        orig_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, orig_flags_ | O_NONBLOCK);

        active_ = true;
    }

    ~KeyInput() { restore(); }

    // Non-blocking: returns the last character read, or 0 if none available.
    char poll() {
        if (!active_) { return 0; }
        char c = 0;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        return (n == 1) ? c : 0;
    }

    // Convenience: true if the user pressed 'q', 'Q', or ESC (27).
    bool quitPressed() {
        char c = poll();
        return (c == 'q' || c == 'Q' || c == 27);
    }

    // Restore the terminal early if you need to (also called by destructor).
    void restore() {
        if (!active_) { return; }
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
        fcntl(STDIN_FILENO, F_SETFL, orig_flags_);
        active_ = false;
    }

private:
    termios orig_{};
    int     orig_flags_ = 0;
    bool    is_tty_ = false;
    bool    active_ = false;
};

#endif