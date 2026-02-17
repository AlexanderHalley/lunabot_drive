#pragma once
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdint>

class MotorLogger {
public:
    explicit MotorLogger(const std::string& log_dir = "/home/lunabot/logs") {
        // Generate filename: motor_log_YYYYMMDD_HHMMSS.csv
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << log_dir << "/motor_log_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        filepath_ = oss.str();

        file_.open(filepath_);
        if (file_.is_open()) {
            file_ << "timestamp_s,motor_id,velocity_rpm,position_rot,"
                     "current_a,voltage_v,temperature_c,duty_cycle,"
                     "faults,sticky_faults\n";
        }
    }

    ~MotorLogger() {
        if (file_.is_open()) file_.close();
    }

    void log(double timestamp, int motor_id,
             float velocity, float position,
             float current, float voltage,
             float temperature, float duty_cycle,
             uint16_t faults, uint16_t sticky_faults)
    {
        if (!file_.is_open()) return;
        file_ << std::fixed << std::setprecision(4)
              << timestamp     << ","
              << motor_id      << ","
              << velocity      << ","
              << position      << ","
              << current       << ","
              << voltage       << ","
              << temperature   << ","
              << duty_cycle    << ","
              << faults        << ","
              << sticky_faults << "\n";
    }

    bool is_open() const { return file_.is_open(); }
    std::string filepath() const { return filepath_; }

private:
    std::ofstream file_;
    std::string filepath_;
};
