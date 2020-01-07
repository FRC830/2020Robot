// A wrapper around smartdashboard for setting and consuming persistent values
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

template<typename T>
T raw_get(nt::NetworkTableEntry nt_val, T default_value) {}

template<>
double raw_get(nt::NetworkTableEntry nt_val, double default_value) {
  return nt_val.GetDouble(default_value);
}
template<>
int raw_get(nt::NetworkTableEntry nt_val, int default_value) {
  return nt_val.GetDouble(default_value);
}
template<>
std::string raw_get(nt::NetworkTableEntry nt_val, std::string default_value) {
  return nt_val.GetString(default_value);
}
template<>
bool raw_get(nt::NetworkTableEntry nt_val, bool default_value) {
  return nt_val.GetBoolean(default_value);
}

template <typename T>
class persistent {
  public:

    nt::NetworkTableEntry nt_val;
    std::string tab_name = "persistent";
    T default_value;
    T last_value;
    // map of values
    persistent(std::string name, T value){
      default_value = value;
      last_value = value;
      nt_val = frc::Shuffleboard::GetTab(tab_name).AddPersistent(name, value).GetEntry();
    }
    T get() {
        return raw_get<T>(nt_val, default_value);
    }
    bool justUpdated() {
        T new_value = raw_get<T>(nt_val, default_value);
        bool is_updated = (new_value != last_value);
        last_value = new_value;
        return is_updated;
    }
};