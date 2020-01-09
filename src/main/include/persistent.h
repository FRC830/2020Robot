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
template<typename T>
void raw_set(nt::NetworkTableEntry nt_val, T value) {}
template<>
void raw_set(nt::NetworkTableEntry nt_val, double value) {
  nt_val.SetDouble(value);
}
void raw_set(nt::NetworkTableEntry nt_val, int value) {
  nt_val.SetDouble(value);
}
void raw_set(nt::NetworkTableEntry nt_val, std::string value) {
  nt_val.SetString(value);
}
void raw_set(nt::NetworkTableEntry nt_val, bool value) {
  nt_val.SetBoolean(value);
}
template <typename T>
class persistent {
  public:

    nt::NetworkTableEntry nt_val;
    std::string tab_name = "persistent";
    T default_value;
    T last_value;

    persistent(std::string name, T value){
      default_value = value;
      last_value = value;
      nt_val = frc::Shuffleboard::GetTab(tab_name).AddPersistent(name, value).GetEntry();
    }
    persistent(std::string name, T value, frc::BuiltInWidgets widget) {
      default_value = value;
      last_value = value;
      nt_val = frc::Shuffleboard::GetTab(tab_name).AddPersistent(name, value).WithWidget(widget).GetEntry();
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
    void set(T val) {
      return raw_set<T>(nt_val, val);
    }
};