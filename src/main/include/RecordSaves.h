#include <fstream>
#include <string>
#include <vector>
#include <sstream>

class RecordSaves {
public:
    std::vector<double> leftLeadMotorValues;
    std::vector<double> rightLeadMotorValues;
    std::vector<double> leftFollowMotorValues;
    std::vector<double> rightFollowMotorValues;

    RecordSaves(std::string path = ""){
        if(path != ""){
            std::ifstream values;

            values.open(path);

            for (std::vector<double>* vect : {
                    &leftLeadMotorValues,
                    &rightLeadMotorValues,
                    &leftFollowMotorValues,       
                    &rightFollowMotorValues,
                }) {
                std::string input_str;
                std::getline(values, input_str);
                std::stringstream ss(input_str);
                double i;
                while (ss >> i)
                {
                    vect->push_back(i);
                    if (ss.peek() == ',')
                        ss.ignore();
                }
            }
        }

        if (leftFollowMotorValues.empty() || rightFollowMotorValues.empty() || leftFollowMotorValues.empty() || rightFollowMotorValues.empty())
        {
            std::cout << "\n Vecotrs Empty " << path << std::endl;
        }
    }
    void outputToFile(std::string filename) {
        std::ofstream values;
        // "/home/lvuser/vectors.txt"
        values.open(filename);
        std::vector<std::vector<double>> input {
            leftLeadMotorValues,
            rightLeadMotorValues,
            leftFollowMotorValues,       
            rightFollowMotorValues,
        };
        
        for (size_t j = 0; j < input.size(); j++) {
            for (size_t i = 0; i < input.at(j).size(); i++) {
                values << input.at(j).at(i) << ',';
            }
            values << "\n";
        }
        values.close();
    }
    void setToIndex(int Vindex, std::vector<rev::CANSparkMax*> motor){
        std::cout << motor.size() << "motor vector size" << std::endl;
        
        motor.at(0)->GetPIDController().SetReference(leftLeadMotorValues.at(Vindex), rev::ControlType::kPosition);
		motor.at(1)->GetPIDController().SetReference(leftFollowMotorValues.at(Vindex), rev::ControlType::kPosition);
		motor.at(2)->GetPIDController().SetReference(rightLeadMotorValues.at(Vindex), rev::ControlType::kPosition);
		motor.at(3)->GetPIDController().SetReference(rightFollowMotorValues.at(Vindex), rev::ControlType::kPosition);
        std::cout <<  "motors moved" << std::endl;
    }
    



};