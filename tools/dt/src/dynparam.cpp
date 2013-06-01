#include <ros/ros.h>

#include <dynparam/stringparam.h>
#include <dynparam/intparam.h>
#include <dynparam/doubleparam.h>
#include <dynparam/boolparam.h>

#include <iostream>

BoolParam bparam;
IntParam iparam;
DoubleParam dparam;
StringParam sparam;

#define SET_ARGC 3
#define MIN_ARGC 2

int main(int argc, char** argv) {
	if (argc < MIN_ARGC || argc > SET_ARGC) {
		std::cerr << "Usage: rosrun dynparam_tools " << argv[0] << " <type> <name> [value]" << std::endl;
		return 1;
	}

	ros::init(argc, argv, "dynparam_tool");
	ros::NodeHandle nh("~");

	std::string name(argv[1]);

    if (!nh.hasParam(name)) {
		std::cerr << "Invalid parameter name." << std::endl;
		std::cerr << name << " does not exist in the parameter server." << std::endl;
		std::cerr << "Check the name and that the parameter has been set at least once." << std::endl;

		return 4;
	}

    std::string type("");

    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);

    ros::master::V_TopicInfo::iterator tIter = topicList.begin();
    for (; tIter != topicList.end(); ++tIter) {
        if (tIter->name == name) {
            if (tIter->datatype == "std_msgs/Bool") {
                type = "bool";
            } else if (tIter->datatype == "std_msgs/Int32") {
                type = "int";
            } else if (tIter->datatype == "std_msgs/Float64") {
                type = "double";
            } else if (tIter->datatype == "std_msgs/String") {
                type = "string";
            } else {
                std::cerr << "Unknown value type specified." << std::endl;
                std::cerr << "Acceptable values are:" << std::endl;
                std::cerr << "\tbool, int, double, string" << std::endl;

                return 3;
            }

            break;
        }
    }

    if (type == "") {
        std::cerr << "Unknown error determining parameter type." << std::endl;
        std::cerr << "The parameter server has the parameter but the topic was not found." << std::endl;
        return 6;
    }

	if (type == "bool") {
		bparam.init(&nh, name, false);
	} else if (type == "int") {
		iparam.init(&nh, name, 0);
	} else if (type == "double") {
		dparam.init(&nh, name, 0.0);
	} else if (type == "string") {
		sparam.init(&nh, name, "");
    }

	if (argc == SET_ARGC) {
		std::string value(argv[2]);
		std::cout << "Attempting to set parameter with name " << name << " to value " << value << std::endl;

		if (type == "bool") {
			bool val;
			if (value == "0" || value == "false" || value == "False" || value == "f" || value == "F") {
				val = false;
			} else if (value == "1" || value == "True" || value == "true" || value == "t" || value == "T") {
				val = true;
			} else {
				std::cerr << "Unknown bool type value specified." << std::endl;
				std::cerr << "Acceptable values are:" << std::endl;
				std::cerr << "\t0, false, False, f, F" << std::endl;
				std::cerr << "\t1, true, True, t, T" << std::endl;

				return 2;
			}

			bparam.set(val);
		} else if (type == "int") {
			int val = atoi(value.c_str());
			iparam.set(val);
		} else if (type == "double") {
			double val = atof(value.c_str());
			dparam.set(val);
		} else if (type == "string") {
			sparam.set(value);
		}
	}

	std::cout << "Reading current value of " << name << std::endl;

	if (type == "bool") {
		std::cout << "bool param " << name << " has value:\n" << bparam.get() << std::endl;
	} else if (type == "int") {
		std::cout << "int param " << name << " has value:\n" << iparam.get() << std::endl;
	} else if (type == "double") {
		std::cout << "double param " << name << " has value:\n" << dparam.get() << std::endl;
	} else if (type == "string") {
		std::cout << "string param " << name << " has value:\n" << sparam.get() << std::endl;
	}

	ros::spinOnce();
	sleep(1);
	ros::spinOnce();

	return 0;
}
