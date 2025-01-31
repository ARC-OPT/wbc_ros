#ifndef WBC_ROS_CONTACTS_INTERFACE_HPP
#define WBC_ROS_CONTACTS_INTERFACE_HPP

#include <wbc/core/RobotModel.hpp>
#include <Eigen/Core>

namespace wbc_ros{
    class ContactsInterface{
        public:
            Eigen::VectorXd raw_data;
            wbc::RobotModelPtr robot_model;
            std::vector<wbc::Contact> contacts;
            std::vector<std::string> interface_names;
            ContactsInterface(wbc::RobotModelPtr model) :
                robot_model(model){
                contacts = model->getContacts();
                raw_data.resize(model->getContacts().size());
                for(uint i = 0; i < contacts.size(); i++){
                    raw_data[i] = (double)model->getContacts()[i].active;
                    interface_names.push_back(contacts[i].frame_id + "/contact/active");
                }
            }
            void update(){                    
                for(uint i = 0; i < contacts.size(); i++)
                    contacts[i].active = (int)raw_data[i];
                robot_model->setContacts(contacts);
            }
    };

}

#endif