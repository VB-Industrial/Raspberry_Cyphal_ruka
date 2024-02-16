

#include "ruka_hardware.hpp"
#include "ruka_sensor_hardware.hpp"
#include "ruka_joints.h"

#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iomanip> 

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


#include "cyphal/allocators/o1/o1_allocator.h"
#include "cyphal/cyphal.h"
#include "cyphal/providers/LinuxCAN.h"
#include "cyphal/subscriptions/subscription.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/Twist_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/State_0_1.h"


TYPE_ALIAS(Twist, reg_udral_physics_kinematics_cartesian_Twist_0_1)
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(State, reg_udral_physics_kinematics_cartesian_State_0_1)

std::byte buffer[sizeof(CyphalInterface) + sizeof(LinuxCAN) + sizeof(O1Allocator)];

void error_handler() {std::cout << "error" << std::endl; std::exit(EXIT_FAILURE);}
uint64_t micros_64() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
UtilityConfig utilities(micros_64, error_handler);

std::shared_ptr<CyphalInterface> cy_interface;

uint32_t uptime = 0;
void heartbeat() {
    static uint8_t hbeat_buffer[HBeat::buffer_size];
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    cy_interface->send_msg<HBeat>(&heartbeat_msg, hbeat_buffer, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
    uptime += 1;
}

float j_pos_0 = 0.0;
float j_pos_1 = 0.0;
float j_pos_2 = 0.0;



class JSReader_01: public AbstractSubscription<JS_msg> {
public:
    JSReader_01(InterfacePtr interface): AbstractSubscription<JS_msg>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        AGENT_JS_SUB_PORT
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_read, CanardRxTransfer* transfer) override {
        std::cout << "Node id: " << +transfer->metadata.remote_node_id << std::endl;
        std::cout << "pos: " << js_read.angular_position.radian << std::endl;
        std::cout << "vel: " << js_read.angular_velocity.radian_per_second << std::endl;
        std::cout << "eff: " << js_read.angular_acceleration.radian_per_second_per_second << std::endl;
        j_pos_0 = js_read.angular_position.radian;
        j_pos_1 = js_read.angular_velocity.radian_per_second;
        j_pos_2 = js_read.angular_acceleration.radian_per_second_per_second;
    }
};
JSReader_01 * JS_reader_01;


// class IMUReader: public AbstractSubscription<Twist> {
// public:
//     IMUReader(InterfacePtr interface): AbstractSubscription<Twist>(interface,
//         // Тут параметры - port_id, transfer kind или только port_id
//         AGENT_IMU_PORT
//     ) {};
//     void handler(const reg_udral_physics_kinematics_cartesian_Twist_0_1& IMU_read, CanardRxTransfer* transfer) override {
//         // std::cout << "Node id: " << +transfer->metadata.remote_node_id << std::endl;
//         // std::cout << "av1: " << IMU_read.linear.meter_per_second[0] << std::endl;
//         // std::cout << "av2: " << IMU_read.linear.meter_per_second[1] << std::endl;
//         // std::cout << "av3: " << IMU_read.linear.meter_per_second[2]<< std::endl;
//         // std::cout << "aa1: " << IMU_read.angular.radian_per_second[0] << std::endl;
//         // std::cout << "aa2: " << IMU_read.angular.radian_per_second[1] << std::endl;
//         // std::cout << "aa3: " << IMU_read.angular.radian_per_second[2]<< std::endl;

//         av1 = IMU_read.linear.meter_per_second[0];
//         av2 = IMU_read.linear.meter_per_second[1];
//         av3 = IMU_read.linear.meter_per_second[2];
//         aa1 = IMU_read.angular.radian_per_second[0];
//         aa2 = IMU_read.angular.radian_per_second[1];
//         aa3 = IMU_read.angular.radian_per_second[2];
//     }
// };
// IMUReader * IMU_reader;


float qw = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float avx = 0.0;
float avy = 0.0;
float avz = 0.0;
float lax = 0.0;
float lay = 0.0;
float laz = 0.0;

class IMUReader: public AbstractSubscription<State> {
public:
    IMUReader(InterfacePtr interface): AbstractSubscription<State>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        AGENT_IMU_PORT
    ) {};
    void handler(const reg_udral_physics_kinematics_cartesian_State_0_1& IMU_read, CanardRxTransfer* transfer) override {
        //std::cout << "Node id: " << +transfer->metadata.remote_node_id << std::endl;
        qw = IMU_read.pose.orientation.wxyz[0];
        qx = IMU_read.pose.orientation.wxyz[1];
        qy = IMU_read.pose.orientation.wxyz[2];
        qz = IMU_read.pose.orientation.wxyz[3];

        lax = IMU_read.twist.linear.meter_per_second[0];
        lay = IMU_read.twist.linear.meter_per_second[1];
        laz = IMU_read.twist.linear.meter_per_second[2];

        avx = IMU_read.twist.angular.radian_per_second[0];
        avy = IMU_read.twist.angular.radian_per_second[1];
        avz = IMU_read.twist.angular.radian_per_second[2];

    }
};
IMUReader * IMU_reader;

class HBeatReader: public AbstractSubscription<HBeat> {
public:
    HBeatReader(InterfacePtr interface): AbstractSubscription<HBeat>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const uavcan_node_Heartbeat_1_0& hbeat, CanardRxTransfer* transfer) override {
        std::cout << +transfer->metadata.remote_node_id << ": " << hbeat.uptime <<  std::endl;
    }
};
HBeatReader * reader;


namespace ruka
{

hardware_interface::CallbackReturn RukaSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  //hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);
  // // // END: This part here is for exemplary purposes - Please do not copy to your production code 

  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  std::cout<<"info_.sensors[0].state_interfaces.size() "<<info_.sensors[0].state_interfaces.size()<<std::endl;
  std::cout<<"info_.sensors[0].name "<<info_.sensors[0].name<<std::endl;

    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
       std::cout<<"info_.sensors[0].state_interfaces[i].name "<<info_.sensors[0].state_interfaces[i].name<<std::endl;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RukaSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn RukaSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RukaSensor"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Successfully activated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RukaSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RukaSensor"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RukaSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;

    hw_sensor_states_[0] = qx;  //orientation_x_
    hw_sensor_states_[1] = qy;  //orientation_y_
    hw_sensor_states_[2] = qz;  //orientation_z_ 
    hw_sensor_states_[3] = qw;  //orientation_w_

    hw_sensor_states_[4] = avx;  //angular_velocity_x_
    hw_sensor_states_[5] = avy;  //angular_velocity_y_
    hw_sensor_states_[6] = avz;  //angular_velocity_z_

    hw_sensor_states_[7] = lax;  //linear_acceleration_x_
    hw_sensor_states_[8] = lay;  //linear_acceleration_y_
    hw_sensor_states_[9] = laz;  //linear_acceleration_z_

      // RCLCPP_INFO(
      // rclcpp::get_logger("RukaSensor"), "Got state %f for sensor %u!",
      // hw_sensor_states_[i], i);
  }
  return hardware_interface::return_type::OK;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn RukaSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  cy_interface = CyphalInterface::create_heap<LinuxCAN, O1Allocator>(100, "can0", 1000, utilities);
  reader = new HBeatReader(cy_interface);
  JS_reader_01 = new JSReader_01(cy_interface);
  IMU_reader = new IMUReader(cy_interface);

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  // force sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> RukaSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RukaSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type RukaSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber

  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  joint_position_[0] = j_pos_0;
  joint_position_[1] = j_pos_1;
  joint_position_[2] = j_pos_2;
  joint_position_[3] = -j_pos_0;
  joint_position_[4] = -j_pos_1;
  joint_position_[5] = -j_pos_2;
  cy_interface->loop();
  return return_type::OK;
}

int itera = 0;
return_type RukaSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{

 // for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
    // Simulate sending commands to the hardware
 //   printf("Got command %.5f for joint %li \n", joint_position_[i], i);

  //}
 if (itera > 100)
 {
    heartbeat();
    std::cout<<"HB sent"<<std::endl;
    itera = 0;
  }
  cy_interface->loop();
  itera++;
  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  ruka::RukaSystem, hardware_interface::SystemInterface)

PLUGINLIB_EXPORT_CLASS(
  ruka::RukaSensor,
  hardware_interface::SensorInterface)
