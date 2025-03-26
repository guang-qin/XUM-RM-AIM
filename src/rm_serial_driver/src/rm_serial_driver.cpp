// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <auto_aim_interfaces/msg/detail/gimbal_cmd__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

// 决策依赖库
// #include "decision_moudle/msg/hp.hpp"
// #include "decision_moudle/msg/site.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
//设置参数，参数在yaml文件里面，不要动
  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  //tf变换，前者确定（？），后者确定广播坐标变换的类
  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  //visusalization_msgs::msg::Marker是用来表示可视化标记的消息类型，它通常用于在机器人仿真或视觉效果中显示特定的几何形状、文本或其他类型的图形元素。

  // site_pub = this->create_publisher<decision_moudle::msg::Site>("/incident", 10);
  // health_pub = this->create_publisher<decision_moudle::msg::Hp>("/allhealth", 10);
  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  //这行代码的主要目的是初始化一个rclcpp::AsyncParametersClient实例，以便RMSerialDriver类能够异步地获取和设置与"armor_detector"相关的参数。
  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
  //创建客户端，用于调用"/tracker/reset"服务
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    //开启串口
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
  //在rviz2中显示瞄准点·
  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription

  //创建订阅者，订阅话题"/solver/cmd_gimbal"
  //auto_aim_interfaces::msg::GimbalCmd存在于auto_aim_interfaces包下的msg中
  //其发布者位于armor_solver包下的armor_solver_node.cpp文件中
  gimbal_cmd_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCmd>(
    "/solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    //SensorDataQoS() 是一个服务质量（QoS）配置对象，用于指定订阅者的服务质量设置。
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
    //bind函数用于创建一个可调用对象，将函数或成员函数与部分参数绑定在一起。
    
    //这一段貌似重复了？
    // gimbal_cmd_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCmd>(
    //   "/solver/cmd_gimbal", rclcpp::SensorDataQoS());
    //   std::bind(&RMSerialDriver::sendData, , std::placeholders::_1);

//这一段是发布点移动，主要是为了服务导航，自瞄可以不看，从这里开始————AAAAA
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //Twist消息结构包括linear(线速度)和angular(角速度)两个方面
    //linear跟angular分别存在三个分量
    //linear的x，y，z是沿着坐标轴方向上的速度，angular是沿着x，y，z三个轴上的角速度
    "/cmd_vel", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
    //共享指针（？
      try{
        CmdVelPacket packet;
        packet.linear_x = msg->linear.x;
        packet.linear_y = msg->linear.y;
        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

        std::vector<uint8_t> data = cmd_toVector(packet);

        serial_driver_->port()->send(data);
        RCLCPP_INFO(get_logger(), "Send cmd_vel data: %f, %f, %d", packet.linear_x, packet.linear_y, packet.checksum);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending cmd_vel data: %s", ex.what());
        reopenPort();
      }
    });
    //到这里结束————AAAAA
}

//析构函数
RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    //receive_thread_.joinable()用于检查线程是否可连接，即判断该线程对象代表的线程是否已经启动且尚未结束。
    receive_thread_.join();
    //join()函数用于等待线程结束，即阻塞当前线程直到与之关联的线程执行完毕。
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  //owned_ctx_的内容在hpp文件里面有说明
  //owned_ctx_->waitForExit()用于等待当前线程上下文（IoContext）中的所有工作完成。
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;

  // 用于提取前11位数据的掩码
  // uint32_t mask = 0x7FF;
  // uint32_t extracted_bits;
  // uint16_t purchase_bullet;
  // uint8_t team = -1;

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A)        // 自瞄
      {
        data.resize(sizeof(ReceivePacket) - 1);
        //resize()函数用于调整容器的大小，使其能够容纳指定数量的元素。
        //-1是因为前面已经接收了一个字节的头信息，所以需要减去1
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        //insert()：这是 std::vector 类的一个成员函数，用于在指定位置插入一个或多个元素。
        //这行代码的含义是：在 data 向量的开头（即第一个元素之前）插入 header[0] 中的值。
        //前插之后data向量的大小会增加1，并且header[0]的值会成为data向量的第一个元素，不会数据溢出;
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          //crc16::Verify_CRC16_Check_Sum()函数用于验证给定数据的 CRC-16 校验和是否正确，实现在crc.cpp文件中,可以不用管
          //！！！但需要注意，若下位机没做CRC校验，则可能出现数据传输错误的很离谱的情况，这个时候传输给下位机的数据出现跳变是这个问题。
          //CRC错误与代码无关，若CRC经常出现问题，建议检查硬件。
        if (crc_ok) {
          RCLCPP_INFO(this->get_logger(),"CRC OK");

          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            //根据下位机传输的颜色数据来设置瞄准颜色，注意下位机传输颜色是否为裁判系统颜色，可能会在rqt中看到detect_color在0与1之间跳动（下位机的问题）。
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }
          //度数转换为弧度
          packet.roll = packet.roll * (M_PI / 180.0);
          packet.pitch = packet.pitch * (M_PI / 180.0);
          packet.yaw = packet.yaw * (M_PI / 180.0);  

          geometry_msgs::msg::TransformStamped t;
          //geometry_msgs是ROS中用于表示几何和变换消息的标准包
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          //timestamp_offset_是参数服务器中的一个参数，用于调整时间戳的偏移量。
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          //坐标变换，将全局坐标系转换为云台坐标系。（保留意见）
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);
          //坐标变换完并广播

          //  if (packet.detect_color == 1)
          //    team = 0;
          //   else
          //    team = 1;
          //收到瞄准点，则在rviz2中显示瞄准点。
          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } else {
          //可以在这里把注释去掉查看CRC错误频率。
          //RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      // } 
      // else if (header[0] == 0xA6)   // 导航决策
      // {
      // 	RCLCPP_INFO(get_logger(), "gggggggggggggggg");
      //   data.resize(sizeof(NavPacket) - 1);
      //   serial_driver_->port()->receive(data);
      //   data.insert(data.begin(), header[0]);
      //   NavPacket nav_packet = nav_fromVector(data);
      //   // CRC 校验
      //   bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t*>(&nav_packet), sizeof(nav_packet));
      //   if (true || crc_ok) 
      //   {
      //   // 数据包有效，处理数据
      //     // Hp.msg
      //     hp_msg.red_1_robot_hp = nav_packet.game_robot_HP_t.red_1_robot_HP;
      //     hp_msg.red_3_robot_hp = nav_packet.game_robot_HP_t.red_3_robot_HP;
      //     hp_msg.red_4_robot_hp = nav_packet.game_robot_HP_t.red_4_robot_HP;
      //     hp_msg.red_5_robot_hp = nav_packet.game_robot_HP_t.red_5_robot_HP;
      //     hp_msg.red_7_robot_hp = nav_packet.game_robot_HP_t.red_7_robot_HP;
      //     hp_msg.blue_1_robot_hp = nav_packet.game_robot_HP_t.blue_1_robot_HP;
      //     hp_msg.blue_3_robot_hp = nav_packet.game_robot_HP_t.blue_3_robot_HP;
      //     hp_msg.blue_4_robot_hp = nav_packet.game_robot_HP_t.blue_4_robot_HP;
      //     hp_msg.blue_5_robot_hp = nav_packet.game_robot_HP_t.blue_5_robot_HP;
      //     hp_msg.blue_7_robot_hp = nav_packet.game_robot_HP_t.blue_7_robot_HP;
      //     hp_msg.rest_bullet = nav_packet.projectile_allowance_t;
      //     // 使用掩码提取第0到10位数据
      //     extracted_bits = nav_packet.sentry_info & mask;
      //     // 将提取到的数据转换为uint16_t类型
      //     purchase_bullet = static_cast<uint16_t>(extracted_bits);
      //     hp_msg.purchase_bullet = 350 - purchase_bullet; // 350发可购买数-已经购买数
      //     health_pub->publish(hp_msg);
      //     // Site.msg
      //     uint8_t bit[12] = {0}; 
      //     for(int i = 0; i <=11; i++)
      //     {
      //        bit[i] = (nav_packet.event_data_t.event_data >> i) & 1;
      //     }
      //     if (bit[3] || bit[4] || bit[5])
      //       site_msg.energy = 1;
      //     else
      //       site_msg.energy = 0;
      //     if (bit[6] == 1 && bit[7] == 0)
      //       site_msg.r2 = 1;
      //     else if (bit[6] == 0 && bit[7] == 1)
      //       site_msg.r2 = 2;
      //     else
      //       site_msg.r2 = 0;
      //     if (bit[8] == 1 && bit[9] == 0)
      //       site_msg.r3 = 1;
      //     else if (bit[8] == 0 && bit[9] == 1)
      //       site_msg.r3 = 2;
      //     else
      //       site_msg.r3 = 0;
      //     if (bit[10] == 1 && bit[11] == 0)
      //       site_msg.r4 = 1;
      //     else if (bit[10] == 0 && bit[11] == 1)
      //       site_msg.r4 = 2;
      //     else
      //       site_msg.r4 = 0;
      //     site_msg.resttime = nav_packet.game_status_t.stage_remain_time;
      //     site_msg.blue_outpost_hp = nav_packet.game_robot_HP_t.blue_outpost_HP;
      //     site_msg.blue_base_hp = nav_packet.game_robot_HP_t.blue_base_HP;
      //     site_msg.red_outpost_hp = nav_packet.game_robot_HP_t.red_outpost_HP;
      //     site_msg.red_base_hp = nav_packet.game_robot_HP_t.red_base_HP;
      //     site_msg.team = team;
      //     site_pub->publish(site_msg);
      //   } 

        // else 
        // {
        //   RCLCPP_ERROR(get_logger(), "CRC error!");
        // }
      }
      
      else {
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      //捕获异常，重新打开串口。
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::GimbalCmd::SharedPtr msg)//云台消息包，定义内容在auto_aim_interfaces中定义
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    // 定义了一个名为id_unit8_map的静态常量映射，它将字符串键（例如"1"、"2"、"3"等）映射到uint8_t类型的值。
    //rqt上看到显示数字的地方
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    //这一整段都是数据传输，注释掉的数据为之前旧代码所发送的。

    // packet.tracking = msg->tracking;
    // packet.id = id_unit8_map.at(msg->id);
    // packet.armors_num = msg->armors_num;
    // packet.x = msg->position.x;
    // packet.y = msg->position.y;
    // packet.z = msg->position.z;
    // packet.yaw = msg->yaw;
    // packet.vx = msg->velocity.x;
    // packet.vy = msg->velocity.y;
    // packet.vz = msg->velocity.z;
    // packet.v_yaw = msg->v_yaw;
    // packet.r1 = msg->radius_1;
    // packet.r2 = msg->radius_2;
    // packet.dz = msg->dz;

    //以下为新代码所发送的数据
    packet.pitch = msg->pitch;
    packet.pitch_diff = msg->pitch_diff;
    packet.yaw = msg->yaw;
    packet.yaw_diff = msg->yaw_diff;
    packet.distance = msg->distance;
    packet.fire_advice = msg->fire_advice;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    //reinterpret_cast 是 C++ 中的一种强制类型转换运算符，用于在不同类型的指针、引用或者指针和整数类型之间进行转换。
    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);
    std_msgs::msg::Float64 latency;
    //获取延迟时间
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    //this->now() 获取当前时间，msg->header.stamp 是接收到的消息的时间戳。两者相减得到延迟（单位为秒），然后乘以1000转换为毫秒。
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    //std::exception：这是 C++ 标准库中所有标准异常类的基类。当发生异常时，它会捕获到这个基类的对象。
    RCLCPP_ERROR(get_logger(), "Error while sending auto_aim data: %s", ex.what());
    reopenPort();
    //重新打开串口，尝试恢复通信。
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
  } }catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc,
       pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      //rclcpp::sleep_for是一个函数，它接受一个时间间隔作为参数，并导致调用线程在该时间段内暂停执行。
      //等1秒重开串口
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    //reset_tracker_client_这是一个指向服务客户端的指针;
    //service_is_ready()：这是服务客户端类的一个成员函数，用于检查服务是否已准备好被调用。
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  //创建一个共享指针request，指向Trigger::Request类型的对象。
  //trigger::Request是一个服务请求消息，通常用于触发某个动作或事件。
  reset_tracker_client_->async_send_request(request);
  //发送异步请求，重置跟踪器。
  RCLCPP_INFO(get_logger(), "Reset tracker!");
  //由于这是一个异步操作，程序可以继续执行而不会等待服务响应。
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
