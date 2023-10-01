#ifndef _GZ_POSE_PLUGIN_HH
#define _GZ_POSE_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/ModelState.hh>
#include <stdio.h>
#include <ignition/math/Vector3.hh>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include "Messages_GRVCopter.h"
#include <thread>

#define PORT  15005+1

#define num_motores 6


namespace gazebo {
/// \brief A plugin to control a Velodyne sensor.
  


  class gz_pose_plugin : public ModelPlugin
  {
    public: void receive(){
      while(true){
        char buffer[MSG_GRVCOPTER::MSG_SIZE];
          int recibido = recv(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0);

          MSG_GRVCOPTER::Message_Bytes msg_rec;
          if (recibido >= (MSG_GRVCOPTER::MSG_SIZE-1)){
            MSG_GRVCOPTER::unpack_message(buffer, &msg_rec);
            if(msg_rec.MSG_ID.value == MSG_GRVCOPTER::PWM_MSG_ID){
              for (int i = 0; i < num_motores; i++){
                //std::cout << "PWMS MEssgae" << std::endl;
                pwm_state[i] = (int)msg_rec.DATA[i+1].value;  
              }
            }
          }        
      }
    }


    private: physics::ModelPtr model = nullptr;
    private: int sock;
    private: struct sockaddr_in cliaddr;
    private: event::ConnectionPtr updateConnection;

    private: int pwm_state[12];
    private: std::thread hilo_recepcion = std::thread(&gz_pose_plugin::receive, this);

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    private: transport::SubscriberPtr sub2;

    //private: float z_uav = 0.0;

    private: float rc_state[14] {0.0};

    private: ignition::math::Vector3<double> forces = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
    private: ignition::math::Vector3<double> forces_ref = ignition::math::Vector3<double>(0.0, 0.0, 0.0);


    /// \brief Constructor
    public: gz_pose_plugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe simulator plugin is attached to model[" <<
        _model->GetName() << "]\n";

      this->model = _model;

        // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(_model->GetWorld()->GetName());
      #else
      this->node->Init(_model->GetWorld()->Name());
      #endif

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&gz_pose_plugin::OnUpdate, this));
      
      //Create socket
      struct sockaddr_in servaddr;
      sock = socket(AF_INET, SOCK_DGRAM, 0);
      memset(&servaddr, 0, sizeof(servaddr));
      servaddr.sin_family = AF_INET;
      servaddr.sin_addr.s_addr = INADDR_ANY;
      servaddr.sin_port = htons(5500);
      if(bind(sock, (const struct sockaddr*)&servaddr, sizeof(servaddr))){
        std::cout << "Error con bind" << std::endl;
      }
      
      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 10000;
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

      std::string ip_client;
      if (_sdf->HasElement("ipclient")){
        ip_client = _sdf->GetElement("ipclient")->Get<std::string>();


      } else {
        ip_client = "127.0.0.1";
      }
      std::cerr << "\nThe pose plugin is sending to ip [" << ip_client.c_str() << "]\n";
      memset(&servaddr, 0, sizeof(cliaddr));
      cliaddr.sin_family = AF_INET;
      cliaddr.sin_addr.s_addr = inet_addr(ip_client.c_str());
      cliaddr.sin_port = htons(PORT-1);

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe("/gazebo/default/pose/info", &gz_pose_plugin::OnMsg, this);

      // Subscribe to the topic, and register a callback
      this->sub2 = this->node->Subscribe("/gazebo/startWrench", &gz_pose_plugin::OnMsg2, this);

      //All channel at 1500 except for throttle, flight mode and arm.
      for(int i = 0; i < 14; i++){
        rc_state[i] = 0.5;
      }
      rc_state[2] = 0.0;
      rc_state[4] = 0.0;
      rc_state[6] = 0.0;
    }

private: void pwm_to_force(int* _pwm, float* _force){
  for(int i = 0; i < num_motores; i++){
    //Meter la ecuacion de la fuerza segun pwm.
    _force[i] = 7.059*1e-6*_pwm[i]*_pwm[i] - 0.01*_pwm[i] + 2.5433;
  }
}

private: void force_motors_2_to_force_uav(float* _force_motor, float* _force_uav, float* _torque_uav){
  float roll_factor[] = {0.1372, 0.2151, 0.0779, -0.1372, -0.2151, -0.0779};
  float pitch_factor[] = {0.1692, -0.0342, -0.2034, -0.1692, 0.0342, 0.2034};
  float yaw_factor[] = {-0.1783, 0.1783, -0.1783, 0.1783, -0.1783, 0.1783};
  float x_factor[] = {0.1355, 0.3214, -0.4569, 0.1355, 0.3214, -0.4569};
  float y_factor[] = {-0.4493, 0.3420, 0.1073, -0.4493, 0.3420, 0.1073};
  float z_factor[] = {0.8832, 0.8832, 0.8832, 0.8832, 0.8832, 0.8832};

  for (int i=0; i<num_motores; i++){
    _torque_uav[0] += roll_factor[i]*_force_motor[i];
    _torque_uav[1] -= pitch_factor[i]*_force_motor[i];
    _torque_uav[2] -= yaw_factor[i]*_force_motor[i];
    _force_uav[0] += x_factor[i]*_force_motor[i];
    _force_uav[1] -= y_factor[i]*_force_motor[i];
    _force_uav[2] += z_factor[i]*_force_motor[i];
  }

  //Rotate forces to uav frame


}

//Aqui deberia coger los PWMs y calcular las fuerzas resultantes y aplicarla en el solido.
public: void OnUpdate(){ 
  /*char buffer[MSG_GRVCOPTER::MSG_SIZE];
  int recibido = recv(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0);

  MSG_GRVCOPTER::Message_Bytes msg_rec;
  if (recibido >= (MSG_GRVCOPTER::MSG_SIZE-1)){
    MSG_GRVCOPTER::unpack_message(buffer, &msg_rec);
    if(msg_rec.MSG_ID.value == MSG_GRVCOPTER::PWM_MSG_ID){
      for (int i = 0; i < num_motores; i++){
        pwm_state[i] = (int)msg_rec.DATA[i+1].value;
      }
    }
  }  */

  float forces_motors[num_motores] {0.0};
  pwm_to_force(pwm_state, forces_motors);
  float force_uav[3] {0.0};
  float torque_uav[3] {0.0};
  force_motors_2_to_force_uav(forces_motors, force_uav, torque_uav);

  //ignition::math::Vector3<double> offset = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
  ignition::math::Vector3<double> force_v3 = ignition::math::Vector3<double>(force_uav[0], force_uav[1], force_uav[2]);
  this->model->GetLinks()[0]->AddRelativeForce(force_v3);
  ignition::math::Vector3<double> torque_v3 = ignition::math::Vector3<double>(torque_uav[0], torque_uav[1], torque_uav[2]);
  this->model->GetLinks()[0]->AddRelativeTorque(torque_v3);

}

//Function to transform from quaternion to euler 
private: void quat_2_euler(msgs::Quaternion quat, float*  euler){
    double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
    double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
    euler[0] = (float)(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (quat.w() * quat.y() - quat.x() * quat.z()));
    double cosp = std::sqrt(1 - 2 * (quat.w() * quat.y() - quat.x() * quat.z()));
    euler[1] = -(float)(2 * std::atan2(sinp, cosp) - M_PI / 2);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
    double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
    euler[2] = -(float)(std::atan2(siny_cosp, cosy_cosp));
}

private: void OnMsg2(ConstVector3dPtr &msg){
  rc_state[((int)msg->x())-1] = msg->y();
  std::cerr << "\nChannel " << (int)msg->x() <<": " << msg->y();
}



//Aqui se recibe el mensaje del estado del UAV, por lo tanto habra que transformarlo para enviarlo a GRVCOPTER
private:
void OnMsg(ConstPosesStampedPtr &msg)
{

  msgs::PoseStamped pose_temp;
  float robot_pose[8];
  float id = 0;
  for (int ind = 0; ind < msg->pose_size(); ind++)
  {
    if (msg->pose(ind).name() == this->model->GetName())
    {

      // Get orientation and send to grvcopter:
      msgs::Quaternion quat = msg->pose(ind).orientation();
      float attitude_euler[3]{0.0};
      quat_2_euler(quat, attitude_euler);

      MSG_GRVCOPTER::Message_Bytes msgGrvcopter;
      MSG_GRVCOPTER::pack_att_message(attitude_euler, &msgGrvcopter);
      char msg_bytes[MSG_GRVCOPTER::MSG_SIZE] {0};
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      // End of send attitude.

      //Get position and send to grvcopter:
      msgs::Vector3d position = msg->pose(ind).position();
      float position_uav[3] {0.0};
      position_uav[0] = position.x()*100.0;
      position_uav[1] = -position.y()*100.0;
      position_uav[2] = position.z()*100.0;
      MSG_GRVCOPTER::pack_pos_message(position_uav, &msgGrvcopter);
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      // End of send position.

      //Send linear velocity
      ignition::math::Vector3d linearVel = this->model->RelativeLinearVel();
      float velocity_uav[3] {0.0};
      velocity_uav[0] = linearVel[0]*100.0;
      velocity_uav[1] = -linearVel[1]*100.0;
      velocity_uav[2] = linearVel[2]*100.0;
      MSG_GRVCOPTER::pack_vel_message(velocity_uav, &msgGrvcopter);
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      //End of send linear velocity.

      //Send angular velocity
      ignition::math::Vector3d angular_vel = this->model->RelativeAngularVel();
      float rate_uav[3] {0.0};
      rate_uav[0] = angular_vel[0];
      rate_uav[1] = -angular_vel[1];
      rate_uav[2] = -angular_vel[2];
      MSG_GRVCOPTER::pack_rate_message(rate_uav, &msgGrvcopter);
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      //End of send angular velocity.

      //Send RC values.
      MSG_GRVCOPTER::pack_rc_message(rc_state, &msgGrvcopter);
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      //End of RC send.

      //Send pos_target.
      float pos_target_msg[MSG_GRVCOPTER::MSG_DATA_LENGTH] = {0.0};
      pos_target_msg[0] = 0.0;
      pos_target_msg[1] = 0.0;
      pos_target_msg[2] = -1.0;
      MSG_GRVCOPTER::pack_generic_message(MSG_GRVCOPTER::TARGET_POS_MSG_ID, pos_target_msg, &msgGrvcopter);
      MSG_GRVCOPTER::get_bytes_msg(&msgGrvcopter, msg_bytes);
      sendto(sock, msg_bytes, MSG_GRVCOPTER::MSG_SIZE, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
      //End of pos_target send.

    }
  }
}
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(gz_pose_plugin)


}
#endif
