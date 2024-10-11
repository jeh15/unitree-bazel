#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;

const int G1_NUM_MOTOR = 29;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};

struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

enum MotorType { GearboxS = 0, GearboxM = 1, GearboxL = 2 };

std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    // clang-format off
    // legs
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    // waist
    GearboxM, GearboxS, GearboxS,
    // arms
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS,
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS
    // clang-format on
};

enum PRorAB { PR = 0, AB = 1 };

enum G1JointValidIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

float GetMotorKp(MotorType type) {
  switch (type) {
    case GearboxS:
      return 40;
    case GearboxM:
      return 40;
    case GearboxL:
      return 100;
    default:
      return 0;
  }
}

float GetMotorKd(MotorType type) {
  switch (type) {
    case GearboxS:
      return 1;
    case GearboxM:
      return 1;
    case GearboxL:
      return 1;
    default:
      return 0;
  }
}

class G1Example {
 private:
  double time_;
  double control_dt_;  // [2ms]
  double duration_;    // [3 s]
  PRorAB mode_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

 public:
  G1Example(std::string networkInterface)
      : time_(0.0), control_dt_(0.002), duration_(3.0), mode_(PR) {
    ChannelFactory::Instance()->Init(0, networkInterface);

    // create publisher
    lowcmd_publisher_.reset(
        new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    // create subscriber
    lowstate_subscriber_.reset(
        new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);

    // create threads
    command_writer_ptr_ =
        CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000,
                                &G1Example::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx(
        "control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }

  void ReportRPY() {
    const std::shared_ptr<const ImuState> imu_tmp_ptr =
        imu_state_buffer_.GetData();
    if (imu_tmp_ptr) {
      std::cout << "rpy: [" << imu_tmp_ptr->rpy.at(0) << ", "
                << imu_tmp_ptr->rpy.at(1) << ", " << imu_tmp_ptr->rpy.at(2)
                << "]" << std::endl;
    }
  }

  void LowStateHandler(const void *message) {
    unitree_hg::msg::dds_::LowState_ low_state =
        *(const unitree_hg::msg::dds_::LowState_ *)message;

    if (low_state.crc() !=
        Crc32Core((uint32_t *)&low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();

      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code "
                  << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // check mode machine
    const uint8_t desired_mode_machine = 1;
    if (low_state.mode_machine() != desired_mode_machine)
      std::cout << "[ERROR] mode_machine: "
                << unsigned(low_state.mode_machine()) << "\n";
  }

  void LowCommandWriter() {
    unitree_hg::msg::dds_::LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = mode_;   // {0:PR, 1:AB}
    dds_low_command.mode_machine() = 1;  // {1:23dof, 2:29dof, 3:27dof, 9:14dof}

    const std::shared_ptr<const MotorCommand> mc =
        motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command,
                                        (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
  }

  void Control() {
    ReportRPY();

    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);

          double q_des = 0;
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.q_target.at(i) =
              (q_des - ms->q.at(i)) * ratio + ms->q.at(i);
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }
      } else if (time_ < duration_ * 2) {
        // [Stage 2]: swing ankle's PR
        mode_ = PR;
        double max_P = M_PI * 30.0 / 180.0;
        double max_R = M_PI * 10.0 / 180.0;
        double t = time_ - duration_ * 1;
        double L_P_des = +max_P * std::sin(2.0 * M_PI * t);
        double L_R_des = +max_R * std::sin(2.0 * M_PI * t);
        double R_P_des = +max_P * std::sin(2.0 * M_PI * t);
        double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.q_target.at(i) = 0.0;
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }

        motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
        motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
        motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
        motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
      } else {
        // [Stage 3]: swing ankle's AB
        mode_ = AB;
        double max_A = M_PI * 30.0 / 180.0;
        double max_B = M_PI * 10.0 / 180.0;
        double t = time_ - duration_ * 2;
        double L_A_des = +max_A * std::sin(M_PI * t);
        double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
        double R_A_des = -max_A * std::sin(M_PI * t);
        double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.q_target.at(i) = 0.0;
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }

        motor_command_tmp.q_target.at(LeftAnkleA) = L_A_des;
        motor_command_tmp.q_target.at(LeftAnkleB) = L_B_des;
        motor_command_tmp.q_target.at(RightAnkleA) = R_A_des;
        motor_command_tmp.q_target.at(RightAnkleB) = R_B_des;
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_XXdof_example network_interface_name" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);

  while (true) sleep(10);

  return 0;
}