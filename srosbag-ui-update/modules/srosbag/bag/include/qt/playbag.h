#ifndef PLAYBAG_H
#define PLAYBAG_H

#include <QMainWindow>
#include "dialog.h"
#include "ui_playbag.h"
#include <mutex>
//#include "core/msg/geometry_msg.hpp"
//#include "core/msg/std_msg.hpp"
#include "modules/srosbag/message/geometry_msg.hpp"
#include "modules/srosbag/message/std_msg.hpp"

#include"../../../../core/pose.h"
typedef struct
{
  int update_times = 0;
  double range_max;
  double first_angle_min;
  double first_angle_max;
  double second_angle_min;
  double second_angle_max;
  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;

  std::vector<float> ranges;
  std::vector<float> intensities;
  std::string sensor_name;

  std::string topic_;
  int64_t time_;

} LaserParam;

typedef struct
{
  //TODO:具体数据还需要确定后再填入，接口先写好
  int width_, height_;
  std::string topic_;

} ImageParam;

typedef struct
{
  std::string topic_;
  std_msgs::Headers header;
  geometry_msgss::Quaternion orientation;
  geometry_msgss::Vector3 angular_velocity;
  geometry_msgss::Vector3 linear_acceleration;

} ImuParam;

typedef struct
{
  std::string topic_;
  std_msgs::Headers header;
  std::string child_frame_id;
  geometry_msgss::Pose pose;
  geometry_msgss::Twist twist;

} OdomParam;

typedef struct
{
  std::string topic_;
  uint32_t seq;
  uint64_t session_id;
  sros::core::Pose pose;

} PoseParam;

namespace Ui
{
  class MainWindow;
}

class MainWindow : public QMainWindow, public Ui_MainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  int GetData();
  bool GetMouse();
  bool GetMouse2();
  void Switch(bool p);
  bool GetSlider();
  bool GetParam();
  bool SwitchStop();
  bool ExitOutput();
  void ExitChange(bool p);
  //控制传感器显示函数
  void QtShowLidar(LaserParam lidar);
  void QtShowImage(ImageParam image);
  void QtShowImu(ImuParam imu);
  void QtShowOdom(OdomParam odom);
  void QtShowPose(PoseParam pose);

  void QtShowLine();
  void QtShowUse();
  void QtShowParamName(QString s);

  //进度条总时间显示函数
  void QtProgressShow(int max);

private slots:
  void on_pushButton_clicked();

  // void on_lineedit_textChanged();
public slots:
  void PlayClickButton();
  void StopClickButton();
  void SpeedIncreaseClickButton();
  void SpeedSlowClickButton();
  void StepClickButton();
  void ExitClickButton();
  void TopicClickButton();
  void on_horizontalSlider_sliderMoved(int max, int min, double input);
  void on_horizontalSlider_sliderMoved(int pos);
  void Initialize();
  void MousePress();
  void SliderMove();
  //复选框声明
  int LidarClickButton();
  int CameraClickButton();
  int OdomClickButton();
  int PoseClickButton();
  int ImuClickButton();
  int CheckClickButton(); //这个主要用于显示省略号内容，因为数据量太大，可能会造成界面卡顿

public:
  LaserParam GetLaser1();
  LaserParam GetLaser2();

  ImageParam GetImage1();
  ImuParam   GetImu();
  OdomParam  GetOdom();
  PoseParam  GetPose();

private:
  Ui::MainWindow *ui;
  Dialog *dialog;
  QString filePath, fileFull;
  int position = 0;
  int position2 = 0;
  bool switch_mouse = false;
  bool switch_mouse_2 = false;
  bool slider_move = false;

  bool progress_show = true;

  //单步调试状态
  bool msg_step = false;
  //复选框的状态
  bool msg_lidar = false;
  bool msg_check = false;
  bool msg_imu = false;
  bool msg_odom = false;
  bool msg_camera = false;
  bool msg_pose = false;
  
  bool switch_stop = true;
  bool textclear = true;

  bool exit=false;
  std::mutex mutex_;

private:
  LaserParam lidar_1;
  LaserParam lidar_2;
  ImageParam image_1;
  ImuParam my_imu;

  OdomParam odom_1;
  PoseParam pose_1;
  int count = 0;
};

#endif //
