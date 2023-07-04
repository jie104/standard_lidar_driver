#include "qt/playbag.h"
#include <unistd.h>
#include "qt/ui_dialog.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <iostream>
#include <QTimer>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include "bag/message_bag.h"
#include "../../../../core/msg/image_msg.hpp"
#include "../../../../core/msg/imu_msg.hpp"
#include "../../../../core/msg/odometry_msg.hpp"
#include "../../../../core/msg/PoseStampedMsg.h"
//#include"../../../../modules/srosbag/message/odometry_msg.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include "factory/my_factory.h"
#include <iomanip>
#include <unistd.h>
auto rosbag_1 = bag::MsgBag::Create("/home/zxj");


//bag_pub_2 = nd.advertise<sensor_msgs::LaserScan>("/lidar2_an", 50);
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  connect(ui->PlayButton, SIGNAL(clicked()), this, SLOT(PlayClickButton()));
  connect(ui->StopButton, SIGNAL(clicked()), this, SLOT(StopClickButton()));
  connect(ui->SpeedIncreaseButton, SIGNAL(clicked()), this, SLOT(SpeedIncreaseClickButton()));
  connect(ui->SpeedSlowButton, SIGNAL(clicked()), this, SLOT(SpeedSlowClickButton()));
  connect(ui->StepButton, SIGNAL(clicked()), this, SLOT(StepClickButton()));
  connect(ui->ExitButton, SIGNAL(clicked()), this, SLOT(ExitClickButton()));
 // connect(ui->TopicShowButton, SIGNAL(clicked()), this, SLOT(TopicClickButton()));

  connect(ui->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(Initialize())); //鼠标释放

  connect(ui->horizontalSlider, SIGNAL(sliderPressed()), this, SLOT(MousePress())); //鼠标按压

  connect(ui->horizontalSlider, SIGNAL(sliderMoved(int)), this, SLOT(SliderMove(bool))); //进度条拖动

  /*复选框事件*/
  connect(ui->lidarBox, SIGNAL(stateChanged(int)), SLOT(LidarClickButton()));
  connect(ui->camera2, SIGNAL(stateChanged(int)), SLOT(CameraClickButton()));
  connect(ui->odom4, SIGNAL(stateChanged(int)), SLOT(OdomClickButton()));
  connect(ui->imu3, SIGNAL(stateChanged(int)), SLOT(ImuClickButton()));
  connect(ui->pose5, SIGNAL(stateChanged(int)), SLOT(PoseClickButton()));
  connect(ui->checkBox, SIGNAL(stateChanged(int)), SLOT(CheckClickButton()));
}

MainWindow::~MainWindow()
{

  delete ui;
}

void MainWindow::on_pushButton_clicked()
{

  QString fileName, fileSuffix;
  QFileInfo fileinfo;
  fileFull = QFileDialog::getOpenFileName(this, tr("file"), "/", tr("sbag(*.sbag)")); //获取整个文件名
  //fileFull = E:\QtCode\newExample\myTry\新建文本文档.txt

  //获取文件信息
  fileinfo = QFileInfo(fileFull);
  //fileinfo = E:\QtCode\newExample\myTry\新建文本文档.txt

  //获取文件名字
  fileName = fileinfo.fileName();
  //fileName = 新建文本文档.txt

  //获取文件后缀
  fileSuffix = fileinfo.suffix();
  //fileSuffix = txt

  //获取文件绝对路径
  filePath = fileinfo.absolutePath();

  if (!fileFull.isNull())
  {
    QFile file(fileFull);
    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
      QMessageBox::warning(this, tr("Error"), tr("read file error:&1").arg(file.errorString()));
      return;
    }
    QTextStream in(&file);
    QApplication::setOverrideCursor(Qt::WaitCursor);

    dialog = new Dialog(this);
    dialog->ui->lineEdit->setText(fileFull);
    dialog->ui->lineEdit_2->setText(fileName);
    dialog->ui->textEdit->setPlainText(in.readAll());

    QApplication::restoreOverrideCursor();

    QString hint = " Successuful:bag has been choosed!";
    QString str = "<font color=\"#008000\">" + hint + "</font>";

    ui->SensorBrowser->append(str);

    //  dialog->show();
  }
  else
  {
    qDebug() << "choose your bag";
  }
}

void MainWindow::PlayClickButton()
{

  std::string s = fileFull.toStdString();

  rosbag_1->playBack(s, false);
}
void MainWindow::StopClickButton()
{
  switch_stop = !switch_stop;
  rosbag_1->setPause();
}

void MainWindow::SpeedIncreaseClickButton()
{
  rosbag_1->setSpeed(2);
}
void MainWindow::SpeedSlowClickButton()
{
  rosbag_1->setSpeed(0.1);
}

void MainWindow::StepClickButton()
{
  msg_step = true;
  rosbag_1->nextStep();
  LaserParam lidar1 = MainWindow::GetLaser1();
  LaserParam lidar2 = MainWindow::GetLaser2();
  ImageParam image1 = MainWindow::GetImage1();
  ImuParam   imu1=MainWindow::GetImu();
  OdomParam  odom=MainWindow::GetOdom();
  PoseParam  pose=MainWindow::GetPose();

  MainWindow::QtShowLidar(lidar1);
  MainWindow::QtShowLidar(lidar2);


  MainWindow::QtShowImage(image1);
  MainWindow::QtShowOdom(odom);
  MainWindow::QtShowImu(imu1);
  MainWindow::QtShowPose(pose);

  msg_step = false;
}
void MainWindow::ExitClickButton()
{
  rosbag_1->closePlay();
  progress_show = true;
  exit=true;
  ui->label->clear();
}

void MainWindow::on_horizontalSlider_sliderMoved(int max, int min, double input)
{
  // std::cout<<position<<std::endl;

  ui->horizontalSlider->setMinimum(min);
  //设置滑动条控件的最大值
  ui->horizontalSlider->setMaximum(max);
  ui->horizontalSlider->setValue(input);
}

int MainWindow::GetData()
{
  return position2;
}

void MainWindow::on_horizontalSlider_sliderMoved(int pos)
{

  position = pos;
 
}

bool MainWindow::SwitchStop()
{
  return switch_stop;
}
bool MainWindow::GetMouse()
{
  return switch_mouse; //鼠标按压
}

bool MainWindow::GetMouse2()
{
  return switch_mouse_2;
}

void MainWindow::Initialize() //鼠标释放
{
  position2 = position;
  switch_mouse_2 = true; // 鼠标抬起来了
  switch_mouse = false;
  // slider_move=false;
}

void MainWindow::MousePress()
{
  switch_mouse = true;
  switch_mouse_2 = false;
  //return switch_mouse;
}
void MainWindow::Switch(bool p)
{
  switch_mouse_2 = p;
}

bool MainWindow::GetSlider()
{
  return slider_move;
}

void MainWindow::SliderMove()
{
  
  slider_move = true;

}

bool MainWindow::GetParam()
{
  return msg_lidar;
}

bool MainWindow::ExitOutput()
{
  return exit;
}
void MainWindow::ExitChange(bool p)
{
  exit=p;
  
}


/******************传感器消息获取***********/
LaserParam MainWindow::GetLaser1()
{
  return lidar_1;
}

LaserParam MainWindow::GetLaser2()
{
  return lidar_2;
}

ImageParam MainWindow::GetImage1()
{
  return image_1;
}

ImuParam MainWindow::GetImu()
{
  return my_imu;
}

OdomParam MainWindow::GetOdom()
{
  return odom_1;

}

PoseParam MainWindow::GetPose()
{
  return pose_1;
}

/***************************************/

void MainWindow::QtShowLine()
{

  QString s = "--------------------------------";
  ui->textBrowser_3->append(s);
}
void MainWindow::QtProgressShow(int max)
{
  if (progress_show && max != 0)
  {
  //  ui->textBrowser_2->append(QString::number(max));//显示进度条长度
    ui->label->setText(QString::number(max));
    progress_show = false;
  }
}
void MainWindow::TopicClickButton()
{

  //msg_param = true;
}

/*************复选框定义代码************/

int MainWindow::LidarClickButton()
{

  if (ui->lidarBox->isChecked())
  {
    msg_lidar = true;
  }
  else
  {
    msg_lidar = false;
    ui->textBrowser_3->clear();
  }
}

int MainWindow::CameraClickButton()
{
  if (ui->camera2->isChecked())
  {
    msg_camera = true;
  }
  else
  {
    msg_camera = false;
    ui->textBrowser_3->clear();
  }
}

int MainWindow::ImuClickButton()
{
  if(ui->imu3->isChecked())
  {
    msg_imu=true;
  }
  else
  {
   msg_imu=false;
   ui->textBrowser_3->clear();
  }
  
}

int MainWindow::OdomClickButton()
{
 if(ui->odom4->isChecked())
 {
   msg_odom=true;
 }
 else
 {
   msg_odom=false;
   ui->textBrowser_3->clear();
 }
 

}

int MainWindow::PoseClickButton()
{
  if(ui->pose5->isChecked())
  {
    msg_pose=true;
  }
  else
  {
    msg_pose=false;
    ui->textBrowser_3->clear();
  }



}

int MainWindow::CheckClickButton()
{
  if (ui->checkBox->isChecked())
  {
    msg_check = true;
  }
  else
  {

    msg_check = false;
  }
}

/**************************************/
void MainWindow::QtShowUse()
{
  QString describe="******使用前请一定要阅读readme*******";
  QString play_message = "-play: 播放bag";
  QString pause_message = "-pause: 暂停/继续播放 ";
  QString speed_message = "-speed: 加速播放 ";
  QString slow_message = "-slow: 减速播放 ";
  QString stop_message = "-stop: 停止播放";
  QString step_message = "-step: 在暂停播放后，点此单步播放 ";
  QString choosebag_message = "-choosebag: 选择你的srosbag包!";
  QString exit_message = "-exit: 退出播放";
  QString  bug="发现任何使用问题，请钉钉联系安希旭修改";
  QString ui_message = describe+"\n"+choosebag_message + "\n" + play_message + "\n" + pause_message + "\n" +
                       speed_message + "\n" + slow_message + "\n"  + step_message + "\n" +
                       choosebag_message + "\n" + exit_message + "\n"+bug+"\n";
  QString m = "包内传感器话题:";

  ui->textBrowser->append(ui_message);
  ui->textBrowser->append(m + "\n");
}
/*显示目前的bag内有哪些传感器*/
void MainWindow::QtShowParamName(QString s)
{

  ui->textBrowser->append(s);
}

void MainWindow::QtShowLidar(LaserParam lidar)
{

  QString range_max = QString::number(lidar.range_max, 10, 2);
  QString first_angle_max = QString::number(lidar.first_angle_max, 10, 2);
  QString first_angle_min = QString::number(lidar.first_angle_min, 10, 2);
  QString second_angle_min = QString::number(lidar.second_angle_min, 10, 2);
  QString second_angle_max = QString::number(lidar.second_angle_max, 10, 2);
  QString angle_min = QString::number(lidar.angle_min, 10, 2);
  QString angle_max = QString::number(lidar.angle_max, 10, 2);
  QString angle_increment = QString::number(lidar.angle_increment, 10, 2);
  QString time_increment = QString::number(lidar.time_increment, 10, 2);
  QString scan_time = QString::number(lidar.scan_time, 10, 2);
  QString range_min = QString::number(lidar.range_min, 10, 2);
  QString time_ = QString::number(lidar.time_, 10, 2);
  //QString ranges=QString::number(lidar.ranges,10,2);
  // QString intensities=QString::number(lidar.intensities,10,2);
  QString sensor_name = QString::fromStdString(lidar.sensor_name);
  QString topic = QString::fromStdString(lidar.topic_);
  QString topic_color = "<font color=\"#008000\">" + topic + "</font>";
  QString space = "   ";

  std::string lidar_range = "lidar range: ";
  std::string lidar_intensity = "lidar intensity: ";

  for (int i = 0; i < lidar.ranges.size(); i++)
  {
    double m = (double)lidar.ranges[i];

    lidar_range = lidar_range + " " + "[" + std::to_string(i) + "]" + " " + std::to_string(m) + "\n";
  }

  for (int ii = 0; ii < lidar.intensities.size(); ii++)
  {
    double mm = (double)lidar.intensities[ii];
    lidar_intensity = lidar_intensity + " " + "[" + std::to_string(ii) + "]" + " " + std::to_string(mm) + "\n";
  }

  //字符串省略号化
  //range
  QString range_qt = QString::fromStdString(lidar_range);
  ui->textBrowser->setAlignment(Qt::AlignTop);
  QFontMetrics fontWidth(ui->textBrowser->font()); //得到每个字符的宽度
  QString elideNote1 = fontWidth.elidedText(range_qt, Qt::ElideRight, 300);

  //intensity
  QString intensity_qt = QString::fromStdString(lidar_intensity);
  QString elideNote2 = fontWidth.elidedText(intensity_qt, Qt::ElideRight, 300);

  //TODO:  topic名字我直接赋值，报了个警告
  QString lidar_string = "Topic: " + topic + "\n" +
                         "range_max: " + range_max + "\n" + "range_min: " + range_min + "\n" +
                         "first_angle_max: " + first_angle_max + "\n" + "first_angle_min: " + first_angle_min + "\n" +
                         "second_angle_min: " + second_angle_min + "\n" + "second_angle_max: " + second_angle_min + "\n" +
                         "angle_min: " + angle_min + "\n" + "angle_max: " + angle_max + "\n" + "angle_increment: " + angle_increment + "\n" +
                         "time_increment: " + time_increment + "\n" + "scan_time: " + scan_time + "\n" + "sensor_name: " + sensor_name + "\n" +
                         "timestamp: " + time_ + "\n";
  //单步调试复选框显示
  if (msg_step && msg_lidar && !lidar.topic_.empty())
  {
    ui->textBrowser_3->append(lidar_string);
    if (lidar.intensities.size() != 0)
    {

      std::string size = std::to_string(lidar.intensities.size());
      QString lidar_size = QString::fromStdString(size);
      QString size_show = "lidar size: ";
      QString sss = size_show + lidar_size;
      ui->textBrowser->append(sss);
    }
    if (!msg_check)
    {
      //range intensity 不完全显示
      ui->textBrowser_3->append(elideNote1);
      ui->textBrowser_3->setToolTip(range_qt + "\n");

      ui->textBrowser_3->append(elideNote2);
      ui->textBrowser_3->setToolTip(intensity_qt + "\n");
    }
    if (msg_check)
    {
      //range intensity 完全显示
      ui->textBrowser_3->append(range_qt);
      ui->textBrowser_3->append(intensity_qt);
    }
  }

  //进度条显示
  if (switch_mouse_2 && msg_lidar && !lidar.topic_.empty()) //检测进度条被拖拽且释放
  {

    if (lidar.intensities.size() != 0)
    {

      std::string size = std::to_string(lidar.intensities.size());
      QString lidar_size = QString::fromStdString(size);
      QString size_show = "lidar size: ";
      QString sss = size_show + lidar_size;
      ui->textBrowser->append(sss);
    }

    ui->textBrowser_3->append(lidar_string);
    if (msg_check)
    {
      //range intensity 完全显示
      ui->textBrowser_3->append(range_qt);
      ui->textBrowser_3->append(intensity_qt);
    }
    if (!msg_check)
    {
      //range intensity 不完全显示
      ui->textBrowser_3->append(elideNote1);
      ui->textBrowser_3->setToolTip(range_qt + "\n");

      ui->textBrowser_3->append(elideNote2);
      ui->textBrowser_3->setToolTip(intensity_qt + "\n");
    }
    //switch_mouse_2=!switch_mouse_2;
  }
  if (msg_lidar)
  {
    rosbag_1->setMsgHandle<sros::core::LaserScanMsg>([=](const sros::core::LaserScanMsg &laser)
                                                   {
                                                     

                                                     lidar_1.range_max = laser.range_max;
                                                     lidar_1.first_angle_max = laser.first_angle_max;
                                                     lidar_1.first_angle_min = laser.first_angle_min;
                                                     lidar_1.second_angle_min = laser.second_angle_min;
                                                     lidar_1.second_angle_max = laser.second_angle_max;
                                                     lidar_1.angle_min = laser.angle_min;
                                                     lidar_1.angle_max = laser.angle_max;
                                                     lidar_1.angle_increment = laser.angle_increment;
                                                     lidar_1.time_increment = laser.time_increment;
                                                     lidar_1.scan_time = laser.scan_time;
                                                     lidar_1.range_min = laser.range_min;
                                                     lidar_1.ranges = laser.ranges;
                                                     lidar_1.intensities = laser.intensities;
                                                     lidar_1.sensor_name = laser.sensor_name;

                                                     lidar_1.time_ = laser.time_;
                                                     lidar_1.topic_ = "first_laser_";
                                                     lidar_1.update_times++;


                                           
                                                   },
                                                   "FIRST_SCAN");

    rosbag_1->setMsgHandle<sros::core::LaserScanMsg>([=](const sros::core::LaserScanMsg &laser)
                                                   {
                                                     lidar_2.range_max = laser.range_max;
                                                     lidar_2.first_angle_max = laser.first_angle_max;
                                                     lidar_2.first_angle_min = laser.first_angle_min;
                                                     lidar_2.second_angle_min = laser.second_angle_min;
                                                     lidar_2.second_angle_max = laser.second_angle_max;
                                                     lidar_2.angle_min = laser.angle_min;
                                                     lidar_2.angle_max = laser.angle_max;
                                                     lidar_2.angle_increment = laser.angle_increment;
                                                     lidar_2.time_increment = laser.time_increment;
                                                     lidar_2.scan_time = laser.scan_time;
                                                     lidar_2.range_min = laser.range_min;
                                                     lidar_2.ranges = laser.ranges;
                                                     lidar_2.intensities = laser.intensities;
                                                     lidar_2.sensor_name = laser.sensor_name;

                                                     lidar_2.time_ = laser.time_;
                                                     lidar_2.topic_ = "second_laser_";
                                                   },
                                                   "SECOND_SCAN");

    count++;
  }
}

//TODO:   未测试 图像只添加了图片的长和宽
void MainWindow::QtShowImage(ImageParam image)
{

  QString width = QString::number(image.width_);
  QString height = QString::number(image.height_);
  QString topic = QString::fromStdString(image.topic_);

  QString image_string = "Topic: " + topic + "\n" +
                         "image width: " + width + "\n" +
                         "image height: " + height + "\n";

  if (msg_camera)
  {
    rosbag_1->setMsgHandle<sros::core::ImageMsg>([=](const sros::core::ImageMsg &Image)
                                               {
                                                 image_1.width_ = Image.width_;
                                                 image_1.height_ = Image.height_;
                                                 image_1.topic_ = "first_camera";
                                               },
                                               "first_camera");
  }

  if (!image.topic_.empty() && msg_camera) //拖拽进度条
  {

    ui->textBrowser_3->append(image_string);
  }

  if (msg_camera && msg_step) //单步调试显示
  {
    ui->textBrowser_3->append(image_string);
  }
}

//TODO:  未测试
void MainWindow::QtShowImu(ImuParam imu)
{

 
  QString frame_id = QString::fromStdString(imu.header.frame_id);
  QString stamp = QString::number(imu.header.stamp);
  QString sync_stamp = QString::number(imu.header.sync_stamp);
  QString angular_velocity = QString::number(imu.angular_velocity(0, 0)) + " " + QString::number(imu.angular_velocity(1, 0)) + " " + QString::number(imu.angular_velocity(2, 0));

  QString linear_acceleration = QString::number(imu.linear_acceleration(0, 0)) + " " + QString::number(imu.linear_acceleration(1, 0)) + " " + QString::number(imu.linear_acceleration(2, 0));
 
  QString orientation=QString::number(imu.orientation.x())+" "+QString::number(imu.orientation.y())+" "+
                      QString::number(imu.orientation.z())+" "+QString::number(imu.orientation.w());


  QString imu_string = "frame_id: " + frame_id + "\n" +
                       "stamp: " + stamp + "\n" +
                       "angular_v: " + angular_velocity + "\n" +
                       "orientation(x y z w): "+orientation + "\n"+
                       "linear_v: " + linear_acceleration + "\n";

  if (msg_imu)
  {
    rosbag_1->setMsgHandle<Imu>([=](const Imu &IMU)
                              {
                                my_imu.header.frame_id = IMU.header.frame_id;
                                my_imu.header.stamp = IMU.header.stamp;
                                my_imu.header.sync_stamp = IMU.header.sync_stamp;
                                my_imu.orientation = IMU.orientation;
                                my_imu.angular_velocity = IMU.angular_velocity;
                                my_imu.linear_acceleration = IMU.linear_acceleration;
                                my_imu.topic_ = "imu";
                               
                              },
                              "imu");

    
  }

  if(msg_imu &&GetMouse2())
  {
     ui->textBrowser_3->append(imu_string); // 拖拽进度条显示
  }
  if (msg_imu && msg_step) //单步调试
  {
    
    ui->textBrowser_3->append(imu_string);
    
  }
}

void MainWindow::QtShowOdom(OdomParam odom)
{

  QString frame_id = QString::fromStdString(odom.header.frame_id);
  QString stamp = QString::number(odom.header.stamp);
  QString sync_stamp = QString::number(odom.header.sync_stamp);
  QString position = QString::number(odom.pose.position(0, 0)) + " " + QString::number(odom.pose.position(1, 0)) + " " + QString::number(odom.pose.position(2, 0));

  QString orientation=QString::number(odom.pose.orientation.x())+" "+QString::number(odom.pose.orientation.y())+" "+
                      QString::number(odom.pose.orientation.z())+" "+QString::number(odom.pose.orientation.w());
  
  


  QString linear = QString::number(odom.twist.linear(0, 0)) + " " + QString::number(odom.twist.linear(1, 0)) +" "+ QString::number(odom.twist.linear(2, 0));
  QString angular = QString::number(odom.twist.angular(0, 0)) + " " + QString::number(odom.twist.angular(1, 0)) + " " + QString::number(odom.twist.angular(2, 0));

  //std::cout<<odom.twist.angular(0,0)<<std::endl;
 
  QString odom_string = "frame_id: " + frame_id + "\n" +
                        "stamp: " + stamp + "\n" +
                        "position: " + position + "\n" +
                        "linear: " + linear + "\n" +
                         "orientation(x y z w)： "+orientation + "\n"+
                        "angular: " + angular + "\n";

 if (msg_odom)
  {
    rosbag_1->setMsgHandle<sros::core::nav_msgs::Odometry>([=](const sros::core::nav_msgs::Odometry &Odom_1)
                                                         {
                                                           odom_1.header.stamp = Odom_1.header.stamp;
                                                           odom_1.header.sync_stamp = Odom_1.header.sync_stamp;
                                                           odom_1.header.frame_id = Odom_1.header.frame_id;
                                                           //odom_1.pose = Odom_1.pose;
                                                          // odom_1.twist = Odom_1.twist;
                                                           odom_1.topic_ = "odometryBag";
                                                          
                                                         },
                                                         "odometryBag");
   
  }

  if(msg_odom&&GetMouse2())
  {
    ui->textBrowser_3->append(odom_string); //进度条拖动
  }

  if (msg_odom && msg_step) //单步调试
  {

    ui->textBrowser_3->append(odom_string);
  }
}

//TODO:     只输出了xy  其他有需要可以继续添加
void MainWindow::QtShowPose(PoseParam pose)
{

  QString topic = QString::fromStdString(pose.topic_);
  QString seq = QString::number(pose.seq);
  QString session_id = QString::number(pose.session_id);
  QString position_x = QString::number(pose.pose.x());
  QString position_y = QString::number(pose.pose.y());

  QString pose_string = "topic: " + topic + "\n" +
                        "seq: " + seq + "\n" +
                        "position_x: " + position_x + "\n" +
                        "position_y: " + position_y + "\n" ;
                       

  if (msg_pose)
  {

    rosbag_1->setMsgHandle<sros::core::PoseStampedMsg>([=](const sros::core::PoseStampedMsg &Pose_1)
                                                     {
                                                       pose_1.topic_ = "pose";
                                                       pose_1.seq = Pose_1.seq;
                                                       pose_1.session_id = Pose_1.session_id;
                                                       pose_1.pose = Pose_1.pose;
                                                     },
                                                     "pose");
                               
  }

  if(msg_pose&&GetMouse2())
  {
     ui->textBrowser_3->append(pose_string);
  }
  if (msg_pose && msg_step) //单步调试
  {

    ui->textBrowser_3->append(pose_string);
  }

}