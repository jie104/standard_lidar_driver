#include <chrono>
#include <cstdlib>
#include <glog/logging.h>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdlib.h>

#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "bag/message_bag.h"


#include <QDialog>
#include "qt/progressbar.h"
#include "qt/playbag.h"
#include <QTimer>
#include <QLabel>
#include <QApplication>
#include <mutex>
#include <future>
#include <thread>
#include <unistd.h>
#include <QProgressDialog>


int bag::MsgBag::qtShow(int argc, char *argv[])
{

  QApplication a(argc, argv);
  //   QGuiApplication b(argc,argv);

  std::shared_ptr<MainWindow> W(new MainWindow);
  W->QtShowUse(); //使用说明显示
  QTimer *timer1 = new QTimer;
  QTimer *timer2 = new QTimer;
  bool param_type = true;
  int max = 0;
  timer1->start(1000);
  timer2->start(20);
  QObject::connect(timer1, &QTimer::timeout, [=, &W, &param_type, &max]()
                   {
                     max = (end_time_ - begin_time_) / 1e6;
                     W->QtProgressShow(max);

                     double progress = (time_stamp_ - begin_time_) / 1e6;

                     LaserParam lidar1 = W->GetLaser1();
                     LaserParam lidar2 = W->GetLaser2();
                     ImageParam image1 = W->GetImage1();
                     ImuParam imu1 = W->GetImu();
                     OdomParam odom1 = W->GetOdom();
                     PoseParam pose1 = W->GetPose();

                     //点击exit  进度条清0
                     if (W->ExitOutput())
                     {
                       // progress=0;
                       // W->ExitChange(false);
                     }
                     //输出传感器名字
                     if (msg_vector.size() != 0 && param_type)
                     {
                       for (auto iter = msg_vector.cbegin(); iter != msg_vector.cend(); iter++)
                       {
                         QString s = QString::fromStdString(*iter);
                         W->QtShowParamName(s);
                       }
                       param_type = !param_type;
                     }

                     std::cout << "进度条进度: " << progress << std::endl;
                     std::cout << "stamp1的值是： " << time_stamp_ << std::endl;

                     W->on_horizontalSlider_sliderMoved(max, 0, progress);

                     std::cout << "no mouse control" << std::endl;

                     if (W->GetMouse())
                     {

                       // std::cout << "mouse press" << std::endl;
                     }

                     if (W->GetMouse2()) //鼠标释放
                     {
                       std::cout << "mouse realse" << std::endl;

                       time_stamp_ = W->GetData() * 1e6 + begin_time_;

                       setSwitchTime(time_stamp_);
                       double progress1 = (time_stamp_ - begin_time_) / 1e6;
                       W->on_horizontalSlider_sliderMoved(max, 0, progress1);

                       timer1->start();
                       W->QtShowLine();
                       //释放鼠标显示雷达信息
                       W->QtShowLidar(lidar1);
                       W->QtShowLidar(lidar2);

                       //释放鼠标显示相机信息
                       W->QtShowImage(image1);

                       //释放鼠标显示IMU信息
                       W->QtShowImu(imu1);
                       //释放鼠标显示Odom信息
                       W->QtShowOdom(odom1);
                       //释放鼠标显示Pose信息
                       W->QtShowPose(pose1);
                       W->QtShowLine();
                       // std::cout << "stamp2的值是： " << time_stamp_ << std::endl;
                       //   std::cout << "手动给的值: " << (time_stamp_ - begin_time_) / 1e6 << std::endl;

                       std::cout << "realse" << std::endl;

                       W->Switch(false);
                     }
                   });

  QObject::connect(timer2, &QTimer::timeout, [=, &W, &timer1]()
                   {
                     //std::cout<<"GetSlider:  "<<W->GetSlider()<<std::endl;

                     if (W->GetSlider())
                     {

                       timer1->stop();

                       timer1->setInterval(1000);
                     }
                   });

  W->show();

  return a.exec();
}