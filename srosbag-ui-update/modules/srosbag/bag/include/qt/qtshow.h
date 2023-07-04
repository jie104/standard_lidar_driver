#pragma once

#include "bag/message_bag.h"
#include <QDialog>
#include "qt/progressbar.h"
#include "qt/playbag.h"
#include <QTimer>
#include <QLabel>
#include <QApplication>
#include <QProgressDialog>

class QtShow 
{
  

public:
  QtShow(const QtShow &) = delete;
 // QtShow();
  
  int Show(int argc, char *argv[]);
  
private:
};