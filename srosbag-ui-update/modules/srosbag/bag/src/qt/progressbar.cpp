#include "qt/progressbar.h"
#include <QString>
#include "bag/message_bag.h"
ProgressBar::ProgressBar(QWidget *parent)
    : QMainWindow(parent)
{
}

ProgressBar::~ProgressBar()
{

}

void Progressbar::stepOne()
{
    if(this->value()+1 <= this->maximum())
    {
       // this->setValue(this->value() + 1);
        this->update();
        strText = "进度条 : " + this->text();
        this->setWindowTitle(strText);
    }
    else
    {
        this->setValue(this->minimum());
    }
}

