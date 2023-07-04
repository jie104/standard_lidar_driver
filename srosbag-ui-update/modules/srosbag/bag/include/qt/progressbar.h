#ifndef PROGRESSBAR_H
#define PROGRESSBAR_H

#include <QMainWindow>
#include <QProgressBar>

class ProgressBar : public QMainWindow
{
    Q_OBJECT

public:
    ProgressBar(QWidget *parent = 0);
    ~ProgressBar();
};

//新建类
class QString;
class Progressbar : public QProgressBar
{
    Q_OBJECT
public:
    Progressbar(QWidget *parent = 0):QProgressBar(parent){}
    QString strText;

public slots:
    void stepOne();
};

#endif // PROGRESSBAR_H

