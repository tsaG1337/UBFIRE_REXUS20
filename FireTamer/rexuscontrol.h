#ifndef REXUSCONTROL_H
#define REXUSCONTROL_H

#include <QDialog>

namespace Ui {
class RexusControl;
}

class RexusControl : public QDialog
{
    Q_OBJECT

public:
    explicit RexusControl(QWidget *parent = 0);
    ~RexusControl();

private slots:
    void on_pushButton_clicked();

private:
    Ui::RexusControl *ui;
};

#endif // REXUSCONTROL_H
