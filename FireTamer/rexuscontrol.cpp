#include "rexuscontrol.h"
#include "ui_rexuscontrol.h"

RexusControl::RexusControl(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RexusControl)
{
    ui->setupUi(this);
}

RexusControl::~RexusControl()
{
    delete ui;
}

void RexusControl::on_pushButton_clicked()
{
    hide();
}
