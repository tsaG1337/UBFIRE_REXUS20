#include "rexusdialog.h"
#include "ui_rexusdialog.h"

rexusdialog::rexusdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rexusdialog)
{
    ui->setupUi(this);
}

rexusdialog::~rexusdialog()
{
    delete ui;
}
