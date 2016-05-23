#ifndef REXUSDIALOG_H
#define REXUSDIALOG_H

#include <QDialog>

namespace Ui {
class RexusDialog;
}

class RexusDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RexusDialog(QWidget *parent = 0);
    ~RexusDialog();

private:
    Ui::RexusDialog *ui;
};

#endif // REXUSDIALOG_H
