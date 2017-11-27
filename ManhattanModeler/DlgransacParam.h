#pragma once

#include <QDialog>
#include "ui_ransacParam.h"
#include "main_window.h"

class CDlg_ransacParam: public QDialog, public Ui::DlgRANSAC
{
public:

	Q_OBJECT

public:
	CDlg_ransacParam(QWidget *parent=0);
	~CDlg_ransacParam(void);

	private slots: 
		void accept();
private:
	MainWindow *parentWindow;
};
