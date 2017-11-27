#pragma once

#include <QDialog>
#include "ui_dlgFittingBox.h"
#include "main_window.h"

class DlgFittingBox: public QDialog, public Ui::frmFittingBox
{
public:

	Q_OBJECT
public: 
	DlgFittingBox(QWidget *parent);
	~DlgFittingBox(void);
 
	private slots: 
		void accept();
private:
	MainWindow *parentWindow;
};
