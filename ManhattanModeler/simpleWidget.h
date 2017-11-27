#pragma once


#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton> 

class simpleWidget: public QDialog
{
	Q_OBJECT

public:
	simpleWidget( QWidget *parent = 0 );
	~simpleWidget(void);
	
	private slots:
		void onDlgBtnClick( void );


private:
	void setupUi();
		
private:
	QLabel *m_label;
	QPushButton *m_pushBtn;
	QLineEdit *m_lineEdit;


};