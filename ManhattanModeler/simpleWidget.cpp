#include "simpleWidget.h"

#include "Parameters.h"
  
simpleWidget::simpleWidget( QWidget *parent /*= 0*/ )
{ 
	setupUi();
 
	connect(m_pushBtn, SIGNAL(clicked()), this, SLOT(onDlgBtnClick())); 
}

simpleWidget::~simpleWidget(void)
{
}

void simpleWidget::setupUi()
{
	m_label = new QLabel(this);
	m_label->setText(QString("point size:"));
	m_label->setObjectName(QString::fromUtf8("label_1"));
	m_label->setGeometry(20,10,45,21);
 
	m_pushBtn = new QPushButton(this);
	m_pushBtn->setObjectName(QString::fromUtf8("pushButton"));
	m_pushBtn->setGeometry(QRect(50, 40, 75, 23));
	m_pushBtn->setText(QString("set"));

	m_lineEdit = new QLineEdit(this);
	m_lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
	m_lineEdit->setGeometry(QRect(80, 10, 61, 20));
	m_lineEdit->setText(QString("2"));

}
void simpleWidget::onDlgBtnClick( void )
{ 
	int num = m_lineEdit->text().toUInt();
	lml_param::showPointSize = num;
	
	this->done(QDialog::Accepted);
}

