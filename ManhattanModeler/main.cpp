#include <QApplication>
#include <iostream>
#include <QLocale>
#include <QTranslator>
#include <QTextCodec>
#include <QDebug>
#include <time.h>
#include "main_window.h"


int main(int argc, char **argv)
{
	srand(time(0));
	QApplication app(argc, argv);

	MainWindow win;		
	win.show();

	return app.exec();
};
