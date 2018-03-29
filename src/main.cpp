#include "UI/lobosimulatorwindow.h"
#include <QtWidgets/QApplication>
#include <QtWidgets/qdesktopwidget.h>
#include "vegatest.h"

int main(int argc, char *argv[])
{
	SetDllDirectory((LPCWSTR)L"./dll");

	QApplication a(argc, argv);
	LoboSimulatorMainWindow mainWindow;

	int desktopArea = QApplication::desktop()->width() *
		QApplication::desktop()->height();
	
	int widgetArea = mainWindow.width() * mainWindow.height();

	if (((float)widgetArea / (float)desktopArea) < 0.75f)
		mainWindow.show();
	else
		mainWindow.showMaximized();

	int temp;
	
	return a.exec();
}
