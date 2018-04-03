#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_TongueQT.h"

class TongueQT : public QMainWindow
{
	Q_OBJECT

public:
	TongueQT(QWidget *parent = Q_NULLPTR);

private:
	Ui::TongueQTClass ui;
};
